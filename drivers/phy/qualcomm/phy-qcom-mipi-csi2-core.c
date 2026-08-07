// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025, Linaro Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "phy-qcom-mipi-csi2.h"

/*
 * Debug: take the top CSIPHY timer rate regardless of C-PHY. Independent of
 * cphy_force so D-PHY at 400 MHz can be measured on its own -- this driver
 * should then produce settle_cnt 0x1d, matching Windows' D-PHY formula for the
 * same link, which checks that both stacks agree about settle arithmetic.
 */
static bool timer_400;
module_param(timer_400, bool, 0644);
MODULE_PARM_DESC(timer_400, "x1e80100 debug: force the 400 MHz CSIPHY timer");

#define CAMSS_CLOCK_MARGIN_NUMERATOR 105
#define CAMSS_CLOCK_MARGIN_DENOMINATOR 100

static inline void phy_qcom_mipi_csi2_add_clock_margin(u64 *rate)
{
	*rate *= CAMSS_CLOCK_MARGIN_NUMERATOR;
	*rate = div_u64(*rate, CAMSS_CLOCK_MARGIN_DENOMINATOR);
}

static int
phy_qcom_mipi_csi2_set_clock_rates(struct mipi_csi2phy_device *csi2phy,
				   s64 link_freq)
{
	const struct mipi_csi2phy_soc_cfg *soc_cfg = csi2phy->soc_cfg;
	struct device *dev = csi2phy->dev;
	int i, j;
	int ret;

	for (i = 0; i < soc_cfg->num_clk; i++) {
		const struct mipi_csi2phy_clk_freq *clk_freq = &soc_cfg->clk_freq[i];
		const char *clk_name = soc_cfg->clk_names[i];
		struct clk *clk = csi2phy->clks[i].clk;
		u64 min_rate = link_freq / 4;
		long round_rate;

		phy_qcom_mipi_csi2_add_clock_margin(&min_rate);

		/* This clock should be enabled only not set */
		if (!clk_freq->num_freq)
			continue;

		for (j = 0; j < clk_freq->num_freq; j++)
			if (min_rate < clk_freq->freq[j])
				break;

		if (j == clk_freq->num_freq) {
			dev_err(dev,
				"Pixel clock %llu is too high for %s\n",
				min_rate, clk_name);
			return -EINVAL;
		}

		/* if sensor pixel clock is not available
		 * set highest possible CSIPHY clock rate
		 */
		/*
		 * cphy_force: the recovered C-PHY settle counts are in 2.5 ns
		 * ticks, i.e. a 400 MHz timer. The default pick lands one
		 * entry low here, which stretches every settle by 1.5x.
		 */
		if (min_rate == 0 || cphy_force || timer_400)
			j = clk_freq->num_freq - 1;

		round_rate = clk_round_rate(clk, clk_freq->freq[j]);
		if (round_rate < 0) {
			dev_err(dev, "clk round rate failed: %ld\n",
				round_rate);
			return -EINVAL;
		}

		csi2phy->timer_clk_rate = round_rate;

		dev_dbg(dev, "set clk %s %lu Hz\n",
			clk_name, round_rate);

		ret = clk_set_rate(clk, csi2phy->timer_clk_rate);
		if (ret < 0) {
			dev_err(dev, "clk set rate failed: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int phy_qcom_mipi_csi2_configure(struct phy *phy,
					union phy_configure_opts *opts)
{
	struct mipi_csi2phy_device *csi2phy = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *dphy_cfg_opts = &opts->mipi_dphy;
	struct mipi_csi2phy_stream_cfg *stream_cfg = &csi2phy->stream_cfg;
	int ret;
	int i;

	ret = phy_mipi_dphy_config_validate(dphy_cfg_opts);
	if (ret)
		return ret;

	if (dphy_cfg_opts->lanes < 1 || dphy_cfg_opts->lanes > CSI2_MAX_DATA_LANES)
		return -EINVAL;

	stream_cfg->combo_mode = 0;
	stream_cfg->link_freq = dphy_cfg_opts->hs_clk_rate;
	stream_cfg->num_data_lanes = dphy_cfg_opts->lanes;

	/*
	 * phy_configure_opts_mipi_dphy.lanes starts from zero to
	 * the maximum number of enabled lanes.
	 *
	 * TODO: add support for bitmask of enabled lanes and polarities
	 * of those lanes to the phy_configure_opts_mipi_dphy struct.
	 * For now take the polarities as zero and the position as fixed
	 * this is fine as no current upstream implementation maps otherwise.
	 */
	for (i = 0; i < stream_cfg->num_data_lanes; i++) {
		stream_cfg->lane_cfg.data[i].pol = 0;
		stream_cfg->lane_cfg.data[i].pos = i;
	}

	stream_cfg->lane_cfg.clk.pol = 0;
	stream_cfg->lane_cfg.clk.pos = 7;

	return 0;
}

static int phy_qcom_mipi_csi2_power_on(struct phy *phy)
{
	struct mipi_csi2phy_device *csi2phy = phy_get_drvdata(phy);
	const struct mipi_csi2phy_hw_ops *ops = csi2phy->soc_cfg->ops;
	struct device *dev = &phy->dev;
	int ret;

	ret = regulator_bulk_enable(csi2phy->soc_cfg->num_supplies,
				    csi2phy->supplies);
	if (ret)
		return ret;

	ret = phy_qcom_mipi_csi2_set_clock_rates(csi2phy, csi2phy->stream_cfg.link_freq);
	if (ret)
		goto poweroff_phy;

	ret = clk_bulk_prepare_enable(csi2phy->soc_cfg->num_clk,
				      csi2phy->clks);
	if (ret) {
		dev_err(dev, "failed to enable clocks, %d\n", ret);
		goto poweroff_phy;
	}

	ops->hw_version_read(csi2phy);

	return ops->lanes_enable(csi2phy, &csi2phy->stream_cfg);

poweroff_phy:
	regulator_bulk_disable(csi2phy->soc_cfg->num_supplies,
			       csi2phy->supplies);

	return ret;
}

static int phy_qcom_mipi_csi2_power_off(struct phy *phy)
{
	struct mipi_csi2phy_device *csi2phy = phy_get_drvdata(phy);

	clk_bulk_disable_unprepare(csi2phy->soc_cfg->num_clk,
				   csi2phy->clks);
	regulator_bulk_disable(csi2phy->soc_cfg->num_supplies,
			       csi2phy->supplies);

	return 0;
}

static const struct phy_ops phy_qcom_mipi_csi2_ops = {
	.configure	= phy_qcom_mipi_csi2_configure,
	.power_on	= phy_qcom_mipi_csi2_power_on,
	.power_off	= phy_qcom_mipi_csi2_power_off,
	.owner		= THIS_MODULE,
};

static int phy_qcom_mipi_csi2_probe(struct platform_device *pdev)
{
	unsigned int i, num_clk, num_supplies;
	struct mipi_csi2phy_device *csi2phy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct phy *generic_phy;
	int ret;

	csi2phy = devm_kzalloc(dev, sizeof(*csi2phy), GFP_KERNEL);
	if (!csi2phy)
		return -ENOMEM;

	csi2phy->dev = dev;
	csi2phy->soc_cfg = device_get_match_data(&pdev->dev);

	if (!csi2phy->soc_cfg)
		return -EINVAL;

	num_clk = csi2phy->soc_cfg->num_clk;
	csi2phy->clks = devm_kzalloc(dev, sizeof(*csi2phy->clks) * num_clk, GFP_KERNEL);
	if (!csi2phy->clks)
		return -ENOMEM;

	for (i = 0; i < num_clk; i++)
		csi2phy->clks[i].id = csi2phy->soc_cfg->clk_names[i];

	ret = devm_clk_bulk_get(dev, num_clk, csi2phy->clks);
	if (ret) {
		dev_err(dev, "Failed to get clocks %d\n", ret);
		return ret;
	}

	ret = clk_bulk_prepare_enable(num_clk, csi2phy->clks);
	if (ret) {
		dev_err(dev, "apq8016 clk_enable failed\n");
		return ret;
	}

	num_supplies = csi2phy->soc_cfg->num_supplies;
	csi2phy->supplies = devm_kzalloc(dev, sizeof(*csi2phy->supplies) * num_supplies,
					 GFP_KERNEL);
	if (!csi2phy->supplies)
		return -ENOMEM;

	for (i = 0; i < num_supplies; i++)
		csi2phy->supplies[i].supply = csi2phy->soc_cfg->supply_names[i];

	ret = devm_regulator_bulk_get(dev, num_supplies, csi2phy->supplies);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get regulator supplies\n");

	csi2phy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(csi2phy->base))
		return PTR_ERR(csi2phy->base);

	/*
	 * The CSIPHY interrupt: the DT routes it and ops->isr decodes it, but
	 * nothing ever requested the line, so the block's own error reporting
	 * has never reached anyone. lanes_enable() masks every source unless
	 * cphy_force is set, so on the default path this changes nothing.
	 */
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_warn(dev, "no CSIPHY irq, error reporting stays off: %d\n",
			 ret);
	} else {
		csi2phy->irq = ret;
		ret = devm_request_irq(dev, csi2phy->irq,
				       csi2phy->soc_cfg->ops->isr, 0,
				       dev_name(dev), csi2phy);
		if (ret) {
			dev_warn(dev, "CSIPHY irq %d request failed: %d\n",
				 csi2phy->irq, ret);
			csi2phy->irq = 0;
		} else {
			dev_info(dev, "CSIPHY irq %d requested\n",
				 csi2phy->irq);
		}
	}

	generic_phy = devm_phy_create(dev, NULL, &phy_qcom_mipi_csi2_ops);
	if (IS_ERR(generic_phy)) {
		ret = PTR_ERR(generic_phy);
		dev_err(dev, "failed to create phy, %d\n", ret);
		return ret;
	}
	csi2phy->phy = generic_phy;

	phy_set_drvdata(generic_phy, csi2phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (!IS_ERR(phy_provider))
		dev_dbg(dev, "Registered MIPI CSI2 PHY device\n");
	else
		pm_runtime_disable(dev);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id phy_qcom_mipi_csi2_of_match_table[] = {
	{ .compatible	= "qcom,x1e80100-mipi-csi2-combo-phy", .data = &mipi_csi2_dphy_4nm_x1e },
	{ }
};
MODULE_DEVICE_TABLE(of, phy_qcom_mipi_csi2_of_match_table);

static struct platform_driver phy_qcom_mipi_csi2_driver = {
	.probe		= phy_qcom_mipi_csi2_probe,
	.driver = {
		.name	= "qcom-mipi-csi2-phy",
		.of_match_table = phy_qcom_mipi_csi2_of_match_table,
	},
};

module_platform_driver(phy_qcom_mipi_csi2_driver);

MODULE_DESCRIPTION("Qualcomm MIPI CSI2 PHY driver");
MODULE_DESCRIPTION("Bryan O'Donoghue <bryan.odonoghue@linaro.org>");
MODULE_LICENSE("GPL");
