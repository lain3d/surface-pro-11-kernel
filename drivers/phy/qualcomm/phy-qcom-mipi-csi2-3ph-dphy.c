// SPDX-License-Identifier: GPL-2.0
/*
 * camss-phy_qcom_mipi_csi2-3ph-1-0.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY Module 3phase v1.0
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016-2025 Linaro Ltd.
 */
#define DEBUG
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#include "phy-qcom-mipi-csi2.h"

#define CSIPHY_3PH_LNn_CFG1(n)				(0x000 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG1_SWI_REC_DLY_PRG		(BIT(7) | BIT(6))
#define CSIPHY_3PH_LNn_CFG2(n)				(0x004 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG2_LP_REC_EN_INT		BIT(3)
#define CSIPHY_3PH_LNn_CFG3(n)				(0x008 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG4(n)				(0x00c + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG4_T_HS_CLK_MISS		0xa4
#define CSIPHY_3PH_LNn_CFG4_T_HS_CLK_MISS_660		0xa5
#define CSIPHY_3PH_LNn_CFG5(n)				(0x010 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG5_T_HS_DTERM			0x02
#define CSIPHY_3PH_LNn_CFG5_HS_REC_EQ_FQ_INT		0x50
#define CSIPHY_3PH_LNn_TEST_IMP(n)			(0x01c + 0x100 * (n))
#define CSIPHY_3PH_LNn_TEST_IMP_HS_TERM_IMP		0xa
#define CSIPHY_3PH_LNn_MISC1(n)				(0x028 + 0x100 * (n))
#define CSIPHY_3PH_LNn_MISC1_IS_CLKLANE			BIT(2)
#define CSIPHY_3PH_LNn_CFG6(n)				(0x02c + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG6_SWI_FORCE_INIT_EXIT		BIT(0)
#define CSIPHY_3PH_LNn_CFG7(n)				(0x030 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG7_SWI_T_INIT			0x2
#define CSIPHY_3PH_LNn_CFG8(n)				(0x034 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG8_SWI_SKIP_WAKEUP		BIT(0)
#define CSIPHY_3PH_LNn_CFG8_SKEW_FILTER_ENABLE		BIT(1)
#define CSIPHY_3PH_LNn_CFG9(n)				(0x038 + 0x100 * (n))
#define CSIPHY_3PH_LNn_CFG9_SWI_T_WAKEUP		0x1
#define CSIPHY_3PH_LNn_CSI_LANE_CTRL15(n)		(0x03c + 0x100 * (n))
#define CSIPHY_3PH_LNn_CSI_LANE_CTRL15_SWI_SOT_SYMBOL	0xb8

#define CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(offset, n)	((offset) + 0x4 * (n))
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL5_CLK_ENABLE	BIT(7)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B	BIT(0)
#define CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID	BIT(1)
#define CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(offset, n)	((offset) + 0xb0 + 0x4 * (n))

#define CSIPHY_DEFAULT_PARAMS				0
#define CSIPHY_LANE_ENABLE				1
#define CSIPHY_SETTLE_CNT_LOWER_BYTE			2
#define CSIPHY_SETTLE_CNT_HIGHER_BYTE			3
#define CSIPHY_DNP_PARAMS				4
#define CSIPHY_2PH_REGS					5
#define CSIPHY_3PH_REGS					6
#define CSIPHY_SKEW_CAL					7

/* 4nm 2PH v 2.1.2 2p5Gbps 4 lane DPHY mode */
/* 4nm C-PHY, x1e80100, dataRate < 2.0 Gbps. Recovered from
 * qccammipicsi8380.sys; see data/csiphy-cphy-x1e80100.txt in arm64-egpu.
 * Per-lane blocks repeat at stride 0x400; C-PHY drives the odd lanes only,
 * so the settle count lands at 0x20c / 0x60c / 0xa0c. */
static const struct
mipi_csi2phy_lane_regs lane_regs_x1e80100_cphy[] = {
	{0x0268, 0xf1, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0278, 0x45, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0288, 0x20, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x026c, 0x05, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x028c, 0x37, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0294, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0270, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0274, 0x03, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0668, 0xf1, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0678, 0x45, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0688, 0x20, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x066c, 0x05, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x068c, 0x37, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0694, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0670, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0674, 0x03, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a68, 0xf1, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a78, 0x45, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a88, 0x20, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a6c, 0x05, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a8c, 0x37, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a94, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a70, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a74, 0x03, 0x989680, CSIPHY_DEFAULT_PARAMS},
	{0x0294, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02f4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02f8, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02fc, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02f0, 0xef, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0694, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06f4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06f8, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06fc, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06f0, 0xef, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a94, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0af4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0af8, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0afc, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0af0, 0xef, 0x989680, CSIPHY_DEFAULT_PARAMS},
	{0x0208, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x020c, 0x2e, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0210, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0214, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0200, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0204, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0608, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x060c, 0x2e, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0610, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0614, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0600, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0604, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a08, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a0c, 0x2e, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0a10, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a14, 0x09, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a00, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a04, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02e4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02e8, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02ec, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0218, 0x3e, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x021c, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0220, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0224, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0228, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x022c, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06e4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06e8, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06ec, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0618, 0x3e, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x061c, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0620, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0624, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0628, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x062c, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0ae4, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0ae8, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0aec, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a18, 0x3e, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a1c, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a20, 0x41, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a24, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a28, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a2c, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0264, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0244, 0xb2, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0310, 0x35, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x02bc, 0xd0, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0254, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0240, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0260, 0xa8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0284, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0290, 0x02, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0664, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0644, 0xb2, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0710, 0x35, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x06bc, 0xd0, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0654, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0640, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0660, 0xa8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0684, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0690, 0x02, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a64, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a44, 0xb2, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0b10, 0x35, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0abc, 0xd0, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a54, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a40, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a60, 0xa8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a84, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0a90, 0x02, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x102c, 0xff, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1030, 0xfe, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1034, 0xe6, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1038, 0xdf, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x103c, 0xdf, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1040, 0xfc, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1044, 0xfb, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1048, 0x9b, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x104c, 0x7f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1050, 0xbf, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1054, 0xff, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1000, 0x0e, 0x00, CSIPHY_DEFAULT_PARAMS},
};

/*
 * Debug: force C-PHY lane programming. The IMX681 on the Surface Pro 11 is put
 * into CSI-2 C-PHY mode by its vendor init table -- CCS CSI_SIGNALLING_MODE
 * (0x0111) reads 0x02 freshly powered and 0x03 once the driver has programmed
 * it, measured on the part. This driver implements D-PHY only, which is why the
 * sensor reports itself streaming and vfe0 has never taken an interrupt.
 *
 * Off by default; nothing changes unless it is set. Gated on the parameter
 * alone rather than on the SoC, because it is an experiment, not a feature.
 *
 * Not static: phy_qcom_mipi_csi2_set_clock_rates() in the core file reads it
 * to pick the 400 MHz timer rate the recovered settle counts assume.
 */
bool cphy_force;
module_param(cphy_force, bool, 0644);
MODULE_PARM_DESC(cphy_force, "x1e80100 debug: force C-PHY lane programming");

/*
 * C-PHY settle count. Windows does not compute this -- on the C-PHY side it is
 * a threshold table keyed on the symbol rate. "First threshold greater than the
 * symbol rate wins."
 */
struct mipi_csi2phy_cphy_settle {
	u32 max_msps;
	u8 settle;
};

static const struct mipi_csi2phy_cphy_settle cphy_settle_x1e80100[] = {
	{  500, 0x66 },
	{  600, 0x58 },
	{  700, 0x4e },
	{  800, 0x46 },
	{  900, 0x40 },
	{ 1000, 0x39 },
	{ 1100, 0x38 },
	{ 1200, 0x35 },
	{ 1300, 0x2f },
	{ 1400, 0x2b },
	{ 1500, 0x2e },
	{ 1600, 0x2c },
	{ 1700, 0x2a },
	{ 1800, 0x29 },
	{ 1900, 0x28 },
	{ 2000, 0x27 },
	{ 2100, 0x26 },
	{ 2200, 0x25 },
	{ 2300, 0x24 },
	{ 2400, 0x23 },
	{ 2500, 0x22 },
	{ 2600, 0x22 },
	{ 2700, 0x21 },
	{ 2800, 0x21 },
	{ 2900, 0x20 },
	{ 3000, 0x20 },
};

/*
 * link_freq is the D-PHY DDR convention the DT and camss use, so the bit rate
 * is 2 * link_freq. Windows converts to symbols at 2.28 bits per C-PHY symbol.
 */
static u8 phy_qcom_mipi_csi2_cphy_settle_cnt(s64 link_freq)
{
	u64 msps;
	int i;

	if (link_freq <= 0)
		return 0;

	msps = div_u64((u64)link_freq * 2 * 100, 228);
	msps = div_u64(msps, 1000000);

	for (i = 0; i < ARRAY_SIZE(cphy_settle_x1e80100); i++)
		if (msps < cphy_settle_x1e80100[i].max_msps)
			return cphy_settle_x1e80100[i].settle;

	return cphy_settle_x1e80100[ARRAY_SIZE(cphy_settle_x1e80100) - 1].settle;
}

static const struct
mipi_csi2phy_lane_regs lane_regs_x1e80100[] = {
	/* Power up lanes 2ph mode */
	{0x1014, 0xD5, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x101C, 0x7A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x1018, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},

	{0x0094, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x00A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0090, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0098, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0x07, 0x01, CSIPHY_DEFAULT_PARAMS},
	{0x0030, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0000, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0038, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x002C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0034, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x001C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0014, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x003C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0004, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0020, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0008, 0x10, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0010, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0094, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x005C, 0x00, 0x00, CSIPHY_SKEW_CAL},
	{0x0060, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0064, 0x7F, 0x00, CSIPHY_SKEW_CAL},

	{0x0E94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0EA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E94, 0x07, 0x01, CSIPHY_DEFAULT_PARAMS},
	{0x0E30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E28, 0x04, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E00, 0x80, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E0C, 0xFF, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E38, 0x1F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0E08, 0x10, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0E10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},

	{0x0494, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x04A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0490, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0498, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0x07, 0x01, CSIPHY_DEFAULT_PARAMS},
	{0x0430, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0400, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0438, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x042C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0434, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x041C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0414, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x043C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0404, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0420, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0408, 0x10, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0410, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0494, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x045C, 0x00, 0x00, CSIPHY_SKEW_CAL},
	{0x0460, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0464, 0x7F, 0x00, CSIPHY_SKEW_CAL},

	{0x0894, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x08A0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0890, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0898, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0x07, 0x01, CSIPHY_DEFAULT_PARAMS},
	{0x0830, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0800, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0838, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x082C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0834, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x081C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0814, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x083C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0804, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0820, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0808, 0x10, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0810, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0894, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x085C, 0x00, 0x00, CSIPHY_SKEW_CAL},
	{0x0860, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0864, 0x7F, 0x00, CSIPHY_SKEW_CAL},

	{0x0C94, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0CA0, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C90, 0x0f, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C98, 0x08, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0x07, 0x01, CSIPHY_DEFAULT_PARAMS},
	{0x0C30, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C00, 0x8E, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C38, 0xFE, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C2C, 0x01, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C34, 0x0F, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C1C, 0x0A, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C14, 0x60, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C3C, 0xB8, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C04, 0x0C, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C20, 0x00, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C08, 0x10, 0x00, CSIPHY_SETTLE_CNT_LOWER_BYTE},
	{0x0C10, 0x52, 0x00, CSIPHY_DEFAULT_PARAMS},
	{0x0C94, 0xD7, 0x00, CSIPHY_SKEW_CAL},
	{0x0C5C, 0x00, 0x00, CSIPHY_SKEW_CAL},
	{0x0C60, 0xBD, 0x00, CSIPHY_SKEW_CAL},
	{0x0C64, 0x7F, 0x00, CSIPHY_SKEW_CAL},
};

static inline const struct mipi_csi2phy_device_regs *
csi2phy_dev_to_regs(const struct mipi_csi2phy_device *csi2phy)
{
	return &csi2phy->soc_cfg->reg_info;
}

static void phy_qcom_mipi_csi2_hw_version_read(struct mipi_csi2phy_device *csi2phy)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	u32 hw_version;

	writel(CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_SHOW_REV_ID, csi2phy->base +
	       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 6));

	hw_version = readl_relaxed(csi2phy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->offset, 12));
	hw_version |= readl_relaxed(csi2phy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->offset, 13)) << 8;
	hw_version |= readl_relaxed(csi2phy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->offset, 14)) << 16;
	hw_version |= readl_relaxed(csi2phy->base +
				   CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->offset, 15)) << 24;

	csi2phy->hw_version = hw_version;

	dev_dbg(csi2phy->dev, "CSIPHY 3PH HW Version = 0x%08x\n", hw_version);
}

/*
 * phy_qcom_mipi_csi2_reset - Perform software reset on CSIPHY module
 * @phy_qcom_mipi_csi2: CSIPHY device
 */
static void phy_qcom_mipi_csi2_reset(struct mipi_csi2phy_device *csi2phy)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);

	writel_relaxed(0x1, csi2phy->base +
		      CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 0));
	usleep_range(5000, 8000);
	writel_relaxed(0x0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 0));
}

static irqreturn_t phy_qcom_mipi_csi2_isr(int irq, void *dev)
{
	const struct mipi_csi2phy_device *csi2phy = dev;
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	int i;

	for (i = 0; i < 11; i++) {
		int c = i + 22;
		u8 val = readl_relaxed(csi2phy->base +
				       CSIPHY_3PH_CMN_CSI_COMMON_STATUSn(regs->offset, i));

		writel_relaxed(val, csi2phy->base +
			       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, c));
	}

	writel_relaxed(0x1, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 10));
	writel_relaxed(0x0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 10));

	for (i = 22; i < 33; i++) {
		writel_relaxed(0x0, csi2phy->base +
			       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, i));
	}

	return IRQ_HANDLED;
}

/*
 * phy_qcom_mipi_csi2_settle_cnt_calc - Calculate settle count value
 *
 * Helper function to calculate settle count value. This is
 * based on the CSI2 T_hs_settle parameter which in turn
 * is calculated based on the CSI2 transmitter link frequency.
 *
 * Return settle count value or 0 if the CSI2 link frequency
 * is not available
 */
static u8 phy_qcom_mipi_csi2_settle_cnt_calc(s64 link_freq, u32 timer_clk_rate)
{
	u32 ui; /* ps */
	u32 timer_period; /* ps */
	u32 t_hs_prepare_max; /* ps */
	u32 t_hs_settle; /* ps */
	u8 settle_cnt;

	if (link_freq <= 0)
		return 0;

	ui = div_u64(1000000000000LL, link_freq);
	ui /= 2;
	t_hs_prepare_max = 85000 + 6 * ui;
	t_hs_settle = t_hs_prepare_max;

	timer_period = div_u64(1000000000000LL, timer_clk_rate);
	settle_cnt = t_hs_settle / timer_period - 6;

	return settle_cnt;
}

static void phy_qcom_mipi_csi2_gen1_config_lanes(struct mipi_csi2phy_device *csi2phy,
						 struct mipi_csi2phy_stream_cfg *cfg,
						 u8 settle_cnt)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	struct mipi_csi2phy_lanes_cfg *lane_cfg = &cfg->lane_cfg;
	int i, l = 0;
	u8 val;

	for (i = 0; i <= cfg->num_data_lanes; i++) {
		if (i == cfg->num_data_lanes)
			l = 7;
		else
			l = lane_cfg->data[i].pos * 2;

		val = CSIPHY_3PH_LNn_CFG1_SWI_REC_DLY_PRG;
		val |= 0x17;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG1(l));

		val = CSIPHY_3PH_LNn_CFG2_LP_REC_EN_INT;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG2(l));

		val = settle_cnt;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG3(l));

		val = CSIPHY_3PH_LNn_CFG5_T_HS_DTERM |
			CSIPHY_3PH_LNn_CFG5_HS_REC_EQ_FQ_INT;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG5(l));

		val = CSIPHY_3PH_LNn_CFG6_SWI_FORCE_INIT_EXIT;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG6(l));

		val = CSIPHY_3PH_LNn_CFG7_SWI_T_INIT;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG7(l));

		val = CSIPHY_3PH_LNn_CFG8_SWI_SKIP_WAKEUP |
			CSIPHY_3PH_LNn_CFG8_SKEW_FILTER_ENABLE;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG8(l));

		val = CSIPHY_3PH_LNn_CFG9_SWI_T_WAKEUP;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG9(l));

		val = CSIPHY_3PH_LNn_TEST_IMP_HS_TERM_IMP;
		writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_TEST_IMP(l));

		val = CSIPHY_3PH_LNn_CSI_LANE_CTRL15_SWI_SOT_SYMBOL;
		writel_relaxed(val, csi2phy->base +
				    CSIPHY_3PH_LNn_CSI_LANE_CTRL15(l));
	}

	val = CSIPHY_3PH_LNn_CFG1_SWI_REC_DLY_PRG;
	writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG1(l));

	if (regs->generation == GEN1_660)
		val = CSIPHY_3PH_LNn_CFG4_T_HS_CLK_MISS_660;
	else
		val = CSIPHY_3PH_LNn_CFG4_T_HS_CLK_MISS;
	writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_CFG4(l));

	val = CSIPHY_3PH_LNn_MISC1_IS_CLKLANE;
	writel_relaxed(val, csi2phy->base + CSIPHY_3PH_LNn_MISC1(l));
}

static void
phy_qcom_mipi_csi2_gen2_config_lanes(struct mipi_csi2phy_device *csi2phy,
				     u8 settle_cnt)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	const struct mipi_csi2phy_lane_regs *r = regs->init_seq;
	int i, array_size = regs->lane_array_size;
	u32 val;

	if (cphy_force) {
		r = lane_regs_x1e80100_cphy;
		array_size = ARRAY_SIZE(lane_regs_x1e80100_cphy);
	}

	for (i = 0; i < array_size; i++, r++) {
		switch (r->mipi_csi2phy_param_type) {
		case CSIPHY_SETTLE_CNT_LOWER_BYTE:
			val = settle_cnt & 0xff;
			break;
		case CSIPHY_SKEW_CAL:
			/* TODO: support application of skew from dt flag */
			continue;
		case CSIPHY_DNP_PARAMS:
			continue;
		default:
			val = r->reg_data;
			break;
		}
		writel_relaxed(val, csi2phy->base + r->reg_addr);
		if (r->delay_us)
			udelay(r->delay_us);
	}
}

static bool phy_qcom_mipi_csi2_is_gen2(struct mipi_csi2phy_device *csi2phy)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);

	return regs->generation == GEN2;
}

static int phy_qcom_mipi_csi2_lanes_enable(struct mipi_csi2phy_device *csi2phy,
					   struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);
	struct mipi_csi2phy_lanes_cfg *lane_cfg = &cfg->lane_cfg;
	u8 settle_cnt;
	u8 val;
	u8 ctrl5_val;
	int i;

	if (cphy_force)
		settle_cnt = phy_qcom_mipi_csi2_cphy_settle_cnt(cfg->link_freq);
	else
		settle_cnt = phy_qcom_mipi_csi2_settle_cnt_calc(cfg->link_freq,
								csi2phy->timer_clk_rate);

	if (cphy_force) {
		/*
		 * One C-PHY trio. Windows writes 0x02 and, unlike D-PHY's 0x81,
		 * sets no clock-lane bit -- C-PHY embeds the clock.
		 */
		val = 0x02;
	} else {
		val = CSIPHY_3PH_CMN_CSI_COMMON_CTRL5_CLK_ENABLE;
		for (i = 0; i < cfg->num_data_lanes; i++)
			val |= BIT(lane_cfg->data[i].pos * 2);
	}
	ctrl5_val = val;

	writel_relaxed(val, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 5));

	val = CSIPHY_3PH_CMN_CSI_COMMON_CTRL6_COMMON_PWRDN_B;
	writel_relaxed(val, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 6));

	val = 0x02;
	writel_relaxed(val, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 7));

	val = 0x00;
	writel_relaxed(val, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 0));

	dev_info(csi2phy->dev,
		 "csiphy: %s lanes, settle_cnt 0x%02x, ctrl5 0x%02x, link_freq %lld, timer_clk %u\n",
		 cphy_force ? "C-PHY (forced)" : "D-PHY",
		 settle_cnt, ctrl5_val, cfg->link_freq, csi2phy->timer_clk_rate);

	if (phy_qcom_mipi_csi2_is_gen2(csi2phy))
		phy_qcom_mipi_csi2_gen2_config_lanes(csi2phy, settle_cnt);
	else
		phy_qcom_mipi_csi2_gen1_config_lanes(csi2phy, cfg, settle_cnt);

	/* IRQ_MASK registers - disable all interrupts */
	for (i = 11; i < 22; i++) {
		writel_relaxed(0, csi2phy->base +
			       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, i));
	}

	return 0;
}

static void
phy_qcom_mipi_csi2_lanes_disable(struct mipi_csi2phy_device *csi2phy,
				 struct mipi_csi2phy_stream_cfg *cfg)
{
	const struct mipi_csi2phy_device_regs *regs = csi2phy_dev_to_regs(csi2phy);

	writel_relaxed(0, csi2phy->base +
		       CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 5));

	writel_relaxed(0, csi2phy->base +
			  CSIPHY_3PH_CMN_CSI_COMMON_CTRLn(regs->offset, 6));
}

static int phy_qcom_mipi_csi2_init(struct mipi_csi2phy_device *csi2phy)
{
	return 0;
}

const struct mipi_csi2phy_hw_ops phy_qcom_mipi_csi2_ops_3ph_1_0 = {
	.hw_version_read = phy_qcom_mipi_csi2_hw_version_read,
	.reset = phy_qcom_mipi_csi2_reset,
	.lanes_enable = phy_qcom_mipi_csi2_lanes_enable,
	.lanes_disable = phy_qcom_mipi_csi2_lanes_disable,
	.isr = phy_qcom_mipi_csi2_isr,
	.init = phy_qcom_mipi_csi2_init,
};

const struct mipi_csi2phy_clk_freq zero = { 0 };

const struct mipi_csi2phy_clk_freq dphy_4nm_x1e_csiphy = {
	.freq = {
		300000000, 400000000, 480000000
	},
	.num_freq = 3,
};

const struct mipi_csi2phy_clk_freq dphy_4nm_x1e_csiphy_timer = {
	.freq = {
		266666667, 400000000
	},
	.num_freq = 2,
};

const struct mipi_csi2phy_soc_cfg mipi_csi2_dphy_4nm_x1e = {
	.ops = &phy_qcom_mipi_csi2_ops_3ph_1_0,
	.reg_info = {
		.init_seq = lane_regs_x1e80100,
		.lane_array_size = ARRAY_SIZE(lane_regs_x1e80100),
		.offset = 0x1000,
		.generation = GEN2,
	},
	.supply_names = (const char *[]){
		"vdda-0p8",
		"vdda-1p2"
	},
	.num_supplies = 2,
	.clk_names = (const char *[]) {
		"camnoc_axi",
		"cpas_ahb",
		"csiphy",
		"csiphy_timer"
	},
	.num_clk = 4,
	.clk_freq = {
		zero,
		zero,
		dphy_4nm_x1e_csiphy,
		dphy_4nm_x1e_csiphy_timer,
	},
};
