// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX681 image sensor driver
 *
 * The front camera of the Microsoft Surface Pro 11 (Snapdragon X1E80100),
 * where it appears in ACPI as SONY0681.
 *
 * No datasheet was available. The register sequences below were recovered from
 * the QTI Chromatix sensor-module blob that ships with the Windows camera
 * driver (com.surface.sensormodule.ffc_imx681.bin), whose format was worked out
 * by decoding the same structure for the OV13858 -- a sensor whose real tables
 * are in mainline as ov13858.c -- and checking that the decode reproduced them.
 *
 * The recovered values corroborate each other and the platform description:
 *
 *   0x0136/0x0137 = 0x13/0x33   EXTCLK 19.2 MHz in 8.8 fixed point, matching
 *                               the MCLK rate in the ACPI power tables
 *   0x0112/0x0113 = 0x0a/0x0a   RAW10
 *   0x0114        = 0x00        one CSI-2 data lane
 *   0x30eb        = 0x05, 0x0c  the usual Sony vendor unlock
 *
 * and every mode's x_addr_end - x_addr_start + 1 equals its x_output_size.
 *
 * UNVERIFIED ASSUMPTIONS
 *
 * Nothing has streamed with this driver, and the blob does not describe
 * everything a driver needs. These are inferences from the SMIA/CCS
 * conventions Sony follows, not things the recovered data states:
 *
 *  - Bayer order. The blob never writes 0x0101 (image_orientation), so the
 *    CFA phase is unknown. SRGGB10 is a guess and has a one-in-four chance
 *    of being right; a captured frame with wrong colour will say so at once.
 *  - Streaming is started and stopped through 0x0100. Standard for SMIA,
 *    but the blob never writes it -- the Windows stack does that itself.
 *  - Exposure at 0x0202 and analogue gain at 0x0204 are the SMIA-standard
 *    locations. The blob writes 0x0204 (to zero) but never 0x0202.
 *  - Analogue gain range 0..1023. A typical IMX value, not a measured one.
 *  - Link frequency is computed from the op-PLL dividers assuming
 *    op_sys_clk_div is 1, because the blob never writes 0x030b.
 *  - There is no chip-ID check, because no model-ID register appears in the
 *    blob and none is documented here. The driver will therefore bind to
 *    whatever answers at its I2C address.
 *
 * Frame length (0x0340) is not written by any recovered table either, so
 * rather than invent one the driver reads it back from the sensor after
 * programming a mode and derives the vertical blanking limits from that.
 *
 * WHAT THE VENDOR'S OWN SENSOR LIB DOES
 *
 * Qualcomm's CamX carries exactly one IMX681-specific function,
 * GetSensorModeIndex(). It is entered for a request of 4032x3024 at 30 fps or
 * below, and returns the index of the 3520x2640 mode -- so the vendor caps
 * this sensor at 30 fps and substitutes the smaller mode for a full-array
 * request at that rate.
 *
 * The mode list below is still ordered largest-first, per the usual
 * convention. Whether 4032x3024 is actually reachable at 30 fps over a single
 * lane is untested; if it is not, that substitution is the reason, and a
 * consumer that wants 30 fps should ask for 3520x2640.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define IMX681_REG_MODE_SELECT		0x0100
#define IMX681_MODE_STANDBY		0x00
#define IMX681_MODE_STREAMING		0x01

#define IMX681_REG_FRAME_LENGTH		0x0340
#define IMX681_REG_EXPOSURE		0x0202
#define IMX681_EXPOSURE_MIN		4
#define IMX681_EXPOSURE_STEP		1
#define IMX681_EXPOSURE_DEFAULT		0x0640

#define IMX681_REG_ANALOG_GAIN		0x0204
#define IMX681_ANA_GAIN_MIN		0
#define IMX681_ANA_GAIN_MAX		1023
#define IMX681_ANA_GAIN_STEP		1
#define IMX681_ANA_GAIN_DEFAULT		0

/* Sony sensors express EXTCLK as 8.8 fixed point; 0x1333 is 19.2 MHz. */
#define IMX681_XCLK_FREQ		19200000

#define IMX681_VBLANK_MIN		4
#define IMX681_VTS_MAX			0xffff

/*
 * Used only if reading frame_length_lines back from the sensor fails. It is a
 * placeholder, not a specified value.
 */
#define IMX681_VTS_FALLBACK(h)		((h) + 128)

static const char * const imx681_supply_names[] = {
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
};

struct imx681_reg {
	u16 address;
	u8 val;
};

struct imx681_reg_list {
	u32 num_of_regs;
	const struct imx681_reg *regs;
};

struct imx681_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 link_freq_index;
	struct imx681_reg_list reg_list;
};

static const struct imx681_reg imx681_init_regs[] = {
	{0x0136, 0x13},
	{0x0137, 0x33},
	{0x002c, 0x05},
	{0x002d, 0x05},
	{0x0111, 0x03},
	{0x30eb, 0x05},
	{0x30eb, 0x0c},
	{0x300a, 0xff},
	{0x300b, 0xff},
	{0x3532, 0xff},
	{0x3533, 0xff},
	{0x051e, 0x00},
	{0x0905, 0x04},
	{0x2029, 0x01},
	{0x202a, 0x11},
	{0x20a1, 0x00},
	{0x20a2, 0x02},
	{0x20a3, 0x03},
	{0x20ac, 0x01},
	{0x20ad, 0x01},
	{0x20ae, 0x01},
	{0x20af, 0x01},
	{0x20b0, 0x00},
	{0x20b1, 0x01},
	{0x20b2, 0x02},
	{0x20b3, 0x03},
	{0x706f, 0x00},
	{0x7130, 0x08},
	{0x7131, 0x08},
	{0x7408, 0x89},
	{0x7437, 0x3d},
	{0x7439, 0x29},
	{0x7443, 0x38},
	{0x7447, 0x55},
	{0x744b, 0x00},
	{0x7451, 0x8e},
	{0x746d, 0x29},
	{0x747d, 0x68},
	{0x7481, 0x60},
	{0x7491, 0x2d},
	{0x7493, 0x31},
	{0x74a5, 0x52},
	{0x74af, 0x4a},
	{0x74b5, 0x1f},
	{0x74b7, 0x31},
	{0x74bd, 0x75},
	{0x74c5, 0x06},
	{0x74c9, 0x52},
	{0x74d3, 0x4a},
	{0x74d9, 0x1f},
	{0x74db, 0x31},
	{0x74e1, 0x75},
	{0x74e9, 0x06},
	{0x74ed, 0x52},
	{0x74f7, 0x4a},
	{0x74fd, 0x1f},
	{0x74ff, 0x31},
	{0x7505, 0x75},
	{0x750d, 0x06},
	{0x7537, 0x38},
	{0x753d, 0x4a},
	{0x753f, 0x4a},
	{0x7541, 0x4a},
	{0x7549, 0x8e},
	{0x754f, 0x75},
	{0x7551, 0x75},
	{0x7553, 0x75},
	{0x792b, 0x39},
	{0x792d, 0x43},
	{0x79d3, 0x25},
	{0x79d6, 0x8e},
	{0x79d7, 0x01},
	{0x79d8, 0xe7},
	{0x79d9, 0x25},
	{0x79db, 0x76},
	{0x79dc, 0x8e},
	{0x79dd, 0x01},
	{0x79de, 0xe7},
	{0x79df, 0x25},
	{0x79e1, 0x76},
	{0x79e2, 0x8e},
	{0x79e3, 0x01},
	{0x79e4, 0xe7},
	{0x79e5, 0x25},
	{0x79e7, 0x76},
	{0x79e8, 0x8e},
	{0x7a01, 0xff},
	{0x7a29, 0x6c},
	{0x7a2b, 0xda},
	{0x7a34, 0x6c},
	{0x7a37, 0xda},
	{0x7a40, 0x6c},
	{0x7a43, 0xda},
	{0x7b08, 0x00},
	{0x7b09, 0x01},
	{0x7c03, 0x38},
	{0x7c09, 0x4a},
	{0x7c0b, 0x4a},
	{0x7c0d, 0x4a},
	{0x7c13, 0x8e},
	{0x7c19, 0x75},
	{0x7c1b, 0x75},
	{0x7c1d, 0x75},
	{0x7c90, 0x00},
	{0x7c91, 0x00},
	{0x7c92, 0x00},
	{0x7c9d, 0x01},
	{0x7c9e, 0x01},
	{0x7c9f, 0x01},
	{0x7e9b, 0x07},
	{0x7f09, 0x00},
	{0x7f36, 0x00},
	{0x7f4f, 0x0a},
	{0x7f50, 0x0a},
	{0x7f51, 0x0a},
	{0x7f55, 0x05},
	{0x7f56, 0x05},
	{0x7f57, 0x05},
	{0x7f5b, 0x03},
	{0x7f5c, 0x03},
	{0x7f5d, 0x03},
	{0x7f61, 0x03},
	{0x7f62, 0x03},
	{0x7f63, 0x03},
	{0x7f67, 0x03},
	{0x7f68, 0x03},
	{0x7f69, 0x03},
	{0x7f6a, 0x05},
	{0x7f6b, 0x05},
	{0x7f6c, 0x05},
	{0x7f6d, 0x11},
	{0x7f6e, 0x14},
	{0x7f6f, 0x14},
	{0x7f73, 0x14},
	{0x7f74, 0x1c},
	{0x7f75, 0x14},
	{0x7f76, 0x08},
	{0x7f79, 0x14},
	{0x7f7a, 0x1c},
	{0x7f7b, 0x14},
	{0x7f7f, 0x1c},
	{0x7f80, 0x1c},
	{0x7f81, 0x1c},
	{0x7f85, 0x1c},
	{0x7f86, 0x1c},
	{0x7f87, 0x1c},
	{0x7f9d, 0x09},
	{0x7f9e, 0x09},
	{0x7f9f, 0x09},
	{0x7fa3, 0x09},
	{0x7fa4, 0x09},
	{0x7fa5, 0x09},
	{0x7fac, 0x00},
	{0x7fad, 0x00},
	{0x7fae, 0x00},
	{0x7faf, 0x00},
	{0x7fb0, 0x00},
	{0x7fb1, 0x00},
	{0x7fb2, 0x00},
	{0x7fb3, 0x00},
	{0x7fb4, 0x00},
	{0x7fb5, 0x00},
	{0x7fb6, 0x00},
	{0x7fb7, 0x00},
	{0x7fb8, 0x00},
	{0x7fb9, 0x00},
	{0x7fba, 0x00},
	{0x7fbb, 0x00},
	{0x7fbc, 0x00},
	{0x7fbd, 0x00},
	{0x7fbe, 0x00},
	{0x7fbf, 0x00},
	{0x7fc0, 0x00},
	{0x7fc1, 0x00},
	{0x7fc2, 0x00},
	{0x7fc3, 0x00},
	{0x7fcb, 0x37},
	{0x7fcd, 0x37},
	{0x7fcf, 0x37},
	{0x7fd7, 0x44},
	{0x7fd9, 0x44},
	{0x7fdb, 0x44},
	{0x7fdd, 0x38},
	{0x7fe3, 0x4a},
	{0x7fe5, 0x4a},
	{0x7fe7, 0x4a},
	{0x7fef, 0x4a},
	{0x7ff1, 0x4a},
	{0x7ff3, 0x4a},
	{0x7ffb, 0x4a},
	{0x7ffd, 0x4a},
	{0x7fff, 0x4a},
	{0x8007, 0x62},
	{0x8009, 0x62},
	{0x800b, 0x62},
	{0x8013, 0x6f},
	{0x8015, 0x6f},
	{0x8017, 0x6f},
	{0x8019, 0x8e},
	{0x801f, 0x75},
	{0x8021, 0x75},
	{0x8023, 0x75},
	{0x802b, 0x75},
	{0x802d, 0x75},
	{0x802f, 0x75},
	{0x8037, 0x75},
	{0x8039, 0x75},
	{0x803b, 0x75},
	{0x803c, 0x13},
	{0x803d, 0x17},
	{0x803e, 0x15},
	{0x803f, 0x11},
	{0x8040, 0x0a},
	{0x8041, 0x08},
	{0x8047, 0x17},
	{0x80f0, 0x24},
	{0x80f1, 0x1b},
	{0x80f2, 0x1a},
	{0x80f3, 0x14},
	{0x80f4, 0x14},
	{0x80f5, 0x12},
	{0x80f6, 0x25},
	{0x80f7, 0x1c},
	{0x80f8, 0x1b},
	{0x80f9, 0x18},
	{0x80fa, 0x17},
	{0x80fb, 0x18},
	{0x80fc, 0x26},
	{0x80fd, 0x1e},
	{0x80fe, 0x1d},
	{0x80ff, 0x1c},
	{0x8100, 0x1b},
	{0x8101, 0x1c},
	{0x8102, 0x27},
	{0x8103, 0x1e},
	{0x8104, 0x1d},
	{0x8105, 0x1e},
	{0x8106, 0x1e},
	{0x8107, 0x1e},
	{0x8108, 0x27},
	{0x8109, 0x1e},
	{0x810a, 0x1e},
	{0x810b, 0x1e},
	{0x810c, 0x1e},
	{0x810d, 0x1f},
	{0x810e, 0x00},
	{0x8168, 0x0b},
	{0x8169, 0x0b},
	{0x816a, 0x09},
	{0x816b, 0x0f},
	{0x816c, 0x0f},
	{0x816d, 0x0f},
	{0x816e, 0x0b},
	{0x816f, 0x0b},
	{0x8170, 0x0a},
	{0x8171, 0x0f},
	{0x8172, 0x0f},
	{0x8173, 0x0f},
	{0x8174, 0x0d},
	{0x8175, 0x0c},
	{0x8176, 0x09},
	{0x8177, 0x0f},
	{0x8178, 0x0f},
	{0x8179, 0x0f},
	{0x817a, 0x0c},
	{0x817b, 0x0d},
	{0x817c, 0x09},
	{0x817d, 0x0f},
	{0x817e, 0x0f},
	{0x817f, 0x0f},
	{0x8180, 0x0d},
	{0x8181, 0x0d},
	{0x8182, 0x09},
	{0x8183, 0x0f},
	{0x8184, 0x0f},
	{0x8185, 0x0f},
	{0x81b0, 0x03},
	{0x81e3, 0x04},
	{0x81e4, 0x04},
	{0x81e9, 0x04},
	{0x81ea, 0x04},
	{0x81ef, 0x04},
	{0x81f0, 0x04},
	{0x9186, 0x00},
	{0xd030, 0x01},
	{0xd04c, 0x10},
	{0xd123, 0x75},
	{0xd144, 0x10},
	{0xd1af, 0x08},
	{0xd1bd, 0x67},
	{0xd1d4, 0x04},
	{0xd1d5, 0x04},
	{0xd1d6, 0x07},
	{0xd1d7, 0x07},
	{0xd1d9, 0x40},
	{0xd1db, 0x58},
	{0xd1dd, 0xd4},
	{0xd1df, 0xd4},
	{0xd1e1, 0xd4},
	{0xd348, 0x0f},
	{0xd357, 0x00},
	{0xd3ae, 0x11},
	{0xd3af, 0x44},
	{0xd3b1, 0x7d},
	{0xd803, 0xf0},
	{0xd80b, 0xf0},
	{0xd813, 0xf1},
	{0xd81b, 0xf0},
	{0xd843, 0xf1},
	{0xd84f, 0xf0},
	{0xd934, 0x23},
	{0xd935, 0xc8},
	{0xd938, 0x27},
	{0xd939, 0x10},
	{0xd93a, 0x23},
	{0xd93b, 0xc8},
	{0xd955, 0x07},
	{0xd95a, 0x04},
	{0xd95b, 0x0a},
	{0xd95c, 0x1e},
	{0xd95d, 0x00},
	{0xd95e, 0x14},
	{0xd95f, 0x21},
	{0xd960, 0x00},
	{0xd961, 0x00},
	{0xd962, 0x0a},
	{0xd963, 0x50},
	{0xd964, 0x0a},
	{0xd965, 0xa0},
	{0xd966, 0x00},
	{0xd967, 0x28},
	{0xd968, 0x0a},
	{0xd969, 0x50},
	{0xd96a, 0x0a},
	{0xd96b, 0xa0},
	{0xd96c, 0x00},
	{0xd96d, 0x00},
	{0xd96e, 0x0a},
	{0xd96f, 0x44},
	{0xd970, 0x0a},
	{0xd971, 0x50},
	{0xd972, 0x00},
	{0xd973, 0x00},
	{0xd974, 0x0a},
	{0xd975, 0x44},
	{0xd976, 0x0a},
	{0xd977, 0x50},
	{0xda10, 0x00},
	{0xda11, 0x14},
	{0xda12, 0x64},
	{0xda13, 0x00},
	{0xda14, 0x14},
	{0xda15, 0xc8},
	{0xda22, 0x00},
	{0xda23, 0x56},
	{0xda24, 0x00},
	{0xda25, 0xb5},
	{0xda26, 0x00},
	{0xda27, 0xe8},
	{0xda28, 0x08},
	{0xda29, 0xa6},
	{0xda2a, 0x00},
	{0xda2b, 0xa2},
};

static const struct imx681_reg imx681_mode_4032x3024[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x1a},
	{0x0343, 0x60},
	{0x033d, 0x00},
	{0x033e, 0x0d},
	{0x033f, 0xe2},
	{0x0344, 0x00},
	{0x0345, 0x08},
	{0x0346, 0x00},
	{0x0347, 0x40},
	{0x0348, 0x0f},
	{0x0349, 0xc7},
	{0x034a, 0x0c},
	{0x034b, 0x0f},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x02},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xc0},
	{0x040e, 0x0b},
	{0x040f, 0xd0},
	{0x034c, 0x0f},
	{0x034d, 0xc0},
	{0x034e, 0x0b},
	{0x034f, 0xd0},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xb4},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0xd0},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x03},
	{0x022b, 0xe8},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x00},
	{0x7e9b, 0x07},
	{0xd1ce, 0x00},
	{0xdc3c, 0x00},
	{0x0368, 0x00},
	{0x036a, 0x08},
	{0x036b, 0x70},
	{0xe801, 0x06},
	{0xe802, 0x01},
	{0xe803, 0x32},
	{0xe800, 0x00},
	{0xe804, 0x08},
	{0xe805, 0x08},
	{0xe806, 0x02},
	{0xe807, 0x08},
	{0xe808, 0x00},
	{0xe809, 0x04},
	{0xe80a, 0x00},
	{0xe80b, 0x00},
	{0xe80c, 0x01},
	{0xe80d, 0x00},
	{0xe80e, 0x01},
	{0xe80f, 0x00},
	{0xe810, 0x01},
	{0xe811, 0x00},
	{0xe812, 0x01},
	{0xe813, 0x00},
	{0xe815, 0x01},
	{0xe816, 0x2e},
	{0xe817, 0xbc},
	{0xe819, 0x01},
	{0xe81a, 0x2e},
	{0xe81b, 0xce},
	{0xe81c, 0x0c},
	{0xe81d, 0x18},
	{0xe81e, 0x00},
	{0xe81f, 0x08},
	{0xe820, 0x00},
	{0xe821, 0x40},
	{0xe822, 0x0f},
	{0xe823, 0xc7},
	{0xe824, 0x0c},
	{0xe825, 0x1f},
	{0xe826, 0x01},
	{0xe827, 0xf8},
	{0xe828, 0x01},
	{0xe829, 0x7a},
	{0xe82a, 0x00},
	{0xe82b, 0x00},
	{0xe82c, 0x00},
	{0xe82d, 0x00},
	{0xe82e, 0x01},
	{0xe82f, 0xf8},
	{0xe830, 0x01},
	{0xe831, 0x7a},
	{0xe835, 0x04},
	{0xe836, 0x00},
	{0xe837, 0x60},
	{0xe834, 0x00},
	{0xe838, 0x08},
	{0xe839, 0x08},
	{0xe83e, 0x00},
	{0xe83f, 0x00},
	{0xe840, 0x01},
	{0xe841, 0x00},
	{0xe842, 0x01},
	{0xe843, 0x00},
	{0xe844, 0x01},
	{0xe845, 0x00},
	{0xe846, 0x01},
	{0xe847, 0x00},
	{0xe849, 0x00},
	{0xe84a, 0x03},
	{0xe84b, 0xe8},
	{0xe84d, 0x01},
	{0xe84e, 0x6c},
	{0xe84f, 0x50},
	{0xe850, 0x06},
	{0xe851, 0x08},
	{0xe852, 0x00},
	{0xe853, 0x68},
	{0xe854, 0x00},
	{0xe855, 0x88},
	{0xe856, 0x0f},
	{0xe857, 0x67},
	{0xe858, 0x0b},
	{0xe859, 0xc7},
	{0xe85a, 0x01},
	{0xe85b, 0x00},
	{0xe85c, 0x00},
	{0xe85d, 0x78},
	{0xe85e, 0x00},
	{0xe85f, 0x00},
	{0xe860, 0x00},
	{0xe861, 0x00},
	{0xe862, 0x00},
	{0xe863, 0xa0},
	{0xe864, 0x00},
	{0xe865, 0x78},
	{0xe869, 0x04},
	{0xe86a, 0x00},
	{0xe86b, 0x60},
	{0xe868, 0x00},
	{0xe86c, 0x08},
	{0xe86d, 0x08},
	{0xe872, 0x00},
	{0xe873, 0x00},
	{0xe874, 0x01},
	{0xe875, 0x00},
	{0xe876, 0x01},
	{0xe877, 0x00},
	{0xe878, 0x01},
	{0xe879, 0x00},
	{0xe87a, 0x01},
	{0xe87b, 0x00},
	{0xe87d, 0x00},
	{0xe87e, 0x03},
	{0xe87f, 0xe8},
	{0xe881, 0x00},
	{0xe882, 0x52},
	{0xe883, 0x86},
	{0xe884, 0x0d},
	{0xe885, 0x50},
	{0xe886, 0x00},
	{0xe887, 0x68},
	{0xe888, 0x00},
	{0xe889, 0x88},
	{0xe88a, 0x0f},
	{0xe88b, 0x67},
	{0xe88c, 0x0b},
	{0xe88d, 0xc7},
	{0xe88e, 0x01},
	{0xe88f, 0x00},
	{0xe890, 0x00},
	{0xe891, 0x3c},
	{0xe892, 0x00},
	{0xe893, 0x00},
	{0xe894, 0x00},
	{0xe895, 0x00},
	{0xe896, 0x00},
	{0xe897, 0x50},
	{0xe898, 0x00},
	{0xe899, 0x3c},
};

static const struct imx681_reg imx681_mode_3840x2640[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x1a},
	{0x0343, 0x60},
	{0x033d, 0x00},
	{0x033e, 0x0d},
	{0x033f, 0xe2},
	{0x0344, 0x00},
	{0x0345, 0x68},
	{0x0346, 0x01},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0x67},
	{0x034a, 0x0b},
	{0x034b, 0x4f},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x01},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0x00},
	{0x040e, 0x0a},
	{0x040f, 0x50},
	{0x034c, 0x0f},
	{0x034d, 0x00},
	{0x034e, 0x0a},
	{0x034f, 0x50},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xe1},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x77},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x0d},
	{0x022b, 0xda},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x03},
	{0x7e9b, 0x02},
	{0xd1ce, 0x00},
	{0xdc3c, 0x01},
	{0x0368, 0x00},
	{0x036a, 0x08},
	{0x036b, 0x70},
};

static const struct imx681_reg imx681_mode_3520x2640[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x1a},
	{0x0343, 0x60},
	{0x033d, 0x00},
	{0x033e, 0x0d},
	{0x033f, 0xe2},
	{0x0344, 0x01},
	{0x0345, 0x08},
	{0x0346, 0x01},
	{0x0347, 0x00},
	{0x0348, 0x0e},
	{0x0349, 0xc7},
	{0x034a, 0x0b},
	{0x034b, 0x4f},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x01},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0d},
	{0x040d, 0xc0},
	{0x040e, 0x0a},
	{0x040f, 0x50},
	{0x034c, 0x0d},
	{0x034d, 0xc0},
	{0x034e, 0x0a},
	{0x034f, 0x50},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xe1},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x77},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x0d},
	{0x022b, 0xda},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x03},
	{0x7e9b, 0x02},
	{0xd1ce, 0x00},
	{0xdc3c, 0x01},
	{0x0368, 0x00},
	{0x036a, 0x08},
	{0x036b, 0x70},
};

static const struct imx681_reg imx681_mode_3660x2440[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x1a},
	{0x0343, 0x60},
	{0x033d, 0x00},
	{0x033e, 0x0d},
	{0x033f, 0xe2},
	{0x0344, 0x00},
	{0x0345, 0xc0},
	{0x0346, 0x01},
	{0x0347, 0x64},
	{0x0348, 0x0f},
	{0x0349, 0x0b},
	{0x034a, 0x0a},
	{0x034b, 0xeb},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x01},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0e},
	{0x040d, 0x4c},
	{0x040e, 0x09},
	{0x040f, 0x88},
	{0x034c, 0x0e},
	{0x034d, 0x4c},
	{0x034e, 0x09},
	{0x034f, 0x88},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xe1},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x77},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x0d},
	{0x022b, 0xda},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x03},
	{0x7e9b, 0x02},
	{0xd1ce, 0x00},
	{0xdc3c, 0x01},
	{0x0368, 0x00},
	{0x036a, 0x08},
	{0x036b, 0x70},
};

static const struct imx681_reg imx681_mode_3840x2160_1[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x15},
	{0x0343, 0x20},
	{0x033d, 0x00},
	{0x033e, 0x08},
	{0x033f, 0xaa},
	{0x0344, 0x00},
	{0x0345, 0x68},
	{0x0346, 0x01},
	{0x0347, 0xf0},
	{0x0348, 0x0f},
	{0x0349, 0x67},
	{0x034a, 0x0a},
	{0x034b, 0x5f},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x01},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0x00},
	{0x040e, 0x08},
	{0x040f, 0x70},
	{0x034c, 0x0f},
	{0x034d, 0x00},
	{0x034e, 0x08},
	{0x034f, 0x70},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xe1},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x77},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x08},
	{0x022b, 0xa2},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x03},
	{0x7e9b, 0x02},
	{0xd1ce, 0x00},
	{0xdc3c, 0x01},
	{0x0368, 0x01},
	{0x036a, 0x08},
	{0x036b, 0x70},
};

static const struct imx681_reg imx681_mode_3840x2160_2[] = {
	{0x0110, 0x00},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x00},
	{0x0342, 0x1a},
	{0x0343, 0x60},
	{0x033d, 0x00},
	{0x033e, 0x0d},
	{0x033f, 0xe2},
	{0x0344, 0x00},
	{0x0345, 0x68},
	{0x0346, 0x01},
	{0x0347, 0xf0},
	{0x0348, 0x0f},
	{0x0349, 0x67},
	{0x034a, 0x0a},
	{0x034b, 0x5f},
	{0x017c, 0x01},
	{0x017d, 0x01},
	{0x017e, 0x00},
	{0x017f, 0x01},
	{0x0180, 0x00},
	{0x038c, 0x13},
	{0x038d, 0x33},
	{0x2000, 0x01},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0x00},
	{0x040e, 0x08},
	{0x040f, 0x70},
	{0x034c, 0x0f},
	{0x034d, 0x00},
	{0x034e, 0x08},
	{0x034f, 0x70},
	{0x0301, 0x06},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xe1},
	{0x030d, 0x03},
	{0x030e, 0x01},
	{0x030f, 0x77},
	{0x0323, 0x00},
	{0x0229, 0x00},
	{0x022a, 0x0d},
	{0x022b, 0xda},
	{0xd383, 0x01},
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020e, 0x01},
	{0x020f, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},
	{0x6a83, 0x03},
	{0x7e9b, 0x02},
	{0xd1ce, 0x00},
	{0xdc3c, 0x01},
	{0x0368, 0x00},
	{0x036a, 0x08},
	{0x036b, 0x70},
};

static const s64 link_freq_menu_items[] = {
	998400000ULL,
	1200000000ULL,
};

static const struct imx681_mode supported_modes[] = {
	{
		.width = 4032,
		.height = 3024,
		.hts = 6752,
		.link_freq_index = 0,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_4032x3024),
			.regs = imx681_mode_4032x3024,
		},
	},
	{
		.width = 3840,
		.height = 2640,
		.hts = 6752,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_3840x2640),
			.regs = imx681_mode_3840x2640,
		},
	},
	{
		.width = 3520,
		.height = 2640,
		.hts = 6752,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_3520x2640),
			.regs = imx681_mode_3520x2640,
		},
	},
	{
		.width = 3660,
		.height = 2440,
		.hts = 6752,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_3660x2440),
			.regs = imx681_mode_3660x2440,
		},
	},
	{
		.width = 3840,
		.height = 2160,
		.hts = 5408,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_3840x2160_1),
			.regs = imx681_mode_3840x2160_1,
		},
	},
	{
		.width = 3840,
		.height = 2160,
		.hts = 6752,
		.link_freq_index = 1,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(imx681_mode_3840x2160_2),
			.regs = imx681_mode_3840x2160_2,
		},
	},
};


struct imx681 {
	struct device *dev;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx681_supply_names)];

	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;

	const struct imx681_mode *cur_mode;
	u32 cur_vts;
};

static inline struct imx681 *to_imx681(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx681, sd);
}

static u64 imx681_pixel_rate(const struct imx681_mode *mode)
{
	/*
	 * RAW10 over a single lane: pixel_rate = link_freq * 2 * lanes / bpp.
	 */
	return div_u64(link_freq_menu_items[mode->link_freq_index] * 2, 10);
}

static int imx681_write_regs(struct imx681 *imx681,
			     const struct imx681_reg *regs, u32 len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = cci_write(imx681->regmap, CCI_REG8(regs[i].address),
				regs[i].val, NULL);
		if (ret) {
			dev_err_ratelimited(imx681->dev,
					    "write 0x%04x failed: %d\n",
					    regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int imx681_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx681 *imx681 = to_imx681(sd);
	int ret;

	ret = clk_prepare_enable(imx681->xclk);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx681_supply_names),
				    imx681->supplies);
	if (ret) {
		clk_disable_unprepare(imx681->xclk);
		return ret;
	}

	if (imx681->reset) {
		usleep_range(2000, 2200);
		gpiod_set_value_cansleep(imx681->reset, 0);
		usleep_range(8000, 8500);
	}

	return 0;
}

static int imx681_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx681 *imx681 = to_imx681(sd);

	gpiod_set_value_cansleep(imx681->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(imx681_supply_names),
			       imx681->supplies);
	clk_disable_unprepare(imx681->xclk);

	return 0;
}

/*
 * Read back what the sensor thinks its configuration is, after the mode has
 * been programmed.
 *
 * The recovered tables are not known to be complete: they came from the vendor
 * blob, and the Windows stack demonstrably writes some things itself (0x0100 is
 * the known example). When the sensor is programmed but emits nothing, the
 * question is always which of these is wrong, and guessing from outside has
 * already cost a round trip.
 *
 * frame_length_lines is the one to watch. No recovered table writes 0x0340, so
 * if it reads back 0 the sensor has no frame height and will not produce frames,
 * whatever else is correct.
 */
static void imx681_debug_dump_state(struct imx681 *imx681, const char *when)
{
	static const struct {
		u32 reg;
		const char *name;
	} regs[] = {
		{ CCI_REG8(0x0100), "mode_select      " },
		{ CCI_REG8(0x0114), "csi_lane_mode    " },
		{ CCI_REG16(0x0112), "csi_data_format  " },
		{ CCI_REG16(0x0136), "extclk_freq_mhz  " },
		{ CCI_REG16(0x0340), "frame_length_line" },
		{ CCI_REG16(0x0342), "line_length_pck  " },
		{ CCI_REG16(0x034c), "x_output_size    " },
		{ CCI_REG16(0x034e), "y_output_size    " },
		{ CCI_REG16(0x0202), "coarse_integ_time" },
		{ CCI_REG16(0x0204), "analogue_gain    " },
		{ CCI_REG8(0x0301), "vt_pix_clk_div   " },
		{ CCI_REG8(0x0303), "vt_sys_clk_div   " },
		{ CCI_REG8(0x0305), "pre_pll_clk_div  " },
		{ CCI_REG16(0x0306), "pll_multiplier   " },
		{ CCI_REG8(0x0309), "op_pix_clk_div   " },
		{ CCI_REG8(0x030b), "op_sys_clk_div   " },
		{ CCI_REG8(0x030d), "op_pre_pll_div   " },
		{ CCI_REG16(0x030e), "op_pll_multiplier" },
	};
	unsigned int i;
	u64 val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		ret = cci_read(imx681->regmap, regs[i].reg, &val, NULL);
		if (ret)
			dev_info(imx681->dev, "STATE[%s]: %s READ FAILED (%d)\n",
				 when, regs[i].name, ret);
		else
			dev_info(imx681->dev, "STATE[%s]: %s = 0x%04llx (%llu)\n",
				 when, regs[i].name, val, val);
	}
}

static int imx681_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx681 *imx681 = to_imx681(sd);
	u64 val;
	int ret;

	if (!enable) {
		cci_write(imx681->regmap, CCI_REG8(IMX681_REG_MODE_SELECT),
			  IMX681_MODE_STANDBY, NULL);
		pm_runtime_put(imx681->dev);
		return 0;
	}

	ret = pm_runtime_resume_and_get(imx681->dev);
	if (ret)
		return ret;

	ret = imx681_write_regs(imx681, imx681_init_regs,
				ARRAY_SIZE(imx681_init_regs));
	if (ret)
		goto err;

	ret = imx681_write_regs(imx681, imx681->cur_mode->reg_list.regs,
				imx681->cur_mode->reg_list.num_of_regs);
	if (ret)
		goto err;

	/*
	 * No recovered table sets frame_length_lines, so ask the sensor what
	 * the mode left it at rather than assuming a value.
	 */
	ret = cci_read(imx681->regmap, CCI_REG16(IMX681_REG_FRAME_LENGTH),
		       &val, NULL);
	if (!ret && val > imx681->cur_mode->height) {
		imx681->cur_vts = val;
		__v4l2_ctrl_modify_range(imx681->vblank, IMX681_VBLANK_MIN,
					 IMX681_VTS_MAX - imx681->cur_mode->height,
					 1, val - imx681->cur_mode->height);
	} else {
		dev_warn(imx681->dev,
			 "frame_length_lines unusable: ret=%d val=%llu height=%u -- the sensor will probably not stream\n",
			 ret, val, imx681->cur_mode->height);
	}

	imx681_debug_dump_state(imx681, "after-mode");

	ret = __v4l2_ctrl_handler_setup(&imx681->ctrl_handler);
	if (ret)
		goto err;

	ret = cci_write(imx681->regmap, CCI_REG8(IMX681_REG_MODE_SELECT),
			IMX681_MODE_STREAMING, NULL);
	if (ret)
		goto err;

	imx681_debug_dump_state(imx681, "streaming");

	return 0;

err:
	pm_runtime_put(imx681->dev);
	return ret;
}

static int imx681_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx681 *imx681 =
		container_of(ctrl->handler, struct imx681, ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = imx681->cur_mode->height + ctrl->val;


		__v4l2_ctrl_modify_range(imx681->exposure,
					 imx681->exposure->minimum, max - 8,
					 imx681->exposure->step, max - 16);
	}

	if (!pm_runtime_get_if_in_use(imx681->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx681->regmap, CCI_REG16(IMX681_REG_EXPOSURE),
				ctrl->val, NULL);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx681->regmap,
				CCI_REG16(IMX681_REG_ANALOG_GAIN),
				ctrl->val, NULL);
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx681->regmap,
				CCI_REG16(IMX681_REG_FRAME_LENGTH),
				imx681->cur_mode->height + ctrl->val, NULL);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(imx681->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx681_ctrl_ops = {
	.s_ctrl = imx681_set_ctrl,
};

static int imx681_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SRGGB10_1X10;

	return 0;
}

static int imx681_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SRGGB10_1X10)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void imx681_fill_format(const struct imx681_mode *mode,
			       struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
}

static int imx681_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx681 *imx681 = to_imx681(sd);
	const struct imx681_mode *mode;
	s64 hblank;

	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	imx681_fill_format(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	imx681->cur_mode = mode;
	__v4l2_ctrl_s_ctrl(imx681->link_freq, mode->link_freq_index);
	__v4l2_ctrl_s_ctrl_int64(imx681->pixel_rate, imx681_pixel_rate(mode));

	hblank = mode->hts - mode->width;
	__v4l2_ctrl_modify_range(imx681->hblank, hblank, hblank, 1, hblank);

	__v4l2_ctrl_modify_range(imx681->vblank, IMX681_VBLANK_MIN,
				 IMX681_VTS_MAX - mode->height, 1,
				 IMX681_VTS_FALLBACK(mode->height) - mode->height);

	return 0;
}

static int imx681_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	imx681_fill_format(&supported_modes[0],
			   v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops imx681_video_ops = {
	.s_stream = imx681_set_stream,
};

static const struct v4l2_subdev_pad_ops imx681_pad_ops = {
	.enum_mbus_code = imx681_enum_mbus_code,
	.enum_frame_size = imx681_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx681_set_format,
};

static const struct v4l2_subdev_ops imx681_subdev_ops = {
	.video = &imx681_video_ops,
	.pad = &imx681_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx681_internal_ops = {
	.init_state = imx681_init_state,
};

static int imx681_init_controls(struct imx681 *imx681)
{
	const struct imx681_mode *mode = imx681->cur_mode;
	struct v4l2_ctrl_handler *hdlr = &imx681->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	s64 hblank, vblank_def, vblank_max;
	int ret;

	ret = v4l2_ctrl_handler_init(hdlr, 10);
	if (ret)
		return ret;

	imx681->link_freq =
		v4l2_ctrl_new_int_menu(hdlr, &imx681_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(link_freq_menu_items) - 1, 0,
				       link_freq_menu_items);
	if (imx681->link_freq)
		imx681->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx681->pixel_rate =
		v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops, V4L2_CID_PIXEL_RATE,
				  1, imx681_pixel_rate(mode), 1,
				  imx681_pixel_rate(mode));
	if (imx681->pixel_rate)
		imx681->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = mode->hts - mode->width;
	imx681->hblank = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
					   V4L2_CID_HBLANK, hblank, hblank, 1,
					   hblank);
	if (imx681->hblank)
		imx681->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = IMX681_VTS_FALLBACK(mode->height) - mode->height;
	vblank_max = IMX681_VTS_MAX - mode->height;
	imx681->vblank = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
					   V4L2_CID_VBLANK, IMX681_VBLANK_MIN,
					   vblank_max, 1, vblank_def);

	imx681->exposure = v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX681_EXPOSURE_MIN,
					     IMX681_VTS_FALLBACK(mode->height) - 8,
					     IMX681_EXPOSURE_STEP,
					     IMX681_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdlr, &imx681_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX681_ANA_GAIN_MIN, IMX681_ANA_GAIN_MAX,
			  IMX681_ANA_GAIN_STEP, IMX681_ANA_GAIN_DEFAULT);

	if (hdlr->error) {
		ret = hdlr->error;
		goto err;
	}

	ret = v4l2_fwnode_device_parse(imx681->dev, &props);
	if (ret)
		goto err;

	ret = v4l2_ctrl_new_fwnode_properties(hdlr, &imx681_ctrl_ops, &props);
	if (ret)
		goto err;

	imx681->sd.ctrl_handler = hdlr;

	return 0;

err:
	v4l2_ctrl_handler_free(hdlr);

	return ret;
}

static int imx681_get_resources(struct imx681 *imx681)
{
	struct device *dev = imx681->dev;
	unsigned int i;
	u32 rate;
	int ret;

	imx681->xclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx681->xclk))
		return dev_err_probe(dev, PTR_ERR(imx681->xclk),
				     "cannot get xclk\n");

	rate = clk_get_rate(imx681->xclk);
	if (rate && rate != IMX681_XCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "xclk is %u, expected %u\n", rate,
				     IMX681_XCLK_FREQ);

	imx681->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx681->reset))
		return dev_err_probe(dev, PTR_ERR(imx681->reset),
				     "cannot get reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(imx681_supply_names); i++)
		imx681->supplies[i].supply = imx681_supply_names[i];

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx681_supply_names),
				      imx681->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get regulators\n");

	return 0;
}

/*
 * DEBUG ONLY -- not for upstream.
 *
 * The sensor's I2C address is not recoverable from the Windows firmware: every
 * Chromatix blob stores exactly one non-empty slaveAddr and it is the module
 * EEPROM's 0xa0, while sensorSlaveAddress is length 0. A plain i2cdetect cannot
 * find it either, because nothing powers the sensor until a driver binds --
 * imx681_power_on() is what enables MCLK, both regulators and releases reset.
 *
 * So scan from inside probe, in the window where the part is powered. The i2c
 * client is created from the DT node whether or not the device answers, so this
 * runs even though the node's reg is a placeholder.
 *
 * Expect two responders on the front sensor's bus: 0x50, the module EEPROM,
 * which confirms the bus is the right one, and the sensor itself.
 */
static bool imx681_addr_scan = true;
module_param_named(addr_scan, imx681_addr_scan, bool, 0644);
MODULE_PARM_DESC(addr_scan, "DEBUG: scan the I2C bus while the sensor is powered");

static void imx681_debug_scan_bus(struct i2c_client *client)
{
	struct i2c_adapter *adap = client->adapter;
	unsigned int addr;
	int found = 0;
	u8 val;

	dev_info(&client->dev, "DEBUG: scanning %s with the sensor powered\n",
		 adap->name);

	for (addr = 0x08; addr <= 0x77; addr++) {
		struct i2c_msg msg = {
			.addr = addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &val,
		};

		if (i2c_transfer(adap, &msg, 1) != 1)
			continue;

		dev_info(&client->dev, "DEBUG:   0x%02x ACK%s\n", addr,
			 addr == 0x50 ? "  (module EEPROM)" : "");
		found++;
	}

	dev_info(&client->dev, "DEBUG: %d responder(s); DT node says 0x%02x\n",
		 found, client->addr);
}

/*
 * Ask the device what it is.
 *
 * Probe otherwise performs no I2C at all: an i2c client is declared by the DT
 * node, not discovered, so it is created whether or not anything answers at
 * that address. Without this, a successful probe proves the clock, the rails,
 * the reset line and the media graph -- and says nothing about the silicon.
 *
 * There is no value to check against, because the vendor blob carries no
 * model-ID register: no probe/chipId/expectedData record group exists in any of
 * the three sensor-module blobs. But that is a fact about the blob, not about
 * the part. SMIA puts a model ID at 0x0000 and CCS a sensor model ID at 0x0016,
 * and Sony's IMX parts follow those conventions, so the registers are very
 * likely implemented even though Qualcomm's stack never reads them.
 *
 * Therefore: logged, never fatal. Gating probe on a value nobody has observed
 * would be inventing a constant that looks researched. If these read back
 * 0x0681 the question is settled and this becomes a real chip-ID check.
 */
static void imx681_debug_identify(struct imx681 *imx681)
{
	static const struct {
		u32 reg;
		const char *name;
	} ids[] = {
		{ CCI_REG16(0x0000), "model_id        (SMIA 0x0000)" },
		{ CCI_REG16(0x0016), "sensor_model_id (CCS  0x0016)" },
	};
	unsigned int i;
	u64 val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		ret = cci_read(imx681->regmap, ids[i].reg, &val, NULL);
		if (ret)
			dev_info(imx681->dev, "ID: %s read failed (%d)\n",
				 ids[i].name, ret);
		else
			dev_info(imx681->dev, "ID: %s = 0x%04llx\n",
				 ids[i].name, val);
	}
}

static int imx681_probe(struct i2c_client *client)
{
	struct imx681 *imx681;
	int ret;

	imx681 = devm_kzalloc(&client->dev, sizeof(*imx681), GFP_KERNEL);
	if (!imx681)
		return -ENOMEM;

	imx681->dev = &client->dev;
	imx681->cur_mode = &supported_modes[0];

	imx681->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx681->regmap))
		return dev_err_probe(imx681->dev, PTR_ERR(imx681->regmap),
				     "cannot init regmap\n");

	ret = imx681_get_resources(imx681);
	if (ret)
		return ret;

	v4l2_i2c_subdev_init(&imx681->sd, client, &imx681_subdev_ops);
	imx681->sd.internal_ops = &imx681_internal_ops;
	imx681->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx681->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx681->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = imx681_power_on(imx681->dev);
	if (ret)
		return ret;

	if (imx681_addr_scan)
		imx681_debug_scan_bus(client);

	/* The only I2C that probe() ever does. */
	imx681_debug_identify(imx681);

	ret = imx681_init_controls(imx681);
	if (ret)
		goto err_power_off;

	ret = media_entity_pads_init(&imx681->sd.entity, 1, &imx681->pad);
	if (ret)
		goto err_free_ctrls;

	ret = v4l2_subdev_init_finalize(&imx681->sd);
	if (ret)
		goto err_media_cleanup;

	pm_runtime_set_active(imx681->dev);
	pm_runtime_enable(imx681->dev);

	ret = v4l2_async_register_subdev_sensor(&imx681->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(imx681->dev);

	return 0;

err_pm:
	pm_runtime_disable(imx681->dev);
	pm_runtime_set_suspended(imx681->dev);
	v4l2_subdev_cleanup(&imx681->sd);
err_media_cleanup:
	media_entity_cleanup(&imx681->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&imx681->ctrl_handler);
err_power_off:
	imx681_power_off(imx681->dev);

	return ret;
}

static void imx681_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx681 *imx681 = to_imx681(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&imx681->ctrl_handler);

	pm_runtime_disable(imx681->dev);
	if (!pm_runtime_status_suspended(imx681->dev)) {
		imx681_power_off(imx681->dev);
		pm_runtime_set_suspended(imx681->dev);
	}
}

static const struct of_device_id imx681_of_match[] = {
	{ .compatible = "sony,imx681" },
	{ }
};
MODULE_DEVICE_TABLE(of, imx681_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id imx681_acpi_ids[] = {
	{ "SONY0681" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, imx681_acpi_ids);
#endif

static DEFINE_RUNTIME_DEV_PM_OPS(imx681_pm_ops, imx681_power_off,
				 imx681_power_on, NULL);

static struct i2c_driver imx681_i2c_driver = {
	.driver = {
		.name = "imx681",
		.of_match_table = imx681_of_match,
		.acpi_match_table = ACPI_PTR(imx681_acpi_ids),
		.pm = pm_sleep_ptr(&imx681_pm_ops),
	},
	.probe = imx681_probe,
	.remove = imx681_remove,
};
module_i2c_driver(imx681_i2c_driver);

MODULE_DESCRIPTION("Sony IMX681 sensor driver");
MODULE_LICENSE("GPL");
