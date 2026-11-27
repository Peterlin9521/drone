// SPDX-License-Identifier: GPL-2.0-only
/*
 * OV9281 (1MP mono global shutter) sensor driver for the Allwinner VIN
 * camera framework, tested target: Orange Pi Zero 3W (Allwinner A733,
 * vendor kernel 6.6.98-sun60iw2).
 *
 * Structure (power sequence, VIN glue, CCI probe) is derived from the
 * Allwinner vendor IMX219 driver:
 *   Copyright (c) 2017-2023 Allwinner Technology Co., Ltd.
 *   Authors: Zhao Wei, Yang Feng
 *
 * Register tables are taken from the mainline Linux ov9282 driver
 * (drivers/media/i2c/ov9282.c), which also supports OV9281:
 *   Copyright (C) 2021 Intel Corporation
 *
 * Notes
 * - VIN has no MEDIA_BUS_FMT_Y10 path, so the mono RAW10 stream is reported
 *   as MEDIA_BUS_FMT_SBGGR10_1X10. The MIPI data type (RAW10) is the same;
 *   every pixel is simply a grey value. Capture it with ISP bypassed
 *   (sensor0_isp_used = 0) as V4L2_PIX_FMT_SBGGR10 and treat it as Y10.
 * - One mode: 1280x800, RAW10, 2 lanes, 800 Mbps/lane, ~60 fps.
 */
#include "../../utility/vin_log.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include "camera.h"
#include "sensor_helper.h"

MODULE_AUTHOR("BoShuo (Peter) Lin");
MODULE_DESCRIPTION("OV9281 mono global shutter sensor driver for Allwinner VIN");
MODULE_LICENSE("GPL");

#define MCLK			(24 * 1000 * 1000)
#define V4L2_IDENT_SENSOR	0x9281

/* 7-bit I2C address 0x60 (SID low). VIN uses the 8-bit form from the DT. */
#define I2C_ADDR		0xc0

#define SENSOR_NUM		2
#define SENSOR_NAME		"ov9281_mipi"
#define SENSOR_NAME_2		"ov9281_mipi_2"

/* Chip ID */
#define OV9281_REG_ID_H		0x300a
#define OV9281_REG_ID_L		0x300b
#define OV9281_CHIP_ID		0x9281

/* Exposure (Q4 lines, 20 bits in 0x3500..0x3502) and analog gain (Q4) */
#define OV9281_REG_EXPO_H	0x3500
#define OV9281_REG_EXPO_M	0x3501
#define OV9281_REG_EXPO_L	0x3502
#define OV9281_REG_AGAIN	0x3509
#define OV9281_REG_HOLD		0x3308
#define OV9281_EXP_MARGIN	25	/* lines, from mainline */
#define OV9281_GAIN_MIN		0x10	/* 1.0x */
#define OV9281_GAIN_MAX		0xff	/* ~15.9x */

/* Timing for the single mode */
#define OV9281_WIDTH		1280
#define OV9281_HEIGHT		800
#define OV9281_HTS		1456	/* pixels; register holds HTS/2 = 0x02d8 */
#define OV9281_VTS		1822	/* lines: 800 + 1022 vblank (0x071e) */
#define OV9281_PIXEL_RATE	160000000	/* 400 MHz link * 2 * 2 lanes / 10 */
#define OV9281_MIPI_BPS		(800 * 1000 * 1000)
#define OV9281_FPS		60

static int ov9281_vts = OV9281_VTS;

/* Debug: leave the sensor powered after probe so the bus can be inspected */
static int keep_power;
module_param(keep_power, int, 0644);
MODULE_PARM_DESC(keep_power, "1 = skip PWR_OFF (debug: i2cdetect / measure rails)");

/* Start-up exposure/gain, same units as the V4L2 controls (Q4: 16 = 1 line / 1.0x) */
static int exposure = 642 << 4;
module_param(exposure, int, 0644);
MODULE_PARM_DESC(exposure, "start-up exposure, lines x16 (16..28752, default 10272 = ~5.8 ms)");
static int gain = 0x10;
module_param(gain, int, 0644);
MODULE_PARM_DESC(gain, "start-up analog gain x16 (16 = 1.0x .. 255)");

/*
 * sensor_write_array() batches consecutive register addresses and can read
 * one element past the end of an array while doing so. Every table ends
 * with this harmless sentinel (0 ms delay) so that never matters.
 */
#define REG_END			{REG_DLY, 0x00}

/*
 * Default registers: software reset, continuous MIPI clock, then the
 * mainline common_regs table.
 */
static struct regval_list sensor_default_regs[] = {
	{0x0103, 0x01},		/* software reset */
	{REG_DLY, 10},
	{0x0100, 0x00},		/* standby */
	{0x4800, 0x00},		/* MIPI_CTRL00: continuous clock */

	{0x0302, 0x32},
	{0x030e, 0x02},
	{0x3001, 0x00},
	{0x3004, 0x00},		/* OUTPUT_ENABLE4 */
	{0x3005, 0x00},		/* OUTPUT_ENABLE5 */
	{0x3006, 0x04},		/* OUTPUT_ENABLE6: ILPWM */
	{0x3011, 0x0a},
	{0x3013, 0x18},
	{0x301c, 0xf0},
	{0x3022, 0x01},
	{0x3030, 0x10},
	{0x3039, 0x32},
	{0x303a, 0x00},
	{0x3503, 0x08},		/* AEC manual, gain precision 1/16 */
	{0x3505, 0x8c},
	{0x3507, 0x03},
	{0x3508, 0x00},
	{0x3610, 0x80},
	{0x3611, 0xa0},
	{0x3620, 0x6e},
	{0x3632, 0x56},
	{0x3633, 0x78},
	{0x3666, 0x00},
	{0x366f, 0x5a},
	{0x3680, 0x84},
	{0x3712, 0x80},
	{0x372d, 0x22},
	{0x3731, 0x80},
	{0x3732, 0x30},
	{0x377d, 0x22},
	{0x3788, 0x02},
	{0x3789, 0xa4},
	{0x378a, 0x00},
	{0x378b, 0x4a},
	{0x3799, 0x20},
	{0x3881, 0x42},
	{0x38a8, 0x02},
	{0x38a9, 0x80},
	{0x38b1, 0x00},
	{0x38c4, 0x00},
	{0x38c5, 0xc0},
	{0x38c6, 0x04},
	{0x38c7, 0x80},
	{0x3920, 0xff},
	{0x4010, 0x40},
	{0x4043, 0x40},
	{0x4307, 0x30},
	{0x4317, 0x00},
	{0x4501, 0x00},
	{0x450a, 0x08},
	{0x4601, 0x04},
	{0x470f, 0x00},
	{0x4f07, 0x00},
	{0x5000, 0x9f},
	{0x5001, 0x00},
	{0x5e00, 0x00},
	{0x5d00, 0x07},
	{0x5d01, 0x00},
	{0x0101, 0x01},
	{0x1000, 0x03},
	{0x5a08, 0x84},
	REG_END,
};

/* RAW10 bit-depth settings */
static struct regval_list sensor_fmt_raw10[] = {
	{0x030d, 0x50},		/* PLL_CTRL_0D: RAW10 */
	{0x3662, 0x05},		/* ANA_CORE_2: RAW10 */
	REG_END,
};

/* 1280x800 mode (mainline mode_1280x800_regs) + HTS/VTS + exposure/gain */
static struct regval_list sensor_1280x800_regs[] = {
	{0x3778, 0x00},
	{0x3800, 0x00},
	{0x3801, 0x00},
	{0x3802, 0x00},
	{0x3803, 0x00},
	{0x3804, 0x05},
	{0x3805, 0x0f},
	{0x3806, 0x03},
	{0x3807, 0x2f},
	{0x3808, 0x05},		/* output width 1280 */
	{0x3809, 0x00},
	{0x380a, 0x03},		/* output height 800 */
	{0x380b, 0x20},
	{0x380c, 0x02},		/* HTS/2 = 728 */
	{0x380d, 0xd8},
	{0x380e, 0x07},		/* VTS = 1822 */
	{0x380f, 0x1e},
	{0x3810, 0x00},
	{0x3811, 0x08},
	{0x3812, 0x00},
	{0x3813, 0x08},
	{0x3814, 0x11},
	{0x3815, 0x11},
	{0x3820, 0x40},
	{0x3821, 0x00},
	{0x4003, 0x40},
	{0x4008, 0x04},
	{0x4009, 0x0b},
	{0x400c, 0x00},
	{0x400d, 0x07},
	{0x4507, 0x00},
	{0x4509, 0x00},
	{0x3500, 0x00},		/* exposure 642 lines (Q4: 0x2820) ~5.8 ms */
	{0x3501, 0x28},
	{0x3502, 0x20},
	{0x3509, 0x10},		/* analog gain 1.0x */
	{0x0100, 0x01},		/* streaming on */
	REG_END,
};

static struct regval_list sensor_fmt_raw[] = {
	REG_END,
};

static int sensor_write_exp(struct v4l2_subdev *sd, unsigned int exp_q4)
{
	sensor_write(sd, OV9281_REG_EXPO_H, (exp_q4 >> 16) & 0x0f);
	sensor_write(sd, OV9281_REG_EXPO_M, (exp_q4 >> 8) & 0xff);
	return sensor_write(sd, OV9281_REG_EXPO_L, exp_q4 & 0xf0);
}

static unsigned int clamp_exp(unsigned int exp_val)
{
	unsigned int max = (ov9281_vts - OV9281_EXP_MARGIN) << 4;

	if (exp_val < 16)
		exp_val = 16;
	if (exp_val > max)
		exp_val = max;
	return exp_val;
}

static int clamp_gain(int gain_val)
{
	if (gain_val < OV9281_GAIN_MIN)
		gain_val = OV9281_GAIN_MIN;
	if (gain_val > OV9281_GAIN_MAX)
		gain_val = OV9281_GAIN_MAX;
	return gain_val;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	struct sensor_info *info = to_state(sd);
	unsigned int exp_val = clamp_exp(exp_gain->exp_val);
	int gain_val = clamp_gain(exp_gain->gain_val);

	sensor_write(sd, OV9281_REG_HOLD, 0x01);
	sensor_write_exp(sd, exp_val);
	sensor_write(sd, OV9281_REG_AGAIN, gain_val);
	sensor_write(sd, OV9281_REG_HOLD, 0x00);

	info->exp = exp_val;
	info->gain = gain_val;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);

	exp_val = clamp_exp(exp_val);
	sensor_write_exp(sd, exp_val);
	info->exp = exp_val;
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->gain;
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);

	gain_val = clamp_gain(gain_val);
	sensor_write(sd, OV9281_REG_AGAIN, gain_val);
	info->gain = gain_val;
	return 0;
}

/*
 * Power sequence kept identical to the vendor IMX219 driver: on this board
 * the CSI connector's enable line is the sensor's PWDN GPIO (PE6), which
 * switches on the camera module's regulators (Raspberry Pi style modules).
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		cci_lock(sd);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		cci_unlock(sd);
		vin_set_mclk(sd, OFF);
		break;
	case STBY_OFF:
		cci_lock(sd);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		msleep(20);
		cci_unlock(sd);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		break;
	case PWR_ON:
		sensor_print("[ov9281] PWR_ON\n");
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		vin_set_mclk_freq(sd, MCLK);
		vin_set_mclk(sd, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_set_pmu_channel(sd, DVDD, ON);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(30000, 31000);	/* OV9281 needs >= 8192 MCLK */
		cci_unlock(sd);
		break;
	case PWR_OFF:
		if (keep_power) {
			sensor_print("[ov9281] keep_power=1: leaving sensor powered\n");
			break;
		}
		sensor_dbg("PWR_OFF!\n");
		cci_lock(sd);
		vin_set_mclk(sd, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		usleep_range(10000, 12000);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(10000, 12000);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(10000, 12000);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type hi = 0, lo = 0;
	int ret;

	ret = sensor_read(sd, OV9281_REG_ID_H, &hi);
	ret |= sensor_read(sd, OV9281_REG_ID_L, &lo);
	if (ret) {
		sensor_err("[ov9281] I2C read failed (no ACK at 0x60?)\n");
		return -ENODEV;
	}

	if (((hi << 8) | lo) != OV9281_CHIP_ID) {
		sensor_err("[ov9281] wrong chip id 0x%02x%02x (want 0x9281)\n",
			   hi, lo);
		return -ENODEV;
	}

	sensor_print("[ov9281] found OV9281, chip id 0x%02x%02x\n", hi, lo);
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct sensor_info *info = to_state(sd);
	int ret;

	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = OV9281_WIDTH;
	info->height = OV9281_HEIGHT;
	info->hflip = 0;
	info->vflip = 0;
	/* info is zeroed at probe: take the module defaults once, then keep
	 * whatever user space set through the controls across re-opens */
	if (!info->exp)
		info->exp = clamp_exp(exposure);
	if (!info->gain)
		info->gain = clamp_gain(gain);
	info->tpf.numerator = 1;
	info->tpf.denominator = OV9281_FPS;
	info->preview_first_flag = 1;
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sensor_info *info = to_state(sd);
	int ret = 0;

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins) {
			memcpy(arg, info->current_wins,
			       sizeof(*info->current_wins));
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct sensor_format_struct sensor_formats[] = {
	{
		/* Mono Y10 data, reported as BGGR10 (see header comment) */
		.desc = "Mono RAW10 (as BGGR10)",
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1,
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

static struct sensor_win_size sensor_win_sizes[] = {
	{
		.width = OV9281_WIDTH,
		.height = OV9281_HEIGHT,
		.hoffset = 0,
		.voffset = 0,
		.hts = OV9281_HTS,
		.vts = OV9281_VTS,
		.pclk = OV9281_PIXEL_RATE,
		.mipi_bps = OV9281_MIPI_BPS,
		.fps_fixed = 1,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = (OV9281_VTS - OV9281_EXP_MARGIN) << 4,
		.gain_min = OV9281_GAIN_MIN,
		.gain_max = OV9281_GAIN_MAX,
		.regs = sensor_1280x800_regs,
		.regs_size = ARRAY_SIZE(sensor_1280x800_regs),
		.set_size = NULL,
	},
};
#define N_WIN_SIZES ARRAY_SIZE(sensor_win_sizes)

static int sensor_reg_init(struct sensor_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_win_size *wsize = info->current_wins;
	int ret;

	ret = sensor_write_array(sd, sensor_default_regs,
				 ARRAY_SIZE(sensor_default_regs));
	if (ret < 0) {
		sensor_err("write sensor_default_regs error\n");
		return ret;
	}

	ret = sensor_write_array(sd, sensor_fmt_raw10,
				 ARRAY_SIZE(sensor_fmt_raw10));
	if (ret < 0) {
		sensor_err("write sensor_fmt_raw10 error\n");
		return ret;
	}

	if (wsize->regs) {
		ret = sensor_write_array(sd, wsize->regs, wsize->regs_size);
		if (ret < 0) {
			sensor_err("write mode regs error\n");
			return ret;
		}
	}

	info->width = wsize->width;
	info->height = wsize->height;
	ov9281_vts = wsize->vts;

	/* The mode table writes a default exposure/gain; put back the values
	 * set through V4L2 (or the module parameters) under group hold. */
	sensor_write(sd, OV9281_REG_HOLD, 0x01);
	sensor_write_exp(sd, clamp_exp(info->exp));
	sensor_write(sd, OV9281_REG_AGAIN, clamp_gain(info->gain));
	sensor_write(sd, OV9281_REG_HOLD, 0x00);

	sensor_print("[ov9281] streaming %ux%u RAW10 @ ~%d fps, exp %u (Q4) gain %d\n",
		     wsize->width, wsize->height, OV9281_FPS, info->exp, info->gain);
	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width, info->current_wins->height,
		     info->fmt->mbus_code);

	if (!enable) {
		sensor_write(sd, 0x0100, 0x00);	/* standby; ignore errors */
		return 0;
	}
	return sensor_reg_init(info);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
	cfg->bus.mipi_csi2.num_data_lanes =
		0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#else
	cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#endif
	return 0;
}

static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
		container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	}
	return -EINVAL;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

/* ----------------------------------------------------------------------- */

static struct cci_driver cci_drv[] = {
	{
		.name = SENSOR_NAME,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}, {
		.name = SENSOR_NAME_2,
		.addr_width = CCI_BITS_16,
		.data_width = CCI_BITS_8,
	}
};

static int sensor_init_controls(struct v4l2_subdev *sd,
				const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int ret = 0;

	v4l2_ctrl_handler_init(handler, 2);

	/* Exposure in Q4 lines, gain in Q4 (16 = 1.0x) */
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 16,
				 (OV9281_VTS - OV9281_EXP_MARGIN) << 4, 1,
				 642 << 4);
	/* VOLATILE alone makes the control framework skip s_ctrl (volatile
	 * controls never count as changed), so writes were silently dropped
	 * in v0.1/v0.2. EXECUTE_ON_WRITE makes every write reach the sensor. */
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE |
			       V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, OV9281_GAIN_MIN,
			  OV9281_GAIN_MAX, 1, OV9281_GAIN_MIN);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}

	sd->ctrl_handler = handler;
	return ret;
}

static int sensor_dev_id;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static int sensor_probe(struct i2c_client *client)
#else
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#endif
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
	int i;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	sd = &info->sd;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++)
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		if (i == SENSOR_NUM)
			i = 0;
		cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv[i]);
	} else {
		cci_dev_probe_helper(sd, client, &sensor_ops,
				     &cci_drv[sensor_dev_id++]);
	}

	sensor_init_controls(sd, &sensor_ctrl_ops);
	mutex_init(&info->lock);

#if IS_ENABLED(CONFIG_SAME_I2C)
	info->sensor_i2c_addr = I2C_ADDR >> 1;
#endif
	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->af_first_flag = 1;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sensor_remove(struct i2c_client *client)
#else
static void sensor_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd;
	int i;

	if (client) {
		for (i = 0; i < SENSOR_NUM; i++)
			if (!strcmp(cci_drv[i].name, client->name))
				break;
		if (i == SENSOR_NUM)
			i = 0;
		sd = cci_dev_remove_helper(client, &cci_drv[i]);
	} else {
		sd = cci_dev_remove_helper(client, &cci_drv[sensor_dev_id++]);
	}
	kfree(to_state(sd));
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};

static const struct i2c_device_id sensor_id_2[] = {
	{SENSOR_NAME_2, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver[] = {
	{
		.driver = {
			.owner = THIS_MODULE,
			.name = SENSOR_NAME,
		},
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id,
	}, {
		.driver = {
			.owner = THIS_MODULE,
			.name = SENSOR_NAME_2,
		},
		.probe = sensor_probe,
		.remove = sensor_remove,
		.id_table = sensor_id_2,
	},
};

static __init int init_sensor(void)
{
	int i, ret = 0;

	sensor_dev_id = 0;
	for (i = 0; i < SENSOR_NUM; i++)
		ret = cci_dev_init_helper(&sensor_driver[i]);
	return ret;
}

static __exit void exit_sensor(void)
{
	int i;

	sensor_dev_id = 0;
	for (i = 0; i < SENSOR_NUM; i++)
		cci_dev_exit_helper(&sensor_driver[i]);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
