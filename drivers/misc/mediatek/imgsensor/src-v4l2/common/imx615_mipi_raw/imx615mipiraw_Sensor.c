// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 IMX615mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "imx615mipiraw_Sensor.h"
#include "imx615_ana_gain_table.h"
#include "imx615_eeprom.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define imx615_table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u8(__VA_ARGS__)

#undef VENDOR_EDIT

/***************Modify Following Strings for Debug**********************/
#define DEBUG_LOG_EN 0
#define PFX "IMX615_camera_sensor"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(...) do { if ((DEBUG_LOG_EN)) LOG_INF(__VA_ARGS__); } while (0)

#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 765 /* trans# max is 255, each 3 bytes */
#else
#define I2C_BUFFER_LEN 3
#endif

#ifdef VENDOR_EDIT
#define MODULE_ID_OFFSET 0x0000
#endif

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = IMX615_SENSOR_ID,

	.checksum_value = 0x8ac2d94a,
	.pre = {
		.pclk = 285600000,
		.linelength = 3768,
		.framelength = 2518,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 2464,
		.mipi_data_lp2hs_settle_dc = 85,
		/* following for GetDefaultFramerateByScenario() */
		.mipi_pixel_rate = 267600000,
		.max_framerate = 300, /* 30fps */
	},
	.cap = {
		.pclk = 600000000,
		.linelength = 3768,
		.framelength = 5290,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 2464,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 574800000,
		.max_framerate = 300, /* 30fps */
	},
	.normal_video = {
		.pclk = 465600000,
		.linelength = 3768,
		.framelength = 4104,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 1856,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 785280000,
		.max_framerate = 300, /* 30fps */
	},
	.hs_video = {
		.pclk = 441600000,
		.linelength = 3768,
		.framelength = 1946,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 1856,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 418000000,
		.max_framerate = 600, /* 60fps */
	},
	.slim_video = {
		.pclk = 600000000,
		.linelength = 3768,
		.framelength = 2644,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 2464,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 600,
		.mipi_pixel_rate = 574800000,
	},
	.custom1 = {
		.pclk = 864000000,
		.linelength = 11480,
		.framelength = 5017,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 6528,
		.grabwindow_height = 4896,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 150,
		.mipi_pixel_rate = 515200000,
	},
	.custom2 = {
		.pclk = 441600000,
		.linelength = 3768,
		.framelength = 1946,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3280,
		.grabwindow_height = 1856,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 418000000,
		.max_framerate = 600, /* 60fps */
	},
	.min_gain = BASEGAIN, /*1x gain*/
	.max_gain = BASEGAIN * 16, /*16x gain*/
	.min_gain_iso = 100,
	.margin = 32,		/* sensor framelength & shutter margin */
	.min_shutter = 5,	/* min shutter */
	.gain_step = 1,
	.gain_type = 0,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.temperature_support = 1, /* 1, support; 0,not support */
	.sensor_mode_num = 7,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.hs_video_delay_frame = 2,
	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */
	.custom1_delay_frame = 2,	/* enter custom1 delay frame num */
	.custom2_delay_frame = 2,	/* enter custom2 delay frame num */
	.custom3_delay_frame = 2,
	.frame_time_delay_frame = 3,

	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	/* .mipi_sensor_type = MIPI_OPHY_NCSI2, */
	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2, /* 0,MIPI_OPHY_NCSI2; 1,MIPI_OPHY_CSI2 */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_HW_BAYER_R,
	.mclk = 24, /* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	/*.mipi_lane_num = SENSOR_MIPI_4_LANE,*/
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x34,0x20, 0xff},
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_speed = 1000, /* i2c read/write speed */
};



/* Sensor output window information */
#if 0
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* Preview */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* capture */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* video */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* hs_video */
	{6560, 4928, 000, 000, 6560, 4928, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* slim_video */
	{6560, 4928, 000, 000, 6560, 4928, 6560, 4928,0000, 0000, 6560, 4928, 0, 0, 6560, 4928}, /* remosaic */
};
#else
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[7] = {
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* Preview */
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* capture */
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* video */
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* hs_video */
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 0000, 3280, 2464, 0, 0, 3280, 2464}, /* slim_video */
	{6528, 4896, 000, 000, 6528, 4896, 6528, 4896,0000, 0000, 6528, 4896, 0, 0, 6528, 4896}, /* remosaic */
	{6528, 4896, 000, 000, 6528, 4896, 3280, 2464,0000, 304,  3280, 1856, 0, 0, 3280, 1856}, /* custom2 */
};
#endif

#if MULTI_WRITE
#define IMX615_EEPROM_READ_ID  0xA4
static void imx615_apply_LRC(struct subdrv_ctx *ctx)
{
	u8 imx615_LRC_data[352] = {0};

	LOG_DEBUG("E");

	read_imx615_LRC(ctx, imx615_LRC_data);

	/* L */
	subdrv_i2c_wr_p8(ctx, 0x7520, imx615_LRC_data, 70);

	/* R */
	subdrv_i2c_wr_p8(ctx, 0x7568, imx615_LRC_data + 70, 70);

	LOG_DEBUG("readback LRC, L1(%d) L70(%d) R1(%d) R70(%d)\n",
		read_cmos_sensor_8(ctx, 0x7520), read_cmos_sensor_8(ctx, 0x7565),
		read_cmos_sensor_8(ctx, 0x7568), read_cmos_sensor_8(ctx, 0x75AD));

}

static kal_uint16 read_cmos_eeprom_8(struct subdrv_ctx *ctx, kal_uint16 addr)
{
	kal_uint16 get_byte = 0;

	adaptor_i2c_rd_u8(ctx->i2c_client, IMX615_EEPROM_READ_ID >> 1, addr, (u8 *)&get_byte);
	return get_byte;
}
#define QSC_SIZE 1560
#define QSC_EEPROM_ADDR 0x8BF
static kal_uint16 imx615_QSC_setting[QSC_SIZE];
static void read_sensor_Cali(struct subdrv_ctx *ctx)
{
	kal_uint16 idx = 0;
	kal_uint16 addr_qsc = QSC_EEPROM_ADDR;

	for (idx = 0; idx < QSC_SIZE; idx++) {
		addr_qsc = QSC_EEPROM_ADDR + idx;
		imx615_QSC_setting[idx] = read_cmos_eeprom_8(ctx, addr_qsc);
	}
	ctx->is_read_preload_eeprom = 1;
}

static void write_sensor_QSC(struct subdrv_ctx *ctx)
{
    kal_uint16 i = 0;
    for (i = 0; i < QSC_SIZE; i++) {
        write_cmos_sensor_8(ctx, 0xC500+i, imx615_QSC_setting[i]);
    }
}

/*write AWB gain to sensor*/
static kal_uint32 imx615_awb_gain(struct subdrv_ctx *ctx, struct SET_SENSOR_AWB_GAIN *pSetSensorAWB)
{
	UINT32 rgain_32, grgain_32, gbgain_32, bgain_32;

	pr_debug("%s\n", __func__);

	grgain_32 = (pSetSensorAWB->ABS_GAIN_GR << 8) >> 9;
	rgain_32 = (pSetSensorAWB->ABS_GAIN_R << 8) >> 9;
	bgain_32 = (pSetSensorAWB->ABS_GAIN_B << 8) >> 9;
	gbgain_32 = (pSetSensorAWB->ABS_GAIN_GB << 8) >> 9;

	pr_debug(
		"[%s] ABS_GAIN_GR:%d, grgain_32:%d\n, ABS_GAIN_R:%d, rgain_32:%d\n, ABS_GAIN_B:%d, bgain_32:%d,ABS_GAIN_GB:%d, gbgain_32:%d\n",
		__func__,
		pSetSensorAWB->ABS_GAIN_GR, grgain_32,
		pSetSensorAWB->ABS_GAIN_R, rgain_32,
		pSetSensorAWB->ABS_GAIN_B, bgain_32,
		pSetSensorAWB->ABS_GAIN_GB, gbgain_32);

	write_cmos_sensor_8(ctx, 0x0b8e, (grgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b8f, grgain_32 & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b90, (rgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b91, rgain_32 & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b92, (bgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b93, bgain_32 & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b94, (gbgain_32 >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0b95, gbgain_32 & 0xFF);
	return ERROR_NONE;
}

static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);
	/* return;*/ /* for test */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);

	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0342, ctx->line_length >> 8);
	write_cmos_sensor_8(ctx, 0x0343, ctx->line_length & 0xFF);

	write_cmos_sensor_8(ctx, 0x0104, 0x00);

}	/*	set_dummy  */

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{
	kal_uint8 itemp;

	LOG_DEBUG("image_mirror = %d\n", image_mirror);
	itemp = read_cmos_sensor_8(ctx, 0x0101);
	itemp &= ~0x03;

	switch (image_mirror) {

	case IMAGE_NORMAL:
	write_cmos_sensor_8(ctx, 0x0101, itemp);
	break;

	case IMAGE_V_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x02);
	break;

	case IMAGE_H_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x01);
	break;

	case IMAGE_HV_MIRROR:
	write_cmos_sensor_8(ctx, 0x0101, itemp | 0x03);
	break;
	}
}

static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{
	/*kal_int16 dummy_line;*/
	kal_uint32 frame_length = ctx->frame_length;

	LOG_DEBUG("framerate = %d, min framelength should enable %d\n", framerate,
		min_framelength_en);

	frame_length = ctx->pclk / framerate * 10 / ctx->line_length;
	if (frame_length >= ctx->min_frame_length)
		ctx->frame_length = frame_length;
	else
		ctx->frame_length = ctx->min_frame_length;

	ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length) {
		ctx->frame_length = imgsensor_info.max_frame_length;
		ctx->dummy_line =
			ctx->frame_length - ctx->min_frame_length;
	}
	if (min_framelength_en)
		ctx->min_frame_length = ctx->frame_length;
	set_dummy(ctx);
}	/*	set_max_framerate  */

#define MAX_CIT_LSHIFT 7
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint16 l_shift = 1;

	//if (shutter > ctx->min_frame_length - imgsensor_info.margin)
	//	ctx->frame_length = shutter + imgsensor_info.margin;
	//else
	ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (shutter < imgsensor_info.min_shutter)
		shutter = imgsensor_info.min_shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10
				/ ctx->frame_length;
		LOG_DEBUG("autoflicker enable, realtime_fps = %d\n",
			realtime_fps);
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}

	/* long expsoure */
	if (shutter >
		(imgsensor_info.max_frame_length - imgsensor_info.margin)) {

		for (l_shift = 1; l_shift < MAX_CIT_LSHIFT; l_shift++) {
			if ((shutter >> l_shift)
		    < (imgsensor_info.max_frame_length - imgsensor_info.margin))

				break;
		}
		if (l_shift > MAX_CIT_LSHIFT) {
			LOG_DEBUG(
			    "Unable to set such a long exposure %d, set to max\n",
			    shutter);

			l_shift = MAX_CIT_LSHIFT;
		}
		shutter = shutter >> l_shift;
		//ctx->frame_length = shutter + imgsensor_info.margin;
		LOG_DEBUG("enter long exposure mode, time is %d", l_shift);
		write_cmos_sensor_8(ctx, 0x3100,
			read_cmos_sensor(ctx, 0x3100) | (l_shift & 0x7));
		/* Frame exposure mode customization for LE*/
		ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		ctx->current_ae_effective_frame = 2;
	} else {
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
		write_cmos_sensor_8(ctx, 0x3100,
			read_cmos_sensor(ctx, 0x3100) & 0xf8);
		write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor_8(ctx, 0x0104, 0x00);
		ctx->current_ae_effective_frame = 2;
		LOG_DEBUG("set frame_length\n");
	}

	/* Update Shutter */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
	if (!ctx->ae_ctrl_gph_en)
		write_cmos_sensor_8(ctx, 0x0104, 0x00);

	LOG_DEBUG("shutter =%d, framelength =%d\n",
		shutter, ctx->frame_length);
}	/*	write_shutter  */

/*************************************************************************
 * FUNCTION
 *	set_shutter
 *
 * DESCRIPTION
 *	This function set e-shutter of sensor to change exposure time.
 *
 * PARAMETERS
 *	iShutter : exposured lines
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static void set_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter)
{
	ctx->shutter = shutter;

	write_shutter(ctx, shutter);
} /* set_shutter */


static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);

	LOG_DEBUG("Framelength: set=%d/input=%d/min=%d, auto_extend=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length,
		read_cmos_sensor_8(ctx, 0x0350));
}

/*************************************************************************
 * FUNCTION
 *	set_shutter_frame_length
 *
 * DESCRIPTION
 *	for frame & 3A sync
 *
 *************************************************************************/
static void set_shutter_frame_length(struct subdrv_ctx *ctx, kal_uint16 shutter,
				     kal_uint16 frame_length,
				     kal_bool auto_extend_en)
{	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	ctx->shutter = shutter;

	/*0x3500, 0x3501, 0x3502 will increase VBLANK to
	 *get exposure larger than frame exposure
	 *AE doesn't update sensor gain at capture mode,
	 *thus extra exposure lines must be updated here.
	 */

	/* OV Recommend Solution */
	/*if shutter bigger than frame_length,
	 *should extend frame length first
	 */
	/* Change frame time */
	dummy_line = frame_length - ctx->frame_length;
	ctx->frame_length = ctx->frame_length + dummy_line;
	ctx->min_frame_length = ctx->frame_length;

	if (shutter > ctx->min_frame_length - imgsensor_info.margin)
		ctx->frame_length = shutter + imgsensor_info.margin;
	else
		ctx->frame_length = ctx->min_frame_length;
	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
			? imgsensor_info.min_shutter : shutter;
	shutter =
	(shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (ctx->autoflicker_en) {
		realtime_fps = ctx->pclk / ctx->line_length * 10 /
				ctx->frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
		else {
			/* Extend frame length */
			write_cmos_sensor_8(ctx, 0x0104, 0x01);
			write_cmos_sensor_8(ctx, 0x0340,
					ctx->frame_length >> 8);
			write_cmos_sensor_8(ctx, 0x0341,
					ctx->frame_length & 0xFF);
			write_cmos_sensor_8(ctx, 0x0104, 0x00);
		}
	} else {
		/* Extend frame length */
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
		write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
		write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
		write_cmos_sensor_8(ctx, 0x0104, 0x00);
	}

	/* Update Shutter */
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	if (auto_extend_en)
		write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);
	LOG_DEBUG(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, ctx->frame_length, frame_length,
		dummy_line, read_cmos_sensor_8(ctx, 0x0350));

}	/* set_shutter_frame_length */

static kal_uint32 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint32 reg_gain = 0x0;

	reg_gain = 1024 - (1024*BASEGAIN)/gain;
	return (kal_uint32) reg_gain;
}

/*************************************************************************
 * FUNCTION
 *	set_gain
 *
 * DESCRIPTION
 *	This function is to set global gain to sensor.
 *
 * PARAMETERS
 *	iGain : sensor global gain(base: 0x40)
 *
 * RETURNS
 *	the actually gain set to sensor.
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 set_gain(struct subdrv_ctx *ctx, kal_uint32 gain)
{
	kal_uint32 reg_gain;

	if (gain < imgsensor_info.min_gain || gain > imgsensor_info.max_gain) {
		LOG_DEBUG("Error gain setting");

		if (gain < imgsensor_info.min_gain)
			gain = imgsensor_info.min_gain;
		else
			gain = imgsensor_info.max_gain;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_DEBUG("gain = %d, reg_gain = 0x%x\n ", gain, reg_gain);
	if (!ctx->ae_ctrl_gph_en)
		write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0204, (reg_gain>>8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0205, reg_gain & 0xFF);
	write_cmos_sensor_8(ctx, 0x0104, 0x00);

	return gain;
} /* set_gain */

/*************************************************************************
 * FUNCTION
 *	night_mode
 *
 * DESCRIPTION
 *	This function night mode of sensor.
 *
 * PARAMETERS
 *	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	LOG_DEBUG("streaming_enable(0=Sw Standby,1=streaming): %d\n",
		enable);
	if (enable)
		write_cmos_sensor_8(ctx, 0x0100, 0x01);
	else
		write_cmos_sensor_8(ctx, 0x0100, 0x00);
	return ERROR_NONE;
}

static kal_uint16 imx615_init_setting[] = {
    0x0136, 0x18,
    0x0137, 0x00,
    0x3C7E, 0x02,
    0x3C7F, 0x01,
    0x0111, 0x02,
    0x380C, 0x00,
    0x3C00, 0x01,
    0x3C01, 0x00,
    0x3C02, 0x00,
    0x3C03, 0x03,
    0x3C04, 0xFF,
    0x3C05, 0x01,
    0x3C06, 0x00,
    0x3C07, 0x00,
    0x3C08, 0x03,
    0x3C09, 0xFF,
    0x3C0A, 0x00,
    0x3C0B, 0x00,
    0x3C0C, 0x10,
    0x3C0D, 0x10,
    0x3C0E, 0x10,
    0x3C0F, 0x10,
    0x3C10, 0x10,
    0x3C11, 0x20,
    0x3C15, 0x00,
    0x3C16, 0x00,
    0x3C17, 0x01,
    0x3C18, 0x00,
    0x3C19, 0x01,
    0x3C1A, 0x00,
    0x3C1B, 0x01,
    0x3C1C, 0x00,
    0x3C1D, 0x01,
    0x3C1E, 0x00,
    0x3C1F, 0x00,
    0x3F89, 0x01,
    0x3F8F, 0x01,
    0x53B9, 0x01,
    0x62C4, 0x04,
    0x658F, 0x07,
    0x6590, 0x05,
    0x6591, 0x07,
    0x6592, 0x05,
    0x6593, 0x07,
    0x6594, 0x05,
    0x6595, 0x07,
    0x6596, 0x05,
    0x6597, 0x05,
    0x6598, 0x05,
    0x6599, 0x05,
    0x659A, 0x05,
    0x659B, 0x05,
    0x659C, 0x05,
    0x659D, 0x05,
    0x659E, 0x07,
    0x659F, 0x05,
    0x65A0, 0x07,
    0x65A1, 0x05,
    0x65A2, 0x07,
    0x65A3, 0x05,
    0x65A4, 0x07,
    0x65A5, 0x05,
    0x65A6, 0x05,
    0x65A7, 0x05,
    0x65A8, 0x05,
    0x65A9, 0x05,
    0x65AA, 0x05,
    0x65AB, 0x05,
    0x65AC, 0x05,
    0x65AD, 0x07,
    0x65AE, 0x07,
    0x65AF, 0x07,
    0x65B0, 0x05,
    0x65B1, 0x05,
    0x65B2, 0x05,
    0x65B3, 0x05,
    0x65B4, 0x07,
    0x65B5, 0x07,
    0x65B6, 0x07,
    0x65B7, 0x07,
    0x65B8, 0x05,
    0x65B9, 0x05,
    0x65BA, 0x05,
    0x65BB, 0x05,
    0x65BC, 0x05,
    0x65BD, 0x05,
    0x65BE, 0x05,
    0x65BF, 0x05,
    0x65C0, 0x05,
    0x65C1, 0x05,
    0x65C2, 0x05,
    0x65C3, 0x05,
    0x65C4, 0x05,
    0x65C5, 0x05,
    0x6E1C, 0x00,
    0x6E1D, 0x00,
    0x6E25, 0x00,
    0x6E38, 0x03,
    0x895C, 0x01,
    0x895D, 0x00,
    0x8966, 0x00,
    0x8967, 0x4E,
    0x896A, 0x00,
    0x896B, 0x24,
    0x896F, 0x34,
    0x8976, 0x00,
    0x8977, 0x00,
    0x9004, 0x1F,
    0x9200, 0xB7,
    0x9201, 0x34,
    0x9202, 0xB7,
    0x9203, 0x36,
    0x9204, 0xB7,
    0x9205, 0x37,
    0x9206, 0xB7,
    0x9207, 0x38,
    0x9208, 0xB7,
    0x9209, 0x39,
    0x920A, 0xB7,
    0x920B, 0x3A,
    0x920C, 0xB7,
    0x920D, 0x3C,
    0x920E, 0xB7,
    0x920F, 0x3D,
    0x9210, 0xB7,
    0x9211, 0x3E,
    0x9212, 0xB7,
    0x9213, 0x3F,
    0x9214, 0xF6,
    0x9215, 0x13,
    0x9216, 0xF6,
    0x9217, 0x34,
    0x9218, 0xF4,
    0x9219, 0xA7,
    0x921A, 0xF4,
    0x921B, 0xAA,
    0x921C, 0xF4,
    0x921D, 0xAD,
    0x921E, 0xF4,
    0x921F, 0xB0,
    0x9220, 0xF4,
    0x9221, 0xB3,
    0x9222, 0x85,
    0x9223, 0x77,
    0x9224, 0xC4,
    0x9225, 0x4B,
    0x9226, 0xC4,
    0x9227, 0x4C,
    0x9228, 0xC4,
    0x9229, 0x4D,
    0x922A, 0xF5,
    0x922B, 0x5E,
    0x922C, 0xF5,
    0x922D, 0x5F,
    0x922E, 0xF5,
    0x922F, 0x64,
    0x9230, 0xF5,
    0x9231, 0x65,
    0x9232, 0xF5,
    0x9233, 0x6A,
    0x9234, 0xF5,
    0x9235, 0x6B,
    0x9236, 0xF5,
    0x9237, 0x70,
    0x9238, 0xF5,
    0x9239, 0x71,
    0x923A, 0xF5,
    0x923B, 0x76,
    0x923C, 0xF5,
    0x923D, 0x77,
    0x9810, 0x14,
    0x9814, 0x14,
    0xC020, 0x00,
    0xC026, 0x00,
    0xC027, 0x00,
    0xC448, 0x01,
    0xC44F, 0x01,
    0xC450, 0x00,
    0xC451, 0x00,
    0xC452, 0x01,
    0xC455, 0x00,
    0xE186, 0x36,
    0xE206, 0x35,
    0xE226, 0x33,
    0xE266, 0x34,
    0xE2A6, 0x31,
    0xE2C6, 0x37,
    0xE2E6, 0x32,
};

static kal_uint16 imx615_capture_30_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x14,
    0x0341,	0xAA,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x00,
    0x0347,	0x00,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x13,
    0x034B,	0x3F,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x09,
    0x040F,	0xA0,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x09,
    0x034F,	0xA0,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xFA,
    0x030B,	0x01,
    0x030D,	0x18,
    0x030E,	0x05,
    0x030F,	0x9D,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x14,
    0x0203,	0x7A,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
    /*global timimg
    0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0xA7,
    0x080C, 0x00,
    0x080D, 0x5F,
    0x080E, 0x00,
    0x080F, 0xAF,
    0x0810, 0x00,
    0x0811, 0x5F,
    0x0812, 0x00,
    0x0813, 0x0E,
    0x0814, 0x00,
    0x0815, 0x5F,
    0x0816, 0x01,
    0x0817, 0x97,
    0x0818, 0x00,
    0x0819, 0x4F,
    0x0824, 0x00,
    0x0825, 0x9F,
    0x0826, 0x00,
    0x0827, 0x0F,*/
};

static kal_uint16 imx615_preview_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x09,
    0x0341,	0xD6,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x00,
    0x0347,	0x00,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x13,
    0x034B,	0x3F,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x09,
    0x040F,	0xA0,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x09,
    0x034F,	0xA0,
    0x0301,	0x05,
    0x0303,	0x04,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xEE,
    0x030B,	0x02,
    0x030D,	0x04,
    0x030E,	0x00,
    0x030F,	0xDF,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x09,
    0x0203,	0xA6,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
    /*global ,timimg*/
    /*0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0x6F,
    0x080C, 0x00,
    0x080D, 0x2F,
    0x080E, 0x00,
    0x080F, 0x57,
    0x0810, 0x00,
    0x0811, 0x2F,
    0x0812, 0x00,
    0x0813, 0x2F,
    0x0814, 0x00,
    0x0815, 0x2F,
    0x0816, 0x00,
    0x0817, 0xBF,
    0x0818, 0x00,
    0x0819, 0x27,
    0x0824, 0x00,
    0x0825, 0x4F,
    0x0826, 0x00,
    0x0827, 0x0F,*/
};

static kal_uint16 imx615_normal_video_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x10,
    0x0341,	0x08,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x02,
    0x0347,	0x60,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x10,
    0x034B,	0xDF,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x07,
    0x040F,	0x40,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x07,
    0x034F,	0x40,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xC2,
    0x030B,	0x01,
    0x030D,	0x05,
    0x030E,	0x01,
    0x030F,	0x99,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x0F,
    0x0203,	0xD8,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
    /*global timimg
    0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0xD7,
    0x080C, 0x00,
    0x080D, 0x8F,
    0x080E, 0x00,
    0x080F, 0xF7,
    0x0810, 0x00,
    0x0811, 0x87,
    0x0812, 0x00,
    0x0813, 0x87,
    0x0814, 0x00,
    0x0815, 0x87,
    0x0816, 0x02,
    0x0817, 0x47,
    0x0818, 0x00,
    0x0819, 0x6F,
    0x0824, 0x00,
    0x0825, 0xDF,
    0x0826, 0x00,
    0x0827, 0x0F,*/
};

static kal_uint16 imx615_hs_video_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x07,
    0x0341,	0x9A,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x02,
    0x0347,	0x60,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x10,
    0x034B,	0xDF,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x07,
    0x040F,	0x40,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x07,
    0x034F,	0x40,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xB8,
    0x030B,	0x02,
    0x030D,	0x18,
    0x030E,	0x08,
    0x030F,	0x2A,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x07,
    0x0203,	0x6A,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
    /*global timimg
    0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0xD7,
    0x080C, 0x00,
    0x080D, 0x8F,
    0x080E, 0x00,
    0x080F, 0xF7,
    0x0810, 0x00,
    0x0811, 0x87,
    0x0812, 0x00,
    0x0813, 0x87,
    0x0814, 0x00,
    0x0815, 0x87,
    0x0816, 0x02,
    0x0817, 0x47,
    0x0818, 0x00,
    0x0819, 0x6F,
    0x0824, 0x00,
    0x0825, 0xDF,
    0x0826, 0x00,
    0x0827, 0x0F,*/
};

static kal_uint16 imx615_slim_video_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x0A,
    0x0341,	0x54,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x00,
    0x0347,	0x00,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x13,
    0x034B,	0x3F,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x09,
    0x040F,	0xA0,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x09,
    0x034F,	0xA0,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xFA,
    0x030B,	0x01,
    0x030D,	0x18,
    0x030E,	0x05,
    0x030F,	0x9D,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x0A,
    0x0203,	0x24,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
    /*global timimg
    0x0808, 0x02,
    0x080A, 0x00,
    0x080B, 0xD7,
    0x080C, 0x00,
    0x080D, 0x8F,
    0x080E, 0x00,
    0x080F, 0xF7,
    0x0810, 0x00,
    0x0811, 0x87,
    0x0812, 0x00,
    0x0813, 0x87,
    0x0814, 0x00,
    0x0815, 0x87,
    0x0816, 0x02,
    0x0817, 0x47,
    0x0818, 0x00,
    0x0819, 0x6F,
    0x0824, 0x00,
    0x0825, 0xDF,
    0x0826, 0x00,
    0x0827, 0x0F,*/
};

static kal_uint16 imx615_custom1_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x2C,
    0x0343,	0xD8,
    0x0340,	0x13,
    0x0341,	0x99,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x00,
    0x0347,	0x10,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x13,
    0x034B,	0x2F,
    0x0900,	0x00,
    0x0901,	0x11,
    0x0902,	0x0A,
    0x3246,	0x01,
    0x3247,	0x01,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x10,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x19,
    0x040D,	0x80,
    0x040E,	0x13,
    0x040F,	0x20,
    0x034C,	0x19,
    0x034D,	0x80,
    0x034E,	0x13,
    0x034F,	0x20,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x01,
    0x0307,	0x68,
    0x030B,	0x01,
    0x030D,	0x18,
    0x030E,	0x05,
    0x030F,	0x08,
    0x0310,	0x01,
    0x3620,	0x01,
    0x3621,	0x01,
    0x3C12,	0x62,
    0x3C13,	0x32,
    0x3C14,	0x20,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0x46,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x6E,
    0x3FFE,	0x00,
    0x3FFF,	0x64,
    0x0202,	0x13,
    0x0203,	0x69,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
};

static kal_uint16 imx615_custom2_setting[] = {
    0x0112,	0x0A,
    0x0113,	0x0A,
    0x0114,	0x03,
    0x0342,	0x0E,
    0x0343,	0xB8,
    0x0340,	0x07,
    0x0341,	0x9A,
    0x0344,	0x00,
    0x0345,	0x00,
    0x0346,	0x02,
    0x0347,	0x60,
    0x0348,	0x19,
    0x0349,	0x9F,
    0x034A,	0x10,
    0x034B,	0xDF,
    0x0900,	0x01,
    0x0901,	0x22,
    0x0902,	0x09,
    0x3246,	0x81,
    0x3247,	0x81,
    0x0401,	0x00,
    0x0404,	0x00,
    0x0405,	0x10,
    0x0408,	0x00,
    0x0409,	0x00,
    0x040A,	0x00,
    0x040B,	0x00,
    0x040C,	0x0C,
    0x040D,	0xD0,
    0x040E,	0x07,
    0x040F,	0x40,
    0x034C,	0x0C,
    0x034D,	0xD0,
    0x034E,	0x07,
    0x034F,	0x40,
    0x0301,	0x05,
    0x0303,	0x02,
    0x0305,	0x04,
    0x0306,	0x00,
    0x0307,	0xB8,
    0x030B,	0x02,
    0x030D,	0x18,
    0x030E,	0x08,
    0x030F,	0x2A,
    0x0310,	0x01,
    0x3620,	0x00,
    0x3621,	0x00,
    0x3C12,	0x56,
    0x3C13,	0x52,
    0x3C14,	0x3E,
    0x3F0C,	0x00,
    0x3F14,	0x01,
    0x3F80,	0x00,
    0x3F81,	0xA0,
    0x3F8C,	0x00,
    0x3F8D,	0x00,
    0x3FFC,	0x00,
    0x3FFD,	0x1E,
    0x3FFE,	0x00,
    0x3FFF,	0xDC,
    0x0202,	0x07,
    0x0203,	0x6A,
    0x0204,	0x00,
    0x0205,	0x70,
    0x020E,	0x01,
    0x020F,	0x00,
    0x0210,	0x01,
    0x0211,	0x00,
    0x0212,	0x01,
    0x0213,	0x00,
    0x0214,	0x01,
    0x0215,	0x00,
};

static kal_uint16 imx615_custom3_setting[] = {
	0x0111, 0x03,
	0x0112, 0x0A,
	0x0113, 0x0A,
	0x0114, 0x02,
	0x0342, 0x32,
	0x0343, 0x00,
	0x0340, 0x07,
	0x0341, 0x1E,
	0x0344, 0x00,
	0x0345, 0x00,
	0x0346, 0x00,
	0x0347, 0x00,
	0x0348, 0x12,
	0x0349, 0x2F,
	0x034A, 0x0D,
	0x034B, 0xA7,
	0x0220, 0x00,
	0x0221, 0x11,
	0x0222, 0x01,
	0x0900, 0x01,
	0x0901, 0x22,
	0x0902, 0x0A,
	0x3F4C, 0x01,
	0x3F4D, 0x03,
	0x4254, 0x7F,
	0x0401, 0x00,
	0x0404, 0x00,
	0x0405, 0x10,
	0x0408, 0x00,
	0x0409, 0x00,
	0x040A, 0x00,
	0x040B, 0x00,
	0x040C, 0x09,
	0x040D, 0x18,
	0x040E, 0x06,
	0x040F, 0xD4,
	0x034C, 0x09,
	0x034D, 0x18,
	0x034E, 0x06,
	0x034F, 0xD4,
	0x0301, 0x06,
	0x0303, 0x02,
	0x0305, 0x04,
	0x0306, 0x01,
	0x0307, 0x5E,
	0x0309, 0x0A,
	0x030B, 0x04,
	0x030D, 0x04,
	0x030E, 0x01,
	0x030F, 0x20,
	0x0310, 0x01,
	0x0820, 0x05,
	0x0821, 0x10,
	0x0822, 0x00,
	0x0823, 0x00,
	0x0106, 0x00,
	0x0B00, 0x00,
	0x3230, 0x00,
	0x3F14, 0x01,
	0x3F3C, 0x01,
	0x3F0D, 0x0A,
	0x3C06, 0x00,
	0x3C07, 0x00,
	0x3C0A, 0x01,
	0x3C0B, 0xC8,
	0x3F78, 0x02,
	0x3F79, 0x18,
	0x3F7C, 0x00,
	0x3F7D, 0x00,
	0x0202, 0x03,
	0x0203, 0xE8,
	0x0224, 0x01,
	0x0225, 0xF4,
	0x0204, 0x01,
	0x0205, 0x2B,
	0x0216, 0x00,
	0x0217, 0x00,
	0x020E, 0x01,
	0x020F, 0x00,
	0x0218, 0x01,
	0x0219, 0x00,
	0x3E20, 0x01,
	0x3E37, 0x00,
};

#endif

static void sensor_init(struct subdrv_ctx *ctx)
{
	int readNumer = 0;
	kal_uint8 itemp3,itemp4;
	LOG_DEBUG("E\n");
	write_cmos_sensor_8(ctx, 0x0A02,0x7F);
	write_cmos_sensor_8(ctx, 0x0A00,0x01);
	itemp3 = read_cmos_sensor_8(ctx, 0x0A01);
	while(itemp3 != 0x01)
	{
		readNumer++;
		itemp3 = read_cmos_sensor_8(ctx, 0x0A01);
		if(readNumer > 10)
		{
			LOG_INF("read 0x0A01 FAIL");
			break;
		}
	}
	itemp4 = read_cmos_sensor_8(ctx, 0x0A1F);
	if(itemp4 != 0xB4)
	{
		write_cmos_sensor_8(ctx, 0x0A00,0x00);
		imx615_table_write_cmos_sensor(ctx, imx615_init_setting,
		sizeof(imx615_init_setting)/sizeof(kal_uint16));
	}else{
		kal_uint8 itemp5;
		itemp5 = read_cmos_sensor_8(ctx, 0x0A20);
		if(itemp5 == 0x01)
		{
			write_cmos_sensor_8(ctx, 0x0A00,0x00);
			imx615_table_write_cmos_sensor(ctx, imx615_init_setting,
			sizeof(imx615_init_setting)/sizeof(kal_uint16));
			write_cmos_sensor_8(ctx, 0x574B, 0x01);
			write_cmos_sensor_8(ctx, 0x5765, 0x33);
		}else{
			write_cmos_sensor_8(ctx, 0x0A00,0x00);
			imx615_table_write_cmos_sensor(ctx, imx615_init_setting,
			sizeof(imx615_init_setting)/sizeof(kal_uint16));
		}
	}
	set_mirror_flip(ctx, ctx->mirror);

	LOG_DEBUG("X");
}	/*	  sensor_init  */

static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("E\n");

	imx615_table_write_cmos_sensor(ctx, imx615_preview_setting,
		sizeof(imx615_preview_setting)/sizeof(kal_uint16));

	LOG_DEBUG("X");
} /* preview_setting */

/*full size 30fps*/
static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s(PD 012515) 30 fps E! currefps:%d\n", __func__, currefps);
	/*************MIPI output setting************/

	imx615_table_write_cmos_sensor(ctx, imx615_capture_30_setting,
		sizeof(imx615_capture_30_setting)/sizeof(kal_uint16));
	LOG_DEBUG("%s(PD 012515) 30 fpsX\n", __func__);
}

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s E! currefps:%d\n", __func__, currefps);
	imx615_table_write_cmos_sensor(ctx, imx615_normal_video_setting,
	sizeof(imx615_normal_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E! currefps 120\n", __func__);
	/*************MIPI output setting************/
	imx615_table_write_cmos_sensor(ctx, imx615_hs_video_setting,
	sizeof(imx615_hs_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E\n", __func__);
	imx615_table_write_cmos_sensor(ctx, imx615_slim_video_setting,
	sizeof(imx615_slim_video_setting)/sizeof(kal_uint16));
	LOG_DEBUG("X\n");
}

/*full size 16M@24fps*/
static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 240 fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx615_table_write_cmos_sensor(ctx, imx615_custom1_setting,
		sizeof(imx615_custom1_setting)/sizeof(kal_uint16));

	LOG_DEBUG("X");
}

/*full size 8M@24fps*/
static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 480 fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx615_table_write_cmos_sensor(ctx, imx615_custom2_setting,
		sizeof(imx615_custom2_setting)/sizeof(kal_uint16));

	LOG_DEBUG("X");
}

/*full size 16M@24fps*/
static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s 4M*60 fps E! currefps\n", __func__);
	/*************MIPI output setting************/

	imx615_table_write_cmos_sensor(ctx, imx615_custom3_setting,
		sizeof(imx615_custom3_setting)/sizeof(kal_uint16));

	LOG_DEBUG("X");
}

/*************************************************************************
 * FUNCTION
 *	get_imgsensor_id
 *
 * DESCRIPTION
 *	This function get the sensor ID
 *
 * PARAMETERS
 *	*sensorID : return the sensor ID
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int get_imgsensor_id(struct subdrv_ctx *ctx, UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017));
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("imx615 i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				return ERROR_NONE;
			}

			printk("imx615 Read sensor id fail, i2c addr: 0x%x,sensor id: 0x%x\n",
				ctx->i2c_write_id,*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		/*if Sensor ID is not correct,
		 *Must set *sensor_id to 0xFFFFFFFF
		 */
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
 * FUNCTION
 *	open
 *
 * DESCRIPTION
 *	This function initialize the registers of CMOS sensor
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int open(struct subdrv_ctx *ctx)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint16 sensor_id = 0;

	/*sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = ((read_cmos_sensor_8(ctx, 0x0016) << 8)
					| read_cmos_sensor_8(ctx, 0x0017));
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}
			LOG_DEBUG("Read sensor id fail, i2c addr: 0x%x,sensor id: 0x%x\n",
				ctx->i2c_write_id,sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	/* initail sequence write in  */
	sensor_init(ctx);


	ctx->autoflicker_en = KAL_FALSE;
	ctx->sensor_mode = IMGSENSOR_MODE_INIT;
	ctx->shutter = 0x3D0;
	ctx->gain = 0x100;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->dummy_pixel = 0;
	ctx->dummy_line = 0;
	ctx->ihdr_mode = 0;
	ctx->test_pattern = KAL_FALSE;
	ctx->current_fps = imgsensor_info.pre.max_framerate;

	return ERROR_NONE;
} /* open */

/*************************************************************************
 * FUNCTION
 *	close
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *	None
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static int close(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("E\n");

	/*No Need to implement this function*/

	write_cmos_sensor_8(ctx, 0x0100, 0x00);

	return ERROR_NONE;
} /* close */


/*************************************************************************
 * FUNCTION
 * preview
 *
 * DESCRIPTION
 *	This function start the sensor preview.
 *
 * PARAMETERS
 *	*image_window : address pointer of pixel numbers in one period of HSYNC
 *  *sensor_config_data : address pointer of line numbers in one period of VSYNC
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 preview(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_PREVIEW;
	ctx->pclk = imgsensor_info.pre.pclk;
	ctx->line_length = imgsensor_info.pre.linelength;
	ctx->frame_length = imgsensor_info.pre.framelength;
	ctx->min_frame_length = imgsensor_info.pre.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	preview_setting(ctx);

	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);

	return ERROR_NONE;
} /* preview */

/*************************************************************************
 * FUNCTION
 *	capture
 *
 * DESCRIPTION
 *	This function setup the CMOS sensor in capture MY_OUTPUT mode
 *
 * PARAMETERS
 *
 * RETURNS
 *	None
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
static kal_uint32 capture(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;

	if (ctx->current_fps != imgsensor_info.cap.max_framerate)
		LOG_DEBUG(
			"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
			ctx->current_fps,
			imgsensor_info.cap.max_framerate / 10);
	ctx->pclk = imgsensor_info.cap.pclk;
	ctx->line_length = imgsensor_info.cap.linelength;
	ctx->frame_length = imgsensor_info.cap.framelength;
	ctx->min_frame_length = imgsensor_info.cap.framelength;
	ctx->autoflicker_en = KAL_FALSE;


	capture_setting(ctx, ctx->current_fps);

	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);

	/* set_mirror_flip(ctx, ctx->mirror); */

	return ERROR_NONE;
}	/* capture(ctx) */
static kal_uint32 normal_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	normal_video_setting(ctx, ctx->current_fps);
	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);

	/*set_mirror_flip(ctx, ctx->mirror);*/

	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/*ctx->video_mode = KAL_TRUE;*/
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/*ctx->current_fps = 300;*/
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);
	/*set_mirror_flip(ctx, ctx->mirror);*/

	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("E\n");

	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	/*ctx->video_mode = KAL_TRUE;*/
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/*ctx->current_fps = 300;*/
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);
	/*set_mirror_flip(ctx, ctx->mirror);*/

	return ERROR_NONE;
}	/* slim_video */


static kal_uint32 custom1(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);

	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);
	write_sensor_QSC(ctx);

	return ERROR_NONE;
}	/* custom1 */

static kal_uint32 custom2(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);

	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);

	return ERROR_NONE;
}	/* custom2 */

static kal_uint32 custom3(struct subdrv_ctx *ctx,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s.\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->autoflicker_en = KAL_FALSE;
	custom3_setting(ctx);

	if (ctx->pdaf_mode)
		imx615_apply_LRC(ctx);

	return ERROR_NONE;
}	/* custom3 */

static int get_resolution(struct subdrv_ctx *ctx,
		MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	int i = 0;

	for (i = SENSOR_SCENARIO_ID_MIN; i < SENSOR_SCENARIO_ID_MAX; i++) {
		if (i < imgsensor_info.sensor_mode_num) {
			sensor_resolution->SensorWidth[i] = imgsensor_winsize_info[i].w2_tg_size;
			sensor_resolution->SensorHeight[i] = imgsensor_winsize_info[i].h2_tg_size;
		} else {
			sensor_resolution->SensorWidth[i] = 0;
			sensor_resolution->SensorHeight[i] = 0;
		}
	}



	return ERROR_NONE;
} /* get_resolution */

static int get_info(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_INFO_STRUCT *sensor_info,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;

	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_PREVIEW] =
		imgsensor_info.pre_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_CAPTURE] =
		imgsensor_info.cap_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_NORMAL_VIDEO] =
		imgsensor_info.video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO] =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_SLIM_VIDEO] =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM1] =
		imgsensor_info.custom1_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM2] =
		imgsensor_info.custom2_delay_frame;
	sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM3] =
		imgsensor_info.custom3_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->TEMPERATURE_SUPPORT = imgsensor_info.temperature_support;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->PDAF_Support = 2;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */



	sensor_info->SensorWidthSampling = 0; /* 0 is default 1x */
	sensor_info->SensorHightSampling = 0; /* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;
	return ERROR_NONE;
}	/*	get_info  */


static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("scenario_id = %d\n", scenario_id);
	ctx->current_scenario_id = scenario_id;
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		preview(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		capture(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		normal_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		hs_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		slim_video(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		custom1(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		custom2(ctx, image_window, sensor_config_data);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		custom3(ctx, image_window, sensor_config_data);
		break;
	default:
		LOG_DEBUG("Error ScenarioId setting");
		preview(ctx, image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control(ctx) */



static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	LOG_DEBUG("framerate = %d\n ", framerate);
	/* SetVideoMode Function should fix framerate */
	if (framerate == 0)
		/* Dynamic frame rate */
		return ERROR_NONE;
	if ((framerate == 300) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 296;
	else if ((framerate == 150) && (ctx->autoflicker_en == KAL_TRUE))
		ctx->current_fps = 146;
	else
		ctx->current_fps = framerate;
	set_max_framerate(ctx, ctx->current_fps, 1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(struct subdrv_ctx *ctx,
		kal_bool enable, UINT16 framerate)
{
	LOG_DEBUG("enable = %d, framerate = %d\n", enable, framerate);
	if (enable) /*enable auto flicker*/
		ctx->autoflicker_en = KAL_TRUE;
	else /*Cancel Auto flick*/
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_DEBUG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk / framerate * 10
				/ imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
		? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk /
				framerate * 10 /
				imgsensor_info.normal_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.normal_video.framelength)
		? (frame_length - imgsensor_info.normal_video.framelength)
		: 0;
		ctx->frame_length =
			imgsensor_info.normal_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG(
				"Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n"
				, framerate
				, imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10
					/ imgsensor_info.cap.linelength;

		if (frame_length > imgsensor_info.max_frame_length) {
			LOG_DEBUG(
				"Warning: frame_length %d > max_frame_length %d!\n"
				, frame_length
				, imgsensor_info.max_frame_length);
			break;
		}

		ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk / framerate * 10
				/ imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
			  ? (frame_length - imgsensor_info.hs_video.framelength)
			  : 0;
		ctx->frame_length =
			imgsensor_info.hs_video.framelength
				+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk / framerate * 10
			/ imgsensor_info.slim_video.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			? (frame_length - imgsensor_info.slim_video.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.slim_video.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk / framerate * 10
				/ imgsensor_info.custom1.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom1.framelength)
			? (frame_length - imgsensor_info.custom1.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom1.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk / framerate * 10
				/ imgsensor_info.custom2.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom2.framelength)
			? (frame_length - imgsensor_info.custom2.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom2.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk / framerate * 10
				/ imgsensor_info.custom3.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.custom3.framelength)
			? (frame_length - imgsensor_info.custom3.framelength)
			: 0;
		ctx->frame_length =
			imgsensor_info.custom3.framelength
			+ ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		break;

	default:  /*coding with  preview scenario by default*/
		frame_length = imgsensor_info.pre.pclk / framerate * 10
			/ imgsensor_info.pre.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		LOG_DEBUG("error scenario_id = %d, we use preview scenario\n",
			scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
		enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM1:
		*framerate = imgsensor_info.custom1.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		*framerate = imgsensor_info.custom3.max_framerate;
		break;
	default:
		break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(struct subdrv_ctx *ctx, kal_uint32 mode)
{
	if (mode != ctx->test_pattern)
		pr_debug("test_pattern mode: %d\n", mode);
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	if (mode)
		write_cmos_sensor_8(ctx, 0x0601, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		write_cmos_sensor_8(ctx, 0x0601, 0x00); /*No pattern*/
	write_cmos_sensor_8(ctx, 0x0104, 0x00);

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_data(struct subdrv_ctx *ctx, struct mtk_test_pattern_data *data)
{

	pr_debug("test_patterndata R = %x, Gr = %x,Gb = %x,B = %x\n",
		data->Channel_R >> 22, data->Channel_Gr >> 22,
		data->Channel_Gb >> 22, data->Channel_B >> 22);
	write_cmos_sensor_8(ctx, 0x0104, 0x01);
	write_cmos_sensor_8(ctx, 0x0602, (data->Channel_R >> 30) & 0x3);
	write_cmos_sensor_8(ctx, 0x0603, (data->Channel_R >> 22) & 0xff);
	write_cmos_sensor_8(ctx, 0x0604, (data->Channel_Gr >> 30) & 0x3);
	write_cmos_sensor_8(ctx, 0x0605, (data->Channel_Gr >> 22) & 0xff);
	write_cmos_sensor_8(ctx, 0x0606, (data->Channel_B >> 30) & 0x3);
	write_cmos_sensor_8(ctx, 0x0607, (data->Channel_B >> 22) & 0xff);
	write_cmos_sensor_8(ctx, 0x0608, (data->Channel_Gb >> 30) & 0x3);
	write_cmos_sensor_8(ctx, 0x0609, (data->Channel_Gb >> 22) & 0xff);
	return ERROR_NONE;
}

static kal_int32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	UINT8 temperature = 0;
	INT32 temperature_convert = 0;

	temperature = read_cmos_sensor_8(ctx, 0x013a);

	if (temperature <= 0x60)
		temperature_convert = temperature;
	else if (temperature >= 0x61 && temperature <= 0x7F)
		temperature_convert = 97;
	else if (temperature >= 0x80 && temperature <= 0xE2)
		temperature_convert = -30;
	else
		temperature_convert = (INT8)temperature | 0xFFFFFF0;

/* LOG_DEBUG("temp_c(%d), read_reg(%d)\n", temperature_convert, temperature); */

	return temperature_convert;
}
static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
				 UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *) feature_para;
	/* unsigned long long *feature_return_para
	 *  = (unsigned long long *) feature_para;
	 */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
	struct SET_SENSOR_AWB_GAIN *pSetSensorAWB =
	       (struct SET_SENSOR_AWB_GAIN *)feature_para;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data
		= (MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	/*LOG_DEBUG("feature_id = %d\n", feature_id);*/
	switch (feature_id) {
	case SENSOR_FEATURE_GET_OUTPUT_FORMAT_BY_SCENARIO:
		switch (*feature_data) {
        case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
        case SENSOR_SCENARIO_ID_CUSTOM1:
			*(feature_data + 1)
			= (enum ACDK_SENSOR_OUTPUT_DATA_FORMAT_ENUM)
				imgsensor_info.sensor_output_dataformat;
			break;
		}
	break;
	case SENSOR_FEATURE_GET_ANA_GAIN_TABLE:
		if ((void *)(uintptr_t) (*(feature_data + 1)) == NULL) {
			*(feature_data + 0) =
				sizeof(imx615_ana_gain_table);
		} else {
			memcpy((void *)(uintptr_t) (*(feature_data + 1)),
			(void *)imx615_ana_gain_table,
			sizeof(imx615_ana_gain_table));
		}
		break;
	case SENSOR_FEATURE_GET_GAIN_RANGE_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_gain;
		*(feature_data + 2) = imgsensor_info.max_gain;
		break;
	case SENSOR_FEATURE_GET_BASE_GAIN_ISO_AND_STEP:
		*(feature_data + 0) = imgsensor_info.min_gain_iso;
		*(feature_data + 1) = imgsensor_info.gain_step;
		*(feature_data + 2) = imgsensor_info.gain_type;
		break;
	case SENSOR_FEATURE_GET_MIN_SHUTTER_BY_SCENARIO:
		*(feature_data + 1) = imgsensor_info.min_shutter;
		break;
	case SENSOR_FEATURE_GET_OFFSET_TO_START_OF_EXPOSURE:
		*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = 3000000;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.pclk;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.pclk;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom1.framelength << 16)
				+ imgsensor_info.custom1.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom2.framelength << 16)
				+ imgsensor_info.custom2.linelength;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.custom3.framelength << 16)
				+ imgsensor_info.custom3.linelength;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
			break;
		}
		break;
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = ctx->line_length;
		*feature_return_para_16 = ctx->frame_length;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		*feature_return_para_32 = ctx->pclk;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		 set_shutter(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_SET_NIGHTMODE:
		 /* night_mode((BOOL) *feature_data); */
		break;
	#ifdef VENDOR_EDIT
	case SENSOR_FEATURE_CHECK_MODULE_ID:
		*feature_return_para_32 = imgsensor_info.module_id;
		break;
	#endif
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT16) *feature_data);
		break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor_8(ctx, sensor_reg_data->RegAddr,
				    sensor_reg_data->RegData);
		break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor_8(ctx, sensor_reg_data->RegAddr);
		break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM
		 * or just return LENS_DRIVER_ID_DO_NOT_CARE
		 * if EEPROM does not exist in camera module.
		 */
		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(ctx, *feature_data);
		break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(ctx, feature_return_para_32);
		break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode(ctx, (BOOL)*feature_data_16,
				      *(feature_data_16+1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		 set_max_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*feature_data,
				*(feature_data+1));
		break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		 get_default_framerate_by_scenario(ctx,
				(enum MSDK_SCENARIO_ID_ENUM)*(feature_data),
				(MUINT32 *)(uintptr_t)(*(feature_data+1)));
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (BOOL)*feature_data);
		break;
	case SENSOR_FEATURE_SET_TEST_PATTERN_DATA:
		set_test_pattern_data(ctx, (struct mtk_test_pattern_data *)feature_data);
		break;
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		/* for factory mode auto testing */
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_DEBUG("current fps :%d\n", (UINT32)*feature_data_32);
		ctx->current_fps = *feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_DEBUG("ihdr enable :%d\n", (BOOL)*feature_data_32);
		ctx->ihdr_mode = *feature_data_32;
		break;
	case SENSOR_FEATURE_GET_CROP_INFO:
		LOG_DEBUG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
			(UINT32)*feature_data);
		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

		switch (*feature_data_32) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[1],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			memcpy((void *)wininfo,
				(void *)&imgsensor_winsize_info[2],
				sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[6],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[7],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
			break;
		}
		break;
	case SENSOR_FEATURE_GET_BINNING_TYPE:
		switch (*(feature_data + 1)) {
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*feature_return_para_32 = 1000; /*BINNING_NONE*/
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		case SENSOR_SCENARIO_ID_CUSTOM2:
		case SENSOR_SCENARIO_ID_CUSTOM3:
		case SENSOR_SCENARIO_ID_CUSTOM4:
		case SENSOR_SCENARIO_ID_CUSTOM5:
		default:
			*feature_return_para_32 = 4000; /*BINNING_AVERAGED*/
			break;
		}
		pr_debug("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_DEBUG("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16)*feature_data,
			(UINT16)*(feature_data+1),
			(UINT16)*(feature_data+2));
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx, (UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		/*
		 * 1, if driver support new sw frame sync
		 * set_shutter_frame_length(ctx) support third para auto_extend_en
		 */
		*(feature_data + 1) = 1;
		/* margin info by scenario */
		*(feature_data + 2) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		imx615_awb_gain(ctx, pSetSensorAWB);
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16)*feature_data, (UINT16)*(feature_data+1));
		break;
	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(ctx, KAL_FALSE);
		break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		LOG_DEBUG("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
			*feature_data);
		if (*feature_data != 0)
			set_shutter(ctx, *feature_data);
		streaming_control(ctx, KAL_TRUE);
		break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
		memcpy(feature_return_para_32,
		&ctx->ae_frm_mode, sizeof(struct IMGSENSOR_AE_FRM_MODE));
		break;
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = ctx->current_ae_effective_frame;
		break;
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
	{
		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
	}
	break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_PRELOAD_EEPROM_DATA:
		/*get eeprom preloader data*/
		*feature_return_para_32 = ctx->is_read_preload_eeprom;
		*feature_para_len = 4;
		if (ctx->is_read_preload_eeprom != 1)
			read_sensor_Cali(ctx);
		break;
	default:
		break;
	}

	return ERROR_NONE;
} /* feature_control(ctx) */

#ifdef IMGSENSOR_VC_ROUTING
static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0CD0,
			.vsize = 0x09A0,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cd0,
			.vsize = 0x0740,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cd0,
			.vsize = 0x0740,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cd0,
			.vsize = 0x09A0,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1980,
			.vsize = 0x1320,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0cd0,
			.vsize = 0x0740,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0918,
			.vsize = 0x06d4,
		},
	},
};

static int get_frame_desc(struct subdrv_ctx *ctx,
		int scenario_id, struct mtk_mbus_frame_desc *fd)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_CUSTOM1:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust1);
		memcpy(fd->entry, frame_desc_cust1, sizeof(frame_desc_cust1));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM2:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust2);
		memcpy(fd->entry, frame_desc_cust2, sizeof(frame_desc_cust2));
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cust3);
		memcpy(fd->entry, frame_desc_cust3, sizeof(frame_desc_cust3));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_prev);
		memcpy(fd->entry, frame_desc_prev, sizeof(frame_desc_prev));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_cap);
		memcpy(fd->entry, frame_desc_cap, sizeof(frame_desc_cap));
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_vid);
		memcpy(fd->entry, frame_desc_vid, sizeof(frame_desc_vid));
		break;
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_hs_vid);
		memcpy(fd->entry, frame_desc_hs_vid, sizeof(frame_desc_hs_vid));
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		fd->type = MTK_MBUS_FRAME_DESC_TYPE_CSI2;
		fd->num_entries = ARRAY_SIZE(frame_desc_slim_vid);
		memcpy(fd->entry, frame_desc_slim_vid, sizeof(frame_desc_slim_vid));
		break;
	default:
		return -1;
	}

	return 0;
}
#endif
static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	/* support long exposure at most 128 times) */
	.exposure_max = (0xffff * 128) - 32,
	.exposure_min = 2,
	.exposure_step = 1,
	.frame_time_delay_frame = 3,
	.margin = 32,
	.is_hflip = 1,
	.is_vflip = 1,
	.max_frame_length = 0xffff,

	.mirror = IMAGE_NORMAL,	/* mirrorflip information */
	.sensor_mode = IMGSENSOR_MODE_INIT,
	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.shutter = 0x3D0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 300,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,
	.ihdr_mode = 0, /* sensor need support LE, SE with HDR feature */
	.i2c_write_id = 0x34, /* record current sensor's i2c write id */
	.current_ae_effective_frame = 2,
	.ae_ctrl_gph_en = 0,
};

static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}

static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		csi_param->dphy_trail = 0x47;
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
	case SENSOR_SCENARIO_ID_CUSTOM2:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 0;
		csi_param->not_fixed_dphy_settle = 1;
		csi_param->dphy_data_settle = 0x13;
		csi_param->dphy_clk_settle = 0x13;
		csi_param->dphy_trail = 0x31;
		csi_param->dphy_csi2_resync_dmy_cycle = 0xF;
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		csi_param->dphy_trail = 0x37;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 1;
		csi_param->not_fixed_dphy_settle = 1;
		switch (ctx->aov_csi_clk) {
		case 242:
			csi_param->dphy_data_settle = 0x13;
			csi_param->dphy_clk_settle = 0x13;
			csi_param->dphy_trail = 0x7F;
			csi_param->dphy_csi2_resync_dmy_cycle = 0x25;
			break;
		case 130:
			csi_param->dphy_data_settle = 0xA;
			csi_param->dphy_clk_settle = 0xA;
			csi_param->dphy_trail = 0x44;
			csi_param->dphy_csi2_resync_dmy_cycle = 0x14;
			break;
		}
		break;
	default:
		csi_param->legacy_phy = 0;
		csi_param->not_fixed_trail_settle = 0;
		csi_param->not_fixed_dphy_settle = 0;
		break;
	}
	return 0;
}

static struct subdrv_ops ops = {
	.get_id = get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = open,
	.get_info = get_info,
	.get_resolution = get_resolution,
	.control = control,
	.feature_control = feature_control,
	.close = close,
#ifdef IMGSENSOR_VC_ROUTING
	.get_frame_desc = get_frame_desc,
#endif
	.get_temp = get_temp,
	.get_csi_param = get_csi_param,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_MCLK, 24, 0},
	{HW_ID_PDN, 1, 0},
	{HW_ID_RST, 0, 0},
	{HW_ID_AVDD, 2800000, 0},
	{HW_ID_DVDD, 1200000, 0},
	// front camera bringup.@{
	{HW_ID_DVDD1, 1200000, 0},
	//@}
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 5},
	{HW_ID_RST, 1, 1},
};

const struct subdrv_entry imx615_mipi_raw_entry = {
	.name = "imx615_mipi_raw",
	.id = IMX615_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

