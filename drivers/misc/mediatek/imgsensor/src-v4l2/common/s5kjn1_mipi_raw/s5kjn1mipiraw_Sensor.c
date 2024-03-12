// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5kjn1mipiraw_Sensor.c
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


#define PFX "s5kjn1_camera_sensor"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__


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

#include "s5kjn1mipiraw_Sensor.h"
#include "s5kjn1_ana_gain_table.h"

#include "adaptor-subdrv.h"
#include "adaptor-i2c.h"

#define read_cmos_sensor_8(...) subdrv_i2c_rd_u8(__VA_ARGS__)
#define read_cmos_sensor(...) subdrv_i2c_rd_u16(__VA_ARGS__)
#define write_cmos_sensor_8(...) subdrv_i2c_wr_u8(__VA_ARGS__)
#define write_cmos_sensor(...) subdrv_i2c_wr_u16(__VA_ARGS__)
#define table_write_cmos_sensor(...) subdrv_i2c_wr_regs_u16(__VA_ARGS__)

#define DEBUG_LOG_EN 0
#define PFX "s5kjn1_camera_sensor"
#define LOG_INF(format, args...) pr_info(PFX "[%s] " format, __func__, ##args)
#define LOG_DEBUG(...) do { if ((DEBUG_LOG_EN)) LOG_INF(__VA_ARGS__); } while (0)
static void sensor_init(struct subdrv_ctx *ctx);

static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KJN1_SENSOR_ID,//0x38e1
	.checksum_value = 0x40631970,
	.pre = {
		.pclk = 560000000,
		.linelength = 4352,
		.framelength = 4288,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 700800000,
	},
	.cap = {
		.pclk = 560000000,
		.linelength = 4352,
		.framelength = 4288,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 700800000,
	},
	.normal_video =  {
		.pclk = 560000000,
		.linelength = 5888,
		.framelength = 3168,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 793600000,
	},
	.hs_video = {
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 4554,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.slim_video = {
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 4554,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2656,
		.grabwindow_height = 1992,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 662400000,
	},
	.custom1 = {
		.pclk = 560000000,
		.linelength = 8688,
		.framelength = 6400,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 8160,
		.grabwindow_height = 6144,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 100,
		.mipi_pixel_rate = 556000000,
	},
	.custom2 = {
		.pclk = 560000000,
		.linelength = 4096,
		.framelength = 2276,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 3840,
		.grabwindow_height = 2160,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 700800000,
		.max_framerate = 600,
	},
	.custom3 = {
		.pclk = 600000000,
		.linelength = 4848,
		.framelength = 4124,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2040,
		.grabwindow_height = 1536,
		.mipi_data_lp2hs_settle_dc = 85,
		.max_framerate = 300,
		.mipi_pixel_rate = 320000000,
	},

	.min_gain = BASEGAIN, /*1x gain*/
	.max_gain = BASEGAIN * 64, /*16x gain*/
	.min_gain_iso = 100,
	.gain_step = 1,
	.gain_type = 2,
	.margin = 32,		/* sensor framelength & shutter margin */
	.min_shutter = 4,	/* min shutter */

	/* max framelength by sensor register's limitation */
	.max_frame_length = 0xffff,

	/* shutter delay frame for AE cycle,
	 * 2 frame with ispGain_delay-shut_delay=2-0=2
	 */
	.ae_shut_delay_frame = 0,

	/* sensor gain delay frame for AE cycle,
	 * 2 frame with ispGain_delay-sensor_gain_delay=2-0=2
	 */
	.ae_sensor_gain_delay_frame = 0,

	.ae_ispGain_delay_frame = 2,	/* isp gain delay frame for AE cycle */
	.ihdr_support = 0,	/* 1, support; 0,not support */
	.ihdr_le_firstline = 0,	/* 1,le first ; 0, se first */
	.sensor_mode_num = 8,	/* support sensor mode num */

	.cap_delay_frame = 2,	/* enter capture delay frame num */
	.pre_delay_frame = 2,	/* enter preview delay frame num */
	.video_delay_frame = 2,	/* enter video delay frame num */
	.custom1_delay_frame = 2, /* enter custom1 delay frame num */
	.custom2_delay_frame = 2, /* enter custom2 delay frame num */
	.custom3_delay_frame = 2, /* enter custom3 delay frame num */
	/* enter high speed video  delay frame num */
	.hs_video_delay_frame = 2,

	.slim_video_delay_frame = 2,	/* enter slim video delay frame num */

	.frame_time_delay_frame = 2,

	.isp_driving_current = ISP_DRIVING_8MA,	/* mclk driving current */

	/* sensor_interface_type */
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,

	/* 0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2 */
	.mipi_sensor_type = MIPI_OPHY_NCSI2,

	/* 0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL */
	.mipi_settle_delay_mode = 0,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_4CELL_BAYER_Gr,
	.mclk = 24,	/* mclk value, suggest 24 or 26 for 24Mhz or 26Mhz */
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_speed = 1000, /*support 1MHz write*/
	/* record sensor support all write id addr,
	 * only supprt 4 must end with 0xff
	 */
	.i2c_addr_table = {0x5a,0xff},
};




//int chip_id;
/* VC_Num, VC_PixelNum, ModeSelect, EXPO_Ratio, ODValue, RG_STATSMODE */
/* VC0_ID, VC0_DataType, VC0_SIZEH, VC0_SIZE,
 * VC1_ID, VC1_DataType, VC1_SIZEH, VC1_SIZEV
 */
/* VC2_ID, VC2_DataType, VC2_SIZEH, VC2_SIZE,
 * VC3_ID, VC3_DataType, VC3_SIZEH, VC3_SIZEV
 */

/*static SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[3]=
 *  {// Preview mode setting
 *  {0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *  0x00, 0x2B, 0x0910, 0x06D0, 0x01, 0x00, 0x0000, 0x0000,
 *  0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
 * // Video mode setting
 *{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
 *0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000},
 * // Capture mode setting
 *{0x02, 0x0A,   0x00,   0x08, 0x40, 0x00,
 *0x00, 0x2B, 0x1220, 0x0DA0, 0x01, 0x00, 0x0000, 0x0000,
 *0x02, 0x30, 0x00B4, 0x0360, 0x03, 0x00, 0x0000, 0x0000}};
 */

/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[8] = {
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0,   4080, 3072,    0,    0, 4080, 3072}, // preveiw
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   0,   4080, 3072,    0,    0, 4080, 3072}, // capture
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0,   388, 4080, 2296,    0,    0, 4080, 2296}, // video
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  500, 456, 3080, 2160,    0,    0, 3080, 2160}, // hs_video
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  712, 540, 2656, 1992,    0,    0, 2656, 1992}, // slim_video
	{ 8160, 6144,    0,    0, 8160, 6144, 8160, 6144,  0,   0,   8160, 6144,    0,    0, 8160, 6144}, // custom1
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  120, 456, 3840, 2160,    0,    0, 3840, 2160}, // custom2
	{ 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  1020,768, 2040, 1536,    0,    0, 2040, 1536}, // custom3
};


/* no mirror flip, and no binning -revised by dj */
/* static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
 * .i4OffsetX = 16,
 * .i4OffsetY = 16,
 * .i4PitchX  = 64,
 * .i4PitchY  = 64,
 * .i4PairNum  =16,
 * .i4SubBlkW  =16,
 * .i4SubBlkH  =16,
 * .i4PosL = {{20,23},{72,23},{36,27},{56,27},{24,43},{68,43},{40,47},
 * {52,47},{40,55},{52,55},{24,59},{68,59},{36,75},{56,75},{20,79},{72,79}},
 * .i4PosR = {{20,27},{72,27},{36,31},{56,31},{24,39},{68,39},{40,43},{52,43},
 * {40,59},{52,59},{24,63},{68,63},{36,71},{56,71},{20,75},{72,75}},
 * .iMirrorFlip = 0,
 * .i4BlockNumX = 72,
 * .i4BlockNumY = 54,
 * };
 */

#define RWB_ID_OFFSET 0x0F73
#define EEPROM_READ_ID  0xA2
#define EEPROM_WRITE_ID   0xA5




static void set_dummy(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("dummyline = %d, dummypixels = %d\n",
		ctx->dummy_line, ctx->dummy_pixel);

	/* return; //for test */
	write_cmos_sensor_8(ctx, 0x0340,(ctx->frame_length>>8));
	write_cmos_sensor_8(ctx, 0x0341,(ctx->frame_length&0xff));
	write_cmos_sensor_8(ctx, 0x0342,(ctx->line_length>>8));
	write_cmos_sensor_8(ctx, 0x0343,(ctx->line_length&0xff));
}				/*      set_dummy  */


static void set_max_framerate(struct subdrv_ctx *ctx, UINT16 framerate, kal_bool min_framelength_en)
{

	kal_uint32 frame_length = ctx->frame_length;

	LOG_DEBUG("framerate = %d, min framelength should enable %d\n",
		framerate, min_framelength_en);

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
}				/*      set_max_framerate  */

kal_bool s5kjn1_long_exposure_status = 0;
static void write_shutter(struct subdrv_ctx *ctx, kal_uint32 shutter, kal_bool gph)
{
    kal_uint16 realtime_fps = 0;
    if (shutter > ctx->min_frame_length - imgsensor_info.margin)
        ctx->frame_length = shutter + imgsensor_info.margin;
    else
        ctx->frame_length = ctx->min_frame_length;

    if (ctx->frame_length > imgsensor_info.max_frame_length)
        ctx->frame_length = imgsensor_info.max_frame_length;
    if (shutter < imgsensor_info.min_shutter)
        shutter = imgsensor_info.min_shutter;

	  if (ctx->autoflicker_en) {
	      realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
	      if (realtime_fps >= 297 && realtime_fps <= 305)
	          set_max_framerate(ctx, 296, 0);
	      else if (realtime_fps >= 147 && realtime_fps <= 150)
	          set_max_framerate(ctx, 146, 0);
	  }

    if (shutter >= (imgsensor_info.max_frame_length - imgsensor_info.margin))
    {
        kal_uint16 new_framelength;
        kal_uint16 long_shutter = 0;

        s5kjn1_long_exposure_status = 1;
        long_shutter = shutter / 128;
        LOG_DEBUG("Enter Long Exposure Mode: long_shutter = %d, frameLength = %d\n", shutter, ctx->frame_length);
        new_framelength = long_shutter + 74;

        write_cmos_sensor(ctx, 0x0340, new_framelength & 0xFFFF);
        write_cmos_sensor(ctx, 0x0202, long_shutter & 0xFFFF);
        write_cmos_sensor(ctx, 0x0702, 0x0700);
        write_cmos_sensor(ctx, 0x0704, 0x0700);

        ctx->ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        ctx->ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        ctx->current_ae_effective_frame = 2;
    }
    else if (s5kjn1_long_exposure_status == 1)
    {
        s5kjn1_long_exposure_status = 0;
        write_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
        write_cmos_sensor(ctx, 0x0202, shutter & 0xFFFF);
        write_cmos_sensor(ctx, 0x0702, 0x0000);
        write_cmos_sensor(ctx, 0x0704, 0x0000);
        ctx->current_ae_effective_frame = 2;
        LOG_DEBUG("Exit Long Shutter Mode: shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
    }
    else
    {
        write_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
        write_cmos_sensor(ctx, 0x0202, shutter & 0xFFFF);
        LOG_DEBUG("Normal Exposure Mode: shutter =%d, framelength =%d\n", shutter, ctx->frame_length);
    }

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
	kal_bool gph = KAL_TRUE;
	ctx->shutter = shutter;

	write_shutter(ctx, shutter, gph);
}				/*      set_shutter */


static void set_frame_length(struct subdrv_ctx *ctx, kal_uint16 frame_length)
{
	if (frame_length > 1)
		ctx->frame_length = frame_length;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	if (ctx->min_frame_length > ctx->frame_length)
		ctx->frame_length = ctx->min_frame_length;

	/* Extend frame length */
	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);

	LOG_DEBUG("Framelength: set=%d/input=%d/min=%d, auto_extend=%d\n",
		ctx->frame_length, frame_length, ctx->min_frame_length,read_cmos_sensor(ctx, 0x0350));
}

static void set_multi_shutter_frame_length(struct subdrv_ctx *ctx,
				kal_uint32 *shutters, kal_uint16 shutter_cnt,
				kal_uint16 frame_length)
{
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;

	if (shutter_cnt == 1) {
		ctx->shutter = shutters[0];

		/* Change frame time */
		if (frame_length > 1)
			dummy_line = frame_length - ctx->frame_length;
		ctx->frame_length = ctx->frame_length + dummy_line;

		/*  */
		if (shutters[0] > ctx->frame_length - imgsensor_info.margin)
			ctx->frame_length = shutters[0] + imgsensor_info.margin;

		if (ctx->frame_length > imgsensor_info.max_frame_length)
			ctx->frame_length = imgsensor_info.max_frame_length;

		shutters[0] = (shutters[0] < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutters[0];
		shutters[0] = (shutters[0] > (imgsensor_info.max_frame_length - imgsensor_info.margin))
			? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutters[0];

		if (ctx->autoflicker_en) {
			realtime_fps = ctx->pclk / ctx->line_length * 10 / ctx->frame_length;
			if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(ctx, 296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(ctx, 146, 0);
		}

		/* Update Shutter */
		write_cmos_sensor(ctx, 0x0340, ctx->frame_length & 0xFFFF);
		write_cmos_sensor(ctx, 0x0202, shutters[0] & 0xFFFF);

		LOG_DEBUG("shutters[0] =%d, framelength =%d\n",
				shutters[0], ctx->frame_length);
	}
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

	/* OV Recommend Solution */
	/*if shutter bigger than frame_length,
	 *should extend frame length first
	 */
	/* Change frame time */
	if (frame_length > 1)
		dummy_line = frame_length - ctx->frame_length;

	ctx->frame_length = ctx->frame_length + dummy_line;
	/*remove for sony sensor have auto-extend*/
	//if (shutter > ctx->frame_length - imgsensor_info.margin)
	//	ctx->frame_length = shutter + imgsensor_info.margin;

	if (ctx->frame_length > imgsensor_info.max_frame_length)
		ctx->frame_length = imgsensor_info.max_frame_length;
	shutter = (shutter < imgsensor_info.min_shutter)
		? imgsensor_info.min_shutter
		: shutter;

	shutter = (shutter > (imgsensor_info.max_frame_length
			      - imgsensor_info.margin))
		? (imgsensor_info.max_frame_length - imgsensor_info.margin)
		: shutter;

	if (ctx->autoflicker_en) {
		realtime_fps
			= ctx->pclk
			/ ctx->line_length * 10
			/ ctx->frame_length;

		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(ctx, 296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(ctx, 146, 0);
	}

	/* Update Shutter */
	if (auto_extend_en)
		write_cmos_sensor_8(ctx, 0x0350, 0x01); /* Enable auto extend */
	else
		write_cmos_sensor_8(ctx, 0x0350, 0x00); /* Disable auto extend */
	write_cmos_sensor_8(ctx, 0x0340, ctx->frame_length >> 8);
	write_cmos_sensor_8(ctx, 0x0341, ctx->frame_length & 0xFF);
	write_cmos_sensor_8(ctx, 0x0202, (shutter >> 8) & 0xFF);
	write_cmos_sensor_8(ctx, 0x0203, shutter  & 0xFF);

	LOG_DEBUG(
		"Exit! shutter =%d, framelength =%d/%d, dummy_line=%d, auto_extend=%d\n",
		shutter, ctx->frame_length, frame_length, dummy_line,read_cmos_sensor(ctx, 0x0350));
}	/* set_shutter_frame_length */

static kal_uint32 gain2reg(struct subdrv_ctx *ctx, const kal_uint32 gain)
{
	kal_uint32 reg_gain = 0x0;

	reg_gain = gain * 32 / BASEGAIN;
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

	/* gain=1024;//for test */
	/* return; //for test */

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_DEBUG("Error gain setting");

		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 64 * BASEGAIN)
			gain = 64 * BASEGAIN;
	}

	reg_gain = gain2reg(ctx, gain);
	ctx->gain = reg_gain;
	LOG_DEBUG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(ctx, 0x0204, reg_gain);
	write_cmos_sensor_8(ctx, 0x0204,((reg_gain >> 8) & 0xFF));
	write_cmos_sensor_8(ctx, 0x0205,(reg_gain & 0xff));

	return gain;
}				/*      set_gain  */

static void set_mirror_flip(struct subdrv_ctx *ctx, kal_uint8 image_mirror)
{

	kal_uint8 itemp;

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

static kal_uint32 streaming_control(struct subdrv_ctx *ctx, kal_bool enable)
{
	//write_cmos_sensor_byte

	int i = 0;
	int framecnt = 0;
	int isStreamOn = 0;

	LOG_DEBUG("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable)
	{
		for (i = 0; i < 1000; i++)
		{
			write_cmos_sensor_8(ctx,0x0100, 0x01);
			isStreamOn = read_cmos_sensor_8(ctx,0x0100); /* waiting for sensor to  stop output  then  set the  setting */
			LOG_DEBUG("isStreamOn %d ", isStreamOn);

			if ((isStreamOn & 0x1) == 0x01)
			{
				return ERROR_NONE;
			}
			else
			{
				mdelay(1);
			}
		}
	}
	else
	{
		for (i = 0; i < 1000; i++)
		{
			write_cmos_sensor_8(ctx, 0x0100, 0x00);
			framecnt = read_cmos_sensor_8(ctx,0x0005);
			if ((framecnt & 0xff) == 0xFF)
			{
				LOG_DEBUG("StreamOff OK at framecnt=%d.\n", framecnt);
				break;
			}
			else
			{
				LOG_DEBUG("StreamOFF is not on, %d, i=%d", framecnt, i);
				mdelay(1);
			}
		}
	}
	return ERROR_NONE;
}

static kal_uint16 addr_data_pair_init_jn1[] = {
    0x6028,	0x2400,
    0x602A,	0x1354,
    0x6F12,	0x0100,
    0x6F12,	0x7017,
    0x602A,	0x13B2,
    0x6F12,	0x0000,
    0x602A,	0x1236,
    0x6F12,	0x0000,
    0x602A,	0x1A0A,
    0x6F12,	0x4C0A,
    0x602A,	0x2210,
    0x6F12,	0x3401,
    0x602A,	0x2176,
    0x6F12,	0x6400,
    0x602A,	0x222E,
    0x6F12,	0x0001,
    0x602A,	0x06B6,
    0x6F12,	0x0A00,
    0x602A,	0x06BC,
    0x6F12,	0x1001,
    0x602A,	0x2140,
    0x6F12,	0x0101,
    0x602A,	0x1A0E,
    0x6F12,	0x9600,
    0x6028,	0x4000,
    0xF44E,	0x0011,
    0xF44C,	0x0B0B,
    0xF44A,	0x0006,
    0x0118,	0x0002,
    0x011A,	0x0001,
};

static void sensor_init(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E\n", __func__);
	/* initial sequence */
	// Convert from : "InitGlobal.sset"


	write_cmos_sensor(ctx,0x6028,0x4000);
	write_cmos_sensor(ctx,0x0000,0x0003);
	write_cmos_sensor(ctx,0x0000,0x38E1);
	write_cmos_sensor(ctx,0x001E,0x0007);
	write_cmos_sensor(ctx,0x6028,0x4000);
	write_cmos_sensor(ctx,0x6010,0x0001);
    mdelay(5);
	write_cmos_sensor(ctx,0x6226, 0x0001);
	mdelay(10);

	table_write_cmos_sensor(ctx, addr_data_pair_init_jn1,
		sizeof(addr_data_pair_init_jn1) / sizeof(kal_uint16));

}				/*      sensor_init  */

static kal_uint16 addr_data_pair_pre_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1201,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1F04,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x121C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x0FF0,
    0x034E, 0x0C00,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0092,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x10C0,
    0x0342, 0x1100,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
    0x0816, 0x1C00,
    };


static void preview_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s E\n", __func__);

	/* Convert from : "jn1_5M_2592x1940_30fps_MIPI534mbps.sset"*/


	/*$MV1[MCLK:24,Width:2592,Height:1940,Format:MIPI_RAW10,mipi_lane:4*/
	/*,mipi_datarate:534,pvi_pclk_inverse:0]*/

	/* ExtClk :	24	MHz*/
	/* Vt_pix_clk :	688	MHz*/
	/* MIPI_output_speed :	534	Mbps/lane*/
	/* Crop_Width :	5200	px*/
	/* Crop_Height :	3880	px*/
	/* Output_Width :	2592	px*/
	/* Output_Height :	1940	px*/
	/* Frame rate :	30.02	fps*/
	/* Output format :	Raw10*/
	/* H-size :	9008	px*/
	/* H-blank :	6416	px*/
	/* V-size :	2544	line*/
	/* V-blank :	604	line*/
	/* Lane :	4	lane*/
	/* First Pixel :	Gr	First*/
	table_write_cmos_sensor(ctx, addr_data_pair_pre_jn1,
			sizeof(addr_data_pair_pre_jn1) / sizeof(kal_uint16));



}				/*      preview_setting  */



static kal_uint16 addr_data_pair_cap_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0100,
    0x602A, 0x139C,
    0x6F12, 0x0000,
    0x602A, 0x13A0,
    0x6F12, 0x0A00,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0000,
    0x602A, 0x1A64,
    0x6F12, 0x0301,
    0x6F12, 0xFF00,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3401,
    0x602A, 0x19FC,
    0x6F12, 0x0B00,
    0x602A, 0x19F4,
    0x6F12, 0x0606,
    0x602A, 0x19F8,
    0x6F12, 0x1010,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x6207,
    0x602A, 0x1A48,
    0x6F12, 0x6207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8A00,
    0x602A, 0x1A52,
    0x6F12, 0xBF00,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x1000,
    0x602A, 0x1480,
    0x6F12, 0x1000,
    0x602A, 0x19F6,
    0x6F12, 0x0904,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0100,
    0x602A, 0x09C0,
    0x6F12, 0x2008,
    0x602A, 0x09C4,
    0x6F12, 0x2000,
    0x602A, 0x19FE,
    0x6F12, 0x0E1C,
    0x602A, 0x4D92,
    0x6F12, 0x0100,
    0x602A, 0x84C8,
    0x6F12, 0x0100,
    0x602A, 0x4D94,
    0x6F12, 0x0005,
    0x6F12, 0x000A,
    0x6F12, 0x0010,
    0x6F12, 0x0810,
    0x6F12, 0x000A,
    0x6F12, 0x0040,
    0x6F12, 0x0810,
    0x6F12, 0x0810,
    0x6F12, 0x8002,
    0x6F12, 0xFD03,
    0x6F12, 0x0010,
    0x6F12, 0x1510,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x1201,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x1F04,
    0x602A, 0x2080,
    0x6F12, 0x0101,
    0x6F12, 0xFF00,
    0x6F12, 0x7F01,
    0x6F12, 0x0001,
    0x6F12, 0x8001,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x20BA,
    0x6F12, 0x121C,
    0x6F12, 0x111C,
    0x6F12, 0x54F4,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0101,
    0x602A, 0x0718,
    0x6F12, 0x0001,
    0x602A, 0x0710,
    0x6F12, 0x0002,
    0x6F12, 0x0804,
    0x6F12, 0x0100,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x7701,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0100,
    0x602A, 0x1376,
    0x6F12, 0x0100,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0300,
    0x6F12, 0x0000,
    0x6F12, 0x0900,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x3000,
    0x6F12, 0x3000,
    0x602A, 0x0CB6,
    0x6F12, 0x0100,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0100,
    0x602A, 0x11F6,
    0x6F12, 0x0020,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0xD8FF,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x0FF0,
    0x034E, 0x0C00,
    0x0350, 0x0008,
    0x0352, 0x0008,
    0x0900, 0x0122,
    0x0380, 0x0002,
    0x0382, 0x0002,
    0x0384, 0x0002,
    0x0386, 0x0002,
    0x0110, 0x1002,
    0x0114, 0x0301,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0092,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x10C0,
    0x0342, 0x1100,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0101,
    0x0D02, 0x0101,
    0x0D04, 0x0102,
    0x6226, 0x0000,
    0x0816, 0x1C00,
    };



static void capture_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s E! currefps:%d\n", __func__, currefps);

/*
 * /  write_cmos_sensor(ctx, 0x6028, 0x4000);
 *    write_cmos_sensor_8(ctx, 0x0100, 0x00);
 *    if (currefps == 150){
 *	 Convert from : "Init.txt"
 *	 No setfile ready yet    } else if (currefps == 240){
 *	 Convert from : "Init.txt"
 *
 *
 *	No setfile ready yet    } else {	//30fps
 *	Convert from : "jn1_20M_5184x3880_30fps_MIPI1680mbps.sset"
 *
 *
 *	$MV1[MCLK:24,Width:5184,Height:3880,Format:MIPI_RAW10,mipi_lane:4
 *         ,mipi_datarate:1680,pvi_pclk_inverse:0]
 *	 ExtClk :	24	MHz
 *	 Vt_pix_clk :	678.4	MHz
 *	 MIPI_output_speed :	1680	Mbps/lane
 *	 Crop_Width :	5184	px
 *	 Crop_Height :	3880	px
 *	 Output_Width :	5184	px
 *	 Output_Height :	3880	px
 *	 Frame rate :	30.01	fps
 *	 Output format :	Raw10
 *	 H-size :	5640	px
 *	 H-blank :	456	px
 *	 V-size :	4008	line
 *	 V-blank :	128	line
 *	 Lane :	4	lane
 *	 First Pixel :	Gr	First
 */
	table_write_cmos_sensor(ctx, addr_data_pair_cap_jn1,
			sizeof(addr_data_pair_cap_jn1) / sizeof(kal_uint16));


}

static kal_uint16 addr_data_pair_video_jn1[] = {
	0x6028,	0x2400,
	0x602A,	0x1A28,
	0x6F12,	0x4C00,
	0x602A,	0x065A,
	0x6F12,	0x0000,
	0x602A,	0x139E,
	0x6F12,	0x0100,
	0x602A,	0x139C,
	0x6F12,	0x0000,
	0x602A,	0x13A0,
	0x6F12,	0x0A00,
	0x6F12,	0x0120,
	0x602A,	0x2072,
	0x6F12,	0x0000,
	0x602A,	0x1A64,
	0x6F12,	0x0301,
	0x6F12,	0xFF00,
	0x602A,	0x19E6,
	0x6F12,	0x0200,
	0x602A,	0x1A30,
	0x6F12,	0x3401,
	0x602A,	0x19FC,
	0x6F12,	0x0B00,
	0x602A,	0x19F4,
	0x6F12,	0x0606,
	0x602A,	0x19F8,
	0x6F12,	0x1010,
	0x602A,	0x1B26,
	0x6F12,	0x6F80,
	0x6F12,	0xA060,
	0x602A,	0x1A3C,
	0x6F12,	0x6207,
	0x602A,	0x1A48,
	0x6F12,	0x6207,
	0x602A,	0x1444,
	0x6F12,	0x2000,
	0x6F12,	0x2000,
	0x602A,	0x144C,
	0x6F12,	0x3F00,
	0x6F12,	0x3F00,
	0x602A,	0x7F6C,
	0x6F12,	0x0100,
	0x6F12,	0x2F00,
	0x6F12,	0xFA00,
	0x6F12,	0x2400,
	0x6F12,	0xE500,
	0x602A,	0x0650,
	0x6F12,	0x0600,
	0x602A,	0x0654,
	0x6F12,	0x0000,
	0x602A,	0x1A46,
	0x6F12,	0x8A00,
	0x602A,	0x1A52,
	0x6F12,	0xBF00,
	0x602A,	0x0674,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x0668,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x602A,	0x0684,
	0x6F12,	0x4001,
	0x602A,	0x0688,
	0x6F12,	0x4001,
	0x602A,	0x147C,
	0x6F12,	0x1000,
	0x602A,	0x1480,
	0x6F12,	0x1000,
	0x602A,	0x19F6,
	0x6F12,	0x0904,
	0x602A,	0x0812,
	0x6F12,	0x0000,
	0x602A,	0x1A02,
	0x6F12,	0x1800,
	0x602A,	0x2148,
	0x6F12,	0x0100,
	0x602A,	0x2042,
	0x6F12,	0x1A00,
	0x602A,	0x0874,
	0x6F12,	0x0100,
	0x602A,	0x09C0,
	0x6F12,	0x2008,
	0x602A,	0x09C4,
	0x6F12,	0x2000,
	0x602A,	0x19FE,
	0x6F12,	0x0E1C,
	0x602A,	0x4D92,
	0x6F12,	0x0100,
	0x602A,	0x84C8,
	0x6F12,	0x0100,
	0x602A,	0x4D94,
	0x6F12,	0x0005,
	0x6F12,	0x000A,
	0x6F12,	0x0010,
	0x6F12,	0x0810,
	0x6F12,	0x000A,
	0x6F12,	0x0040,
	0x6F12,	0x0810,
	0x6F12,	0x0810,
	0x6F12,	0x8002,
	0x6F12,	0xFD03,
	0x6F12,	0x0010,
	0x6F12,	0x1510,
	0x602A,	0x3570,
	0x6F12,	0x0000,
	0x602A,	0x3574,
	0x6F12,	0x1201,
	0x602A,	0x21E4,
	0x6F12,	0x0400,
	0x602A,	0x21EC,
	0x6F12,	0x1F04,
	0x602A,	0x2080,
	0x6F12,	0x0101,
	0x6F12,	0xFF00,
	0x6F12,	0x7F01,
	0x6F12,	0x0001,
	0x6F12,	0x8001,
	0x6F12,	0xD244,
	0x6F12,	0xD244,
	0x6F12,	0x14F4,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x20BA,
	0x6F12,	0x121C,
	0x6F12,	0x111C,
	0x6F12,	0x54F4,
	0x602A,	0x120E,
	0x6F12,	0x1000,
	0x602A,	0x212E,
	0x6F12,	0x0200,
	0x602A,	0x13AE,
	0x6F12,	0x0101,
	0x602A,	0x0718,
	0x6F12,	0x0001,
	0x602A,	0x0710,
	0x6F12,	0x0002,
	0x6F12,	0x0804,
	0x6F12,	0x0100,
	0x602A,	0x1B5C,
	0x6F12,	0x0000,
	0x602A,	0x0786,
	0x6F12,	0x7701,
	0x602A,	0x2022,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x1360,
	0x6F12,	0x0100,
	0x602A,	0x1376,
	0x6F12,	0x0100,
	0x6F12,	0x6038,
	0x6F12,	0x7038,
	0x6F12,	0x8038,
	0x602A,	0x1386,
	0x6F12,	0x0B00,
	0x602A,	0x06FA,
	0x6F12,	0x0000,
	0x602A,	0x4A94,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x0A76,
	0x6F12,	0x1000,
	0x602A,	0x0AEE,
	0x6F12,	0x1000,
	0x602A,	0x0B66,
	0x6F12,	0x1000,
	0x602A,	0x0BDE,
	0x6F12,	0x1000,
	0x602A,	0x0BE8,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0C56,
	0x6F12,	0x1000,
	0x602A,	0x0C60,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0CB6,
	0x6F12,	0x0100,
	0x602A,	0x0CF2,
	0x6F12,	0x0001,
	0x602A,	0x0CF0,
	0x6F12,	0x0101,
	0x602A,	0x11B8,
	0x6F12,	0x0100,
	0x602A,	0x11F6,
	0x6F12,	0x0020,
	0x602A,	0x4A74,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x218E,
	0x6F12,	0x0000,
	0x602A,	0x2268,
	0x6F12,	0xF279,
	0x602A,	0x5006,
	0x6F12,	0x0000,
	0x602A,	0x500E,
	0x6F12,	0x0100,
	0x602A,	0x4E70,
	0x6F12,	0x2062,
	0x6F12,	0x5501,
	0x602A,	0x06DC,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6028,	0x4000,
	0xF46A,	0xAE80,
	0x0344,	0x0000,
	0x0346,	0x0308,
	0x0348,	0x1FFF,
	0x034A,	0x1517,
	0x034C,	0x0FF0,
	0x034E,	0x08F8,
	0x0350,	0x0008,
	0x0352,	0x0008,
	0x0900,	0x0122,
	0x0380,	0x0002,
	0x0382,	0x0002,
	0x0384,	0x0002,
	0x0386,	0x0002,
	0x0110,	0x1002,
	0x0114,	0x0301,
	0x0116,	0x3000,
	0x0136,	0x1800,
	0x013E,	0x0000,
	0x0300,	0x0006,
	0x0302,	0x0001,
	0x0304,	0x0004,
	0x0306,	0x008C,
	0x0308,	0x0008,
	0x030A,	0x0001,
	0x030C,	0x0000,
	0x030E,	0x0003,
	0x0310,	0x007C,
	0x0312,	0x0000,
	0x080E,	0x0000,
	0x0340,	0x0C60,
	0x0342,	0x1700,
	0x0702,	0x0000,
	0x0202,	0x0100,
	0x0200,	0x0100,
	0x0D00,	0x0101,
	0x0D02,	0x0101,
	0x0D04,	0x0102,
	0x011E,	0x0100,
	0x6226,	0x0000,
};

static void normal_video_setting(struct subdrv_ctx *ctx, kal_uint16 currefps)
{
	LOG_DEBUG("%s currefps:%d", __func__, currefps);


	/*Convert from : "jn1_20M_5184x3880_30fps_MIPI1680mbps.sset"*/



	/*//$MV1[MCLK:24,Width:5184,Height:3880,
	 *Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:1680,pvi_pclk_inverse:0]
	 */
	/*
	 * ExtClk :	24	MHz
	 * Vt_pix_clk :	678.4	MHz
	 * MIPI_output_speed :	1680	Mbps/lane
	 * Crop_Width :	5184	px
	 * Crop_Height :	3880	px
	 * Output_Width :	5184	px
	 * Output_Height :	3880	px
	 * Frame rate :	30.01	fps
	 * Output format :	Raw10
	 * H-size :	5640	px
	 * H-blank :	456	px
	 * V-size :	4008	line
	 * V-blank :	128	line
	 * Lane :	4	lane
	 * First Pixel :	Gr	First
	 */

	table_write_cmos_sensor(ctx, addr_data_pair_video_jn1,
		sizeof(addr_data_pair_video_jn1) / sizeof(kal_uint16));



}

static kal_uint16 addr_data_pair_hs_jn1[] = {
	0x6028,	0x2400,
	0x602A,	0x1A28,
	0x6F12,	0x4C00,
	0x602A,	0x065A,
	0x6F12,	0x0000,
	0x602A,	0x139E,
	0x6F12,	0x0100,
	0x602A,	0x139C,
	0x6F12,	0x0000,
	0x602A,	0x13A0,
	0x6F12,	0x0A00,
	0x6F12,	0x0120,
	0x602A,	0x2072,
	0x6F12,	0x0000,
	0x602A,	0x1A64,
	0x6F12,	0x0301,
	0x6F12,	0xFF00,
	0x602A,	0x19E6,
	0x6F12,	0x0200,
	0x602A,	0x1A30,
	0x6F12,	0x3401,
	0x602A,	0x19FC,
	0x6F12,	0x0B00,
	0x602A,	0x19F4,
	0x6F12,	0x0606,
	0x602A,	0x19F8,
	0x6F12,	0x1010,
	0x602A,	0x1B26,
	0x6F12,	0x6F80,
	0x6F12,	0xA060,
	0x602A,	0x1A3C,
	0x6F12,	0x6207,
	0x602A,	0x1A48,
	0x6F12,	0x6207,
	0x602A,	0x1444,
	0x6F12,	0x2000,
	0x6F12,	0x2000,
	0x602A,	0x144C,
	0x6F12,	0x3F00,
	0x6F12,	0x3F00,
	0x602A,	0x7F6C,
	0x6F12,	0x0100,
	0x6F12,	0x2F00,
	0x6F12,	0xFA00,
	0x6F12,	0x2400,
	0x6F12,	0xE500,
	0x602A,	0x0650,
	0x6F12,	0x0600,
	0x602A,	0x0654,
	0x6F12,	0x0000,
	0x602A,	0x1A46,
	0x6F12,	0x8A00,
	0x602A,	0x1A52,
	0x6F12,	0xBF00,
	0x602A,	0x0674,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x0668,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x602A,	0x0684,
	0x6F12,	0x4001,
	0x602A,	0x0688,
	0x6F12,	0x4001,
	0x602A,	0x147C,
	0x6F12,	0x1000,
	0x602A,	0x1480,
	0x6F12,	0x1000,
	0x602A,	0x19F6,
	0x6F12,	0x0904,
	0x602A,	0x0812,
	0x6F12,	0x0000,
	0x602A,	0x1A02,
	0x6F12,	0x1800,
	0x602A,	0x2148,
	0x6F12,	0x0100,
	0x602A,	0x2042,
	0x6F12,	0x1A00,
	0x602A,	0x0874,
	0x6F12,	0x0100,
	0x602A,	0x09C0,
	0x6F12,	0x2008,
	0x602A,	0x09C4,
	0x6F12,	0x2000,
	0x602A,	0x19FE,
	0x6F12,	0x0E1C,
	0x602A,	0x4D92,
	0x6F12,	0x0100,
	0x602A,	0x84C8,
	0x6F12,	0x0100,
	0x602A,	0x4D94,
	0x6F12,	0x0005,
	0x6F12,	0x000A,
	0x6F12,	0x0010,
	0x6F12,	0x0810,
	0x6F12,	0x000A,
	0x6F12,	0x0040,
	0x6F12,	0x0810,
	0x6F12,	0x0810,
	0x6F12,	0x8002,
	0x6F12,	0xFD03,
	0x6F12,	0x0010,
	0x6F12,	0x1510,
	0x602A,	0x3570,
	0x6F12,	0x0000,
	0x602A,	0x3574,
	0x6F12,	0x1201,
	0x602A,	0x21E4,
	0x6F12,	0x0400,
	0x602A,	0x21EC,
	0x6F12,	0x1F04,
	0x602A,	0x2080,
	0x6F12,	0x0101,
	0x6F12,	0xFF00,
	0x6F12,	0x7F01,
	0x6F12,	0x0001,
	0x6F12,	0x8001,
	0x6F12,	0xD244,
	0x6F12,	0xD244,
	0x6F12,	0x14F4,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x20BA,
	0x6F12,	0x121C,
	0x6F12,	0x111C,
	0x6F12,	0x54F4,
	0x602A,	0x120E,
	0x6F12,	0x1000,
	0x602A,	0x212E,
	0x6F12,	0x0200,
	0x602A,	0x13AE,
	0x6F12,	0x0101,
	0x602A,	0x0718,
	0x6F12,	0x0001,
	0x602A,	0x0710,
	0x6F12,	0x0002,
	0x6F12,	0x0804,
	0x6F12,	0x0100,
	0x602A,	0x1B5C,
	0x6F12,	0x0000,
	0x602A,	0x0786,
	0x6F12,	0x7701,
	0x602A,	0x2022,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x1360,
	0x6F12,	0x0100,
	0x602A,	0x1376,
	0x6F12,	0x0100,
	0x6F12,	0x6038,
	0x6F12,	0x7038,
	0x6F12,	0x8038,
	0x602A,	0x1386,
	0x6F12,	0x0B00,
	0x602A,	0x06FA,
	0x6F12,	0x1000,
	0x602A,	0x4A94,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x0A76,
	0x6F12,	0x1000,
	0x602A,	0x0AEE,
	0x6F12,	0x1000,
	0x602A,	0x0B66,
	0x6F12,	0x1000,
	0x602A,	0x0BDE,
	0x6F12,	0x1000,
	0x602A,	0x0BE8,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0C56,
	0x6F12,	0x1000,
	0x602A,	0x0C60,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0CB6,
	0x6F12,	0x0100,
	0x602A,	0x0CF2,
	0x6F12,	0x0001,
	0x602A,	0x0CF0,
	0x6F12,	0x0101,
	0x602A,	0x11B8,
	0x6F12,	0x0100,
	0x602A,	0x11F6,
	0x6F12,	0x0020,
	0x602A,	0x4A74,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x218E,
	0x6F12,	0x0000,
	0x602A,	0x2268,
	0x6F12,	0xF279,
	0x602A,	0x5006,
	0x6F12,	0x0000,
	0x602A,	0x500E,
	0x6F12,	0x0100,
	0x602A,	0x4E70,
	0x6F12,	0x2062,
	0x6F12,	0x5501,
	0x602A,	0x06DC,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6028,	0x4000,
	0xF46A,	0xAE80,
	0x0344,	0x0190,
	0x0346,	0x0370,
	0x0348,	0x1FAF,
	0x034A,	0x148F,
	0x034C,	0x0F00,
	0x034E,	0x0870,
	0x0350,	0x0008,
	0x0352,	0x0010,
	0x0900,	0x0122,
	0x0380,	0x0002,
	0x0382,	0x0002,
	0x0384,	0x0002,
	0x0386,	0x0002,
	0x0110,	0x1002,
	0x0114,	0x0300,
	0x0116,	0x3000,
	0x0136,	0x1800,
	0x013E,	0x0000,
	0x0300,	0x0006,
	0x0302,	0x0001,
	0x0304,	0x0004,
	0x0306,	0x008C,
	0x0308,	0x0008,
	0x030A,	0x0001,
	0x030C,	0x0000,
	0x030E,	0x0004,
	0x0310,	0x008A,
	0x0312,	0x0000,
	0x080E,	0x0000,
	0x0340,	0x11C0,
	0x0342,	0x1000,
	0x0702,	0x0000,
	0x0202,	0x0100,
	0x0200,	0x0100,
	0x0D00,	0x0101,
	0x0D02,	0x0001,
	0x0D04,	0x0102,
	0x6226,	0x0000,
};

static void hs_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s", __func__);
	table_write_cmos_sensor(ctx, addr_data_pair_hs_jn1,
			sizeof(addr_data_pair_hs_jn1) / sizeof(kal_uint16));


}

static kal_uint16 addr_data_pair_slim_jn1[] = {
	0x6028,	0x2400,
	0x602A,	0x1A28,
	0x6F12,	0x4C00,
	0x602A,	0x065A,
	0x6F12,	0x0000,
	0x602A,	0x139E,
	0x6F12,	0x0100,
	0x602A,	0x139C,
	0x6F12,	0x0000,
	0x602A,	0x13A0,
	0x6F12,	0x0A00,
	0x6F12,	0x0120,
	0x602A,	0x2072,
	0x6F12,	0x0000,
	0x602A,	0x1A64,
	0x6F12,	0x0301,
	0x6F12,	0xFF00,
	0x602A,	0x19E6,
	0x6F12,	0x0200,
	0x602A,	0x1A30,
	0x6F12,	0x3401,
	0x602A,	0x19FC,
	0x6F12,	0x0B00,
	0x602A,	0x19F4,
	0x6F12,	0x0606,
	0x602A,	0x19F8,
	0x6F12,	0x1010,
	0x602A,	0x1B26,
	0x6F12,	0x6F80,
	0x6F12,	0xA060,
	0x602A,	0x1A3C,
	0x6F12,	0x6207,
	0x602A,	0x1A48,
	0x6F12,	0x6207,
	0x602A,	0x1444,
	0x6F12,	0x2000,
	0x6F12,	0x2000,
	0x602A,	0x144C,
	0x6F12,	0x3F00,
	0x6F12,	0x3F00,
	0x602A,	0x7F6C,
	0x6F12,	0x0100,
	0x6F12,	0x2F00,
	0x6F12,	0xFA00,
	0x6F12,	0x2400,
	0x6F12,	0xE500,
	0x602A,	0x0650,
	0x6F12,	0x0600,
	0x602A,	0x0654,
	0x6F12,	0x0000,
	0x602A,	0x1A46,
	0x6F12,	0x8A00,
	0x602A,	0x1A52,
	0x6F12,	0xBF00,
	0x602A,	0x0674,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x0668,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x6F12,	0x0800,
	0x602A,	0x0684,
	0x6F12,	0x4001,
	0x602A,	0x0688,
	0x6F12,	0x4001,
	0x602A,	0x147C,
	0x6F12,	0x1000,
	0x602A,	0x1480,
	0x6F12,	0x1000,
	0x602A,	0x19F6,
	0x6F12,	0x0904,
	0x602A,	0x0812,
	0x6F12,	0x0000,
	0x602A,	0x1A02,
	0x6F12,	0x1800,
	0x602A,	0x2148,
	0x6F12,	0x0100,
	0x602A,	0x2042,
	0x6F12,	0x1A00,
	0x602A,	0x0874,
	0x6F12,	0x0100,
	0x602A,	0x09C0,
	0x6F12,	0x2008,
	0x602A,	0x09C4,
	0x6F12,	0x2000,
	0x602A,	0x19FE,
	0x6F12,	0x0E1C,
	0x602A,	0x4D92,
	0x6F12,	0x0100,
	0x602A,	0x84C8,
	0x6F12,	0x0100,
	0x602A,	0x4D94,
	0x6F12,	0x0005,
	0x6F12,	0x000A,
	0x6F12,	0x0010,
	0x6F12,	0x0810,
	0x6F12,	0x000A,
	0x6F12,	0x0040,
	0x6F12,	0x0810,
	0x6F12,	0x0810,
	0x6F12,	0x8002,
	0x6F12,	0xFD03,
	0x6F12,	0x0010,
	0x6F12,	0x1510,
	0x602A,	0x3570,
	0x6F12,	0x0000,
	0x602A,	0x3574,
	0x6F12,	0x0000,
	0x602A,	0x21E4,
	0x6F12,	0x0400,
	0x602A,	0x21EC,
	0x6F12,	0x4801,
	0x602A,	0x2080,
	0x6F12,	0x0101,
	0x6F12,	0xFF00,
	0x6F12,	0x7F01,
	0x6F12,	0x0001,
	0x6F12,	0x8001,
	0x6F12,	0xD244,
	0x6F12,	0xD244,
	0x6F12,	0x14F4,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x20BA,
	0x6F12,	0x121C,
	0x6F12,	0x111C,
	0x6F12,	0x54F4,
	0x602A,	0x120E,
	0x6F12,	0x1000,
	0x602A,	0x212E,
	0x6F12,	0x0200,
	0x602A,	0x13AE,
	0x6F12,	0x0101,
	0x602A,	0x0718,
	0x6F12,	0x0001,
	0x602A,	0x0710,
	0x6F12,	0x0002,
	0x6F12,	0x0804,
	0x6F12,	0x0100,
	0x602A,	0x1B5C,
	0x6F12,	0x0000,
	0x602A,	0x0786,
	0x6F12,	0x7701,
	0x602A,	0x2022,
	0x6F12,	0x0500,
	0x6F12,	0x0500,
	0x602A,	0x1360,
	0x6F12,	0x0100,
	0x602A,	0x1376,
	0x6F12,	0x0100,
	0x6F12,	0x6038,
	0x6F12,	0x7038,
	0x6F12,	0x8038,
	0x602A,	0x1386,
	0x6F12,	0x0B00,
	0x602A,	0x06FA,
	0x6F12,	0x1000,
	0x602A,	0x4A94,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0300,
	0x6F12,	0x0000,
	0x6F12,	0x0900,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x0A76,
	0x6F12,	0x1000,
	0x602A,	0x0AEE,
	0x6F12,	0x1000,
	0x602A,	0x0B66,
	0x6F12,	0x1000,
	0x602A,	0x0BDE,
	0x6F12,	0x1000,
	0x602A,	0x0BE8,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0C56,
	0x6F12,	0x1000,
	0x602A,	0x0C60,
	0x6F12,	0x3000,
	0x6F12,	0x3000,
	0x602A,	0x0CB6,
	0x6F12,	0x0100,
	0x602A,	0x0CF2,
	0x6F12,	0x0001,
	0x602A,	0x0CF0,
	0x6F12,	0x0101,
	0x602A,	0x11B8,
	0x6F12,	0x0100,
	0x602A,	0x11F6,
	0x6F12,	0x0020,
	0x602A,	0x4A74,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0xD8FF,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x602A,	0x218E,
	0x6F12,	0x0000,
	0x602A,	0x2268,
	0x6F12,	0xF279,
	0x602A,	0x5006,
	0x6F12,	0x0000,
	0x602A,	0x500E,
	0x6F12,	0x0100,
	0x602A,	0x4E70,
	0x6F12,	0x2062,
	0x6F12,	0x5501,
	0x602A,	0x06DC,
	0x6F12,	0x1000,
	0x6F12,	0x1000,
	0x6F12,	0x0000,
	0x6F12,	0x0000,
	0x6028,	0x4000,
	0xF46A,	0xAE80,
	0x0344,	0x0630,
	0x0346,	0x0418,
	0x0348,	0x1B0F,
	0x034A,	0x13E7,
	0x034C,	0x0A60,
	0x034E,	0x07C8,
	0x0350,	0x0008,
	0x0352,	0x0010,
	0x0900,	0x0122,
	0x0380,	0x0002,
	0x0382,	0x0002,
	0x0384,	0x0002,
	0x0386,	0x0002,
	0x0110,	0x1002,
	0x0114,	0x0300,
	0x0116,	0x3000,
	0x0136,	0x1800,
	0x013E,	0x0000,
	0x0300,	0x0006,
	0x0302,	0x0001,
	0x0304,	0x0004,
	0x0306,	0x008C,
	0x0308,	0x0008,
	0x030A,	0x0001,
	0x030C,	0x0000,
	0x030E,	0x0004,
	0x0310,	0x008A,
	0x0312,	0x0000,
	0x080E,	0x0000,
	0x0340,	0x11C0,
	0x0342,	0x1000,
	0x0702,	0x0000,
	0x0202,	0x0100,
	0x0200,	0x0100,
	0x0D00,	0x0101,
	0x0D02,	0x0001,
	0x0D04,	0x0102,
	0x6226,	0x0000,
};

static void slim_video_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s", __func__);
	/* 1080p 60fps */

	/* Convert from : "Init.txt"*/

	table_write_cmos_sensor(ctx, addr_data_pair_slim_jn1,
		sizeof(addr_data_pair_slim_jn1) / sizeof(kal_uint16));

}

static kal_uint16 addr_data_pair_custom1_jn1[] = {
    0x6028, 0x2400,
    0x602A, 0x1A28,
    0x6F12, 0x4C00,
    0x602A, 0x065A,
    0x6F12, 0x0000,
    0x602A, 0x139E,
    0x6F12, 0x0400,
    0x602A, 0x139C,
    0x6F12, 0x0100,
    0x602A, 0x13A0,
    0x6F12, 0x0500,
    0x6F12, 0x0120,
    0x602A, 0x2072,
    0x6F12, 0x0101,
    0x602A, 0x1A64,
    0x6F12, 0x0001,
    0x6F12, 0x0000,
    0x602A, 0x19E6,
    0x6F12, 0x0200,
    0x602A, 0x1A30,
    0x6F12, 0x3403,
    0x602A, 0x19FC,
    0x6F12, 0x0700,
    0x602A, 0x19F4,
    0x6F12, 0x0707,
    0x602A, 0x19F8,
    0x6F12, 0x0B0B,
    0x602A, 0x1B26,
    0x6F12, 0x6F80,
    0x6F12, 0xA060,
    0x602A, 0x1A3C,
    0x6F12, 0x8207,
    0x602A, 0x1A48,
    0x6F12, 0x8207,
    0x602A, 0x1444,
    0x6F12, 0x2000,
    0x6F12, 0x2000,
    0x602A, 0x144C,
    0x6F12, 0x3F00,
    0x6F12, 0x3F00,
    0x602A, 0x7F6C,
    0x6F12, 0x0100,
    0x6F12, 0x2F00,
    0x6F12, 0xFA00,
    0x6F12, 0x2400,
    0x6F12, 0xE500,
    0x602A, 0x0650,
    0x6F12, 0x0600,
    0x602A, 0x0654,
    0x6F12, 0x0000,
    0x602A, 0x1A46,
    0x6F12, 0x8500,
    0x602A, 0x1A52,
    0x6F12, 0x9800,
    0x602A, 0x0674,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x0668,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0684,
    0x6F12, 0x4001,
    0x602A, 0x0688,
    0x6F12, 0x4001,
    0x602A, 0x147C,
    0x6F12, 0x0400,
    0x602A, 0x1480,
    0x6F12, 0x0400,
    0x602A, 0x19F6,
    0x6F12, 0x0404,
    0x602A, 0x0812,
    0x6F12, 0x0000,
    0x602A, 0x1A02,
    0x6F12, 0x1800,
    0x602A, 0x2148,
    0x6F12, 0x0100,
    0x602A, 0x2042,
    0x6F12, 0x1A00,
    0x602A, 0x0874,
    0x6F12, 0x0106,
    0x602A, 0x09C0,
    0x6F12, 0x4000,
    0x602A, 0x09C4,
    0x6F12, 0x4000,
    0x602A, 0x19FE,
    0x6F12, 0x0C1C,
    0x602A, 0x4D92,
    0x6F12, 0x0000,
    0x602A, 0x84C8,
    0x6F12, 0x0000,
    0x602A, 0x4D94,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x3570,
    0x6F12, 0x0000,
    0x602A, 0x3574,
    0x6F12, 0x7306,
    0x602A, 0x21E4,
    0x6F12, 0x0400,
    0x602A, 0x21EC,
    0x6F12, 0x6902,
    0x602A, 0x2080,
    0x6F12, 0x0100,
    0x6F12, 0xFF00,
    0x6F12, 0x0002,
    0x6F12, 0x0001,
    0x6F12, 0x0002,
    0x6F12, 0xD244,
    0x6F12, 0xD244,
    0x6F12, 0x14F4,
    0x6F12, 0x101C,
    0x6F12, 0x0D1C,
    0x6F12, 0x54F4,
    0x602A, 0x20BA,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x120E,
    0x6F12, 0x1000,
    0x602A, 0x212E,
    0x6F12, 0x0200,
    0x602A, 0x13AE,
    0x6F12, 0x0100,
    0x602A, 0x0718,
    0x6F12, 0x0000,
    0x602A, 0x0710,
    0x6F12, 0x0010,
    0x6F12, 0x0201,
    0x6F12, 0x0800,
    0x602A, 0x1B5C,
    0x6F12, 0x0000,
    0x602A, 0x0786,
    0x6F12, 0x1401,
    0x602A, 0x2022,
    0x6F12, 0x0500,
    0x6F12, 0x0500,
    0x602A, 0x1360,
    0x6F12, 0x0000,
    0x602A, 0x1376,
    0x6F12, 0x0000,
    0x6F12, 0x6038,
    0x6F12, 0x7038,
    0x6F12, 0x8038,
    0x602A, 0x1386,
    0x6F12, 0x0B00,
    0x602A, 0x06FA,
    0x6F12, 0x1000,
    0x602A, 0x4A94,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0400,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x6F12, 0x0800,
    0x602A, 0x0A76,
    0x6F12, 0x1000,
    0x602A, 0x0AEE,
    0x6F12, 0x1000,
    0x602A, 0x0B66,
    0x6F12, 0x1000,
    0x602A, 0x0BDE,
    0x6F12, 0x1000,
    0x602A, 0x0BE8,
    0x6F12, 0x5000,
    0x6F12, 0x5000,
    0x602A, 0x0C56,
    0x6F12, 0x1000,
    0x602A, 0x0C60,
    0x6F12, 0x5000,
    0x6F12, 0x5000,
    0x602A, 0x0CB6,
    0x6F12, 0x0000,
    0x602A, 0x0CF2,
    0x6F12, 0x0001,
    0x602A, 0x0CF0,
    0x6F12, 0x0101,
    0x602A, 0x11B8,
    0x6F12, 0x0000,
    0x602A, 0x11F6,
    0x6F12, 0x0010,
    0x602A, 0x4A74,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x602A, 0x218E,
    0x6F12, 0x0000,
    0x602A, 0x2268,
    0x6F12, 0xF279,
    0x602A, 0x5006,
    0x6F12, 0x0000,
    0x602A, 0x500E,
    0x6F12, 0x0100,
    0x602A, 0x4E70,
    0x6F12, 0x2062,
    0x6F12, 0x5501,
    0x602A, 0x06DC,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6F12, 0x0000,
    0x6028, 0x4000,
    0xF46A, 0xAE80,
    0x0344, 0x0000,
    0x0346, 0x0000,
    0x0348, 0x1FFF,
    0x034A, 0x181F,
    0x034C, 0x1FE0,
    0x034E, 0x1800,
    0x0350, 0x0010,
    0x0352, 0x0010,
    0x0900, 0x0111,
    0x0380, 0x0001,
    0x0382, 0x0001,
    0x0384, 0x0001,
    0x0386, 0x0001,
    0x0110, 0x1002,
    0x0114, 0x0300,
    0x0116, 0x3000,
    0x0136, 0x1800,
    0x013E, 0x0000,
    0x0300, 0x0006,
    0x0302, 0x0001,
    0x0304, 0x0004,
    0x0306, 0x008C,
    0x0308, 0x0008,
    0x030A, 0x0001,
    0x030C, 0x0000,
    0x030E, 0x0004,
    0x0310, 0x0074,
    0x0312, 0x0000,
    0x080E, 0x0000,
    0x0340, 0x1900,
    0x0342, 0x21F0,
    0x0702, 0x0000,
    0x0202, 0x0100,
    0x0200, 0x0100,
    0x0D00, 0x0100,
    0x0D02, 0x0001,
    0x0D04, 0x0002,
    0x6226, 0x0000,
};

static void custom1_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s", __func__);
	/* 1080p 60fps */

	/* Convert from : "Init.txt"*/

	table_write_cmos_sensor(ctx, addr_data_pair_custom1_jn1,
		sizeof(addr_data_pair_custom1_jn1) / sizeof(kal_uint16));

}

static kal_uint16 addr_data_pair_custom2_jn1[] = {
    0x6028,	0x2400,
    0x602A,	0x1A28,
    0x6F12,	0x4C00,
    0x602A,	0x065A,
    0x6F12,	0x0000,
    0x602A,	0x139E,
    0x6F12,	0x0100,
    0x602A,	0x139C,
    0x6F12,	0x0000,
    0x602A,	0x13A0,
    0x6F12,	0x0A00,
    0x6F12,	0x0120,
    0x602A,	0x2072,
    0x6F12,	0x0000,
    0x602A,	0x1A64,
    0x6F12,	0x0301,
    0x6F12,	0xFF00,
    0x602A,	0x19E6,
    0x6F12,	0x0200,
    0x602A,	0x1A30,
    0x6F12,	0x3401,
    0x602A,	0x19FC,
    0x6F12,	0x0B00,
    0x602A,	0x19F4,
    0x6F12,	0x0606,
    0x602A,	0x19F8,
    0x6F12,	0x1010,
    0x602A,	0x1B26,
    0x6F12,	0x6F80,
    0x6F12,	0xA060,
    0x602A,	0x1A3C,
    0x6F12,	0x6207,
    0x602A,	0x1A48,
    0x6F12,	0x6207,
    0x602A,	0x1444,
    0x6F12,	0x2000,
    0x6F12,	0x2000,
    0x602A,	0x144C,
    0x6F12,	0x3F00,
    0x6F12,	0x3F00,
    0x602A,	0x7F6C,
    0x6F12,	0x0100,
    0x6F12,	0x2F00,
    0x6F12,	0xFA00,
    0x6F12,	0x2400,
    0x6F12,	0xE500,
    0x602A,	0x0650,
    0x6F12,	0x0600,
    0x602A,	0x0654,
    0x6F12,	0x0000,
    0x602A,	0x1A46,
    0x6F12,	0x8A00,
    0x602A,	0x1A52,
    0x6F12,	0xBF00,
    0x602A,	0x0674,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x602A,	0x0668,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x602A,	0x0684,
    0x6F12,	0x4001,
    0x602A,	0x0688,
    0x6F12,	0x4001,
    0x602A,	0x147C,
    0x6F12,	0x1000,
    0x602A,	0x1480,
    0x6F12,	0x1000,
    0x602A,	0x19F6,
    0x6F12,	0x0904,
    0x602A,	0x0812,
    0x6F12,	0x0000,
    0x602A,	0x1A02,
    0x6F12,	0x1800,
    0x602A,	0x2148,
    0x6F12,	0x0100,
    0x602A,	0x2042,
    0x6F12,	0x1A00,
    0x602A,	0x0874,
    0x6F12,	0x0100,
    0x602A,	0x09C0,
    0x6F12,	0x2008,
    0x602A,	0x09C4,
    0x6F12,	0x9800,
    0x602A,	0x19FE,
    0x6F12,	0x0E1C,
    0x602A,	0x4D92,
    0x6F12,	0x0100,
    0x602A,	0x84C8,
    0x6F12,	0x0100,
    0x602A,	0x4D94,
    0x6F12,	0x0005,
    0x6F12,	0x000A,
    0x6F12,	0x0010,
    0x6F12,	0x0810,
    0x6F12,	0x000A,
    0x6F12,	0x0040,
    0x6F12,	0x0810,
    0x6F12,	0x0810,
    0x6F12,	0x8002,
    0x6F12,	0xFD03,
    0x6F12,	0x0010,
    0x6F12,	0x1510,
    0x602A,	0x3570,
    0x6F12,	0x0000,
    0x602A,	0x3574,
    0x6F12,	0x6100,
    0x602A,	0x21E4,
    0x6F12,	0x0400,
    0x602A,	0x21EC,
    0x6F12,	0xEF03,
    0x602A,	0x2080,
    0x6F12,	0x0101,
    0x6F12,	0xFF00,
    0x6F12,	0x7F01,
    0x6F12,	0x0001,
    0x6F12,	0x8001,
    0x6F12,	0xD244,
    0x6F12,	0xD244,
    0x6F12,	0x14F4,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x20BA,
    0x6F12,	0x121C,
    0x6F12,	0x111C,
    0x6F12,	0x54F4,
    0x602A,	0x120E,
    0x6F12,	0x1000,
    0x602A,	0x212E,
    0x6F12,	0x0200,
    0x602A,	0x13AE,
    0x6F12,	0x0101,
    0x602A,	0x0718,
    0x6F12,	0x0001,
    0x602A,	0x0710,
    0x6F12,	0x0002,
    0x6F12,	0x0804,
    0x6F12,	0x0100,
    0x602A,	0x1B5C,
    0x6F12,	0x0000,
    0x602A,	0x0786,
    0x6F12,	0x7701,
    0x602A,	0x2022,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x602A,	0x1360,
    0x6F12,	0x0100,
    0x602A,	0x1376,
    0x6F12,	0x0100,
    0x6F12,	0x6038,
    0x6F12,	0x7038,
    0x6F12,	0x8038,
    0x602A,	0x1386,
    0x6F12,	0x0B00,
    0x602A,	0x06FA,
    0x6F12,	0x1000,
    0x602A,	0x4A94,
    0x6F12,	0x0900,
    0x6F12,	0x0000,
    0x6F12,	0x0300,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0300,
    0x6F12,	0x0000,
    0x6F12,	0x0900,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x0A76,
    0x6F12,	0x1000,
    0x602A,	0x0AEE,
    0x6F12,	0x1000,
    0x602A,	0x0B66,
    0x6F12,	0x1000,
    0x602A,	0x0BDE,
    0x6F12,	0x1000,
    0x602A,	0x0BE8,
    0x6F12,	0x3000,
    0x6F12,	0x3000,
    0x602A,	0x0C56,
    0x6F12,	0x1000,
    0x602A,	0x0C60,
    0x6F12,	0x3000,
    0x6F12,	0x3000,
    0x602A,	0x0CB6,
    0x6F12,	0x0100,
    0x602A,	0x0CF2,
    0x6F12,	0x0001,
    0x602A,	0x0CF0,
    0x6F12,	0x0101,
    0x602A,	0x11B8,
    0x6F12,	0x0100,
    0x602A,	0x11F6,
    0x6F12,	0x0020,
    0x602A,	0x4A74,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0xD8FF,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0xD8FF,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x218E,
    0x6F12,	0x0000,
    0x602A,	0x2268,
    0x6F12,	0xF279,
    0x602A,	0x5006,
    0x6F12,	0x0000,
    0x602A,	0x500E,
    0x6F12,	0x0100,
    0x602A,	0x4E70,
    0x6F12,	0x2062,
    0x6F12,	0x5501,
    0x602A,	0x06DC,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6028,	0x4000,
    0xF46A,	0xAE80,
    0x0344,	0x00F0,
    0x0346,	0x0390,
    0x0348,	0x1F0F,
    0x034A,	0x148F,
    0x034C,	0x0F00,
    0x034E,	0x0870,
    0x0350,	0x0008,
    0x0352,	0x0008,
    0x0900,	0x0122,
    0x0380,	0x0002,
    0x0382,	0x0002,
    0x0384,	0x0002,
    0x0386,	0x0002,
    0x0110,	0x1002,
    0x0114,	0x0301,
    0x0116,	0x3000,
    0x0136,	0x1800,
    0x013E,	0x0000,
    0x0300,	0x0006,
    0x0302,	0x0001,
    0x0304,	0x0004,
    0x0306,	0x008C,
    0x0308,	0x0008,
    0x030A,	0x0001,
    0x030C,	0x0000,
    0x030E,	0x0004,
    0x0310,	0x0092,
    0x0312,	0x0000,
    0x080E,	0x0000,
    0x0340,	0x08E4,
    0x0342,	0x1000,
    0x0702,	0x0000,
    0x0202,	0x0100,
    0x0200,	0x0100,
    0x0D00,	0x0101,
    0x0D02,	0x0101,
    0x0D04,	0x0102,
    0x6226,	0x0000,
    0x0816,	0x1C00,

};

static void custom2_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s", __func__);
	/* 1080p 60fps */

	/* Convert from : "Init.txt"*/

	table_write_cmos_sensor(ctx, addr_data_pair_custom2_jn1,
		sizeof(addr_data_pair_custom2_jn1) / sizeof(kal_uint16));
}

static kal_uint16 addr_data_pair_custom3_jn1[] = {
    0x6028,	0x2400,
    0x602A,	0x1A28,
    0x6F12,	0x4C00,
    0x602A,	0x065A,
    0x6F12,	0x0000,
    0x602A,	0x139E,
    0x6F12,	0x0300,
    0x602A,	0x139C,
    0x6F12,	0x0000,
    0x602A,	0x13A0,
    0x6F12,	0x0A00,
    0x6F12,	0x0020,
    0x602A,	0x2072,
    0x6F12,	0x0000,
    0x602A,	0x1A64,
    0x6F12,	0x0301,
    0x6F12,	0x3F00,
    0x602A,	0x19E6,
    0x6F12,	0x0201,
    0x602A,	0x1A30,
    0x6F12,	0x3401,
    0x602A,	0x19FC,
    0x6F12,	0x0B00,
    0x602A,	0x19F4,
    0x6F12,	0x0606,
    0x602A,	0x19F8,
    0x6F12,	0x1010,
    0x602A,	0x1B26,
    0x6F12,	0x6F80,
    0x6F12,	0xA020,
    0x602A,	0x1A3C,
    0x6F12,	0x5207,
    0x602A,	0x1A48,
    0x6F12,	0x5207,
    0x602A,	0x1444,
    0x6F12,	0x2100,
    0x6F12,	0x2100,
    0x602A,	0x144C,
    0x6F12,	0x4200,
    0x6F12,	0x4200,
    0x602A,	0x7F6C,
    0x6F12,	0x0100,
    0x6F12,	0x3100,
    0x6F12,	0xF700,
    0x6F12,	0x2600,
    0x6F12,	0xE100,
    0x602A,	0x0650,
    0x6F12,	0x0600,
    0x602A,	0x0654,
    0x6F12,	0x0000,
    0x602A,	0x1A46,
    0x6F12,	0x8600,
    0x602A,	0x1A52,
    0x6F12,	0xBF00,
    0x602A,	0x0674,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x6F12,	0x0500,
    0x602A,	0x0668,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x6F12,	0x0800,
    0x602A,	0x0684,
    0x6F12,	0x4001,
    0x602A,	0x0688,
    0x6F12,	0x4001,
    0x602A,	0x147C,
    0x6F12,	0x1000,
    0x602A,	0x1480,
    0x6F12,	0x1000,
    0x602A,	0x19F6,
    0x6F12,	0x0904,
    0x602A,	0x0812,
    0x6F12,	0x0000,
    0x602A,	0x1A02,
    0x6F12,	0x0800,
    0x602A,	0x2148,
    0x6F12,	0x0100,
    0x602A,	0x2042,
    0x6F12,	0x1A00,
    0x602A,	0x0874,
    0x6F12,	0x1100,
    0x602A,	0x09C0,
    0x6F12,	0x9800,
    0x602A,	0x09C4,
    0x6F12,	0x9800,
    0x602A,	0x19FE,
    0x6F12,	0x0E1C,
    0x602A,	0x4D92,
    0x6F12,	0x0100,
    0x602A,	0x84C8,
    0x6F12,	0x0100,
    0x602A,	0x4D94,
    0x6F12,	0x4001,
    0x6F12,	0x0004,
    0x6F12,	0x0010,
    0x6F12,	0x0810,
    0x6F12,	0x0004,
    0x6F12,	0x0010,
    0x6F12,	0x0810,
    0x6F12,	0x0810,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0010,
    0x6F12,	0x0010,
    0x602A,	0x3570,
    0x6F12,	0x0000,
    0x602A,	0x3574,
    0x6F12,	0x9400,
    0x602A,	0x21E4,
    0x6F12,	0x0400,
    0x602A,	0x21EC,
    0x6F12,	0x4F01,
    0x602A,	0x2080,
    0x6F12,	0x0100,
    0x6F12,	0x7F00,
    0x6F12,	0x0002,
    0x6F12,	0x8000,
    0x6F12,	0x0002,
    0x6F12,	0xC244,
    0x6F12,	0xD244,
    0x6F12,	0x14F4,
    0x6F12,	0x141C,
    0x6F12,	0x111C,
    0x6F12,	0x54F4,
    0x602A,	0x20BA,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x120E,
    0x6F12,	0x1000,
    0x602A,	0x212E,
    0x6F12,	0x0A00,
    0x602A,	0x13AE,
    0x6F12,	0x0102,
    0x602A,	0x0718,
    0x6F12,	0x0005,
    0x602A,	0x0710,
    0x6F12,	0x0004,
    0x6F12,	0x0401,
    0x6F12,	0x0100,
    0x602A,	0x1B5C,
    0x6F12,	0x0300,
    0x602A,	0x0786,
    0x6F12,	0x7701,
    0x602A,	0x2022,
    0x6F12,	0x0101,
    0x6F12,	0x0101,
    0x602A,	0x1360,
    0x6F12,	0x0000,
    0x602A,	0x1376,
    0x6F12,	0x0200,
    0x6F12,	0x6038,
    0x6F12,	0x7038,
    0x6F12,	0x8038,
    0x602A,	0x1386,
    0x6F12,	0x0B00,
    0x602A,	0x06FA,
    0x6F12,	0x0000,
    0x602A,	0x4A94,
    0x6F12,	0x0C00,
    0x6F12,	0x0000,
    0x6F12,	0x0600,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0600,
    0x6F12,	0x0000,
    0x6F12,	0x0C00,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x0A76,
    0x6F12,	0x1000,
    0x602A,	0x0AEE,
    0x6F12,	0x1000,
    0x602A,	0x0B66,
    0x6F12,	0x1000,
    0x602A,	0x0BDE,
    0x6F12,	0x1000,
    0x602A,	0x0BE8,
    0x6F12,	0x3000,
    0x6F12,	0x3000,
    0x602A,	0x0C56,
    0x6F12,	0x1000,
    0x602A,	0x0C60,
    0x6F12,	0x3000,
    0x6F12,	0x3000,
    0x602A,	0x0CB6,
    0x6F12,	0x0000,
    0x602A,	0x0CF2,
    0x6F12,	0x0001,
    0x602A,	0x0CF0,
    0x6F12,	0x0101,
    0x602A,	0x11B8,
    0x6F12,	0x0000,
    0x602A,	0x11F6,
    0x6F12,	0x0010,
    0x602A,	0x4A74,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0xD8FF,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0xD8FF,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x602A,	0x218E,
    0x6F12,	0x0000,
    0x602A,	0x2268,
    0x6F12,	0xF279,
    0x602A,	0x5006,
    0x6F12,	0x0000,
    0x602A,	0x500E,
    0x6F12,	0x0100,
    0x602A,	0x4E70,
    0x6F12,	0x2062,
    0x6F12,	0x5501,
    0x602A,	0x06DC,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6F12,	0x0000,
    0x6028,	0x4000,
    0xF46A,	0xAE80,
    0x0344,	0x0000,
    0x0346,	0x0000,
    0x0348,	0x1FFF,
    0x034A,	0x181F,
    0x034C,	0x07F8,
    0x034E,	0x0600,
    0x0350,	0x0004,
    0x0352,	0x0004,
    0x0900,	0x0144,
    0x0380,	0x0002,
    0x0382,	0x0006,
    0x0384,	0x0002,
    0x0386,	0x0006,
    0x0110,	0x1002,
    0x0114,	0x0301,
    0x0116,	0x3000,
    0x0136,	0x1800,
    0x013E,	0x0000,
    0x0300,	0x0006,
    0x0302,	0x0001,
    0x0304,	0x0004,
    0x0306,	0x0096,
    0x0308,	0x0008,
    0x030A,	0x0001,
    0x030C,	0x0000,
    0x030E,	0x0003,
    0x0310,	0x0064,
    0x0312,	0x0001,
    0x080E,	0x0000,
    0x0340,	0x101C,
    0x0342,	0x12F0,
    0x0702,	0x0000,
    0x0202,	0x0100,
    0x0200,	0x0100,
    0x0D00,	0x0101,
    0x0D02,	0x0101,
    0x0D04,	0x0102,
    0x011E,	0x0100,
    0x6226,	0x0000,
};

static void custom3_setting(struct subdrv_ctx *ctx)
{
	LOG_DEBUG("%s", __func__);
	/* 1080p 60fps */

	/* Convert from : "Init.txt"*/

	table_write_cmos_sensor(ctx, addr_data_pair_custom3_jn1,
		sizeof(addr_data_pair_custom3_jn1) / sizeof(kal_uint16));

}

#define FOUR_CELL_SIZE 9490/*size = 3072 = 0xc00*/
#define FOUR_CELL_SIZE_XTC 3502
#define FOUR_CELL_SIZE_SENSOR_XTC 768
#define FOUR_CELL_SIZE_PDXTC 4000
#define FOUR_CELL_SIZE_SW_GGC 626
static int Is_Read_4Cell;
static char Four_Cell_Array[FOUR_CELL_SIZE + 2];
static void read_4cell_from_eeprom(struct subdrv_ctx *ctx, char *data)
{
	int ret;
	int addr_xtc = 0x7B9;/*Start of 4 cell data*/
	int addr_sensor_xtc = 0x1569;
	int addr_pdxtc = 0x186b;
	int addr_sw_ggc = 0x280d;

	char temp;

	if (Is_Read_4Cell != 1) {
		LOG_DEBUG("Need to read i2C\n");

		/* Check I2C is normal */
		ret = adaptor_i2c_rd_u8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, addr_xtc, &temp);
		if (ret != 0) {
			LOG_DEBUG("iReadRegI2C error\n");
			return;
		}

		Four_Cell_Array[0] = (FOUR_CELL_SIZE & 0xff);/*Low*/
		Four_Cell_Array[1] = ((FOUR_CELL_SIZE >> 8) & 0xff);/*High*/

		/*Multi-Read*/
		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, addr_xtc,
			&Four_Cell_Array[2], FOUR_CELL_SIZE_XTC);
		
		memset(&Four_Cell_Array[3504],0,594);

		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, addr_sensor_xtc,
			&Four_Cell_Array[4098], FOUR_CELL_SIZE_SENSOR_XTC);

		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, addr_pdxtc,
			&Four_Cell_Array[4866], FOUR_CELL_SIZE_PDXTC);

		adaptor_i2c_rd_p8(ctx->i2c_client,
			EEPROM_READ_ID >> 1, addr_sw_ggc,
			&Four_Cell_Array[8866], FOUR_CELL_SIZE_SW_GGC);
		
		Is_Read_4Cell = 1;
	}

	if (data != NULL) {
		LOG_DEBUG("return data\n");
		memcpy(data, Four_Cell_Array, FOUR_CELL_SIZE);
	}
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
//	kal_uint16 sp8spFlag = 0;

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 *we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			*sensor_id = ((read_cmos_sensor_8(ctx, 0x0000) << 8)
				      | read_cmos_sensor_8(ctx, 0x0001));
			if (*sensor_id == imgsensor_info.sensor_id) {
				printk("s5kjn1 i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, *sensor_id);
				/* preload 4cell data */
				read_4cell_from_eeprom(ctx, NULL);
				return ERROR_NONE;
			}
			printk("s5kjn1 Read sensor id fail, id: 0x%x, sensor id: 0x%x\n",
				ctx->i2c_write_id,*sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
	/* if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF */
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
	kal_uint8 retry = 3;
	kal_uint16 sensor_id = 0;

	LOG_DEBUG("%s", __func__);

	/* sensor have two i2c address 0x6c 0x6d & 0x21 0x20,
	 * we should detect the module used i2c address
	 */
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		ctx->i2c_write_id = imgsensor_info.i2c_addr_table[i];
		do {
			sensor_id = (
		(read_cmos_sensor_8(ctx, 0x0000) << 8) | read_cmos_sensor_8(ctx, 0x0001));

			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_DEBUG("s5kjn1 open i2c write id: 0x%x, sensor id: 0x%x\n",
					ctx->i2c_write_id, sensor_id);
				break;
			}

			LOG_DEBUG("s5kjn1 open Read sensor id fail, id: 0x%x, sensor id: 0x%x\n",
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
}				/*      open  */



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

	return ERROR_NONE;
}				/*      close  */


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
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      preview   */

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
	LOG_DEBUG("%s E\n", __func__);
	ctx->sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (ctx->current_fps == imgsensor_info.cap1.max_framerate) {
	/* PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M */
		ctx->pclk = imgsensor_info.cap1.pclk;
		ctx->line_length = imgsensor_info.cap1.linelength;
		ctx->frame_length = imgsensor_info.cap1.framelength;
		ctx->min_frame_length = imgsensor_info.cap1.framelength;
		ctx->autoflicker_en = KAL_FALSE;
	} else if (ctx->current_fps == imgsensor_info.cap2.max_framerate) {
		ctx->pclk = imgsensor_info.cap2.pclk;
		ctx->line_length = imgsensor_info.cap2.linelength;
		ctx->frame_length = imgsensor_info.cap2.framelength;
		ctx->min_frame_length = imgsensor_info.cap2.framelength;
		ctx->autoflicker_en = KAL_FALSE;
	} else {

		if (ctx->current_fps != imgsensor_info.cap.max_framerate) {
			LOG_DEBUG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				ctx->current_fps,
				imgsensor_info.cap.max_framerate / 10);
		}

		ctx->pclk = imgsensor_info.cap.pclk;
		ctx->line_length = imgsensor_info.cap.linelength;
		ctx->frame_length = imgsensor_info.cap.framelength;
		ctx->min_frame_length = imgsensor_info.cap.framelength;
		ctx->autoflicker_en = KAL_FALSE;
	}

	capture_setting(ctx, ctx->current_fps);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/* capture(ctx) */

static kal_uint32 normal_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_VIDEO;
	ctx->pclk = imgsensor_info.normal_video.pclk;
	ctx->line_length = imgsensor_info.normal_video.linelength;
	ctx->frame_length = imgsensor_info.normal_video.framelength;
	ctx->min_frame_length = imgsensor_info.normal_video.framelength;
	ctx->autoflicker_en = KAL_FALSE;

	normal_video_setting(ctx, ctx->current_fps);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      normal_video   */

static kal_uint32 hs_video(struct subdrv_ctx *ctx, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	ctx->pclk = imgsensor_info.hs_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.hs_video.linelength;
	ctx->frame_length = imgsensor_info.hs_video.framelength;
	ctx->min_frame_length = imgsensor_info.hs_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	hs_video_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      hs_video   */

static kal_uint32 slim_video(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	ctx->pclk = imgsensor_info.slim_video.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.slim_video.linelength;
	ctx->frame_length = imgsensor_info.slim_video.framelength;
	ctx->min_frame_length = imgsensor_info.slim_video.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	slim_video_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      slim_video       */

static kal_uint32 custom1(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM1;
	ctx->pclk = imgsensor_info.custom1.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.custom1.linelength;
	ctx->frame_length = imgsensor_info.custom1.framelength;
	ctx->min_frame_length = imgsensor_info.custom1.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	custom1_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      custom1       */

static kal_uint32 custom2(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	ctx->pclk = imgsensor_info.custom2.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.custom2.linelength;
	ctx->frame_length = imgsensor_info.custom2.framelength;
	ctx->min_frame_length = imgsensor_info.custom2.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	custom2_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      custom2       */


static kal_uint32 custom3(struct subdrv_ctx *ctx,
	MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
	MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_DEBUG("%s E\n", __func__);

	ctx->sensor_mode = IMGSENSOR_MODE_CUSTOM3;
	ctx->pclk = imgsensor_info.custom3.pclk;
	/* ctx->video_mode = KAL_TRUE; */
	ctx->line_length = imgsensor_info.custom3.linelength;
	ctx->frame_length = imgsensor_info.custom3.framelength;
	ctx->min_frame_length = imgsensor_info.custom3.framelength;
	ctx->dummy_line = 0;
	ctx->dummy_pixel = 0;
	/* ctx->current_fps = 300; */
	ctx->autoflicker_en = KAL_FALSE;
	custom3_setting(ctx);
	set_mirror_flip(ctx, ctx->mirror);

	return ERROR_NONE;
}				/*      custom3       */

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
}				/*      get_resolution  */

static int get_info(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			   MSDK_SENSOR_INFO_STRUCT *sensor_info,
			   MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	/*LOG_DEBUG("get_info -> scenario_id = %d\n", scenario_id);*/

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* not use */
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;

	/* inverse with datasheet */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4;	/* not use */
	sensor_info->SensorResetActiveHigh = FALSE;	/* not use */
	sensor_info->SensorResetDelayCount = 5;	/* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	//sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;

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
	// sensor_info->DelayFrame[SENSOR_SCENARIO_ID_CUSTOM4] =
	// 	imgsensor_info.custom4_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0;	/* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

	/* The frame of setting shutter default 0 for TG int */
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;

	/* The frame of setting sensor gain*/
	sensor_info->AESensorGainDelayFrame =
				imgsensor_info.ae_sensor_gain_delay_frame;

	sensor_info->AEISPGainDelayFrame =
				imgsensor_info.ae_ispGain_delay_frame;

	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

	/* change pdaf support mode to pdaf VC mode */
	sensor_info->PDAF_Support = 0;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;	/* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;	/* not use */
	sensor_info->SensorPixelClockCount = 3;	/* not use */
	sensor_info->SensorDataLatchCount = 2;	/* not use */

	//sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	//sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorHightSampling = 0;	/* 0 is default 1x */
	sensor_info->SensorPacketECCOrder = 1;


	return ERROR_NONE;
}				/*      get_info  */


static int control(struct subdrv_ctx *ctx, enum MSDK_SCENARIO_ID_ENUM scenario_id,
			  MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
			  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
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
}				/* control(ctx) */

static kal_uint32 set_video_mode(struct subdrv_ctx *ctx, UINT16 framerate)
{
	/* //LOG_DEBUG("framerate = %d\n ", framerate); */
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
	if (enable)		/* enable auto flicker */
		ctx->autoflicker_en = KAL_TRUE;
	else			/* Cancel Auto flick */
		ctx->autoflicker_en = KAL_FALSE;
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id,	MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_DEBUG("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length = imgsensor_info.normal_video.pclk
		    / framerate * 10 / imgsensor_info.normal_video.linelength;

		ctx->dummy_line =
	    (frame_length > imgsensor_info.normal_video.framelength)
	  ? (frame_length - imgsensor_info.normal_video.  framelength) : 0;

		ctx->frame_length =
		 imgsensor_info.normal_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	if (ctx->current_fps == imgsensor_info.cap1.max_framerate) {

		frame_length = imgsensor_info.cap1.pclk
			/ framerate * 10 / imgsensor_info.cap1.linelength;

		ctx->dummy_line =
		      (frame_length > imgsensor_info.cap1.framelength)
		    ? (frame_length - imgsensor_info.cap1.  framelength) : 0;

		ctx->frame_length =
			imgsensor_info.cap1.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
	} else if (ctx->current_fps == imgsensor_info.cap2.max_framerate) {
		frame_length = imgsensor_info.cap2.pclk
			/ framerate * 10 / imgsensor_info.cap2.linelength;
		ctx->dummy_line =
		      (frame_length > imgsensor_info.cap2.framelength)
		    ? (frame_length - imgsensor_info.cap2.  framelength) : 0;

		ctx->frame_length =
			imgsensor_info.cap2.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
	} else {
		if (ctx->current_fps != imgsensor_info.cap.max_framerate)
			LOG_DEBUG("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",
				framerate,
				imgsensor_info.cap.max_framerate / 10);

		frame_length = imgsensor_info.cap.pclk
			/ framerate * 10 / imgsensor_info.cap.linelength;
		ctx->dummy_line =
			(frame_length > imgsensor_info.cap.framelength)
			? (frame_length - imgsensor_info.cap.framelength) : 0;
		ctx->frame_length =
			imgsensor_info.cap.framelength + ctx->dummy_line;
		ctx->min_frame_length = ctx->frame_length;
	}
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		frame_length = imgsensor_info.hs_video.pclk
			/ framerate * 10 / imgsensor_info.hs_video.linelength;
		ctx->dummy_line =
		  (frame_length > imgsensor_info.hs_video.framelength)
		? (frame_length - imgsensor_info.hs_video.  framelength) : 0;

		ctx->frame_length =
		    imgsensor_info.hs_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		frame_length = imgsensor_info.slim_video.pclk
			/ framerate * 10 / imgsensor_info.slim_video.linelength;

		ctx->dummy_line =
		  (frame_length > imgsensor_info.slim_video.framelength)
		? (frame_length - imgsensor_info.slim_video.  framelength) : 0;

		ctx->frame_length =
		  imgsensor_info.slim_video.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

		case SENSOR_SCENARIO_ID_CUSTOM1:
		frame_length = imgsensor_info.custom1.pclk
			/ framerate * 10 / imgsensor_info.custom1.linelength;

		ctx->dummy_line =
		  (frame_length > imgsensor_info.custom1.framelength)
		? (frame_length - imgsensor_info.custom1.  framelength) : 0;

		ctx->frame_length =
		  imgsensor_info.custom1.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

		case SENSOR_SCENARIO_ID_CUSTOM2:
		frame_length = imgsensor_info.custom2.pclk
			/ framerate * 10 / imgsensor_info.custom2.linelength;

		ctx->dummy_line =
		  (frame_length > imgsensor_info.custom2.framelength)
		? (frame_length - imgsensor_info.custom2.  framelength) : 0;

		ctx->frame_length =
		  imgsensor_info.custom2.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

		case SENSOR_SCENARIO_ID_CUSTOM3:
		frame_length = imgsensor_info.custom3.pclk
			/ framerate * 10 / imgsensor_info.custom3.linelength;

		ctx->dummy_line =
		  (frame_length > imgsensor_info.custom3.framelength)
		? (frame_length - imgsensor_info.custom3.  framelength) : 0;

		ctx->frame_length =
		  imgsensor_info.custom3.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		break;

	default:		/* coding with  preview scenario by default */
		frame_length = imgsensor_info.pre.pclk
			/ framerate * 10 / imgsensor_info.pre.linelength;

		ctx->dummy_line =
			(frame_length > imgsensor_info.pre.framelength)
			? (frame_length - imgsensor_info.pre.framelength) : 0;

		ctx->frame_length =
			imgsensor_info.pre.framelength + ctx->dummy_line;

		ctx->min_frame_length = ctx->frame_length;
		if (ctx->frame_length > ctx->shutter)
			set_dummy(ctx);
		else {
			/*No need to set*/
			LOG_DEBUG("frame_length %d < shutter %d",
				ctx->frame_length, ctx->shutter);
		}
		LOG_DEBUG("error scenario_id = %d, we use preview scenario\n",
		scenario_id);
		break;
	}
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(struct subdrv_ctx *ctx,
	enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
	/*LOG_DEBUG("scenario_id = %d\n", scenario_id);*/

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
	LOG_DEBUG("mode: %d\n", mode);

	if (mode)
		write_cmos_sensor(ctx, 0x0600, mode); /*100% Color bar*/
	else if (ctx->test_pattern)
		write_cmos_sensor(ctx, 0x0600, 0x0000); /*No pattern*/

	ctx->test_pattern = mode;
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_data(struct subdrv_ctx *ctx, struct mtk_test_pattern_data *data)
{
	u16 R = (data->Channel_R >> 22) & 0x3ff;
	u16 Gr = (data->Channel_Gr >> 22) & 0x3ff;
	u16 Gb = (data->Channel_Gb >> 22) & 0x3ff;
	u16 B = (data->Channel_B >> 22) & 0x3ff;

	subdrv_i2c_wr_u16(ctx, 0x0604, R);
	subdrv_i2c_wr_u16(ctx, 0x0602, Gr);
	subdrv_i2c_wr_u16(ctx, 0x0606, B);
	subdrv_i2c_wr_u16(ctx, 0x0608, Gb);

	LOG_DEBUG("mode(%u) R/Gr/Gb/B = 0x%04x/0x%04x/0x%04x/0x%04x\n",
		ctx->test_pattern, R, Gr, Gb, B);
	return ERROR_NONE;
}

static kal_uint32 get_sensor_temperature(struct subdrv_ctx *ctx)
{
	UINT8 temperature;
	INT32 temperature_convert;

	temperature = read_cmos_sensor_8(ctx, 0x013a);

	if (temperature >= 0x0 && temperature <= 0x78)
		temperature_convert = temperature;
	else
		temperature_convert = -1;

	/*pr_info("temp_c(%d), read_reg(%d), enable %d\n",
	 *	temperature_convert, temperature, read_cmos_sensor_8(ctx, 0x0138));
	 */

	return temperature_convert;
}


static int feature_control(struct subdrv_ctx *ctx, MSDK_SENSOR_FEATURE_ENUM feature_id,
				  UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	INT32 *feature_return_para_i32 = (INT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;

	/* SET_PD_BLOCK_INFO_T *PDAFinfo; */
	/* SENSOR_VC_INFO_STRUCT *pvcinfo; */
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

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
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
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
	/* night_mode((BOOL) *feature_data); no need to implement this mode */
		break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain(ctx, (UINT32) *feature_data);
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
		/* get the lens driver ID from EEPROM or
		 * just return LENS_DRIVER_ID_DO_NOT_CARE
		 */
		/* if EEPROM does not exist in camera module. */
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
		set_auto_flicker_mode(ctx, (BOOL) (*feature_data_16),
					*(feature_data_16 + 1));
		break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario(ctx,
	    (enum MSDK_SCENARIO_ID_ENUM) *feature_data, *(feature_data + 1));
		break;

	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario(ctx,
			(enum MSDK_SCENARIO_ID_ENUM) *(feature_data),
			  (MUINT32 *) (uintptr_t) (*(feature_data + 1)));
		break;

	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode(ctx, (UINT32)*feature_data);
		break;

	case SENSOR_FEATURE_SET_TEST_PATTERN_DATA:
		set_test_pattern_data(ctx, (struct mtk_test_pattern_data *)feature_data);
		break;
	/* for factory mode auto testing */
	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;
	case SENSOR_FEATURE_SET_FRAMERATE:
		LOG_DEBUG("current fps :%d\n", *feature_data_32);
		ctx->current_fps = (UINT16)*feature_data_32;
		break;
	case SENSOR_FEATURE_SET_HDR:
		LOG_DEBUG("ihdr enable :%d\n", *feature_data_32);
		ctx->ihdr_mode = (UINT8)*feature_data_32;
		break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		/* LOG_DEBUG("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
		 *	(UINT32) *feature_data);
		 */

		wininfo =
	(struct SENSOR_WINSIZE_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));

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
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		LOG_DEBUG("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1),
			(UINT16) *(feature_data + 2));

/* ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),
 * (UINT16)*(feature_data+2));
 */
		break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
		break;
	case SENSOR_FEATURE_GET_FRAME_CTRL_INFO_BY_SCENARIO:
		*(feature_data + 1) = 1; //always 1
		(*(feature_data + 2)) = imgsensor_info.margin;
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		LOG_DEBUG("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
			(UINT16) *feature_data,
			(UINT16) *(feature_data + 1));
/* ihdr_write_shutter((UINT16)*feature_data,(UINT16)*(feature_data+1)); */
		break;

	case SENSOR_FEATURE_GET_4CELL_DATA:/*get 4 cell data from eeprom*/
	{
		int type = (kal_uint16)(*feature_data);
		char *data = (char *)(uintptr_t)(*(feature_data+1));

		if (type == FOUR_CELL_CAL_TYPE_XTALK_CAL) {
			LOG_DEBUG("Read Cross Talk Start");
			read_4cell_from_eeprom(ctx, data);
			LOG_DEBUG("Read Cross Talk = %02x %02x %02x %02x %02x %02x\n",
				(UINT16)data[0], (UINT16)data[1],
				(UINT16)data[2], (UINT16)data[3],
				(UINT16)data[4], (UINT16)data[5]);
		}
		break;
	}

		/******************** PDAF START >>> *********/
		/*
		 * case SENSOR_FEATURE_GET_PDAF_INFO:
		 * LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
		 * (UINT16)*feature_data);
		 * PDAFinfo =
		 * (SET_PD_BLOCK_INFO_T *)(uintptr_t)(*(feature_data+1));
		 * switch (*feature_data) {
		 * case SENSOR_SCENARIO_ID_NORMAL_CAPTURE: //full
		 * case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		 * case SENSOR_SCENARIO_ID_NORMAL_PREVIEW: //2x2 binning
		 * memcpy((void *)PDAFinfo,
		 * (void *)&imgsensor_pd_info,
		 * sizeof(SET_PD_BLOCK_INFO_T)); //need to check
		 * break;
		 * case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		 * case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		 * default:
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_VC_INFO:
		 * LOG_DEBUG("SENSOR_FEATURE_GET_VC_INFO %d\n",
		 * (UINT16)*feature_data);
		 * pvcinfo =
		 * (SENSOR_VC_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));
		 * switch (*feature_data_32) {
		 * case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[2],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[1],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		 * default:
		 * memcpy((void *)pvcinfo,(void *)&SENSOR_VC_INFO[0],
		 * sizeof(SENSOR_VC_INFO_STRUCT));
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		 * LOG_DEBUG(
		 * "SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
		 * (UINT16)*feature_data);
		 * //PDAF capacity enable or not
		 * switch (*feature_data) {
		 * case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
		 * (MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * break;
		 * case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * // video & capture use same setting
		 * break;
		 * case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * case SENSOR_SCENARIO_ID_SLIM_VIDEO:
		 * //need to check
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 1;
		 * break;
		 * default:
		 * *(MUINT32 *)(uintptr_t)(*(feature_data+1)) = 0;
		 * break;
		 * }
		 * break;
		 * case SENSOR_FEATURE_GET_PDAF_DATA: //get cal data from eeprom
		 * LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_DATA\n");
		 * read_2T7_eeprom((kal_uint16 )(*feature_data),
		 * (char*)(uintptr_t)(*(feature_data+1)),
		 * (kal_uint32)(*(feature_data+2)));
		 * LOG_DEBUG("SENSOR_FEATURE_GET_PDAF_DATA success\n");
		 * break;
		 * case SENSOR_FEATURE_SET_PDAF:
		 * LOG_DEBUG("PDAF mode :%d\n", *feature_data_16);
		 * ctx->pdaf_mode= *feature_data_16;
		 * break;
		 */
		/******************** PDAF END   <<< *********/
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
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
		*feature_return_para_32 = ctx->current_ae_effective_frame;
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
		default:
			*feature_return_para_32 = 1000; /*BINNING_AVERAGED*/
			break;
		}
		LOG_DEBUG("SENSOR_FEATURE_GET_BINNING_TYPE AE_binning_type:%d,\n",
			*feature_return_para_32);
		*feature_para_len = 4;
		break;
if(0){
	case SENSOR_FEATURE_GET_TEMPERATURE_VALUE:
		*feature_return_para_i32 = get_sensor_temperature(ctx);
		*feature_para_len = 4;
		break;
}
	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:

		switch (*feature_data) {
		case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM1:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom1.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom2.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_CUSTOM3:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.custom3.mipi_pixel_rate;
			break;
		case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
			break;
		}
		break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length(ctx, (UINT16) (*feature_data),
					(UINT16) (*(feature_data + 1)),
					(BOOL) (*(feature_data + 2)));
		break;
	case SENSOR_FEATURE_SET_FRAMELENGTH:
		set_frame_length(ctx, (UINT16) (*feature_data));
		break;
	case SENSOR_FEATURE_SET_MULTI_SHUTTER_FRAME_TIME:
		set_multi_shutter_frame_length(ctx, (UINT32 *)(*feature_data),
					(UINT16) (*(feature_data + 1)),
					(UINT16) (*(feature_data + 2)));
		break;
	default:
		break;
	}

	return ERROR_NONE;
}				/*      feature_control(ctx)  */

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
		},
	},
};

static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0FF0,
			.vsize = 0x0C00,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1230,
			.vsize = 0x08f8,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0a60,
			.vsize = 0x7c8,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x1FE0,
			.vsize = 0x1800,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x0F00,
			.vsize = 0x0870,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cust3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2b,
			.hsize = 0x07f8,
			.vsize = 0x0600,
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
static int get_csi_param(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id,
	struct mtk_csi_param *csi_param)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_CUSTOM2:
		csi_param->dphy_trail = 75;
		break;
	case SENSOR_SCENARIO_ID_CUSTOM3:
		csi_param->dphy_trail = 143;
		break;
	default:
		break;
	}
	return 0;
}
static const struct subdrv_ctx defctx = {

	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_max = BASEGAIN * 64,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_step = 1,
	.exposure_def = 0x3D0,
	.exposure_max = (0xffff * 128) - 4,
	.exposure_min = 4,
	.exposure_step = 1,
	.frame_time_delay_frame = 2,
	.margin = 32,
	.max_frame_length = 0xffff,

	.mirror = IMAGE_NORMAL,	/* mirrorflip information */

	/* IMGSENSOR_MODE enum value,record current sensor mode,such as:
	 * INIT, Preview, Capture, Video,High Speed Video, Slim Video
	 */
	.sensor_mode = IMGSENSOR_MODE_INIT,

	.shutter = 0x3D0,	/* current shutter */
	.gain = BASEGAIN * 4,		/* current gain */
	.dummy_pixel = 0,	/* current dummypixel */
	.dummy_line = 0,	/* current dummyline */
	.current_fps = 0,	/* full size current fps : 24fps for PIP,
				 * 30fps for Normal or ZSD
				 */

	/* auto flicker enable: KAL_FALSE for disable auto flicker,
	 * KAL_TRUE for enable auto flicker
	 */
	.autoflicker_en = KAL_FALSE,

		/* test pattern mode or not.
		 * KAL_FALSE for in test pattern mode,
		 * KAL_TRUE for normal output
		 */
	.test_pattern = KAL_FALSE,

	/* current scenario id */
	.current_scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW,

	/* sensor need support LE, SE with HDR feature */
	.ihdr_mode = KAL_FALSE,
	.i2c_write_id = 0x5A,	/* record current sensor's i2c write id */
	.current_ae_effective_frame = 2,
};
#if 0
static int get_temp(struct subdrv_ctx *ctx, int *temp)
{
	*temp = get_sensor_temperature(ctx) * 1000;
	return 0;
}
#endif
static int init_ctx(struct subdrv_ctx *ctx,
		struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(ctx, &defctx, sizeof(*ctx));
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
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
	.get_frame_desc = get_frame_desc,
	.get_csi_param = get_csi_param,
	//.get_temp = get_temp,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	//{HW_ID_PDN, 0, 1},
	{HW_ID_MCLK, 24, 0},
	{HW_ID_RST, 0, 5},
	{HW_ID_DOVDD, 1800000, 1},
	//{HW_ID_PDN, 1, 1},
	//{HW_ID_DVDD1, 1050000, 10},
	//{HW_ID_AVDD1, 2800000, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 8, 1},
	{HW_ID_RST, 1, 5},
};

const struct subdrv_entry s5kjn1_mipi_raw_entry = {
	.name = "s5kjn1_mipi_raw",
	.id = S5KJN1_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

