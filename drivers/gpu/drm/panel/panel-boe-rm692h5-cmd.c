// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
#include "../mediatek/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"
#endif
#include "include/panel-boe-rm692h5-cmd.h"
/* i2c control start */
extern int tp_gesture_flag;
extern int nt_board_id;
extern void lcm_tp_vdd_disable(void);
extern int lcm_pinctrl_select_suspend(void);
extern int lcm_pinctrl_select_release(void);
extern void lcm_tp_reset_disable(void);
extern struct gpio_desc *lcm_vddi_136;

#define BLK_LEVEL_OFFSET			(0)
#define BLK_LEVEL_MAP3				(3515)

unsigned int bl_level;
struct lcm *g_ctx;
int g_vrefresh = -1;
static struct kobject *kobj = NULL;
extern unsigned int get_lcm_id(void);
extern unsigned int lcm_set_bk(unsigned char val1, unsigned char val2);
extern unsigned int get_lcm_bl_val(void);
extern unsigned int lcm_set_page(unsigned int case_num, unsigned char val1, unsigned char val2);
extern unsigned int lcm_now_state;
static void lcm_pannel_reconfig_blk(struct lcm *ctx);
extern unsigned char get_reg_val(unsigned int case_num, unsigned char val);
extern unsigned int is_system_resume;
extern bool reading_base;

/*liumiao add for cat sku start*/
#define SKU_IND "hardware.sku=IND"
extern bool sku_is_ind;

extern bool need_brightness_sync;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *pm_enable_gpio;

	bool prepared;
	bool enabled;

	bool hbm_en;
	bool hbm_wait;
	bool hbm_stat;           //0-OutHBM  1-InHBM

	int error;
};


#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_change_state(void *dsi, dcs_write_gce cb, void *handle, bool doze)
{
	char tb[] = {0xFE, 0x00};
	char state_on[] = {0x38};
	char state_doze[] = {0x39};

	if (doze){
		cb(dsi, handle, tb, ARRAY_SIZE(tb));
		cb(dsi, handle, state_doze, ARRAY_SIZE(state_doze));
	} else {
		cb(dsi, handle, tb, ARRAY_SIZE(tb));
		cb(dsi, handle, state_on, ARRAY_SIZE(state_on));
		cb(dsi, handle, tb, ARRAY_SIZE(tb));
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (ret == 0) {
		ret = lcm_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
static struct regulator *disp_bias_pos;
static struct regulator *disp_bias_neg;


static int lcm_panel_bias_regulator_init(void)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	/* please only get regulator once in a driver */
	disp_bias_pos = regulator_get(NULL, "dsv_pos");
	if (IS_ERR(disp_bias_pos)) { /* handle return value */
		ret = PTR_ERR(disp_bias_pos);
		pr_err("get dsv_pos fail, error: %d\n", ret);
		return ret;
	}

	disp_bias_neg = regulator_get(NULL, "dsv_neg");
	if (IS_ERR(disp_bias_neg)) { /* handle return value */
		ret = PTR_ERR(disp_bias_neg);
		pr_err("get dsv_neg fail, error: %d\n", ret);
		return ret;
	}

	regulator_inited = 1;
	return ret; /* must be 0 */

}

static int lcm_panel_bias_enable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	/* set voltage with min & max*/
	ret = regulator_set_voltage(disp_bias_pos, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_set_voltage(disp_bias_neg, 5400000, 5400000);
	if (ret < 0)
		pr_err("set voltage disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	/* enable regulator */
	ret = regulator_enable(disp_bias_pos);
	if (ret < 0)
		pr_err("enable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_enable(disp_bias_neg);
	if (ret < 0)
		pr_err("enable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}

static int lcm_panel_bias_disable(void)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_bias_regulator_init();

	ret = regulator_disable(disp_bias_neg);
	if (ret < 0)
		pr_err("disable regulator disp_bias_neg fail, ret = %d\n", ret);
	retval |= ret;

	ret = regulator_disable(disp_bias_pos);
	if (ret < 0)
		pr_err("disable regulator disp_bias_pos fail, ret = %d\n", ret);
	retval |= ret;

	return retval;
}
#endif

static void lcm_pannel_reconfig_blk(struct lcm *ctx)
{
	char bl_tb0[] = {0x51,0x0F,0xFF};
	unsigned int reg_level = 125;
	pr_err("[%s][%d]bl_level:%d \n",__func__,__LINE__,bl_level);

	if(bl_level)
		reg_level = bl_level;
	else
		reg_level = 0;
	bl_tb0[1] = (reg_level>>8)&0xf;
	bl_tb0[2] = (reg_level)&0xff;
	lcm_dcs_write(ctx,bl_tb0,ARRAY_SIZE(bl_tb0));

}

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return;
	}

	pr_err("gezi----------%s----%d,bl_level %d\n",__func__,__LINE__,bl_level);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 0);
	udelay(10 * 1000);
	gpiod_set_value(ctx->reset_gpio, 1);
	// udelay(250 * 1000);
	mdelay(50);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	lcm_dcs_write_seq_static(ctx,0xFA,0x01);

	lcm_dcs_write_seq_static(ctx,0xFE,0xD2);
	//lcm_dcs_write_seq_static(ctx,0x97,0x00);
	lcm_dcs_write_seq_static(ctx,0x97,0x08);

	lcm_dcs_write_seq_static(ctx,0xFE,0xD2);
	lcm_dcs_write_seq_static(ctx,0x97,0x08);

	lcm_dcs_write_seq_static(ctx,0x36,0x11);
	lcm_dcs_write_seq_static(ctx,0x39,0xAB);
	lcm_dcs_write_seq_static(ctx,0x3A,0x30);
	lcm_dcs_write_seq_static(ctx,0x3B,0x80);
	lcm_dcs_write_seq_static(ctx,0x3D,0x09);
	lcm_dcs_write_seq_static(ctx,0x3F,0x6c);
	lcm_dcs_write_seq_static(ctx,0x40,0x04);
	lcm_dcs_write_seq_static(ctx,0x41,0x3c);
	lcm_dcs_write_seq_static(ctx,0x42,0x00);
	lcm_dcs_write_seq_static(ctx,0x43,0x0c);
	lcm_dcs_write_seq_static(ctx,0x44,0x02);
	lcm_dcs_write_seq_static(ctx,0x45,0x1e);
	lcm_dcs_write_seq_static(ctx,0x46,0x02);
	lcm_dcs_write_seq_static(ctx,0x47,0x1e);
	lcm_dcs_write_seq_static(ctx,0x48,0x02);
	lcm_dcs_write_seq_static(ctx,0x49,0x00);
	lcm_dcs_write_seq_static(ctx,0x4A,0x02);
	lcm_dcs_write_seq_static(ctx,0x4B,0x10);
	lcm_dcs_write_seq_static(ctx,0x4D,0x20);
	lcm_dcs_write_seq_static(ctx,0x4E,0x01);
	lcm_dcs_write_seq_static(ctx,0x4F,0x20);
	lcm_dcs_write_seq_static(ctx,0x50,0x00);
	lcm_dcs_write_seq_static(ctx,0x51,0x07);
	lcm_dcs_write_seq_static(ctx,0x53,0x0c);
	lcm_dcs_write_seq_static(ctx,0x54,0x08);
	lcm_dcs_write_seq_static(ctx,0x55,0xbb);
	lcm_dcs_write_seq_static(ctx,0x56,0x08);
	lcm_dcs_write_seq_static(ctx,0x58,0x6e);
	lcm_dcs_write_seq_static(ctx,0x59,0x18);
	lcm_dcs_write_seq_static(ctx,0x5A,0x00);
	lcm_dcs_write_seq_static(ctx,0x5B,0x10);
	lcm_dcs_write_seq_static(ctx,0x5C,0xf0);
	lcm_dcs_write_seq_static(ctx,0x5D,0x07);
	lcm_dcs_write_seq_static(ctx,0x5E,0x10);
	lcm_dcs_write_seq_static(ctx,0x5F,0x20);
	lcm_dcs_write_seq_static(ctx,0x60,0x00);
	lcm_dcs_write_seq_static(ctx,0x61,0x06);
	lcm_dcs_write_seq_static(ctx,0x62,0x0F);
	lcm_dcs_write_seq_static(ctx,0x63,0x0F);
	lcm_dcs_write_seq_static(ctx,0x64,0x33);
	lcm_dcs_write_seq_static(ctx,0x65,0x0e);
	lcm_dcs_write_seq_static(ctx,0x66,0x1c);
	lcm_dcs_write_seq_static(ctx,0x67,0x2a);
	lcm_dcs_write_seq_static(ctx,0x68,0x38);
	lcm_dcs_write_seq_static(ctx,0x69,0x46);
	lcm_dcs_write_seq_static(ctx,0x6A,0x54);
	lcm_dcs_write_seq_static(ctx,0x6B,0x62);
	lcm_dcs_write_seq_static(ctx,0x6C,0x69);
	lcm_dcs_write_seq_static(ctx,0x6D,0x70);
	lcm_dcs_write_seq_static(ctx,0x6E,0x77);
	lcm_dcs_write_seq_static(ctx,0x6F,0x79);
	lcm_dcs_write_seq_static(ctx,0x70,0x7b);
	lcm_dcs_write_seq_static(ctx,0x71,0x7d);
	lcm_dcs_write_seq_static(ctx,0x72,0x7e);
	lcm_dcs_write_seq_static(ctx,0x73,0x02);
	lcm_dcs_write_seq_static(ctx,0x74,0x02);
	lcm_dcs_write_seq_static(ctx,0x75,0x22);
	lcm_dcs_write_seq_static(ctx,0x76,0x00);
	lcm_dcs_write_seq_static(ctx,0x77,0x2A);
	lcm_dcs_write_seq_static(ctx,0x78,0x40);
	lcm_dcs_write_seq_static(ctx,0x79,0x2A);
	lcm_dcs_write_seq_static(ctx,0x7A,0xbe);
	lcm_dcs_write_seq_static(ctx,0x7B,0x3A);
	lcm_dcs_write_seq_static(ctx,0x7C,0xfc);
	lcm_dcs_write_seq_static(ctx,0x7D,0x3A);
	lcm_dcs_write_seq_static(ctx,0x7E,0xfa);
	lcm_dcs_write_seq_static(ctx,0x7F,0x3A);
	lcm_dcs_write_seq_static(ctx,0x80,0xf8);
	lcm_dcs_write_seq_static(ctx,0x81,0x3B);
	lcm_dcs_write_seq_static(ctx,0x82,0x38);
	lcm_dcs_write_seq_static(ctx,0x83,0x3B);
	lcm_dcs_write_seq_static(ctx,0x84,0x78);
	lcm_dcs_write_seq_static(ctx,0x85,0x3B);
	lcm_dcs_write_seq_static(ctx,0x86,0xb6);
	lcm_dcs_write_seq_static(ctx,0x87,0x4B);
	lcm_dcs_write_seq_static(ctx,0x88,0xf6);
	lcm_dcs_write_seq_static(ctx,0x89,0x4C);
	lcm_dcs_write_seq_static(ctx,0x8A,0x34);
	lcm_dcs_write_seq_static(ctx,0x8B,0x4C);
	lcm_dcs_write_seq_static(ctx,0x8C,0x74);
	lcm_dcs_write_seq_static(ctx,0x8D,0x5C);
	lcm_dcs_write_seq_static(ctx,0x8E,0x74);
	lcm_dcs_write_seq_static(ctx,0x8F,0x8C);
	lcm_dcs_write_seq_static(ctx,0x90,0xf4);
	lcm_dcs_write_seq_static(ctx,0x91,0x00);
	lcm_dcs_write_seq_static(ctx,0x92,0x00);
	lcm_dcs_write_seq_static(ctx,0x93,0x00);
	lcm_dcs_write_seq_static(ctx,0x94,0x00);
	lcm_dcs_write_seq_static(ctx,0x95,0x00);
	lcm_dcs_write_seq_static(ctx,0x96,0x00);

        lcm_dcs_write_seq_static(ctx,0xFE,0xA0);
        lcm_dcs_write_seq_static(ctx,0x06,0x36);
        lcm_dcs_write_seq_static(ctx,0x7C,0x15);
        lcm_dcs_write_seq_static(ctx,0xFE,0x42);
        lcm_dcs_write_seq_static(ctx,0x17,0x02);

	lcm_dcs_write_seq_static(ctx,0xFE,0xD4);
	lcm_dcs_write_seq_static(ctx,0x40,0x03);
	lcm_dcs_write_seq_static(ctx,0xFE,0xFD);
	lcm_dcs_write_seq_static(ctx,0x80,0x06);
	lcm_dcs_write_seq_static(ctx,0x83,0x00);

	lcm_dcs_write_seq_static(ctx,0xFE,0xD0);
	lcm_dcs_write_seq_static(ctx,0x7E,0x80);
	lcm_dcs_write_seq_static(ctx,0xFE,0xB6);
	lcm_dcs_write_seq_static(ctx,0x01,0x74);
	lcm_dcs_write_seq_static(ctx,0xFE,0xA1);
	lcm_dcs_write_seq_static(ctx,0xCD,0x00);
	lcm_dcs_write_seq_static(ctx,0xCE,0x00);
	lcm_dcs_write_seq_static(ctx,0xB3,0x7F);//  hs gatting off
	lcm_dcs_write_seq_static(ctx,0x74,0x72);
	lcm_dcs_write_seq_static(ctx,0xC3,0xC3);//C3
	lcm_dcs_write_seq_static(ctx,0xC4,0xFF);//FF
	lcm_dcs_write_seq_static(ctx,0xC5,0x7F);//7F

	if(!sku_is_ind) {
		lcm_dcs_write_seq_static(ctx,0xFE,0x82);
		lcm_dcs_write_seq_static(ctx,0x00,0x50);// off od
	}
	else {
	lcm_dcs_write_seq_static(ctx,0xFE,0x58);
lcm_dcs_write_seq_static(ctx,0xC1,0x02);
lcm_dcs_write_seq_static(ctx,0xFE,0x82);
lcm_dcs_write_seq_static(ctx,0x00,0x50);
lcm_dcs_write_seq_static(ctx,0x01,0x40);
lcm_dcs_write_seq_static(ctx,0x02,0x00);
lcm_dcs_write_seq_static(ctx,0x03,0x00);
lcm_dcs_write_seq_static(ctx,0x05,0x20);
lcm_dcs_write_seq_static(ctx,0x06,0x10);
lcm_dcs_write_seq_static(ctx,0x07,0x00);
lcm_dcs_write_seq_static(ctx,0x08,0x20);
lcm_dcs_write_seq_static(ctx,0x09,0x00);
lcm_dcs_write_seq_static(ctx,0x0a,0x20);
lcm_dcs_write_seq_static(ctx,0x0b,0x00);
lcm_dcs_write_seq_static(ctx,0x0c,0x0a);
lcm_dcs_write_seq_static(ctx,0x0d,0xff);
lcm_dcs_write_seq_static(ctx,0x0e,0x00);
lcm_dcs_write_seq_static(ctx,0xFE,0x85);
lcm_dcs_write_seq_static(ctx,0xd3,0x01);
lcm_dcs_write_seq_static(ctx,0xd4,0x86);
lcm_dcs_write_seq_static(ctx,0xd5,0x02);
lcm_dcs_write_seq_static(ctx,0xd6,0x5d);
lcm_dcs_write_seq_static(ctx,0xd7,0x3e);
lcm_dcs_write_seq_static(ctx,0xd8,0x46);
lcm_dcs_write_seq_static(ctx,0xd9,0x3f);
lcm_dcs_write_seq_static(ctx,0xda,0x88);
lcm_dcs_write_seq_static(ctx,0xdb,0x68);
lcm_dcs_write_seq_static(ctx,0xdc,0x26);
lcm_dcs_write_seq_static(ctx,0xdd,0xbb);
lcm_dcs_write_seq_static(ctx,0xde,0xcd);
lcm_dcs_write_seq_static(ctx,0xdf,0xf0);
lcm_dcs_write_seq_static(ctx,0xe0,0xff);
lcm_dcs_write_seq_static(ctx,0xe1,0xff);
lcm_dcs_write_seq_static(ctx,0xe2,0xff);
lcm_dcs_write_seq_static(ctx,0xe3,0xff);
lcm_dcs_write_seq_static(ctx,0xe4,0xff);
lcm_dcs_write_seq_static(ctx,0xe5,0xa6);
lcm_dcs_write_seq_static(ctx,0xe6,0x64);
lcm_dcs_write_seq_static(ctx,0xe7,0x9a);
lcm_dcs_write_seq_static(ctx,0xe8,0x22);
lcm_dcs_write_seq_static(ctx,0xe9,0xe0);
lcm_dcs_write_seq_static(ctx,0xea,0xbb);
lcm_dcs_write_seq_static(ctx,0xeb,0x9e);
lcm_dcs_write_seq_static(ctx,0xec,0x5c);
lcm_dcs_write_seq_static(ctx,0xed,0xcd);
lcm_dcs_write_seq_static(ctx,0xee,0x1a);
lcm_dcs_write_seq_static(ctx,0xef,0xd8);
lcm_dcs_write_seq_static(ctx,0xf0,0xee);
lcm_dcs_write_seq_static(ctx,0xf2,0x96);
lcm_dcs_write_seq_static(ctx,0xf4,0xff);
lcm_dcs_write_seq_static(ctx,0xf5,0xff);
lcm_dcs_write_seq_static(ctx,0xFE,0x82);
lcm_dcs_write_seq_static(ctx,0x4e,0x51);
lcm_dcs_write_seq_static(ctx,0x4f,0x10);
lcm_dcs_write_seq_static(ctx,0x50,0x0f);
lcm_dcs_write_seq_static(ctx,0x51,0x0c);
lcm_dcs_write_seq_static(ctx,0x52,0x04);
lcm_dcs_write_seq_static(ctx,0x53,0x3c);
lcm_dcs_write_seq_static(ctx,0xc8,0x00);
lcm_dcs_write_seq_static(ctx,0xc9,0x40);
lcm_dcs_write_seq_static(ctx,0xca,0x00);
lcm_dcs_write_seq_static(ctx,0xcb,0x88);
lcm_dcs_write_seq_static(ctx,0xcc,0x00);
lcm_dcs_write_seq_static(ctx,0xcd,0x00);
lcm_dcs_write_seq_static(ctx,0xce,0x00);
lcm_dcs_write_seq_static(ctx,0xcf,0x02);
lcm_dcs_write_seq_static(ctx,0xd0,0x00);
lcm_dcs_write_seq_static(ctx,0xd1,0x00);
lcm_dcs_write_seq_static(ctx,0xd2,0x44);
lcm_dcs_write_seq_static(ctx,0xd3,0x88);
lcm_dcs_write_seq_static(ctx,0xd4,0x0f);
lcm_dcs_write_seq_static(ctx,0xd5,0x44);
lcm_dcs_write_seq_static(ctx,0xd6,0xff);
lcm_dcs_write_seq_static(ctx,0xd7,0xff);
lcm_dcs_write_seq_static(ctx,0xd8,0xff);
lcm_dcs_write_seq_static(ctx,0xd9,0xff);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x4D,0x52);
lcm_dcs_write_seq_static(ctx,0x4E,0xA0);
lcm_dcs_write_seq_static(ctx,0x4F,0x00);
lcm_dcs_write_seq_static(ctx,0x50,0x00);
lcm_dcs_write_seq_static(ctx,0xFE,0x40);
lcm_dcs_write_seq_static(ctx,0xa3,0x02);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x51,0x84);
lcm_dcs_write_seq_static(ctx,0x52,0x73);
lcm_dcs_write_seq_static(ctx,0x53,0x5f);
lcm_dcs_write_seq_static(ctx,0xFE,0x53);
lcm_dcs_write_seq_static(ctx,0x1b,0x80);
lcm_dcs_write_seq_static(ctx,0xFE,0x82);
lcm_dcs_write_seq_static(ctx,0x5c,0x00);
lcm_dcs_write_seq_static(ctx,0x5d,0x04);
lcm_dcs_write_seq_static(ctx,0x5e,0x08);
lcm_dcs_write_seq_static(ctx,0x5f,0x10);
lcm_dcs_write_seq_static(ctx,0x60,0x18);
lcm_dcs_write_seq_static(ctx,0x61,0x20);
lcm_dcs_write_seq_static(ctx,0x62,0x30);
lcm_dcs_write_seq_static(ctx,0x63,0x40);
lcm_dcs_write_seq_static(ctx,0x64,0x50);
lcm_dcs_write_seq_static(ctx,0x65,0x60);
lcm_dcs_write_seq_static(ctx,0x66,0x80);
lcm_dcs_write_seq_static(ctx,0x67,0xa0);
lcm_dcs_write_seq_static(ctx,0x68,0xc0);
lcm_dcs_write_seq_static(ctx,0x69,0xe0);
lcm_dcs_write_seq_static(ctx,0x6a,0xff);
lcm_dcs_write_seq_static(ctx,0x6f,0x40);
lcm_dcs_write_seq_static(ctx,0x70,0x00);
lcm_dcs_write_seq_static(ctx,0x71,0x40);
lcm_dcs_write_seq_static(ctx,0x72,0x00);
lcm_dcs_write_seq_static(ctx,0x73,0x20);
lcm_dcs_write_seq_static(ctx,0x74,0x00);
lcm_dcs_write_seq_static(ctx,0x75,0x20);
lcm_dcs_write_seq_static(ctx,0x76,0x00);
lcm_dcs_write_seq_static(ctx,0x77,0x20);
lcm_dcs_write_seq_static(ctx,0x78,0x00);
lcm_dcs_write_seq_static(ctx,0x79,0x10);
lcm_dcs_write_seq_static(ctx,0x7a,0x00);
lcm_dcs_write_seq_static(ctx,0x7b,0x10);
lcm_dcs_write_seq_static(ctx,0x7c,0x00);
lcm_dcs_write_seq_static(ctx,0x7d,0x10);
lcm_dcs_write_seq_static(ctx,0x7e,0x00);
lcm_dcs_write_seq_static(ctx,0x7f,0x10);
lcm_dcs_write_seq_static(ctx,0x80,0x00);
lcm_dcs_write_seq_static(ctx,0x81,0x08);
lcm_dcs_write_seq_static(ctx,0x82,0x00);
lcm_dcs_write_seq_static(ctx,0x83,0x08);
lcm_dcs_write_seq_static(ctx,0x84,0x00);
lcm_dcs_write_seq_static(ctx,0x85,0x08);
lcm_dcs_write_seq_static(ctx,0x86,0x00);
lcm_dcs_write_seq_static(ctx,0x87,0x08);
lcm_dcs_write_seq_static(ctx,0x88,0x00);
lcm_dcs_write_seq_static(ctx,0x89,0x08);
lcm_dcs_write_seq_static(ctx,0x8a,0x42);
lcm_dcs_write_seq_static(ctx,0xda,0x07);
lcm_dcs_write_seq_static(ctx,0xdb,0x35);
lcm_dcs_write_seq_static(ctx,0xc6,0x02);
lcm_dcs_write_seq_static(ctx,0xc7,0xc4);
lcm_dcs_write_seq_static(ctx,0x46,0x04);
lcm_dcs_write_seq_static(ctx,0xdc,0x00);
lcm_dcs_write_seq_static(ctx,0xdd,0x02);
lcm_dcs_write_seq_static(ctx,0xde,0x05);
lcm_dcs_write_seq_static(ctx,0xdf,0x0c);
lcm_dcs_write_seq_static(ctx,0xe0,0x08);
lcm_dcs_write_seq_static(ctx,0xe1,0xba);
lcm_dcs_write_seq_static(ctx,0xe2,0x0c);
lcm_dcs_write_seq_static(ctx,0xe3,0x7c);
lcm_dcs_write_seq_static(ctx,0x54,0x0c);
lcm_dcs_write_seq_static(ctx,0x55,0x7e);
lcm_dcs_write_seq_static(ctx,0x56,0x11);
lcm_dcs_write_seq_static(ctx,0x58,0x10);
lcm_dcs_write_seq_static(ctx,0x59,0x1b);
lcm_dcs_write_seq_static(ctx,0x5a,0x76);
lcm_dcs_write_seq_static(ctx,0xe4,0x00);
lcm_dcs_write_seq_static(ctx,0xe5,0xcb);
lcm_dcs_write_seq_static(ctx,0xea,0x00);
lcm_dcs_write_seq_static(ctx,0xe6,0x01);
lcm_dcs_write_seq_static(ctx,0xe7,0x16);
lcm_dcs_write_seq_static(ctx,0xe8,0x01);
lcm_dcs_write_seq_static(ctx,0xe9,0x10);
lcm_dcs_write_seq_static(ctx,0x47,0xff);
lcm_dcs_write_seq_static(ctx,0x48,0xff);
lcm_dcs_write_seq_static(ctx,0x49,0x00);
lcm_dcs_write_seq_static(ctx,0x4a,0xe0);
lcm_dcs_write_seq_static(ctx,0x4b,0x00);
lcm_dcs_write_seq_static(ctx,0x4c,0x62);
lcm_dcs_write_seq_static(ctx,0xec,0x00);
lcm_dcs_write_seq_static(ctx,0xed,0x0d);
lcm_dcs_write_seq_static(ctx,0xee,0x1a);
lcm_dcs_write_seq_static(ctx,0xef,0x27);
lcm_dcs_write_seq_static(ctx,0xf0,0x34);
lcm_dcs_write_seq_static(ctx,0xeb,0x03);
lcm_dcs_write_seq_static(ctx,0xf2,0x0d);
lcm_dcs_write_seq_static(ctx,0xf3,0x1a);
lcm_dcs_write_seq_static(ctx,0xf4,0x27);
lcm_dcs_write_seq_static(ctx,0xc4,0x00);
lcm_dcs_write_seq_static(ctx,0xc5,0x68);
lcm_dcs_write_seq_static(ctx,0xf5,0x73);
lcm_dcs_write_seq_static(ctx,0xf6,0x78);
lcm_dcs_write_seq_static(ctx,0xf7,0xf0);
lcm_dcs_write_seq_static(ctx,0xf8,0x68);
lcm_dcs_write_seq_static(ctx,0xf9,0xe0);
lcm_dcs_write_seq_static(ctx,0x8f,0x0e);
lcm_dcs_write_seq_static(ctx,0x24,0x20);
lcm_dcs_write_seq_static(ctx,0x25,0x00);
lcm_dcs_write_seq_static(ctx,0x90,0x00);
lcm_dcs_write_seq_static(ctx,0x91,0x00);
lcm_dcs_write_seq_static(ctx,0x10,0x01);
lcm_dcs_write_seq_static(ctx,0x20,0x31);
lcm_dcs_write_seq_static(ctx,0x21,0x0e);
lcm_dcs_write_seq_static(ctx,0x22,0x0e);
lcm_dcs_write_seq_static(ctx,0x23,0x0e);
lcm_dcs_write_seq_static(ctx,0x27,0x01);
lcm_dcs_write_seq_static(ctx,0x26,0x02);
lcm_dcs_write_seq_static(ctx,0x11,0x00);
lcm_dcs_write_seq_static(ctx,0x12,0x04);
lcm_dcs_write_seq_static(ctx,0x13,0x08);
lcm_dcs_write_seq_static(ctx,0x14,0x10);
lcm_dcs_write_seq_static(ctx,0x15,0x18);
lcm_dcs_write_seq_static(ctx,0x16,0x20);
lcm_dcs_write_seq_static(ctx,0x17,0x30);
lcm_dcs_write_seq_static(ctx,0x18,0x40);
lcm_dcs_write_seq_static(ctx,0x19,0x50);
lcm_dcs_write_seq_static(ctx,0x1a,0x60);
lcm_dcs_write_seq_static(ctx,0x1b,0x80);
lcm_dcs_write_seq_static(ctx,0x1c,0xa0);
lcm_dcs_write_seq_static(ctx,0x1d,0xc0);
lcm_dcs_write_seq_static(ctx,0x1e,0xe0);
lcm_dcs_write_seq_static(ctx,0x1f,0xff);
lcm_dcs_write_seq_static(ctx,0x28,0x00);
lcm_dcs_write_seq_static(ctx,0x29,0x04);
lcm_dcs_write_seq_static(ctx,0x2a,0x08);
lcm_dcs_write_seq_static(ctx,0x2b,0x10);
lcm_dcs_write_seq_static(ctx,0x2d,0x18);
lcm_dcs_write_seq_static(ctx,0x2f,0x20);
lcm_dcs_write_seq_static(ctx,0x30,0x30);
lcm_dcs_write_seq_static(ctx,0x31,0x40);
lcm_dcs_write_seq_static(ctx,0x32,0x50);
lcm_dcs_write_seq_static(ctx,0x33,0x60);
lcm_dcs_write_seq_static(ctx,0x34,0x80);
lcm_dcs_write_seq_static(ctx,0x35,0xa0);
lcm_dcs_write_seq_static(ctx,0x36,0xc0);
lcm_dcs_write_seq_static(ctx,0x37,0xe0);
lcm_dcs_write_seq_static(ctx,0x38,0xff);
lcm_dcs_write_seq_static(ctx,0xFE,0x82);
lcm_dcs_write_seq_static(ctx,0x00,0xd0);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x16,0x92);
lcm_dcs_write_seq_static(ctx,0xFE,0x00);
lcm_dcs_write_seq_static(ctx,0x77,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x1a,0x1a,0x1a,0x21,0x21,0x21,0x22,0x22,0x22,0x1c,0x1c,0x1c,0x27,0x27,0x27,0x38,0x38,0x38,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x1a,0x1a,0x1a,0x21,0x21,0x21,0x22,0x22,0x22,0x1c,0x1c,0x1c,0x27,0x27,0x27,0x38,0x38,0x38,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x1a,0x1a,0x1a,0x21,0x21,0x21,0x22,0x22,0x22,0x1c,0x1c,0x1c,0x27,0x27,0x27,0x38,0x38,0x38,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x0e,0x0e,0x0e,0x0d,0x0d,0x0d,0x1a,0x1a,0x1a,0x21,0x21);
lcm_dcs_write_seq_static(ctx,0x78,0x21,0x22,0x22,0x22,0x1c,0x1c,0x1c,0x27,0x27,0x27,0x38,0x38,0x38,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0x3f,0xff,0xff,0xff,0x24,0x24,0x24,0x13,0x13,0x13,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x12,0x12,0x12,0x0d,0x0d,0x0d,0x09,0x09,0x09,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xff,0xff,0xff,0x24,0x24,0x24,0x13,0x13,0x13,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x12,0x12,0x12,0x0d,0x0d,0x0d,0x09,0x09,0x09,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xff,0xff,0xff,0x24,0x24,0x24,0x13,0x13,0x13,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x12,0x12,0x12,0x0d,0x0d,0x0d,0x09);
lcm_dcs_write_seq_static(ctx,0x78,0x09,0x09,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xff,0xff,0xff,0x24,0x24,0x24,0x13,0x13,0x13,0x0f,0x0f,0x0f,0x0f,0x0f,0x0f,0x12,0x12,0x12,0x0d,0x0d,0x0d,0x09,0x09,0x09,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x01,0xb8,0x00,0x03,0x6f,0x00,0x06,0xde,0x00,0x0d,0xbb,0x00,0x0f,0xff,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x04,0x01,0x00,0x00,0x00,0x03,0x07,0x02,0x00,0x00,0x00,0x06,0x0c,0x04,0x00,0x00,0x00,0x07,0x0c,0x04,0x01,0x01,0x01,0x08,0x0c,0x05);
lcm_dcs_write_seq_static(ctx,0x78,0x02,0x02,0x02,0x0a,0x0e,0x07,0x05,0x09,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x00,0x01,0x00,0x03,0x02,0x02,0x01,0x02,0x01,0x03,0x03,0x03,0x01,0x02,0x01,0x04,0x04,0x04,0x02,0x03,0x02,0x05,0x06,0x05,0x03,0x04,0x03,0x06,0x08,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x03,0x04,0x02,0x02,0x02);
lcm_dcs_write_seq_static(ctx,0x78,0x01,0x05,0x05,0x04,0x03,0x03,0x02,0x06,0x07,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x02,0x00,0x01,0x00,0x01,0x02,0x02,0x00,0x01,0x01,0x02,0x03,0x03,0x01,0x02,0x01,0x02,0x04,0x04,0x01,0x02,0x02,0x03,0x05,0x05,0x02,0x03,0x02,0x04,0x06,0x06,0x03,0x04,0x03,0x05,0x07,0x07,0x04,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x03,0x02,0x00,0x02,0x01,0x02,0x04,0x03,0x01,0x02,0x01,0x03,0x05,0x04,0x02,0x03,0x02,0x04,0x06,0x05,0x03,0x04,0x02,0x06);
lcm_dcs_write_seq_static(ctx,0x78,0x08,0x06,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x02,0x04,0x02,0x01,0x02,0x01,0x03,0x05,0x02,0x02,0x03,0x01,0x04,0x06,0x03,0x03,0x04,0x02,0x06,0x07,0x04,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x04,0x01,0x00,0x00,0x00,0x03,0x07,0x02,0x00,0x00,0x00,0x06,0x0c,0x04,0x00,0x00,0x00,0x07,0x0c,0x04,0x01,0x01,0x01,0x08,0x0c,0x05,0x02,0x02,0x02,0x0a,0x0e,0x07,0x05,0x09,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x00,0x01,0x00,0x03,0x02,0x02,0x01,0x02,0x01,0x03,0x03,0x03,0x01,0x02,0x01,0x04,0x04,0x04,0x02,0x03,0x02,0x05,0x06,0x05,0x03,0x04,0x03,0x06,0x08,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x03,0x04,0x02,0x02,0x02,0x01,0x05,0x05,0x04,0x03,0x03,0x02,0x06,0x07,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x02,0x00,0x01,0x00,0x01,0x02,0x02,0x00,0x01,0x01,0x02,0x03,0x03,0x01,0x02,0x01,0x02,0x04,0x04,0x01,0x02,0x02,0x03,0x05,0x05,0x02,0x03,0x02,0x04,0x06,0x06,0x03,0x04,0x03,0x05,0x07,0x07,0x04,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x03,0x02,0x00,0x02,0x01,0x02,0x04,0x03,0x01,0x02,0x01,0x03,0x05,0x04,0x02,0x03,0x02,0x04,0x06,0x05,0x03,0x04,0x02,0x06,0x08,0x06,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01);
lcm_dcs_write_seq_static(ctx,0x78,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x02,0x04,0x02,0x01,0x02,0x01,0x03,0x05,0x02,0x02,0x03,0x01,0x04,0x06,0x03,0x03,0x04,0x02,0x06,0x07,0x04,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x04,0x01,0x00,0x00,0x00,0x03,0x07,0x02,0x00,0x00,0x00,0x06,0x0c,0x04,0x00,0x00,0x00,0x07,0x0c,0x04,0x01,0x01,0x01,0x08,0x0c,0x05,0x02,0x02,0x02,0x0a,0x0e,0x07,0x05,0x09,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x01,0x00,0x02,0x02,0x01);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x01,0x00,0x03,0x02,0x02,0x01,0x02,0x01,0x03,0x03,0x03,0x01,0x02,0x01,0x04,0x04,0x04,0x02,0x03,0x02,0x05,0x06,0x05,0x03,0x04,0x03,0x06,0x08,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x03,0x04,0x02,0x02,0x02,0x01,0x05,0x05,0x04,0x03,0x03,0x02,0x06,0x07,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x02,0x00,0x01,0x00,0x01,0x02,0x02,0x00,0x01,0x01,0x02,0x03,0x03,0x01,0x02);
lcm_dcs_write_seq_static(ctx,0x78,0x01,0x02,0x04,0x04,0x01,0x02,0x02,0x03,0x05,0x05,0x02,0x03,0x02,0x04,0x06,0x06,0x03,0x04,0x03,0x05,0x07,0x07,0x04,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x03,0x02,0x00,0x02,0x01,0x02,0x04,0x03,0x01,0x02,0x01,0x03,0x05,0x04,0x02,0x03,0x02,0x04,0x06,0x05,0x03,0x04,0x02,0x06,0x08,0x06,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x02,0x04,0x02,0x01,0x02,0x01,0x03);
lcm_dcs_write_seq_static(ctx,0x78,0x05,0x02,0x02,0x03,0x01,0x04,0x06,0x03,0x03,0x04,0x02,0x06,0x07,0x04,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x02,0x04,0x01,0x00,0x00,0x00,0x03,0x07,0x02,0x00,0x00,0x00,0x06,0x0c,0x04,0x00,0x00,0x00,0x07,0x0c,0x04,0x01,0x01,0x01,0x08,0x0c,0x05,0x02,0x02,0x02,0x0a,0x0e,0x07,0x05,0x09,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x00,0x01,0x00,0x03,0x02,0x02,0x01,0x02,0x01,0x03,0x03,0x03,0x01,0x02,0x01,0x04,0x04,0x04,0x02,0x03,0x02,0x05,0x06,0x05);
lcm_dcs_write_seq_static(ctx,0x78,0x03,0x04,0x03,0x06,0x08,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x02,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x03,0x04,0x02,0x02,0x02,0x01,0x05,0x05,0x04,0x03,0x03,0x02,0x06,0x07,0x06,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x02,0x00,0x01,0x00,0x01,0x02,0x02,0x00,0x01,0x01,0x02,0x03,0x03,0x01,0x02,0x01,0x02,0x04,0x04,0x01,0x02,0x02,0x03,0x05,0x05,0x02,0x03,0x02,0x04,0x06,0x06,0x03,0x04,0x03,0x05,0x07,0x07,0x04,0x05);
lcm_dcs_write_seq_static(ctx,0x78,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x00,0x01,0x00,0x01,0x03,0x02,0x00,0x02,0x01,0x02,0x04,0x03,0x01,0x02,0x01,0x03,0x05,0x04,0x02,0x03,0x02,0x04,0x06,0x05,0x03,0x04,0x02,0x06,0x08,0x06,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x01,0x01,0x00,0x00,0x00,0x01,0x02,0x01,0x01,0x01,0x00,0x02,0x03,0x02,0x01,0x01,0x00,0x02,0x04,0x02,0x01,0x02,0x01,0x03,0x05,0x02,0x02,0x03,0x01,0x04,0x06,0x03,0x03,0x04,0x02,0x06,0x07,0x04,0x04,0x05,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x16,0x00);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x16,0x93);
lcm_dcs_write_seq_static(ctx,0xFE,0x00);
lcm_dcs_write_seq_static(ctx,0x77,0x07,0x09,0x07,0x05,0x05,0x05,0x09,0x0b,0x08,0x06,0x06,0x06,0x0b,0x0d,0x09,0x07,0x08,0x07,0x0d,0x0f,0x0a,0x09,0x09,0x08,0x0f,0x11,0x0b,0x0a,0x0a,0x09,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff);
lcm_dcs_write_seq_static(ctx,0x78,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
lcm_dcs_write_seq_static(ctx,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff);
lcm_dcs_write_seq_static(ctx,0x78,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff);
lcm_dcs_write_seq_static(ctx,0x78,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff);
lcm_dcs_write_seq_static(ctx,0xFE,0x42);
lcm_dcs_write_seq_static(ctx,0x16,0x00);
lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	}
	lcm_dcs_write_seq_static(ctx,0xFE,0xD1);
	lcm_dcs_write_seq_static(ctx,0xB5,0xC8);
	lcm_dcs_write_seq_static(ctx,0xCE,0x02);

	lcm_dcs_write_seq_static(ctx,0xFE,0x98);
	lcm_dcs_write_seq_static(ctx,0x10,0xB2);//A2

	lcm_dcs_write_seq_static(ctx,0xFE,0x42);
        lcm_dcs_write_seq_static(ctx,0x44,0x04);
	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	lcm_dcs_write_seq_static(ctx,0x2F,0x00);  //60HZ
//	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
//	lcm_dcs_write_seq_static(ctx,0xFA,0x01);
	lcm_dcs_write_seq_static(ctx,0xC2,0x08);
	lcm_dcs_write_seq_static(ctx,0x35,0x00);
	lcm_dcs_write_seq_static(ctx,0x51,0x0D,0xBB);   //DBV
	lcm_pannel_reconfig_blk(ctx);
	lcm_dcs_write_seq_static(ctx,0x11,0x00);
	mdelay(120);
	lcm_dcs_write_seq_static(ctx,0x29,0x00);

	mdelay(10);

}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;
	pr_err("gezi----exit------%s-----%d\n",__func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	mdelay(20);
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(100);

	ctx->error = 0;
	ctx->prepared = false;
	is_system_resume = 0;
#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_disable();
	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);
#else
	pr_err("gezi------exit----%s-----%d\n",__func__,__LINE__);
	//reset
	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	udelay(3000);
	// 138   --  2.8
	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);

	udelay(3000);

	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);

	//137  -  1.2
	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 0);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(2000);

	if (tp_gesture_flag == 0) {
		pr_err("%s-%d-nt_board_id:%d\n",__func__,__LINE__,nt_board_id);
		lcm_pinctrl_select_suspend();
		lcm_tp_reset_disable();
		udelay(1200);
		ctx->pm_enable_gpio = lcm_vddi_136;
		if (IS_ERR(ctx->pm_enable_gpio)) {
			dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
        			__func__, PTR_ERR(ctx->pm_enable_gpio));
			return PTR_ERR(ctx->pm_enable_gpio);
		}
		gpiod_set_value(ctx->pm_enable_gpio, 0);
		udelay(500);
		if (nt_board_id > 2)
			lcm_tp_vdd_disable();
	}

#endif
	ctx->hbm_en = false;
	ctx->hbm_stat = false;
	need_brightness_sync = false;

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);
	is_system_resume = 1;

	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;

#if defined(CONFIG_MTK_PANEL_EXT)
	//mtk_panel_tch_rst(panel);
	pr_err("gezi----------%s-----%d\n",__func__,__LINE__);
#endif
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
	lcm_now_state = 0;
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}
	ctx->enabled = true;

	return 0;
}

#define VAC (2412)
#define HAC (1084)
static u32 fake_heigh = 2412;
static u32 fake_width = 1084;
static bool need_fake_resolution;

static const struct drm_display_mode switch_mode_120 = {
	.clock = ((FRAME_WIDTH+MODE_2_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_2_VFP+VSA+VBP)*(MODE_2_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_2_HFP,
	.hsync_end = FRAME_WIDTH + MODE_2_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_2_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_2_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_2_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_2_VFP + VSA + VBP,
};


static const struct drm_display_mode switch_mode_90 = {
	.clock = ((FRAME_WIDTH+MODE_1_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_1_VFP+VSA+VBP)*(MODE_1_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_1_HFP,
	.hsync_end = FRAME_WIDTH + MODE_1_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_1_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_1_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_1_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_1_VFP + VSA + VBP,
};

static const struct drm_display_mode default_mode = {
	.clock = ((FRAME_WIDTH+MODE_0_HFP+HSA+HBP)*(FRAME_HEIGHT+MODE_0_VFP+VSA+VBP)*(MODE_0_FPS)/1000),
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + MODE_0_HFP,
	.hsync_end = FRAME_WIDTH + MODE_0_HFP + HSA,
	.htotal = FRAME_WIDTH + MODE_0_HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + MODE_0_VFP,
	.vsync_end = FRAME_HEIGHT + MODE_0_VFP + VSA,
	.vtotal = FRAME_HEIGHT + MODE_0_VFP + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)
static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	//struct gpio_desc *id2_gpio = NULL;
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	unsigned char data[3] = {0x00, 0x00, 0x00};
	//unsigned char id[3] = {0x0, 0x80, 0x0};
	ssize_t ret;

	//lcm_dcs_write_seq_static(ctx,0xFE,0xC2);
	ret = mipi_dsi_dcs_read(dsi, 0x04, data, 3);
	if (ret < 0) {
		pr_err("%s error\n", __func__);
		return 0;
	}

	printk("panel_ata_check-ATA read 0x04 data %x %x %x\n", data[0], data[1], data[2]);

	if (data[1] == 0x80) {
		return 1;
	}

	return 0;

}

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb,
	void *handle, unsigned int level)
{
	char bl_tb0[] = {0x51,0x07,0xFF};
	unsigned int reg_level = 125;
	char set_page00_tb[] = {0xfe, 0x00};
	char esd_page_tb[] = {0xfe,0xed};
	pr_err("[rm692h5]%s level=%d\n", __func__, level);

	if(level){
		reg_level = level;
		bl_level = level;
	} else {
		reg_level = 0;
	}

	if (g_ctx->hbm_stat == true) {
		return 0;
	}


	if (lcm_now_state) {
		if (level >= 1200) {
			bl_tb0[1] = 0x0f;
			bl_tb0[2] = 0xff;
		} else if (level >= 800 && level < 1200) {
			bl_tb0[1] = 0x0b;
			bl_tb0[2] = 0x5f;
		} else {
			bl_tb0[1] = 0x00;
			bl_tb0[2] = 0x03;
		}
		cb(dsi, handle, set_page00_tb, ARRAY_SIZE(set_page00_tb));
		cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

		return 0;
	}

	if (reg_level != 0 && reg_level < 33) {
		reg_level = 33;
	}

	bl_tb0[1] = (reg_level>>8)&0xf;
	bl_tb0[2] = (reg_level)&0xff;

	if (!cb)
		return -1;

	if(reading_base)
		cb(dsi, handle, set_page00_tb, ARRAY_SIZE(set_page00_tb));
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	if(reading_base)
		cb(dsi, handle, esd_page_tb, ARRAY_SIZE(esd_page_tb));

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	// unsigned int level_hbm = 255;
	unsigned int level_normal = 125;
	char normal_tb0[] = {0x51, 0x07,0xFF};
	char hbm_tb[] = {0x51,0x0F,0xF0};
	char set_page00_tb[] = {0xfe, 0x00};
	char esd_page_tb[] = {0xfe,0xed};
	struct lcm *ctx = panel_to_lcm(panel);

	if (!cb)
		return -1;

	//if (ctx->hbm_en == en)
	//	goto done;

	if (en)
	{
		if (lcm_now_state) {
			lcm_change_state(dsi, cb, handle, 0);
		}

		pr_err("[panel] %s : set HBM\n",__func__);
		g_ctx->hbm_stat = true;
		if(reading_base)
			cb(dsi, handle, set_page00_tb, ARRAY_SIZE(set_page00_tb));
		cb(dsi, handle, hbm_tb, ARRAY_SIZE(hbm_tb));
		if(reading_base)
			cb(dsi, handle, esd_page_tb, ARRAY_SIZE(esd_page_tb));
	}
	else
	{
		pr_err("[panel] %s : set normal = %d\n",__func__,bl_level);
		level_normal = bl_level;
		normal_tb0[1] = (level_normal>>8)&0xff;
		normal_tb0[2] = (level_normal)&0xff;

		g_ctx->hbm_stat = false;

		if(reading_base)
			cb(dsi, handle, set_page00_tb, ARRAY_SIZE(set_page00_tb));
		cb(dsi, handle, normal_tb0, ARRAY_SIZE(normal_tb0));
		if(reading_base)
			cb(dsi, handle, esd_page_tb, ARRAY_SIZE(esd_page_tb));

		if (lcm_now_state) {
			lcm_change_state(dsi, cb, handle, 1);
		}
	}

	ctx->hbm_en = en;
	ctx->hbm_wait = true;

 // done:
	return 0;
}

static void panel_hbm_get_state(struct drm_panel *panel, bool *state)
{
	struct lcm *ctx = panel_to_lcm(panel);

	*state = ctx->hbm_en;
}

static int lcm_get_virtual_heigh(void)
{
	return VAC;
}

static int lcm_get_virtual_width(void)
{
	return HAC;
}

static struct mtk_panel_params ext_params_120 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x75,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x77,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lp_perline_en = 1,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			// .bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,

	.dyn_fps = {
		.switch_en = 1,
	},
	.data_rate = MODE_2_DATA_RATE,
	// .bdg_ssc_disable = 1,
	// .bdg_ssc_enable = 0,
	// .ssc_disable = 1,
	.ssc_enable = 0,
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_2_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.wait_before_hbm = 1,
};

static struct mtk_panel_params ext_params_90 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x75,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x77,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lp_perline_en = 1,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			// .bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,

	.dyn_fps = {
		.switch_en = 1,
	},
	.data_rate = MODE_2_DATA_RATE,
	// .bdg_ssc_disable = 1,
	// .bdg_ssc_enable = 0,
	// .ssc_disable = 1,
	.ssc_enable = 0,
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_1_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.wait_before_hbm = 1,
};


static struct mtk_panel_params ext_params = {
	// .pll_clk = 373,
	// .vfp_low_power = 743,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x75,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x77,
		.count = 1,
		.para_list[0] = 0x00,
	},
	.lp_perline_en = 1,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.physical_width_um = PHYSICAL_WIDTH,
	.physical_height_um = PHYSICAL_HEIGHT,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			// .bdg_dsc_enable = 0,
			.ver                   =  DSC_VER,
			.slice_mode            =  DSC_SLICE_MODE,
			.rgb_swap              =  DSC_RGB_SWAP,
			.dsc_cfg               =  DSC_DSC_CFG,
			.rct_on                =  DSC_RCT_ON,
			.bit_per_channel       =  DSC_BIT_PER_CHANNEL,
			.dsc_line_buf_depth    =  DSC_DSC_LINE_BUF_DEPTH,
			.bp_enable             =  DSC_BP_ENABLE,
			.bit_per_pixel         =  DSC_BIT_PER_PIXEL,
			.pic_height            =  FRAME_HEIGHT,
			.pic_width             =  FRAME_WIDTH,
			.slice_height          =  DSC_SLICE_HEIGHT,
			.slice_width           =  DSC_SLICE_WIDTH,
			.chunk_size            =  DSC_CHUNK_SIZE,
			.xmit_delay            =  DSC_XMIT_DELAY,
			.dec_delay             =  DSC_DEC_DELAY,
			.scale_value           =  DSC_SCALE_VALUE,
			.increment_interval    =  DSC_INCREMENT_INTERVAL,
			.decrement_interval    =  DSC_DECREMENT_INTERVAL,
			.line_bpg_offset       =  DSC_LINE_BPG_OFFSET,
			.nfl_bpg_offset        =  DSC_NFL_BPG_OFFSET,
			.slice_bpg_offset      =  DSC_SLICE_BPG_OFFSET,
			.initial_offset        =  DSC_INITIAL_OFFSET,
			.final_offset          =  DSC_FINAL_OFFSET,
			.flatness_minqp        =  DSC_FLATNESS_MINQP,
			.flatness_maxqp        =  DSC_FLATNESS_MAXQP,
			.rc_model_size         =  DSC_RC_MODEL_SIZE,
			.rc_edge_factor        =  DSC_RC_EDGE_FACTOR,
			.rc_quant_incr_limit0  =  DSC_RC_QUANT_INCR_LIMIT0,
			.rc_quant_incr_limit1  =  DSC_RC_QUANT_INCR_LIMIT1,
			.rc_tgt_offset_hi      =  DSC_RC_TGT_OFFSET_HI,
			.rc_tgt_offset_lo      =  DSC_RC_TGT_OFFSET_LO,
		},
//	.wait_sof_before_dec_vfp = 1,
	.data_rate = MODE_2_DATA_RATE,
	// .bdg_ssc_disable = 1,
	// .bdg_ssc_enable = 0,
	// .ssc_disable = 1,
	.ssc_enable = 0,
	.dyn_fps = {
		.switch_en = 1,
	},
	.dyn = {
		.switch_en = 1,
		.data_rate = MODE_0_DATA_RATE,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 1,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.wait_before_hbm = 1,
};

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m,  &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			 struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);
	// printk("[panel] %s,vrefresh = %d\n",__func__,m->vrefresh);
	if (drm_mode_vrefresh(m) == MODE_0_FPS)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == MODE_1_FPS)
		ext->params = &ext_params_90;
	else if (drm_mode_vrefresh(m) == MODE_2_FPS)
		ext->params = &ext_params_120;
	else
		ret = 1;

	return ret;
}

static void mode_switch_to_120(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_err("[panel] %s\n",__func__);
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0x2F,0x05);
		if(reading_base)
			lcm_dcs_write_seq_static(ctx,0xFE,0xed);
		else
			lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	}
}

static void mode_switch_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_err("[panel] %s\n",__func__);
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0x2F,0x06);
		if(reading_base)
			lcm_dcs_write_seq_static(ctx,0xFE,0xed);
		else
			lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	}
}

static void mode_switch_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_err("[panel] %s\n",__func__);
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0x2F,0x00);
		if(reading_base)
			lcm_dcs_write_seq_static(ctx,0xFE,0xed);
		else
			lcm_dcs_write_seq_static(ctx,0xFE,0x00);

	}
}

static int mode_switch(struct drm_panel *panel,
		struct drm_connector *connector, unsigned int cur_mode,
		unsigned int dst_mode, enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	pr_err("[panel] %s,cur_mode = %d,dst_mode = %d\n",__func__,cur_mode,dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (drm_mode_vrefresh(m) == MODE_0_FPS) { /*switch to 60 */
		mode_switch_to_60(panel, stage);
	} else if (drm_mode_vrefresh(m) == MODE_1_FPS) { /*switch to 90 */
		mode_switch_to_90(panel, stage);
	} else if (drm_mode_vrefresh(m) == MODE_2_FPS) { /*switch to 120 */
		mode_switch_to_120(panel, stage);
	} else
		ret = 1;

	g_vrefresh = drm_mode_vrefresh(m);
	return ret;
}

extern void mtk_switch_esd_aod(bool enable);
static int panel_doze_enable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);
	lcm_now_state = 1;

	if(!sku_is_ind)
		mtk_switch_esd_aod(false);

	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x39);

	if (bl_level >= 1200) {
		lcm_dcs_write_seq_static(ctx, 0x51, 0x0F, 0xFF);
	} else if (bl_level >= 800 && bl_level < 1200) {
		lcm_dcs_write_seq_static(ctx, 0x51, 0x0b, 0x5f);
	} else {
		lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x03);
	}

	g_ctx->hbm_stat = false;
	g_ctx->hbm_en = false;
	need_brightness_sync = false;

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	struct lcm *ctx = panel_to_lcm(panel);
	char bl_tb[2] = {0x00,0x00};

	pr_info("panel %s\n", __func__);
	lcm_now_state = 0;
	bl_tb[0] = (bl_level>>8)&0xf;
	bl_tb[1] = (bl_level)&0xff;

	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x38);
	if(g_ctx->hbm_stat == false)
		lcm_dcs_write_seq(ctx, 0x51, bl_tb[0], bl_tb[1]);
	lcm_dcs_write_seq_static(ctx, 0xFE, 0x00);

	if(!sku_is_ind)
		mtk_switch_esd_aod(true);

	return 0;
}

static int panel_set_aod_light_mode(void *dsi,
	dcs_write_gce cb, void *handle, unsigned int mode)
{
	// int i = 0;

	pr_info("panel %s\n", __func__);

	return 0;
}


static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.ata_check = panel_ata_check,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.hbm_get_state = panel_hbm_get_state,
	.get_virtual_heigh = lcm_get_virtual_heigh,
	.get_virtual_width = lcm_get_virtual_width,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch = mode_switch,
	/*aod mode*/
	.doze_enable = panel_doze_enable,
	//.doze_enable_start = panel_doze_enable_start,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};
/*
static void change_drm_disp_mode_params(struct drm_display_mode *mode)
{
	if (fake_heigh > 0 && fake_heigh < VAC) {
		mode->vdisplay = fake_heigh;
		mode->vsync_start = fake_heigh + VFP;
		mode->vsync_end = fake_heigh + VFP + VSA;
		mode->vtotal = fake_heigh + VFP + VSA + VBP;
	}
	if (fake_width > 0 && fake_width < HAC) {
		mode->hdisplay = fake_width;
		mode->hsync_start = fake_width + HFP;
		mode->hsync_end = fake_width + HFP + HSA;
		mode->htotal = fake_width + HFP + HSA + HBP;
	}
}
*/
static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode_1;
	struct drm_display_mode *mode_2;

	//if (need_fake_resolution)
	//	change_drm_disp_mode_params(&default_mode);

	mode = drm_mode_duplicate(connector->dev, &switch_mode_120);
	if (!mode) {
		dev_err(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			switch_mode_120.hdisplay, switch_mode_120.vdisplay,
			drm_mode_vrefresh(&switch_mode_120));
		return -ENOMEM;
	}
	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER ;
	drm_mode_probed_add(connector, mode);
	pr_err("[panel] %s,333\n",__func__);


	mode_1 = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode_1) {
		dev_err(connector->dev->dev, "failed to add mode_1 %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}
	drm_mode_set_name(mode_1);
	mode_1->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode_1);
	pr_err("[panel] %s,111\n",__func__);

	mode_2 = drm_mode_duplicate(connector->dev, &switch_mode_90);
	if (!mode_2) {
		dev_err(connector->dev->dev, "failed to add mode_2 %ux%ux@%u\n",
			switch_mode_90.hdisplay, switch_mode_90.vdisplay,
			drm_mode_vrefresh(&switch_mode_90));
		return -ENOMEM;
	}
	drm_mode_set_name(mode_2);
	mode_2->type = DRM_MODE_TYPE_DRIVER ;
	drm_mode_probed_add(connector, mode_2);
	pr_err("[panel] %s,222\n",__func__);



	connector->display_info.width_mm = 64;
	connector->display_info.height_mm = 129;

	return 3;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static void check_is_need_fake_resolution(struct device *dev)
{
	unsigned int ret = 0;

	ret = of_property_read_u32(dev->of_node, "fake_heigh", &fake_heigh);
	if (ret)
		need_fake_resolution = false;
	ret = of_property_read_u32(dev->of_node, "fake_width", &fake_width);
	if (ret)
		need_fake_resolution = false;
	if (fake_heigh > 0 && fake_heigh < VAC)
		need_fake_resolution = true;
	if (fake_width > 0 && fake_width < HAC)
		need_fake_resolution = true;
	pr_err("%s------need_fake_resolution = %d------%d\n", __func__,need_fake_resolution,__LINE__);
}

static ssize_t displayid_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
	int count = 0;
	unsigned int display_id = 0;

	display_id = get_lcm_id();
	printk("[%s] display_id:0x%06x!!!\n",__func__, display_id);
	count = sprintf(buf, "%06x\n",display_id);

	return count;
}

static ssize_t displayid_store(struct device *dev,
           struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t brightnessid_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
	int count = 0;
	unsigned int display_bl = 0;

	display_bl = get_lcm_bl_val();
	printk("[%s] display_bl:%d!!!\n",__func__, display_bl);
	count = sprintf(buf, "%d\n",display_bl);

	return count;
}

static ssize_t brightnessid_store(struct device *dev,
           struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned int state;
	unsigned char val1, val2;

	ret = kstrtouint(buf, 10, &state);
	if (ret < 0) {
		goto err;
	}
	printk("[%s] brightness level:%d\n", __func__, state);
	val1 = (state>>8)&0xf;
	val2 = (state)&0xff;
	lcm_set_bk(val1, val2);

err:
	return size;
}

static ssize_t boeesd_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
	int count = 0;
	unsigned char val1 = 0;
	unsigned char val2 = 0;

	lcm_set_page(6, 0xFE, 0xED);  //FE-ED
	val1 =  get_reg_val(1, 0x75);

	val2 =  get_reg_val(1, 0x77);



	count = sprintf(buf, "75-0x%x|77-0x%x\n",val1,val2);
	lcm_set_page(6, 0xFE, 0x00);
	return count;
}

static ssize_t boeesd_store(struct device *dev,
           struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}
static ssize_t hbm_mode_show(struct kobject* kodjs,struct kobj_attribute *attr,char *buf)
{
	int count = 0;
	count = sprintf(buf, "hbm state: %d\n",g_ctx->hbm_stat);
	return count;
}

static ssize_t hbm_mode_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	int ret;
	unsigned int state;
	unsigned char val1, val2;
	static unsigned int display_bl_now = 0;

	ret = kstrtouint(buf, 10, &state);
	if (ret < 0) {
		goto err;
	}
	printk("[%s]  hbm state:%d\n", __func__, state);

	if (state) {
		if (g_ctx->hbm_stat)
			goto err;
		display_bl_now = get_lcm_bl_val();
		g_ctx->hbm_stat = true;
		lcm_set_bk(0x0F, 0xF0);
	} else {
		if (!g_ctx->hbm_stat)
			goto err;
		g_ctx->hbm_stat = false;
		val1 = (display_bl_now>>8)&0xf;
		val2 = (display_bl_now)&0xff;
		lcm_set_bk(val1, val2);
	}

err:
	return count;
}

static ssize_t brightness_sync_show(struct kobject* kodjs,struct kobj_attribute *attr,char *buf)
{
	return sprintf(buf, "need brightness sync: %d\n", need_brightness_sync);
}

static ssize_t brightness_sync_store(struct kobject *kobj, struct kobj_attribute *attr,const char *buf, size_t count)
{
	int value = 0;
	if (0 > kstrtouint(buf, 10, &value)) {
		goto err;
	}
	printk("set brightness sync:%d\n", value);
	need_brightness_sync = value == 1;
err:
	return count;
}

static DEVICE_ATTR(displayid, 0664, displayid_show, displayid_store);
static DEVICE_ATTR(brightnessid, 0664, brightnessid_show, brightnessid_store);
static DEVICE_ATTR(boeesd, 0664, boeesd_show, boeesd_store);
static struct kobj_attribute hbm_mode_attr = __ATTR(hbm_mode, 0664, hbm_mode_show, hbm_mode_store);
static struct kobj_attribute brightness_sync_attr = __ATTR(brightness_sync, 0664, brightness_sync_show, brightness_sync_store);

static struct attribute *displayid_attributes[] = {
	&dev_attr_displayid.attr,
	&dev_attr_brightnessid.attr,
	&dev_attr_boeesd.attr,
	NULL
};

static struct attribute_group displayid_attribute_group = {
	.attrs = displayid_attributes,
};

static const struct of_device_id displayid_of_match[] = {
	{.compatible = "mediatek,display_id",},
	{},
};
MODULE_DEVICE_TABLE(of, displayid_of_match);

static int displayid_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = sysfs_create_group(&pdev->dev.kobj, &displayid_attribute_group);
	if (ret < 0) {
		printk("[%s] sysfs_create_group failed\n",__func__);
	}
	printk("%s is OK!!!\n",__func__);

	return 0;
}

static struct platform_driver displayid_driver = {
	.probe = displayid_probe,
	.driver = {
		   .name = "displayid",
		   .of_match_table = displayid_of_match,
	},
};

int displayid_node_init(void)
{
	return platform_driver_register(&displayid_driver);
}

int sys_node_init(void)
{
	int ret = 0;

	kobj = kobject_create_and_add("panel_feature", NULL);
	if (kobj == NULL) {
		return -ENOMEM;
	}

	ret = sysfs_create_file(kobj, &hbm_mode_attr.attr);
	if (ret < 0) {
		printk("[%s] sysfs_create_group failed\n",__func__);
		return -1;
	}

	ret = sysfs_create_file(kobj, &brightness_sync_attr.attr);
	if (ret < 0) {
		printk("[%s] sysfs_create_group failed\n",__func__);
		return -1;
	}

	printk("[%s] is OK!!!\n", __func__);
	return 0;
}



void lcm_parse_sku(void)
{
	struct device_node *of_chosen;
	char *bootargs;

	of_chosen = of_find_node_by_path("/chosen");
	if (!of_chosen)
		goto done;

	bootargs = (char *)of_get_property(of_chosen,
			"bootargs", NULL);
	if (!bootargs)
		goto done;

	if (strstr(bootargs, SKU_IND)){
		sku_is_ind = true;
		printk("liumiao esd not enable\n");
	}
	else {
		sku_is_ind = false;
		printk("liumiao esd  enable\n");
	}
done:
	printk("liumiao esd  end\n");

}
/*liumiao add for cat sku end*/

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;

	pr_err("gezi ---------%d-----\n",__LINE__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_err("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_err("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_err("gezi ---- %s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	// dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
	// 		 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET
	// 		 | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	// dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;
	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	pr_err("gezi ---------%d-----\n",__LINE__);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_err(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(dev, "%s: cannot get bias-pos 0 %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(dev, "%s: cannot get bias-neg 1 %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}

	pr_err("gezi ---------%d-----\n",__LINE__);

	devm_gpiod_put(dev, ctx->bias_neg);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &lcm_drm_funcs;

	drm_panel_add(&ctx->panel);
	// ret = drm_panel_add(&ctx->panel);
	// if (ret < 0)
		// return ret;

	pr_err("gezi ---------%d-----\n",__LINE__);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	//mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif
	check_is_need_fake_resolution(dev);
	pr_err("%s------------%d\n", __func__,__LINE__);

	displayid_node_init();
	// lcm_panel_init(ctx);
	/*liumiao add for get sku*/
	lcm_parse_sku();

	g_ctx = ctx;
	ctx->hbm_en = false;
	g_ctx->hbm_stat = false;
	sys_node_init();
	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	lcm_pinctrl_select_release();
	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "Boe,rm692h5,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver rm692h5_boe_lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-rm692h5-boe-cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

static int __init rm692h5_boe_lcm_driver_init(void)
{
	int ret = 0;

	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	ret = mipi_dsi_driver_register(&rm692h5_boe_lcm_driver);
	if (ret < 0)
		pr_notice("%s, Failed to register rm692h5_boe_lcm_driver: %d\n",
			__func__, ret);

	mtk_panel_unlock();
	pr_notice("%s- ret:%d\n", __func__, ret);
	return 0;
}

static void __exit rm692h5_boe_lcm_driver_exit(void)
{
	pr_notice("%s+\n", __func__);
	mtk_panel_lock();
	mipi_dsi_driver_unregister(&rm692h5_boe_lcm_driver);
	mtk_panel_unlock();
	pr_notice("%s-\n", __func__);
}
module_init(rm692h5_boe_lcm_driver_init);
module_exit(rm692h5_boe_lcm_driver_exit);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("rm692h5 BOE CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
