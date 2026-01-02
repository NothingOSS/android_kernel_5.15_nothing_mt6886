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
#include "include/panel-novatek-rm692e5-cmd.h"
/* i2c control start */

#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/hardware_info/hardware_info.h"
#endif

#define BLK_LEVEL_OFFSET			(0)
#define BLK_LEVEL_MAP3				(3515)
static unsigned int Gamma_to_level[] = {
0     ,
1     ,
1     ,
1     ,
2     ,
2     ,
2     ,
3     ,
4     ,
5     ,
6     ,
7     ,
9     ,
10    ,
12    ,
13    ,
15    ,
17    ,
19    ,
21    ,
23    ,
26    ,
28    ,
31    ,
33    ,
36    ,
39    ,
42    ,
45    ,
48    ,
51    ,
55    ,
58    ,
62    ,
66    ,
70    ,
74    ,
78    ,
82    ,
86    ,
90    ,
95    ,
99    ,
104   ,
109   ,
114   ,
119   ,
124   ,
129   ,
135   ,
140   ,
146   ,
151   ,
157   ,
163   ,
169   ,
175   ,
181   ,
188   ,
194   ,
201   ,
207   ,
214   ,
221   ,
228   ,
235   ,
242   ,
249   ,
257   ,
264   ,
272   ,
280   ,
288   ,
296   ,
304   ,
312   ,
320   ,
328   ,
337   ,
345   ,
354   ,
363   ,
372   ,
381   ,
390   ,
399   ,
409   ,
418   ,
428   ,
437   ,
447   ,
457   ,
467   ,
477   ,
487   ,
498   ,
508   ,
519   ,
529   ,
540   ,
551   ,
562   ,
573   ,
584   ,
595   ,
607   ,
618   ,
630   ,
642   ,
654   ,
666   ,
678   ,
690   ,
702   ,
714   ,
727   ,
739   ,
752   ,
765   ,
778   ,
791   ,
804   ,
817   ,
831   ,
844   ,
858   ,
871   ,
885   ,
899   ,
913   ,
927   ,
941   ,
956   ,
970   ,
985   ,
999   ,
1014  ,
1024  ,
1044  ,
1059  ,
1074  ,
1089  ,
1105  ,
1120  ,
1136  ,
1152  ,
1168  ,
1184  ,
1200  ,
1216  ,
1232  ,
1248  ,
1265  ,
1281  ,
1298  ,
1315  ,
1332  ,
1349  ,
1366  ,
1383  ,
1401  ,
1418  ,
1436  ,
1453  ,
1471  ,
1489  ,
1507  ,
1525  ,
1543  ,
1562  ,
1580  ,
1599  ,
1617  ,
1636  ,
1655  ,
1674  ,
1693  ,
1712  ,
1732  ,
1751  ,
1770  ,
1790  ,
1810  ,
1830  ,
1850  ,
1870  ,
1890  ,
1910  ,
1930  ,
1951  ,
1972  ,
1992  ,
2013  ,
2034  ,
2055  ,
2076  ,
2097  ,
2119  ,
2140  ,
2162  ,
2183  ,
2205  ,
2227  ,
2249  ,
2271  ,
2293  ,
2316  ,
2338  ,
2361  ,
2383  ,
2406  ,
2429  ,
2452  ,
2475  ,
2498  ,
2522  ,
2545  ,
2568  ,
2592  ,
2616  ,
2640  ,
2664  ,
2688  ,
2712  ,
2736  ,
2760  ,
2785  ,
2810  ,
2834  ,
2859  ,
2884  ,
2909  ,
2934  ,
2959  ,
2985  ,
3010  ,
3036  ,
3061  ,
3087  ,
3113  ,
3139  ,
3165  ,
3191  ,
3218  ,
3244  ,
3271  ,
3297  ,
3324  ,
3351  ,
3378  ,
3405  ,
3432  ,
3460  ,
3487  ,
3515  ,
3515
};

unsigned int bl_level;
struct lcm *g_ctx;
int g_vrefresh = -1;

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
	bool hbm_stat;           //0Î´ÔÚHBM  1ÔÚHBM

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
		reg_level = Gamma_to_level[bl_level] + BLK_LEVEL_OFFSET;
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

//R
//R
	lcm_dcs_write_seq_static(ctx,0xFE,0x40);
	lcm_dcs_write_seq_static(ctx,0xBD,0x00);
	lcm_dcs_write_seq_static(ctx,0xFE,0xA1);
	lcm_dcs_write_seq_static(ctx,0xCD,0x6B); 
	lcm_dcs_write_seq_static(ctx,0xCE,0xBB); 
	lcm_dcs_write_seq_static(ctx,0xFE,0XD1);
	lcm_dcs_write_seq_static(ctx,0XB4,0x01);
	lcm_dcs_write_seq_static(ctx,0xFE,0x38);
	lcm_dcs_write_seq_static(ctx,0x17,0x0F);
	lcm_dcs_write_seq_static(ctx,0x18,0x0F);

lcm_dcs_write_seq_static(ctx,0xFE,0xD2); // switch to D2 page
lcm_dcs_write_seq_static(ctx,0x50,0x11); // pps000
lcm_dcs_write_seq_static(ctx,0x51,0x8b); // pps003
lcm_dcs_write_seq_static(ctx,0x52,0x30); // pps004
lcm_dcs_write_seq_static(ctx,0x53,0x09); // pps006
lcm_dcs_write_seq_static(ctx,0x54,0x60); // pps007
lcm_dcs_write_seq_static(ctx,0x55,0x04); // pps008
lcm_dcs_write_seq_static(ctx,0x56,0x38); // pps009
lcm_dcs_write_seq_static(ctx,0x58,0x00); // pps010
lcm_dcs_write_seq_static(ctx,0x59,0x14); // pps011
lcm_dcs_write_seq_static(ctx,0x5a,0x02); // pps012
lcm_dcs_write_seq_static(ctx,0x5b,0x1c); // pps013
lcm_dcs_write_seq_static(ctx,0x5c,0x02); // pps016
lcm_dcs_write_seq_static(ctx,0x5d,0x00); // pps017
lcm_dcs_write_seq_static(ctx,0x5e,0x20); // pps021
lcm_dcs_write_seq_static(ctx,0x5f,0x01); // pps022
lcm_dcs_write_seq_static(ctx,0x60,0xe8); // pps023
lcm_dcs_write_seq_static(ctx,0x61,0x00); // pps024
lcm_dcs_write_seq_static(ctx,0x62,0x07); // pps025
lcm_dcs_write_seq_static(ctx,0x63,0x0c); // pps027
lcm_dcs_write_seq_static(ctx,0x64,0x05); // pps028
lcm_dcs_write_seq_static(ctx,0x65,0x0e); // pps029
lcm_dcs_write_seq_static(ctx,0x66,0x05); // pps030
lcm_dcs_write_seq_static(ctx,0x67,0x16); // pps031
lcm_dcs_write_seq_static(ctx,0x68,0x18); // pps032
lcm_dcs_write_seq_static(ctx,0x69,0x00); // pps033
lcm_dcs_write_seq_static(ctx,0x6a,0x10); // pps034
lcm_dcs_write_seq_static(ctx,0x6b,0xf0); // pps035
lcm_dcs_write_seq_static(ctx,0x6c,0x07); // pps036
lcm_dcs_write_seq_static(ctx,0x6d,0x10); // pps037
lcm_dcs_write_seq_static(ctx,0x6e,0x20); // pps038
lcm_dcs_write_seq_static(ctx,0x6f,0x00); // pps039
lcm_dcs_write_seq_static(ctx,0x70,0x06); // pps040
lcm_dcs_write_seq_static(ctx,0x71,0x0f); // pps041
lcm_dcs_write_seq_static(ctx,0x72,0x0f); // pps042
lcm_dcs_write_seq_static(ctx,0x73,0x33); // pps043
lcm_dcs_write_seq_static(ctx,0x74,0x0e); // pps044
lcm_dcs_write_seq_static(ctx,0x75,0x1c); // pps045
lcm_dcs_write_seq_static(ctx,0x76,0x2a); // pps046
lcm_dcs_write_seq_static(ctx,0x77,0x38); // pps047
lcm_dcs_write_seq_static(ctx,0x78,0x46); // pps048
lcm_dcs_write_seq_static(ctx,0x79,0x54); // pps049
lcm_dcs_write_seq_static(ctx,0x7a,0x62); // pps050
lcm_dcs_write_seq_static(ctx,0x7b,0x69); // pps051
lcm_dcs_write_seq_static(ctx,0x7c,0x70); // pps052
lcm_dcs_write_seq_static(ctx,0x7d,0x77); // pps053
lcm_dcs_write_seq_static(ctx,0x7e,0x79); // pps054
lcm_dcs_write_seq_static(ctx,0x7f,0x7b); // pps055
lcm_dcs_write_seq_static(ctx,0x80,0x7d); // pps056
lcm_dcs_write_seq_static(ctx,0x81,0x7e); // pps057
lcm_dcs_write_seq_static(ctx,0x82,0x02); // pps058
lcm_dcs_write_seq_static(ctx,0x83,0x02); // pps059
lcm_dcs_write_seq_static(ctx,0x84,0x22); // pps060
lcm_dcs_write_seq_static(ctx,0x85,0x00); // pps061
lcm_dcs_write_seq_static(ctx,0x86,0x2a); // pps062
lcm_dcs_write_seq_static(ctx,0x87,0x40); // pps063
lcm_dcs_write_seq_static(ctx,0x88,0x2a); // pps064
lcm_dcs_write_seq_static(ctx,0x89,0xbe); // pps065
lcm_dcs_write_seq_static(ctx,0x8a,0x3a); // pps066
lcm_dcs_write_seq_static(ctx,0x8b,0xfc); // pps067
lcm_dcs_write_seq_static(ctx,0x8c,0x3a); // pps068
lcm_dcs_write_seq_static(ctx,0x8d,0xfa); // pps069
lcm_dcs_write_seq_static(ctx,0x8e,0x3a); // pps070
lcm_dcs_write_seq_static(ctx,0x8f,0xf8); // pps071
lcm_dcs_write_seq_static(ctx,0x90,0x3b); // pps072
lcm_dcs_write_seq_static(ctx,0x91,0x38); // pps073
lcm_dcs_write_seq_static(ctx,0x92,0x3b); // pps074
lcm_dcs_write_seq_static(ctx,0x93,0x78); // pps075
lcm_dcs_write_seq_static(ctx,0x94,0x3b); // pps076
lcm_dcs_write_seq_static(ctx,0x95,0xb6); // pps077
lcm_dcs_write_seq_static(ctx,0x96,0x4b); // pps078
lcm_dcs_write_seq_static(ctx,0x97,0xf6); // pps079
lcm_dcs_write_seq_static(ctx,0x98,0x4c); // pps080
lcm_dcs_write_seq_static(ctx,0x99,0x34); // pps081
lcm_dcs_write_seq_static(ctx,0x9a,0x4c); // pps082
lcm_dcs_write_seq_static(ctx,0x9b,0x74); // pps083
lcm_dcs_write_seq_static(ctx,0x9c,0x5c); // pps084
lcm_dcs_write_seq_static(ctx,0x9d,0x74); // pps085
lcm_dcs_write_seq_static(ctx,0x9e,0x8c); // pps086
lcm_dcs_write_seq_static(ctx,0x9f,0xf4); // pps087
lcm_dcs_write_seq_static(ctx,0xa2,0x02); // pps014
lcm_dcs_write_seq_static(ctx,0xa3,0x1c); // pps015
lcm_dcs_write_seq_static(ctx,0xa4,0x00); // pps088
lcm_dcs_write_seq_static(ctx,0xa5,0x00); // pps089
lcm_dcs_write_seq_static(ctx,0xa6,0x00); // pps090
lcm_dcs_write_seq_static(ctx,0xa7,0x00); // pps091
lcm_dcs_write_seq_static(ctx,0xa9,0x00); // pps092
lcm_dcs_write_seq_static(ctx,0xaa,0x00); // pps093
lcm_dcs_write_seq_static(ctx,0xa0,0x80); // pps005

	lcm_dcs_write_seq_static(ctx,0xFE,0x00);
	lcm_dcs_write_seq_static(ctx,0xFA,0x01);
	lcm_dcs_write_seq_static(ctx,0xC2,0x08);
	lcm_dcs_write_seq_static(ctx,0x35,0x00);
	lcm_dcs_write_seq_static(ctx,0x51,0x0D,0xBB);

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
	lcm_dcs_write_seq_static(ctx, 0x10);
	msleep(120);

	ctx->error = 0;
	ctx->prepared = false;
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



	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);


#endif
	ctx->hbm_en = false;
	ctx->hbm_stat = false;
	
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_info("%s\n", __func__);

	if (ctx->prepared)
		return 0;

#if defined(CONFIG_RT5081_PMU_DSV) || defined(CONFIG_MT6370_PMU_DSV)
	lcm_panel_bias_enable();
#else


	ctx->pm_enable_gpio = devm_gpiod_get_index(ctx->dev,
		"pm-enable", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->pm_enable_gpio)) {
		dev_err(ctx->dev, "%s: cannot get pm_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->pm_enable_gpio));
		return PTR_ERR(ctx->pm_enable_gpio);
	}
	gpiod_set_value(ctx->pm_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->pm_enable_gpio);

	udelay(3000);


	ctx->bias_pos = devm_gpiod_get_index(ctx->dev,
		"bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_err(ctx->dev, "%s: cannot get bias_pos %ld\n",
			__func__, PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	udelay(3000);

	ctx->bias_neg = devm_gpiod_get_index(ctx->dev,
		"bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_err(ctx->dev, "%s: cannot get bias_neg %ld\n",
			__func__, PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
#endif


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

#define VAC (2400)
#define HAC (1080)
static u32 fake_heigh = 2400;
static u32 fake_width = 1080;
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
	 pr_err("[rm692e5]%s level=%d\n", __func__, level);


	if(level){
		reg_level = Gamma_to_level[level] + BLK_LEVEL_OFFSET;
		bl_level = level; 
	} else {
		reg_level = 0;
	}
	bl_tb0[1] = (reg_level>>8)&0xf;
	bl_tb0[2] = (reg_level)&0xff;
	pr_err("level{ %d - %d },bl_tb0[1] = %d,bl_tb0[2] = %d\n",level,reg_level,bl_tb0[1],bl_tb0[2]);
	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	
	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
			      dcs_write_gce cb, void *handle, bool en)
{
	// unsigned int level_hbm = 255;
	unsigned int level_normal = 125;
	char normal_tb0[] = {0x51, 0x07,0xFF};
	char hbm_tb[] = {0x51,0x0F,0xFF};
	struct lcm *ctx = panel_to_lcm(panel);

	if (!cb)
		return -1;

	//if (ctx->hbm_en == en)
	//	goto done;

	if (en)
	{
		pr_err("[panel] %s : set HBM\n",__func__);
	#if 0
		lcm_dcs_write_seq_static(ctx,0xF0,0x55, 0xAA, 0x52, 0x08, 0x00);   //ELVSS
		udelay(100);
		lcm_dcs_write_seq_static(ctx,0xB5,0x80,0x80);
		lcm_dcs_write_seq_static(ctx,0x6F,0x07);
		lcm_dcs_write_seq_static(ctx,0xB5,0x1D);
	#endif
		g_ctx->hbm_stat = true;
		cb(dsi, handle, hbm_tb, ARRAY_SIZE(hbm_tb));
	}
	else
	{
		pr_err("[panel] %s : set normal = %d\n",__func__,bl_level);
		level_normal = bl_level * BLK_LEVEL_MAP3/255 + BLK_LEVEL_OFFSET;
		normal_tb0[1] = (level_normal>>8)&0xff;
		normal_tb0[2] = (level_normal)&0xff;
	#if 0
		lcm_dcs_write_seq_static(ctx,0xF0,0x55, 0xAA, 0x52, 0x08, 0x00);   //ELVSS
		udelay(100);
		lcm_dcs_write_seq_static(ctx,0xB5,0x80,0x80);
		lcm_dcs_write_seq_static(ctx,0x6F,0x07);
		lcm_dcs_write_seq_static(ctx,0xB5,0x23);
	#endif
		
		g_ctx->hbm_stat = false;
		cb(dsi, handle, normal_tb0, ARRAY_SIZE(normal_tb0));
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
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.para_list[1] = 0xdc,
	},
	.lp_perline_en = 1,
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
};

static struct mtk_panel_params ext_params_90 = {
	// .vfp_low_power = 743,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.para_list[1] = 0xdc,
	},
	.lp_perline_en = 1,
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
};


static struct mtk_panel_params ext_params = {
	// .pll_clk = 373,
	// .vfp_low_power = 743,
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	.lcm_esd_check_table[0] = {
		.cmd = 0x0a,
		.count = 1,
		.para_list[0] = 0x9c,
		.para_list[1] = 0xdc,
	},
	.lp_perline_en = 1,
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
};


#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
void lcm_get_hardware_info(struct hardware_info *hwinfo)
{
	if (hwinfo != NULL) {
		strcpy(hwinfo->chip,"RM692E5");
		strcpy(hwinfo->vendor,"Raydium");
		strcpy(hwinfo->more,"1080*2400");
	}
}
EXPORT_SYMBOL_GPL(lcm_get_hardware_info);
#endif

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
		lcm_dcs_write_seq_static(ctx,0xFE,0x40);
		lcm_dcs_write_seq_static(ctx,0xBD,0x07);
		lcm_dcs_write_seq_static(ctx,0xFE,0xA1);
		lcm_dcs_write_seq_static(ctx,0xCD,0x6B);  
		lcm_dcs_write_seq_static(ctx,0xCE,0xBB);  
		lcm_dcs_write_seq_static(ctx,0xFE,0XD1);
		lcm_dcs_write_seq_static(ctx,0XB4,0x01);
		lcm_dcs_write_seq_static(ctx,0xFE,0x38);
		lcm_dcs_write_seq_static(ctx,0x17,0x0F);  
		lcm_dcs_write_seq_static(ctx,0x18,0x0F);  
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0xFA,0x01);
		lcm_dcs_write_seq_static(ctx,0xC2,0x08);
		lcm_dcs_write_seq_static(ctx,0x35,0x00);
	}
}

static void mode_switch_to_90(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_err("[panel] %s\n",__func__);
		lcm_dcs_write_seq_static(ctx,0xFE,0x40);
		lcm_dcs_write_seq_static(ctx,0xBD,0x06);
		lcm_dcs_write_seq_static(ctx,0xFE,0xA1);
		lcm_dcs_write_seq_static(ctx,0xCD,0x6B); 
		lcm_dcs_write_seq_static(ctx,0xCE,0xBB); 
		lcm_dcs_write_seq_static(ctx,0xFE,0XD1);
		lcm_dcs_write_seq_static(ctx,0XB4,0x01);
		lcm_dcs_write_seq_static(ctx,0xFE,0x38);
		lcm_dcs_write_seq_static(ctx,0x17,0x0F); 
		lcm_dcs_write_seq_static(ctx,0x18,0x0F);
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0xFA,0x01);
		lcm_dcs_write_seq_static(ctx,0xC2,0x08);
		lcm_dcs_write_seq_static(ctx,0x35,0x00);
	}
}

static void mode_switch_to_60(struct drm_panel *panel,
	enum MTK_PANEL_MODE_SWITCH_STAGE stage)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (stage == BEFORE_DSI_POWERDOWN) {
		pr_err("[panel] %s\n",__func__);
		lcm_dcs_write_seq_static(ctx,0xFE,0x40);
		lcm_dcs_write_seq_static(ctx,0xBD,0x00);
		lcm_dcs_write_seq_static(ctx,0xFE,0xA1);
		lcm_dcs_write_seq_static(ctx,0xCD,0x6B); 
		lcm_dcs_write_seq_static(ctx,0xCE,0xBB); 
		lcm_dcs_write_seq_static(ctx,0xFE,0XD1);
		lcm_dcs_write_seq_static(ctx,0XB4,0x01);
		lcm_dcs_write_seq_static(ctx,0xFE,0x38);
		lcm_dcs_write_seq_static(ctx,0x17,0x0F);
		lcm_dcs_write_seq_static(ctx,0x18,0x0F);
		lcm_dcs_write_seq_static(ctx,0xFE,0x00);
		lcm_dcs_write_seq_static(ctx,0xFA,0x01);
		lcm_dcs_write_seq_static(ctx,0xC2,0x08);
		lcm_dcs_write_seq_static(ctx,0x35,0x00);
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


static int panel_doze_enable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel,
	void *dsi, dcs_write_gce cb, void *handle)
{
	//struct lcm *ctx = panel_to_lcm(panel);

	pr_info("panel %s\n", __func__);

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

	// lcm_panel_init(ctx);
	g_ctx = ctx;
	ctx->hbm_en = false;
	g_ctx->hbm_stat = false;

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{ .compatible = "Novatek,rm692e5,cmd", },
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver rm692e5_lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "panel-rm692e5-cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(rm692e5_lcm_driver);

MODULE_AUTHOR("Yi-Lun Wang <Yi-Lun.Wang@mediatek.com>");
MODULE_DESCRIPTION("rm692e5 CMD LCD Panel Driver");
MODULE_LICENSE("GPL v2");
