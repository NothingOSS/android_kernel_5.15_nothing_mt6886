// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/err.h>
#include <linux/module.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

#include <drm/drm_panel.h>

#include "mtk_panel_ext.h"

#include "mtk_disp_notify.h"


struct _panel_rst_ctx {
	struct drm_panel *panel;
	panel_tch_rst rst_cb;
};

static DEFINE_MUTEX(panel_ext_lock);
static DEFINE_MUTEX(panel_boot_lock);
static LIST_HEAD(panel_ext_list);
static struct _panel_rst_ctx panel_rst_ctx;
static enum mtk_lcm_version g_lcm_version;
static int g_lcm_hbm_status = 0;
static int g_lcm_ui_status = 0;

void mtk_panel_update_lcm_state_to_fingerprint()
{
	static int g_lcm_last_hbm_status = 0;
	static int g_lcm_last_ui_status = 0;
	int lcm_hbm_ui_status_chg = 0;
	struct fp_notify_event event;

	if (g_lcm_last_hbm_status != g_lcm_hbm_status) {
		printk("[%s] g_lcm_last_hbm_status:%d change to  g_lcm_hbm_status:%d\n", __func__,
			g_lcm_last_hbm_status,
			g_lcm_hbm_status);
		g_lcm_last_hbm_status = g_lcm_hbm_status;
		lcm_hbm_ui_status_chg = 1;
	}

	if (g_lcm_last_ui_status != g_lcm_ui_status) {
		printk("[%s] g_lcm_last_ui_status:%d change to  g_lcm_ui_status:%d\n", __func__,
			g_lcm_last_ui_status,
			g_lcm_ui_status);
		g_lcm_last_ui_status = g_lcm_ui_status;
		lcm_hbm_ui_status_chg = 1;
	}

	if (lcm_hbm_ui_status_chg) {
		event.hbm_status = g_lcm_hbm_status;
		event.ui_status = g_lcm_ui_status;
		mtk_disp_notifier_call_chain(FP_NOTIFIER_EVENT_UI, &event);
	}
}
EXPORT_SYMBOL(mtk_panel_update_lcm_state_to_fingerprint);

void mtk_panel_proc_hbm(int hbm_status)
{
	printk("[%s] hbm_status: %d\n",__func__, hbm_status);
	g_lcm_hbm_status = hbm_status;
	mtk_panel_update_lcm_state_to_fingerprint();
}
EXPORT_SYMBOL(mtk_panel_proc_hbm);

int mtk_panel_get_ui_status()
{
	printk("[%s] ui_status: %d\n",__func__, g_lcm_ui_status);
	return g_lcm_ui_status;
}
EXPORT_SYMBOL(mtk_panel_get_ui_status);

void mtk_panel_proc_ui_status(int ui_status)
{
	printk("[%s] ui_status: %d\n",__func__, ui_status);
	g_lcm_ui_status = ui_status;
	mtk_panel_update_lcm_state_to_fingerprint();
}
EXPORT_SYMBOL(mtk_panel_proc_ui_status);

void mtk_panel_init(struct mtk_panel_ctx *ctx)
{
	INIT_LIST_HEAD(&ctx->list);
}

void mtk_panel_add(struct mtk_panel_ctx *ctx)
{
	mutex_lock(&panel_ext_lock);
	list_add_tail(&ctx->list, &panel_ext_list);
	mutex_unlock(&panel_ext_lock);
}

void mtk_panel_remove(struct mtk_panel_ctx *ctx)
{
	mutex_lock(&panel_ext_lock);
	list_del_init(&ctx->list);
	mutex_unlock(&panel_ext_lock);
}
EXPORT_SYMBOL(mtk_panel_remove);

int mtk_panel_attach(struct mtk_panel_ctx *ctx, struct drm_panel *panel)
{
	if (ctx->panel)
		return -EBUSY;

	ctx->panel = panel;

	return 0;
}

int mtk_panel_tch_handle_reg(struct drm_panel *panel)
{
	mutex_lock(&panel_ext_lock);
	if (panel_rst_ctx.panel) {
		mutex_unlock(&panel_ext_lock);
		return -EEXIST;
	}
	panel_rst_ctx.panel = panel;
	mutex_unlock(&panel_ext_lock);

	return 0;
}
EXPORT_SYMBOL(mtk_panel_tch_handle_reg);

void **mtk_panel_tch_handle_init(void)
{
	return (void **)&panel_rst_ctx.rst_cb;
}
EXPORT_SYMBOL(mtk_panel_tch_handle_init);

int mtk_panel_tch_rst(struct drm_panel *panel)
{
	int ret = 0;

	mutex_lock(&panel_ext_lock);
	if (panel_rst_ctx.rst_cb && panel_rst_ctx.panel == panel)
		panel_rst_ctx.rst_cb();
	else
		ret = -EEXIST;
	mutex_unlock(&panel_ext_lock);

	return ret;
}
EXPORT_SYMBOL(mtk_panel_tch_rst);

void mtk_panel_lock(void)
{
	mutex_lock(&panel_boot_lock);
}
EXPORT_SYMBOL(mtk_panel_lock);

void mtk_panel_unlock(void)
{
	mutex_unlock(&panel_boot_lock);
}
EXPORT_SYMBOL(mtk_panel_unlock);

int mtk_panel_detach(struct mtk_panel_ctx *ctx)
{
	ctx->panel = NULL;

	return 0;
}
EXPORT_SYMBOL(mtk_panel_detach);

int mtk_panel_ext_create(struct device *dev,
			 struct mtk_panel_params *ext_params,
			 struct mtk_panel_funcs *ext_funcs,
			 struct drm_panel *panel)
{
	struct mtk_panel_ctx *ext_ctx;
	struct mtk_panel_ext *ext;

	ext_ctx = devm_kzalloc(dev, sizeof(struct mtk_panel_ctx), GFP_KERNEL);
	if (!ext_ctx)
		return -ENOMEM;

	ext = devm_kzalloc(dev, sizeof(struct mtk_panel_ext), GFP_KERNEL);
	if (!ext)
		return -ENOMEM;

	mtk_panel_init(ext_ctx);
	ext->params = ext_params;
	ext->funcs = ext_funcs;
	ext->is_connected = -1;
	ext_ctx->ext = ext;

	mtk_panel_add(ext_ctx);
	mtk_panel_attach(ext_ctx, panel);
	if (IS_ERR_OR_NULL(ext_funcs->get_lcm_version))
		g_lcm_version = MTK_LEGACY_LCM_DRV;
	else
		g_lcm_version = ext_funcs->get_lcm_version();

	return 0;
}
EXPORT_SYMBOL(mtk_panel_ext_create);

struct mtk_panel_ctx *find_panel_ctx(struct drm_panel *panel)
{
	struct mtk_panel_ctx *ctx;

	mutex_lock(&panel_ext_lock);

	list_for_each_entry(ctx, &panel_ext_list, list) {
		if (ctx->panel == panel) {
			mutex_unlock(&panel_ext_lock);
			return ctx;
		}
	}

	mutex_unlock(&panel_ext_lock);
	return NULL;
}
EXPORT_SYMBOL(find_panel_ctx);

struct mtk_panel_ext *find_panel_ext(struct drm_panel *panel)
{
	struct mtk_panel_ctx *ctx = find_panel_ctx(panel);

	if (ctx)
		return ctx->ext;

	return NULL;
}
EXPORT_SYMBOL(find_panel_ext);

enum mtk_lcm_version mtk_drm_get_lcm_version(void)
{
	return g_lcm_version;
}
EXPORT_SYMBOL(mtk_drm_get_lcm_version);

MODULE_AUTHOR("Tai-Hua Tseng <tai-hua.tseng@mediatek.com>");
MODULE_DESCRIPTION("MTK DRM panel infrastructure");
MODULE_LICENSE("GPL v2");
