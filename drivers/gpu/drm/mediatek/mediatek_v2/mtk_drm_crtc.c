// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <asm/barrier.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fourcc.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/mailbox_controller.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <soc/mediatek/smi.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include <drm/drm_vblank.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <drm/drm_crtc.h>
#include <linux/kmemleak.h>
#include <linux/time.h>

#ifndef DRM_CMDQ_DISABLE
#include <linux/soc/mediatek/mtk-cmdq-ext.h>
#else
#include "mtk-cmdq-ext.h"
#endif

#include "mtk_drm_arr.h"
#include "mtk_drm_drv.h"
#include "mtk_drm_crtc.h"
#include "mtk_drm_ddp.h"
#include "mtk_drm_ddp_comp.h"
#include "mtk_drm_gem.h"
#include "mtk_drm_plane.h"
#include "mtk_writeback.h"
#include "mtk_fence.h"
#include "mtk_sync.h"
#include "mtk_drm_session.h"
#include "mtk_dump.h"
#include "mtk_drm_fb.h"
#include "mtk_rect.h"
#include "mtk_drm_ddp_addon.h"
#include "mtk_drm_helper.h"
#include "mtk_drm_lowpower.h"
#include "mtk_drm_assert.h"
#include "mtk_drm_mmp.h"
#include "mtk_disp_recovery.h"
#include "mtk_drm_arr.h"
#include "mtk_drm_trace.h"
#include "cmdq-util.h"
#include "mtk_disp_ccorr.h"
#include "mtk_debug.h"
#include "mtk_disp_oddmr/mtk_disp_oddmr.h"
#include "platform/mtk_drm_platform.h"

/* *****Panel_Master*********** */
#include "mtk_fbconfig_kdebug.h"
#include "mtk_layering_rule_base.h"

#include "mtk-mml.h"
#include "mtk-mml-drm-adaptor.h"
#include "mtk-mml-driver.h"
#include "mtk-mml-color.h"

#include <soc/mediatek/mmqos.h>

#include "mtk_disp_bdg.h"

struct drm_crtc *g_crtc;
EXPORT_SYMBOL(g_crtc);
extern int mtk_dsi_get_vendor_id(void);

static struct mtk_drm_property mtk_crtc_property[CRTC_PROP_MAX] = {
	{DRM_MODE_PROP_ATOMIC, "OVERLAP_LAYER_NUM", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "LAYERING_IDX", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "PRESENT_FENCE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "SF_PRESENT_FENCE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "DOZE_ACTIVE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_ENABLE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_BUFF_IDX", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_X", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_Y", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_WIDTH", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_HEIGHT", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_DST_WIDTH", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_DST_HEIGHT", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_FB_ID", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "INTF_BUFF_IDX", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "DISP_MODE_IDX", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "HBM_ENABLE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "COLOR_TRANSFORM", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "USER_SCEN", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "HDR_ENABLE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "MSYNC2_0_ENABLE", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OVL_DSI_SEQ", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "OUTPUT_SCENARIO", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC | DRM_MODE_PROP_IMMUTABLE,
						"CAPS_BLOB_ID", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "AOSP_CCORR_LINEAR", 0, UINT_MAX, 0},
	{DRM_MODE_PROP_ATOMIC, "NT_HDR_BRIGHTNESS", 0, UINT_MAX, 0},
};

static struct cmdq_pkt *sb_cmdq_handle;
static unsigned int sb_backlight;

struct timespec64 atomic_flush_tval;
struct timespec64 rdma_sof_tval;

bool hdr_en;
static const char * const crtc_gce_client_str[] = {
	DECLARE_GCE_CLIENT(DECLARE_STR)};

struct drm_mtk_ccorr_caps drm_ccorr_caps;

/* Overlay bw monitor define */
struct layer_compress_ratio_data
display_compress_ratio_table[MAX_LAYER_RATIO_NUMBER];
struct layer_compress_ratio_data
display_fbt_compress_ratio_table;

struct layer_compress_ratio_item
normal_layer_compress_ratio_tb[MAX_FRAME_RATIO_NUMBER*MAX_LAYER_RATIO_NUMBER];
struct layer_compress_ratio_item
fbt_layer_compress_ratio_tb[MAX_FRAME_RATIO_NUMBER];

struct layer_compress_ratio_item
unchanged_compress_ratio_table[MAX_LAYER_RATIO_NUMBER];
struct layer_compress_ratio_item
fbt_compress_ratio_table[MAX_FRAME_RATIO_NUMBER];

#ifndef DRM_CMDQ_DISABLE
/* frame number index for ring buffer to record ratio */
static unsigned int fn;
#endif
/* overlay bandwidth monitor BURST ACC Window size */
unsigned int ovl_win_size;

static bool g_ccorr_linear;

bool need_brightness_sync = false;
EXPORT_SYMBOL(need_brightness_sync);
unsigned int brightness_now = 0;

#define ALIGN_TO_32(x) ALIGN_TO(x, 32)

#define DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL 0x308
#define DISP_REG_CONFIG_BYPASS_MUX_SHADOW 0xf00
#define DISP_REG_CONFIG_OVLSYS_GCE_EVENT_SEL 0x308


#define DISP_MUTEX0_EN 0xA0
#define DISP_MUTEX0_CTL 0xAc
#define DISP_MUTEX0_MOD0 0xB0
#define DISP_MUTEX0_MOD1 0xB4

/* OVL Bandwidth monitor */
#define DISP_REG_OVL_LX_BURST_ACC(n) (0x940UL + 0x4 * (n))
#define DISP_REG_OVL_ELX_BURST_ACC(n) (0x950UL + 0x4 * (n))
#define DISP_REG_OVL_LX_BURST_ACC_WIN_MAX(n) (0x960UL + 0x4 * (n))
#define DISP_REG_OVL_ELX_BURST_ACC_WIN_MAX(n) (0x970UL + 0x4 * (n))

#define DISP_REG_OVL_GREQ_LAYER_CNT (0x234UL)

struct drm_crtc *_get_context(void)
{
	static int is_context_inited;
	static struct drm_crtc g_context;

	if (!is_context_inited) {
		memset((void *)&g_context, 0, sizeof(
					struct drm_crtc));
		is_context_inited = 1;
	}

	return &g_context;
}

static void mtk_drm_crtc_finish_page_flip(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	drm_crtc_send_vblank_event(crtc, mtk_crtc->event);
	drm_crtc_vblank_put(crtc);
	mtk_crtc->event = NULL;
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void mtk_drm_finish_page_flip(struct mtk_drm_crtc *mtk_crtc)
{
	drm_crtc_handle_vblank(&mtk_crtc->base);
	if (mtk_crtc->pending_needs_vblank) {
		mtk_drm_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}
}

static void mtk_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	mtk_disp_mutex_put(mtk_crtc->mutex[0]);

	drm_crtc_cleanup(crtc);
}

static void mtk_drm_crtc_reset(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state;

	if (crtc->state) {
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

		state = to_mtk_crtc_state(crtc->state);
		memset(state, 0, sizeof(*state));
	} else {
		state = kzalloc(sizeof(*state), GFP_KERNEL);
		if (!state)
			return;
		crtc->state = &state->base;
	}

	state->base.crtc = crtc;
}

static int mtk_drm_wait_blank(struct mtk_drm_crtc *mtk_crtc,
	bool blank, long timeout)
{
	int ret;

	ret = wait_event_timeout(mtk_crtc->state_wait_queue,
		mtk_crtc->crtc_blank == blank, timeout);

	return ret;
}

int mtk_drm_crtc_wait_blank(struct mtk_drm_crtc *mtk_crtc)
{
	int ret = 0;

	if (mtk_crtc->crtc_blank == false)
		return ret;

	DDPMSG("%s wait TUI finish\n", __func__);
	while (mtk_crtc->crtc_blank == true) {
//		DDP_MUTEX_UNLOCK(&mtk_crtc->blank_lock, __func__, __LINE__);
		ret |= mtk_drm_wait_blank(mtk_crtc, false, HZ / 5);
//		DDP_MUTEX_LOCK(&mtk_crtc->blank_lock, __func__, __LINE__);
	}
	DDPMSG("%s TUI done state=%d\n", __func__,
		mtk_crtc->crtc_blank);

	return ret;
}

static void mtk_drm_crtc_addon_dump(struct drm_crtc *crtc,
			const struct mtk_addon_scenario_data *addon_data)
{
	int i, j;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	const struct mtk_addon_path_data *addon_path;
	enum addon_module module;
	struct mtk_ddp_comp *comp;

	if (!addon_data)
		return;

	for (i = 0; i < addon_data->module_num; i++) {
		module = addon_data->module_data[i].module;
		addon_path = mtk_addon_module_get_path(module);

		for (j = 0; j < addon_path->path_len; j++) {
			if (mtk_ddp_comp_get_type(addon_path->path[j])
				== MTK_DISP_VIRTUAL)
				continue;
			comp = priv->ddp_comp[addon_path->path[j]];
			mtk_dump_reg(comp);
			mtk_ddp_comp_dump(comp);
		}
	}
}

void mtk_drm_crtc_mini_dump(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_ddp_comp *comp;
	int crtc_id = drm_crtc_index(crtc);

	if (crtc_id < 0) {
		DDPPR_ERR("%s: Invalid crtc_id:%d\n", __func__, crtc_id);
		return;
	}

	if (priv->mmsys_dev && pm_runtime_get_if_in_use(priv->mmsys_dev) <= 0)
		goto done_return;
	if (priv->side_mmsys_dev && pm_runtime_get_if_in_use(priv->side_mmsys_dev) <= 0)
		goto done_return;
	if (priv->ovlsys_dev && pm_runtime_get_if_in_use(priv->ovlsys_dev) <= 0)
		goto done_return;
	if (priv->side_ovlsys_dev && pm_runtime_get_if_in_use(priv->side_ovlsys_dev) <= 0)
		goto done_return;

	DDPFUNC("crtc%d\n", crtc_id);

	switch (priv->data->mmsys_id) {
	case MMSYS_MT2701:
	case MMSYS_MT2712:
	case MMSYS_MT8173:
	case MMSYS_MT6779:
		break;
	case MMSYS_MT6885:
		mmsys_config_dump_reg_mt6885(mtk_crtc->config_regs);
		break;
	case MMSYS_MT6983:
		mmsys_config_dump_reg_mt6983(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs)
			mmsys_config_dump_reg_mt6983(mtk_crtc->side_config_regs);
		break;
	case MMSYS_MT6985:
		DDPDUMP("== DISP pipe-0 OVLSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->ovlsys0_regs_pa);
		ovlsys_config_dump_reg_mt6985(mtk_crtc->ovlsys0_regs);
		if (mtk_crtc->ovlsys1_regs) {
			DDPDUMP("== DISP pipe-1 OVLSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->ovlsys1_regs_pa);
			ovlsys_config_dump_reg_mt6985(mtk_crtc->ovlsys1_regs);
		}

		DDPDUMP("== DISP pipe-0 MMSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->config_regs_pa);
		mmsys_config_dump_reg_mt6985(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DISP pipe-1 MMSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_reg_mt6985(mtk_crtc->side_config_regs);
		}
		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DSI)))
					mtk_dump_reg(comp);
			}
		}
		break;
	case MMSYS_MT6895:
	case MMSYS_MT6886:
		DDPDUMP("== DISP pipe0 MMSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->config_regs_pa);
		mmsys_config_dump_reg_mt6895(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DISP pipe1 MMSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_reg_mt6895(mtk_crtc->side_config_regs);
		}
		break;
	case MMSYS_MT6873:
	case MMSYS_MT6853:
	case MMSYS_MT6833:
	case MMSYS_MT6879:
	case MMSYS_MT6855:
		mmsys_config_dump_reg_mt6879(mtk_crtc->config_regs);
		break;
	default:
		DDPPR_ERR("%s mtk drm not support mmsys id %d\n",
			__func__, priv->data->mmsys_id);
		break;
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
			(mtk_ddp_comp_get_type(comp->id) == MTK_DSI)))
			mtk_dump_reg(comp);
	}

	if (priv->side_ovlsys_dev)
		pm_runtime_put(priv->side_ovlsys_dev);
	if (priv->ovlsys_dev)
		pm_runtime_put(priv->ovlsys_dev);
	if (priv->side_mmsys_dev)
		pm_runtime_put(priv->side_mmsys_dev);
	if (priv->mmsys_dev)
		pm_runtime_put(priv->mmsys_dev);

done_return:
	return;
}

void mtk_drm_crtc_dump(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	const struct mtk_addon_scenario_data *addon_data;
	struct mtk_ddp_comp *comp;
	int crtc_id = drm_crtc_index(crtc);
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	enum addon_scenario scn;

	if (crtc_id < 0) {
		DDPPR_ERR("%s: Invalid crtc_id:%d\n", __func__, crtc_id);
		return;
	}

	if (priv->mmsys_dev && pm_runtime_get_if_in_use(priv->mmsys_dev) <= 0)
		goto done_return;
	if (priv->side_mmsys_dev && pm_runtime_get_if_in_use(priv->side_mmsys_dev) <= 0)
		goto done_return;
	if (priv->ovlsys_dev && pm_runtime_get_if_in_use(priv->ovlsys_dev) <= 0)
		goto done_return;
	if (priv->side_ovlsys_dev && pm_runtime_get_if_in_use(priv->side_ovlsys_dev) <= 0)
		goto done_return;

	DDPFUNC("crtc%d\n", crtc_id);

	switch (priv->data->mmsys_id) {
	case MMSYS_MT2701:
		break;
	case MMSYS_MT2712:
		break;
	case MMSYS_MT8173:
		break;
	case MMSYS_MT6779:
		break;
	case MMSYS_MT6885:
		mmsys_config_dump_reg_mt6885(mtk_crtc->config_regs);
		mutex_dump_reg_mt6885(mtk_crtc->mutex[0]);
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_reg(comp);
		break;
	case MMSYS_MT6983:
		mmsys_config_dump_reg_mt6983(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs)
			mmsys_config_dump_reg_mt6983(mtk_crtc->side_config_regs);
		mutex_dump_reg_mt6983(mtk_crtc->mutex[0]);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_reg(comp);
		break;
	case MMSYS_MT6985:
		DDPDUMP("== DISP pipe-0 OVLSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->ovlsys0_regs_pa);
		ovlsys_config_dump_reg_mt6985(mtk_crtc->ovlsys0_regs);
		if (mtk_crtc->ovlsys1_regs) {
			DDPDUMP("== DISP pipe-1 OVLSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->ovlsys1_regs_pa);
			ovlsys_config_dump_reg_mt6985(mtk_crtc->ovlsys1_regs);
		}

		DDPDUMP("== DISP pipe-0 MMSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->config_regs_pa);
		mmsys_config_dump_reg_mt6985(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DISP pipe-1 MMSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_reg_mt6985(mtk_crtc->side_config_regs);
		}

		mutex_dump_reg_mt6985(mtk_crtc->mutex[0]);
		mutex_ovlsys_dump_reg_mt6985(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6895:
	case MMSYS_MT6886:
		DDPDUMP("== DISP pipe0 MMSYS_CONFIG REGS:0x%x ==\n",
					mtk_crtc->config_regs_pa);
		mmsys_config_dump_reg_mt6895(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DISP pipe1 MMSYS_CONFIG REGS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_reg_mt6895(mtk_crtc->side_config_regs);
		}
		mutex_dump_reg_mt6895(mtk_crtc->mutex[0]);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_reg(comp);
		break;
	case MMSYS_MT6873:
	case MMSYS_MT6853:
	case MMSYS_MT6833:
	case MMSYS_MT6879:
	case MMSYS_MT6855:
	case MMSYS_MT6835:
		mmsys_config_dump_reg_mt6879(mtk_crtc->config_regs);
		mutex_dump_reg_mt6879(mtk_crtc->mutex[0]);
		break;
	default:
		DDPPR_ERR("%s mtk drm not support mmsys id %d\n",
			__func__, priv->data->mmsys_id);
		break;
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) mtk_dump_reg(comp);

	//addon from CWB
	if (mtk_crtc->cwb_info && mtk_crtc->cwb_info->enable
		&& !mtk_crtc->cwb_info->is_sec) {
		addon_data = mtk_addon_get_scenario_data(__func__, crtc,
						mtk_crtc->cwb_info->scn);
		mtk_drm_crtc_addon_dump(crtc, addon_data);
		if (mtk_crtc->is_dual_pipe) {
			addon_data = mtk_addon_get_scenario_data_dual
				(__func__, crtc, mtk_crtc->cwb_info->scn);
			mtk_drm_crtc_addon_dump(crtc, addon_data);
		}
	}

	//addon from disp_dump
	if (!crtc->state)
		DDPDUMP("%s dump nothing for null state\n", __func__);
	else {
		state = to_mtk_crtc_state(crtc->state);
		if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
			if (state->prop_val[CRTC_PROP_OUTPUT_SCENARIO] == 0)
				scn = WDMA_WRITE_BACK_OVL;
			else
				scn = WDMA_WRITE_BACK;
			addon_data = mtk_addon_get_scenario_data(__func__, crtc,
						scn);
			mtk_drm_crtc_addon_dump(crtc, addon_data);
			if (mtk_crtc->is_dual_pipe) {
				addon_data = mtk_addon_get_scenario_data_dual
					(__func__, crtc, scn);
				mtk_drm_crtc_addon_dump(crtc, addon_data);
			}
		}
	}

	//addon from layering rule
	if (!crtc->state)
		DDPDUMP("%s dump nothing for null state\n", __func__);
	else {
		state = to_mtk_crtc_state(crtc->state);
		addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					state->lye_state.scn[crtc_id]);
		mtk_drm_crtc_addon_dump(crtc, addon_data);
		if (mtk_crtc->is_dual_pipe) {
			addon_data = mtk_addon_get_scenario_data_dual
				(__func__, crtc, state->lye_state.scn[crtc_id]);
			mtk_drm_crtc_addon_dump(crtc, addon_data);
		}
	}

	if (panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		if (crtc_id == 3)
			comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		mtk_dump_reg(comp);
	}

	if (priv->side_ovlsys_dev)
		pm_runtime_put(priv->side_ovlsys_dev);
	if (priv->ovlsys_dev)
		pm_runtime_put(priv->ovlsys_dev);
	if (priv->side_mmsys_dev)
		pm_runtime_put(priv->side_mmsys_dev);
	if (priv->mmsys_dev)
		pm_runtime_put(priv->mmsys_dev);

done_return:
	return;
}

static void mtk_drm_crtc_addon_analysis(struct drm_crtc *crtc,
			const struct mtk_addon_scenario_data *addon_data)
{
	int i, j;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	const struct mtk_addon_path_data *addon_path;
	enum addon_module module;
	struct mtk_ddp_comp *comp;

	if (!addon_data)
		return;

	for (i = 0; i < addon_data->module_num; i++) {
		module = addon_data->module_data[i].module;
		addon_path = mtk_addon_module_get_path(module);

		for (j = 0; j < addon_path->path_len; j++) {
			if (mtk_ddp_comp_get_type(addon_path->path[j])
				== MTK_DISP_VIRTUAL)
				continue;
			comp = priv->ddp_comp[addon_path->path[j]];
			mtk_dump_analysis(comp);
		}
	}
}

void mtk_drm_crtc_mini_analysis(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_ddp_comp *comp;
	int crtc_id = drm_crtc_index(crtc);

	if (crtc_id < 0) {
		DDPPR_ERR("%s: Invalid crtc_id:%d\n", __func__, crtc_id);
		return;
	}

	if (priv->mmsys_dev && pm_runtime_get_if_in_use(priv->mmsys_dev) <= 0)
		goto done_return;
	if (priv->side_mmsys_dev && pm_runtime_get_if_in_use(priv->side_mmsys_dev) <= 0)
		goto done_return;
	if (priv->ovlsys_dev && pm_runtime_get_if_in_use(priv->ovlsys_dev) <= 0)
		goto done_return;
	if (priv->side_ovlsys_dev && pm_runtime_get_if_in_use(priv->side_ovlsys_dev) <= 0)
		goto done_return;

	DDPFUNC("crtc%d\n", crtc_id);

	switch (priv->data->mmsys_id) {
	case MMSYS_MT2701:
	case MMSYS_MT2712:
	case MMSYS_MT8173:
	case MMSYS_MT6779:
		break;
	case MMSYS_MT6885:
		mmsys_config_dump_analysis_mt6885(mtk_crtc->config_regs);
		if (mtk_crtc->is_dual_pipe) {
			DDPDUMP("anlysis dual pipe\n");
			mtk_ddp_dual_pipe_dump(mtk_crtc);
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DSI))) {
					mtk_dump_analysis(comp);
					mtk_dump_reg(comp);
				}
			}
		}
		break;
	case MMSYS_MT6983:
		mmsys_config_dump_analysis_mt6983(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("DUMP DISPSYS1\n");
			mmsys_config_dump_analysis_mt6983(mtk_crtc->side_config_regs);
		}
		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DSI))) {
					mtk_dump_analysis(comp);
					mtk_dump_reg(comp);
				}
			}
		}
		break;
	case MMSYS_MT6985:
		DDPDUMP("== DUMP OVLSYS pipe-0 ANALYSIS:0x%x ==\n",
			mtk_crtc->ovlsys0_regs_pa);
		ovlsys_config_dump_analysis_mt6985(mtk_crtc->ovlsys0_regs);
		if (mtk_crtc->ovlsys1_regs) {
			DDPDUMP("== DUMP OVLSYS pipe-1 ANALYSIS:0x%x ==\n",
				mtk_crtc->ovlsys1_regs_pa);
			ovlsys_config_dump_analysis_mt6985(mtk_crtc->ovlsys1_regs);
		}

		DDPDUMP("== DUMP DISP pipe-0 ANALYSIS:0x%x ==\n",
			mtk_crtc->config_regs_pa);
		mmsys_config_dump_analysis_mt6985(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DUMP DISP pipe-1 ANALYSIS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_analysis_mt6985(mtk_crtc->side_config_regs);
		}

		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DSI)))
					mtk_dump_analysis(comp);
			}
		}
		break;
	case MMSYS_MT6895:
	case MMSYS_MT6886:
		DDPDUMP("== DUMP DISP pipe0 ANALYSIS:0x%x ==\n",
			mtk_crtc->config_regs_pa);
		mmsys_config_dump_analysis_mt6895(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DUMP DISP pipe1 ANALYSIS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_analysis_mt6895(mtk_crtc->side_config_regs);
		}
		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DSI))) {
					mtk_dump_analysis(comp);
					mtk_dump_reg(comp);
				}
			}
		}
		break;
	case MMSYS_MT6873:
	case MMSYS_MT6853:
	case MMSYS_MT6833:
	case MMSYS_MT6879:
	case MMSYS_MT6855:
		break;
	default:
		DDPPR_ERR("%s mtk drm not support mmsys id %d\n",
			__func__, priv->data->mmsys_id);
		break;
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (comp && ((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) ||
			(mtk_ddp_comp_get_type(comp->id) == MTK_DSI)))
			mtk_dump_analysis(comp);
	}

	if (priv->side_ovlsys_dev)
		pm_runtime_put(priv->side_ovlsys_dev);
	if (priv->ovlsys_dev)
		pm_runtime_put(priv->ovlsys_dev);
	if (priv->side_mmsys_dev)
		pm_runtime_put(priv->side_mmsys_dev);
	if (priv->mmsys_dev)
		pm_runtime_put(priv->mmsys_dev);

done_return:
	return;
}

void mtk_drm_crtc_analysis(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	const struct mtk_addon_scenario_data *addon_data;
	struct mtk_ddp_comp *comp;
	int crtc_id = drm_crtc_index(crtc);
	enum addon_scenario scn;

	if (crtc_id < 0) {
		DDPPR_ERR("%s: Invalid crtc_id:%d\n", __func__, crtc_id);
		return;
	}

	if (priv->mmsys_dev && pm_runtime_get_if_in_use(priv->mmsys_dev) <= 0)
		goto done_return;
	if (priv->side_mmsys_dev && pm_runtime_get_if_in_use(priv->side_mmsys_dev) <= 0)
		goto done_return;
	if (priv->ovlsys_dev && pm_runtime_get_if_in_use(priv->ovlsys_dev) <= 0)
		goto done_return;
	if (priv->side_ovlsys_dev && pm_runtime_get_if_in_use(priv->side_ovlsys_dev) <= 0)
		goto done_return;

	DDPFUNC("crtc%d\n", crtc_id);

	switch (priv->data->mmsys_id) {
	case MMSYS_MT2701:
		break;
	case MMSYS_MT2712:
		break;
	case MMSYS_MT8173:
		break;
	case MMSYS_MT6779:
		break;
	case MMSYS_MT6885:
		mmsys_config_dump_analysis_mt6885(mtk_crtc->config_regs);
		if (mtk_crtc->is_dual_pipe) {
			DDPDUMP("anlysis dual pipe\n");
			mtk_ddp_dual_pipe_dump(mtk_crtc);
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				mtk_dump_analysis(comp);
				mtk_dump_reg(comp);
			}
		}
		mutex_dump_analysis_mt6885(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6983:
		mmsys_config_dump_analysis_mt6983(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("DUMP DISPSYS1\n");
			mmsys_config_dump_analysis_mt6983(mtk_crtc->side_config_regs);
		}
		mutex_dump_analysis_mt6983(mtk_crtc->mutex[0]);
		if (mtk_crtc->is_dual_pipe) {
			DDPDUMP("anlysis dual pipe\n");
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				mtk_dump_analysis(comp);
				mtk_dump_reg(comp);
			}
		}
		break;
	case MMSYS_MT6985:
		DDPDUMP("== DUMP OVLSYS pipe-0 ANALYSIS:0x%x ==\n",
			mtk_crtc->ovlsys0_regs_pa);
		ovlsys_config_dump_analysis_mt6985(mtk_crtc->ovlsys0_regs);
		if (mtk_crtc->ovlsys1_regs) {
			DDPDUMP("== DUMP OVLSYS pipe-1 ANALYSIS:0x%x ==\n",
				mtk_crtc->ovlsys1_regs_pa);
			ovlsys_config_dump_analysis_mt6985(mtk_crtc->ovlsys1_regs);
		}

		DDPDUMP("== DUMP DISP pipe-0 ANALYSIS:0x%x ==\n",
			mtk_crtc->config_regs_pa);
		mmsys_config_dump_analysis_mt6985(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DUMP DISP pipe-1 ANALYSIS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_analysis_mt6985(mtk_crtc->side_config_regs);
		}

		mutex_dump_analysis_mt6985(mtk_crtc->mutex[0]);
		mutex_ovlsys_dump_analysis_mt6985(mtk_crtc->mutex[0]);
		if (mtk_crtc->is_dual_pipe) {
			DDPDUMP("anlysis dual pipe\n");
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				mtk_dump_analysis(comp);
				mtk_dump_reg(comp);
			}
		}
		break;
	case MMSYS_MT6895:
	case MMSYS_MT6886:
		DDPDUMP("== DUMP DISP pipe0 ANALYSIS:0x%x ==\n",
			mtk_crtc->config_regs_pa);
		mmsys_config_dump_analysis_mt6895(mtk_crtc->config_regs);
		if (mtk_crtc->side_config_regs) {
			DDPDUMP("== DUMP DISP pipe1 ANALYSIS:0x%x ==\n",
				mtk_crtc->side_config_regs_pa);
			mmsys_config_dump_analysis_mt6895(mtk_crtc->side_config_regs);
		}
		mutex_dump_analysis_mt6895(mtk_crtc->mutex[0]);
		if (mtk_crtc->is_dual_pipe) {
			DDPDUMP("anlysis dual pipe\n");
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				mtk_dump_analysis(comp);
				mtk_dump_reg(comp);
			}
		}
		break;
	case MMSYS_MT6873:
		mmsys_config_dump_analysis_mt6873(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6873(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6853:
		mmsys_config_dump_analysis_mt6853(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6853(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6833:
		mmsys_config_dump_analysis_mt6833(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6833(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6879:
		mmsys_config_dump_analysis_mt6879(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6879(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6855:
		mmsys_config_dump_analysis_mt6855(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6855(mtk_crtc->mutex[0]);
		break;
	case MMSYS_MT6835:
		mmsys_config_dump_analysis_mt6835(mtk_crtc->config_regs);
		mutex_dump_analysis_mt6835(mtk_crtc->mutex[0]);
		break;
	default:
		DDPPR_ERR("%s mtk drm not support mmsys id %d\n",
			__func__, priv->data->mmsys_id);
		break;
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_dump_analysis(comp);

	//addon from CWB
	if (mtk_crtc->cwb_info && mtk_crtc->cwb_info->enable
		&& !mtk_crtc->cwb_info->is_sec) {
		addon_data = mtk_addon_get_scenario_data(__func__, crtc,
						mtk_crtc->cwb_info->scn);
		mtk_drm_crtc_addon_analysis(crtc, addon_data);
		if (mtk_crtc->is_dual_pipe) {
			addon_data = mtk_addon_get_scenario_data_dual
				(__func__, crtc, mtk_crtc->cwb_info->scn);
			mtk_drm_crtc_addon_analysis(crtc, addon_data);
		}
	}

	//addon from disp_dump
	if (!crtc->state)
		DDPDUMP("%s dump nothing for null state\n", __func__);
	else {
		state = to_mtk_crtc_state(crtc->state);
		if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
			if (state->prop_val[CRTC_PROP_OUTPUT_SCENARIO] == 0)
				scn = WDMA_WRITE_BACK_OVL;
			else
				scn = WDMA_WRITE_BACK;
			addon_data = mtk_addon_get_scenario_data(__func__, crtc,
						scn);
			mtk_drm_crtc_addon_analysis(crtc, addon_data);
			if (mtk_crtc->is_dual_pipe) {
				addon_data = mtk_addon_get_scenario_data_dual
					(__func__, crtc, scn);
				mtk_drm_crtc_addon_analysis(crtc, addon_data);
			}
		}
	}

	//addon from layering rule
	if (!crtc->state)
		DDPDUMP("%s dump nothing for null state\n", __func__);
	else if (crtc_id < 0)
		DDPPR_ERR("%s: Invalid crtc_id:%d\n", __func__, crtc_id);
	else {
		state = to_mtk_crtc_state(crtc->state);
		addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					state->lye_state.scn[crtc_id]);
		mtk_drm_crtc_addon_analysis(crtc, addon_data);

		if (mtk_crtc->is_dual_pipe) {
			addon_data = mtk_addon_get_scenario_data_dual
				(__func__, crtc, state->lye_state.scn[crtc_id]);
			mtk_drm_crtc_addon_analysis(crtc, addon_data);
		}
	}

	if (priv->side_ovlsys_dev)
		pm_runtime_put(priv->side_ovlsys_dev);
	if (priv->ovlsys_dev)
		pm_runtime_put(priv->ovlsys_dev);
	if (priv->side_mmsys_dev)
		pm_runtime_put(priv->side_mmsys_dev);
	if (priv->mmsys_dev)
		pm_runtime_put(priv->mmsys_dev);

done_return:
	return;
}

struct mtk_ddp_comp *mtk_ddp_comp_request_output(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *comp;
	int i, j;

	for_each_comp_in_crtc_path_reverse(
		comp, mtk_crtc, i,
		j)
		if (mtk_ddp_comp_is_output(comp))
			return comp;

	/* This CRTC does not contain output comp */
	return NULL;
}

void mtk_crtc_change_output_mode(struct drm_crtc *crtc, int aod_en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp)
		return;

	DDPINFO("%s\n", __func__);
	switch (comp->id) {
	case DDP_COMPONENT_DSI0:
	case DDP_COMPONENT_DSI1:
		mtk_ddp_comp_io_cmd(comp, NULL, DSI_CHANGE_MODE, &aod_en);
		break;
	default:
		break;
	}
}

bool mtk_crtc_is_connector_enable(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	bool enable = 0;

	if (comp == NULL || mtk_ddp_comp_get_type(comp->id) != MTK_DSI)
		return enable;

	if (comp->funcs && comp->funcs->io_cmd)
		comp->funcs->io_cmd(comp, NULL, CONNECTOR_IS_ENABLE, &enable);

	return enable;
}

static struct drm_crtc_state *
mtk_drm_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state, *old_state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	if (!crtc) {
		DDPPR_ERR("NULL crtc\n");
		kfree(state);
		return NULL;
	}

	if (crtc->state)
		__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	if (state->base.crtc != crtc)
		DDPAEE("%s:%d, invalid crtc:(%p,%p)\n",
			__func__, __LINE__,
			state->base.crtc, crtc);
	state->base.crtc = crtc;

	if (crtc->state) {
		struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

		old_state = to_mtk_crtc_state(crtc->state);
		state->lye_state = old_state->lye_state;
		state->rsz_src_roi = old_state->rsz_src_roi;
		state->rsz_dst_roi = old_state->rsz_dst_roi;
		state->prop_val[CRTC_PROP_DOZE_ACTIVE] =
			old_state->prop_val[CRTC_PROP_DOZE_ACTIVE];
		if (mtk_crtc->res_switch != RES_SWITCH_NO_USE)
			state->prop_val[CRTC_PROP_DISP_MODE_IDX] =
				old_state->prop_val[CRTC_PROP_DISP_MODE_IDX];
		state->prop_val[CRTC_PROP_PRES_FENCE_IDX] =
			old_state->prop_val[CRTC_PROP_PRES_FENCE_IDX];
	}

	return &state->base;
}

static void mtk_drm_crtc_destroy_state(struct drm_crtc *crtc,
				       struct drm_crtc_state *state)
{
	struct mtk_crtc_state *s;

	s = to_mtk_crtc_state(state);

	__drm_atomic_helper_crtc_destroy_state(state);
	DDPINFO("destroy/free crtc_state, 0x%llx\n", (u64)state);
	kfree(s);
}

static int mtk_drm_crtc_set_property(struct drm_crtc *crtc,
				     struct drm_crtc_state *state,
				     struct drm_property *property,
				     uint64_t val)
{
	struct drm_device *dev = crtc->dev;
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(state);
	int index = drm_crtc_index(crtc);
	int ret = 0;
	int i;

	if (index < 0)
		return -EINVAL;

	for (i = 0; i < CRTC_PROP_MAX; i++) {
		if (private->crtc_property[index][i] == property) {
			crtc_state->prop_val[i] = (unsigned int)val;
			DDPINFO("crtc:%d set property:%s %d\n",
					index, property->name,
					(unsigned int)val);
			return ret;
		}
	}

	DDPPR_ERR("fail to set property:%s %d\n", property->name,
		  (unsigned int)val);
	return -EINVAL;
}

static int mtk_drm_crtc_get_property(struct drm_crtc *crtc,
				     const struct drm_crtc_state *state,
				     struct drm_property *property,
				     uint64_t *val)
{
	struct drm_device *dev = crtc->dev;
	struct mtk_drm_private *private = dev->dev_private;
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(state);
	int ret = 0;
	int index = drm_crtc_index(crtc);
	int i;

	if (index < 0) {
		DDPPR_ERR("%s invalid crtc index\n", __func__);
		return -EINVAL;
	}
	for (i = 0; i < CRTC_PROP_MAX; i++) {
		if (private->crtc_property[index][i] == property) {
			*val = crtc_state->prop_val[i];
			DDPINFO("get property:%s %lld\n", property->name, *val);
			return ret;
		}
	}

	DDPPR_ERR("fail to get property:%s %p\n", property->name, val);
	return -EINVAL;
}

struct mtk_ddp_comp *mtk_crtc_get_comp(struct drm_crtc *crtc,
				       unsigned int mode_id,
				       unsigned int path_id,
				       unsigned int comp_idx)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx = mtk_crtc->ddp_ctx;

	if (unlikely(mode_id > DDP_MINOR)) {
		DDPPR_ERR("invalid ddp mode:%u!\n", mtk_crtc->ddp_mode);
		return NULL;
	}

	if (unlikely(path_id >= DDP_PATH_NR)) {
		DDPPR_ERR("invalid path id:%u!\n", path_id);
		return NULL;
	}

	if (unlikely(__mtk_crtc_path_len(mtk_crtc, mode_id, path_id) == 0)) {
		DDPPR_ERR("invalid path len: mode %u, path %u!\n", mode_id, path_id);
		return NULL;
	}

	if (ddp_ctx[mode_id].ovl_comp_nr[path_id] &&
			comp_idx < ddp_ctx[mode_id].ovl_comp_nr[path_id])
		return ddp_ctx[mode_id].ovl_comp[path_id][comp_idx];

	/* shift comp_idx from global idx to local idx for ddp_comp array */
	comp_idx -= ddp_ctx[mode_id].ovl_comp_nr[path_id];
	return ddp_ctx[mode_id].ddp_comp[path_id][comp_idx];
}

struct mtk_ddp_comp *mtk_crtc_get_dual_comp(struct drm_crtc *crtc,
				       unsigned int path_id,
				       unsigned int comp_idx)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx = &mtk_crtc->dual_pipe_ddp_ctx;

	if (ddp_ctx->ovl_comp_nr[path_id] &&
			comp_idx < ddp_ctx->ovl_comp_nr[path_id])
		return ddp_ctx->ovl_comp[path_id][comp_idx];

	/* shift comp_idx from global idx to local idx for ddp_comp array */
	comp_idx -= ddp_ctx->ovl_comp_nr[path_id];
	return ddp_ctx->ddp_comp[path_id][comp_idx];
}

unsigned int mtk_get_mmsys_id(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv;

	if (crtc && crtc->dev && crtc->dev->dev_private) {
		priv = crtc->dev->dev_private;

		return priv->data->mmsys_id;
	}

	DDPPR_ERR("%s no vailid CRTC\n", __func__);
	return 0;
}

static void mtk_drm_crtc_lfr_update(struct drm_crtc *crtc,
					struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp =
		mtk_ddp_comp_request_output(mtk_crtc);

	mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_LFR_UPDATE, NULL);
}

static bool mtk_drm_crtc_mode_fixup(struct drm_crtc *crtc,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	/* Nothing to do here, but this callback is mandatory. */
	return true;
}

static void mtk_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);

	state->pending_width = crtc->mode.hdisplay;
	state->pending_height = crtc->mode.vdisplay;
	state->pending_vrefresh = drm_mode_vrefresh(&crtc->mode);
	wmb(); /* Make sure the above parameters are set before update */
	state->pending_config = true;
}

static int mtk_crtc_enable_vblank_thread(void *data)
{
	int ret = 0;
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	while (1) {
		ret = wait_event_interruptible(
			mtk_crtc->vblank_enable_wq,
			atomic_read(&mtk_crtc->vblank_enable_task_active));

		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
		if (mtk_crtc->enabled)
			mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		atomic_set(&mtk_crtc->vblank_enable_task_active, 0);

		if (kthread_should_stop()) {
			DDPPR_ERR("%s stopped\n", __func__);
			break;
		}
	}

	return 0;
}

int mtk_drm_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *drm = crtc->dev;
	unsigned int pipe = crtc->index;
	struct mtk_drm_private *priv = drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);
	struct mtk_ddp_comp *comp = mtk_crtc_get_comp(&mtk_crtc->base, mtk_crtc->ddp_mode, 0, 0);
	struct mtk_panel_params *panel_params;

	mtk_crtc->vblank_en = 1;

	if (!mtk_crtc->enabled) {
		CRTC_MMP_MARK((int) pipe, enable_vblank, 0xFFFFFFFF,
			0xFFFFFFFF);
		return 0;
	}

	/* We only consider CRTC0 vsync so far, need to modify to DPI, DPTX */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_IDLE_MGR) &&
	    drm_crtc_index(&mtk_crtc->base) == 0) {
		panel_params = mtk_drm_get_lcm_ext_params(crtc);
		if (panel_params && panel_params->vblank_off) {
			mtk_crtc->vblank_en = 0;
			return -EPERM;
		}
		/* The enable vblank is called in spinlock, so we create another
		 * thread to kick idle mode for cmd mode vsync
		 */
		atomic_set(&mtk_crtc->vblank_enable_task_active, 1);
		wake_up_interruptible(&mtk_crtc->vblank_enable_wq);
	}

	CRTC_MMP_MARK((int) pipe, enable_vblank, (unsigned long)comp,
			(unsigned long)&mtk_crtc->base);

	return 0;
}

void mtk_crtc_v_idle_apsrc_control(struct drm_crtc *crtc,
	struct cmdq_pkt *_cmdq_handle, bool reset, bool condition_check,
	unsigned int crtc_id, bool enable)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		return;

	if (priv->data->sodi_apsrc_config)
		priv->data->sodi_apsrc_config(crtc, _cmdq_handle,
			reset, condition_check, crtc_id, enable);
}

static void bl_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

static bool msync_is_on(struct mtk_drm_private *priv,
						struct mtk_panel_params *params,
						unsigned int crtc_id,
						struct mtk_crtc_state *state,
						struct mtk_crtc_state *old_state)
{
	if (priv == NULL || params == NULL ||
			state == NULL || old_state == NULL)
		return false;
	if (crtc_id == 0 &&
			mtk_drm_helper_get_opt(priv->helper_opt,
					MTK_DRM_OPT_MSYNC2_0) &&
			params->msync2_enable &&
			((state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0) ||
			(old_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0)))
		return true;
	return false;
}

int mtk_drm_setbacklight(struct drm_crtc *crtc, unsigned int level,
	unsigned int panel_ext_param, unsigned int cfg_flag)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct mtk_ddp_comp *oddmr_comp;
	struct mtk_cmdq_cb_data *cb_data;
	struct mtk_bl_ext_config bl_ext_config;
	static unsigned int bl_cnt;
	bool is_frame_mode;
	int index = drm_crtc_index(crtc);
	int ret = 0;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	//sync brightness and frame commit @{
	if (need_brightness_sync) {
		DDPINFO("%s: need brightness sync, level: %d\n", __func__, level);
		return 0;
	}
	//@}

	CRTC_MMP_EVENT_START(index, backlight, (unsigned long)crtc,
			level);

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS])
		sb_backlight = level;

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (!(mtk_crtc->enabled)) {
		DDPINFO("Sleep State set backlight stop --crtc not ebable\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		CRTC_MMP_EVENT_END(index, backlight, 0, 0);

		return -EINVAL;
	}

	if (!comp) {
		DDPINFO("%s no output comp\n", __func__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		CRTC_MMP_EVENT_END(index, backlight, 0, 1);

		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		DDPPR_ERR("cb data creation failed\n");
		CRTC_MMP_EVENT_END(index, backlight, 0, 2);
		return -EINVAL;
	}

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	/* SILKY BRIGHTNESS control flow only support CRTC0 */
	if (index == 0 && m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS] &&
		sb_cmdq_handle != NULL) {
		cmdq_handle = sb_cmdq_handle;
		sb_cmdq_handle = NULL;
	} else {
		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	}

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	/* set backlight */

	oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];
	mtk_ddp_comp_io_cmd(oddmr_comp, cmdq_handle, ODDMR_BL_CHG, &level);

	if ((cfg_flag & (0x1<<SET_BACKLIGHT_LEVEL)) && !(cfg_flag & (0x1<<SET_ELVSS_PN))
		&& !(cfg_flag & (0x1<<ENABLE_DYN_ELVSS))
		&& !(cfg_flag & (0x1<<DISABLE_DYN_ELVSS))) {

		DDPINFO("%s cfg_flag = %d,level=%d\n", __func__, cfg_flag, level);
		if (comp && comp->funcs && comp->funcs->io_cmd) {
			if (is_bdg_supported()) {
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_STOP_VDO_MODE, NULL);
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_START_VDO_MODE, NULL);
			} else
				comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level);
		}
	} else {

		/* set backlight and elvss*/
		bl_ext_config.cfg_flag = cfg_flag;
		bl_ext_config.backlight_level = level;
		bl_ext_config.elvss_pn = panel_ext_param;

		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL_ELVSS, &bl_ext_config);

	}
	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	CRTC_MMP_MARK(index, backlight, bl_cnt, 0);
	bl_cnt++;

	cb_data->crtc = crtc;
	cb_data->cmdq_handle = cmdq_handle;

	if (cmdq_pkt_flush_threaded(cmdq_handle, bl_cmdq_cb, cb_data) < 0) {
		DDPPR_ERR("failed to flush bl_cmdq_cb\n");
		ret = -EINVAL;
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	CRTC_MMP_EVENT_END(index, backlight, (unsigned long)crtc,
			level);

	/*sync brightness and frame commit*/
	brightness_now = level;

	return ret;
}

//sync brightness and frame commit @{
unsigned int level_pending = 0;
static void mtk_drm_setbacklight_delay(struct work_struct *ws)
{
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_crtc *crtc;
	struct mtk_ddp_comp *comp;
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_client *client;
	bool is_frame_mode;

	mtk_crtc = container_of(ws, struct mtk_drm_crtc, delay_set_brightness_work);
	crtc = &mtk_crtc->base;
	comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!(comp && comp->funcs && comp->funcs->io_cmd))
		return;

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s: skip, slept\n", __func__);
		return;
	}
	brightness_now = level_pending;
	if (level_pending == 0) {
		DDPPR_ERR("%s: invalid level: %d\n", __func__, level_pending);
		return;
	}

	mtk_drm_trace_begin("setbacklight delay: %d", level_pending);
	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	DDPINFO("%s: %d\n", __func__, level_pending);

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	/* setHBM would use VM CMD in  DSI VDO mode only */
	client = (is_frame_mode || mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] == NULL) ?
		mtk_crtc->gce_obj.client[CLIENT_CFG] : mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];
	cmdq_handle =
		cmdq_pkt_create(client);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	/*
	if (comp && mtk_ddp_comp_get_type(comp->id) == MTK_DSI) {
		struct mtk_dsi *dsi = container_of(comp, struct mtk_dsi, ddp_comp);

		if (dsi->ext && dsi->ext->params && dsi->ext->params->wait_before_hbm && en) {
			mtk_drm_crtc_hbm_delay(crtc);
		}
	}
	*/

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL, &level_pending);

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	mtk_drm_trace_end();
}
//@}

int mtk_drm_setbacklight_grp(struct drm_crtc *crtc, unsigned int level,
	unsigned int panel_ext_param, unsigned int cfg_flag)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	bool is_frame_mode;
	struct mtk_bl_ext_config bl_ext_config;
	int index = drm_crtc_index(crtc);

	CRTC_MMP_EVENT_START(index, backlight_grp, (unsigned long)crtc,
			level);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s:%d, crtc is slept\n", __func__,
				__LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		CRTC_MMP_EVENT_END(index, backlight_grp, 0, 0);

		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(crtc);
	cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);


	if ((cfg_flag & (0x1<<SET_BACKLIGHT_LEVEL)) && !(cfg_flag & (0x1<<SET_ELVSS_PN))) {
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL_GRP, &level);
	} else {

		bl_ext_config.cfg_flag = cfg_flag;
		bl_ext_config.backlight_level = level;
		bl_ext_config.elvss_pn = panel_ext_param;
		if (comp && comp->funcs && comp->funcs->io_cmd)
			comp->funcs->io_cmd(comp, cmdq_handle, DSI_SET_BL_ELVSS, &bl_ext_config);

	}

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	CRTC_MMP_EVENT_END(index, backlight_grp, (unsigned long)crtc,
			level);

	return 0;
}

static void mtk_drm_crtc_wk_lock(struct drm_crtc *crtc, bool get,
	const char *func, int line);

int mtk_drm_aod_setbacklight(struct drm_crtc *crtc, unsigned int level)
{

	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp, *comp;
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct cmdq_pkt *cmdq_handle;
	bool is_frame_mode;
	struct cmdq_client *client;
	int i, j;
	struct mtk_crtc_state *crtc_state;

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	crtc_state = to_mtk_crtc_state(crtc->state);
	if (mtk_crtc->enabled && !crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		DDPINFO("%s:%d, crtc is on and not in doze mode\n",
			__func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

		return -EINVAL;
	}

	CRTC_MMP_EVENT_START(0, backlight, 0x123,
			level);
	mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);

		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -ENODEV;
	}

	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	if (!mtk_crtc->enabled) {
		/* 1. power on mtcmos */
		mtk_drm_top_clk_prepare_enable(crtc->dev);

		/*APSRC control*/
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, false, false, crtc_id, true);

		cmdq_mbox_enable(client->chan);
		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);

		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_start_trig_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_ENABLE, NULL);

		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_dump_analysis(comp);
	}

	/* send LCM CMD */
	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	if (is_frame_mode || mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] == NULL)
		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);
	else
		cmdq_handle =
			cmdq_pkt_create(
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_FIRST_PATH, 0);
	/* set backlight */
	if (output_comp->funcs && output_comp->funcs->io_cmd)
		output_comp->funcs->io_cmd(output_comp,
			cmdq_handle, DSI_SET_BL_AOD, &level);

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	if (!mtk_crtc->enabled) {

		if (mtk_crtc_with_trigger_loop(crtc))
			mtk_crtc_stop_trig_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_stop_event_loop(crtc);

		mtk_ddp_comp_io_cmd(output_comp, NULL, CONNECTOR_DISABLE, NULL);

		cmdq_mbox_disable(client->chan);
		mtk_drm_top_clk_disable_unprepare(crtc->dev);
	}

	mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);
	CRTC_MMP_EVENT_END(0, backlight, 0x123,
			level);

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

int mtk_drm_aod_scp_get_dsi_ulps_wakeup_prd(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	unsigned int ulps_wakeup_prd = 0;

	if (!(comp && comp->funcs && comp->funcs->io_cmd))
		return -EINVAL;

	comp->funcs->io_cmd(comp, NULL, DSI_AOD_SCP_GET_DSI_PARAM, &ulps_wakeup_prd);
	return ulps_wakeup_prd;
}

extern unsigned long long last_te_time;
/*
int mtk_drm_crtc_hbm_delay(struct drm_crtc *crtc)
{
	int fps = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	unsigned long long frame_interval = 1000 * 1000 * 1000 / fps;
	unsigned long long cur_time = sched_clock();
	long long to_best_time = last_te_time + frame_interval * 4 / 5 - cur_time;
	long long sleep_time = 0;

	if (to_best_time > 1 * 1000 * 1000) {
		sleep_time = to_best_time;
	} else if (to_best_time < -1000 * 1000) {
		sleep_time = to_best_time + frame_interval;
	}

	DDPINFO("%s:set hbm frame_interval=%lld, to_best_time=%lld, sleep_time=%lld, fps=%d\n",
		__func__,frame_interval, to_best_time, sleep_time, fps);

	if (sleep_time > 0 && sleep_time < frame_interval) {
		do_div(sleep_time, 1000);
	} else {
		sleep_time = 0;
	}

	DDPINFO("%s:set hbm sleep=%lld\n", __func__, sleep_time);

	if (sleep_time > 0) {
		usleep_range(sleep_time-500, sleep_time);
	}

	return 0;
}
*/

int mtk_drm_crtc_set_panel_hbm(struct drm_crtc *crtc, bool en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_client *client;
	bool is_frame_mode;
	bool state = false;
	int fps = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	struct mtk_crtc_state *mtk_state = to_mtk_crtc_state(crtc->state);

	if (!(comp && comp->funcs && comp->funcs->io_cmd))
		return -EINVAL;

	comp->funcs->io_cmd(comp, NULL, DSI_HBM_GET_STATE, &state);
	if (state == en)
		return 0;

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s: skip, slept\n", __func__);
		return -EINVAL;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	DDPINFO("%s:set LCM hbm en:%d\n", __func__, en);

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base);

	/* setHBM would use VM CMD in  DSI VDO mode only */
	client = (is_frame_mode || mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] == NULL) ?
		mtk_crtc->gce_obj.client[CLIENT_CFG] : mtk_crtc->gce_obj.client[CLIENT_DSI_CFG];
	cmdq_handle =
		cmdq_pkt_create(client);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	/*Wait TE, then set hbm cmd*/
	if (!mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE]) {
		comp->funcs->io_cmd(comp, NULL, DSI_HBM_WAIT, NULL);
		if (fps == 90) {
			usleep_range(5500, 6000);
		}

		if (fps == 60) {
			usleep_range(8400, 8900);
		}
	}

	/*
	if (comp && mtk_ddp_comp_get_type(comp->id) == MTK_DSI) {
		struct mtk_dsi *dsi = container_of(comp, struct mtk_dsi, ddp_comp);

		if (dsi->ext && dsi->ext->params && dsi->ext->params->wait_before_hbm && en) {
			mtk_drm_crtc_hbm_delay(crtc);
		}
	}
	*/

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);

	if (is_frame_mode) {
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	}

	comp->funcs->io_cmd(comp, cmdq_handle, DSI_HBM_SET, &en);

	if (is_frame_mode) {
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	return 0;
}

int mtk_drm_crtc_hbm_wait(struct drm_crtc *crtc, bool en)
{
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);
	bool wait = false;
	unsigned int wait_count = 0;

	if (!(comp && comp->funcs && comp->funcs->io_cmd))
		return -EINVAL;

	comp->funcs->io_cmd(comp, NULL, DSI_HBM_GET_WAIT_STATE, &wait);
	if (wait != true)
		return 0;

	if (!panel_ext)
		return -EINVAL;

	wait_count = en ? panel_ext->hbm_en_time : panel_ext->hbm_dis_time;

	DDPINFO("LCM hbm %s wait %u-TE\n", en ? "enable" : "disable",
		wait_count);

	while (wait_count) {
		mtk_drm_idlemgr_kick(__func__, crtc, 0);
		wait_count--;
		comp->funcs->io_cmd(comp, NULL, DSI_HBM_WAIT, NULL);
	}

	wait = false;
	comp->funcs->io_cmd(comp, NULL, DSI_HBM_SET_WAIT_STATE, &wait);

	return 0;
}

void mtk_drm_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *drm = crtc->dev;
	unsigned int pipe = crtc->index;
	struct mtk_drm_private *priv = drm->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(priv->crtc[pipe]);
	struct mtk_ddp_comp *comp = mtk_crtc_get_comp(&mtk_crtc->base, mtk_crtc->ddp_mode, 0, 0);

	DDPINFO("%s\n", __func__);

	mtk_crtc->vblank_en = 0;

	CRTC_MMP_MARK((int) pipe, disable_vblank, (unsigned long)comp,
			(unsigned long)&mtk_crtc->base);
}

bool mtk_crtc_get_vblank_timestamp(struct drm_crtc *crtc,
				 int *max_error,
				 ktime_t *vblank_time,
				 bool in_vblank_irq)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	*vblank_time = timespec64_to_ktime(mtk_crtc->vblank_time);
	return true;
}

#define MTK_PCLK_2K60 241500

/*dp 4k resolution 3840*2160*/
bool mtk_crtc_is_dual_pipe(struct drm_crtc *crtc)
{
	struct mtk_panel_params *panel_ext =
		mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if ((drm_crtc_index(crtc) == 0) &&
		panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DUAL_PORT) {
		return true;
	}

	if (drm_crtc_index(crtc) == 1) {
		if ((crtc->state->adjusted_mode.hdisplay == 1920*2) ||
			(drm_mode_vrefresh(&crtc->state->adjusted_mode) == 120) ||
			(crtc->state->adjusted_mode.clock >= MTK_PCLK_2K60)) {
			DDPFUNC();
			return true;
		}
	}

	if (drm_crtc_index(crtc) == 3 &&
		mtk_crtc && mtk_crtc->path_data->is_discrete_path)
		return true;

	if (((drm_crtc_index(crtc) == 0) || (drm_crtc_index(crtc) == 3)) &&
		mtk_drm_helper_get_opt(priv->helper_opt,
			    MTK_DRM_OPT_PRIM_DUAL_PIPE) &&
		panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT &&
		panel_ext->dsc_params.slice_mode == 1) {
		return true;
	}

	return false;
}

void mtk_crtc_prepare_dual_pipe_path(struct device *dev, struct mtk_drm_private *priv,
	struct mtk_drm_crtc *mtk_crtc)
{
	int i, j;
	enum mtk_ddp_comp_id comp_id;
	struct mtk_ddp_comp *comp;

	for (i = 0; i < DDP_PATH_NR; i++) {
		mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i] =
			mtk_crtc->path_data->dual_ovl_path_len[i];
		mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[i] = devm_kmalloc_array(
			dev, mtk_crtc->path_data->dual_ovl_path_len[i],
			sizeof(struct mtk_ddp_comp *), GFP_KERNEL);
		mtk_crtc->dual_pipe_ddp_ctx.ddp_comp_nr[i] =
			mtk_crtc->path_data->dual_path_len[i];
		mtk_crtc->dual_pipe_ddp_ctx.ddp_comp[i] = devm_kmalloc_array(
			dev, mtk_crtc->path_data->dual_path_len[i],
			sizeof(struct mtk_ddp_comp *), GFP_KERNEL);
		DDPDBG("i:%d,com_nr:%d,path_len:%d\n",
			i, mtk_crtc->dual_pipe_ddp_ctx.ddp_comp_nr[i],
			mtk_crtc->path_data->dual_path_len[i]);
	}

	for_each_comp_id_in_dual_pipe(comp_id, mtk_crtc->path_data, i, j) {
		DDPDBG("prepare comp id in dual pipe %s, type=%d\n",
				mtk_dump_comp_str_id(comp_id),
				mtk_ddp_comp_get_type(comp_id));
		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL ||
			mtk_ddp_comp_get_type(comp_id) == MTK_DISP_PWM) {
			struct mtk_ddp_comp *comp;

			comp = kzalloc(sizeof(*comp), GFP_KERNEL);
			if (!comp) {
				DDPPR_ERR("%s allocate comp fail\n", __func__);
				return;
			}
			comp->id = comp_id;
			if (mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i] &&
					j < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i]) {
				mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[i][j] = comp;
			} else {
			/* shift comp_idx from global idx to local idx for ddp_comp array */
				unsigned int ovl_comp_nr;

				ovl_comp_nr = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i];
				mtk_crtc->dual_pipe_ddp_ctx.ddp_comp[i]
					[j - ovl_comp_nr] = comp;
			}
			continue;
		}

		comp = priv->ddp_comp[comp_id];
		if (!comp) {
			DDPPR_ERR("%s: Invalid comp_id:%d\n", __func__, comp_id);
			return;
		}

		if (mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i] &&
				j < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i])
			mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[i][j] = comp;
		else
			/* shift comp_idx from global idx to local idx for ddp_comp array */
			mtk_crtc->dual_pipe_ddp_ctx.ddp_comp[i]
				[j - mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i]] = comp;
	}
}

void mtk_crtc_prepare_dual_pipe(struct mtk_drm_crtc *mtk_crtc)
{
	int i, j;
	enum mtk_ddp_comp_id comp_id;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct drm_crtc *crtc = &mtk_crtc->base;
	int en = 1;

	if (mtk_crtc_is_dual_pipe(&(mtk_crtc->base))) {
		mtk_crtc->is_dual_pipe = true;

		if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_MMDVFS_SUPPORT)) {
			if (drm_crtc_index(&mtk_crtc->base) == 1) {
				mtk_drm_set_mmclk(&mtk_crtc->base, 3, false, __func__);
				//DDPFUNC("current freq: %d\n",
				//pm_qos_request(PM_QOS_DISP_FREQ));
			}
		}
	} else {
		mtk_crtc->is_dual_pipe = false;

		if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_MMDVFS_SUPPORT)) {
			if (drm_crtc_index(&mtk_crtc->base) == 1)
				mtk_drm_set_mmclk(&mtk_crtc->base, 0, false, __func__);
			//DDPFUNC("crtc%d single pipe current freq: %d\n",
			//	drm_crtc_index(&mtk_crtc->base),
			//	pm_qos_request(PM_QOS_DISP_FREQ));
		}

		return;
	}

	for_each_comp_id_in_dual_pipe(comp_id, mtk_crtc->path_data, i, j) {
		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL ||
			mtk_ddp_comp_get_type(comp_id) == MTK_DISP_PWM) {
			continue;
		} else if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_DSC) {
			/*4k 30 use DISP_MERGE1, 4k 60 use DSC*/
			//to do: dp in 6983 4k60 can use merge, only 8k30 must use dsc
			if ((drm_crtc_index(&mtk_crtc->base) == 1) &&
				(drm_mode_vrefresh(&crtc->state->adjusted_mode) == 30)) {
				comp = priv->ddp_comp[DDP_COMPONENT_MERGE1];
				if (mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i] &&
						j < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i]) {
					mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[i][j] = comp;
				} else {
					unsigned int dual_ovl_comp_nr =
						mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[i];
				/* shift comp_idx from global idx to local idx for ddp_comp array */
					mtk_crtc->dual_pipe_ddp_ctx.ddp_comp[i]
						[j - dual_ovl_comp_nr] = comp;
				}
				comp->mtk_crtc = mtk_crtc;
			}
			continue;
		}
		comp = priv->ddp_comp[comp_id];
		if (!comp) {
			DDPPR_ERR("%s: Invalid comp_id:%d\n", __func__, comp_id);
			return;
		}
		comp->mtk_crtc = mtk_crtc;
	}
	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (comp)
		mtk_ddp_comp_io_cmd(comp, NULL, SET_MMCLK_BY_DATARATE, &en);

	if (drm_crtc_index(&mtk_crtc->base) == 0)
		mtk_mmqos_is_dualpipe_enable(mtk_crtc->is_dual_pipe);
}

static void user_cmd_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

bool mtk_crtc_in_dual_pipe(struct mtk_drm_crtc *mtk_crtc, struct mtk_ddp_comp *comp)
{
	int i = 0, j = 0;
	struct mtk_ddp_comp *comp_dual;

	for_each_comp_in_dual_pipe(comp_dual, mtk_crtc, i, j) {
		if (comp->id == comp_dual->id)
			return true;
	}
	return false;
}

int mtk_crtc_user_cmd(struct drm_crtc *crtc, struct mtk_ddp_comp *comp,
		unsigned int cmd, void *params)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_cmdq_cb_data *cb_data;
	static unsigned int user_cmd_cnt;
	struct DRM_DISP_CCORR_COEF_T *ccorr_config;

	int index = 0;

	if (!mtk_crtc) {
		DDPPR_ERR("%s:%d, invalid crtc:0x%p\n",
				__func__, __LINE__, crtc);
		return -1;
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	CRTC_MMP_EVENT_START(index, user_cmd, (unsigned long)crtc,
			(unsigned long)comp);

	if ((!crtc) || (!comp)) {
		DDPPR_ERR("%s:%d, invalid arg:(0x%p,0x%p)\n",
				__func__, __LINE__,
				crtc, comp);
		CRTC_MMP_MARK(index, user_cmd, 0, 0);
		goto err;
	}

	index = drm_crtc_index(crtc);
	if (index) {
		DDPPR_ERR("%s:%d, invalid crtc:0x%p, index:%d\n",
				__func__, __LINE__, crtc, index);
		CRTC_MMP_MARK(index, user_cmd, 0, 1);
		goto err;
	}

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s:%d, slepted\n", __func__, __LINE__);
		CRTC_MMP_EVENT_END(index, user_cmd, 0, 2);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return 1;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("cb data creation failed\n");
		CRTC_MMP_MARK(index, user_cmd, 0, 3);
		goto err;
	}

	CRTC_MMP_MARK(index, user_cmd, comp->id, cmd);
	cmdq_handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	CRTC_MMP_MARK(index, user_cmd, user_cmd_cnt, (unsigned long)cmdq_handle);

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
			DDP_SECOND_PATH, 0);
	else {
		if (mtk_crtc->is_dual_pipe) {
			if (!mtk_crtc_in_dual_pipe(mtk_crtc, comp))
				mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,
								DDP_FIRST_PATH, 0);
		} else {
			mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle,	DDP_FIRST_PATH, 0);
		}
	}
	CRTC_MMP_MARK(index, user_cmd, user_cmd_cnt, 2);

	/* set user command */
	if (comp && comp->funcs && comp->funcs->user_cmd && !comp->blank_mode)
		comp->funcs->user_cmd(comp, cmdq_handle, cmd, (void *)params);
	else {
		DDPPR_ERR("%s:%d, invalid comp:(0x%p,0x%p)\n",
				__func__, __LINE__, comp, comp->funcs);
		CRTC_MMP_MARK(index, user_cmd, 0, 4);
		goto err2;
	}
	CRTC_MMP_MARK(index, user_cmd, user_cmd_cnt, 3);

	/* add counter to check update frequency */
	CRTC_MMP_MARK(index, user_cmd, user_cmd_cnt, 4);
	user_cmd_cnt++;

	if (m_new_pq_persist_property[DISP_PQ_CCORR_SILKY_BRIGHTNESS]) {
		if (((comp->id == DDP_COMPONENT_CCORR1) ||
			((drm_ccorr_caps.ccorr_linear & 0x1) &&
			(comp->id == DDP_COMPONENT_CCORR0))) && cmd == 0) {
			ccorr_config = params;
			if (ccorr_config->silky_bright_flag == 1 &&
				ccorr_config->FinalBacklight != sb_backlight) {
				sb_cmdq_handle = cmdq_handle;
				kfree(cb_data);
			} else {
				cb_data->crtc = crtc;
				cb_data->cmdq_handle = cmdq_handle;
				if (cmdq_pkt_flush_threaded(cmdq_handle,
					user_cmd_cmdq_cb, cb_data) < 0)
					DDPPR_ERR("failed to flush user_cmd\n");
			}
		} else {
			cb_data->crtc = crtc;
			cb_data->cmdq_handle = cmdq_handle;
			if (cmdq_pkt_flush_threaded(cmdq_handle,
				user_cmd_cmdq_cb, cb_data) < 0)
				DDPPR_ERR("failed to flush user_cmd\n");
		}
	} else {
		cb_data->crtc = crtc;
		cb_data->cmdq_handle = cmdq_handle;
		if (cmdq_pkt_flush_threaded(cmdq_handle, user_cmd_cmdq_cb, cb_data) < 0)
			DDPPR_ERR("failed to flush user_cmd\n");
	}

	CRTC_MMP_EVENT_END(index, user_cmd, (unsigned long)cmd,
			(unsigned long)params);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;

err:
	CRTC_MMP_EVENT_END(index, user_cmd, 0, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return -1;

err2:
	kfree(cb_data);
	CRTC_MMP_EVENT_END(index, user_cmd, 0, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return -1;
}

/* power on all modules on this CRTC */
void mtk_crtc_ddp_prepare(struct mtk_drm_crtc *mtk_crtc)
{
	int i, j, k;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_path_data *addon_path;
	enum addon_module module;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_panel_params *panel_ext =
	    mtk_drm_get_lcm_ext_params(crtc);
	unsigned int comp_id;

	for_each_comp_id_target_mode_path_in_path_data(comp_id, mtk_crtc->path_data, i,
			 mtk_crtc->ddp_mode, 0) {
		comp = priv->ddp_comp[comp_id];
		mtk_ddp_comp_prepare(comp);
	}

	for (i = 0; i < ADDON_SCN_NR; i++) {
		addon_data = mtk_addon_get_scenario_data(__func__,
							 &mtk_crtc->base, i);
		if (!addon_data)
			break;

		for (j = 0; j < addon_data->module_num; j++) {
			module = addon_data->module_data[j].module;
			addon_path = mtk_addon_module_get_path(module);

			for (k = 0; k < addon_path->path_len; k++) {
				comp = priv->ddp_comp[addon_path->path[k]];
				mtk_ddp_comp_prepare(comp);
			}
		}
	}
	if (panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		if (drm_crtc_index(crtc) == 3)
			comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		mtk_ddp_comp_clk_prepare(comp);
	}
	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_id_in_dual_pipe(comp_id, mtk_crtc->path_data, i, j) {
			comp = priv->ddp_comp[comp_id];
			mtk_ddp_comp_clk_prepare(comp);
			mtk_ddp_comp_prepare(comp);
		}
		for (i = 0; i < ADDON_SCN_NR; i++) {
			addon_data = mtk_addon_get_scenario_data_dual(__func__,
								 &mtk_crtc->base, i);
			if (!addon_data)
				break;

			for (j = 0; j < addon_data->module_num; j++) {
				module = addon_data->module_data[j].module;
				addon_path = mtk_addon_module_get_path(module);

				for (k = 0; k < addon_path->path_len; k++) {
					comp = priv->ddp_comp[addon_path->path[k]];
					mtk_ddp_comp_prepare(comp);
				}
			}

		}
	}
	/*TODO , Move to prop place rdma4/5 VDE from DP_VDE*/
	if (drm_crtc_index(crtc) == 1)
		writel_relaxed(0x220000, mtk_crtc->config_regs + 0xE10);
}

void mtk_crtc_ddp_unprepare(struct mtk_drm_crtc *mtk_crtc)
{
	int i, j, k;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_path_data *addon_path;
	enum addon_module module;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_panel_params *panel_ext =
	    mtk_drm_get_lcm_ext_params(crtc);
	unsigned int comp_id;

	for_each_comp_id_target_mode_path_in_path_data(comp_id, mtk_crtc->path_data, i,
			 mtk_crtc->ddp_mode, 0) {
		comp = priv->ddp_comp[comp_id];
		mtk_ddp_comp_unprepare(comp);
	}

	for (i = 0; i < ADDON_SCN_NR; i++) {
		addon_data = mtk_addon_get_scenario_data(__func__,
							 &mtk_crtc->base, i);
		if (!addon_data)
			break;

		for (j = 0; j < addon_data->module_num; j++) {
			module = addon_data->module_data[j].module;
			addon_path = mtk_addon_module_get_path(module);

			for (k = 0; k < addon_path->path_len; k++) {
				comp = priv->ddp_comp[addon_path->path[k]];
				mtk_ddp_comp_unprepare(comp);
			}
		}
	}
	if (panel_ext && panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		if (drm_crtc_index(crtc) == 3)
			comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		mtk_ddp_comp_clk_unprepare(comp);
	}

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_id_in_dual_pipe(comp_id, mtk_crtc->path_data, i, j) {
			comp = priv->ddp_comp[comp_id];
			mtk_ddp_comp_clk_unprepare(comp);
			mtk_ddp_comp_unprepare(comp);
		}

		for (i = 0; i < ADDON_SCN_NR; i++) {
			addon_data = mtk_addon_get_scenario_data_dual(__func__,
								 &mtk_crtc->base, i);
			if (!addon_data)
				break;

			for (j = 0; j < addon_data->module_num; j++) {
				module = addon_data->module_data[j].module;
				addon_path = mtk_addon_module_get_path(module);

				for (k = 0; k < addon_path->path_len; k++) {
					comp = priv->ddp_comp[addon_path->path[k]];
					mtk_ddp_comp_unprepare(comp);
				}
			}
		}
	}
	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_MMDVFS_SUPPORT)) {
		/*restore default mm freq, PM_QOS_MM_FREQ_DEFAULT_VALUE*/
		if (drm_crtc_index(&mtk_crtc->base) == 1)
			mtk_drm_set_mmclk(&mtk_crtc->base, -1, false, __func__);
	}
}
struct drm_framebuffer *mtk_drm_framebuffer_lookup(struct drm_device *dev,
	unsigned int id);
#ifdef MTK_DRM_ADVANCE
static struct mtk_ddp_comp *
mtk_crtc_get_plane_comp(struct drm_crtc *crtc,
			struct mtk_plane_state *plane_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = NULL;
	int i, j;

	if (plane_state->comp_state.comp_id == 0)
		return mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, 0, 0);

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i,
				       j)
		if (comp->id == plane_state->comp_state.comp_id) {
			DDPINFO("%s i:%d, ovl_comp_id:%d\n",
				__func__, i,
				plane_state->comp_state.comp_id);
			DDPINFO("lye_id:%d, ext_lye_id:%d\n",
				plane_state->comp_state.lye_id,
				plane_state->comp_state.ext_lye_id);
			return comp;
		}

	return comp;
}

static int mtk_crtc_get_dc_fb_size(struct drm_crtc *crtc)
{
	/* DC buffer color format is RGB888 */
	return crtc->state->adjusted_mode.vdisplay *
	       crtc->state->adjusted_mode.hdisplay * 3;
}

static void mtk_crtc_cwb_set_sec(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_cwb_info *cwb_info;
	struct drm_framebuffer *fb;
	int i;

	cwb_info = mtk_crtc->cwb_info;

	if (!cwb_info)
		return;

	cwb_info->is_sec = false;
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;

		fb = plane->state->fb;
		if (mtk_drm_fb_is_secure(fb)) {
			DDPDBG("[capture] plane%d has secure content !!", i);
			cwb_info->is_sec = true;
		}
	}
}

static void calc_mml_config(struct drm_crtc *crtc,
	union mtk_addon_config *addon_config,
	struct mtk_crtc_state *crtc_state)
{
	struct mtk_ddp_comp *comp = NULL;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	addon_config->addon_mml_config.mml_dst_roi[0] = crtc_state->mml_dst_roi_dual[0];
	addon_config->addon_mml_config.mml_dst_roi[1] = crtc_state->mml_dst_roi_dual[1];
	comp = priv->ddp_comp[DDP_COMPONENT_MML_MML0];
	if (addon_config->addon_mml_config.submit.info.mode != MML_MODE_DIRECT_LINK)
		mtk_ddp_comp_mml_calc_cfg(comp, addon_config);

	addon_config->addon_mml_config.mml_dst_roi[1].x = 0; // -= (mid_line - to_right)

	crtc_state->mml_src_roi[0] = addon_config->addon_mml_config.mml_src_roi[0];
	crtc_state->mml_dst_roi_dual[0] = addon_config->addon_mml_config.mml_dst_roi[0];
	DDPINFO("%s dual:src[0](%d,%d,%d,%d), dst[0](%d,%d,%d,%d)\n", __func__,
		addon_config->addon_mml_config.mml_src_roi[0].x,
		addon_config->addon_mml_config.mml_src_roi[0].y,
		addon_config->addon_mml_config.mml_src_roi[0].width,
		addon_config->addon_mml_config.mml_src_roi[0].height,
		addon_config->addon_mml_config.mml_dst_roi[0].x,
		addon_config->addon_mml_config.mml_dst_roi[0].y,
		addon_config->addon_mml_config.mml_dst_roi[0].width,
		addon_config->addon_mml_config.mml_dst_roi[0].height);

	if (mtk_crtc->is_dual_pipe) {
		crtc_state->mml_src_roi[1] = addon_config->addon_mml_config.mml_src_roi[1];
		crtc_state->mml_dst_roi_dual[1] = addon_config->addon_mml_config.mml_dst_roi[1];
		DDPINFO("%s dual:src[1](%d,%d,%d,%d), dst[1](%d,%d,%d,%d)\n", __func__,
			addon_config->addon_mml_config.mml_src_roi[1].x,
			addon_config->addon_mml_config.mml_src_roi[1].y,
			addon_config->addon_mml_config.mml_src_roi[1].width,
			addon_config->addon_mml_config.mml_src_roi[1].height,
			addon_config->addon_mml_config.mml_dst_roi[1].x,
			addon_config->addon_mml_config.mml_dst_roi[1].y,
			addon_config->addon_mml_config.mml_dst_roi[1].width,
			addon_config->addon_mml_config.mml_dst_roi[1].height);
	}
}

static void mml_addon_module_connect(struct drm_crtc *crtc, unsigned int ddp_mode,
				     const struct mtk_addon_module_data *addon_module,
				     const struct mtk_addon_module_data *addon_module_dual,
				     union mtk_addon_config *addon_config,
				     struct cmdq_pkt *cmdq_handle)
{
	struct mtk_ddp_comp *output_comp = NULL;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	int i = 0;
	struct mml_job _job;
	struct mml_pq_param _pq_param[MML_MAX_OUTPUTS];
	struct mtk_addon_mml_config *c = &addon_config->addon_mml_config;
	const struct mtk_addon_module_data *m[] = {addon_module, addon_module_dual};

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!mtk_crtc->is_mml || (mtk_crtc->mml_cfg_pq == NULL)) {
		DDPMSG("%s:%d we do submit mml, but layering rule happened\n", __func__, __LINE__);
		return;
	}

	if (!output_comp) {
		DDPPR_ERR("%s:%d output_comp is NULL\n", __func__, __LINE__);
		return;
	}

	c->config_type.type = ADDON_CONNECT;
	c->dual = mtk_crtc->is_dual_pipe;
	c->mutex.sof_src = (int)output_comp->id;
	c->mutex.eof_src = (int)output_comp->id;
	c->mutex.is_cmd_mode = mtk_crtc_is_frame_trigger_mode(crtc);
	c->is_entering = mtk_crtc->mml_ir_state == MML_IR_ENTERING ? true : false;

	c->submit.job = &_job;
	for (i = 0; i < MML_MAX_OUTPUTS; ++i)
		c->submit.pq_param[i] = &_pq_param[i];
	copy_mml_submit(mtk_crtc->mml_cfg_pq, &(c->submit));

	/* call mml_calc_cfg to calc how to split for rsz dual pipe */
	calc_mml_config(crtc, addon_config, crtc_state);

	for (i = 0; i <= (mtk_crtc->is_dual_pipe && addon_module_dual); ++i) {
		c->pipe = i;
		if (addon_module->type == ADDON_BETWEEN) {
			c->is_yuv = MML_FMT_IS_YUV(c->submit.info.src.format);
			mtk_addon_connect_between(crtc, ddp_mode, m[i], addon_config, cmdq_handle);
		} else if (addon_module->type == ADDON_EMBED) {
			c->is_yuv = false; /* use Y2R inside OVL instead */
			mtk_addon_connect_embed(crtc, ddp_mode, m[i], addon_config, cmdq_handle);
		}
	}

	CRTC_MMP_MARK(0, mml_dbg, (unsigned long)cmdq_handle, MMP_ADDON_CONNECT);
}

static void mml_addon_module_disconnect(struct drm_crtc *crtc,
	unsigned int ddp_mode,
	const struct mtk_addon_module_data *addon_module,
	const struct mtk_addon_module_data *addon_module_dual,
	union mtk_addon_config *addon_config,
	struct cmdq_pkt *cmdq_handle)
{
	int w = crtc->state->adjusted_mode.hdisplay;
	int h = crtc->state->adjusted_mode.vdisplay;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct mtk_rect ovl_roi = {0, 0, w, h};
	unsigned int i = 0;
	const struct mtk_addon_module_data *m[] = {addon_module, addon_module_dual};

	addon_config->addon_mml_config.config_type.type = ADDON_DISCONNECT;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp &&
		drm_crtc_index(crtc) == 0) {
		ovl_roi.width = mtk_ddp_comp_io_cmd(
			output_comp, NULL,
			DSI_GET_VIRTUAL_WIDTH, NULL);
		ovl_roi.height = mtk_ddp_comp_io_cmd(
			output_comp, NULL,
			DSI_GET_VIRTUAL_HEIGH, NULL);
	}

	if (mtk_crtc->is_dual_pipe)
		ovl_roi.width /= 2;

	addon_config->addon_mml_config.mml_src_roi[0] = ovl_roi;
	addon_config->addon_mml_config.mml_dst_roi[0] = ovl_roi;

	if (mtk_crtc->is_dual_pipe) {
		addon_config->addon_mml_config.mml_src_roi[1] = ovl_roi;
		addon_config->addon_mml_config.mml_dst_roi[1] = ovl_roi;
	}

	mtk_crtc_attach_addon_path_comp(crtc, addon_module, true);

	for (i = 0; i <= (mtk_crtc->is_dual_pipe && addon_module_dual); ++i) {
		addon_config->addon_mml_config.pipe = i;
		if (addon_module->type == ADDON_BETWEEN)
			mtk_addon_disconnect_between(crtc, ddp_mode, m[i], addon_config,
						     cmdq_handle);
		else if (addon_module->type == ADDON_EMBED)
			mtk_addon_disconnect_embed(crtc, ddp_mode, m[i], addon_config, cmdq_handle);
	}

	CRTC_MMP_MARK(0, mml_dbg, (unsigned long)cmdq_handle, MMP_ADDON_DISCONNECT);
}

int get_addon_path_wait_event(struct drm_crtc *crtc,
			const struct mtk_addon_path_data *path_data)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = NULL;
	int i;

	for (i = 0; i < path_data->path_len; i++) {
		if (mtk_ddp_comp_get_type(path_data->path[i]) ==
		    MTK_DISP_VIRTUAL)
			continue;
		comp = priv->ddp_comp[path_data->path[i]];
		if (mtk_ddp_comp_is_output(comp))
			break;
	}

	if (!comp) {
		DDPPR_ERR("%s, Cannot find output component\n", __func__);
		return -EINVAL;
	}

	if (comp->id == DDP_COMPONENT_WDMA0)
		return mtk_crtc->gce_obj.event[EVENT_WDMA0_EOF];
	else if (comp->id == DDP_COMPONENT_OVLSYS_WDMA0)
		return mtk_crtc->gce_obj.event[EVENT_OVLSYS_WDMA0_EOF];
	else if (comp->id == DDP_COMPONENT_OVLSYS_WDMA1)
		return mtk_crtc->gce_obj.event[EVENT_OVLSYS_WDMA1_EOF];

	DDPPR_ERR("The output component has not frame done event\n");
	return -EINVAL;
}

int mtk_crtc_wb_addon_get_event(struct drm_crtc *crtc)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_module_data *addon_module;
	const struct mtk_addon_path_data *path_data;
	int index = drm_crtc_index(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	enum addon_scenario scn;
	int gce_event;

	if (index != 0 || mtk_crtc_is_dc_mode(crtc))
		return -EINVAL;

	if (state->prop_val[CRTC_PROP_OUTPUT_SCENARIO] == 0)
		scn = WDMA_WRITE_BACK_OVL;
	else
		scn = WDMA_WRITE_BACK;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc, scn);
	if (!addon_data)
		return -EINVAL;

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		path_data = mtk_addon_module_get_path(addon_module->module);

		gce_event = get_addon_path_wait_event(crtc, path_data);
		if (gce_event < 0)
			continue;
		return gce_event;
	}
	return -EINVAL;
}

void _mtk_crtc_wb_addon_module_disconnect(
	struct drm_crtc *crtc, unsigned int ddp_mode,
	struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	union mtk_addon_config addon_config;
	int index = drm_crtc_index(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	enum addon_scenario scn;

	if (index != 0 || mtk_crtc_is_dc_mode(crtc))
		return;

	if (state->prop_val[CRTC_PROP_OUTPUT_SCENARIO] == 0)
		scn = WDMA_WRITE_BACK_OVL;
	else
		scn = WDMA_WRITE_BACK;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc, scn);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual(__func__, crtc, scn);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if ((addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v2) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v3) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_OVLSYS_WDMA0)) {
			if (mtk_crtc->is_dual_pipe) {
				/* disconnect left pipe */
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
				/* disconnect right pipe */
				addon_module = &addon_data_dual->module_data[i];
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
			} else {
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
								  &addon_config,
								  cmdq_handle);
			}
		} else
			DDPPR_ERR("addon type1:%d + module:%d not support\n",
					  addon_module->type,
					  addon_module->module);
	}
}

static void _mtk_crtc_cwb_addon_module_disconnect(
	struct drm_crtc *crtc, unsigned int ddp_mode,
	struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	union mtk_addon_config addon_config;
	int index = drm_crtc_index(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_cwb_info *cwb_info;

	cwb_info = mtk_crtc->cwb_info;
	if (index != 0 || mtk_crtc_is_dc_mode(crtc) ||
		!cwb_info)
		return;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc,
						cwb_info->scn);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual
			(__func__, crtc, cwb_info->scn);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if ((addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v2) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v3) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_OVLSYS_WDMA0)) {
			if (mtk_crtc->is_dual_pipe) {
				/* disconnect left pipe */
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
				/* disconnect right pipe */
				addon_module = &addon_data_dual->module_data[i];
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
			} else {
				mtk_addon_disconnect_after(crtc, ddp_mode, addon_module,
								  &addon_config,
								  cmdq_handle);
			}
		} else
			DDPPR_ERR("addon type1:%d + module:%d not support\n",
					  addon_module->type,
					  addon_module->module);
	}
}

static void _mtk_crtc_lye_addon_module_disconnect(
	struct drm_crtc *crtc, unsigned int ddp_mode,
	struct mtk_lye_ddp_state *lye_state, struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	const struct mtk_addon_module_data *addon_module_dual;
	union mtk_addon_config addon_config;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					lye_state->scn[drm_crtc_index(crtc)]);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual
			(__func__, crtc, lye_state->scn[drm_crtc_index(crtc)]);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if (addon_module->type == ADDON_BETWEEN &&
		    (addon_module->module == DISP_MML_SRAM_ONLY)) {
		} else if (addon_module->module == DISP_MML_IR_PQ ||
			   addon_module->module == DISP_MML_IR_PQ_v2 ||
			   addon_module->module == DISP_MML_IR_PQ_v3) {
			if (mtk_crtc->is_dual_pipe)
				addon_module_dual = &(addon_data_dual->module_data[i]);
			else
				addon_module_dual = NULL;
			if (mtk_crtc->is_force_mml_scen)
				mml_addon_module_disconnect(crtc, ddp_mode, addon_module,
							    addon_module_dual, &addon_config,
							    cmdq_handle);
		} else if (addon_module->type == ADDON_BETWEEN &&
			   (addon_module->module == DISP_RSZ ||
			    addon_module->module == DISP_RSZ_v2 ||
			    addon_module->module == DISP_RSZ_v5)) {
			int w = crtc->state->adjusted_mode.hdisplay;
			int h = crtc->state->adjusted_mode.vdisplay;
			struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
			struct mtk_ddp_comp *output_comp;
			struct mtk_rect rsz_roi = {0, 0, w, h};

			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp &&
				drm_crtc_index(crtc) == 0) {
				rsz_roi.width = mtk_ddp_comp_io_cmd(
					output_comp, NULL,
					DSI_GET_VIRTUAL_WIDTH, NULL);
				rsz_roi.height = mtk_ddp_comp_io_cmd(
					output_comp, NULL,
					DSI_GET_VIRTUAL_HEIGH, NULL);
			}

			if (mtk_crtc->is_dual_pipe)
				rsz_roi.width /= 2;

			addon_config.addon_rsz_config.rsz_src_roi = rsz_roi;
			addon_config.addon_rsz_config.rsz_dst_roi = rsz_roi;
			addon_config.addon_rsz_config.lc_tgt_layer =
				lye_state->lc_tgt_layer;

			mtk_addon_disconnect_between(
				crtc, ddp_mode, addon_module, &addon_config,
				cmdq_handle);

			if (mtk_crtc->is_dual_pipe) {
				addon_module = &addon_data_dual->module_data[i];
				addon_config.config_type.module = addon_module->module;
				addon_config.config_type.type = addon_module->type;

				mtk_addon_disconnect_between(
					crtc, ddp_mode, addon_module, &addon_config,
					cmdq_handle);
			}
		} else if (addon_module->type == ADDON_EMBED &&
			   (addon_module->module == OVL_RSZ ||
			    addon_module->module == OVL_RSZ_1)) {
			int w = crtc->state->adjusted_mode.hdisplay;
			int h = crtc->state->adjusted_mode.vdisplay;
			struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
			struct mtk_ddp_comp *output_comp;
			struct mtk_rect rsz_roi = {0, 0, w, h};

			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp && drm_crtc_index(crtc) == 0) {
				rsz_roi.width = mtk_ddp_comp_io_cmd(output_comp, NULL,
								    DSI_GET_VIRTUAL_WIDTH, NULL);
				rsz_roi.height = mtk_ddp_comp_io_cmd(output_comp, NULL,
								     DSI_GET_VIRTUAL_HEIGH, NULL);
			}

			if (mtk_crtc->is_dual_pipe)
				rsz_roi.width /= 2;

			addon_config.addon_rsz_config.rsz_src_roi = rsz_roi;
			addon_config.addon_rsz_config.rsz_dst_roi = rsz_roi;
			addon_config.addon_rsz_config.lc_tgt_layer = lye_state->lc_tgt_layer;

			mtk_addon_disconnect_embed(crtc, ddp_mode, addon_module, &addon_config,
						   cmdq_handle);

			if (mtk_crtc->is_dual_pipe) {
				addon_module = &addon_data_dual->module_data[i];
				addon_config.config_type.module = addon_module->module;
				addon_config.config_type.type = addon_module->type;

				mtk_addon_disconnect_embed(crtc, ddp_mode, addon_module,
							   &addon_config, cmdq_handle);
			}
		} else
			DDPPR_ERR("addon type:%d + module:%d not support\n",
				  addon_module->type, addon_module->module);
	}
}

static void _mtk_crtc_atmoic_addon_module_disconnect(
	struct drm_crtc *crtc, unsigned int ddp_mode,
	struct mtk_lye_ddp_state *lye_state, struct cmdq_pkt *cmdq_handle)
{
	_mtk_crtc_cwb_addon_module_disconnect(
			crtc, ddp_mode, cmdq_handle);

	_mtk_crtc_lye_addon_module_disconnect(
			crtc, ddp_mode, lye_state, cmdq_handle);
}

void
_mtk_crtc_wb_addon_module_connect(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	union mtk_addon_config addon_config;
	int index = drm_crtc_index(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct total_tile_overhead to_info;
	enum addon_scenario scn;

	if (index != 0 || mtk_crtc_is_dc_mode(crtc) ||
		!state->prop_val[CRTC_PROP_OUTPUT_ENABLE])
		return;

	to_info = mtk_crtc_get_total_overhead(mtk_crtc);
	if (state->prop_val[CRTC_PROP_OUTPUT_SCENARIO] == 0)
		scn = WDMA_WRITE_BACK_OVL;
	else
		scn = WDMA_WRITE_BACK;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc, scn);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual(__func__, crtc, scn);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if ((addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v2) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v3) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_OVLSYS_WDMA0)) {
			struct mtk_rect src_roi = {0};
			struct mtk_rect dst_roi = {0};
			struct drm_framebuffer *fb;

			mtk_crtc_set_width_height(&(src_roi.width), &(src_roi.height),
				crtc, (scn == WDMA_WRITE_BACK));
			dst_roi.x = state->prop_val[CRTC_PROP_OUTPUT_X];
			dst_roi.y = state->prop_val[CRTC_PROP_OUTPUT_Y];
			dst_roi.width = state->prop_val[CRTC_PROP_OUTPUT_WIDTH];
			dst_roi.height = state->prop_val[CRTC_PROP_OUTPUT_HEIGHT];
			fb = mtk_drm_framebuffer_lookup(crtc->dev,
				state->prop_val[CRTC_PROP_OUTPUT_FB_ID]);
			/* get fb reference conut, put at wb_cmdq_cb */
//			drm_framebuffer_get(fb);
			if (!fb) {
				DDPPR_ERR("fb is NULL\n");
				return;
			}
			addon_config.addon_wdma_config.wdma_src_roi = src_roi;
			addon_config.addon_wdma_config.wdma_dst_roi = dst_roi;
			addon_config.addon_wdma_config.pitch = fb->pitches[0];
			addon_config.addon_wdma_config.addr = mtk_fb_get_dma(fb);
			addon_config.addon_wdma_config.fb = fb;
			addon_config.addon_wdma_config.p_golden_setting_context
				= __get_golden_setting_context(mtk_crtc);

			DDPFENCE("S+/PL12/e1/id%d/mva0x%08llx/size0x%08lx\n",
				(unsigned int)state->prop_val[CRTC_PROP_OUTPUT_FENCE_IDX],
				mtk_fb_get_dma(fb), mtk_fb_get_size(fb), mtk_drm_fb_is_secure(fb));

			if (mtk_crtc->is_dual_pipe) {
				int src_w = 0, dst_w = 0;
				struct mtk_rect src_roi_l, src_roi_r;
				struct mtk_rect dst_roi_l, dst_roi_r;
				unsigned int r_buff_off = 0;
				unsigned int Bpp;

				src_roi_l = src_roi_r = src_roi;
				dst_roi_l = dst_roi_r = dst_roi;

				/*
				 * If support tile overhead, front WDMA will be affected
				 * and WDMA after RSZ will be affected by ap-resolution switch
				 * need to assign proper width height
				 */
				src_w = mtk_crtc_get_width_by_comp(__func__, crtc,
					NULL, (scn == WDMA_WRITE_BACK));
				dst_w = mtk_crtc_get_width_by_comp(__func__, crtc,
					NULL, (scn == WDMA_WRITE_BACK));
				if (to_info.is_support && scn == WDMA_WRITE_BACK_OVL)
					src_w = to_info.left_in_width + to_info.right_in_width;

				/*
				 * If support tile overhead, front WDMA will be affected
				 * and WDMA after RSZ will be affected by ap-resolution switch
				 * need to assign proper width height
				 */
				if (to_info.is_support && scn == WDMA_WRITE_BACK_OVL) {
					src_roi_l.width = to_info.left_in_width;
					src_roi_r.width = to_info.right_in_width;
				} else {
					src_roi_l.width = src_w/2;
					src_roi_r.width = src_w/2;
				}

				/* Check if dst roi exceed src roi range */
				if (dst_roi.x + dst_roi.width < dst_w/2) {
					/* handle dst ROI locate in left pipe */
					dst_roi_r.x = 0;
					dst_roi_r.y = 0;
					dst_roi_r.width = 0;
				} else if (dst_roi.x >= dst_w/2) {
					/* handle dst ROI locate in right pipe */
					dst_roi_l.x = 0;
					dst_roi_l.width = 0;
					dst_roi_r.x = dst_roi.x - dst_w/2;
				} else {
					/* handle dst ROI locate in both display pipe */
					dst_roi_l.width = dst_w/2 - dst_roi_l.x;
					dst_roi_r.x = 0;
					if (to_info.is_support && scn == WDMA_WRITE_BACK_OVL)
						dst_roi_r.x += to_info.right_overhead;
					dst_roi_r.width = dst_roi.width - dst_roi_l.width;
					r_buff_off = dst_roi_l.width;
				}

				addon_config.addon_wdma_config.wdma_src_roi = src_roi_l;
				addon_config.addon_wdma_config.wdma_dst_roi = dst_roi_l;

				/* connect left pipe */
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);

				addon_module = &addon_data_dual->module_data[i];
				addon_config.addon_wdma_config.wdma_src_roi = src_roi_r;
				addon_config.addon_wdma_config.wdma_dst_roi = dst_roi_r;
				Bpp = mtk_drm_format_plane_cpp(fb->format->format, 0);
				addon_config.addon_wdma_config.addr += r_buff_off * Bpp;
				/* connect right pipe */
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
			} else {
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
			}
		} else
			DDPPR_ERR("addon type:%d + module:%d not support\n",
				  addon_module->type, addon_module->module);
	}
}

static void
_mtk_crtc_cwb_addon_module_connect(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	union mtk_addon_config addon_config;
	int index = drm_crtc_index(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_cwb_info *cwb_info;
	unsigned int buf_idx;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct drm_framebuffer *fb;
	struct total_tile_overhead to_info;
	int Bpp = 3;

	cwb_info = mtk_crtc->cwb_info;
	to_info = mtk_crtc_get_total_overhead(mtk_crtc);

	if (index != 0 || mtk_crtc_is_dc_mode(crtc) ||
		!cwb_info)
		return;

	mtk_crtc_cwb_set_sec(crtc);
	if (!cwb_info->enable || cwb_info->is_sec ||
			priv->need_cwb_path_disconnect)
		return;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					cwb_info->scn);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual
			(__func__, crtc, cwb_info->scn);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if ((addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v2) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_WDMA0_v3) ||
			(addon_module->type == ADDON_AFTER &&
			addon_module->module == DISP_OVLSYS_WDMA0)) {
			buf_idx = cwb_info->buf_idx;
			fb = cwb_info->buffer[buf_idx].fb;
			Bpp = mtk_get_format_bpp(fb->format->format);

			addon_config.addon_wdma_config.wdma_src_roi =
				cwb_info->src_roi;
			addon_config.addon_wdma_config.wdma_dst_roi =
				cwb_info->buffer[buf_idx].dst_roi;
			addon_config.addon_wdma_config.pitch =
				cwb_info->buffer[buf_idx].dst_roi.width * Bpp;
			addon_config.addon_wdma_config.addr =
				cwb_info->buffer[buf_idx].addr_mva;
			addon_config.addon_wdma_config.fb = fb;
			addon_config.addon_wdma_config.p_golden_setting_context
				= __get_golden_setting_context(mtk_crtc);
			if (mtk_crtc->is_dual_pipe) {
				int src_w = 0, dst_w = 0;
				struct mtk_rect src_roi_l;
				struct mtk_rect src_roi_r;
				struct mtk_rect dst_roi_l;
				struct mtk_rect dst_roi_r;
				unsigned int r_buff_off = 0;

				/*
				 * If support tile overhead, front WDMA will be affected
				 * and WDMA after RSZ will be affected by ap-resolution switch
				 * need to assign proper width height
				 */
				src_w = mtk_crtc_get_width_by_comp(__func__, crtc,
					NULL, (cwb_info->scn == WDMA_WRITE_BACK));
				dst_w = mtk_crtc_get_width_by_comp(__func__, crtc,
					NULL, (cwb_info->scn == WDMA_WRITE_BACK));
				if (to_info.is_support &&
					cwb_info->scn == WDMA_WRITE_BACK_OVL)
					src_w = to_info.left_in_width + to_info.right_in_width;

				src_roi_l = src_roi_r = cwb_info->src_roi;
				dst_roi_l = dst_roi_r = cwb_info->buffer[buf_idx].dst_roi;

				src_roi_l.x = 0;
				src_roi_r.x = 0;

				/*
				 * If support tile overhead, front WDMA will be affected
				 * and WDMA after RSZ will be affected by ap-resolution switch
				 * need to assign proper width height
				 */
				if (to_info.is_support &&
					cwb_info->scn == WDMA_WRITE_BACK_OVL) {
					src_roi_l.width = to_info.left_in_width;
					src_roi_r.width = to_info.right_in_width;
				} else {
					src_roi_l.width = src_w/2;
					src_roi_r.width = src_w/2;
				}

				if (cwb_info->buffer[buf_idx].dst_roi.x +
					cwb_info->buffer[buf_idx].dst_roi.width < dst_w/2) {
				/* handle source ROI locate in left pipe*/
					dst_roi_r.x = 0;
					dst_roi_r.y = 0;
					dst_roi_r.width = 0;
				} else if (cwb_info->buffer[buf_idx].dst_roi.x >= dst_w/2) {
				/* handle source ROI locate in right pipe*/
					dst_roi_l.x = 0;
					dst_roi_l.width = 0;
					dst_roi_r.x = cwb_info->buffer[buf_idx].dst_roi.x - dst_w/2;
					/* add tile overhead offest */
					if (to_info.is_support &&
						cwb_info->scn == WDMA_WRITE_BACK_OVL)
						dst_roi_r.x += to_info.right_overhead;
				} else {
				/* handle source ROI locate in both display pipe*/
					dst_roi_l.width = dst_w/2 -
						cwb_info->buffer[buf_idx].dst_roi.x;
					dst_roi_r.x = 0;
					if (to_info.is_support &&
						cwb_info->scn == WDMA_WRITE_BACK_OVL)
						dst_roi_r.x += to_info.right_overhead;
					dst_roi_r.width =
						cwb_info->buffer[buf_idx].dst_roi.width -
						dst_roi_l.width;
					r_buff_off = dst_roi_l.width;
				}
				DDPINFO("cwb (%u, %u %u, %u) (%u ,%u %u,%u)\n",
					cwb_info->src_roi.width, cwb_info->src_roi.height,
					cwb_info->src_roi.x, cwb_info->src_roi.y,
					cwb_info->buffer[buf_idx].dst_roi.width,
					cwb_info->buffer[buf_idx].dst_roi.height,
					cwb_info->buffer[buf_idx].dst_roi.x,
					cwb_info->buffer[buf_idx].dst_roi.y);
				DDPDBG("L s(%u,%u %u,%u), d(%u,%u %u,%u)\n",
					src_roi_l.width, src_roi_l.height,
					src_roi_l.x, src_roi_l.y,
					dst_roi_l.width, dst_roi_l.height,
					dst_roi_l.x, dst_roi_l.y);
				DDPDBG("R s(%u,%u %u,%u), d(%u,%u %u,%u)\n",
					src_roi_r.width, src_roi_r.height,
					src_roi_r.x, src_roi_r.y,
					dst_roi_r.width, dst_roi_r.height,
					dst_roi_r.x, dst_roi_r.y);
				addon_config.addon_wdma_config.wdma_src_roi =
					src_roi_l;
				addon_config.addon_wdma_config.wdma_dst_roi =
					dst_roi_l;

				/* connect left pipe */
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);

				addon_module = &addon_data_dual->module_data[i];
				addon_config.addon_wdma_config.wdma_src_roi =
					src_roi_r;
				addon_config.addon_wdma_config.wdma_dst_roi =
					dst_roi_r;
				addon_config.addon_wdma_config.addr += (u64)r_buff_off * (u64)Bpp;
				/* connect right pipe */
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
			} else
				mtk_addon_connect_after(crtc, ddp_mode, addon_module,
							  &addon_config, cmdq_handle);
		} else
			DDPPR_ERR("addon type:%d + module:%d not support\n",
				  addon_module->type, addon_module->module);
	}
}

static void
_mtk_crtc_lye_addon_module_connect(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct mtk_lye_ddp_state *lye_state,
				      struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	const struct mtk_addon_module_data *addon_module_dual;
	union mtk_addon_config addon_config;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					lye_state->scn[drm_crtc_index(crtc)]);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual
			(__func__, crtc, lye_state->scn[drm_crtc_index(crtc)]);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if (mtk_crtc->is_dual_pipe)
			addon_module_dual = &(addon_data_dual->module_data[i]);
		else
			addon_module_dual = NULL;

		if ((addon_module->type == ADDON_BETWEEN) &&
			(addon_module->module == DISP_MML_SRAM_ONLY)) {
			/* do nothing yet */
		} else if (addon_module->module == DISP_MML_IR_PQ ||
			   addon_module->module == DISP_MML_IR_PQ_v2 ||
			   addon_module->module == DISP_MML_IR_PQ_v3) {
			if (mtk_crtc->is_force_mml_scen && mtk_crtc->is_mml)
				mml_addon_module_connect(crtc, ddp_mode, addon_module,
							 addon_module_dual, &addon_config,
							 cmdq_handle);
		} else if (addon_module->module == DISP_RSZ ||
			   addon_module->module == DISP_RSZ_v2 ||
			   addon_module->module == DISP_RSZ_v5 ||
			   addon_module->module == OVL_RSZ) {
			int pipe;
			struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
			const struct mtk_addon_module_data *m[] = {addon_module, addon_module_dual};

			addon_config.addon_rsz_config.rsz_src_roi = state->rsz_src_roi;
			addon_config.addon_rsz_config.rsz_dst_roi = state->rsz_dst_roi;
			addon_config.addon_rsz_config.lc_tgt_layer = lye_state->lc_tgt_layer;

			if (mtk_crtc->is_dual_pipe) {
				for (pipe = 0; pipe <= mtk_crtc->is_dual_pipe; ++pipe) {
					if (!state->rsz_param[pipe].in_len || !m[pipe])
						continue;

					memcpy(&(addon_config.addon_rsz_config.rsz_param),
					  &state->rsz_param[pipe], sizeof(struct mtk_rsz_param));
					addon_config.addon_rsz_config.rsz_src_roi.width =
						state->rsz_param[pipe].in_len;
					addon_config.addon_rsz_config.rsz_dst_roi.width =
						state->rsz_param[pipe].out_len;
					addon_config.addon_rsz_config.rsz_dst_roi.x =
						state->rsz_param[pipe].out_x;

					addon_config.config_type.module = m[pipe]->module;
					addon_config.config_type.type = m[pipe]->type;

					if (m[pipe]->type == ADDON_BETWEEN)
						mtk_addon_connect_between(crtc, ddp_mode, m[pipe],
									&addon_config, cmdq_handle);
					else if (m[pipe]->type == ADDON_EMBED)
						mtk_addon_connect_embed(crtc, ddp_mode, m[pipe],
									&addon_config, cmdq_handle);
				}
			} else {
				if (addon_module->type == ADDON_BETWEEN)
					mtk_addon_connect_between(crtc, ddp_mode, addon_module,
								&addon_config, cmdq_handle);
				else if (addon_module->type == ADDON_EMBED)
					mtk_addon_connect_embed(crtc, ddp_mode, addon_module,
								&addon_config, cmdq_handle);
			}
		} else
			DDPPR_ERR("addon type:%d + module:%d not support\n",
				  addon_module->type, addon_module->module);
	}
}

static void
_mtk_crtc_lye_addon_module_config(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct mtk_lye_ddp_state *lye_state,
				      struct cmdq_pkt *cmdq_handle)
{
	int i;
	const struct mtk_addon_scenario_data *addon_data;
	const struct mtk_addon_scenario_data *addon_data_dual;
	const struct mtk_addon_module_data *addon_module;
	union mtk_addon_config addon_config;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
//	struct mtk_drm_private *priv = crtc->dev->dev_private;
//	struct mtk_ddp_comp *comp = NULL;

	addon_data = mtk_addon_get_scenario_data(__func__, crtc,
					lye_state->scn[drm_crtc_index(crtc)]);
	if (!addon_data)
		return;

	if (mtk_crtc->is_dual_pipe) {
		addon_data_dual = mtk_addon_get_scenario_data_dual
			(__func__, crtc, lye_state->scn[drm_crtc_index(crtc)]);

		if (!addon_data_dual)
			return;
	}

	for (i = 0; i < addon_data->module_num; i++) {
		addon_module = &addon_data->module_data[i];
		addon_config.config_type.module = addon_module->module;
		addon_config.config_type.type = addon_module->type;

		if (addon_module->type == ADDON_BETWEEN &&
		    (addon_module->module == DISP_MML_IR_PQ ||
		     addon_module->module == DISP_MML_IR_PQ_1 ||
		     addon_module->module == DISP_MML_IR_PQ_v2 ||
		     addon_module->module == DISP_MML_IR_PQ_v2_1 ||
		     addon_module->module == DISP_MML_IR_PQ_v3)) {
			mtk_addon_path_config(crtc, addon_module, &addon_config, cmdq_handle);
		} else
			DDPMSG("addon type:%d + module:%d not support\n", addon_module->type,
			       addon_module->module);
	}
}

static void
_mtk_crtc_atmoic_addon_module_connect(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct mtk_lye_ddp_state *lye_state,
				      struct cmdq_pkt *cmdq_handle)
{
	_mtk_crtc_wb_addon_module_connect(
				   crtc, ddp_mode, cmdq_handle);

	_mtk_crtc_cwb_addon_module_connect(
				   crtc, ddp_mode, cmdq_handle);

	_mtk_crtc_lye_addon_module_connect(
			crtc, ddp_mode, lye_state, cmdq_handle);
}

void
_mtk_crtc_atmoic_addon_module_config(
				      struct drm_crtc *crtc,
				      unsigned int ddp_mode,
				      struct mtk_lye_ddp_state *lye_state,
				      struct cmdq_pkt *cmdq_handle)
{

	_mtk_crtc_lye_addon_module_config(
			crtc, ddp_mode, lye_state, cmdq_handle);
}


void mtk_crtc_cwb_path_disconnect(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	struct cmdq_pkt *handle;
	int ddp_mode = mtk_crtc->ddp_mode;

	if (!mtk_crtc->cwb_info || !mtk_crtc->cwb_info->enable) {
		priv->need_cwb_path_disconnect = true;
		priv->cwb_is_preempted = true;
		return;
	}

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	priv->need_cwb_path_disconnect = true;
	mtk_crtc_pkt_create(&handle, crtc, client);

	if (!handle) {
		DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
		return;
	}

	mtk_crtc_wait_frame_done(mtk_crtc, handle, DDP_FIRST_PATH, 0);
	_mtk_crtc_cwb_addon_module_disconnect(crtc, ddp_mode, handle);
	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);
	priv->cwb_is_preempted = true;

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}

bool mtk_crtc_alloc_sram(struct mtk_drm_crtc *mtk_crtc, unsigned int hrt_idx)
{
	int ret = 0;
	struct slbc_data *sram = NULL;
	struct mtk_drm_sram_list *sram_acquired;

	if (!mtk_crtc)
		return false;

	mutex_lock(&mtk_crtc->mml_ir_sram.lock);

	if (kref_read(&mtk_crtc->mml_ir_sram.ref) < 1) {
		sram = &mtk_crtc->mml_ir_sram.data;

		ret = slbc_request(sram);
		if (ret < 0) {
			DDPMSG("%s slbc_request fail %d", __func__, ret);
			goto done;
		}
		ret = slbc_power_on(sram);
		if (ret < 0) {
			DDPMSG("%s slbc_power_on fail %d", __func__, ret);
			goto done;
		}

		DRM_MMP_MARK(sram_alloc, (unsigned long)sram->paddr, sram->size);
		DDPMSG("%s success - ret:%d address:0x%lx size:0x%lx\n", __func__, ret,
		       (unsigned long)sram->paddr, sram->size);

		kref_init(&mtk_crtc->mml_ir_sram.ref);
		mtk_crtc->mml_ir_sram.bk_hrt_idx = 0;
	} else {
		kref_get(&mtk_crtc->mml_ir_sram.ref);
	}

	sram_acquired = kzalloc(sizeof(struct mtk_drm_sram_list), GFP_KERNEL);
	sram_acquired->hrt_idx = hrt_idx;
	list_add_tail(&sram_acquired->head, &mtk_crtc->mml_ir_sram.list.head);

done:
	mutex_unlock(&mtk_crtc->mml_ir_sram.lock);
	return (ret == 0 ? true : false);
}

#ifndef DRM_CMDQ_DISABLE
static void mtk_crtc_free_sram(struct mtk_drm_crtc *mtk_crtc)
{
	if (!mtk_crtc)
		return;

	DDPMSG("%s address:0x%lx size:0x%lx\n", __func__,
	       (unsigned long)mtk_crtc->mml_ir_sram.data.paddr, mtk_crtc->mml_ir_sram.data.size);
	slbc_power_off(&mtk_crtc->mml_ir_sram.data);
	slbc_release(&mtk_crtc->mml_ir_sram.data);
	DRM_MMP_MARK(sram_free, (unsigned long)mtk_crtc->mml_ir_sram.data.paddr,
		     mtk_crtc->mml_ir_sram.data.size);
}

static void mtk_crtc_mml_clean(struct kref *kref)
{
	struct mtk_drm_sram *s = container_of(kref, typeof(*s), ref);
	struct mtk_drm_crtc *mtk_crtc = container_of(s, typeof(*mtk_crtc), mml_ir_sram);

	DDPMSG("%s: sram_list is empty, free sram\n", __func__);
	mtk_crtc_free_sram(mtk_crtc);

	if (mtk_crtc->mml_cfg) {
		mtk_free_mml_submit(mtk_crtc->mml_cfg);
		mtk_crtc->mml_cfg = NULL;
	}

	if (mtk_crtc->mml_cfg_pq) {
		mtk_free_mml_submit(mtk_crtc->mml_cfg_pq);
		mtk_crtc->mml_cfg_pq = NULL;
	}
}
#endif

static void mtk_crtc_atomic_ddp_config(struct drm_crtc *crtc,
				struct mtk_drm_lyeblob_ids *lyeblob_ids,
				struct cmdq_pkt *cmdq_handle)
{
	struct mtk_lye_ddp_state *lye_state, *old_lye_state;
	struct drm_property_blob *blob;
	struct drm_device *dev = crtc->dev;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	enum addon_scenario scn = NONE;

	if (lyeblob_ids->ddp_blob_id) {
		blob = drm_property_lookup_blob(dev, lyeblob_ids->ddp_blob_id);
		if (!blob) {
			DDPPR_ERR("%s directly return due to NULL blob\n", __func__);
			return;
		}
		lye_state = (struct mtk_lye_ddp_state *)blob->data;
		drm_property_blob_put(blob);
		old_lye_state = &state->lye_state;
		scn = lye_state->scn[drm_crtc_index(crtc)];

		/* reset ovl_pq_in_cb and ovl_pq_out_cb for safety */
		if (scn == MML_RSZ || scn == ONE_SCALING)
			mtk_ddp_clean_ovl_pq_crossbar(mtk_crtc, cmdq_handle);

		/* skip MML_WITH_PQ connection/disconnection */
		if ((lye_state->scn[drm_crtc_index(crtc)] == MML_WITH_PQ) &&
				(old_lye_state->scn[drm_crtc_index(crtc)] == MML_WITH_PQ))
			_mtk_crtc_atmoic_addon_module_config(crtc,
							mtk_crtc->ddp_mode,
							lye_state,
							cmdq_handle);
		else {
			_mtk_crtc_atmoic_addon_module_disconnect(crtc,
							mtk_crtc->ddp_mode,
							old_lye_state,
							cmdq_handle);

		/* When open VDS path switch feature, Don't need RSZ */
			if (!(mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_VDS_PATH_SWITCH) &&
				priv->need_vds_path_switch))
				_mtk_crtc_atmoic_addon_module_connect(crtc,
					mtk_crtc->ddp_mode,
					lye_state,
					cmdq_handle);
		}

		state->lye_state = *lye_state;
	}
}

static void mtk_crtc_free_ddpblob_ids(struct drm_crtc *crtc,
				struct mtk_drm_lyeblob_ids *lyeblob_ids)
{
	struct drm_device *dev = crtc->dev;
	struct drm_property_blob *blob;

	if (lyeblob_ids->ddp_blob_id) {
		DRM_MMP_MARK(layering_blob, 0xffffffff, lyeblob_ids->lye_idx);
		blob = drm_property_lookup_blob(dev, lyeblob_ids->ddp_blob_id);
		drm_property_blob_put(blob);
		drm_property_blob_put(blob);

		list_del(&lyeblob_ids->list);
		kfree(lyeblob_ids);
	}
}

static void mtk_crtc_free_lyeblob_ids(struct drm_crtc *crtc,
				struct mtk_drm_lyeblob_ids *lyeblob_ids)
{
	struct drm_device *dev = crtc->dev;
	struct drm_property_blob *blob;
	int32_t blob_id;
	int i, j;

	for (j = 0; j < MAX_CRTC; j++) {
		if (!((lyeblob_ids->ref_cnt_mask >> j) & 0x1))
			continue;

		for (i = 0; i < OVL_LAYER_NR; i++) {
			blob_id = lyeblob_ids->lye_plane_blob_id[j][i];
			if (blob_id > 0) {
				blob = drm_property_lookup_blob(dev, blob_id);
				drm_property_blob_put(blob);
				drm_property_blob_put(blob);
			}
		}
	}
}

static unsigned int dual_comp_map_mt6885(unsigned int comp_id)
{
	unsigned int ret = 0;

	switch (comp_id) {
	case DDP_COMPONENT_OVL0:
		ret = DDP_COMPONENT_OVL1;
		break;
	case DDP_COMPONENT_OVL0_2L:
		ret = DDP_COMPONENT_OVL1_2L;
		break;
	case DDP_COMPONENT_OVL2_2L:
		ret = DDP_COMPONENT_OVL3_2L;
		break;
	case DDP_COMPONENT_AAL0:
		ret = DDP_COMPONENT_AAL1;
		break;
	case DDP_COMPONENT_WDMA0:
		ret = DDP_COMPONENT_WDMA1;
		break;
	case DDP_COMPONENT_OVLSYS_WDMA0:
		ret = DDP_COMPONENT_OVLSYS_WDMA2;
		break;
	default:
		DDPMSG("unknown comp %u for %s\n", comp_id, __func__);
	}

	return ret;
}

static unsigned int dual_comp_map_mt6983(unsigned int comp_id)
{
	unsigned int ret = 0;

	switch (comp_id) {
	case DDP_COMPONENT_OVL0:
		ret = DDP_COMPONENT_OVL1;
		break;
	case DDP_COMPONENT_OVL0_2L:
		ret = DDP_COMPONENT_OVL2_2L;
		break;
	case DDP_COMPONENT_OVL1_2L:
		ret = DDP_COMPONENT_OVL3_2L;
		break;
	case DDP_COMPONENT_OVL0_2L_NWCG:
		ret = DDP_COMPONENT_OVL2_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL1_2L_NWCG:
		ret = DDP_COMPONENT_OVL3_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL2_2L_NWCG:
		ret = DDP_COMPONENT_OVL0_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL3_2L_NWCG:
		ret = DDP_COMPONENT_OVL1_2L_NWCG;
		break;
	case DDP_COMPONENT_AAL0:
		ret = DDP_COMPONENT_AAL1;
		break;
	case DDP_COMPONENT_WDMA0:
		ret = DDP_COMPONENT_WDMA2;
		break;
	default:
		DDPMSG("unknown comp %u for %s\n", comp_id, __func__);
	}

	return ret;
}

static unsigned int dual_comp_map_mt6985(unsigned int comp_id)
{
	unsigned int ret = 0;

//need check
	switch (comp_id) {
	case DDP_COMPONENT_OVL0:
		ret = DDP_COMPONENT_OVL1;
		break;
	case DDP_COMPONENT_OVL0_2L:
		ret = DDP_COMPONENT_OVL4_2L;
		break;
	case DDP_COMPONENT_OVL1_2L:
		ret = DDP_COMPONENT_OVL5_2L;
		break;
	case DDP_COMPONENT_OVL2_2L:
		ret = DDP_COMPONENT_OVL6_2L;
		break;
	case DDP_COMPONENT_OVL3_2L:
		ret = DDP_COMPONENT_OVL7_2L;
		break;
	case DDP_COMPONENT_OVL7_2L:
		ret = DDP_COMPONENT_OVL3_2L;
		break;
	case DDP_COMPONENT_OVL0_2L_NWCG:
		ret = DDP_COMPONENT_OVL2_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL1_2L_NWCG:
		ret = DDP_COMPONENT_OVL3_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL2_2L_NWCG:
		ret = DDP_COMPONENT_OVL0_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL3_2L_NWCG:
		ret = DDP_COMPONENT_OVL1_2L_NWCG;
		break;
	case DDP_COMPONENT_AAL0:
		ret = DDP_COMPONENT_AAL1;
		break;
	case DDP_COMPONENT_WDMA0:
		ret = DDP_COMPONENT_WDMA1;
		break;
	case DDP_COMPONENT_MDP_RDMA1:
		ret = DDP_COMPONENT_MDP_RDMA0;
		break;
	case DDP_COMPONENT_OVLSYS_WDMA0:
		ret = DDP_COMPONENT_OVLSYS_WDMA2;
		break;
	case DDP_COMPONENT_OVLSYS_WDMA1:
		ret = DDP_COMPONENT_OVLSYS_WDMA3;
		break;
	default:
		DDPMSG("unknown comp %u for %s\n", comp_id, __func__);
	}

	return ret;
}

static unsigned int dual_comp_map_mt6895(unsigned int comp_id)
{
	unsigned int ret = 0;

	switch (comp_id) {
	case DDP_COMPONENT_OVL0:
		ret = DDP_COMPONENT_OVL1;
		break;
	case DDP_COMPONENT_OVL0_2L:
		ret = DDP_COMPONENT_OVL2_2L;
		break;
	case DDP_COMPONENT_OVL1_2L:
		ret = DDP_COMPONENT_OVL3_2L;
		break;
	case DDP_COMPONENT_OVL2_2L:
		ret = DDP_COMPONENT_OVL0_2L;
		break;
	case DDP_COMPONENT_OVL3_2L:
		ret = DDP_COMPONENT_OVL1_2L;
		break;
	case DDP_COMPONENT_OVL0_2L_NWCG:
		ret = DDP_COMPONENT_OVL2_2L_NWCG;
		break;
	case DDP_COMPONENT_OVL1_2L_NWCG:
		ret = DDP_COMPONENT_OVL3_2L_NWCG;
		break;
	case DDP_COMPONENT_AAL0:
		ret = DDP_COMPONENT_AAL1;
		break;
	case DDP_COMPONENT_WDMA0:
		ret = DDP_COMPONENT_WDMA2;
		break;
	default:
		DDPMSG("unknown comp %u for %s\n", comp_id, __func__);
	}

	return ret;
}

unsigned int dual_pipe_comp_mapping(unsigned int mmsys_id, unsigned int comp_id)
{
	unsigned int ret = 0;

	switch (mmsys_id) {
	case MMSYS_MT6983:
		ret = dual_comp_map_mt6983(comp_id);
		break;
	case MMSYS_MT6985:
		ret = dual_comp_map_mt6985(comp_id);
		break;
	case MMSYS_MT6895:
		ret = dual_comp_map_mt6895(comp_id);
		break;
	case MMSYS_MT6885:
		ret = dual_comp_map_mt6885(comp_id);
		break;
	default:
		DDPMSG("unknown mmsys %x for %s\n", mmsys_id, __func__);
	}

	return ret;
}

static void mtk_crtc_get_plane_comp_state(struct drm_crtc *crtc,
					  struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_plane_comp_state *comp_state;
	int i, j, k;

	for (i = mtk_crtc->layer_nr - 1; i >= 0; i--) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state;
		struct mtk_ddp_comp *comp = NULL;

		plane_state = to_mtk_plane_state(plane->state);
		comp_state = &(plane_state->comp_state);
		/* TODO: check plane_state by pending.enable */
		if (plane_state->base.visible) {
			for_each_comp_in_cur_crtc_path(
				comp, mtk_crtc, j,
				k) {
				if (comp->id != comp_state->comp_id)
					continue;

				mtk_ddp_comp_layer_off(
					comp,
					comp_state->lye_id,
					comp_state->ext_lye_id,
					cmdq_handle);

				if (mtk_crtc->is_dual_pipe) {
					struct mtk_drm_private *priv =
						mtk_crtc->base.dev->dev_private;

					comp = priv->ddp_comp
						[dual_pipe_comp_mapping
						(priv->data->mmsys_id, comp_state->comp_id)];
					mtk_ddp_comp_layer_off(
							comp,
							comp_state->lye_id,
							comp_state->ext_lye_id,
							cmdq_handle);
				}
				break;
			}

		}
		/* Set the crtc to plane state for releasing fence purpose.*/
		plane_state->crtc = crtc;

		mtk_plane_get_comp_state(plane, &plane_state->comp_state, crtc, 0);

		if (plane_state->comp_state.layer_caps & MTK_MML_DISP_DECOUPLE_LAYER)
			plane_state->mml_mode = MML_MODE_MML_DECOUPLE;
		else if (plane_state->comp_state.layer_caps & MTK_MML_DISP_MDP_LAYER)
			plane_state->mml_mode = MML_MODE_MDP_DECOUPLE;
	}
}
unsigned int mtk_drm_primary_frame_bw(struct drm_crtc *crtc)
{
	unsigned long long bw = 0;
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_drm_private *priv;
	struct mtk_ddp_comp *output_comp;

	if (unlikely(crtc == NULL || crtc->dev == NULL)) {
		DDPPR_ERR("%s %d NULL crtc\n", __func__, __LINE__);
		return 0;
	}
	priv = crtc->dev->dev_private;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT) &&
			drm_crtc_index(crtc) != 0) {
		DDPPR_ERR("%s no support CRTC%u", __func__,
			drm_crtc_index(crtc));
		crtc = priv->crtc[0];
	}

	if (unlikely(crtc == NULL)) {
		DDPPR_ERR("%s %d NULL crtc\n", __func__, __LINE__);
		return 0;
	}
	mtk_crtc = to_mtk_crtc(crtc);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL,
				GET_FRAME_HRT_BW_BY_DATARATE, &bw);

	return (unsigned int)bw;
}

static unsigned int overlap_to_bw(struct drm_crtc *crtc,
	unsigned int overlap_num, struct mtk_drm_lyeblob_ids *lyeblob_ids)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	int crtc_idx = drm_crtc_index(crtc);
	unsigned int bw_base = mtk_drm_primary_frame_bw(crtc);
	unsigned int bw = 0;
	unsigned int ori_overlap_num = overlap_num;
	int discount = hrt_lp_switch_get();

	if (discount >= 200) {
		if (discount > overlap_num)
			DDPINFO("overlap_num discount:%d > original:%d\n",
				discount, overlap_num);
		else {
			overlap_num -= discount;

			if (overlap_num < 400) {
				DDPINFO("overlap_num of discount < 400, change to 400\n");
				overlap_num  = 400;
			}
		}

		DDPINFO("%s:%d ori overlap_num=%u, after discount overlap_num=%u\n",
			__func__, __LINE__, ori_overlap_num, overlap_num);
	}

	bw = bw_base * overlap_num / 400;

	DDPDBG("%s:%d bw_base:%u overlap:%u bw:%u\n", __func__, __LINE__,
		bw_base, overlap_num, bw);

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_OVL_BW_MONITOR) &&
		(crtc_idx == 0) && lyeblob_ids &&
		(lyeblob_ids->frame_weight_of_bwm != 0)) {
		bw = bw_base * lyeblob_ids->frame_weight_of_bwm / 400;
		DDPDBG("%s:%d BWM bw_base:%u overlap:%u bw:%u\n", __func__, __LINE__,
				bw_base, lyeblob_ids->frame_weight_of_bwm, bw);
	}
	return bw;
}

static void mtk_crtc_update_hrt_state(struct drm_crtc *crtc,
				      unsigned int frame_weight,
				      struct mtk_drm_lyeblob_ids *lyeblob_ids,
				      struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	unsigned int bw = overlap_to_bw(crtc, frame_weight, lyeblob_ids);
	int crtc_idx = drm_crtc_index(crtc);
	unsigned int ovl0_2l_no_compress_num;
	struct mtk_ddp_comp *output_comp;
	struct drm_display_mode *mode = NULL;
	unsigned int max_fps = 0;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int en = 0;

	if (unlikely(!mtk_crtc || !mtk_crtc->qos_ctx)) {
		DDPPR_ERR("%s invalid qos_ctx\n", __func__);
		return;
	}

	if (mtk_crtc->path_data->is_discrete_path) {
		if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
			mtk_disp_set_hrt_bw(mtk_crtc, bw);
		return;
	}

	DDPINFO("%s bw=%d, last_hrt_req=%d, overlap=%d\n",
			__func__, bw, mtk_crtc->qos_ctx->last_hrt_req, frame_weight);

	if (atomic_read(&mtk_crtc->force_high_step) == 1) {
		unsigned int step_size = mtk_drm_get_mmclk_step_size();

		if (!mtk_crtc->force_high_enabled) {
			DDPMSG("start SET MMCLK step 0\n");
			/* set MMCLK highest step for next 2048 frame */
			mtk_crtc->force_high_enabled = 2048;
		}
		mtk_crtc->force_high_enabled--;

		if (mtk_crtc->force_high_enabled > 0) {
			mtk_drm_set_mmclk(crtc, step_size - 1, false, __func__);
		} else {
			en = 1;
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp) {
				DDPMSG("set MMCLK back, and enable underrun irq\n");
				mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE, &en);
				/* enable dsi underrun irq*/
				mtk_ddp_comp_io_cmd(output_comp, NULL, IRQ_UNDERRUN, &en);
			}
			atomic_set(&mtk_crtc->force_high_step, 0);
		}
	} else {
		if (mtk_crtc->force_high_enabled != 0) {
			en = 1;
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp) {
				/* enable dsi underrun irq*/
				DDPMSG("enable underrun irq after force_high_step set to 0\n");
				mtk_ddp_comp_io_cmd(output_comp, NULL, IRQ_UNDERRUN, &en);
			}
		}
		mtk_crtc->force_high_enabled = 0;
	}

	if (priv->data->has_smi_limitation && lyeblob_ids) {
		output_comp = mtk_ddp_comp_request_output(mtk_crtc);

		if (output_comp && ((output_comp->id == DDP_COMPONENT_DSI0) ||
				(output_comp->id == DDP_COMPONENT_DSI1))
				&& !(mtk_dsi_is_cmd_mode(output_comp)))
			mtk_ddp_comp_io_cmd(output_comp, NULL,
				DSI_GET_MODE_BY_MAX_VREFRESH, &mode);
		if (mode)
			max_fps = drm_mode_vrefresh(mode);

		ovl0_2l_no_compress_num =
			HRT_GET_NO_COMPRESS_FLAG(lyeblob_ids->hrt_num);

		DDPINFO("%s CRTC%u bw:%d, no_compress_num:%d max_fps:%d\n",
			__func__, crtc_idx, bw, ovl0_2l_no_compress_num, max_fps);

		/* Workaround for 120hz SMI larb BW limitation */
		if (crtc_idx == 0 && max_fps == 120) {
			if (ovl0_2l_no_compress_num == 1 &&
				bw < 2944) {
				bw = 2944;
				DDPINFO("%s CRTC%u dram freq to 1600hz\n",
					__func__, crtc_idx);
			} else if (ovl0_2l_no_compress_num == 2 &&
				bw < 3433) {
				bw = 3433;
				DDPINFO("%s CRTC%u dram freq to 2400hz\n",
					__func__, crtc_idx);
			}
		}
	}

	/* can't access backup slot since top clk off */
	if (priv->power_state == false)
		return;

	/* Only update HRT information on path with HRT comp */
	if (bw > mtk_crtc->qos_ctx->last_hrt_req) {
		if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
			mtk_disp_set_hrt_bw(mtk_crtc, bw);

		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			       mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_CUR_HRT_LEVEL),
			       NO_PENDING_HRT, ~0);
	} else if (bw < mtk_crtc->qos_ctx->last_hrt_req) {
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			       mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_CUR_HRT_LEVEL),
			       bw, ~0);
	}

	mtk_crtc->qos_ctx->last_hrt_req = bw;

	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
		       mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_CUR_HRT_IDX),
		       crtc_state->prop_val[CRTC_PROP_LYE_IDX], ~0);
}

static void copy_drm_disp_mode(struct drm_display_mode *src,
	struct drm_display_mode *dst)
{
	dst->clock       = src->clock;
	dst->hdisplay    = src->hdisplay;
	dst->hsync_start = src->hsync_start;
	dst->hsync_end   = src->hsync_end;
	dst->htotal      = src->htotal;
	dst->vdisplay    = src->vdisplay;
	dst->vsync_start = src->vsync_start;
	dst->vsync_end   = src->vsync_end;
	dst->vtotal      = src->vtotal;
}

struct golden_setting_context *
__get_golden_setting_context(struct mtk_drm_crtc *mtk_crtc)
{
	static struct golden_setting_context gs_ctx[MAX_CRTC];
	struct drm_crtc *crtc = &mtk_crtc->base;
	int idx = drm_crtc_index(&mtk_crtc->base);

	/* default setting */
	gs_ctx[idx].is_dc = 0;

	/* primary_display */
	switch (idx) {
	case 0:
	case 3:
		gs_ctx[idx].is_vdo_mode =
				mtk_crtc_is_frame_trigger_mode(crtc) ? 0 : 1;
		gs_ctx[idx].dst_width = crtc->state->adjusted_mode.hdisplay;
		gs_ctx[idx].dst_height = crtc->state->adjusted_mode.vdisplay;
		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				gs_ctx[idx].vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				gs_ctx[idx].vrefresh =
					drm_mode_vrefresh(&crtc->state->adjusted_mode);
		} else
			gs_ctx[idx].vrefresh =
				drm_mode_vrefresh(&crtc->state->adjusted_mode);
		break;
	case 1:
		/* TO DO: need more smart judge */
		gs_ctx[idx].is_vdo_mode = 1;
		gs_ctx[idx].dst_width = crtc->state->adjusted_mode.hdisplay;
		gs_ctx[idx].dst_height = crtc->state->adjusted_mode.vdisplay;
		gs_ctx[idx].vrefresh = (crtc->state->adjusted_mode.clock * 1000) /
			(crtc->state->adjusted_mode.htotal * crtc->state->adjusted_mode.vtotal);
		DDPMSG("%s: crtc[%d], htt: %d, vtt: %d, clk :%d, fps: %d\n", __func__, idx,
			crtc->state->adjusted_mode.htotal, crtc->state->adjusted_mode.vtotal,
			crtc->state->adjusted_mode.clock, gs_ctx[idx].vrefresh);
		break;
	case 2:
		/* TO DO: need more smart judge */
		gs_ctx[idx].is_vdo_mode = 0;
		break;
	}

	return &gs_ctx[idx];
}

struct golden_setting_context *
__get_scaling_golden_setting_context(struct mtk_drm_crtc *mtk_crtc)
{
	static struct golden_setting_context scaling_gs_ctx[MAX_CRTC];
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned int idx = drm_crtc_index(&mtk_crtc->base);
	struct drm_display_mode *mode;
	struct golden_setting_context *ctx = __get_golden_setting_context(mtk_crtc);

	scaling_gs_ctx[idx].is_dc = ctx->is_dc;

	/* primary_display */
	switch (idx) {
	case 0:
		mode = mtk_crtc_get_display_mode_by_comp(__func__, crtc, NULL, true);
		scaling_gs_ctx[idx].is_vdo_mode =
				mtk_crtc_is_frame_trigger_mode(crtc) ? 0 : 1;

		if (mode == NULL) {
			DDPPR_ERR("display_mode is NULL\n");
			break;
		}

		scaling_gs_ctx[idx].dst_width = mode->hdisplay;
		scaling_gs_ctx[idx].dst_height = mode->vdisplay;

		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				scaling_gs_ctx[idx].vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				scaling_gs_ctx[idx].vrefresh =
					drm_mode_vrefresh(mode);
		} else
			scaling_gs_ctx[idx].vrefresh =
				drm_mode_vrefresh(mode);
		break;

	default:
		DDPPR_ERR("%s unknown for %d\n", __func__, idx);
	}

	return &scaling_gs_ctx[idx];
}

unsigned int mtk_crtc_get_idle_interval(struct drm_crtc *crtc, unsigned int fps)
{

	unsigned int idle_interval = mtk_drm_get_idle_check_interval(crtc);
	struct mtk_panel_params *panel_params = mtk_drm_get_lcm_ext_params(crtc);
	bool vblank_off = panel_params ? panel_params->vblank_off : false;
	/*calculate the timeout to enter idle in ms*/

	if (!vblank_off && idle_interval > 50)
		return 0;

	if (fps > 90)
		idle_interval = (4 * 1000) / fps + 1;
	else
		idle_interval = (3 * 1000) / fps + 1;

	if (idle_interval > 50)
		idle_interval = 50;

	DDPMSG("[fps]:%s,[fps->idle interval][%d fps->%d ms]\n",
		__func__, fps, idle_interval);

	return idle_interval;
}

void mtk_drm_crtc_mode_check(struct drm_crtc *crtc,
	struct drm_crtc_state *old_state, struct drm_crtc_state *new_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_display_mode *mode;
	struct mtk_crtc_state *old_mtk_state = NULL;
	struct mtk_crtc_state *new_mtk_state = NULL;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (!new_state || !old_state)
		return;

	old_mtk_state = to_mtk_crtc_state(old_state);
	new_mtk_state = to_mtk_crtc_state(new_state);
	mtk_crtc->mode_chg = false;

	if (old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX] ==
		new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX])
		return;

	/*connector is changed , update mode_idx to new one*/
	if (of_property_read_bool(priv->mmsys_dev->of_node, "enable-output-int-switch")
		&& old_state->connectors_changed) {
		DDPMSG("%s++ from %u to %u when connectors changed\n", __func__,
		old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
		new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);

		old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX] =
			new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX];
	}
	if (mtk_crtc->res_switch != RES_SWITCH_NO_USE) {
		//workaround for hwc
		if (mtk_crtc->mode_idx
			== old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]) {
			new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]
				= mtk_crtc->mode_idx;
		}
	}

	DDPMSG("%s++ from %u to %u\n", __func__,
		old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
		new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);

	/* Update mode & adjusted_mode in CRTC */
	mode = mtk_drm_crtc_avail_disp_mode(crtc,
		new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
	if (IS_ERR_OR_NULL(mode)) {
		DDPPR_ERR("%s crtc%u invalid disp_mode %u\n", __func__,
			drm_crtc_index(crtc), new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
		return;
	}

	copy_drm_disp_mode(mode, &new_state->mode);
	new_state->mode.hskew = mode->hskew;
	drm_mode_set_crtcinfo(&new_state->mode, 0);

	copy_drm_disp_mode(mode, &new_state->adjusted_mode);
	new_state->adjusted_mode.hskew = mode->hskew;
	drm_mode_set_crtcinfo(&new_state->adjusted_mode, 0);

	if (old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX] !=
		new_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX])
		mtk_crtc->mode_chg = true;
}

void mtk_crtc_load_round_corner_pattern(struct drm_crtc *crtc,
					struct cmdq_pkt *handle);

void mtk_crtc_mode_switch_on_ap_config(struct mtk_drm_crtc *mtk_crtc,
	struct drm_crtc_state *old_state)
{
	int i, j;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *cevent_cmdq_handle, *cmdq_handle, *sevent_cmdq_handle;
	struct mtk_ddp_config cfg = {0};
	struct mtk_ddp_config scaling_cfg = {0};
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_comp *output_comp;

	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		DDPMSG("video mode does not support resolution switch!!!\n");
		return;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!output_comp) {
		DDPMSG("output_comp is null!\n");
		return;
	}

	cfg.w = crtc->state->adjusted_mode.hdisplay;
	cfg.h = crtc->state->adjusted_mode.vdisplay;
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params &&
		mtk_crtc->panel_ext->params->dyn_fps.switch_en == 1
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		cfg.vrefresh =
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	else
		cfg.vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg.bpc = mtk_crtc->bpc;
	cfg.p_golden_setting_context = __get_golden_setting_context(mtk_crtc);

	cfg.rsz_src_w = cfg.w;
	cfg.rsz_src_h = cfg.h;

	if (mtk_crtc->scaling_ctx.scaling_en) {
		memcpy(&scaling_cfg, &cfg, sizeof(struct mtk_ddp_config));
		scaling_cfg.w = mtk_crtc->scaling_ctx.scaling_mode->hdisplay;
		scaling_cfg.h = mtk_crtc->scaling_ctx.scaling_mode->vdisplay;
		scaling_cfg.p_golden_setting_context =
			__get_scaling_golden_setting_context(mtk_crtc);
	}

	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 1);

	mtk_crtc_pkt_create(&cevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);
	/* 1. wait frame done & wait DSI not busy */
	cmdq_pkt_wait_no_clear(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	/* Clear stream block to prevent trigger loop start */
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_flush(cevent_cmdq_handle);
	cmdq_pkt_destroy(cevent_cmdq_handle);

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (mtk_crtc->is_dual_pipe &&
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_TILE_OVERHEAD)) {
		cfg.tile_overhead.is_support = true;
		cfg.tile_overhead.left_in_width = cfg.w / 2;
		cfg.tile_overhead.right_in_width = cfg.w / 2;

		if (mtk_crtc->scaling_ctx.scaling_en) {
			scaling_cfg.tile_overhead.is_support = true;
			scaling_cfg.tile_overhead.left_in_width = scaling_cfg.w / 2;
			scaling_cfg.tile_overhead.right_in_width = scaling_cfg.w / 2;
		}

	} else {
		cfg.tile_overhead.is_support = false;
		cfg.tile_overhead.left_in_width = cfg.w;
		cfg.tile_overhead.right_in_width = cfg.w;
	}

	/*Calculate total overhead*/
	for_each_comp_in_crtc_path_reverse(comp, mtk_crtc, i, j) {

		if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path) {
			mtk_ddp_comp_config_overhead(comp, &scaling_cfg);
			DDPINFO("%s:comp %s (scaling_path) width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				scaling_cfg.tile_overhead.left_in_width,
				scaling_cfg.tile_overhead.right_in_width,
				scaling_cfg.tile_overhead.left_overhead,
				scaling_cfg.tile_overhead.right_overhead);
		} else {
			mtk_ddp_comp_config_overhead(comp, &cfg);
			DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				cfg.tile_overhead.left_in_width,
				cfg.tile_overhead.right_in_width,
				cfg.tile_overhead.left_overhead,
				cfg.tile_overhead.right_overhead);
		}

		if (mtk_crtc->scaling_ctx.scaling_en
			&& mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
			cfg.tile_overhead = scaling_cfg.tile_overhead;
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		/* skip config comp after rsz, except chist */
		if (comp->in_scaling_path &&
			(!((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_RSZ) ||
				(mtk_ddp_comp_get_type(comp->id) == MTK_DISP_CHIST))))
			continue;

		DDPDBG("%s:%s, scaling_path:%d\n", __func__,
			mtk_dump_comp_str(comp), comp->in_scaling_path);
		mtk_ddp_comp_stop(comp, cmdq_handle);

		if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path)
			mtk_ddp_comp_config(comp, &scaling_cfg, cmdq_handle);
		else
			mtk_ddp_comp_config(comp, &cfg, cmdq_handle);

		mtk_ddp_comp_start(comp, cmdq_handle);
		if (!mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_USE_PQ))
			mtk_ddp_comp_bypass(comp, 1, cmdq_handle);

		if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) {
			cfg.source_bpc = mtk_ddp_comp_io_cmd(comp, NULL,
				OVL_GET_SOURCE_BPC, NULL);
			scaling_cfg.source_bpc = cfg.source_bpc;
		}
	}

	if (mtk_crtc->is_dual_pipe) {

		/*Calculate total overhead*/
		for_each_comp_in_dual_pipe_reverse(comp, mtk_crtc, i, j) {

			if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path) {
				mtk_ddp_comp_config_overhead(comp, &scaling_cfg);
				DDPINFO("%s:comp %s (scaling_path) width L:%d R:%d, overhead L:%d R:%d\n",
					__func__, mtk_dump_comp_str(comp),
					scaling_cfg.tile_overhead.left_in_width,
					scaling_cfg.tile_overhead.right_in_width,
					scaling_cfg.tile_overhead.left_overhead,
					scaling_cfg.tile_overhead.right_overhead);
			} else {
				mtk_ddp_comp_config_overhead(comp, &cfg);
				DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
					__func__, mtk_dump_comp_str(comp),
					cfg.tile_overhead.left_in_width,
					cfg.tile_overhead.right_in_width,
					cfg.tile_overhead.left_overhead,
					cfg.tile_overhead.right_overhead);
			}

			if (mtk_crtc->scaling_ctx.scaling_en
				&& mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
				cfg.tile_overhead = scaling_cfg.tile_overhead;
		}

		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			/* skip config comp after rsz, except chist */
			if (comp->in_scaling_path &&
				(!((mtk_ddp_comp_get_type(comp->id) == MTK_DISP_RSZ) ||
					(mtk_ddp_comp_get_type(comp->id) == MTK_DISP_CHIST))))
				continue;

			DDPDBG("%s:%s, scaling_path:%d\n", __func__,
				mtk_dump_comp_str(comp), comp->in_scaling_path);
			mtk_ddp_comp_stop(comp, cmdq_handle);

			if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path)
				mtk_ddp_comp_config(comp, &scaling_cfg, cmdq_handle);
			else
				mtk_ddp_comp_config(comp, &cfg, cmdq_handle);

			mtk_ddp_comp_start(comp, cmdq_handle);
			if (!mtk_drm_helper_get_opt(priv->helper_opt,
					MTK_DRM_OPT_USE_PQ))
				mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
		}
	}

	/*store total overhead data*/
	mtk_crtc_store_total_overhead(mtk_crtc, cfg.tile_overhead);

	drm_update_dal(&mtk_crtc->base, cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 2);

	mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_TIMING_CHANGE, old_state);
	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 3);

	/* adjust trigger loop in different display mode */
	if (mtk_crtc_with_trigger_loop(crtc) &&
			mtk_crtc_with_event_loop(crtc) &&
			mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_crtc_stop_trig_loop(crtc);
		mtk_crtc_stop_event_loop(crtc);

		mtk_crtc_start_event_loop(crtc);
		mtk_crtc_start_trig_loop(crtc);
	}

	/* set frame done */
	mtk_crtc_pkt_create(&sevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_flush(sevent_cmdq_handle);
	cmdq_pkt_destroy(sevent_cmdq_handle);
}

void mtk_crtc_mode_switch_config(struct mtk_drm_crtc *mtk_crtc,
	struct drm_crtc_state *old_state)
{
	int i, j;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct cmdq_pkt *cevent_cmdq_handle, *cmdq_handle, *sevent_cmdq_handle;
	struct mtk_ddp_config cfg = {0};
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_comp *output_comp;

	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		DDPMSG("video mode does not support resolution switch!!!\n");
		return;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!output_comp) {
		DDPMSG("output_comp is null!\n");
		return;
	}

	cfg.w = crtc->state->adjusted_mode.hdisplay;
	cfg.h = crtc->state->adjusted_mode.vdisplay;
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params &&
		mtk_crtc->panel_ext->params->dyn_fps.switch_en == 1
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		cfg.vrefresh =
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	else
		cfg.vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg.bpc = mtk_crtc->bpc;
	cfg.p_golden_setting_context = __get_golden_setting_context(mtk_crtc);

	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 1);

	mtk_crtc_pkt_create(&cevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cevent_cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cevent_cmdq_handle\n", __func__, __LINE__);
		return;
	}

	/* 1. wait frame done & wait DSI not busy */
	cmdq_pkt_wait_no_clear(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	/* Clear stream block to prevent trigger loop start */
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_flush(cevent_cmdq_handle);
	cmdq_pkt_destroy(cevent_cmdq_handle);

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq_handle\n", __func__, __LINE__);
		return;
	}

	mtk_crtc_load_round_corner_pattern(&mtk_crtc->base, cmdq_handle);

	if (mtk_crtc->is_dual_pipe &&
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_TILE_OVERHEAD)) {
		cfg.tile_overhead.is_support = true;
		cfg.tile_overhead.left_in_width = cfg.w / 2;
		cfg.tile_overhead.right_in_width = cfg.w / 2;
	} else {
		cfg.tile_overhead.is_support = false;
		cfg.tile_overhead.left_in_width = cfg.w;
		cfg.tile_overhead.right_in_width = cfg.w;
	}

	/* Calculate total overhead */
	for_each_comp_in_crtc_path_reverse(comp, mtk_crtc, i, j) {
		mtk_ddp_comp_config_overhead(comp, &cfg);
		DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
			__func__, mtk_dump_comp_str(comp),
			cfg.tile_overhead.left_in_width,
			cfg.tile_overhead.right_in_width,
			cfg.tile_overhead.left_overhead,
			cfg.tile_overhead.right_overhead);
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		DDPDBG("%s:%s\n", __func__, mtk_dump_comp_str(comp));
		mtk_ddp_comp_stop(comp, cmdq_handle);
		mtk_ddp_comp_config(comp, &cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);
		if (!mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_USE_PQ))
			mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
	}

	if (mtk_crtc->is_dual_pipe) {
		/* Calculate total overhead */
		for_each_comp_in_dual_pipe_reverse(comp, mtk_crtc, i, j) {
			mtk_ddp_comp_config_overhead(comp, &cfg);
			DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				cfg.tile_overhead.left_in_width,
				cfg.tile_overhead.right_in_width,
				cfg.tile_overhead.left_overhead,
				cfg.tile_overhead.right_overhead);
		}

		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			DDPDBG("%s:%s\n", __func__, mtk_dump_comp_str(comp));
			mtk_ddp_comp_stop(comp, cmdq_handle);
			mtk_ddp_comp_config(comp, &cfg, cmdq_handle);
			mtk_ddp_comp_start(comp, cmdq_handle);
			if (!mtk_drm_helper_get_opt(priv->helper_opt,
					MTK_DRM_OPT_USE_PQ))
				mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
		}
	}

	/* store total overhead data */
	mtk_crtc_store_total_overhead(mtk_crtc, cfg.tile_overhead);

	if (panel_ext && panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		struct mtk_ddp_comp *dsc_comp;

		if (drm_crtc_index(crtc) == 3)
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		dsc_comp->mtk_crtc = mtk_crtc;

		DDPDBG("%s:%s\n", __func__, mtk_dump_comp_str(dsc_comp));
		mtk_ddp_comp_stop(dsc_comp, cmdq_handle);
		mtk_ddp_comp_config(dsc_comp, &cfg, cmdq_handle);
		mtk_ddp_comp_start(dsc_comp, cmdq_handle);
	}

	drm_update_dal(&mtk_crtc->base, cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 2);

	mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_TIMING_CHANGE, old_state);
	CRTC_MMP_MARK((int) drm_crtc_index(crtc), mode_switch, 0, 3);

	/* adjust trigger loop in different display mode */
	if (mtk_crtc_with_trigger_loop(crtc) &&
			mtk_crtc_with_event_loop(crtc) &&
			mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_crtc_stop_trig_loop(crtc);
		mtk_crtc_stop_event_loop(crtc);

		mtk_crtc_start_event_loop(crtc);
		mtk_crtc_start_trig_loop(crtc);
	}

	/* set frame done */
	mtk_crtc_pkt_create(&sevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!sevent_cmdq_handle) {
		DDPPR_ERR("%s:%d NULL sevent_cmdq_handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_flush(sevent_cmdq_handle);
	cmdq_pkt_destroy(sevent_cmdq_handle);
}

static void mtk_crtc_disp_mode_switch_begin(struct drm_crtc *crtc,
	struct drm_crtc_state *old_state, struct mtk_crtc_state *mtk_state,
	struct cmdq_pkt *cmdq_handle)
{
	struct mtk_crtc_state *old_mtk_state = to_mtk_crtc_state(old_state);
	struct mtk_ddp_config cfg;
	struct mtk_oddmr_timing oddmr_timing;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	unsigned int fps_src, fps_dst;
	unsigned int mode_chg_index = 0;
	unsigned int _idle_timeout = 50;/*ms*/
	int en = 1;
	struct mtk_ddp_comp *output_comp;
	struct mtk_ddp_comp *oddmr_comp;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	/* Check if disp_mode_idx change */
	if (old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX] ==
		mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]) {
		if (drm_need_fisrt_invoke_fps_callbacks()) {
			fps_dst = drm_mode_vrefresh(&crtc->state->mode);
			drm_invoke_fps_chg_callbacks(fps_dst);
		}
		return;
	}

	CRTC_MMP_EVENT_START((int) drm_crtc_index(crtc), mode_switch,
		old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
		mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);

	DDPMSG("%s++ from %u to %u\n", __func__,
		old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
		mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);

	fps_src = drm_mode_vrefresh(&old_state->mode);
	fps_dst = drm_mode_vrefresh(&crtc->state->mode);

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp) {
		mtk_ddp_comp_io_cmd(output_comp, NULL, MODE_SWITCH_INDEX,
				old_state);
		mode_chg_index = output_comp->mtk_crtc->mode_change_index;
	}
	//to do fps change index adjust
	if ((mode_chg_index & MODE_DSI_CLK)
		|| ((mode_chg_index & MODE_DSI_HFP)
			&& !mtk_crtc_is_frame_trigger_mode(crtc))) {
		unsigned int i, j;

		/*ToDo HFP/MIPI CLOCK solution*/
		DDPMSG("%s,Update RDMA golden_setting\n", __func__);

		/* Update RDMA golden_setting */
		cfg.w = crtc->state->mode.hdisplay;
		cfg.h = crtc->state->mode.vdisplay;
		cfg.vrefresh = fps_dst;
		cfg.bpc = mtk_crtc->bpc;
		cfg.p_golden_setting_context =
			__get_golden_setting_context(mtk_crtc);

		if (!(mode_chg_index & MODE_DSI_RES)) {
			for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
				mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					MTK_IO_CMD_RDMA_GOLDEN_SETTING, &cfg);
		}
	}
	mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, DSI_LFR_SET, &en);
	/* pull up mm clk if dst fps is higher than src fps */
	if (output_comp && fps_dst >= fps_src
		&& !mtk_crtc_is_frame_trigger_mode(crtc)) {
		if (mtk_crtc->qos_ctx)
			mtk_crtc->qos_ctx->last_mmclk_req_idx += 1;
		mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE,
				&en);
	}

	if ((mode_chg_index & MODE_DSI_RES)
		&& (mtk_crtc->res_switch == RES_SWITCH_ON_DDIC))
		mtk_crtc_mode_switch_config(mtk_crtc, old_state);
	else if ((mode_chg_index & MODE_DSI_RES)
		&& (mtk_crtc->res_switch == RES_SWITCH_ON_AP))
		mtk_crtc_mode_switch_on_ap_config(mtk_crtc, old_state);
	else if (output_comp) {/* Change DSI mipi clk & send LCM cmd */
		mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_TIMING_CHANGE,
				old_state);

		/* adjust trigger loop in different display mode */
		if (mtk_crtc_with_trigger_loop(crtc) &&
				mtk_crtc_with_event_loop(crtc) &&
				mtk_crtc_is_frame_trigger_mode(crtc)) {
			mtk_crtc_stop_trig_loop(crtc);
			mtk_crtc_stop_event_loop(crtc);

			mtk_crtc_start_event_loop(crtc);
			mtk_crtc_start_trig_loop(crtc);
		}
	}
	/* scaling path */
	oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];
	oddmr_timing.hdisplay = mtk_crtc_get_width_by_comp(__func__, crtc, oddmr_comp, false);
	oddmr_timing.vdisplay = mtk_crtc_get_height_by_comp(__func__, crtc, oddmr_comp, false);
	oddmr_timing.mode_chg_index = mode_chg_index;
	oddmr_timing.vrefresh = fps_dst;
	mtk_ddp_comp_io_cmd(oddmr_comp, cmdq_handle, ODDMR_TIMING_CHG, &oddmr_timing);

	drm_invoke_fps_chg_callbacks(fps_dst);

	/* update framedur_ns for VSYNC report */
	drm_calc_timestamping_constants(crtc, &crtc->state->mode);

	/* update idle timeout*/
	if (fps_dst > 0)
		_idle_timeout = mtk_crtc_get_idle_interval(crtc, fps_dst);
	if (_idle_timeout > 0)
		mtk_drm_set_idle_check_interval(crtc, _idle_timeout);

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	CRTC_MMP_EVENT_END((int) drm_crtc_index(crtc), mode_switch, fps_src, fps_dst);
	DDPMSG("%s--\n", __func__);
}

static void mtk_crtc_msync2_switch_begin(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *cevent_cmdq_handle, *sevent_cmdq_handle;
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_comp *output_comp;
	unsigned int crtc_id = drm_crtc_index(crtc);

	struct mtk_ddp_comp *oddmr_comp;
	unsigned int od_trigger = 0;

	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		return;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!output_comp) {
		DDPMSG("output_comp is null!\n");
		return;
	}

	mtk_crtc_pkt_create(&cevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cevent_cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cevent_cmdq_handle\n", __func__, __LINE__);
		return;
	}

	/* 1. wait frame done & wait DSI not busy */
	cmdq_pkt_wait_no_clear(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	/* Clear stream block to prevent trigger loop start */
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_clear_event(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
	cmdq_pkt_wfe(cevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_flush(cevent_cmdq_handle);
	cmdq_pkt_destroy(cevent_cmdq_handle);


	oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];

	if (mtk_crtc->msync2.msync_frame_status == 0)
		od_trigger = 1;

	mtk_ddp_comp_io_cmd(oddmr_comp, NULL, ODDMR_TRIG_CTL, &od_trigger);

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, NULL,
			FORCE_TRIG_CTL, &mtk_crtc->msync2.msync_frame_status);

	mtk_crtc_v_idle_apsrc_control(crtc, NULL, false, false, crtc_id, true);
	mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_PLL_SWITCH_REFERENCE_CNT_CTL,
		&mtk_crtc->msync2.msync_frame_status);

	if (mtk_crtc_with_trigger_loop(crtc) && mtk_crtc_with_event_loop(crtc)) {
		mtk_crtc_stop_trig_loop(crtc);
		mtk_crtc_stop_event_loop(crtc);

		mtk_crtc_start_event_loop(crtc);
		mtk_crtc_start_trig_loop(crtc);
	}

	/* set frame done */
	mtk_crtc_pkt_create(&sevent_cmdq_handle, &mtk_crtc->base,
				mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!sevent_cmdq_handle) {
		DDPPR_ERR("%s:%d NULL sevent_cmdq_handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
	cmdq_pkt_set_event(sevent_cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_flush(sevent_cmdq_handle);
	cmdq_pkt_destroy(sevent_cmdq_handle);
}

bool already_free;
bool mtk_crtc_frame_buffer_existed(void)
{
	DDPMSG("%s, frame buffer is freed:%d\n", __func__, already_free);
	return !already_free;
}

static int free_reserved_buf(phys_addr_t start_phys, phys_addr_t end_phys)
{
	phys_addr_t pos;

	WARN_ON(start_phys & ~PAGE_MASK);
	WARN_ON(end_phys & ~PAGE_MASK);

	if (end_phys <= start_phys) {
		DDPPR_ERR("%s end_phys:0x%lx is smaller than start_phys:0x%lx\n",
			__func__, (unsigned long)end_phys, (unsigned long)start_phys);

		return -1;
	}

	for (pos = start_phys; pos < end_phys; pos += PAGE_SIZE)
		free_reserved_page(phys_to_page(pos));

	return 0;
}

int free_fb_buf(void)
{
	phys_addr_t fb_base;
	unsigned int vramsize, fps;

	_parse_tag_videolfb(&vramsize, &fb_base, &fps);

	if (fb_base)
		free_reserved_buf(fb_base, fb_base + vramsize);
	else {
		DDPINFO("%s:get fb pa error\n", __func__);
		return -1;
	}

	return 0;
}

static void mtk_crtc_frame_buffer_release(struct drm_crtc *crtc,
		int index, bool hrt_valid)
{
#ifndef CONFIG_MTK_DISP_NO_LK
	struct drm_device *dev = NULL;

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		if (already_free == true || IS_ERR_OR_NULL(crtc))
			return;

		if (index == 0 && hrt_valid == true) {
			/*free fb buf after the 1st valid input buffer is unused*/
			DDPMSG("%s, free frame buffer\n", __func__);
			dev = crtc->dev;
			mtk_drm_fb_gem_release(dev);
			free_fb_buf();
			already_free = true;
		}
	}
#endif
}

void mtk_crtc_ovl_connect_change(struct drm_crtc *crtc, unsigned int ovl_res,
		struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_private *priv;
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_ddp_config cfg = {0};
	struct mtk_ddp_comp *comp, *output_comp;
	unsigned int i;
	unsigned int comp_id, ovl_comp_idx;
	unsigned int *comp_id_list = NULL, comp_id_nr;
	unsigned int ddp_mode;
	bool is_frame_mode;

	if (unlikely(!crtc)) {
		DDPPR_ERR("%s invalid CRTC\n", __func__);
		return;
	}

	if (unlikely(!crtc->dev)) {
		DDPPR_ERR("%s invalid DRM device\n", __func__);
		return;
	}

	priv = crtc->dev->dev_private;
	mtk_crtc = to_mtk_crtc(crtc);
	ddp_mode = mtk_crtc->ddp_mode;
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	is_frame_mode = mtk_crtc_is_frame_trigger_mode(crtc);

	if (mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] == 0) {
		DDPINFO("%s no valid ovl_comp\n", __func__);
		return;
	}
	cfg.w = crtc->state->adjusted_mode.hdisplay;
	cfg.h = crtc->state->adjusted_mode.vdisplay;
	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI) {
		cfg.w = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_WIDTH, NULL);
		cfg.h = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_HEIGH, NULL);
	}
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params &&
		mtk_crtc->panel_ext->params->dyn_fps.switch_en == 1
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		cfg.vrefresh =
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	else
		cfg.vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg.bpc = mtk_crtc->bpc;
	cfg.p_golden_setting_context = __get_golden_setting_context(mtk_crtc);

	cfg.tile_overhead = mtk_crtc_get_total_overhead(mtk_crtc);
	DDPINFO("%s:overhead is_support:%d, width L:%d R:%d\n", __func__,
		cfg.tile_overhead.is_support, cfg.tile_overhead.left_in_width,
		cfg.tile_overhead.right_in_width);

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		mtk_ddp_comp_stop(comp, cmdq_handle);
		mtk_disp_mutex_remove_comp_with_cmdq(mtk_crtc, comp->id, cmdq_handle, 0);
	}

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] - 1 ; ++i) {
		unsigned int cur, next;

		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		cur = comp->id;

		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i + 1];
		next = comp->id;

		mtk_ddp_remove_comp_from_path_with_cmdq(mtk_crtc, cur, next, cmdq_handle);
	}

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		comp->mtk_crtc = NULL;
	}

	comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH]
		[mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] - 1];
	ovl_comp_idx = 0;
	comp_id_nr = mtk_ddp_ovl_resource_list(priv, &comp_id_list);

	for (i = 0 ; i < comp_id_nr ; ++i) {
		if ((ovl_res & BIT(i)) == 0)
			continue;

		comp_id = comp_id_list[i];
		mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] =
			priv->ddp_comp[comp_id];
		++ovl_comp_idx;
	}
	mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] = comp;
	mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] = ovl_comp_idx + 1;

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		comp->mtk_crtc = mtk_crtc;
	}

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] - 1 ; ++i) {
		unsigned int cur, next, prev;

		if (i == 0) {
			prev = DDP_COMPONENT_ID_MAX;
		} else {
			comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i - 1];
			prev = comp->id;
		}

		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		cur = comp->id;

		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i + 1];
		next = comp->id;

		mtk_ddp_add_comp_to_path_with_cmdq(mtk_crtc, cur, prev, next, cmdq_handle);
		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
	}

	for (i = 0 ; i < mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		mtk_ddp_comp_config(comp, &cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);

		mtk_disp_mutex_add_comp_with_cmdq(mtk_crtc, comp->id,
				is_frame_mode, cmdq_handle, 0);
	}

	if (!mtk_crtc->is_dual_pipe)
		return;

	if (mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] == 0) {
		DDPINFO("%s no valid dual ovl_comp\n", __func__);
		return;
	}

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		mtk_ddp_comp_stop(comp, cmdq_handle);
		mtk_disp_mutex_remove_comp_with_cmdq(mtk_crtc, comp->id, cmdq_handle, 0);
	}

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] - 1; ++i) {
		unsigned int cur, next;

		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
		cur = comp->id;

		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i + 1];
		next = comp->id;

		mtk_ddp_remove_comp_from_path_with_cmdq(mtk_crtc, cur, next, cmdq_handle);
	}

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		comp->mtk_crtc = NULL;
	}

	comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH]
		[mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] - 1];
	ovl_comp_idx = 0;
	comp_id_nr = mtk_ddp_ovl_resource_list(priv, &comp_id_list);

	for (i = 0 ; i < comp_id_nr ; ++i) {
		if ((ovl_res & BIT(i)) == 0)
			continue;

		comp_id = dual_pipe_comp_mapping(priv->data->mmsys_id, comp_id_list[i]);
		mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] =
			priv->ddp_comp[comp_id];
		++ovl_comp_idx;
	}
	mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] = comp;
	mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] = ovl_comp_idx + 1;

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
		if (!comp) {
			DDPPR_ERR("%s errors with NULL comp\n", __func__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		comp->mtk_crtc = mtk_crtc;
	}

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] - 1; ++i) {
		unsigned int cur, next, prev;

		if (i == 0) {
			prev = DDP_COMPONENT_ID_MAX;
		} else {
			comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i - 1];
			prev = comp->id;
		}

		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
		cur = comp->id;

		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i + 1];
		next = comp->id;

		mtk_ddp_add_comp_to_path_with_cmdq(mtk_crtc, cur, prev, next, cmdq_handle);
		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];
	}

	for (i = 0 ; i < mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] ; ++i) {
		comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][i];

		if (!comp) {
			DDPPR_ERR("%s:%d errors with NULL comp\n", __func__, __LINE__);
			continue;
		} else if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			continue;

		mtk_ddp_comp_config(comp, &cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);

		mtk_disp_mutex_add_comp_with_cmdq(mtk_crtc, comp->id,
				is_frame_mode, cmdq_handle, 0);
	}

}

static void mtk_crtc_update_ddp_state(struct drm_crtc *crtc,
				      struct drm_crtc_state *old_crtc_state,
				      struct mtk_crtc_state *crtc_state,
				      struct cmdq_pkt *cmdq_handle)
{
	struct mtk_crtc_state *old_mtk_state =
		to_mtk_crtc_state(old_crtc_state);
	struct mtk_drm_lyeblob_ids *lyeblob_ids, *next;
	struct mtk_drm_private *mtk_drm = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct list_head *lyeblob_head;
	int index = drm_crtc_index(crtc);
	int crtc_mask = 0x1 << index;
	unsigned int prop_lye_idx;
	unsigned int pan_disp_frame_weight = 4;
	bool hrt_valid = false;
	int sphrt_enable;

	mutex_lock(&mtk_drm->lyeblob_list_mutex);
	prop_lye_idx = crtc_state->prop_val[CRTC_PROP_LYE_IDX];
	sphrt_enable = mtk_drm_helper_get_opt(mtk_drm->helper_opt, MTK_DRM_OPT_SPHRT);
	lyeblob_head = (sphrt_enable == 0) ? (&mtk_drm->lyeblob_head) : (&mtk_crtc->lyeblob_head);
	list_for_each_entry_safe(lyeblob_ids, next, lyeblob_head,
				 list) {
		if (lyeblob_ids->lye_idx > prop_lye_idx) {
			DDPMSG("lyeblob lost ID:%d\n", prop_lye_idx);
			break;
		} else if (lyeblob_ids->lye_idx == prop_lye_idx) {
			if (index == 0 || sphrt_enable) {
				mtk_crtc_disp_mode_switch_begin(crtc,
					old_crtc_state, crtc_state,
					cmdq_handle);

				hrt_valid = lyeblob_ids->hrt_valid;
				if (hrt_valid == true) {
					mtk_crtc_update_hrt_state(
						crtc, lyeblob_ids->frame_weight,
						lyeblob_ids, cmdq_handle);
					DRM_MMP_MARK(layering_blob, lyeblob_ids->lye_idx,
						lyeblob_ids->frame_weight | 0xffff0000);
				}
			}

			if (mtk_drm_helper_get_opt(mtk_drm->helper_opt, MTK_DRM_OPT_SPHRT) &&
					mtk_drm->pre_defined_bw[index] == 0xffffffff) {
				unsigned int i, req_ovl;
				unsigned int usage_list = lyeblob_ids->disp_status;

				req_ovl = mtk_drm->ovl_usage[index];
				for (i = 0 ; i < MAX_CRTC ; ++i) {
					if ((usage_list & BIT(i)) == 0 || i == index)
						continue;
					req_ovl = req_ovl & ~mtk_drm->ovl_usage[i];
				}

				if (mtk_crtc->ovl_usage_status != req_ovl) {
					DDPINFO("req_ovl %x usage_list %x\n", req_ovl, usage_list);
					mtk_crtc_ovl_connect_change(crtc, req_ovl, cmdq_handle);
					mtk_crtc->ovl_usage_status = req_ovl;
				}
				// set DISP_ENABLE if this display is checked in hrt
				old_mtk_state->pending_usage_update = true;
				old_mtk_state->pending_usage_list = usage_list;
			} else {
				old_mtk_state->pending_usage_update = false;
			}
			mtk_crtc_get_plane_comp_state(crtc, cmdq_handle);
			mtk_crtc_atomic_ddp_config(crtc, lyeblob_ids,
						   cmdq_handle);
			break;
		} else if (lyeblob_ids->lye_idx < prop_lye_idx) {
			if (lyeblob_ids->ref_cnt) {
				DDPINFO("free:(0x%x,0x%x), cnt:%d\n",
					 lyeblob_ids->free_cnt_mask,
					 crtc_mask,
					 lyeblob_ids->ref_cnt);
				if (lyeblob_ids->free_cnt_mask & crtc_mask) {
					lyeblob_ids->free_cnt_mask &=
						(~crtc_mask);
					lyeblob_ids->ref_cnt--;
					DDPINFO("free:(0x%x,0x%x), cnt:%d\n",
						 lyeblob_ids->free_cnt_mask,
						 crtc_mask,
						 lyeblob_ids->ref_cnt);
				}
			}
			if (!lyeblob_ids->ref_cnt) {
				mtk_crtc_frame_buffer_release(crtc, index,
					lyeblob_ids->hrt_valid);
				DDPINFO("free lyeblob:(%d,%d)\n",
					lyeblob_ids->lye_idx,
					prop_lye_idx);
				mtk_crtc_free_lyeblob_ids(crtc,
						lyeblob_ids);
				mtk_crtc_free_ddpblob_ids(crtc,
						lyeblob_ids);
			}
		}
	}
	/*set_hrt_bw for pan display ,set 4 for two RGB layer*/
	if ((index == 0 || mtk_crtc->path_data->is_discrete_path) && hrt_valid == false) {
		if (mtk_drm_helper_get_opt(mtk_drm->helper_opt, MTK_DRM_OPT_HRT))
			DDPMSG("%s frame:%u correct invalid hrt to:%u, mode:%u->%u\n",
				__func__, prop_lye_idx, pan_disp_frame_weight,
				old_mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
				crtc_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
		/*
		 * prop_lye_idx is 0 when suspend. Update display mode to avoid
		 * the dsi params not sync with the mode of new crtc state.
		 */
		mtk_crtc_disp_mode_switch_begin(crtc,
			old_crtc_state, crtc_state,
			cmdq_handle);
		mtk_crtc_update_hrt_state(crtc, pan_disp_frame_weight, NULL,
			cmdq_handle);
		DRM_MMP_MARK(layering_blob, 0,
			pan_disp_frame_weight | 0xffff0000);
	}
	mutex_unlock(&mtk_drm->lyeblob_list_mutex);
}

#ifdef MTK_DRM_FENCE_SUPPORT
static void mtk_crtc_release_lye_idx(struct drm_crtc *crtc)
{
	struct mtk_drm_lyeblob_ids *lyeblob_ids, *next;
	struct mtk_drm_private *mtk_drm = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct list_head *lyeblob_head;
	int index = drm_crtc_index(crtc);
	int crtc_mask = 0x1 << index;
	int sphrt_enable;

	mutex_lock(&mtk_drm->lyeblob_list_mutex);
	sphrt_enable = mtk_drm_helper_get_opt(mtk_drm->helper_opt, MTK_DRM_OPT_SPHRT);
	lyeblob_head = (sphrt_enable == 0) ? (&mtk_drm->lyeblob_head) : (&mtk_crtc->lyeblob_head);
	list_for_each_entry_safe(lyeblob_ids, next, lyeblob_head,
		list) {
		if (lyeblob_ids->ref_cnt) {
			DDPINFO("%s:%d free:(0x%x,0x%x), cnt:%d\n",
				__func__, __LINE__,
				lyeblob_ids->free_cnt_mask,
				crtc_mask,
				lyeblob_ids->ref_cnt);
			if (lyeblob_ids->free_cnt_mask & crtc_mask) {
				lyeblob_ids->free_cnt_mask &= (~crtc_mask);
				lyeblob_ids->ref_cnt--;
				DDPINFO("%s:%d free:(0x%x,0x%x), cnt:%d\n",
					__func__, __LINE__,
					lyeblob_ids->free_cnt_mask,
					crtc_mask,
					lyeblob_ids->ref_cnt);
			}
		}
		if (!lyeblob_ids->ref_cnt) {
			DDPINFO("%s:%d free lyeblob:%d\n",
				__func__, __LINE__,
				lyeblob_ids->lye_idx);
			mtk_crtc_free_lyeblob_ids(crtc,
				lyeblob_ids);
			mtk_crtc_free_ddpblob_ids(crtc,
				lyeblob_ids);
		}
	}
	mutex_unlock(&mtk_drm->lyeblob_list_mutex);
}
#endif
#endif

bool mtk_crtc_with_trigger_loop(struct drm_crtc *crtc)
{
#ifndef DRM_CMDQ_DISABLE
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP])
		return true;
#endif
	return false;
}

/* sw workaround to fix gce hw bug */
bool mtk_crtc_with_sodi_loop(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;

	priv = mtk_crtc->base.dev->dev_private;
	if (mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP])
		return true;
	return false;
}

bool mtk_crtc_with_event_loop(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;

	priv = mtk_crtc->base.dev->dev_private;

	if (!(mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_PREFETCH_TE) ||
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_APSRC_OFF) ||
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_DSI_PLL_OFF) ||
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_CHECK_TRIGGER_MERGE)
		))
		return false;

	if (mtk_crtc->gce_obj.client[CLIENT_EVENT_LOOP])
		return true;
	return false;
}

bool mtk_crtc_is_frame_trigger_mode(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv =
		(crtc && crtc->dev && crtc->dev->dev_private) ?
		crtc->dev->dev_private : NULL;
	struct mtk_drm_crtc *mtk_crtc = crtc ? to_mtk_crtc(crtc) : NULL;
	int crtc_id = crtc ? drm_crtc_index(crtc) : (-EINVAL);
	struct mtk_ddp_comp *comp = NULL;
	int i;

	if (!priv || !mtk_crtc || (crtc_id == -EINVAL)) {
		DDPPR_ERR("%s, Cannot find priv or mtk_crtc, crtc_id:%d\n",
							__func__, crtc_id);
		return false;
	}

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (crtc_id == 0 && comp)
		return mtk_dsi_is_cmd_mode(comp);
	else if (crtc_id == 0)
		return mtk_dsi_is_cmd_mode(priv->ddp_comp[DDP_COMPONENT_DSI0]);

	for_each_comp_in_crtc_target_path(
		comp, mtk_crtc, i,
		DDP_FIRST_PATH)
		if (mtk_ddp_comp_is_output(comp))
			break;

	if (!comp) {
		DDPPR_ERR("%s, Cannot find output component\n", __func__);
		return false;
	}

	if (mtk_ddp_comp_get_type(comp->id) == MTK_DSI)
		return mtk_dsi_is_cmd_mode(priv->ddp_comp[comp->id]);

	if (comp->id == DDP_COMPONENT_DP_INTF0 ||
		comp->id == DDP_COMPONENT_DPI0 ||
		comp->id == DDP_COMPONENT_DPI1) {
		return false;
	}

	return true;
}

static bool mtk_crtc_target_is_dc_mode(struct drm_crtc *crtc,
				unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (ddp_mode >= DDP_MODE_NR)
		return false;

	if (mtk_crtc->ddp_ctx[ddp_mode].ddp_comp_nr[1])
		return true;
	return false;
}

static bool mtk_crtc_support_dc_mode(struct drm_crtc *crtc)
{
	int i;

	for (i = 0; i < DDP_MODE_NR; i++)
		if (mtk_crtc_target_is_dc_mode(crtc, i))
			return true;
	return false;
}

bool mtk_crtc_is_dc_mode(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	return mtk_crtc_target_is_dc_mode(crtc, mtk_crtc->ddp_mode);
}

bool mtk_crtc_is_mem_mode(struct drm_crtc *crtc)
{
	/* for find memory session */
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (!comp)
		return false;

	if (comp->id == DDP_COMPONENT_WDMA0 ||
		comp->id == DDP_COMPONENT_WDMA1 ||
		comp->id == DDP_COMPONENT_OVLSYS_WDMA2)
		return true;

	return false;
}

int get_comp_wait_event(struct mtk_drm_crtc *mtk_crtc,
			struct mtk_ddp_comp *comp)
{
	if (comp->id == DDP_COMPONENT_DSI0 || comp->id == DDP_COMPONENT_DSI1) {
		if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
			return mtk_crtc->gce_obj.event[EVENT_STREAM_EOF];
		else
			return mtk_crtc->gce_obj.event[EVENT_CMD_EOF];

	} else if (comp->id == DDP_COMPONENT_DP_INTF0) {
		return mtk_crtc->gce_obj.event[EVENT_VDO_EOF];
	} else if (comp->id == DDP_COMPONENT_WDMA0) {
		return mtk_crtc->gce_obj.event[EVENT_WDMA0_EOF];
	} else if (comp->id == DDP_COMPONENT_WDMA1) {
		return mtk_crtc->gce_obj.event[EVENT_WDMA1_EOF];
	} else if (comp->id == DDP_COMPONENT_OVLSYS_WDMA2) {
		return mtk_crtc->gce_obj.event[EVENT_OVLSYS1_WDMA0_EOF];
	} else if (comp->id == DDP_COMPONENT_MDP_RDMA0) {
		return mtk_crtc->gce_obj.event[EVENT_MDP_RDMA0_EOF];
	} else if (comp->id == DDP_COMPONENT_MDP_RDMA1) {
		return mtk_crtc->gce_obj.event[EVENT_MDP_RDMA1_EOF];
	}

	DDPPR_ERR("The component has not frame done event\n");
	return -EINVAL;
}

int get_path_wait_event(struct mtk_drm_crtc *mtk_crtc,
			enum CRTC_DDP_PATH ddp_path)
{
	struct mtk_ddp_comp *comp = NULL;
	int i;

	if (ddp_path < 0 || ddp_path >= DDP_PATH_NR) {
		DDPPR_ERR("%s, invalid ddp_path value\n", __func__);
		return -EINVAL;
	}

	for_each_comp_in_crtc_target_path(
		comp, mtk_crtc, i,
		ddp_path)
		if (mtk_ddp_comp_is_output(comp))
			break;

	if (!comp) {
		DDPPR_ERR("%s, Cannot find output component\n", __func__);
		return -EINVAL;
	}

	return get_comp_wait_event(mtk_crtc, comp);
}

void mtk_crtc_clr_comp_done(struct mtk_drm_crtc *mtk_crtc,
			      struct cmdq_pkt *cmdq_handle,
			      struct mtk_ddp_comp *comp,
			      int clear_event)
{
	int gce_event;

	gce_event = get_comp_wait_event(mtk_crtc, comp);
	if (gce_event < 0)
		return;
	if (gce_event == mtk_crtc->gce_obj.event[EVENT_MDP_RDMA0_EOF] ||
		gce_event == mtk_crtc->gce_obj.event[EVENT_MDP_RDMA1_EOF])
		cmdq_pkt_clear_event(cmdq_handle, gce_event);
	else
		DDPPR_ERR("The component has not frame done event\n");
}

void mtk_crtc_wait_comp_done(struct mtk_drm_crtc *mtk_crtc,
			      struct cmdq_pkt *cmdq_handle,
			      struct mtk_ddp_comp *comp,
			      int clear_event)
{
	int gce_event;

	gce_event = get_comp_wait_event(mtk_crtc, comp);
	if (gce_event < 0)
		return;
	if (gce_event == mtk_crtc->gce_obj.event[EVENT_MDP_RDMA0_EOF] ||
		gce_event == mtk_crtc->gce_obj.event[EVENT_MDP_RDMA1_EOF])
		cmdq_pkt_wfe(cmdq_handle, gce_event);
	else
		DDPPR_ERR("The component has not frame done event\n");
}

void mtk_crtc_wait_frame_done(struct mtk_drm_crtc *mtk_crtc,
			      struct cmdq_pkt *cmdq_handle,
			      enum CRTC_DDP_PATH ddp_path,
			      int clear_event)
{
	int gce_event;

	gce_event = get_path_wait_event(mtk_crtc, ddp_path);
	if (gce_event < 0)
		return;
	if (gce_event == mtk_crtc->gce_obj.event[EVENT_STREAM_EOF] ||
	    gce_event == mtk_crtc->gce_obj.event[EVENT_CMD_EOF] ||
	    gce_event == mtk_crtc->gce_obj.event[EVENT_VDO_EOF]) {
		struct mtk_drm_private *priv;

		if (clear_event)
			cmdq_pkt_wfe(cmdq_handle, gce_event);
		else
			cmdq_pkt_wait_no_clear(cmdq_handle, gce_event);
		priv = mtk_crtc->base.dev->dev_private;
		if (gce_event == mtk_crtc->gce_obj.event[EVENT_CMD_EOF] &&
		    mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_LAYER_REC) &&
		    mtk_crtc->layer_rec_en) {
			cmdq_pkt_wait_no_clear(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		}
	} else if (gce_event == mtk_crtc->gce_obj.event[EVENT_WDMA0_EOF]) {
		/* Must clear WDMA_EOF in decouple mode */
		if (mtk_crtc_is_dc_mode(&mtk_crtc->base))
			cmdq_pkt_wfe(cmdq_handle, gce_event);
	} else if (gce_event == mtk_crtc->gce_obj.event[EVENT_WDMA1_EOF]) {
		if (mtk_crtc_is_dc_mode(&mtk_crtc->base))
			cmdq_pkt_wfe(cmdq_handle, gce_event);
	} else  if (gce_event == mtk_crtc->gce_obj.event[EVENT_OVLSYS1_WDMA0_EOF]) {
		if (mtk_crtc_is_dc_mode(&mtk_crtc->base))
			cmdq_pkt_wfe(cmdq_handle, gce_event);
	} else
		DDPPR_ERR("The output component has not frame done event\n");
}

static int _mtk_crtc_cmdq_retrig(void *data)
{
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *) data;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct sched_param param = {.sched_priority = 94 };
	int ret;

	sched_setscheduler(current, SCHED_RR, &param);

	atomic_set(&mtk_crtc->cmdq_trig, 0);
	while (1) {
		ret = wait_event_interruptible(mtk_crtc->trigger_cmdq,
			atomic_read(&mtk_crtc->cmdq_trig));
		if (ret < 0)
			DDPPR_ERR("wait %s fail, ret=%d\n", __func__, ret);
		atomic_set(&mtk_crtc->cmdq_trig, 0);

		mtk_crtc_clear_wait_event(crtc);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static void mtk_crtc_cmdq_timeout_cb(struct cmdq_cb_data data)
{
	struct drm_crtc *crtc = data.data;

#ifndef DRM_CMDQ_DISABLE
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_client *cl;
	dma_addr_t trig_pc;
	u64 *inst;
#endif

	if (!crtc) {
		DDPPR_ERR("%s find crtc fail\n", __func__);
		return;
	}

	DDPPR_ERR("%s cmdq timeout, crtc id:%d\n", __func__,
		drm_crtc_index(crtc));
	mtk_drm_crtc_analysis(crtc);
	mtk_drm_crtc_dump(crtc);

#ifndef DRM_CMDQ_DISABLE
	if ((mtk_crtc->trig_loop_cmdq_handle) &&
			(mtk_crtc->trig_loop_cmdq_handle->cl)) {
		cl = (struct cmdq_client *)mtk_crtc->trig_loop_cmdq_handle->cl;
		DDPMSG("++++++ Dump trigger loop ++++++\n");
		cmdq_thread_dump(cl->chan, mtk_crtc->trig_loop_cmdq_handle,
				&inst, &trig_pc);
		cmdq_dump_pkt(mtk_crtc->trig_loop_cmdq_handle, trig_pc, true);

		DDPMSG("------ Dump trigger loop ------\n");
	}
	atomic_set(&mtk_crtc->cmdq_trig, 1);
#endif

	/* CMDQ driver would not trigger aee when timeout. */
	DDPAEE("%s cmdq timeout, crtc id:%d\n", __func__, drm_crtc_index(crtc));
}

void mtk_crtc_pkt_create(struct cmdq_pkt **cmdq_handle, struct drm_crtc *crtc,
	struct cmdq_client *cl)
{
	*cmdq_handle = cmdq_pkt_create(cl);
	if (IS_ERR_OR_NULL(*cmdq_handle)) {
		DDPPR_ERR("%s create handle fail, %x\n",
				__func__, *cmdq_handle);
		return;
	}

	(*cmdq_handle)->err_cb.cb = mtk_crtc_cmdq_timeout_cb;
	(*cmdq_handle)->err_cb.data = crtc;
}

static void sub_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(cb_data->crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	int session_id = -1, id = drm_crtc_index(cb_data->crtc), i;
	unsigned int intr_fence = 0;

	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if ((id + 1) == MTK_SESSION_TYPE(priv->session_id[i])) {
			session_id = priv->session_id[i];
			break;
		}
	}

	/* Release output buffer fence */
	if (priv->power_state)
		intr_fence = *(unsigned int *)
			(mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_CUR_INTERFACE_FENCE));

	if (intr_fence >= 1) {
		DDPINFO("intr fence_idx:%d\n", intr_fence);
		mtk_release_fence(session_id,
			mtk_fence_get_interface_timeline_id(), intr_fence - 1);
	}

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

void mtk_crtc_release_output_buffer_fence_by_idx(
	struct drm_crtc *crtc, int session_id, unsigned int fence_idx)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;

	/* fence release already during suspend */
	if (priv->power_state == false)
		return;

	if (fence_idx && fence_idx != -1) {
		DDPINFO("output fence_idx:%d\n", fence_idx);
		mtk_release_fence(session_id,
			mtk_fence_get_output_timeline_id(), fence_idx);
	}
}

void mtk_crtc_release_output_buffer_fence(
	struct drm_crtc *crtc, int session_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	unsigned int fence_idx = 0;

	/* fence release already during suspend */
	if (priv->power_state == false)
		return;

	fence_idx = *(unsigned int *)
		mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_CUR_OUTPUT_FENCE);
	if (fence_idx && fence_idx != -1) {
		DDPINFO("output fence_idx:%d\n", fence_idx);
		mtk_release_fence(session_id,
			mtk_fence_get_output_timeline_id(), fence_idx);
	}
}

struct drm_framebuffer *mtk_drm_framebuffer_lookup(struct drm_device *dev,
	unsigned int id)
{
	struct drm_framebuffer *fb = NULL;

	fb = drm_framebuffer_lookup(dev, NULL, id);

	if (!fb)
		return NULL;

	/* CRITICAL: drop the reference we picked up in framebuffer lookup */
	drm_framebuffer_put(fb);

	return fb;
}

#ifdef IF_ZERO /* not ready for dummy register method */
static void mtk_crtc_dc_config_color_matrix(struct drm_crtc *crtc,
				struct cmdq_pkt *cmdq_handle)
{
	int i, j, mode, ccorr_matrix[16], all_zero = 1;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt_buffer *cmdq_buf = &(mtk_crtc->gce_obj.buf);
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	bool set = false;
	struct mtk_ddp_comp *comp_ccorr;

	/* Get color matrix data from backup slot*/
	mode = *(int *)(cmdq_buf->va_base + DISP_SLOT_COLOR_MATRIX_PARAMS(0));
	for (i = 0; i < 16; i++)
		ccorr_matrix[i] = *(int *)(cmdq_buf->va_base +
					DISP_SLOT_COLOR_MATRIX_PARAMS(i + 1));

	for (i = 0; i <= 15; i += 5) {
		if (ccorr_matrix[i] != 0) {
			all_zero = 0;
			break;
		}
	}

	if (all_zero)
		DDPPR_ERR("CCORR color matrix backup param is zero matrix\n");
	else {
		struct mtk_ddp_comp *comp;

		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i, DDP_SECOND_PATH) {
			if (comp->id == DDP_COMPONENT_CCORR0) {
				disp_ccorr_set_color_matrix(comp, cmdq_handle,
					ccorr_matrix, mode, false, g_ccorr_linear);
				set = true;
				break;
			}
		}
		if (mtk_crtc->is_dual_pipe) {
			i = 0;
			j = 0;
			for_each_comp_in_dual_pipe(comp_ccorr, mtk_crtc, i, j) {
				if (((drm_ccorr_caps.ccorr_number == 1) &&
					(comp_ccorr->id == DDP_COMPONENT_CCORR1)) ||
					((drm_ccorr_caps.ccorr_number == 2) &&
					(comp_ccorr->id == DDP_COMPONENT_CCORR2))) {
					disp_ccorr_set_color_matrix(comp_ccorr, cmdq_handle,
						ccorr_matrix, mode, false, g_ccorr_linear);
					set = true;
					break;
				}
			}
		}

		if (!set)
			DDPPR_ERR("Cannot not find DDP_COMPONENT_CCORR0\n");
	}
}
#endif

void mtk_crtc_dc_prim_path_update(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct mtk_plane_state plane_state;
	struct drm_framebuffer *fb;
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	struct mtk_cmdq_cb_data *cb_data;
	unsigned int fb_idx, fb_id;
	int session_id;

	DDPINFO("%s+\n", __func__);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (!mtk_crtc_is_dc_mode(crtc)) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return;
	}

	session_id = mtk_get_session_id(crtc);
	mtk_crtc_release_output_buffer_fence(crtc, session_id);

	/* find fb for RDMA */
	fb_idx = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_RDMA_FB_IDX);
	fb_id = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_RDMA_FB_ID);

	/* 1-to-2*/
	if (!fb_id)
		goto end;

	fb = mtk_drm_framebuffer_lookup(mtk_crtc->base.dev, fb_id);
	if (fb == NULL) {
		DDPPR_ERR("%s cannot find fb fb_id:%u\n", __func__, fb_id);
		goto end;
	}

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

		DDPPR_ERR("cb data creation failed\n");
		return;
	}

	mtk_crtc_pkt_create(&cmdq_handle, crtc,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_wait_no_clear(cmdq_handle,
			       get_path_wait_event(mtk_crtc, DDP_FIRST_PATH));
	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_SECOND_PATH, 0);

	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];

	plane_state.pending.enable = true;
	plane_state.pending.pitch = fb->pitches[0];
	plane_state.pending.format = fb->format->format;
	plane_state.pending.addr =
		mtk_fb_get_dma(fb) +
		(dma_addr_t)mtk_crtc_get_dc_fb_size(crtc) *
		(dma_addr_t)fb_idx;
	plane_state.pending.size = mtk_fb_get_size(fb);
	plane_state.pending.src_x = 0;
	plane_state.pending.src_y = 0;
	plane_state.pending.dst_x = 0;
	plane_state.pending.dst_y = 0;
	plane_state.pending.width = fb->width;
	plane_state.pending.height = fb->height;
	mtk_ddp_comp_layer_config(ddp_ctx->ddp_comp[DDP_SECOND_PATH][0], 0,
				  &plane_state, cmdq_handle);

	/* support DC with color matrix no more */
	/* mtk_crtc_dc_config_color_matrix(crtc, cmdq_handle);*/
	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

	cb_data->crtc = crtc;
	cb_data->cmdq_handle = cmdq_handle;
	if (cmdq_pkt_flush_threaded(cmdq_handle, sub_cmdq_cb, cb_data) < 0)
		DDPPR_ERR("failed to flush sub\n");
end:
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}

#ifndef DRM_CMDQ_DISABLE
static void mtk_crtc_release_input_layer_fence(
	struct drm_crtc *crtc, int session_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	int i;
	unsigned int fence_idx = 0;

	/* fence release already during suspend */
	if (priv->power_state == false)
		return;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		unsigned int subtractor = 0;

		fence_idx = *(unsigned int *)
			mtk_get_gce_backup_slot_va(mtk_crtc,
			DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, i)));
		subtractor = *(unsigned int *)
			mtk_get_gce_backup_slot_va(mtk_crtc,
			DISP_SLOT_SUBTRACTOR_WHEN_FREE(mtk_get_plane_slot_idx(mtk_crtc, i)));
		subtractor &= 0xFFFF;
		if (drm_crtc_index(crtc) == 2)
			DDPINFO("%d, fence_idx:%d, subtractor:%d\n",
					i, fence_idx, subtractor);
		mtk_release_fence(session_id, i, fence_idx - subtractor);
	}
}

static void mtk_crtc_update_hrt_qos(struct drm_crtc *crtc,
		unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv =
			mtk_crtc->base.dev->dev_private;
	struct mtk_ddp_comp *comp;
	unsigned int cur_hrt_bw, hrt_idx, crtc_idx;
	int i, j;

	crtc_idx = drm_crtc_index(crtc);
	if (crtc_idx < MAX_CRTC && priv->usage[crtc_idx] == DISP_ENABLE) {
		for_each_comp_in_target_ddp_mode_bound(comp, mtk_crtc,
				i, j, ddp_mode, 0)
			mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_BW, NULL);
	}

	if (priv->power_state == false)
		return;

	hrt_idx = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_CUR_HRT_IDX);
	atomic_set(&mtk_crtc->qos_ctx->last_hrt_idx, hrt_idx);
	atomic_set(&mtk_crtc->qos_ctx->hrt_cond_sig, 1);
	/* adjust hrt usage callback only for primary display */
	if (crtc_idx == 0)
		wake_up(&mtk_crtc->qos_ctx->hrt_cond_wq);
	cur_hrt_bw = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_CUR_HRT_LEVEL);
	if (cur_hrt_bw != NO_PENDING_HRT &&
		cur_hrt_bw <= mtk_crtc->qos_ctx->last_hrt_req) {

		DDPINFO("CRTC%u cur:%u last:%u, release HRT to last_hrt_req:%u\n",
			crtc_idx, cur_hrt_bw,	mtk_crtc->qos_ctx->last_hrt_req,
			mtk_crtc->qos_ctx->last_hrt_req);

		if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_MMQOS_SUPPORT))
			mtk_disp_set_hrt_bw(mtk_crtc,
					mtk_crtc->qos_ctx->last_hrt_req);

		*(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc, DISP_SLOT_CUR_HRT_LEVEL) =
				NO_PENDING_HRT;
	}
}

static void mtk_drm_ovl_bw_monitor_ratio_prework(struct drm_crtc *crtc,
		struct drm_atomic_state *atomic_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(atomic_state, crtc);
	struct mtk_crtc_state *old_mtk_state =
		to_mtk_crtc_state(old_crtc_state);
	unsigned int frame_idx = old_mtk_state->prop_val[CRTC_PROP_OVL_DSI_SEQ];
	unsigned int plane_mask = old_crtc_state->plane_mask;
	struct drm_plane *plane = NULL;
	int i = 0;
	int prop_lye_idx = old_mtk_state->prop_val[CRTC_PROP_LYE_IDX];
	struct mtk_drm_lyeblob_ids *lyeblob_ids, *next;
	struct mtk_drm_private *mtk_drm = crtc->dev->dev_private;
	int fbt_gles_head = -1;
	int fbt_gles_tail = -1;
	int fbt_layer_id = -1;

	for (i = 0; i < MAX_LAYER_RATIO_NUMBER; i++) {
		display_compress_ratio_table[i].frame_idx = 0;
		display_compress_ratio_table[i].key_value = 0;
		display_compress_ratio_table[i].valid = 0;
		display_compress_ratio_table[i].average_ratio = NULL;
		display_compress_ratio_table[i].peak_ratio = NULL;
		display_compress_ratio_table[i].active = 0;
	}

	display_fbt_compress_ratio_table.frame_idx = 0;
	display_fbt_compress_ratio_table.key_value = 0;
	display_fbt_compress_ratio_table.valid = 0;
	display_fbt_compress_ratio_table.average_ratio = NULL;
	display_fbt_compress_ratio_table.peak_ratio = NULL;
	display_fbt_compress_ratio_table.active = 0;

	list_for_each_entry_safe(lyeblob_ids, next, &mtk_drm->lyeblob_head,
				 list) {
		if (lyeblob_ids->lye_idx == prop_lye_idx) {
			fbt_gles_head = lyeblob_ids->fbt_gles_head;
			fbt_gles_tail = lyeblob_ids->fbt_gles_tail;
			fbt_layer_id = lyeblob_ids->fbt_layer_id;
			mtk_crtc->fbt_layer_id = fbt_layer_id;
			break;
		}
	}

	drm_for_each_plane_mask(plane, crtc->dev, plane_mask) {
		unsigned int plane_index = to_crtc_plane_index(plane->index);
		struct mtk_plane_state *plane_state =
			to_mtk_plane_state(plane->state);
		int index = plane_index;
		int is_active = 0;
		int need_skip = 0;

		DDPDBG("BWM: layer caps:0x%08x\n", plane_state->comp_state.layer_caps);
		if ((plane_state->comp_state.layer_caps & MTK_HWC_UNCHANGED_LAYER) ||
			(plane_state->comp_state.layer_caps & MTK_HWC_INACTIVE_LAYER) ||
			(plane_state->comp_state.layer_caps & MTK_HWC_UNCHANGED_FBT_LAYER)) {
			is_active = 0;
			if (!g_ovl_bwm_debug)
				need_skip = 0;
		} else {
			is_active = 1;
			if (!g_ovl_bwm_debug)
				need_skip = 1;
		}

		if (((plane_state->comp_state.layer_caps & MTK_DISP_UNCHANGED_RATIO_VALID) ||
			(plane_state->comp_state.layer_caps & MTK_DISP_FBT_RATIO_VALID)) &&
			(need_skip == 0) && !g_ovl_bwm_debug)
			need_skip = 1;

		DDPDBG("BWM: need skip:%d\n", need_skip);
		if ((fbt_gles_head != -1) && (fbt_gles_tail != -1) && (need_skip != 1)) {

			if ((plane_index == fbt_layer_id) &&
					(plane_index == fbt_gles_head) &&
					(fbt_layer_id != -1) &&
					(plane_state->pending.enable)) {
				display_fbt_compress_ratio_table.frame_idx = frame_idx;
				display_fbt_compress_ratio_table.key_value = frame_idx;
				display_fbt_compress_ratio_table.valid = 0;
				display_fbt_compress_ratio_table.average_ratio =
					(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_LAYER_AVG_RATIO(index)));
				display_fbt_compress_ratio_table.peak_ratio =
					(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_LAYER_PEAK_RATIO(index)));
				if (is_active)
					display_fbt_compress_ratio_table.active = 1;

			} else if ((plane_index < MAX_LAYER_RATIO_NUMBER) &&
					(plane_state->pending.enable) && (index >= 0)) {
				display_compress_ratio_table[index].frame_idx = frame_idx;
				display_compress_ratio_table[index].key_value = frame_idx +
					plane_state->prop_val[PLANE_PROP_BUFFER_ALLOC_ID];
				display_compress_ratio_table[index].valid = 0;
				display_compress_ratio_table[index].average_ratio =
					(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_LAYER_AVG_RATIO(index)));
				display_compress_ratio_table[index].peak_ratio =
					(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_LAYER_PEAK_RATIO(index)));
				if (is_active)
					display_compress_ratio_table[index].active = 1;
			}
		} else if ((plane_index < MAX_LAYER_RATIO_NUMBER) &&
				(plane_state->pending.enable) && (need_skip != 1) && (index >= 0)) {
			display_compress_ratio_table[index].frame_idx = frame_idx;
			display_compress_ratio_table[index].key_value = frame_idx +
				plane_state->prop_val[PLANE_PROP_BUFFER_ALLOC_ID];
			display_compress_ratio_table[index].valid = 0;
			display_compress_ratio_table[index].average_ratio =
				(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
				DISP_SLOT_LAYER_AVG_RATIO(index)));
			display_compress_ratio_table[index].peak_ratio =
				(unsigned int *)(mtk_get_gce_backup_slot_va(mtk_crtc,
				DISP_SLOT_LAYER_PEAK_RATIO(index)));
			if (is_active)
				display_compress_ratio_table[index].active = 1;
		}

		DDPDBG("BWM: frame idx:%d alloc_id:%lu plane_index:%u enable:%u\n",
				frame_idx, plane_state->prop_val[PLANE_PROP_BUFFER_ALLOC_ID],
				plane_index, plane_state->pending.enable);
		DDPDBG("BWM: fn:%u index:%d fbt_layer_id:%d fbt_head:%d fbt_tail:%d\n",
				fn, index, fbt_layer_id, fbt_gles_head, fbt_gles_tail);
	}
}

static void mtk_drm_ovl_bw_monitor_ratio_get(struct drm_crtc *crtc,
		struct drm_atomic_state *atomic_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc->state);
	struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(atomic_state, crtc);
	struct drm_plane *plane = NULL;
	unsigned int plane_mask = 0;
	struct mtk_crtc_state *state = mtk_crtc_state;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	plane_mask = old_crtc_state->plane_mask;

	drm_for_each_plane_mask(plane, crtc->dev, plane_mask) {
		unsigned int plane_index = to_crtc_plane_index(plane->index);
		struct mtk_plane_state *plane_state =
			to_mtk_plane_state(plane->state);
		int index = plane_index;
		struct mtk_ddp_comp *comp =
			mtk_crtc_get_plane_comp(crtc, plane_state);
		int lye_id = plane_state->comp_state.lye_id;
		int ext_lye_id = plane_state->comp_state.ext_lye_id;
		unsigned int src_w = plane_state->pending.width;
		unsigned int src_h = plane_state->pending.height;
		unsigned int bpp =
			mtk_get_format_bpp(plane_state->pending.format);
		unsigned int is_compress =
			plane_state->pending.prop_val[PLANE_PROP_COMPRESS];
		dma_addr_t avg_slot = mtk_get_gce_backup_slot_pa(mtk_crtc,
			DISP_SLOT_LAYER_AVG_RATIO(index));
		dma_addr_t peak_slot = mtk_get_gce_backup_slot_pa(mtk_crtc,
			DISP_SLOT_LAYER_PEAK_RATIO(index));
		unsigned int expand = 1 << 24;
		unsigned int narrow = 14;
		unsigned int avg_inter_value = 0;
		unsigned int peak_inter_value = 0;
		struct cmdq_operand lop;
		struct cmdq_operand rop;
		int is_active = 0;
		int need_skip = 0;

		DDPDBG("BWM: layer caps:0x%08x\n", plane_state->comp_state.layer_caps);
		if ((plane_state->comp_state.layer_caps & MTK_HWC_UNCHANGED_LAYER) ||
			(plane_state->comp_state.layer_caps & MTK_HWC_INACTIVE_LAYER) ||
			(plane_state->comp_state.layer_caps & MTK_HWC_UNCHANGED_FBT_LAYER)) {
			is_active = 0;
			if (!g_ovl_bwm_debug)
				need_skip = 0;
		} else {
			is_active = 1;
			if (!g_ovl_bwm_debug)
				need_skip = 1;
		}

		if (((plane_state->comp_state.layer_caps & MTK_DISP_UNCHANGED_RATIO_VALID) ||
			(plane_state->comp_state.layer_caps & MTK_DISP_FBT_RATIO_VALID)) &&
			(need_skip == 0) && !g_ovl_bwm_debug)
			need_skip = 1;
		DDPDBG("BWM: need skip:%d\n", need_skip);

		if (!comp) {
			DDPPR_ERR("%s run next plane with NULL comp\n", __func__);
			break;
		}

		if (mtk_drm_helper_get_opt(priv->helper_opt,
			    MTK_DRM_OPT_PRIM_DUAL_PIPE))
			src_w = src_w/2;

		/*
		 * Layer SRT compress ratio =
		 * ((BURST_ACC*16*2^24)/(src_w*src_h*bpp))/2^14
		 * Layer HRT compress ratio =
		 * ((BURST_ACC_WIN_MAX*16*2^24)/(src_w*win_h*bpp))/2^14
		 * Attention: win_h = (reg_BURST_ACC_WIN_SIZE+1)*(fbdc_en?4:1)
		 */
		if (src_w * src_h * bpp)
			avg_inter_value = (16 * expand)/(src_w * src_h * bpp);
		else {
			need_skip = 1;
			DDPDBG("BWM: division by zero, need skip:%d\n", need_skip);
		}
		if (src_w * ovl_win_size * (is_compress?4:1) * bpp)
			peak_inter_value = (16 * expand)/
				(src_w * ovl_win_size * (is_compress?4:1) * bpp);
		else {
			need_skip = 1;
			DDPDBG("BWM: division by zero, need skip:%d\n", need_skip);
		}

		DDPDBG("BWM: plane_index:%u fn:%u index:%d lye_id:%d ext_lye_id:%d\n",
			plane_index, fn, index, lye_id, ext_lye_id);
		DDPDBG("BWM: win_size:%d compress:%d bpp:%d src_w:%d src_h:%d\n",
			ovl_win_size, is_compress, bpp, src_w, src_h);
		DDPDBG("BWM: avg_inter_value:%u peak_inter_value:%u\n",
			avg_inter_value, peak_inter_value);
		if (!comp) {
			DDPINFO("comp is NULL");
			return;
		}
		if ((ext_lye_id) &&
			(plane_state->pending.enable) && (need_skip != 1)) {
			cmdq_pkt_read(state->cmdq_handle, NULL,
				comp->regs_pa + DISP_REG_OVL_ELX_BURST_ACC(ext_lye_id-1),
				CMDQ_THR_SPR_IDX1);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = avg_inter_value;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_MULTIPLY, CMDQ_THR_SPR_IDX1, &lop, &rop);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = narrow;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_RIGHT_SHIFT, CMDQ_THR_SPR_IDX1, &lop, &rop);

			cmdq_pkt_write_indriect(state->cmdq_handle, comp->cmdq_base,
				avg_slot, CMDQ_THR_SPR_IDX1, ~0);

			cmdq_pkt_read(state->cmdq_handle, NULL,
				comp->regs_pa + DISP_REG_OVL_ELX_BURST_ACC_WIN_MAX(ext_lye_id-1),
				CMDQ_THR_SPR_IDX1);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = peak_inter_value;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_MULTIPLY, CMDQ_THR_SPR_IDX1, &lop, &rop);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = narrow;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_RIGHT_SHIFT, CMDQ_THR_SPR_IDX1, &lop, &rop);

			cmdq_pkt_write_indriect(state->cmdq_handle, comp->cmdq_base,
				peak_slot, CMDQ_THR_SPR_IDX1, ~0);

		} else if (plane_state->pending.enable && (need_skip != 1)) {
			cmdq_pkt_read(state->cmdq_handle, NULL,
				comp->regs_pa + DISP_REG_OVL_LX_BURST_ACC(lye_id),
				CMDQ_THR_SPR_IDX1);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = avg_inter_value;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_MULTIPLY, CMDQ_THR_SPR_IDX1, &lop, &rop);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = narrow;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_RIGHT_SHIFT, CMDQ_THR_SPR_IDX1, &lop, &rop);

			cmdq_pkt_write_indriect(state->cmdq_handle, comp->cmdq_base,
				avg_slot, CMDQ_THR_SPR_IDX1, ~0);

			cmdq_pkt_read(state->cmdq_handle, NULL,
				comp->regs_pa + DISP_REG_OVL_LX_BURST_ACC_WIN_MAX(lye_id),
				CMDQ_THR_SPR_IDX1);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = peak_inter_value;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_MULTIPLY, CMDQ_THR_SPR_IDX1, &lop, &rop);

			lop.reg = true;
			lop.idx = CMDQ_THR_SPR_IDX1;
			rop.reg = false;
			rop.value = narrow;
			cmdq_pkt_logic_command(state->cmdq_handle,
				CMDQ_LOGIC_RIGHT_SHIFT, CMDQ_THR_SPR_IDX1, &lop, &rop);

			cmdq_pkt_write_indriect(state->cmdq_handle, comp->cmdq_base,
				peak_slot, CMDQ_THR_SPR_IDX1, ~0);
		}
	}
}

static void mtk_drm_ovl_bw_monitor_ratio_save(unsigned int frame_idx)
{
	unsigned int i = 0;

	for (i = 0; i < MAX_LAYER_RATIO_NUMBER; i++) {
		if (display_compress_ratio_table[i].key_value)
			display_compress_ratio_table[i].valid = 1;
	}

	if (display_fbt_compress_ratio_table.key_value)
		display_fbt_compress_ratio_table.valid = 1;

	if (g_ovl_bwm_debug) {
		/* Clear fn frame record for recording next frame */
		for (i = 0; i < MAX_LAYER_RATIO_NUMBER; i++) {
			unsigned int index = fn*MAX_LAYER_RATIO_NUMBER + i;

			if (index >= MAX_FRAME_RATIO_NUMBER*MAX_LAYER_RATIO_NUMBER) {
				DDPPR_ERR("%s errors due to index %d\n", __func__, index);
				return;
			}
			memset(&normal_layer_compress_ratio_tb[index], 0,
				sizeof(struct layer_compress_ratio_item));
		}
		memset(&fbt_layer_compress_ratio_tb[fn], 0,
			sizeof(struct layer_compress_ratio_item));
	}

	/* Copy one frame ratio to table */
	for (i = 0; i < MAX_LAYER_RATIO_NUMBER; i++) {
		unsigned int index = fn * MAX_LAYER_RATIO_NUMBER + i;

		DDPDBG("BWM: fn:%u frame_idx:%u index:%d\n", fn, frame_idx, index);

		if (index >= MAX_FRAME_RATIO_NUMBER*MAX_LAYER_RATIO_NUMBER) {
			DDPPR_ERR("%s errors due to index %d\n", __func__, index);
			return;
		}

		if ((display_compress_ratio_table[i].key_value) &&
			(display_compress_ratio_table[i].average_ratio != NULL) &&
			(display_compress_ratio_table[i].peak_ratio != NULL)) {
			DDPDBG("BWM: fn:%u frame_idx:%u index:%d i:%d\n", fn, frame_idx, index, i);

			normal_layer_compress_ratio_tb[index].frame_idx =
				display_compress_ratio_table[i].frame_idx;
			normal_layer_compress_ratio_tb[index].key_value =
				display_compress_ratio_table[i].key_value;
			normal_layer_compress_ratio_tb[index].average_ratio =
				*(display_compress_ratio_table[i].average_ratio);
			normal_layer_compress_ratio_tb[index].peak_ratio =
				*(display_compress_ratio_table[i].peak_ratio);
			normal_layer_compress_ratio_tb[index].valid =
				display_compress_ratio_table[i].valid;
			normal_layer_compress_ratio_tb[index].active =
				display_compress_ratio_table[i].active;

			unchanged_compress_ratio_table[i].frame_idx =
				display_compress_ratio_table[i].frame_idx;
			unchanged_compress_ratio_table[i].key_value =
				normal_layer_compress_ratio_tb[index].key_value -
				normal_layer_compress_ratio_tb[index].frame_idx;
			unchanged_compress_ratio_table[i].average_ratio =
				*(display_compress_ratio_table[i].average_ratio);
			unchanged_compress_ratio_table[i].peak_ratio =
				*(display_compress_ratio_table[i].peak_ratio);
			unchanged_compress_ratio_table[i].valid =
				display_compress_ratio_table[i].valid;
			unchanged_compress_ratio_table[i].active =
				display_compress_ratio_table[i].active;
		}
	}

	if ((display_fbt_compress_ratio_table.key_value) &&
		(display_fbt_compress_ratio_table.average_ratio != NULL) &&
		(display_fbt_compress_ratio_table.peak_ratio != NULL)) {

		fbt_layer_compress_ratio_tb[fn].frame_idx =
			display_fbt_compress_ratio_table.frame_idx;
		fbt_layer_compress_ratio_tb[fn].key_value =
			display_fbt_compress_ratio_table.key_value;
		fbt_layer_compress_ratio_tb[fn].average_ratio =
			*(display_fbt_compress_ratio_table.average_ratio);
		fbt_layer_compress_ratio_tb[fn].peak_ratio =
			*(display_fbt_compress_ratio_table.peak_ratio);
		fbt_layer_compress_ratio_tb[fn].valid =
			display_fbt_compress_ratio_table.valid;
		fbt_layer_compress_ratio_tb[fn].active =
			display_fbt_compress_ratio_table.active;
	}

	DDPDBG("BWM: fn:%u frame_idx:%u\n", fn, frame_idx);

	DDPDBG("BWMT===== normal_layer_compress_ratio_tb =====\n");
	DDPDBG("BWMT===== Item     Frame    Key     avg    peak     valid    active=====\n");
	for (i = 0; i < 60; i++) {
		if (normal_layer_compress_ratio_tb[i].key_value)
			DDPDBG("BWMT===== %4d     %u     %lu     %u    %u     %u    %u =====\n", i,
				normal_layer_compress_ratio_tb[i].frame_idx,
				normal_layer_compress_ratio_tb[i].key_value,
				normal_layer_compress_ratio_tb[i].average_ratio,
				normal_layer_compress_ratio_tb[i].peak_ratio,
				normal_layer_compress_ratio_tb[i].valid,
				normal_layer_compress_ratio_tb[i].active);
	}

	DDPDBG("BWMT===== fbt_layer_compress_ratio_tb =====\n");
	DDPDBG("BWMT===== Item     Frame    Key     avg    peak     valid    active=====\n");
	for (i = 0; i < 5; i++) {
		if (fbt_layer_compress_ratio_tb[i].key_value)
			DDPDBG("BWMT===== %4d     %u     %lu     %u    %u     %u    %u =====\n", i,
				fbt_layer_compress_ratio_tb[i].frame_idx,
				fbt_layer_compress_ratio_tb[i].key_value,
				fbt_layer_compress_ratio_tb[i].average_ratio,
				fbt_layer_compress_ratio_tb[i].peak_ratio,
				fbt_layer_compress_ratio_tb[i].valid,
				fbt_layer_compress_ratio_tb[i].active);
	}
	DDPDBG("BWMT===== unchanged_compress_ratio_table =====\n");
	DDPDBG("BWMT===== Item     Frame    Key     avg    peak     valid    active=====\n");
	for (i = 0; i < MAX_LAYER_RATIO_NUMBER; i++) {
		if (unchanged_compress_ratio_table[i].key_value)
			DDPDBG("BWMT===== %4d     %u     %lu     %u    %u     %u    %u =====\n", i,
				unchanged_compress_ratio_table[i].frame_idx,
				unchanged_compress_ratio_table[i].key_value,
				unchanged_compress_ratio_table[i].average_ratio,
				unchanged_compress_ratio_table[i].peak_ratio,
				unchanged_compress_ratio_table[i].valid,
				unchanged_compress_ratio_table[i].active);
	}


	fn++;
	/* Ring buffer config */
	if (fn >= MAX_FRAME_RATIO_NUMBER)
		fn = 0;
}


#endif

int mtk_crtc_fill_fb_para(struct mtk_drm_crtc *mtk_crtc)
{
	unsigned int vramsize = 0, fps = 0;
	phys_addr_t fb_base = 0;
	struct mtk_drm_gem_obj *mtk_gem;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct mtk_ddp_fb_info *fb_info = &priv->fb_info;

	if (_parse_tag_videolfb(&vramsize, &fb_base, &fps) < 0) {
		DDPPR_ERR("Can't access buffer info from dts\n");
	} else {
		fb_info->fb_pa = fb_base;
		fb_info->width = ALIGN_TO_32(mtk_crtc->base.mode.hdisplay);
		fb_info->height = ALIGN_TO_32(mtk_crtc->base.mode.vdisplay) * 3;
		fb_info->pitch = fb_info->width * 4;
		fb_info->size = fb_info->pitch * fb_info->height;

		mtk_gem = mtk_drm_fb_gem_insert(mtk_crtc->base.dev,
					fb_info->size, fb_base, vramsize);

		if (IS_ERR(mtk_gem))
			return PTR_ERR(mtk_gem);
		kmemleak_ignore(mtk_gem);
		fb_info->fb_gem = mtk_gem;
	}

	return 0;
}

static void mtk_crtc_enable_iommu(struct mtk_drm_crtc *mtk_crtc,
			   struct cmdq_pkt *handle)
{
	int i, j, p_mode;
	struct mtk_ddp_comp *comp;

	for_each_comp_in_all_crtc_mode(comp, mtk_crtc, i, j, p_mode)
		mtk_ddp_comp_iommu_enable(comp, handle);
	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
			mtk_ddp_comp_iommu_enable(comp, handle);
	}
}

#ifndef DRM_CMDQ_DISABLE
static void mtk_crtc_exec_atf_prebuilt_instr(struct mtk_drm_crtc *mtk_crtc,
			   struct cmdq_pkt *handle)
{
	/*note: put the prebuilt instr into cmdq_inst_disp_va[] in cmdq-prebuilt.h*/

	/*set DISP_VA_START event to atf*/
	cmdq_pkt_set_event(handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_DISP_VA_START]);

	/*wait DISP_VA_END event from atf*/
	cmdq_pkt_wfe(handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_DISP_VA_END]);

	//SMC Call
	cmdq_util_enable_disp_va();
}
#endif

void mtk_crtc_enable_iommu_runtime(struct mtk_drm_crtc *mtk_crtc,
			   struct cmdq_pkt *handle)
{
	int i, j;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	static bool is_inited;

	if (drm_crtc_index(&mtk_crtc->base) == 0)
		mtk_crtc_fill_fb_para(mtk_crtc);

#ifndef DRM_CMDQ_DISABLE
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_USE_M4U)) {
		if (!is_inited && (priv->data->mmsys_id == MMSYS_MT6983 ||
			priv->data->mmsys_id == MMSYS_MT6985 ||
			priv->data->mmsys_id == MMSYS_MT6879 ||
			priv->data->mmsys_id == MMSYS_MT6895 ||
			priv->data->mmsys_id == MMSYS_MT6835 ||
			priv->data->mmsys_id == MMSYS_MT6855 ||
			priv->data->mmsys_id == MMSYS_MT6886)) {
			/*set smi_larb_sec_con reg as 1*/
			mtk_crtc_exec_atf_prebuilt_instr(mtk_crtc, handle);
			is_inited = true;
		}
	}
#endif

	mtk_crtc_enable_iommu(mtk_crtc, handle);


	if (priv->fb_info.fb_gem) {
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_ddp_comp_io_cmd(comp, handle, OVL_REPLACE_BOOTUP_MVA,
					    &priv->fb_info);
		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
				mtk_ddp_comp_io_cmd(comp, handle,
					OVL_REPLACE_BOOTUP_MVA, &priv->fb_info);
		}
	}
}

#ifndef DRM_CMDQ_DISABLE
static ktime_t mtk_check_preset_fence_timestamp(struct drm_crtc *crtc)
{
	int id = drm_crtc_index(crtc);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int vrefresh = 0;
	bool is_frame_mode;
	ktime_t cur_time, diff_time;
	ktime_t start_time, wait_time;
	bool pass = false;
	unsigned long flags;

	is_frame_mode = mtk_crtc_is_frame_trigger_mode(crtc);
	cur_time = mtk_crtc->pf_time;

	if (id == 0) {
		vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
		if (vrefresh == 0) {
			DDPINFO("%s: invalid fps:%u\n", __func__, vrefresh);
			vrefresh = 60;
		}

		start_time = mtk_crtc->sof_time;
		do {
			wait_event_interruptible_timeout(
				mtk_crtc->signal_irq_for_pre_fence_wq
				, atomic_read(&mtk_crtc->signal_irq_for_pre_fence)
				, msecs_to_jiffies(1000 / vrefresh));

			spin_lock_irqsave(&mtk_crtc->pf_time_lock, flags);
			cur_time = mtk_crtc->pf_time;

			if (start_time > cur_time) {
				diff_time = (start_time - cur_time) / 1000000;
				if (diff_time < (1000 / vrefresh / 2))
					pass = true;
			} else
				pass = true;

			if (!pass)
				atomic_set(&mtk_crtc->signal_irq_for_pre_fence, 0);
			spin_unlock_irqrestore(&mtk_crtc->pf_time_lock, flags);

		} while (!pass && is_frame_mode);

		wait_time = ktime_get();

		if (((wait_time - start_time) / 1000000) > (1000 / vrefresh / 2)) {
			DDPINFO("Wait time over %d ms, start %lld, wait %lld, cur %lld\n",
				(1000 / vrefresh / 2), start_time, wait_time, cur_time);
			CRTC_MMP_MARK(id, present_fence_timestamp, start_time, wait_time);
		}
	}

	return cur_time;
}
#ifdef MTK_DRM_CMDQ_ASYNC
#ifdef MTK_DRM_ASYNC_HANDLE
static void mtk_disp_signal_fence_worker_signal(struct drm_crtc *crtc, struct cmdq_cb_data data)
{
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct cb_data_store *cb_data_s;
	struct mtk_cmdq_cb_data *cb_data = data.data;
	int ret;

	if (IS_ERR_OR_NULL(crtc))
		return;

	mtk_crtc = to_mtk_crtc(crtc);
	if (unlikely(!mtk_crtc)) {
		DDPINFO("%s:invalid ESD context, crtc id:%d\n",
			__func__, drm_crtc_index(crtc));
		return;
	}

	cb_data_s = cb_data->store_cb_data;
	if (!cb_data_s)
		DDPMSG("create cb_data_s fail\n");
	else {
		cb_data->signal_ts = ktime_get();
		cb_data_s->data = data;
		INIT_LIST_HEAD(&cb_data_s->link);
		ret = mtk_drm_add_cb_data(cb_data_s, drm_crtc_index(crtc));
		if (ret != 0)
			kfree(cb_data_s);
	}
	atomic_set(&mtk_crtc->cmdq_done, 1);
	wake_up_interruptible(&mtk_crtc->signal_fence_task_wq);
}

static void ddp_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	struct drm_crtc_state *crtc_state = cb_data->state;
	struct drm_crtc *crtc = crtc_state->crtc;

	mtk_disp_signal_fence_worker_signal(crtc, data);
}

static void _ddp_cmdq_cb(struct cmdq_cb_data data)
#else
static void ddp_cmdq_cb(struct cmdq_cb_data data)
#endif
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	struct drm_crtc_state *crtc_state = cb_data->state;
	struct drm_atomic_state *atomic_state = crtc_state->state;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;
	struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(atomic_state, crtc);
	struct mtk_crtc_state *old_mtk_state =
		to_mtk_crtc_state(old_crtc_state);
	unsigned int frame_idx = old_mtk_state->prop_val[CRTC_PROP_OVL_DSI_SEQ];

	int session_id, id;
	unsigned int ovl_status = 0;
	/*Msync2.0 related*/
	unsigned int is_vfp_period = 0;
	unsigned int _dsi_state_dbg7 = 0;
	unsigned int _dsi_state_dbg7_2 = 0;
	ktime_t pf_time;

	if (unlikely(!mtk_crtc)) {
		DDPPR_ERR("%s:%d invalid pointer mtk_crtc\n",
			__func__, __LINE__);
		return;
	}

	if (mtk_crtc->base.dev && mtk_crtc->base.dev->dev_private)
		priv = mtk_crtc->base.dev->dev_private;
	else if (!priv) {
		DDPPR_ERR("%s:%d errors with NULL mtk_crtc->base.dev->dev_private\n",
			__func__, __LINE__);
		return;
	}

	DDPINFO("crtc_state:0x%llx, atomic_state:%x, crtc:%x, pf:%u\n",
		(u64)crtc_state, atomic_state, crtc, cb_data->pres_fence_idx);

	session_id = mtk_get_session_id(crtc);

	id = drm_crtc_index(crtc);

	CRTC_MMP_EVENT_START(id, frame_cfg, (unsigned long)cb_data->cmdq_handle, 0);

	if ((drm_crtc_index(crtc) != 2) && (priv && priv->power_state)) {
		// only VDO mode panel use CMDQ call
		if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base) &&
				!cb_data->msync2_enable) {
			enum PF_TS_TYPE pf_ts_type;

			pf_time = 0;
			pf_ts_type = mtk_crtc->pf_ts_type;
			if (mtk_crtc->pf_time && (pf_ts_type == IRQ_RDMA_EOF ||
				pf_ts_type == IRQ_DSI_EOF))
				pf_time = mtk_check_preset_fence_timestamp(crtc);
			else if (cb_data->signal_ts && pf_ts_type == IRQ_CMDQ_CB)
				pf_time = cb_data->signal_ts;
			else
				DDPINFO("current pf_time is NULL\n");

			mtk_release_present_fence(session_id, cb_data->pres_fence_idx,
				pf_time);
		}
	}

	if (cb_data->is_mml) {
		atomic_set(&(mtk_crtc->wait_mml_last_job_is_flushed), 1);
		wake_up_interruptible(&(mtk_crtc->signal_mml_last_job_is_flushed_wq));
	}

	DDP_MUTEX_LOCK(&priv->commit.lock, __func__, cb_data->pres_fence_idx);
	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	if ((id == 0) && (priv && priv->power_state)) {
		ovl_status = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
				DISP_SLOT_OVL_STATUS);

		/* BW monitor: Set valid to 1 */
		if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_OVL_BW_MONITOR))
			mtk_drm_ovl_bw_monitor_ratio_save(frame_idx);

		if (ovl_status & 1) {
			DDPPR_ERR("ovl status error:0x%x\n", ovl_status);
			if (priv->data->mmsys_id == MMSYS_MT6985) {
				DDPAEE("ovl status error. TS: 0x%08x\n", ovl_status);
				mtk_drm_crtc_mini_analysis(crtc);
				mtk_drm_crtc_mini_dump(crtc);
				cmdq_dump_pkt(cb_data->cmdq_handle, 0, true);
			} else {
				cmdq_util_hw_trace_dump(0, 0);
				mtk_drm_crtc_analysis(crtc);
				mtk_drm_crtc_dump(crtc);
				cmdq_dump_pkt(cb_data->cmdq_handle, 0, true);
			}
		}
		/*Msync 2.0 related function*/
		if (ovl_status & 1)
			CRTC_MMP_MARK(id, ovl_status_err, ovl_status, 0);

		if (cb_data->msync2_enable) {
			_dsi_state_dbg7 = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
						DISP_SLOT_DSI_STATE_DBG7);
			DDPDBG("[Msync]_dsi_state_dbg7=0x%x\n", _dsi_state_dbg7);
			CRTC_MMP_MARK(id, dsi_state_dbg7, _dsi_state_dbg7, 0);

			is_vfp_period = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
						DISP_SLOT_VFP_PERIOD);
			DDPDBG("[Msync]is_vfp_period=%d\n", is_vfp_period);

			if (is_vfp_period == 0) {
				CRTC_MMP_MARK(id, not_vfp_period, 1, 0);
				DDPMSG("[Msync]not vfp period\n");
			} else
				CRTC_MMP_MARK(id, vfp_period, 1, 0);

			/*Msync ToDo: for debug*/
			_dsi_state_dbg7_2 = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
						DISP_SLOT_DSI_STATE_DBG7_2);
			DDPDBG("[Msync]_dsi_state_dbg7 after sof=0x%x\n", _dsi_state_dbg7_2);
			CRTC_MMP_MARK(id, dsi_dbg7_after_sof, _dsi_state_dbg7_2, 0);
		}


		if (atomic_read(&priv->need_recover))
			atomic_set(&priv->need_recover, 0);
	}
	CRTC_MMP_MARK(id, frame_cfg, ovl_status, 0);

	if (old_mtk_state->pending_usage_update) {
		unsigned int i;

		DDPDBG("%s usage_list %u\n", __func__, old_mtk_state->pending_usage_list);
		for (i = 0 ; i < MAX_CRTC ; ++i) {
			if (priv && priv->usage[i] == DISP_OPENING &&
					((old_mtk_state->pending_usage_list >> i) & 0x1))
				priv->usage[i] = DISP_ENABLE;
		}
	}
	mtk_crtc_release_input_layer_fence(crtc, session_id);

	// release present fence
	if ((drm_crtc_index(crtc) != 2) && (priv && priv->power_state)) {
		unsigned int fence_idx = readl(mtk_get_gce_backup_slot_va(mtk_crtc,
				DISP_SLOT_PRESENT_FENCE(drm_crtc_index(crtc))));

		if (fence_idx != cb_data->pres_fence_idx) {
			DDPPR_ERR("%s:fence_idx:%d, cb_data->pres_fence_idx:%d\n",
				__func__, fence_idx, cb_data->pres_fence_idx);
			mtk_release_present_fence(session_id, cb_data->pres_fence_idx, 0);
		}
		// only VDO mode panel use CMDQ call
		if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
			if (cb_data->msync2_enable)
				mtk_release_present_fence(session_id,
						fence_idx, ktime_get());
		}
	}

	/* for wfd latency debug */
	if ((id == 0 || id == 2) && (priv && priv->power_state)) {
		unsigned int ovl_dsi_seq = 0;
		unsigned int slot = (id == 0) ? DISP_SLOT_OVL_DSI_SEQ :
							DISP_SLOT_OVL_WDMA_SEQ;

		ovl_dsi_seq = readl(mtk_get_gce_backup_slot_va(mtk_crtc, slot));

		if (ovl_dsi_seq) {
			if (id == 0)
				mtk_drm_trace_async_end("OVL0-DSI|%d", ovl_dsi_seq);
			else if (id == 2)
				mtk_drm_trace_async_end("OVL2-WDMA|%d", ovl_dsi_seq);
		}
	}

	if (!mtk_crtc_is_dc_mode(crtc) && id != 0)
		mtk_crtc_release_output_buffer_fence(crtc, session_id);

	if (priv && priv->power_state)
		mtk_crtc_update_hrt_qos(crtc, cb_data->misc);
	else
		DDPINFO("crtc%d is disabled so skip update hrt qos\n", id);

	if (mtk_crtc->pending_needs_vblank) {
		mtk_drm_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}

	drm_atomic_state_put(atomic_state);

	if (mtk_crtc->wb_enable == true) {
		mtk_crtc->wb_enable = false;
		drm_writeback_signal_completion(&mtk_crtc->wb_connector, 0);
	}

	{
		struct mtk_drm_sram_list *entry, *tmp;

		mutex_lock(&mtk_crtc->mml_ir_sram.lock);
		list_for_each_entry_safe(entry, tmp, &mtk_crtc->mml_ir_sram.list.head, head) {
			if (cb_data->hrt_idx > entry->hrt_idx) {
				list_del_init(&entry->head);
				kfree(entry);
				kref_put(&mtk_crtc->mml_ir_sram.ref, mtk_crtc_mml_clean);
			}
		}
		mutex_unlock(&mtk_crtc->mml_ir_sram.lock);
	}

	{	/* OVL reset debug */
		unsigned int i;
		unsigned int ovl_comp_id;

		for (i = 0; i < OVL_RT_LOG_NR; i++) {
			ovl_comp_id = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
							DISP_SLOT_OVL_COMP_ID(i));

			if (ovl_comp_id) {
				DDPPR_ERR("crtc%d(%d)%s reset, 0x%x\n", id, frame_idx,
					mtk_dump_comp_str_id(ovl_comp_id),
					*(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_OVL_GREQ_CNT(i)));
				writel(0, mtk_get_gce_backup_slot_va(mtk_crtc,
					DISP_SLOT_OVL_COMP_ID(i)));
			}
		}
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
	DDP_MUTEX_UNLOCK(&priv->commit.lock, __func__, cb_data->pres_fence_idx);
#ifdef MTK_DRM_ASYNC_HANDLE
	cmdq_pkt_wait_complete(cb_data->cmdq_handle);
	mtk_drm_del_cb_data(data, id);
#endif
	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);

	CRTC_MMP_EVENT_END(id, frame_cfg, 0, 0);
}

#ifdef MTK_DRM_ASYNC_HANDLE
static int mtk_drm_signal_fence_worker_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	int ret = 0;
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *)data;
	struct cb_data_store *store_data;
	int crtc_id = drm_crtc_index(&mtk_crtc->base);

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(
			mtk_crtc->signal_fence_task_wq
			, atomic_read(&mtk_crtc->cmdq_done));
			atomic_set(&mtk_crtc->cmdq_done, 0);
		while (mtk_drm_get_cb_data(crtc_id)) {
			store_data = mtk_drm_get_cb_data(crtc_id);
			_ddp_cmdq_cb(store_data->data);
		}
	}
	return 0;
}
#endif
#else
/* ddp_cmdq_cb_blocking should be called within locked function */
static void ddp_cmdq_cb_blocking(struct mtk_cmdq_cb_data *cb_data)
{
	struct drm_crtc_state *crtc_state = cb_data->state;
	struct drm_atomic_state *atomic_state = crtc_state->state;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *private;
	int session_id = -1, id, i;

	DDPINFO("%s:%d, cb_data:%x\n",
		__func__, __LINE__,
		cb_data);
	DDPINFO("crtc_state:%x, atomic_state:%x, crtc:%x\n",
		crtc_state,
		atomic_state,
		crtc);

	id = drm_crtc_index(crtc);
	private = mtk_crtc->base.dev->dev_private;
	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if ((id + 1) == MTK_SESSION_TYPE(private->session_id[i])) {
			session_id = private->session_id[i];
			break;
		}
	}

	mtk_crtc_release_input_layer_fence(crtc, session_id);

	mtk_crtc_release_output_buffer_fence(crtc, session_id);

	mtk_crtc_update_hrt_qos(crtc, cb_data->misc);

	if (mtk_crtc->pending_needs_vblank) {
		mtk_drm_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}

	drm_atomic_state_put(atomic_state);

	if (mtk_crtc->wb_enable == true) {
		mtk_crtc->wb_enable = false;
		drm_writeback_signal_completion(&mtk_crtc->wb_connector, 0);
	}

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}
#endif
#endif

static void mtk_crtc_ddp_config(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(mtk_crtc->base.state);
	struct mtk_ddp_comp *comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, 0, 0);
	struct mtk_ddp_config cfg;
	struct cmdq_pkt *cmdq_handle = state->cmdq_handle;
	unsigned int i;
	unsigned int ovl_is_busy;

#ifndef DRM_CMDQ_DISABLE
	unsigned int last_fence, cur_fence, sub;
#endif

	/*
	 * TODO: instead of updating the registers here, we should prepare
	 * working registers in atomic_commit and let the hardware command
	 * queue update module registers on vblank.
	 */
	if (!comp) {
		DDPINFO("comp is NULL in ddp config\n");
		return;
	}
	ovl_is_busy = readl(comp->regs) & 0x1UL;
	if (ovl_is_busy == 0x1UL)
		return;

	if ((state->pending_config) == true) {
		cfg.w = state->pending_width;
		cfg.h = state->pending_height;
		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				cfg.vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				cfg.vrefresh = state->pending_vrefresh;
		} else
			cfg.vrefresh = state->pending_vrefresh;
		cfg.bpc = 0;
		mtk_ddp_comp_config(comp, &cfg, cmdq_handle);

		state->pending_config = false;
	}

	if ((mtk_crtc->pending_planes) == false)
		return;

#ifdef DRM_CMDQ_DISABLE
	mtk_wb_atomic_commit(mtk_crtc);
#endif
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		if ((plane_state->pending.config) == false)
			continue;

		mtk_ddp_comp_layer_config(comp, i, plane_state, cmdq_handle);

#ifndef DRM_CMDQ_DISABLE
		last_fence = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
			       DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, i)));
		cur_fence =
			(unsigned int)plane_state->pending.prop_val[PLANE_PROP_NEXT_BUFF_IDX];

		if (cur_fence != -1 && cur_fence > last_fence)
			cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			       mtk_get_gce_backup_slot_pa(mtk_crtc,
			       DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, i))),
			       cur_fence, ~0);

		sub = 1;
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			       mtk_get_gce_backup_slot_pa(mtk_crtc,
			       DISP_SLOT_SUBTRACTOR_WHEN_FREE(mtk_get_plane_slot_idx(mtk_crtc, i))),
			       sub, ~0);
#endif
		plane_state->pending.config = false;
	}

	mtk_crtc->pending_planes = false;
	if (mtk_crtc->wb_enable == true) {
		mtk_crtc->wb_enable = false;
		drm_writeback_signal_completion(&mtk_crtc->wb_connector, 0);
	}
}

static void mtk_crtc_comp_trigger(struct mtk_drm_crtc *mtk_crtc,
				  struct cmdq_pkt *cmdq_handle,
				  enum mtk_ddp_comp_trigger_flag trig_flag)
{
	int i, j;
	struct mtk_ddp_comp *comp;

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_config_trigger(comp, cmdq_handle, trig_flag);

	/* aware there might be redudant operation if same comp in dual pipe path */
	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
			mtk_ddp_comp_config_trigger(comp, cmdq_handle, trig_flag);
	}
}

int mtk_crtc_comp_is_busy(struct mtk_drm_crtc *mtk_crtc)
{
	int ret = 0;
	int i, j;
	struct mtk_ddp_comp *comp;

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		ret = mtk_ddp_comp_is_busy(comp);
		if (ret)
			return ret;
	}

	return ret;
}

#ifndef DRM_CMDQ_DISABLE
/* TODO: need to remove this in vdo mode for lowpower */
static void trig_done_cb(struct cmdq_cb_data data)
{
	CRTC_MMP_MARK((unsigned long)data.data, trig_loop_done, 0, 0);
}
#endif

void mtk_crtc_clear_wait_event(struct drm_crtc *crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_crtc_pkt_create(&cmdq_handle, crtc,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return;
		}

		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_ESD_EOF]);
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}

}

#ifndef DRM_CMDQ_DISABLE
#ifdef IF_ZERO /* not ready for dummy register method */
static void mtk_crtc_rec_trig_cnt(struct mtk_drm_crtc *mtk_crtc,
				  struct cmdq_pkt *cmdq_handle)
{
	struct cmdq_pkt_buffer *cmdq_buf = &mtk_crtc->gce_obj.buf;
	struct cmdq_operand lop, rop;

	lop.reg = true;
	lop.idx = CMDQ_CPR_DISP_CNT;
	rop.reg = false;
	rop.value = 1;

	cmdq_pkt_logic_command(cmdq_handle, CMDQ_LOGIC_ADD, CMDQ_CPR_DISP_CNT,
			       &lop, &rop);
	cmdq_pkt_write_reg_addr(cmdq_handle,
				cmdq_buf->pa_base + DISP_SLOT_TRIG_CNT,
				CMDQ_CPR_DISP_CNT, U32_MAX);
}
#endif
#endif

/* sw workaround to fix gce hw bug */
void mtk_crtc_start_sodi_loop(struct drm_crtc *crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;
	unsigned long crtc_id = (unsigned long)drm_crtc_index(crtc);

	if (crtc_id) {
		DDPDBG("%s:%d invalid crtc:%ld\n",
			__func__, __LINE__, crtc_id);
		return;
	}

	if (mtk_crtc->sodi_loop_cmdq_handle) {
		DDPDBG("exist sodi loop, skip %s\n", __func__);
		return;
	}

	priv = mtk_crtc->base.dev->dev_private;
	mtk_crtc->sodi_loop_cmdq_handle = cmdq_pkt_create(
			mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
	cmdq_handle = mtk_crtc->sodi_loop_cmdq_handle;

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}


	cmdq_pkt_wait_no_clear(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);

	cmdq_pkt_write(cmdq_handle, NULL,
		GCE_BASE_ADDR + GCE_GCTL_VALUE, GCE_DDR_EN, GCE_DDR_EN);

	cmdq_pkt_wfe(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_SODI]);

	cmdq_pkt_finalize_loop(cmdq_handle);
	cmdq_pkt_flush_async(cmdq_handle, NULL, (void *)crtc_id);
}

void mtk_crtc_start_event_loop(struct drm_crtc *crtc)
{
/****************************************************************************/
/* dsi_check : from TE to dsi_check                                         */
/* v_idle    : from v_idle to pf (prefetch)                                 */
/* mt        : from merge trigger to pf, cannot over vidle if vidle is on   */
/* pf        : from prefetch to next TE                                     */
/*                                                                          */
/*    TE   dsi_check                   v_idle     mt     pf     TE          */
/*  __|______|___________~________________|________|_____|______|_          */
/****************************************************************************/
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;
	unsigned long crtc_id = (unsigned long)drm_crtc_index(crtc);
	unsigned int cur_fps = 0;
	unsigned int dsi_pll_check_off_offset = 500;
	unsigned int v_idle_power_off_offset = 300;
	unsigned int merge_trigger_offset = 150;
	unsigned int prefetch_te_offset = 150;
	unsigned int frame_time = 0;
	int en = 1;

	struct mtk_ddp_comp *output_comp;

	struct cmdq_operand lop, rop;

	const u16 reg_jump = CMDQ_THR_SPR_IDX1;
	const u16 var1 = CMDQ_THR_SPR_IDX2;
	const u16 var2 = CMDQ_THR_SPR_IDX3;
	//const u16 var3 = 0;

	u32 inst_condi_jump;
	u64 *inst, jump_pa;

	priv = mtk_crtc->base.dev->dev_private;

	mtk_crtc->pre_te_cfg.prefetch_te_en = false;
	mtk_crtc->pre_te_cfg.vidle_apsrc_off_en = false;
	mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en = false;
	mtk_crtc->pre_te_cfg.merge_trigger_en = false;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (crtc_id) {
		DDPDBG("%s:%d invalid crtc:%ld\n",
			__func__, __LINE__, crtc_id);
		return;
	}

	if (mtk_crtc->event_loop_cmdq_handle) {
		DDPDBG("exist event loop, skip %s\n", __func__);
		return;
	}

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_PREFETCH_TE) &&
		!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_APSRC_OFF) &&
		!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_DSI_PLL_OFF) &&
		!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_CHECK_TRIGGER_MERGE)
		) {
		DDPINFO("no need to start event loop.\n");
		return;
	}

	if (mtk_crtc->msync2.msync_frame_status == 1) {
		DDPINFO("msync 2.0 case, disable event loop.\n");
		return;
	}

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_PREFETCH_TE))
		prefetch_te_offset = 0;
	else
		mtk_crtc->pre_te_cfg.prefetch_te_en = true;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_CHECK_TRIGGER_MERGE))
		merge_trigger_offset = 0;
	else
		mtk_crtc->pre_te_cfg.merge_trigger_en = true;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_APSRC_OFF) &&
		!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_DSI_PLL_OFF)) {
		v_idle_power_off_offset = 0;
		dsi_pll_check_off_offset = 0;
	} else {
		if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_APSRC_OFF)) {
			mtk_crtc->pre_te_cfg.vidle_apsrc_off_en = true;
			if (mtk_crtc->pre_te_cfg.merge_trigger_en == true) {
				if (v_idle_power_off_offset > merge_trigger_offset)
					v_idle_power_off_offset -= merge_trigger_offset;
				else {
					merge_trigger_offset = v_idle_power_off_offset;
					v_idle_power_off_offset = 0;
				}
			}
		}

		if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_DSI_PLL_OFF))
			mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en = true;
		else
			dsi_pll_check_off_offset = 0;
	}

	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0) {
		cur_fps = mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	} else if (crtc && crtc->state) {
		cur_fps = drm_mode_vrefresh(&crtc->state->mode);
	}

	if (cur_fps)
		frame_time = 1000000 / cur_fps;

	DDPINFO("%s: cur_fps:%d, frame_time:%d\n", __func__, cur_fps, frame_time);

	if (frame_time <=
		(dsi_pll_check_off_offset
		+ prefetch_te_offset
		+ v_idle_power_off_offset
		+ merge_trigger_offset
		)) {
		mtk_crtc->pre_te_cfg.prefetch_te_en = false;
		mtk_crtc->pre_te_cfg.vidle_apsrc_off_en = false;
		mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en = false;
		mtk_crtc->pre_te_cfg.merge_trigger_en = false;
		DDPINFO("not start event loop, not support timing.\n");
		return;
	}

	priv = mtk_crtc->base.dev->dev_private;
	mtk_crtc->event_loop_cmdq_handle = cmdq_pkt_create(
			mtk_crtc->gce_obj.client[CLIENT_EVENT_LOOP]);
	cmdq_handle = mtk_crtc->event_loop_cmdq_handle;

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_clear_event(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);

	cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_TE]);
	if (mtk_drm_lcm_is_connect(mtk_crtc))
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_TE]);

	cmdq_pkt_set_event(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);

	if (mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en) {
		cmdq_pkt_sleep(cmdq_handle,
			CMDQ_US_TO_TICK(dsi_pll_check_off_offset), CMDQ_GPR_R07);

		lop.reg = true;
		lop.idx = var1;
		rop.reg = false;
		rop.value = 31;

		//check dsi is busy or not
		cmdq_pkt_read(cmdq_handle, NULL,
			output_comp->regs_pa + 0x0C/*DSI_INTSTA*/, var1);

		cmdq_pkt_logic_command(cmdq_handle, CMDQ_LOGIC_RIGHT_SHIFT, var1, &lop, &rop);

		rop.reg = true;
		rop.idx = var2;

		//check pll switch function is enable or not
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
						DSI_PLL_SWITCH_REFERENCE_CNT_GET, &(rop.idx));

		cmdq_pkt_logic_command(cmdq_handle, CMDQ_LOGIC_OR, var1, &lop, &rop);

		rop.reg = false;
		rop.value = 0;

		/*mark condition jump */
		inst_condi_jump = cmdq_handle->cmd_buf_size;
		cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);

		cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump, &lop, &rop,
			CMDQ_NOT_EQUAL);

		/* if condition false, will jump here */
		en = 0;
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
						DSI_PLL_SWITCH_ON_OFF, &en);

		/* if condition true, will jump curreent postzion */
		inst = cmdq_pkt_get_va_by_offset(cmdq_handle,  inst_condi_jump);
		jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
					cmdq_handle->cmd_buf_size);
		*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);
	}

	/* set VIDLE_POWER_OFF event after TE event T_Vtotal - 150us - 300us */
	cmdq_pkt_sleep(cmdq_handle, CMDQ_US_TO_TICK(
					(frame_time
					- dsi_pll_check_off_offset
					- v_idle_power_off_offset
					- merge_trigger_offset
					- prefetch_te_offset)),
					CMDQ_GPR_R07);

	/* set pre-TE event after TE event T_Vtotal - 150us */
	cmdq_pkt_clear_event(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);

	if (mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en) {
		en = 1;
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
							DSI_PLL_SWITCH_ON_OFF, &en);
	}

	if (mtk_crtc->pre_te_cfg.vidle_apsrc_off_en ||
		mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VIDLE_POWER_ON]);

		if (mtk_crtc->pre_te_cfg.merge_trigger_en ||
			mtk_crtc->pre_te_cfg.prefetch_te_en)
			cmdq_pkt_sleep(cmdq_handle,
				CMDQ_US_TO_TICK(v_idle_power_off_offset), CMDQ_GPR_R07);
	}

	if (mtk_crtc->pre_te_cfg.merge_trigger_en) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_CHECK_TRIGGER_MERGE]);

		if (mtk_crtc->pre_te_cfg.prefetch_te_en) {
			cmdq_pkt_sleep(cmdq_handle,
				CMDQ_US_TO_TICK(merge_trigger_offset), CMDQ_GPR_R07);
		}
	}

	if (mtk_crtc->pre_te_cfg.prefetch_te_en) {
		cmdq_pkt_set_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_PREFETCH_TE]);
	}

	cmdq_pkt_finalize_loop(cmdq_handle);

	cmdq_pkt_flush_async(cmdq_handle, NULL, (void *)crtc_id);
}

#ifndef DRM_CMDQ_DISABLE

static void cmdq_pkt_request_te(struct cmdq_pkt *cmdq_handle,
		struct mtk_drm_crtc *mtk_crtc)
{
	const u16 reg_jump = CMDQ_THR_SPR_IDX2;
	const u16 request_en = CMDQ_THR_SPR_IDX3;
	struct cmdq_operand lop;
	struct cmdq_operand rop;
	u32 inst_condi_jump, inst_jump_end;
	u64 *inst, jump_pa;

	unsigned int fps_level = 0;
	struct mtk_ddp_comp *comp = NULL;

	fps_level = 0xEEEE;

	cmdq_pkt_read(cmdq_handle, NULL,
		mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_REQUEST_TE_EN), request_en);
	lop.reg = true;
	lop.idx = request_en;
	rop.reg = false;
	rop.value = ENABLE_REQUSET_TE;

	inst_condi_jump = cmdq_handle->cmd_buf_size;
	cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);
	/* check whether te1_en is enabled*/
	cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump,
		&lop, &rop, CMDQ_EQUAL);

	/* condition not match, here is nop jump */

	inst_jump_end = cmdq_handle->cmd_buf_size;
	cmdq_pkt_jump_addr(cmdq_handle, 0);

	/* following instructinos is condition TRUE,
	 * thus conditional jump should jump current offset
	 */
	if (unlikely(!cmdq_handle->avail_buf_size))
		cmdq_pkt_add_cmd_buffer(cmdq_handle);
	inst = cmdq_pkt_get_va_by_offset(cmdq_handle, inst_condi_jump);

	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
			cmdq_handle->cmd_buf_size);
	*inst = *inst & ((u64)0xFFFFFFFF << 32);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);

	/* condition match, here is nop jump */

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	mtk_ddp_comp_io_cmd(comp, cmdq_handle, DSI_MSYNC_SWITCH_TE_LEVEL_GRP, &fps_level);

	/* this is end of whole condition, thus condition
	 * FALSE part should jump here
	 */
	if (unlikely(!cmdq_handle->avail_buf_size))
		cmdq_pkt_add_cmd_buffer(cmdq_handle);
	inst = cmdq_pkt_get_va_by_offset(cmdq_handle, inst_jump_end);

	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
			cmdq_handle->cmd_buf_size);
	*inst = *inst & ((u64)0xFFFFFFFF << 32);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);
}

static void cmdq_pkt_wait_te(struct cmdq_pkt *cmdq_handle,
		struct mtk_drm_crtc *mtk_crtc)
{
	const u16 reg_jump = CMDQ_THR_SPR_IDX2;
	const u16 te1_en = CMDQ_THR_SPR_IDX3;
	struct cmdq_operand lop;
	struct cmdq_operand rop;
	u32 inst_condi_jump, inst_jump_end;
	u64 *inst, jump_pa;
	bool panel_connected;

	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_panel_params *params_lcm =
		mtk_drm_get_lcm_ext_params(crtc);

	cmdq_pkt_read(cmdq_handle, NULL,
		mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_TE1_EN), te1_en);
	lop.reg = true;
	lop.idx = te1_en;
	rop.reg = false;
	rop.value = 0x1;

	panel_connected = mtk_drm_lcm_is_connect(mtk_crtc);
	inst_condi_jump = cmdq_handle->cmd_buf_size;
	cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);
	/* check whether te1_en is enabled*/
	cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump,
		&lop, &rop, CMDQ_EQUAL);

	/* condition not match, here is nop jump */
	cmdq_pkt_clear_event(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_TE]);

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MSYNC2_0)
			&& params_lcm && params_lcm->msync2_enable)
		cmdq_pkt_request_te(cmdq_handle, mtk_crtc);

	if (panel_connected)
		cmdq_pkt_wfe(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_TE]);

	inst_jump_end = cmdq_handle->cmd_buf_size;
	cmdq_pkt_jump_addr(cmdq_handle, 0);

	/* following instructinos is condition TRUE,
	 * thus conditional jump should jump current offset
	 */
	if (unlikely(!cmdq_handle->avail_buf_size))
		cmdq_pkt_add_cmd_buffer(cmdq_handle);
	inst = cmdq_pkt_get_va_by_offset(cmdq_handle, inst_condi_jump);

	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
			cmdq_handle->cmd_buf_size);
	*inst = *inst & ((u64)0xFFFFFFFF << 32);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);

	/* condition match, here is nop jump */
	cmdq_pkt_clear_event(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_GPIO_TE1]);

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MSYNC2_0)
				&& params_lcm && params_lcm->msync2_enable)
		cmdq_pkt_request_te(cmdq_handle, mtk_crtc);

	if (panel_connected)
		cmdq_pkt_wfe(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_GPIO_TE1]);

	/* this is end of whole condition, thus condition
	 * FALSE part should jump here
	 */
	if (unlikely(!cmdq_handle->avail_buf_size))
		cmdq_pkt_add_cmd_buffer(cmdq_handle);
	inst = cmdq_pkt_get_va_by_offset(cmdq_handle, inst_jump_end);

	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
			cmdq_handle->cmd_buf_size);
	*inst = *inst & ((u64)0xFFFFFFFF << 32);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);
}

static void cmdq_pkt_reset_ovl_by_crtc(struct cmdq_pkt *cmdq_handle, struct mtk_ddp_comp *comp,
	struct mtk_drm_crtc *mtk_crtc, unsigned int index)
{
	const u16 reg_jump = CMDQ_THR_SPR_IDX2;
	const u16 ovl_greq_cnt = CMDQ_THR_SPR_IDX3;
	struct cmdq_operand lop;
	struct cmdq_operand rop;
	u32 inst_condi_jump;
	u64 *inst, jump_pa;

	lop.reg = true;
	lop.idx = ovl_greq_cnt;
	rop.reg = false;
	rop.value = 0;		/* condition */

	cmdq_pkt_read(cmdq_handle, NULL,
		comp->regs_pa + DISP_REG_OVL_GREQ_LAYER_CNT, ovl_greq_cnt);

	inst_condi_jump = cmdq_handle->cmd_buf_size;
	cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);
	/* check OVL GREQ_LAYER_CNT is 0 */
	cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump,
		&lop, &rop, CMDQ_EQUAL);

	/* if condition false, will jump here */
	{
		dma_addr_t addr_oci = mtk_get_gce_backup_slot_pa(mtk_crtc,
			DISP_SLOT_OVL_COMP_ID(index));
		dma_addr_t addr_ogc = mtk_get_gce_backup_slot_pa(mtk_crtc,
			DISP_SLOT_OVL_GREQ_CNT(index));

		mtk_ddp_comp_reset(comp, cmdq_handle);
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base, addr_oci, comp->id, ~0);
		cmdq_pkt_mem_move(cmdq_handle, comp->cmdq_base,
			comp->regs_pa + DISP_REG_OVL_GREQ_LAYER_CNT, addr_ogc, CMDQ_THR_SPR_IDX1);
	}

	/* if condition true, will jump current postzion */
	inst = cmdq_pkt_get_va_by_offset(cmdq_handle,  inst_condi_jump);
	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
				cmdq_handle->cmd_buf_size);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);
}

/* check ovl GREQ_CNT then reset ovl */
static void cmdq_pkt_reset_ovl(struct cmdq_pkt *cmdq_handle,
		struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *comp = NULL;
	int type;
	int i, j;
	unsigned int index = 0;

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		type = mtk_ddp_comp_get_type(comp->id);
		if (type != MTK_DISP_OVL)
			continue;
		cmdq_pkt_reset_ovl_by_crtc(cmdq_handle, comp, mtk_crtc, index);
		if (index < OVL_RT_LOG_NR - 1)
			index++;
	}

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			type = mtk_ddp_comp_get_type(comp->id);
			if (type != MTK_DISP_OVL)
				continue;
			cmdq_pkt_reset_ovl_by_crtc(cmdq_handle, comp, mtk_crtc, index);
			if (index < OVL_RT_LOG_NR - 1)
				index++;
		}
	}
}

#endif

void mtk_crtc_start_trig_loop(struct drm_crtc *crtc)
{
#ifdef DRM_CMDQ_DISABLE
	DDPINFO("%s+\n", __func__);
	return;
#else
	int ret = 0;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned long crtc_id = (unsigned long)drm_crtc_index(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_operand lop, rop;
	struct mtk_panel_params *params = NULL;

	struct mtk_panel_params *params_lcm =
		mtk_drm_get_lcm_ext_params(crtc);
	dma_addr_t slot_src_addr;
	dma_addr_t slot_dts_addr;

	struct mtk_ddp_comp *output_comp;
	int en = 1;
	bool panel_connected = mtk_drm_lcm_is_connect(mtk_crtc);

	const u16 reg_jump = CMDQ_THR_SPR_IDX1;
	const u16 var1 = CMDQ_CPR_DDR_USR_CNT;
	const u16 var2 = 0;

	u32 inst_condi_jump;
	u64 *inst, jump_pa;

	lop.reg = true;
	lop.idx = var1;
	rop.reg = false;
	rop.idx = var2;

	if (mtk_crtc->trig_loop_cmdq_handle) {
		DDPDBG("exist trigger loop, skip %s\n", __func__);
		return;
	}

	if (crtc_id == 2) {
		DDPPR_ERR("%s:%d invalid crtc:%ld\n",
			__func__, __LINE__, crtc_id);
		return;
	}

	mtk_crtc->trig_loop_cmdq_handle = cmdq_pkt_create(
		mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
	cmdq_handle = mtk_crtc->trig_loop_cmdq_handle;

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}


	if (priv->data->mmsys_id == MMSYS_MT6879) {
		//workaround for gce can't wait dsi te event done
		//cmdq_set_outpin_event(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP],
		//		true);
	}

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_PREFETCH_TE) ||
		(mtk_crtc->msync2.msync_frame_status == 1))
		mtk_crtc->pre_te_cfg.prefetch_te_en = false;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_APSRC_OFF) ||
		(mtk_crtc->msync2.msync_frame_status == 1))
		mtk_crtc->pre_te_cfg.vidle_apsrc_off_en = false;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_VIDLE_DSI_PLL_OFF) ||
		(mtk_crtc->msync2.msync_frame_status == 1))
		mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en = false;

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_CHECK_TRIGGER_MERGE) ||
		(mtk_crtc->msync2.msync_frame_status == 1))
		mtk_crtc->pre_te_cfg.merge_trigger_en = false;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		/* The STREAM BLOCK EVENT is used for stopping frame trigger if
		 * the engine is stopped
		 */
		cmdq_pkt_wait_no_clear(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

		if (mtk_crtc->pre_te_cfg.merge_trigger_en == false)
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
			cmdq_pkt_wfe(cmdq_handle,
							mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_wait_no_clear(cmdq_handle,
							mtk_crtc->gce_obj.event[EVENT_ESD_EOF]);
		}

		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);

		if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
			if (mtk_drm_helper_get_opt(priv->helper_opt,
						MTK_DRM_OPT_DUAL_TE)) {
				cmdq_pkt_wait_te(cmdq_handle, mtk_crtc);
				if (mtk_crtc->pre_te_cfg.prefetch_te_en == true) {
					if (priv->data->mmsys_id == MMSYS_MT6985)
						mtk_oddmr_ddren(cmdq_handle, crtc, 1);

					if (mtk_crtc->pre_te_cfg.merge_trigger_en == true)
						cmdq_pkt_clear_event(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

					mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0], cmdq_handle,
					mtk_crtc->gce_obj.base);
				}
			} else {
				if (mtk_crtc->pre_te_cfg.vidle_apsrc_off_en == true ||
					mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en == true
					) {
					cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VIDLE_POWER_ON]);
					cmdq_pkt_wfe(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VIDLE_POWER_ON]);

					if (crtc_id == 0) {
						/*APSRC Power ON*/
						mtk_crtc_v_idle_apsrc_control(crtc, cmdq_handle,
							false, false, crtc_id, true);
					}
				}

				if (mtk_crtc->pre_te_cfg.merge_trigger_en == true) {
					cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[
						EVENT_SYNC_TOKEN_CHECK_TRIGGER_MERGE]);
					cmdq_pkt_wfe(cmdq_handle,
					mtk_crtc->gce_obj.event[
						EVENT_SYNC_TOKEN_CHECK_TRIGGER_MERGE]);

					cmdq_pkt_clear_event(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
				}

				if (mtk_crtc->pre_te_cfg.prefetch_te_en == true) {
					cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_PREFETCH_TE]);
					cmdq_pkt_wfe(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_PREFETCH_TE]);
					if (priv->data->mmsys_id == MMSYS_MT6985)
						mtk_oddmr_ddren(cmdq_handle, crtc, 1);

					if (mtk_crtc->pre_te_cfg.merge_trigger_en == true)
						cmdq_pkt_clear_event(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

					mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0], cmdq_handle,
						mtk_crtc->gce_obj.base);
				}

				if (mtk_crtc->pre_te_cfg.vidle_apsrc_off_en == false &&
					mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en == false &&
					mtk_crtc->pre_te_cfg.prefetch_te_en == false &&
					mtk_crtc->pre_te_cfg.merge_trigger_en == false
					) {
					cmdq_pkt_clear_event(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_TE]);

					if (mtk_drm_helper_get_opt(priv->helper_opt,
							MTK_DRM_OPT_MSYNC2_0)
							&& params_lcm && params_lcm->msync2_enable)
						cmdq_pkt_request_te(cmdq_handle, mtk_crtc);

					if (panel_connected)
						cmdq_pkt_wfe(cmdq_handle,
							mtk_crtc->gce_obj.event[EVENT_TE]);
				}
			}
		}

		/*Trigger*/
		if (mtk_crtc->pre_te_cfg.vidle_apsrc_off_en == true ||
			mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en == true ||
			mtk_crtc->pre_te_cfg.prefetch_te_en == true ||
			mtk_crtc->pre_te_cfg.merge_trigger_en == true
			) {
			cmdq_pkt_wfe(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_TE]);
		}

		if (mtk_crtc->pre_te_cfg.prefetch_te_en == false) {
			if (priv->data->mmsys_id == MMSYS_MT6985)
				mtk_oddmr_ddren(cmdq_handle, crtc, 1);
			mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle,
						MTK_TRIG_FLAG_PRE_TRIGGER);

			if (mtk_crtc->pre_te_cfg.merge_trigger_en == true)
				cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

			mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0], cmdq_handle,
						   mtk_crtc->gce_obj.base);
		}

		mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle,
					  MTK_TRIG_FLAG_TRIGGER);

		cmdq_pkt_wfe(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
		mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle, MTK_TRIG_FLAG_EOF);

#ifdef IF_ZERO /* not ready for dummy register method */
		if (mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_LAYER_REC)) {
			mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle,
					      MTK_TRIG_FLAG_LAYER_REC);

			mtk_crtc_rec_trig_cnt(mtk_crtc, cmdq_handle);

			mtk_crtc->layer_rec_en = true;
		} else {
			mtk_crtc->layer_rec_en = false;
		}
#endif
		if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MSYNC2_0)
				&& params_lcm && params_lcm->msync2_enable) {

			slot_src_addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
				DISP_SLOT_REQUEST_TE_PREPARE);
			slot_dts_addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
				DISP_SLOT_REQUEST_TE_EN);

			cmdq_pkt_mem_move(cmdq_handle, mtk_crtc->gce_obj.base, slot_src_addr,
				slot_dts_addr, CMDQ_THR_SPR_IDX3);
		}

		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);

		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		if (crtc_id == 0) {
			if (mtk_crtc->pre_te_cfg.vidle_apsrc_off_en == true) {
				/*APSRC Power OFF*/
				mtk_crtc_v_idle_apsrc_control(crtc, cmdq_handle,
					false, false, crtc_id, false);
			}
			if (mtk_crtc->pre_te_cfg.vidle_dsi_pll_off_en == true) {
				if (output_comp) {
					lop.reg = true;
					lop.idx = var1;

					//check pll switch function is enable or not
					mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
						DSI_PLL_SWITCH_REFERENCE_CNT_GET, &(lop.idx));

					rop.reg = false;
					rop.value = 0;

					/*mark condition jump */
					inst_condi_jump = cmdq_handle->cmd_buf_size;
					cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);

					cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump, &lop, &rop,
						CMDQ_NOT_EQUAL);

					/* if condition false, will jump here */
					en = 0;
					mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
								DSI_PLL_SWITCH_ON_OFF, &en);

					/* if condition true, will jump curreent postzion */
					inst = cmdq_pkt_get_va_by_offset(cmdq_handle,
								inst_condi_jump);
					jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
								cmdq_handle->cmd_buf_size);
					*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);

				}
			}
		}
	} else {
		mtk_disp_mutex_submit_sof(mtk_crtc->mutex[0]);
		if (crtc_id == 0) {
			if (mtk_crtc->panel_ext)
				params = mtk_crtc->panel_ext->params;
			/*Msync 2.0 add vfp period token instead of EOF*/
			if (mtk_drm_helper_get_opt(priv->helper_opt,
					MTK_DRM_OPT_MSYNC2_0) && params &&
					params->msync2_enable) {
				DDPDBG("[Msync]%s, add set vfp period token\n", __func__);
				cmdq_pkt_wfe(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
				/*clear last SOF*/
				cmdq_pkt_clear_event(cmdq_handle,
						mtk_crtc->gce_obj.event[EVENT_DSI_SOF]);

				/*for dynamic Msync on/off,set vfp period token*/
				cmdq_pkt_set_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);
			} else {
				cmdq_pkt_wfe(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
			}

		} else if (crtc_id == 1) {
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
				cmdq_pkt_wfe(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
			else
				cmdq_pkt_wfe(cmdq_handle,
					 mtk_crtc->gce_obj.event[EVENT_VDO_EOF]);
		} else {
			cmdq_pkt_wfe(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
		}

		/* sw workaround to fix gce hw bug */
		if (mtk_crtc_with_sodi_loop(crtc)) {
			cmdq_pkt_read(cmdq_handle, NULL,
				GCE_BASE_ADDR + GCE_DEBUG_START_ADDR, var1);

			/*mark condition jump */
			inst_condi_jump = cmdq_handle->cmd_buf_size;
			cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);

			cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump, &lop, &rop,
				CMDQ_NOT_EQUAL);

			/* if condition false, will jump here */
			cmdq_pkt_write(cmdq_handle, NULL,
				GCE_BASE_ADDR + GCE_GCTL_VALUE, 0, GCE_DDR_EN);

		      /* if condition true, will jump curreent postzion */
			inst = cmdq_pkt_get_va_by_offset(cmdq_handle,  inst_condi_jump);
			jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
						cmdq_handle->cmd_buf_size);
			*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);

			cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_SODI]);
		}

#ifdef IF_ZERO /* not ready for dummy register method */
		if (mtk_drm_helper_get_opt(priv->helper_opt,
					   MTK_DRM_OPT_LAYER_REC)) {
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_RDMA0_EOF]);
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

			cmdq_pkt_wfe(cmdq_handle,
				     mtk_crtc->gce_obj.event[EVENT_RDMA0_EOF]);
			mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle,
					      MTK_TRIG_FLAG_LAYER_REC);
			mtk_crtc_rec_trig_cnt(mtk_crtc, cmdq_handle);

			cmdq_pkt_set_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

			mtk_crtc->layer_rec_en = true;
		} else {
			mtk_crtc->layer_rec_en = false;
		}
#endif
		/*Msync 2.0 add vfp period token instead of EOF*/
		if (crtc_id == 0) {
			if (mtk_drm_helper_get_opt(priv->helper_opt,
					MTK_DRM_OPT_MSYNC2_0) && params &&
					params->msync2_enable) {
				/*wait next SOF*/
				cmdq_pkt_wait_no_clear(cmdq_handle,
						    mtk_crtc->gce_obj.event[EVENT_DSI_SOF]);
				/*clear last EOF*/
				cmdq_pkt_clear_event(cmdq_handle,
							mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
				/*clear vfp period token*/
				cmdq_pkt_clear_event(cmdq_handle,
					mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);
			}
		}
	}
	cmdq_pkt_finalize_loop(cmdq_handle);
	ret = cmdq_pkt_flush_async(cmdq_handle, trig_done_cb, (void *)crtc_id);

	mtk_crtc_clear_wait_event(crtc);
#endif
}

#ifdef DRM_CMDQ_DISABLE
void trigger_without_cmdq(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *cmdq_handle = state->cmdq_handle;
#ifndef CONFIG_FPGA_EARLY_PORTING
	struct mtk_drm_private *priv = crtc->dev->dev_private;
#endif

	DDPDBG("%s+\n",	__func__);

#ifndef CONFIG_FPGA_EARLY_PORTING
	/* wait for TE, fpga no TE signal */
	drm_wait_one_vblank(priv->drm, 0);
#endif

	DDPDBG("%s:%d for early porting\n",
		__func__, __LINE__);
	/*Trigger without cmdq*/
	mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0], cmdq_handle,
		mtk_crtc->gce_obj.base);
	mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle,
		MTK_TRIG_FLAG_TRIGGER);
	//loop for check idle of dsi, maybe timeout
	mtk_crtc_comp_trigger(mtk_crtc, cmdq_handle, MTK_TRIG_FLAG_EOF);

	DDPDBG("%s-\n",	__func__);
}
#endif

void mtk_crtc_hw_block_ready(struct drm_crtc *crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!mtk_crtc->trig_loop_cmdq_handle) {
		DDPDBG("%s: trig_loop is stopped\n", __func__);
		return;
	}

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_set_event(cmdq_handle,
			   mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

void mtk_crtc_stop_trig_loop(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!mtk_crtc->trig_loop_cmdq_handle) {
		DDPDBG("%s: trig_loop already stopped\n", __func__);
		return;
	}

	cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_TRIG_LOOP]);
	cmdq_pkt_destroy(mtk_crtc->trig_loop_cmdq_handle);
	mtk_crtc->trig_loop_cmdq_handle = NULL;
}

/* sw workaround to fix gce hw bug */
void mtk_crtc_stop_sodi_loop(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;

	if (!mtk_crtc->sodi_loop_cmdq_handle) {
		DDPDBG("%s: sodi_loop already stopped\n", __func__);
		return;
	}

	priv = mtk_crtc->base.dev->dev_private;
	cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_SODI_LOOP]);
	cmdq_pkt_destroy(mtk_crtc->sodi_loop_cmdq_handle);
	mtk_crtc->sodi_loop_cmdq_handle = NULL;
}

void mtk_crtc_stop_event_loop(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = NULL;

	if (!mtk_crtc->event_loop_cmdq_handle) {
		DDPDBG("%s: sodi_loop already stopped\n", __func__);
		return;
	}

	priv = mtk_crtc->base.dev->dev_private;
	cmdq_mbox_stop(mtk_crtc->gce_obj.client[CLIENT_EVENT_LOOP]);
	cmdq_pkt_destroy(mtk_crtc->event_loop_cmdq_handle);
	mtk_crtc->event_loop_cmdq_handle = NULL;
}

long mtk_crtc_wait_status(struct drm_crtc *crtc, bool status, long timeout)
{
	long ret;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	ret = wait_event_interruptible_timeout(mtk_crtc->crtc_status_wq,
				 mtk_crtc->enabled == status, timeout);
	if (!ret)
		DDPPR_ERR("wait %s fail, ret=%d\n", __func__, ret);

	return ret;
}

bool mtk_crtc_set_status(struct drm_crtc *crtc, bool status)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	bool old_status = mtk_crtc->enabled;

	mtk_crtc->enabled = status;
	wake_up(&mtk_crtc->crtc_status_wq);

	return old_status;
}

int mtk_crtc_attach_addon_path_comp(struct drm_crtc *crtc,
	const struct mtk_addon_module_data *module_data, bool is_attach)

{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	const struct mtk_addon_path_data *path_data =
		mtk_addon_module_get_path(module_data->module);
	struct mtk_ddp_comp *comp;
	int i;

	for (i = 0; i < path_data->path_len; i++) {
		if (mtk_ddp_comp_get_type(path_data->path[i]) ==
		    MTK_DISP_VIRTUAL)
			continue;
		comp = priv->ddp_comp[path_data->path[i]];
		if (is_attach)
			comp->mtk_crtc = mtk_crtc;
		else
			comp->mtk_crtc = NULL;
	}

	return 0;
}

int mtk_crtc_attach_ddp_comp(struct drm_crtc *crtc, int ddp_mode,
			     bool is_attach)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = (crtc && crtc->dev) ?
			crtc->dev->dev_private : NULL;
	struct mtk_ddp_comp *comp;
	int i, j;
	bool only_output;

	if (ddp_mode < 0)
		return -EINVAL;

	only_output = (priv && priv->usage[drm_crtc_index(crtc)] == DISP_OPENING);
	for_each_comp_in_crtc_target_mode(comp, mtk_crtc, i, j, ddp_mode) {
		if (only_output && !mtk_ddp_comp_is_output(comp))
			continue;
		if (is_attach)
			comp->mtk_crtc = mtk_crtc;
		else
			comp->mtk_crtc = NULL;
	}

	return 0;
}

int mtk_crtc_update_ddp_sw_status(struct drm_crtc *crtc, int enable)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (enable)
		mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);
	else
		mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, false);

	return 0;
}

static void mtk_crtc_addon_connector_disconnect(struct drm_crtc *crtc,
	struct cmdq_pkt *handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_ddp_comp *dsc_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;

	if (panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		if (drm_crtc_index(crtc) == 3)
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		switch (priv->data->mmsys_id) {
		case MMSYS_MT6885:
			mtk_ddp_remove_dsc_prim_MT6885(mtk_crtc, handle);
			break;
		case MMSYS_MT6983:
			mtk_ddp_remove_dsc_prim_MT6983(mtk_crtc, handle);
			break;
		case MMSYS_MT6985:
			if (drm_crtc_index(crtc) == 3)
				mtk_ddp_remove_dsc_ext_MT6985(mtk_crtc, handle);
			else
				mtk_ddp_remove_dsc_prim_MT6985(mtk_crtc, handle);
			break;
		case MMSYS_MT6895:
		case MMSYS_MT6886:
			mtk_ddp_remove_dsc_prim_MT6895(mtk_crtc, handle);
			break;
		case MMSYS_MT6873:
			mtk_ddp_remove_dsc_prim_MT6873(mtk_crtc, handle);
			break;
		case MMSYS_MT6853:
			mtk_ddp_remove_dsc_prim_MT6853(mtk_crtc, handle);
			break;
		case MMSYS_MT6879:
			mtk_ddp_remove_dsc_prim_MT6879(mtk_crtc, handle);
			break;
		case MMSYS_MT6855:
			mtk_ddp_remove_dsc_prim_MT6855(mtk_crtc, handle);
			break;
		default:
			DDPINFO("%s mtk drm not support mmsys id %d\n",
				__func__, priv->data->mmsys_id);
			break;
		}

		mtk_disp_mutex_remove_comp_with_cmdq(mtk_crtc, dsc_comp->id,
			handle, 0);
		mtk_ddp_comp_stop(dsc_comp, handle);
	}

}

void mtk_crtc_disconnect_addon_module(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];

	mtk_crtc_pkt_create(&handle, crtc, client);

	if (!handle) {
		DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
		return;
	}

	_mtk_crtc_atmoic_addon_module_disconnect(
		crtc, mtk_crtc->ddp_mode, &crtc_state->lye_state, handle);

	mtk_crtc_addon_connector_disconnect(crtc, handle);

	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);
}

void mtk_crtc_addon_connector_connect(struct drm_crtc *crtc,
	struct cmdq_pkt *_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_ddp_comp *dsc_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct mtk_ddp_comp *output_comp;

	if (panel_ext &&
		panel_ext->output_mode == MTK_PANEL_DSC_SINGLE_PORT) {
		struct mtk_ddp_config cfg;
		bool flush = false;
		struct cmdq_pkt *handle = NULL;

		if (_handle) {
			handle = _handle;
		} else {
			mtk_crtc_pkt_create(&handle, crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);
			flush = true;
		}

		if (!handle) {
			DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
			return;
		}

		if (drm_crtc_index(crtc) == 3)
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC1];
		else
			dsc_comp = priv->ddp_comp[DDP_COMPONENT_DSC0];

		if (mtk_crtc->scaling_ctx.scaling_en) {
			cfg.w = mtk_crtc_get_width_by_comp(__func__, crtc, dsc_comp, true);
			cfg.h = mtk_crtc_get_height_by_comp(__func__, crtc, dsc_comp, true);
		} else {
			cfg.w = crtc->state->adjusted_mode.hdisplay;
			cfg.h = crtc->state->adjusted_mode.vdisplay;
			output_comp = mtk_ddp_comp_request_output(mtk_crtc);
			if (output_comp && drm_crtc_index(crtc) == 0) {
				cfg.w = mtk_ddp_comp_io_cmd(
						output_comp, NULL,
						DSI_GET_VIRTUAL_WIDTH, NULL);
				cfg.h = mtk_ddp_comp_io_cmd(
						output_comp, NULL,
						DSI_GET_VIRTUAL_HEIGH, NULL);
			}
		}

		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				cfg.vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				cfg.vrefresh =
					drm_mode_vrefresh(&crtc->state->adjusted_mode);
		} else
			cfg.vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
		cfg.bpc = mtk_crtc->bpc;

		if (mtk_crtc->scaling_ctx.scaling_en)
			cfg.p_golden_setting_context =
				__get_scaling_golden_setting_context(mtk_crtc);
		else
			cfg.p_golden_setting_context =
				__get_golden_setting_context(mtk_crtc);

		dsc_comp->mtk_crtc = mtk_crtc;

		/* insert DSC */
		switch (priv->data->mmsys_id) {
		case MMSYS_MT6885:
			mtk_ddp_insert_dsc_prim_MT6885(mtk_crtc, handle);
			break;
		case MMSYS_MT6983:
			mtk_ddp_insert_dsc_prim_MT6983(mtk_crtc, handle);
			break;
		case MMSYS_MT6985:
			if (drm_crtc_index(crtc) == 3)
				mtk_ddp_insert_dsc_ext_MT6985(mtk_crtc, handle);
			else
				mtk_ddp_insert_dsc_prim_MT6985(mtk_crtc, handle);
			break;
		case MMSYS_MT6873:
			mtk_ddp_insert_dsc_prim_MT6873(mtk_crtc, handle);
			break;
		case MMSYS_MT6895:
		case MMSYS_MT6886:
			mtk_ddp_insert_dsc_prim_MT6895(mtk_crtc, handle);
			break;
		case MMSYS_MT6853:
			mtk_ddp_insert_dsc_prim_MT6853(mtk_crtc, handle);
			break;
		case MMSYS_MT6879:
			mtk_ddp_insert_dsc_prim_MT6879(mtk_crtc, handle);
			break;
		case MMSYS_MT6855:
			mtk_ddp_insert_dsc_prim_MT6855(mtk_crtc, handle);
			break;
		default:
			DDPINFO("%s mtk drm not support mmsys id %d\n",
				__func__, priv->data->mmsys_id);
			break;
		}

		mtk_disp_mutex_add_comp_with_cmdq(mtk_crtc, dsc_comp->id,
			mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base),
			handle, 0);

		mtk_ddp_comp_config(dsc_comp, &cfg, handle);
		mtk_ddp_comp_start(dsc_comp, handle);

		if (flush) {
			cmdq_pkt_flush(handle);
			cmdq_pkt_destroy(handle);
		}
	}

}

void mtk_crtc_connect_addon_module(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];

	mtk_crtc_pkt_create(&handle, crtc, client);

	if (!handle) {
		DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
		return;
	}

	_mtk_crtc_atmoic_addon_module_connect(crtc, mtk_crtc->ddp_mode,
					      &crtc_state->lye_state, handle);

	mtk_crtc_addon_connector_connect(crtc, handle);

	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);
}

/* set mutex & path mux for this CRTC default path */
void mtk_crtc_connect_default_path(struct mtk_drm_crtc *mtk_crtc)
{
	unsigned int i, j;
	struct mtk_ddp_comp *comp;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = (crtc && crtc->dev) ?
		crtc->dev->dev_private : NULL;
	enum mtk_ddp_comp_id prev_id, next_id;
	bool only_output;

	only_output = (priv && priv->usage[drm_crtc_index(crtc)] == DISP_OPENING);
	/* connect path */
	for_each_comp_in_crtc_path_bound(comp, mtk_crtc, i, j, 1) {
		struct mtk_ddp_comp *tmp_comp;

		if (only_output)
			break;
		if (j == 0) {
			prev_id = DDP_COMPONENT_ID_MAX;
		} else {
			tmp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j - 1);
			prev_id = tmp_comp ? tmp_comp->id : DDP_COMPONENT_ID_MAX;
		}

		tmp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j + 1);
		next_id = tmp_comp ? tmp_comp->id : DDP_COMPONENT_ID_MAX;

		DDPINFO("%s, %s --> %s --> %s\n", __func__,
			mtk_dump_comp_str_id(prev_id),
			mtk_dump_comp_str(comp),
			mtk_dump_comp_str_id(next_id));
		mtk_ddp_add_comp_to_path(mtk_crtc, comp, prev_id,
					 next_id);
	}

	/* add module in mutex */
	if (mtk_crtc_is_dc_mode(crtc)) {
		/* DC mode no need skip output */
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_FIRST_PATH)
			mtk_disp_mutex_add_comp(mtk_crtc->mutex[1], comp->id);
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_SECOND_PATH)
			mtk_disp_mutex_add_comp(mtk_crtc->mutex[0], comp->id);
	} else {
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_FIRST_PATH) {
			if (only_output && !mtk_ddp_comp_is_output(comp))
				continue;
			mtk_disp_mutex_add_comp(mtk_crtc->mutex[0], comp->id);
		}
	}

	if (mtk_crtc->is_dual_pipe) {
		if (!only_output)
			mtk_ddp_connect_dual_pipe_path(mtk_crtc, mtk_crtc->mutex[0]);
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			if (only_output && !mtk_ddp_comp_is_output(comp))
				continue;
			mtk_disp_mutex_add_comp(mtk_crtc->mutex[0], comp->id);
		}

		/* TODO: should check DUAL port DSI */
		//if (drm_crtc_index(crtc) == 0)
		//	mtk_disp_mutex_add_comp
		//		(mtk_crtc->mutex[0], DDP_COMPONENT_DSI1);
	}
	/* set mutex sof, eof */
	mtk_disp_mutex_src_set(mtk_crtc, mtk_crtc_is_frame_trigger_mode(crtc));
	/* if VDO mode, enable mutex by CPU here */
	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_disp_mutex_enable(mtk_crtc->mutex[0]);
}

void mtk_crtc_init_plane_setting(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_plane_state *plane_state;
	struct mtk_plane_pending_state *pending;
	struct drm_plane *plane = &mtk_crtc->planes[0].base;

	plane_state = to_mtk_plane_state(plane->state);
	pending = &plane_state->pending;

	pending->pitch = mtk_crtc->base.state->adjusted_mode.hdisplay*3;
	pending->format = DRM_FORMAT_RGB888;
	pending->src_x = 0;
	pending->src_y = 0;
	pending->dst_x = 0;
	pending->dst_y = 0;
	pending->height = mtk_crtc->base.state->adjusted_mode.vdisplay;
	pending->width = mtk_crtc->base.state->adjusted_mode.hdisplay;
	pending->config = 1;
	pending->dirty = 1;
	pending->enable = true;

	/*constant color layer*/
	pending->addr = 0;
	pending->prop_val[PLANE_PROP_PLANE_ALPHA] = 0xFF;
	pending->prop_val[PLANE_PROP_COMPRESS] = 0;
	pending->prop_val[PLANE_PROP_ALPHA_CON] = 0;

	plane_state->comp_state.lye_id = 0;
	plane_state->comp_state.ext_lye_id = 0;
}

void mtk_crtc_dual_layer_config(struct mtk_drm_crtc *mtk_crtc,
		struct mtk_ddp_comp *comp, unsigned int idx,
		struct mtk_plane_state *plane_state, struct cmdq_pkt *cmdq_handle)
{
	struct drm_crtc *crtc = NULL;
	struct mtk_drm_private *priv = NULL;
	struct mtk_plane_state plane_state_l = {0};
	struct mtk_plane_state plane_state_r = {0};
	struct mtk_ddp_comp *p_comp = NULL;

	if (!mtk_crtc || !comp) {
		DDPINFO("%s failed with NULL argument mtk_crtc or comp\n", __func__);
		return;
	}

	crtc = &mtk_crtc->base;
	priv = mtk_crtc->base.dev->dev_private;
	mtk_drm_layer_dispatch_to_dual_pipe(mtk_crtc, priv->data->mmsys_id,
		plane_state, &plane_state_l, &plane_state_r,
		crtc->state->adjusted_mode.hdisplay);

	if (plane_state->comp_state.comp_id == 0)
		plane_state_r.comp_state.comp_id = 0;

	p_comp = priv->ddp_comp[dual_pipe_comp_mapping(priv->data->mmsys_id, comp->id)];
	mtk_ddp_comp_layer_config(p_comp, idx,
				&plane_state_r, cmdq_handle);
	DDPINFO("%s+ comp_id:%d, comp_id:%d\n",
		__func__, p_comp->id,
		plane_state_r.comp_state.comp_id);

	p_comp = comp;
	mtk_ddp_comp_layer_config(p_comp, idx, &plane_state_l,
				  cmdq_handle);
}
/* restore ovl layer config and set dal layer if any */
void mtk_crtc_restore_plane_setting(struct mtk_drm_crtc *mtk_crtc)
{
	unsigned int i, j;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (drm_crtc_index(crtc) == 1)
		mtk_crtc_init_plane_setting(mtk_crtc);

	//discrete mdp_rdma do not restore
	if (mtk_crtc->path_data->is_discrete_path) {
		cmdq_pkt_destroy(cmdq_handle);
		return;
	}

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state;
		struct mtk_crtc_state *mtk_crtc_state = NULL;

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->base.crtc == NULL && plane_state->pending.enable) {
			DDPINFO("%s CRTC NULL but plane%d pending, skip restore\n", __func__, i);
			break;
		}

		mtk_crtc_state = to_mtk_crtc_state(mtk_crtc->base.state);
		if (!plane_state->pending.enable ||
			(i >= OVL_PHY_LAYER_NR && !plane_state->comp_state.comp_id) ||
			((plane_state->comp_state.layer_caps & MTK_DISP_RSZ_LAYER) &&
			mtk_crtc_state &&
			(mtk_crtc_state->rsz_param[0].out_len == 0 &&
			(mtk_crtc->is_dual_pipe && mtk_crtc_state->rsz_param[1].out_len == 0)))) {
			DDPINFO("%s i=%d comp_id=%u (%d,%d,%d,%d) continue\n", __func__, i,
				plane_state->comp_state.comp_id,
				(plane_state->comp_state.layer_caps & MTK_DISP_RSZ_LAYER),
				plane_state->pending.enable,
				mtk_crtc_state->rsz_param[0].out_len,
				mtk_crtc_state->rsz_param[1].out_len);
			continue;
		}

		if (plane_state->comp_state.comp_id)
			comp = priv->ddp_comp[plane_state->comp_state.comp_id];
		else {
			/* TODO: all plane should contain proper mtk_plane_state
			 */
			comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, DDP_FIRST_PATH, 0);
			DDPINFO("%s no comp_id, assign to comp:%d\n", __func__,
							comp ? comp->id : -EINVAL);
		}

		if (comp == NULL)
			continue;

		if (mtk_crtc->is_dual_pipe)
			mtk_crtc_dual_layer_config(mtk_crtc, comp, i, plane_state, cmdq_handle);
		else
			mtk_ddp_comp_layer_config(comp, i, plane_state, cmdq_handle);

#ifdef IF_ZERO
		if (comp->id == DDP_COMPONENT_OVL2_2L
			&& mtk_crtc->is_dual_pipe) {
			DDPFUNC();
			if (plane_state->pending.addr)
				plane_state->pending.addr +=
					plane_state->pending.pitch/2;

			plane_state->pending.dst_x = 0;
			plane_state->pending.dst_y = 0;
			comp = mtk_crtc_get_dual_comp(crtc, DDP_FIRST_PATH, 0);
			plane_state->comp_state.comp_id =
						DDP_COMPONENT_OVL3_2L;
			mtk_ddp_comp_layer_config(comp,
						i, plane_state, cmdq_handle);
			//will be used next time
			plane_state->comp_state.comp_id = DDP_COMPONENT_OVL2_2L;
		}
#endif

	}

	if (mtk_drm_dal_enable() && drm_crtc_index(crtc) == 0)
		drm_set_dal(&mtk_crtc->base, cmdq_handle);

	if (mtk_crtc->is_mml) {
		comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, DDP_FIRST_PATH, 0);

		if (comp) {
			mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, DDP_FIRST_PATH, 0);
			cmdq_pkt_write(cmdq_handle, NULL, comp->larb_cons[0], GENMASK(19, 16),
			       GENMASK(19, 16));
		} else
			DDPPR_ERR("%s with NULL comp\n", __func__);

		if (mtk_crtc->is_dual_pipe) {
			comp = mtk_crtc_get_dual_comp(crtc, DDP_FIRST_PATH, 0);
			cmdq_pkt_write(cmdq_handle, NULL, comp->larb_cons[0], GENMASK(19, 16),
				       GENMASK(19, 16));
		}
	}

	/* Update QOS BW*/
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, cmdq_handle,
			PMQOS_UPDATE_BW, NULL);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}
static void mtk_crtc_disable_plane_setting(struct mtk_drm_crtc *mtk_crtc)
{
	unsigned int i;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state
			= to_mtk_plane_state(plane->state);

		if (i < OVL_PHY_LAYER_NR || plane_state->comp_state.comp_id)
			plane_state->pending.enable = 0;
	}
}

static void set_dirty_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

void mtk_crtc_set_dirty(struct mtk_drm_crtc *mtk_crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_cmdq_cb_data *cb_data;

	// Temp code. This flow will only enter by debug command
	// (CMD mode will enter MML IR by debug command), we don't
	// have to worry about it will effect the original flow.
	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base) &&
		mtk_crtc->is_mml && mtk_crtc->mml_cfg) {
		return;
	}

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPINFO("%s:%d, cb data creation failed\n",
			__func__, __LINE__);
		return;
	}

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_set_event(cmdq_handle,
		mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

	cb_data->cmdq_handle = cmdq_handle;
	if (cmdq_pkt_flush_threaded(cmdq_handle,
	    set_dirty_cmdq_cb, cb_data) < 0)
		DDPPR_ERR("failed to flush set_dirty\n");
}

static int __mtk_check_trigger(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	int index = drm_crtc_index(crtc);
	struct mtk_crtc_state *mtk_state;

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	CRTC_MMP_EVENT_START(index, check_trigger, 0, 0);

	if (!mtk_crtc->enabled) {
		CRTC_MMP_EVENT_END(index, check_trigger, 0, 1);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

		return 0;
	}

	mtk_drm_idlemgr_kick(__func__, &mtk_crtc->base, 0);

	mtk_state = to_mtk_crtc_state(crtc->state);
	if (!mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE] ||
		(mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE] &&
		atomic_read(&mtk_crtc->already_config))) {
		mtk_crtc_set_dirty(mtk_crtc);
	} else
		DDPINFO("%s skip mtk_crtc_set_dirty\n", __func__);

	CRTC_MMP_EVENT_END(index, check_trigger, 0, 0);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

static int _mtk_crtc_check_trigger(void *data)
{
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *) data;
	struct sched_param param = {.sched_priority = 94 };
	int ret;

	sched_setscheduler(current, SCHED_RR, &param);

	atomic_set(&mtk_crtc->trig_event_act, 0);
	while (1) {
		ret = wait_event_interruptible(mtk_crtc->trigger_event,
			atomic_read(&mtk_crtc->trig_event_act));
		if (ret < 0)
			DDPPR_ERR("wait %s fail, ret=%d\n", __func__, ret);
		atomic_set(&mtk_crtc->trig_event_act, 0);

		__mtk_check_trigger(mtk_crtc);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static int _mtk_crtc_check_trigger_delay(void *data)
{
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *) data;
	struct sched_param param = {.sched_priority = 94 };
	int ret;

	sched_setscheduler(current, SCHED_RR, &param);

	atomic_set(&mtk_crtc->trig_delay_act, 0);

	while (1) {
		ret = wait_event_interruptible(mtk_crtc->trigger_delay,
			atomic_read(&mtk_crtc->trig_delay_act));
		if (ret < 0)
			DDPPR_ERR("wait %s fail, ret=%d\n", __func__, ret);
		atomic_set(&mtk_crtc->trig_delay_act, 0);
		atomic_set(&mtk_crtc->delayed_trig, 0);

		usleep_range(32000, 33000);
		if (!atomic_read(&mtk_crtc->delayed_trig))
			__mtk_check_trigger(mtk_crtc);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

void mtk_crtc_check_trigger(struct mtk_drm_crtc *mtk_crtc, bool delay,
		bool need_lock)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	int index = 0;
	struct mtk_crtc_state *mtk_state;
	struct mtk_panel_ext *panel_ext;

	if (!mtk_crtc) {
		DDPPR_ERR("%s:%d, invalid crtc:0x%p\n",
				__func__, __LINE__, crtc);
		return;
	}

	if (need_lock)
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	CRTC_MMP_EVENT_START(index, kick_trigger, (unsigned long)crtc, 0);

	index = drm_crtc_index(crtc);
	if (index) {
		DDPPR_ERR("%s:%d, invalid crtc:0x%p, index:%d\n",
				__func__, __LINE__, crtc, index);
		CRTC_MMP_MARK(index, kick_trigger, 0, 1);
		goto err;
	}

	if (!(mtk_crtc->enabled)) {
		DDPINFO("%s:%d, slepted\n", __func__, __LINE__);
		CRTC_MMP_MARK(index, kick_trigger, 0, 2);
		goto err;
	}

	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		DDPINFO("%s:%d, not in trigger mode\n", __func__, __LINE__);
		CRTC_MMP_MARK(index, kick_trigger, 0, 3);
		goto err;
	}

	if (mtk_crtc->is_mml) {
		DDPINFO("%s:%d, skip check trigger when MML IR\n", __func__, __LINE__);
		CRTC_MMP_MARK(index, kick_trigger, 0, 4);
		goto err;
	}

	panel_ext = mtk_crtc->panel_ext;
	mtk_state = to_mtk_crtc_state(crtc->state);
	if (mtk_crtc_is_frame_trigger_mode(crtc) &&
		mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE] &&
		panel_ext && panel_ext->params->doze_delay > 1){
		DDPINFO("%s:%d, doze not to trigger\n", __func__, __LINE__);
		goto err;
	}

	if (delay) {
		/* implicit way make sure wait queue was initiated */
		if (unlikely(&mtk_crtc->trigger_delay_task == NULL)) {
			CRTC_MMP_MARK(index, kick_trigger, 0, 5);
			goto err;
		}
		atomic_set(&mtk_crtc->trig_delay_act, 1);
		wake_up_interruptible(&mtk_crtc->trigger_delay);
	} else {
		/* implicit way make sure wait queue was initiated */
		if (unlikely(&mtk_crtc->trigger_event_task == NULL)) {
			CRTC_MMP_MARK(index, kick_trigger, 0, 6);
			goto err;
		}
		atomic_set(&mtk_crtc->trig_event_act, 1);
		wake_up_interruptible(&mtk_crtc->trigger_event);
	}

err:
	CRTC_MMP_EVENT_END(index, kick_trigger, 0, 0);
	if (need_lock)
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
}

void mtk_crtc_config_default_path(struct mtk_drm_crtc *mtk_crtc)
{
	int i, j;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_config cfg = {0};
	struct mtk_ddp_config scaling_cfg = {0};
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_comp *output_comp;
	bool only_output;

	if ((mtk_crtc->res_switch == RES_SWITCH_ON_AP)
		&& mtk_crtc->scaling_ctx.scaling_en
		&& (crtc->state->adjusted_mode.hdisplay == mtk_crtc->scaling_ctx.lcm_width)
		&& (crtc->state->adjusted_mode.vdisplay == mtk_crtc->scaling_ctx.lcm_height)) {
		DDPMSG("%s scaling_en mismatch, reset to false\n", __func__);
		mtk_crtc->scaling_ctx.scaling_en = false;
	}

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	cfg.w = crtc->state->adjusted_mode.hdisplay;
	cfg.h = crtc->state->adjusted_mode.vdisplay;
	cfg.clock = crtc->state->adjusted_mode.clock;
	if (output_comp && drm_crtc_index(crtc) == 0) {
		cfg.w = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_WIDTH, NULL);
		cfg.h = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_HEIGH, NULL);
	}
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params &&
		mtk_crtc->panel_ext->params->dyn_fps.switch_en == 1
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		cfg.vrefresh =
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	else
		cfg.vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg.bpc = mtk_crtc->bpc;
	cfg.p_golden_setting_context = __get_golden_setting_context(mtk_crtc);

	cfg.rsz_src_w = cfg.w;
	cfg.rsz_src_h = cfg.h;

	if (mtk_crtc->scaling_ctx.scaling_en) {
		memcpy(&scaling_cfg, &cfg, sizeof(struct mtk_ddp_config));
		scaling_cfg.w = mtk_crtc->scaling_ctx.scaling_mode->hdisplay;
		scaling_cfg.h = mtk_crtc->scaling_ctx.scaling_mode->vdisplay;
		scaling_cfg.p_golden_setting_context =
			__get_scaling_golden_setting_context(mtk_crtc);
	}

#ifndef DRM_CMDQ_DISABLE
	if (priv->data->mmsys_id == MMSYS_MT6983 ||
		priv->data->mmsys_id == MMSYS_MT6879 ||
		priv->data->mmsys_id == MMSYS_MT6895 ||
		priv->data->mmsys_id == MMSYS_MT6835 ||
		priv->data->mmsys_id == MMSYS_MT6855 ||
		priv->data->mmsys_id == MMSYS_MT6886) {
		/*Set EVENT_GCED_EN EVENT_GCEM_EN*/
		writel(0x3, mtk_crtc->config_regs +
				DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

		/*Set BYPASS_MUX_SHADOW*/
		writel(0x1, mtk_crtc->config_regs +
				DISP_REG_CONFIG_BYPASS_MUX_SHADOW);

		if (mtk_crtc->side_config_regs) {
			writel(0x3, mtk_crtc->side_config_regs +
					DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

			/*Set BYPASS_MUX_SHADOW*/
			writel(0x1, mtk_crtc->side_config_regs +
					DISP_REG_CONFIG_BYPASS_MUX_SHADOW);
		}
	} else if (priv->data->mmsys_id == MMSYS_MT6985) {
		/*Set EVENT_GCED_EN EVENT_GCEM_EN*/
		writel(0x3, mtk_crtc->config_regs +
				DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

		if (mtk_crtc->side_config_regs)
			writel(0x3, mtk_crtc->side_config_regs +
					DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

		/*Set EVENT_GCED_EN EVENT_GCEM_EN <OVLSYS>*/
		writel(0x3, mtk_crtc->ovlsys0_regs +
				DISP_REG_CONFIG_OVLSYS_GCE_EVENT_SEL);

		if (mtk_crtc->ovlsys1_regs)
			writel(0x3, mtk_crtc->ovlsys1_regs +
					DISP_REG_CONFIG_OVLSYS_GCE_EVENT_SEL);
	}
#endif

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (mtk_crtc->is_dual_pipe &&
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_TILE_OVERHEAD)) {
		cfg.tile_overhead.is_support = true;
		cfg.tile_overhead.left_in_width = cfg.w / 2;
		cfg.tile_overhead.right_in_width = cfg.w / 2;

		if (mtk_crtc->scaling_ctx.scaling_en) {
			scaling_cfg.tile_overhead.is_support = true;
			scaling_cfg.tile_overhead.left_in_width = scaling_cfg.w / 2;
			scaling_cfg.tile_overhead.right_in_width = scaling_cfg.w / 2;
		}

	} else {
		cfg.tile_overhead.is_support = false;
		cfg.tile_overhead.left_in_width = cfg.w;
		cfg.tile_overhead.right_in_width = cfg.w;
	}

	/*Calculate total overhead*/
	for_each_comp_in_crtc_path_reverse(comp, mtk_crtc, i, j) {

		if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path) {
			mtk_ddp_comp_config_overhead(comp, &scaling_cfg);
			DDPINFO("%s:comp %s (scaling_path) width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				scaling_cfg.tile_overhead.left_in_width,
				scaling_cfg.tile_overhead.right_in_width,
				scaling_cfg.tile_overhead.left_overhead,
				scaling_cfg.tile_overhead.right_overhead);
		} else {
			mtk_ddp_comp_config_overhead(comp, &cfg);
			DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				cfg.tile_overhead.left_in_width,
				cfg.tile_overhead.right_in_width,
				cfg.tile_overhead.left_overhead,
				cfg.tile_overhead.right_overhead);
		}

		if (mtk_crtc->scaling_ctx.scaling_en
			&& mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
			cfg.tile_overhead = scaling_cfg.tile_overhead;
	}
	only_output = (priv->usage[drm_crtc_index(crtc)] == DISP_OPENING);
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (only_output && !mtk_ddp_comp_is_output(comp))
			continue;
		if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path)
			mtk_ddp_comp_config(comp, &scaling_cfg, cmdq_handle);
		else
			mtk_ddp_comp_config(comp, &cfg, cmdq_handle);

		mtk_ddp_comp_start(comp, cmdq_handle);

		if (!mtk_drm_helper_get_opt(
				    priv->helper_opt,
				    MTK_DRM_OPT_USE_PQ))
			mtk_ddp_comp_bypass(comp, 1, cmdq_handle);

		if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL) {
			cfg.source_bpc = mtk_ddp_comp_io_cmd(comp, NULL,
				OVL_GET_SOURCE_BPC, NULL);
			scaling_cfg.source_bpc = cfg.source_bpc;
		}
	}

	if (mtk_crtc->is_dual_pipe) {
		/*Calculate total overhead*/
		for_each_comp_in_dual_pipe_reverse(comp, mtk_crtc, i, j) {

			if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path) {
				mtk_ddp_comp_config_overhead(comp, &scaling_cfg);
				DDPINFO("%s:comp %s (scaling_path) width L:%d R:%d, overhead L:%d R:%d\n",
					__func__, mtk_dump_comp_str(comp),
					scaling_cfg.tile_overhead.left_in_width,
					scaling_cfg.tile_overhead.right_in_width,
					scaling_cfg.tile_overhead.left_overhead,
					scaling_cfg.tile_overhead.right_overhead);
			} else {
				mtk_ddp_comp_config_overhead(comp, &cfg);
				DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
					__func__, mtk_dump_comp_str(comp),
					cfg.tile_overhead.left_in_width,
					cfg.tile_overhead.right_in_width,
					cfg.tile_overhead.left_overhead,
					cfg.tile_overhead.right_overhead);
			}

			if (mtk_crtc->scaling_ctx.scaling_en
				&& mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
				cfg.tile_overhead = scaling_cfg.tile_overhead;
		}

		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			if (only_output && !mtk_ddp_comp_is_output(comp))
				continue;
			if (mtk_crtc->scaling_ctx.scaling_en && comp->in_scaling_path)
				mtk_ddp_comp_config(comp, &scaling_cfg, cmdq_handle);
			else
				mtk_ddp_comp_config(comp, &cfg, cmdq_handle);

			mtk_ddp_comp_start(comp, cmdq_handle);

			if (!mtk_drm_helper_get_opt(
				    priv->helper_opt,
				    MTK_DRM_OPT_USE_PQ))
				mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
		}
	}

	/*store total overhead data*/
	mtk_crtc_store_total_overhead(mtk_crtc, cfg.tile_overhead);

	/* Althought some of the m4u port may be enabled in LK stage.
	 * To make sure the driver independent, we still enable all the
	 * componets port here.
	 */
	mtk_crtc_enable_iommu(mtk_crtc, cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

static void mtk_crtc_all_layer_off(struct mtk_drm_crtc *mtk_crtc,
				   struct cmdq_pkt *cmdq_handle)
{
	int i, j, keep_first_layer;
	struct mtk_ddp_comp *comp;

	keep_first_layer = true;
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, cmdq_handle,
			OVL_ALL_LAYER_OFF, &keep_first_layer);

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				OVL_ALL_LAYER_OFF, &keep_first_layer);
	}
}

void mtk_crtc_stop_ddp(struct mtk_drm_crtc *mtk_crtc,
		       struct cmdq_pkt *cmdq_handle)
{
	int i, j;
	struct mtk_ddp_comp *comp;
	struct drm_crtc *crtc = mtk_crtc ? (&mtk_crtc->base) : NULL;
	struct mtk_drm_private *priv = (crtc && crtc->dev) ?
				crtc->dev->dev_private : NULL;
	unsigned int crtc_idx;
	bool only_output = false;

	if (IS_ERR_OR_NULL(mtk_crtc)) {
		DDPPR_ERR("%s:%u invalid mtk_crtc\n", __func__, __LINE__);
		return;
	}

	if (crtc) {
		crtc_idx = drm_crtc_index(crtc);
		if (crtc_idx < MAX_CRTC)
			only_output = (priv && priv->usage[crtc_idx] == DISP_OPENING);
	}

	/* If VDO mode, stop DSI mode first */
	if (!mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base) &&
	    mtk_crtc_is_connector_enable(mtk_crtc)) {
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					    DSI_STOP_VDO_MODE, NULL);
	}

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (only_output && !mtk_ddp_comp_is_output(comp))
			continue;
		mtk_ddp_comp_stop(comp, cmdq_handle);
	}

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			if (only_output && !mtk_ddp_comp_is_output(comp))
				continue;
			mtk_ddp_comp_stop(comp, cmdq_handle);
		}
	}
}

/* Stop trig loop and stop all modules in this CRTC */
void mtk_crtc_stop(struct mtk_drm_crtc *mtk_crtc, bool need_wait)
{
#ifdef DRM_CMDQ_DISABLE
	DDPINFO("%s:%d +\n", __func__, __LINE__);
	return;
#else
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp;
	int i, j;

	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct drm_device *dev = crtc->dev;
	struct mtk_drm_private *priv = dev->dev_private;

	DDPINFO("%s:%d +\n", __func__, __LINE__);

	/* 0. Waiting CLIENT_DSI_CFG thread done */
	if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] != NULL) {
		mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return;
		}

		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (!need_wait)
		goto skip;

	if (crtc_id == 2) {
		int gce_event =
			get_path_wait_event(mtk_crtc, mtk_crtc->ddp_mode);

		if (gce_event > 0)
			cmdq_pkt_wait_no_clear(cmdq_handle, gce_event);
	} else if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		/* 1. wait stream eof & clear tocken */
		/* clear eof token to prevent any config after this command */
		cmdq_pkt_clear_event(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		/* clear dirty token to prevent trigger loop start */
		cmdq_pkt_clear_event(
			cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	} else if (mtk_crtc_is_connector_enable(mtk_crtc)) {
		/* In vdo mode, DSI would be stop when disable connector
		 * Do not wait frame done in this case.
		 */
		cmdq_pkt_wfe(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
	}

skip:
	/* 2. stop all modules in this CRTC */
	mtk_crtc_stop_ddp(mtk_crtc, cmdq_handle);

	/* 3. Reset QOS BW after CRTC stop */
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, cmdq_handle,
			PMQOS_UPDATE_BW, NULL);

	/* 3.1 stop the last mml pkt */
	if (kref_read(&mtk_crtc->mml_ir_sram.ref)) {
		struct mtk_drm_sram_list *entry, *tmp;

		if (mtk_crtc_is_frame_trigger_mode(crtc) || mtk_crtc_is_connector_enable(mtk_crtc))
			mtk_crtc_mml_racing_stop_sync(crtc, cmdq_handle, false);

		mutex_lock(&mtk_crtc->mml_ir_sram.lock);
		entry = list_last_entry(&mtk_crtc->mml_ir_sram.list.head,
					struct mtk_drm_sram_list, head);
		mtk_crtc->mml_ir_sram.bk_hrt_idx = entry->hrt_idx;
		list_for_each_entry_safe(entry, tmp, &mtk_crtc->mml_ir_sram.list.head, head) {
			list_del_init(&entry->head);
			kfree(entry);
		}
		mtk_crtc_free_sram(mtk_crtc);
		refcount_set(&mtk_crtc->mml_ir_sram.ref.refcount, 0);
		mutex_unlock(&mtk_crtc->mml_ir_sram.lock);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	/* 4. Set QOS BW to 0 */
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_BW, NULL);

	/* 5. Set HRT BW to 0 */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
			MTK_DRM_OPT_MMQOS_SUPPORT))
			mtk_disp_set_hrt_bw(mtk_crtc, 0);

	/* 6. stop trig loop  */
	if (mtk_crtc_with_trigger_loop(crtc)) {
		mtk_crtc_stop_trig_loop(crtc);
		if (mtk_crtc_with_sodi_loop(crtc) &&
				(!mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_stop_sodi_loop(crtc);
	}

	if (mtk_crtc_with_event_loop(crtc) &&
			(mtk_crtc_is_frame_trigger_mode(crtc)))
		mtk_crtc_stop_event_loop(crtc);

	atomic_set(&mtk_crtc->force_high_step, 0);
	DDPINFO("%s:%d -\n", __func__, __LINE__);
#endif
}

/* TODO: how to remove add-on module? */
void mtk_crtc_disconnect_default_path(struct mtk_drm_crtc *mtk_crtc)
{
	int i, j;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_ddp_comp *comp;

	/* if VDO mode, disable mutex by CPU here */
	if (!mtk_crtc_is_frame_trigger_mode(crtc))
		mtk_disp_mutex_disable(mtk_crtc->mutex[0]);

	for_each_comp_in_crtc_path_bound(comp, mtk_crtc, i, j, 1) {
		struct mtk_ddp_comp *tmp_comp;
		unsigned int next_comp_id;

		tmp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j + 1);
		next_comp_id = tmp_comp ? tmp_comp->id : DDP_COMPONENT_ID_MAX;
		mtk_ddp_remove_comp_from_path(mtk_crtc,
			comp->id, next_comp_id);
	}

	/* reset dsc output_swap state */
	mtk_crtc->is_dsc_output_swap = false;

	if (mtk_crtc_is_dc_mode(crtc)) {
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_FIRST_PATH)
			mtk_disp_mutex_remove_comp(mtk_crtc->mutex[1],
						  comp->id);
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_SECOND_PATH)
			mtk_disp_mutex_remove_comp(mtk_crtc->mutex[0],
						  comp->id);
	} else {
		for_each_comp_in_crtc_target_path(comp, mtk_crtc, i,
						  DDP_FIRST_PATH)
			mtk_disp_mutex_remove_comp(mtk_crtc->mutex[0],
						   comp->id);
	}

	if (mtk_crtc->is_dual_pipe) {
		mtk_ddp_disconnect_dual_pipe_path(mtk_crtc, mtk_crtc->mutex[0]);

		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			struct mtk_ddp_comp *tmp_comp;
			unsigned int next_comp_id;

			if (j + 1 >= __mtk_crtc_dual_path_len(mtk_crtc, i))
				tmp_comp = NULL;
			else
				tmp_comp = mtk_crtc_get_dual_comp(crtc, i, j + 1);
			next_comp_id = tmp_comp ? tmp_comp->id : DDP_COMPONENT_ID_MAX;
			if (next_comp_id < DDP_COMPONENT_ID_MAX && comp->id < DDP_COMPONENT_ID_MAX)
				mtk_ddp_remove_comp_from_path(mtk_crtc,
					comp->id, next_comp_id);
			mtk_disp_mutex_remove_comp(mtk_crtc->mutex[0],
				comp->id);
		}
	}
}

#ifndef DRM_CMDQ_DISABLE
void mtk_crtc_prepare_instr(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *handle;

	if (priv->data->mmsys_id == MMSYS_MT6983 ||
		priv->data->mmsys_id == MMSYS_MT6879 ||
		priv->data->mmsys_id == MMSYS_MT6895 ||
		priv->data->mmsys_id == MMSYS_MT6886 ||
		priv->data->mmsys_id == MMSYS_MT6835 ||
		priv->data->mmsys_id == MMSYS_MT6855) {
		handle = cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!handle) {
			DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
			return;
		}

		mtk_crtc_exec_atf_prebuilt_instr(mtk_crtc, handle);
		cmdq_pkt_flush(handle);
		cmdq_pkt_destroy(handle);
	}
	if (priv->data->mmsys_id == MMSYS_MT6985)
		cmdq_util_disp_smc_cmd(drm_crtc_index(crtc), DISP_CMD_CRTC_ENABLE);
}
#endif

int mtk_drm_crtc_usage_enable(struct mtk_drm_private *priv,
			unsigned int crtc_id)
{
	unsigned int i, main_disp_idx = 0xFFFFFFFF;
	unsigned int occupied_ovl = 0;

	if (unlikely(crtc_id >= MAX_CRTC)) {
		DDPPR_ERR("%s invalid crtc_id %u\n", __func__, crtc_id);
		return DISP_ENABLE;
	}

	/*  find main display index */
	for (i = 0; i < MAX_CRTC ; ++i) {
		if (priv->pre_defined_bw[i] == 0xFFFFFFFF) {
			main_disp_idx = i;
			continue;
		}

		if (priv->usage[i] == DISP_ENABLE)
			occupied_ovl |= priv->ovl_usage[i];
	}

	/* can't find main display, free run sage state control */
	if (unlikely(main_disp_idx == 0xFFFFFFFF))
		return DISP_ENABLE;

	/* main display could enable imediately */
	if (crtc_id == main_disp_idx) {
		if (priv->ovl_usage[main_disp_idx] != 0 &&
				(priv->ovl_usage[main_disp_idx] & ~occupied_ovl) == 0) {
			DDPPR_ERR("main display ovl drained, occupied_ovl %x\n", occupied_ovl);
			return DISP_OPENING;
		}
		return DISP_ENABLE;
	}

	/* other display could also enable imediately when main display disable */
	if (priv->usage[main_disp_idx] == DISP_DISABLE)
		return DISP_ENABLE;

	/* this crtc OVL usage conflict with main display, pending */
	if ((priv->ovl_usage[main_disp_idx] & priv->ovl_usage[crtc_id]) != 0)
		return DISP_OPENING;

	/* enable non HRT display imediateky */
	if (priv->pre_defined_bw[crtc_id] == 0)
		return DISP_ENABLE;

	return DISP_OPENING;
}

static void mtk_drm_crtc_path_adjust(struct mtk_drm_private *priv, struct drm_crtc *crtc,
							unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	unsigned int i, main_disp_idx = 0xFFFFFFFF;
	unsigned int ovl_res, comp_id_nr, ovl_comp_idx, comp_id, *comp_id_list = NULL;
	unsigned int occupied_ovl = 0;

	if (mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] == 0)
		return;

	/*  find main display index and count occupied_ovl */
	for (i = 0; i < MAX_CRTC ; ++i) {
		if (priv->pre_defined_bw[i] == 0xFFFFFFFF) {
			main_disp_idx = i;
			continue;
		}

		if (priv->usage[i] == DISP_ENABLE)
			occupied_ovl |= priv->ovl_usage[i];
	}

	/* can't find main display, free run sage state control */
	if (unlikely(main_disp_idx == 0xFFFFFFFF))
		return;

	/* only main display need adjust path */
	if (main_disp_idx != drm_crtc_index(crtc))
		return;

	if (priv->ovl_usage[main_disp_idx] == 0)
		return;

	ovl_res = priv->ovl_usage[main_disp_idx] & ~occupied_ovl;

	if (mtk_crtc->ovl_usage_status == ovl_res)
		return;

	comp_id_nr = mtk_ddp_ovl_resource_list(priv, &comp_id_list);

	DDPINFO("need adjust crtc path from %u to %u while resume\n",
			mtk_crtc->ovl_usage_status, ovl_res);

	/* adjust main path */
	comp = mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH]
		[mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] - 1];
	ovl_comp_idx = 0;

	for (i = 0 ; i < comp_id_nr ; ++i) {
		if ((ovl_res & BIT(i)) == 0)
			continue;

		comp_id = comp_id_list[i];
		mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] =
			priv->ddp_comp[comp_id];
		++ovl_comp_idx;
	}
	mtk_crtc->ddp_ctx[ddp_mode].ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] = comp;
	mtk_crtc->ddp_ctx[ddp_mode].ovl_comp_nr[DDP_FIRST_PATH] = ovl_comp_idx + 1;

	mtk_crtc->ovl_usage_status = ovl_res;

	if (mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] == 0)
		return;

	/* adjust dual pipe path */
	comp = mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH]
		[mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] - 1];
	ovl_comp_idx = 0;

	for (i = 0 ; i < comp_id_nr ; ++i) {
		if ((ovl_res & BIT(i)) == 0)
			continue;

		comp_id = dual_pipe_comp_mapping(priv->data->mmsys_id, comp_id_list[i]);
		mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] =
			priv->ddp_comp[comp_id];
		++ovl_comp_idx;
	}
	mtk_crtc->dual_pipe_ddp_ctx.ovl_comp[DDP_FIRST_PATH][ovl_comp_idx] = comp;
	mtk_crtc->dual_pipe_ddp_ctx.ovl_comp_nr[DDP_FIRST_PATH] = ovl_comp_idx + 1;
}

void mtk_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *mtk_state = to_mtk_crtc_state(crtc->state);
	unsigned int crtc_id = drm_crtc_index(crtc);
#ifndef DRM_CMDQ_DISABLE
	struct cmdq_client *client;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
#endif
	struct mtk_ddp_comp *comp;
	int i, j;
	struct mtk_ddp_comp *output_comp = NULL;
	int en = 1;
	bool only_output;
	g_crtc = crtc;
	CRTC_MMP_EVENT_START((int) crtc_id, enable,
			mtk_crtc->enabled, 0);

	if (IS_ERR_OR_NULL(priv)) {
		DDPPR_ERR("%s:%u invalid priv\n", __func__, __LINE__);
		goto end;
	}
	if (mtk_crtc->enabled) {
		CRTC_MMP_MARK(crtc_id, enable, 0, 0);
		DDPINFO("crtc%d skip %s\n", crtc_id, __func__);
		goto end;
	} else if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK(crtc_id, enable, 0, 1);
		DDPINFO("crtc%d skip %s, ddp_mode: NO_USE\n", crtc_id,
			__func__);
		goto end;
	}
	DDPINFO("crtc%d do %s\n", crtc_id, __func__);
	DDP_PROFILE("[PROFILE] %s+\n", __func__);
	CRTC_MMP_MARK(crtc_id, enable, 1, 0);

	/*
	 * for case display does not have multiple display mode,
	 * copy current crtc state's display mode to avail_modes for layering rule
	 */
	if (mtk_crtc->avail_modes_num == 0 && mtk_crtc->avail_modes)
		drm_mode_copy(mtk_crtc->avail_modes, &crtc->state->adjusted_mode);

	only_output = (crtc_id < MAX_CRTC && priv->usage[crtc_id] == DISP_OPENING);
	/* adjust path for ovl switch if necessary */
	mtk_drm_crtc_path_adjust(priv, crtc, mtk_crtc->ddp_mode);

	/*for dual pipe*/
	mtk_crtc_prepare_dual_pipe(mtk_crtc);

	/* for ap res switch */
	mtk_crtc_divide_default_path_by_rsz(mtk_crtc);

	/* attach the crtc to each componet */
	mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE,
				&en);

	/* connector seemless switch's resume frame not update screen */
	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI) {
		struct mtk_dsi *dsi = container_of(output_comp, struct mtk_dsi, ddp_comp);

		mtk_crtc->skip_frame = dsi->pending_switch;
	}

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		/* 1. power on mtcmos & init apsrc*/
		mtk_drm_top_clk_prepare_enable(crtc->dev);
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, true, true,
			MTK_APSRC_CRTC_DEFAULT, false);

		/* 2. prepare modules would be used in this CRTC */
		mtk_crtc_ddp_prepare(mtk_crtc);
	}

	mtk_gce_backup_slot_init(mtk_crtc);

#ifndef DRM_CMDQ_DISABLE
	/* 3. power on cmdq client */
	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	cmdq_mbox_enable(client->chan);
	CRTC_MMP_MARK((int) crtc_id, enable, 1, 1);
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_USE_M4U))
		mtk_crtc_prepare_instr(crtc);
#endif

	/* 4. start trigger loop first to keep gce alive */
	if (mtk_crtc_with_trigger_loop(crtc)) {
		if (mtk_crtc_with_sodi_loop(crtc) &&
			(!mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_sodi_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
				(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);
		mtk_crtc_start_trig_loop(crtc);
	}

	if (mtk_crtc_is_mem_mode(crtc) || mtk_crtc_is_dc_mode(crtc)) {
		struct golden_setting_context *ctx =
					__get_golden_setting_context(mtk_crtc);
		struct cmdq_pkt *cmdq_handle;
		int gce_event =
			get_path_wait_event(mtk_crtc, mtk_crtc->ddp_mode);

		ctx->is_dc = 1;

		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return;
		}

		if (gce_event > 0)
			cmdq_pkt_set_event(cmdq_handle, gce_event);
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}

	CRTC_MMP_MARK((int) crtc_id, enable, 1, 2);

	mutex_lock(&priv->path_ctrl_lock);

	/* 5. connect path */
	mtk_crtc_connect_default_path(mtk_crtc);

	mtk_crtc->qos_ctx->last_hrt_req = 0;

	/* 6. disconnect addon module and config */
	mtk_crtc_connect_addon_module(crtc);

	mutex_unlock(&priv->path_ctrl_lock);

	/* 7. config ddp engine */
	mtk_crtc_config_default_path(mtk_crtc);
	CRTC_MMP_MARK((int) crtc_id, enable, 1, 3);

	/* 8. restore OVL setting */
	if (!only_output)
		mtk_crtc_restore_plane_setting(mtk_crtc);

	/* 9. Set QOS BW */
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
		mtk_ddp_comp_io_cmd(comp, NULL, PMQOS_SET_BW, NULL);

	/* 10. set dirty for cmd mode */
	if (mtk_crtc_is_frame_trigger_mode(crtc) &&
		!mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE] &&
		!mtk_state->doze_changed && !mtk_crtc->skip_frame)
		mtk_crtc_set_dirty(mtk_crtc);

	/* 11. set vblank*/
	drm_crtc_vblank_on(crtc);

	/* 12. enable ESD check */
	if (mtk_drm_lcm_is_connect(mtk_crtc)) {
		if ((mtk_dsi_get_vendor_id() == 2) || (mtk_dsi_get_vendor_id() == 3)) {
			mtk_disp_esd_check_switch(crtc, 0);
		} else {
			mtk_disp_esd_check_switch(crtc, true);
		}
	}
	/* 13. enable fake vsync if need*/
	mtk_drm_fake_vsync_switch(crtc, true);

	/* 14. set CRTC SW status */
	if (!only_output)
		mtk_crtc_set_status(crtc, true);

	/* 15. alloc sram if last is MML */
	if (mtk_crtc->is_mml)
		mtk_crtc_alloc_sram(mtk_crtc, mtk_crtc->mml_ir_sram.bk_hrt_idx);

end:
	CRTC_MMP_EVENT_END((int) crtc_id, enable,
			mtk_crtc->enabled, 0);
	DDP_PROFILE("[PROFILE] %s-\n", __func__);
}

static void mtk_drm_crtc_wk_lock(struct drm_crtc *crtc, bool get,
	const char *func, int line)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	DDPMSG("CRTC%d %s wakelock %s %d\n",
		drm_crtc_index(crtc), (get ? "hold" : "release"),
		func, line);

	if (get)
		__pm_stay_awake(mtk_crtc->wk_lock);
	else
		__pm_relax(mtk_crtc->wk_lock);
}

unsigned int mtk_drm_dump_wk_lock(
	struct mtk_drm_private *priv, char *stringbuf, int buf_len)
{
	unsigned int len = 0;
	int i = 0;
	struct drm_crtc *crtc;
	struct mtk_drm_crtc *mtk_crtc;

	len += scnprintf(stringbuf + len, buf_len - len,
		 "==========    wakelock Info    ==========\n");

	for (i = 0; i < MAX_CRTC ; i++) {
		crtc = priv->crtc[i];
		if (!crtc)
			continue;
		mtk_crtc = to_mtk_crtc(crtc);

		len += scnprintf(stringbuf + len, buf_len - len,
			 "CRTC%d wk active:%d;  ", i,
			 mtk_crtc->wk_lock->active);
	}

	len += scnprintf(stringbuf + len, buf_len - len, "\n\n");

	return len;
}

static int mtk_drm_crtc_update_ddp_mode(
	struct mtk_drm_crtc *mtk_crtc, enum mtk_ddp_comp_id id)
{
	int ddp_mode = DDP_MAJOR;
	int i, j;
	struct mtk_ddp_comp *comp;

	for (; ddp_mode < DDP_MODE_NR; ddp_mode++) {
		for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j, ddp_mode) {
			if (comp->id == id)
				return ddp_mode;
		}
	}
	return DDP_MAJOR;
}

static void mtk_drm_crtc_fix_conn_mode(struct drm_crtc *crtc, struct drm_display_mode *timing,
			struct mtk_ddp_comp *output_comp)
{
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_display_mode *mode;
	struct mtk_crtc_state *mtk_state;

	mtk_crtc = to_mtk_crtc(crtc);

	if (!crtc->state) {
		DDPPR_ERR("%s invalid CRTC state\n", __func__);
		return;
	}

	mtk_state = to_mtk_crtc_state(crtc->state);

	crtc->mode.hdisplay = timing->hdisplay;
	crtc->mode.vdisplay = timing->vdisplay;
	crtc->state->adjusted_mode.clock	= timing->clock;
	crtc->state->adjusted_mode.hdisplay	= timing->hdisplay;
	crtc->state->adjusted_mode.hsync_start	= timing->hsync_start;
	crtc->state->adjusted_mode.hsync_end	= timing->hsync_end;
	crtc->state->adjusted_mode.htotal	= timing->htotal;
	crtc->state->adjusted_mode.hskew	= timing->hskew;
	crtc->state->adjusted_mode.vdisplay	= timing->vdisplay;
	crtc->state->adjusted_mode.vsync_start	= timing->vsync_start;
	crtc->state->adjusted_mode.vsync_end	= timing->vsync_end;
	crtc->state->adjusted_mode.vtotal	= timing->vtotal;
	crtc->state->adjusted_mode.vscan	= timing->vscan;
	vfree(mtk_crtc->avail_modes);
	mtk_crtc->avail_modes = NULL;
	mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_SET_CRTC_AVAIL_MODES, mtk_crtc);

	/* Update mode & adjusted_mode in CRTC */
	mode = mtk_drm_crtc_avail_disp_mode(crtc,
		mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
	if (IS_ERR_OR_NULL(mode)) {
		DDPPR_ERR("%s crtc%u invalid disp_mode %u\n", __func__,
			drm_crtc_index(crtc), mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX]);
		return;
	}

	copy_drm_disp_mode(mode, &crtc->state->mode);
	crtc->state->mode.hskew = mode->hskew;
	drm_mode_set_crtcinfo(&crtc->state->mode, 0);

	copy_drm_disp_mode(mode, &crtc->state->adjusted_mode);
	crtc->state->adjusted_mode.hskew = mode->hskew;
	drm_mode_set_crtcinfo(&crtc->state->adjusted_mode, 0);

	/* update DAL setting */
	drm_update_dal(crtc, NULL);
}

extern bool g_dsi_switched;
static void mtk_drm_crtc_update_interface(struct drm_crtc *crtc,
	struct drm_atomic_state *state)
{
	int i;
	struct mtk_drm_crtc *mtk_crtc;
	struct drm_connector *connector;
	struct drm_connector_state *new_conn_state;
	struct mtk_ddp_comp *output_comp = NULL;
	enum mtk_ddp_comp_id comp_id = 0;
	struct drm_display_mode *timing = NULL;
	struct drm_crtc_state *crtc_state;
	struct mtk_crtc_state *mtk_crtc_state;
	unsigned int crtc_index;

	mtk_crtc = to_mtk_crtc(crtc);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	for_each_new_connector_in_state(state, connector, new_conn_state, i) {
		if (new_conn_state && new_conn_state->crtc == crtc) {
			if (connector->connector_type == DRM_MODE_CONNECTOR_DSI &&
				output_comp &&
				mtk_dsi_get_comp_id(connector) != output_comp->id) {
				g_dsi_switched = true;
				/*output component is changed*/
				comp_id = mtk_dsi_get_comp_id(connector);
				mtk_crtc->ddp_mode =
					mtk_drm_crtc_update_ddp_mode(mtk_crtc, comp_id);

				DDPMSG("%s ddp mode is %d, comp id is %d\n",
					__func__, mtk_crtc->ddp_mode, comp_id);

				mtk_crtc_update_gce_event(mtk_crtc);

				crtc_state = drm_atomic_get_crtc_state(state, crtc);
				mtk_crtc_state = (crtc_state) ?
						to_mtk_crtc_state(crtc_state) : NULL;
				if (mtk_crtc_state) {
					crtc_index = drm_crtc_index(crtc);
					mtk_crtc_state->lye_state.scn[crtc_index] = NONE;
				}
				/*update mtk_crtc->panel_ext*/
				output_comp = mtk_ddp_comp_request_output(mtk_crtc);
				if (!output_comp)
					continue;
				mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_PANEL_EXT,
						    &mtk_crtc->panel_ext);
				mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_GET_TIMING, &timing);

				mtk_drm_crtc_fix_conn_mode(crtc, timing, output_comp);
			}
		}
	}
	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI &&
			timing == NULL) {
		mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_GET_TIMING, &timing);
		if (timing == NULL) {
			DDPPR_ERR("%s %d fail to get default timing\n",
				__func__, __LINE__);
			return;
		}

		if (crtc->mode.hdisplay != timing->hdisplay ||
				crtc->mode.vdisplay != timing->vdisplay) {
			DDPMSG("crtc mode different from connector state, change mode\n");
			mtk_drm_crtc_fix_conn_mode(crtc, timing, output_comp);
		}
	}
}

void mtk_drm_crtc_atomic_resume(struct drm_crtc *crtc,
				struct drm_atomic_state *atomic_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int index = drm_crtc_index(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc0 = to_mtk_crtc(priv->crtc[0]);
	struct mtk_ddp_comp *output_comp;

	/* When open VDS path switch feature, After VDS created,
	 * VDS will call setcrtc, So atomic commit will be called,
	 * but OVL0_2L is in use by main disp, So we need to skip
	 * this action.
	 */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_VDS_PATH_SWITCH) && (index == 2)) {
		if (atomic_read(&mtk_crtc0->already_config) &&
			(!priv->vds_path_switch_done)) {
			DDPMSG("Switch vds: VDS need skip first crtc enable\n");
			return;
		} else if (!atomic_read(&mtk_crtc0->already_config)) {
			DDPMSG("Switch vds: VDS no need skip as crtc0 disable\n");
			priv->vds_path_enable = 1;
		}
	}

	CRTC_MMP_EVENT_START((int) index, resume,
			mtk_crtc->enabled, index);

	/* hold wakelock */
	mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);

	if (mtk_crtc->path_data->is_discrete_path)
		mtk_crtc->skip_frame = true;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT) && index < MAX_CRTC) {
		if (priv->usage[index] == DISP_ENABLE || priv->usage[index] == DISP_OPENING) {
			DDPPR_ERR("%s usage control exception crtc%d cur usage %u\n",
				__func__, index, priv->usage[index]);
			return;
		}

		priv->usage[index] = mtk_drm_crtc_usage_enable(priv, index);

		if (priv->usage[index] == DISP_OPENING) {
			DDPINFO("%s %d wait for opening\n", __func__, index);
			if (!(output_comp &&
					mtk_ddp_comp_get_type(output_comp->id) == MTK_DISP_WDMA))
				return;
		}
	}

	/*update interface when connector is changed*/
	if (of_property_read_bool(priv->mmsys_dev->of_node, "enable-output-int-switch"))
		mtk_drm_crtc_update_interface(crtc, atomic_state);

	mtk_drm_crtc_enable(crtc);
	CRTC_MMP_EVENT_END((int) index, resume,
			mtk_crtc->enabled, 0);
}

bool mtk_crtc_with_sub_path(struct drm_crtc *crtc, unsigned int ddp_mode);

void mtk_crtc_config_round_corner(struct drm_crtc *crtc,
				  struct cmdq_pkt *handle)
{

	struct mtk_ddp_config cfg;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i, j;
	int cur_path_idx;

//	cfg.w = crtc->mode.hdisplay;
//	cfg.h = crtc->mode.vdisplay;

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		cur_path_idx = DDP_SECOND_PATH;
	else
		cur_path_idx = DDP_FIRST_PATH;

	mtk_crtc_wait_frame_done(mtk_crtc,
		handle, cur_path_idx, 0);

	for_each_comp_in_cur_crtc_path(
		comp, mtk_crtc, i, j)
		if (comp->id == DDP_COMPONENT_POSTMASK0 ||
			comp->id == DDP_COMPONENT_POSTMASK1) {
			cfg.w = mtk_crtc_get_width_by_comp(__func__, crtc, comp, false);
			cfg.h = mtk_crtc_get_height_by_comp(__func__, crtc, comp, false);
			mtk_ddp_comp_config(comp, &cfg, handle);
			break;
		}

	if (!mtk_crtc->is_dual_pipe)
		return;

	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		if (comp->id == DDP_COMPONENT_POSTMASK0 ||
			comp->id == DDP_COMPONENT_POSTMASK1) {
			cfg.w = mtk_crtc_get_width_by_comp(__func__, crtc, comp, false);
			cfg.h = mtk_crtc_get_height_by_comp(__func__, crtc, comp, false);
			mtk_ddp_comp_config(comp, &cfg, handle);
			break;
		}
	}
}

static inline struct mtk_drm_gem_obj *
__load_rc_pattern(struct drm_crtc *crtc, size_t size, void *addr)
{
	struct mtk_drm_gem_obj *gem;

	if (!size || !addr) {
		DDPPR_ERR("%s invalid round_corner size or addr\n",
			__func__);
		return NULL;
	}

	gem = mtk_drm_gem_create(
		crtc->dev, size, true);

	if (!gem) {
		DDPPR_ERR("%s gem create fail\n", __func__);
		return NULL;
	}

	memcpy(gem->kvaddr, addr,
	       size);

	return gem;
}

void __load_rc_memory_free(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (mtk_crtc->round_corner_gem) {
		mtk_drm_gem_free_object(&mtk_crtc->round_corner_gem->base);
		mtk_crtc->round_corner_gem = NULL;
	}

	if (mtk_crtc->round_corner_gem_l) {
		mtk_drm_gem_free_object(&mtk_crtc->round_corner_gem_l->base);
		mtk_crtc->round_corner_gem_l = NULL;
	}

	if (mtk_crtc->round_corner_gem_r) {
		mtk_drm_gem_free_object(&mtk_crtc->round_corner_gem_r->base);
		mtk_crtc->round_corner_gem_r = NULL;
	}
}

void mtk_crtc_load_round_corner_pattern(struct drm_crtc *crtc,
					struct cmdq_pkt *handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_panel_params *panel_ext = mtk_drm_get_lcm_ext_params(crtc);

	if (panel_ext && panel_ext->round_corner_en) {
		__load_rc_memory_free(crtc);

		mtk_crtc->round_corner_gem =
			__load_rc_pattern(crtc, panel_ext->corner_pattern_tp_size,
						panel_ext->corner_pattern_lt_addr);

		mtk_crtc->round_corner_gem_l =
			__load_rc_pattern(crtc, panel_ext->corner_pattern_tp_size_l,
						panel_ext->corner_pattern_lt_addr_l);
		mtk_crtc->round_corner_gem_r =
			__load_rc_pattern(crtc, panel_ext->corner_pattern_tp_size_r,
						panel_ext->corner_pattern_lt_addr_r);

	}
}

struct drm_display_mode *mtk_drm_crtc_avail_disp_mode(struct drm_crtc *crtc,
	unsigned int idx)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (IS_ERR_OR_NULL(mtk_crtc)) {
		DDPPR_ERR("%s:%u invalid mtk_crtc\n", __func__, __LINE__);
		return NULL;
	}

	/* For not multiple display mode's display, would use first display_mode since resume */
	if (mtk_crtc->avail_modes_num == 0 && mtk_crtc->avail_modes)
		return mtk_crtc->avail_modes;

	/* If not crtc0 and avail_modes is NULL, use crtc0 instead. */

	if (drm_crtc_index(crtc) != 0 && (mtk_crtc->avail_modes == NULL)) {
		struct drm_crtc *crtc0;

		DDPPR_ERR("%s no support CRTC%u", __func__,
			drm_crtc_index(crtc));

		drm_for_each_crtc(crtc0, crtc->dev) {
			if (drm_crtc_index(crtc0) == 0)
				break;
		}

		mtk_crtc = to_mtk_crtc(crtc0);
	}

	if (idx >= mtk_crtc->avail_modes_num) {
		DDPPR_ERR("%s idx:%u exceed avail_num:%u", __func__,
			idx, mtk_crtc->avail_modes_num);
		idx = 0;
	}

	if (mtk_crtc->avail_modes == NULL)
		return NULL;

	return &mtk_crtc->avail_modes[idx];
}

int mtk_drm_crtc_get_panel_original_size(struct drm_crtc *crtc, unsigned int *width,
	unsigned int *height)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int ret = 0;

	if (mtk_crtc->avail_modes_num > 0) {
		int i;

		for (i = 0; i < mtk_crtc->avail_modes_num; i++) {
			struct drm_display_mode *mode = &mtk_crtc->avail_modes[i];

			if (mode->hdisplay > *width)
				*width = mode->hdisplay;

			if (mode->vdisplay > *height)
				*height = mode->vdisplay;
		}
	} else {
		ret = -EINVAL;
		DDPMSG("invalid display mode\n");
	}

	DDPMSG("panel original size:%dx%d\n", *width, *height);

	return ret;
}

void mtk_drm_crtc_init_para(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct mtk_ddp_comp *comp;
	struct drm_display_mode *timing = NULL;
	struct mtk_drm_private *priv =
			mtk_crtc->base.dev->dev_private;
	int en = 1;
	unsigned int invoke_fps, init_idle_timeout = 50;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (comp == NULL)
		return;

	mtk_ddp_comp_io_cmd(comp, NULL, DSI_FILL_MODE_BY_CONNETOR, NULL);
	mtk_ddp_comp_io_cmd(comp, NULL, DSI_GET_TIMING, &timing);
	if (timing == NULL) {
		DDPMSG("%s, %d, failed to get default timing\n", __func__, __LINE__);
		mtk_crtc->avail_modes_num = 0;
		mtk_crtc->avail_modes = vzalloc(sizeof(struct drm_display_mode));
		return;
	}

	crtc->mode.hdisplay = timing->hdisplay;
	crtc->mode.vdisplay = timing->vdisplay;
	crtc->state->adjusted_mode.clock        = timing->clock;
	crtc->state->adjusted_mode.hdisplay     = timing->hdisplay;
	crtc->state->adjusted_mode.hsync_start  = timing->hsync_start;
	crtc->state->adjusted_mode.hsync_end    = timing->hsync_end;
	crtc->state->adjusted_mode.htotal       = timing->htotal;
	crtc->state->adjusted_mode.hskew        = timing->hskew;
	crtc->state->adjusted_mode.vdisplay     = timing->vdisplay;
	crtc->state->adjusted_mode.vsync_start  = timing->vsync_start;
	crtc->state->adjusted_mode.vsync_end    = timing->vsync_end;
	crtc->state->adjusted_mode.vtotal       = timing->vtotal;
	crtc->state->adjusted_mode.vscan        = timing->vscan;

	invoke_fps = drm_mode_vrefresh(timing);

	drm_invoke_fps_chg_callbacks(invoke_fps);
	mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);

	if (invoke_fps > 0)
		init_idle_timeout = mtk_crtc_get_idle_interval(crtc, invoke_fps);
	if (init_idle_timeout > 0)
		mtk_drm_set_idle_check_interval(crtc, init_idle_timeout);

	/* backup display context */
	if (crtc_id == 0) {
		pgc->mode = *timing;
		DDPMSG("width:%d, height:%d\n", pgc->mode.hdisplay,
			pgc->mode.vdisplay);
	}

	/* store display mode for crtc0 only */
	if (comp && mtk_ddp_comp_get_type(comp->id) == MTK_DSI) {
		mtk_ddp_comp_io_cmd(comp, NULL,
			DSI_SET_CRTC_AVAIL_MODES, mtk_crtc);

		/* 0:no use, 1:on ddic, 2:on ap */
		DDPMSG("%s resolution switch type: %d\n", __func__, mtk_crtc->res_switch);

		if (mtk_crtc->res_switch == RES_SWITCH_ON_AP)
			mtk_drm_crtc_get_panel_original_size(crtc,
				&mtk_crtc->scaling_ctx.lcm_width,
				&mtk_crtc->scaling_ctx.lcm_height);

		/* fill connector prop caps for hwc */
		mtk_ddp_comp_io_cmd(comp, NULL, DSI_FILL_CONNECTOR_PROP_CAPS, mtk_crtc);
		mtk_ddp_comp_io_cmd(comp, NULL, SET_MMCLK_BY_DATARATE, &en);
		/*need enable hrt_bw for pan display*/
		if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_MMQOS_SUPPORT))
			mtk_drm_pan_disp_set_hrt_bw(crtc, __func__);
	} else {
		mtk_crtc->avail_modes_num = 0;
		mtk_crtc->avail_modes = vzalloc(sizeof(struct drm_display_mode));
	}

	mtk_crtc->mml_cfg = NULL;
	mtk_crtc->mml_cfg_pq = NULL;
}

void mtk_crtc_first_enable_ddp_config(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_ddp_comp *comp;
	struct mtk_ddp_config cfg = {0};
	int i, j;
	struct mtk_ddp_comp *output_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	cfg.w = crtc->mode.hdisplay;
	cfg.h = crtc->mode.vdisplay;
	cfg.bpc = mtk_crtc->bpc;
	if (output_comp && drm_crtc_index(crtc) == 0) {
		cfg.w = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_WIDTH, NULL);
		cfg.h = mtk_ddp_comp_io_cmd(output_comp, NULL,
					DSI_GET_VIRTUAL_HEIGH, NULL);
	}
	cfg.p_golden_setting_context =
			__get_golden_setting_context(mtk_crtc);

#ifndef DRM_CMDQ_DISABLE
	if (priv->data->mmsys_id == MMSYS_MT6983 ||
		priv->data->mmsys_id == MMSYS_MT6985 ||
		priv->data->mmsys_id == MMSYS_MT6879 ||
		priv->data->mmsys_id == MMSYS_MT6895 ||
		priv->data->mmsys_id == MMSYS_MT6835 ||
		priv->data->mmsys_id == MMSYS_MT6855 ||
		priv->data->mmsys_id == MMSYS_MT6886) {
		/*Set EVENT_GCED_EN EVENT_GCEM_EN*/
		writel(0x3, mtk_crtc->config_regs +
				DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

		/*Set BYPASS_MUX_SHADOW*/
		writel(0x1, mtk_crtc->config_regs +
				DISP_REG_CONFIG_BYPASS_MUX_SHADOW);

		if (mtk_crtc->side_config_regs) {
			writel(0x3, mtk_crtc->side_config_regs +
					DISP_REG_CONFIG_MMSYS_GCE_EVENT_SEL);

			/*Set BYPASS_MUX_SHADOW*/
			writel(0x1, mtk_crtc->side_config_regs +
					DISP_REG_CONFIG_BYPASS_MUX_SHADOW);
		}

		if (priv->data->mmsys_id == MMSYS_MT6985) {
			/*Set EVENT_GCED_EN EVENT_GCEM_EN <OVLSYS>*/
			writel(0x3, mtk_crtc->ovlsys0_regs +
					DISP_REG_CONFIG_OVLSYS_GCE_EVENT_SEL);

			/*Set BYPASS_MUX_SHADOW*/
			writel(0x1, mtk_crtc->ovlsys0_regs +
					DISP_REG_CONFIG_BYPASS_MUX_SHADOW);

			if (mtk_crtc->ovlsys1_regs) {
				writel(0x3, mtk_crtc->ovlsys1_regs +
						DISP_REG_CONFIG_OVLSYS_GCE_EVENT_SEL);

				/*Set BYPASS_MUX_SHADOW*/
				writel(0x1, mtk_crtc->ovlsys1_regs +
						DISP_REG_CONFIG_BYPASS_MUX_SHADOW);
			}
		}
	}
#endif

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	cmdq_pkt_clear_event(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
	cmdq_pkt_clear_event(cmdq_handle,
			     mtk_crtc->gce_obj.event[EVENT_VDO_EOF]);
	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);

	/*1. Show LK logo only */
	mtk_crtc_all_layer_off(mtk_crtc, cmdq_handle);

	/*2. Load Round Corner */
	mtk_crtc_load_round_corner_pattern(&mtk_crtc->base, cmdq_handle);
	mtk_crtc_config_round_corner(crtc, cmdq_handle);

	/*3. Enable M4U port and replace OVL address to mva */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_USE_M4U)) {
		DDPINFO("%s crtc%u, enable iommu\n", __func__, drm_crtc_index(crtc));
		mtk_crtc_enable_iommu_runtime(mtk_crtc, cmdq_handle);
	}

	if (mtk_crtc->is_dual_pipe &&
		mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_TILE_OVERHEAD)) {
		cfg.tile_overhead.is_support = true;
		cfg.tile_overhead.left_in_width = cfg.w / 2;
		cfg.tile_overhead.right_in_width = cfg.w / 2;
	} else {
		cfg.tile_overhead.is_support = false;
		cfg.tile_overhead.left_in_width = cfg.w;
		cfg.tile_overhead.right_in_width = cfg.w;
	}

	/*4. Calculate total overhead */
	for_each_comp_in_crtc_path_reverse(comp, mtk_crtc, i, j) {
		mtk_ddp_comp_config_overhead(comp, &cfg);
		DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
			__func__, mtk_dump_comp_str(comp),
			cfg.tile_overhead.left_in_width,
			cfg.tile_overhead.right_in_width,
			cfg.tile_overhead.left_overhead,
			cfg.tile_overhead.right_overhead);
	}

	/*5. Enable Frame done IRQ &  process first config */
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		mtk_ddp_comp_first_cfg(comp, &cfg, cmdq_handle);
		mtk_ddp_comp_io_cmd(comp, cmdq_handle, IRQ_LEVEL_NORMAL, NULL);
		mtk_ddp_comp_io_cmd(comp, cmdq_handle, DSI_SET_TARGET_LINE, &cfg);
		mtk_ddp_comp_io_cmd(comp, cmdq_handle,
			MTK_IO_CMD_RDMA_GOLDEN_SETTING, &cfg);
		if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_OVL)
			cfg.source_bpc = mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				OVL_GET_SOURCE_BPC, NULL);
	}
	if (mtk_crtc->is_dual_pipe) {
		DDPFUNC();

		/*Calculate total overhead */
		for_each_comp_in_dual_pipe_reverse(comp, mtk_crtc, i, j) {
			mtk_ddp_comp_config_overhead(comp, &cfg);
			DDPINFO("%s:comp %s width L:%d R:%d, overhead L:%d R:%d\n",
				__func__, mtk_dump_comp_str(comp),
				cfg.tile_overhead.left_in_width,
				cfg.tile_overhead.right_in_width,
				cfg.tile_overhead.left_overhead,
				cfg.tile_overhead.right_overhead);
		}

		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			mtk_ddp_comp_first_cfg(comp, &cfg, cmdq_handle);
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				IRQ_LEVEL_NORMAL, NULL);
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				MTK_IO_CMD_RDMA_GOLDEN_SETTING, &cfg);
		}
	}

	/*store total overhead data*/
	mtk_crtc_store_total_overhead(mtk_crtc, cfg.tile_overhead);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		mtk_crtc_set_dirty(mtk_crtc);
}

void mtk_drm_crtc_first_enable(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	mtk_drm_crtc_init_para(crtc);

	if (mtk_crtc->enabled) {
		DDPINFO("crtc%d skip %s\n", crtc_id, __func__);
		return;
	}
	DDPINFO("crtc%d do %s\n", crtc_id, __func__);
#ifndef DRM_CMDQ_DISABLE
	cmdq_mbox_enable(mtk_crtc->gce_obj.client[CLIENT_CFG]->chan);
#endif

	/* 1. hold wakelock */
	mtk_drm_crtc_wk_lock(crtc, 1, __func__, __LINE__);

	/*for dual pipe*/
	mtk_crtc_prepare_dual_pipe(mtk_crtc);

	/* for ap res switch */
	mtk_crtc_divide_default_path_by_rsz(mtk_crtc);

	/* 2. start trigger loop first to keep gce alive */
	if (mtk_crtc_with_trigger_loop(crtc)) {
		if (mtk_crtc_with_sodi_loop(crtc) &&
			(!mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_sodi_loop(crtc);

		if (mtk_crtc_with_event_loop(crtc) &&
			(mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_start_event_loop(crtc);

		mtk_crtc_start_trig_loop(crtc);
	}

	/* 3. Regsister configuration */
	mtk_crtc_first_enable_ddp_config(mtk_crtc);

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		/* 4. power on mtcmos & init apsrc*/
		mtk_drm_top_clk_prepare_enable(crtc->dev);
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, true, false,
			MTK_APSRC_CRTC_DEFAULT, false);

		/* 5. prepare modules would be used in this CRTC */
		mtk_crtc_ddp_prepare(mtk_crtc);

		/* 6. sodi config */
		if (priv->data->sodi_config) {
			struct mtk_ddp_comp *comp;
			int i, j;
			bool en = 1;

			for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j)
				priv->data->sodi_config(crtc->dev, comp->id, NULL, &en);
		}
	}

	mtk_gce_backup_slot_init(mtk_crtc);

	/* 7. set vblank*/
	drm_crtc_vblank_on(crtc);

	/* 8. enable fake vsync if necessary */
	mtk_drm_fake_vsync_switch(crtc, true);

	/* 9. set CRTC SW status */
	mtk_crtc_set_status(crtc, true);

	/* move power off mtcmos to kms init flow for multiple display in LK */
}

void mtk_drm_crtc_disable(struct drm_crtc *crtc, bool need_wait)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = (crtc && crtc->dev) ?
			crtc->dev->dev_private : NULL;
	unsigned int crtc_id = (crtc) ? drm_crtc_index(crtc) : 0;
	struct mtk_crtc_state *mtk_state;
	struct mtk_ddp_comp *comp = NULL;
	struct mtk_ddp_comp *output_comp = NULL;
#ifndef DRM_CMDQ_DISABLE
	struct cmdq_client *client;
#endif
	int en = 0;
	g_crtc = crtc;
	if (IS_ERR_OR_NULL(crtc)) {
		DDPPR_ERR("%s:%u invalid crtc\n", __func__, __LINE__);
		goto end;
	}

	if (IS_ERR_OR_NULL(priv)) {
		DDPPR_ERR("%s:%u invalid priv\n", __func__, __LINE__);
		goto end;
	}

	CRTC_MMP_EVENT_START((int) crtc_id, disable,
			mtk_crtc->enabled, 0);

	if (mtk_crtc->qos_ctx)
		mtk_crtc->qos_ctx->last_mmclk_req_idx += 1;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	mtk_state = (aod_scp_flag && crtc) ? to_mtk_crtc_state(crtc->state) : NULL;

	if ((output_comp) &&
		!((aod_scp_flag) && (mtk_state) && (!crtc->state->active)
			&& (mtk_state->prop_val[CRTC_PROP_DOZE_ACTIVE])))
		mtk_ddp_comp_io_cmd(output_comp, NULL, SET_MMCLK_BY_DATARATE,
				&en);
	if (crtc_id < MAX_CRTC && priv->usage[crtc_id] == DISP_OPENING &&
			output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DISP_WDMA) {
		;/* no goto end when display WDMA output in OPENING state */
	} else if (!mtk_crtc->enabled) {
		CRTC_MMP_MARK((int) crtc_id, disable, 0, 0);
		DDPINFO("crtc%d skip %s\n", crtc_id, __func__);
		goto end;
	} else if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK((int) crtc_id, disable, 0, 1);
		DDPINFO("crtc%d skip %s, ddp_mode: NO_USE\n", crtc_id,
			__func__);
		goto end;
	}
	DDPINFO("%s:%d crtc%d+\n", __func__, __LINE__, crtc_id);
	CRTC_MMP_MARK((int) crtc_id, disable, 1, 0);

	/* 1. kick idle */
	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	/* 2. disable fake vsync if need */
	mtk_drm_fake_vsync_switch(crtc, false);

	/* 3. disable ESD check */
	if (mtk_drm_lcm_is_connect(mtk_crtc))
		mtk_disp_esd_check_switch(crtc, false);

	/* 4. stop CRTC */
	mtk_crtc_stop(mtk_crtc, need_wait);
	CRTC_MMP_MARK((int) crtc_id, disable, 1, 1);

	mutex_lock(&priv->path_ctrl_lock);

	/* 5. disconnect addon module and recover config */
	mtk_crtc_disconnect_addon_module(crtc);

	/* 6. disconnect path */
	mtk_crtc_disconnect_default_path(mtk_crtc);

	mutex_unlock(&priv->path_ctrl_lock);

	/* 7. disable vblank */
	drm_crtc_vblank_off(crtc);

#ifndef DRM_CMDQ_DISABLE
	/* 8. power off cmdq client */
	client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	cmdq_mbox_disable(client->chan);
	CRTC_MMP_MARK((int) crtc_id, disable, 1, 2);
#endif

	if (disp_helper_get_stage() == DISP_HELPER_STAGE_NORMAL) {
		/* 9. power off all modules in this CRTC */
		mtk_crtc_ddp_unprepare(mtk_crtc);

		/*APSRC control, turn off*/
		mtk_crtc_v_idle_apsrc_control(crtc, NULL, false, false, crtc_id, false);

		/* 10. power off MTCMOS*/
		/* TODO: need to check how to unprepare MTCMOS */
		mtk_drm_top_clk_disable_unprepare(crtc->dev);
	}

	/* Workaround: if CRTC2, reset wdma->fb to NULL to prevent CRTC2
	 * config wdma and cause KE
	 */
	if (crtc_id == 2) {
		comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_WDMA0);
		if (!comp)
			comp = mtk_ddp_comp_find_by_id(crtc,
					DDP_COMPONENT_WDMA1);
		if (!comp)
			comp = mtk_ddp_comp_find_by_id(crtc,
					DDP_COMPONENT_OVLSYS_WDMA2);
		if (comp)
			comp->fb = NULL;
	}

	if ((crtc_id == 0) && priv && priv->mml_ctx && mml_drm_ctx_idle(priv->mml_ctx)) {
		mml_drm_put_context(priv->mml_ctx);
		priv->mml_ctx = NULL;
	}

	/* 11. set CRTC SW status */
	mtk_crtc_set_status(crtc, false);

end:
	CRTC_MMP_EVENT_END((int) crtc_id, disable,
			mtk_crtc->enabled, 0);
	DDPINFO("%s:%d -\n", __func__, __LINE__);
}

#ifdef MTK_DRM_FENCE_SUPPORT
static void mtk_drm_crtc_release_fence(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	unsigned int id = drm_crtc_index(crtc), i;
	int session_id = -1;

	for (i = 0; i < MAX_SESSION_COUNT; i++) {
		if (id + 1 == MTK_SESSION_TYPE(priv->session_id[i])) {
			session_id = priv->session_id[i];
			break;
		}
	}

	if (session_id == -1) {
		DDPPR_ERR("%s no session for CRTC%u\n", __func__, id);
		return;
	}

	/* release input layer fence */
	DDPMSG("CRTC%u release input fence\n", id);
	for (i = 0; i < MTK_TIMELINE_OUTPUT_TIMELINE_ID; i++)
		mtk_release_layer_fence(session_id, i);

	/* release output fence for crtc2 */
	if (id == 2) {
		DDPMSG("CRTC%u release output fence\n", id);
		mtk_release_layer_fence(session_id,
					MTK_TIMELINE_OUTPUT_TIMELINE_ID);
	}

	/* release present fence */
	if (MTK_SESSION_TYPE(session_id) == MTK_SESSION_PRIMARY ||
			MTK_SESSION_TYPE(session_id) == MTK_SESSION_EXTERNAL ||
			MTK_SESSION_TYPE(session_id) == MTK_SESSION_SP) {
		mtk_drm_suspend_release_present_fence(crtc->dev->dev, id);
		/* TODO: sf present fence is obsolete, should remove corresponding code */
		/* mtk_drm_suspend_release_sf_present_fence(crtc->dev->dev, id); */
	}
}
#endif

void mtk_drm_crtc_suspend(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int index = drm_crtc_index(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	//Temp workaround for MT6855 suspend/resume issue
	switch (priv->data->mmsys_id) {
	case MMSYS_MT6855:
		DDPMSG("%s force return\n", __func__);
		return;
	default:
		break;
	}

	CRTC_MMP_EVENT_START(index, suspend,
			mtk_crtc->enabled, 0);

	mtk_drm_crtc_wait_blank(mtk_crtc);

	/* disable engine secure state */
	if (index == 2 && mtk_crtc->sec_on) {
		mtk_crtc_disable_secure_state(crtc);
		mtk_crtc->sec_on = false;
	}

	mtk_drm_crtc_disable(crtc, true);

	mtk_crtc_disable_plane_setting(mtk_crtc);

/* release all fence */
#ifdef MTK_DRM_FENCE_SUPPORT
	mtk_drm_crtc_release_fence(crtc);
	mtk_crtc_release_lye_idx(crtc);
#endif
	atomic_set(&mtk_crtc->already_config, 0);

	if (index >= 0 && index < MAX_CRTC)
		priv->usage[index] = DISP_DISABLE;

	/* release wakelock */
	mtk_drm_crtc_wk_lock(crtc, 0, __func__, __LINE__);

	CRTC_MMP_EVENT_END(index, suspend,
			mtk_crtc->enabled, 0);
}

int mtk_crtc_check_out_sec(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	int out_sec = 0;

	if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
		/* Output buffer configuration for virtual display */

		out_sec = mtk_drm_fb_is_secure(
				mtk_drm_framebuffer_lookup(crtc->dev,
				state->prop_val[CRTC_PROP_OUTPUT_FB_ID]));

		DDPINFO("%s lookup wb fb:%u sec:%d\n", __func__,
			state->prop_val[CRTC_PROP_OUTPUT_FB_ID], out_sec);
	}

	return out_sec;
}

void mtk_crtc_disable_secure_state(struct drm_crtc *crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = NULL;
	u32 idx = drm_crtc_index(crtc);

	comp = mtk_ddp_comp_request_output(mtk_crtc);

	DDPINFO("%s+ crtc%d\n", __func__, drm_crtc_index(crtc));
	mtk_crtc_pkt_create(&cmdq_handle, crtc,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);

	if (idx == 2)
		mtk_ddp_comp_io_cmd(comp, cmdq_handle, IRQ_LEVEL_NORMAL, NULL);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
	DDPINFO("%s-\n", __func__);
}

void mml_cmdq_pkt_init(struct drm_crtc *crtc, struct cmdq_pkt *cmdq_handle)
{
	u8 i = 0;
	struct mtk_drm_crtc *mtk_crtc = NULL;
	struct mml_drm_ctx *mml_ctx = NULL;
	struct mtk_drm_private *priv = NULL;
	struct mtk_ddp_comp *comp = NULL;
	const enum mtk_ddp_comp_id id[] = {DDP_COMPONENT_INLINE_ROTATE0,
					   DDP_COMPONENT_INLINE_ROTATE1};

	if (!crtc || !cmdq_handle)
		return;

	mml_ctx = mtk_drm_get_mml_drm_ctx(crtc->dev, crtc);
	if (!mml_ctx)
		return;
	mtk_crtc = to_mtk_crtc(crtc);
	priv = crtc->dev->dev_private;

	switch (mtk_crtc->mml_ir_state) {
	case MML_IR_ENTERING:
		for (; i <= mtk_crtc->is_dual_pipe; ++i) {
			comp = priv->ddp_comp[id[i]];
			comp->mtk_crtc = mtk_crtc;
			mtk_ddp_comp_reset(comp, cmdq_handle);
			mtk_ddp_comp_addon_config(comp, 0, 0, NULL, cmdq_handle);
			mtk_disp_mutex_add_comp_with_cmdq(mtk_crtc, id[i], false, cmdq_handle, 0);
		}
	case MML_IR_RACING:
		mml_drm_racing_config_sync(mml_ctx, cmdq_handle);
		break;
	case MML_IR_LEAVING:
		mtk_crtc_mml_racing_stop_sync(crtc, cmdq_handle, false);
		break;
	default:
		break;
	}
}

struct cmdq_pkt *mtk_crtc_gce_commit_begin(struct drm_crtc *crtc,
						struct drm_crtc_state *old_crtc_state,
						struct mtk_crtc_state *crtc_state,
						bool need_sync_mml)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	/*Msync 2.0*/
	unsigned long crtc_id = (unsigned long)drm_crtc_index(crtc);
	struct mtk_crtc_state *old_mtk_state = NULL;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_panel_params *params =
			mtk_drm_get_lcm_ext_params(crtc);
	struct mtk_ddp_comp *comp = NULL;

	if (mtk_crtc_is_dc_mode(crtc) || mtk_crtc->is_mml)
		mtk_crtc_pkt_create(&cmdq_handle, crtc,
			mtk_crtc->gce_obj.client[CLIENT_SUB_CFG]);
	else
		mtk_crtc_pkt_create(&cmdq_handle, crtc,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

	/* mml need to power on InlineRotate and sync with mml */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MML_PRIMARY) &&
		need_sync_mml)
		mml_cmdq_pkt_init(crtc, cmdq_handle);

	if (old_crtc_state != NULL)
		old_mtk_state = to_mtk_crtc_state(old_crtc_state);
	/*Msync 2.0 change to check vfp period token instead of EOF*/
	if (!mtk_crtc_is_frame_trigger_mode(crtc) &&
			msync_is_on(priv, params, crtc_id,
				crtc_state, old_mtk_state) &&
			!mtk_crtc->msync2.msync_disabled) {
		/* Msync 2.0 ToDo, consider move into mtk_crtc_wait_frame_done?
		 * if Msync on, change to wait vfp period token,instead of eof
		 * if 0->1 enable Msync at current frame
		 * if 1->0 disable Msync at next frame
		 */
		DDPDBG("%s:%d, msync = 1, wait vfp period sw token\n",
			__func__, __LINE__);
		cmdq_pkt_wait_no_clear(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);
	} else {
		mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);
	}


	if (mtk_crtc->sec_on) {
		comp = mtk_ddp_comp_request_output(mtk_crtc);
		if (crtc_id == 2)
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					IRQ_LEVEL_IDLE, NULL);
		DDPDBG("%s:%d crtc:0x%p, sec_on:%d +\n",
			__func__, __LINE__, crtc, mtk_crtc->sec_on);
	}
	return cmdq_handle;
}

/******************Msync 2.0 function start**********************/
#define msync_index_inc(index) \
	((index) = ((index) + 1) % MSYNC_MAX_RECORD)
#define msync_index_dec(index) \
	((index) = ((index) + MSYNC_MAX_RECORD - 1) % MSYNC_MAX_RECORD)

#ifndef DRM_CMDQ_DISABLE
static void msync_add_frame_time(struct mtk_drm_crtc *mtk_crtc,
		enum MSYNC_RECORD_TYPE type, u64 time)
{
	static bool feature_on;	/*  C spec, static / global will init to 0(default) */
	struct mtk_crtc_state *state;
	struct drm_display_mode *mode;
	struct mtk_msync2_dy *msync_dy = &mtk_crtc->msync2.msync_dy;
	unsigned int mode_idx;
	unsigned int last_msync_idx;
	unsigned int fps;
	u64 time_diff;

	if (msync_dy->dy_en == 0 || msync_dy->record_index < 0)
		return;

	switch (type) {
	case ENABLE_MSYNC:
		DDPDBG("[Msync][%s] ENABLE_MSYNC\n", __func__);
		mtk_crtc->msync2.msync_disabled = false;
		feature_on = true;
		msync_dy->record[msync_dy->record_index].type = type;
		msync_dy->record[msync_dy->record_index].time = time;
		msync_index_inc(msync_dy->record_index);
		break;
	case DISABLE_MSYNC:
		DDPDBG("[Msync][%s] DISABLE_MSYNC\n", __func__);
		mtk_crtc->msync2.msync_disabled = false;
		feature_on = false;
		msync_dy->record[msync_dy->record_index].type = type;
		msync_dy->record[msync_dy->record_index].time = time;
		msync_index_inc(msync_dy->record_index);
		break;
	case FRAME_TIME:
		DDPDBG("[Msync][%s] FRAME_TIME\n", __func__);
		if (!feature_on)
			return;
		msync_dy->record[msync_dy->record_index].type = type;
		msync_dy->record[msync_dy->record_index].time = time;
		last_msync_idx = msync_dy->record_index;
		msync_index_dec(last_msync_idx);
		if (msync_dy->record[last_msync_idx].type != ENABLE_MSYNC &&
			msync_dy->record[last_msync_idx].type != DISABLE_MSYNC &&
			msync_dy->record[last_msync_idx].type != FRAME_TIME) {
			msync_dy->record[msync_dy->record_index].low_frame = false;
			msync_index_inc(msync_dy->record_index);
			DDPDBG("[Msync] low_frame set false, return\n");
			return;
		}
		state = to_mtk_crtc_state(mtk_crtc->base.state);
		mode_idx = state->prop_val[CRTC_PROP_DISP_MODE_IDX];
		mode = &(mtk_crtc->avail_modes[mode_idx]);
		fps = drm_mode_vrefresh(mode);
		time_diff = msync_dy->record[msync_dy->record_index].time -
			msync_dy->record[last_msync_idx].time;
		DDPDBG("[Msync] min fps:%f, fps:%d, time_diff:%d\n", MSYNC_MIN_FPS, fps, time_diff);
		if (1000 * 1000 * 1000 / MSYNC_MIN_FPS < time_diff) {
			msync_dy->record[msync_dy->record_index].low_frame = true;
			DDPDBG("[Msync] low_frame = true\n");
		} else {
			msync_dy->record[msync_dy->record_index].low_frame = false;
			DDPDBG("[Msync] low_frame = false\n");
		}
		msync_index_inc(msync_dy->record_index);
		break;
	default:
		break;
	}
}
#endif

static bool msync_need_disable(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_msync2_dy *msync_dy = &mtk_crtc->msync2.msync_dy;
	int index = msync_dy->record_index;
	int low_frame_count = 0;
	int i = 0;

	if (msync_dy->dy_en == 0 || mtk_crtc->msync2.msync_disabled)
		return false;

	msync_index_dec(index);
	while (i++ < MSYNC_MAX_RECORD) {
		if (msync_dy->record[index].type != FRAME_TIME)
			break;
		if (msync_dy->record[index].low_frame)
			low_frame_count++;
		msync_index_dec(index);
	}
	return low_frame_count >= MSYNC_LOWFRAME_THRESHOLD;
}

#ifndef DRM_CMDQ_DISABLE
static bool msync_need_enable(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_msync2_dy *msync_dy = &mtk_crtc->msync2.msync_dy;
	int index = msync_dy->record_index;
	int low_frame_count = 0;
	int i = 0;

	if (msync_dy->dy_en == 0 || !mtk_crtc->msync2.msync_disabled)
		return false;

	msync_index_dec(index);
	while (i++ < MSYNC_MAX_RECORD) {
		if (msync_dy->record[index].type != FRAME_TIME)
			break;
		if (msync_dy->record[index].low_frame)
			low_frame_count++;
		msync_index_dec(index);
	}
	if (i >= MSYNC_MAX_RECORD &&
			low_frame_count == 0)
		return true;
	return false;
}
#endif

static void mtk_crtc_msync2_add_cmds_bef_cfg(struct drm_crtc *crtc,
				      struct mtk_crtc_state *old_mtk_state,
				      struct mtk_crtc_state *crtc_state,
				      struct cmdq_pkt *cmdq_handle)
{
#ifdef DRM_CMDQ_DISABLE
	return;
#else
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int index = drm_crtc_index(crtc);
	bool need_enable = false;
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	/*cmdq if/else condition*/
	struct cmdq_operand lop;
	struct cmdq_operand rop;

	u32 inst_condi_jump = 0;
	u64 *inst = NULL;
	dma_addr_t jump_pa = 0;

	const u16 reg_jump = CMDQ_THR_SPR_IDX1;
	const u16 vfp_period = CMDQ_THR_SPR_IDX2;

	struct mtk_ddp_comp *output_comp;
	dma_addr_t addr;

	/*0->1*/
	if ((old_mtk_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] == 0) &&
				(crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0)) {
		DDPMSG("%s, Msync 0 -> 1\n", __func__);
		CRTC_MMP_EVENT_START(index, msync_enable, 1, 0);
		msync_add_frame_time(mtk_crtc, ENABLE_MSYNC, sched_clock());
	}
	/*1->0*/
	else if ((old_mtk_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0) &&
				(crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] == 0)) {
		DDPMSG("%s, Msync 1 -> 0\n", __func__);
		CRTC_MMP_EVENT_END(index, msync_enable, 0, 0);
		msync_add_frame_time(mtk_crtc, DISABLE_MSYNC, sched_clock());
	} else if (crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0) {
		msync_add_frame_time(mtk_crtc, FRAME_TIME, sched_clock());
	}

	if (crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0)
		need_enable = msync_need_enable(mtk_crtc);

	DDPDBG("%s, Msync change cfg thread cmds\n", __func__);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (((old_mtk_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] == 0) &&
		(crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0)) ||
		need_enable) {
		/*0->1 need disable LFR and init vfp early stop register*/
		if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_LFR)) {
			int en = 0;

			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
					DSI_LFR_SET, &en);
			mtk_drm_helper_set_opt_by_name(priv->helper_opt, "MTK_DRM_OPT_LFR", en);
			mtk_crtc->msync2.LFR_disabled = true;
		}
		if (output_comp) {
			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
				DSI_INIT_VFP_EARLY_STOP, NULL);
			mtk_crtc->msync2.msync_on = true;
		}
		if (need_enable) {
			mtk_crtc->msync2.msync_disabled = false;
			DDPINFO("[Msync] msync_disabled = false\n");
		}
	}
	if (mtk_crtc->msync2.msync_disabled)
		return;

	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
			DSI_ADD_VFP_FOR_MSYNC, NULL);
	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_VFP_PERIOD);
	/* for debug: first init vfp period slot to 1*/
	cmdq_pkt_write(cmdq_handle,
		mtk_crtc->gce_obj.base, addr, 1, ~0);
	lop.reg = true;
	lop.idx = vfp_period;

	rop.reg = false;
	rop.value = 0x1000;

	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
			DSI_READ_VFP_PERIOD, (void *)&vfp_period);

	cmdq_pkt_logic_command(cmdq_handle,
		CMDQ_LOGIC_AND, vfp_period, &lop, &rop);

	inst_condi_jump = (cmdq_handle)->cmd_buf_size;
	cmdq_pkt_assign_command(cmdq_handle, reg_jump, 0);
	/* check whether vfp_period == 1*/
	cmdq_pkt_cond_jump_abs(cmdq_handle, reg_jump,
		&lop, &rop, CMDQ_EQUAL);

	/* false case:
	 * condition not match
	 * if vfp_period != 1, add wait next vfp period cmd
	 */
	cmdq_pkt_write(cmdq_handle,
		mtk_crtc->gce_obj.base, addr, 0, ~0);

	/*clear last EOF*/
	cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
	/*clear last vfp period sw token*/
	cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);
	cmdq_pkt_wait_no_clear(cmdq_handle,
		     mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);

	inst = cmdq_pkt_get_va_by_offset(cmdq_handle,
			inst_condi_jump);
	jump_pa = cmdq_pkt_get_pa_by_offset(cmdq_handle,
			(cmdq_handle)->cmd_buf_size);
	*inst = *inst | CMDQ_REG_SHIFT_ADDR(jump_pa);

	/* true case:
	 * condition match
	 * only if vfp_period ==1, we can config directly
	 */
#endif
}

static struct msync_parameter_table msync_params_tb;
static struct msync_level_table msync_level_tb[MSYNC_MAX_LEVEL];
unsigned int msync_cmd_level_tb_dirty;

int (*mtk_drm_get_target_fps_fp)(unsigned int vrefresh, unsigned int atomic_fps);
EXPORT_SYMBOL(mtk_drm_get_target_fps_fp);

int (*mtk_sync_te_level_decision_fp)(void *level_tb, unsigned int te_type,
	unsigned int target_fps, unsigned int *fps_level,
	unsigned int *min_fps, unsigned long x_time);
EXPORT_SYMBOL(mtk_sync_te_level_decision_fp);

int (*mtk_sync_slow_descent_fp)(unsigned int fps_level, unsigned int fps_level_old,
	unsigned int delay_frame_num);
EXPORT_SYMBOL(mtk_sync_slow_descent_fp);

void mtk_drm_sort_msync_level_table(void)
{
	struct msync_level_table tmp = {0};
	int i = 0;
	int j = 0;

	for (i = MSYNC_MAX_LEVEL - 1; i >= 0; --i) {
		for (j = 0; j < MSYNC_MAX_LEVEL - 1; ++j) {
			if ((msync_level_tb[j].level_id == 0) ||
				(msync_level_tb[j+1].level_id == 0))
				continue;
			if (msync_level_tb[j].level_fps >
					msync_level_tb[j+1].level_fps) {
				tmp.level_fps = msync_level_tb[j].level_fps;
				msync_level_tb[j].level_fps = msync_level_tb[j+1].level_fps;
				msync_level_tb[j+1].level_fps = tmp.level_fps;

				tmp.max_fps = msync_level_tb[j].max_fps;
				msync_level_tb[j].max_fps = msync_level_tb[j+1].max_fps;
				msync_level_tb[j+1].max_fps = tmp.max_fps;

				tmp.min_fps = msync_level_tb[j].min_fps;
				msync_level_tb[j].min_fps = msync_level_tb[j+1].min_fps;
				msync_level_tb[j+1].min_fps = tmp.min_fps;
			}
		}
	}
}

int mtk_drm_set_msync_cmd_level_table(unsigned int level_id, unsigned int level_fps,
		unsigned int max_fps, unsigned int min_fps)
{
	struct msync_level_table *level_tb = NULL;
	int i = 0;

	DDPINFO("msync_level_tb_set: set level_id;%d, level_fps:%d, max_fps:%d, min_fps:%d\n",
			level_id, level_fps, max_fps, min_fps);

	if (level_id > MSYNC_MAX_LEVEL) {
		DDPINFO("msync_level_tb_set: ERROR level out of max_level\n");
		return 0;
	}

	if (level_id <= 0) {
		DDPINFO("msync_level_tb_set: ERROR level must start from level 1\n");
		return 0;
	}

	for (i = 0; i < MSYNC_MAX_LEVEL - 1; i++) {
		if (msync_level_tb[i].level_id == 0)
			if (level_id > (i+1)) {
				DDPINFO("msync_level_tb_set: ERROR next level must be %d\n", i+1);
				return 0;
			}
	}

	level_tb = &msync_level_tb[level_id-1];

	level_tb->level_id = level_id;
	level_tb->level_fps = level_fps;
	level_tb->max_fps = max_fps;
	level_tb->min_fps = min_fps;

	mtk_drm_sort_msync_level_table();
	msync_cmd_level_tb_dirty = 1;
	DDPINFO("msync_level_tb_set: Set Successful!\n");

	return 0;
}

void mtk_drm_clear_msync_cmd_level_table(void)
{
	int i = 0;
	struct msync_level_table *level_tb = NULL;

	for (i = 0; i < MSYNC_MAX_LEVEL; i++) {
		level_tb = &msync_level_tb[i];

		level_tb->level_id = 0;
		level_tb->level_fps = 0;
		level_tb->max_fps = 0;
		level_tb->min_fps = 0;
	}

	DDPMSG("========clear msync_level_tb done========\n");

	msync_cmd_level_tb_dirty = 0;
}

void mtk_drm_get_msync_cmd_level_table(void)
{
	int i = 0;
	struct msync_level_table *level_tb = NULL;

	DDPMSG("========msync_level_tb_get start========\n");
	for (i = 0; i < MSYNC_MAX_LEVEL; i++) {
		level_tb = &msync_level_tb[i];
		DDPMSG("msync_level_tb_get:level%u level_fps:%u max_fps:%u min_fps:%u\n",
		level_tb->level_id, level_tb->level_fps,
		level_tb->max_fps, level_tb->min_fps);
	}
	DDPMSG("========msync_level_tb_get end========\n");
}

int mtk_drm_get_atomic_fps(void)
{
	struct timespec64 tval;
	static unsigned long sec;
	static unsigned long usec;
	static unsigned long sec1;
	static unsigned long usec1;
	unsigned long x_time = 0;
	static int count;
	unsigned int target_fps = 0;

	/* TODO: add err handle */
	if (count == 0) {
		ktime_get_real_ts64(&tval);
		sec = tval.tv_sec;
		usec = tval.tv_nsec/1000;
		count++;
	} else {
		ktime_get_real_ts64(&tval);
		sec1 = tval.tv_sec - sec;
		usec1 = tval.tv_nsec/1000 - usec;
		x_time = sec1 * 1000000 + usec1;

		target_fps = 1000000/x_time;
		sec = tval.tv_sec;
		usec = tval.tv_nsec/1000;
	}

	return target_fps;
}

static void mtk_crtc_msync2_send_cmds_bef_cfg(struct drm_crtc *crtc, unsigned int target_fps)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc ? mtk_ddp_comp_request_output(mtk_crtc) : NULL;
	struct mtk_panel_params *params = mtk_drm_get_lcm_ext_params(crtc);
	unsigned int fps_level = 0;
	unsigned int min_fps = 0;
	unsigned int need_send_cmd = 0;
	int index = drm_crtc_index(crtc);
	static unsigned int fps_level_old;
	static unsigned int min_fps_old;
	static unsigned long sec;
	static unsigned long usec;
	unsigned long x_time = 0;
	static unsigned int count;
	static unsigned int count1;
	static unsigned int msync_is_close;
	dma_addr_t addr = 0;
	struct mtk_oddmr_timing oddmr_timing = { 0 };
	struct mtk_ddp_comp *oddmr_comp;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct drm_display_mode *mode;


	DDPMSG("[Msync2.0] Cmd mode send cmds before config\n");

	if (!params || !state || !mtk_crtc || !comp) {
		DDPPR_ERR("[Msync2.0] Some pointer is NULL\n");
		return;
	}

	if (!mtk_drm_get_target_fps_fp) {
		DDPPR_ERR("[Msync2.0] mtk_drm_get_target_fps_fp is NULL\n");
		return;
	}

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_REQUEST_TE_PREPARE);

	/* Get SOF - atomic_flush time*/
	sec = rdma_sof_tval.tv_sec - atomic_flush_tval.tv_sec;
	usec = rdma_sof_tval.tv_nsec/1000 - atomic_flush_tval.tv_nsec/1000;
	x_time = sec * 1000000 + usec;  /* time is usec as unit */
	DDPMSG("[Msync2.0]Get SOF - atomic_flush time:%lu\n", x_time);

	/* If need request TE, to do it here */
	if (params->msync_cmd_table.te_type == REQUEST_TE) {
		struct msync_request_te_table *rte_tb =
			&params->msync_cmd_table.request_te_tb;
		int level_min_fps = 0;

		DDPMSG("%s:%d RTE\n", __func__, __LINE__);
		if (target_fps == 0xFFFF) {
			DDPMSG("[Msync2.0] Msync Need close R-TE\n");
			fps_level = 0xFFFF;
			min_fps = drm_mode_vrefresh(&crtc->state->mode);
		} else if (mtk_sync_te_level_decision_fp) {
			x_time = 0;
			mtk_sync_te_level_decision_fp((void *)rte_tb->rte_te_level,
				REQUEST_TE, target_fps, &fps_level, &min_fps, x_time);
			level_min_fps = rte_tb->rte_te_level[0].min_fps;

			if (msync_cmd_level_tb_dirty) {
				mtk_sync_te_level_decision_fp((void *)msync_level_tb,
					REQUEST_TE, target_fps, &fps_level, &min_fps, x_time);
				level_min_fps = msync_level_tb[0].min_fps;
			}
		} else {
			DDPMSG("[Msync2.0] Not have level decision function\n");
			return;
		}

		DDPMSG("[Msync2.0] R-TE target_fps:%u fps_level:%u dirty:%u min_fps:%u\n",
			target_fps, fps_level, msync_cmd_level_tb_dirty, min_fps);
		DDPMSG("[Msync2.0] R-TE fps_level_old:%u min_fps_old:%u\n",
			fps_level_old, min_fps_old);


		if (target_fps == 0xFFFF)
			goto rte_target;

		if (target_fps < level_min_fps)
			count++;

		if (target_fps >= level_min_fps)
			count = 0;

		if (count == 5) {
			count = 0;
			if (msync_is_close == 0) {
				mtk_crtc_msync2_send_cmds_bef_cfg(crtc, 0xFFFF);
				cmdq_pkt_write(state->cmdq_handle, mtk_crtc->gce_obj.base,
					addr, DISABLE_REQUEST_TE, ~0);
			}
			msync_is_close = 1;
			DDPMSG("[Msync2.0] low min fps close msync\n");
			return;
		}

		if (target_fps > level_min_fps)
			count1++;

		if (target_fps <= level_min_fps)
			count1 = 0;

		DDPMSG("[Msync2.0] count1:%u msync_is_close:%d\n",
				count1, msync_is_close);
		if ((count1 != 5) && (msync_is_close == 1)) {
			DDPMSG("[Msync2.0] continue close msync\n");
			return;
		}

		count1 = 0;
		cmdq_pkt_write(state->cmdq_handle, mtk_crtc->gce_obj.base, addr,
			ENABLE_REQUSET_TE, ~0);
		msync_is_close = 0;
		DDPMSG("[Msync2.0] come back enable msync\n");

rte_target:
		if (mtk_sync_slow_descent_fp) {
			int ret = mtk_sync_slow_descent_fp(fps_level, fps_level_old,
				params->msync_cmd_table.delay_frame_num);
			DDPMSG("[Msync2.0] mtk_sync_slow_descent ret:%d\n", ret);
			if (ret == 1)
				return;
		}

		/*Msync 2.0: add Msync trace info*/
		mtk_drm_trace_begin("msync_level_fps:%u[%u] to %u[%u]",
			fps_level_old, min_fps_old,
			fps_level, min_fps);
		CRTC_MMP_MARK(index, atomic_begin, fps_level, min_fps);

		if ((fps_level != fps_level_old) ||
			(msync_cmd_level_tb_dirty && (min_fps != min_fps_old))) {
			/*
			 * 1,clear CABC_EOF to avoid backlight
			 *   which using DSI_CFG and async
			 *	if BL change to use CFG thread, remove this part
			 * 2,clear dirty before sending cmd to
			 *  avoid trigger loop be started during sending cmd
			 */
			cmdq_pkt_wfe(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_clear_event(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
			need_send_cmd = 1;
		}

		/* Switch msync TE level */
		if (fps_level != fps_level_old) {

			/* scaling path */
			oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];
			mode = mtk_crtc_get_display_mode_by_comp(__func__, crtc, oddmr_comp, false);
			oddmr_timing.hdisplay = mode->hdisplay;
			oddmr_timing.vdisplay = mode->vdisplay;
			if (fps_level == 0xFFFF)
				oddmr_timing.vrefresh =  drm_mode_vrefresh(mode);
			else
				oddmr_timing.vrefresh = fps_level;

			mtk_ddp_comp_io_cmd(oddmr_comp, state->cmdq_handle,
				ODDMR_TIMING_CHG, &oddmr_timing);
			mtk_ddp_comp_io_cmd(comp, state->cmdq_handle,
				DSI_MSYNC_SWITCH_TE_LEVEL_GRP, &fps_level);
			fps_level_old = fps_level;
		}

		/*
		 * Dynamically modify minfps of level
		 * nt37801 do not support dynamically modify minfps so disable
		 * this part code. if other lcm support, need enable this part
		 * code.
		 */
		if (1 == 0) {
			/* Set min fps */
			if ((msync_cmd_level_tb_dirty && (min_fps != min_fps_old))
					|| (fps_level == 0xFFFF)) {
				unsigned int flag = 0;

				flag = (fps_level << 16) | min_fps;
				mtk_ddp_comp_io_cmd(comp, state->cmdq_handle,
						DSI_MSYNC_CMD_SET_MIN_FPS, &flag);
				min_fps_old = min_fps;
			}
		}

		if (need_send_cmd) {
			/*release CABC_EOF lock to allow backlight keep going*/
			cmdq_pkt_set_event(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		}

		mtk_drm_trace_end();

	/* If need Multi TE, to do it here */
	} else if (params->msync_cmd_table.te_type == MULTI_TE) {
		struct msync_multi_te_table *mte_tb =
			&params->msync_cmd_table.multi_te_tb;
		int level_min_fps = 0;

		DDPMSG("[Msync2.0] M-TE\n");

		if (target_fps == 0xFFFF) {
			DDPMSG("[Msync2.0] Msync Need close M-TE\n");
			fps_level = 0xFFFF;
			min_fps = drm_mode_vrefresh(&crtc->state->mode);
		} else if (mtk_sync_te_level_decision_fp) {
			mtk_sync_te_level_decision_fp((void *)mte_tb->multi_te_level,
				MULTI_TE, target_fps, &fps_level, &min_fps, x_time);
			level_min_fps = mte_tb->multi_te_level[0].min_fps;

			if (msync_cmd_level_tb_dirty) {
				mtk_sync_te_level_decision_fp((void *)msync_level_tb,
					MULTI_TE, target_fps, &fps_level, &min_fps, x_time);
				level_min_fps = msync_level_tb[0].min_fps;
			}
		} else {
			DDPMSG("[Msync2.0] Not have level decision function\n");
			return;
		}

		DDPMSG("[Msync2.0] M-TE target_fps:%u fps_level:%u dirty:%u min_fps:%u\n",
			target_fps, fps_level, msync_cmd_level_tb_dirty, min_fps);
		DDPMSG("[Msync2.0] M-TE fps_level_old:%u min_fps_old:%u\n",
			fps_level_old, min_fps_old);

		if (target_fps == 0xFFFF)
			goto mte_target;

		if (target_fps < level_min_fps)
			count++;

		if (target_fps >= level_min_fps)
			count = 0;

		if (count == 5) {
			count = 0;
			if (msync_is_close == 0)
				mtk_crtc_msync2_send_cmds_bef_cfg(crtc, 0xFFFF);
			msync_is_close = 1;
			DDPMSG("[Msync2.0] low min fps close msync\n");
			return;
		}

		if (target_fps > level_min_fps)
			count1++;

		if (target_fps <= level_min_fps)
			count1 = 0;

		DDPMSG("[Msync2.0] count1:%u msync_is_close:%d\n",
				count1, msync_is_close);
		if ((count1 != 5) && (msync_is_close == 1)) {
			DDPMSG("[Msync2.0] continue close msync\n");
			return;
		}

		count1 = 0;
		msync_is_close = 0;
		DDPMSG("[Msync2.0] come back enable msync\n");

mte_target:
		if (mtk_sync_slow_descent_fp) {
			int ret = mtk_sync_slow_descent_fp(fps_level, fps_level_old,
				params->msync_cmd_table.delay_frame_num);
			DDPMSG("[Msync2.0] mtk_sync_slow_descent ret:%d\n", ret);
			if (ret == 1)
				return;
		}

		/*Msync 2.0: add Msync trace info*/
		mtk_drm_trace_begin("msync_level_fps:%u[%u] to %u[%u]",
			fps_level_old, min_fps_old,
			fps_level, min_fps);
		CRTC_MMP_MARK(index, atomic_begin, fps_level, min_fps);

		if ((fps_level != fps_level_old) ||
			(msync_cmd_level_tb_dirty && (min_fps != min_fps_old))) {
			/*
			 * 1,clear CABC_EOF to avoid backlight
			 *   which using DSI_CFG and async
			 *	if BL change to use CFG thread, remove this part
			 * 2,clear dirty before sending cmd to
			 *  avoid trigger loop be started during sending cmd
			 */
			cmdq_pkt_wfe(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
			cmdq_pkt_clear_event(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
			need_send_cmd = 1;

		}

		/* Switch msync TE level */
		if (fps_level != fps_level_old) {

			/* scaling path */
			oddmr_comp = priv->ddp_comp[DDP_COMPONENT_ODDMR0];
			mode = mtk_crtc_get_display_mode_by_comp(__func__, crtc, oddmr_comp, false);
			oddmr_timing.hdisplay = mode->hdisplay;
			oddmr_timing.vdisplay = mode->vdisplay;
			if (fps_level == 0xFFFF)
				oddmr_timing.vrefresh =  drm_mode_vrefresh(mode);
			else
				oddmr_timing.vrefresh = fps_level;

			mtk_ddp_comp_io_cmd(oddmr_comp, state->cmdq_handle,
				ODDMR_TIMING_CHG, &oddmr_timing);
			mtk_ddp_comp_io_cmd(comp, state->cmdq_handle,
					DSI_MSYNC_SWITCH_TE_LEVEL_GRP, &fps_level);
			fps_level_old = fps_level;
		}

		/* Set min fps */
		if ((msync_cmd_level_tb_dirty && (min_fps != min_fps_old))
			|| (fps_level == 0xFFFF)) {
			unsigned int flag = 0;

			flag = (fps_level << 16) | min_fps;
			mtk_ddp_comp_io_cmd(comp, state->cmdq_handle,
					DSI_MSYNC_CMD_SET_MIN_FPS, &flag);
			min_fps_old = min_fps;
		}

		if (need_send_cmd) {
			/*release CABC_EOF lock to allow backlight keep going*/
			cmdq_pkt_set_event(state->cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		}

		mtk_drm_trace_end();

	} else if (params->msync_cmd_table.te_type == TRIGGER_LEVEL_TE) {
		/* TODO: Add Trigger Level Te */

	} else {
		DDPPR_ERR("%s:%d No TE Type\n", __func__, __LINE__);
	}
}

/******************Msync 2.0 function end**********************/


static void update_frame_weight(struct drm_crtc *crtc,
		struct mtk_crtc_state *crtc_state)
{
	struct mtk_drm_lyeblob_ids *lyeblob_ids, *next;
	struct mtk_drm_private *mtk_drm = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_plane *plane;
	struct mtk_plane_state *plane_state;
	struct mtk_plane_pending_state *pending;
	struct list_head *lyeblob_head;
	unsigned int prop_lye_idx;
	int phy_lay_num = 0, sphrt_enable;
	int i;

	if (drm_crtc_index(crtc) != 0)
		return;

	for (i = 0; i < OVL_LAYER_NR; i++) {
		plane = &mtk_crtc->planes[i].base;
		plane_state = to_mtk_plane_state(plane->state);
		pending = &plane_state->pending;
		if ((pending->enable == true &&
				plane_state->comp_state.ext_lye_id == LYE_NORMAL) ||
				((mtk_crtc->fake_layer.fake_layer_mask & BIT(i)) &&
				i < PRIMARY_OVL_PHY_LAYER_NR))
			phy_lay_num++;
	}

	mutex_lock(&mtk_drm->lyeblob_list_mutex);
	prop_lye_idx = crtc_state->prop_val[CRTC_PROP_LYE_IDX];
	sphrt_enable = mtk_drm_helper_get_opt(mtk_drm->helper_opt, MTK_DRM_OPT_SPHRT);
	lyeblob_head = (sphrt_enable == 0) ? (&mtk_drm->lyeblob_head) : (&mtk_crtc->lyeblob_head);
	list_for_each_entry_safe(lyeblob_ids, next, lyeblob_head, list) {
		if (lyeblob_ids->lye_idx == prop_lye_idx)
			lyeblob_ids->frame_weight = phy_lay_num * 2;
	}
	mutex_unlock(&mtk_drm->lyeblob_list_mutex);
}

static void mtk_drm_crtc_atomic_begin(struct drm_crtc *crtc,
				      struct drm_atomic_state *atomic_state)
{
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc->state);
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(atomic_state, crtc);

	int index = drm_crtc_index(crtc);
	struct mtk_ddp_comp *comp;
	int i, j;
	unsigned int crtc_idx = drm_crtc_index(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	/*Msync 2.0*/
	unsigned long crtc_id = (unsigned long)drm_crtc_index(crtc);
	struct mtk_crtc_state *old_mtk_state =
		to_mtk_crtc_state(old_crtc_state);
	struct mtk_panel_params *params =
			mtk_drm_get_lcm_ext_params(crtc);
	unsigned int target_fps = 0;
	unsigned int atomic_fps = 0;
	static unsigned int msync_may_close;
	unsigned int msync_fps_record[60];
	static unsigned int position;
	dma_addr_t msync_slot_addr;
	bool msync20_status_changed = 0;
	struct mtk_ddp_comp *output_comp;
#ifndef DRM_CMDQ_DISABLE
	struct cmdq_client *client;
#endif

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	/* When open VDS path switch feature, we will resume VDS crtc
	 * in it's second atomic commit, and the crtc will be resumed
	 * one time.
	 */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_VDS_PATH_SWITCH) && (crtc_idx == 2))
		if (priv->vds_path_switch_done &&
			!priv->vds_path_enable) {
			DDPMSG("Switch vds: CRTC2 vds enable\n");
			mtk_drm_crtc_atomic_resume(crtc, NULL);
			priv->vds_path_enable = 1;
		}

	CRTC_MMP_EVENT_START(index, atomic_begin,
			(unsigned long)mtk_crtc->event,
			(unsigned long)mtk_crtc_state->base.event);
	if (mtk_crtc->event && mtk_crtc_state->base.event)
		DRM_ERROR("new event while there is still a pending event\n");

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT) && crtc_idx < MAX_CRTC) {
		if (priv->usage[crtc_idx] == DISP_OPENING) {
			if (!(comp && mtk_ddp_comp_get_type(comp->id) == MTK_DISP_WDMA)) {
				DDPINFO("%s %d skip due to still opening\n", __func__, crtc_idx);
				CRTC_MMP_MARK(index, atomic_begin, 0, 0xF);
				goto end;
			}
		} else if (mtk_crtc->enabled == 0 && priv->usage[crtc_idx] == DISP_ENABLE) {
			mtk_drm_crtc_enable(crtc);
			if (comp && mtk_ddp_comp_get_type(comp->id) == MTK_DISP_WDMA) {
				/* top clk/CG  have been enabledprepared while DISP_OPENNING,
				 *disable that iteration prevent unbalanced ref cnt.
				 */
#ifndef DRM_CMDQ_DISABLE
				client = mtk_crtc->gce_obj.client[CLIENT_CFG];
				cmdq_mbox_disable(client->chan);
#endif
				mtk_crtc_ddp_unprepare(mtk_crtc);
				mtk_drm_top_clk_disable_unprepare(crtc->dev);
			}
			if (comp)
				mtk_ddp_comp_io_cmd(comp, NULL, CONNECTOR_PANEL_ENABLE, NULL);
			else
				DDPPR_ERR("%s %d invalid output_comp\n", __func__, __LINE__);
			mtk_crtc_hw_block_ready(crtc);
		}
	}

	if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK(index, atomic_begin, 0, 0);
		goto end;
	}

	mtk_drm_idlemgr_kick(__func__, crtc, 0);

	if (mtk_crtc_state->base.event) {
		mtk_crtc_state->base.event->pipe = index;
		if (drm_crtc_vblank_get(crtc) != 0)
			DDPAEE("%s:%d, invalid vblank:%d, crtc:%p\n",
				__func__, __LINE__,
				drm_crtc_vblank_get(crtc), crtc);
		mtk_crtc->event = mtk_crtc_state->base.event;
		mtk_crtc_state->base.event = NULL;
	}

	/*Msync 2.0: add Msync trace info*/
	mtk_drm_trace_begin("mtk_drm_crtc_atomic:%d-%d-%d-%d",
				crtc_idx, mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX],
				mtk_crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE],
				mtk_crtc->msync2.msync_disabled);

#ifndef DRM_CMDQ_DISABLE
	/* BW monitor: Record Key, Clear valid, Set pointer */
	mtk_crtc->fbt_layer_id = -1;
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_OVL_BW_MONITOR) &&
		(crtc_idx == 0)) {
		mtk_drm_ovl_bw_monitor_ratio_prework(crtc, atomic_state);
	}
#endif

	mtk_crtc_state->cmdq_handle =
		mtk_crtc_gce_commit_begin(crtc, old_crtc_state, mtk_crtc_state, true);
	CRTC_MMP_MARK(index, atomic_begin, (unsigned long)mtk_crtc_state->cmdq_handle, 0);

#ifndef DRM_CMDQ_DISABLE
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT) &&
		crtc_idx < MAX_CRTC && priv->usage[crtc_idx] == DISP_OPENING) {
		DDPINFO("%s %d skip reset ovl due to still opening\n", __func__, crtc_idx);
	} else {
		/* reset mml ir ovl, TODO: support random ovl, not the first comp of the path */
		if (((mtk_crtc->mml_ir_state == MML_IR_ENTERING) && (crtc_idx == 0)) ||
			atomic_read(&priv->need_recover)) {
			comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, DDP_FIRST_PATH, 0);
			mtk_ddp_comp_reset(comp, mtk_crtc_state->cmdq_handle);
			if (mtk_crtc->is_dual_pipe) {
				comp = mtk_crtc_get_dual_comp(crtc, DDP_FIRST_PATH, 0);
				mtk_ddp_comp_reset(comp, mtk_crtc_state->cmdq_handle);
			}
		}
		cmdq_pkt_reset_ovl(mtk_crtc_state->cmdq_handle, mtk_crtc);
	}

	/* BW monitor: Read and Save BW info */
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_OVL_BW_MONITOR) &&
		(crtc_idx == 0)) {
		mtk_drm_ovl_bw_monitor_ratio_get(crtc, atomic_state);
	}
#endif

	/*Msync 2.0: add cmds to cfg thread*/
	if (!mtk_crtc_is_frame_trigger_mode(crtc) &&
		msync_is_on(priv, params, crtc_id,
			mtk_crtc_state, old_mtk_state)) {
		mtk_crtc_msync2_add_cmds_bef_cfg(crtc, old_mtk_state, mtk_crtc_state,
				mtk_crtc_state->cmdq_handle);
	} else if (mtk_crtc_is_frame_trigger_mode(crtc) &&
			msync_is_on(priv, params, crtc_id,
				mtk_crtc_state, old_mtk_state) && (index == 0)) {

		/* Get target fps for msync2.0 */
		atomic_fps = mtk_drm_get_atomic_fps();
		/* cycle record fps */
		msync_fps_record[position] = atomic_fps;
		if (position < (sizeof(msync_fps_record)/sizeof(unsigned int)) - 1)
			position++;
		else
			position = 0;

		WARN_ON(!mtk_drm_get_target_fps_fp);
		if (mtk_drm_get_target_fps_fp)
			target_fps = mtk_drm_get_target_fps_fp(
				drm_mode_vrefresh(&crtc->state->adjusted_mode),
				atomic_fps);
		else
			target_fps = drm_mode_vrefresh(&crtc->state->adjusted_mode);

		CRTC_MMP_MARK(index, atomic_begin, 0, 1);

		if (g_msync_debug == 0)
			mtk_crtc_msync2_send_cmds_bef_cfg(crtc, target_fps);

		CRTC_MMP_MARK(index, atomic_begin, 0, 2);

		msync_may_close = 1;
		if (mtk_crtc->msync2.msync_frame_status == 0)
			msync20_status_changed = 1;
		mtk_crtc->msync2.msync_frame_status = 1;
	} else if (mtk_crtc_is_frame_trigger_mode(crtc) &&
			!msync_is_on(priv, params, crtc_id,
				mtk_crtc_state, old_mtk_state) && (index == 0)) {
		if ((msync_may_close == 1) && (g_msync_debug == 0)) {
			mtk_crtc_msync2_send_cmds_bef_cfg(crtc, 0xFFFF);
			msync_may_close = 0;
			msync_slot_addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
				DISP_SLOT_REQUEST_TE_PREPARE);
			cmdq_pkt_write(mtk_crtc_state->cmdq_handle, mtk_crtc->gce_obj.base,
				msync_slot_addr, DISABLE_REQUEST_TE, ~0);
		}

		if (mtk_crtc->msync2.msync_frame_status == 1)
			msync20_status_changed = 1;
		mtk_crtc->msync2.msync_frame_status = 0;
	}

	if ((msync20_status_changed && (crtc_id == 0)) || g_vidle_apsrc_debug) {
		g_vidle_apsrc_debug = 0;

		/* adjust trigger loop in different display mode */
		DDPINFO("%s msync20_status_changed to %d\n", __func__,
				mtk_crtc->msync2.msync_frame_status);

		mtk_crtc_msync2_switch_begin(crtc);
	}

#ifdef MTK_DRM_ADVANCE
	if (mtk_crtc->fake_layer.fake_layer_mask)
		update_frame_weight(crtc, mtk_crtc_state);
	mtk_crtc_update_ddp_state(crtc, old_crtc_state, mtk_crtc_state,
				  mtk_crtc_state->cmdq_handle);
#endif

	if ((crtc_idx < MAX_CRTC && priv->usage[crtc_idx] == DISP_OPENING) &&
			comp && mtk_ddp_comp_get_type(comp->id) == MTK_DISP_WDMA)
		goto end;

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		comp->qos_bw = 0;
		comp->qos_bw_other = 0;
		comp->fbdc_bw = 0;
		comp->hrt_bw = 0;
		mtk_ddp_comp_config_begin(comp, mtk_crtc_state->cmdq_handle);
	}
	if (!mtk_crtc->is_dual_pipe)
		goto end;

	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		comp->qos_bw = 0;
		comp->qos_bw_other = 0;
		comp->fbdc_bw = 0;
		comp->hrt_bw = 0;
		mtk_ddp_comp_config_begin(comp, mtk_crtc_state->cmdq_handle);
	}

end:
	CRTC_MMP_EVENT_END(index, atomic_begin,
			(unsigned long)mtk_crtc->event,
			(unsigned long)mtk_crtc_state->base.event);
}

void mtk_drm_layer_dispatch_to_dual_pipe(
	struct mtk_drm_crtc *mtk_crtc, unsigned int mmsys_id,
	struct mtk_plane_state *plane_state,
	struct mtk_plane_state *plane_state_l,
	struct mtk_plane_state *plane_state_r,
	unsigned int w)
{
	int src_w, src_h, dst_w, dst_h;
	int left_bg = w/2;
	int right_bg = w/2;
	int roi_w = w;
	struct mtk_crtc_state *crtc_state = NULL;
	struct total_tile_overhead to_info = {0};

	if (mtk_crtc)
		to_info = mtk_crtc_get_total_overhead(mtk_crtc);
	else
		to_info.is_support = 0;
	DDPINFO("%s:overhead is_support:%d, width L:%d R:%d\n", __func__,
			to_info.is_support, to_info.left_in_width, to_info.right_in_width);
	if (to_info.is_support) {
		left_bg += to_info.left_overhead;
		right_bg += to_info.right_overhead;
	}

	memcpy(plane_state_l,
		plane_state, sizeof(struct mtk_plane_state));
	memcpy(plane_state_r,
		plane_state, sizeof(struct mtk_plane_state));

	if (plane_state->pending.mml_mode == MML_MODE_RACING &&
		plane_state->base.crtc != NULL) {
		struct mtk_drm_crtc *mtk_crtc_local = to_mtk_crtc(plane_state->base.crtc);

		if (mtk_crtc_local && mtk_crtc_local->is_mml &&
			mtk_crtc_local->is_force_mml_scen) {
			crtc_state = to_mtk_crtc_state(plane_state->base.crtc->state);
			plane_state_l->pending.width = crtc_state->mml_src_roi[0].width;
			plane_state_l->pending.height = crtc_state->mml_src_roi[0].height;
			// crtc_state->mml_src_roi[0].x and crtc_state->mml_src_roi[0].y
			// should be zero in actually. MML WROT should output at (0,0)
			// in SRAM.
			plane_state_l->pending.src_x = crtc_state->mml_src_roi[0].x;
			plane_state_l->pending.src_y = crtc_state->mml_src_roi[0].y;
			plane_state_l->pending.dst_x = 0;
			plane_state_l->pending.dst_y = 0;

			DDPINFO("%s:%d, plane_l (%u,%u) (%u,%u), (%u,%u)\n",
				__func__, __LINE__,
				plane_state_l->pending.src_x, plane_state_l->pending.src_y,
				plane_state_l->pending.dst_x, plane_state_l->pending.dst_y,
				plane_state_l->pending.width, plane_state_l->pending.height);

			plane_state_r->pending.width = crtc_state->mml_src_roi[1].width;
			plane_state_r->pending.height = crtc_state->mml_src_roi[1].height;
			plane_state_r->pending.src_x = crtc_state->mml_src_roi[1].x;
			plane_state_r->pending.src_y = crtc_state->mml_src_roi[1].y;
			plane_state_r->pending.dst_x = 0;
			plane_state_r->pending.dst_y = 0;

			DDPINFO("%s:%d, plane_r (%u,%u) (%u,%u), (%u,%u)\n",
				__func__, __LINE__,
				plane_state_r->pending.src_x, plane_state_r->pending.src_y,
				plane_state_r->pending.dst_x, plane_state_r->pending.dst_y,
				plane_state_r->pending.width, plane_state_r->pending.height);
			return;
		}
	}
	if (plane_state->base.crtc != NULL) {
		crtc_state = to_mtk_crtc_state(plane_state->base.crtc->state);

		src_w = drm_rect_width(&plane_state->base.src) >> 16;
		src_h = drm_rect_height(&plane_state->base.src) >> 16;
		dst_w = drm_rect_width(&plane_state->base.dst);
		dst_h = drm_rect_height(&plane_state->base.dst);

		if (src_w < dst_w || src_h < dst_h) {
			left_bg = crtc_state->rsz_param[0].in_len;
			right_bg = crtc_state->rsz_param[1].in_len;
			roi_w = crtc_state->rsz_src_roi.width;
			DDPDBG("dual rsz %u, %u, %u\n", left_bg, right_bg, roi_w);
		}
	} else {
		DDPINFO("%s crtc is NULL\n", __func__);
	}

	/*left path*/
	plane_state_l->pending.width  = left_bg -
		plane_state_l->pending.dst_x;

	if (plane_state_l->pending.width > plane_state->pending.width)
		plane_state_l->pending.width = plane_state->pending.width;

	if ((to_info.is_support ? left_bg : w/2) <= plane_state->pending.dst_x) {
		plane_state_l->pending.dst_x = 0;
		plane_state_l->pending.width = 0;
		plane_state_l->pending.height = 0;
		plane_state_l->pending.enable = 0;
	}

	if (plane_state_l->pending.width == 0)
		plane_state_l->pending.height = 0;

	if (plane_state_l->pending.width > (to_info.is_support ? left_bg : w/2))
		plane_state_l->pending.width = (to_info.is_support ? left_bg : w/2);

	DDPDBG("plane_l (%u,%u) (%u,%u), (%u,%u)\n",
		plane_state_l->pending.src_x, plane_state_l->pending.src_y,
		plane_state_l->pending.dst_x, plane_state_l->pending.dst_y,
		plane_state_l->pending.width, plane_state_l->pending.height);

	/*right path*/

	plane_state_r->comp_state.comp_id =
		dual_pipe_comp_mapping(mmsys_id, plane_state->comp_state.comp_id);
	plane_state_r->pending.width +=
		plane_state_r->pending.dst_x - (roi_w - right_bg);

	if (plane_state_r->pending.width
		> plane_state->pending.width)
		plane_state_r->pending.width =
		plane_state->pending.width;

	plane_state_r->pending.dst_x +=
		plane_state->pending.width -
		plane_state_r->pending.width - (roi_w - right_bg);

	plane_state_r->pending.src_x +=
		plane_state->pending.width -
		plane_state_r->pending.width;

	if ((roi_w - right_bg) > (plane_state->pending.width
		+ plane_state->pending.dst_x)) {
		plane_state_r->pending.dst_x = 0;
		plane_state_r->pending.width = 0;
		plane_state_r->pending.height = 0;
		plane_state_r->pending.enable = 0;
	}

	if (plane_state_r->pending.width == 0)
		plane_state_r->pending.height = 0;

	if (plane_state_r->pending.width > (to_info.is_support ? right_bg : w/2))
		plane_state_r->pending.width = (to_info.is_support ? right_bg : w/2);

	DDPDBG("plane_r (%u,%u) (%u,%u), (%u,%u)\n",
		plane_state_r->pending.src_x, plane_state_r->pending.src_y,
		plane_state_r->pending.dst_x, plane_state_r->pending.dst_y,
		plane_state_r->pending.width, plane_state_r->pending.height);
}

void mtk_drm_crtc_plane_disable(struct drm_crtc *crtc, struct drm_plane *plane,
			       struct mtk_plane_state *plane_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int plane_index = to_crtc_plane_index(plane->index);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc_state);
	struct mtk_ddp_comp *comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, 0, 0);
	struct mtk_plane_comp_state *comp_state;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
#ifndef DRM_CMDQ_DISABLE
	unsigned int v = crtc->state->adjusted_mode.vdisplay;
	unsigned int h = crtc->state->adjusted_mode.hdisplay;
	unsigned int last_fence, cur_fence, sub;
	dma_addr_t addr;
#endif
	struct cmdq_pkt *cmdq_handle = state->cmdq_handle;

	if (comp)
		DDPINFO("%s+ plane_id:%d, comp_id:%d, comp_id:%d\n", __func__,
			plane->index, comp->id, plane_state->comp_state.comp_id);

	if (plane_state->pending.enable) {
		if (mtk_crtc->is_dual_pipe) {
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);
			if (comp)
				mtk_crtc_dual_layer_config(mtk_crtc, comp, plane_index,
					plane_state, cmdq_handle);
		} else {
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);
			if (comp)
				mtk_ddp_comp_layer_config(comp, plane_index, plane_state,
						  cmdq_handle);
		}
#ifndef DRM_CMDQ_DISABLE
		mtk_wb_atomic_commit(mtk_crtc, v, h, state->cmdq_handle);
#else
		mtk_wb_atomic_commit(mtk_crtc);
#endif
	} else {
		comp_state = &(plane_state->comp_state);

		if (comp_state->comp_id) {
			if (mtk_crtc->is_dual_pipe) {
				unsigned int comp_id;

				comp_id = dual_pipe_comp_mapping(priv->data->mmsys_id,
						comp_state->comp_id);
				comp = priv->ddp_comp[comp_id];
				/* disable right pipe's layer */
				mtk_ddp_comp_layer_off(comp, comp_state->lye_id,
						comp_state->ext_lye_id, cmdq_handle);
				DDPINFO("disable layer dual comp_id:%d\n", comp->id);
			}
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);
			mtk_ddp_comp_layer_off(comp, comp_state->lye_id,
					comp_state->ext_lye_id, cmdq_handle);
		} else {
			struct mtk_plane_state *state =
				to_mtk_plane_state(plane->state);

			/* for the case do not contain crtc info, we assume this plane assign to
			 * first component of display path
			 */
			if (!state->crtc && comp) {
				if (mtk_crtc->is_dual_pipe) {
					struct mtk_ddp_comp *comp_r;
					unsigned int comp_r_id;

					comp_r_id = dual_pipe_comp_mapping(priv->data->mmsys_id,
								comp->id);
					comp_r = priv->ddp_comp[comp_r_id];
					mtk_ddp_comp_layer_off(comp_r, plane->index,
							0, cmdq_handle);
					mtk_ddp_comp_layer_off(comp, plane->index, 0, cmdq_handle);
				} else {
					mtk_ddp_comp_layer_off(comp, plane->index, 0, cmdq_handle);
				}
			}
		}
	}
#ifndef DRM_CMDQ_DISABLE
	last_fence = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
		       DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	cur_fence = plane_state->pending.prop_val[PLANE_PROP_NEXT_BUFF_IDX];

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
		DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	if (cur_fence != -1 && cur_fence > last_fence)
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base, addr,
			       cur_fence, ~0);

	if (plane_state->pending.enable &&
	    plane_state->pending.format != DRM_FORMAT_C8)
		sub = 1;
	else
		sub = 0;
	addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
		DISP_SLOT_SUBTRACTOR_WHEN_FREE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base, addr, sub, ~0);
#endif
	DDPINFO("%s-\n", __func__);
}

void mtk_drm_crtc_plane_update(struct drm_crtc *crtc, struct drm_plane *plane,
			       struct mtk_plane_state *plane_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	unsigned int plane_index = to_crtc_plane_index(plane->index);
	struct drm_crtc_state *crtc_state = crtc->state;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc_state);
	struct mtk_ddp_comp *comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, 0, 0);
#ifndef DRM_CMDQ_DISABLE
	unsigned int v = crtc->state->adjusted_mode.vdisplay;
	unsigned int h = crtc->state->adjusted_mode.hdisplay;
	unsigned int last_fence, cur_fence, sub;
	dma_addr_t addr;
#endif
	struct cmdq_pkt *cmdq_handle = state->cmdq_handle;

	if (comp)
		DDPINFO("%s+ plane_id:%d, comp_id:%s, comp_id:%s\n", __func__,
			plane->index, mtk_dump_comp_str_id(comp->id),
			mtk_dump_comp_str_id(plane_state->comp_state.comp_id));

	if (plane_state->pending.enable) {
		mtk_dprec_mmp_dump_ovl_layer(plane_state);
		if (mtk_crtc->is_dual_pipe) {
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);
			mtk_crtc_dual_layer_config(mtk_crtc, comp, plane_index,
					plane_state, cmdq_handle);
		} else {
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);

			mtk_ddp_comp_layer_config(comp, plane_index, plane_state,
						  cmdq_handle);
		}
		//discrete path need re-trigger by itself after plane 0
		if (mtk_crtc->path_data->is_discrete_path &&
			plane_index != 0 && mtk_crtc->pending_handle) {
			mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[1],
				mtk_crtc->pending_handle, mtk_crtc->gce_obj.base);
		}
#ifndef DRM_CMDQ_DISABLE
		mtk_wb_atomic_commit(mtk_crtc, v, h, state->cmdq_handle);
#else
		mtk_wb_atomic_commit(mtk_crtc);
#endif
	} else if (state->prop_val[CRTC_PROP_USER_SCEN] &
		USER_SCEN_BLANK) {
	/* plane disable at mtk_crtc_get_plane_comp_state() actually */
	/* following statement is for disable all layers during suspend */

		if (mtk_crtc->is_dual_pipe) {
			struct mtk_plane_state plane_state_l;
			struct mtk_plane_state plane_state_r;

			if (plane_state->comp_state.comp_id == 0) {
				if (comp)
					plane_state->comp_state.comp_id = comp->id;
				else
					DDPPR_ERR("%s NULL comp_id error\n", __func__, __LINE__);
			}

			mtk_drm_layer_dispatch_to_dual_pipe(mtk_crtc, priv->data->mmsys_id,
				plane_state, &plane_state_l, &plane_state_r,
				crtc->state->adjusted_mode.hdisplay);

			comp = priv->ddp_comp[plane_state_r.comp_state.comp_id];
			mtk_ddp_comp_layer_config(comp, plane_index,
						&plane_state_r, cmdq_handle);
			DDPINFO("%s+D comp_id:%d, comp_id:%d\n",
				__func__, comp->id,
				plane_state_r.comp_state.comp_id);

			comp = mtk_crtc_get_plane_comp(crtc, &plane_state_l);

			mtk_ddp_comp_layer_config(comp, plane_index, &plane_state_l,
						  cmdq_handle);
		}  else {
			comp = mtk_crtc_get_plane_comp(crtc, plane_state);

			mtk_ddp_comp_layer_config(comp, plane_index, plane_state,
						  cmdq_handle);
		}
	}

#ifndef DRM_CMDQ_DISABLE
	last_fence = *(unsigned int *)mtk_get_gce_backup_slot_va(mtk_crtc,
		       DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	cur_fence = (unsigned int)plane_state->pending.prop_val[PLANE_PROP_NEXT_BUFF_IDX];

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
		DISP_SLOT_CUR_CONFIG_FENCE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	if (cur_fence != -1 && cur_fence > last_fence)
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base, addr,
			       cur_fence, ~0);

	if (plane_state->pending.enable &&
	    plane_state->pending.format != DRM_FORMAT_C8)
		sub = 1;
	else
		sub = 0;
	addr = mtk_get_gce_backup_slot_pa(mtk_crtc,
		DISP_SLOT_SUBTRACTOR_WHEN_FREE(mtk_get_plane_slot_idx(mtk_crtc, plane_index)));
	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base, addr, sub, ~0);
#endif
	DDPINFO("%s-\n", __func__);
}

static void mtk_crtc_wb_comp_config(struct drm_crtc *crtc,
	struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_ddp_comp *comp = NULL;
	struct mtk_crtc_ddp_ctx *ddp_ctx = NULL;
	struct mtk_ddp_config cfg;

	comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_WDMA0);
	if (!comp)
		comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_WDMA1);
	if (!comp)
		comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_OVLSYS_WDMA2);
	if (!comp) {
		DDPPR_ERR("The wb component is not exsit\n");
		return;
	}

	memset(&cfg, 0x0, sizeof(struct mtk_ddp_config));
	if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
		/* Output buffer configuration for virtual display */
		DDPINFO("lookup wb fb:%u\n",
			state->prop_val[CRTC_PROP_OUTPUT_FB_ID]);
		comp->fb = mtk_drm_framebuffer_lookup(crtc->dev,
				state->prop_val[CRTC_PROP_OUTPUT_FB_ID]);
		if (comp->fb == NULL) {
			DDPPR_ERR("%s cannot find fb fb_id:%u\n", __func__,
				state->prop_val[CRTC_PROP_OUTPUT_FB_ID]);
			return;
		}
		cfg.w = state->prop_val[CRTC_PROP_OUTPUT_WIDTH];
		cfg.h = state->prop_val[CRTC_PROP_OUTPUT_HEIGHT];
		cfg.x = state->prop_val[CRTC_PROP_OUTPUT_X];
		cfg.y = state->prop_val[CRTC_PROP_OUTPUT_Y];
		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				cfg.vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				cfg.vrefresh =
					drm_mode_vrefresh(&crtc->state->adjusted_mode);
		} else
			cfg.vrefresh =
				drm_mode_vrefresh(&crtc->state->adjusted_mode);
		cfg.bpc = mtk_crtc->bpc;
		cfg.p_golden_setting_context =
				__get_golden_setting_context(mtk_crtc);
		cfg.p_golden_setting_context->dst_width = cfg.w;
		cfg.p_golden_setting_context->dst_height = cfg.h;
	} else {
		/* Output buffer configuration for internal decouple mode */
		ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];
		comp->fb = ddp_ctx->dc_fb;
		ddp_ctx->dc_fb_idx =
			(ddp_ctx->dc_fb_idx + 1) % MAX_CRTC_DC_FB;
		ddp_ctx->dc_fb->offsets[0] =
			mtk_crtc_get_dc_fb_size(crtc) * ddp_ctx->dc_fb_idx;
		cfg.w = crtc->state->adjusted_mode.hdisplay;
		cfg.h = crtc->state->adjusted_mode.vdisplay;
		if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
			struct mtk_panel_params *params;

			params = mtk_crtc->panel_ext->params;
			if (params->dyn_fps.switch_en == 1 &&
				params->dyn_fps.vact_timing_fps != 0)
				cfg.vrefresh =
					params->dyn_fps.vact_timing_fps;
			else
				cfg.vrefresh =
					drm_mode_vrefresh(&crtc->state->adjusted_mode);
		} else
			cfg.vrefresh =
				drm_mode_vrefresh(&crtc->state->adjusted_mode);
		cfg.bpc = mtk_crtc->bpc;
		cfg.p_golden_setting_context =
				__get_golden_setting_context(mtk_crtc);
	}

	mtk_ddp_comp_config(comp, &cfg, cmdq_handle);
}

static void mtk_crtc_wb_backup_to_slot(struct drm_crtc *crtc,
	struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_crtc_ddp_ctx *ddp_ctx = NULL;
	dma_addr_t addr;

	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_RDMA_FB_IDX);
	if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			addr, 0, ~0);
	} else {
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			addr, ddp_ctx->dc_fb_idx, ~0);
	}

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_RDMA_FB_ID);
	if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]) {
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			addr, state->prop_val[CRTC_PROP_OUTPUT_FB_ID], ~0);
	} else {
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			addr, ddp_ctx->dc_fb->base.id, ~0);
	}

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_CUR_OUTPUT_FENCE);
	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
		addr, state->prop_val[CRTC_PROP_OUTPUT_FENCE_IDX], ~0);

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_CUR_INTERFACE_FENCE);
	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
		addr, state->prop_val[CRTC_PROP_INTF_FENCE_IDX], ~0);
}

int mtk_crtc_gec_flush_check(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_ddp_comp *output_comp = NULL;

	output_comp = mtk_ddp_comp_request_output(mtk_crtc);

	if (output_comp) {
		switch (output_comp->id) {
		case DDP_COMPONENT_WDMA0:
		case DDP_COMPONENT_WDMA1:
		case DDP_COMPONENT_OVLSYS_WDMA2:
			if (!state->prop_val[CRTC_PROP_OUTPUT_ENABLE])
				return -EINVAL;
			break;
		default:
			break;
		}
	}

	return 0;
}

static struct disp_ccorr_config *mtk_crtc_get_color_matrix_data(
						struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	int blob_id;
	struct disp_ccorr_config *ccorr_config = NULL;
	struct drm_property_blob *blob;
	int *color_matrix;

	blob_id = state->prop_val[CRTC_PROP_COLOR_TRANSFORM];

	/* if blod_id == 0 means this time no new color matrix need to set */
	if (!blob_id)
		goto end;

	if (state->prop_val[CRTC_PROP_AOSP_CCORR_LINEAR])
		g_ccorr_linear = true;
	else
		g_ccorr_linear = false;

	blob = drm_property_lookup_blob(crtc->dev, blob_id);
	if (!blob) {
		DDPPR_ERR("Cannot get color matrix blob: %d!\n", blob_id);
		goto end;
	}

	ccorr_config = (struct disp_ccorr_config *)blob->data;
	drm_property_blob_put(blob);

	if (ccorr_config) {
		int i = 0, all_zero = 1;

		color_matrix = ccorr_config->color_matrix;
		for (i = 0; i <= 15; i += 5) {
			if (color_matrix[i] != 0) {
				all_zero = 0;
				break;
			}
		}
		if (all_zero) {
			DDPPR_ERR("HWC set zero color matrix!\n");
			goto end;
		}
	} else
		DDPPR_ERR("Blob cannot get ccorr_config data!\n");

end:
	return ccorr_config;
}

#ifdef IF_ZERO /* not ready for dummy register method */
static void mtk_crtc_backup_color_matrix_data(struct drm_crtc *crtc,
				struct disp_ccorr_config *ccorr_config,
				struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt_buffer *cmdq_buf = &(mtk_crtc->gce_obj.buf);
	dma_addr_t addr;
	int i;

	if (!ccorr_config)
		return;

	addr = cmdq_buf->pa_base + DISP_SLOT_COLOR_MATRIX_PARAMS(0);
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
			addr, ccorr_config->mode, ~0);

	for (i = 0; i < 16; i++) {
		addr = cmdq_buf->pa_base +
				DISP_SLOT_COLOR_MATRIX_PARAMS(i + 1);
		cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
		addr, ccorr_config->color_matrix[i], ~0);
	}
}
#endif

static void mtk_crtc_dl_config_color_matrix(struct drm_crtc *crtc,
				struct disp_ccorr_config *ccorr_config,
				struct cmdq_pkt *cmdq_handle)
{

	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	bool set =  false;
	int i, j;
	struct mtk_ddp_comp *comp_ccorr, *comp;

	if (!ccorr_config)
		return;

	for_each_comp_in_crtc_target_path(comp, mtk_crtc, i, DDP_FIRST_PATH) {
		if (comp->id == DDP_COMPONENT_CCORR0) {
			disp_ccorr_set_color_matrix(comp, cmdq_handle,
					ccorr_config->color_matrix, ccorr_config->mode,
					ccorr_config->featureFlag, g_ccorr_linear);
			set = true;
			break;
		}
	}
	if (mtk_crtc->is_dual_pipe) {
		i = 0;
		j = 0;
		for_each_comp_in_dual_pipe(comp_ccorr, mtk_crtc, i, j) {
			if (((drm_ccorr_caps.ccorr_number == 1) &&
				(comp_ccorr->id == DDP_COMPONENT_CCORR1)) ||
				((drm_ccorr_caps.ccorr_number == 2) &&
				(comp_ccorr->id == DDP_COMPONENT_CCORR2))) {
				disp_ccorr_set_color_matrix(comp_ccorr, cmdq_handle,
						ccorr_config->color_matrix, ccorr_config->mode,
						ccorr_config->featureFlag, g_ccorr_linear);
				set = true;
				break;
			}
		}
	}

#ifdef IF_ZERO /* not ready for dummy register method */
	if (set)
		mtk_crtc_backup_color_matrix_data(crtc, ccorr_config,
						cmdq_handle);
	else
#else
	if (!set)
#endif
		DDPPR_ERR("Cannot not find DDP_COMPONENT_CCORR0\n");
}

static void mtk_drm_discrete_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	int crtc_index = drm_crtc_index(cb_data->crtc);

	DDPINFO("[discrete] destroy hnd:0x%x\n", cb_data->cmdq_handle);
	CRTC_MMP_MARK(crtc_index, discrete, 0,
			(unsigned long)cb_data->cmdq_handle);
	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

static void mtk_drm_wb_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;
	struct drm_crtc *crtc = cb_data->crtc;
	int session_id;
	unsigned int fence_idx = cb_data->wb_fence_idx;

	/* fb reference conut will also have 1 after put */
//	drm_framebuffer_put(cb_data->wb_fb);

	session_id = mtk_get_session_id(crtc);
	mtk_crtc_release_output_buffer_fence_by_idx(crtc, session_id, fence_idx);

	CRTC_MMP_MARK(0, wbBmpDump, 1, fence_idx);
	mtk_dprec_mmp_dump_wdma_layer(crtc, cb_data->wb_fb);

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

int mtk_crtc_gce_flush(struct drm_crtc *crtc, void *gce_cb,
	void *cb_data, struct cmdq_pkt *cmdq_handle)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct disp_ccorr_config *ccorr_config = NULL;
	int is_from_dal = 0, event = 0;
	int crtc_index = drm_crtc_index(crtc);
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	struct mtk_cmdq_cb_data *wb_cb_data;
	struct mtk_cmdq_cb_data *discrete_cb_data;
	unsigned int r_comp_id;
	struct mtk_ddp_comp *first_comp, *r_comp;
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (mtk_crtc_gec_flush_check(crtc) < 0)	{
		if (cb_data) {
			struct drm_crtc_state *crtc_state;
			struct drm_atomic_state *atomic_state;

			crtc_state = ((struct mtk_cmdq_cb_data *)cb_data)->state;
			atomic_state = crtc_state->state;
			drm_atomic_state_put(atomic_state);
		}
		cmdq_pkt_destroy(cmdq_handle);
		kfree(cb_data);
		DDPPR_ERR("flush check failed\n");
		return -1;
	}

	/* if gce flush is form dal_show, we do not update and disconnect WDMA */
	/* because WDMA addon path is not connected */
	if (cb_data == cmdq_handle)
		is_from_dal = 1;

	/* apply color matrix if crtc0 is DL */
	ccorr_config = mtk_crtc_get_color_matrix_data(crtc);
	if (drm_crtc_index(crtc) == 0 && (!mtk_crtc_is_dc_mode(crtc)))
		mtk_crtc_dl_config_color_matrix(crtc, ccorr_config,
						cmdq_handle);

	//discrete mdp_rdma need fill full frame
	if (mtk_crtc->path_data->is_discrete_path) {
		first_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, 0, 0);
		if (first_comp != NULL) {
			mtk_ddp_comp_io_cmd(first_comp, cmdq_handle,
					MDP_RDMA_FILL_FRAME, NULL);
			if (mtk_crtc->is_dual_pipe) {
				r_comp_id = dual_pipe_comp_mapping(
						priv->data->mmsys_id, first_comp->id);
				r_comp = priv->ddp_comp[r_comp_id];
				mtk_ddp_comp_io_cmd(r_comp, cmdq_handle,
						MDP_RDMA_FILL_FRAME, NULL);
			}
		}
	}

	if (mtk_crtc_is_dc_mode(crtc) ||
		(state->prop_val[CRTC_PROP_OUTPUT_ENABLE] && crtc_index != 0)) {
		int gce_event =
			get_path_wait_event(mtk_crtc, mtk_crtc->ddp_mode);

		mtk_crtc_wb_comp_config(crtc, cmdq_handle);

		if (mtk_crtc_is_dc_mode(crtc))
			/* Decouple and Decouple mirror mode */
			mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[1],
					cmdq_handle, mtk_crtc->gce_obj.base);
		else {
			/* For virtual display write-back path */
			cmdq_pkt_clear_event(cmdq_handle, gce_event);
			mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0],
					cmdq_handle, mtk_crtc->gce_obj.base);
		}

		cmdq_pkt_wait_no_clear(cmdq_handle, gce_event);
	} else if (mtk_crtc_is_frame_trigger_mode(crtc) &&
					mtk_crtc_with_trigger_loop(crtc)) {
		if ((mtk_crtc->is_mml && is_from_dal) ||
				mtk_crtc->skip_frame) {
		} else {
			/* DL with trigger loop */
			cmdq_pkt_set_event(cmdq_handle,
					   mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
		}
	} else {
		/* DL without trigger loop */
		mtk_disp_mutex_enable_cmdq(mtk_crtc->mutex[0],
			cmdq_handle, mtk_crtc->gce_obj.base);
	}

	if (mtk_crtc_is_dc_mode(crtc) ||
		(state->prop_val[CRTC_PROP_OUTPUT_ENABLE] && crtc_index != 0)) {
		mtk_crtc_wb_backup_to_slot(crtc, cmdq_handle);

		/* backup color matrix for DC and DC Mirror for RDMA update*/
#ifdef IF_ZERO /* not ready for dummy register method */
		if (mtk_crtc_is_dc_mode(crtc))
			mtk_crtc_backup_color_matrix_data(crtc, ccorr_config,
							cmdq_handle);
#endif
	}

	mtk_crtc->skip_frame = false;
#ifdef MTK_DRM_CMDQ_ASYNC
#ifdef MTK_DRM_ASYNC_HANDLE
	if (gce_cb) {
		((struct mtk_cmdq_cb_data *)cb_data)->store_cb_data
			= kzalloc(sizeof(struct cb_data_store), GFP_KERNEL);
		if (!((struct mtk_cmdq_cb_data *)cb_data)->store_cb_data) {
			DDPPR_ERR("store_cb_data alloc fail\n");
			cmdq_pkt_destroy(cmdq_handle);
			kfree(cb_data);
			return -EINVAL;
		}
	}

	if (cmdq_pkt_flush_async(cmdq_handle, gce_cb, cb_data) < 0)
		DDPPR_ERR("failed to flush gce_cb async\n");
#else
	if (cmdq_pkt_flush_threaded(cmdq_handle, gce_cb, cb_data) < 0)
		DDPPR_ERR("failed to flush gce_cb threaded\n");
#endif
#else
	cmdq_pkt_flush(cmdq_handle);
#endif

	//discrete flush pending job
	if (mtk_crtc->path_data->is_discrete_path &&
		mtk_crtc->pending_handle) {
		struct cmdq_pkt *pending_handle;

		pending_handle = mtk_crtc->pending_handle;
		mtk_crtc->pending_handle = NULL;
		discrete_cb_data = kmalloc(sizeof(*discrete_cb_data), GFP_KERNEL);
		if (discrete_cb_data != NULL) {
			discrete_cb_data->cmdq_handle = pending_handle;
			discrete_cb_data->crtc = crtc;

			DDPINFO("[discrete] flush pending hnd:0x%x\n",
					pending_handle);
			CRTC_MMP_MARK(crtc_index, discrete, 0,
					(unsigned long)pending_handle);
			cmdq_pkt_flush_threaded(pending_handle,
					mtk_drm_discrete_cb, discrete_cb_data);
		}
	}

	/* For DL write-back path */
	/* wait WDMA frame done and disconnect immediately */
	if (state->prop_val[CRTC_PROP_OUTPUT_ENABLE]
		&& crtc_index == 0 && !is_from_dal) {
		wb_cb_data = kmalloc(sizeof(*wb_cb_data), GFP_KERNEL);

		event = mtk_crtc_wb_addon_get_event(crtc);
		mtk_crtc_pkt_create(&handle, crtc, client);

		if (!handle) {
			DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
			return -EINVAL;
		}

		cmdq_pkt_clear_event(handle, event);
		cmdq_pkt_wfe(handle, event);
		_mtk_crtc_wb_addon_module_disconnect(crtc, mtk_crtc->ddp_mode, handle);

		wb_cb_data->cmdq_handle = handle;
		wb_cb_data->crtc = crtc;
		wb_cb_data->wb_fb =
			mtk_drm_framebuffer_lookup(crtc->dev,
			state->prop_val[CRTC_PROP_OUTPUT_FB_ID]);
		wb_cb_data->wb_fence_idx = state->prop_val[CRTC_PROP_OUTPUT_FENCE_IDX];
		CRTC_MMP_MARK(crtc_index, wbBmpDump, (unsigned long)handle,
							wb_cb_data->wb_fence_idx);
		cmdq_pkt_flush_threaded(handle, mtk_drm_wb_cb, wb_cb_data);
	}

	return 0;
}

static int mtk_drm_crtc_find_ovl_comp_id(struct mtk_drm_private *priv,
	int is_ovl_2l, int is_dual_pipe)
{
	const struct mtk_crtc_path_data *path_data;
	const enum mtk_ddp_comp_id *path;
	unsigned int path_len;
	int start_id, end_id;
	int i, comp_id;

	path_data = priv->data->main_path_data != NULL ? priv->data->main_path_data : NULL;
	if (path_data == NULL)
		return -1;
	if (is_dual_pipe) {/* dual pipe path */
		path = path_data->dual_path[0];
		path_len = path_data->dual_path_len[0];
	} else { /* main path */
		path = path_data->path[DDP_MAJOR][0];
		path_len = path_data->path_len[DDP_MAJOR][0];
	}
	if (is_ovl_2l) { /* ovl_2l */
		start_id = DDP_COMPONENT_OVL0_2L;
		end_id = DDP_COMPONENT_OVL6_2L;
	} else { /* ovl */
		start_id = DDP_COMPONENT_OVL0;
		end_id = DDP_COMPONENT_OVL2;
	}
	comp_id = -1;
	for (i = 0; i < path_len; i++) {
		if (path[i] >= start_id && path[i] <= end_id) {
			comp_id = path[i];
			break;
		}
	}
	DDPINFO("%s comp_id:%d, is_ovl_2l %d, is_dual_pipe %d\n", __func__,
		comp_id, is_ovl_2l, is_dual_pipe);
	return comp_id;
}

bool mtk_crtc_check_is_scaling_comp(struct mtk_drm_crtc *mtk_crtc,
	enum mtk_ddp_comp_id comp_id)
{
	struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	const struct mtk_crtc_path_data *path_data;
	enum mtk_ddp_comp_id scaling_comp_id, scaling_comp_dual_id;
	bool ret = false;

	if (drm_crtc_index(&mtk_crtc->base) != 0)
		return false;

	path_data = priv->data->main_path_data != NULL ? priv->data->main_path_data : NULL;
	if ((path_data == NULL) || (path_data->scaling_data == NULL))
		return false;

	scaling_comp_id = path_data->scaling_data[0];

	if (mtk_crtc->is_dual_pipe && (path_data->scaling_data_dual != NULL)) {
		scaling_comp_dual_id = path_data->scaling_data_dual[0];
		ret = ((comp_id == scaling_comp_id) || (comp_id == scaling_comp_dual_id));
	} else
		ret = (comp_id == scaling_comp_id);

	return ret;
}

void mtk_crtc_divide_default_path_by_rsz(struct mtk_drm_crtc *mtk_crtc)
{
	struct mtk_ddp_comp *comp;
	int i, j;
	bool in_scaling_path = false;

	if ((drm_crtc_index(&mtk_crtc->base) != 0) ||
		(mtk_crtc->res_switch != RES_SWITCH_ON_AP))
		return;

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
			in_scaling_path = true;
		comp->in_scaling_path = in_scaling_path;
		DDPDBG("%s: %s is %s scaling path\n", __func__,
			mtk_dump_comp_str(comp), (comp->in_scaling_path ? "in" : "before"));
	}

	in_scaling_path = false;
	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			if (mtk_crtc_check_is_scaling_comp(mtk_crtc, comp->id))
				in_scaling_path = true;
			comp->in_scaling_path = in_scaling_path;
			DDPDBG("%s: %s is %s scaling path\n", __func__,
				mtk_dump_comp_str(comp), (comp->in_scaling_path ? "in" : "before"));
		}
	}
}

struct drm_display_mode *mtk_crtc_get_display_mode_by_comp(
	const char *source,
	struct drm_crtc *crtc,
	struct mtk_ddp_comp *comp,
	bool in_scaling_path)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_display_mode *mode;

	if ((crtc == NULL) || (crtc->state == NULL) || (mtk_crtc == NULL)) {
		DDPPR_ERR("%s null pointer!!\n", __func__);
		dump_stack();
		return NULL;
	}

	if (mtk_crtc->scaling_ctx.scaling_en
			&& (in_scaling_path || ((comp != NULL) && comp->in_scaling_path))
			&& (mtk_crtc->scaling_ctx.scaling_mode != NULL))
		mode = mtk_crtc->scaling_ctx.scaling_mode;
	else
		mode = &crtc->state->adjusted_mode;

	DDPDBG("%s scaling_en: %d, comp: %s, in_scaling_path: %d, mode: %ux%u@%d\n", source,
		mtk_crtc->scaling_ctx.scaling_en,
		(comp != NULL) ? mtk_dump_comp_str(comp) : "unknown",
		(((comp != NULL) && comp->in_scaling_path) || in_scaling_path),
		mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));

	return mode;
}

unsigned int mtk_crtc_get_width_by_comp(
	const char *source,
	struct drm_crtc *crtc,
	struct mtk_ddp_comp *comp,
	bool in_scaling_path)
{
	struct drm_display_mode *mode;

	mode = mtk_crtc_get_display_mode_by_comp(source, crtc, comp, in_scaling_path);
	if (mode == NULL) {
		DDPPR_ERR("%s null pointer!!\n", __func__);
		return 0;
	}

	return mode->hdisplay;
}

unsigned int mtk_crtc_get_height_by_comp(
	const char *source,
	struct drm_crtc *crtc,
	struct mtk_ddp_comp *comp,
	bool in_scaling_path)
{
	struct drm_display_mode *mode;

	mode = mtk_crtc_get_display_mode_by_comp(source, crtc, comp, in_scaling_path);
	if (mode == NULL) {
		DDPPR_ERR("%s null pointer!!\n", __func__);
		return 0;
	}

	return mode->vdisplay;
}

void mtk_crtc_set_width_height(
	int *w,
	int *h,
	struct drm_crtc *crtc,
	bool is_scaling_path)
{
	*w = mtk_crtc_get_width_by_comp(__func__, crtc, NULL, is_scaling_path);
	*h = mtk_crtc_get_height_by_comp(__func__, crtc, NULL, is_scaling_path);
}

static void mtk_drm_crtc_enable_fake_layer(struct drm_crtc *crtc,
				      struct drm_crtc_state *old_crtc_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_plane *plane;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_plane_state *plane_state;
	struct mtk_plane_pending_state *pending;
	struct mtk_ddp_comp *comp;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	struct mtk_drm_fake_layer *fake_layer = &mtk_crtc->fake_layer;
	int i, idx, layer_num, layer_num2;
	int ovl_comp_id, ovl_2l_comp_id;
	int dual_pipe_ovl_comp_id, dual_pipe_ovl_2l_comp_id;

	if (drm_crtc_index(crtc) != 0)
		return;

	DDPINFO("%s\n", __func__);

	ovl_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 0, 0);
	if (ovl_comp_id < 0)
		ovl_comp_id = DDP_COMPONENT_OVL0;
	ovl_2l_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 1, 0);
	if (ovl_2l_comp_id < 0)
		ovl_2l_comp_id = DDP_COMPONENT_OVL0_2L;
	if (mtk_crtc->is_dual_pipe) {
		dual_pipe_ovl_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 0, 1);
		if (dual_pipe_ovl_comp_id < 0)
			dual_pipe_ovl_comp_id = DDP_COMPONENT_OVL1;
		dual_pipe_ovl_2l_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 1, 1);
		if (dual_pipe_ovl_2l_comp_id < 0)
			dual_pipe_ovl_2l_comp_id = DDP_COMPONENT_OVL1_2L;
	}

	for (i = 0 ; i < PRIMARY_OVL_PHY_LAYER_NR ; i++) {
		plane = &mtk_crtc->planes[i].base;
		plane_state = to_mtk_plane_state(plane->state);
		pending = &plane_state->pending;

		pending->addr = mtk_fb_get_dma(fake_layer->fake_layer_buf[i]);
		pending->size = mtk_fb_get_size(fake_layer->fake_layer_buf[i]);
		pending->pitch = fake_layer->fake_layer_buf[i]->pitches[0];
		pending->format = fake_layer->fake_layer_buf[i]->format->format;
		pending->modifier = fake_layer->fake_layer_buf[i]->modifier;
		pending->src_x = 0;
		pending->src_y = 0;
		pending->dst_x = 0;
		pending->dst_y = 0;
		pending->height = fake_layer->fake_layer_buf[i]->height;
		pending->width = fake_layer->fake_layer_buf[i]->width;
		pending->config = 1;
		pending->dirty = 1;

		if (mtk_crtc->fake_layer.fake_layer_mask & BIT(i))
			pending->enable = true;
		else
			pending->enable = false;

		pending->prop_val[PLANE_PROP_ALPHA_CON] = 0x1;
		pending->prop_val[PLANE_PROP_PLANE_ALPHA] = 0xFF;
		pending->prop_val[PLANE_PROP_COMPRESS] = 0;

		layer_num = mtk_ovl_layer_num(
				priv->ddp_comp[ovl_2l_comp_id]);
		if (layer_num < 0) {
			DDPPR_ERR("invalid layer num:%d\n", layer_num);
			continue;
		}

		if (priv->data->mmsys_id == MMSYS_MT6985) {
			layer_num2 = mtk_ovl_layer_num(
					priv->ddp_comp[DDP_COMPONENT_OVL1_2L]);

			if (layer_num2 < 0) {
				DDPPR_ERR("invalid layer num:%d\n", layer_num2);
				continue;
			}
			if (i < layer_num) {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL0_2L];
				idx = i;
			} else if (i < layer_num + layer_num2) {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL1_2L];
				idx = i - layer_num;
			} else {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL2_2L];
				idx = i - layer_num - layer_num2;
			}
		} else {
			if (i < layer_num) {
				comp = priv->ddp_comp[ovl_2l_comp_id];
				idx = i;
			} else {
				comp = priv->ddp_comp[ovl_comp_id];
				idx = i - layer_num;
			}
		}

		plane_state->comp_state.comp_id = comp->id;
		plane_state->comp_state.lye_id = idx;
		plane_state->comp_state.ext_lye_id = 0;

		if (mtk_crtc->is_dual_pipe)
			mtk_crtc_dual_layer_config(mtk_crtc, comp, plane_state->comp_state.lye_id,
						plane_state, state->cmdq_handle);
		else
			mtk_ddp_comp_layer_config(comp, plane_state->comp_state.lye_id,
						plane_state, state->cmdq_handle);
	}

	if (!mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_OVL_EXT_LAYER))
		return;

	for (i = 0 ; i < PRIMARY_OVL_EXT_LAYER_NR ; i++) {
		plane = &mtk_crtc->planes[i + PRIMARY_OVL_PHY_LAYER_NR].base;
		plane_state = to_mtk_plane_state(plane->state);
		pending = &plane_state->pending;

		pending->dirty = 1;
		pending->enable = false;

		if (i < (PRIMARY_OVL_EXT_LAYER_NR / 2)) {
			comp = priv->ddp_comp[ovl_2l_comp_id];
			idx = i + 1;
		} else {
			comp = priv->ddp_comp[ovl_comp_id];
			idx = i + 1 - (PRIMARY_OVL_EXT_LAYER_NR / 2);
		}
		plane_state->comp_state.comp_id = comp->id;
		plane_state->comp_state.lye_id = 0;
		plane_state->comp_state.ext_lye_id = idx;

		if (mtk_crtc->is_dual_pipe)
			mtk_crtc_dual_layer_config(mtk_crtc, comp, plane_state->comp_state.lye_id,
						plane_state, state->cmdq_handle);
		else
			mtk_ddp_comp_layer_config(comp, plane_state->comp_state.lye_id,
						plane_state, state->cmdq_handle);
	}
}

static void mtk_drm_crtc_disable_fake_layer(struct drm_crtc *crtc,
				struct drm_crtc_state *old_crtc_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_plane *plane;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_plane_state *plane_state;
	struct mtk_plane_pending_state *pending;
	struct mtk_ddp_comp *comp;
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);
	int i, idx, layer_num, layer_num2;
	int ovl_comp_id, ovl_2l_comp_id;
	int dual_pipe_ovl_comp_id, dual_pipe_ovl_2l_comp_id;

	if (drm_crtc_index(crtc) != 0)
		return;

	DDPINFO("%s\n", __func__);

	ovl_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 0, 0);
	if (ovl_comp_id < 0)
		ovl_comp_id = DDP_COMPONENT_OVL0;
	ovl_2l_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 1, 0);
	if (ovl_2l_comp_id < 0 || ovl_2l_comp_id >= DDP_COMPONENT_ID_MAX)
		ovl_2l_comp_id = DDP_COMPONENT_OVL0_2L;
	if (mtk_crtc->is_dual_pipe) {
		dual_pipe_ovl_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 0, 1);
		if (dual_pipe_ovl_comp_id < 0)
			dual_pipe_ovl_comp_id = DDP_COMPONENT_OVL1;
		dual_pipe_ovl_2l_comp_id = mtk_drm_crtc_find_ovl_comp_id(priv, 1, 1);
		if (dual_pipe_ovl_2l_comp_id < 0)
			dual_pipe_ovl_2l_comp_id = DDP_COMPONENT_OVL1_2L;
	}

	for (i = 0 ; i < PRIMARY_OVL_PHY_LAYER_NR ; i++) {
		plane = &mtk_crtc->planes[i].base;
		plane_state = to_mtk_plane_state(plane->state);
		pending = &plane_state->pending;

		pending->dirty = 1;
		pending->enable = false;

		layer_num = mtk_ovl_layer_num(
				priv->ddp_comp[ovl_2l_comp_id]);
		if (layer_num < 0) {
			DDPPR_ERR("invalid layer num:%d\n", layer_num);
			continue;
		}

		if (priv->data->mmsys_id == MMSYS_MT6985) {
			layer_num2 = mtk_ovl_layer_num(
					priv->ddp_comp[DDP_COMPONENT_OVL1_2L]);

			if (layer_num2 < 0) {
				DDPPR_ERR("invalid layer num:%d\n", layer_num2);
				continue;
			}
			if (i < layer_num) {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL0_2L];
				idx = i;
			} else if (i < layer_num + layer_num2) {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL1_2L];
				idx = i - layer_num;
			} else {
				comp = priv->ddp_comp[DDP_COMPONENT_OVL2_2L];
				idx = i - layer_num - layer_num2;
			}
		} else {

			if (i < layer_num) {
				comp = priv->ddp_comp[ovl_2l_comp_id];
				idx = i;
			} else {
				comp = priv->ddp_comp[ovl_comp_id];
				idx = i - layer_num;
			}
		}
		plane_state->comp_state.comp_id = comp->id;
		plane_state->comp_state.lye_id = idx;
		plane_state->comp_state.ext_lye_id = 0;

		if (mtk_crtc->is_dual_pipe)
			mtk_crtc_dual_layer_config(mtk_crtc, comp, plane_state->comp_state.lye_id,
					plane_state, state->cmdq_handle);
		else
			mtk_ddp_comp_layer_config(comp, plane_state->comp_state.lye_id,
					plane_state, state->cmdq_handle);
	}
}

static void msync_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

//keep pq when hbm on @{
/*
static void mtk_atomic_hbm_bypass_pq(struct drm_crtc *crtc,
		struct cmdq_pkt *handle, int en)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i, j;

	DDPINFO("%s: enter en[%d]\n", __func__, en);

	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		if (comp && (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_AAL ||
				mtk_ddp_comp_get_type(comp->id) == MTK_DISP_CCORR)) {
			if (comp->funcs && comp->funcs->bypass)
				mtk_ddp_comp_bypass(comp, en, handle);
		}
	}

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			if (comp && (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_AAL ||
					mtk_ddp_comp_get_type(comp->id) == MTK_DISP_CCORR)) {
				if (comp->funcs && comp->funcs->bypass)
					mtk_ddp_comp_bypass(comp, en, handle);
			}
		}
	}
}
*/
//@}

#ifdef IF_ZERO /* not ready for dummy register method */
static void sf_cmdq_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}
#endif

static void mtk_drm_crtc_atomic_flush(struct drm_crtc *crtc,
				      struct drm_atomic_state *atomic_state)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	unsigned int index = drm_crtc_index(crtc);
	unsigned int pending_planes = 0;
	unsigned int i, j;
#ifndef DRM_CMDQ_DISABLE
	unsigned int ret = 0;
	unsigned long cmdq_handle_cache = 0;
	unsigned long cmdq_handle_size_cache = 0;
#endif
	struct drm_crtc_state *crtc_state = crtc->state;
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc_state);
	struct drm_crtc_state *old_crtc_state =
		drm_atomic_get_old_crtc_state(atomic_state, crtc);
	struct cmdq_pkt *cmdq_handle = mtk_crtc_state->cmdq_handle;
	struct mtk_cmdq_cb_data *cb_data;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_crtc *mtk_crtc0 = to_mtk_crtc(priv->crtc[0]);
	/*Msync 2.0*/
	struct mtk_crtc_state *old_mtk_state =
		to_mtk_crtc_state(old_crtc_state);
	struct mtk_panel_params *params =
			mtk_drm_get_lcm_ext_params(crtc);
	bool need_disable = false;
	bool only_output = false;

	CRTC_MMP_EVENT_START((int) index, atomic_flush, (unsigned long)crtc_state,
			(unsigned long)old_crtc_state);
	if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK((int) index, atomic_flush, 0, 0);
		goto end;
	}

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT) &&
			index < MAX_CRTC && priv->usage[index] == DISP_OPENING) {
		struct mtk_ddp_comp *output_comp = mtk_ddp_comp_request_output(mtk_crtc);

		if (!(output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DISP_WDMA)) {
			DDPINFO("%s: %d skip in opening\n", __func__, index);
			CRTC_MMP_MARK((int) index, atomic_flush, 0, __LINE__);
			/* release all fence when opening CRTC */
			mtk_drm_crtc_release_fence(crtc);
			goto end;
		} else {
			only_output = true;
		}
	}

	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("cb data creation failed\n");
		CRTC_MMP_MARK((int) index, atomic_flush, 0, 1);
		goto end;
	}

	if (mtk_crtc->event)
		mtk_crtc->pending_needs_vblank = true;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i].base;
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->pending.dirty) {
			plane_state->pending.config = true;
			plane_state->pending.dirty = false;
			pending_planes |= BIT(i);
		}
	}

	if (pending_planes)
		mtk_crtc->pending_planes = true;

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_HBM)) {
		bool hbm_en = false;

		hbm_en = (bool)mtk_crtc_state->prop_val[CRTC_PROP_HBM_ENABLE];
		mtk_drm_crtc_set_panel_hbm(crtc, hbm_en);
		mtk_drm_crtc_hbm_wait(crtc, hbm_en);

		//keep pq when hbm on @{
		//if (!mtk_crtc_state->prop_val[CRTC_PROP_DOZE_ACTIVE])
			//mtk_atomic_hbm_bypass_pq(crtc, cmdq_handle, hbm_en);
		//@}
	}

	if (need_brightness_sync) {
		uint32_t value = (uint32_t)mtk_crtc_state->prop_val[CRTC_PROP_NT_HDR_BRIGHTNESS];
		if (brightness_now != value) {
			level_pending = value;
			mtk_drm_trace_begin("setbacklight: %d", value);
			schedule_work(&mtk_crtc->delay_set_brightness_work);
			mtk_drm_trace_end();
		}
	}

	hdr_en = (bool)mtk_crtc_state->prop_val[CRTC_PROP_HDR_ENABLE];

	if (mtk_crtc->fake_layer.fake_layer_mask)
		mtk_drm_crtc_enable_fake_layer(crtc, old_crtc_state);
	else if (mtk_crtc->fake_layer.first_dis) {
		mtk_drm_crtc_disable_fake_layer(crtc, old_crtc_state);
		mtk_crtc->fake_layer.first_dis = false;
	}

	if (!only_output) {
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
			if (crtc->state->color_mgmt_changed)
				mtk_ddp_gamma_set(comp, crtc->state, cmdq_handle);
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					COMP_ODDMR_CFG, &mtk_crtc->sec_on);
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					PMQOS_UPDATE_BW, NULL);
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
					FRAME_DIRTY, NULL);
		}
		if (mtk_crtc->is_dual_pipe) {
			for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
				if (crtc->state->color_mgmt_changed)
					mtk_ddp_gamma_set(comp, crtc->state, cmdq_handle);
				mtk_ddp_comp_io_cmd(comp, cmdq_handle,
						COMP_ODDMR_CFG, &mtk_crtc->sec_on);
				mtk_ddp_comp_io_cmd(comp, cmdq_handle,
						PMQOS_UPDATE_BW, NULL);
				mtk_ddp_comp_io_cmd(comp, cmdq_handle,
						FRAME_DIRTY, NULL);
			}
		}
	}

	if ((priv->data->shadow_register) == true) {
		mtk_disp_mutex_acquire(mtk_crtc->mutex[0]);
		mtk_crtc_ddp_config(crtc);
		mtk_disp_mutex_release(mtk_crtc->mutex[0]);
	}

	/* backup present fence */
	if (mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX] != (unsigned int)-1) {
		dma_addr_t addr =
			mtk_get_gce_backup_slot_pa(mtk_crtc,
			DISP_SLOT_PRESENT_FENCE(index));

		cmdq_pkt_write(cmdq_handle,
			mtk_crtc->gce_obj.base, addr,
			mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX], ~0);
		CRTC_MMP_MARK((int) index, update_present_fence, 0,
			mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX]);
	}

	/* for wfd latency debug */
	if (index == 0 || index == 2) {
		dma_addr_t addr = (index == 0) ?
			mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_OVL_DSI_SEQ) :
			mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_OVL_WDMA_SEQ);

		cmdq_pkt_write(cmdq_handle,
			mtk_crtc->gce_obj.base, addr,
			mtk_crtc_state->prop_val[CRTC_PROP_OVL_DSI_SEQ], ~0);

		if (mtk_crtc_state->prop_val[CRTC_PROP_OVL_DSI_SEQ]) {
			if (index == 0)
				mtk_drm_trace_async_begin("OVL0-DSI|%d",
					mtk_crtc_state->prop_val[CRTC_PROP_OVL_DSI_SEQ]);
			else if (index == 2)
				mtk_drm_trace_async_begin("OVL2-WDMA|%d",
			mtk_crtc_state->prop_val[CRTC_PROP_OVL_DSI_SEQ]);
		}
	}

	atomic_set(&mtk_crtc->delayed_trig, 1);
	cb_data->state = old_crtc_state;
	cb_data->cmdq_handle = cmdq_handle;
	cb_data->misc = mtk_crtc->ddp_mode;
	cb_data->msync2_enable = 0;
	cb_data->is_mml = mtk_crtc->is_mml;

	if (mtk_crtc_state->prop_val[CRTC_PROP_LYE_IDX] != (unsigned int)-1)
		cb_data->hrt_idx = mtk_crtc_state->prop_val[CRTC_PROP_LYE_IDX];

	if (mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX] != (unsigned int)-1)
		cb_data->pres_fence_idx = mtk_crtc_state->prop_val[CRTC_PROP_PRES_FENCE_IDX];

	/* This refcnt would be release in ddp_cmdq_cb */
	drm_atomic_state_get(old_crtc_state->state);
	mtk_drm_crtc_lfr_update(crtc, cmdq_handle);
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SF_PF) &&
	   (mtk_crtc_state->prop_val[CRTC_PROP_SF_PRES_FENCE_IDX] != (unsigned int)-1)) {
		if (index == 0)
			cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_DSI_SOF]);
	}

	/*Msync 2.0*/
	if (!mtk_crtc_is_frame_trigger_mode(crtc) &&
			msync_is_on(priv, params, index,
				mtk_crtc_state, old_mtk_state) &&
			!mtk_crtc->msync2.msync_disabled) {
		/*VFP early stop 0->1*/
		struct mtk_ddp_comp *output_comp;
		u32 vfp_early_stop = 1;

		output_comp = mtk_ddp_comp_request_output(mtk_crtc);
		if (output_comp)
			mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
					DSI_VFP_EARLYSTOP, &vfp_early_stop);
		/*clear EOF*/
		/* Avoid other operation during VFP after vfp ealry stop*/
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_CMD_EOF]);
		/*clear vfp period sw token*/
		cmdq_pkt_clear_event(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_SYNC_TOKEN_VFP_PERIOD]);
		cb_data->msync2_enable = 1;

		DDPDBG("[Msync]cmdq pkt size = %d\n", cmdq_handle->cmd_buf_size);
		if (cmdq_handle->cmd_buf_size >= 4096) {
			/*ToDo: if larger than 4096 need consider change pages*/
			DDPPR_ERR("[Msync]cmdq pkt size = %d\n", cmdq_handle->cmd_buf_size);
		}
	}

	/* backup ovl0 2l status for crtc0 */
	if (index == 0) {
		comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_OVL0_2L);
		if (IS_ERR_OR_NULL(comp)) {
			comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_OVL1_2L);
			if (IS_ERR_OR_NULL(comp)) {
				comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_OVL0);
				if (IS_ERR_OR_NULL(comp))
					comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_OVL1);
			}
		}
		if (IS_ERR_OR_NULL(comp))
			DDPMSG("%s: failed to backup ovl status\n", __func__);
		else
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				BACKUP_OVL_STATUS, NULL);
	}

	/* need to check mml is submit done */
	if (mtk_crtc->is_mml)
		mtk_drm_wait_mml_submit_done(&(mtk_crtc->mml_cb));

#if IS_ENABLED(CONFIG_MTK_DISP_DEBUG)
	if (g_wr_reg.after_commit == 1) {
		int k;

		for (k = 0; k < g_wr_reg.index; k++) {
			DDPINFO("[reg_dbg] reg_wr: addr:0x%x, val:0x%x, mask:0x%x\n",
				g_wr_reg.reg[k].addr, g_wr_reg.reg[k].val, g_wr_reg.reg[k].mask);
			cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
				g_wr_reg.reg[k].addr, g_wr_reg.reg[k].val, g_wr_reg.reg[k].mask);
		}
	}
#endif

	mtk_drm_idlemgr_kick(__func__, crtc, false); /* update kick timestamp */

#ifndef DRM_CMDQ_DISABLE
#ifdef MTK_DRM_CMDQ_ASYNC
	cmdq_handle_cache = (unsigned long)cmdq_handle;
	cmdq_handle_size_cache = (unsigned long)cmdq_handle->cmd_buf_size;
	ret = mtk_crtc_gce_flush(crtc, ddp_cmdq_cb, cb_data, cmdq_handle);
	if (ret) {
		DDPPR_ERR("mtk_crtc_gce_flush failed!\n");
		goto end;
	}
	CRTC_MMP_MARK((int) index, atomic_flush, cmdq_handle_cache,
			cmdq_handle_size_cache);
#else
	ret = mtk_crtc_gce_flush(crtc, NULL, NULL, cmdq_handle);
	if (ret) {
		DDPPR_ERR("mtk_crtc_gce_flush failed!\n");
		goto end;
	}
	ddp_cmdq_cb_blocking(cb_data);
#endif
#endif

	/*Msync 2.0*/
	if (!mtk_crtc_is_frame_trigger_mode(crtc) &&
			msync_is_on(priv, params, index,
				mtk_crtc_state, old_mtk_state) &&
			!mtk_crtc->msync2.msync_disabled) {
		struct cmdq_pkt *cmdq_handle;
		struct mtk_cmdq_cb_data *msync_cb_data;
		struct mtk_ddp_comp *output_comp;

		cmdq_handle =
			cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			goto end;
		}

		msync_cb_data = kmalloc(sizeof(*msync_cb_data), GFP_KERNEL);
		if (!msync_cb_data) {
			DDPPR_ERR("cb data creation failed\n");
			CRTC_MMP_MARK((int) index, atomic_flush, 1, 1);
			goto end;
		}
		output_comp = mtk_ddp_comp_request_output(mtk_crtc);
		/*add wait SOF cmd*/
		cmdq_pkt_wait_no_clear(cmdq_handle,
				mtk_crtc->gce_obj.event[EVENT_DSI_SOF]);
		if (mtk_crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] != 0)
			need_disable = msync_need_disable(mtk_crtc);
		/* if 1->0 or msync_need_disable*/
		if (mtk_crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] == 0 ||
				need_disable) {
			/* disable msync and enable LFR*/
			if (output_comp) {
				mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
						DSI_DISABLE_VFP_EALRY_STOP, NULL);
				mtk_crtc->msync2.msync_on = false;
			}
			if (need_disable) {
				mtk_crtc->msync2.msync_disabled = true;
				DDPINFO("[Msync] msync_disabled = true\n");
			}
			if (mtk_crtc->msync2.LFR_disabled &&
					mtk_crtc_state->prop_val[CRTC_PROP_MSYNC2_0_ENABLE] == 0) {
				int en = 1;

				mtk_drm_helper_set_opt_by_name(priv->helper_opt,
						"MTK_DRM_OPT_LFR", en);
				if (atomic_read(&mtk_crtc->msync2.LFR_final_state) != 0)
					mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
							DSI_LFR_SET, &en);
				mtk_crtc->msync2.LFR_disabled = false;
			}
		} else { /*if 0->1 or 1->1*/
			/* restore VFP*/
			if (output_comp)
				mtk_ddp_comp_io_cmd(output_comp, cmdq_handle,
						DSI_RESTORE_VFP_FOR_MSYNC, NULL);
		}
		msync_cb_data->cmdq_handle = cmdq_handle;
		if (cmdq_pkt_flush_threaded(cmdq_handle,
			msync_cmdq_cb, msync_cb_data) < 0)
			DDPPR_ERR("failed to flush msync_cmdq_cb\n");
	}

	/* When open VDS path switch feature, After VDS created
	 * we need take away the OVL0_2L from main display.
	 */
	if (mtk_drm_helper_get_opt(priv->helper_opt,
		MTK_DRM_OPT_VDS_PATH_SWITCH) &&
		priv->vds_path_switch_dirty &&
		!priv->vds_path_switch_done) {
		if ((index == 0) && atomic_read(&mtk_crtc0->already_config)) {
			DDPMSG("Switch vds: mtk_crtc0 enable:%d\n",
				atomic_read(&mtk_crtc0->already_config));
			mtk_need_vds_path_switch(crtc);
		}

		if ((index == 2) && (!atomic_read(&mtk_crtc0->already_config))) {
			DDPMSG("Switch vds: mtk_crtc0 enable:%d\n",
				atomic_read(&mtk_crtc0->already_config));
			mtk_need_vds_path_switch(priv->crtc[0]);
		}
	}

end:
#ifdef DRM_CMDQ_DISABLE
	trigger_without_cmdq(crtc);
#endif
	CRTC_MMP_EVENT_END((int) index, atomic_flush, (unsigned long)crtc_state,
			(unsigned long)old_crtc_state);
	mtk_drm_trace_end();
	ktime_get_real_ts64(&atomic_flush_tval);
}

static const struct drm_crtc_funcs mtk_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = mtk_drm_crtc_destroy,
	.reset = mtk_drm_crtc_reset,
	.atomic_duplicate_state = mtk_drm_crtc_duplicate_state,
	.atomic_destroy_state = mtk_drm_crtc_destroy_state,
	.atomic_set_property = mtk_drm_crtc_set_property,
	.atomic_get_property = mtk_drm_crtc_get_property,
	.enable_vblank = mtk_drm_crtc_enable_vblank,
	.disable_vblank = mtk_drm_crtc_disable_vblank,
	.get_vblank_timestamp = mtk_crtc_get_vblank_timestamp,
};

static const struct drm_crtc_helper_funcs mtk_crtc_helper_funcs = {
	.mode_fixup = mtk_drm_crtc_mode_fixup,
	.mode_set_nofb = mtk_drm_crtc_mode_set_nofb,
	.atomic_enable = mtk_drm_crtc_atomic_resume,
	.disable = mtk_drm_crtc_suspend,
	.atomic_begin = mtk_drm_crtc_atomic_begin,
	.atomic_flush = mtk_drm_crtc_atomic_flush,
};

static void mtk_drm_set_crtc_caps(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct mtk_drm_crtc_caps crtc_caps;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_property_blob *blob;
	uint32_t blob_id = 0;

	memset(&crtc_caps, 0, sizeof(crtc_caps));

	blob = drm_property_create_blob(dev, sizeof(crtc_caps), &mtk_crtc->crtc_caps);
	if (!IS_ERR_OR_NULL(blob))
		blob_id = blob->base.id;
	else
		DDPPR_ERR("create_blob error\n");

	mtk_crtc_property[CRTC_PROP_CAPS_BLOB_ID].val = blob_id;
}

int mtk_crtc_ability_chk(struct mtk_drm_crtc *mtk_crtc, enum MTK_CRTC_ABILITY ability)
{
	return !!(mtk_crtc->crtc_caps.crtc_ability & ability);
}

static void mtk_drm_crtc_attach_property(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct mtk_drm_private *private = dev->dev_private;
	struct drm_property *prop;
	static struct drm_property *mtk_crtc_prop[CRTC_PROP_MAX];
	struct mtk_drm_property *crtc_prop;
	int index = drm_crtc_index(crtc);
	int i;
	static int num;

	DDPINFO("%s:%d crtc:%d\n", __func__, __LINE__, index);

	if (num == 0) {
		for (i = 0; i < CRTC_PROP_MAX; i++) {
			crtc_prop = &(mtk_crtc_property[i]);
			mtk_crtc_prop[i] = drm_property_create_range(
				dev, crtc_prop->flags, crtc_prop->name,
				crtc_prop->min, crtc_prop->max);
			if (!mtk_crtc_prop[i]) {
				DDPPR_ERR("fail to create property:%s\n",
					  crtc_prop->name);
				return;
			}
			DDPINFO("create property:%s, flags:0x%x\n",
				crtc_prop->name, mtk_crtc_prop[i]->flags);
		}
		num++;
	}

	mtk_drm_set_crtc_caps(crtc);

	for (i = 0; i < CRTC_PROP_MAX; i++) {
		prop = private->crtc_property[index][i];
		crtc_prop = &(mtk_crtc_property[i]);
		DDPINFO("%s:%d prop:%p\n", __func__, __LINE__, prop);
		if (!prop) {
			prop = mtk_crtc_prop[i];
		      private
			->crtc_property[index][i] = prop;
			drm_object_attach_property(&crtc->base, prop,
						   crtc_prop->val);
		}
	}
}

static int mtk_drm_crtc_init(struct drm_device *drm,
			     struct mtk_drm_crtc *mtk_crtc,
			     struct drm_plane *primary,
			     struct drm_plane *cursor, unsigned int pipe)
{
	int ret;

	DDPINFO("%s+\n", __func__);
	ret = drm_crtc_init_with_planes(drm, &mtk_crtc->base, primary, cursor,
					&mtk_crtc_funcs, NULL);
	if (ret)
		goto err_cleanup_crtc;

	drm_crtc_helper_add(&mtk_crtc->base, &mtk_crtc_helper_funcs);

	mtk_drm_crtc_attach_property(&mtk_crtc->base);
	DDPINFO("%s-\n", __func__);

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(&mtk_crtc->base);
	return ret;
}

void mtk_crtc_ddp_irq(struct drm_crtc *crtc, struct mtk_ddp_comp *comp)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	if (priv->data->shadow_register == false)
		mtk_crtc_ddp_config(crtc);

	mtk_drm_finish_page_flip(mtk_crtc);
}

void mtk_crtc_vblank_irq(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	char tag_name[100] = {'\0'};
	ktime_t ktime = ktime_get();

	mtk_crtc->vblank_time = ktime_to_timespec64(ktime);

	sprintf(tag_name, "%d|HW_VSYNC|%lld",
		DRM_TRACE_VSYNC_ID, ktime);
	mtk_drm_trace_c("%s", tag_name);

/*
 *	DDPMSG("%s CRTC%d %s\n", __func__,
 *			drm_crtc_index(crtc), tag_name);
 */
	drm_crtc_handle_vblank(&mtk_crtc->base);

	sprintf(tag_name, "%d|HW_VSYNC|%d",
		DRM_TRACE_VSYNC_ID, 0);
	mtk_drm_trace_c("%s", tag_name);

}

static void mtk_crtc_get_output_comp_name(struct mtk_drm_crtc *mtk_crtc,
					  char *buf, int buf_len)
{
	int i, j;
	struct mtk_ddp_comp *comp;

	for_each_comp_in_crtc_path_reverse(comp, mtk_crtc, i,
					   j)
		if (mtk_ddp_comp_is_output(comp)) {
			mtk_ddp_comp_get_name(comp, buf, buf_len);
			return;
		}

	DDPPR_ERR("%s(), no output comp found for crtc%d, set buf to 0\n",
		  __func__, drm_crtc_index(&mtk_crtc->base));
	memset(buf, 0, buf_len);
}
static void mtk_crtc_get_event_name(struct mtk_drm_crtc *mtk_crtc, char *buf,
				    int buf_len, int event_id)
{
	int crtc_id, len;
	char output_comp[20];

	/* TODO: remove hardcode comp event */
	crtc_id = drm_crtc_index(&mtk_crtc->base);
	switch (event_id) {
	case EVENT_STREAM_DIRTY:
		len = snprintf(buf, buf_len, "disp_token_stream_dirty%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_SODI:
		len = snprintf(buf, buf_len, "disp_token_sodi%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_STREAM_EOF:
		len = snprintf(buf, buf_len, "disp_token_stream_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_VDO_EOF:
		len = snprintf(buf, buf_len, "disp_mutex%d_eof",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_CMD_EOF:
		mtk_crtc_get_output_comp_name(mtk_crtc, output_comp,
					      sizeof(output_comp));
		len = snprintf(buf, buf_len, "disp_%s_eof", output_comp);
		break;
	case EVENT_TE:
		mtk_crtc_get_output_comp_name(mtk_crtc, output_comp,
					      sizeof(output_comp));
		len = snprintf(buf, buf_len, "disp_wait_%s_te", output_comp);
		break;
	case EVENT_ESD_EOF:
		len = snprintf(buf, buf_len, "disp_token_esd_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_RDMA0_EOF:
		len = snprintf(buf, buf_len, "disp_rdma0_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_WDMA0_EOF:
		len = snprintf(buf, buf_len, "disp_wdma0_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_WDMA1_EOF:
		len = snprintf(buf, buf_len, "disp_wdma1_eof%d",
				   drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_STREAM_BLOCK:
		len = snprintf(buf, buf_len, "disp_token_stream_block%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_CABC_EOF:
		len = snprintf(buf, buf_len, "disp_token_cabc_eof%d",
					drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_DSI_SOF:
		mtk_crtc_get_output_comp_name(mtk_crtc, output_comp,
					      sizeof(output_comp));
		len = snprintf(buf, buf_len, "disp_%s_sof0", output_comp);
		break;
	/*Msync 2.0*/
	case EVENT_SYNC_TOKEN_VFP_PERIOD:
		len = snprintf(buf, buf_len, "disp_token_vfp_period%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_GPIO_TE0:
		len = snprintf(buf, buf_len, "disp_gpio_te0");
		break;
	case EVENT_GPIO_TE1:
		len = snprintf(buf, buf_len, "disp_gpio_te1");
		break;
	case EVENT_SYNC_TOKEN_DISP_VA_START:
		len = snprintf(buf, buf_len, "disp_token_disp_va_start%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_DISP_VA_END:
		len = snprintf(buf, buf_len, "disp_token_disp_va_end%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_TE:
		len = snprintf(buf, buf_len, "disp_token_disp_te%d",
				   drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_PREFETCH_TE:
		len = snprintf(buf, buf_len, "disp_token_disp_prefetch_te%d",
				   drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_DSI0_TARGET_LINE:
		len = snprintf(buf, buf_len, "disp_dsi0_targetline%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_OVLSYS_WDMA0_EOF:
		len = snprintf(buf, buf_len, "disp_ovlsys_wdma0_eof%d",
				drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_OVLSYS1_WDMA0_EOF:
		len = snprintf(buf, buf_len, "disp_ovlsys_wdma2_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_VIDLE_POWER_ON:
		len = snprintf(buf, buf_len, "disp_token_disp_v_idle_power_on%d",
					drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_SYNC_TOKEN_CHECK_TRIGGER_MERGE:
		len = snprintf(buf, buf_len, "disp_token_disp_check_trigger_merge%d",
					drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_OVLSYS_WDMA1_EOF:
		len = snprintf(buf, buf_len, "disp_ovlsys_wdma1_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_MDP_RDMA0_EOF:
		len = snprintf(buf, buf_len, "disp_mdp_rdma0_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	case EVENT_MDP_RDMA1_EOF:
		len = snprintf(buf, buf_len, "disp_mdp_rdma1_eof%d",
			       drm_crtc_index(&mtk_crtc->base));
		break;
	default:
		DDPPR_ERR("%s invalid event_id:%d\n", __func__, event_id);
		memset(output_comp, 0, sizeof(output_comp));
	}
}

void mtk_crtc_update_gce_event(struct mtk_drm_crtc *mtk_crtc)
{
	char buf[50];
	int i;
	struct device *dev = mtk_crtc->base.dev->dev;

	/* Load CRTC GCE event again after re-enable crtc */
	for (i = 0; i < EVENT_TYPE_MAX; i++) {
		mtk_crtc_get_event_name(mtk_crtc, buf, sizeof(buf), i);
		mtk_crtc->gce_obj.event[i] = cmdq_dev_get_event(dev, buf);
	}
}

#ifndef DRM_CMDQ_DISABLE
#ifdef IF_ZERO /* not ready for dummy register method */
static void mtk_crtc_init_color_matrix_data_slot(
					struct mtk_drm_crtc *mtk_crtc)
{
	struct cmdq_pkt *cmdq_handle;
	struct disp_ccorr_config ccorr_config = {.mode = 1,
						 .color_matrix = {
						 1024, 0, 0, 0,
						 0, 1024, 0, 0,
						 0, 0, 1024, 0,
						 0, 0, 0, 1024},
						 .featureFlag = false };

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return;
	}

	mtk_crtc_backup_color_matrix_data(&mtk_crtc->base, &ccorr_config,
					cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}
#endif
#endif

static void mtk_crtc_init_gce_obj(struct drm_device *drm_dev,
				  struct mtk_drm_crtc *mtk_crtc)
{
	struct device *dev = drm_dev->dev;
	struct cmdq_pkt_buffer *cmdq_buf;
	char buf[50];
	int len, index, i;

	/* Load CRTC GCE client */
	for (i = 0; i < CLIENT_TYPE_MAX; i++) {
		DRM_INFO("%s(), %s, %d", __func__,
			 crtc_gce_client_str[i],
			 drm_crtc_index(&mtk_crtc->base));
		len = snprintf(buf, sizeof(buf), "%s%d", crtc_gce_client_str[i],
			       drm_crtc_index(&mtk_crtc->base));
		if (len < 0) {
			/* Handle snprintf() error */
			DDPPR_ERR("%s:snprintf error\n", __func__);
			return;
		}
		index = of_property_match_string(dev->of_node,
						 "gce-client-names", buf);
		if (index < 0) {
			mtk_crtc->gce_obj.client[i] = NULL;
			continue;
		}
		mtk_crtc->gce_obj.client[i] =
			cmdq_mbox_create(dev, index);
	}

	/* Load CRTC GCE event */
	for (i = 0; i < EVENT_TYPE_MAX; i++) {
		mtk_crtc_get_event_name(mtk_crtc, buf, sizeof(buf), i);
		mtk_crtc->gce_obj.event[i] = cmdq_dev_get_event(dev, buf);
	}

	cmdq_buf = &(mtk_crtc->gce_obj.buf);
#ifndef DRM_CMDQ_DISABLE
	if (mtk_crtc->gce_obj.client[CLIENT_CFG]) {
		DDPINFO("[CRTC][CHECK-1]0x%p\n",
			mtk_crtc->gce_obj.client[CLIENT_CFG]);
		if (mtk_crtc->gce_obj.client[CLIENT_CFG]->chan) {
			DDPINFO("[CRTC][CHECK-2]0x%p\n",
				mtk_crtc->gce_obj.client[CLIENT_CFG]->chan);
			if (mtk_crtc->gce_obj.client[CLIENT_CFG]->chan->mbox) {
				DDPINFO("[CRTC][CHECK-3]0x%p\n",
					mtk_crtc->gce_obj.client[CLIENT_CFG]
						->chan->mbox);
				if (mtk_crtc->gce_obj.client[CLIENT_CFG]
					    ->chan->mbox->dev) {
					DDPINFO("[CRTC][CHECK-4]0x%p\n",
						mtk_crtc->gce_obj
							.client[CLIENT_CFG]
							->chan->mbox->dev);
				}
			}
		}
	}
	cmdq_buf->va_base = cmdq_mbox_buf_alloc(
		mtk_crtc->gce_obj.client[CLIENT_CFG],
		&(cmdq_buf->pa_base));

	if (!cmdq_buf->va_base) {
		DDPPR_ERR("va base is NULL\n");
		return;
	}

	memset(cmdq_buf->va_base, 0, DISP_SLOT_SIZE);

	/* support DC with color matrix config no more */
	/* mtk_crtc_init_color_matrix_data_slot(mtk_crtc); */

	mtk_crtc->gce_obj.base = cmdq_register_device(dev);
#endif
}

unsigned int mtk_get_plane_slot_idx(struct mtk_drm_crtc *mtk_crtc, unsigned int idx)
{
	struct drm_crtc *crtc = &mtk_crtc->base;

	if (drm_crtc_index(crtc) > 0)
		idx += OVL_LAYER_NR;
	if (drm_crtc_index(crtc) > 1)
		idx += EXTERNAL_INPUT_LAYER_NR;

	return idx;
}

/* for platform that store information in register rather than mermory */
void mtk_gce_backup_slot_init(struct mtk_drm_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	size_t size = 0;
	struct dummy_mapping *table;
	unsigned int mmsys_id = 0;
	int i;

	mmsys_id = mtk_get_mmsys_id(crtc);

	if ((mmsys_id == MMSYS_MT6983) ||
		(mmsys_id == MMSYS_MT6895)) {
		table = mt6983_dispsys_dummy_register;
		size = MT6983_DUMMY_REG_CNT;
	} else if (mmsys_id == MMSYS_MT6879) {
		table = mt6879_dispsys_dummy_register;
		size = MT6879_DUMMY_REG_CNT;
	}

	for (i = 0 ; i < size ; i++)
		writel(0x0, table[i].addr + table[i].offset);
}

unsigned int *mtk_get_gce_backup_slot_va(struct mtk_drm_crtc *mtk_crtc,
			unsigned int slot_index)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	size_t size = 0;
	unsigned int offset = 0;
	struct dummy_mapping *table;
	unsigned int idx, mmsys_id = 0;

	if (slot_index > DISP_SLOT_SIZE) {
		DDPPR_ERR("%s invalid slot_index", __func__);
		return NULL;
	}

	mmsys_id = mtk_get_mmsys_id(crtc);
	idx = slot_index / sizeof(unsigned int);

	if ((mmsys_id == MMSYS_MT6983) ||
		(mmsys_id == MMSYS_MT6895)) {
		table = mt6983_dispsys_dummy_register;
		size = MT6983_DUMMY_REG_CNT;
	} else if (mmsys_id == MMSYS_MT6879) {
		table = mt6879_dispsys_dummy_register;
		size = MT6879_DUMMY_REG_CNT;
	} else {
		struct cmdq_pkt_buffer *cmdq_buf = &(mtk_crtc->gce_obj.buf);

		if (cmdq_buf == NULL) {
			DDPPR_ERR("%s invalid cmdq_buffer\n", __func__);
			return NULL;
		}

		return (cmdq_buf->va_base + slot_index);
	}

	if (idx < size) {
		offset = table[idx].offset;
		if (table[idx].addr == NULL) {
			DDPPR_ERR("NULL pointer\n");
			return NULL;
		}
		return table[idx].addr + offset;
	}

	DDPPR_ERR("%s exceed avilable register table\n", __func__);

	return NULL;
}

dma_addr_t mtk_get_gce_backup_slot_pa(struct mtk_drm_crtc *mtk_crtc,
			unsigned int slot_index)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	size_t size = 0;
	unsigned int offset = 0;
	struct dummy_mapping *table;
	unsigned int idx, mmsys_id = 0;

	if (slot_index > DISP_SLOT_SIZE) {
		DDPPR_ERR("%s invalid slot_index", __func__);
		return 0;
	}

	mmsys_id = mtk_get_mmsys_id(crtc);
	idx = slot_index / sizeof(unsigned int);

	if ((mmsys_id == MMSYS_MT6983) ||
		(mmsys_id == MMSYS_MT6895)) {
		table = mt6983_dispsys_dummy_register;
		size = MT6983_DUMMY_REG_CNT;
	} else if (mmsys_id == MMSYS_MT6879) {
		table = mt6879_dispsys_dummy_register;
		size = MT6879_DUMMY_REG_CNT;
	} else {
		struct cmdq_pkt_buffer *cmdq_buf = &(mtk_crtc->gce_obj.buf);

		if (cmdq_buf == NULL) {
			DDPPR_ERR("%s invalid cmdq_buffer\n", __func__);
			return 0;
		}

		return (cmdq_buf->pa_base + slot_index);
	}

	if (idx < size) {
		offset = table[idx].offset;
		if (table[idx].pa_addr == 0) {
			DDPPR_ERR("NULL pointer\n");
			return 0;
		}
		return table[idx].pa_addr + offset;
	}

	DDPPR_ERR("%s exceed avilable register table\n", __func__);

	return 0;
}


static int mtk_drm_cwb_copy_buf(struct drm_crtc *crtc,
			  struct mtk_cwb_info *cwb_info,
			  void *buffer, unsigned int buf_idx)
{
	unsigned long addr_va = cwb_info->buffer[buf_idx].addr_va;
	enum CWB_BUFFER_TYPE type = cwb_info->type;
	struct drm_framebuffer *fb = cwb_info->buffer[0].fb;
	int Bpp = mtk_get_format_bpp(fb->format->format);
	int width, height, pitch, size;
	unsigned long long time = sched_clock();

	//double confirm user_buffer still exists
	if (!cwb_info->funcs || !cwb_info->funcs->get_buffer) {
		if (!cwb_info->user_buffer)
			return -1;
		buffer = cwb_info->user_buffer;
		cwb_info->user_buffer = 0;
	}

	width = cwb_info->copy_w;
	height = cwb_info->copy_h;
	pitch = width * Bpp;
	size = pitch * height;

	if (type == IMAGE_ONLY) {
		u8 *tmp = (u8 *)buffer;

		memcpy(tmp, (void *)addr_va, size);
	} else if (type == CARRY_METADATA) {
		struct user_cwb_buffer *tmp = (struct user_cwb_buffer *)buffer;

		tmp->data.width = width;
		tmp->data.height = height;
		tmp->meta.frameIndex = cwb_info->count;
		tmp->meta.timestamp = cwb_info->buffer[buf_idx].timestamp;
		memcpy(tmp->data.image, (void *)addr_va, size);
	}
	DDPMSG("[capture] copy buf from 0x%x, (w,h)=(%d,%d), ts:%llu done\n",
			addr_va, width, height, time);

	return 0;
}

static void mtk_drm_cwb_cb(struct cmdq_cb_data data)
{
	struct mtk_cmdq_cb_data *cb_data = data.data;

	cmdq_pkt_destroy(cb_data->cmdq_handle);
	kfree(cb_data);
}

static void mtk_drm_cwb_give_buf(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_cwb_info *cwb_info;
	struct mtk_ddp_comp *comp;
	unsigned int target_idx, next_idx;
	dma_addr_t addr = 0;
	void *user_buffer;
	struct cmdq_pkt *handle;
	struct cmdq_client *client = mtk_crtc->gce_obj.client[CLIENT_CFG];
	struct mtk_cmdq_cb_data *cb_data;
	const struct mtk_cwb_funcs *funcs;
	int ubuf_px = 0, write_done_px = 0;

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	if (!mtk_crtc->enabled) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return;
	}
	cwb_info = mtk_crtc->cwb_info;
	user_buffer = 0;
	ubuf_px = cwb_info->buffer[0].dst_roi.width *
			cwb_info->buffer[0].dst_roi.height;
	write_done_px = cwb_info->copy_w * cwb_info->copy_h;
	funcs = cwb_info->funcs;
	if (funcs && funcs->get_buffer)
		funcs->get_buffer(&user_buffer);
	else
		user_buffer = cwb_info->user_buffer;

	if (!user_buffer) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return;
	} else if (ubuf_px != write_done_px) {
		DDPMSG("[capture] ubuf_px:%d != done_px:%d, wait new frame\n",
			ubuf_px, write_done_px);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return;
	}

	target_idx = cwb_info->buf_idx;
	next_idx = 1 - cwb_info->buf_idx;

	//1. config next addr to wdma addr
	addr = cwb_info->buffer[next_idx].addr_mva;
	comp = cwb_info->comp;
	cb_data = kmalloc(sizeof(*cb_data), GFP_KERNEL);
	if (!cb_data) {
		DDPPR_ERR("%s:%d, cb data creation failed\n",
			__func__, __LINE__);
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return;
	}

	mtk_crtc_pkt_create(&handle, crtc, client);

	if (!handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	mtk_crtc_wait_frame_done(mtk_crtc, handle, DDP_FIRST_PATH, 0);
	mtk_ddp_comp_io_cmd(comp, handle, WDMA_WRITE_DST_ADDR0, &addr);
	if (mtk_crtc->is_dual_pipe) {
		comp = priv->ddp_comp[dual_pipe_comp_mapping(priv->data->mmsys_id, comp->id)];
		mtk_ddp_comp_io_cmd(comp, handle, WDMA_WRITE_DST_ADDR0, &addr);
	}
	cb_data->cmdq_handle = handle;
	if (cmdq_pkt_flush_threaded(handle, mtk_drm_cwb_cb, cb_data) < 0)
		DDPPR_ERR("failed to flush write_back\n");

	//2. idx change
	cwb_info->buf_idx = next_idx;
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	//3. copy target_buf_idx context
	DDP_MUTEX_LOCK(&mtk_crtc->cwb_lock, __func__, __LINE__);
	mtk_drm_cwb_copy_buf(crtc, cwb_info, user_buffer, target_idx);
	mtk_dprec_mmp_dump_cwb_buffer(crtc, user_buffer, target_idx);
	DDP_MUTEX_UNLOCK(&mtk_crtc->cwb_lock, __func__, __LINE__);

	//4. notify user
	if (funcs && funcs->copy_done)
		funcs->copy_done(user_buffer, cwb_info->type);
	else
		DDPPR_ERR("User has no notify callback funciton/n");
}

static int mtk_drm_cwb_monitor_thread(void *data)
{
	int ret = 0;
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_cwb_info *cwb_info;

	msleep(16000);
	while (1) {
		ret = wait_event_interruptible(
			mtk_crtc->cwb_wq,
			atomic_read(&mtk_crtc->cwb_task_active));
		atomic_set(&mtk_crtc->cwb_task_active, 0);

		cwb_info = mtk_crtc->cwb_info;

		if (!cwb_info || !cwb_info->enable)
			continue;

		mtk_drm_cwb_give_buf(crtc);

		if (kthread_should_stop())
			break;
	}

	return 0;
}

static int mtk_drm_cwb_init(struct drm_crtc *crtc)
{
#define LEN 50
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	char name[LEN];

	snprintf(name, LEN, "mtk_drm_cwb");
	mtk_crtc->cwb_task =
		kthread_create(mtk_drm_cwb_monitor_thread, crtc, name);
	init_waitqueue_head(&mtk_crtc->cwb_wq);
	atomic_set(&mtk_crtc->cwb_task_active, 0);

	wake_up_process(mtk_crtc->cwb_task);

	return 0;
}

void mtk_drm_fake_vsync_switch(struct drm_crtc *crtc, bool enable)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_fake_vsync *fake_vsync = mtk_crtc->fake_vsync;

	if (mtk_drm_lcm_is_connect(mtk_crtc) ||
		!mtk_crtc_is_frame_trigger_mode(crtc) ||
		mtk_crtc_is_mem_mode(crtc))
		return;

	if (unlikely(!fake_vsync)) {
		DDPPR_ERR("%s:invalid fake_vsync pointer\n", __func__);
		return;
	}

	atomic_set(&fake_vsync->fvsync_active, enable);
	if (enable)
		wake_up_interruptible(&fake_vsync->fvsync_wq);
}

static int mtk_drm_fake_vsync_kthread(void *data)
{
	struct sched_param param = {.sched_priority = 87 };
	struct drm_crtc *crtc = (struct drm_crtc *)data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_fake_vsync *fake_vsync = mtk_crtc->fake_vsync;
	int ret = 0;

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		ret = wait_event_interruptible(fake_vsync->fvsync_wq,
				atomic_read(&fake_vsync->fvsync_active));

		mtk_crtc_vblank_irq(crtc);
		usleep_range(16700, 17700);

		if (kthread_should_stop())
			break;
	}
	return 0;
}

void mtk_drm_fake_vsync_init(struct drm_crtc *crtc)
{
#define LEN 50
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_fake_vsync *fake_vsync =
		kzalloc(sizeof(struct mtk_drm_fake_vsync), GFP_KERNEL);
	char name[LEN];

	snprintf(name, LEN, "mtk_drm_fake_vsync:%d", drm_crtc_index(crtc));
	fake_vsync->fvsync_task = kthread_create(mtk_drm_fake_vsync_kthread,
					crtc, name);
	init_waitqueue_head(&fake_vsync->fvsync_wq);
	atomic_set(&fake_vsync->fvsync_active, 0);
	mtk_crtc->fake_vsync = fake_vsync;

	wake_up_process(fake_vsync->fvsync_task);
}

static int dc_main_path_commit_thread(void *data)
{
	int ret;
	struct sched_param param = {.sched_priority = 94 };
	struct drm_crtc *crtc = data;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	sched_setscheduler(current, SCHED_RR, &param);

	while (1) {
		ret = wait_event_interruptible(mtk_crtc->dc_main_path_commit_wq,
			atomic_read(&mtk_crtc->dc_main_path_commit_event));
		if (ret == 0) {
			atomic_set(&mtk_crtc->dc_main_path_commit_event, 0);
			mtk_crtc_dc_prim_path_update(crtc);
		} else {
			DDPINFO("wait dc commit event interrupted, ret = %d\n",
				     ret);
		}
		if (kthread_should_stop())
			break;
	}

	return 0;
}

/* This pf release thread only is aiming for frame trigger mode's CRTC */
static int mtk_drm_pf_release_thread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	struct mtk_drm_private *private;
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *)data;
	struct drm_crtc *crtc;
	unsigned int crtc_idx;
#ifndef DRM_CMDQ_DISABLE
	ktime_t pf_time;
	unsigned int fence_idx = 0;
#endif

	crtc = &mtk_crtc->base;
	private = crtc->dev->dev_private;
	crtc_idx = drm_crtc_index(crtc);
	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		wait_event_interruptible(mtk_crtc->present_fence_wq,
				 atomic_read(&mtk_crtc->pf_event));
		atomic_set(&mtk_crtc->pf_event, 0);

		if (likely(mtk_drm_lcm_is_connect(mtk_crtc)))
			pf_time = mtk_check_preset_fence_timestamp(crtc);
		else
			pf_time = 0;
#ifndef DRM_CMDQ_DISABLE
		fence_idx = atomic_read(&private->crtc_rel_present[crtc_idx]);

		mtk_release_present_fence(private->session_id[crtc_idx],
					  fence_idx, pf_time);
		if (crtc_idx == 0)
			ktime_get_real_ts64(&rdma_sof_tval);
#endif
	}

	return 0;
}

static int mtk_drm_sf_pf_release_thread(void *data)
{
	struct sched_param param = {.sched_priority = 87};
	struct mtk_drm_private *private;
	struct mtk_drm_crtc *mtk_crtc = (struct mtk_drm_crtc *)data;
	struct drm_crtc *crtc;


	crtc = &mtk_crtc->base;
	private = crtc->dev->dev_private;
	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		wait_event_interruptible(mtk_crtc->sf_present_fence_wq,
					 atomic_read(&mtk_crtc->sf_pf_event));
		atomic_set(&mtk_crtc->sf_pf_event, 0);

#ifndef DRM_CMDQ_DISABLE
		mutex_lock(&private->commit.lock);

		mutex_unlock(&private->commit.lock);
#endif
	}

	return 0;
}

static int disp_mutex_dispatch(struct mtk_drm_private *priv, struct mtk_drm_crtc *mtk_crtc,
			const struct mtk_crtc_path_data *path_data, unsigned int pipe)
{
	int i, j;
	unsigned int max_path = 0, cur = 0;
	struct mtk_ddp *ddp = dev_get_drvdata(priv->mutex_dev);

	if (unlikely(!priv || !mtk_crtc || !path_data)) {
		DDPPR_ERR("%s invalid parameter\n", __func__);
		return -EINVAL;
	}

	for (i = 0 ; i < DDP_MODE_NR ; i++) {
		for (j = cur; j < DDP_PATH_NR ; j++) {
			if (path_data->path_len[i][j] == 0)
				continue;
			++cur;
		}
		if (max_path < cur)
			max_path = cur;
	}
	ddp->mtk_crtc[pipe] = mtk_crtc;

	if (path_data->is_discrete_path)
		max_path++;

	for (i = 0, cur = 0 ; i < 10 ; ++i) {
		struct mtk_disp_mutex *mutex;

		mutex = mtk_disp_mutex_get(priv->mutex_dev, i);
		if (IS_ERR(mutex))
			continue;
		mtk_crtc->mutex[cur] = mutex;
		DDPDBG("%s assign mutex %d need %u mutex\n", __func__, i, max_path);
		++cur;
		if (cur >= max_path)
			break;
	}

	if (unlikely(i >= 10)) {
		DDPPR_ERR("%s can't allocate proper disp_mutex\n");
		return -ENOSPC;
	}

	return 0;
}

int mtk_drm_crtc_create(struct drm_device *drm_dev,
			const struct mtk_crtc_path_data *path_data)
{
	struct mtk_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;
	struct mtk_drm_crtc *mtk_crtc;
	struct mtk_ddp_comp *output_comp;
	enum drm_plane_type type;
	unsigned int zpos;
	int pipe = priv->num_pipes;
	int ret;
	int i, j, p_mode;
	enum mtk_ddp_comp_id comp_id;
	static cpumask_t cpumask;
	u32 caps_value = 0;
#define CAPS_CHAR_SIZE 25
	char crtc_caps[CAPS_CHAR_SIZE];
	int caps_len, of_red;

	DDPMSG("%s+\n", __func__);

	if (!path_data)
		return 0;

	for_each_comp_id_in_path_data(comp_id, path_data, i, j, p_mode) {
		struct device_node *node;

		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL)
			continue;

		node = priv->comp_node[comp_id];
		if (!node) {
			if (!of_property_read_bool(priv->mmsys_dev->of_node,
				"enable-output-int-switch")
				&& mtk_ddp_comp_is_output_by_id(comp_id)
				&& pipe == 0 && p_mode == DDP_MINOR) {
				DDPMSG("skip this error because %d is not enabled.\n", comp_id);
				continue;
			}
			dev_info(
				dev,
				"Not creating crtc %d because component %d is disabled or missing\n",
				pipe, comp_id);
			DDPPR_ERR(
				"Not creating crtc %d because component %d is disabled or missing\n",
				pipe, comp_id);

			return 0;
		}
	}

	mtk_crtc = devm_kzalloc(dev, sizeof(*mtk_crtc), GFP_KERNEL);
	if (!mtk_crtc)
		return -ENOMEM;

	// TODO: It should use platform_driverdata or device tree to define it.
	// It's just for P90 temp workaround, will be modifyed later.
	if (pipe == 0)
		mtk_crtc->layer_nr = OVL_LAYER_NR;
	else if (pipe == 1)
		mtk_crtc->layer_nr = EXTERNAL_INPUT_LAYER_NR;
	else if (pipe == 2)
		mtk_crtc->layer_nr = MEMORY_INPUT_LAYER_NR;
	else if (pipe == 3)
		mtk_crtc->layer_nr = SP_INPUT_LAYER_NR;

	mutex_init(&mtk_crtc->lock);
	mutex_init(&mtk_crtc->cwb_lock);
	mutex_init(&mtk_crtc->mml_ir_sram.lock);
	spin_lock_init(&mtk_crtc->pf_time_lock);
	mtk_crtc->config_regs = priv->config_regs;
	mtk_crtc->config_regs_pa = priv->config_regs_pa;
	mtk_crtc->dispsys_num = priv->dispsys_num;
	mtk_crtc->side_config_regs = priv->side_config_regs;
	mtk_crtc->side_config_regs_pa = priv->side_config_regs_pa;
	mtk_crtc->mmsys_reg_data = priv->reg_data;
	mtk_crtc->path_data = path_data;
	mtk_crtc->is_dual_pipe = false;
	mtk_crtc->is_force_mml_scen = mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MML_PQ);
	mtk_crtc->mml_ir_enable = mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_MML_IR);
	mtk_crtc->slbc_state = SLBC_UNREGISTER;
	mtk_crtc->mml_ir_sram.data.type = TP_BUFFER;
	mtk_crtc->mml_ir_sram.data.uid = UID_DISP;

	if (priv->data->mmsys_id == MMSYS_MT6985) {
		mtk_crtc->ovlsys0_regs = priv->ovlsys0_regs;
		mtk_crtc->ovlsys0_regs_pa = priv->ovlsys0_regs_pa;
		mtk_crtc->ovlsys_num = priv->ovlsys_num;
		mtk_crtc->ovlsys1_regs = priv->ovlsys1_regs;
		mtk_crtc->ovlsys1_regs_pa = priv->ovlsys1_regs_pa;
	}
	INIT_LIST_HEAD(&mtk_crtc->mml_ir_sram.list.head);

	for (i = 0; i < DDP_MODE_NR; i++) {
		for (j = 0; j < DDP_PATH_NR; j++) {
			mtk_crtc->ddp_ctx[i].ddp_comp_nr[j] =
				path_data->path_len[i][j];
			mtk_crtc->ddp_ctx[i].ddp_comp[j] = devm_kmalloc_array(
				dev, path_data->path_len[i][j],
				sizeof(struct mtk_ddp_comp *), GFP_KERNEL | __GFP_ZERO);
			mtk_crtc->ddp_ctx[i].ovl_comp_nr[j] =
				path_data->ovl_path_len[i][j];
			mtk_crtc->ddp_ctx[i].ovl_comp[j] = devm_kmalloc_array(
				dev, path_data->ovl_path_len[i][j],
				sizeof(struct mtk_ddp_comp *), GFP_KERNEL | __GFP_ZERO);
			mtk_crtc->ddp_ctx[i].req_hrt[j] =
				path_data->path_req_hrt[i][j];
		}
		mtk_crtc->ddp_ctx[i].wb_comp_nr = path_data->wb_path_len[i];
		mtk_crtc->ddp_ctx[i].wb_comp = devm_kmalloc_array(
			dev, path_data->wb_path_len[i],
			sizeof(struct mtk_ddp_comp *), GFP_KERNEL | __GFP_ZERO);
	}

	ret = disp_mutex_dispatch(priv, mtk_crtc, path_data, pipe);
	if (ret)
		DDPPR_ERR("mutex_dispatch fail %d\n", ret);

	for_each_comp_id_in_path_data(comp_id, path_data, i, j, p_mode) {
		struct mtk_ddp_comp *comp;
		struct device_node *node;
		bool *rdma_memory_mode;

		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL) {
			struct mtk_ddp_comp *comp;

			comp = kzalloc(sizeof(*comp), GFP_KERNEL);
			comp->id = comp_id;
			if (mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i] &&
					j < mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i])
				mtk_crtc->ddp_ctx[p_mode].ovl_comp[i][j] = comp;
			else
				/* shift comp_idx from global idx to local idx for ddp_comp array */
				mtk_crtc->ddp_ctx[p_mode].ddp_comp[i]
					[j - mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i]] = comp;
			continue;
		}
		if (!of_property_read_bool(priv->mmsys_dev->of_node, "enable-output-int-switch")
			&& mtk_ddp_comp_is_output_by_id(comp_id)
			&& pipe == 0 && p_mode == DDP_MINOR) {
			DDPMSG("skip this component because %d is not enabled.\n", comp_id);
			continue;
		}

		node = priv->comp_node[comp_id];
		comp = priv->ddp_comp[comp_id];

		if (!comp) {
			dev_err(dev, "Component %s not initialized\n",
				node->full_name);
			return -ENODEV;
		}

		if ((p_mode == DDP_MAJOR) && (comp_id == DDP_COMPONENT_WDMA0 ||
					      comp_id == DDP_COMPONENT_WDMA1 ||
					      comp_id == DDP_COMPONENT_OVLSYS_WDMA2)) {
			ret = mtk_wb_connector_init(drm_dev, mtk_crtc);
			if (ret != 0)
				return ret;

			ret = mtk_wb_set_possible_crtcs(drm_dev, mtk_crtc,
							BIT(pipe));
			if (ret != 0)
				return ret;
		}

		if ((j == 0) && (p_mode == DDP_MAJOR) &&
		    (comp_id == DDP_COMPONENT_RDMA0 ||
		     comp_id == DDP_COMPONENT_RDMA1 ||
		     comp_id == DDP_COMPONENT_RDMA2 ||
		     comp_id == DDP_COMPONENT_RDMA3 ||
		     comp_id == DDP_COMPONENT_RDMA4 ||
		     comp_id == DDP_COMPONENT_RDMA5)) {
			rdma_memory_mode = comp->comp_mode;
			*rdma_memory_mode = true;
			mtk_crtc->layer_nr = RDMA_LAYER_NR;
		}

		if (mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i] &&
				j < mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i])
			mtk_crtc->ddp_ctx[p_mode].ovl_comp[i][j] = comp;
		else
			/* shift comp_idx from global idx to local idx for ddp_comp array */
			mtk_crtc->ddp_ctx[p_mode].ddp_comp[i]
				[j - mtk_crtc->ddp_ctx[p_mode].ovl_comp_nr[i]] = comp;
	}

	mtk_crtc_prepare_dual_pipe_path(dev, priv, mtk_crtc);

	for_each_wb_comp_id_in_path_data(comp_id, path_data, i, p_mode) {
		struct mtk_ddp_comp *comp;
		struct device_node *node;

		if (comp_id < 0) {
			DDPPR_ERR("%s: Invalid comp_id:%d\n", __func__, comp_id);
			return 0;
		}

		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL) {
			struct mtk_ddp_comp *comp;

			comp = kzalloc(sizeof(*comp), GFP_KERNEL);
			comp->id = comp_id;
			mtk_crtc->ddp_ctx[p_mode].wb_comp[i] = comp;
			continue;
		}
		if (comp_id < 0) {
			DDPPR_ERR("%s: Invalid comp_id:%d\n", __func__, comp_id);
			return 0;
		}
		node = priv->comp_node[comp_id];
		comp = priv->ddp_comp[comp_id];

		if (!comp) {
			dev_err(dev, "Component %s not initialized\n",
				node->full_name);
			return -ENODEV;
		}
		mtk_crtc->ddp_ctx[p_mode].wb_comp[i] = comp;
	}

	caps_len = snprintf(crtc_caps, CAPS_CHAR_SIZE, "crtc_ability_%u", pipe);
	if (caps_len < 0)
		DDPPR_ERR("%s %d snprintf fail\n", __func__, __LINE__);

	of_red = of_property_read_u32(dev->of_node, crtc_caps, &caps_value);
	if (of_red == 0) {
		mtk_crtc->crtc_caps.crtc_ability = caps_value;
	} else {
		DDPINFO("%s pipe %d of_property_read_u32 not found : %d\n",
			__func__, pipe, of_red);
		/* define each pipe's CRTC ability's default value */
		/* use it when CRTC not define ability in DTS */
		if (pipe == 0) {
			mtk_crtc->crtc_caps.wb_caps[0].support = 1;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_IDLEMGR;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_ESD_CHECK;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_PQ;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_RSZ;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_MML;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_WCG;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_FBDC;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_EXT_LAYER;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_CWB;
			mtk_crtc->crtc_caps.crtc_ability |= ABILITY_MSYNC20;
		}
		mtk_crtc->crtc_caps.ovl_csc_bit_number = 18;
	}
	DDPINFO("%s:%x\n", crtc_caps, caps_value);

	mtk_crtc->ovl_usage_status = priv->ovl_usage[pipe];
	mtk_crtc->planes = devm_kzalloc(dev,
			mtk_crtc->layer_nr * sizeof(struct mtk_drm_plane),
			GFP_KERNEL);
	if (!mtk_crtc->planes)
		return -ENOMEM;

	for (zpos = 0; zpos < mtk_crtc->layer_nr; zpos++) {
		type = (zpos == 0) ? DRM_PLANE_TYPE_PRIMARY
				   : (zpos == (mtk_crtc->layer_nr - 1UL))
					     ? DRM_PLANE_TYPE_CURSOR
					     : DRM_PLANE_TYPE_OVERLAY;
		ret = mtk_plane_init(drm_dev, &mtk_crtc->planes[zpos], zpos,
				     BIT(pipe), type);
		if (ret)
			return ret;
	}

	if (mtk_crtc->layer_nr == 1UL) {
		ret = mtk_drm_crtc_init(drm_dev, mtk_crtc,
					&mtk_crtc->planes[0].base, NULL, pipe);
	} else {
		ret = mtk_drm_crtc_init(
			drm_dev, mtk_crtc, &mtk_crtc->planes[0].base,
			&mtk_crtc->planes[mtk_crtc->layer_nr - 1UL].base, pipe);
	}
	if (ret < 0)
		return ret;

	/*
	 * Workaround:
	 * If is_fake_path=true, the path is not used and it only creat a crtc structure to
	 * ensure next crtc can be created.
	 */
	if (path_data->is_fake_path) {
		mtk_crtc->wb_connector.base.connector_type = DRM_MODE_CONNECTOR_DisplayPort;
		DDPMSG("%s, set CRTC%d connector type is invalid(%d)\n",
		       __func__, drm_crtc_index(&mtk_crtc->base),
		       mtk_crtc->wb_connector.base.connector_type);
	}
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (output_comp)
		mtk_ddp_comp_io_cmd(output_comp, NULL, REQ_PANEL_EXT,
				    &mtk_crtc->panel_ext);

	drm_mode_crtc_set_gamma_size(&mtk_crtc->base, MTK_LUT_SIZE);
	/* TODO: Skip color mgmt first */
	// drm_crtc_enable_color_mgmt(&mtk_crtc->base, 0, false, MTK_LUT_SIZE);
	priv->crtc[pipe] = &mtk_crtc->base;
	priv->num_pipes++;

	dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	mtk_crtc_init_gce_obj(drm_dev, mtk_crtc);

	mtk_crtc->vblank_en = 1;
	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_IDLE_MGR) &&
	    !IS_ERR_OR_NULL(output_comp) &&
	    mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI) {
		char name[50];

		mtk_drm_idlemgr_init(&mtk_crtc->base,
				     drm_crtc_index(&mtk_crtc->base));

		snprintf(name, sizeof(name), "enable_vblank");
		mtk_crtc->vblank_enable_task = kthread_create(
			mtk_crtc_enable_vblank_thread, priv->crtc[pipe], name);
		init_waitqueue_head(&mtk_crtc->vblank_enable_wq);
		wake_up_process(mtk_crtc->vblank_enable_task);
	}

	init_waitqueue_head(&mtk_crtc->crtc_status_wq);

	mtk_disp_hrt_cond_init(&mtk_crtc->base);

	if (drm_crtc_index(&mtk_crtc->base) == 0)
		init_waitqueue_head(&mtk_crtc->qos_ctx->hrt_cond_wq);

	if (drm_crtc_index(&mtk_crtc->base) == 0)
		mtk_drm_cwb_init(&mtk_crtc->base);

	mtk_disp_chk_recover_init(&mtk_crtc->base);

	if (output_comp && mtk_ddp_comp_get_type(output_comp->id) == MTK_DSI)
		mtk_drm_fake_vsync_init(&mtk_crtc->base);

	if (mtk_crtc_support_dc_mode(&mtk_crtc->base)) {
		mtk_crtc->dc_main_path_commit_task = kthread_create(
				dc_main_path_commit_thread,
				&mtk_crtc->base, "decouple_update_rdma_cfg");
		atomic_set(&mtk_crtc->dc_main_path_commit_event, 1);
		init_waitqueue_head(&mtk_crtc->dc_main_path_commit_wq);
		wake_up_process(mtk_crtc->dc_main_path_commit_task);
	}

	if (drm_crtc_index(&mtk_crtc->base) == 0) {
		init_waitqueue_head(&mtk_crtc->trigger_event);
		mtk_crtc->trigger_event_task =
			kthread_create(_mtk_crtc_check_trigger,
						mtk_crtc, "ddp_trig");
		wake_up_process(mtk_crtc->trigger_event_task);

		init_waitqueue_head(&mtk_crtc->trigger_delay);
		mtk_crtc->trigger_delay_task =
			kthread_create(_mtk_crtc_check_trigger_delay,
						mtk_crtc, "ddp_trig_d");
		wake_up_process(mtk_crtc->trigger_delay_task);
		/* For protect crtc blank state */
		mutex_init(&mtk_crtc->blank_lock);
		init_waitqueue_head(&mtk_crtc->state_wait_queue);

		init_waitqueue_head(&mtk_crtc->trigger_cmdq);
		mtk_crtc->trig_cmdq_task =
			kthread_run(_mtk_crtc_cmdq_retrig,
					mtk_crtc, "ddp_cmdq_trig");
	}

	init_waitqueue_head(&mtk_crtc->present_fence_wq);
	atomic_set(&mtk_crtc->pf_event, 0);
	mtk_crtc->pf_release_thread =
		kthread_create(mtk_drm_pf_release_thread,
						mtk_crtc, "pf_release_thread");
	/* Do not use CPU0 to avoid serious delays */
	cpumask_xor(&cpumask, cpu_all_mask, cpumask_of(0));
	kthread_bind_mask(mtk_crtc->pf_release_thread, &cpumask);
	wake_up_process(mtk_crtc->pf_release_thread);

	init_waitqueue_head(&mtk_crtc->sf_present_fence_wq);
	atomic_set(&mtk_crtc->sf_pf_event, 0);
	mtk_crtc->sf_pf_release_thread =
				kthread_run(mtk_drm_sf_pf_release_thread,
					    mtk_crtc, "sf_pf_release_thread");
	atomic_set(&mtk_crtc->force_high_step, 0);

	INIT_LIST_HEAD(&mtk_crtc->lyeblob_head);

	/* init wakelock resources */
	{
		unsigned int len = 21;

		mtk_crtc->wk_lock_name = vzalloc(len * sizeof(char));

		snprintf(mtk_crtc->wk_lock_name, len * sizeof(char),
			 "disp_crtc%u_wakelock",
			 drm_crtc_index(&mtk_crtc->base));

		mtk_crtc->wk_lock =
			wakeup_source_create(mtk_crtc->wk_lock_name);
		wakeup_source_add(mtk_crtc->wk_lock);
	}

	/* set bpc by panel info */
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params) {
		if (output_comp)
			mtk_ddp_comp_io_cmd(output_comp, NULL, DSI_GET_BPC,
						&mtk_crtc->bpc);
		DDPINFO("%s, bpc = %d\n", __func__, mtk_crtc->bpc);
	} else if (!mtk_crtc->panel_ext) {
		DDPINFO("%s, set bpc fail, mtk_crtc->bpc NULL\n", __func__);
	} else if (!mtk_crtc->panel_ext->params) {
		DDPINFO("%s, set bpc fail, mtk_crtc->panel_ext->params NULL\n", __func__);
	}

#ifndef DRM_CMDQ_DISABLE
#ifdef MTK_DRM_CMDQ_ASYNC
#ifdef MTK_DRM_ASYNC_HANDLE
	mtk_crtc->signal_present_fece_task = kthread_create(
		mtk_drm_signal_fence_worker_kthread, mtk_crtc, "signal_fence");
	cpumask_xor(&cpumask, cpu_all_mask, cpumask_of(0));
	kthread_bind_mask(mtk_crtc->signal_present_fece_task, &cpumask);
	init_waitqueue_head(&mtk_crtc->signal_fence_task_wq);
	atomic_set(&mtk_crtc->cmdq_done, 0);
	wake_up_process(mtk_crtc->signal_present_fece_task);
#endif
#endif
#endif
	atomic_set(&(mtk_crtc->mml_cb.mml_job_submit_done), 0);
	init_waitqueue_head(&(mtk_crtc->mml_cb.mml_job_submit_wq));
	atomic_set(&(mtk_crtc->wait_mml_last_job_is_flushed), 0);
	init_waitqueue_head(&(mtk_crtc->signal_mml_last_job_is_flushed_wq));

	if (drm_crtc_index(&mtk_crtc->base) == 0)
		mtk_crtc->pf_ts_type = priv->data->pf_ts_type;
	atomic_set(&mtk_crtc->signal_irq_for_pre_fence, 0);
	init_waitqueue_head(&(mtk_crtc->signal_irq_for_pre_fence_wq));
	if (output_comp && mtk_drm_helper_get_opt(priv->helper_opt,
				MTK_DRM_OPT_DUAL_TE))
		mtk_ddp_comp_io_cmd(output_comp, NULL, DUAL_TE_INIT,
				&mtk_crtc->base);
	mtk_crtc->last_aee_trigger_ts = 0;
	DDPMSG("%s-CRTC%d create successfully\n", __func__,
		priv->num_pipes - 1);

	//sync brightness and frame commit @{
	INIT_WORK(&mtk_crtc->delay_set_brightness_work, mtk_drm_setbacklight_delay);
	//@}

	return 0;
}

int mtk_drm_set_msync_cmd_table(struct drm_device *dev,
		struct msync_parameter_table *config_dst)
{
	struct msync_parameter_table *config_src = &msync_params_tb;
	int i = 0;
	struct msync_level_table *level_tb = NULL;

	DDPMSG("%s:%d +++\n", __func__, __LINE__);
	config_src->level_tb = msync_level_tb;

	config_src->msync_max_fps = config_dst->msync_max_fps;
	config_src->msync_min_fps = config_dst->msync_min_fps;
	config_src->msync_level_num = config_dst->msync_level_num;

	if (config_src->msync_level_num > MSYNC_MAX_LEVEL) {
		DDPPR_ERR("msync level num out of max level\n");
		return -EFAULT;
	}

	if (config_dst->level_tb != NULL) {
		if (copy_from_user(config_src->level_tb,
					config_dst->level_tb,
					sizeof(struct msync_level_table) *
					config_src->msync_level_num)) {
			DDPPR_ERR("%s:%d copy failed:(0x%p,0x%p), size:%ld\n",
					__func__, __LINE__,
					config_src->level_tb,
					config_dst->level_tb,
					sizeof(struct msync_level_table) *
					config_src->msync_level_num);
			return -EFAULT;
		}
	}

	for (i = config_src->msync_level_num; i < MSYNC_MAX_LEVEL; i++) {
		level_tb = &config_src->level_tb[i];

		level_tb->level_id = 0;
		level_tb->level_fps = 0;
		level_tb->max_fps = 0;
		level_tb->min_fps = 0;
	}

	DDPMSG("%s:%d ---\n", __func__, __LINE__);
	return 0;
}

int check_msync_config_info(struct msync_parameter_table *config)
{
	unsigned int msync_level_num = config->msync_level_num;
	struct msync_level_table *level_tb = config->level_tb;
	int i = 0;
	struct msync_level_table check_level_tb[MSYNC_MAX_LEVEL];

	DDPMSG("%s:%d +++\n", __func__, __LINE__);
	if (!level_tb) {
		DDPPR_ERR("%s:%d The level table pointer is NULL !!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if ((msync_level_num <= 0) ||
			(msync_level_num > MSYNC_MAX_LEVEL)) {
		DDPPR_ERR("%s:%d The level num is error!!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (level_tb != NULL) {
		if (copy_from_user(check_level_tb,
					level_tb,
					sizeof(struct msync_level_table) *
					msync_level_num)) {
			DDPPR_ERR("%s:%d copy failed:(0x%p,0x%p), size:%ld\n",
					__func__, __LINE__,
					config->level_tb,
					level_tb,
					sizeof(struct msync_level_table) *
					msync_level_num);
			return -EFAULT;
		}
	}

	if (check_level_tb[0].level_id != 1) {
		DDPPR_ERR("%s:%d Level must start from level 1!!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	for (i = 0; i < msync_level_num; i++) {
		DDPMSG("level_id:%u level_fps:%u max_fps:%u min_fps:%u\n",
				check_level_tb[i].level_id, check_level_tb[i].level_fps,
				check_level_tb[i].max_fps, check_level_tb[i].min_fps);
	}

	for (i = 0; i < msync_level_num - 1; i++) {
		if (check_level_tb[i+1].level_id != (check_level_tb[i].level_id + 1)) {
			DDPPR_ERR("%s:%d Level must set like 1,2,3 ... !!!\n",
					__func__, __LINE__);
			return -EFAULT;
		}
	}
	DDPMSG("%s:%d ---\n", __func__, __LINE__);

	return 0;
}

int mtk_drm_set_msync_params_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	int ret = 0;
	struct msync_parameter_table *config = data;

	DDPMSG("%s:%d +++\n", __func__, __LINE__);

	if (!config) {
		DDPPR_ERR("%s:%d The data pointer is NULL !!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (check_msync_config_info(config) < 0) {
		DDPPR_ERR("check msync config info fail!\n");
		return -EFAULT;
	}

	mtk_drm_clear_msync_cmd_level_table();

	ret = mtk_drm_set_msync_cmd_table(dev, config);
	if (ret != 0) {
		DDPPR_ERR("%s:%d copy failed!!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	mtk_drm_sort_msync_level_table();
	msync_cmd_level_tb_dirty = 1;
	DDPMSG("%s:%d ---\n", __func__, __LINE__);
	return 0;
}

int mtk_drm_get_msync_params_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct msync_parameter_table *config = data;
	struct msync_parameter_table *config_dst = NULL;
	struct drm_crtc *crtc = NULL;
	struct mtk_panel_params *params = NULL;
	struct msync_level_table *level_tb = NULL;

	DDPMSG("%s:%d +++\n", __func__, __LINE__);
	if (!config) {
		DDPPR_ERR("%s:%d The data pointer is NULL !!!\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	if (msync_cmd_level_tb_dirty == 1) {
		config_dst = &msync_params_tb;
		config_dst->level_tb = msync_level_tb;
		config->msync_max_fps = config_dst->msync_max_fps;
		config->msync_min_fps = config_dst->msync_min_fps;
		if (config_dst->msync_level_num > MSYNC_MAX_LEVEL)
			config->msync_level_num = MSYNC_MAX_LEVEL;
		else
			config->msync_level_num = config_dst->msync_level_num;

		if (config_dst->level_tb != NULL) {
			if (copy_to_user(config->level_tb,
					config_dst->level_tb,
					sizeof(struct msync_level_table) *
					config->msync_level_num)) {
				DDPPR_ERR("%s:%d copy failed:(0x%p,0x%p), size:%ld\n",
					__func__, __LINE__,
					config->level_tb,
					config_dst->level_tb,
					sizeof(struct msync_level_table) *
					config->msync_level_num);
				return -EFAULT;
			}
		}
	} else {
		crtc = list_first_entry(&(dev)->mode_config.crtc_list,
			typeof(*crtc), head);
		if (!crtc) {
			DDPPR_ERR("find crtc fail\n");
			return -EINVAL;
		}

		params = mtk_drm_get_lcm_ext_params(crtc);
		if (!params) {
			DDPPR_ERR("[Msync2.0] lcm params pointer is NULL\n");
			return -EINVAL;
		}

		if (params->msync_cmd_table.te_type == REQUEST_TE) {
			/* TODO: Add Request Te */
		} else if (params->msync_cmd_table.te_type == MULTI_TE) {
			config->msync_max_fps = params->msync_cmd_table.msync_max_fps;
			config->msync_min_fps = params->msync_cmd_table.msync_min_fps;
			config->msync_level_num = MSYNC_MAX_LEVEL;
			level_tb = params->msync_cmd_table.multi_te_tb.multi_te_level;
		} else if (params->msync_cmd_table.te_type == TRIGGER_LEVEL_TE) {
			/* TODO: Add Trigger Level Te */
		} else {
			DDPPR_ERR("%s:%d No TE Type\n", __func__, __LINE__);
			return -EINVAL;
		}

		if (level_tb != NULL) {
			if (copy_to_user(config->level_tb,
					level_tb,
					sizeof(struct msync_level_table) *
					config->msync_level_num)) {
				DDPPR_ERR("%s:%d copy failed:(0x%p,0x%p), size:%ld\n",
					__func__, __LINE__,
					config->level_tb,
					level_tb,
					sizeof(struct msync_level_table) *
					config->msync_level_num);
				return -EFAULT;
			}
		}
	}

	DDPMSG("%s:%d ---\n", __func__, __LINE__);

	return 0;
}

int mtk_drm_crtc_getfence_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_mtk_fence *args = data;
	struct mtk_drm_private *private;
	struct fence_data fence;
	unsigned int fence_idx;
	struct mtk_fence_info *l_info = NULL;
	int tl, idx;

	crtc = drm_crtc_find(dev, file_priv, args->crtc_id);
	if (!crtc) {
		DDPPR_ERR("Unknown CRTC ID %d\n", args->crtc_id);
		ret = -ENOENT;
		return ret;
	}
	DDPDBG("[CRTC:%d:%s]\n", crtc->base.id, crtc->name);

	idx = drm_crtc_index(crtc);
	if (!crtc->dev) {
		DDPPR_ERR("%s:%d dev is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}
	if (!crtc->dev->dev_private) {
		DDPPR_ERR("%s:%d dev private is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}
	private = crtc->dev->dev_private;
	fence_idx = atomic_read(&private->crtc_present[idx]);
	tl = mtk_fence_get_present_timeline_id(mtk_get_session_id(crtc));
	l_info = mtk_fence_get_layer_info(mtk_get_session_id(crtc), tl);
	if (!l_info) {
		DDPPR_ERR("%s:%d layer_info is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}

	/* async kick idle */
	mtk_drm_idlemgr_kick_async(crtc);

	/* create fence */
	fence.fence = MTK_INVALID_FENCE_FD;
	fence.value = ++fence_idx;
	atomic_inc(&private->crtc_present[idx]);
	ret = mtk_sync_fence_create(l_info->timeline, &fence);
	if (ret) {
		DDPPR_ERR("%d,L%d create Fence Object failed!\n",
			  MTK_SESSION_DEV(mtk_get_session_id(crtc)), tl);
		ret = -EFAULT;
	}

	args->fence_fd = fence.fence;
	args->fence_idx = fence.value;

	DDPFENCE("P+/%d/L%d/idx%d/fd%d\n",
		 MTK_SESSION_DEV(mtk_get_session_id(crtc)),
		 tl, args->fence_idx,
		 args->fence_fd);
	return ret;
}

int mtk_drm_crtc_fence_release_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_mtk_fence *args = data;

	crtc = drm_crtc_find(dev, file_priv, args->crtc_id);
	if (!crtc) {
		DDPPR_ERR("Unknown CRTC ID %d\n", args->crtc_id);
		ret = -ENOENT;
		return ret;
	}

	DDPINFO("%s: %d", __func__, args->crtc_id);

	mtk_drm_crtc_release_fence(crtc);

	return ret;
}

int mtk_drm_crtc_get_sf_fence_ioctl(struct drm_device *dev, void *data,
				    struct drm_file *file_priv)
{
	int ret = 0;
	struct drm_crtc *crtc;
	struct drm_mtk_fence *args = data;
	struct mtk_drm_private *private;
	struct fence_data fence;
	unsigned int fence_idx;
	struct mtk_fence_info *l_info = NULL;
	int tl, idx;

	crtc = drm_crtc_find(dev, file_priv, args->crtc_id);
	if (!crtc) {
		DDPPR_ERR("Unknown CRTC ID %d\n", args->crtc_id);
		ret = -ENOENT;
		return ret;
	}
	DDPDBG("[CRTC:%d:%s]\n", crtc->base.id, crtc->name);

	idx = drm_crtc_index(crtc);
	if (!crtc->dev) {
		DDPPR_ERR("%s:%d dev is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}
	if (!crtc->dev->dev_private) {
		DDPPR_ERR("%s:%d dev private is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}
	private = crtc->dev->dev_private;
	fence_idx = atomic_read(&private->crtc_sf_present[idx]);
	tl = mtk_fence_get_sf_present_timeline_id(mtk_get_session_id(crtc));
	l_info = mtk_fence_get_layer_info(mtk_get_session_id(crtc), tl);
	if (!l_info) {
		DDPPR_ERR("%s:%d layer_info is null\n", __func__, __LINE__);
		ret = -EFAULT;
		return ret;
	}

	if (mtk_drm_helper_get_opt(private->helper_opt, MTK_DRM_OPT_SF_PF)) {
		/* create fence */
		fence.fence = MTK_INVALID_FENCE_FD;
		fence.value = ++fence_idx;
		atomic_inc(&private->crtc_sf_present[idx]);
		ret = mtk_sync_fence_create(l_info->timeline, &fence);
		if (ret) {
			DDPPR_ERR("%d,L%d create Fence Object failed!\n",
				  MTK_SESSION_DEV(mtk_get_session_id(crtc)), tl);
			ret = -EFAULT;
		}

		args->fence_fd = fence.fence;
		args->fence_idx = fence.value;
	} else {
		args->fence_fd = MTK_INVALID_FENCE_FD;
		args->fence_idx = fence_idx;
	}

	DDPFENCE("P+/%d/L%d/idx%d/fd%d\n",
		 MTK_SESSION_DEV(mtk_get_session_id(crtc)),
		 tl, args->fence_idx,
		 args->fence_fd);
	return ret;
}

int mtk_drm_ioctl_get_pq_caps(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct mtk_drm_pq_caps_info *pq_info = data;

	mtk_get_ccorr_caps(&pq_info->ccorr_caps);
	memcpy(&drm_ccorr_caps, &pq_info->ccorr_caps, sizeof(drm_ccorr_caps));
	return 0;
}

int mtk_drm_ioctl_set_pq_caps(struct drm_device *dev, void *data,
	struct drm_file *file_priv)
{
	struct mtk_drm_pq_caps_info *pq_info = data;

	mtk_set_ccorr_caps(&pq_info->ccorr_caps);
	memcpy(&drm_ccorr_caps, &pq_info->ccorr_caps, sizeof(drm_ccorr_caps));
	return 0;
}


static int __crtc_need_composition_wb(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx;

	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];
	if (ddp_ctx->wb_comp_nr != 0)
		return true;
	else
		return false;
}

static void mtk_crtc_disconnect_single_path_cmdq(struct drm_crtc *crtc,
						 struct cmdq_pkt *cmdq_handle,
						 unsigned int path_idx,
						 unsigned int ddp_mode,
						 unsigned int mutex_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i;

	for (i = 0; i < __mtk_crtc_path_len(mtk_crtc, ddp_mode, path_idx) - 1; ++i) {
		struct mtk_ddp_comp *temp_comp;

		comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, i);
		temp_comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, i + 1);

		if (comp && temp_comp)
			mtk_ddp_remove_comp_from_path_with_cmdq(
				mtk_crtc, comp->id, temp_comp->id, cmdq_handle);
		else {
			DDPPR_ERR("%s, comp or temp_comp is NULL\n", __func__);
			break;
		}
	}
	for_each_comp_in_crtc_target_mode_path(comp, mtk_crtc, i, ddp_mode, path_idx)
		mtk_disp_mutex_remove_comp_with_cmdq(
			mtk_crtc, comp->id, cmdq_handle, mutex_id);
}

static void mtk_crtc_connect_single_path_cmdq(struct drm_crtc *crtc,
					      struct cmdq_pkt *cmdq_handle,
					      unsigned int path_idx,
					      unsigned int ddp_mode,
					      unsigned int mutex_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	unsigned int i;

	for (i = 0; i < __mtk_crtc_path_len(mtk_crtc, ddp_mode, path_idx) - 1; ++i) {
		struct mtk_ddp_comp *temp_comp;
		unsigned int prev_id;

		if (i == 0) {
			prev_id = DDP_COMPONENT_ID_MAX;
		} else {
			comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, i - 1);
			prev_id = comp->id;
		}

		comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, i);
		temp_comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, i + 1);

		if (comp && temp_comp)
			mtk_ddp_add_comp_to_path_with_cmdq(
				mtk_crtc, comp->id, prev_id, temp_comp->id, cmdq_handle);
		else {
			DDPPR_ERR("%s, comp or temp_comp is NULL\n", __func__);
			break;
		}
	}
	for_each_comp_in_crtc_target_mode_path(comp, mtk_crtc, i, ddp_mode, path_idx)
		mtk_disp_mutex_add_comp_with_cmdq(
			mtk_crtc, comp->id,
			mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base),
			cmdq_handle, mutex_id);
}

static void mtk_crtc_config_dual_pipe_cmdq(struct mtk_drm_crtc *mtk_crtc,
					     struct cmdq_pkt *cmdq_handle,
					     unsigned int path_idx,
					     struct mtk_ddp_config *cfg)
{
	//struct mtk_drm_private *priv = mtk_crtc->base.dev->dev_private;
	struct mtk_ddp_comp *comp;
	int i;

	DDPFUNC();
	for_each_comp_in_dual_pipe(comp, mtk_crtc, path_idx, i) {
		if (comp->id == DDP_COMPONENT_RDMA4 ||
		    comp->id == DDP_COMPONENT_RDMA5) {
			bool *rdma_memory_mode =
				comp->comp_mode;

				*rdma_memory_mode = false;
			break;
		}
	}

	for_each_comp_in_dual_pipe(comp, mtk_crtc, path_idx, i) {
		mtk_ddp_comp_config(comp, cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);
	}
}

static void mtk_crtc_config_single_path_cmdq(struct drm_crtc *crtc,
					     struct cmdq_pkt *cmdq_handle,
					     unsigned int path_idx,
					     unsigned int ddp_mode,
					     struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_ddp_comp *comp;
	int i;

	for_each_comp_in_crtc_target_mode_path(comp, mtk_crtc, i, ddp_mode, path_idx) {
		if (comp->id == DDP_COMPONENT_RDMA0 ||
		    comp->id == DDP_COMPONENT_RDMA1 ||
		    comp->id == DDP_COMPONENT_RDMA2 ||
		    comp->id == DDP_COMPONENT_RDMA4 ||
		    comp->id == DDP_COMPONENT_RDMA5) {
			bool *rdma_memory_mode = comp->comp_mode;

			if (i == 0)
				*rdma_memory_mode = true;
			else
				*rdma_memory_mode = false;
			break;
		}
	}

	for_each_comp_in_crtc_target_mode_path(comp, mtk_crtc, i, ddp_mode, path_idx) {
		mtk_ddp_comp_config(comp, cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);
		if (!mtk_drm_helper_get_opt(
				    priv->helper_opt,
				    MTK_DRM_OPT_USE_PQ))
			mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
	}

	if (mtk_crtc->is_dual_pipe)
		mtk_crtc_config_dual_pipe_cmdq(mtk_crtc, cmdq_handle,
					     path_idx, cfg);
}

static void mtk_crtc_create_wb_path_cmdq(struct drm_crtc *crtc,
					 struct cmdq_pkt *cmdq_handle,
					 unsigned int ddp_mode,
					 unsigned int mutex_id,
					 struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	struct mtk_drm_gem_obj *mtk_gem;
	struct mtk_ddp_comp *comp;
	dma_addr_t addr;
	unsigned int i;

	ddp_ctx = &mtk_crtc->ddp_ctx[ddp_mode];
	for (i = 0; i < ddp_ctx->wb_comp_nr - 1; i++) {
		unsigned int prev_id;

		if (i == 0)
			prev_id = DDP_COMPONENT_ID_MAX;
		else
			prev_id = ddp_ctx->wb_comp[i - 1]->id;

		mtk_ddp_add_comp_to_path_with_cmdq(
			mtk_crtc, ddp_ctx->wb_comp[i]->id, prev_id,
			ddp_ctx->wb_comp[i + 1]->id, cmdq_handle);
	}
	for (i = 0; i < ddp_ctx->wb_comp_nr; i++)
		mtk_disp_mutex_add_comp_with_cmdq(
			mtk_crtc, ddp_ctx->wb_comp[i]->id,
			mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base),
			cmdq_handle, mutex_id);

	if (!ddp_ctx->wb_fb) {
		struct drm_mode_fb_cmd2 mode = {0};

		mode.width = crtc->state->adjusted_mode.hdisplay;
		mode.height = crtc->state->adjusted_mode.vdisplay;
		mtk_gem = mtk_drm_gem_create(
			crtc->dev, mode.width * mode.height * 3, true);
		mode.pixel_format = DRM_FORMAT_RGB888;
		mode.pitches[0] = mode.width * 3;

		ddp_ctx->wb_fb = mtk_drm_framebuffer_create(
			crtc->dev, &mode, &mtk_gem->base);
	}

	for (i = 0; i < ddp_ctx->wb_comp_nr; i++) {
		comp = ddp_ctx->wb_comp[i];
		if (comp->id == DDP_COMPONENT_WDMA0 ||
		    comp->id == DDP_COMPONENT_WDMA1 ||
		    comp->id == DDP_COMPONENT_OVLSYS_WDMA2) {
			comp->fb = ddp_ctx->wb_fb;
			break;
		}
	}

	/* All the 1to2 path shoulbe be real-time */
	for (i = 0; i < ddp_ctx->wb_comp_nr; i++) {
		struct mtk_ddp_comp *comp = ddp_ctx->wb_comp[i];

		mtk_ddp_comp_config(comp, cfg, cmdq_handle);
		mtk_ddp_comp_start(comp, cmdq_handle);
		if (!mtk_drm_helper_get_opt(
				    priv->helper_opt,
				    MTK_DRM_OPT_USE_PQ))
			mtk_ddp_comp_bypass(comp, 1, cmdq_handle);
	}

	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_RDMA_FB_ID);
	cmdq_pkt_write(cmdq_handle, mtk_crtc->gce_obj.base,
		addr, 0, ~0);
}

static void mtk_crtc_destroy_wb_path_cmdq(struct drm_crtc *crtc,
					  struct cmdq_pkt *cmdq_handle,
					  unsigned int mutex_id,
					  struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	int i;

	if (!__crtc_need_composition_wb(crtc))
		return;

	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];
	for (i = 0; i < ddp_ctx->wb_comp_nr - 1; i++)
		mtk_ddp_remove_comp_from_path_with_cmdq(
			mtk_crtc, ddp_ctx->wb_comp[i]->id,
			ddp_ctx->wb_comp[i + 1]->id, cmdq_handle);

	for (i = 0; i < ddp_ctx->wb_comp_nr; i++)
		mtk_disp_mutex_remove_comp_with_cmdq(mtk_crtc,
						     ddp_ctx->wb_comp[i]->id,
						     cmdq_handle, mutex_id);
}

static void mtk_crtc_config_wb_path_cmdq(struct drm_crtc *crtc,
					 struct cmdq_pkt *cmdq_handle,
					 unsigned int path_idx,
					 unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	struct mtk_plane_state plane_state;
	struct drm_framebuffer *fb = NULL;
	struct mtk_ddp_comp *comp;
	int i;

	if (!__crtc_need_composition_wb(crtc))
		return;

	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];
	for (i = 0; i < ddp_ctx->wb_comp_nr; i++) {
		comp = ddp_ctx->wb_comp[i];
		if (comp->id == DDP_COMPONENT_WDMA0 ||
		    comp->id == DDP_COMPONENT_WDMA1 ||
		    comp->id == DDP_COMPONENT_OVLSYS_WDMA2) {
			fb = comp->fb;
			break;
		}
	}

	if (!fb) {
		DDPPR_ERR("%s, fb is empty\n", __func__);
		return;
	}
	plane_state.pending.enable = true;
	plane_state.pending.pitch = fb->pitches[0];
	plane_state.pending.format = fb->format->format;
	plane_state.pending.addr = mtk_fb_get_dma(fb);
	plane_state.pending.size = mtk_fb_get_size(fb);
	plane_state.pending.src_x = 0;
	plane_state.pending.src_y = 0;
	plane_state.pending.dst_x = 0;
	plane_state.pending.dst_y = 0;
	plane_state.pending.width = fb->width;
	plane_state.pending.height = fb->height;
	comp = mtk_crtc_get_comp(crtc, ddp_mode, path_idx, 0);
	mtk_ddp_comp_layer_config(comp, 0,
				  &plane_state, cmdq_handle);
}

static int __mtk_crtc_composition_wb(
				     struct drm_crtc *crtc,
				     unsigned int ddp_mode,
				     struct mtk_ddp_config *cfg)
{
	struct cmdq_pkt *cmdq_handle;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int gce_event;

	if (!__crtc_need_composition_wb(crtc))
		return 0;

	DDPINFO("%s\n", __func__);

	gce_event = get_path_wait_event(mtk_crtc, mtk_crtc->ddp_mode);
	mtk_crtc_pkt_create(&cmdq_handle, crtc,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);
	mtk_crtc_create_wb_path_cmdq(crtc, cmdq_handle, mtk_crtc->ddp_mode, 0,
				     cfg);
	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
	if (gce_event > 0) {
		cmdq_pkt_clear_event(cmdq_handle, gce_event);
		cmdq_pkt_wait_no_clear(cmdq_handle, gce_event);
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	return 0;
}

bool mtk_crtc_with_sub_path(struct drm_crtc *crtc, unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	return mtk_crtc->ddp_ctx[ddp_mode].ddp_comp_nr[1];
}

static void __mtk_crtc_prim_path_switch(struct drm_crtc *crtc,
					unsigned int ddp_mode,
					struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *cmdq_handle;
	int cur_path_idx, next_path_idx;

	DDPINFO("%s\n", __func__);

	if (mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		cur_path_idx = DDP_SECOND_PATH;
	else
		cur_path_idx = DDP_FIRST_PATH;
	if (mtk_crtc_with_sub_path(crtc, ddp_mode))
		next_path_idx = DDP_SECOND_PATH;
	else
		next_path_idx = DDP_FIRST_PATH;

	mtk_crtc_pkt_create(&cmdq_handle,
		crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		cmdq_pkt_clear_event(
			cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);
	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, cur_path_idx, 0);

	/* 1. Disconnect current path and remove component mutexs */
	if (!mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode)) {
		_mtk_crtc_atmoic_addon_module_disconnect(
			crtc, mtk_crtc->ddp_mode, &crtc_state->lye_state,
			cmdq_handle);
	}
	mtk_crtc_addon_connector_disconnect(crtc, cmdq_handle);
	mtk_crtc_disconnect_single_path_cmdq(crtc, cmdq_handle, cur_path_idx,
					     mtk_crtc->ddp_mode, 0);

	/* 2. Remove composition path and cooresonding mutexs */
	mtk_crtc_destroy_wb_path_cmdq(crtc, cmdq_handle, 0, cfg);

	/* 3. Connect new primary path and add component mutexs */
	mtk_crtc_connect_single_path_cmdq(crtc, cmdq_handle, next_path_idx,
					  ddp_mode, 0);
	/* TODO: refine addon_connector */
	mtk_crtc_addon_connector_connect(crtc, cmdq_handle);

	/* 4. Primary path configurations */
	cfg->p_golden_setting_context =
				__get_golden_setting_context(mtk_crtc);
	if (mtk_crtc_target_is_dc_mode(crtc, ddp_mode))
		cfg->p_golden_setting_context->is_dc = 1;
	else
		cfg->p_golden_setting_context->is_dc = 0;

	mtk_crtc_config_single_path_cmdq(crtc, cmdq_handle, next_path_idx,
					 ddp_mode, cfg);

	if (!mtk_crtc_with_sub_path(crtc, ddp_mode))
		_mtk_crtc_atmoic_addon_module_connect(
			crtc, ddp_mode, &crtc_state->lye_state, cmdq_handle);

	/* 5. Set composed write back buffer */
	mtk_crtc_config_wb_path_cmdq(crtc, cmdq_handle, next_path_idx,
				     ddp_mode);

	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		cmdq_pkt_set_event(cmdq_handle,
				   mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

static void __mtk_crtc_old_sub_path_destroy(struct drm_crtc *crtc,
					    struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *cmdq_handle;
	int i;
	struct mtk_ddp_comp *comp = NULL;
	int index = drm_crtc_index(crtc);

	if (!mtk_crtc_with_sub_path(crtc, mtk_crtc->ddp_mode))
		return;

	mtk_crtc_pkt_create(&cmdq_handle, crtc,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	mtk_crtc_wait_frame_done(mtk_crtc, cmdq_handle, DDP_FIRST_PATH, 0);
	_mtk_crtc_atmoic_addon_module_disconnect(crtc, mtk_crtc->ddp_mode,
					&crtc_state->lye_state,
					cmdq_handle);

	for_each_comp_in_crtc_target_path(comp, mtk_crtc, i, DDP_FIRST_PATH)
		mtk_ddp_comp_stop(comp, cmdq_handle);

	mtk_crtc_disconnect_single_path_cmdq(crtc, cmdq_handle,
		DDP_FIRST_PATH,	mtk_crtc->ddp_mode, 1);

	/* Workaround: if CRTC0, reset wdma->fb to NULL to prevent CRTC2
	 * config wdma and cause KE
	 */
	if (index == 0) {
		comp = mtk_ddp_comp_find_by_id(crtc, DDP_COMPONENT_WDMA0);
		if (!comp)
			comp = mtk_ddp_comp_find_by_id(crtc,
					DDP_COMPONENT_WDMA1);
		if (!comp)
			comp = mtk_ddp_comp_find_by_id(crtc,
					DDP_COMPONENT_OVLSYS_WDMA2);
		if (comp)
			comp->fb = NULL;
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

static void __mtk_crtc_sub_path_create(
				       struct drm_crtc *crtc,
				       unsigned int ddp_mode,
				       struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);
	struct cmdq_pkt *cmdq_handle;

	if (!mtk_crtc_with_sub_path(crtc, ddp_mode))
		return;

	mtk_crtc_pkt_create(&cmdq_handle, crtc,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base))
		cmdq_pkt_clear_event(
			cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_DIRTY]);

	/* 1. Connect Sub Path*/
	mtk_crtc_connect_single_path_cmdq(crtc, cmdq_handle, DDP_FIRST_PATH,
					  ddp_mode, 1);

	/* 2. Sub path configuration */
	mtk_crtc_config_single_path_cmdq(crtc, cmdq_handle, DDP_FIRST_PATH,
					 ddp_mode, cfg);

	_mtk_crtc_atmoic_addon_module_connect(
		crtc, ddp_mode, &crtc_state->lye_state, cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

static void mtk_crtc_dc_fb_control(struct drm_crtc *crtc,
				unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_ddp_ctx *ddp_ctx;
	struct drm_mode_fb_cmd2 mode = {0};
	struct mtk_drm_gem_obj *mtk_gem;

	DDPINFO("%s\n", __func__);

	ddp_ctx = &mtk_crtc->ddp_ctx[ddp_mode];
	if (mtk_crtc_target_is_dc_mode(crtc, ddp_mode) &&
		ddp_ctx->dc_fb == NULL) {
		ddp_ctx = &mtk_crtc->ddp_ctx[ddp_mode];
		mode.width = crtc->state->adjusted_mode.hdisplay;
		mode.height = crtc->state->adjusted_mode.vdisplay;
		mtk_gem = mtk_drm_gem_create(
			crtc->dev,
			mtk_crtc_get_dc_fb_size(crtc) * MAX_CRTC_DC_FB, true);
		mode.pixel_format = DRM_FORMAT_RGB888;
		mode.pitches[0] = mode.width * 3;
		ddp_ctx->dc_fb = mtk_drm_framebuffer_create(crtc->dev, &mode,
							    &mtk_gem->base);
	}

	/* do not create wb_fb & dc buffer repeatedly */
#ifdef IF_ZERO
	ddp_ctx = &mtk_crtc->ddp_ctx[mtk_crtc->ddp_mode];
	if (!mtk_crtc_target_is_dc_mode(crtc, mtk_crtc->ddp_mode)
		&& ddp_ctx->dc_fb) {
		drm_framebuffer_cleanup(ddp_ctx->dc_fb);
		ddp_ctx->dc_fb_idx = 0;
	}

	if (ddp_ctx->wb_fb) {
		drm_framebuffer_cleanup(ddp_ctx->wb_fb);
		ddp_ctx->wb_fb = NULL;
	}
#endif
}

void mtk_crtc_path_switch_prepare(struct drm_crtc *crtc, unsigned int ddp_mode,
				  struct mtk_ddp_config *cfg)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	cfg->w = crtc->state->adjusted_mode.hdisplay;
	cfg->h = crtc->state->adjusted_mode.vdisplay;
	if (mtk_crtc->panel_ext && mtk_crtc->panel_ext->params &&
		mtk_crtc->panel_ext->params->dyn_fps.switch_en == 1
		&& mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps != 0)
		cfg->vrefresh =
			mtk_crtc->panel_ext->params->dyn_fps.vact_timing_fps;
	else
		cfg->vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);
	cfg->bpc = mtk_crtc->bpc;
	cfg->x = 0;
	cfg->y = 0;
	cfg->p_golden_setting_context = __get_golden_setting_context(mtk_crtc);

	/* The components in target ddp_mode may be used during path switching,
	 * so attach
	 * the CRTC to them.
	 */
	mtk_crtc_attach_ddp_comp(crtc, ddp_mode, true);
}

void mtk_crtc_path_switch_update_ddp_status(struct drm_crtc *crtc,
					    unsigned int ddp_mode)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, false);
	mtk_crtc_attach_ddp_comp(crtc, ddp_mode, true);
}

void mtk_need_vds_path_switch(struct drm_crtc *crtc)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int index = drm_crtc_index(crtc);
	int i = 0;
	int comp_nr = 0;

	if (!(priv && mtk_crtc && (index >= 0))) {
		DDPPR_ERR("%s:%d:Error Invalid params\n");
		return;
	}

	/* In order to confirm it will be called one time,
	 * when switch to or switch back, So need some flags
	 * to control it.
	 */
	if (priv->vds_path_switch_dirty &&
		!priv->vds_path_switch_done && (index == 0)) {

		/* kick idle */
		mtk_drm_idlemgr_kick(__func__, crtc, 0);
		CRTC_MMP_EVENT_START(index, path_switch, mtk_crtc->ddp_mode, 0);

		/* Switch main display path, take away ovl0_2l from main display */
		if (priv->need_vds_path_switch) {
			struct mtk_ddp_comp *comp_ovl0;
			struct mtk_ddp_comp *comp_ovl0_2l;
			struct cmdq_pkt *cmdq_handle;
			struct mtk_crtc_state *crtc_state = to_mtk_crtc_state(crtc->state);

			cmdq_handle =
				cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

			if (!cmdq_handle) {
				DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
				return;
			}

			mtk_crtc_wait_frame_done(mtk_crtc,
					cmdq_handle, DDP_FIRST_PATH, 0);
			/* Disconnect current path and remove component mutexs */
			_mtk_crtc_atmoic_addon_module_disconnect(
				crtc, mtk_crtc->ddp_mode,
				&crtc_state->lye_state, cmdq_handle);
			/* Stop ovl 2l */
			comp_ovl0_2l = priv->ddp_comp[DDP_COMPONENT_OVL0_2L];
			mtk_ddp_comp_stop(comp_ovl0_2l, cmdq_handle);
			/* Change ovl0 bg mode frmoe DL mode to const mode */
			comp_ovl0 = priv->ddp_comp[DDP_COMPONENT_OVL0];
			cmdq_pkt_write(cmdq_handle, comp_ovl0->cmdq_base,
				comp_ovl0->regs_pa + DISP_OVL_DATAPATH_CON, 0x0, 0x4);
			mtk_ddp_remove_comp_from_path_with_cmdq(
				mtk_crtc, DDP_COMPONENT_OVL0_2L,
				DDP_COMPONENT_OVL0, cmdq_handle);
			mtk_disp_mutex_remove_comp_with_cmdq(
					mtk_crtc, DDP_COMPONENT_OVL0_2L, cmdq_handle, 0);
			cmdq_pkt_flush(cmdq_handle);
			cmdq_pkt_destroy(cmdq_handle);
			/* unprepare ovl 2l */
			mtk_ddp_comp_unprepare(comp_ovl0_2l);

			CRTC_MMP_MARK(index, path_switch, 0xFFFF, 1);
			DDPMSG("Switch vds: Switch ovl0_2l to vds\n");

			/* Update ddp ctx ddp_comp_nr */
			mtk_crtc->ddp_ctx[DDP_MAJOR].ddp_comp_nr[DDP_FIRST_PATH]
				= mtk_crtc->path_data->path_len[
				DDP_MAJOR][DDP_FIRST_PATH] - 1;
			/* Update ddp ctx ddp_comp */
			comp_nr = mtk_crtc->path_data->path_len[
				DDP_MAJOR][DDP_FIRST_PATH];
			for (i = 0; i < comp_nr - 1; i++)
				mtk_crtc->ddp_ctx[
					DDP_MAJOR].ddp_comp[DDP_FIRST_PATH][i] =
					mtk_crtc->ddp_ctx[
					DDP_MAJOR].ddp_comp[DDP_FIRST_PATH][i+1];
			mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);
			/* Update Switch done flag */
			priv->vds_path_switch_done = 1;
		/* Switch main display path, take back ovl0_2l to main display */
		} else {
			struct mtk_ddp_comp *comp_ovl0;
			struct mtk_ddp_comp *comp_ovl0_2l;
			int width = crtc->state->adjusted_mode.hdisplay;
			int height = crtc->state->adjusted_mode.vdisplay;
			struct cmdq_pkt *cmdq_handle;

			cmdq_handle =
				cmdq_pkt_create(mtk_crtc->gce_obj.client[CLIENT_CFG]);

			if (!cmdq_handle) {
				DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
				return;
			}

			mtk_crtc_wait_frame_done(mtk_crtc,
					cmdq_handle, DDP_FIRST_PATH, 0);

			/* Change ovl0 bg mode frmoe const mode to DL mode */
			comp_ovl0 = priv->ddp_comp[DDP_COMPONENT_OVL0];
			cmdq_pkt_write(cmdq_handle, comp_ovl0->cmdq_base,
				comp_ovl0->regs_pa + DISP_OVL_DATAPATH_CON, 0x4, 0x4);

			/* Change ovl0 ROI size */
			comp_ovl0_2l = priv->ddp_comp[DDP_COMPONENT_OVL0_2L];
			cmdq_pkt_write(cmdq_handle, comp_ovl0_2l->cmdq_base,
				comp_ovl0_2l->regs_pa + DISP_OVL_ROI_SIZE,
				height << 16 | width, ~0);

			/* Connect cur path components */
			mtk_ddp_add_comp_to_path_with_cmdq(
				mtk_crtc, DDP_COMPONENT_OVL0_2L, DDP_COMPONENT_ID_MAX,
				DDP_COMPONENT_OVL0, cmdq_handle);
			mtk_disp_mutex_add_comp_with_cmdq(
				mtk_crtc, DDP_COMPONENT_OVL0_2L,
				mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base),
				cmdq_handle, 0);

			/* Switch back need reprepare ovl0_2l */
			mtk_ddp_comp_prepare(comp_ovl0_2l);
			mtk_ddp_comp_start(comp_ovl0_2l, cmdq_handle);

			cmdq_pkt_flush(cmdq_handle);
			cmdq_pkt_destroy(cmdq_handle);

			CRTC_MMP_MARK(index, path_switch, 0xFFFF, 2);
			DDPMSG("Switch vds: Switch ovl0_2l to main disp\n");

			/* Update ddp ctx ddp_comp_nr */
			mtk_crtc->ddp_ctx[DDP_MAJOR].ddp_comp_nr[DDP_FIRST_PATH]
				= mtk_crtc->path_data->path_len[
				DDP_MAJOR][DDP_FIRST_PATH];
			/* Update ddp ctx ddp_comp */
			comp_nr = mtk_crtc->path_data->path_len[
				DDP_MAJOR][DDP_FIRST_PATH];
			for (i = comp_nr - 1; i > 0; i--)
				mtk_crtc->ddp_ctx[
					DDP_MAJOR].ddp_comp[DDP_FIRST_PATH][i] =
					mtk_crtc->ddp_ctx[
					DDP_MAJOR].ddp_comp[DDP_FIRST_PATH][i-1];
			mtk_crtc->ddp_ctx[
				DDP_MAJOR].ddp_comp[DDP_FIRST_PATH][0] =
				priv->ddp_comp[DDP_COMPONENT_OVL0_2L];
			mtk_crtc_attach_ddp_comp(crtc, mtk_crtc->ddp_mode, true);
			/* Update Switch done flag */
			priv->vds_path_switch_dirty = 0;
		}

		DDPMSG("Switch vds: Switch ovl0_2l Done\n");
		CRTC_MMP_EVENT_END(index, path_switch, crtc->enabled, 0);
	}
}

int mtk_crtc_path_switch(struct drm_crtc *crtc, unsigned int ddp_mode,
			 int need_lock)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_config cfg;
	int index = drm_crtc_index(crtc);
	bool need_wait;

	if (ddp_mode > DDP_NO_USE)	/* ddp_mode wrong */
		return 0;

	CRTC_MMP_EVENT_START(index, path_switch, mtk_crtc->ddp_mode,
			ddp_mode);

	if (need_lock)
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	if (ddp_mode == mtk_crtc->ddp_mode || !crtc->enabled) {
		DDPINFO("CRTC%d skip path switch %u->%u, enable:%d\n",
			index, mtk_crtc->ddp_mode,
			ddp_mode, crtc->enabled);
		CRTC_MMP_MARK(index, path_switch, 0, 0);
		goto done;
	}
	if ((index == 0) && !mtk_crtc->enabled) {
		DDPINFO("CRTC%d skip path switch %u->%u, enable:%d\n",
			index, mtk_crtc->ddp_mode,
			ddp_mode, mtk_crtc->enabled);
		CRTC_MMP_MARK(index, path_switch, 0, 1);
		goto done2;
	}

	mtk_drm_crtc_wait_blank(mtk_crtc);

	DDPINFO("%s crtc%d path switch(%d->%d)\n", __func__, index,
		mtk_crtc->ddp_mode, ddp_mode);
	mtk_drm_idlemgr_kick(__func__, crtc, 0);
	CRTC_MMP_MARK(index, path_switch, 1, 0);

	/* 0. Special NO_USE ddp mode control. In NO_USE ddp mode, the HW path
	 *     is disabled currently or to be disabled. So the control use the
	 *     CRTC enable/disable function to create/destroy path.
	 */
	if (ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK(index, path_switch, 0, 2);
		if ((mtk_crtc->ddp_mode == DDP_MAJOR) && (index == 2))
			need_wait = false;
		else
			need_wait = true;
		mtk_drm_crtc_disable(crtc, need_wait);
		goto done;
	} else if (mtk_crtc->ddp_mode == DDP_NO_USE) {
		CRTC_MMP_MARK(index, path_switch, 0, 3);
		mtk_crtc->ddp_mode = ddp_mode;
		mtk_drm_crtc_enable(crtc);
		goto done;
	}

	mtk_crtc_path_switch_prepare(crtc, ddp_mode, &cfg);

	/* 1 Destroy original sub path */
	__mtk_crtc_old_sub_path_destroy(crtc, &cfg);

	/* 2 Composing planes and write back to buffer */
	__mtk_crtc_composition_wb(crtc, ddp_mode, &cfg);

	/* 3. Composing planes and write back to buffer */
	__mtk_crtc_prim_path_switch(crtc, ddp_mode, &cfg);
	CRTC_MMP_MARK(index, path_switch, 1, 1);

	/* 4. Create new sub path */
	__mtk_crtc_sub_path_create(crtc, ddp_mode, &cfg);

	/* 5. create and destroy the fb used in ddp */
	mtk_crtc_dc_fb_control(crtc, ddp_mode);

done:
	mtk_crtc->ddp_mode = ddp_mode;
	if (crtc->enabled && mtk_crtc->ddp_mode != DDP_NO_USE)
		mtk_crtc_update_ddp_sw_status(crtc, true);

	if (need_lock)
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	CRTC_MMP_EVENT_END(index, path_switch, crtc->enabled,
			need_lock);

	DDPINFO("%s:%d -\n", __func__, __LINE__);
	return 0;

done2:
	if (need_lock)
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	CRTC_MMP_EVENT_END(index, path_switch, crtc->enabled,
			need_lock);

	DDPINFO("%s:%d -\n", __func__, __LINE__);
	return 0;
}

int mtk_crtc_get_mutex_id(struct drm_crtc *crtc, unsigned int ddp_mode,
			  enum mtk_ddp_comp_id find_comp)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int crtc_id = drm_crtc_index(crtc);
	int i, j;
	struct mtk_ddp_comp *comp;
	int find = -1;
	bool has_connector = true;

	for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j,
					 ddp_mode)
		if (comp->id == find_comp) {
			find = i;
			break;
		}

	if (find == -1 && mtk_crtc->is_dual_pipe) {
		i = j = 0;
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
			if (comp->id == find_comp) {
				find = i;
				break;
			}
	}

	if (find == -1) {
		DDPPR_ERR("component %d is not found in path %d of crtc %d\n",
			  find_comp, ddp_mode, crtc_id);
		return -1;
	}

	if (mtk_crtc_with_sub_path(crtc, ddp_mode) && find == 0)
		has_connector = false;

	switch (crtc_id) {
	case 0:
		if (has_connector)
			return 0;
		else
			return 1;
	case 1:
		if (has_connector)
			return 2;
		else
			return 3;
	case 2:
		if (has_connector)
			return 4;
		else
			return 5;
	default:
		DDPPR_ERR("not define mutex id in crtc %d\n", crtc_id);
	}

	return -1;
}

void mtk_crtc_disconnect_path_between_component(struct drm_crtc *crtc,
						unsigned int ddp_mode,
						enum mtk_ddp_comp_id prev,
						enum mtk_ddp_comp_id next,
						struct cmdq_pkt *cmdq_handle)
{
	int i, j;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int mutex_id = mtk_crtc_get_mutex_id(crtc, ddp_mode, prev);
	int find_idx = -1;

	if (mutex_id < 0) {
		DDPPR_ERR("invalid mutex id:%d\n", mutex_id);
		return;
	}

	for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j, ddp_mode) {
		if (comp->id == prev)
			find_idx = j;

		if (find_idx > -1 && j > find_idx && j > 0) {
			struct mtk_ddp_comp *temp_comp;

			temp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j - 1);

			if (temp_comp)
				mtk_ddp_remove_comp_from_path_with_cmdq(
					mtk_crtc, temp_comp->id,
					comp->id, cmdq_handle);
			else {
				DDPPR_ERR("%s, temp_comp is NULL\n", __func__);
				break;
			}

			if (comp->id != next)
				mtk_disp_mutex_remove_comp_with_cmdq(
					mtk_crtc, comp->id, cmdq_handle,
					mutex_id);
		}

		if (comp->id == next)
			return;
	}


	if (!mtk_crtc->is_dual_pipe)
		return;

	i = j = 0;
	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		if (comp->id == prev)
			find_idx = j;

		if (find_idx > -1 && j > find_idx) {
			struct mtk_ddp_comp *temp_comp;

			temp_comp = mtk_crtc_get_dual_comp(crtc, i, j - 1);
			mtk_ddp_remove_comp_from_path_with_cmdq(
					mtk_crtc, temp_comp->id,
					comp->id, cmdq_handle);

			if (comp->id != next)
				mtk_disp_mutex_remove_comp_with_cmdq(
						mtk_crtc, comp->id, cmdq_handle,
						mutex_id);
		}

		if (comp->id == next)
			return;
	}
}

void mtk_crtc_connect_path_between_component(struct drm_crtc *crtc,
					     unsigned int ddp_mode,
					     enum mtk_ddp_comp_id prev,
					     enum mtk_ddp_comp_id next,
					     struct cmdq_pkt *cmdq_handle)
{
	int i, j;
	struct mtk_ddp_comp *comp;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int mutex_id = mtk_crtc_get_mutex_id(crtc, ddp_mode, prev);
	int find_idx = -1;

	if (mutex_id < 0) {
		DDPPR_ERR("invalid mutex id:%d\n", mutex_id);
		return;
	}

	for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j, ddp_mode) {
		if (comp->id == prev)
			find_idx = j;

		if (find_idx > -1 && j > find_idx && j > 0) {
			struct mtk_ddp_comp *temp_comp;
			unsigned int prev_id;

			if (j < 2) {
				prev_id = DDP_COMPONENT_ID_MAX;
			} else {
				temp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j - 2);
				prev_id = temp_comp ? temp_comp->id : -EINVAL;
			}
			temp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, j - 1);

			if (!temp_comp || prev_id > DDP_COMPONENT_ID_MAX) {
				DDPPR_ERR("%s, temp_comp is NULL or invalid prev_id\n",
						__func__, prev_id);
				break;
			}

			mtk_ddp_add_comp_to_path_with_cmdq(
				mtk_crtc, temp_comp->id, prev_id,
				comp->id, cmdq_handle);

			if (comp->id != next)
				mtk_disp_mutex_add_comp_with_cmdq(
					mtk_crtc, comp->id,
					mtk_crtc_is_frame_trigger_mode(crtc),
					cmdq_handle, mutex_id);
		}

		if (comp->id == next)
			return;
	}

	if (!mtk_crtc->is_dual_pipe)
		return;

	i = j = 0;

	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		if (comp->id == prev)
			find_idx = j;

		if (find_idx > -1 && j > find_idx) {
			struct mtk_ddp_comp *temp_comp;
			unsigned int prev_id;

			if (j < 2) {
				prev_id = DDP_COMPONENT_ID_MAX;
			} else {
				temp_comp = mtk_crtc_get_dual_comp(crtc, i, j - 2);
				prev_id = temp_comp ? temp_comp->id : -EINVAL;
			}
			temp_comp = mtk_crtc_get_dual_comp(crtc, i, j - 1);

			if (!temp_comp || prev_id > DDP_COMPONENT_ID_MAX) {
				DDPPR_ERR("%s, temp_comp is NULL or prev_id %d\n",
						__func__, prev_id);
				break;
			}

			mtk_ddp_add_comp_to_path_with_cmdq(
					mtk_crtc, temp_comp->id, prev_id,
					comp->id, cmdq_handle);

			if (comp->id != next)
				mtk_disp_mutex_add_comp_with_cmdq(
						mtk_crtc, comp->id,
						mtk_crtc_is_frame_trigger_mode(crtc),
						cmdq_handle, mutex_id);
		}

		if (comp->id == next)
			return;
	}
}

int mtk_crtc_find_comp(struct drm_crtc *crtc, unsigned int ddp_mode,
		       enum mtk_ddp_comp_id comp_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i, j;

	for_each_comp_in_target_ddp_mode(
		comp, mtk_crtc, i, j,
		ddp_mode)
		if (comp->id == comp_id &&
			mtk_ddp_comp_get_type(comp_id) !=
			MTK_DISP_VIRTUAL)
			return comp->id;

	if (!mtk_crtc->is_dual_pipe)
		return -1;

	i = j = 0;
	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j)
		if (comp->id == comp_id &&
				mtk_ddp_comp_get_type(comp_id) !=
				MTK_DISP_VIRTUAL)
			return comp->id;

	return -1;
}

int mtk_crtc_find_next_comp(struct drm_crtc *crtc, unsigned int ddp_mode,
			    enum mtk_ddp_comp_id comp_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int id;
	int i, j, k;

	for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j, ddp_mode) {
		if (comp->id == comp_id) {
			for (k = j + 1; k < __mtk_crtc_path_len(mtk_crtc, ddp_mode, i) ; k++) {
				struct mtk_ddp_comp *temp_comp;

				temp_comp = mtk_crtc_get_comp(crtc, mtk_crtc->ddp_mode, i, k);
				id = temp_comp ? temp_comp->id : -EINVAL;
				if ((id != -EINVAL) &&
					(mtk_ddp_comp_get_type(id) != MTK_DISP_VIRTUAL))
					return id;
				else
					return -1;
			}

			return -1;
		}
	}

	if (!mtk_crtc->is_dual_pipe)
		return -1;

	i = j = 0;
	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		if (comp->id == comp_id) {
			struct mtk_ddp_comp *temp_comp;

			for (k = j + 1; k < __mtk_crtc_dual_path_len(mtk_crtc, i); k++) {
				temp_comp = mtk_crtc_get_dual_comp(crtc, i, k);
				id = temp_comp->id;
				if (mtk_ddp_comp_get_type(id) !=
						MTK_DISP_VIRTUAL)
					return id;
			}

			return -1;
		}
	}

	return -1;
}

int mtk_crtc_find_prev_comp(struct drm_crtc *crtc, unsigned int ddp_mode,
			    enum mtk_ddp_comp_id comp_id)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int real_comp_id = -1;
	int real_comp_path_idx = -1;
	int i, j;

	for_each_comp_in_target_ddp_mode(comp, mtk_crtc, i, j, ddp_mode) {
		if (comp->id == comp_id) {
			if (i == real_comp_path_idx)
				return real_comp_id;
			else
				return -1;
		}

		if (mtk_ddp_comp_get_type(comp->id) != MTK_DISP_VIRTUAL) {
			real_comp_id = comp->id;
			real_comp_path_idx = i;
		}
	}

	if (!mtk_crtc->is_dual_pipe)
		return -1;

	i = j = 0;
	real_comp_id = -1;
	real_comp_path_idx = -1;
	for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
		if (comp->id == comp_id) {
			if (i == real_comp_path_idx)
				return real_comp_id;
			else
				return -1;
		}

		if (mtk_ddp_comp_get_type(comp->id) != MTK_DISP_VIRTUAL) {
			real_comp_id = comp->id;
			real_comp_path_idx = i;
		}
	}

	return -1;
}

int mtk_crtc_mipi_freq_switch(struct drm_crtc *crtc, unsigned int en,
			unsigned int userdata)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct mtk_panel_ext *ext = mtk_crtc->panel_ext;

	if (mtk_crtc->mipi_hopping_sta == en)
		return 0;

	if (!(ext && ext->params &&
			ext->params->dyn.switch_en == 1))
		return 0;

	DDPMSG("%s, userdata=%d, en=%d\n", __func__, userdata, en);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_crtc->mipi_hopping_sta = en;


	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		DDPPR_ERR("request output fail\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	mtk_ddp_comp_io_cmd(comp,
			NULL, MIPI_HOPPING, &en);

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

char *mtk_crtc_index_spy(int crtc_index)
{
	switch (crtc_index) {
	case 0:
		return "P";
	case 1:
		return "E";
	case 2:
		return "M";
	case 3:
		return "N";
	default:
		return "Unknown";
	}
}

int mtk_crtc_osc_freq_switch(struct drm_crtc *crtc, unsigned int en,
			unsigned int userdata)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct mtk_panel_ext *ext = mtk_crtc->panel_ext;

	if (mtk_crtc->panel_osc_hopping_sta == en)
		return 0;

	if (!(ext && ext->params))
		return 0;

	DDPMSG("%s, userdata=%d, en=%d\n", __func__, userdata, en);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_crtc->panel_osc_hopping_sta = en;

	if (!mtk_crtc->enabled)
		goto done;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (!comp) {
		DDPPR_ERR("request output fail\n");
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		return -EINVAL;
	}

	/* Following section is for customization */
	/* Start */
	/* e.g. lmtk_ddp_comp_io_cmd(comp,
	 *	NULL, PANEL_OSC_HOPPING, &en);
	 */
	/* End */

done:
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

int mtk_crtc_enter_tui(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *cmdq_handle, *cmdq_handle2;
	unsigned int hrt_idx;
	int i;

	DDPMSG("%s\n", __func__);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_crtc->crtc_blank = true;
	mtk_disp_esd_check_switch(crtc, 0);

	mtk_drm_set_idlemgr(crtc, 0, 0);

	if (mtk_drm_helper_get_opt(priv->helper_opt, MTK_DRM_OPT_SPHRT))
		hrt_idx = _layering_rule_get_hrt_idx(drm_crtc_index(crtc));
	else
		hrt_idx = _layering_rule_get_hrt_idx(0);
	hrt_idx++;
	atomic_set(&priv->rollback_all, 1);
	drm_trigger_repaint(DRM_REPAINT_FOR_IDLE, crtc->dev);
	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	/* TODO: Potential risk, display suspend after release lock */
	for (i = 0; i < 60; ++i) {
		usleep_range(16667, 17000);
		if (atomic_read(&mtk_crtc->qos_ctx->last_hrt_idx) >=
			hrt_idx)
			break;
	}
	if (i >= 60)
		DDPPR_ERR("wait repaint %d\n", i);

	DDP_MUTEX_LOCK(&mtk_crtc->blank_lock, __func__, __LINE__);

	/* TODO: HardCode select OVL0, maybe store in platform data */
	switch (priv->data->mmsys_id) {
	case MMSYS_MT6985:
		priv->ddp_comp[DDP_COMPONENT_OVL1_2L]->blank_mode = true;

		if (mtk_crtc->is_dual_pipe)
			priv->ddp_comp[DDP_COMPONENT_OVL5_2L]->blank_mode = true;
		break;
	default:
		priv->ddp_comp[DDP_COMPONENT_OVL0]->blank_mode = true;

		if (mtk_crtc->is_dual_pipe)
			priv->ddp_comp[DDP_COMPONENT_OVL1]->blank_mode = true;
		break;
	}

	if (mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_drm_idlemgr_kick(__func__, crtc, 0);

		mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
					mtk_crtc->gce_obj.client[CLIENT_CFG]);
		/* 1. wait frame done & wait DSI not busy */
		cmdq_pkt_wait_no_clear(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		/* Clear stream block to prevent trigger loop start */
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);

		if (mtk_crtc_with_trigger_loop(crtc)) {
			mtk_crtc_stop_trig_loop(crtc);
			mtk_crtc_start_trig_loop(crtc);
		}

		mtk_crtc_pkt_create(&cmdq_handle2, &mtk_crtc->base,
					mtk_crtc->gce_obj.client[CLIENT_CFG]);

		cmdq_pkt_set_event(cmdq_handle2,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);

		cmdq_pkt_flush(cmdq_handle2);
		cmdq_pkt_destroy(cmdq_handle2);
	}

	DDP_MUTEX_UNLOCK(&mtk_crtc->blank_lock, __func__, __LINE__);

	wake_up(&mtk_crtc->state_wait_queue);

	return 0;
}

int mtk_crtc_exit_tui(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct cmdq_pkt *cmdq_handle, *cmdq_handle2;

	DDPMSG("%s\n", __func__);

	DDP_MUTEX_LOCK(&mtk_crtc->blank_lock, __func__, __LINE__);

	/* TODO: Hard Code select OVL0, maybe store in platform data */
	switch (priv->data->mmsys_id) {
	case MMSYS_MT6985:
		priv->ddp_comp[DDP_COMPONENT_OVL1_2L]->blank_mode = false;

		if (mtk_crtc->is_dual_pipe)
			priv->ddp_comp[DDP_COMPONENT_OVL5_2L]->blank_mode = false;
		break;
	default:
		priv->ddp_comp[DDP_COMPONENT_OVL0]->blank_mode = false;

		if (mtk_crtc->is_dual_pipe)
			priv->ddp_comp[DDP_COMPONENT_OVL1]->blank_mode = false;
		break;
	}

	if (mtk_crtc_is_frame_trigger_mode(crtc)) {

		mtk_drm_idlemgr_kick(__func__, crtc, 0);
		mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
					mtk_crtc->gce_obj.client[CLIENT_CFG]);
		/* 1. wait frame done & wait DSI not busy */
		cmdq_pkt_wait_no_clear(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);
		/* Clear stream block to prevent trigger loop start */
		cmdq_pkt_clear_event(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_CABC_EOF]);
		cmdq_pkt_wfe(cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);

		if (mtk_crtc_with_trigger_loop(crtc)) {
			mtk_crtc_stop_trig_loop(crtc);
			mtk_crtc_start_trig_loop(crtc);
		}

		mtk_crtc_pkt_create(&cmdq_handle2, &mtk_crtc->base,
					mtk_crtc->gce_obj.client[CLIENT_CFG]);
		cmdq_pkt_set_event(cmdq_handle2,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
		cmdq_pkt_flush(cmdq_handle2);
		cmdq_pkt_destroy(cmdq_handle2);
	}

	mtk_crtc->crtc_blank = false;

	atomic_set(&priv->rollback_all, 0);

	DDP_MUTEX_UNLOCK(&mtk_crtc->blank_lock, __func__, __LINE__);

	wake_up(&mtk_crtc->state_wait_queue);

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);

	mtk_drm_set_idlemgr(crtc, 1, 0);

	mtk_disp_esd_check_switch(crtc, 1);

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	return 0;
}

/********************** Legacy DISP API ****************************/
unsigned int DISP_GetScreenWidth(void)
{
	if (!pgc) {
		DDPPR_ERR("LCM is not registered.\n");
		return 0;
	}

	return pgc->mode.hdisplay;
}

unsigned int DISP_GetScreenHeight(void)
{
	if (!pgc) {
		DDPPR_ERR("LCM is not registered.\n");
		return 0;
	}

	return pgc->mode.vdisplay;
}

int mtk_crtc_lcm_ATA(struct drm_crtc *crtc)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp;
	struct cmdq_pkt *cmdq_handle;
	struct mtk_panel_params *panel_ext =
		mtk_drm_get_lcm_ext_params(crtc);
	int ret = 0;

	DDPINFO("%s\n", __func__);

	mtk_disp_esd_check_switch(crtc, 0);
	output_comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!output_comp)) {
		DDPPR_ERR("%s:invalid output comp\n", __func__);
		ret = -EINVAL;
		goto out;
	}
	if (unlikely(!panel_ext)) {
		DDPPR_ERR("%s:invalid panel_ext\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	DDPINFO("[ATA_LCM]primary display path stop[begin]\n");
	if (!mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_crtc_pkt_create(&cmdq_handle, crtc,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n",
			__func__, __LINE__);
			return -EINVAL;
		}

		mtk_ddp_comp_io_cmd(output_comp,
			cmdq_handle, DSI_STOP_VDO_MODE, NULL);
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}
	DDPINFO("[ATA_LCM]primary display path stop[end]\n");

	mtk_ddp_comp_io_cmd(output_comp, NULL, LCM_ATA_CHECK, &ret);

	if (!mtk_crtc_is_frame_trigger_mode(crtc)) {
		mtk_crtc_pkt_create(&cmdq_handle, crtc,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return -EINVAL;
		}

		mtk_ddp_comp_io_cmd(output_comp,
			cmdq_handle, DSI_START_VDO_MODE, NULL);
		mtk_disp_mutex_trigger(mtk_crtc->mutex[0], cmdq_handle);
		mtk_ddp_comp_io_cmd(output_comp, cmdq_handle, COMP_REG_START,
				    NULL);
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}
out:
	mtk_disp_esd_check_switch(crtc, 1);

	return ret;
}
unsigned int mtk_drm_primary_display_get_debug_state(
	struct mtk_drm_private *priv, char *stringbuf, int buf_len)
{
	int len = 0;

	struct drm_crtc *crtc = priv->crtc[0];
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *mtk_state;
	struct mtk_ddp_comp *comp;
	char *panel_name;

	comp = mtk_ddp_comp_request_output(mtk_crtc);
	if (unlikely(!comp))
		DDPPR_ERR("%s:invalid output comp\n", __func__);

	mtk_ddp_comp_io_cmd(comp, NULL, GET_PANEL_NAME,
				    &panel_name);

	len += scnprintf(stringbuf + len, buf_len - len,
			 "==========    Primary Display Info    ==========\n");

	DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	mtk_state = to_mtk_crtc_state(crtc->state);

	len += scnprintf(stringbuf + len, buf_len - len,
			 "LCM Driver=[%s] Resolution=%ux%u, Connected:%s\n",
			  panel_name, crtc->state->adjusted_mode.hdisplay,
			  crtc->state->adjusted_mode.vdisplay,
			  (mtk_drm_lcm_is_connect(mtk_crtc) ? "Y" : "N"));

	len += scnprintf(stringbuf + len, buf_len - len,
			 "FPS = %d, display mode idx = %u, %s mode %d\n",
			 drm_mode_vrefresh(&crtc->state->adjusted_mode),
			 mtk_state->prop_val[CRTC_PROP_DISP_MODE_IDX],
			 (mtk_crtc_is_frame_trigger_mode(crtc) ?
			  "cmd" : "vdo"), hrt_lp_switch_get());

	DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);

	len += scnprintf(stringbuf + len, buf_len - len,
		"================================================\n\n");

	return len;
}

int MMPathTraceCrtcPlanes(struct drm_crtc *crtc,
	char *str, int strlen, int n)
{
	unsigned int i = 0, addr;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct mtk_plane_state *state =
		    to_mtk_plane_state(mtk_crtc->planes[i].base.state);
		struct mtk_plane_pending_state *pending = &state->pending;

		if (pending->enable == 0)
			continue;

		addr = (unsigned int)pending->addr;

		n += scnprintf(str + n, strlen - n, "in_%d=0x%x, ",
			i, addr);
		n += scnprintf(str + n, strlen - n, "in_%d_width=%u, ",
			i, pending->width);
		n += scnprintf(str + n, strlen - n, "in_%d_height=%u, ",
			i, pending->height);
		n += scnprintf(str + n, strlen - n, "in_%d_fmt=%s, ",
			i, mtk_get_format_name(pending->format));
		n += scnprintf(str + n, strlen - n, "in_%d_bpp=%u, ",
			i, mtk_get_format_bpp(pending->format));
		n += scnprintf(str + n, strlen - n, "in_%d_compr=%u, ",
			i, (unsigned int)pending->prop_val[PLANE_PROP_COMPRESS]);
	}

	return n;
}

/* ************   Panel Master   **************** */

void mtk_crtc_start_for_pm(struct drm_crtc *crtc)
{
	int i, j;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	struct cmdq_pkt *cmdq_handle;
	/* start trig loop */
	if (mtk_crtc_with_trigger_loop(crtc))
		mtk_crtc_start_trig_loop(crtc);
	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	/*if VDO mode start DSI MODE */
	if (!mtk_crtc_is_frame_trigger_mode(crtc) &&
		mtk_crtc_is_connector_enable(mtk_crtc)) {
		for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
			mtk_ddp_comp_io_cmd(comp, cmdq_handle,
				DSI_START_VDO_MODE, NULL);
		}
	}
	for_each_comp_in_cur_crtc_path(comp, mtk_crtc, i, j) {
		mtk_ddp_comp_start(comp, cmdq_handle);
	}

	if (mtk_crtc->is_dual_pipe) {
		for_each_comp_in_dual_pipe(comp, mtk_crtc, i, j) {
			mtk_ddp_comp_start(comp, cmdq_handle);
		}
	}

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);
}

void mtk_crtc_stop_for_pm(struct mtk_drm_crtc *mtk_crtc, bool need_wait)
{
	struct cmdq_pkt *cmdq_handle;

	unsigned int crtc_id = drm_crtc_index(&mtk_crtc->base);
	struct drm_crtc *crtc = &mtk_crtc->base;

	DDPINFO("%s:%d +\n", __func__, __LINE__);

	/* 0. Waiting CLIENT_DSI_CFG thread done */
	if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] != NULL) {
		mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);

		if (!cmdq_handle) {
			DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
			return;
		}

		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}

	mtk_crtc_pkt_create(&cmdq_handle, &mtk_crtc->base,
		mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (!need_wait)
		goto skip;

	if (crtc_id == 2) {
		cmdq_pkt_wait_no_clear(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_WDMA0_EOF]);
	} else if (mtk_crtc_is_frame_trigger_mode(&mtk_crtc->base)) {
		/* 1. wait stream eof & clear tocken */
		/* clear eof token to prevent any config after this command */
		cmdq_pkt_wfe(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_STREAM_EOF]);

		/* clear dirty token to prevent trigger loop start */
		cmdq_pkt_clear_event(
			cmdq_handle,
			mtk_crtc->gce_obj.event[EVENT_STREAM_BLOCK]);
	} else if (mtk_crtc_is_connector_enable(mtk_crtc)) {
		/* In vdo mode, DSI would be stop when disable connector
		 * Do not wait frame done in this case.
		 */
		cmdq_pkt_wfe(cmdq_handle,
				 mtk_crtc->gce_obj.event[EVENT_VDO_EOF]);
	}

skip:
	/* 2. stop all modules in this CRTC */
	mtk_crtc_stop_ddp(mtk_crtc, cmdq_handle);

	cmdq_pkt_flush(cmdq_handle);
	cmdq_pkt_destroy(cmdq_handle);

	/* 3. stop trig loop  */
	if (mtk_crtc_with_trigger_loop(crtc)) {
		mtk_crtc_stop_trig_loop(crtc);
		if (mtk_crtc_with_sodi_loop(crtc) &&
				(!mtk_crtc_is_frame_trigger_mode(crtc)))
			mtk_crtc_stop_sodi_loop(crtc);
	}

	if (mtk_crtc_with_event_loop(crtc) &&
			(mtk_crtc_is_frame_trigger_mode(crtc)))
		mtk_crtc_stop_event_loop(crtc);

	DDPINFO("%s:%d -\n", __func__, __LINE__);
}
/* ***********  Panel Master end ************** */

bool mtk_drm_get_hdr_property(void)
{
	return hdr_en;
}

/********************** Legacy DRM API *****************************/
int mtk_drm_format_plane_cpp(uint32_t format, unsigned int plane)
{
	const struct drm_format_info *info;

	info = drm_format_info(format);
	if (!info || plane >= info->num_planes)
		return 0;

	if (info->cpp[plane] == 0)
		return info->char_per_block[plane];

	return info->cpp[plane];
}

int mtk_drm_switch_te(struct drm_crtc *crtc, int te_num, bool need_lock)
{
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *private = crtc->dev->dev_private;
	struct dual_te *d_te = &mtk_crtc->d_te;
	struct cmdq_pkt *handle;
	static bool set_outpin;
	dma_addr_t addr;

	if (!mtk_drm_helper_get_opt(private->helper_opt,
				MTK_DRM_OPT_DUAL_TE) || !d_te->en)
		return -EINVAL;

	if (need_lock) {
		mutex_lock(&private->commit.lock);
		DDP_MUTEX_LOCK(&mtk_crtc->lock, __func__, __LINE__);
	}

	if (mtk_crtc->gce_obj.client[CLIENT_DSI_CFG] != NULL)
		mtk_crtc_pkt_create(&handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]);
	else
		mtk_crtc_pkt_create(&handle, &mtk_crtc->base,
			mtk_crtc->gce_obj.client[CLIENT_CFG]);

	if (!handle) {
		DDPPR_ERR("%s:%d NULL handle\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (!set_outpin && mtk_crtc->gce_obj.client[CLIENT_DSI_CFG]) {
		cmdq_set_outpin_event(mtk_crtc->gce_obj.client[CLIENT_DSI_CFG],
				true);
		set_outpin = true;
	} else if (!set_outpin) {
		cmdq_set_outpin_event(mtk_crtc->gce_obj.client[CLIENT_CFG],
				true);
		set_outpin = true;
	}
	addr = mtk_get_gce_backup_slot_pa(mtk_crtc, DISP_SLOT_TE1_EN);
	if (te_num == 1) {
		DDPMSG("switched to te1!\n");
		atomic_set(&d_te->te_switched, 1);
		enable_irq(d_te->te1);
		cmdq_pkt_write(handle,
			mtk_crtc->gce_obj.base, addr, 1, ~0);
	} else {
		DDPMSG("switched to te0!\n");
		atomic_set(&d_te->te_switched, 0);
		disable_irq(d_te->te1);
		cmdq_pkt_write(handle,
			mtk_crtc->gce_obj.base, addr, 0, ~0);
	}

	cmdq_pkt_flush(handle);
	cmdq_pkt_destroy(handle);

	if (need_lock) {
		DDP_MUTEX_UNLOCK(&mtk_crtc->lock, __func__, __LINE__);
		mutex_unlock(&private->commit.lock);
	}
	return 0;
}

void mtk_crtc_mml_racing_resubmit(struct drm_crtc *crtc, struct cmdq_pkt *_cmdq_handle)
{
	u8 i = 0;
	bool flush = false;
	struct cmdq_pkt *cmdq_handle = NULL;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mml_drm_ctx *mml_ctx = mtk_drm_get_mml_drm_ctx(crtc->dev, crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct mtk_ddp_comp *comp = NULL;
	const enum mtk_ddp_comp_id id[] = {DDP_COMPONENT_INLINE_ROTATE0,
					   DDP_COMPONENT_INLINE_ROTATE1};

	if (!mml_ctx || !mtk_crtc->is_mml) {
		DDPMSG("%s !mml_ctx or !is_mml\n", __func__);
		return;
	}

	mml_drm_submit(mml_ctx, mtk_crtc->mml_cfg, &(mtk_crtc->mml_cb));

	if (_cmdq_handle) {
		cmdq_handle = _cmdq_handle;
	} else {
		mtk_crtc_pkt_create(&cmdq_handle, crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);
		flush = true;
	}

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	for (; i <= mtk_crtc->is_dual_pipe; ++i) {
		comp = priv->ddp_comp[id[i]];
		mtk_ddp_comp_reset(comp, cmdq_handle);
		mtk_ddp_comp_addon_config(comp, 0, 0, NULL, cmdq_handle);
		mtk_disp_mutex_add_comp_with_cmdq(mtk_crtc, id[i], false, cmdq_handle, 0);
	}

	/* prepare racing packet */
	mml_drm_racing_config_sync(mml_ctx, cmdq_handle);

	/* blocking wait until mml submit is flushed */
	mtk_drm_wait_mml_submit_done(&(mtk_crtc->mml_cb));

	if (flush) {
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}
}

void mtk_crtc_mml_racing_stop_sync(struct drm_crtc *crtc, struct cmdq_pkt *_cmdq_handle,
				   bool force)
{
	bool flush = false;
	struct cmdq_pkt *cmdq_handle = NULL;
	struct mtk_drm_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mml_drm_ctx *mml_ctx = mtk_drm_get_mml_drm_ctx(crtc->dev, crtc);

	if (!mml_ctx) {
		DDPMSG("%s !mml_ctx\n", __func__);
		return;
	}

	if (_cmdq_handle) {
		cmdq_handle = _cmdq_handle;
	} else {
		mtk_crtc_pkt_create(&cmdq_handle, crtc, mtk_crtc->gce_obj.client[CLIENT_CFG]);
		flush = true;
	}

	if (!cmdq_handle) {
		DDPPR_ERR("%s:%d NULL cmdq handle\n", __func__, __LINE__);
		return;
	}

	if (force && mtk_crtc->mml_cfg)
		mml_drm_stop(mml_ctx, mtk_crtc->mml_cfg, false);
	else
		mml_drm_racing_stop_sync(mml_ctx, cmdq_handle);

	if (flush) {
		cmdq_pkt_flush(cmdq_handle);
		cmdq_pkt_destroy(cmdq_handle);
	}
}

void mtk_crtc_store_total_overhead(struct mtk_drm_crtc *mtk_crtc,
	struct total_tile_overhead info)
{
	mtk_crtc->tile_overhead = info;
}

struct total_tile_overhead mtk_crtc_get_total_overhead(struct mtk_drm_crtc *mtk_crtc)
{
	return mtk_crtc->tile_overhead;
}
