/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MTK_MML_RSZ_FW_H__
#define __MTK_MML_RSZ_FW_H__

#include <linux/types.h>
#include "mtk-mml-core.h"

struct rsz_fw_in {
	u32 in_width;
	u32 in_height;
	u32 out_width;
	u32 out_height;
	struct mml_crop crop;
	bool power_saving;
	bool use121filter;
};

struct rsz_fw_out {
	u32 hori_step;
	u32 vert_step;
	u32 precision_x;
	u32 precision_y;
	u32 hori_int_ofst;
	u32 hori_sub_ofst;
	u32 vert_int_ofst;
	u32 vert_sub_ofst;
	u32 hori_scale;
	u32 hori_algo;
	u32 vert_scale;
	u32 vert_algo;
	u32 vert_first;
	u32 vert_cubic_trunc;
	u32 con1;
	u32 con2;
	u32 tap_adapt;
	u32 etc_ctrl;
	u32 etc_switch_max_min1;
	u32 etc_switch_max_min2;
	u32 etc_ring;
	u32 etc_ring_gaincon1;
	u32 etc_ring_gaincon2;
	u32 etc_ring_gaincon3;
	u32 etc_sim_port_gaincon1;
	u32 etc_sim_port_gaincon2;
	u32 etc_sim_port_gaincon3;
	u32 etc_blend;
};

struct rsz_cal_param {
	u32 yuv_422_t_yuv_444;
	s32 hori_luma_int_ofst;
	s32 hori_luma_sub_ofst;
	s32 vert_luma_int_ofst;
	s32 vert_luma_sub_ofst;
	bool int_wclr_en;
	bool tap_adapt_en;
	s32 tap_adapt_slope;
	u32 tap_adapt_fallback_ratio;
	u32 tap_adapt_var_coring;
	u32 tap_adapt_dc_coring;
	u32 tap_adapt_edge_thr;
	u32 signal_enhance_mode;
	u32 hori_tbl;
	u32 vert_tbl;
	bool hori_cubic_trunc_en;
	u32 hori_luma_cubic_trunc_bit;
	u32 hori_chroma_cubic_trunc_bit;
	u32 vert_luma_cubic_trunc_bit;
	u32 vert_chroma_cubic_trunc_bit;
	s32 hori_trunc_bit;
	s32 vert_trunc_bit;
};

/* rsz_fw - RSZ firmware calculate RSZ settings
 *
 * @in:		struct rsz_fw_in contains size information.
 * @out:	struct rsz_fw_out contains RSZ setting results.
 * @en_ur:	enable UR flag
 */
void rsz_fw(struct rsz_fw_in *in, struct rsz_fw_out *out, bool en_ur);

#endif	/* __MTK_MML_RSZ_FW_H__ */
