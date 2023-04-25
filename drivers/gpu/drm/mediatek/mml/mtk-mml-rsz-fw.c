// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include "mtk-mml-rsz-fw.h"
#include "DpTileScaler.h"

#define RSZ_ACCU_STEPCOUNTER_BIT 20
#define RSZ_6TAP_STEPCOUNTER_BIT 15
#define RSZ_PREC_SHIFT 20
#define RSZ_RATIO_SHIFT 10
#define RSZ_ALG_TH0 1
#define RSZ_ALG_TH1 24

static void rsz_init(struct rsz_fw_out *out,
	struct rsz_cal_param *cal_param)
{
	out->con1 = out->con2 = out->tap_adapt = 0;
	cal_param->int_wclr_en = 1;
	cal_param->tap_adapt_edge_thr = 2;
	cal_param->tap_adapt_dc_coring = 2;
	cal_param->tap_adapt_var_coring = 2;
	cal_param->tap_adapt_fallback_ratio = 0;
	cal_param->tap_adapt_slope = 8;
	cal_param->signal_enhance_mode = 1;
	out->etc_ctrl              = 0x34220000;
	out->etc_switch_max_min1   = 0x23012ac0;
	out->etc_switch_max_min2   = 0x1e232800;
	out->etc_ring              = 0x05260c17;
	out->etc_ring_gaincon1     = 0x1400600d;
	out->etc_ring_gaincon2     = 0x141b3a00;
	out->etc_ring_gaincon3     = 0x0e01000a;
	out->etc_sim_port_gaincon1 = 0x05040f16;
	out->etc_sim_port_gaincon2 = 0x021d0500;
	out->etc_sim_port_gaincon3 = 0x04004000;
	out->etc_blend             = 0x28000000;
}

static void rsz_config_ctrl_regs(struct rsz_fw_in *in,
	struct rsz_fw_out *out, struct rsz_cal_param *cal_param)
{
	if (in->in_width == in->out_width &&
	    in->crop.r.width == in->out_width &&
	    !cal_param->yuv_422_t_yuv_444)
		out->hori_scale = 0;
	else
		out->hori_scale = 1;

	if (in->in_height == in->out_height &&
	    in->crop.r.height == in->out_height)
		out->vert_scale = 0;
	else
		out->vert_scale = 1;

	cal_param->tap_adapt_en = 1;
}

static void rsz_config(struct rsz_fw_in *in, struct rsz_fw_out *out,
	bool is_hor, struct rsz_cal_param *cal_param)
{
	s32 prec = 0, max_nm = 0, max_nm_prec = 0;
	s32 shift = 0, coeff_index_approx_ini = 0;
	s32 n_m1, m_m1;
	s64 m_m1_zoom, n_m1_zoom;
	s64 offset; /* the offset defined by firmware */
	s64 ratio;
	s32 coeff_rs = 0;
	s32 coeff_step = 0;

	u32 dst_width = in->out_width;
	u32 dst_height = in->out_height;
	u32 crop_offset_x = in->crop.r.left;
	u32 crop_subpix_x = in->crop.x_sub_px;
	u32 crop_offset_y = in->crop.r.top;
	u32 crop_subpix_y = in->crop.y_sub_px;
	u32 crop_width = in->crop.r.width;
	u32 crop_subpix_w = in->crop.w_sub_px;
	u32 crop_height = in->crop.r.height;
	u32 crop_subpix_h = in->crop.h_sub_px;
	s32 alg;

	if (is_hor)
		alg = out->hori_algo;
	else
		alg = out->vert_algo;

	/* Load the parameters needed for resizer configuration */
	if (is_hor) { /* horizontal scaling */
		n_m1 = dst_width;
		m_m1 = crop_width;
		ratio = 1;

		/* 1048576x */
		n_m1_zoom = (n_m1 * ratio) << RSZ_PREC_SHIFT;
		m_m1_zoom = ((m_m1 * ratio) << RSZ_PREC_SHIFT) + crop_subpix_w;
		offset = ((s64)crop_offset_x << RSZ_PREC_SHIFT) + crop_subpix_x;
	} else { /* vertical scaling */
		n_m1 = dst_height;
		m_m1 = crop_height;
		ratio = 1;

		/* 1048576x */
		n_m1_zoom = (n_m1 * ratio) << RSZ_PREC_SHIFT;
		m_m1_zoom = ((m_m1 * ratio) << RSZ_PREC_SHIFT) + crop_subpix_h;
		offset = ((s64)crop_offset_y << RSZ_PREC_SHIFT) + crop_subpix_y;
	}

	mml_msg("%s m_m1_zoom[%lld]  n_m1_zoom[%lld] prec[%d] offset[%lld]",
		__func__, m_m1_zoom, n_m1_zoom, prec, offset);
	m_m1_zoom -= (1 << RSZ_PREC_SHIFT);
	n_m1_zoom -= (1 << RSZ_PREC_SHIFT);

	/* Resizer parameter configuration */
	if (alg == 0) { /* 6-tap FIR */
		prec = 1 << RSZ_6TAP_STEPCOUNTER_BIT;
		max_nm = 1;
		max_nm_prec = max_nm * prec;

		coeff_step = mult_frac(max_nm_prec, m_m1_zoom, n_m1_zoom);

		shift = offset >> RSZ_PREC_SHIFT;
		coeff_index_approx_ini = (offset * prec -
			((u64)shift << RSZ_PREC_SHIFT) *
			prec) >> RSZ_PREC_SHIFT;
	} else if (alg == 1 || alg == 2) {
		prec = 1 << (RSZ_ACCU_STEPCOUNTER_BIT - 6);
		max_nm = 64;
		max_nm_prec = max_nm * prec;

		/* determination of coeff_step */
		if (n_m1_zoom == m_m1_zoom) {
			coeff_step = max_nm_prec;
		} else {
			coeff_step = mult_frac(max_nm_prec, n_m1_zoom, m_m1_zoom);
			if (max_nm_prec != mult_frac(coeff_step, m_m1_zoom, n_m1_zoom))
				coeff_step += 1; /* ceiling */
		}

		/* determination of the initial position of the step counter */
		if (offset == 0) {
			shift = 0; /* integer offset defined in the output domain */
			coeff_index_approx_ini = 0;
		} else {
			shift = ((coeff_step * offset) / max_nm_prec) >>
				RSZ_PREC_SHIFT;
			coeff_index_approx_ini = (coeff_step * offset -
				(((u64)shift * max_nm_prec) <<
				RSZ_PREC_SHIFT)) >> RSZ_PREC_SHIFT;
		}
	}
	/*
	 * Special needs for hardware design:
	 * left shift the value of the coefficients
	 * as there is not enough bit-depth to store
	 * the result of accumulation when output/input < 64
	 */

	if (alg == 2) {
		if (m_m1 <= 64 * n_m1)
			coeff_rs = 0;
		else if (m_m1 <= 128 * n_m1)
			coeff_rs = 1;
		else if (m_m1 <= 256 * n_m1)
			coeff_rs = 2;
		else if (m_m1 <= 512 * n_m1)
			coeff_rs = 3;
		else if (m_m1 <= 1024 * n_m1)
			coeff_rs = 4;
		else if (m_m1 <= 2048 * n_m1)
			coeff_rs = 5;
		else
			coeff_rs = 6;
	} else if (alg == 1) {
		coeff_rs = 0;
	}

	/* Save the coefficients to the parameters */
	if (is_hor) { /* for horizontal */
		out->hori_step = coeff_step;
		cal_param->hori_luma_int_ofst = shift;
		cal_param->hori_luma_sub_ofst = coeff_index_approx_ini;
		cal_param->hori_trunc_bit = coeff_rs;
		out->precision_x = max_nm_prec;
	} else { /* for vertical */
		out->vert_step = coeff_step;
		cal_param->vert_trunc_bit = coeff_rs;
		cal_param->vert_luma_int_ofst = shift;
		cal_param->vert_luma_sub_ofst = coeff_index_approx_ini;
		out->precision_y = max_nm_prec;

		if (crop_width < dst_width)
			out->vert_first = 1;
		else
			out->vert_first = 0;
	}
}

static u32 rsz_tbl_sel(u32 alg, u32 step)
{
	u32 table = 0;

	if (alg == 0) {
		if (step == 32768)
			table = 7;
		else
			table = 9;
	} else if (alg == 1 || alg == 2) {
		table = 17;
	}

	return table;
}

static void rsz_auto_align(struct rsz_fw_in *in, struct rsz_fw_out *out,
	bool is_hor, struct rsz_cal_param *cal_param)
{
	s32 prec, max_nm, max_nm_prec, offset = 0;
	u32 dst_width = in->out_width;
	u32 dst_height = in->out_height;
	u32 crop_width = in->crop.r.width;
	u32 crop_subpix_w = in->crop.w_sub_px;
	u32 crop_height = in->crop.r.height;
	u32 crop_subpix_h = in->crop.h_sub_px;
	s32 alg;

	if (is_hor)
		alg = out->hori_algo;
	else
		alg = out->vert_algo;

	if (alg == 0) { /* 6Tap FIR */
		prec = 1 << RSZ_6TAP_STEPCOUNTER_BIT;
		max_nm = 1;
	} else { /* 6nTap FIR or Source accumulation */
		prec = 1 << (RSZ_ACCU_STEPCOUNTER_BIT - 6); /* 6 for "max_nm=64=2^6" */
		max_nm = 64;
	}
	max_nm_prec = max_nm * prec;

	if (is_hor) { /* for horizontal */
		if (alg == 0) {
			/*
			 * 6-tap FIR: prec=32768; max_nm=1; coeff_step =
			 * (int)((((M_m1_zoom*max_nm)/N_m1_zoom)*prec) + 0.5);
			 */
			cal_param->hori_luma_sub_ofst +=
				(prec * (crop_width - 1) +
				(crop_subpix_w >> 5) -
				(out->hori_step * (dst_width - 1))) / 2;

			/* hardware requirement: always positive subpixel offset */
			if (cal_param->hori_luma_sub_ofst < 0) {
				cal_param->hori_luma_int_ofst--;
				cal_param->hori_luma_sub_ofst = prec +
					cal_param->hori_luma_sub_ofst;
			}
			if (cal_param->hori_luma_sub_ofst >= prec) {
				cal_param->hori_luma_int_ofst++;
				cal_param->hori_luma_sub_ofst =
					cal_param->hori_luma_sub_ofst - prec;
			}
		}
		if (alg == 2) {
			/*
			 * 6n-tap FIR: prec=16384; max_nm=64; coeff_step =
			 * (int)(((N_m1_zoom*max_nm)/M_m1_zoom)*prec) + 1;
			 */
			if (crop_width == dst_width) { /* 1x */
				cal_param->hori_luma_int_ofst += 0;
				cal_param->hori_luma_sub_ofst += 0;
			} else { /* <1x */
				cal_param->hori_luma_int_ofst += 0;

				offset = (((crop_width - 1) * out->hori_step) +
					((crop_subpix_w * out->hori_step) >>
					RSZ_PREC_SHIFT) - ((dst_width - 1) *
					max_nm_prec)) / 2;
				cal_param->hori_luma_sub_ofst += offset;
			}

			/* hardware requirement: always positive subpixel offset */
			if (cal_param->hori_luma_sub_ofst < 0) {
				cal_param->hori_luma_int_ofst--;
				cal_param->hori_luma_sub_ofst =
					max_nm_prec +
					cal_param->hori_luma_sub_ofst;
			}
			if (cal_param->hori_luma_sub_ofst >= max_nm_prec) {
				cal_param->hori_luma_int_ofst++;
				cal_param->hori_luma_sub_ofst =
					cal_param->hori_luma_sub_ofst -
					max_nm_prec;
			}
		}
	} else { /* for vertical */
		if (alg == 0) {
			/*
			 * 6-tap FIR: prec=32768; max_nm=1; coeff_step =
			 * (int)((((M_m1_zoom*max_nm)/N_m1_zoom)*prec) + 0.5);
			 */
			cal_param->vert_luma_sub_ofst +=
				(prec * (crop_height - 1) +
				(crop_subpix_h >> 5) -
				(out->vert_step * (dst_height - 1))) / 2;

			/* hardware requirement: always positive subpixel offset */
			if (cal_param->vert_luma_sub_ofst < 0) {
				cal_param->vert_luma_int_ofst--;
				cal_param->vert_luma_sub_ofst = prec +
					cal_param->vert_luma_sub_ofst;
			}
			if (cal_param->vert_luma_sub_ofst >= prec) {
				cal_param->vert_luma_int_ofst++;
				cal_param->vert_luma_sub_ofst =
					cal_param->vert_luma_sub_ofst - prec;
			}
		}
		/* Auto subpixel shift for fun_255.c */
		if (alg == 2) {
			/*
			 * 6n-tap FIR: prec=16384; max_nm=64; coeff_step =
			 * (int)(((N_m1_zoom*max_nm)/M_m1_zoom)*prec) + 1;
			 */
			if (crop_height == dst_height) { /* 1x */
				cal_param->vert_luma_int_ofst += 0;
				cal_param->vert_luma_sub_ofst += 0;
			} else { /* <1x */
				cal_param->vert_luma_int_ofst += 0;

				offset = (((crop_height - 1) * out->vert_step) +
					((crop_subpix_h * out->vert_step) >>
					RSZ_PREC_SHIFT) - ((dst_height - 1) *
					max_nm_prec)) / 2;
				cal_param->vert_luma_sub_ofst += offset;
			}

			/* hardware requirement: always positive subpixel offset */
			if (cal_param->vert_luma_sub_ofst < 0) {
				cal_param->vert_luma_int_ofst--;
				cal_param->vert_luma_sub_ofst =
					max_nm_prec +
					cal_param->vert_luma_sub_ofst;
			}
			if (cal_param->vert_luma_sub_ofst >= max_nm_prec) {
				cal_param->vert_luma_int_ofst++;
				cal_param->vert_luma_sub_ofst =
					cal_param->vert_luma_sub_ofst -
					max_nm_prec;
			}
		}
	}
}

static void rsz_ofst_check(struct rsz_fw_out *out,
	struct rsz_cal_param *cal_param)
{
	s32 step_size_6tap = 1 << RSZ_6TAP_STEPCOUNTER_BIT;
	s32 step_size_acc = 1 << RSZ_ACCU_STEPCOUNTER_BIT;

	if (out->hori_algo == 0) {
		if (cal_param->hori_luma_sub_ofst >= step_size_6tap) {
			cal_param->hori_luma_int_ofst +=
				cal_param->hori_luma_sub_ofst / step_size_6tap;
			cal_param->hori_luma_sub_ofst =
				cal_param->hori_luma_sub_ofst % step_size_6tap;
		}
	} else {
		if (cal_param->hori_luma_sub_ofst >= step_size_acc) {
			cal_param->hori_luma_int_ofst +=
				cal_param->hori_luma_sub_ofst / step_size_acc;
			cal_param->hori_luma_sub_ofst =
				cal_param->hori_luma_sub_ofst % step_size_acc;
		}
	}

	if (out->vert_algo == 0) {
		if (cal_param->vert_luma_sub_ofst >= step_size_6tap) {
			cal_param->vert_luma_int_ofst +=
				cal_param->vert_luma_sub_ofst / step_size_6tap;
			cal_param->vert_luma_sub_ofst =
				cal_param->vert_luma_sub_ofst % step_size_6tap;
		}
	} else {
		if (cal_param->vert_luma_sub_ofst >= step_size_acc) {
			cal_param->vert_luma_int_ofst +=
				cal_param->vert_luma_sub_ofst / step_size_acc;
			cal_param->vert_luma_sub_ofst =
				cal_param->vert_luma_sub_ofst % step_size_acc;
		}
	}
}

static void rsz_auto_coef_trunc(struct rsz_fw_in *in, struct rsz_fw_out *out,
	struct rsz_cal_param *cal_param)
{
	u32 hori_ratio, vert_ratio;

	hori_ratio = ((in->out_width << RSZ_RATIO_SHIFT) - 1) / in->crop.r.width + 1;
	vert_ratio = ((in->out_height << RSZ_RATIO_SHIFT) - 1) / in->crop.r.height + 1;
	cal_param->hori_cubic_trunc_en = 0;
	cal_param->hori_luma_cubic_trunc_bit = 0;
	cal_param->hori_chroma_cubic_trunc_bit = 0;
	if (out->vert_algo != 2 || hori_ratio <= 512) {
		out->vert_cubic_trunc = 0;
		cal_param->vert_luma_cubic_trunc_bit = 0;
		cal_param->vert_chroma_cubic_trunc_bit = 0;
		return;
	}

	out->vert_cubic_trunc = 1;
	if (in->power_saving) {
		if (vert_ratio > 512 && vert_ratio <= 1024) {
			cal_param->vert_luma_cubic_trunc_bit = 2;
			cal_param->vert_chroma_cubic_trunc_bit = 0;
		} else if (vert_ratio > 256 && vert_ratio <= 512) {
			cal_param->vert_luma_cubic_trunc_bit = 3;
			cal_param->vert_chroma_cubic_trunc_bit = 1;
		} else if (vert_ratio > 128 && vert_ratio <= 256) {
			cal_param->vert_luma_cubic_trunc_bit = 4;
			cal_param->vert_chroma_cubic_trunc_bit = 2;
		} else if (vert_ratio > 64 && vert_ratio <= 128) {
			cal_param->vert_luma_cubic_trunc_bit = 5;
			cal_param->vert_chroma_cubic_trunc_bit = 3;
		} else if (vert_ratio >= 32 && vert_ratio <= 64) {
			cal_param->vert_luma_cubic_trunc_bit = 6;
			cal_param->vert_chroma_cubic_trunc_bit = 4;
		}
	} else {
		if (vert_ratio > 512 && vert_ratio <= 1024) {
			cal_param->vert_luma_cubic_trunc_bit = 4;
			cal_param->vert_chroma_cubic_trunc_bit = 2;
		} else if (vert_ratio > 256 && vert_ratio <= 512) {
			cal_param->vert_luma_cubic_trunc_bit = 5;
			cal_param->vert_chroma_cubic_trunc_bit = 3;
		} else if (vert_ratio > 128 && vert_ratio <= 256) {
			cal_param->vert_luma_cubic_trunc_bit = 6;
			cal_param->vert_chroma_cubic_trunc_bit = 4;
		} else if (vert_ratio > 64 && vert_ratio <= 128) {
			cal_param->vert_luma_cubic_trunc_bit = 7;
			cal_param->vert_chroma_cubic_trunc_bit = 5;
		} else if (vert_ratio >= 32 && vert_ratio <= 64) {
			out->vert_algo = 1;
		}
	}
}

static s32 alpha_blending(s32 data1, s32 data2, s32 alpha, s32 bits)
{
	return (alpha == 0) ? data2 :
		(alpha == (1 << bits) - 1) ? data1 :
		(data1 * alpha + data2 * ((1 << bits) - alpha)) >> bits;
}

static s32 rsz_ultra_res_reg(s32 in_value, s32 in_ratio,
	s32 reg_ratio_thr0, s32 reg_ratio_thr1,
	s32 reg_ratio_thr2, s32 reg_clip1, s32 reg_clip2,
	s32 reg_min, s32 reg_max)
{
	const s32 ALPHA_BITS = 8;
	s32 tar_value1 = clamp(in_value + reg_clip1, reg_min, reg_max);
	s32 tar_value2 = clamp(in_value + reg_clip1 + reg_clip2,
		reg_min, reg_max);
	s32 out_value = 0, alpha = 0;

	if (in_ratio <= reg_ratio_thr0) {
		out_value = in_value;
	} else if (in_ratio <= reg_ratio_thr1) {
		alpha = (in_ratio - reg_ratio_thr0) * (1 << ALPHA_BITS) /
			(reg_ratio_thr1 - reg_ratio_thr0);
		out_value = alpha_blending(tar_value1, in_value, alpha, ALPHA_BITS);
	} else if (in_ratio <= reg_ratio_thr2) {
		alpha = (in_ratio - reg_ratio_thr1) * (1 << ALPHA_BITS) /
			(reg_ratio_thr2 - reg_ratio_thr1);
		out_value = alpha_blending(tar_value2, tar_value1, alpha, ALPHA_BITS);
	} else {
		out_value = tar_value2;
	}

	return out_value;
}

static void rsz_ultra_res(struct rsz_fw_out *out,
	struct rsz_cal_param *cal_param)
{
	u32 ratio = 0;
	u32 max_step = max(out->hori_step, out->vert_step);
	u64 prec = 1 << RSZ_6TAP_STEPCOUNTER_BIT;

	if (max_step < 4096) /* scaling ratio > 8x */
		ratio = 1 << 13; /* ratio = 8196: 8x */
	else /* 1x < scaling ratio < 8x */
		ratio = ((prec << RSZ_PREC_SHIFT) / max_step) >>
			(RSZ_PREC_SHIFT - 10);

	cal_param->tap_adapt_slope = rsz_ultra_res_reg(
		cal_param->tap_adapt_slope,
		ratio,
		1024, /* 1x */
		1536, /* 1.5x */
		2048, /* 2x */
		0, -7, 0, 15);
}

void rsz_fw(struct rsz_fw_in *in, struct rsz_fw_out *out, bool en_ur)
{
	struct rsz_cal_param cal_param;

	rsz_init(out, &cal_param);

	if (in->out_width > 2) {
		if (in->crop.r.width <= RSZ_ALG_TH0 * in->out_width ||
		    (in->crop.r.width == in->out_width && in->crop.r.left == 0))
			out->hori_algo = 0;
		else if (in->crop.r.width > RSZ_ALG_TH1 * in->out_width)
			out->hori_algo = 1;
		else
			out->hori_algo = 2;
	} else { /* when the width of output image <= 2 */
		if (in->crop.r.width <= RSZ_ALG_TH0 * in->out_width ||
		    (in->crop.r.width == in->out_width && in->crop.r.left == 0))
			out->hori_algo = 0;
		else
			out->hori_algo = 1;
	}

	if (in->out_height > 2) {
		if (in->crop.r.height <= RSZ_ALG_TH0 * in->out_height ||
		    (in->crop.r.height == in->out_height && in->crop.r.top == 0))
			out->vert_algo = 0;
		else if (in->crop.r.height > RSZ_ALG_TH1 * in->out_height ||
		    (in->crop.r.height - 1) > 4096 * (in->out_height - 1))
			out->vert_algo = 1;
		else
			out->vert_algo = 2;
	} else { /* output width or height <= 2 */
		if (in->crop.r.height <= RSZ_ALG_TH0 * in->out_height ||
		    (in->crop.r.height == in->out_height && in->crop.r.top == 0))
			out->vert_algo = 0;
		else
			out->vert_algo = 1;
	}

	out->vert_cubic_trunc = 0;
	cal_param.yuv_422_t_yuv_444 = 0;

	rsz_config_ctrl_regs(in, out, &cal_param);
	rsz_config(in, out, true, &cal_param);
	rsz_config(in, out, false, &cal_param);

	if (out->hori_scale == 0 || out->vert_scale == 0 ||
	    out->vert_first == 0 ||
	    out->vert_algo != 0 || out->hori_algo != 0) {
		cal_param.tap_adapt_en = 0;
		cal_param.signal_enhance_mode = 0;
	}

	cal_param.hori_tbl = rsz_tbl_sel(out->hori_algo, out->hori_step);
	cal_param.vert_tbl = rsz_tbl_sel(out->vert_algo, out->vert_step);
	rsz_auto_align(in, out, true, &cal_param);
	rsz_auto_align(in, out, false, &cal_param);
	rsz_ofst_check(out, &cal_param);
	rsz_auto_coef_trunc(in, out, &cal_param);

	if (en_ur)
		rsz_ultra_res(out, &cal_param);
	else
		cal_param.tap_adapt_slope = 8;

	if (out->hori_algo == SCALER_6_TAPS) {
		out->hori_int_ofst = cal_param.hori_luma_int_ofst;
		out->hori_sub_ofst = ((s64)cal_param.hori_luma_sub_ofst <<
			TILE_SCALER_SUBPIXEL_SHIFT) / out->precision_x;
	} else { /* (1 << TILE_SCALER_SUBPIXEL_SHIFT) == out->precision_x */
		s64 subpix_x;

		subpix_x = ((s64)cal_param.hori_luma_int_ofst <<
			TILE_SCALER_SUBPIXEL_SHIFT) + cal_param.hori_luma_sub_ofst;
		subpix_x = ((subpix_x << TILE_SCALER_SUBPIXEL_SHIFT) +
			out->hori_step - 1) / out->hori_step;

		out->hori_int_ofst = subpix_x >> TILE_SCALER_SUBPIXEL_SHIFT;
		out->hori_sub_ofst = subpix_x -
			((s64)out->hori_int_ofst << TILE_SCALER_SUBPIXEL_SHIFT);
	}

	if (out->vert_algo == SCALER_6_TAPS) {
		out->vert_int_ofst = cal_param.vert_luma_int_ofst;
		out->vert_sub_ofst = ((s64)cal_param.vert_luma_sub_ofst <<
			TILE_SCALER_SUBPIXEL_SHIFT) / out->precision_y;
	} else { /* (1 << TILE_SCALER_SUBPIXEL_SHIFT) == out->precision_y */
		s64 subpix_y;

		subpix_y = ((s64)cal_param.vert_luma_int_ofst <<
			TILE_SCALER_SUBPIXEL_SHIFT) + cal_param.vert_luma_sub_ofst;
		subpix_y = ((subpix_y << TILE_SCALER_SUBPIXEL_SHIFT) +
			out->vert_step - 1) / out->vert_step;

		out->vert_int_ofst = subpix_y >> TILE_SCALER_SUBPIXEL_SHIFT;
		out->vert_sub_ofst = subpix_y -
			((s64)out->vert_int_ofst << TILE_SCALER_SUBPIXEL_SHIFT);
	}

	/* always enable hor and ver */
	out->hori_scale = 1;
	out->vert_scale = 1;
	/* Scaling size is 1, need to bound input */
	if (in->crop.r.width == in->out_width)
		out->vert_first = 1;

	out->con1 = out->hori_scale |
		    out->vert_scale << 1 |
		    out->vert_first << 4 |
		    out->hori_algo << 5 |
		    out->vert_algo << 7 |
		    cal_param.hori_trunc_bit << 10 |
		    cal_param.vert_trunc_bit << 13 |
		    cal_param.hori_tbl << 16 |
		    cal_param.vert_tbl << 21 |
		    in->use121filter << 26 |
		    cal_param.int_wclr_en << 31;
	out->con2 = cal_param.tap_adapt_en << 7 |
		    in->power_saving << 9 |
		    cal_param.hori_chroma_cubic_trunc_bit << 14 |
		    cal_param.hori_luma_cubic_trunc_bit << 17 |
		    cal_param.hori_cubic_trunc_en << 20 |
		    cal_param.vert_chroma_cubic_trunc_bit << 21 |
		    cal_param.vert_luma_cubic_trunc_bit << 24 |
		    out->vert_cubic_trunc << 27;
	out->tap_adapt = cal_param.tap_adapt_slope |
			 cal_param.tap_adapt_fallback_ratio << 4 |
			 cal_param.tap_adapt_var_coring << 10 |
			 cal_param.tap_adapt_dc_coring << 15 |
			 cal_param.tap_adapt_edge_thr << 20;
	out->etc_ctrl |= (cal_param.signal_enhance_mode << 30);
	out->hori_step &= 0x007fffff;
	out->vert_step &= 0x007fffff;
}

MODULE_DESCRIPTION("MTK MML RSZ FW");
MODULE_AUTHOR("Chris-YC Chen<chris-yc.chen@mediatek.com>");
MODULE_LICENSE("GPL");
