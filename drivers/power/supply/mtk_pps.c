// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 */

#include <linux/alarmtimer.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include "mtk_charger_algorithm_class.h"
#include "mtk_pps.h"

static int log_level = PPS_DBG_LEVEL;

int pps_get_log_level(void)
{
	return log_level;
}
#define _MS_TO_NS(msec)		((msec) * (NSEC_PER_MSEC))

/* Parameters */
#define PPS_VTA_INIT		5000	/* mV */
#define PPS_ITA_INIT		3000	/* mA */
#define PPS_TA_WDT_MIN		10000	/* ms */
#define PPS_VTA_GAP_MIN	200	/* mV */
#define PPS_VTA_VAR_MIN	103	/* % */
#define PPS_ITA_TRACKING_GAP	150	/* mA */
#define PPS_DVCHG_VBUSALM_GAP	100	/* mV */
#define PPS_DVCHG_STARTUP_CONVERT_RATIO	210
#define PPS_DVCHG_CHARGING_CONVERT_RATIO	202
#define PPS_VBUSOVP_RATIO	110
#define PPS_IBUSOCP_RATIO	150
#define PPS_VBATOVP_RATIO	110
#define PPS_IBATOCP_RATIO	150
#define PPS_ITAOCP_RATIO	150
#define PPS_IBUSUCPF_RECHECK		250	/* mA */
#define PPS_VBUS_CALI_THRESHOLD	150	/* mV */
#define PPS_CV_LOWER_BOUND_GAP		50	/* mV */
#define PPS_INIT_POLLING_INTERVAL	500	/* ms */
#define PPS_INIT_RETRY_MAX	3
#define PPS_MEASURE_R_RETRY_MAX	3
#define PPS_MEASURE_R_AVG_TIMES	10
#define PPS_VSYS_UPPER_BOUND            4700    /* mV */
#define PPS_VSYS_UPPER_BOUND_GAP        40      /* mV */
#define PPS_CV_UPPER_GAP        5      /* mV */
#define PPS_CV_DOWN_GAP         35      /* mV */
#define PPS_IEOC_CURR	800  /* mA */

#define PPS_HWERR_NOTIFY \
	(BIT(EVT_VBUSOVP) | BIT(EVT_IBUSOCP) | BIT(EVT_VBATOVP) | \
	 BIT(EVT_IBATOCP) | BIT(EVT_VOUTOVP) | BIT(EVT_VDROVP) | \
	 BIT(EVT_IBUSUCP_FALL))

#define PPS_RESET_NOTIFY \
	(BIT(EVT_DETACH) | BIT(EVT_HARDRESET))

static const char *const pps_dvchg_role_name[PPS_DVCHG_MAX] = {
	"master", "slave",
};

static const char *const pps_algo_state_name[PPS_ALGO_STATE_MAX] = {
	"INIT", "MEASURE_R", "SS_SWCHG", "SS_DVCHG", "CC_CV", "STOP",
};

/* If there's no property in dts, these values will be applied */
static const struct pps_algo_desc algo_desc_defval = {
	.polling_interval = 500,
	.ta_cv_ss_repeat_tmin = 25,
	.vbat_cv = 4480,
	.start_soc_min = 5,
	.start_soc_max = 80,
	.start_vbat_max = 4300,
	.idvchg_term = 500,
	.idvchg_step = 50,
	.ita_level = {3000, 2700, 2400, 2000},
	.rcable_level = {250, 278, 313, 375},
	.ita_level_dual = {4000, 3700, 3400, 3000},
	.rcable_level_dual = {188, 203, 221, 250},
	.idvchg_ss_init = 500,
	.idvchg_ss_step = 250,
	.idvchg_ss_step1 = 100,
	.idvchg_ss_step2 = 50,
	.idvchg_ss_step1_vbat = 4000,
	.idvchg_ss_step2_vbat = 4200,
	.ta_blanking = 500,
	.swchg_aicr = 0,
	.swchg_ichg = 0,
	.swchg_aicr_ss_init = 400,
	.swchg_aicr_ss_step = 200,
	.swchg_off_vbat = 4250,
	.force_ta_cv_vbat = 4250,
	.chg_time_max = 5400,
	.tta_level_def = {0, 0, 0, 0, 25, 40, 50, 60, 70},
	.tta_curlmt = {0, 0, 0, 0, 0, 300, 600, 900, -1},
	.tta_recovery_area = 3,
	.tbat_level_def = {10, 10, 10, 10, 20, 35, 40, 45, 50},
	.tbat_curlmt = {-1, -1, -1, 0, 0, 0, 0, 0, -1},
	/*very warm 40~45*/
	.tbat_t3_to_t4_cv = {4250, 4380, 4450, 4480, 4500},
	.tbat_t3_to_t4_curlmt = {0, 0, 0, 500, 1250},
	/*warm 35~40*/
	.tbat_t2_to_t3_cv = {4250, 4380, 4450, 4480, 4500},
	.tbat_t2_to_t3_curlmt = {0, 0, 0, 500, 1250},
	/*normal 20~35*/
	.tbat_t1_to_t2_cv = {4250, 4380, 4450, 4480, 4500},
	.tbat_t1_to_t2_curlmt = {0, 0, 0, 500, 1250},
	/*cool 10~20*/
	.tbat_t0_to_t1_cv = {4150, 4250, 4350, 4480, 4500},
	.tbat_t0_to_t1_curlmt = {0, 0, 500, 1000, 1250},
	.tbat_recovery_area = 3,
	.tdvchg_level_def = {0, 0, 0, 5, 25, 55, 60, 65, 70},
	.tdvchg_curlmt = {-1, -1, -1, 300, 0, 300, 600, 900, -1},
	.tdvchg_recovery_area = 3,
	.tswchg_level_def = {0, 0, 0, 5, 25, 55, 60, 65, 70},
	.tswchg_curlmt = {-1, -1, -1, 200, 0, 200, 300, 400, -1},
	.tswchg_recovery_area = 3,
	.ifod_threshold = 200,
	.rsw_min = 20,
	.ircmp_rbat = 40,
	.ircmp_vclamp = 0,
	.vta_cap_min = 6800,
	.vta_cap_max = 11000,
	.ita_cap_min = 1000,
	.allow_not_check_ta_status = true,
};

/*
 * @reset_ta: set output voltage/current of TA to 5V/3A and disable
 *            direct charge
 * @hardreset: send hardreset to port partner
 * Note: hardreset's priority is higher than reset_ta
 */
struct pps_stop_info {
	bool hardreset_ta;
	bool reset_ta;
};

static inline enum chg_idx to_chgidx(enum pps_dvchg_role role)
{
	if (role == PPS_DVCHG_MASTER)
		return DVCHG1;
	return DVCHG2;
}

/* Check if there is error notification coming from H/W */
static bool pps_is_hwerr_notified(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	bool err = false;
	u32 hwerr = PPS_HWERR_NOTIFY;

	mutex_lock(&data->notify_lock);
	if (data->ignore_ibusucpf)
		hwerr &= ~BIT(EVT_IBUSUCP_FALL);
	err = !!(data->notify & hwerr);
	if (err)
		PPS_ERR("H/W error(0x%08X)", hwerr);
	mutex_unlock(&data->notify_lock);
	return err;
}

/*
 * Get ADC value from divider charger
 * Note: ibus will sum up value from all enabled chargers
 * (master dvchg, slave dvchg and swchg)
 */
static int pps_stop(struct pps_algo_info *info, struct pps_stop_info *sinfo);
static int pps_get_adc(struct pps_algo_info *info, enum pps_adc_channel chan,
			int *val)
{
	struct pps_algo_data *data = info->data;
	int ret, i, ibus;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (atomic_read(&data->stop_algo)) {
		PPS_INFO("stop algo\n");
		goto stop;
	}
	*val = 0;
	if (chan == PPS_ADCCHAN_IBUS) {
		for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
			if (!data->is_dvchg_en[i])
				continue;
			ret = pps_hal_get_adc(info->alg, to_chgidx(i),
					       PPS_ADCCHAN_IBUS, &ibus);
			if (ret < 0) {
				PPS_ERR("get dvchg ibus fail(%d)\n", ret);
				return ret;
			}
			*val += ibus;
		}
		if (data->is_swchg_en) {
			ret = pps_hal_get_adc(info->alg, CHG1,
					       PPS_ADCCHAN_IBUS, &ibus);
			if (ret < 0) {
				PPS_ERR("get swchg ibus fail(%d)\n", ret);
				return ret;
			}
			*val += ibus;
		}
		return 0;
	}
	return pps_hal_get_adc(info->alg, DVCHG1, chan, val);
stop:
	pps_stop(info, &sinfo);
	return -EIO;
}

/*
 * Calculate VBUS for divider charger
 * If divider charger is charging, the VBUS only needs to be 2 times of VOUT.
 */
static inline u32 pps_vout2vbus(struct pps_algo_info *info, u32 vout)
{
	struct pps_algo_data *data = info->data;
	u32 ratio = data->is_dvchg_en[PPS_DVCHG_MASTER] ?
		PPS_DVCHG_CHARGING_CONVERT_RATIO :
		PPS_DVCHG_STARTUP_CONVERT_RATIO;

	return percent_2(vout, ratio);
}

/*
 * Maximum PPS_VTA_VAR_MIN(%) variation (from PD's sepcification)
 * Keep vta_setting PPS_VTA_VAR_MIN(%) higher than vta_measure
 * and make sure it has minimum gap, PPS_VTA_GAP_MIN
 */
static inline u32 pps_vta_add_gap(struct pps_algo_info *info, u32 vta)
{
	return max(percent_2(vta, PPS_VTA_VAR_MIN), vta + PPS_VTA_GAP_MIN);
}

/*
 * Get output current and voltage measured by TA
 * and updates measured data
 */
static inline int pps_get_ta_cap(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;

	return pps_hal_get_ta_output(info->alg, &data->vta_measure,
				      &data->ita_measure);
}

/*
 * Get output current and voltage measured by TA
 * and updates measured data
 * If ta does not support measure capability, dvchg's ADC is used instead
 */
static inline int pps_get_ta_cap_by_supportive(struct pps_algo_info *info,
						 int *vta, int *ita)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	if (auth_data->support_meas_cap) {
		ret = pps_get_ta_cap(info);
		if (ret < 0) {
			PPS_ERR("get ta cap fail(%d)\n", ret);
			return ret;
		}
		*vta = data->vta_measure;
		*ita = data->ita_measure;
		return 0;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, vta);
	if (ret < 0) {
		PPS_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	return pps_get_adc(info, PPS_ADCCHAN_IBUS, ita);
}

/* Calculate ibat from ita */
static inline u32 pps_cal_ibat(struct pps_algo_info *info, u32 ita)
{
	struct pps_algo_data *data = info->data;

	return 2 * (data->is_swchg_en ? (ita - data->aicr_setting) : ita);
}

/*
 * Calculate calibrated output voltage of TA by measured resistence
 * Firstly, calculate voltage needed by divider charger
 * Finally, calculate voltage outputing from TA
 *
 * @ita: expected output current of TA
 * @vta: calibrated output voltage of TA
 */
static int pps_get_cali_vta(struct pps_algo_info *info, u32 ita, u32 *vta)
{
	int ret, vbat;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 ibat, vbus, _vta, comp;

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}
	ibat = pps_cal_ibat(info, ita);
	vbus = pps_vout2vbus(info, vbat + div1000_2(ibat * data->r_sw));
	*vta = vbus + (data->vbus_cali + data->vta_comp +
	       div1000_2(ita * data->r_cable_by_swchg));
	if (data->is_dvchg_en[PPS_DVCHG_MASTER]) {
		ret = pps_get_ta_cap(info);
		if (ret < 0) {
			PPS_ERR("get ta cap fail(%d)\n", ret);
			return ret;
		}
		_vta = pps_vta_add_gap(info, data->vta_measure);
		if (_vta > *vta) {
			comp = _vta - *vta;
			data->vta_comp += comp;
			PPS_DBG("comp,add=(%d,%d)\n", data->vta_comp, comp);
		}
		*vta = max(*vta, _vta);
	}
	if (*vta >= auth_data->vcap_max)
		*vta = auth_data->vcap_max;
	return 0;
}

/*
 * Tracking vbus of divider charger using vbusovp alarm
 * If vbusovp alarm is triggered, algorithm needs to come up with a new vbus
 */
static int pps_set_vbus_tracking(struct pps_algo_info *info)
{
	int ret, vbus;

	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
	if (ret < 0) {
		PPS_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	return pps_hal_set_vbusovp_alarm(info->alg, DVCHG1,
					  vbus + PPS_DVCHG_VBUSALM_GAP);
}

/* Calculate power limited ita according to TA's power limitation */
static u32 pps_get_ita_pwr_lmt_by_vta(struct pps_algo_info *info, u32 vta)
{
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 ita_pwr_lmt;

	PPS_INFO("pwr_lmt(%d),pdp(%d)\n", auth_data->pwr_lmt, auth_data->pdp);
	if (!auth_data->pwr_lmt || !auth_data->pdp)
		return data->ita_lmt;

	ita_pwr_lmt = precise_div_2(auth_data->pdp * 1000000, vta);
	/* Round to nearest level */
	if (auth_data->support_cc) {
		ita_pwr_lmt /= auth_data->ita_step;
		ita_pwr_lmt *= auth_data->ita_step;
	}
	return min(ita_pwr_lmt, data->ita_lmt);
}

static inline u32 pps_get_ita_tracking_max(u32 ita)
{
	return min(percent_2(ita, PPS_ITAOCP_RATIO),
		   (u32)(ita + PPS_ITA_TRACKING_GAP));
}

/*
 * Set output capability of TA in CC mode and update setting in data
 *
 * @vta: output voltage of TA, mV
 * @ita: output current of TA, mA
 */
static int
pps_force_ta_cv(struct pps_algo_info *info, struct pps_stop_info *sinfo);
static inline int pps_set_ta_cap_cc(struct pps_algo_info *info, u32 vta,
				      u32 ita)
{
	int ret, vbat;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	bool set_opt_vta = true;
	bool is_ta_cc = false;
	u32 opt_vta;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->vta_setting == vta && data->ita_setting == ita &&
	    data->state != PPS_ALGO_INIT)
		return 0;
	while (true) {
		if (atomic_read(&data->stop_algo)) {
			PPS_INFO("stop algo\n");
			goto stop;
		}
		/* Check TA's PDP */
		data->ita_pwr_lmt = pps_get_ita_pwr_lmt_by_vta(info, vta);
		if (data->ita_pwr_lmt < ita) {
			PPS_INFO("ita(%d) > ita_pwr_lmt(%d)\n", ita,
				 data->ita_pwr_lmt);
			ita = data->ita_pwr_lmt;
		}
		ret = pps_hal_set_ta_cap(info->alg, vta, ita);
		if (ret < 0) {
			PPS_ERR("set ta cap fail(%d)\n", ret);
			return ret;
		}
		msleep(desc->ta_blanking);
		if (!data->is_dvchg_en[PPS_DVCHG_MASTER])
			break;
		ret = pps_hal_is_ta_cc(info->alg, &is_ta_cc);
		if (ret < 0) {
			PPS_ERR("get ta cc mode fail(%d)\n", ret);
			return ret;
		}
		ret = pps_get_ta_cap(info);
		if (ret < 0) {
			PPS_ERR("get ta cap fail(%d)\n", ret);
			return ret;
		}
		PPS_DBG("vta(set,meas,comp),ita(set,meas)=(%d,%d,%d),(%d,%d)\n",
			vta, data->vta_measure, data->vta_comp, ita,
			data->ita_measure);
		if (is_ta_cc) {
			opt_vta = pps_vta_add_gap(info, data->vta_measure);
			if (vta > opt_vta && set_opt_vta) {
				data->vta_comp -= (vta - opt_vta);
				vta = opt_vta;
				set_opt_vta = false;
				continue;
			}
			break;
		}
		if (vta >= auth_data->vcap_max) {
			PPS_ERR("vta(%d) over capability(%d)\n", vta,
				auth_data->vcap_max);
			goto stop;
		}
		if (pps_is_hwerr_notified(info)) {
			PPS_ERR("H/W error notified\n");
			goto stop;
		}
		ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
		if (ret < 0) {
			PPS_ERR("get vbat fail(%d)\n", ret);
			return ret;
		}
		if (vbat >= data->vbat_cv) {
			PPS_INFO("vbat(%d), decrease ita immediately\n", vbat);
			ita -= auth_data->ita_step;
			continue;
		}
		PPS_ERR("Not in cc mode\n");
		if (data->ita_measure > pps_get_ita_tracking_max(data->ita_setting)) {
			ret = pps_force_ta_cv(info, &sinfo);
			if (ret < 0)
				goto stop;
			return 0;
		}
		set_opt_vta = false;
		data->vta_comp += auth_data->vta_step;
		vta += auth_data->vta_step;
		vta = min(vta, (u32)auth_data->vcap_max);
	}
	data->vta_setting = vta;
	data->ita_setting = ita;
	PPS_INFO("vta,ita = (%d,%d)\n", vta, ita);
	pps_set_vbus_tracking(info);

	return 0;
stop:
	pps_stop(info, &sinfo);
	return -EIO;
}

/*
 * Set TA's output voltage & current by a given current and
 * calculated voltage
 */
static inline int pps_set_ta_cap_cc_by_cali_vta(struct pps_algo_info *info,
						 u32 ita)
{
	int ret;
	u32 vta;

	ret = pps_get_cali_vta(info, ita, &vta);
	if (ret < 0) {
		PPS_ERR("get cali vta fail(%d)\n", ret);
		return ret;
	}
	return pps_set_ta_cap_cc(info, vta, ita);
}

static inline void pps_update_ita_gap(struct pps_algo_info *info, u32 ita_gap)
{
	int i;
	u32 val = 0, avg_cnt = PPS_ITA_GAP_WINDOW_SIZE;
	struct pps_algo_data *data = info->data;

	if (ita_gap < data->ita_gap_per_vstep)
		return;
	data->ita_gap_window_idx = (data->ita_gap_window_idx + 1) %
				   PPS_ITA_GAP_WINDOW_SIZE;
	data->ita_gaps[data->ita_gap_window_idx] = ita_gap;

	for (i = 0; i < PPS_ITA_GAP_WINDOW_SIZE; i++) {
		if (data->ita_gaps[i] == 0)
			avg_cnt--;
		else
			val += data->ita_gaps[i];
	}
	data->ita_gap_per_vstep = avg_cnt != 0 ? precise_div_2(val, avg_cnt) : 0;
}

static inline int pps_set_ta_cap_cv(struct pps_algo_info *info, u32 vta,
				     u32 ita)
{
	int ret, ita_meas_pre, ita_meas_post, vta_meas;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 vstep_cnt, ita_gap, vta_gap;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->vta_setting == vta && data->ita_setting == ita)
		return 0;
	while (true) {
		if (pps_is_hwerr_notified(info)) {
			PPS_ERR("H/W error notified\n");
			goto stop;
		}
		if (atomic_read(&data->stop_algo)) {
			PPS_INFO("stop algo\n");
			goto stop;
		}
		if (vta > auth_data->vcap_max) {
			PPS_ERR("vta(%d) over capability(%d)\n", vta,
				 auth_data->vcap_max);
			goto stop;
		}
		if (ita < auth_data->ita_min) {
			PPS_INFO("ita(%d) under ita_min(%d)\n", ita,
				  auth_data->ita_min);
			ita = auth_data->ita_min;
		}
		vta_gap = abs(data->vta_setting - vta);

		/* Get ta cap before setting */
		ret = pps_get_ta_cap_by_supportive(info, &vta_meas,
						    &ita_meas_pre);
		if (ret < 0) {
			PPS_ERR("get ta cap by supportive fail(%d)\n", ret);
			return ret;
		}

		/* Not to increase vta if it exceeds pwr_lmt */
		data->ita_pwr_lmt = pps_get_ita_pwr_lmt_by_vta(info, vta);
		if (vta > data->vta_setting &&
		    (data->ita_pwr_lmt <
		     ita_meas_pre + data->ita_gap_per_vstep)) {
			PPS_INFO("ita_meas(%d) + ita_gap(%d) > pwr_lmt(%d)\n",
				  ita_meas_pre, data->ita_gap_per_vstep,
				  data->ita_pwr_lmt);
			return 0;
		}

		/* Set ta cap */
		ret = pps_hal_set_ta_cap(info->alg, vta, ita);
		if (ret < 0) {
			PPS_ERR("set ta cap fail(%d)\n", ret);
			return ret;
		}
		if (vta_gap > auth_data->vta_step ||
		    data->state != PPS_ALGO_SS_DVCHG)
			msleep(desc->ta_blanking);

		/* Get ta cap after setting */
		ret = pps_get_ta_cap_by_supportive(info, &vta_meas,
						    &ita_meas_post);
		if (ret < 0) {
			PPS_ERR("get ta cap by supportive fail(%d)\n", ret);
			return ret;
		}

		if (data->is_dvchg_en[PPS_DVCHG_MASTER] &&
		    (ita_meas_post > ita_meas_pre) &&
		    (vta > data->vta_setting)) {
			vstep_cnt = precise_div_2(max(vta, (u32)vta_meas) -
						data->vta_setting,
						auth_data->vta_step);
			ita_gap = precise_div_2(ita_meas_post - ita_meas_pre,
					      vstep_cnt);
			pps_update_ita_gap(info, ita_gap);
			PPS_INFO("ita gap(now,updated)=(%d,%d)\n",
				  ita_gap, data->ita_gap_per_vstep);
		}
		data->vta_setting = vta;
		data->ita_setting = ita;
		if (ita_meas_post <= pps_get_ita_tracking_max(ita))
			break;
		vta -= auth_data->vta_step;
		PPS_INFO("ita_meas %dmA over setting %dmA, keep tracking...\n",
			  ita_meas_post, ita);
	}

	data->vta_measure = vta_meas;
	data->ita_measure = ita_meas_post;
	PPS_INFO("vta(set,meas):(%d,%d),ita(set,meas):(%d,%d)\n",
		 data->vta_setting, data->vta_measure, data->ita_setting,
		 data->ita_measure);
	return 0;
stop:
	pps_stop(info, &sinfo);
	return -EIO;
}

static inline void pps_calculate_vbat_ircmp(struct pps_algo_info *info)
{
	int ret, ibat;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 ircmp;

	if (!data->is_dvchg_en[PPS_DVCHG_MASTER]) {
		data->vbat_ircmp = 0;
		return;
	}

	ret = pps_get_adc(info, PPS_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PPS_ERR("get ibat fail(%d)\n", ret);
		return;
	}
	ircmp = max(div1000_2(ibat * data->r_bat), desc->ircmp_vclamp);
	/*
	 * For safety,
	 * if state is CC_CV, ircmp can only be smaller than previous one
	 */
	if (data->state == PPS_ALGO_CC_CV)
		ircmp = min(data->vbat_ircmp, ircmp);
	data->vbat_ircmp = min(desc->ircmp_vclamp, ircmp);
	PPS_INFO("vbat_ircmp(vclamp,ibat,rbat)=%d(%d,%d,%d)\n",
		 data->vbat_ircmp, desc->ircmp_vclamp, ibat, data->r_bat);
}

static inline void pps_select_vbat_cv(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 cv = data->vbat_cv;
	u32 cv_no_ircmp = desc->vbat_cv;

	mutex_lock(&data->ext_lock);
	if (data->cv_limit > 0)
		cv_no_ircmp = min(cv_no_ircmp, (u32)data->cv_limit);

	if (cv_no_ircmp != data->vbat_cv_no_ircmp)
		data->vbat_cv_no_ircmp = cv_no_ircmp;

	cv = data->vbat_cv_no_ircmp + data->vbat_ircmp;
	if (cv == data->vbat_cv)
		goto out;

	/* VBATOVP ALARM */
	ret = pps_hal_set_vbatovp_alarm(info->alg, DVCHG1, cv);
	if (ret < 0) {
		PPS_ERR("set vbatovp alarm fail(%d)\n", ret);
		goto out;
	}
	data->vbat_cv = cv;
	data->cv_lower_bound = data->vbat_cv - PPS_CV_LOWER_BOUND_GAP;
out:
	PPS_INFO("vbat_cv(org,limit,no_ircmp,low_bound)=%d(%d,%d,%d,%d)\n",
		  data->vbat_cv, desc->vbat_cv, data->cv_limit,
		  data->vbat_cv_no_ircmp, data->cv_lower_bound);
	mutex_unlock(&data->ext_lock);
}

static inline enum pps_jeita_level pps_check_cv_level(int vol ,int* cv)
{
	enum pps_jeita_level i;

	for(i = PPS_JEITA_LEVEL_0; i < PPS_JEITA_MAX; i++){
		if(vol && (vol < cv[i])){
			return i;
		}
	}
	return PPS_JEITA_MAX;
}

static inline int pps_select_vbat_cv_lmt(struct pps_algo_info *info)
{
	int ret = 0, vbat = 0,cv_lmt = 0;
	enum pps_jeita_level level = PPS_JEITA_MAX;
	struct pps_algo_desc *desc = info->desc;
	struct pps_algo_data *data = info->data;

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}

	switch(data->tbat_level){
	case PPS_THERMAL_COOL:
		level = pps_check_cv_level(vbat,&desc->tbat_t0_to_t1_cv[0]);
		if(level != PPS_JEITA_MAX){
			cv_lmt = desc->tbat_t0_to_t1_curlmt[level];
		}
		break;
	case PPS_THERMAL_NORMAL:
		level = pps_check_cv_level(vbat,&desc->tbat_t1_to_t2_cv[0]);
		if(level != PPS_JEITA_MAX){
			cv_lmt = desc->tbat_t1_to_t2_curlmt[level];
		}
		break;
	case PPS_THERMAL_WARM:
		level = pps_check_cv_level(vbat,&desc->tbat_t2_to_t3_cv[0]);
		if(level != PPS_JEITA_MAX){
			cv_lmt = desc->tbat_t2_to_t3_curlmt[level];
		}
		break;
	case PPS_THERMAL_VERY_WARM:
		level = pps_check_cv_level(vbat,&desc->tbat_t3_to_t4_cv[0]);
		if(level != PPS_JEITA_MAX){
			cv_lmt = desc->tbat_t3_to_t4_curlmt[level];
		}
		break;
	default:
		PPS_ERR("tbat is too cold or high!\n");
		break;
	}
	PPS_INFO("(vbat,tbat_level)(%d,%d),cv_lmt(%d)\n", vbat, data->tbat_level, cv_lmt);
	return cv_lmt;
}
/*
 * Select current limit according to severial status
 * If switching charger is charging, add AICR setting to ita
 * For now, the following features are taken into consider
 * 1. Resistence
 * 2. Phone's thermal throttling
 * 3. TA's power limit
 * 4. TA's temperature
 * 5. Battery's temperature
 * 6. Divider charger's temperature
 */
static inline int pps_get_ita_lmt(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 ita = data->ita_lmt;
	u32 cv_lmt = 0;

	mutex_lock(&data->ext_lock);
	if (data->input_current_limit >= 0)
		ita = min(ita, (u32)data->input_current_limit);
	if (data->ita_pwr_lmt > 0)
		ita = min(ita, data->ita_pwr_lmt);
	cv_lmt = pps_select_vbat_cv_lmt(info);
	ita = min(ita, data->ita_lmt - cv_lmt);
	if (data->tried_dual_dvchg) {
		ita = min(ita, data->ita_lmt - (2 * desc->tta_curlmt[data->tta_level]));
		ita = min(ita, data->ita_lmt - (2 * desc->tbat_curlmt[data->tbat_level]));
		ita = min(ita, data->ita_lmt - (2 * desc->tdvchg_curlmt[data->tdvchg_level]));
	} else {
		ita = min(ita, data->ita_lmt - desc->tta_curlmt[data->tta_level]);
		ita = min(ita, data->ita_lmt - desc->tbat_curlmt[data->tbat_level]);
		ita = min(ita, data->ita_lmt - desc->tdvchg_curlmt[data->tdvchg_level]);
	}
	PPS_INFO("ita(org,tta,tbat,tdvchg,prlmt,throt)=%d(%d,%d,%d,%d,%d,%d)\n",
		 ita, data->ita_lmt, desc->tta_curlmt[data->tta_level],
		 desc->tbat_curlmt[data->tbat_level],
		 desc->tdvchg_curlmt[data->tdvchg_level], data->ita_pwr_lmt,
		 data->input_current_limit);
	mutex_unlock(&data->ext_lock);
	return ita;
}

static inline int pps_get_idvchg_lmt(struct pps_algo_info *info)
{
	u32 ita_lmt, idvchg_lmt;
	struct pps_algo_data *data = info->data;

	ita_lmt = pps_get_ita_lmt(info);
	idvchg_lmt = min(data->idvchg_cc, ita_lmt);
	PPS_INFO("idvchg_lmt(ita_lmt,idvchg_cc)=%d(%d,%d)\n", idvchg_lmt,
		 ita_lmt, data->idvchg_cc);
	return idvchg_lmt;
}

/* Calculate VBUSOV S/W level */
static u32 pps_get_dvchg_vbusovp(struct pps_algo_info *info, u32 ita)
{
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 vout, ibat;

	ibat = pps_cal_ibat(info, ita);
	vout = desc->vbat_cv + div1000_2(ibat * data->r_sw);
	return min(percent_2(pps_vout2vbus(info, vout), PPS_VBUSOVP_RATIO),
		   data->vbusovp);
}

/* Calculate IBUSOC S/W level */
static u32 pps_get_dvchg_ibusocp(struct pps_algo_info *info, u32 ita)
{
	struct pps_algo_data *data = info->data;
	u32 ibus, ratio = PPS_IBUSOCP_RATIO;

	ibus = data->is_swchg_en ? (ita - data->aicr_setting) : ita;
	/* Add 10% for unbalance tolerance */
	if (data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
		ibus = precise_div_2(ibus, 2);
		ratio += 10;
	}
	return percent_2(ibus, ratio);
}

/* Calculate VBATOV S/W level */
static u32 pps_get_vbatovp(struct pps_algo_info *info)
{
	struct pps_algo_desc *desc = info->desc;

	return percent_2(desc->vbat_cv + desc->ircmp_vclamp, PPS_VBATOVP_RATIO);
}

/* Calculate IBATOC S/W level */
static u32 pps_get_ibatocp(struct pps_algo_info *info, u32 ita)
{
	struct pps_algo_data *data = info->data;
	u32 ibat;

	ibat = pps_cal_ibat(info, ita);
	if (data->is_swchg_en)
		ibat += data->ichg_setting;
	return percent_2(ibat, PPS_IBATOCP_RATIO);
}

/* Calculate ITAOC S/W level */
static u32 pps_get_itaocp(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;

	return percent_2(data->ita_setting, PPS_ITAOCP_RATIO);
}

static int pps_set_dvchg_protection(struct pps_algo_info *info, bool dual)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 vout, idvchg_lmt;
	u32 vbusovp, ibusocp, vbatovp, ibatocp;

	/* VBATOVP ALARM */
	ret = pps_hal_set_vbatovp_alarm(info->alg, DVCHG1, desc->vbat_cv);
	if (ret < 0) {
		PPS_ERR("set vbatovp alarm fail(%d)\n", ret);
		return ret;
	}

	/* VBUSOVP */
	vout = desc->vbat_cv + div1000_2(2 * data->idvchg_cc * data->r_sw);
	vbusovp = percent_2(pps_vout2vbus(info, vout), PPS_VBUSOVP_RATIO);
	vbusovp = min(vbusovp, (u32)auth_data->vcap_max);
	ret = pps_hal_set_vbusovp(info->alg, DVCHG1, vbusovp);
	if (ret < 0) {
		PPS_ERR("set vbusovp fail(%d)\n", ret);
		return ret;
	}
	data->vbusovp = vbusovp;
	/* For TA CV mode, vbusovp alarm is not required */
	if (!auth_data->support_cc) {
		ret = pps_hal_set_vbusovp_alarm(info->alg, DVCHG1, vbusovp);
		if (ret < 0) {
			PPS_ERR("set vbusovp alarm fail(%d)\n", ret);
			return ret;
		}
	}

	/* IBUSOCP */
	idvchg_lmt = min(data->idvchg_cc, (u32)auth_data->ita_max);
	ibusocp = percent_2(idvchg_lmt, PPS_IBUSOCP_RATIO);
	if (data->is_dvchg_exist[PPS_DVCHG_SLAVE] && dual) {
		/* Add 10% for unbalance tolerance */
		ibusocp = percent_2(precise_div_2(idvchg_lmt, 2),
				  PPS_IBUSOCP_RATIO + 10);
		ret = pps_hal_set_ibusocp(info->alg, DVCHG2, ibusocp);
		if (ret < 0) {
			PPS_ERR("set slave ibusocp fail(%d)\n", ret);
			return ret;
		}
	}
	ret = pps_hal_set_ibusocp(info->alg, DVCHG1, ibusocp);
	if (ret < 0) {
		PPS_ERR("set ibusocp fail(%d)\n", ret);
		return ret;
	}

	/* VBATOVP */
	vbatovp = percent_2(desc->vbat_cv + desc->ircmp_vclamp,
			  PPS_VBATOVP_RATIO);
	ret = pps_hal_set_vbatovp(info->alg, DVCHG1, vbatovp);
	if (ret < 0) {
		PPS_ERR("set vbatovp fail(%d)\n", ret);
		return ret;
	}

	/* IBATOCP */
	ibatocp = percent_2(2 * data->idvchg_cc + desc->swchg_ichg,
			  PPS_IBATOCP_RATIO);
	ret = pps_hal_set_ibatocp(info->alg, DVCHG1, ibatocp);
	if (ret < 0) {
		PPS_ERR("set ibatocp fail(%d)\n", ret);
		return ret;
	}
	PPS_INFO("vbusovp,ibusocp,vbatovp,ibatocp = (%d,%d,%d,%d)\n",
		 vbusovp, ibusocp, vbatovp, ibatocp);
	return 0;
}

/*
 * Enable/Disable divider charger
 *
 * @en: enable/disable
 */
static int pps_enable_dvchg_charging(struct pps_algo_info *info,
				      enum pps_dvchg_role role, bool en)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;

	if (!data->is_dvchg_exist[role])
		return -ENODEV;
	if (data->is_dvchg_en[role] == en)
		return 0;
	PPS_INFO("en[%s] = %d\n", pps_dvchg_role_name[role], en);
	ret = pps_hal_enable_charging(info->alg, to_chgidx(role), en);
	if (ret < 0) {
		PPS_ERR("en chg fail(%d)\n", ret);
		return ret;
	}
	data->is_dvchg_en[role] = en;
	msleep(desc->ta_blanking);
	return 0;
}

/*
 * Set protection parameters, disable swchg and  enable divider charger
 *
 * @en: enable/disable
 */
static int pps_set_dvchg_charging(struct pps_algo_info *info, bool en)
{
	int ret;
	struct pps_algo_data *data = info->data;

	if (!data->is_dvchg_exist[PPS_DVCHG_MASTER])
		return -ENODEV;

	PPS_INFO("en = %d\n", en);

	if (en) {
		ret = pps_hal_enable_hz(info->alg, CHG1, true);
		if (ret < 0) {
			PPS_ERR("set swchg hz fail(%d)\n", ret);
			return ret;
		}
		ret = pps_set_operating_mode(info->alg,DVCHG1,true);
		if (ret < 0) {
			PPS_ERR("set operating mode fail(%d)\n", ret);
			return ret;
		}
		ret = pps_set_dvchg_protection(info, false);
		if (ret < 0) {
			PPS_ERR("set protection fail(%d)\n", ret);
			return ret;
		}
	}
	ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER, en);
	if (ret < 0)
		return ret;
	if (!en) {
		ret = pps_hal_enable_hz(info->alg, CHG1, false);
		if (ret < 0) {
			PPS_ERR("disable swchg hz fail(%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

/*
 * Enable charging of switching charger
 * For divide by two algorithm, according to swchg_ichg to decide enable or not
 *
 * @en: enable/disable
 */
static int pps_enable_swchg_charging(struct pps_algo_info *info, bool en)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;

	PPS_INFO("en = %d\n", en);
	if (en) {
		ret = pps_hal_enable_charging(info->alg, CHG1, true);
		if (ret < 0) {
			PPS_ERR("en swchg fail(%d)\n", ret);
			return ret;
		}
		ret = pps_hal_enable_hz(info->alg, CHG1, false);
		if (ret < 0) {
			PPS_ERR("disable hz fail(%d)\n", ret);
			return ret;
		}
	} else {
		ret = pps_hal_enable_hz(info->alg, CHG1, true);
		if (ret < 0) {
			PPS_ERR("disable hz fail(%d)\n", ret);
			return ret;
		}
		ret = pps_hal_enable_charging(info->alg, CHG1, false);
		if (ret < 0) {
			PPS_ERR("en swchg fail(%d)\n", ret);
			return ret;
		}
	}
	data->is_swchg_en = en;
	ret = pps_hal_set_vbatovp_alarm(info->alg, DVCHG1,
		en ? desc->swchg_off_vbat : desc->vbat_cv);
	if (ret < 0) {
		PPS_ERR("set vbatovp alarm fail(%d)\n", ret);
		return ret;
	}
	return 0;
}

/*
 * Set AICR & ICHG of switching charger
 *
 * @aicr: setting of AICR
 * @ichg: setting of ICHG
 */
static int pps_set_swchg_cap(struct pps_algo_info *info, u32 aicr)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 ichg, vbat, vbus;

	if (aicr == data->aicr_setting)
		goto set_ichg;
	ret = pps_hal_set_aicr(info->alg, CHG1, aicr);
	if (ret < 0) {
		PPS_ERR("set aicr fail(%d)\n", ret);
		return ret;
	}
	data->aicr_setting = aicr;
set_ichg:
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
	if (ret < 0) {
		PPS_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	/* 90% charging efficiency */
	ichg = precise_div_2(percent_2(vbus * aicr, 90), vbat);
	ichg = min(ichg, desc->swchg_ichg);
	if (ichg == data->ichg_setting)
		return 0;
	ret = pps_hal_set_ichg(info->alg, CHG1, ichg);
	if (ret < 0) {
		PPS_ERR("set_ichg fail(%d)\n", ret);
		return ret;
	}
	data->ichg_setting = ichg;
	PPS_INFO("AICR = %d, ICHG = %d\n", aicr, ichg);
	return 0;
}

/*
 * Enable TA by algo
 *
 * @en: enable/disable
 * @mV: requested output voltage
 * @mA: requested output current
 */
static int pps_enable_ta_charging(struct pps_algo_info *info, bool en, int mV,
				   int mA)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 wdt = max(desc->polling_interval * 2, (u32)PPS_TA_WDT_MIN);

	PPS_INFO("en = %d\n", en);
	if (en) {
		ret = pps_hal_set_ta_wdt(info->alg, wdt);
		if (ret < 0) {
			PPS_ERR("set ta wdt fail(%d)\n", ret);
			return ret;
		}
		ret = pps_hal_enable_ta_wdt(info->alg, true);
		if (ret < 0) {
			PPS_ERR("en ta wdt fail(%d)\n", ret);
			return ret;
		}
	}
	ret = pps_hal_enable_ta_charging(info->alg, en, mV, mA);
	if (ret < 0) {
		PPS_ERR("en ta charging fail(%d)\n", ret);
		return ret;
	}
	if (!en) {
		ret = pps_hal_enable_ta_wdt(info->alg, false);
		if (ret < 0)
			PPS_ERR("disable ta wdt fail(%d)\n", ret);
	}
	data->vta_setting = mV;
	data->ita_setting = mA;
	return ret;
}

static int pps_send_notification(struct pps_algo_info *info,
				  unsigned long val,
				  struct chg_alg_notify *notify)
{
	return srcu_notifier_call_chain(&info->alg->evt_nh, val, notify);
}

/* Stop PPS charging and reset parameter */
static int pps_stop(struct pps_algo_info *info, struct pps_stop_info *sinfo)
{
	struct pps_algo_data *data = info->data;
	struct chg_alg_notify notify = {
		.evt = EVT_ALGO_STOP,
	};

	if (data->state == PPS_ALGO_STOP) {
		/*
		 * Always clear stop_algo,
		 * in case it is called from pps_stop_algo
		 */
		atomic_set(&data->stop_algo, 0);
		PPS_DBG("already stop\n");
		return 0;
	}

	PPS_INFO("reset ta(%d), hardreset ta(%d)\n", sinfo->reset_ta,
		 sinfo->hardreset_ta);
	data->state = PPS_ALGO_STOP;
	atomic_set(&data->stop_algo, 0);
	alarm_cancel(&data->timer);

	if (data->is_swchg_en)
		pps_enable_swchg_charging(info, false);
	pps_enable_dvchg_charging(info, PPS_DVCHG_SLAVE, false);
	pps_set_dvchg_charging(info, false);
	if (!(data->notify & PPS_RESET_NOTIFY)) {
		if (sinfo->hardreset_ta)
			pps_hal_send_ta_hardreset(info->alg);
		else if (sinfo->reset_ta) {
			pps_hal_set_ta_cap(info->alg, PPS_VTA_INIT,
					    PPS_ITA_INIT);
			pps_enable_ta_charging(info, false, PPS_VTA_INIT,
						PPS_ITA_INIT);
		}
	}
	pps_hal_enable_sw_vbusovp(info->alg, true);
	pps_send_notification(info, EVT_ALGO_STOP, &notify);
	return 0;
}

static inline void pps_init_algo_data(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 *rcable_level = desc->rcable_level;
	u32 *ita_level = desc->ita_level;

	data->ita_lmt = min(ita_level[PPS_RCABLE_NORMAL],
			    (u32)auth_data->ita_max);
	data->idvchg_ss_init = max(data->idvchg_ss_init,
				   (u32)auth_data->ita_min);
	data->idvchg_ss_init = min(data->idvchg_ss_init, data->ita_lmt);
	data->ita_pwr_lmt = 0;
	data->idvchg_cc = ita_level[PPS_RCABLE_NORMAL] - desc->swchg_aicr;
	data->idvchg_term = desc->idvchg_term;
	data->err_retry_cnt = 0;
	data->is_swchg_en = false;
	data->is_dvchg_en[PPS_DVCHG_MASTER] = false;
	data->is_dvchg_en[PPS_DVCHG_SLAVE] = false;
	data->suspect_ta_cc = false;
	data->aicr_setting = 0;
	data->ichg_setting = 0;
	data->vta_setting = PPS_VTA_INIT;
	data->ita_setting = PPS_ITA_INIT;
	data->ita_gap_per_vstep = 0;
	data->ita_gap_window_idx = 0;
	memset(data->ita_gaps, 0, sizeof(data->ita_gaps));
	data->is_vbat_over_cv = false;
	data->ignore_ibusucpf = false;
	data->force_ta_cv = false;
	data->vbat_cv = desc->vbat_cv;
	data->vbat_cv_no_ircmp = desc->vbat_cv;
	data->cv_lower_bound = desc->vbat_cv - PPS_CV_LOWER_BOUND_GAP;
	data->vta_comp = 0;
	data->zcv = 0;
	data->r_bat = desc->ircmp_rbat;
	data->r_sw = desc->rsw_min;
	data->r_cable = rcable_level[PPS_RCABLE_NORMAL];
	data->r_cable_by_swchg = rcable_level[PPS_RCABLE_NORMAL];
	data->tbat_level = PPS_THERMAL_NORMAL;
	data->tta_level = PPS_THERMAL_NORMAL;
	data->tdvchg_level = PPS_THERMAL_NORMAL;
	data->tswchg_level = PPS_THERMAL_NORMAL;
	data->run_once = true;
	data->state = PPS_ALGO_INIT;
	mutex_lock(&data->notify_lock);
	data->notify = 0;
	mutex_unlock(&data->notify_lock);
	data->stime = ktime_get_boottime();
}

static int pps_earily_restart(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	ret = pps_enable_dvchg_charging(info, PPS_DVCHG_SLAVE, false);
	if (ret < 0) {
		PPS_ERR("disable slave dvchg fail(%d)\n", ret);
		return ret;
	}
	ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER, false);
	if (ret < 0) {
		PPS_ERR("disable master dvchg fail(%d)\n", ret);
		return ret;
	}
	if (auth_data->support_cc) {
		ret = pps_set_ta_cap_cc(info, PPS_VTA_INIT, PPS_ITA_INIT);
		if (ret < 0) {
			PPS_ERR("set ta cap fail(%d)\n", ret);
			return ret;
		}
	}
	ret = pps_enable_ta_charging(info, false, PPS_VTA_INIT,
				      PPS_ITA_INIT);
	if (ret < 0) {
		PPS_ERR("disable ta charging fail(%d)\n", ret);
		return ret;
	}
	pps_init_algo_data(info);
	return 0;
}

/*
 * Start pps timer and run algo
 * It cannot start algo again if algo has been started once before
 * Run once flag will be reset after plugging out TA
 */
static inline int pps_start(struct pps_algo_info *info)
{
	int ret, ibus, vbat, vbus, ita, i;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	ktime_t ktime = ktime_set(0, _MS_TO_NS(PPS_INIT_POLLING_INTERVAL));

	PPS_DBG("++\n");

	if (data->run_once) {
		PPS_ERR("already run PPS once\n");
		return -EINVAL;
	}

	data->idvchg_ss_init = desc->idvchg_ss_init;
	ret = pps_hal_set_aicr(info->alg, CHG1, 3000);
	if (ret < 0) {
		PPS_ERR("set aicr fail(%d)\n", ret);
		goto start;
	}
	ret = pps_hal_set_ichg(info->alg, CHG1, 3000);
	if (ret < 0) {
		PPS_ERR("set ichg fail(%d)\n", ret);
		goto start;
	}
	ret = pps_hal_enable_charging(info->alg, CHG1, true);
	if (ret < 0) {
		PPS_ERR("en swchg fail(%d)\n", ret);
		goto start;
	}
	/*
			msleep(1000);
	*/
	msleep(50);
	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VBUS, &vbus);
	if (ret < 0) {
		PPS_ERR("get swchg vbus fail(%d)\n", ret);
		goto start;
	}
	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_IBUS, &ibus);
	if (ret < 0) {
		PPS_ERR("get swchg ibus fail(%d)\n", ret);
		goto start;
	}
	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get swchg vbat fail(%d)\n", ret);
		goto start;
	}
	ita = precise_div_2(percent_2(vbus * ibus, 90), 2 * vbat);
	/*
	if (ita < desc->idvchg_term) {
		PPS_ERR("estimated ita(%d) < idvchg_term(%d)\n", ita,
			desc->idvchg_term);
		return -EINVAL;
	}
	*/
	/* Update idvchg_ss_init */
	if (ita >= auth_data->ita_min) {
		PPS_INFO("set idvchg_ss_init(%d)->(%d)\n",
			  desc->idvchg_ss_init, ita);
		data->idvchg_ss_init = ita;
	}
start:
	/* disable charger */
	ret = pps_hal_enable_charging(info->alg, CHG1, false);
	if (ret < 0) {
		PPS_ERR("disable charger fail\n");
		return ret;
	}

	/*
			msleep(1000);
	*/
	msleep(50); /* wait for battery to recovery */

	/* Check DVCHG registers stat first */
	for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_exist[i])
			continue;
		ret = pps_hal_init_chip(info->alg, to_chgidx(i));
		if (ret < 0) {
			PPS_ERR("(%s) init chip fail(%d)\n",
				pps_dvchg_role_name[i], ret);
			return ret;
		}
	}

	/* Parameters that only reset by restarting from outside */
	mutex_lock(&data->ext_lock);
	data->input_current_limit = -1;
	data->cv_limit = -1;
	mutex_unlock(&data->ext_lock);
	data->tried_dual_dvchg = false;
	pps_init_algo_data(info);
	alarm_start_relative(&data->timer, ktime);
	return 0;
}

/* =================================================================== */
/* PPS Algo State Machine                                              */
/* =================================================================== */

static int pps_calculate_rcable_by_swchg(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	int vbus1 = 0, vbus2 = 0, vbus_max = 0, vbus_min = 0;
	int ibus1 = 0, ibus2 = 0, ibus_max = 0, ibus_min = 0;
	int ret = 0, aicr = 0, ichg = 0, i = 0;
	int val_vbus = 0, val_ibus = 0;

	ret = pps_hal_get_aicr(info->alg, CHG1, &aicr);
	if (ret < 0) {
		PPS_ERR("get aicr fail(%d)\n", ret);
		return ret;
	}

	ret = pps_hal_get_ichg(info->alg, CHG1, &ichg);
	if (ret < 0) {
		PPS_ERR("get ichg fail(%d)\n", ret);
		return ret;
	}

	ret = pps_hal_set_aicr(info->alg, CHG1, 300);
	if (ret < 0) {
		PPS_ERR("set aicr fail(%d)\n", ret);
		return ret;
	}

	ret = pps_hal_set_ichg(info->alg, CHG1, 3000);
	if (ret < 0) {
		PPS_ERR("set ichg fail(%d)\n", ret);
		return ret;
	}

	pps_hal_enable_charging(info->alg, CHG1, true);

	for (i = 0; i < PPS_MEASURE_R_AVG_TIMES + 2; i++) {
		ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VBUS, &val_vbus);
		if (ret < 0) {
			PPS_ERR("set aicr fail(%d)\n", ret);
			return ret;
		}
		ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_IBUS, &val_ibus);
		if (ret < 0) {
			PPS_ERR("set aicr fail(%d)\n", ret);
			return ret;
		}

		if (i == 0) {
			vbus_max = vbus_min = val_vbus;
			ibus_max = ibus_min = val_ibus;
		} else {
			vbus_max = max(vbus_max, val_vbus);
			ibus_max = max(ibus_max, val_ibus);
			vbus_min = min(vbus_min, val_vbus);
			ibus_min = min(ibus_min, val_ibus);
		}
		vbus1 += val_vbus;
		ibus1 += val_ibus;
		PPS_ERR("vbus=%d ibus=%d vbus(max,min)=(%d,%d) ibus(max,min)=(%d,%d) vbus1=%d ibus1=%d",
				val_vbus, val_ibus, vbus_max, vbus_min, ibus_max, ibus_min, vbus1, ibus1);
	}

	vbus1 -= (vbus_min + vbus_max);
	vbus1 = precise_div_2(vbus1, PPS_MEASURE_R_AVG_TIMES);

	ibus1 -= (ibus_min + ibus_max);
	ibus1 = precise_div_2(ibus1, PPS_MEASURE_R_AVG_TIMES);

	ret = pps_hal_set_aicr(info->alg, CHG1, 400);
	if (ret < 0) {
		PPS_ERR("set aicr fail(%d)\n", ret);
		return ret;
	}

	for (i = 0; i < PPS_MEASURE_R_AVG_TIMES + 2; i++) {
		ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VBUS, &val_vbus);
		if (ret < 0) {
			PPS_ERR("set aicr fail(%d)\n", ret);
			return ret;
		}
		ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_IBUS, &val_ibus);
		if (ret < 0) {
			PPS_ERR("set aicr fail(%d)\n", ret);
			return ret;
		}

		if (i == 0) {
			vbus_max = vbus_min = val_vbus;
			ibus_max = ibus_min = val_ibus;
		} else {
			vbus_max = max(vbus_max, val_vbus);
			ibus_max = max(ibus_max, val_ibus);
			vbus_min = min(vbus_min, val_vbus);
			ibus_min = min(ibus_min, val_ibus);
		}
		vbus2 += val_vbus;
		ibus2 += val_ibus;
		PPS_ERR("vbus=%d ibus=%d vbus(max,min)=(%d,%d) ibus(max,min)=(%d,%d) vbus2=%d ibus2=%d",
				val_vbus, val_ibus, vbus_max, vbus_min, ibus_max, ibus_min, vbus2, ibus2);
	}

	vbus2 -= (vbus_min + vbus_max);
	vbus2 = precise_div_2(vbus2, PPS_MEASURE_R_AVG_TIMES);

	ibus2 -= (ibus_min + ibus_max);
	ibus2 = precise_div_2(ibus2, PPS_MEASURE_R_AVG_TIMES);

	data->r_cable_by_swchg = precise_div_2(abs(vbus2 - vbus1) * 1000,
					     abs(ibus2 - ibus1));

	pps_hal_enable_charging(info->alg, CHG1, false);

	ret = pps_hal_set_aicr(info->alg, CHG1, aicr);
	if (ret < 0) {
		PPS_ERR("set aicr fail(%d)\n", ret);
		return ret;
	}

	ret = pps_hal_set_ichg(info->alg, CHG1, ichg);
	if (ret < 0) {
		PPS_ERR("set ichg fail(%d)\n", ret);
		return ret;
	}

	return 0;
}

static int pps_algo_init_with_ta_cc(struct pps_algo_info *info)
{
	int ret, i, vbus, vbat;
	int ita_avg = 0, vta_avg = 0, vbus_avg = 0, vbat_avg = 0;
	const int avg_times = 10;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_stop_info sinfo = {
		.hardreset_ta = false,
		.reset_ta = true,
	};

	PPS_DBG("++\n");

	/* Change charging policy first */
	ret = pps_enable_ta_charging(info, true, PPS_VTA_INIT, PPS_ITA_INIT);
	if (ret < 0) {
		PPS_ERR("enable ta charging fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}

	/* Check VBAT after disabling CHG_EN and before enabling HZ */
	for (i = 0; i < avg_times; i++) {
		ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
		if (ret < 0) {
			PPS_ERR("get vbus fail(%d)\n", ret);
			goto err;
		}
		vbat_avg += vbat;
	}
	vbat_avg = precise_div_2(vbat_avg, avg_times);
	data->zcv = vbat_avg;

	if (vbat_avg > desc->start_vbat_max) {
		PPS_INFO("finish PPS, vbat(%d) > %d\n", vbat_avg,
			 desc->start_vbat_max);
		goto out;
	}

	ret = pps_set_ta_cap_cv(info, 8000, 1000);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		goto err;
	}

	ret = pps_calculate_rcable_by_swchg(info);
	if (ret < 0) {
		PPS_ERR("calculate rcable by swchg fail(%d)\n", ret);
	}

	ret = pps_hal_enable_hz(info->alg, CHG1, true);
	if (ret < 0) {
		PPS_ERR("set swchg hz fail(%d)\n", ret);
		goto err;
	}
	/*
			msleep(500);
	*/
	msleep(50); /* Wait current stable  */

	/* Initial setting, no need to check ita_lmt */
	ret = pps_set_ta_cap_cc_by_cali_vta(info, data->idvchg_ss_init);
	if (ret < 0) {
		PPS_ERR("set ta cap by algo fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}

	for (i = 0; i < avg_times; i++) {
		ret = pps_get_ta_cap(info);
		if (ret < 0) {
			PPS_ERR("get ta cap fail(%d)\n", ret);
			sinfo.hardreset_ta = true;
			goto err;
		}
		ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
		if (ret < 0) {
			PPS_ERR("get vbus fail(%d)\n", ret);
			goto err;
		}
		ita_avg += data->ita_measure;
		vta_avg += data->vta_measure;
		vbus_avg += vbus;
	}
	ita_avg = precise_div_2(ita_avg, avg_times);
	vta_avg = precise_div_2(vta_avg, avg_times);
	vbus_avg = precise_div_2(vbus_avg, avg_times);

	/* vbus calibration: voltage difference between TA & device */
	data->vbus_cali = vta_avg - vbus_avg;
	PPS_INFO("avg(ita,vta,vbus,vbat):(%d, %d, %d, %d), vbus cali:%d\n",
		  ita_avg, vta_avg, vbus_avg, vbat_avg, data->vbus_cali);
	if (abs(data->vbus_cali) > PPS_VBUS_CALI_THRESHOLD) {
		PPS_ERR("vbus cali (%d) > (%d)\n", data->vbus_cali,
			 PPS_VBUS_CALI_THRESHOLD);
		goto err;
	}
	if (ita_avg > desc->ifod_threshold) {
		PPS_ERR("foreign object detected, ita(%d) > (%d)\n",
			 ita_avg, desc->ifod_threshold);
		goto err;
	}

	ret = pps_set_dvchg_charging(info, true);
	if (ret < 0) {
		PPS_ERR("en dvchg fail\n");
		goto err;
	}

	ret = pps_set_ta_cap_cc_by_cali_vta(info, data->idvchg_ss_init);
	if (ret < 0) {
		PPS_ERR("set ta cap by algo fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}
	data->err_retry_cnt = 0;
	data->state = PPS_ALGO_MEASURE_R;
	return 0;

err:
	if (data->err_retry_cnt < PPS_INIT_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}
out:
	return pps_stop(info, &sinfo);
}

static int pps_algo_init_with_ta_cv(struct pps_algo_info *info)
{
	int ret, i, vbus, vbat, vout;
	int ita_avg = 0, vta_avg = 0, vbus_avg = 0, vbat_avg = 0;
	bool err;
	u32 vta;
	const int avg_times = 10;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	/* Change charging policy first */
	ret = pps_enable_ta_charging(info, true, PPS_VTA_INIT, PPS_ITA_INIT);
	if (ret < 0) {
		PPS_ERR("enable ta charge fail(%d)\n", ret);
		ret = pps_enable_ta_charging(info, true, 8000, 1000);
		if(ret < 0){
			sinfo.hardreset_ta = true;
			goto err;
		}
	}

	/* Check VBAT after disabling CHG_EN and before enabling HZ */
	for (i = 0; i < avg_times; i++) {
		ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
		if (ret < 0) {
			PPS_ERR("get vbus fail(%d)\n", ret);
			goto err;
		}
		vbat_avg += vbat;
	}
	vbat_avg = precise_div_2(vbat_avg, avg_times);
	data->zcv = vbat_avg;
	PPS_INFO("avg(vbat):(%d)\n", vbat_avg);

	if (vbat_avg >= desc->start_vbat_max) {
		PPS_INFO("finish PPS, vbat(%d) > %d\n", vbat_avg,
			  desc->start_vbat_max);
		goto out;
	}
	ret = pps_set_ta_cap_cv(info, 8000, 1000);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		goto err;
	}

	ret = pps_calculate_rcable_by_swchg(info);
	if (ret < 0) {
		PPS_ERR("calculate rcable by swchg fail(%d)\n", ret);
	}

	ret = pps_hal_enable_hz(info->alg, CHG1, true);
	if (ret < 0) {
		PPS_ERR("set swchg hz fail(%d)\n", ret);
		goto err;
	}
	/*
			msleep(500);
	*/
	msleep(50); /* Wait current stable  */

	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
	if (ret < 0) {
		PPS_ERR("get vbus fail(%d)\n", ret);
		goto err;
	}
	ret = pps_hal_sync_ta_volt(info->alg, vbus);
	if (ret < 0 && ret != -EOPNOTSUPP) {
		PPS_ERR("sync ta setting fail(%d)\n", ret);
		goto err;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VOUT, &vout);
	if (ret < 0) {
		PPS_ERR("get vout fail(%d)\n", ret);
		goto err;
	}

	/* Adjust VBUS to make sure DVCHG can be turned on */
	vta = pps_vout2vbus(info, vout);
	ret = pps_set_ta_cap_cv(info, vta, data->idvchg_ss_init);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		goto err;
	}
	while (true) {
		ret = pps_hal_is_vbuslowerr(info->alg, DVCHG1, &err);
		if (ret < 0) {
			PPS_ERR("get vbuslowerr fail(%d)\n", ret);
			goto err;
		}
		if (!err)
			break;
		vta = data->vta_setting + auth_data->vta_step;
		ret = pps_set_ta_cap_cv(info, vta, data->idvchg_ss_init);
		if (ret < 0) {
			PPS_ERR("set ta cap fail(%d)\n", ret);
			goto err;
		}
	}

	for (i = 0; i < avg_times; i++) {
		if (auth_data->support_meas_cap) {
			ret = pps_get_ta_cap(info);
			if (ret < 0) {
				PPS_ERR("get ta cap fail(%d)\n", ret);
				sinfo.hardreset_ta = true;
				goto err;
			}
			ita_avg += data->ita_measure;
			vta_avg += data->vta_measure;
			ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
			if (ret < 0) {
				PPS_ERR("get vbus fail(%d)\n", ret);
				goto err;
			}
			vbus_avg += vbus;
		}
	}
	if (auth_data->support_meas_cap) {
		ita_avg = precise_div_2(ita_avg, avg_times);
		vta_avg = precise_div_2(vta_avg, avg_times);
		vbus_avg = precise_div_2(vbus_avg, avg_times);
	}

	if (auth_data->support_meas_cap) {
		/* vbus calibration: voltage difference between TA & device */
		data->vbus_cali = vta_avg - vbus_avg;
		PPS_INFO("avg(ita,vta,vbus):(%d,%d,%d), vbus_cali:%d\n",
			  ita_avg, vta_avg, vbus_avg, data->vbus_cali);
		if (abs(data->vbus_cali) > PPS_VBUS_CALI_THRESHOLD) {
			PPS_ERR("vbus cali (%d) > (%d)\n", data->vbus_cali,
				 PPS_VBUS_CALI_THRESHOLD);
			goto err;
		}
		if (ita_avg > desc->ifod_threshold) {
			PPS_ERR("foreign object detected, ita(%d) > (%d)\n",
				 ita_avg, desc->ifod_threshold);
			goto err;
		}
	}

	ret = pps_set_dvchg_charging(info, true);
	if (ret < 0) {
		PPS_ERR("en dvchg fail\n");
		goto err;
	}

	/* Get ita measure after enable dvchg */
	ret = pps_get_ta_cap_by_supportive(info, &data->vta_measure,
					    &data->ita_measure);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = auth_data->support_meas_cap;
		goto out;
	}
	data->err_retry_cnt = 0;
	data->state = PPS_ALGO_MEASURE_R;
	return 0;
err:
	if (data->err_retry_cnt < PPS_INIT_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}
out:
	data->waiver = true;
	return pps_stop(info, &sinfo);
}

/*
 * PPS algorithm initial state
 * It does Foreign Object Detection(FOD)
 */
static int pps_algo_init(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	return auth_data->support_cc ? pps_algo_init_with_ta_cc(info) :
				       pps_algo_init_with_ta_cv(info);
}

struct meas_r_info_t {
	u32 vbus;
	u32 ibus;
	u32 vbat;
	u32 ibat;
	u32 vout;
	u32 vta;
	u32 ita;
	u32 r_cable;
	u32 r_bat;
	u32 r_sw;
};

static int pps_algo_get_r_info(struct pps_algo_info *info,
				struct meas_r_info_t *r_info,
				struct pps_stop_info *sinfo)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	memset(r_info, 0, sizeof(struct meas_r_info_t));
	if (auth_data->support_meas_cap) {
		ret = pps_get_ta_cap(info);
		if (ret < 0) {
			PPS_ERR("get ta cap fail(%d)\n", ret);
			sinfo->hardreset_ta = true;
			return ret;
		}
		r_info->ita = data->ita_measure;
		r_info->vta = data->vta_measure;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &r_info->vbus);
	if (ret < 0) {
		PPS_ERR("get vbus fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_IBUS, &r_info->ibus);
	if (ret < 0) {
		PPS_ERR("get ibus fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VOUT, &r_info->vout);
	if (ret < 0) {
		PPS_ERR("get vout fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &r_info->vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_IBAT, &r_info->ibat);
	if (ret < 0) {
		PPS_ERR("get ibat fail(%d)\n", ret);
		return ret;
	}
	PPS_DBG("vta:%d,ita:%d,vbus:%d,ibus:%d,vout:%d,vbat:%d,ibat:%d\n",
		 r_info->vta, r_info->ita, r_info->vbus, r_info->ibus,
		 r_info->vout, r_info->vbat, r_info->ibat);
	return 0;
}

static int pps_algo_cal_r_info_with_ta_cap(struct pps_algo_info *info,
					    struct pps_stop_info *sinfo)
{
	int ret, i;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct meas_r_info_t r_info, max_r_info, min_r_info;
	struct pps_stop_info _sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	memset(&max_r_info, 0, sizeof(struct meas_r_info_t));
	memset(&min_r_info, 0, sizeof(struct meas_r_info_t));
	data->r_bat = data->r_sw = data->r_cable = 0;
	for (i = 0; i < PPS_MEASURE_R_AVG_TIMES + 2; i++) {
		if (atomic_read(&data->stop_algo)) {
			PPS_INFO("stop algo\n");
			goto stop;
		}
		ret = pps_algo_get_r_info(info, &r_info, sinfo);
		if (ret < 0) {
			PPS_ERR("get r info fail(%d)\n", ret);
			return ret;
		}
		if (r_info.ibat == 0) {
			PPS_ERR("ibat == 0 fail\n");
			return -EINVAL;
		}
		if (r_info.ita == 0) {
			PPS_ERR("ita == 0 fail\n");
			sinfo->hardreset_ta = true;
			data->waiver = true;
			return -EINVAL;
		}
		if (r_info.ita < data->idvchg_term &&
		    r_info.vbat >= data->vbat_cv) {
			PPS_INFO("finish PPS charging\n");
			return -EINVAL;
		}else if (r_info.ita < data->idvchg_term) {
			PPS_ERR("ita < idvchg_term fail\n");
			sinfo->hardreset_ta = true;
			data->waiver = true;
			return -EINVAL;
		}

		/* Use absolute instead of relative calculation */
		r_info.r_bat = precise_div_2(abs(r_info.vbat - data->zcv) * 1000,
					   abs(r_info.ibat));
		if (r_info.r_bat > desc->ircmp_rbat)
			r_info.r_bat = desc->ircmp_rbat;

		r_info.r_sw = precise_div_2(abs(r_info.vout - r_info.vbat) * 1000,
					  abs(r_info.ibat));
		if (r_info.r_sw < desc->rsw_min)
			r_info.r_sw = desc->rsw_min;

		r_info.r_cable = precise_div_2(abs(r_info.vta - data->vbus_cali -
					     r_info.vbus) * 1000,
					     abs(r_info.ita));

		PPS_INFO("r_sw:%d, r_bat:%d, r_cable:%d\n", r_info.r_sw,
			  r_info.r_bat, r_info.r_cable);

		if (i == 0) {
			memcpy(&max_r_info, &r_info,
			       sizeof(struct meas_r_info_t));
			memcpy(&min_r_info, &r_info,
			       sizeof(struct meas_r_info_t));
		} else {
			max_r_info.r_bat = max(max_r_info.r_bat, r_info.r_bat);
			max_r_info.r_sw = max(max_r_info.r_sw, r_info.r_sw);
			max_r_info.r_cable = max(max_r_info.r_cable,
						 r_info.r_cable);
			min_r_info.r_bat = min(min_r_info.r_bat, r_info.r_bat);
			min_r_info.r_sw = min(min_r_info.r_sw, r_info.r_sw);
			min_r_info.r_cable = min(min_r_info.r_cable,
						 r_info.r_cable);
		}
		data->r_bat += r_info.r_bat;
		data->r_sw += r_info.r_sw;
		data->r_cable += r_info.r_cable;
	}
	data->r_bat -= (max_r_info.r_bat + min_r_info.r_bat);
	data->r_sw -= (max_r_info.r_sw + min_r_info.r_sw);
	data->r_cable -= (max_r_info.r_cable + min_r_info.r_cable);
	data->r_bat = precise_div_2(data->r_bat, PPS_MEASURE_R_AVG_TIMES);
	data->r_sw = precise_div_2(data->r_sw, PPS_MEASURE_R_AVG_TIMES);
	data->r_cable = precise_div_2(data->r_cable,
				    PPS_MEASURE_R_AVG_TIMES);
	data->r_total = data->r_bat + data->r_sw + data->r_cable;
	return 0;
stop:
	pps_stop(info, &_sinfo);
	return -EIO;
}

static int pps_select_ita_lmt_by_r(struct pps_algo_info *info, bool dual)
{
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 ita_lmt_by_r, ita_lmt;
	u32 *rcable_level = dual ? desc->rcable_level_dual : desc->rcable_level;
	u32 *ita_level = dual ? desc->ita_level_dual : desc->ita_level;

	if (!auth_data->support_meas_cap) {
		ita_lmt_by_r = ita_level[PPS_RCABLE_NORMAL];
		goto out;
	}
	if (data->r_cable_by_swchg <= rcable_level[PPS_RCABLE_NORMAL])
		ita_lmt_by_r = ita_level[PPS_RCABLE_NORMAL];
	else if (data->r_cable_by_swchg <= rcable_level[PPS_RCABLE_BAD1])
		ita_lmt_by_r = ita_level[PPS_RCABLE_BAD1];
	else if (data->r_cable_by_swchg <= rcable_level[PPS_RCABLE_BAD2])
		ita_lmt_by_r = ita_level[PPS_RCABLE_BAD2];
	else if (data->r_cable_by_swchg <= rcable_level[PPS_RCABLE_BAD3])
		ita_lmt_by_r = ita_level[PPS_RCABLE_BAD3];
	else {
		PPS_ERR("r_cable_by_swchg(%d) too worse\n", data->r_cable_by_swchg);
		PPS_ERR("r_cable(%d) too worse\n", data->r_cable);
		return -EINVAL;
	}
	PPS_ERR("r_cable_by_swchg: %d\n", data->r_cable_by_swchg);
	PPS_ERR("r_cable: %d\n", data->r_cable);
out:
	PPS_INFO("ita limited by r = %d\n", ita_lmt_by_r);
	data->ita_lmt = min(ita_lmt_by_r, (u32)auth_data->ita_max);
	data->ita_pwr_lmt = pps_get_ita_pwr_lmt_by_vta(info,
							data->vta_setting);
	ita_lmt = pps_get_ita_lmt(info);
	if (ita_lmt < data->idvchg_term) {
		PPS_ERR("ita_lmt(%d) < dvchg_term(%d)\n", ita_lmt,
			 data->idvchg_term);
		return -EINVAL;
	}
	return 0;
}

static int pps_algo_measure_r_with_ta_cc(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 ita, idvchg_lmt;
	u32 rcable_retry_level = (data->is_dvchg_exist[PPS_DVCHG_SLAVE] &&
				  !data->tried_dual_dvchg) ?
				  desc->rcable_level_dual[PPS_RCABLE_NORMAL] :
				  desc->rcable_level[PPS_RCABLE_NORMAL];
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	ret = pps_algo_cal_r_info_with_ta_cap(info, &sinfo);
	if (ret < 0)
		goto err;
	if (data->r_cable > rcable_retry_level &&
	    data->err_retry_cnt < PPS_MEASURE_R_RETRY_MAX) {
		PPS_INFO("rcable(%d) is worse than normal\n", data->r_cable);
		goto err;
	}
	PPS_ERR("avg_r(sw,bat,cable):(%d,%d,%d), r_total:%d\n",
		 data->r_sw, data->r_bat, data->r_cable, data->r_total);

	/* If haven't tried dual dvchg, try it once */
	if (data->is_dvchg_exist[PPS_DVCHG_SLAVE] && !data->tried_dual_dvchg &&
	    !data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
		PPS_INFO("try dual dvchg\n");
		data->tried_dual_dvchg = true;
		data->idvchg_term = 2 * desc->idvchg_term;
		data->idvchg_cc = desc->ita_level_dual[PPS_RCABLE_NORMAL] -
				  desc->swchg_aicr;
		ret = pps_select_ita_lmt_by_r(info, true);
		if (ret < 0) {
			PPS_ERR("select dual dvchg ita lmt fail(%d)\n", ret);
			goto single_dvchg_select_ita;
		}
		/* Turn on slave dvchg if idvchg_lmt >= 2 * idvchg_term */
		idvchg_lmt = pps_get_idvchg_lmt(info);
		if (idvchg_lmt < data->idvchg_term) {
			PPS_ERR("idvchg_lmt(%d) < 2 * idvchg_term(%d)\n",
				 idvchg_lmt, data->idvchg_term);
			goto single_dvchg_select_ita;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER,
						 false);
		if (ret < 0) {
			PPS_ERR("disable master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		data->ignore_ibusucpf = true;
		ret = pps_set_operating_mode(info->alg,DVCHG1,true);
		if (ret < 0) {
			PPS_ERR("set operating mode fail(%d)\n", ret);
			return ret;
		}
		ret = pps_set_operating_mode(info->alg,DVCHG2,true);
		if (ret < 0) {
			PPS_ERR("set operating mode fail(%d)\n", ret);
			return ret;
		}
		ret = pps_set_dvchg_protection(info, true);
		if (ret < 0) {
			PPS_ERR("set dual dvchg protection fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_SLAVE, true);
		if (ret < 0) {
			PPS_ERR("en slave dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ita = max(data->idvchg_term, data->ita_setting);
		ita = min(ita, idvchg_lmt);
		ret = pps_set_ta_cap_cc_by_cali_vta(info, ita);
		if (ret < 0) {
			PPS_ERR("set ta cap fail(%d)\n", ret);
			sinfo.hardreset_ta = true;
			goto out;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER, true);
		if (ret < 0) {
			PPS_ERR("en master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		goto ss_dvchg;
single_dvchg_restart:
		ret = pps_earily_restart(info);
		if (ret < 0) {
			PPS_ERR("earily restart fail(%d)\n", ret);
			goto out;
		}
		return 0;
	}
single_dvchg_select_ita:
	data->idvchg_term = desc->idvchg_term;
	data->idvchg_cc = desc->ita_level[PPS_RCABLE_NORMAL] -
			  desc->swchg_aicr;
	ret = pps_select_ita_lmt_by_r(info, false);
	if (ret < 0) {
		PPS_ERR("select dvchg ita lmt fail(%d)\n", ret);
		goto out;
	}
ss_dvchg:
	data->err_retry_cnt = 0;
	data->state = PPS_ALGO_SS_DVCHG;
	return 0;
err:
	if (data->err_retry_cnt < PPS_MEASURE_R_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}
out:
	return pps_stop(info, &sinfo);
}

static int pps_algo_measure_r_with_ta_cv(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 rcable_retry_level = (data->is_dvchg_exist[PPS_DVCHG_SLAVE] &&
				  !data->tried_dual_dvchg) ?
				  desc->rcable_level_dual[PPS_RCABLE_NORMAL] :
				  desc->rcable_level[PPS_RCABLE_NORMAL];
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	/*
	 * Ignore measuring r,
	 * treat as normal cable if meas_cap is not supported
	 */
	if (!auth_data->support_meas_cap) {
		PPS_INFO("ignore measuring resistance\n");
		goto select_ita;
	}
	ret = pps_algo_cal_r_info_with_ta_cap(info, &sinfo);
	if (ret < 0) {
		PPS_ERR("get r info fail(%d)\n", ret);
		goto err;
	}
	if (data->r_cable > rcable_retry_level &&
	    data->err_retry_cnt < PPS_MEASURE_R_RETRY_MAX) {
		PPS_INFO("rcable(%d) is worse than normal\n", data->r_cable);
		goto err;
	}
	PPS_ERR("avg_r(sw,bat,cable):(%d,%d,%d), r_total:%d\n",
		 data->r_sw, data->r_bat, data->r_cable, data->r_total);
select_ita:
	ret = pps_select_ita_lmt_by_r(info, false);
	if (ret < 0) {
		PPS_ERR("select dvchg ita lmt fail(%d)\n", ret);
		goto out;
	}
	data->err_retry_cnt = 0;
	data->state = PPS_ALGO_SS_DVCHG;
	return 0;
err:
	if (data->err_retry_cnt < PPS_MEASURE_R_RETRY_MAX) {
		data->err_retry_cnt++;
		return 0;
	}
out:
	data->waiver = true;
	return pps_stop(info, &sinfo);
}

/* Measure resistance of cable/battery/sw and get corressponding ita limit */
static int pps_algo_measure_r(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	return (auth_data->support_cc && !data->force_ta_cv) ?
	       pps_algo_measure_r_with_ta_cc(info) :
	       pps_algo_measure_r_with_ta_cv(info);
}

static int pps_check_slave_dvchg_off(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;

	data->idvchg_cc = desc->ita_level[PPS_RCABLE_NORMAL] -
			  desc->swchg_aicr;
	data->idvchg_term = desc->idvchg_term;
	ret = pps_enable_dvchg_charging(info, PPS_DVCHG_SLAVE, false);
	if (ret < 0) {
		PPS_ERR("disable slave dvchg fail(%d)\n", ret);
		return ret;
	}
	ret = pps_select_ita_lmt_by_r(info, false);
	if (ret < 0) {
		PPS_ERR("select dvchg ita lmt fail(%d)\n", ret);
		return ret;
	}
	ret = pps_set_dvchg_protection(info, false);
	if (ret < 0) {
		PPS_ERR("dvchg protection fail(%d)\n", ret);
		return ret;
	}
	return 0;
}

static int pps_force_ta_cv(struct pps_algo_info *info,
			    struct pps_stop_info *sinfo)
{
	int ret;
	u32 ita, vta;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	PPS_DBG("++\n");
	ret = pps_hal_set_vbusovp_alarm(info->alg, DVCHG1, data->vbusovp);
	if (ret < 0) {
		PPS_ERR("set vbusovp alarm fail(%d)\n", ret);
		return ret;
	}
	ret = pps_get_ta_cap(info);
	if (ret < 0) {
		PPS_ERR("get ta cap fail\n");
		sinfo->hardreset_ta = true;
		return ret;
	}
	ita = min(data->ita_measure, data->ita_setting);
	vta = min(data->vta_measure, (u32)auth_data->vcap_max);
	ret = pps_set_ta_cap_cv(info, vta, ita);
	if (ret < 0) {
		PPS_ERR("set ta cap fail\n");
		return ret;
	}
	data->force_ta_cv = true;
	return 0;
}

static int pps_check_force_ta_cv(struct pps_algo_info *info,
				  struct pps_stop_info *sinfo)
{
	int ret;
	u32 vbat;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}

	if (desc->force_ta_cv_vbat != 0 && vbat >= desc->force_ta_cv_vbat &&
	    !data->is_swchg_en) {
		ret = pps_force_ta_cv(info, sinfo);
		if (ret < 0) {
			PPS_ERR("force ta cv fail(%d)\n", ret);
			return ret;
		}
	}
	return 0;
}

static int pps_algo_ss_dvchg_with_ta_cc(struct pps_algo_info *info)
{
	int ret, vbat;
	u32 ita, ita_lmt, idvchg_lmt;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	ret = pps_check_force_ta_cv(info, &sinfo);
	if (ret < 0) {
		PPS_ERR("check force ta cv fail(%d)\n", ret);
		goto err;
	}
	if (data->force_ta_cv) {
		PPS_INFO("force switching to ta cv mode\n");
		return 0;
	}

	ret = pps_get_ta_cap(info);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		goto err;
	}

	idvchg_lmt = pps_get_idvchg_lmt(info);
	if (idvchg_lmt < data->idvchg_term) {
		PPS_INFO("idvchg_lmt(%d) < idvchg_term(%d)\n", idvchg_lmt,
			  data->idvchg_term);
		goto err;
	}

	/* VBAT reaches CV level */
	if (vbat >= data->vbat_cv) {
		if (data->ita_measure < data->idvchg_term) {
			if (data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
				ret = pps_check_slave_dvchg_off(info);
				if (ret < 0) {
					PPS_INFO("slave off fail(%d)\n", ret);
					goto err;
				}
				idvchg_lmt = pps_get_idvchg_lmt(info);
				goto cc_cv;
			}
			PPS_INFO("finish PPS charging, vbat(%d), ita(%d)\n",
				  vbat, data->ita_measure);
			goto err;
		}
cc_cv:
		ita = min(data->ita_setting - desc->idvchg_ss_step, idvchg_lmt);
		data->state = PPS_ALGO_CC_CV;
		goto out_set_cap;
	}

	/* ITA reaches CC level */
	if (data->ita_measure >= idvchg_lmt ||
	    data->ita_setting >= idvchg_lmt) {
		/*
		 * Turn on switching charger only if divider charger
		 * is charging by it's maximum setting current
		 */
		ita_lmt = pps_get_ita_lmt(info);
		if (vbat < desc->swchg_off_vbat && desc->swchg_aicr > 0 &&
		    desc->swchg_ichg > 0 &&
		    (ita_lmt > data->idvchg_cc + desc->swchg_aicr_ss_init)) {
			ret = pps_set_swchg_cap(info,
						 desc->swchg_aicr_ss_init);
			if (ret < 0) {
				PPS_ERR("set swchg cap fail(%d)\n", ret);
				goto err;
			}
			ret = pps_enable_swchg_charging(info, true);
			if (ret < 0) {
				PPS_ERR("en swchg fail(%d)\n", ret);
				goto err;
			}
			ita = idvchg_lmt + desc->swchg_aicr_ss_init;
			data->aicr_init_lmt = ita_lmt - data->idvchg_cc;
			data->aicr_lmt = data->aicr_init_lmt;
			data->state = PPS_ALGO_SS_SWCHG;
			goto out_set_cap;
		}
		data->state = PPS_ALGO_CC_CV;
		ita = idvchg_lmt;
		goto out_set_cap;
	}

	/* Increase ita according to vbat level */
	if (vbat < desc->idvchg_ss_step1_vbat)
		ita = data->ita_setting + desc->idvchg_ss_step;
	else if (vbat < desc->idvchg_ss_step2_vbat)
		ita = data->ita_setting + desc->idvchg_ss_step1;
	else
		ita = data->ita_setting + desc->idvchg_ss_step2;
	ita = min(ita, idvchg_lmt);

out_set_cap:
	ret = pps_set_ta_cap_cc_by_cali_vta(info, ita);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}
	return 0;
err:
	return pps_stop(info, &sinfo);
}

static int pps_algo_ss_dvchg_with_ta_cv(struct pps_algo_info *info)
{
	int ret, vbat;
	ktime_t start_time, end_time;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 idvchg_lmt, vta, ita, delta_time;
	u32 ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
				data->ita_gap_per_vstep :
				auth_data->ita_gap_per_vstep;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

repeat:
	PPS_DBG("++\n");
	vta = data->vta_setting;
	start_time = ktime_get();
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		goto out;
	}
	ret = pps_get_ta_cap_by_supportive(info, &data->vta_measure,
					    &data->ita_measure);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = auth_data->support_meas_cap;
		goto out;
	}

	/* Turn on slave dvchg if idvchg_lmt >= 2 * idvchg_term */
	ita = data->idvchg_term * 2;
	if (data->is_dvchg_exist[PPS_DVCHG_SLAVE] && !data->tried_dual_dvchg &&
	    !data->is_dvchg_en[PPS_DVCHG_SLAVE] &&
	    (data->ita_measure >= ita)) {
		PPS_INFO("try dual dvchg\n");
		data->tried_dual_dvchg = true;
		data->idvchg_term = 2 * desc->idvchg_term;
		data->idvchg_cc = desc->ita_level_dual[PPS_RCABLE_NORMAL] -
				  desc->swchg_aicr;
		ret = pps_select_ita_lmt_by_r(info, true);
		if (ret < 0) {
			PPS_ERR("select dual dvchg ita lmt fail(%d)\n", ret);
			goto single_dvchg_select_ita;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER,
						 false);
		if (ret < 0) {
			PPS_ERR("disable master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ret = pps_set_operating_mode(info->alg,DVCHG2,true);
		if (ret < 0) {
			PPS_ERR("set operating mode fail(%d)\n", ret);
			return ret;
		}
		ret = pps_set_operating_mode(info->alg,DVCHG1,true);
		if (ret < 0) {
			PPS_ERR("set operating mode fail(%d)\n", ret);
			return ret;
		}
		data->ignore_ibusucpf = true;
		ret = pps_set_dvchg_protection(info, true);
		if (ret < 0) {
			PPS_ERR("set dual dvchg protection fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_SLAVE, true);
		if (ret < 0) {
			PPS_ERR("en slave dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		ret = pps_enable_dvchg_charging(info, PPS_DVCHG_MASTER, true);
		if (ret < 0) {
			PPS_ERR("en master dvchg fail(%d)\n", ret);
			goto single_dvchg_restart;
		}
		goto ss_dvchg;
single_dvchg_restart:
		ret = pps_earily_restart(info);
		if (ret < 0) {
			PPS_ERR("earily restart fail(%d)\n", ret);
			goto out;
		}
		return 0;
single_dvchg_select_ita:
		data->idvchg_term = desc->idvchg_term;
		data->idvchg_cc = desc->ita_level[PPS_RCABLE_NORMAL] -
				  desc->swchg_aicr;
		ret = pps_select_ita_lmt_by_r(info, false);
		if (ret < 0) {
			PPS_ERR("select dvchg ita lmt fail(%d)\n", ret);
			goto out;
		}
	}

ss_dvchg:
	ita = data->ita_setting;
	idvchg_lmt = pps_get_idvchg_lmt(info);
	if (idvchg_lmt < data->idvchg_term) {
		PPS_INFO("idvchg_lmt(%d) < idvchg_term(%d)\n", idvchg_lmt,
			 data->idvchg_term);
		goto out;
	}

	/* VBAT reaches CV level */
	if (vbat >= data->vbat_cv) {
		if (data->ita_measure < data->idvchg_term) {
			if (data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
				ret = pps_check_slave_dvchg_off(info);
				if (ret < 0) {
					PPS_INFO("slave off fail(%d)\n", ret);
					goto out;
				}
				idvchg_lmt = pps_get_idvchg_lmt(info);
				goto cc_cv;
			}
			PPS_INFO("finish PPS charging, vbat(%d), ita(%d)\n",
				  vbat, data->ita_measure);
			goto out;
		}
cc_cv:
		vta -= auth_data->vta_step;
		ita -= ita_gap_per_vstep;
		data->state = PPS_ALGO_CC_CV;
		goto out_set_cap;
	}

	/* IBUS reaches CC level */
	if (data->ita_measure + ita_gap_per_vstep > idvchg_lmt ||
	    vta == auth_data->vcap_max)
		data->state = PPS_ALGO_CC_CV;
	else {
		vta += auth_data->vta_step;
		vta = min(vta, (u32)auth_data->vcap_max);
		ita += ita_gap_per_vstep;
		ita = min(ita, idvchg_lmt);
	}

out_set_cap:
	ret = pps_set_ta_cap_cv(info, vta, ita);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}
	if (data->state == PPS_ALGO_SS_DVCHG) {
		msleep(desc->ta_cv_ss_repeat_tmin);
		end_time = ktime_get();
		delta_time = ktime_ms_delta(end_time, start_time);
		PPS_INFO("delta time %dms\n", delta_time);
		/*
		if (delta_time < desc->ta_cv_ss_repeat_tmin)
			msleep(desc->ta_cv_ss_repeat_tmin - delta_time);
		*/
		goto repeat;
	}
	return 0;
out:
	return pps_stop(info, &sinfo);
}

/* Soft start of divider charger */
static int pps_algo_ss_dvchg(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	return (auth_data->support_cc && !data->force_ta_cv) ?
		pps_algo_ss_dvchg_with_ta_cc(info) :
		pps_algo_ss_dvchg_with_ta_cv(info);
}

static int pps_check_swchg_off(struct pps_algo_info *info,
				struct pps_stop_info *sinfo)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 aicr = data->aicr_setting, ita, vbat;

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return ret;
	}
	if (!data->is_swchg_en || (vbat < desc->swchg_off_vbat &&
	    desc->tswchg_curlmt[data->tswchg_level] <= 0 &&
	    aicr <= data->aicr_lmt))
		return 0;
	PPS_INFO("vbat(%d),swchg_off_vbat(%d),aicr_lmt(%d)\n", vbat,
		 desc->swchg_off_vbat, data->aicr_lmt);
	/* Calculate AICR */
	if (vbat >= desc->swchg_off_vbat)
		aicr -= desc->swchg_aicr_ss_step;
	aicr = min(aicr, data->aicr_lmt);
	/* Calculate ITA */
	if (aicr >= desc->swchg_aicr_ss_init)
		ita = data->ita_setting - (data->aicr_setting - aicr);
	else
		ita = data->ita_setting - data->aicr_setting;
	ret = pps_set_ta_cap_cc_by_cali_vta(info, ita);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		sinfo->hardreset_ta = true;
		return ret;
	}
	return (aicr >= desc->swchg_aicr_ss_init) ?
		pps_set_swchg_cap(info, aicr) :
		pps_enable_swchg_charging(info, false);
}

static int pps_update_aicr_lmt(struct pps_algo_info *info)
{
	struct pps_algo_desc *desc = info->desc;
	struct pps_algo_data *data = info->data;
	u32 ita_lmt;

	if (!data->is_swchg_en)
		return 0;
	ita_lmt = pps_get_ita_lmt(info);
	/* Turn off swchg and use dvchg only */
	if (ita_lmt < data->idvchg_cc + desc->swchg_aicr_ss_init) {
		data->aicr_lmt = 0;
		return -EINVAL;
	}
	data->aicr_lmt = min(data->aicr_lmt, ita_lmt - data->idvchg_cc);
	if (data->aicr_lmt < desc->swchg_aicr_ss_init) {
		data->aicr_lmt = 0;
		return -EINVAL;
	}
	return 0;
}

static int pps_algo_ss_swchg(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_desc *desc = info->desc;
	struct pps_algo_data *data = info->data;
	u32 ita = data->ita_setting, aicr = 0, vbat;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	if (pps_update_aicr_lmt(info) < 0)
		goto out;
	/* Set new AICR & TA cap */
	aicr = min(data->aicr_setting + desc->swchg_aicr_ss_step,
		   data->aicr_lmt);
	ita += (aicr - data->aicr_setting);
	ret = pps_set_swchg_cap(info, aicr);
	if (ret < 0) {
		PPS_ERR("set swchg cap fail(%d)\n", ret);
		goto err;
	}
	ret = pps_set_ta_cap_cc_by_cali_vta(info, ita);
	if (ret < 0) {
		PPS_ERR("set ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}
out:
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		goto err;
	}
	ret = pps_check_swchg_off(info, &sinfo);
	if (ret < 0) {
		PPS_ERR("check swchg off fail(%d)\n", ret);
		goto err;
	}
	if (!data->is_swchg_en || vbat >= desc->swchg_off_vbat ||
	    aicr >= data->aicr_lmt)
		data->state = PPS_ALGO_CC_CV;
	return 0;
err:
	return pps_stop(info, &sinfo);
}

static int pps_algo_cc_cv_with_ta_cc(struct pps_algo_info *info)
{
	int ret, vbat = 0;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	u32 ita = data->ita_setting, ita_lmt;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	ret = pps_check_force_ta_cv(info, &sinfo);
	if (ret < 0) {
		PPS_ERR("check force ta cv fail(%d)\n", ret);
		goto err;
	}
	if (data->force_ta_cv) {
		PPS_INFO("force switching to ta cv mode\n");
		return 0;
	}
	/* Check swchg */
	pps_update_aicr_lmt(info);
	ret = pps_check_swchg_off(info, &sinfo);
	if (ret < 0) {
		PPS_ERR("check swchg off fail(%d)\n", ret);
		goto err;
	}

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0)
		PPS_ERR("get vbat fail(%d)\n", ret);

	ret = pps_get_ta_cap(info);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}

	if (data->ita_measure < data->idvchg_term) {
		if (data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
			ret = pps_check_slave_dvchg_off(info);
			if (ret < 0) {
				PPS_INFO("slave off fail(%d)\n", ret);
				goto err;
			}
			goto cc_cv;
		}
		PPS_INFO("finish PPS charging\n");
		goto err;
	}
cc_cv:
	ita_lmt = pps_get_ita_lmt(info);
	/* Consider AICR is decreased */
	ita_lmt = min(ita_lmt, data->is_swchg_en ?
		      (data->idvchg_cc + data->aicr_setting) : data->idvchg_cc);
	if (ita_lmt < data->idvchg_term) {
		PPS_INFO("ita_lmt(%d) < idvchg_term(%d)\n", ita_lmt,
			 data->idvchg_term);
		goto err;
	}

	if (vbat >= data->vbat_cv) {
		ita = data->ita_setting - desc->idvchg_step;
		data->is_vbat_over_cv = true;
	} else if (vbat < desc->idvchg_ss_step1_vbat && ita < ita_lmt) {
		PPS_INFO("++ita(set,lmt,add)=(%d,%d,%d)\n", ita, ita_lmt,
			  desc->idvchg_ss_step);
		ita = data->ita_setting + desc->idvchg_ss_step;
	} else if (vbat < desc->idvchg_ss_step2_vbat && ita < ita_lmt) {
		PPS_INFO("++ita(set,lmt,add)=(%d,%d,%d)\n", ita, ita_lmt,
			  desc->idvchg_ss_step1);
		ita = data->ita_setting + desc->idvchg_ss_step1;
	} else if (!data->is_vbat_over_cv &&
		   vbat <= data->cv_lower_bound && ita < ita_lmt) {
		PPS_INFO("++ita(set,lmt,add)=(%d,%d,%d)\n", ita, ita_lmt,
			  desc->idvchg_step);
		ita = data->ita_setting + desc->idvchg_step;
	} else if (data->is_vbat_over_cv)
		data->is_vbat_over_cv = false;

	ita = min(ita, ita_lmt);
	ret = pps_set_ta_cap_cc_by_cali_vta(info, ita);
	if (ret < 0) {
		PPS_ERR("set_ta_cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto err;
	}
	return 0;
err:
	return pps_stop(info, &sinfo);
}

static int pps_algo_cc_cv_with_ta_cv(struct pps_algo_info *info)
{
	int ret, vbat, vsys = 0;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 idvchg_lmt, vta = data->vta_setting, ita = data->ita_setting;
	u32 ita_gap_per_vstep = data->ita_gap_per_vstep > 0 ?
				data->ita_gap_per_vstep :
				auth_data->ita_gap_per_vstep;
	u32 vta_measure, ita_measure, suspect_ta_cc = false;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");

	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		goto out;
	}

	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VSYS,
				   &vsys);
	if (ret < 0) {
		PPS_ERR("get vsys fail(%d)\n", ret);
		goto out;
	}

	ret = pps_get_ta_cap_by_supportive(info, &data->vta_measure,
					    &data->ita_measure);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = auth_data->support_meas_cap;
		goto out;
	}
	if (data->ita_measure <= data->idvchg_term && vbat >= (data->vbat_cv - PPS_CV_DOWN_GAP)) {
		if (data->is_dvchg_en[PPS_DVCHG_SLAVE]) {
			ret = pps_check_slave_dvchg_off(info);
			if (ret < 0) {
				PPS_INFO("slave off fail(%d)\n", ret);
				goto out;
			}
			goto cc_cv;
		}
		pps_hal_set_ieoc(info->alg, CHG1,PPS_IEOC_CURR);
		data->finish = true;
		PPS_INFO("finish PPS charging\n");
		goto out;
	}
cc_cv:
	idvchg_lmt = pps_get_idvchg_lmt(info);
	if (idvchg_lmt < data->idvchg_term && vbat >= (data->vbat_cv - PPS_CV_DOWN_GAP)) {
		PPS_INFO("idvchg_lmt(%d) < idvchg_term(%d)\n", idvchg_lmt,
			  data->idvchg_term);
		goto out;
	}

	if (vbat >= data->vbat_cv + PPS_CV_UPPER_GAP) {
		PPS_INFO("--vbat >= vbat_cv, %d > %d\n", vbat, data->vbat_cv);
		vta -= auth_data->vta_step;
		ita -= ita_gap_per_vstep;
		data->is_vbat_over_cv = true;
	} else if (data->ita_measure > idvchg_lmt  || vsys >= PPS_VSYS_UPPER_BOUND) {
		vta -= auth_data->vta_step;
		ita -= ita_gap_per_vstep;
		ita = max(ita, idvchg_lmt);
		PPS_INFO("--vta, ita(meas,lmt)=(%d,%d)\n", data->ita_measure,
			  idvchg_lmt);
	} else if (!data->is_vbat_over_cv && vbat <= data->cv_lower_bound &&
		   data->ita_measure <= (idvchg_lmt - ita_gap_per_vstep) &&
		   vta < auth_data->vcap_max && !data->suspect_ta_cc &&
		   vsys < (PPS_VSYS_UPPER_BOUND - PPS_VSYS_UPPER_BOUND_GAP)) {
		vta += auth_data->vta_step;
		vta = min(vta, (u32)auth_data->vcap_max);
		ita += ita_gap_per_vstep;
		ita = min(ita, idvchg_lmt);
		if (ita == data->ita_setting)
			suspect_ta_cc = true;
		PPS_INFO("++vta, ita(meas,lmt)=(%d,%d),vcap_max=(%d),vsys=(%d)\n", data->ita_measure,
			  idvchg_lmt,auth_data->vcap_max,vsys);
	} else if (data->is_vbat_over_cv)
		data->is_vbat_over_cv = false;
	PPS_INFO("set (vta,ita)=(%d , %d)\n", vta, ita);
	ret = pps_set_ta_cap_cv(info, vta, ita);
	if (ret < 0) {
		PPS_ERR("set_ta_cap fail(%d)\n", ret);
		sinfo.hardreset_ta = true;
		goto out;
	}
	ret = pps_get_ta_cap_by_supportive(info, &vta_measure, &ita_measure);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		sinfo.hardreset_ta = auth_data->support_meas_cap;
		goto out;
	}
	data->suspect_ta_cc = (suspect_ta_cc &&
			       data->ita_measure == ita_measure);
	return 0;
out:
	return pps_stop(info, &sinfo);
}

static int pps_algo_cc_cv(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	return (auth_data->support_cc && !data->force_ta_cv) ?
		pps_algo_cc_cv_with_ta_cc(info) :
		pps_algo_cc_cv_with_ta_cv(info);
}

/*
 * Check TA's status
 * Get status from TA and check temperature, OCP, OTP, and OVP, etc...
 *
 * return true if TA is normal and false if it is abnormal
 */
static bool pps_check_ta_status(struct pps_algo_info *info,
				 struct pps_stop_info *sinfo)
{
	int ret;
	struct pps_ta_status status;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	if (!auth_data->support_status)
		return desc->allow_not_check_ta_status;
	ret = pps_hal_get_ta_status(info->alg, &status);
	if (ret < 0) {
		PPS_ERR("get ta status fail(%d)\n", ret);
		goto err;
	}

	PPS_INFO("temp = %d, (OVP,OCP,OTP) = (%d,%d,%d)\n",
		  status.temperature, status.ovp, status.ocp, status.otp);
	if (status.ocp || status.otp || status.ovp)
		goto err;
	return true;
err:
	sinfo->hardreset_ta = true;
	return false;
}

static bool pps_check_dvchg_ibusocp(struct pps_algo_info *info,
				     struct pps_stop_info *sinfo)
{
	int ret, i, ibus, acc = 0;
	struct pps_algo_data *data = info->data;
	u32 ibusocp;

	if (!data->is_dvchg_en[PPS_DVCHG_MASTER])
		return true;
	ibusocp = pps_get_dvchg_ibusocp(info, data->ita_setting);
	for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		pps_hal_get_adc_accuracy(info->alg, to_chgidx(i),
					  PPS_ADCCHAN_IBUS, &acc);
		ret = pps_hal_get_adc(info->alg, to_chgidx(i),
				       PPS_ADCCHAN_IBUS, &ibus);
		if (ret < 0) {
			PPS_ERR("get ibus fail(%d)\n", ret);
			return false;
		}
		PPS_INFO("(%s)ibus(%d+-%dmA), ibusocp(%dmA)\n",
			 pps_dvchg_role_name[i], ibus, acc, ibusocp);
		if (ibus > acc)
			ibus -= acc;
		if (ibus > ibusocp) {
			PPS_ERR("(%s)ibus(%dmA) > ibusocp(%dmA)\n",
				 pps_dvchg_role_name[i], ibus, ibusocp);
			return false;
		}
	}
	return true;
}

static bool pps_check_ta_ibusocp(struct pps_algo_info *info,
				  struct pps_stop_info *sinfo)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 itaocp;

	if (!auth_data->support_meas_cap)
		return true;
	ret = pps_get_ta_cap(info);
	if (ret < 0) {
		PPS_ERR("get ta cap fail(%d)\n", ret);
		goto err;
	}
	itaocp = pps_get_itaocp(info);
	PPS_INFO("ita(%dmA), itaocp(%dmA)\n", data->ita_measure, itaocp);
	if (data->ita_measure > itaocp) {
		PPS_ERR("ita(%dmA) > itaocp(%dmA)\n", data->ita_measure,
			 itaocp);
		/* double confirm using dvchg */
		if (!pps_check_dvchg_ibusocp(info, sinfo))
			goto err;
	}
	return true;

err:
	sinfo->hardreset_ta = true;
	return false;
}

/*
 * Check VBUS voltage of divider charger
 * return false if VBUS is over voltage otherwise return true
 */
static bool pps_check_dvchg_vbusovp(struct pps_algo_info *info,
				     struct pps_stop_info *sinfo)
{
	int ret, vbus, i;
	struct pps_algo_data *data = info->data;
	u32 vbusovp;

	vbusovp = pps_get_dvchg_vbusovp(info, data->ita_setting);
	for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pps_hal_get_adc(info->alg, to_chgidx(i),
				       PPS_ADCCHAN_VBUS, &vbus);
		if (ret < 0) {
			PPS_ERR("get vbus fail(%d)\n", ret);
			return false;
		}
		PPS_INFO("(%s)vbus(%dmV), vbusovp(%dmV)\n",
			  pps_dvchg_role_name[i], vbus, vbusovp);
		if (vbus > vbusovp) {
			PPS_ERR("(%s)vbus(%dmV) > vbusovp(%dmV)\n",
				 pps_dvchg_role_name[i], vbus, vbusovp);
			return false;
		}
	}
	return true;
}

static bool pps_check_vbatovp(struct pps_algo_info *info,
			       struct pps_stop_info *sinfo)
{
	int ret, vbat;
	u32 vbatovp;

	vbatovp = pps_get_vbatovp(info);
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0) {
		PPS_ERR("get vbat fail(%d)\n", ret);
		return false;
	}
	PPS_INFO("vbat(%dmV), vbatovp(%dmV)\n", vbat, vbatovp);
	if (vbat > vbatovp) {
		PPS_ERR("vbat(%dmV) > vbatovp(%dmV)\n", vbat, vbatovp);
		return false;
	}
	return true;
}

static bool pps_check_ibatocp(struct pps_algo_info *info,
			       struct pps_stop_info *sinfo)
{
	int ret, ibat;
	struct pps_algo_data *data = info->data;
	u32 ibatocp;

	if (!data->is_dvchg_en[PPS_DVCHG_MASTER])
		return true;
	ibatocp = pps_get_ibatocp(info, data->ita_setting);
	ret = pps_get_adc(info, PPS_ADCCHAN_IBAT, &ibat);
	if (ret < 0) {
		PPS_ERR("get ibat fail(%d)\n", ret);
		return false;
	}
	PPS_INFO("ibat(%dmA), ibatocp(%dmA)\n", ibat, ibatocp);
	if (ibat>0 && ibat > ibatocp) {
		PPS_ERR("ibat(%dmA) > ibatocp(%dmA)\n", ibat, ibatocp);
		return false;
	}
	return true;
}

struct pps_thermal_data {
	const char *name;
	int temp;
	enum pps_thermal_level *temp_level;
	int *temp_level_def;
	int *curlmt;
	int recovery_area;
};

static bool pps_check_thermal_level(struct pps_algo_info *info,
				     struct pps_thermal_data *tdata)
{
	if (tdata->temp >= tdata->temp_level_def[PPS_THERMAL_VERY_HOT]) {
		if (tdata->curlmt[PPS_THERMAL_VERY_HOT] == 0)
			return true;
		PPS_ERR("%s(%d) is over max(%d)\n", tdata->name, tdata->temp,
			tdata->temp_level_def[PPS_THERMAL_VERY_HOT]);
		return false;
	}
	if (tdata->temp <= tdata->temp_level_def[PPS_THERMAL_VERY_COLD]) {
		if (tdata->curlmt[PPS_THERMAL_VERY_COLD] == 0)
			return true;
		PPS_ERR("%s(%d) is under min(%d)\n", tdata->name, tdata->temp,
			tdata->temp_level_def[PPS_THERMAL_VERY_COLD]);
		return false;
	}
	switch (*tdata->temp_level) {
	case PPS_THERMAL_COLD:
		if (tdata->temp >= (tdata->temp_level_def[PPS_THERMAL_COLD] +
				    tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_VERY_COOL;
		break;
	case PPS_THERMAL_VERY_COOL:
		if (tdata->temp >=
		    (tdata->temp_level_def[PPS_THERMAL_VERY_COOL] +
		     tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_COOL;
		else if (tdata->temp <=
			 tdata->temp_level_def[PPS_THERMAL_COLD] &&
			 tdata->curlmt[PPS_THERMAL_COLD] > 0)
			*tdata->temp_level = PPS_THERMAL_COLD;
		break;
	case PPS_THERMAL_COOL:
		if (tdata->temp >= (tdata->temp_level_def[PPS_THERMAL_COOL] +
				    tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_NORMAL;
		else if (tdata->temp <=
			 tdata->temp_level_def[PPS_THERMAL_VERY_COOL] &&
			 tdata->curlmt[PPS_THERMAL_VERY_COOL] > 0)
			*tdata->temp_level = PPS_THERMAL_VERY_COOL;
		break;
	case PPS_THERMAL_NORMAL:
		if (tdata->temp >= tdata->temp_level_def[PPS_THERMAL_WARM] &&
		    tdata->curlmt[PPS_THERMAL_WARM] > 0)
			*tdata->temp_level = PPS_THERMAL_WARM;
		else if (tdata->temp <=
			 tdata->temp_level_def[PPS_THERMAL_COOL] &&
			 tdata->curlmt[PPS_THERMAL_COOL] > 0)
			*tdata->temp_level = PPS_THERMAL_COOL;
		break;
	case PPS_THERMAL_WARM:
		if (tdata->temp <= (tdata->temp_level_def[PPS_THERMAL_WARM] -
				    tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_NORMAL;
		else if (tdata->temp >=
			 tdata->temp_level_def[PPS_THERMAL_VERY_WARM] &&
			 tdata->curlmt[PPS_THERMAL_VERY_WARM] > 0)
			*tdata->temp_level = PPS_THERMAL_VERY_WARM;
		break;
	case PPS_THERMAL_VERY_WARM:
		if (tdata->temp <=
		    (tdata->temp_level_def[PPS_THERMAL_VERY_WARM] -
		     tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_WARM;
		else if (tdata->temp >=
			 tdata->temp_level_def[PPS_THERMAL_HOT] &&
			 tdata->curlmt[PPS_THERMAL_HOT] > 0)
			*tdata->temp_level = PPS_THERMAL_HOT;
		break;
	case PPS_THERMAL_HOT:
		if (tdata->temp <= (tdata->temp_level_def[PPS_THERMAL_HOT] -
				    tdata->recovery_area))
			*tdata->temp_level = PPS_THERMAL_VERY_WARM;
		break;
	default:
		PPS_ERR("NO SUCH STATE\n");
		return false;
	}
	PPS_INFO("%s(%d,%d)\n", tdata->name, tdata->temp, *tdata->temp_level);
	return true;
}

/*
 * Check and adjust battery's temperature level
 * return false if battery's temperature is over maximum or under minimum
 * otherwise return true
 */
static bool pps_check_tbat_level(struct pps_algo_info *info,
				  struct pps_stop_info *sinfo)
{
	int ret, tbat;
	/*10(very_cold) 10(cold) 10(very_cool) 10(cool) 20(normal) 35(warm) 40(very_warm) 45(hot) 50(very_hot)*/
	int tbat_curlmt[PPS_THERMAL_MAX]={-1, -1, -1, 1, 1, 1, 1, 1, -1};
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_thermal_data tdata = {
		.name = "tbat",
		.temp_level_def = desc->tbat_level_def,
		.curlmt = tbat_curlmt,
		.temp_level = &data->tbat_level,
		.recovery_area = desc->tbat_recovery_area,
	};

	ret = pps_get_adc(info, PPS_ADCCHAN_TBAT, &tbat);
	if (ret < 0) {
		PPS_ERR("get tbat fail(%d)\n", ret);
		return false;
	}
	tdata.temp = tbat;
	return pps_check_thermal_level(info, &tdata);
}

/*
 * Check and adjust TA's temperature level
 * return false if TA's temperature is over maximum
 * otherwise return true
 */
static bool pps_check_tta_level(struct pps_algo_info *info,
				 struct pps_stop_info *sinfo)
{
/*
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	struct pps_ta_status status;
	struct pps_thermal_data tdata = {
		.name = "tta",
		.temp_level_def = desc->tta_level_def,
		.curlmt = desc->tta_curlmt,
		.temp_level = &data->tta_level,
		.recovery_area = desc->tta_recovery_area,
	};

	if (!auth_data->support_status)
		return desc->allow_not_check_ta_status;

	ret = pps_hal_get_ta_status(info->alg, &status);
	if (ret < 0) {
		PPS_ERR("get tta fail(%d)\n", ret);
		sinfo->hardreset_ta = true;
		return false;
	}

	tdata.temp = status.temperature;
	return pps_check_thermal_level(info, &tdata);
*/
	return true;
}

/*
 * Check and adjust divider charger's temperature level
 * return false if divider charger's temperature is over maximum
 * otherwise return true
 */
static bool pps_check_tdvchg_level(struct pps_algo_info *info,
				    struct pps_stop_info *sinfo)
{
	int ret, i, tdvchg;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	char buf[14];
	struct pps_thermal_data tdata = {
		.temp_level_def = desc->tdvchg_level_def,
		.curlmt = desc->tdvchg_curlmt,
		.temp_level = &data->tdvchg_level,
		.recovery_area = desc->tdvchg_recovery_area,
	};

	for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pps_hal_get_adc(info->alg, to_chgidx(i),
				       PPS_ADCCHAN_TCHG, &tdvchg);
		if (ret < 0) {
			PPS_ERR("get tdvchg fail(%d)\n", ret);
			return false;
		}
		snprintf(buf, 8 + strlen(pps_dvchg_role_name[i]), "tdvchg_%s",
			 pps_dvchg_role_name[i]);
		tdata.name = buf;
		tdata.temp = tdvchg;
		if (!pps_check_thermal_level(info, &tdata))
			return false;
	}
	return true;
}

/*
 * Check and adjust switching charger's temperature level
 * return false if switching charger's temperature is over maximum
 * otherwise return true
 */
static bool pps_check_tswchg_level(struct pps_algo_info *info,
				    struct pps_stop_info *sinfo)
{
	int ret, tswchg;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_thermal_data tdata = {
		.name = "tswchg",
		.temp_level_def = desc->tswchg_level_def,
		.curlmt = desc->tswchg_curlmt,
		.temp_level = &data->tswchg_level,
		.recovery_area = desc->tswchg_recovery_area,
	};

	if (!data->is_swchg_en)
		return true;

	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_TCHG, &tswchg);
	if (ret < 0) {
		PPS_ERR("get tswchg fail(%d)\n", ret);
		return false;
	}
	tdata.temp = tswchg;
	if (!pps_check_thermal_level(info, &tdata))
		return false;
	data->aicr_lmt = min(data->aicr_lmt, data->aicr_init_lmt -
			     desc->tswchg_curlmt[data->tswchg_level]);
	return true;
}

static bool
(*pps_safety_check_fn[])(struct pps_algo_info *info,
			  struct pps_stop_info *sinfo) = {
	pps_check_ta_status,
	pps_check_ta_ibusocp,
	pps_check_dvchg_vbusovp,
	pps_check_dvchg_ibusocp,
	pps_check_vbatovp,
	pps_check_ibatocp,
	pps_check_tbat_level,
	pps_check_tta_level,
	pps_check_tdvchg_level,
	pps_check_tswchg_level,
};

static bool pps_algo_safety_check(struct pps_algo_info *info)
{
	int i;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_DBG("++\n");
	for (i = 0; i < ARRAY_SIZE(pps_safety_check_fn); i++) {
		if (!pps_safety_check_fn[i](info, &sinfo))
			goto err;
	}
	return true;

err:
	pps_stop(info, &sinfo);
	return false;
}

static bool pps_is_ta_rdy(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_desc *desc = info->desc;
	struct pps_algo_data *data = info->data;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;

	if (!data->ta_ready) {
		auth_data->vcap_min = desc->vta_cap_min;
		auth_data->vcap_max = desc->vta_cap_max;
		auth_data->icap_min = desc->ita_cap_min;
		ret = pps_hal_authenticate_ta(info->alg, auth_data);
		if (ret < 0)
			return false;
		data->ta_ready = true;
	}
	return true;
}

static inline void pps_wakeup_algo_thread(struct pps_algo_data *data)
{
	PPS_DBG("++\n");
	atomic_set(&data->wakeup_thread, 1);
	wake_up_interruptible(&data->wq);
}

static enum alarmtimer_restart
pps_algo_timer_cb(struct alarm *alarm, ktime_t now)
{
	struct pps_algo_data *data =
		container_of(alarm, struct pps_algo_data, timer);

	PPS_DBG("++\n");
	pps_wakeup_algo_thread(data);
	return ALARMTIMER_NORESTART;
}

/*
 * Check charging time of pe5.0 algorithm
 * return false if timeout otherwise return true
 */
static bool pps_algo_check_charging_time(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	ktime_t etime, time_diff;
	struct timespec64 dtime;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	etime = ktime_get_boottime();
	time_diff = ktime_sub(etime, data->stime);
	dtime = ktime_to_timespec64(time_diff);
	if (dtime.tv_sec >= desc->chg_time_max) {
		PPS_ERR("PPS algo timeout(%d, %d)\n", (int)dtime.tv_sec,
			 desc->chg_time_max);
		pps_stop(info, &sinfo);
		return false;
	}
	return true;
}

static inline int __pps_plugout_reset(struct pps_algo_info *info,
				       struct pps_stop_info *sinfo)
{
	struct pps_algo_data *data = info->data;

	PPS_DBG("++\n");
	data->ta_ready = false;
	data->run_once = false;
	return pps_stop(info, sinfo);
}

static int pps_notify_hardreset_hdlr(struct pps_algo_info *info)
{
	struct pps_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	PPS_INFO("++\n");
	return __pps_plugout_reset(info, &sinfo);
}

static int pps_notify_detach_hdlr(struct pps_algo_info *info)
{
	struct pps_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	PPS_INFO("++\n");
	return __pps_plugout_reset(info, &sinfo);
}

static int pps_notify_hwerr_hdlr(struct pps_algo_info *info)
{
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	PPS_INFO("++\n");
	return pps_stop(info, &sinfo);
}

static int pps_notify_ibusucpf_hdlr(struct pps_algo_info *info)
{
	int ret, ibus;
	struct pps_algo_data *data = info->data;

	if (data->ignore_ibusucpf) {
		PPS_INFO("ignore ibusucpf\n");
		data->ignore_ibusucpf = false;
		return 0;
	}
	if (!data->is_dvchg_en[PPS_DVCHG_MASTER]) {
		PPS_INFO("master dvchg is off\n");
		return 0;
	}
	/* Last chance */
	ret = pps_hal_get_adc(info->alg, DVCHG1, PPS_ADCCHAN_IBUS, &ibus);
	if (ret < 0) {
		PPS_ERR("get dvchg ibus fail(%d)\n", ret);
		goto out;
	}
	if (ibus < PPS_IBUSUCPF_RECHECK) {
		PPS_ERR("ibus(%d) < recheck(%d)\n", ibus,
			 PPS_IBUSUCPF_RECHECK);
		goto out;
	}
	PPS_INFO("recheck ibus and it is not ucp\n");
	return 0;
out:
	return pps_notify_hwerr_hdlr(info);
}

static int pps_notify_vbatovp_alarm_hdlr(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->state == PPS_ALGO_STOP)
		return 0;
	PPS_INFO("++\n");
	ret = pps_hal_reset_vbatovp_alarm(info->alg, DVCHG1);
	if (ret < 0) {
		PPS_ERR("reset vbatovp alarm fail(%d)\n", ret);
		return pps_stop(info, &sinfo);
	}
	return 0;
}

static int pps_notify_vbusovp_alarm_hdlr(struct pps_algo_info *info)
{
	int ret;
	struct pps_algo_data *data = info->data;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	if (data->state == PPS_ALGO_STOP)
		return 0;
	PPS_INFO("++\n");
	ret = pps_hal_reset_vbusovp_alarm(info->alg, DVCHG1);
	if (ret < 0) {
		PPS_ERR("reset vbusovp alarm fail(%d)\n", ret);
		return pps_stop(info, &sinfo);
	}
	return 0;
}

static int
(*pps_notify_pre_hdlr[EVT_MAX])(struct pps_algo_info *info) = {
	[EVT_DETACH] = pps_notify_detach_hdlr,
	[EVT_HARDRESET] = pps_notify_hardreset_hdlr,
	[EVT_VBUSOVP] = pps_notify_hwerr_hdlr,
	[EVT_IBUSOCP] = pps_notify_hwerr_hdlr,
	[EVT_IBUSUCP_FALL] = pps_notify_ibusucpf_hdlr,
	[EVT_VBATOVP] = pps_notify_hwerr_hdlr,
	[EVT_IBATOCP] = pps_notify_hwerr_hdlr,
	[EVT_VOUTOVP] = pps_notify_hwerr_hdlr,
	[EVT_VDROVP] = pps_notify_hwerr_hdlr,
	[EVT_VBATOVP_ALARM] = pps_notify_vbatovp_alarm_hdlr,
};

static int
(*pps_notify_post_hdlr[EVT_MAX])(struct pps_algo_info *info) = {
	[EVT_DETACH] = pps_notify_detach_hdlr,
	[EVT_HARDRESET] = pps_notify_hardreset_hdlr,
	[EVT_VBUSOVP] = pps_notify_hwerr_hdlr,
	[EVT_IBUSOCP] = pps_notify_hwerr_hdlr,
	[EVT_IBUSUCP_FALL] = pps_notify_ibusucpf_hdlr,
	[EVT_VBATOVP] = pps_notify_hwerr_hdlr,
	[EVT_IBATOCP] = pps_notify_hwerr_hdlr,
	[EVT_VOUTOVP] = pps_notify_hwerr_hdlr,
	[EVT_VDROVP] = pps_notify_hwerr_hdlr,
	[EVT_VBATOVP_ALARM] = pps_notify_vbatovp_alarm_hdlr,
	[EVT_VBUSOVP_ALARM] = pps_notify_vbusovp_alarm_hdlr,
};

static int pps_pre_handle_notify_evt(struct pps_algo_info *info)
{
	int i;
	struct pps_algo_data *data = info->data;

	mutex_lock(&data->notify_lock);
	PPS_DBG("0x%08X\n", data->notify);
	for (i = 0; i < EVT_MAX; i++) {
		if ((data->notify & BIT(i)) && pps_notify_pre_hdlr[i]) {
			data->notify &= ~BIT(i);
			mutex_unlock(&data->notify_lock);
			pps_notify_pre_hdlr[i](info);
			mutex_lock(&data->notify_lock);
		}
	}
	mutex_unlock(&data->notify_lock);
	return 0;
}

static int pps_post_handle_notify_evt(struct pps_algo_info *info)
{
	int i;
	struct pps_algo_data *data = info->data;

	mutex_lock(&data->notify_lock);
	PPS_DBG("0x%08X\n", data->notify);
	for (i = 0; i < EVT_MAX; i++) {
		if ((data->notify & BIT(i)) && pps_notify_post_hdlr[i]) {
			data->notify &= ~BIT(i);
			mutex_unlock(&data->notify_lock);
			pps_notify_post_hdlr[i](info);
			mutex_lock(&data->notify_lock);
		}
	}
	mutex_unlock(&data->notify_lock);
	return 0;
}

static int pps_dump_charging_info(struct pps_algo_info *info)
{
	int ret, i;
	int vbus, ibus[PPS_DVCHG_MAX] = {0}, ibus_swchg = 0, vbat, ibat, vout[PPS_DVCHG_MAX] = {0};
	int ibus_total, vsys, tbat;
	u32 soc;
	struct pps_algo_data *data = info->data;

	/* vbus */
	ret = pps_get_adc(info, PPS_ADCCHAN_VBUS, &vbus);
	if (ret < 0)
		PPS_ERR("get vbus fail(%d)\n", ret);
	/* ibus */
	for (i = 0; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pps_hal_get_adc(info->alg, to_chgidx(i),
				       PPS_ADCCHAN_IBUS, &ibus[i]);
		if (ret < 0) {
			PPS_ERR("get %s ibus fail\n", pps_dvchg_role_name[i]);
			continue;
		}
	}

	ibus_total = ibus[PPS_DVCHG_MASTER] + ibus[PPS_DVCHG_SLAVE];

	if (data->is_swchg_en) {
		ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_IBUS,
				       &ibus_swchg);
		if (ret < 0)
			PPS_ERR("get swchg ibus fail\n");
	}
	/* vbat */
	ret = pps_get_adc(info, PPS_ADCCHAN_VBAT, &vbat);
	if (ret < 0)
		PPS_ERR("get vbat fail\n");
	/* ibat */
	ret = pps_get_adc(info, PPS_ADCCHAN_IBAT, &ibat);
	if (ret < 0)
		PPS_ERR("get ibat fail\n");

	ret = pps_get_ta_cap_by_supportive(info, &data->vta_measure,
					     &data->ita_measure);
	if (ret < 0)
		PPS_ERR("get ta measure cap fail(%d)\n", ret);

	/* vout */
	for (i = PPS_DVCHG_MASTER; i < PPS_DVCHG_MAX; i++) {
		if (!data->is_dvchg_en[i])
			continue;
		ret = pps_hal_get_adc(info->alg, to_chgidx(i), PPS_ADCCHAN_VOUT,
					   &vout[i]);
		if (ret < 0) {
			PPS_ERR("get %s ibus fail\n", pps_dvchg_role_name[i]);
			continue;
		}
	}

	ret = pps_hal_get_adc(info->alg, CHG1, PPS_ADCCHAN_VSYS,
				   &vsys);
	if (ret < 0) {
		PPS_ERR("get vsys from swchg fail\n");
	}

	ret = pps_get_adc(info, PPS_ADCCHAN_TBAT, &tbat);

	ret = pps_hal_get_soc(info->alg, &soc);
	if (ret < 0) {
		PPS_ERR("get soc fail\n");
	}

	PPS_INFO("vbus,ibus(master,slave,sw),vbat,ibat=%d,(%d,%d,%d),%d,%d\n",
		 vbus, ibus[PPS_DVCHG_MASTER], ibus[PPS_DVCHG_SLAVE],
		 ibus_swchg, vbat, ibat);
	PPS_INFO("vta,ita(set,meas)=(%d,%d),(%d,%d),force_cv=%d\n",
		 data->vta_setting, data->vta_measure, data->ita_setting,
		 data->ita_measure, data->force_ta_cv);
	PPS_INFO("vout(master,slave)=(%d,%d)\n",
		 vout[PPS_DVCHG_MASTER], vout[PPS_DVCHG_SLAVE]);
	PPS_INFO("[PPS] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		 vbus, ibus[PPS_DVCHG_MASTER], ibus[PPS_DVCHG_SLAVE], ibus_total, vbat, ibat,
		 tbat, vsys, soc,
		 vout[PPS_DVCHG_MASTER], vout[PPS_DVCHG_SLAVE]);

	return 0;
}
static bool pps_dump_hwerr_info(struct pps_algo_info *info)
{
	struct pps_algo_data *data = info->data;
	bool err = false;
	u32 hwerr = PPS_HWERR_NOTIFY;

	mutex_lock(&data->notify_lock);
	if (data->ignore_ibusucpf)
		hwerr &= ~BIT(EVT_IBUSUCP_FALL);
	err = !!(data->notify & hwerr);
	mutex_unlock(&data->notify_lock);
	if (err){
		PPS_ERR("H/W error(0x%08X)", hwerr);
		pps_hal_dump_registers(info->alg, to_chgidx(PPS_DVCHG_MASTER));
	}
	return err;
}
static bool pps_is_algo_waiver(struct chg_alg_device *alg)
{
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	bool waiver = false;

	mutex_lock(&data->lock);
	if (data->waiver) {
		waiver = true;
	}
	mutex_unlock(&data->lock);
	PPS_INFO("waiver = %d\n", waiver);
	return waiver;
}
static int pps_algo_threadfn(void *param)
{
	struct pps_algo_info *info = param;
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	struct pps_ta_auth_data *auth_data = &data->ta_auth_data;
	u32 sec, ms, polling_interval;
	ktime_t ktime;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	while (!kthread_should_stop()) {
		wait_event_interruptible(data->wq,
					 atomic_read(&data->wakeup_thread));
		pm_stay_awake(info->dev);
		if (atomic_read(&data->stop_thread)) {
			pm_relax(info->dev);
			break;
		}
		atomic_set(&data->wakeup_thread, 0);
		mutex_lock(&data->lock);
		PPS_INFO("state = %s\n", pps_algo_state_name[data->state]);
		if (atomic_read(&data->stop_algo))
			pps_stop(info, &sinfo);
		pps_pre_handle_notify_evt(info);
		if (data->state != PPS_ALGO_STOP) {
			pps_algo_check_charging_time(info);
			pps_calculate_vbat_ircmp(info);
			pps_select_vbat_cv(info);
			pps_dump_charging_info(info);
			pps_dump_hwerr_info(info);
		}
		switch (data->state) {
		case PPS_ALGO_INIT:
			pps_algo_init(info);
			break;
		case PPS_ALGO_MEASURE_R:
			pps_algo_measure_r(info);
			break;
		case PPS_ALGO_SS_SWCHG:
			pps_algo_ss_swchg(info);
			break;
		case PPS_ALGO_SS_DVCHG:
			pps_algo_ss_dvchg(info);
			break;
		case PPS_ALGO_CC_CV:
			pps_algo_cc_cv(info);
			break;
		case PPS_ALGO_STOP:
			PPS_INFO("PPS STOP\n");
			break;
		default:
			PPS_ERR("NO SUCH STATE\n");
			break;
		}
		pps_post_handle_notify_evt(info);
		if (data->state != PPS_ALGO_STOP) {
			if (!pps_algo_safety_check(info))
				goto cont;
			pps_dump_charging_info(info);
			if (data->state == PPS_ALGO_CC_CV &&
			    auth_data->support_cc && !data->force_ta_cv)
				polling_interval = desc->polling_interval;
			else
				polling_interval =
					PPS_INIT_POLLING_INTERVAL;
			sec = polling_interval / 1000;
			ms = polling_interval % 1000;
			ktime = ktime_set(sec, _MS_TO_NS(ms));
			alarm_start_relative(&data->timer, ktime);
		}
cont:
		mutex_unlock(&data->lock);
		pm_relax(info->dev);
	}
	return 0;
}

/* =================================================================== */
/* PPS Algo OPS                                                        */
/* =================================================================== */

static int pps_init_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;

	mutex_lock(&data->lock);
	PPS_DBG("++\n");

	if (data->inited) {
		PPS_INFO("already inited\n");
		goto out;
	}
	if (pps_hal_init_hardware(info->alg, desc->support_ta,
				   desc->support_ta_cnt)) {
		PPS_ERR("init hw fail\n", __func__);
		goto out;
	}
	data->inited = true;
	PPS_INFO("successfully\n");
out:
	mutex_unlock(&data->lock);
	return ret;
}

static bool pps_is_algo_running(struct chg_alg_device *alg)
{
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	bool running = true;

	mutex_lock(&data->lock);

	if (!data->inited) {
		running = false;
		goto out_unlock;
	}
	running = !(data->state == PPS_ALGO_STOP);
	PPS_DBG("running = %d\n", running);
out_unlock:
	mutex_unlock(&data->lock);

	return running;
}
static bool pps_is_algo_finish(struct chg_alg_device *alg)
{
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	bool finish = false;

	mutex_lock(&data->lock);
	if (data->finish) {
		finish = true;
	}
	mutex_unlock(&data->lock);
	PPS_INFO("ALGO FINISH  = %d\n", finish);
	return finish;
}

static int pps_is_algo_ready(struct chg_alg_device *alg)
{
	int ret;
	u32 soc;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	/*
	struct pps_algo_data *data = info->data;
	struct pps_algo_desc *desc = info->desc;
	*/
	struct pps_algo_data *data = NULL;
	struct pps_algo_desc *desc = NULL;
	if (info == NULL)
		return ALG_INIT_FAIL;
	data = info->data;
	desc = info->desc;
	if(data == NULL || desc == NULL)
		return ALG_INIT_FAIL;
	if (pps_is_algo_waiver(alg))
		return ALG_WAIVER;
	if (pps_is_algo_running(info->alg))
		return ALG_RUNNING;

	if(pps_is_algo_finish(info->alg))
		return ALG_DONE;
	mutex_lock(&data->lock);
	PPS_DBG("++\n");
	if (!data->inited) {
		ret = ALG_INIT_FAIL;
		goto out;
	}

	PPS_DBG("run once(%d)\n", data->run_once);
	if (data->run_once) {
		if (!(data->notify & PPS_RESET_NOTIFY)) {
			ret = ALG_NOT_READY;
			goto out;
		}
		mutex_lock(&data->notify_lock);
		PPS_INFO("run once but detach/hardreset happened\n");
		data->notify &= ~PPS_RESET_NOTIFY;
		data->run_once = false;
		data->ta_ready = false;
		mutex_unlock(&data->notify_lock);
	}

	ret = pps_hal_get_soc(info->alg, &soc);
	if (ret < 0) {
		PPS_ERR("get SOC fail(%d)\n", ret);
		ret = ALG_INIT_FAIL;
		goto out;
	}
	if (soc < desc->start_soc_min || soc > desc->start_soc_max) {
		if (soc > 0) {
			PPS_INFO("soc(%d) not in range(%d~%d)\n", soc,
				  desc->start_soc_min, desc->start_soc_max);
			ret = ALG_WAIVER;
			data->waiver = true;
			goto out;
		}
		if (soc == -1 && data->ref_vbat > data->vbat_threshold) {
			PPS_INFO("soc(%d) not in range(%d~%d)\n", soc,
				  desc->start_soc_min, desc->start_soc_max);
			ret = ALG_WAIVER;
			data->waiver = true;
			goto out;
		}
	}

	if (!pps_is_ta_rdy(info)) {
		ret = pps_hal_is_pd_adapter_ready(alg);
		if(ALG_READY == ret){
			ret = ALG_WAIVER;
			data->waiver = true;
		}
		goto out;
	}
	ret = ALG_READY;
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pps_start_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;

	if (pps_is_algo_running(alg))
		return ALG_RUNNING;
	mutex_lock(&data->lock);
	PPS_DBG("++\n");
	if (!data->inited || !data->ta_ready) {
		ret = ALG_INIT_FAIL;
		goto out;
	}
	pps_hal_enable_sw_vbusovp(alg, false);
	ret = pps_start(info);
	if (ret < 0) {
		pps_hal_enable_sw_vbusovp(alg, true);
		PPS_ERR("start PPS algo fail\n");
		ret = ALG_INIT_FAIL;
	}
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pps_plugout_reset(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	struct pps_stop_info sinfo = {
		.reset_ta = false,
		.hardreset_ta = false,
	};

	mutex_lock(&data->lock);
	PPS_DBG("++\n");
	if (!data->inited)
		goto out;
	data->waiver = false;
	data->finish = false;
	ret = __pps_plugout_reset(info, &sinfo);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pps_stop_algo(struct chg_alg_device *alg)
{
	int ret = 0;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	struct pps_stop_info sinfo = {
		.reset_ta = true,
		.hardreset_ta = false,
	};

	atomic_set(&data->stop_algo, 1);
	mutex_lock(&data->lock);
	if (!data->inited)
		goto out;
	ret = pps_stop(info, &sinfo);
out:
	mutex_unlock(&data->lock);
	return ret;
}

static int pps_notifier_call(struct chg_alg_device *alg,
			      struct chg_alg_notify *notify)
{
	int ret = 0;
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;

	mutex_lock(&data->notify_lock);
	if (data->state == PPS_ALGO_STOP) {
		if ((notify->evt == EVT_DETACH ||
		     notify->evt == EVT_HARDRESET) && data->run_once) {
			PPS_INFO("detach/hardreset && run once after stop\n");
			data->notify |= BIT(notify->evt);
		}
		goto out;
	}
	PPS_INFO("%s\n", chg_alg_notify_evt_tostring(notify->evt));
	switch (notify->evt) {
	case EVT_DETACH:
	case EVT_HARDRESET:
	case EVT_VBUSOVP:
	case EVT_IBUSOCP:
	case EVT_IBUSUCP_FALL:
	case EVT_VBATOVP:
	case EVT_IBATOCP:
	case EVT_VOUTOVP:
	case EVT_VDROVP:
	case EVT_VBATOVP_ALARM:
	case EVT_VBUSOVP_ALARM:
		data->notify |= BIT(notify->evt);
		break;
	default:
		ret = -EINVAL;
		goto out;
	}
	pps_wakeup_algo_thread(data);
out:
	mutex_unlock(&data->notify_lock);
	return ret;
}

static int pps_set_current_limit(struct chg_alg_device *alg,
				  struct chg_limit_setting *setting)
{
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;
	int cv = micro_to_milli_2(setting->cv);
	int ic = micro_to_milli_2(setting->input_current_limit_dvchg1);

	mutex_lock(&data->ext_lock);
	if (data->cv_limit != cv || data->input_current_limit != ic) {
		data->cv_limit = cv;
		data->input_current_limit = ic;
		PPS_INFO("ic = %d, cv = %d\n", ic, cv);
		pps_wakeup_algo_thread(data);
	}
	mutex_unlock(&data->ext_lock);
	return 0;
}

static int pps_set_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int value)
{
	struct pps_algo_info *info = chg_alg_dev_get_drvdata(alg);
	struct pps_algo_data *data = info->data;

	pr_notice("%s %d %d\n", __func__, s, value);

	switch (s) {
	case ALG_LOG_LEVEL:
		log_level = value;
		break;
	case ALG_REF_VBAT:
		data->ref_vbat = value;
		break;
	default:
		break;
	}

	return 0;
}

int pps_get_prop(struct chg_alg_device *alg,
		enum chg_alg_props s, int *value)
{

	pr_notice("%s\n", __func__);
	if (s == ALG_MAX_VBUS)
		*value = 11000;
	else
		pr_notice("%s does not support prop:%d\n", __func__, s);
	return 0;
}

static struct chg_alg_ops pps_ops = {
	.init_algo = pps_init_algo,
	.is_algo_ready = pps_is_algo_ready,
	.start_algo = pps_start_algo,
	.is_algo_running = pps_is_algo_running,
	.plugout_reset = pps_plugout_reset,
	.stop_algo = pps_stop_algo,
	.notifier_call = pps_notifier_call,
	.set_current_limit = pps_set_current_limit,
	.set_prop = pps_set_prop,
	.get_prop = pps_get_prop,
};

#define PPS_DT_VALPROP_ARR(name, sz) \
	{#name, offsetof(struct pps_algo_desc, name), sz}

#define PPS_DT_VALPROP(name) \
	PPS_DT_VALPROP_ARR(name, 1)

struct pps_dtprop {
	const char *name;
	size_t offset;
	size_t sz;
};

static inline void pps_parse_dt_u32(struct device_node *np, void *desc,
				     const struct pps_dtprop *props,
				     int prop_cnt)
{
	int i;

	for (i = 0; i < prop_cnt; i++) {
		if (unlikely(!props[i].name))
			continue;
		of_property_read_u32(np, props[i].name, desc + props[i].offset);
	}
}

static inline void pps_parse_dt_u32_arr(struct device_node *np, void *desc,
					 const struct pps_dtprop *props,
					 int prop_cnt)
{
	int i;

	for (i = 0; i < prop_cnt; i++) {
		if (unlikely(!props[i].name))
			continue;
		of_property_read_u32_array(np, props[i].name,
					   desc + props[i].offset, props[i].sz);
	}
}

static inline int __of_property_read_s32_array(const struct device_node *np,
					       const char *propname,
					       s32 *out_values, size_t sz)
{
	return of_property_read_u32_array(np, propname, (u32 *)out_values, sz);
}

static inline void pps_parse_dt_s32_arr(struct device_node *np, void *desc,
					 const struct pps_dtprop *props,
					 int prop_cnt)
{
	int i;

	for (i = 0; i < prop_cnt; i++) {
		if (unlikely(!props[i].name))
			continue;
		__of_property_read_s32_array(np, props[i].name,
					     desc + props[i].offset,
					     props[i].sz);
	}
}

static const struct pps_dtprop pps_dtprops_u32[] = {
	PPS_DT_VALPROP(polling_interval),
	PPS_DT_VALPROP(ta_cv_ss_repeat_tmin),
	PPS_DT_VALPROP(vbat_cv),
	PPS_DT_VALPROP(start_soc_min),
	PPS_DT_VALPROP(start_soc_max),
	PPS_DT_VALPROP(start_vbat_max),
	PPS_DT_VALPROP(idvchg_term),
	PPS_DT_VALPROP(idvchg_step),
	PPS_DT_VALPROP(idvchg_ss_init),
	PPS_DT_VALPROP(idvchg_ss_step),
	PPS_DT_VALPROP(idvchg_ss_step1),
	PPS_DT_VALPROP(idvchg_ss_step2),
	PPS_DT_VALPROP(idvchg_ss_step1_vbat),
	PPS_DT_VALPROP(idvchg_ss_step2_vbat),
	PPS_DT_VALPROP(ta_blanking),
	PPS_DT_VALPROP(swchg_aicr),
	PPS_DT_VALPROP(swchg_ichg),
	PPS_DT_VALPROP(swchg_aicr_ss_init),
	PPS_DT_VALPROP(swchg_aicr_ss_step),
	PPS_DT_VALPROP(swchg_off_vbat),
	PPS_DT_VALPROP(force_ta_cv_vbat),
	PPS_DT_VALPROP(chg_time_max),
	PPS_DT_VALPROP(tta_recovery_area),
	PPS_DT_VALPROP(tbat_recovery_area),
	PPS_DT_VALPROP(tdvchg_recovery_area),
	PPS_DT_VALPROP(tswchg_recovery_area),
	PPS_DT_VALPROP(ifod_threshold),
	PPS_DT_VALPROP(rsw_min),
	PPS_DT_VALPROP(ircmp_rbat),
	PPS_DT_VALPROP(ircmp_vclamp),
	PPS_DT_VALPROP(vta_cap_min),
	PPS_DT_VALPROP(vta_cap_max),
	PPS_DT_VALPROP(ita_cap_min),
};

static const struct pps_dtprop pps_dtprops_u32_array[] = {
	PPS_DT_VALPROP_ARR(ita_level, PPS_RCABLE_MAX),
	PPS_DT_VALPROP_ARR(rcable_level, PPS_RCABLE_MAX),
	PPS_DT_VALPROP_ARR(ita_level_dual, PPS_RCABLE_MAX),
	PPS_DT_VALPROP_ARR(rcable_level_dual, PPS_RCABLE_MAX),
};

static const struct pps_dtprop pps_dtprops_s32_array[] = {
	PPS_DT_VALPROP_ARR(tta_level_def, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tta_curlmt, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tbat_level_def, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tbat_curlmt, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tdvchg_level_def, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tdvchg_curlmt, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tswchg_level_def, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tswchg_curlmt, PPS_THERMAL_MAX),
	PPS_DT_VALPROP_ARR(tbat_t0_to_t1_cv, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t0_to_t1_curlmt, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t1_to_t2_cv, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t1_to_t2_curlmt, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t2_to_t3_cv, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t2_to_t3_curlmt, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t3_to_t4_cv, PPS_JEITA_MAX),
	PPS_DT_VALPROP_ARR(tbat_t3_to_t4_curlmt, PPS_JEITA_MAX),
};

static int pps_parse_dt(struct pps_algo_info *info)
{
	int i, ret;
	struct pps_algo_desc *desc;
	struct pps_algo_data *data;
	struct device_node *np = info->dev->of_node;
	u32 val;

	desc = devm_kzalloc(info->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;
	info->desc = desc;
	data = info->data;
	memcpy(desc, &algo_desc_defval, sizeof(*desc));

	ret = of_property_count_strings(np, "support_ta");
	if (ret < 0)
		return ret;
	desc->support_ta_cnt = ret;
	desc->support_ta = devm_kzalloc(info->dev, ret * sizeof(char *),
					GFP_KERNEL);
	if (!desc->support_ta)
		return -ENOMEM;
	for (i = 0; i < desc->support_ta_cnt; i++) {
		ret = of_property_read_string_index(np, "support_ta", i,
						    &desc->support_ta[i]);
		if (ret < 0)
			return ret;
		PPS_INFO("support ta(%s)\n", desc->support_ta[i]);
	}

	desc->allow_not_check_ta_status =
		of_property_read_bool(np, "allow_not_check_ta_status");
	pps_parse_dt_u32(np, (void *)desc, pps_dtprops_u32,
			  ARRAY_SIZE(pps_dtprops_u32));
	pps_parse_dt_u32_arr(np, (void *)desc, pps_dtprops_u32_array,
			      ARRAY_SIZE(pps_dtprops_u32_array));
	pps_parse_dt_s32_arr(np, (void *)desc, pps_dtprops_s32_array,
			      ARRAY_SIZE(pps_dtprops_s32_array));
	if (desc->swchg_aicr == 0 || desc->swchg_ichg == 0) {
		desc->swchg_aicr = 0;
		desc->swchg_ichg = 0;
	}

	if (of_property_read_u32(np, "vbat_threshold", &val) >= 0)
		data->vbat_threshold = val;
	else {
		pr_notice("turn off vbat_threshold checking:%d\n",
			DISABLE_VBAT_THRESHOLD);
		data->vbat_threshold = DISABLE_VBAT_THRESHOLD;
	}

	return 0;
}

static int pps_probe(struct platform_device *pdev)
{
	int ret;
	struct pps_algo_info *info;
	struct pps_algo_data *data;

	dev_info(&pdev->dev, "%s\n", __func__);

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	info->data = data;
	info->dev = &pdev->dev;
	platform_set_drvdata(pdev, info);

	ret = pps_parse_dt(info);
	if (ret < 0) {
		PPS_ERR("%s parse dt fail(%d)\n", __func__, ret);
		return ret;
	}

	mutex_init(&data->notify_lock);
	mutex_init(&data->lock);
	mutex_init(&data->ext_lock);
	init_waitqueue_head(&data->wq);
	atomic_set(&data->wakeup_thread, 0);
	atomic_set(&data->stop_thread, 0);
	data->state = PPS_ALGO_STOP;
	alarm_init(&data->timer, ALARM_REALTIME, pps_algo_timer_cb);
	data->task = kthread_run(pps_algo_threadfn, info, "pps_algo_task");
	if (IS_ERR(data->task)) {
		ret = PTR_ERR(data->task);
		PPS_ERR("%s run task fail(%d)\n", __func__, ret);
		goto err;
	}
	device_init_wakeup(info->dev, true);

	info->alg = chg_alg_device_register("pps", info->dev, info, &pps_ops,
					    NULL);
	if (IS_ERR_OR_NULL(info->alg)) {
		PPS_ERR("%s reg pe5 algo fail(%d)\n", __func__, ret);
		ret = PTR_ERR(info->alg);
		goto err;
	}
	chg_alg_dev_set_drvdata(info->alg, info);

	dev_info(info->dev, "%s successfully\n", __func__);
	return 0;
err:
	mutex_destroy(&data->ext_lock);
	mutex_destroy(&data->lock);
	mutex_destroy(&data->notify_lock);
	chg_alg_device_unregister(info->alg);
	return ret;
}

static int pps_remove(struct platform_device *pdev)
{
	struct pps_algo_info *info = platform_get_drvdata(pdev);
	struct pps_algo_data *data;

	if (info) {
		data = info->data;
		atomic_set(&data->stop_thread, 1);
		pps_wakeup_algo_thread(data);
		kthread_stop(data->task);
		mutex_destroy(&data->ext_lock);
		mutex_destroy(&data->lock);
		mutex_destroy(&data->notify_lock);
		chg_alg_device_unregister(info->alg);
	}

	return 0;
}

static int __maybe_unused pps_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pps_algo_info *info = platform_get_drvdata(pdev);
	struct pps_algo_data *data = info->data;

	dev_info(dev, "%s\n", __func__);
	mutex_lock(&data->lock);
	return 0;
}

static int __maybe_unused pps_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pps_algo_info *info = platform_get_drvdata(pdev);
	struct pps_algo_data *data = info->data;

	dev_info(dev, "%s\n", __func__);
	mutex_unlock(&data->lock);
	return 0;
}

static SIMPLE_DEV_PM_OPS(pps_pm_ops, pps_suspend, pps_resume);

static const struct of_device_id _mtk_pps_of_match[] = {
	{ .compatible = "mediatek,charger,pps", },
	{},
};
MODULE_DEVICE_TABLE(of, _mtk_pps_of_match);

static struct platform_driver _pps_platdrv = {
	.probe = pps_probe,
	.remove = pps_remove,
	.driver = {
		.name = "pps",
		.owner = THIS_MODULE,
		.pm = &pps_pm_ops,
		.of_match_table = _mtk_pps_of_match,
	},
};

static int __init pps_init(void)
{
	return platform_driver_register(&_pps_platdrv);
}

static void __exit pps_exit(void)
{
	platform_driver_unregister(&_pps_platdrv);
}
module_init(pps_init);
module_exit(pps_exit);

MODULE_DESCRIPTION("PPS Driver");
MODULE_LICENSE("GPL");
