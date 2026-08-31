// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

#include "mtk_low_battery_throttling.h"
#include "pmic_lbat_service.h"
#include "pmic_lvsys_notify.h"

#define CREATE_TRACE_POINTS
#include "mtk_low_battery_throttling_trace.h"

#define LBCB_MAX_NUM 16
#define TEMP_MAX_STAGE_NUM 6
#define THD_VOLTS_LENGTH 20
#define POWER_INT0_VOLT 3400
#define POWER_INT1_VOLT 3250
#define POWER_INT2_VOLT 3100
#define POWER_INT3_VOLT 2700
#define LVSYS_THD_VOLT_H 3100
#define LVSYS_THD_VOLT_L 2900
#define MAX_INT 0x7FFFFFFF
#define MIN_LBAT_VOLT 2000
#define LOW_BATTERY_INIT_NUM (LOW_BATTERY_LEVEL_NUM-2)

struct lbat_intr_tbl {
	unsigned int volt_thd;
	unsigned int lt_en;
	unsigned int lt_lv;
	unsigned int ht_en;
	unsigned int ht_lv;
};

struct lbat_thd_tbl {
	unsigned int *thd_volts;
	unsigned int *thd_level;
	int thd_volts_size;
	int thd_level_size;
	struct lbat_intr_tbl *lbat_intr_info;
};

struct low_bat_thl_priv {
	unsigned int hv_thd_volt;
	unsigned int lv1_thd_volt;
	unsigned int lv2_thd_volt;
	unsigned int isThreeLevel;
	int low_bat_thl_level;
	int lbat_thl_intr_level;
	int lvsys_thl_intr_level;
	int low_bat_thl_stop;
	int low_bat_thd_modify;
	unsigned int lvsys_thd_volt_l;
	unsigned int lvsys_thd_volt_h;
	unsigned int ppb_mode;
	struct lbat_user *lbat_pt;
	struct work_struct temp_work;
	struct power_supply *psy;
	int *temp_thd;
	int temp_max_stage;
	int temp_cur_stage;
	int temp_reg_stage;
	int max_throttle_level;
	int pmic_idx;
	struct lbat_thd_tbl lbat_thd_info[TEMP_MAX_STAGE_NUM];
};

struct low_battery_callback_table {
	void (*lbcb)(enum LOW_BATTERY_LEVEL_TAG, void *data);
	void *data;
};

static struct notifier_block lbat_nb;
static struct low_bat_thl_priv *low_bat_thl_data;
static struct low_battery_callback_table lbcb_tb[LBCB_MAX_NUM] = { {0}, {0} };
static DEFINE_MUTEX(exe_thr_lock);

static int rearrange_volt(struct lbat_intr_tbl *intr_info, unsigned int *volt_l,
	unsigned int *volt_h, unsigned int *thd_level, unsigned int num)
{
	unsigned int idx_l = 0, idx_h = 0, idx_t = 0, lv_idx = 0, i;
	unsigned int volt_l_next, volt_h_next, thd_level_next;
	unsigned int thd_level_cnt = 0;

	for (i = 0; i < num - 1; i++) {
		if (volt_l[i] < volt_l[i+1] || volt_h[i] < volt_h[i+1]) {
			pr_notice("[%s] i=%d volt_l(%d, %d) volt_h(%d, %d) error\n",
				__func__, i, volt_l[i], volt_l[i+1], volt_h[i], volt_h[i+1]);
			return -EINVAL;
		}
	}
	for (i = 0; i < num * 2; i++) {
		volt_l_next = (idx_l < num) ? volt_l[idx_l] : 0;
		volt_h_next = (idx_h < num) ? volt_h[idx_h] : 0;
		thd_level_next = (lv_idx < LOW_BATTERY_INIT_NUM) ? thd_level[lv_idx] : 0;
		if (volt_l_next > volt_h_next && volt_l_next > 0) {
			intr_info[idx_t].volt_thd = volt_l_next;
			intr_info[idx_t].lt_en = 1;
			intr_info[idx_t].lt_lv = thd_level_next;
			thd_level_cnt++;
			if (thd_level_cnt == 2)
				lv_idx++;
			idx_l++;
			idx_t++;
		} else if (volt_l_next == volt_h_next && volt_l_next > 0) {
			intr_info[idx_t].volt_thd = volt_l_next;
			intr_info[idx_t].lt_en = 1;
			intr_info[idx_t].lt_lv = thd_level_next;
			intr_info[idx_t].ht_en = 1;
			intr_info[idx_t].ht_lv = thd_level_next;
			lv_idx++;
			idx_l++;
			idx_h++;
			idx_t++;
		} else if (volt_h_next > 0) {
			intr_info[idx_t].volt_thd = volt_h_next;
			intr_info[idx_t].ht_en = 1;
			intr_info[idx_t].ht_lv = thd_level_next;
			lv_idx++;
			thd_level_cnt = 0;
			idx_h++;
			idx_t++;
		} else
			break;
	}
	for (i = 0; i < idx_t; i++) {
		pr_info("[%s] intr_info[%d] = (%d, trig l[%d %d] h[%d %d])\n",
				__func__, i, intr_info[i].volt_thd, intr_info[i].lt_en,
				intr_info[i].lt_lv, intr_info[i].ht_en, intr_info[i].ht_lv);
	}
	return idx_t;
}

static void dump_thd_volts(struct device *dev, unsigned int *thd_volts, unsigned int size)
{
	int i, r = 0;
	char str[128] = "";
	size_t len = sizeof(str) - 1;

	for (i = 0; i < size; i++) {
		r += snprintf(str + r, len - r, "%s%d mV", i ? ", " : "", thd_volts[i]);
		if (r >= len)
			return;
	}
	dev_notice(dev, "%s Done\n", str);
}

static void dump_thd_volts_ext(unsigned int *thd_volts, unsigned int size)
{
	int i, r = 0;
	char str[128] = "";
	size_t len = sizeof(str) - 1;

	for (i = 0; i < size; i++) {
		r += snprintf(str + r, len - r, "%s%d mV", i ? ", " : "", thd_volts[i]);
		if (r >= len)
			return;
	}
	pr_info("%s Done\n", str);
}

int register_low_battery_notify(low_battery_callback lb_cb,
				enum LOW_BATTERY_PRIO_TAG prio_val, void *data)
{
	int ret = 2;

	if (!low_bat_thl_data) {
		pr_info("[%s] low_bat_thl_data not allocate\n", __func__);
		return -EINVAL;
	}

	if (low_bat_thl_data->isThreeLevel)
		ret = 3;

	if (prio_val >= LBCB_MAX_NUM) {
		pr_notice("[%s] prio_val=%d, out of boundary\n",
			  __func__, prio_val);
		return -EINVAL;
	}

	if (!lb_cb)
		return ret;

	if (lbcb_tb[prio_val].lbcb != 0)
		pr_info("[%s] Notice: LBCB has been registered\n", __func__);

	lbcb_tb[prio_val].lbcb = lb_cb;
	lbcb_tb[prio_val].data = data;
	pr_info("[%s] prio_val=%d ret=%d\n", __func__, prio_val, ret);

	if (low_bat_thl_data->low_bat_thl_level && lbcb_tb[prio_val].lbcb) {
		lbcb_tb[prio_val].lbcb(low_bat_thl_data->low_bat_thl_level, lbcb_tb[prio_val].data);
		pr_info("[%s] notify lv=%d\n", __func__, low_bat_thl_data->low_bat_thl_level);
	}

	return ret;
}
EXPORT_SYMBOL(register_low_battery_notify);

void exec_throttle(unsigned int level)
{
	int i;

	if (!low_bat_thl_data) {
		pr_info("[%s] Failed to create low_bat_thl_data\n", __func__);
		return;
	}

	if (low_bat_thl_data->low_bat_thl_level == level) {
		pr_info("[%s] same throttle level\n", __func__);
		return;
	}

	trace_low_battery_throttling_level(low_bat_thl_data->low_bat_thl_level);

	low_bat_thl_data->low_bat_thl_level = level;
	for (i = 0; i < ARRAY_SIZE(lbcb_tb); i++) {
		if (i == LOW_BATTERY_PRIO_UFS && lbcb_tb[i].data == 0) {
			pr_info("[%s] lbcb_tb[%d].data not ready", __func__, i);
			continue;
		}

		if (lbcb_tb[i].lbcb)
			lbcb_tb[i].lbcb(low_bat_thl_data->low_bat_thl_level, lbcb_tb[i].data);
	}

	pr_info("[%s] low_battery_level = %d\n", __func__, level);
}

static unsigned int decide_and_throttle(enum LOW_BATTERY_USER_TAG user, unsigned int input)
{
	struct lbat_thd_tbl *thd_info;
	unsigned int low_thd_volts[LOW_BATTERY_INIT_NUM] = {MIN_LBAT_VOLT+40, MIN_LBAT_VOLT+30,
		MIN_LBAT_VOLT+20, MIN_LBAT_VOLT+10};

	pr_info("%s: user=%d, input=%d\n", __func__, user, input);
	if (!low_bat_thl_data) {
		pr_info("[%s] Failed to create low_bat_thl_data\n", __func__);
		return 0;
	}

	mutex_lock(&exe_thr_lock);
	if (user == LBAT_INTR || user == TEMP_CHANGE) {
		low_bat_thl_data->lbat_thl_intr_level = input;
		if (low_bat_thl_data->low_bat_thl_stop > 0 || low_bat_thl_data->ppb_mode > 0) {
			pr_info("[%s] throttle not apply, low_bat_thl_stop=%d, ppb_mode=%d\n",
			__func__, low_bat_thl_data->low_bat_thl_stop,
			low_bat_thl_data->ppb_mode);
		} else {
			input = MAX(low_bat_thl_data->lbat_thl_intr_level,
						low_bat_thl_data->lvsys_thl_intr_level);
			exec_throttle(input);
		}
		mutex_unlock(&exe_thr_lock);
	} else if (user == LVSYS_INTR) {
		low_bat_thl_data->lvsys_thl_intr_level = input;
		if (low_bat_thl_data->low_bat_thl_stop > 0 || low_bat_thl_data->ppb_mode > 0) {
			pr_info("[%s] low_bat_thl_stop=%d, ppb_mode=%d\n",
			__func__, low_bat_thl_data->low_bat_thl_stop,
			low_bat_thl_data->ppb_mode);
		} else {
			input = MAX(low_bat_thl_data->lbat_thl_intr_level,
						low_bat_thl_data->lvsys_thl_intr_level);
			exec_throttle(input);
		}
		mutex_unlock(&exe_thr_lock);
	} else if (user == PPB) {
		low_bat_thl_data->ppb_mode = input;
		if (low_bat_thl_data->low_bat_thl_stop > 0) {
			pr_info("[%s] ppb not apply, low_bat_thl_stop=%d\n", __func__,
				low_bat_thl_data->low_bat_thl_stop);
			mutex_unlock(&exe_thr_lock);
		} else if (low_bat_thl_data->ppb_mode > 0) {
			low_bat_thl_data->lbat_thl_intr_level = 0;
			exec_throttle(0);
			mutex_unlock(&exe_thr_lock);
			lbat_user_modify_thd_ext_locked(low_bat_thl_data->lbat_pt,
				&low_thd_volts[0], LOW_BATTERY_INIT_NUM);
			dump_thd_volts_ext(&low_thd_volts[0], LOW_BATTERY_INIT_NUM);
		} else {
			input = MAX(low_bat_thl_data->lbat_thl_intr_level,
						low_bat_thl_data->lvsys_thl_intr_level);
			exec_throttle(input);
			thd_info =
				&low_bat_thl_data->lbat_thd_info[low_bat_thl_data->temp_cur_stage];
			low_bat_thl_data->temp_reg_stage = low_bat_thl_data->temp_cur_stage;
			mutex_unlock(&exe_thr_lock);
			lbat_user_modify_thd_ext_locked(low_bat_thl_data->lbat_pt,
				thd_info->thd_volts, thd_info->thd_volts_size);
			dump_thd_volts_ext(thd_info->thd_volts, thd_info->thd_volts_size);
		}
	} else if (user == UT) {
		low_bat_thl_data->low_bat_thl_stop = 1;
		exec_throttle(input);
		mutex_unlock(&exe_thr_lock);
	} else {
		mutex_unlock(&exe_thr_lock);
	}
	return 0;
}

static unsigned int thd_to_level(unsigned int thd, unsigned int temp_stage)
{
	unsigned int i, level = 0;
	struct lbat_intr_tbl *info;
	struct lbat_thd_tbl *thd_info;

	if (!low_bat_thl_data)
		return 0;

	thd_info = &low_bat_thl_data->lbat_thd_info[temp_stage];

	for (i = 0; i < thd_info->thd_volts_size; i++) {
		info = &thd_info->lbat_intr_info[i];
		if (thd == thd_info->thd_volts[i]) {
			low_bat_thl_data->pmic_idx = i;
			if (info->ht_en == 1)
				level = info->ht_lv;
			else if (info->lt_en == 1)
				level = info->lt_lv;
			break;
		}
	}

	if (i == thd_info->thd_volts_size) {
		pr_notice("[%s] wrong threshold=%d\n", __func__, thd);
		return -1;
	}

	pr_info("[%s] temp_id=%d thd=%d cl=%d pl=%d, ht[%d %d] lt[%d %d] new_l=%d\n",
		__func__, temp_stage, thd, low_bat_thl_data->low_bat_thl_level, i,
		info->ht_en, info->ht_lv, info->lt_en, info->lt_lv, level);

	return level;
}

void exec_low_battery_callback(unsigned int thd)
{
	unsigned int level = 0;

	if (!low_bat_thl_data) {
		pr_info("[%s] low_bat_thl_data not allocate\n", __func__);
		return;
	}

	level = thd_to_level(thd, low_bat_thl_data->temp_reg_stage);
	if (level == -1)
		return;

	decide_and_throttle(LBAT_INTR, level);
}

void low_battery_vbat_callback(unsigned int thd)
{
	if (!low_bat_thl_data) {
		pr_info("[%s] low_bat_thl_data not allocate\n", __func__);
		return;
	}

	if (thd == low_bat_thl_data->hv_thd_volt)
		decide_and_throttle(LBAT_INTR, LOW_BATTERY_LEVEL_0);
	else if (thd == low_bat_thl_data->lv1_thd_volt)
		decide_and_throttle(LBAT_INTR, LOW_BATTERY_LEVEL_1);
	else if (thd == low_bat_thl_data->lv2_thd_volt)
		decide_and_throttle(LBAT_INTR, LOW_BATTERY_LEVEL_2);
	else
		pr_notice("[%s] wrong threshold=%d\n", __func__, thd);
}

int lbat_set_ppb_mode(unsigned int mode)
{
	decide_and_throttle(PPB, mode);
	return 0;
}
EXPORT_SYMBOL(lbat_set_ppb_mode);

/*****************************************************************************
 * low battery protect UT
 ******************************************************************************/
static ssize_t low_battery_protect_ut_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	dev_dbg(dev, "low_bat_thl_level=%d\n",
		low_bat_thl_data->low_bat_thl_level);
	return sprintf(buf, "%u\n", low_bat_thl_data->low_bat_thl_level);
}

static ssize_t low_battery_protect_ut_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int val = 0;
	char cmd[21];

	if (sscanf(buf, "%20s %u\n", cmd, &val) != 2) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}

	if (strncmp(cmd, "Utest", 5))
		return -EINVAL;

	if (val > low_bat_thl_data->max_throttle_level) {
		dev_info(dev, "wrong number (%d)\n", val);
		return size;
	}

	dev_info(dev, "your input is %d\n", val);
	decide_and_throttle(UT, val);

	return size;
}
static DEVICE_ATTR_RW(low_battery_protect_ut);

/*****************************************************************************
 * low battery protect stop
 ******************************************************************************/
static ssize_t low_battery_protect_stop_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	dev_dbg(dev, "low_bat_thl_stop=%d\n",
		low_bat_thl_data->low_bat_thl_stop);
	return sprintf(buf, "%u\n", low_bat_thl_data->low_bat_thl_stop);
}

static ssize_t low_battery_protect_stop_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int val = 0;
	char cmd[21];

	if (sscanf(buf, "%20s %u\n", cmd, &val) != 2) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}

	if (strncmp(cmd, "stop", 4))
		return -EINVAL;

	if ((val != 0) && (val != 1)) {
		dev_info(dev, "stop value not correct\n");
		return size;
	}

	low_bat_thl_data->low_bat_thl_stop = val;
	dev_info(dev, "low_bat_thl_stop=%d\n",
		 low_bat_thl_data->low_bat_thl_stop);
	return size;
}
static DEVICE_ATTR_RW(low_battery_protect_stop);

/*****************************************************************************
 * low battery protect level
 ******************************************************************************/
static ssize_t low_battery_protect_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	dev_dbg(dev, "low_bat_thl_level=%d\n",
		low_bat_thl_data->low_bat_thl_level);
	return sprintf(buf, "%u\n", low_bat_thl_data->low_bat_thl_level);
}

static ssize_t low_battery_protect_level_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	dev_dbg(dev, "low_bat_thl_level = %d\n",
		low_bat_thl_data->low_bat_thl_level);
	return size;
}

static DEVICE_ATTR_RW(low_battery_protect_level);

/*****************************************************************************
 * low battery modify threshold
 ******************************************************************************/
static ssize_t low_battery_modify_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lbat_thd_tbl *thd_info = &low_bat_thl_data->lbat_thd_info[0];

	return sprintf(buf, "modify enable: %d, %d %d %d %d\n",
			low_bat_thl_data->low_bat_thd_modify,
			thd_info->thd_volts[LOW_BATTERY_LEVEL_0],
			thd_info->thd_volts[LOW_BATTERY_LEVEL_1],
			thd_info->thd_volts[LOW_BATTERY_LEVEL_2],
			thd_info->thd_volts[LOW_BATTERY_LEVEL_3]);
}

static ssize_t low_battery_modify_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int volt_t_size = 0, j = 0;
	unsigned int thd_0, thd_1, thd_2, thd_3;
	unsigned int thd_level[LOW_BATTERY_INIT_NUM] = {0 ,1, 2, 3};
	unsigned int volt_l[3], volt_h[3];
	struct lbat_thd_tbl *thd_info = &low_bat_thl_data->lbat_thd_info[0];

	if (sscanf(buf, "%u %u %u %u\n", &thd_0, &thd_1, &thd_2, &thd_3) != 4) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}

	if (thd_0 <= 0 || thd_1 <= 0 || thd_2 <= 0 || thd_3 <= 0)
		return -EINVAL;

	volt_l[0] = thd_1;
	volt_l[1] = thd_2;
	volt_l[2] = thd_3;
	volt_h[0] = thd_0;
	volt_h[1] = thd_1;
	volt_h[2] = thd_2;

	volt_t_size = rearrange_volt(thd_info->lbat_intr_info, &volt_l[0], &volt_h[0], thd_level,
			LOW_BATTERY_INIT_NUM - 1);

	if (volt_t_size <= 0) {
		dev_notice(dev, "[%s] Failed to rearrange_volt\n", __func__);
		return -ENODATA;
	}

	thd_info->thd_volts_size = volt_t_size;
	for (j = 0; j < volt_t_size; j++)
		thd_info->thd_volts[j] = thd_info->lbat_intr_info[j].volt_thd;

	dump_thd_volts(dev, thd_info->thd_volts, thd_info->thd_volts_size);
	lbat_user_modify_thd_ext_locked(low_bat_thl_data->lbat_pt, thd_info->thd_volts,
								thd_info->thd_volts_size);
	low_bat_thl_data->low_bat_thd_modify = 1;
	low_bat_thl_data->temp_cur_stage = 0;
	low_bat_thl_data->temp_reg_stage = 0;
	dev_notice(dev, "modify_enable: %d, temp_cur_stage = %d\n",
				low_bat_thl_data->low_bat_thd_modify,
				low_bat_thl_data->temp_cur_stage);

	return size;
}
static DEVICE_ATTR_RW(low_battery_modify_threshold);

static int lbat_psy_event(struct notifier_block *nb, unsigned long event, void *v)
{
	if (!low_bat_thl_data)
		return NOTIFY_DONE;

	low_bat_thl_data->psy = v;
	schedule_work(&low_bat_thl_data->temp_work);
	return NOTIFY_DONE;
}

static int check_duplicate(unsigned int *volt_thd)
{
	int i, j;

	for (i = 0; i < LOW_BATTERY_INIT_NUM - 1; i++) {
		for (j = i + 1; j < LOW_BATTERY_INIT_NUM - 1; j++) {
			if (volt_thd[i] == volt_thd[j]) {
				pr_notice("[%s] volt_thd duplicate = %d\n", __func__, volt_thd[i]);
				return -1;
			}
		}
	}
	return 0;
}

static void temp_handler(struct work_struct *work)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret, temp, temp_stage, temp_thd;
	static int last_temp = MAX_INT;
	bool loop;
	struct lbat_thd_tbl *thd_info, *pre_thd_info;
	int pre_thd_info_level = 0, thd_info_level = 0;
	struct lbat_intr_tbl *pre_lbat_intr_info, *lbat_intr_info;

	if (!low_bat_thl_data)
		return;

	if (!low_bat_thl_data->psy)
		return;

	psy = low_bat_thl_data->psy;

	if (strcmp(psy->desc->name, "battery") != 0)
		return;

	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return;

	temp = val.intval / 10;
	temp_stage = low_bat_thl_data->temp_cur_stage;

	do {
		loop = false;
		if (temp < last_temp) {
			if (temp_stage < low_bat_thl_data->temp_max_stage) {
				temp_thd = low_bat_thl_data->temp_thd[temp_stage];
				if (temp < temp_thd) {
					temp_stage++;
					loop = true;
				}
			}
		} else if (temp > last_temp) {
			if (temp_stage > 0) {
				temp_thd = low_bat_thl_data->temp_thd[temp_stage-1];
				if (temp >= temp_thd) {
					temp_stage--;
					loop = true;
				}
			}
		}
	} while (loop);

	last_temp = temp;

	if (temp_stage <= low_bat_thl_data->temp_max_stage &&
		temp_stage != low_bat_thl_data->temp_cur_stage) {
		if (low_bat_thl_data->ppb_mode == 0 && !low_bat_thl_data->low_bat_thd_modify) {
			pre_thd_info =
				&low_bat_thl_data->lbat_thd_info[low_bat_thl_data->temp_cur_stage];
			thd_info = &low_bat_thl_data->lbat_thd_info[temp_stage];
			low_bat_thl_data->temp_reg_stage = temp_stage;

			pre_lbat_intr_info =
				&pre_thd_info->lbat_intr_info[low_bat_thl_data->pmic_idx];
			if (pre_lbat_intr_info->ht_en == 1)
				pre_thd_info_level = pre_lbat_intr_info->ht_lv;
			else if (pre_lbat_intr_info->lt_en == 1)
				pre_thd_info_level = pre_lbat_intr_info->lt_lv;

			lbat_intr_info = &thd_info->lbat_intr_info[low_bat_thl_data->pmic_idx];
			if (lbat_intr_info->ht_en == 1)
				thd_info_level = lbat_intr_info->ht_lv;
			else if (lbat_intr_info->lt_en == 1)
				thd_info_level = lbat_intr_info->lt_lv;

			if (pre_thd_info_level != thd_info_level)
				decide_and_throttle(TEMP_CHANGE, thd_info_level);

			lbat_user_modify_thd_ext_locked(low_bat_thl_data->lbat_pt,
				thd_info->thd_volts, thd_info->thd_volts_size);
			dump_thd_volts_ext(thd_info->thd_volts, thd_info->thd_volts_size);
		}
		low_bat_thl_data->temp_cur_stage = temp_stage;
	}
		pr_info("temp=%d, last_temp=%d temp_stage=%d, reg_stage=%d, cur_stage=%d\n",
			temp, last_temp, temp_stage, low_bat_thl_data->temp_reg_stage,
			low_bat_thl_data->temp_cur_stage);
}

static int lvsys_notifier_call(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct lbat_thd_tbl *thd_info;
	int level = 0;

	if (!low_bat_thl_data)
		return NOTIFY_DONE;

	event = event & ~(1 << 15);

	if (event == low_bat_thl_data->lvsys_thd_volt_l) {
		thd_info = &low_bat_thl_data->lbat_thd_info[low_bat_thl_data->temp_reg_stage];
		level = thd_info->lbat_intr_info[thd_info->thd_volts_size - 1].lt_lv;
		decide_and_throttle(LVSYS_INTR, level);
	} else if (event == low_bat_thl_data->lvsys_thd_volt_h)
		decide_and_throttle(LVSYS_INTR, LOW_BATTERY_LEVEL_0);
	else
		pr_notice("[%s] wrong lvsys thd = %lu\n", __func__, event);

	return NOTIFY_DONE;
}

static struct notifier_block lvsys_notifier = {
	.notifier_call = lvsys_notifier_call,
};

static int lbat_default_setting(struct platform_device *pdev, struct low_bat_thl_priv *priv)
{
	struct lbat_thd_tbl *thd_info = &priv->lbat_thd_info[0];

	dev_notice(&pdev->dev, "[%s] use lbat default setting\n", __func__);
	priv->temp_max_stage = 0;
	thd_info->thd_volts_size = LOW_BATTERY_INIT_NUM;
	thd_info->thd_volts = devm_kmalloc_array(&pdev->dev, thd_info->thd_volts_size, sizeof(u32),
		GFP_KERNEL);

	if (!thd_info->thd_volts)
		return -ENOMEM;

	thd_info->thd_volts[0] = POWER_INT0_VOLT;
	thd_info->thd_volts[1] = POWER_INT1_VOLT;
	thd_info->thd_volts[2] = POWER_INT2_VOLT;
	thd_info->thd_volts[3] = POWER_INT3_VOLT;

	return 0;
}

static int low_battery_thd_setting(struct platform_device *pdev, struct low_bat_thl_priv *priv)
{
	struct device_node *np = pdev->dev.of_node;
	char thd_volts_l[THD_VOLTS_LENGTH] = "thd-volts-l";
	char thd_volts_h[THD_VOLTS_LENGTH] = "thd-volts-h";
	unsigned int volt_l[LOW_BATTERY_INIT_NUM-1], volt_h[LOW_BATTERY_INIT_NUM-1];
	int ret = 0, bat_type = 0;
	int num, i, j, max_thr_lv, volt_t_size;
	u32 *volt_thd, *thd_level;
	struct device_node *gauge_np = pdev->dev.parent->of_node;
	struct lbat_thd_tbl *thd_info;
	int last_volt_t_size = 0;

	ret = of_property_read_u32(np, "temperature-max-stage", &priv->temp_max_stage);
	num = of_property_count_u32_elems(np, "temperature-stage-threshold");
	if (ret || num != priv->temp_max_stage) {
		pr_notice("get temperature stage error %d, use 0\n", ret);
		priv->temp_max_stage = 0;
	}

	if (priv->temp_max_stage > 0) {
		priv->temp_thd = devm_kmalloc_array(&pdev->dev, priv->temp_max_stage, sizeof(u32),
			GFP_KERNEL);
		if (!priv->temp_thd)
			return -ENOMEM;
		ret = of_property_read_u32_array(np, "temperature-stage-threshold", priv->temp_thd,
			num);
		if (ret) {
			pr_notice("get temperature-stage-threshold error %d, set stage=0\n", ret);
			priv->temp_max_stage = 0;
		}
	}

	gauge_np = of_find_node_by_name(gauge_np, "mtk-gauge");
	if (!gauge_np)
		pr_notice("[%s] get mtk-gauge node fail\n", __func__);
	else {
		ret = of_property_read_u32(gauge_np, "bat_type", &bat_type);
		if (ret)
			dev_notice(&pdev->dev, "[%s] get bat_type fail\n", __func__);

		if (bat_type > 0 && bat_type < 10) {
			ret = snprintf(thd_volts_l, THD_VOLTS_LENGTH, "thd-volts-l-%ds",
				bat_type + 1);
			if (ret < 0)
				pr_info("%s:%d: snprintf error %d\n", __func__, __LINE__, ret);

			ret = snprintf(thd_volts_h, THD_VOLTS_LENGTH, "thd-volts-h-%ds",
				bat_type + 1);
			if (ret < 0)
				pr_info("%s:%d: snprintf error %d\n", __func__, __LINE__, ret);
		}
	}
	dev_notice(&pdev->dev, "[%s] bat_type = %d\n", __func__, bat_type);

	num = of_property_count_elems_of_size(np, thd_volts_l, sizeof(u32));
	if (num != (LOW_BATTERY_INIT_NUM -1) * (priv->temp_max_stage + 1)) {
		dev_notice(&pdev->dev, "[%s] wrong thd-volts-l size %d\n", __func__, num);
		goto default_setting;
	}

	num = of_property_count_elems_of_size(np, thd_volts_h, sizeof(u32));
	if (num != (LOW_BATTERY_INIT_NUM - 1) * (priv->temp_max_stage + 1)) {
		dev_notice(&pdev->dev, "[%s] wrong thd-volts-h size %d\n", __func__, num);
		goto default_setting;
	}

	volt_thd = devm_kmalloc_array(&pdev->dev, num * 2, sizeof(u32), GFP_KERNEL);
	if (!volt_thd)
		return -ENOMEM;

	of_property_read_u32_array(np, thd_volts_l, &volt_thd[0], num);
	of_property_read_u32_array(np, thd_volts_h, &volt_thd[num], num);

	ret = of_property_read_u32(np, "max-throttle-level", &priv->max_throttle_level);
	if (ret)
		dev_notice(&pdev->dev, "[%s] wrong max-throttle-level %d\n", __func__, num);

	thd_level = devm_kmalloc_array(&pdev->dev,
		LOW_BATTERY_INIT_NUM * (priv->temp_max_stage + 1), sizeof(u32), GFP_KERNEL);
	if (!thd_level)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, "lbat-throttle-level", &thd_level[0],
		LOW_BATTERY_INIT_NUM * (priv->temp_max_stage + 1));
	if (ret) {
		dev_notice(&pdev->dev, "[%s] wrong lbat-throttle-level %d\n", __func__, num);
	}

	for (i = 0; i <= priv->temp_max_stage; i++) {
		max_thr_lv = LOW_BATTERY_INIT_NUM - 1;
		for (j = 0; j < max_thr_lv; j++) {
			volt_l[j] = volt_thd[i * max_thr_lv + j];
			volt_h[j] = volt_thd[num + i * max_thr_lv + j];
		}

		ret = check_duplicate(volt_l);
		ret |= check_duplicate(volt_h);

		if (ret < 0) {
			dev_notice(&pdev->dev, "[%s] check duplicate error, %d\n", __func__, ret);
			goto default_setting;
			return -EINVAL;
		}

		thd_info = &priv->lbat_thd_info[i];
		thd_info->thd_level_size = LOW_BATTERY_INIT_NUM;
		thd_info->thd_level = devm_kmalloc_array(&pdev->dev, thd_info->thd_level_size,
			sizeof(u32), GFP_KERNEL);
		memcpy(thd_info->thd_level, &thd_level[i * thd_info->thd_level_size],
			sizeof(u32) * thd_info->thd_level_size);
		thd_info->thd_volts_size = max_thr_lv * 2;
		thd_info->lbat_intr_info = devm_kmalloc_array(&pdev->dev, thd_info->thd_volts_size,
			sizeof(struct lbat_thd_tbl), GFP_KERNEL);

		volt_t_size = rearrange_volt(thd_info->lbat_intr_info, &volt_l[0], &volt_h[0],
			thd_info->thd_level, max_thr_lv);

		if (volt_t_size <= 0) {
			dev_notice(&pdev->dev, "[%s] Failed to rearrange_volt\n", __func__);
			return -ENODATA;
		}

		if (i != 0 && last_volt_t_size != volt_t_size) {
			dev_notice(&pdev->dev,
				"[%s] volt size should be the same, force trigger kernel panic\n",
				__func__);
			BUG_ON(1);
		}
		last_volt_t_size = volt_t_size;

		thd_info->thd_volts_size = volt_t_size;
		thd_info->thd_volts = devm_kmalloc_array(&pdev->dev, thd_info->thd_volts_size,
			sizeof(u32), GFP_KERNEL);

		if (!thd_info->thd_volts)
			return -ENOMEM;

		for (j = 0; j < volt_t_size; j++)
			thd_info->thd_volts[j] = thd_info->lbat_intr_info[j].volt_thd;

		dump_thd_volts(&pdev->dev, thd_info->thd_volts, thd_info->thd_volts_size);
	}

	return 0;

default_setting:
	lbat_default_setting(pdev, priv);
	return 0;
}

static int low_battery_register_setting(struct platform_device *pdev,
		struct low_bat_thl_priv *priv, unsigned int temp_stage)
{
	int ret;
	struct lbat_thd_tbl *thd_info = &priv->lbat_thd_info[temp_stage];

	priv->lbat_pt = lbat_user_register_ext("power throttling", thd_info->thd_volts,
						thd_info->thd_volts_size, exec_low_battery_callback);

	if (IS_ERR(priv->lbat_pt)) {
		ret = PTR_ERR(priv->lbat_pt);
		if (ret != -EPROBE_DEFER) {
			dev_notice(&pdev->dev, "[%s] error ret=%d\n", __func__, ret);
		}
		return ret;
	}
	dev_notice(&pdev->dev, "[%s] register type = %d done\n", __func__, temp_stage);

	return 0;
}

static int low_battery_throttling_probe(struct platform_device *pdev)
{
	int ret, num;
	int lvsys_thd_enable, vbat_thd_enable;
	struct low_bat_thl_priv *priv;
	struct device_node *np = pdev->dev.of_node;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(&pdev->dev, priv);

	INIT_WORK(&priv->temp_work, temp_handler);

	ret = of_property_read_u32(np, "lvsys-thd-enable", &lvsys_thd_enable);
	if (ret) {
		dev_notice(&pdev->dev,
			"[%s] failed to get lvsys-thd-enable ret=%d\n", __func__, ret);
		lvsys_thd_enable = 0;
	}

	ret = of_property_read_u32(np, "vbat-thd-enable", &vbat_thd_enable);
	if (ret) {
		dev_notice(&pdev->dev,
			"[%s] failed to get vbat-thd-enable ret=%d\n", __func__, ret);
		vbat_thd_enable = 1;
	}

	if (vbat_thd_enable) {
		num = of_property_count_elems_of_size(np, "thd-volts-l", sizeof(u32));
		if (num <= 0) {
			priv->isThreeLevel = 0;
			ret = of_property_read_u32(np, "hv-thd-volt", &priv->hv_thd_volt);
			if (ret)
				priv->hv_thd_volt = POWER_INT0_VOLT;

			ret = of_property_read_u32(np, "lv1-thd-volt", &priv->lv1_thd_volt);
			if (ret)
				priv->lv1_thd_volt = POWER_INT1_VOLT;

			ret = of_property_read_u32(np, "lv2-thd-volt", &priv->lv2_thd_volt);
			if (ret)
				priv->lv2_thd_volt = POWER_INT2_VOLT;

			lbat_user_register("power throttling", priv->hv_thd_volt,
				priv->lv1_thd_volt, priv->lv2_thd_volt, low_battery_vbat_callback);
			priv->max_throttle_level = LOW_BATTERY_LEVEL_2;
		} else {
			priv->isThreeLevel = 1;
			ret = low_battery_thd_setting(pdev, priv);
			if (ret) {
					pr_info("[%s] thd_setting error, ret=%d\n", __func__,ret);
				return ret;
			}

			ret = low_battery_register_setting(pdev, priv, 0);
			if (ret) {
				pr_info("[%s] low_battery_register_setting failed, ret=%d\n",
					__func__, ret);
				return ret;
			}

			if (priv->temp_max_stage > 0) {
				lbat_nb.notifier_call = lbat_psy_event;
				ret = power_supply_reg_notifier(&lbat_nb);
				if (ret) {
					dev_notice(&pdev->dev, "[%s] power_supply_reg fail\n",
						__func__);
					return ret;
				}
			}
		}
	}

	if (lvsys_thd_enable) {
		ret = of_property_read_u32(np, "lvsys-thd-volt-l", &priv->lvsys_thd_volt_l);
		if (ret) {
			dev_notice(&pdev->dev,
				"[%s] failed to get lvsys-thd-volt-l ret=%d\n", __func__, ret);
			priv->lvsys_thd_volt_l = LVSYS_THD_VOLT_L;
		}

		ret = of_property_read_u32(np, "lvsys-thd-volt-h", &priv->lvsys_thd_volt_h);
		if (ret) {
			dev_notice(&pdev->dev,
				"[%s] failed to get lvsys-thd-volt-h ret=%d\n", __func__, ret);
			priv->lvsys_thd_volt_h = LVSYS_THD_VOLT_H;
		}

		dev_notice(&pdev->dev, "lvsys_register: %d mV, %d mV\n",
			priv->lvsys_thd_volt_l, priv->lvsys_thd_volt_h);

		ret = lvsys_register_notifier(&lvsys_notifier);
		if (ret)
			dev_notice(&pdev->dev, "lvsys_register_notifier error ret=%d\n", ret);
	}

	ret = device_create_file(&(pdev->dev),
		&dev_attr_low_battery_protect_ut);
	ret |= device_create_file(&(pdev->dev),
		&dev_attr_low_battery_protect_stop);
	ret |= device_create_file(&(pdev->dev),
		&dev_attr_low_battery_protect_level);
	if (priv->isThreeLevel) {
		ret |= device_create_file(&(pdev->dev),
		&dev_attr_low_battery_modify_threshold);
	}

	if (ret) {
		dev_notice(&pdev->dev, "create file error ret=%d\n", ret);
		return ret;
	}

	low_bat_thl_data = priv;
	return 0;
}

static const struct of_device_id low_bat_thl_of_match[] = {
	{ .compatible = "mediatek,low_battery_throttling", },
	{ },
};
MODULE_DEVICE_TABLE(of, low_bat_thl_of_match);

static struct platform_driver low_battery_throttling_driver = {
	.driver = {
		.name = "low_battery_throttling",
		.of_match_table = low_bat_thl_of_match,
	},
	.probe = low_battery_throttling_probe,
};

module_platform_driver(low_battery_throttling_driver);
MODULE_AUTHOR("Jeter Chen <Jeter.Chen@mediatek.com>");
MODULE_DESCRIPTION("MTK low battery throttling driver");
MODULE_LICENSE("GPL");
