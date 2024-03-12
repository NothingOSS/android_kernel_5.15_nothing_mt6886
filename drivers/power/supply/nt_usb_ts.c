/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <tcpm.h>

#include "charger_class.h"
#include "mtk_charger.h"
#include "nt_usb_ts.h"
#include "../../misc/mediatek/extcon/extcon-mtk-usb.h"

#define NT_USB_TS_DRV_VERSION	"1.0.0_NT"

#define USB_TS_0C	0
#define USB_TS_10C	10
#define USB_TS_20C	20
#define USB_TS_30C	30
#define USB_TS_40C	40
#define USB_TS_57C	57
#define USB_TS_50C	50
#define USB_TS_55C	55
#define USB_TS_100C	100
#define HIGH_TEMPERATURE_TH	53
#define NORMAL_SLEEP_TIME	1500 //1.5s
#define MIN_SLEEP_TIME	100 //ms
#define MAX_SLEEP_TIME	100 //ms
#define RETRY_SLEEP_TIME         5  //ms
#define RETRY_COUNT		3
#define USB_TS_NORMAL_VOL_LT	100000
#define USB_TS_NORMAL_VOL_HT	1800000
#define USB_TS_NORMAL_TEMP_TH	40
#define	USB_TS_BATTERY_TEMP_DIFF	3
#define	USB_TS_BATTERY_TEMP_GAP	12
#define	USB_TS_TEMP_INC_DIFF	3
#define BATTERY_NORMAL_TEMP_HT	60
#define BATTERY_NORMAL_TEMP_LT	5
#define LOG_PRINT_TIME	30

extern nt_board_id;
static struct mtk_usb_ts_info *pts_info = NULL;
static int timeout_count = 13; //1.5s count

struct mtk_usb_ts_info {
	struct platform_device *pdev;
	struct device *dev;
	struct usb_temp_parameter temp_parameter;
	wait_queue_head_t wait_que;
	int usb_temp;
	int usb_pre_temp;
	unsigned int usb_ts_voltage;
	int bat_temp;
	int usb_ts_state;
	int vbus_volt;
	bool is_usb_in;
	struct ts_temp *tstable;
	struct charger_device *chg1_dev;
	atomic_t charger_state;
	struct mutex usb_ts_lock;
	struct gpio_desc *usb_ts_ctrl;
	bool is_need_disable_usb;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	struct power_supply *bat_psy;
	struct power_supply *chg_psy;
#if IS_ENABLED(CONFIG_PRIZE_BOARD_ID)
	int board_id;
#endif
	int chr_type;
	bool usb_ts_debug;
	bool is_fast_charge;
	bool is_aging_mode;
	bool is_otg_online;
	bool is_cool_down;
};

#define ABNORMAL_VOL_H	1800000
#define ABNORMAL_VOL_L	40000
static int voltage_to_temperature(struct mtk_usb_ts_info *ts_info, int voltage)
{
	int i = 0;
	int ts_temp = 0;
	struct ts_temp *tstable;
	int vot1 = 0, vot2 = 0, tmp1 = 0, tmp2 = 0;

	if (pts_info->board_id == -1) {
		goto out;
	}
	if (ts_info->board_id <= 2) {
		/* preT0 & T0_1 & T0_2*/
		ts_info->tstable = ts_temp_table_100k;
	} else {
		ts_info->tstable = ts_temp_table_10k;
	}
	tstable = ts_info->tstable;

	if (voltage > ABNORMAL_VOL_H || voltage < ABNORMAL_VOL_L) {
		ts_temp = 0;
	} else if (voltage >= tstable[0].TemperatureVolt) {
		ts_temp = -30;
	} else if (voltage <= tstable[155].TemperatureVolt) {
		ts_temp = 125;
	} else {
		vot1 = tstable[0].TemperatureVolt;
		tmp1 = tstable[0].BatteryTemp;
		for (i = 0; i <= 155; i++) {
			if (voltage >= tstable[i].TemperatureVolt) {
				//ts_temp = tstable[i].BatteryTemp;
				vot2 = tstable[i].TemperatureVolt;
				tmp2 = tstable[i].BatteryTemp;
				break;
			}
			{	/* hidden else */
				vot1 = tstable[i].TemperatureVolt;
				tmp1 = tstable[i].BatteryTemp;
			}
		}
		ts_temp = (((voltage - vot2) * tmp1) +
			((vot1 - voltage) * tmp2)) / (vot1 - vot2);
	}
	//chr_debug("%s, ts_temp:%d, volt:%d, vot1:%d, vot2:%d, tmp1:%d, tmp2:%d\n",
	//			__func__, ts_temp, voltage, vot1, vot2, tmp1, tmp2);
	return ts_temp;

out:
	return 25;
}

static int get_usb_ts_voltage(int *volt)
{
	int ret = 0;
	int ts_volt = 0;
	struct charger_device *chg1_dev;

	if (pts_info->chg1_dev == NULL) {
		chg1_dev = get_charger_by_name("primary_chg");
		if (chg1_dev != NULL) {
			chr_err("%s, 1 Found primary charger\n", __func__);
			ret = charger_dev_get_ts(chg1_dev, &ts_volt);
		}
	} else {
		ret = charger_dev_get_ts(pts_info->chg1_dev, &ts_volt);
	}

	if (ret < 0) {
		chr_err("%s: get ts volt failed: %d\n", __func__, ret);
	} else {
		pts_info->usb_ts_voltage = ts_volt;
		*volt = ts_volt;
	}

	return ret;
}

bool get_usb_otg_status(void)
{
	return pts_info->is_otg_online;
}
EXPORT_SYMBOL(get_usb_otg_status);

struct usb_temp_parameter *get_usb_temp_parameter(void)
{
	return &pts_info->temp_parameter;
}
EXPORT_SYMBOL(get_usb_temp_parameter);

void set_usb_temp_parameter(struct usb_temp_parameter *temp_parameter)
{
	pts_info->temp_parameter.enable =
		temp_parameter->enable;
	pts_info->temp_parameter.sleep_ms =
		temp_parameter->sleep_ms;
	pts_info->temp_parameter.usb_temp =
		temp_parameter->usb_temp;
	pts_info->temp_parameter.usb_battery_gap_temp =
		temp_parameter->usb_battery_gap_temp;
	pts_info->temp_parameter.usb_temp_raise_time =
		temp_parameter->usb_temp_raise_time;
	pts_info->temp_parameter.usb_temp_raise_degree =
		temp_parameter->usb_temp_raise_degree;
	pts_info->temp_parameter.usb_temp_print_log_enable =
		temp_parameter->usb_temp_print_log_enable;
	timeout_count = 1500 / (temp_parameter->sleep_ms + 20); //1.5s count
	chr_debug("%s:en:%d,sleep:%d ms,usb_t:%d,gap_t:%d,raiseTime:%d,raiseDegree:%d,log_en:%d,ct:%d\n",
			__func__,
			pts_info->temp_parameter.enable,
			pts_info->temp_parameter.sleep_ms,
			pts_info->temp_parameter.usb_temp,
			pts_info->temp_parameter.usb_battery_gap_temp,
			pts_info->temp_parameter.usb_temp_raise_time,
			pts_info->temp_parameter.usb_temp_raise_degree,
			pts_info->temp_parameter.usb_temp_print_log_enable,
			timeout_count);
}
EXPORT_SYMBOL(set_usb_temp_parameter);

void usb_temp_parameter_init(struct mtk_usb_ts_info *ts_info)
{
	ts_info->temp_parameter.enable = 1;
	ts_info->temp_parameter.sleep_ms = MIN_SLEEP_TIME;
	ts_info->temp_parameter.usb_temp = HIGH_TEMPERATURE_TH;
	ts_info->temp_parameter.usb_battery_gap_temp = USB_TS_BATTERY_TEMP_GAP;
	ts_info->temp_parameter.usb_temp_raise_time = RETRY_COUNT;
	ts_info->temp_parameter.usb_temp_raise_degree = USB_TS_TEMP_INC_DIFF;
	ts_info->temp_parameter.usb_temp_print_log_enable = 0;
}

static int get_usb_ts_temperature(void)
{
	int temp = 0;
	int volt = 0;
	int ret = 0;

	if (pts_info->usb_ts_debug == true) {
		return pts_info->usb_temp;
	}

	if (pts_info->is_aging_mode == true) {
		return 25;
	}

	ret = get_usb_ts_voltage(&volt);
	if (!ret)
		temp = voltage_to_temperature(pts_info, volt);
	else
		temp = 25;

	if (pts_info->board_id == -1) {
		if(nt_board_id != -1) {
			temp = voltage_to_temperature(pts_info, volt);
			pts_info->usb_pre_temp = temp;
			pts_info->board_id = nt_board_id;
			chr_err("%s: board id update: %d\n", __func__, nt_board_id);
		}
	}

	return temp;
}

void set_usb_temperature(int temp)
{
	if (temp != 0xfefe) {
		pts_info->usb_ts_debug = true;
		pts_info->usb_temp = temp;
	} else {
		pts_info->usb_ts_debug = false;
	}
}
EXPORT_SYMBOL(set_usb_temperature);

int get_usb_temperature(void) {
	if ((pts_info->is_usb_in == false) &&
		(pts_info->usb_ts_state == USB_TS_TEMP_NORMAL)) {
		pts_info->usb_temp = get_usb_ts_temperature();
	}

	return pts_info->usb_temp;
}
EXPORT_SYMBOL(get_usb_temperature);

int get_usb_ts_check_state(void) {
	return pts_info->usb_ts_state;
}
EXPORT_SYMBOL(get_usb_ts_check_state);

static void wake_up_usb_ts_check(struct mtk_usb_ts_info *ts_info, bool en)
{
	if (en) {
		atomic_inc(&ts_info->charger_state);
	} else {
		atomic_set(&ts_info->charger_state, 0);
	}
	wake_up_interruptible(&ts_info->wait_que);
	chr_err("set usb check state:%d\n", ts_info->charger_state);
}

static int usb_ts_get_battery_temperature(struct mtk_usb_ts_info *ts_info, int *temp)
{
	struct power_supply *bat_psy = NULL;
	int ret = 0;
	union  power_supply_propval data;

	if (pts_info->bat_psy == NULL) {
		bat_psy = power_supply_get_by_name("battery");
		if (!IS_ERR_OR_NULL(bat_psy)) {
			ret = power_supply_get_property(bat_psy,
					POWER_SUPPLY_PROP_TEMP, &data);
			if (ret < 0)
				chr_err("get temp data fail\n");
			else
				*temp = data.intval / 10;
		}
	} else {
		ret = power_supply_get_property(pts_info->bat_psy,
					POWER_SUPPLY_PROP_TEMP, &data);
		if (ret < 0)
			chr_err("get temp data fail %d\n", ret);
		else
			*temp = data.intval / 10;
	}

	return ret;
}

int usb_ts_get_charger_type(struct mtk_usb_ts_info *ts_info)
{
	struct mtk_charger *info = NULL;

	if (ts_info->chg_psy == NULL)
		ts_info->chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (ts_info->chg_psy == NULL || IS_ERR(ts_info->chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	} else {
		info = (struct mtk_charger *)power_supply_get_drvdata(ts_info->chg_psy);
		if (info == NULL) {
			chr_err("%s Couldn't get chr_type\n", __func__);
			return -EINVAL;
		}
	}

	ts_info->chr_type = info->chr_type;
	chr_err("%s type:%d\n", __func__, ts_info->chr_type);

	return 0;
}

static int usb_ts_get_pmic_vbus(int *vchr)
{
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	int ret;

	if (chg_psy == NULL)
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	if (chg_psy == NULL) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		ret = -EINVAL;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	}
	*vchr = prop.intval;

	chr_debug("%s vbus:%d\n", __func__, prop.intval);
	return ret;
}

static int usb_ts_get_charger_vbus(struct mtk_usb_ts_info *ts_info)
{
	int ret = 0;
	int vchr = 0;

	if (ts_info->chg1_dev == NULL)
		ts_info->chg1_dev = get_charger_by_name("primary_chg");

	if (ts_info->chg1_dev != NULL) {
		ret = charger_dev_get_vbus(ts_info->chg1_dev, &vchr);
		if (ret < 0) {
			ret = usb_ts_get_pmic_vbus(&vchr);
			if (ret < 0) {
				chr_err("%s: get vbus failed: %d\n", __func__, ret);
				return -EINVAL;
			}
		}
		ts_info->vbus_volt = vchr / 1000;
		//chr_debug("%s: vbus voltage:%d\n", __func__, ts_info->vbus_volt);
	} else {
		chr_err("%s: Error : can't find primary charger\n", __func__);
		return -EINVAL;
	}

	return ret;
}

static void check_usb_ts_state(struct mtk_usb_ts_info *ts_info)
{
	static int check_count = 0;
	int count = 0;
	int i = 0;

	/* condition 1: the usb temp is higher than USB_TS_53C */
	if ((ts_info->bat_temp < USB_TS_50C) &&
			(ts_info->usb_temp >= ts_info->temp_parameter.usb_temp) && (ts_info->usb_temp < USB_TS_100C)) {
		for (i = 0; i < RETRY_COUNT; i++) {
			mdelay(RETRY_SLEEP_TIME);
			ts_info->usb_temp = get_usb_ts_temperature();
			if ((ts_info->usb_temp >= ts_info->temp_parameter.usb_temp) && (ts_info->usb_temp < USB_TS_100C) ) {
				count ++;
				pr_err("condition 1:1, count : %d", count);
			}
		}
		if (count >= RETRY_COUNT) {
			ts_info->usb_ts_state = USB_TS_HIGH_TEMP;
		}
		count = 0;
	} else if ((ts_info->bat_temp >= USB_TS_50C) && (ts_info->usb_temp < USB_TS_100C) &&
			((ts_info->usb_temp - ts_info->bat_temp) > (ts_info->temp_parameter.usb_temp_raise_degree - USB_TS_50C))) {
		for (i = 0; i < ts_info->temp_parameter.usb_temp_raise_time; i++) {
			mdelay(RETRY_SLEEP_TIME);
			ts_info->usb_temp = get_usb_ts_temperature();
			if ((ts_info->usb_temp < USB_TS_100C) &&
				((ts_info->usb_temp - ts_info->bat_temp) > ts_info->temp_parameter.usb_temp_raise_degree)) {
				count ++;
				pr_err("condition 1:2, count : %d", count);
			}
		}
		if (count >= ts_info->temp_parameter.usb_temp_raise_time) {
			ts_info->usb_ts_state = USB_TS_BATT_TEMP_ABNORMAL;
		}
		count = 0;
	/* condition 2: the usb temp uprising too fast */
	} else if ((ts_info->chr_type != POWER_SUPPLY_TYPE_UNKNOWN) && (ts_info->vbus_volt <= 5500) &&
			(ts_info->usb_temp < USB_TS_100C) && (ts_info->usb_temp > ts_info->bat_temp) &&
				(ts_info->usb_temp - ts_info->bat_temp > ts_info->temp_parameter.usb_battery_gap_temp)) {
		if (ts_info->usb_temp - ts_info->usb_pre_temp > ts_info->temp_parameter.usb_temp_raise_degree) {
			for (i = 0; i < ts_info->temp_parameter.usb_temp_raise_time; i++) {
				mdelay(RETRY_SLEEP_TIME);
				ts_info->usb_temp = get_usb_ts_temperature();
				if ((ts_info->usb_temp < USB_TS_100C) &&
					(ts_info->usb_temp - ts_info->usb_pre_temp > ts_info->temp_parameter.usb_temp_raise_degree)) {
					count ++;
					pr_err("condition 2, count : %d", count);
				}
			}
			if ((count >= ts_info->temp_parameter.usb_temp_raise_time) && (ts_info->usb_temp > USB_TS_30C)
									&&(ts_info->usb_temp < USB_TS_100C)) {
				ts_info->usb_ts_state = USB_TS_TEMP_INCREASE_FAST;
				ts_info->is_cool_down = false;
				pr_err(" usb_pre_temp: %d, usb_temp: %d", ts_info->usb_pre_temp, ts_info->usb_temp);
			}
		}
		count = 0;
	}
	count = 0;
	check_count++;
	if(check_count >= timeout_count) {
		ts_info->usb_pre_temp = ts_info->usb_temp;
		check_count = 0;
		chr_debug(" usb_pre_temp: %d", ts_info->usb_pre_temp);
	}
	//pr_err("usb ts state : %d", ts_info->usb_ts_state);
}

static bool is_temperature_normal(struct mtk_usb_ts_info *ts_info)
{
	int bat_temperature = 0;

	usb_ts_get_battery_temperature(ts_info, &bat_temperature);
	ts_info->bat_temp = bat_temperature;
	if ((bat_temperature > BATTERY_NORMAL_TEMP_LT)
		&& (bat_temperature < BATTERY_NORMAL_TEMP_HT)) {
		return true;
	} else {
		return false;
	}
}

static int check_disable_fast_charging_state(struct mtk_usb_ts_info *ts_info)
{
	int i = 0, ret = 0;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;
	struct mtk_charger *info = NULL;

	if (ts_info->chg_psy == NULL || IS_ERR(ts_info->chg_psy))
	{
		ts_info->chg_psy = power_supply_get_by_name("mtk-master-charger");
		if (ts_info->chg_psy == NULL || IS_ERR(ts_info->chg_psy)) {
			chr_err("%s Couldn't get chg_psy\n", __func__);
			return -EINVAL;
		}
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(ts_info->chg_psy);
	if (info == NULL) {
		chr_err("%s Couldn't get chg_info\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < MAX_ALG_NO; i++) {
		alg = info->alg[i];
		if (alg == NULL)
			continue;

		ret = chg_alg_is_algo_ready(alg);
		if ((ret == ALG_RUNNING) || chg_alg_is_algo_running(alg)) {
			is_fast_charge = true;
			break;
		}
	}
	ts_info->is_fast_charge = is_fast_charge;
	chr_err("%s: ret=%d, idx=%d, detect = %d\n", ret, __func__, i, is_fast_charge);

	/* if usb ts abnormal, fast charging disable*/
	if (is_fast_charge && (!ts_info->is_aging_mode) &&
				(ts_info->usb_ts_state != USB_TS_TEMP_NORMAL)) {
		chg_alg_stop_algo(alg);
		msleep(200);
		chr_err("%s Stop fast charging\n", __func__);
	}

	return ret;
}

static int usb_ts_ctrl_power_off(void)
{
	int status = 0;
	pts_info->usb_ts_ctrl = devm_gpiod_get(pts_info->dev, "usb_ts_ctrl",
				   GPIOD_OUT_HIGH);
	if (IS_ERR(pts_info->usb_ts_ctrl)) {
		status = PTR_ERR(pts_info->usb_ts_ctrl);
		chr_err("ERROR!! Failed to enable gpio: %d\n", status);
		return status;
	}
	gpiod_set_value(pts_info->usb_ts_ctrl, 1);
	devm_gpiod_put(pts_info->dev, pts_info->usb_ts_ctrl);
	chr_info("set usb vbus off successful!\n");

	return status;
}

static int  usb_ts_ctrl_power_on(void)
{
	int status = 0;

	pts_info->usb_ts_ctrl = devm_gpiod_get(pts_info->dev, "usb_ts_ctrl",
				   GPIOD_OUT_HIGH);
	if (IS_ERR(pts_info->usb_ts_ctrl)) {
		status = PTR_ERR(pts_info->usb_ts_ctrl);
		chr_err("ERROR!! Failed to enable gpio: %d\n", status);
		return status;
	}
	gpiod_set_value(pts_info->usb_ts_ctrl, 0);
	devm_gpiod_put(pts_info->dev, pts_info->usb_ts_ctrl);
	chr_info("set usb vbus on successful!\n");

	return status;

}

static void usb_ts_chg_suspend(struct mtk_usb_ts_info *ts_info, bool en)
{
	if (ts_info->chg1_dev == NULL)
		ts_info->chg1_dev = get_charger_by_name("primary_chg");

	if (ts_info->chg1_dev != NULL) {
		charger_dev_enable_hz(ts_info->chg1_dev, en);
	} else {
		chr_err("%s Failed to get primary_chg\n", __func__);
	}
}

static void usb_ts_dischg_action(struct mtk_usb_ts_info *ts_info)
{
	int ret = 0;

	if ((ts_info->is_need_disable_usb == false) && (!ts_info->is_aging_mode)) {
		ret = usb_ts_ctrl_power_off();
		if (ret) {
			chr_err("set usb ts ctrl enable failed\n");
		} else {
			ts_info->is_need_disable_usb = true;
			chr_err("set usb ts ctrl enable successfully\n");
		}
	} else {
		chr_err("%s, Aging mode do not set vbus down, %d,%d\n", __func__,
			ts_info->is_aging_mode, ts_info->is_need_disable_usb);
	}
}

static void usb_ts_enchg_action(struct mtk_usb_ts_info *ts_info)
{
	int ret = 0;

	if (!ts_info->is_aging_mode) {
		usb_ts_chg_suspend(ts_info, false);
		ret = usb_ts_ctrl_power_on();
		if (ret) {
			chr_err("set usb ts ctrl disable failed\n");
		} else {
			ts_info->is_need_disable_usb = false;
			ts_info->usb_ts_state = USB_TS_TEMP_NORMAL;
			if (ts_info->is_usb_in == false)
				wake_up_usb_ts_check(ts_info, false);

			chr_err("set usb ts ctrl disable successfully\n");
		}
	} else {
		chr_err("%s, Aging mode, do not need set vbus on\n", __func__);
	}
}

#define DEBUG_CHECK	0
#define CHECK_TEMP_COUNT	60
#define CHECK_TS_BATT_DIFF	3
static void usb_ts_recover_func(struct mtk_usb_ts_info *ts_info)
{
	int count_num = 0;

	if (ts_info->usb_ts_state == USB_TS_TEMP_NORMAL)
		return;

	if ((ts_info->is_need_disable_usb == true) &&
		(ts_info->usb_ts_state == USB_TS_BATT_TEMP_ABNORMAL ||
				ts_info->usb_ts_state == USB_TS_HIGH_TEMP)) {
		do {
			ts_info->usb_temp = get_usb_ts_temperature();
			pr_err("usb temp recovering..."); 
			msleep(1000);
			count_num++;
		} while (!(((ts_info->usb_temp < USB_TS_50C) && (ts_info->usb_temp > USB_TS_10C))
						|| (count_num >= CHECK_TEMP_COUNT)));
		if (count_num >= CHECK_TEMP_COUNT) {
			if (DEBUG_CHECK) {
				usb_ts_enchg_action(ts_info);//recover for debug
				pr_err("usb ts temp recover, reason:timeout");
			} else
				pr_err("temp still high!!");
		} else {
			usb_ts_enchg_action(ts_info);
			pr_err("usb ts temp recover, reason: temp normal"); //recover
		}
	}
}
#define CHECK_COUNT	3
#define USB_TS_BATT_NORMAL_GAP	6
static void update_usb_ts_current_status(struct mtk_usb_ts_info *ts_info)
{
	static int recover_num = 0;
	static int limit_num = 0;
	static int timeout_num = 0;

	if (!ts_info) {
		return;
	}

	if (ts_info->usb_ts_state == USB_TS_TEMP_NORMAL)
		return;

	if (((ts_info->usb_temp < USB_TS_30C) || (ts_info->usb_temp > USB_TS_100C))
							&& !DEBUG_CHECK) {
		limit_num = 0;
		recover_num = 0;
		ts_info->is_cool_down = false;
		return ;
	}

	if (ts_info->usb_temp - ts_info->bat_temp > ts_info->temp_parameter.usb_battery_gap_temp) {
		recover_num = 0;
		limit_num++;
	}

	if ((ts_info->usb_temp > ts_info->bat_temp) &&
		(ts_info->usb_temp - ts_info->bat_temp <= USB_TS_BATT_NORMAL_GAP)) {
		recover_num++;
		limit_num = 0;
	}

	if (recover_num > CHECK_COUNT) {
		limit_num = 0;
		recover_num = 0;
		timeout_num = 0;
		ts_info->is_cool_down = true;
	} else if (limit_num >= CHECK_COUNT) {
		limit_num = 0;
		recover_num = 0;
		ts_info->is_cool_down = false;
	}
	if (DEBUG_CHECK) {
		if (timeout_num > CHECK_COUNT * 15) {
			limit_num = 0;
			recover_num = 0;
			timeout_num = 0;
			ts_info->is_cool_down = true;
			pr_err("usb ts check timeout recover");
		} else {
			timeout_num++; //debug timeout recover
		}
	}

	if ((ts_info->is_need_disable_usb == true) && (ts_info->is_cool_down == true) &&
			(ts_info->usb_ts_state == USB_TS_TEMP_INCREASE_FAST)) {
		pr_err("usb ts temp recover for:%d", ts_info->usb_ts_state);
		usb_ts_enchg_action(ts_info);
	}
}

static void usb_ts_state_init(struct mtk_usb_ts_info *ts_info)
{
	if (ts_info->usb_ts_state == USB_TS_TEMP_NORMAL) {
		ts_info->usb_temp = get_usb_ts_temperature();
		ts_info->usb_pre_temp = ts_info->usb_temp;
	} else {
		if (!ts_info->is_aging_mode) {
			usleep_range(100000,100000); //100ms;
			usb_ts_chg_suspend(ts_info, true);
			pr_err("%s: usb keep suspend", __func__);
		}
		ts_info->is_need_disable_usb = false;
		wake_up_usb_ts_check(ts_info, false);
		usleep_range(50000,50000); //50ms;
	}
	ts_info->is_usb_in = true;
	wake_up_usb_ts_check(ts_info, true);
}

static void usb_ts_state_reset(struct mtk_usb_ts_info *ts_info)
{
	ts_info->is_usb_in = false;
	ts_info->chr_type = POWER_SUPPLY_TYPE_UNKNOWN;

	if (ts_info->usb_ts_state == USB_TS_TEMP_NORMAL)
		wake_up_usb_ts_check(ts_info, false);
}

static int usb_temp_monitor_thread(void *arg)
{
	struct mtk_usb_ts_info *ts_info = arg;
	int ret = 0;
	int sleep_time;

	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(ts_info->wait_que,
			atomic_read(&ts_info->charger_state) > 0);
		if (ret < 0) {
			chr_err("%s: wait event been interrupted(%d)\n", __func__, ret);
			continue;
		}
		mutex_lock(&ts_info->usb_ts_lock);
		if (is_temperature_normal(ts_info) && ts_info->temp_parameter.enable) {
			ts_info->usb_temp = get_usb_ts_temperature();
			//usleep_range(50000,50000);

			update_usb_ts_current_status(ts_info); //update check status
			check_usb_ts_state(ts_info); //is normal state check

			if ((ts_info->usb_ts_state != USB_TS_TEMP_NORMAL) && (!ts_info->is_aging_mode)) {
				check_disable_fast_charging_state(ts_info);
				usleep_range(10000,10000); //10ms;
				usb_ts_chg_suspend(ts_info, true);
				extcon_usb_mode_switch(DUAL_PROP_PR_SNK);
				usb_ts_dischg_action(ts_info);
			}

			if ((ts_info->temp_parameter.usb_temp_print_log_enable == 1)
				|| (ts_info->usb_ts_state != USB_TS_TEMP_NORMAL)) {
				chr_err("ts v:%d mv, usb temp:%d, need dis:%d, bat temp:%d, usb in:%d TS state:%d, hwid:%d\n",
					ts_info->usb_ts_voltage / 1000, ts_info->usb_temp, ts_info->is_need_disable_usb,
					ts_info->bat_temp, ts_info->is_usb_in, ts_info->usb_ts_state, ts_info->board_id);
			}
			usb_ts_get_charger_vbus(ts_info);
			if ((ts_info->is_need_disable_usb == true) &&
						(ts_info->usb_ts_state != USB_TS_TEMP_NORMAL)) {
				usb_ts_recover_func(ts_info); //recover check
			}
		}

		if((ts_info->is_otg_online) && (ts_info->usb_temp < USB_TS_40C)
				&& (ts_info->usb_ts_state == USB_TS_TEMP_NORMAL))
			sleep_time = NORMAL_SLEEP_TIME * 2;
		else
			sleep_time = ts_info->temp_parameter.sleep_ms;

		msleep(sleep_time);

		if ((ts_info->is_usb_in) &&
				(ts_info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)) {
			usb_ts_get_charger_type(ts_info);
		}
		mutex_unlock(&ts_info->usb_ts_lock);
	}

	return 0;
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_usb_ts_info *ts_info = (struct mtk_usb_ts_info *)container_of(nb,
						struct mtk_usb_ts_info, pd_nb);
	uint8_t old_state = TYPEC_UNATTACHED, new_state = TYPEC_UNATTACHED;

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		//chr_info("%s source vbus %d\n", __func__, noti->vbus_state.mv);
		break;
	case TCP_NOTIFY_SINK_VBUS:
		//chr_info("%s sink vbus: %d\n", __func__, noti->vbus_state.mv);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		old_state = noti->typec_state.old_state;
		new_state = noti->typec_state.new_state;

		if (old_state == TYPEC_UNATTACHED &&
		    (new_state == TYPEC_ATTACHED_SNK ||
		     new_state == TYPEC_ATTACHED_NORP_SRC ||
		     new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		     new_state == TYPEC_ATTACHED_DBGACC_SNK)) {
			chr_info("%s Get charger plug in, polarity = %d\n",
				 __func__, noti->typec_state.polarity);
			usb_ts_state_init(ts_info);
		} else if ((old_state == TYPEC_ATTACHED_SNK ||
			    old_state == TYPEC_ATTACHED_NORP_SRC ||
			    old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
			    old_state == TYPEC_ATTACHED_DBGACC_SNK) &&
			    new_state == TYPEC_UNATTACHED) {
			chr_info("%s Get charger plug out\n", __func__);
			/*
			 * report charger plug-out,
			 * and reset device connection
			 */
			usb_ts_state_reset(ts_info);
		} else if (old_state == TYPEC_UNATTACHED &&
			   (new_state == TYPEC_ATTACHED_SRC ||
			    new_state == TYPEC_ATTACHED_DEBUG)) {
			ts_info->is_otg_online = true;
			usb_ts_state_init(ts_info);
			chr_info("%s Get OTG plug in, polarity = %d\n",
				 __func__, noti->typec_state.polarity);
		} else if ((old_state == TYPEC_ATTACHED_SRC ||
			    old_state == TYPEC_ATTACHED_DEBUG) &&
			    new_state == TYPEC_UNATTACHED) {
			ts_info->is_otg_online = false;
			chr_info("%s Get OTG plug out\n", __func__);
			/* disable host connection */
			usb_ts_state_reset(ts_info);
		}

		break;
	case TCP_NOTIFY_PR_SWAP:
		chr_info("%s Get power role swap, new role = %d\n",
				    __func__, noti->swap_state.new_role);
		if (noti->swap_state.new_role == PD_ROLE_SINK) {
			chr_info("%s Get swap power role to sink\n", __func__);
			/*
			 * report charger plug-in without charger type detection
			 * to not interfering with USB2.0 communication
			 */
		} else if (noti->swap_state.new_role == PD_ROLE_SOURCE) {
			chr_info("%s Get swap power role to source\n",
					    __func__);
			/* report charger plug-out */
		}
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

static ssize_t usb_ts_temperature_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;

	if ((ts_info->is_usb_in == false) &&
		(ts_info->usb_ts_state == USB_TS_TEMP_NORMAL)) {
		ts_info->usb_temp = get_usb_ts_temperature();
	}

	chr_err("%s: %d\n", __func__, ts_info->usb_temp);
	return sprintf(buf, "%d\n", ts_info->usb_temp);
}

static ssize_t usb_ts_temperature_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;
	signed int temp;
	int current_temp = 0;
	int volt = 0;
	int ret = 0;

	if (kstrtoint(buf, 10, &temp) == 0) {
		ts_info->usb_temp = temp;
		if (ts_info->usb_ts_debug == false) {
			ts_info->usb_temp = get_usb_ts_temperature();
		}

		ret = get_usb_ts_voltage(&volt);
		if (!ret)
			current_temp = voltage_to_temperature(pts_info, volt);
		else
			current_temp = 25;
		chr_err("%s: usb temperature:%d, current temperature:%d\n",
					__func__, ts_info->usb_temp, current_temp);
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(usb_ts_temperature);

static ssize_t usb_ts_debug_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;

	chr_err("%s: %d\n", __func__, ts_info->usb_ts_debug);
	return sprintf(buf, "%d\n", ts_info->usb_ts_debug);
}

static ssize_t usb_ts_debug_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			ts_info->usb_ts_debug = false;
		else
			ts_info->usb_ts_debug = true;

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(usb_ts_debug);


static ssize_t is_aging_mode_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;

	chr_err("%s: %d\n", __func__, ts_info->is_aging_mode);
	return sprintf(buf, "%d\n", ts_info->is_aging_mode);
}

static ssize_t is_aging_mode_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_usb_ts_info *ts_info = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			ts_info->is_aging_mode = false;
		else
			ts_info->is_aging_mode = true;

		if (ts_info->is_usb_in == true)
			usb_ts_state_init(ts_info);
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(is_aging_mode);

static int mtk_usb_ts_setup_files(struct platform_device *pdev) {
	int ret = 0;

	ret = device_create_file(&(pdev->dev), &dev_attr_usb_ts_debug);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_usb_ts_temperature);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_is_aging_mode);
	if (ret)
		goto _out;

_out:
	return ret;
}

static int mtk_usb_ts_probe(struct platform_device *pdev)
{
	struct mtk_usb_ts_info *ts_info = NULL;
	int status = 0;
	int ret = 0;

	chr_info("%s start\n", __func__);

	ts_info = devm_kzalloc(&pdev->dev, sizeof(*ts_info), GFP_KERNEL);
	if (!ts_info)
		return -ENOMEM;

	platform_set_drvdata(pdev, ts_info);
	ts_info->pdev = pdev;
	ts_info->tstable = ts_temp_table_10k;
	ts_info->dev = &pdev->dev;

	ts_info->usb_ts_ctrl = devm_gpiod_get(ts_info->dev, "usb_ts_ctrl",
				   GPIOD_OUT_HIGH);
	if (IS_ERR(ts_info->usb_ts_ctrl)) {
		status = PTR_ERR(ts_info->usb_ts_ctrl);
		chr_err("ERROR!! Failed to enable gpio: %d\n", status);
		//return status;
	}
	gpiod_set_value(ts_info->usb_ts_ctrl, 0);
	devm_gpiod_put(ts_info->dev, ts_info->usb_ts_ctrl);

	init_waitqueue_head(&ts_info->wait_que);
	ts_info->chg1_dev = get_charger_by_name("primary_chg");
	if (ts_info->chg1_dev) {
		chr_err("%s, Found primary charger\n", __func__);
	} else {
		chr_err("%s, can't find primary charger\n" , __func__);
	}
	ts_info->bat_psy = power_supply_get_by_name("battery");
	if (!IS_ERR_OR_NULL(ts_info->bat_psy)) {
		chr_err("%s, Found battery psy\n", __func__);
	} else {
		chr_err("%s, can't find battery psy\n" , __func__);
	}
	ts_info->chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (ts_info->chg_psy == NULL || IS_ERR(ts_info->chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		chr_err("%s, Found charger psy\n" , __func__);
	}

	ts_info->usb_ts_state = USB_TS_TEMP_NORMAL;
	ts_info->chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
	ts_info->is_need_disable_usb = false;
	atomic_set(&ts_info->charger_state, 0);
	mutex_init(&ts_info->usb_ts_lock);

	mtk_usb_ts_setup_files(pdev);
	ts_info->usb_ts_debug = false;
	ts_info->is_aging_mode = false;

	ts_info->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!ts_info->tcpc_dev) {
		chr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -EINVAL;
	}

	ts_info->pd_nb.notifier_call = pd_tcp_notifier_call;
	ret = register_tcp_dev_notifier(ts_info->tcpc_dev, &ts_info->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		chr_err("%s: register tcpc notifer fail\n", __func__);
		return ret;
	}
	pts_info = ts_info;
#if IS_ENABLED(CONFIG_PRIZE_BOARD_ID)
	ts_info->board_id = -1;
#endif
	usb_temp_parameter_init(ts_info);

	kthread_run(usb_temp_monitor_thread, ts_info, "usb_temp_thread");
	chr_info("%s successfully\n", __func__);

	return 0;
}

static int mtk_usb_ts_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mtk_usb_ts_of_id[] = {
	{ .compatible = "mediatek,usb_ts", },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_usb_ts_of_id);

static struct platform_driver mtk_usb_ts_driver = {
	.driver = {
		.name = "usb_ts",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mtk_usb_ts_of_id),
	},
	.probe = mtk_usb_ts_probe,
	.remove = mtk_usb_ts_remove,
};

static int __init mtk_usb_ts_init(void)
{
	return platform_driver_register(&mtk_usb_ts_driver);
}
module_init(mtk_usb_ts_init);

static void __exit mtk_usb_ts_exit(void)
{
	platform_driver_unregister(&mtk_usb_ts_driver);
}
module_exit(mtk_usb_ts_exit);

MODULE_DESCRIPTION("NT USB TS Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(NT_USB_TS_DRV_VERSION);
