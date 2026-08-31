// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/power_supply.h>
#include "mtk_bp_thl.h"
#define BAT_PERCENT_LIMIT 15
#define BAT_PERCENT_LIMIT_EXT 15
#define BAT_PERCENT_LIMIT_RELEASE_EXT 15
#define BPCB_MAX_NUM 16
#define MAX_VALUE 0x7FFF


static struct task_struct *bp_notify_thread;
static bool bp_notify_flag;
static bool bp_notify_flag_ext;
static DECLARE_WAIT_QUEUE_HEAD(bp_notify_waiter);
static struct wakeup_source *bp_notify_lock;
struct bp_thl_callback_table {
	void (*bpcb)(enum BATTERY_PERCENT_LEVEL_TAG);
};
static struct bp_thl_callback_table bpcb_tb[BPCB_MAX_NUM] = { {0} };
static struct bp_thl_callback_table bpcb_tb_ext[BPCB_MAX_NUM] = { {0} };
static struct notifier_block bp_nb;
struct bp_thl_priv {
	int bp_thl_lv;
	int bp_thl_lv_ext;
	int bp_thl_stop;
	int max_level;
	int *soc_limit;
	int *soc_release;
	int *temp_thd;
	int soc_cur_stage;
	int soc_max_stage;
	int temp_cur_stage;
	int temp_max_stage;
	int *throttle_table;
	struct work_struct soc_work;
	struct power_supply *psy;
};
static struct bp_thl_priv *bp_thl_data;

void register_bp_thl_notify(
	battery_percent_callback bp_cb,
	BATTERY_PERCENT_PRIO prio_val)
{
	if (!bp_thl_data) {
		pr_info("[%s] bp_thl not init\n", __func__);
		return;
	}
	if (prio_val >= BPCB_MAX_NUM) {
		pr_info("[%s] prio_val=%d, out of boundary\n", __func__, prio_val);
		return;
	}
	bpcb_tb[prio_val].bpcb = bp_cb;
	pr_info("[%s] prio_val=%d\n", __func__, prio_val);
	if (bp_thl_data->bp_thl_lv == 1) {
		pr_info("[%s] level 1 happen\n", __func__);
		if (bp_cb != NULL)
			bp_cb(BATTERY_PERCENT_LEVEL_1);
	}
}
EXPORT_SYMBOL(register_bp_thl_notify);

void register_bp_thl_notify_ext(
	battery_percent_callback bp_cb,
	BATTERY_PERCENT_PRIO prio_val)
{
	if (!bp_thl_data) {
		pr_info("[%s] bp_thl not init\n", __func__);
		return;
	}
	if (prio_val >= BPCB_MAX_NUM) {
		pr_info("[%s] prio_val=%d, out of boundary\n", __func__, prio_val);
		return;
	}
	bpcb_tb_ext[prio_val].bpcb = bp_cb;
	pr_info("[%s] prio_val=%d\n", __func__, prio_val);
	if (bp_thl_data->bp_thl_lv_ext == 1) {
		pr_info("[%s] level 1 happen\n", __func__);
		if (bp_cb != NULL)
			bp_cb(BATTERY_PERCENT_LEVEL_1);
	}
}
EXPORT_SYMBOL(register_bp_thl_notify_ext);

void exec_bp_thl_callback(enum BATTERY_PERCENT_LEVEL_TAG bp_level)
{
	int i;
	if (bp_thl_data->bp_thl_stop == 1) {
		pr_info("[%s] bp_thl_data->bp_thl_stop=%d\n"
			, __func__, bp_thl_data->bp_thl_stop);
	} else {
		for (i = 0; i < BPCB_MAX_NUM; i++) {
			if (bpcb_tb[i].bpcb != NULL)
				bpcb_tb[i].bpcb(bp_level);
		}
		pr_info("[%s] bp_level=%d\n", __func__, bp_level);
	}
}

void exec_bp_thl_callback_ext(enum BATTERY_PERCENT_LEVEL_TAG bp_level)
{
	int i;
	if (bp_thl_data->bp_thl_stop == 1) {
		pr_info("[%s] bp_thl_data->bp_thl_stop=%d\n"
			, __func__, bp_thl_data->bp_thl_stop);
	} else {
		for (i = 0; i < BPCB_MAX_NUM; i++) {
			if (bpcb_tb_ext[i].bpcb != NULL)
				bpcb_tb_ext[i].bpcb(bp_level);
		}
		pr_info("[%s] bp_level=%d\n", __func__, bp_level);
	}
}

static ssize_t bp_thl_ut_show(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	/*show_battery_percent_protect_ut */
	pr_info("[%s] g_bp_thl_lv=%d\n",
		__func__, bp_thl_data->bp_thl_lv);
	return sprintf(buf, "%u\n", bp_thl_data->bp_thl_lv);
}

static ssize_t bp_thl_ut_store(
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
	if (val < BATTERY_PERCENT_LEVEL_NUM) {
		pr_info("[%s] your input is %d\n", __func__, val);
		exec_bp_thl_callback(val);
	} else {
		pr_info("[%s] wrong number (%d)\n", __func__, val);
	}
	return size;
}

static DEVICE_ATTR_RW(bp_thl_ut);
static ssize_t bp_thl_stop_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	/*show_battery_percent_protect_stop */
	pr_info("[%s] bp_thl_data->bp_thl_stop=%d\n",
		__func__, bp_thl_data->bp_thl_stop);
	return sprintf(buf, "%u\n", bp_thl_data->bp_thl_stop);
}

static ssize_t bp_thl_stop_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	unsigned int val = 0;
	char cmd[21];
	if (sscanf(buf, "%20s %u\n", cmd, &val) != 2) {
		dev_info(dev, "parameter number not correct\n");
		return -EINVAL;
	}
	if (strncmp(cmd, "stop", 4))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		val = 0;
	bp_thl_data->bp_thl_stop = val;
	pr_info("[%s] bp_thl_data->bp_thl_stop=%d\n",
		__func__, bp_thl_data->bp_thl_stop);
	return size;
}

static DEVICE_ATTR_RW(bp_thl_stop);
static ssize_t bp_thl_level_show(
		struct device *dev, struct device_attribute *attr, char *buf)
{
	/*show_battery_percent_protect_level */
	pr_info("[%s] bp_thl_data->bp_thl_lv=%d\n",
			__func__, bp_thl_data->bp_thl_lv);
	return sprintf(buf, "%u\n", bp_thl_data->bp_thl_lv);
}

static ssize_t bp_thl_level_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf,
					   size_t size)
{
	/*store_battery_percent_protect_level */
	pr_info("[%s] bp_thl_data->bp_thl_lv=%d\n"
		, __func__, bp_thl_data->bp_thl_lv);
	return size;
}

static DEVICE_ATTR_RW(bp_thl_level);
int bp_notify_handler(void *unused)
{
	do {
		wait_event_interruptible(bp_notify_waiter, (bp_notify_flag == true) ||
			(bp_notify_flag_ext == true));
		__pm_stay_awake(bp_notify_lock);
		if (bp_notify_flag) {
			exec_bp_thl_callback(bp_thl_data->bp_thl_lv);
			bp_notify_flag = false;
		}
		if (bp_notify_flag_ext) {
			exec_bp_thl_callback_ext(bp_thl_data->bp_thl_lv_ext);
			bp_notify_flag_ext = false;
		}
		__pm_relax(bp_notify_lock);
	} while (!kthread_should_stop());
	return 0;
}

int bp_psy_event(struct notifier_block *nb, unsigned long event, void *v)
{
	if (!bp_thl_data) {
		pr_info("[%s] bp_thl_data not init\n", __func__);
		return NOTIFY_DONE;
	}

	bp_thl_data->psy = v;
	schedule_work(&bp_thl_data->soc_work);
	return NOTIFY_DONE;
}

static void soc_handler(struct work_struct *work)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret, soc, temp, new_lv, soc_thd, temp_thd, soc_stage, temp_stage;
	static int last_soc = MAX_VALUE, last_temp = MAX_VALUE;
	bool loop;

	if (!bp_thl_data) {
		pr_info("[%s] bp_thl_data not init\n", __func__);
		return;
	}

	if (!bp_thl_data->psy) {
		pr_info("[%s] psy not init\n", __func__);
		return;
	}

	psy =  bp_thl_data->psy;

	if (strcmp(psy->desc->name, "battery") != 0)
		return;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (ret)
		return;
	soc = val.intval;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (ret)
		return;
	temp = val.intval / 10;
	if (soc > 100 || soc <= 0) {
		pr_info("%s:%d return\n", __func__, __LINE__);
		return;
	}
	if (soc == last_soc && temp == last_temp) {
		pr_info("%s:%d soc and temperature are all the same\n", __func__, __LINE__);
		return;
	}
	new_lv = bp_thl_data->bp_thl_lv;
	soc_stage = bp_thl_data->soc_cur_stage;
	temp_stage = bp_thl_data->temp_cur_stage;
	pr_info("init temp_stage=%d soc_stage=%d soc:last=%d:%d temp:last=%d:%d\n", temp_stage,
		soc_stage, soc, last_soc, temp, last_temp);
	do {
		loop = false;
		if (soc < last_soc) {
			if (soc_stage < bp_thl_data->soc_max_stage) {
				soc_thd = bp_thl_data->soc_limit[soc_stage];
				if (soc <= soc_thd) {
					soc_stage++;
					loop = true;
				}
			}
		} else if (soc > last_soc) {
			if (soc_stage > 0) {
				soc_thd = bp_thl_data->soc_release[soc_stage-1];
				if (soc >= soc_thd) {
					soc_stage--;
					loop = true;
				}
			}
		}
		if (temp < last_temp) {
			if (temp_stage < bp_thl_data->temp_max_stage) {
				temp_thd = bp_thl_data->temp_thd[temp_stage];
				if (temp < temp_thd) {
					temp_stage++;
					loop = true;
				}
			}
		} else if (temp > last_temp) {
			if (temp_stage > 0) {
				temp_thd = bp_thl_data->temp_thd[temp_stage-1];
				if (temp >= temp_thd) {
					temp_stage--;
					loop = true;
				}
			}
		}
	} while (loop);
	new_lv = bp_thl_data->throttle_table[soc_stage+temp_stage*(bp_thl_data->soc_max_stage+1)];
	if (new_lv != bp_thl_data->bp_thl_lv) {
		bp_thl_data->bp_thl_lv = new_lv;
		bp_notify_flag = true;
	}
	bp_thl_data->soc_cur_stage = soc_stage;
	bp_thl_data->temp_cur_stage = temp_stage;
	last_temp = temp;
	last_soc = soc;

	if (bp_notify_flag_ext || bp_notify_flag)
		wake_up_interruptible(&bp_notify_waiter);
	return;
}

static int soc_default_setting(struct device *dev, struct bp_thl_priv *priv)
{
	struct device_node *np = dev->of_node;
	int ret;

	priv->max_level = 1;
	priv->soc_max_stage = 1;
	priv->temp_max_stage = 0;
	if (!priv->throttle_table)
		priv->throttle_table = devm_kcalloc(dev, 2, sizeof(*priv->throttle_table),
							GFP_KERNEL);

	if (!priv->throttle_table)
		return -ENOMEM;

	pr_info("use soc limit default setting\n");
	if (!priv->soc_limit) {
		priv->soc_limit = devm_kcalloc(dev, 4, sizeof(*priv->soc_limit),
			GFP_KERNEL);
		if (!priv->soc_limit)
			return -ENOMEM;
		priv->soc_release = priv->soc_limit + 1;
		priv->throttle_table = priv->soc_release + 1;
	}
	ret = of_property_read_u32(np, "soc_limit", priv->soc_limit);
	if (ret)
		*priv->soc_limit = BAT_PERCENT_LIMIT;
	*priv->soc_release = *priv->soc_limit + 1;
	priv->throttle_table[0] = 0;
	priv->throttle_table[1] = 1;
	return 0;
}

static int parse_soc_limit_table(struct device *dev, struct bp_thl_priv *priv)
{
	struct device_node *np = dev->of_node;
	int i,  num, ret;//j, offset;
//	char buf[512];
	ret = of_property_read_u32(np, "max-throttle-level", &num);
	if (ret || num < 0 || num >= BATTERY_PERCENT_LEVEL_NUM) {
		pr_notice("can't get soc-max-level %d\n", ret);
		soc_default_setting(dev, priv);
		goto out;
	}
	priv->max_level = num;
	ret = of_property_read_u32(np, "soc-max-stage", &priv->soc_max_stage);
	num = of_property_count_u32_elems(np, "soc-limit-threshold");
	if (ret || num != priv->soc_max_stage) {
		pr_notice("get soc stage error %d %d %d\n", ret, num, priv->soc_max_stage);
		soc_default_setting(dev, priv);
		goto out;
	}
	priv->soc_limit = devm_kcalloc(dev, priv->soc_max_stage, sizeof(*priv->soc_limit),
		GFP_KERNEL);
	if (!priv->soc_limit)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, "soc-limit-threshold", priv->soc_limit,
		priv->soc_max_stage);
	if (ret) {
		pr_notice("get soc-limit-threshold error %d\n", ret);
		soc_default_setting(dev, priv);
		goto out;
	}
	priv->soc_release = devm_kcalloc(dev, priv->soc_max_stage, sizeof(*priv->soc_release),
		GFP_KERNEL);
	if (!priv->soc_release)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, "soc-release-threshold", priv->soc_release,
		priv->soc_max_stage);
	if (ret) {
		pr_notice("get soc-release-threshold error %d, use soc limit + 1\n", ret);
		for (i = 0; i < priv->soc_max_stage; i++)
			priv->soc_release[i] = priv->soc_limit[i] + 1;
	}
	ret = of_property_read_u32(np, "temperature-max-stage", &priv->temp_max_stage);
	num = of_property_count_u32_elems(np, "temperature-stage-threshold");
	if (ret || num != priv->temp_max_stage) {
		pr_notice("get temperature stage error %d, use 0\n", ret);
		priv->temp_max_stage = 0;
	}
	if (priv->temp_max_stage > 0) {
		priv->temp_thd = devm_kcalloc(dev, priv->temp_max_stage, sizeof(*priv->temp_thd),
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
	num = of_property_count_u32_elems(np, "soc-throttle-level");
	if (num != (priv->temp_max_stage + 1) * (priv->soc_max_stage + 1)) {
		pr_notice("get soc-throttle-level error %d\n", num);
		soc_default_setting(dev, priv);
		goto out;
	}
	num = (priv->temp_max_stage + 1) * (priv->soc_max_stage + 1);
	priv->throttle_table = devm_kcalloc(dev, num, sizeof(*priv->throttle_table), GFP_KERNEL);
	if (!priv->throttle_table)
		return -ENOMEM;
	ret = of_property_read_u32_array(np, "soc-throttle-level", priv->throttle_table, num);
	if (ret) {
		pr_notice("get throttle_table error %d\n", ret);
		soc_default_setting(dev, priv);
		goto out;
	}
out:
	return 0;
}

static int bp_thl_probe(struct platform_device *pdev)
{
	struct bp_thl_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	dev_set_drvdata(&pdev->dev, priv);
	ret = parse_soc_limit_table(&pdev->dev, priv);
	if (ret) {
		pr_notice("parse soc limit table fail\n");
		return ret;
	}
	bp_thl_data = priv;
	INIT_WORK(&bp_thl_data->soc_work, soc_handler);
	bp_notify_lock = wakeup_source_register(NULL, "bp_notify_lock wakelock");
	if (!bp_notify_lock) {
		pr_notice("bp_notify_lock wakeup source fail\n");
		return -ENOMEM;
	}
	bp_notify_thread = kthread_run(bp_notify_handler, 0, "bp_notify_thread");
	if (IS_ERR(bp_notify_thread)) {
		pr_notice("Failed to create bp_notify_thread\n");
		return PTR_ERR(bp_notify_thread);
	}
	bp_nb.notifier_call = bp_psy_event;
	ret = power_supply_reg_notifier(&bp_nb);
	if (ret) {
		pr_notice("power_supply_reg_notifier fail\n");
		return ret;
	}
	ret = device_create_file(&(pdev->dev),
		&dev_attr_bp_thl_ut);
	ret |= device_create_file(&(pdev->dev),
		&dev_attr_bp_thl_stop);
	ret |= device_create_file(&(pdev->dev),
		&dev_attr_bp_thl_level);
	if (ret)
		dev_notice(&pdev->dev, "create file error ret=%d\n", ret);
	return 0;
}

static const struct of_device_id bp_thl_of_match[] = {
	{
		.compatible = "mediatek,mtk-bp-thl",
	},
	{
	},
};
MODULE_DEVICE_TABLE(of, bp_thl_of_match);

static struct platform_driver bp_thl_driver = {
	.driver = {
		.name = "mtk_bp_thl",
		.of_match_table = bp_thl_of_match,
	},
	.probe = bp_thl_probe,
};

module_platform_driver(bp_thl_driver);
MODULE_AUTHOR("Samuel Hsieh");
MODULE_DESCRIPTION("MTK battery percent throttling driver");
MODULE_LICENSE("GPL");
