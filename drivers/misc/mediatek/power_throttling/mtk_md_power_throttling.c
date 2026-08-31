// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "mtk_ccci_common.h"
#include "mtk_md_power_throttling.h"
#include "mtk_battery_oc_throttling.h"
#include "mtk_low_battery_throttling.h"
#include "mtk_bp_thl.h"
#include "pmic_lbat_service.h"

#define MD_TX_REDUCE 6
struct md_pt_priv {
	char max_lv_name[32];
	char limit_name[32];
	u32 max_lv;
	u32 *reduce_tx;
	u32 *threshold;
};

static struct md_pt_priv md_pt_info[POWER_THROTTLING_TYPE_MAX] = {
	[LBAT_POWER_THROTTLING] = {
		.max_lv_name = "lbat-max-level",
		.limit_name = "lbat-reduce-tx-lv",
		.max_lv = LOW_BATTERY_LEVEL_NUM - 1,
	},
	[OC_POWER_THROTTLING] = {
		.max_lv_name = "oc-max-level",
		.limit_name = "oc-reduce-tx-lv",
		.max_lv = BATTERY_OC_LEVEL_NUM - 1,
	}
};

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
static void md_pt_low_battery_cb(enum LOW_BATTERY_LEVEL_TAG level, void *data)
{
	unsigned int md_throttle_cmd;
	int ret, intensity;

	if (level > md_pt_info[LBAT_POWER_THROTTLING].max_lv)
		return;

	if (level != LOW_BATTERY_LEVEL_0)
		intensity = md_pt_info[LBAT_POWER_THROTTLING].reduce_tx[level-1];
	else
		intensity = 0;
	md_throttle_cmd = TMC_CTRL_CMD_TX_POWER | level << 8 |
		PT_LOW_BATTERY_VOLTAGE << 16 | intensity << 24;
	ret = exec_ccci_kern_func(ID_THROTTLING_CFG,
		(char *)&md_throttle_cmd, 4);

	pr_notice("%s: send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret,
					md_throttle_cmd);

	if (ret)
		pr_notice("%s: error, ret=%d, cmd=0x%x l=%d\n", __func__, ret,
			md_throttle_cmd, level);
}

static void md_lbat_dedicate_callback(unsigned int thd)
{
	int i = 0, lv = -1;
	struct md_pt_priv *priv;

	priv = &md_pt_info[LBAT_POWER_THROTTLING];

	for (i = 0; i <= priv->max_lv; i++) {
		if (thd == priv->threshold[i]) {
			lv = i;
			break;
		}
	}

	if (lv >= 0)
		md_pt_low_battery_cb(lv, NULL);
}

#endif
#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
static void md_pt_over_current_cb(enum BATTERY_OC_LEVEL_TAG level, void *data)
{
	unsigned int md_throttle_cmd;
	int ret, intensity;
	if (level > md_pt_info[OC_POWER_THROTTLING].max_lv)
		return;
	if (level < BATTERY_OC_LEVEL_NUM) {
		if (level != BATTERY_OC_LEVEL_0)
			intensity = md_pt_info[OC_POWER_THROTTLING].reduce_tx[level-1];
		else
			intensity = 0;
		md_throttle_cmd = TMC_CTRL_CMD_TX_POWER | level << 8 | PT_OVER_CURRENT << 16 |
			intensity << 24;
		ret = exec_ccci_kern_func(ID_THROTTLING_CFG,
			(char *)&md_throttle_cmd, 4);

		pr_notice("%s: send cmd to CCCI ret=%d, cmd=0x%x\n", __func__, ret,
						md_throttle_cmd);

		if (ret)
			pr_notice("%s: error, ret=%d, cmd=0x%x l=%d\n", __func__, ret,
				md_throttle_cmd, level);
	}
}
#endif
static void md_limit_default_setting(struct device *dev, enum md_pt_type type)
{
	struct device_node *np = dev->of_node;
	int i, max_lv, ret;
	struct md_pt_priv *pt_info_p;

	pt_info_p = &md_pt_info[type];
	if (type == LBAT_POWER_THROTTLING)
		max_lv = LOW_BATTERY_LEVEL_NUM - 1;
	else if (type == OC_POWER_THROTTLING)
		max_lv = 1;
	else
		max_lv = 0;
	pt_info_p->max_lv = max_lv;
	if (!pt_info_p->max_lv)
		return;
	pt_info_p->reduce_tx = kcalloc(pt_info_p->max_lv, sizeof(u32), GFP_KERNEL);
	pt_info_p->reduce_tx[0] = MD_TX_REDUCE;
	if (type == LBAT_POWER_THROTTLING) {
		ret = of_property_read_u32(np, "lbat_md_reduce_tx", &pt_info_p->reduce_tx[0]);
		if (ret < 0)
			pr_notice("%s: get lbat cpu limit fail %d\n", __func__, ret);
	} else if (type == OC_POWER_THROTTLING) {
		ret = of_property_read_u32(np, "oc_md_reduce_tx", &pt_info_p->reduce_tx[0]);
		if (ret < 0)
			pr_notice("%s: get oc cpu limit fail %d\n", __func__, ret);
	}
	for (i = 1; i < pt_info_p->max_lv; i++)
		memcpy(&pt_info_p->reduce_tx[i], &pt_info_p->reduce_tx[0], sizeof(u32));
}
static int parse_md_limit_table(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int i, j, num, ret;
	struct md_pt_priv *pt_info_p;
	char buf[32];

	for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
		pt_info_p = &md_pt_info[i];
		ret = of_property_read_u32(np, pt_info_p->max_lv_name, &num);
		if (ret < 0) {
			md_limit_default_setting(dev, i);
			continue;
		} else if (num <= 0 || num > pt_info_p->max_lv) {
			pt_info_p->max_lv = 0;
			continue;
		}
		pt_info_p->max_lv = num;
		pt_info_p->reduce_tx = kcalloc(pt_info_p->max_lv, sizeof(u32), GFP_KERNEL);
		if (!pt_info_p->reduce_tx)
			return -ENOMEM;
		for (j = 0; j < pt_info_p->max_lv; j++) {
			memset(buf, 0, sizeof(buf));
			ret = snprintf(buf, sizeof(buf), "%s%d", pt_info_p->limit_name, j+1);
			if (ret < 0)
				pr_notice("can't merge %s %d\n", pt_info_p->limit_name, j+1);
			ret = of_property_read_u32(np, buf, &pt_info_p->reduce_tx[j]);
			if (ret < 0)
				pr_notice("%s: get lbat cpu limit fail %d\n", __func__, ret);
		}
		if (i == LBAT_POWER_THROTTLING) {
			num = of_property_count_elems_of_size(np, "lbat-dedicate-volts",
				sizeof(u32));
			if (num != pt_info_p->max_lv + 1) {
				pr_info("lbat-dedicate-volts num error: volt array=%d, max_lv=%d\n",
					num, pt_info_p->max_lv);
				continue;
			}

			pt_info_p->threshold = kcalloc(num, sizeof(u32), GFP_KERNEL);
			if (!pt_info_p->threshold)
				return -ENOMEM;

			ret = of_property_read_u32_array(np, "lbat-dedicate-volts",
				pt_info_p->threshold, num);
			if (ret) {
				pr_notice("get lbat-dedicate-volts error %d\n", ret);
				kfree(pt_info_p->threshold);
				pt_info_p->threshold = NULL;
				continue;
			}
		}
	}

	return 0;
}
static int mtk_md_power_throttling_probe(struct platform_device *pdev)
{
	int ret;
	struct md_pt_priv *priv;

	ret = parse_md_limit_table(&pdev->dev);
	if (ret != 0)
		return ret;
#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
	if (md_pt_info[LBAT_POWER_THROTTLING].max_lv > 0) {
		priv = &md_pt_info[LBAT_POWER_THROTTLING];
		if (priv->threshold) {
			lbat_user_register_ext("md pa dedicate throttle", priv->threshold,
				priv->max_lv + 1, md_lbat_dedicate_callback);
			pr_info("%s: dedicate voltge throttle\n", __func__);
		} else {
			register_low_battery_notify(&md_pt_low_battery_cb, LOW_BATTERY_PRIO_MD,
				NULL);
		}
	}
#endif
#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
	if (md_pt_info[OC_POWER_THROTTLING].max_lv > 0)
		register_battery_oc_notify(&md_pt_over_current_cb, BATTERY_OC_PRIO_MD, NULL);
#endif
	return 0;
}
static int mtk_md_power_throttling_remove(struct platform_device *pdev)
{
	return 0;
}
static const struct of_device_id md_power_throttling_of_match[] = {
	{ .compatible = "mediatek,md-power-throttling", },
	{},
};
MODULE_DEVICE_TABLE(of, md_power_throttling_of_match);
static struct platform_driver md_power_throttling_driver = {
	.probe = mtk_md_power_throttling_probe,
	.remove = mtk_md_power_throttling_remove,
	.driver = {
		.name = "mtk-md_power_throttling",
		.of_match_table = md_power_throttling_of_match,
	},
};
module_platform_driver(md_power_throttling_driver);
MODULE_AUTHOR("Samuel Hsieh");
MODULE_DESCRIPTION("MTK modem power throttling driver");
MODULE_LICENSE("GPL");
