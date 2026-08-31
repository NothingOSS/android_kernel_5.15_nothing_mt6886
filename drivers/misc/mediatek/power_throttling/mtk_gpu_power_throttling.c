// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */
#include <linux/of.h>
#include <linux/platform_device.h>
#include "mtk_gpu_power_throttling.h"
#include "mtk_battery_oc_throttling.h"
#include "mtk_low_battery_throttling.h"
#include "mtk_bp_thl.h"
#include "../../../gpu/mediatek/gpufreq/v2/include/gpufreq_v2.h"

#define CREATE_TRACE_POINTS
#include "mtk_low_battery_throttling_trace.h"

#define GPU_LIMIT_FREQ 485000

struct gpu_pt_priv {
	char max_lv_name[32];
	char limit_name[32];
	u32 max_lv;
	u32 *freq_limit;
};

static struct gpu_pt_priv gpu_pt_info[POWER_THROTTLING_TYPE_MAX] = {
	[LBAT_POWER_THROTTLING] = {
		.max_lv_name = "lbat-max-level",
		.limit_name = "lbat-limit-freq-lv",
		.max_lv = LOW_BATTERY_LEVEL_NUM - 1,
	},
	[OC_POWER_THROTTLING] = {
		.max_lv_name = "oc-max-level",
		.limit_name = "oc-limit-freq-lv",
		.max_lv = BATTERY_OC_LEVEL_NUM - 1,
	},
	[SOC_POWER_THROTTLING] = {
		.max_lv_name = "soc-max-level",
		.limit_name = "soc-limit-freq-lv",
		.max_lv = BATTERY_PERCENT_LEVEL_NUM - 1,
	}
};

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
static void gpu_pt_low_battery_cb(enum LOW_BATTERY_LEVEL_TAG level, void *data)
{
	s32 freq_limit;

	if (level > gpu_pt_info[LBAT_POWER_THROTTLING].max_lv)
		return;

	if (level > LOW_BATTERY_LEVEL_0)
		freq_limit = gpu_pt_info[LBAT_POWER_THROTTLING].freq_limit[level - 1];
	else
		freq_limit = GPUPPM_RESET_IDX;

	trace_low_battery_throttling_gpu_freq(freq_limit);
	gpufreq_set_limit(TARGET_DEFAULT, LIMIT_LOW_BATT, freq_limit, GPUPPM_KEEP_IDX);
}
#endif

#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
static void gpu_pt_over_current_cb(enum BATTERY_OC_LEVEL_TAG level, void *data)
{
	s32 freq_limit;

	if (level > gpu_pt_info[OC_POWER_THROTTLING].max_lv)
		return;

	if (level > BATTERY_OC_LEVEL_0)
		freq_limit = gpu_pt_info[OC_POWER_THROTTLING].freq_limit[level - 1];
	else
		freq_limit = GPUPPM_RESET_IDX;

	gpufreq_set_limit(TARGET_DEFAULT, LIMIT_BATT_OC, freq_limit, GPUPPM_KEEP_IDX);
}
#endif

#if IS_ENABLED(CONFIG_MTK_BATTERY_PERCENT_THROTTLING)
static void gpu_pt_battery_percent_cb(enum BATTERY_PERCENT_LEVEL_TAG level)
{
	s32 freq_limit;

	if (level > gpu_pt_info[SOC_POWER_THROTTLING].max_lv)
		return;

	if (level > BATTERY_PERCENT_LEVEL_0)
		freq_limit = gpu_pt_info[SOC_POWER_THROTTLING].freq_limit[level- 1];
	else
		freq_limit = GPUPPM_RESET_IDX;

	gpufreq_set_limit(TARGET_DEFAULT, LIMIT_BATT_PERCENT, freq_limit, GPUPPM_KEEP_IDX);
}
#endif

static void dump_gpu_setting(struct platform_device *pdev, enum gpu_pt_type type)
{
	struct gpu_pt_priv *gpu_pt_data;
	int i = 0, r = 0;
	char str[128];
	size_t len;

	gpu_pt_data = &gpu_pt_info[type];
	len = sizeof(str) - 1;

	for (i = 0; i < gpu_pt_data->max_lv; i ++) {
		r += snprintf(str + r, len - r, "%d freq ", gpu_pt_data->freq_limit[i]);
		if (r >= len)
			return;
	}
	pr_notice("[%d] %s\n", i, str);
}

static void gpu_limit_default_setting(struct device *dev, enum gpu_pt_type type)
{
	struct gpu_pt_priv *gpu_pt_data;
	int i = 0;

	gpu_pt_data = &gpu_pt_info[type];

	if (type == LBAT_POWER_THROTTLING)
		gpu_pt_data->max_lv = LOW_BATTERY_LEVEL_NUM - 1;
	else if (type == OC_POWER_THROTTLING)
		gpu_pt_data->max_lv = 2;
	else
		gpu_pt_data->max_lv = 1;

	gpu_pt_data->freq_limit = devm_kmalloc_array(dev, gpu_pt_data->max_lv,
							sizeof(u32), GFP_KERNEL);
	for (i = 0; i < gpu_pt_data->max_lv; i ++)
		gpu_pt_data->freq_limit[i] = GPU_LIMIT_FREQ;
}

static int mtk_gpu_power_throttling_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct gpu_pt_priv *gpu_pt_data;
	int i, j, ret = 0, num = 0;
	char buf[32];

	for (i = 0; i < POWER_THROTTLING_TYPE_MAX; i++) {
		gpu_pt_data = &gpu_pt_info[i];
		ret = of_property_read_u32(np, gpu_pt_data->max_lv_name, &num);
		if (ret) {
			gpu_limit_default_setting(&pdev->dev, i);
			continue;
		} else if (num <= 0 || num > gpu_pt_data->max_lv) {
			gpu_pt_data->max_lv = 0;
			continue;
		}

		gpu_pt_data->max_lv = num;
		gpu_pt_data->freq_limit = devm_kmalloc_array(&pdev->dev, gpu_pt_data->max_lv,
							sizeof(u32), GFP_KERNEL);
		if (!gpu_pt_data->freq_limit)
			return -ENOMEM;

		ret = 0;
		for (j = 0; j < gpu_pt_data->max_lv; j++) {
			memset(buf, 0, sizeof(buf));
			ret = snprintf(buf, sizeof(buf), "%s%d", gpu_pt_data->limit_name, j+1);
			if (ret < 0)
				pr_notice("can't merge %s %d\n", gpu_pt_data->limit_name, j+1);

			ret |= of_property_read_u32(np, buf, &gpu_pt_data->freq_limit[j]);
			if (ret < 0)
				pr_notice("%s: get lbat gpu limit fail %d\n", __func__, ret);
		}

		if (ret < 0) {
			kfree(gpu_pt_data->freq_limit);
			gpu_limit_default_setting(&pdev->dev, i);
		} else {
			dump_gpu_setting(pdev, i);
		}
	}

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
	if (gpu_pt_info[LBAT_POWER_THROTTLING].max_lv > 0)
		register_low_battery_notify(&gpu_pt_low_battery_cb, LOW_BATTERY_PRIO_GPU, NULL);
#endif
#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
	if (gpu_pt_info[OC_POWER_THROTTLING].max_lv > 0)
		register_battery_oc_notify(&gpu_pt_over_current_cb, BATTERY_OC_PRIO_GPU, NULL);
#endif
#if IS_ENABLED(CONFIG_MTK_BATTERY_PERCENT_THROTTLING)
	if (gpu_pt_info[SOC_POWER_THROTTLING].max_lv > 0)
		register_bp_thl_notify(&gpu_pt_battery_percent_cb, BATTERY_PERCENT_PRIO_GPU);
#endif

	return 0;
}

static const struct of_device_id gpu_power_throttling_of_match[] = {
	{ .compatible = "mediatek,gpu-power-throttling", },
	{},
};

static int mtk_gpu_power_throttling_remove(struct platform_device *pdev)
{
	return 0;
}

MODULE_DEVICE_TABLE(of, gpu_power_throttling_of_match);
static struct platform_driver gpu_power_throttling_driver = {
	.probe = mtk_gpu_power_throttling_probe,
	.remove = mtk_gpu_power_throttling_remove,
	.driver = {
		.name = "mtk-gpu_power_throttling",
		.of_match_table = gpu_power_throttling_of_match,
	},
};
module_platform_driver(gpu_power_throttling_driver);
MODULE_AUTHOR("Victor Lin");
MODULE_DESCRIPTION("MTK gpu power throttling driver");
MODULE_LICENSE("GPL");
