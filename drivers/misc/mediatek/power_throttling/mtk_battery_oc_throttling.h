/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */

#ifndef __MTK_BATTERY_OC_THROTTLING_H__
#define __MTK_BATTERY_OC_THROTTLING_H__

enum BATTERY_OC_LEVEL_TAG {
	BATTERY_OC_LEVEL_0 = 0,
	BATTERY_OC_LEVEL_1 = 1,
	BATTERY_OC_LEVEL_2 = 2,
	BATTERY_OC_LEVEL_NUM
};

enum BATTERY_OC_PRIO_TAG {
	BATTERY_OC_PRIO_CPU_B = 0,
	BATTERY_OC_PRIO_CPU_L = 1,
	BATTERY_OC_PRIO_GPU = 2,
	BATTERY_OC_PRIO_MD = 3,
	BATTERY_OC_PRIO_MD5 = 4,
	BATTERY_OC_PRIO_FLASHLIGHT = 5,
	BATTERY_OC_PRIO_CHARGER = 6,
	BATTERY_OC_PRIO_UFS = 7,
	BATTERY_OC_PRIO_UT = 15,
};

typedef void (*battery_oc_callback)(enum BATTERY_OC_LEVEL_TAG tag, void *data);

int bat_oc_set_ppb_mode(unsigned int mode);
#if IS_ENABLED(CONFIG_MTK_BATTERY_OC_POWER_THROTTLING)
void register_battery_oc_notify(battery_oc_callback oc_cb,
				enum BATTERY_OC_PRIO_TAG prio_val, void *data);
#endif

#endif /* __MTK_BATTERY_OC_THROTTLING_H__ */
