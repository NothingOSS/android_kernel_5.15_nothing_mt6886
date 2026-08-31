/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 * Author: Samuel Hsieh <samuel.hsieh@mediatek.com>
 */

#ifndef __MTK_LOW_BATTERY_THROTTLING_H__
#define __MTK_LOW_BATTERY_THROTTLING_H__

enum LOW_BATTERY_LEVEL_TAG {
	LOW_BATTERY_LEVEL_0 = 0,
	LOW_BATTERY_LEVEL_1 = 1,
	LOW_BATTERY_LEVEL_2 = 2,
	LOW_BATTERY_LEVEL_3 = 3,
	LOW_BATTERY_LEVEL_4 = 4,
	LOW_BATTERY_LEVEL_5 = 5,
	LOW_BATTERY_LEVEL_NUM
};

enum LOW_BATTERY_PRIO_TAG {
	LOW_BATTERY_PRIO_CPU_B = 0,
	LOW_BATTERY_PRIO_CPU_L = 1,
	LOW_BATTERY_PRIO_GPU = 2,
	LOW_BATTERY_PRIO_MD = 3,
	LOW_BATTERY_PRIO_MD5 = 4,
	LOW_BATTERY_PRIO_FLASHLIGHT = 5,
	LOW_BATTERY_PRIO_VIDEO = 6,
	LOW_BATTERY_PRIO_WIFI = 7,
	LOW_BATTERY_PRIO_BACKLIGHT = 8,
	LOW_BATTERY_PRIO_DLPT = 9,
	LOW_BATTERY_PRIO_UFS = 10,
	LOW_BATTERY_PRIO_UT = 15
};

enum LOW_BATTERY_USER_TAG {
	LBAT_INTR = 0,
	LVSYS_INTR = 1,
	PPB = 2,
	UT = 3,
	TEMP_CHANGE = 4,
	LOW_BATTERY_USER_NUM
};

typedef void (*low_battery_callback)(enum LOW_BATTERY_LEVEL_TAG tag, void *data);

#if IS_ENABLED(CONFIG_MTK_LOW_BATTERY_POWER_THROTTLING)
int register_low_battery_notify(low_battery_callback lb_cb,
				enum LOW_BATTERY_PRIO_TAG prio_val, void *data);
int lbat_set_ppb_mode(unsigned int mode);
#endif

#endif /* __MTK_LOW_BATTERY_THROTTLING_H__ */
