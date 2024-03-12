// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000001, 0x010b00, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000000, do_module_version},
		{0x00000001, 0x00000005, 0x00000002, do_part_number},
		{0x00000001, 0x000000D0, 0x0000074C, do_single_lsc},
		{0x00000001, 0x00000007, 0x0000000E, do_2a_gain},
		{0x00000001, 0x00000820, 0x000005EC, do_pdaf},
		{0x00000001, 0x00000FAE, 0x00000550, do_stereo_data},
		{0x00000001, 0x00000000, 0x00001600, do_dump_all},
		{0x00000001, 0x00000F80, 0x0000000A, do_lens_id},
		{0x00000001, 0x000000B0, 0x00000012, do_fuse_id},
		// eeprom add OisInfo@{
		{0x00000001, 0x00000F00, 0x00000008, do_ois_info},
		//@}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT s5kgn9stech_mtk_eeprom = {
	.name = "s5kgn9stech_mtk_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = S5KGN9STECH_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x4000,
	.enable_preload = 1,
	.preload_size = 0x4000,
	.has_stored_data = 1,
};
