/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2019 MediaTek Inc. */

#ifndef __ADAPTOR_HW_H__
#define __ADAPTOR_HW_H__

enum LDO_NUM {
    LDO1 = 0,
    LDO2 = 1,
    LDO3 = 2,
    LDO4 = 3,
    LDO6 = 5,
    LDO7 = 6,
    LDO_OTHER = 7,
};

int adaptor_hw_power_on(struct adaptor_ctx *ctx);
int adaptor_hw_power_off(struct adaptor_ctx *ctx);
int adaptor_hw_init(struct adaptor_ctx *ctx);
int adaptor_hw_sensor_reset(struct adaptor_ctx *ctx);


#endif
