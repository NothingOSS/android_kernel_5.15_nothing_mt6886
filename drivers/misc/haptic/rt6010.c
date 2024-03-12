/*
 * drivers/haptic/rt6010.c
 *
 * Copyright (c) 2022 ICSense Semiconductor CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <linux/kfifo.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/regmap.h>
#include <linux/sort.h>
#include <linux/sysfs.h>
#include <linux/compat.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/thermal.h>

#include "rt6010.h"
#include "haptic_drv.h"

#define EFS_RETRY_TIMES         3
#define EFS_BYTE_NUM            4
#define RT6010_MAX_RAM_SIZE     1536    // 1.5KB

const int8_t rt6010_f0_wave_data[] =
{
    0,22,44,64,82,98,111,120,125,126,124,117,107,93,76,57,36,14,-7,-30,-51,-71,
    -88,-103,-114,-122,-126,-126,-122,-114,-102,-87,-70,-50,-29,-6,15,37,58,77,
    94,107,118,124,126,125,119,110,97,81,63,43,21,-1,-23,-45,-65,-83,-99,-111,
    -120,-125,-126,-123,-117,-106,-92,-75,-56,-35,-13,9,31,52,72,89,104,115,123,
    126,126,121,113,101,86,69,49,27,5,-17,-39,-60,-78,-95,-108,-118,-124,-126,
    -125,-119,-109,-97,-80,-62,-41,-19,2,25,46,66,84,100,112,121,125,126,123,116,
    105,91,74,55,34,11,-10,-32,-54,-73,-90,-105,-116,-123,-126,-126,-121,-113,
    -101,-85,-68,-47,-26,-3,18,40,61,79,96,109,119,124,127,124,119,109,96,79,61,
    40,18,-3,-26,-47,-68,-85,-101,-113,-121,-126,-126,-123,-116,-105,-90,-73,-54,-32
};

extern char *saved_command_line;
extern int32_t haptic_hw_reset(struct ics_haptic_data *haptic_data);
const uint8_t rl_bst_coe[25] =
{
    0x07,0x08,0x09,0x0A,0x0A,0x0B,0x0B,0x0B,0x0B,0x0B,0x0C,0x0C,0x0C,0x0C,0x0C,
    0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D,0x0D
};

const int8_t rl_wave_data[] =
{
    0,25,50,73,92,108,119,125,126,122,113,99,80,59,35
};

static int32_t rt6010_get_chip_id(struct ics_haptic_data *haptic_data);
static int32_t rt6010_get_play_mode(struct ics_haptic_data *haptic_data);
static int32_t rt6010_set_play_mode(struct ics_haptic_data *haptic_data, uint32_t mode_val);
static int32_t rt6010_play_go(struct ics_haptic_data *haptic_data);
static int32_t rt6010_play_stop(struct ics_haptic_data *haptic_data);
static int32_t rt6010_clear_stream_fifo(struct ics_haptic_data *haptic_data);
static int32_t rt6010_set_stream_data(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t size);
static int32_t rt6010_get_irq_state(struct ics_haptic_data *haptic_data);
static int32_t rt6010_set_waveform_data(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t size);
static int32_t rt6010_clear_protection(struct ics_haptic_data *haptic_data);
static int32_t rt6010_get_lra_resistance_ex(struct ics_haptic_data *haptic_data, int32_t voltage_mode);

static int rt6010_efuse_read(struct ics_haptic_data *haptic_data, uint8_t index, uint8_t *data)
{
    int32_t ret;
    uint16_t cnt = 0;
    uint32_t reg_val;

    reg_val = index;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_EFS_ADDR_INDEX, reg_val);
    check_error_return(ret);
    reg_val = RT6010_BIT_EFS_READ;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_EFS_MODE_CTRL, reg_val);
    check_error_return(ret);

    while (cnt < EFS_RETRY_TIMES)
    {
        usleep_range(10, 20);
        ret = regmap_read(haptic_data->regmap, RT6010_REG_EFS_MODE_CTRL, &reg_val);
        check_error_return(ret);
        if ((reg_val & RT6010_BIT_EFS_READ) == 0)
        {
            ret = regmap_read(haptic_data->regmap, RT6010_REG_EFS_RD_DATA, &reg_val);
            check_error_return(ret);
            *data = (uint8_t)reg_val;
            return 1;
        }
    }

    return -1;
}

static int rt6010_apply_trim(struct ics_haptic_data *haptic_data)
{
    int32_t ret;
    uint32_t reg_val, i;
    uint8_t *efs_data_p;
    uint8_t trim_val;

    // soft reset
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_SOFT_RESET, reg_val);
    check_error_return(ret);

    // read out trim data from efuse
    efs_data_p = (uint8_t*)&haptic_data->efs_data;
    for(i = 0; i < EFS_BYTE_NUM; ++i)
    {
        ret = rt6010_efuse_read(haptic_data, i, efs_data_p + i);
        check_error_return(ret);
    }
    ics_dbg("efuse content: %08X\n", haptic_data->efs_data);
    haptic_data->efs_ver = (haptic_data->efs_data & RT6010_EFS_VERSION_MASK) >> RT6010_EFS_VERSION_OFFSET;
    ics_dbg("efuse efs_ver: %02X\n", haptic_data->efs_ver);

    // apply trim data
    ret = regmap_read(haptic_data->regmap, RT6010_REG_PMU_CFG3, &reg_val);
    check_error_return(ret);
    reg_val &= 0x4F;
    trim_val = (haptic_data->efs_data & RT6010_EFS_OSC_LDO_TRIM_MASK) >> RT6010_EFS_OSC_LDO_TRIM_OFFSET;
    reg_val |= (trim_val << 7);
    trim_val = (haptic_data->efs_data & RT6010_EFS_PMU_LDO_TRIM_MASK) >> RT6010_EFS_PMU_LDO_TRIM_OFFSET;
    reg_val |= (trim_val << 4);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PMU_CFG3, reg_val);
    check_error_return(ret);
    trim_val = (haptic_data->efs_data & RT6010_EFS_BIAS_1P2V_TRIM_MASK) >> RT6010_EFS_BIAS_1P2V_TRIM_OFFSET;
    reg_val = trim_val << 4;
    trim_val = (haptic_data->efs_data & RT6010_EFS_BIAS_I_TRIM_MASK) >> RT6010_EFS_BIAS_I_TRIM_OFFSET;
    reg_val |= trim_val;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PMU_CFG4, reg_val);
    check_error_return(ret);

    trim_val = (haptic_data->efs_data & RT6010_EFS_OSC_TRIM_MASK) >> RT6010_EFS_OSC_TRIM_OFFSET;
    trim_val ^= 0x80;
    reg_val = trim_val;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_OSC_CFG1, reg_val);
    check_error_return(ret);

    reg_val = 0x03;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);

    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x05;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 0x0A;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);

    if (haptic_data->efs_ver == 0x01)
    {
        reg_val = 0x0F;
        ret = regmap_write(haptic_data->regmap, 0x2A, reg_val);
        check_error_return(ret);
        reg_val = 0x00;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
        check_error_return(ret);
        reg_val = 0x50;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG4, reg_val);
        check_error_return(ret);
    }
    else
    {
        reg_val = 0x0B;
        ret = regmap_write(haptic_data->regmap, 0x2A, reg_val);
        check_error_return(ret);
        reg_val = 0x06;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
        check_error_return(ret);
        reg_val = 0x58;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG4, reg_val);
        check_error_return(ret);
    }

    reg_val = 0x1C;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG5, reg_val);
    check_error_return(ret);

    reg_val = 0x2C;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PA_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x03;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PA_CFG2, reg_val);
    check_error_return(ret);

    reg_val = 0x1C;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PMU_CFG2, reg_val);
    check_error_return(ret);

    return 0;
}
#ifdef MODULE
static char __rt6010_cmdline[2048];
static char *rt6010_cmdline = __rt6010_cmdline;

static const char *rt6010_get_cmd(void)
{
    struct device_node *of_chosen = NULL;
    char *bootargs = NULL;

    if (__rt6010_cmdline[0] != 0)
        return rt6010_cmdline;

    of_chosen = of_find_node_by_path("/chosen");
    if (of_chosen) {
        bootargs = (char *)of_get_property(
            of_chosen, "bootargs", NULL);
        if (!bootargs)
            ics_dbg("%s: failed to get bootargs\n", __func__);
        else {
            strcpy(__rt6010_cmdline, bootargs);
            ics_dbg("%s: bootargs: %s\n", __func__, bootargs);
        }
    } else
        ics_dbg("%s: failed to get /chosen\n", __func__);

    return rt6010_cmdline;
}
#else
static const char *rt6010_get_cmd(void)
{
	return saved_command_line;
}
#endif

static int rt6010_get_cmdline_f0_data(void)
{
    char *ptr = NULL;
    int f0_data = 0;
    ptr = strstr(rt6010_get_cmd(),"haptic_f0=");
    if(ptr == NULL)
    {
        ics_dbg("%s get null rt6010_get_cmd = %s\n", __func__, rt6010_get_cmd());
        return -1;
    }
    ics_dbg("%s ptr = %s\n", __func__, ptr);
    ptr += strlen("haptic_f0=");
    f0_data = simple_strtol(ptr, NULL, 10);
    ics_dbg("%s f0_data = %d\n", __func__, f0_data);
    return f0_data;
}

static int rt6010_private_init(struct ics_haptic_data *haptic_data, struct ics_haptic_chip_config *chip_config_data)
{
    int32_t ret, f0;
    uint32_t reg_val;

    haptic_data->reg_size = chip_config_data->reg_size;
    haptic_data->ram_size = chip_config_data->ram_size;

    reg_val = 0x08;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_CFG, reg_val);
    check_error_return(ret);
    ics_dbg("RAM is initialized!\n");

    // initialize ram sections and fifo configuration
    reg_val = (chip_config_data->list_base_addr & 0xFF00) >> 8;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_LIST_BASE_ADDR_H, reg_val);
    check_error_return(ret);
    reg_val = chip_config_data->list_base_addr & 0xFF;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_LIST_BASE_ADDR_L, reg_val);
    check_error_return(ret);
    haptic_data->list_base_addr = chip_config_data->list_base_addr;
    ics_dbg("list_base_addr = 0x%04X\n", haptic_data->list_base_addr);

    reg_val = (chip_config_data->wave_base_addr & 0xFF00) >> 8;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_WAVE_BASE_ADDR_H, reg_val);
    check_error_return(ret);
    reg_val = chip_config_data->wave_base_addr & 0xFF;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_WAVE_BASE_ADDR_L, reg_val);
    check_error_return(ret);
    haptic_data->wave_base_addr = chip_config_data->wave_base_addr;
    ics_dbg("wave_base_addr = 0x%04X\n", haptic_data->wave_base_addr);

    reg_val = (chip_config_data->fifo_ae & 0xFF00) >> 8;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_FIFO_AE_H, reg_val);
    check_error_return(ret);
    reg_val = chip_config_data->fifo_ae & 0xFF;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_FIFO_AE_L, reg_val);
    check_error_return(ret);
    haptic_data->fifo_ae = chip_config_data->fifo_ae;
    ics_dbg("fifo_ae = 0x%04X\n", haptic_data->fifo_ae);

    reg_val = (chip_config_data->fifo_af & 0xFF00) >> 8;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_FIFO_AF_H, reg_val);
    check_error_return(ret);
    reg_val = chip_config_data->fifo_af & 0xFF;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_FIFO_AF_L, reg_val);
    check_error_return(ret);
    haptic_data->fifo_af = chip_config_data->fifo_af;
    ics_dbg("fifo_af = 0x%04X\n", haptic_data->fifo_af);

    // configure boost voltage
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BOOST_CFG3, &reg_val);
    reg_val = (chip_config_data->boost_vol & 0x0F) | (reg_val & 0xF0);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    haptic_data->boost_vol = (chip_config_data->boost_vol & 0x0F);
    ics_dbg("bst_vout = 0x%04X\n", haptic_data->boost_vol);

    // configure gain
    reg_val = chip_config_data->gain;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, reg_val);
    check_error_return(ret);
    haptic_data->gain = chip_config_data->gain;
    ics_dbg("gain = 0x%04X\n", haptic_data->gain);

    // configure pre f0
    reg_val = (chip_config_data->f0 & 0x3F00) >> 8;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_LRA_F0_CFG2, reg_val);
    check_error_return(ret);
    reg_val = chip_config_data->f0 & 0xFF;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_LRA_F0_CFG1, reg_val);
    check_error_return(ret);
    haptic_data->f0 = 1920000 / chip_config_data->f0;
    ics_dbg("lra pre f0 = %u\n", chip_config_data->f0);
    f0 = rt6010_get_cmdline_f0_data();
    if (f0 > 0)
    {
        haptic_data->f0 = f0;
    }
    if (abs((int32_t)haptic_data->f0 - DEFAULT_F0) > 150)
    {
        haptic_data->f0 = DEFAULT_F0;
    }
    ics_dbg("lra get f0 = %u\n", haptic_data->f0);

    // initialize waveform data
    ics_dbg("waveform data size = %u\n", chip_config_data->waveform_size);
    ret = rt6010_set_waveform_data(haptic_data, chip_config_data->waveform_data, chip_config_data->waveform_size);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_chip_init(struct ics_haptic_data *haptic_data,
    const uint8_t* config_data, uint32_t config_size)
{
    int32_t ret = 0;
    struct ics_haptic_chip_config *chip_config_data = (struct ics_haptic_chip_config*)config_data;

    ret = rt6010_get_chip_id(haptic_data);
    check_error_return(ret);
    ics_dbg("rt6010 chip id: 0x%04X\n", haptic_data->chip_id);

    if (haptic_data->chip_id != chip_config_data->chip_id)
    {
        ics_err("chip id doesn't match in chip configuration file!\n");
        return -1;
    }

    // read out calibration value from efuse and apply it to chip
    ret = rt6010_apply_trim(haptic_data);
    if (ret < 0)
    {
        ics_err("%s: Failed to apply chip trim data! ret = %d\n", __func__, ret);
        return ret;
    }

    ret = rt6010_private_init(haptic_data, chip_config_data);
    if (ret < 0)
    {
        ics_err("%s, Failed to initialize ics6b1x private data! ret = %d\n", __func__, ret);
        return ret;
    }

    rt6010_clear_protection(haptic_data);

    return 0;
}

static int32_t rt6010_get_chip_id(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;
    int retry = 5;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_DEV_ID, &reg_val);
    ics_dbg("rt6010 get chip id: 0x%04X reg_val:%x ret:%d\n", haptic_data->chip_id, reg_val, ret);
    while (ret)
    {
        retry--;
        haptic_hw_reset(haptic_data);
        msleep(100);
        ret = regmap_read(haptic_data->regmap, RT6010_REG_DEV_ID, &reg_val);
        ics_dbg("rt6010 chip id:%x ret:%d retry:%d\n", reg_val, ret, retry);
        if (retry <= 0)
        {
            break;
        }
    }
    check_error_return(ret);

    haptic_data->chip_id = reg_val;

    return 0;
}

static int32_t rt6010_get_reg(struct ics_haptic_data *haptic_data, uint32_t reg_addr, uint32_t *reg_data)
{
    int32_t ret = 0;

    ret = regmap_read(haptic_data->regmap, reg_addr, reg_data);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_set_reg(struct ics_haptic_data *haptic_data, uint32_t reg_addr, uint32_t reg_data)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, reg_addr, reg_data);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_get_f0_by_bemf(struct ics_haptic_data *haptic_data)
{
    int32_t ret;
    uint32_t reg_val, reg_val1, reg_val2;
    uint32_t orig_gain, orig_brake;
    uint16_t czVal1 = 0, czVal2 = 0, czVal3 = 0, czVal4 = 0, czVal5 = 0;

    // enable F0 Detection
    reg_val = 0x26;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x20;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG4, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_DETECT_FO_CFG, reg_val);
    check_error_return(ret);

    // reserve gain and brake configuration
    ret = regmap_read(haptic_data->regmap, RT6010_REG_GAIN_CFG, &orig_gain);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BRAKE_CFG1, &orig_brake);
    check_error_return(ret);

    // set gain and disable brake
    reg_val = 0x28;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BRAKE_CFG1, reg_val);
    check_error_return(ret);

    // play and wait for play done!
    rt6010_play_stop(haptic_data);
    rt6010_set_play_mode(haptic_data, PLAY_MODE_STREAM);
    rt6010_clear_stream_fifo(haptic_data);
    rt6010_get_irq_state(haptic_data);
    rt6010_play_go(haptic_data);
    rt6010_set_stream_data(haptic_data, (uint8_t*)rt6010_f0_wave_data, sizeof(rt6010_f0_wave_data));
    msleep(100);

    // restore gain and brake configuration
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, orig_gain);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BRAKE_CFG1, orig_brake);
    check_error_return(ret);

    // calculate F0
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ1_VAL1, &reg_val1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ1_VAL2, &reg_val2);
    check_error_return(ret);
    czVal1 = (uint16_t)(reg_val2 & 0x3F);
    czVal1 = (uint16_t)((czVal1 << 8) | reg_val1);

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ2_VAL1, &reg_val1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ2_VAL2, &reg_val2);
    check_error_return(ret);
    czVal2 = (uint16_t)(reg_val2 & 0x3F);
    czVal2 = (uint16_t)((czVal2 << 8) | reg_val1);

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ3_VAL1, &reg_val1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ3_VAL2, &reg_val2);
    check_error_return(ret);
    czVal3 = (uint16_t)(reg_val2 & 0x3F);
    czVal3 = (uint16_t)((czVal3 << 8) | reg_val1);

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ4_VAL1, &reg_val1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ4_VAL2, &reg_val2);
    check_error_return(ret);
    czVal4 = (uint16_t)(reg_val2 & 0x3F);
    czVal4 = (uint16_t)((czVal4 << 8) | reg_val1);

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ5_VAL1, &reg_val1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CZ5_VAL2, &reg_val2);
    check_error_return(ret);
    czVal5 = (uint16_t)(reg_val2 & 0x3F);
    czVal5 = (uint16_t)((czVal5 << 8) | reg_val1);

    haptic_data->f0 = 1920000 / (((czVal4 - czVal2) + (czVal5 - czVal3)) >> 1);

    return 0;
}

/*static int32_t rt6010_get_f0_by_autotrack(struct ics_haptic_data *haptic_data)
{
    int32_t ret;
    uint32_t reg_val, orig_gain, f0_val;

    // reserve gain configuration
    ret = regmap_read(haptic_data->regmap, RT6010_REG_GAIN_CFG, &orig_gain);
    check_error_return(ret);

    reg_val = 2;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_TRACK_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 3;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_TRACK_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 20;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_TRACK_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x30;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x20;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG4, reg_val);
    check_error_return(ret);
    reg_val = 0x80;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, reg_val);
    check_error_return(ret);
    f0_val = 1920000 / DEFAULT_F0;
    reg_val = (f0_val & 0xFF);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_TRACK_F0_VAL1, reg_val);
    check_error_return(ret);
    reg_val = ((f0_val & 0x3F00) >> 8);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_TRACK_F0_VAL2, reg_val);
    check_error_return(ret);
    reg_val = 0x03;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PLAY_MODE, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PLAY_CTRL, reg_val);
    check_error_return(ret);

    msleep(100);
    // restore gain configuration
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, orig_gain);
    check_error_return(ret);

    f0_val = 0;
    ret = regmap_read(haptic_data->regmap, RT6010_REG_TRACK_F0_VAL1, &reg_val);
    check_error_return(ret);
    f0_val = (reg_val & 0xFF);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_TRACK_F0_VAL2, &reg_val);
    check_error_return(ret);
    f0_val = (f0_val | ((reg_val & 0x3F) << 8));

    haptic_data->f0 = 1920000 / f0_val;
    ics_dbg("%s: after auto track f0 = %u, %u\n", __func__, haptic_data->f0, f0_val);

    return 0;
}*/

static int32_t rt6010_get_f0(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;

    haptic_data->f0 = 0;
    ret = rt6010_get_f0_by_bemf(haptic_data);
    //check_error_return(ret);
    if (abs((int32_t)haptic_data->f0 - DEFAULT_F0) > 100)
    {
        ics_info("lra get f0=%d exceed allowed range, detect again\n", (int32_t)haptic_data->f0);
        usleep_range(130000, 130000);
        ret = rt6010_get_f0_by_bemf(haptic_data);
        check_error_return(ret);
    }

    return ret;
}

static int32_t rt6010_get_play_mode(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_PLAY_MODE, &reg_val);
    check_error_return(ret);

    haptic_data->play_mode = (enum ics_haptic_play_mode)reg_val;

    return 0;
}

static int32_t rt6010_set_play_mode(struct ics_haptic_data *haptic_data, uint32_t mode_val)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_PLAY_MODE, mode_val);
    check_error_return(ret);

    haptic_data->play_mode = (enum ics_haptic_play_mode)mode_val;

    return 0;
}

static int32_t rt6010_play_go(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_PLAY_CTRL, 0x01);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_play_stop(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_PLAY_CTRL, 0x00);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_set_gain(struct ics_haptic_data *haptic_data, uint32_t gain_val)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, gain_val);
    check_error_return(ret);

    haptic_data->gain = gain_val;

    return 0;
}

static int32_t rt6010_set_bst_vol(struct ics_haptic_data *haptic_data, uint32_t bst_vol)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BOOST_CFG3, &reg_val);
    check_error_return(ret);

    reg_val = ((reg_val & 0xF0) | bst_vol);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);

    haptic_data->boost_vol = bst_vol;

    return 0;
}

static int32_t rt6010_set_bst_mode(struct ics_haptic_data *haptic_data, uint32_t bst_mode)
{
    //TODO:
    return 0;
}

static int32_t rt6010_set_play_list(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t size)
{
    int32_t ret = 0;
    uint16_t section_size = haptic_data->wave_base_addr - haptic_data->list_base_addr;

    if (size > section_size)
    {
        ics_err("%s: playlist data is bigger than the section size! data_size = %u, \
            section size = %u\n", __func__, size, section_size);
        return -1;
    }

    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_ADDR_H, haptic_data->list_base_addr >> 8);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_ADDR_L, haptic_data->list_base_addr & 0x00FF);
    check_error_return(ret);

    ret = regmap_raw_write(haptic_data->regmap, RT6010_REG_RAM_DATA, data, size);
    check_error_return(ret);
    ics_dbg("set playlist data successfully! size = %u\n", size);

    return 0;
}

static int32_t rt6010_set_waveform_data(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t size)
{
    int32_t ret = 0;
    int16_t section_size = RT6010_MAX_RAM_SIZE - haptic_data->wave_base_addr;

    if (size > section_size)
    {
        ics_err("%s: waveform data is bigger than the section size! data_size = %u, \
            section size = %u\n", __func__, size, section_size);
        return -1;
    }

    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_ADDR_H, haptic_data->wave_base_addr >> 8);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_ADDR_L, haptic_data->wave_base_addr & 0x00FF);
    check_error_return(ret);

    ret = regmap_raw_write(haptic_data->regmap, RT6010_REG_RAM_DATA, data, size);
    check_error_return(ret);
    ics_dbg("set waveform data successfully! size = %u\n", size);

    return 0;
}

static int32_t rt6010_clear_stream_fifo(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_CFG, 0x01);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_set_stream_data(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t size)
{
    int32_t ret = 0;

    ret = regmap_raw_write(haptic_data->regmap, RT6010_REG_STREAM_DATA, data, size);
    check_error_return(ret);
    ics_dbg("set stream data successfully! size = %u\n", size);

    return 0;
}

static int32_t rt6010_trig_init(struct ics_haptic_data *haptic_data)
{
    // TODO:
    return 0;
}

static int32_t rt6010_get_ram_data(struct ics_haptic_data *haptic_data, uint8_t *data, uint32_t *size)
{
    int32_t ret = 0;
    uint32_t read_size = RT6010_MAX_RAM_SIZE;

    if (*size < RT6010_MAX_RAM_SIZE)
    {
        ics_info("%s: data buffer size is smaller than RAM size! \
            buffer_size = %u, ram_size = %u\n", __func__, *size, RT6010_MAX_RAM_SIZE);
        read_size = *size;
    }

    ret = regmap_raw_read(haptic_data->regmap, RT6010_REG_RAM_DATA_READ, data, read_size);
    check_error_return(ret);
    ics_dbg("get ram data successfully! size = %u\n", read_size);

    return ret;
}

static int32_t rt6010_get_sys_state(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_SYS_STATUS1, &reg_val);
    check_error_return(ret);

    haptic_data->sys_state = (reg_val >> 2);

    return 0;
}

static int32_t rt6010_get_vbat_efs1(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val;
    uint32_t adc_reg_val1, adc_reg_val2;
    uint16_t adc_val = 0;
    uint16_t adc_data = 0;
    int32_t i = 0;
    int32_t offset_val = 0;
    int32_t vbat_det_trim;
    uint8_t trim_val;

    trim_val = (haptic_data->efs_data & RT6010_EFS_VBAT_OFFSET_MASK) >> RT6010_EFS_VBAT_OFFSET_OFFSET;
    offset_val = (trim_val & 0x0F) * 313;
    if ((trim_val & 0x10) != 0)
    {
        offset_val = 0 - offset_val;
    }
    vbat_det_trim = offset_val - 1740;

    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x44;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    reg_val = 0x1D;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG5, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    msleep(2);
    for(i=0; i<4; i++)
    {
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA1, &adc_reg_val1);
        check_error_return(ret);
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA2, &adc_reg_val2);
        check_error_return(ret);
        adc_val = (adc_reg_val1 & 0xFF) | ((adc_reg_val2 & 0x03) << 8);
        if (adc_reg_val2 & 0x02)
        {
            adc_val = (~adc_val + 1) & 0x1FF;
        }
        else
        {
            adc_val = adc_val & 0x1FF;
        }
        adc_data += adc_val;
    }
    adc_data >>= 2;
    haptic_data->vbat = (adc_data * 227 + vbat_det_trim) / 100;
    reg_val = 0x0A;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x1C;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG5, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_compare_func(const void *a, const void *b)
{
    return (*(int32_t *)a - *(int32_t *)b);
}

static int32_t rt6010_get_vbat_efs2(struct ics_haptic_data *haptic_data)
{
    int16_t ret = 0;
    uint32_t reg_val, lowreg, highreg;
    uint32_t data = 0;
    uint8_t vbat_efuse_value=0;
    int32_t adc_data[100], adc_data_len, adc_avg, i;

    adc_data_len = sizeof(adc_data)/sizeof(int32_t);
    vbat_efuse_value = (haptic_data->efs_data & RT6010_EFS_VBAT_OFFSET_MASK) >> RT6010_EFS_VBAT_OFFSET_OFFSET;

    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);
    reg_val = 0x10;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    reg_val = 0x1D;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG5, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 0x46;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    msleep(10);

    for (i = 0; i < adc_data_len; i++)
    {
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA1, &lowreg);
        check_error_return(ret);
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA2, &highreg);
        check_error_return(ret);
        data = (lowreg & 0xFF) | ((highreg & 0x03) << 8);

        if (highreg & 0x2)
        {
            adc_data[i] = (int)((~data+1) & (0x1ff));
        }
        else
        {
            adc_data[i] = (int)(data & (0x1ff));
        }
    }

    sort(adc_data, adc_data_len, sizeof(int32_t), rt6010_compare_func, NULL);

    adc_avg = 0;
    for (i = 40; i < 60; i++)
    {
        adc_avg += adc_data[i];
    }

    adc_avg = adc_avg / 20;
    if (((vbat_efuse_value>>4)&0x01)!=0x01)
    {
        haptic_data->vbat = 18 * (adc_avg * 10 - 1500 + (int32_t)(vbat_efuse_value & 0x0f) * 40) / 100 + 400;
    }
    else
    {
        haptic_data->vbat = 18 * (adc_avg * 10 - 1500 - (int32_t)(vbat_efuse_value & 0x0f) * 40) / 100 + 400;
    }

    reg_val = 0x1C;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG5, reg_val);
    check_error_return(ret);
    reg_val = 0x0A;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x06;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_get_vbat(struct ics_haptic_data *haptic_data)
{
    if (haptic_data->efs_ver == 0x01)
    {
        rt6010_get_vbat_efs1(haptic_data);
    }
    else
    {
        rt6010_get_vbat_efs2(haptic_data);
    }

    return 0;
}

static int32_t rt6010_detect_rl_play(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val =0;
    uint32_t origin_gain = 0, origin_boost_cfg1 = 0, origin_pa_cfg1 = 0, origin_pmu_cfg4 = 0;
    int32_t rl_detect_cal = -1;
    uint8_t n = 0, m = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_GAIN_CFG, &origin_gain);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BOOST_CFG1, &origin_boost_cfg1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_PA_CFG1, &origin_pa_cfg1);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_PMU_CFG4, &origin_pmu_cfg4);
    check_error_return(ret);

    reg_val = 0x0f;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PROTECTION_MASK1, reg_val);
    check_error_return(ret);
    reg_val = 0x0f;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PROTECTION_MASK2, reg_val);
    check_error_return(ret);
    reg_val =0xb7;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PMU_CFG4, reg_val);
    check_error_return(ret);
    reg_val =0x20;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x2b;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PA_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x03;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PA_CFG2, reg_val);
    check_error_return(ret);

    rt6010_play_stop(haptic_data);
    rt6010_set_play_mode(haptic_data, PLAY_MODE_STREAM);
    rt6010_clear_stream_fifo(haptic_data);
    rt6010_get_irq_state(haptic_data);

    for(n = 30;n < 128; n++)
    {
        rl_detect_cal = -1;
        reg_val =n;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, reg_val);
        check_error_return(ret);
        ret = rt6010_play_go(haptic_data);
        check_error_return(ret);
        ret = rt6010_set_stream_data(haptic_data, (uint8_t*)rl_wave_data, sizeof(rl_wave_data));
        check_error_return(ret);
        msleep(2);

        for (m = 0;m < 128; m++)
        {
            ret = regmap_read(haptic_data->regmap, RT6010_REG_INT_STATUS, &reg_val);
            check_error_return(ret);
            if ((reg_val & 0x08) != 0)
            {
                break;
            }
            else if ((reg_val & 0x01) != 0)
            {
                rl_detect_cal = n;
                break;
            }
        }
        if (rl_detect_cal >= 0)
        {
            break;
        }
    }
    ics_info("rl_detect_cal %d \n", rl_detect_cal);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_GAIN_CFG, origin_gain);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG1, origin_boost_cfg1);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PA_CFG1, origin_pa_cfg1);
    check_error_return(ret);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PMU_CFG4, origin_pmu_cfg4);
    check_error_return(ret);

    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PROTECTION_MASK1, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_PROTECTION_MASK2, reg_val);
    check_error_return(ret);

    return rl_detect_cal;
}

static int32_t rt6010_get_lra_resistance_efs1(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val=0;

    uint8_t org_vout;
    int32_t rl_val = 0;
    int32_t rl_detect_cal = 0;
    int32_t rl_vbat_offset=0;
    uint8_t rl_det_trim;

    if (haptic_data->config_data != NULL)
    {
        rt6010_chip_init(haptic_data, haptic_data->config_data, haptic_data->config_size);
    }

    rl_det_trim = (haptic_data->efs_data & RT6010_EFS_RL_OFFSET_MASK) >> RT6010_EFS_RL_OFFSET_OFFSET;
    rt6010_get_vbat(haptic_data);
    rt6010_clear_protection(haptic_data);

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BOOST_CFG3, &reg_val);
    check_error_return(ret);
    org_vout = reg_val;

    rl_vbat_offset=(int)(400 - haptic_data->vbat);
    if (rl_vbat_offset < 0)
    {
        rl_vbat_offset = -rl_vbat_offset;
        rl_vbat_offset = rl_vbat_offset / 10;
        rl_vbat_offset = -rl_vbat_offset;
    }
    else
    {
        rl_vbat_offset = rl_vbat_offset / 10;
    }
    reg_val = rl_bst_coe[10 - rl_vbat_offset];
    ics_info("vbst original config %d \n", reg_val);
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    ret = regmap_read(haptic_data->regmap, RT6010_REG_BOOST_CFG3, &reg_val);
    ics_info("vbst original config read %d \n", reg_val);

    rl_detect_cal = rt6010_detect_rl_play(haptic_data);
    if(rl_detect_cal < 0)
    {
        return -1;
    }

    ics_info("rl_detect_cal:%d haptic_data->rl_det_trim %d\n", rl_detect_cal, rl_det_trim);
    rl_val = (int)(88 * rl_detect_cal / rl_det_trim);
    haptic_data->lra_resistance = (uint32_t)((rl_val * 100) / 10);
    ics_info("lra_resistance %d\n", haptic_data->lra_resistance);

    reg_val = org_vout;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);

    rt6010_clear_protection(haptic_data);

    return 0;
}

static int32_t rt6010_get_lra_resistance_ex(struct ics_haptic_data *haptic_data, int32_t voltage_mode)
{
    int16_t ret = 0;
    uint32_t reg_val, lowreg, highreg;
    int32_t adc_avg = 0, adc_data[300], adc_data_len;
    uint32_t data = 0;
    uint32_t rl_efuse_value = 0;
    int32_t lra_resistance_tmp;
    int32_t i = 0;
#if IS_ENABLED(CONFIG_THERMAL)
    struct thermal_zone_device *tzd;
#endif /* CONFIG_THERMAL */
    int32_t temperature;
    int32_t temp_comp;

    adc_data_len = sizeof(adc_data)/sizeof(int32_t);
    rl_efuse_value = (haptic_data->efs_data & RT6010_EFS_RL_OFFSET_MASK) >> RT6010_EFS_RL_OFFSET_OFFSET;
    ics_info("rl_efuse_value %d\n", rl_efuse_value);

    if (voltage_mode == 65)
    {
        reg_val = 0x12;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
        check_error_return(ret);
    }
    else if (voltage_mode == 110)
    {
        reg_val = 0x1F;
        ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
        check_error_return(ret);
    }
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);
    reg_val = 0x01;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    msleep(2);
    reg_val = 0x86;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    msleep(50);

    for (i = 0; i < adc_data_len; i++)
    {
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA1, &lowreg);
        check_error_return(ret);
        ret = regmap_read(haptic_data->regmap, RT6010_REG_ADC_DATA2, &highreg);
        check_error_return(ret);

        data = (lowreg & 0xFF) | ((highreg & 0x03) <<8);
        if (highreg & 0x2)
        {
            adc_data[i] = (int)((~data+1) & (0x1ff));
        }
        else
        {
            adc_data[i] = (int)(data & (0x1ff));
        }
    }

    sort(adc_data, adc_data_len, sizeof(int32_t), rt6010_compare_func, NULL);

    adc_avg = 0;
    for (i = 100; i < 200; i++)
    {
        adc_avg += adc_data[i];
    }
    adc_avg /= 100;
    ics_info("rl adc_avg %d\n", adc_avg);

    // get system temperature
#if IS_ENABLED(CONFIG_THERMAL)
    tzd = thermal_zone_get_zone_by_name("xtal-ntc");
    ret = thermal_zone_get_temp(tzd, &temperature);
#else
    temperature = 29000;
#endif /* CONFIG_THERMAL */
    ics_dbg("xtal-ntc temperature = %d\n", temperature);
    if (voltage_mode == 65)
    {
        temp_comp = (int)((int)7128 * (temperature - 29000) - 12728000) / 1000000;
        ics_dbg("temp_comp %d\n", (int32_t)temp_comp);
        if(((rl_efuse_value >> 6) & 0x01) != 0x01)
        {
            lra_resistance_tmp = 177 * (adc_avg * 10 - (2270 - (int32_t)(rl_efuse_value & 0x3f) * 4) - (int32_t)temp_comp) / 100 + 880;
        }
        else
        {
            lra_resistance_tmp = 177 * (adc_avg * 10 - (2270 + (int32_t)(rl_efuse_value & 0x3f) * 4) - (int32_t)temp_comp) / 100 + 880;
        }
        ics_dbg("65_lra_resistance_tmp %d\n", lra_resistance_tmp);

        rt6010_get_vbat(haptic_data);
        haptic_data->lra_resistance = (lra_resistance_tmp * 10000 + 4267 * haptic_data->vbat - 17560 * 100) / 10000;
    }
    else if (voltage_mode == 110)
    {
        temp_comp = (int)((int)9330 * (temperature - 29000) + 12265200)/ 10000000;
        ics_dbg("temp_comp %d\n", (int32_t)temp_comp);
        if(((rl_efuse_value >> 6) & 0x01) != 0x01)
        {
            lra_resistance_tmp = 10 * (adc_avg - (410 - (int32_t)(rl_efuse_value & 0x3f)) - (int32_t)temp_comp) + 880;
        }
        else
        {
            lra_resistance_tmp = 10 * (adc_avg - (410 + (int32_t)(rl_efuse_value & 0x3f)) - (int32_t)temp_comp) + 880;
        }
        ics_dbg("11_lra_resistance_tmp %d\n", lra_resistance_tmp);

        rt6010_get_vbat(haptic_data);
        haptic_data->lra_resistance = (lra_resistance_tmp * 10000 + 2288 * haptic_data->vbat - 9294 * 100) / 10000;
    }

    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x24, reg_val);
    check_error_return(ret);
    reg_val = 0x06;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG1, reg_val);
    check_error_return(ret);
    reg_val = 0x0A;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BOOST_CFG3, reg_val);
    check_error_return(ret);
    reg_val = 0x00;
    ret = regmap_write(haptic_data->regmap, 0x2B, reg_val);
    check_error_return(ret);

    return 0;
}

static int32_t rt6010_get_lra_resistance(struct ics_haptic_data *haptic_data)
{
    switch (haptic_data->efs_ver)
    {
        case 0x01:
        {
            rt6010_get_lra_resistance_efs1(haptic_data);
        }
        break;
        case 0x02:
        {
            rt6010_get_lra_resistance_ex(haptic_data, 65);
        }
        break;
        case 0x03:
        {
            rt6010_get_lra_resistance_ex(haptic_data, 110);
        }
        break;
    }
    haptic_data->lra_resistance *= 10;

    return 0;
}

static int32_t rt6010_get_irq_state(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_INT_STATUS, &reg_val);
    haptic_data->irq_state = reg_val;
    ics_info("irq state value = %02X\n", (uint8_t)haptic_data->irq_state);
    check_error_return(ret);

    return 0;
}

static bool rt6010_is_irq_play_done(struct ics_haptic_data *haptic_data)
{
    return ((haptic_data->irq_state & RT6010_BIT_INTS_PLAYDONE) > 0);
}

static bool rt6010_is_irq_fifo_ae(struct ics_haptic_data *haptic_data)
{
    return ((haptic_data->irq_state & RT6010_BIT_INTS_FIFO_AE) > 0);
}

static bool rt6010_is_irq_fifo_af(struct ics_haptic_data *haptic_data)
{
    return ((haptic_data->irq_state & RT6010_BIT_INTS_FIFO_AF) > 0);
}

static bool rt6010_is_irq_protection(struct ics_haptic_data *haptic_data)
{
    return ((haptic_data->irq_state & RT6010_BIT_INTS_PROTECTION) > 0);
}

static int32_t rt6010_clear_fifo(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    int32_t val, retry = 0;

    ret = regmap_write(haptic_data->regmap, RT6010_REG_RAM_CFG, 0x01);
    check_error_return(ret);

    do
    {
        ret = regmap_read(haptic_data->regmap, RT6010_REG_RAM_CFG, &val);
        check_error_return(ret);
    }while((val & 0x01) && (retry++ < 10));

    return 0;
}

static int32_t rt6010_clear_protection(struct ics_haptic_data *haptic_data)
{
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = regmap_read(haptic_data->regmap, RT6010_REG_BEMF_CFG2, &reg_val);
    check_error_return(ret);
    reg_val |= 0x02;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);
    reg_val &= 0xFD;
    ret = regmap_write(haptic_data->regmap, RT6010_REG_BEMF_CFG2, reg_val);
    check_error_return(ret);

    return 0;
}

struct ics_haptic_func rt6010_func_list =
{
    .chip_init = rt6010_chip_init,
    .get_chip_id = rt6010_get_chip_id,
    .get_reg = rt6010_get_reg,
    .set_reg = rt6010_set_reg,
    .get_f0 = rt6010_get_f0,
    .get_play_mode = rt6010_get_play_mode,
    .set_play_mode = rt6010_set_play_mode,
    .play_go = rt6010_play_go,
    .play_stop = rt6010_play_stop,
    .set_gain = rt6010_set_gain,
    .set_bst_vol = rt6010_set_bst_vol,
    .set_bst_mode = rt6010_set_bst_mode,
    .set_play_list = rt6010_set_play_list,
    .set_waveform_data = rt6010_set_waveform_data,
    .clear_stream_fifo = rt6010_clear_stream_fifo,
    .set_stream_data = rt6010_set_stream_data,
    .trig_init = rt6010_trig_init,
    .get_ram_data = rt6010_get_ram_data,
    .get_sys_state = rt6010_get_sys_state,
    .get_vbat = rt6010_get_vbat,
    .get_lra_resistance = rt6010_get_lra_resistance,
    .get_irq_state = rt6010_get_irq_state,
    .is_irq_play_done = rt6010_is_irq_play_done,
    .is_irq_fifo_ae = rt6010_is_irq_fifo_ae,
    .is_irq_fifo_af = rt6010_is_irq_fifo_af,
    .is_irq_protection = rt6010_is_irq_protection,
    .clear_fifo = rt6010_clear_fifo,
    .clear_protection = rt6010_clear_protection,
};
MODULE_LICENSE("GPL");
