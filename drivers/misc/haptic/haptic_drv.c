/*
 * drivers/haptic/haptic_drv.c
 *
 * Copyright (c) 2022 ICSense Semiconductor CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 */
#define  DEBUG

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

#include "haptic_drv.h"
#include "rt6010.h"

#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../hardware_info/hardware_info.h"
extern struct hardware_info current_line_motor_info;
#endif

char *haptic_config_name = "haptic_config.bin";
char preset_waveform_name[][MAX_PRESET_NAME_LEN] =
{
    {"100_Haptic.bin"},
    {"101_Haptic.bin"},
    {"102_Haptic.bin"},
    {"103_Haptic.bin"},
    {"104_Haptic.bin"},
    {"105_Haptic.bin"},
};
uint8_t flag_lra_resistance = 0;
int32_t haptic_hw_reset(struct ics_haptic_data *haptic_data);
static int32_t ics_str2hex(const char *str, uint32_t len,
    uint8_t *buf, uint32_t size)
{
    int32_t ret, i;
    char str_u8[3];
    uint32_t byte_count = len / 2;

    if (byte_count > size)
    {
        ics_info("%s source data size is bigger than hex buffer size\n", __func__);
        byte_count = size;
    }
    for (i = 0; i < byte_count; ++i)
    {
        strlcpy(str_u8, str + i * 2, sizeof(str_u8));
        ret = kstrtou8(str_u8, 16, &buf[i]);
        if (ret < 0)
        {
            ics_info("%s ics_str2hex data format error\n", __func__);
            break;
        }
    }

    return byte_count;
}

static ssize_t f0_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;

    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->get_f0(haptic_data);
    mutex_unlock(&haptic_data->lock);
    if (ret < 0)
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "f0 = 0\n");
    }
    else 
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "f0 = %u\n", haptic_data->f0);
    }
    return len;
}

static ssize_t reg_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;
    uint32_t reg_val = 0, i;

    len += snprintf(buf + len, PAGE_SIZE - len, "reg list:\n");
    for (i = 0; i < haptic_data->reg_size; i++)
    {
        ret = haptic_data->func->get_reg(haptic_data, i, &reg_val);
        check_error_return(ret);
        len += snprintf(buf + len, PAGE_SIZE - len, "0x%02X=0x%02X\n", (uint8_t)i, (uint8_t)reg_val);
    }

    return len;
}

static ssize_t reg_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t data_buf[2] = { 0, 0 };

    if (sscanf(buf, "%X %X", &data_buf[0], &data_buf[1]) == 2)
    {
        ret = haptic_data->func->set_reg(haptic_data, data_buf[0], data_buf[1]);
        check_error_return(ret);
    }

    return count;
}

static ssize_t vmax_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    ssize_t len = 0;

    len += snprintf(buf + len, PAGE_SIZE - len, "vmax = 0x%02X\n", haptic_data->boost_vol);
    return len;
}

static ssize_t vmax_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = kstrtouint(buf, 16, &reg_val);
    check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->set_bst_vol(haptic_data, reg_val);
    mutex_unlock(&haptic_data->lock);
    check_error_return(ret);
    haptic_data->boost_vol = reg_val;

    return count;
}

static ssize_t gain_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    ssize_t len = 0;

    len += snprintf(buf + len, PAGE_SIZE - len, "gain = %u\n", haptic_data->gain);
    return len;
}

static ssize_t gain_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = kstrtouint(buf, 16, &reg_val);
    check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->set_gain(haptic_data, reg_val);
    mutex_unlock(&haptic_data->lock);
    check_error_return(ret);
    haptic_data->gain = reg_val;

    return count;
}

static ssize_t rtp_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t index = 0;
    uint32_t preset_num = sizeof(preset_waveform_name) / MAX_PRESET_NAME_LEN;

    ret = kstrtouint(buf, 0, &index);
    check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    if (index < preset_num)
    {
        haptic_data->preset_wave_index = index;
        ics_info("preset_waveform_name[%u]: %s\n", index, preset_waveform_name[index]);
        schedule_work(&haptic_data->preset_work);
    }
    else
    {
        ics_err("%s: specified invalid preset waveform index : %d\n", __func__, index);
    }
    mutex_unlock(&haptic_data->lock);
    return count;
}

static ssize_t index_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    ssize_t len = 0;

    len += snprintf(buf + len, PAGE_SIZE - len, "index = %u\n", haptic_data->ram_wave_index);
    return len;
}

static ssize_t index_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t val = 0;
    uint8_t buf_play_list[6];

    ret = kstrtouint(buf, 0, &val);
    check_error_return(ret);

    haptic_data->ram_wave_index = val;
    buf_play_list[0] = 0x01;
    buf_play_list[1] = 0x00;
    buf_play_list[2] = 0x01;  //play once
    buf_play_list[3] = (uint8_t)haptic_data->ram_wave_index;
    buf_play_list[4] = 0x00;
    buf_play_list[5] = 0x00;
    haptic_data->func->set_play_list(haptic_data, buf_play_list, sizeof(buf_play_list));
    haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_RAM);
    haptic_data->func->play_go(haptic_data);

    return count;
}

static ssize_t duration_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);

    ktime_t time_rem;
    s64 time_ms = 0;

    if (hrtimer_active(&haptic_data->timer))
    {
        time_rem = hrtimer_get_remaining(&haptic_data->timer);
        time_ms = ktime_to_ms(time_rem);
    }
    return snprintf(buf, PAGE_SIZE, "duration = %lldms\n", time_ms);
}

static ssize_t duration_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t val = 0;

    ret = kstrtouint(buf, 0, &val);
    check_error_return(ret);

    // setting 0 on duration is NOP
    if (val > 0)
    {
        haptic_data->duration = val;
    }

    return count;
}

static ssize_t activate_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);

    return snprintf(buf, PAGE_SIZE, "activate = %d\n", haptic_data->activate_state);
}

static ssize_t activate_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int ret = 0;
    uint32_t val = 0;

    ret = kstrtouint(buf, 0, &val);
    check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    hrtimer_cancel(&haptic_data->timer);
    haptic_data->activate_state = val;
    mutex_unlock(&haptic_data->lock);
    schedule_work(&haptic_data->vibrator_work);

    return count;
}

static ssize_t playlist_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;
    uint32_t i, size = haptic_data->ram_size;

    if (haptic_data->ram_buf == NULL)
    {
        return 0;
    }

    ret = haptic_data->func->get_ram_data(haptic_data, haptic_data->ram_buf, &size);
    if (ret >= 0)
    {
        for(i = haptic_data->list_base_addr; i < haptic_data->wave_base_addr; i++)
        {
            len += snprintf(buf + len, PAGE_SIZE - len, "%02X", haptic_data->ram_buf[i]);
        }
    }
    check_error_return(ret);

    return len;
}

static ssize_t playlist_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t size = count / 2;

    if (haptic_data->ram_buf == NULL)
    {
        return 0;
    }

    size = ics_str2hex(buf, count, haptic_data->ram_buf, size);
    if (size > 0)
    {
        ret = haptic_data->func->set_play_list(haptic_data, haptic_data->ram_buf, size);
    }
    check_error_return(ret);

    return count;
}

static ssize_t waveform_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;
    uint32_t i, size = haptic_data->ram_size;

    if (haptic_data->ram_buf == NULL)
    {
        return 0;
    }

    ret = haptic_data->func->get_ram_data(haptic_data, haptic_data->ram_buf, &size);
    if (ret >= 0)
    {
        for(i = haptic_data->wave_base_addr; i < haptic_data->ram_size; i++)
        {
            len += snprintf(buf + len, PAGE_SIZE - len, "%02X", haptic_data->ram_buf[i]);
        }
    }
    check_error_return(ret);

    return len;
}

static ssize_t waveform_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t size = count / 2;

    if (haptic_data->ram_buf == NULL)
    {
        return 0;
    }

    size = ics_str2hex(buf, count, haptic_data->ram_buf, size);
    if (size > 0)
    {
        ret = haptic_data->func->set_waveform_data(haptic_data, haptic_data->ram_buf, size);
    }
    check_error_return(ret);

    return count;
}

static ssize_t play_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t reg_val = 0;

    ret = kstrtouint(buf, 16, &reg_val);
    check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    if ((reg_val & 0x01) > 0)
    {
        ret = haptic_data->func->play_go(haptic_data);
    }
    else
    {
        ret = haptic_data->func->play_stop(haptic_data);
    }
    mutex_unlock(&haptic_data->lock);
    check_error_return(ret);

    return count;
}

static ssize_t stream_start_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t start;

    ret = kstrtouint(buf, 16, &start);
    //check_error_return(ret);

    mutex_lock(&haptic_data->lock);
    haptic_data->stream_start = true;
    kfifo_reset(&haptic_data->stream_fifo);
    haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_STREAM);
    haptic_data->func->clear_stream_fifo(haptic_data);
    haptic_data->func->play_go(haptic_data);
    mutex_unlock(&haptic_data->lock);

    return count;
}

static ssize_t stream_data_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    ssize_t len = 0;
    uint32_t count;

    count = kfifo_avail(&haptic_data->stream_fifo);
    len += snprintf(buf + len, PAGE_SIZE - len, "%u", count);

    return len;
}

static int32_t send_stream_data(struct ics_haptic_data *haptic_data, uint32_t fifo_available_size)
{
    int32_t ret = 0;
    uint32_t buf_fifo_used = 0, size;

    if (haptic_data->ram_buf == NULL)
    {
        return 0;
    }

    buf_fifo_used = kfifo_len(&haptic_data->stream_fifo);
    size = min(fifo_available_size, buf_fifo_used);
    size = kfifo_out(&haptic_data->stream_fifo, haptic_data->ram_buf, size);
    if (size > 0)
    {
        ret = haptic_data->func->set_stream_data(haptic_data, haptic_data->ram_buf, size);
    }
    check_error_return(ret);

    return 0;
}

static ssize_t stream_data_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    uint32_t size = count / 2, chip_fifo_size = haptic_data->list_base_addr;
    uint32_t available = kfifo_avail(&haptic_data->stream_fifo);
    uint8_t *data_buf = kmalloc(size, GFP_KERNEL);
    if (data_buf == NULL)
    {
        ics_err("%s: failed to allocate memory\n", __func__);
        return -ENOMEM;
    }

    if (size > available)
    {
        ics_dbg("stream data size is bigger than stream fifo available size! \
            available=%u, size=%u\n", available, size);
        size = available;
    }
    size = ics_str2hex(buf, size * 2, data_buf, size);
    if (size > 0)
    {
        kfifo_in(&haptic_data->stream_fifo, data_buf, size);
    }
    kfree(data_buf);

    if (haptic_data->stream_start)
    {
        haptic_data->stream_start = false;

        ret = send_stream_data(haptic_data, chip_fifo_size);
        check_error_return(ret);
    }

    return count;
}

static ssize_t vbat_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;;

    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->get_vbat(haptic_data);
    mutex_unlock(&haptic_data->lock);
    if (ret < 0)
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "vbat = 0\n");
    }
    else 
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "vbat = %u\n", haptic_data->vbat);
    }
    return len;
}

static ssize_t lra_resistance_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;

    flag_lra_resistance = 1;
    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->get_lra_resistance(haptic_data);
    mutex_unlock(&haptic_data->lock);
    flag_lra_resistance = 0;
    if (ret < 0)
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = 0\n");
    }
    else
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "lra_resistance = %u\n", haptic_data->lra_resistance);
    }
    return len;
}

static ssize_t state_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);
    int32_t ret = 0;
    ssize_t len = 0;

    mutex_lock(&haptic_data->lock);
    ret = haptic_data->func->get_sys_state(haptic_data);
    mutex_unlock(&haptic_data->lock);
    if (ret < 0)
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "state = 0\n");
    }
    else 
    {
        len += snprintf(buf + len, PAGE_SIZE - len, "state = %u\n", haptic_data->sys_state);
    }
    return len;
}

static ssize_t reset_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    vib_dev_t *vdev = dev_get_drvdata(dev);
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);

    if (gpio_is_valid(haptic_data->gpio_en))
    {
        haptic_hw_reset(haptic_data);
        ics_info("hardware reset successfully!\n");
    }
    else
    {
        ics_info("hardware reset gpio is NOT valid!\n");
    }

    return count;
}

static ssize_t f0_save_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
    ssize_t len = 0;

    len += snprintf(buf + len, PAGE_SIZE - len, "f0_cali_data = 10\n");
    return len;
}

static ssize_t cali_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    ics_info("hardware cali_store NOT valid!\n");
    return count;
}

///////////////////////////////////////////////////////////////////////////////
// haptic sys attribute nodes
///////////////////////////////////////////////////////////////////////////////
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, f0_show, NULL);
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, reg_show, reg_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, vmax_show, vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, gain_show, gain_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, NULL, rtp_store);
static DEVICE_ATTR(playlist, S_IWUSR | S_IRUGO, playlist_show, playlist_store);
static DEVICE_ATTR(waveform, S_IWUSR | S_IRUGO, waveform_show, waveform_store);
static DEVICE_ATTR(play, S_IWUSR | S_IRUGO, NULL, play_store);
static DEVICE_ATTR(stream_start, S_IWUSR | S_IRUGO, NULL, stream_start_store);
static DEVICE_ATTR(stream_data, S_IWUSR | S_IRUGO, stream_data_show, stream_data_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, index_show, index_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, duration_show, duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, activate_show, activate_store);
static DEVICE_ATTR(vbat, S_IWUSR | S_IRUGO, vbat_show, NULL);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO, lra_resistance_show, NULL);
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, state_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, NULL, reset_store);
static DEVICE_ATTR(f0_save, S_IWUSR | S_IRUGO, f0_save_show, NULL);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, NULL, cali_store);

static struct attribute *ics_haptic_attributes[] = {
    &dev_attr_f0.attr,
    &dev_attr_reg.attr,
    &dev_attr_vmax.attr,
    &dev_attr_gain.attr,
    &dev_attr_rtp.attr,
    &dev_attr_playlist.attr,
    &dev_attr_waveform.attr,
    &dev_attr_play.attr,
    &dev_attr_stream_start.attr,
    &dev_attr_stream_data.attr,
    &dev_attr_index.attr,
    &dev_attr_duration.attr,
    &dev_attr_activate.attr,
    &dev_attr_vbat.attr,
    &dev_attr_lra_resistance.attr,
    &dev_attr_state.attr,
    &dev_attr_reset.attr,
    &dev_attr_f0_save.attr,
    &dev_attr_cali.attr,
    NULL
};

static struct attribute_group ics_haptic_attribute_group = {
    .attrs = ics_haptic_attributes
};

static irqreturn_t ics_haptic_irq_handler(int irq, void *data)
{
    struct ics_haptic_data *haptic_data = data;
    int32_t ret = 0;
    uint32_t data_size, fifo_used;

    if (flag_lra_resistance == 1)
    {
        return IRQ_HANDLED;
    }

    ret = haptic_data->func->get_irq_state(haptic_data);
    ics_dbg("%s: irq state = 0x%02X\n", __func__, (uint8_t)(haptic_data->irq_state));
    if (ret < 0)
    {
        goto irq_exit;
    }

    if (haptic_data->func->is_irq_protection(haptic_data))
    {
        ret = haptic_data->func->clear_protection(haptic_data);
        return ret;
    }

#ifdef AAC_RICHTAP_SUPPORT
    ret = richtap_irq_handler(haptic_data);
    if (ret >= 0)
    {
         return IRQ_HANDLED;
    }
#endif

    if (haptic_data->func->is_irq_fifo_ae(haptic_data))
    {
        if (haptic_data->ram_buf == NULL)
        {
            goto irq_exit;
        }
        data_size = haptic_data->list_base_addr - haptic_data->fifo_ae;
        fifo_used = kfifo_len(&haptic_data->stream_fifo);
        data_size = min(data_size, fifo_used);
        data_size = kfifo_out(&haptic_data->stream_fifo, haptic_data->ram_buf, data_size);
        if (data_size > 0)
        {
            ret = haptic_data->func->set_stream_data(haptic_data, haptic_data->ram_buf, data_size);
            if (ret < 0)
            {
                goto irq_exit;
            }
        }
    }

irq_exit:
    return IRQ_HANDLED;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
    struct ics_haptic_data *haptic_data = container_of(timer, struct ics_haptic_data, timer);

    haptic_data->activate_state = 0;
    schedule_work(&haptic_data->vibrator_work);

    return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct work_struct *work)
{
    struct ics_haptic_data *haptic_data = container_of(work, struct ics_haptic_data,
                           vibrator_work);
    uint8_t buf[6];

    mutex_lock(&haptic_data->lock);
    haptic_data->func->play_stop(haptic_data);
    if (haptic_data->activate_state)
    {
        buf[0] = 0x01;
        buf[1] = 0x00;
        buf[2] = 0x7F;
        buf[3] = 0x00;  //fixed num0 waveform buf[3] = (uint8_t)haptic_data->ram_wave_index;
        buf[4] = 0x00;
        buf[5] = 0x00;
        haptic_data->func->set_play_list(haptic_data, buf, sizeof(buf));
        haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_RAM);
        haptic_data->func->play_go(haptic_data);
        // run ms time
        hrtimer_start(&haptic_data->timer, ktime_set(haptic_data->duration / 1000,
            (haptic_data->duration % 1000) * 1000000), HRTIMER_MODE_REL);
    }
    mutex_unlock(&haptic_data->lock);
}

static void preset_work_routine(struct work_struct *work)
{
    struct ics_haptic_data *haptic_data = container_of(work, struct ics_haptic_data, preset_work);
    int32_t ret = 0;
    const struct firmware *preset_file;
    uint32_t chip_fifo_size = haptic_data->list_base_addr;

    mutex_lock(&haptic_data->preset_lock);
    ret = request_firmware(&preset_file,
                   preset_waveform_name[haptic_data->preset_wave_index],
                   haptic_data->dev);
    if (ret < 0)
    {
        ics_err("%s: failed to read preset file %s\n", __func__,
               preset_waveform_name[haptic_data->preset_wave_index]);
        mutex_unlock(&haptic_data->preset_lock);
        return;
    }
    if (preset_file->size > MAX_STREAM_FIFO_SIZE)
    {
        kfifo_free(&haptic_data->stream_fifo);
        ret = kfifo_alloc(&haptic_data->stream_fifo, preset_file->size, GFP_KERNEL);
        if (ret < 0)
        {
            ics_err("%s: failed to allocate fifo for stream!\n", __func__);
            return;
        }
    }
    kfifo_reset(&haptic_data->stream_fifo);
    kfifo_in(&haptic_data->stream_fifo, preset_file->data, preset_file->size);
    mutex_unlock(&haptic_data->preset_lock);
    release_firmware(preset_file);

    mutex_lock(&haptic_data->lock);
    haptic_data->func->play_stop(haptic_data);
    haptic_data->func->get_irq_state(haptic_data);
    haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_STREAM);
    haptic_data->func->play_go(haptic_data);

    send_stream_data(haptic_data, chip_fifo_size);
    mutex_unlock(&haptic_data->lock);
}

#ifdef TIMED_OUTPUT
static int vibrator_get_time(struct timed_output_dev *dev)
{
    struct ics_haptic_data *haptic_data = container_of(dev, struct ics_haptic_data, vib_dev);

    if (hrtimer_active(&haptic_data->timer))
    {
        ktime_t r = hrtimer_get_remaining(&haptic_data->timer);
        return ktime_to_ms(r);
    }
    return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
    struct ics_haptic_data *haptic_data = container_of(dev, struct ics_haptic_data, vib_dev);

    mutex_lock(&haptic_data->lock);

    haptic_data->func->play_stop(haptic_data);
    if (value > 0)
    {
        //TODO:
        haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_RAM);
        haptic_data->func->play_go(haptic_data);
    }
    mutex_unlock(&haptic_data->lock);
}
#else
static enum led_brightness brightness_get(struct led_classdev *vdev)
{
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);

    return haptic_data->amplitude;
}

static void brightness_set(struct led_classdev *vdev, enum led_brightness level)
{
    struct ics_haptic_data *haptic_data = container_of(vdev, struct ics_haptic_data, vib_dev);

    haptic_data->amplitude = level;
    mutex_lock(&haptic_data->lock);
    haptic_data->func->play_stop(haptic_data);
    if (haptic_data->amplitude > 0)
    {
        // TODO: Build map between amplitude and gain
        haptic_data->func->set_play_mode(haptic_data, PLAY_MODE_RAM);
        haptic_data->func->play_go(haptic_data);
    }
    mutex_unlock(&haptic_data->lock);
}
#endif

static int32_t vibrator_init(struct ics_haptic_data *haptic_data)
{
    int ret = 0;

#ifdef TIMED_OUTPUT
    ics_info("%s: TIMED_OUTPUT framework!\n", __func__);
    haptic_data->vib_dev.name = haptic_data->vib_name;
    haptic_data->vib_dev.get_time = vibrator_get_time;
    haptic_data->vib_dev.enable = vibrator_enable;

    ret = timed_output_dev_register(&haptic_data->vib_dev);
    if (ret < 0)
    {
        ics_err("%s: failed to create timed output dev!\n", __func__);
        return ret;
    }
    ret = sysfs_create_group(&haptic_data->vib_dev.dev->kobj,
                 &ics_haptic_attribute_group);
    if (ret < 0)
    {
        ics_err("%s: failed to create sysfs attr files!\n", __func__);
        return ret;
    }
#else
    ics_info("%s: led cdev framework!\n", __func__);
    haptic_data->vib_dev.name = haptic_data->vib_name;
    haptic_data->vib_dev.brightness_get = brightness_get;
    haptic_data->vib_dev.brightness_set = brightness_set;
    ret = devm_led_classdev_register(&haptic_data->client->dev, &haptic_data->vib_dev);
    if (ret < 0)
    {
        ics_err("%s: fail to create led dev\n", __func__);
        return ret;
    }
    ret = sysfs_create_group(&haptic_data->vib_dev.dev->kobj, &ics_haptic_attribute_group);
    if (ret < 0)
    {
        ics_err("%s: error creating sysfs attr files\n", __func__);
        return ret;
    }
#endif
    hrtimer_init(&haptic_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    haptic_data->timer.function = vibrator_timer_func;
    INIT_WORK(&haptic_data->vibrator_work, vibrator_work_routine);
    INIT_WORK(&haptic_data->preset_work, preset_work_routine);
    mutex_init(&haptic_data->lock);
    mutex_init(&haptic_data->preset_lock);

    return 0;
}

static void load_chip_config(const struct firmware *config_fw, void *context)
{
    struct ics_haptic_data *haptic_data = context;
    int ret = 0;

    ics_info("load chip config\n");

    if (!config_fw)
    {
        ics_err("%s: failed to read %s\n", __func__, haptic_config_name);
        release_firmware(config_fw);
        return;
    }

    haptic_data->config_data = (uint8_t *)vmalloc(config_fw->size);
    if (haptic_data->config_data == NULL)
    {
        ics_err("%s: failed to allocate memory for config data\n", __func__);
        return;
    }
    memcpy(haptic_data->config_data, config_fw->data, config_fw->size);
    haptic_data->config_size = config_fw->size;

    ics_info("chip config firmware size = %lu\n", config_fw->size);
    ret = haptic_data->func->chip_init(haptic_data, config_fw->data, config_fw->size);
    if (ret)
    {
        ics_err("%s: failed to initialize chip!\n", __func__);
    }
    else
    {
        haptic_data->chip_initialized = true;
    }
    release_firmware(config_fw);

    if (haptic_data->ram_size > 0)
    {
        haptic_data->ram_buf = kmalloc(haptic_data->ram_size, GFP_KERNEL);
        if (haptic_data->ram_buf == NULL)
        {
            ics_err("%s: failed to allocate memory for ram buffer\n", __func__);
        }
    }
}

static void chip_init_work_routine(struct work_struct *work)
{
    struct ics_haptic_data *haptic_data = container_of(work, struct ics_haptic_data, chip_init_work.work);

    haptic_data->chip_initialized = false;
    request_firmware_nowait(THIS_MODULE, FW_ACTION_UEVENT,
        haptic_config_name, haptic_data->dev, GFP_KERNEL,
        haptic_data, load_chip_config);
}

static void initialize_chip(struct ics_haptic_data *haptic_data)
{
    int ram_timer_val = 8000;

    INIT_DELAYED_WORK(&haptic_data->chip_init_work, chip_init_work_routine);
    schedule_delayed_work(&haptic_data->chip_init_work, msecs_to_jiffies(ram_timer_val));
}

static int32_t haptic_parse_dt(struct ics_haptic_data *haptic_data)
{
    const char *str_val = NULL;
    struct device_node *dev_node = haptic_data->dev->of_node;
    if (NULL == dev_node)
    {
        ics_err("%s: no device tree node was found\n", __func__);
        return -EINVAL;
    }

    haptic_data->gpio_en = of_get_named_gpio(dev_node, "gpio-en", 0);
    if (haptic_data->gpio_en < 0)
    {
        ics_err("%s: no gpio-en provided\n", __func__);
        return -EPERM;
    }
    else
    {
        ics_info("gpio-en provided ok\n");
    }

    haptic_data->gpio_irq = of_get_named_gpio(dev_node, "gpio-irq", 0);
    if (haptic_data->gpio_irq < 0)
    {
        ics_err("%s: no gpio-irq provided\n", __func__);
        return -EPERM;
    }
    else
    {
        ics_info("gpio-irq provided ok\n");
    }

    if (of_property_read_string(dev_node, "device-name", &str_val))
    {
        ics_err("%s: can NOT find device name in DT!\n", __func__);
        memcpy(haptic_data->vib_name, DEFAULT_DEV_NAME, sizeof(DEFAULT_DEV_NAME));
    }
    else
    {
        memcpy(haptic_data->vib_name, str_val, strlen(str_val));
        ics_info("provided device name is : %s\n", haptic_data->vib_name);
    }

    if (of_property_read_string(dev_node, "richtap-name", &str_val))
    {
        ics_err("%s: can NOT find richtap name in DT!\n", __func__);
        memcpy(haptic_data->misc_name, DEFAULT_RICHTAP_NAME, sizeof(DEFAULT_RICHTAP_NAME));
    }
    else
    {
        memcpy(haptic_data->misc_name, str_val, strlen(str_val));
        ics_info("provided richtap name is : %s\n", haptic_data->misc_name);
    }

    return 0;
}

int32_t haptic_hw_reset(struct ics_haptic_data *haptic_data)
{
    ics_info("haptic hw reset!\n");
    gpio_set_value_cansleep(haptic_data->gpio_en, 0);
    gpio_set_value_cansleep(haptic_data->gpio_irq, 0);
    usleep_range(1000, 2000);
    gpio_set_value_cansleep(haptic_data->gpio_en, 1);
    gpio_set_value_cansleep(haptic_data->gpio_irq, 1);
    usleep_range(1000, 2000);
    ics_info("haptic hw reset end!\n");
    return 0;
}

static struct regmap_config ics_haptic_regmap =
{
    .reg_bits = 8,
    .val_bits = 8,
};

static int ics_haptic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int32_t ret = 0;
    struct ics_haptic_data *haptic_data;
    struct device* dev = &client->dev;

    ics_info("ics haptic probe! addr=0x%X\n", client->addr);

    if ((strlen(current_line_motor_info.chip) > 0) && (!strcmp(current_line_motor_info.chip, "AW86927")))
    {
        ics_err("%s: AW86927 exist ics i2c ignore!\n", __func__);
        return -EIO;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        ics_err("%s: failed to check i2c functionality!\n", __func__);
        return -EIO;
    }

    haptic_data = devm_kzalloc(&client->dev, sizeof(struct ics_haptic_data), GFP_KERNEL);
    if (NULL == haptic_data)
    {
        ics_err("%s: failed to allocate memory for ics haptic data!\n", __func__);
        ret = -ENOMEM;
        goto probe_err;
    }

    haptic_data->dev = dev;
    haptic_data->client = client;
    dev_set_drvdata(dev, haptic_data);
    i2c_set_clientdata(client, haptic_data);

    haptic_data->regmap = devm_regmap_init_i2c(client, &ics_haptic_regmap);
    if (IS_ERR(haptic_data->regmap))
    {
        ret = PTR_ERR(haptic_data->regmap);
        ics_err("%s: failed to initialize register map: %d\n", __func__, ret);
        goto probe_err;
    }

    // TODO: assign function list according to chip id
    haptic_data->func = &rt6010_func_list;
    // enable and irp gpio configuration
    ret = haptic_parse_dt(haptic_data);
    if (ret < 0)
    {
        ics_err("%s: failed to parse device tree: %d\n", __func__, ret);
        goto probe_err;
    }
    haptic_hw_reset(haptic_data);

    ret = haptic_data->func->get_chip_id(haptic_data);
    if (ret < 0)
    {
        ics_err("%s: failed to get chipid!\n", __func__);
        goto probe_err;
    }

    // following initialization steps are not necessary for group broadcast device
    if (client->addr == 0x5C)
    {
        return ret;
    }

    ret = kfifo_alloc(&haptic_data->stream_fifo, MAX_STREAM_FIFO_SIZE, GFP_KERNEL);
    if (ret < 0)
    {
        ics_err("%s: failed to allocate fifo for stream!\n", __func__);
        ret = -ENOMEM;
        goto probe_err;
    }

    // register irq handler
    ret = devm_request_threaded_irq(dev, gpio_to_irq(haptic_data->gpio_irq),
        NULL, ics_haptic_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
        ICS_HAPTIC_NAME, haptic_data);
    if (ret < 0)
    {
        ics_err("%s: failed to request threaded irq! ret = %d\n", __func__, ret);
        goto probe_err;
    }

    ret = vibrator_init(haptic_data);
    if (ret < 0)
    {
        ics_err("%s: failed to initialize vibrator interfaces! ret = %d\n", __func__, ret);
        goto probe_err;
    }
    initialize_chip(haptic_data);


#ifdef AAC_RICHTAP_SUPPORT
    ret = richtap_misc_register(haptic_data);
    if (ret < 0)
    {
        ics_err("%s: failed to initialize richtap device! ret = %d\n", __func__, ret);
        goto probe_err;
    }
#endif

#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
    sprintf(current_line_motor_info.id,"0x%x", haptic_data->chip_id);
    strcpy(current_line_motor_info.vendor, "aac");
    strcpy(current_line_motor_info.chip, "RT6010");
#endif

    return 0;

probe_err:
    kfifo_free(&haptic_data->stream_fifo);
    if (haptic_data->ram_buf != NULL)
    {
        kfree(haptic_data->ram_buf);
    }

    if (haptic_data->config_data != NULL)
    {
        vfree(haptic_data->config_data);
    }

    devm_free_irq(&client->dev, gpio_to_irq(haptic_data->gpio_irq), haptic_data);
    if (gpio_is_valid(haptic_data->gpio_en))
    {
        devm_gpio_free(&client->dev, haptic_data->gpio_en);
    }
    if (haptic_data != NULL)
    {
        devm_kfree(&client->dev, haptic_data);
    }

    return ret;
}

static int ics_haptic_remove(struct i2c_client *client)
{
    struct ics_haptic_data *haptic_data = i2c_get_clientdata(client);

#ifdef AAC_RICHTAP_SUPPORT
    richtap_misc_remove(haptic_data);
#endif
    cancel_work_sync(&haptic_data->preset_work);
    cancel_work_sync(&haptic_data->vibrator_work);
    hrtimer_cancel(&haptic_data->timer);
    mutex_destroy(&haptic_data->lock);
    mutex_destroy(&haptic_data->preset_lock);

    kfifo_free(&haptic_data->stream_fifo);
    if (haptic_data->ram_buf != NULL)
    {
        kfree(haptic_data->ram_buf);
    }

    if (haptic_data->config_data != NULL)
    {
        vfree(haptic_data->config_data);
    }
    
    devm_free_irq(&client->dev, gpio_to_irq(haptic_data->gpio_irq), haptic_data);
    if (gpio_is_valid(haptic_data->gpio_en))
    {
        devm_gpio_free(&client->dev, haptic_data->gpio_en);
    }

    return 0;
}

static int __maybe_unused ics_haptic_suspend(struct device *dev)
{
    int ret = 0;

    return ret;
}

static int __maybe_unused ics_haptic_resume(struct device *dev)
{
    int ret = 0;

    return ret;
}

static SIMPLE_DEV_PM_OPS(ics_haptic_pm_ops, ics_haptic_suspend, ics_haptic_resume);
static const struct i2c_device_id ics_haptic_id[] = {
    { ICS_HAPTIC_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ics_haptic_id);

static struct of_device_id ics_haptic_dt_match[] = {
    { .compatible = "ics,haptic_rt" },
    { },
};

static struct i2c_driver ics_haptic_driver = {
    .driver = {
        .name = ICS_HAPTIC_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ics_haptic_dt_match),
        .pm = &ics_haptic_pm_ops,
    },
    .id_table = ics_haptic_id,
    .probe = ics_haptic_probe,
    .remove = ics_haptic_remove,
};

module_i2c_driver(ics_haptic_driver);

MODULE_DESCRIPTION("ICS Haptic Driver");
MODULE_AUTHOR("chenmaomao@icsense.com.cn, ICSense Semiconductor Co., Ltd");
MODULE_LICENSE("GPL v2");
