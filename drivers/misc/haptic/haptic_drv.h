#ifndef __ICS_HAPTIC_DRV_H__
#define __ICS_HAPTIC_DRV_H__

#include <linux/kfifo.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/types.h>

#define DEBUG
#define AAC_RICHTAP_SUPPORT

#define ics_err(format, ...) \
    pr_info("[ics_haptic]" format, ##__VA_ARGS__)

#define ics_info(format, ...) \
    pr_info("[ics_haptic]" format, ##__VA_ARGS__)

#define ics_dbg(format, ...) \
    pr_info("[ics_haptic]" format, ##__VA_ARGS__)

#define check_error_return(ret)     \
    if (ret < 0) {                  \
        pr_info("%s: check_error_return! ret = %d\n", __func__, ret);    \
        return ret;                 \
    }                               \

#ifdef TIMED_OUTPUT
#include "../staging/android/timed_output.h"
typedef struct timed_output_dev vib_dev_t;
#else
typedef struct led_classdev vib_dev_t;
#endif

#define ICS_HAPTIC_VERSION     "v0.9.3"
#define ICS_HAPTIC_NAME        "haptic_rt"
#define MAX_STREAM_FIFO_SIZE    4096
#define MAX_PRESET_NAME_LEN     64
#define DEFAULT_DEV_NAME        "vibrator"
#define DEFAULT_F0              1700

enum ics_haptic_play_mode
{
    PLAY_MODE_RAM           = 0x01,
    PLAY_MODE_STREAM        = 0x02,
    PLAY_MODE_TRACK         = 0x03,
};

enum ics_haptic_boost_mode
{
    BOOST_MODE_NORMAL       = 0x00,
    BOOST_MODE_BYPASS       = 0x01,
};

enum ics_hatpic_sys_state
{
    SYS_STATE_STOP          = 0x00,
    SYS_STATE_RAM_PLAY      = 0X01,
    SYS_STATE_STREAM_PLAY   = 0X02,
    SYS_STATE_TRACK_PLAY    = 0X03,
    SYS_STATE_TRIG          = 0x04,
    SYS_STATE_DETECT        = 0x05,
    SYS_STATE_BRAKE         = 0X06,
};

struct ics_haptic_chip_config
{
    uint32_t chip_id;
    uint32_t reg_size;
    uint32_t ram_size;
    uint32_t f0;
    uint32_t list_base_addr;
    uint32_t wave_base_addr;
    uint32_t fifo_ae;
    uint32_t fifo_af;
    uint32_t boost_mode;
    uint32_t gain;
    uint32_t boost_vol;

    uint32_t waveform_size;
    uint8_t waveform_data[];
};

struct ics_haptic_data
{
    uint32_t chip_id;
    uint32_t reg_size;
    uint32_t ram_size;
    uint32_t f0;
    uint32_t list_base_addr;
    uint32_t wave_base_addr;
    uint32_t fifo_ae;
    uint32_t fifo_af;
    uint32_t sys_state;
    uint32_t irq_state;
    uint32_t play_mode;
    uint32_t boost_mode;
    uint32_t gain;
    uint32_t boost_vol;
    uint32_t vbat;
    uint32_t lra_resistance;
    //
    int32_t amplitude;
    uint32_t preset_wave_index;
    uint32_t ram_wave_index;
    uint32_t duration;
    uint32_t activate_state;
    //
    bool chip_initialized;
    bool stream_start;
    struct kfifo stream_fifo;
    uint8_t *ram_buf;
    //
    vib_dev_t vib_dev;
    char vib_name[64];
    char misc_name[64];
    //
    struct i2c_client *client;
    struct device *dev;
    struct regmap *regmap;
    struct mutex lock;
    struct mutex preset_lock;
    struct hrtimer timer;
    struct delayed_work chip_init_work;
    struct work_struct vibrator_work;
    struct work_struct preset_work;
    struct ics_haptic_func *func;
    //
    int32_t gpio_en;
    int32_t gpio_irq;
    uint32_t efs_data;
    uint8_t efs_ver;
    uint8_t *config_data;
    uint32_t config_size;
};

struct ics_haptic_func
{
    int32_t (*chip_init)(struct ics_haptic_data *, const uint8_t*, uint32_t);
    int32_t (*get_chip_id)(struct ics_haptic_data *);
    int32_t (*get_reg)(struct ics_haptic_data *, uint32_t, uint32_t *);
    int32_t (*set_reg)(struct ics_haptic_data *, uint32_t, uint32_t);
    int32_t (*get_f0)(struct ics_haptic_data *);
    int32_t (*get_play_mode)(struct ics_haptic_data *);
    int32_t (*set_play_mode)(struct ics_haptic_data *, uint32_t);
    int32_t (*play_go)(struct ics_haptic_data *);
    int32_t (*play_stop)(struct ics_haptic_data *);
    int32_t (*set_gain)(struct ics_haptic_data *, uint32_t);
    int32_t (*set_bst_vol)(struct ics_haptic_data *, uint32_t);
    int32_t (*set_bst_mode)(struct ics_haptic_data *, uint32_t);
    int32_t (*set_play_list)(struct ics_haptic_data *, uint8_t *, uint32_t);
    int32_t (*set_waveform_data)(struct ics_haptic_data *, uint8_t *, uint32_t);
    int32_t (*clear_stream_fifo)(struct ics_haptic_data *);
    int32_t (*set_stream_data)(struct ics_haptic_data *, uint8_t *, uint32_t);
    int32_t (*trig_init)(struct ics_haptic_data *);
    int32_t (*get_ram_data)(struct ics_haptic_data *, uint8_t *, uint32_t *);
    int32_t (*get_sys_state)(struct ics_haptic_data *);
    int32_t (*get_vbat)(struct ics_haptic_data *);
    int32_t (*get_lra_resistance)(struct ics_haptic_data *);
    int32_t (*get_irq_state)(struct ics_haptic_data *);
    bool (*is_irq_play_done)(struct ics_haptic_data *);
    bool (*is_irq_fifo_ae)(struct ics_haptic_data *);
    bool (*is_irq_fifo_af)(struct ics_haptic_data *);
    bool (*is_irq_protection)(struct ics_haptic_data *);
    int32_t (*clear_fifo)(struct ics_haptic_data *);
    int32_t (*clear_protection)(struct ics_haptic_data *);
};

extern struct ics_haptic_func rt6010_func_list;

#ifdef AAC_RICHTAP_SUPPORT
#define DEFAULT_RICHTAP_NAME    "aac_richtap"
extern int32_t richtap_misc_register(struct ics_haptic_data *haptic_data);
extern int32_t richtap_misc_remove(struct ics_haptic_data *haptic_data);
extern int32_t richtap_irq_handler(void *data);
#endif

#endif // __ICS_HAPTIC_DRV_H__