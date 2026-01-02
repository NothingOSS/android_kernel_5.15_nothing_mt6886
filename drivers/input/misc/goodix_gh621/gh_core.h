
#ifndef _GH_CORE_H_
#define _GH_CORE_H_
#include <linux/kernel.h>
#include <linux/mutex.h>
#include "gh621x_common.h"

#define DRIVER_VERSION_MAJOR  		1
#define DRIVER_VERSION_MINOR  		7
#define DRIVER_VERSION_PATCH  		1
#define DRIVER_MAGIC_STR 		"20230808"

#define GH_I2C_NAME			"gh621x"

#define GOODIX_MAX_STR_LABEL_LEN	32
#define MAX_FW_FILE_SIZE			(40*1024)
#define MAX_CFG_FILE_SIZE			(4*1024)

enum power_mode {
	GH621X_MCU_PD = 0,
	GH621X_MCU_DONT_PD
};

#define UEVENT_REPORT
struct gh_core_data;
struct gh_event_data {
	int state[3];
	//int state;
};
struct gh_hw_ops {
	int (*dev_confirm)(struct gh_core_data *cd);
	int (*read)(struct gh_core_data *cd, u16 addr, u8 *data, unsigned int len, int mode);
	int (*write)(struct gh_core_data *cd, u16 addr, u8 *data, unsigned int len, int mode);
	int (*calibration)(struct gh_core_data *cd);
	int (*mode_ctrl)(struct gh_core_data *cd, int mode);
	int (*sw_reset)(struct gh_core_data *cd);
	int (*fw_update)(struct gh_core_data *cd, u8 *config, int cfg_len, u8 *patch, int patch_len);
	int (*work_start)(struct gh_core_data *cd);
	int (*work_stop)(struct gh_core_data *cd);
	int (*event_handler)(struct gh_core_data *cd, struct gh_event_data *event_data);
	int (*fw_ver)(struct gh_core_data *cd, u16 *version);
	int (*resume)(struct gh_core_data *cd);
	int (*suspend)(struct gh_core_data *cd, enum power_mode mode);
};

struct fw_data {
	u8 *data;
	int size;
};

enum goodix_core_init_stage {
	CORE_UNINIT,
	CORE_INIT_STAGE1,
	CORE_INIT_STAGE2
};

#define MAX_CHANNEL_NUM 9
#pragma  pack(1)
typedef struct
{
    short temp_cels;              /**< celsius */
    int temp_raw;           /**< temp raw */
    int pre_raw[MAX_CHANNEL_NUM];         /**< pre raw */
    int useful[MAX_CHANNEL_NUM];         /**< useful */
    int avg[MAX_CHANNEL_NUM];            /**< avg */
    int diff[MAX_CHANNEL_NUM];           /**< diff */
    u16 prox_stat;         /**< 9ch[0:8] status */
    u16 table_stat;       /**< 9ch[0:8]table status */
    u16 body_stat;        /**< 9ch[0:8] body status */
    u16 steady_stat;      /**< 9ch[0:8]steady status */
    u16 comp_stat;        /**< 9ch[0:8]comp status */
    u16 and_or_stat;       /**< and/or status[or:bit1, and:bit0] */
    u16 checksum;
} gh621x_rawdata_t;
#pragma  pack()


struct debug_data {
	u16 fw_version[GH621X_FW_VERSION_MAX_COUNT];
	u16 irq_status[GH621X_IRQ_STATUS_REG_MAX_COUNT];
	gh621x_rawdata_t rawdata;
	u16 ccsel_data[MAX_CHANNEL_NUM];
};

enum work_status {
    GH621X_STATUS_NO_INIT = 0,   /**< no init status */
    GH621X_STATUS_INITED,        /**< inited status */
    GH621X_STATUS_STARTED,       /**< started status */
    GH621X_STATUS_SUSPEND,       /**< suspend status, mcu_pd = 0 & workmode=0 */
    GH621X_STATUS_INVALID,       /**< invaild status */
};

struct gh_core_data {
	int init_stage;
	enum work_status status;
	struct i2c_client *client;
	struct regulator *avdd;
	char avdd_name[GOODIX_MAX_STR_LABEL_LEN];
	int power_on;
	int irq_gpio;
	int irq_trig_type;
	atomic_t irq_enabled;
	int rst_gpio;

	int irq_num;
	struct mutex mutex;

	struct input_dev *input_dev;

	struct fw_data patch;
	struct fw_data cfg;
	u16 fw_version[GH621X_FW_VERSION_MAX_COUNT];
	struct debug_data debug_data;

	struct gh_hw_ops *hw_ops;
};

enum PROXIMITY_STATE {
	STATE_CLOSE,
	STATE_FAR
};

enum GH621X_WORK_MODE {
	WORK_MODE_NORMAL,
	WORK_MODE_RAWDATA
};

typedef enum {
	GH621X_LOG_LEVEL_ERROR = 1,
	GH621X_LOG_LEVEL_WARNING = 2,
	GH621X_LOG_LEVEL_INFO = 3,
	GH621X_LOG_LEVEL_DEBUG = 4,
} GH621X_LOG_LEVEL_ENUM;

int gh_irq_enable(struct gh_core_data *cd, bool enable);

/*--------------utils funcs------------------*/
void gh_log_print(int level, const char *fmt, ...);
void gh_log_level_set(int level);
void gh_be16_to_cpu_array(u16 *buffer, int len);
void gh_be32_to_cpu_array(u16 *src, u32 *target, int num);
/*--------------utils end------------------*/

#define	gh_err(fmt, ...)	gh_log_print(GH621X_LOG_LEVEL_ERROR, "[gh621x_E]"fmt"\n", ##__VA_ARGS__)
#define gh_info(fmt, ...)	gh_log_print(GH621X_LOG_LEVEL_INFO, "[gh621x_I]"fmt"\n", ##__VA_ARGS__)
#define gh_debug(fmt, ...)	gh_log_print(GH621X_LOG_LEVEL_DEBUG, "[gh621x_D]"fmt"\n", ##__VA_ARGS__)

#endif