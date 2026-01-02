#ifndef _AW_SAR_PLAT_HW_INTERFACE_H_
#define _AW_SAR_PLAT_HW_INTERFACE_H_
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
#include <linux/uaccess.h>
#include <linux/syscalls.h>
#include <linux/string.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/kern_levels.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/decompress/mm.h>

enum aw_sar_chip_list_t {
	AW_SAR_NONE_CHECK_CHIP,

	SAR_AW9610X = 1 << 1,
	SAR_AW9610XA = 1 << 2,

	SAR_AW96203 = 1 << 3,
	SAR_AW96205 = 1 << 4,
	SAR_AW96208 = 1 << 5,

	SAR_AW96303 = 1 << 6,
	SAR_AW96305 = 1 << 7,
	SAR_AW96308 = 1 << 8,
	SAR_AW96310 = 1 << 9,
	SAR_AW96312 = 1 << 10,
};

enum AW_SAR_UPDATE_FW_MODE {
	PROT_UPDATE_FW,
	REG_UPDATE_FW,
};

#define AW_SAR_LABLE 		" "
#define AW_SAR_FILENAME(x)		strrchr((x), '/') ? (strrchr((x), '/') + 1) : (x)

#define AW_SAR_DEBUG_LOG
#ifdef AW_SAR_DEBUG_LOG
#define AWLOGD(dev, format, arg...) \
	do {\
		dev_printk(KERN_DEBUG, dev, \
			AW_SAR_LABLE"[%s:%s:%d] "format"\n", AW_SAR_FILENAME(__FILE__), __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGD(dev, format, arg...)
#endif

#define AW_SAR_INFO_LOG
#ifdef AW_SAR_INFO_LOG
#define AWLOGI(dev, format, arg...) \
	do {\
		dev_printk(KERN_INFO, dev, \
			AW_SAR_LABLE"[%s:%s:%d] "format"\n", AW_SAR_FILENAME(__FILE__), __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGI(dev, format, arg...)
#endif

#define AW_SAR_ERR_LOG
#ifdef AW_SAR_ERR_LOG
#define AWLOGE(dev, format, arg...) \
	do {\
		dev_printk(KERN_ERR, dev, \
			AW_SAR_LABLE"[%s:%s:%d] "format"\n", AW_SAR_FILENAME(__FILE__), __func__, __LINE__, ##arg);\
	} while (0)
#else
#define	AWLOGE(dev, format, arg...)
#endif

#ifndef AW_TRUE
#define AW_TRUE					(1)
#endif

#ifndef AW_FALSE
#define AW_FALSE				(0)
#endif

/*
typedef 	unsigned char	uint8_t;
typedef 	unsigned short	uint16_t;
typedef 	unsigned int	uint32_t;

typedef 	char		int8_t;
typedef 	short		int16_t;
typedef 	int		int32_t;

typedef		uint8_t		bool;
*/
enum aw_sar_rst_val {
	AW_OK,
	AW_ERR,
	AW_PARA_ERR,
	AW_VERS_ERR,
	AW_ERR_CHIPID,
	AW_ERR_IRQ_INIT_OVER,
	AW_PROT_UPDATE_ERR,
	AW_REG_LOAD_ERR,
	AW_INVALID_PARA,
	AW_BIN_PARA_INVALID,

	AW_FW_CHECK_ERR,
	AW_BT_CHECK_ERR,
	AW_OTEHR_CHECK_ERR,
	AW_BIN_NAME_CHECK_ERR,
};

#define AW_SAR_FLIP_U16(value) ((((value) & (0x00FF)) << (8)) | (((value) & (0xFF00)) >> (8)))
#define AW_SAR_FLIP_U32(value) ((((value) & (0x000000FF)) << (24)) | ((((value) & (0x0000FF00))) << (8)) | (((value) & (0x00FF0000)) >> (8)) | (((value) & (0xFF000000)) >> (24)))

#ifndef OFFSET_BIT_0
#define OFFSET_BIT_0				(0)
#endif

#ifndef OFFSET_BIT_8
#define OFFSET_BIT_8				(8)
#endif

#ifndef OFFSET_BIT_16
#define OFFSET_BIT_16				(16)
#endif

#ifndef OFFSET_BIT_24
#define OFFSET_BIT_24				(24)
#endif

#ifndef WORD_LEN
#define WORD_LEN				(4)
#endif

#ifndef GET_BITS_7_0
#define GET_BITS_7_0				(0x00FF)
#endif

#ifndef GET_BITS_15_8
#define GET_BITS_15_8				(0xFF00)
#endif

#ifndef GET_BITS_24_16
#define GET_BITS_24_16				(0x00FF0000)
#endif

#ifndef GET_BITS_31_25
#define GET_BITS_31_25				(0xFF000000)
#endif

#define AW_SAR_ABS(x) (((x) > 0) ? (x) : (-(x)))
#define AW_SAR_GET_32_DATA(w, x, y, z)              ((unsigned int)(((w) << 24) | ((x) << 16) | ((y) << 8) | (z)))
#define AW_SAR_GET_16_DATA(a, b) 	                (((uint32_t)(a) << (8)) | ((uint32_t)(b) << (0)))

enum AW_SAR_HOST_IRQ_STAT {
	IRQ_ENABLE,
	IRQ_DISABLE,
};

#define AW_SAR_BIN_NUM_MAX   100
#define AW_SAR_HEADER_LEN    60

struct bin_header_info {
	unsigned int header_len; /* Frame header length */
	unsigned int check_sum; /* Frame header information-Checksum */
	unsigned int header_ver; /* Frame header information-Frame header version */
	unsigned int bin_data_type; /* Frame header information-Data type */
	unsigned int bin_data_ver; /* Frame header information-Data version */
	unsigned int bin_data_len; /* Frame header information-Data length */
	unsigned int ui_ver; /* Frame header information-ui version */
	unsigned char chip_type[8]; /* Frame header information-chip type */
	unsigned int reg_byte_len; /* Frame header information-reg byte len */
	unsigned int data_byte_len; /* Frame header information-data byte len */
	unsigned int device_addr; /* Frame header information-device addr */
	unsigned int valid_data_len; /* Length of valid data obtained after parsing */
	unsigned int valid_data_addr; /* The offset address of the valid data obtained after parsing relative to info */

	unsigned int reg_num; /* The number of registers obtained after parsing */
	unsigned int reg_data_byte_len; /* The byte length of the register obtained after parsing */
	unsigned int download_addr; /* The starting address or download address obtained after parsing */
	unsigned int app_version; /* The software version number obtained after parsing */
};

struct bin_container {
	unsigned int len; /* The size of the bin file obtained from the firmware */
	unsigned char data[]; /* Store the bin file obtained from the firmware */
};

struct aw_bin {
	char *p_addr; /* Offset pointer (backward offset pointer to obtain frame header information and important information) */
	unsigned int all_bin_parse_num; /* The number of all bin files */
	unsigned int multi_bin_parse_num; /* The number of single bin files */
	unsigned int single_bin_parse_num; /* The number of multiple bin files */
	struct bin_header_info header_info[AW_SAR_BIN_NUM_MAX]; /* Frame header information and other important data obtained after parsing */
	struct bin_container info; /* Obtained bin file data that needs to be parsed */
};

//I2C communication API
extern int32_t aw_sar_i2c_read(struct i2c_client *i2c, uint16_t reg_addr16,  uint32_t *reg_data32);
extern int32_t aw_sar_i2c_write(struct i2c_client *i2c, uint16_t reg_addr16, uint32_t reg_data32);
extern int32_t aw_sar_i2c_write_bits(struct i2c_client *i2c, uint16_t reg_addr16, uint32_t mask, uint32_t val);
extern int32_t aw_sar_i2c_write_seq(struct i2c_client *i2c, uint8_t *tr_data, uint16_t len);
extern int32_t aw_sar_i2c_read_seq(struct i2c_client *i2c, uint8_t *addr, uint8_t addr_len, uint8_t *data, uint16_t data_len);
extern void aw_sar_delay_ms(uint32_t ms);

extern int aw_sar_parsing_bin_file(struct aw_bin *bin);
extern uint32_t aw_sar_pow2(uint32_t cnt);
extern int32_t aw_sar_load_reg(struct aw_bin *aw_bin, struct i2c_client *i2c);

#endif
