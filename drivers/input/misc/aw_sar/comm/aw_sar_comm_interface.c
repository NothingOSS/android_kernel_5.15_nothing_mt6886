#include "aw_sar_comm_interface.h"

#define AW_I2C_RW_RETRY_TIME_MIN		(2000)
#define AW_I2C_RW_RETRY_TIME_MAX		(3000)
#define AW_RETRIES				(5)

static int32_t awinic_i2c_write(struct i2c_client *i2c, uint8_t *tr_data, uint16_t len)
{
	struct i2c_msg msg;

	msg.addr = i2c->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = tr_data;

	return i2c_transfer(i2c->adapter, &msg, 1);
}

static int32_t awinic_i2c_read(struct i2c_client *i2c, uint8_t *addr,
				uint8_t addr_len, uint8_t *data, uint16_t data_len)
{
	struct i2c_msg msg[2];

	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = addr_len;
	msg[0].buf = addr;

	msg[1].addr = i2c->addr;
	msg[1].flags = 1;
	msg[1].len = data_len;
	msg[1].buf = data;

	return i2c_transfer(i2c->adapter, msg, 2);
}

/**
 * @brief Read register interface
 *
 * @param i2c: i2c client.
 * @param reg_addr16: 16 bit register address.
 * @param *reg_data32: 32 bit register data.
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
int32_t aw_sar_i2c_read(struct i2c_client *i2c, uint16_t reg_addr16,  uint32_t *reg_data32)
{
	int8_t cnt = 5;
	int32_t ret = -1;
	uint8_t r_buf[6] = { 0 };

	if (i2c == NULL) {
		AWLOGE(&i2c->dev, "i2c_dev is null error!");
		return -AW_ERR;
	}

	r_buf[0] = (unsigned char)(reg_addr16 >> OFFSET_BIT_8);
	r_buf[1] = (unsigned char)(reg_addr16);

	do {
		ret = awinic_i2c_read(i2c, r_buf, 2, &r_buf[2], 4);
		if (ret < 0) {
			AWLOGE(&i2c->dev, "i2c read error reg: 0x%04x, ret= %d cnt= %d", reg_addr16, ret, cnt);
		} else {
			break;
		}
		usleep_range(2000, 3000);
	} while (cnt--);

	if (cnt < 0) {
		AWLOGE(&i2c->dev, "i2c read error!");
		return -AW_ERR;
	}

	*reg_data32 = ((uint32_t)r_buf[5] << OFFSET_BIT_0) | ((uint32_t)r_buf[4] << OFFSET_BIT_8) |
		      ((uint32_t)r_buf[3] << OFFSET_BIT_16) | ((uint32_t)r_buf[2] << OFFSET_BIT_24);

	return AW_OK;
}

/**
 * @brief write register interface
 *
 * @param i2c: i2c client.
 * @param reg_addr16: 16 bit register address.
 * @param reg_data32: 32 bit register data.
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
int32_t aw_sar_i2c_write(struct i2c_client *i2c, uint16_t reg_addr16, uint32_t reg_data32)
{
	int8_t cnt = 5;
	int32_t ret = -1;
	uint8_t w_buf[6] = { 0 };

	if (i2c == NULL) {
		AWLOGE(&i2c->dev, "i2c_dev is null error!");
		return -AW_ERR;
	}

	/*reg_addr*/
	w_buf[0] = (uint8_t)(reg_addr16 >> OFFSET_BIT_8);
	w_buf[1] = (uint8_t)(reg_addr16);
	/*data*/
	w_buf[2] = (uint8_t)(reg_data32 >> OFFSET_BIT_24);
	w_buf[3] = (uint8_t)(reg_data32 >> OFFSET_BIT_16);
	w_buf[4] = (uint8_t)(reg_data32 >> OFFSET_BIT_8);
	w_buf[5] = (uint8_t)(reg_data32);

	do {
		ret = awinic_i2c_write(i2c, w_buf, ARRAY_SIZE(w_buf));
		if (ret < 0) {
			AWLOGE(&i2c->dev, "i2c write error reg: 0x%04x data: 0x%08x, ret= %d cnt= %d",
							reg_addr16, reg_data32, ret, cnt);
		} else {
			break;
		}
	} while (cnt--);

	if (cnt < 0) {
		AWLOGE(&i2c->dev, "i2c write error!");
		return -AW_ERR;
	}

	return AW_OK;
}

/**
 * @brief Write the corresponding bit of the register
 *
 * @param i2c:i2c client.
 * @param reg_addr16: 16 bit register address.
 * @param mask: Write the corresponding bit as 0
 * @param val: Write corresponding data to the register
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
int32_t aw_sar_i2c_write_bits(struct i2c_client *i2c, uint16_t reg_addr16,
						uint32_t mask, uint32_t val)
{
	uint32_t reg_val;

	aw_sar_i2c_read(i2c, reg_addr16, &reg_val);
	reg_val &= mask;
	reg_val |= (val & (~mask));
	aw_sar_i2c_write(i2c, reg_addr16, reg_val);

	return AW_OK;
}

/**
 * @brief Continuously write data to the chip
 *
 * @param i2c:i2c client.
 * @param *tr_data: Data written
 * @param len: Length of data written
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
int32_t aw_sar_i2c_write_seq(struct i2c_client *i2c, uint8_t *tr_data, uint16_t len)
{
	int8_t cnt = AW_RETRIES;
	int32_t ret =  -AW_ERR;

	do {
		ret = awinic_i2c_write(i2c, tr_data, len);
		if (ret < 0)
			AWLOGE(&i2c->dev, "awinic i2c write seq error %d", ret);
		else
			break;
		usleep_range(AW_I2C_RW_RETRY_TIME_MIN, AW_I2C_RW_RETRY_TIME_MAX);
	} while (cnt--);

	if (cnt < 0) {
		AWLOGE(&i2c->dev, "awinic i2c write error!");
		return -AW_ERR;
	}

	return AW_OK;
}

/**
 * @brief Continuously Read data from chip
 *
 * @param i2c:i2c client.
 * @param *addr: Read address
 * @param addr_len: Length of read address (byte)
 * @param *data: Data written
 * @param data_len: Length of data written
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
int32_t aw_sar_i2c_read_seq(struct i2c_client *i2c, uint8_t *addr,
				uint8_t addr_len, uint8_t *data, uint16_t data_len)
{
	int8_t cnt = AW_RETRIES;
	int32_t ret = -AW_ERR;

	do {
		ret = awinic_i2c_read(i2c, addr, addr_len, data, data_len);
		if (ret < 0) {
			AWLOGE(&i2c->dev, "awinic sar i2c write error %d", ret);
		} else {
			break;
		}
		usleep_range(AW_I2C_RW_RETRY_TIME_MIN, AW_I2C_RW_RETRY_TIME_MAX);
	} while (cnt--);

	if (cnt < 0) {
		AWLOGE(&i2c->dev, "awinic sar i2c read error!");
		return -AW_ERR;
	}

	return AW_OK;
}

/*********************************Parse bin file code start**************************************************/

#define AWINIC_CODE_VERSION "V0.0.7-V1.0.4"	/* "code version"-"excel version" */

enum bin_header_version_enum {
	HEADER_VERSION_1_0_0 = 0x01000000,
};

enum data_type_enum {
	DATA_TYPE_REGISTER = 0x00000000,
	DATA_TYPE_DSP_REG = 0x00000010,
	DATA_TYPE_DSP_CFG = 0x00000011,
	DATA_TYPE_SOC_REG = 0x00000020,
	DATA_TYPE_SOC_APP = 0x00000021,
	DATA_TYPE_MULTI_BINS = 0x00002000,
};

enum data_version_enum {
	DATA_VERSION_V1 = 0X00000001,	/*default little edian */
	DATA_VERSION_MAX,
};

#define DEBUG_LOG_LEVEL
#ifdef DEBUG_LOG_LEVEL
#define DBG(fmt, arg...)   do {\
printk("AWINIC_BIN %s,line= %d,"fmt, __func__, __LINE__, ##arg);\
} while (0)
#define DBG_ERR(fmt, arg...)   do {\
printk("AWINIC_BIN_ERR %s,line= %d,"fmt, __func__, __LINE__, ##arg);\
} while (0)
#else
#define DBG(fmt, arg...) do {} while (0)
#define DBG_ERR(fmt, arg...) do {} while (0)
#endif

#define printing_data_code

typedef unsigned short int aw_uint16;
typedef unsigned long int aw_uint32;

#define BigLittleSwap16(A)	((((aw_uint16)(A) & 0xff00) >> 8) | \
				 (((aw_uint16)(A) & 0x00ff) << 8))

#define BigLittleSwap32(A)	((((aw_uint32)(A) & 0xff000000) >> 24) | \
				(((aw_uint32)(A) & 0x00ff0000) >> 8) | \
				(((aw_uint32)(A) & 0x0000ff00) << 8) | \
				(((aw_uint32)(A) & 0x000000ff) << 24))

static int aw_parse_bin_header_1_0_0(struct aw_bin *bin);

/**
*
* Interface function
*
* return value:
*       value = 0 :success;
*       value = -1 :check bin header version
*       value = -2 :check bin data type
*       value = -3 :check sum or check bin data len error
*       value = -4 :check data version
*       value = -5 :check register num
*       value = -6 :check dsp reg num
*       value = -7 :check soc app num
*       value = -8 :bin is NULL point
*
**/

/********************************************************
*
* check sum data
*
********************************************************/
static int aw_check_sum(struct aw_bin *bin, int bin_num)
{
	unsigned int i = 0;
	unsigned int sum_data = 0;
	unsigned int check_sum = 0;
	char *p_check_sum = NULL;

	DBG("enter\n");

	p_check_sum =
	    &(bin->info.data[(bin->header_info[bin_num].valid_data_addr -
			      bin->header_info[bin_num].header_len)]);
	DBG("aw_bin_parse p_check_sum = %p\n", p_check_sum);
	check_sum = AW_SAR_GET_32_DATA(*(p_check_sum + 3),
				*(p_check_sum + 2),
				*(p_check_sum + 1), *(p_check_sum));

	for (i = 4;
	     i <
	     bin->header_info[bin_num].bin_data_len +
	     bin->header_info[bin_num].header_len; i++) {
		sum_data += *(p_check_sum + i);
	}
	DBG("aw_bin_parse bin_num=%d, check_sum = 0x%x, sum_data = 0x%x\n",
		bin_num, check_sum, sum_data);
	if (sum_data != check_sum) {
		p_check_sum = NULL;
		DBG_ERR("aw_bin_parse check sum or check bin data len error\n");
		DBG_ERR("aw_bin_parse bin_num=%d, check_sum = 0x%x, sum_data = 0x%x\n", bin_num, check_sum, sum_data);
		return -3;
	}
	p_check_sum = NULL;

	return 0;
}

static int aw_check_data_version(struct aw_bin *bin, int bin_num)
{
	int i = 0;
	DBG("enter\n");

	for (i = DATA_VERSION_V1; i < DATA_VERSION_MAX; i++) {
		if (bin->header_info[bin_num].bin_data_ver == i) {
			return 0;
		}
	}
	DBG_ERR("aw_bin_parse Unrecognized this bin data version\n");
	return -4;
}

static int aw_check_register_num_v1(struct aw_bin *bin, int bin_num)
{
	unsigned int check_register_num = 0;
	unsigned int parse_register_num = 0;
	char *p_check_sum = NULL;

	DBG("enter\n");

	p_check_sum =
	    &(bin->info.data[(bin->header_info[bin_num].valid_data_addr)]);
	DBG("aw_bin_parse p_check_sum = %p\n", p_check_sum);
	parse_register_num = AW_SAR_GET_32_DATA(*(p_check_sum + 3),
					 *(p_check_sum + 2),
					 *(p_check_sum + 1), *(p_check_sum));
	check_register_num = (bin->header_info[bin_num].bin_data_len - 4) /
	    (bin->header_info[bin_num].reg_byte_len +
	     bin->header_info[bin_num].data_byte_len);
	DBG
	    ("aw_bin_parse bin_num=%d, parse_register_num = 0x%x, check_register_num = 0x%x\n",
	     bin_num, parse_register_num, check_register_num);
	if (parse_register_num != check_register_num) {
		p_check_sum = NULL;
		DBG_ERR("aw_bin_parse register num is error\n");
		DBG_ERR("aw_bin_parse bin_num=%d, parse_register_num = 0x%x, check_register_num = 0x%x\n", bin_num, parse_register_num, check_register_num);
		return -5;
	}
	bin->header_info[bin_num].reg_num = parse_register_num;
	bin->header_info[bin_num].valid_data_len =
	    bin->header_info[bin_num].bin_data_len - 4;
	p_check_sum = NULL;
	bin->header_info[bin_num].valid_data_addr =
	    bin->header_info[bin_num].valid_data_addr + 4;
	return 0;
}

static int aw_check_dsp_reg_num_v1(struct aw_bin *bin, int bin_num)
{
	unsigned int check_dsp_reg_num = 0;
	unsigned int parse_dsp_reg_num = 0;
	char *p_check_sum = NULL;

	DBG("enter\n");

	p_check_sum =
	    &(bin->info.data[(bin->header_info[bin_num].valid_data_addr)]);
	DBG("aw_bin_parse p_check_sum = %p\n", p_check_sum);
	parse_dsp_reg_num = AW_SAR_GET_32_DATA(*(p_check_sum + 7),
					*(p_check_sum + 6),
					*(p_check_sum + 5),
					*(p_check_sum + 4));
	bin->header_info[bin_num].reg_data_byte_len =
	    AW_SAR_GET_32_DATA(*(p_check_sum + 11), *(p_check_sum + 10),
			*(p_check_sum + 9), *(p_check_sum + 8));
	check_dsp_reg_num =
	    (bin->header_info[bin_num].bin_data_len -
	     12) / bin->header_info[bin_num].reg_data_byte_len;
	DBG
	    ("aw_bin_parse bin_num=%d, parse_dsp_reg_num = 0x%x, check_dsp_reg_num = 0x%x\n",
	     bin_num, parse_dsp_reg_num, check_dsp_reg_num);
	if (parse_dsp_reg_num != check_dsp_reg_num) {
		p_check_sum = NULL;
		DBG_ERR("aw_bin_parse dsp reg num is error\n");
		DBG_ERR("aw_bin_parse bin_num=%d, parse_dsp_reg_num = 0x%x, check_dsp_reg_num = 0x%x\n", bin_num, parse_dsp_reg_num, check_dsp_reg_num);
		return -6;
	}
	bin->header_info[bin_num].download_addr =
	    AW_SAR_GET_32_DATA(*(p_check_sum + 3), *(p_check_sum + 2),
			*(p_check_sum + 1), *(p_check_sum));
	bin->header_info[bin_num].reg_num = parse_dsp_reg_num;
	bin->header_info[bin_num].valid_data_len =
	    bin->header_info[bin_num].bin_data_len - 12;
	p_check_sum = NULL;
	bin->header_info[bin_num].valid_data_addr =
	    bin->header_info[bin_num].valid_data_addr + 12;
	return 0;
}

static int aw_check_soc_app_num_v1(struct aw_bin *bin, int bin_num)
{
	unsigned int check_soc_app_num = 0;
	unsigned int parse_soc_app_num = 0;
	char *p_check_sum = NULL;

	DBG("enter\n");

	p_check_sum =
	    &(bin->info.data[(bin->header_info[bin_num].valid_data_addr)]);
	DBG("aw_bin_parse p_check_sum = %p\n", p_check_sum);
	bin->header_info[bin_num].app_version = AW_SAR_GET_32_DATA(*(p_check_sum + 3),
							    *(p_check_sum + 2),
							    *(p_check_sum + 1),
							    *(p_check_sum));
	parse_soc_app_num = AW_SAR_GET_32_DATA(*(p_check_sum + 11),
					*(p_check_sum + 10),
					*(p_check_sum + 9), *(p_check_sum + 8));
	check_soc_app_num = bin->header_info[bin_num].bin_data_len - 12;
	DBG
	    ("aw_bin_parse bin_num=%d, parse_soc_app_num = 0x%x, check_soc_app_num = 0x%x\n",
	     bin_num, parse_soc_app_num, check_soc_app_num);
	if (parse_soc_app_num != check_soc_app_num) {
		p_check_sum = NULL;
		DBG_ERR("aw_bin_parse soc app num is error\n");
		DBG_ERR("aw_bin_parse bin_num=%d, parse_soc_app_num = 0x%x, check_soc_app_num = 0x%x\n", bin_num, parse_soc_app_num, check_soc_app_num);
		return -7;
	}
	bin->header_info[bin_num].reg_num = parse_soc_app_num;
	bin->header_info[bin_num].download_addr =
	    AW_SAR_GET_32_DATA(*(p_check_sum + 7), *(p_check_sum + 6),
			*(p_check_sum + 5), *(p_check_sum + 4));
	bin->header_info[bin_num].valid_data_len =
	    bin->header_info[bin_num].bin_data_len - 12;
	p_check_sum = NULL;
	bin->header_info[bin_num].valid_data_addr =
	    bin->header_info[bin_num].valid_data_addr + 12;
	return 0;
}

/************************
***
***bin header 1_0_0
***
************************/
static void aw_get_single_bin_header_1_0_0(struct aw_bin *bin)
{
	int i;
	DBG("enter %s\n", __func__);
	bin->header_info[bin->all_bin_parse_num].header_len = 60;
	bin->header_info[bin->all_bin_parse_num].check_sum =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 3), *(bin->p_addr + 2),
			*(bin->p_addr + 1), *(bin->p_addr));
	bin->header_info[bin->all_bin_parse_num].header_ver =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 7), *(bin->p_addr + 6),
			*(bin->p_addr + 5), *(bin->p_addr + 4));
	bin->header_info[bin->all_bin_parse_num].bin_data_type =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 11), *(bin->p_addr + 10),
			*(bin->p_addr + 9), *(bin->p_addr + 8));
	bin->header_info[bin->all_bin_parse_num].bin_data_ver =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 15), *(bin->p_addr + 14),
			*(bin->p_addr + 13), *(bin->p_addr + 12));
	bin->header_info[bin->all_bin_parse_num].bin_data_len =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 19), *(bin->p_addr + 18),
			*(bin->p_addr + 17), *(bin->p_addr + 16));
	bin->header_info[bin->all_bin_parse_num].ui_ver =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 23), *(bin->p_addr + 22),
			*(bin->p_addr + 21), *(bin->p_addr + 20));
	bin->header_info[bin->all_bin_parse_num].reg_byte_len =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 35), *(bin->p_addr + 34),
			*(bin->p_addr + 33), *(bin->p_addr + 32));
	bin->header_info[bin->all_bin_parse_num].data_byte_len =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 39), *(bin->p_addr + 38),
			*(bin->p_addr + 37), *(bin->p_addr + 36));
	bin->header_info[bin->all_bin_parse_num].device_addr =
	    AW_SAR_GET_32_DATA(*(bin->p_addr + 43), *(bin->p_addr + 42),
			*(bin->p_addr + 41), *(bin->p_addr + 40));
	for (i = 0; i < 8; i++) {
		bin->header_info[bin->all_bin_parse_num].chip_type[i] =
		    *(bin->p_addr + 24 + i);
	}
//	bin->header_info[bin->all_bin_parse_num].chip_type[i] = '\0';
//	DBG("%s : enter chip_type is %s\n", __func__,
//							bin->header_info[bin->all_bin_parse_num].chip_type);

	bin->header_info[bin->all_bin_parse_num].reg_num = 0x00000000;
	bin->header_info[bin->all_bin_parse_num].reg_data_byte_len = 0x00000000;
	bin->header_info[bin->all_bin_parse_num].download_addr = 0x00000000;
	bin->header_info[bin->all_bin_parse_num].app_version = 0x00000000;
	bin->header_info[bin->all_bin_parse_num].valid_data_len = 0x00000000;
	bin->all_bin_parse_num += 1;
}

static int aw_parse_each_of_multi_bins_1_0_0(unsigned int bin_num, int bin_serial_num,
				      struct aw_bin *bin)
{
	int ret = 0;
	unsigned int bin_start_addr = 0;
	unsigned int valid_data_len = 0;
	DBG("aw_bin_parse enter multi bin branch -- %s\n", __func__);
	if (!bin_serial_num) {
		bin_start_addr = AW_SAR_GET_32_DATA(*(bin->p_addr + 67),
					     *(bin->p_addr + 66),
					     *(bin->p_addr + 65),
					     *(bin->p_addr + 64));
		bin->p_addr += (60 + bin_start_addr);
		bin->header_info[bin->all_bin_parse_num].valid_data_addr =
		    bin->header_info[bin->all_bin_parse_num -
				     1].valid_data_addr + 4 + 8 * bin_num + 60;
	} else {
		valid_data_len =
		    bin->header_info[bin->all_bin_parse_num - 1].bin_data_len;
		bin->p_addr += (60 + valid_data_len);
		bin->header_info[bin->all_bin_parse_num].valid_data_addr =
		    bin->header_info[bin->all_bin_parse_num -
				     1].valid_data_addr +
		    bin->header_info[bin->all_bin_parse_num - 1].bin_data_len +
		    60;
	}

	ret = aw_parse_bin_header_1_0_0(bin);
	return ret;
}

/* Get the number of bins in multi bins, and set a for loop, loop processing each bin data */
static int aw_get_multi_bin_header_1_0_0(struct aw_bin *bin)
{
	int i = 0;
	int ret = 0;
	unsigned int bin_num = 0;
	DBG("aw_bin_parse enter multi bin branch -- %s\n", __func__);
	bin_num = AW_SAR_GET_32_DATA(*(bin->p_addr + 63),
			      *(bin->p_addr + 62),
			      *(bin->p_addr + 61), *(bin->p_addr + 60));
	if (bin->multi_bin_parse_num == 1) {
		bin->header_info[bin->all_bin_parse_num].valid_data_addr = 60;
	}
	aw_get_single_bin_header_1_0_0(bin);

	for (i = 0; i < bin_num; i++) {
		DBG("aw_bin_parse enter multi bin for is %d\n", i);
		ret = aw_parse_each_of_multi_bins_1_0_0(bin_num, i, bin);
		if (ret < 0) {
			return ret;
		}
	}
	return 0;
}

/********************************************************
*
* If the bin framework header version is 1.0.0,
  determine the data type of bin, and then perform different processing
  according to the data type
  If it is a single bin data type, write the data directly into the structure array
  If it is a multi-bin data type, first obtain the number of bins,
  and then recursively call the bin frame header processing function
  according to the bin number to process the frame header information of each bin separately
*
********************************************************/
static int aw_parse_bin_header_1_0_0(struct aw_bin *bin)
{
	int ret = 0;
	unsigned int bin_data_type;
	DBG("enter %s\n", __func__);
	bin_data_type = AW_SAR_GET_32_DATA(*(bin->p_addr + 11),
				    *(bin->p_addr + 10),
				    *(bin->p_addr + 9), *(bin->p_addr + 8));
	DBG("aw_bin_parse bin_data_type 0x%x\n", bin_data_type);
	switch (bin_data_type) {
	case DATA_TYPE_REGISTER:
	case DATA_TYPE_DSP_REG:
	case DATA_TYPE_SOC_APP:
		/* Divided into two processing methods,
		   one is single bin processing,
		   and the other is single bin processing in multi bin */
		DBG("aw_bin_parse enter single bin branch\n");
		bin->single_bin_parse_num += 1;
		DBG("%s bin->single_bin_parse_num is %d\n", __func__,
			bin->single_bin_parse_num);
		if (!bin->multi_bin_parse_num) {
			bin->header_info[bin->
					 all_bin_parse_num].valid_data_addr =
			    60;
		}
		aw_get_single_bin_header_1_0_0(bin);
		break;
	case DATA_TYPE_MULTI_BINS:
		/* Get the number of times to enter multi bins */
		DBG("aw_bin_parse enter multi bin branch\n");
		bin->multi_bin_parse_num += 1;
		DBG("%s bin->multi_bin_parse_num is %d\n", __func__,
			bin->multi_bin_parse_num);
		ret = aw_get_multi_bin_header_1_0_0(bin);
		if (ret < 0) {
			return ret;
		}
		break;
	default:
		DBG_ERR("aw_bin_parse Unrecognized this bin data type\n");
		return -2;
	}
	return 0;
}

/* get the bin's header version */
static int aw_check_bin_header_version(struct aw_bin *bin)
{
	int ret = 0;
	unsigned int header_version = 0;

	header_version = AW_SAR_GET_32_DATA(*(bin->p_addr + 7),
				     *(bin->p_addr + 6),
				     *(bin->p_addr + 5), *(bin->p_addr + 4));

	DBG("aw_bin_parse header_version 0x%x\n", header_version);

	/* Write data to the corresponding structure array
	   according to different formats of the bin frame header version */
	switch (header_version) {
	case HEADER_VERSION_1_0_0:
		ret = aw_parse_bin_header_1_0_0(bin);
		return ret;
	default:
		DBG_ERR("aw_bin_parse Unrecognized this bin header version \n");
		return -1;
	}
}

/**
 * @brief Parse bin file
 *
 * @param bin: Store the contents of the parsed bin file
 * @return 0 if init successed, other if error
 */
int aw_sar_parsing_bin_file(struct aw_bin *bin)
{
	int i = 0;
	int ret = 0;

	DBG("aw_bin_parse code version:%s\n", AWINIC_CODE_VERSION);
	if (!bin) {
		DBG_ERR("aw_bin_parse bin is NULL\n");
		return -8;
	}
	bin->p_addr = bin->info.data;
	bin->all_bin_parse_num = 0;
	bin->multi_bin_parse_num = 0;
	bin->single_bin_parse_num = 0;

	/* filling bins header info */
	ret = aw_check_bin_header_version(bin);
	if (ret < 0) {
		DBG_ERR("aw_bin_parse check bin header version error\n");
		return ret;
	}
	bin->p_addr = NULL;

	/* check bin header info */
	for (i = 0; i < bin->all_bin_parse_num; i++) {
		/* check sum */
		ret = aw_check_sum(bin, i);
		if (ret < 0) {
			DBG_ERR("aw_bin_parse check sum data error\n");
			return ret;
		}
		/* check bin data version */
		ret = aw_check_data_version(bin, i);
		if (ret < 0) {
			DBG_ERR("aw_bin_parse check data version error\n");
			return ret;
		}
		/* check valid data */
		if (bin->header_info[i].bin_data_ver == DATA_VERSION_V1) {
			/* check register num */
			if (bin->header_info[i].bin_data_type ==
			    DATA_TYPE_REGISTER) {
				ret = aw_check_register_num_v1(bin, i);
				if (ret < 0) {
					DBG_ERR
					    ("aw_bin_parse check register num error\n");
					return ret;
				}
				/* check dsp reg num */
			} else if (bin->header_info[i].bin_data_type ==
				   DATA_TYPE_DSP_REG) {
				ret = aw_check_dsp_reg_num_v1(bin, i);
				if (ret < 0) {
					DBG_ERR
					    ("aw_bin_parse check dsp reg num error\n");
					return ret;
				}
				/* check soc app num */
			} else if (bin->header_info[i].bin_data_type ==
				   DATA_TYPE_SOC_APP) {
				ret = aw_check_soc_app_num_v1(bin, i);
				if (ret < 0) {
					DBG_ERR
					    ("aw_bin_parse check soc app num error\n");
					return ret;
				}
			} else {
				bin->header_info[i].valid_data_len =
				    bin->header_info[i].bin_data_len;
			}
		}
	}
	DBG("aw_bin_parse parsing success\n");

	return 0;
}
/*********************************Parse bin file code end**************************************************/

/**
 * @brief Calculate the second power
 *
 * @param cnt: ifrequency
 * @return the second power
 */
uint32_t aw_sar_pow2(uint32_t cnt)
{
	uint32_t i = 0;
	uint32_t sum = 1;

	if (cnt == 0) {
		sum = 1;
	} else {
		for (i = 0; i < cnt; i++) {
			sum *= 2;
		}
	}

	return sum;
}

/**
 * @brief Calculate the second power
 *
 * @param *aw_bin: Information after parsing bin file
 * @param *i2c: i2c client.
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error
 */
int32_t aw_sar_load_reg(struct aw_bin *aw_bin, struct i2c_client *i2c)
{
	uint32_t i = 0;
	int32_t ret = 0;
	uint16_t reg_addr = 0;
	uint32_t reg_data = 0;
	uint32_t start_addr = aw_bin->header_info[0].valid_data_addr;

	for (i = 0; i < aw_bin->header_info[0].valid_data_len;
						i += 6, start_addr += 6) {
		reg_addr = (aw_bin->info.data[start_addr]) |
				aw_bin->info.data[start_addr + 1] << OFFSET_BIT_8;
		reg_data = aw_bin->info.data[start_addr + 2] |
			(aw_bin->info.data[start_addr + 3] << OFFSET_BIT_8) |
			(aw_bin->info.data[start_addr + 4] << OFFSET_BIT_16) |
			(aw_bin->info.data[start_addr + 5] << OFFSET_BIT_24);

		ret = aw_sar_i2c_write(i2c, reg_addr, reg_data);
		if (ret < 0) {
			AWLOGE(&i2c->dev, "i2c write err");
			return -AW_ERR;
		}

		AWLOGD(&i2c->dev, "reg_addr = 0x%04x, reg_data = 0x%08x", reg_addr, reg_data);
	}

	return AW_OK;
}

void aw_sar_delay_ms(uint32_t ms)
{
	mdelay(ms);
}

