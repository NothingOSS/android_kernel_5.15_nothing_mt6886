#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "gh_core.h"
#include "gh621x_common.h"

#define GH621X_REG_ADDR_LEN		2
#define GOODIX_BUS_RETRY_TIMES		3
#define I2C_MAX_TRANSFER_SIZE		256

static int gh621x_read(struct gh_core_data *cd, u16 reg,
		u8 *data, unsigned int len)
{
	struct i2c_client *client = cd->client;
	unsigned int transfer_length = 0;
	unsigned int pos = 0;
	u16 address = reg;
	unsigned char get_buf[128], addr_buf[GH621X_REG_ADDR_LEN];
	int retry, r = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = !I2C_M_RD,
			.buf = &addr_buf[0],
			.len = GH621X_REG_ADDR_LEN,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
		}
	};

	if (likely(len < sizeof(get_buf))) {
		/* code optimize, use stack memory */
		msgs[1].buf = &get_buf[0];
	} else {
		msgs[1].buf = kzalloc(len, GFP_KERNEL);
		if (msgs[1].buf == NULL)
			return -ENOMEM;
	}

	while (pos != len) {
		if (unlikely(len - pos > I2C_MAX_TRANSFER_SIZE))
			transfer_length = I2C_MAX_TRANSFER_SIZE;
		else
			transfer_length = len - pos;

		msgs[0].buf[0] = (address >> 8) & 0xFF;
		msgs[0].buf[1] = address & 0xFF;
		msgs[1].len = transfer_length;

		for (retry = 0; retry < GOODIX_BUS_RETRY_TIMES; retry++) {
			if (likely(i2c_transfer(client->adapter,
						msgs, 2) == 2)) {
				memcpy(&data[pos], msgs[1].buf,
				       transfer_length);
				pos += transfer_length;
				address += transfer_length;
				break;
			}
			gh_info("I2c read retry[%d]:0x%x", retry + 1, reg);
			usleep_range(2000, 2100);
		}
		if (unlikely(retry == GOODIX_BUS_RETRY_TIMES)) {
			gh_err("I2c read failed,dev:%02x,reg:%04x,size:%u",
			       client->addr, reg, len);
			r = -EAGAIN;
			goto read_exit;
		}
	}

read_exit:
	if (unlikely(len >= sizeof(get_buf)))
		kfree(msgs[1].buf);
	return r;
}

static int gh621x_write(struct gh_core_data *cd, u16 reg,
		u8 *data, unsigned int len)
{
	struct i2c_client *client = cd->client;
	unsigned int pos = 0, transfer_length = 0;
	u16 address = reg;
	unsigned char put_buf[128];
	int retry, r = 0;
	struct i2c_msg msg = {
			.addr = client->addr,
			.flags = !I2C_M_RD,
	};

	if (likely(len + GH621X_REG_ADDR_LEN < sizeof(put_buf))) {
		/* code optimize,use stack memory*/
		msg.buf = &put_buf[0];
	} else {
		msg.buf = kmalloc(len + GH621X_REG_ADDR_LEN, GFP_KERNEL);
		if (msg.buf == NULL)
			return -ENOMEM;
	}

	while (pos != len) {
		if (unlikely(len - pos > I2C_MAX_TRANSFER_SIZE -
			     GH621X_REG_ADDR_LEN))
			transfer_length = I2C_MAX_TRANSFER_SIZE -
			     GH621X_REG_ADDR_LEN;
		else
			transfer_length = len - pos;
		msg.buf[0] = (address >> 8) & 0xFF;
		msg.buf[1] = address & 0xFF;

		msg.len = transfer_length + GH621X_REG_ADDR_LEN;
		memcpy(&msg.buf[GH621X_REG_ADDR_LEN],
			&data[pos], transfer_length);

		for (retry = 0; retry < GOODIX_BUS_RETRY_TIMES; retry++) {
			if (likely(i2c_transfer(client->adapter,
						&msg, 1) == 1)) {
				pos += transfer_length;
				address += transfer_length;
				break;
			}
			gh_debug("I2c write retry[%d]", retry + 1);
			msleep(20);
		}
		if (unlikely(retry == GOODIX_BUS_RETRY_TIMES)) {
			gh_err("I2c write failed,dev:%02x,reg:%04x,size:%u",
				client->addr, reg, len);
			r = -EAGAIN;
			goto write_exit;
		}
	}

write_exit:
	if (likely(len + GH621X_REG_ADDR_LEN >= sizeof(put_buf)))
		kfree(msg.buf);
	return r;
}

static int gh621x_reg_read(struct gh_core_data *cd, u16 reg_addr, u16 *reg_val)
{
	int ret;

	ret = gh621x_read(cd, reg_addr, (u8 *)reg_val, 2);
	if (ret) {
		gh_err("%s failed read addr 0x%x, %d", __func__, reg_addr, ret);
		return ret;
	}
	*reg_val = be16_to_cpu(*reg_val);
	return 0;
}

int gh621x_reg_read_u32(struct gh_core_data *cd, u16 reg_addr, u32 *reg_val)
{
	u16 val_low, val_high;
	int ret;

	ret = gh621x_reg_read(cd, reg_addr, &val_low);
	if (ret) {
		gh_err("failed read reg %d, ret %d", reg_addr, ret);
		return ret;
	}

	ret = gh621x_reg_read(cd, reg_addr + 2, &val_high);
	if (ret) {
		gh_err("failed read reg %d, ret %d", reg_addr + 2, ret);
		return ret;
	}

	*reg_val = (val_high << 16) | val_low;
	return 0;
}

int gh621x_regs_read(struct gh_core_data *cd, u16 reg_addr, u16 *reg_value, int len)
{
	int ret = 0;

	ret = gh621x_read(cd, reg_addr, (u8 *)reg_value, len * sizeof(*reg_value));
	if (ret) {
		gh_err("failed read regs 0x%x, len %d, ret %d", reg_addr, len, ret);
		return ret;
	}

	gh_be16_to_cpu_array(reg_value, len);
	return 0;
}

int gh621x_reg_write(struct gh_core_data *cd, u16 reg_addr, u16 reg_val)
{
	reg_val = cpu_to_be16(reg_val);
	return gh621x_write(cd, reg_addr, (u8 *)&reg_val, sizeof(reg_val));
}

static int gh621x_dev_confirm(struct gh_core_data *cd)
{
	int ret;
	u16 id;

	ret = gh621x_reg_read(cd, GH621X_CPU_ID_LO_REG_ADDR, &id);
	gh_info("read dev id return %d, val 0x%x", ret, id);
	return ret;
}

int gh621x_sw_reset(struct gh_core_data *cd)
{
	int ret;

	ret = gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_RESET_HOST);
	if (!ret)
		msleep(50);
	else
		gh_err("failed write sw reset command");

	gh_info("do soft reset %d", ret);
	return ret;
}

int gh621x_load_config(struct gh_core_data *cd, gh621x_reg_t *cfg_array, int size)
{
	int ret, i;

	for (i = 0; i < size; i++) {
		ret = gh621x_reg_write(cd, cfg_array[i].addr, cfg_array[i].value);
		if (ret) {
			gh_err("failed write config 0x%x, value 0x%x, ret %d",
				 cfg_array[i].addr, cfg_array[i].value, ret);
			return ret;
		}
	}
	usleep_range(5000, 5100);
	return 0;
}

int gh621x_load_patch(struct gh_core_data *cd, u8 *patch_buffer, int size)
{
	int ret = 0;

	u16 patch_start_addr = GH621X_MAKEUP_WORD(patch_buffer[GH621X_PATCH_CODE_START_ADDR_H_INDEX],
							patch_buffer[GH621X_PATCH_CODE_START_ADDR_L_INDEX]);
	u16 patch_code_bytes_len = GH621X_MAKEUP_WORD(patch_buffer[GH621X_PATCH_CODE_END_ADDR_H_INDEX],
								patch_buffer[GH621X_PATCH_CODE_END_ADDR_L_INDEX]) -
								patch_start_addr;
	u16 reg_data_tmp = 0;
	u16 valid_block_size = GH621X_MAX_BLOCK_SIZE_SUPPORT - GH621X_REG_ADDR_SIZE; // 2 bytes for reg addr
	u16 patch_code_bytes_len_tmp = patch_code_bytes_len;
	u16 patch_block_dest_addr = (u16)patch_start_addr;
	u8 *patch_block_src_addr = &patch_buffer[GH621X_PATCH_CODE_REAL_DATA_INDEX];
	u8 patch_block[GH621X_MAX_BLOCK_SIZE_SUPPORT] = {0};
	u16 valid_write_size;

	ret += gh621x_reg_write(cd, GH621X_IRAM_CRC_SADDR_REG_ADDR, patch_start_addr);
	ret += gh621x_reg_write(cd, GH621X_CRC_PATTERN_REG_ADDR, (GH621X_MAKEUP_WORD(patch_buffer[GH621X_PATCH_CODE_CRC_H_INDEX],
										patch_buffer[GH621X_PATCH_CODE_CRC_L_INDEX])));
	ret += gh621x_reg_write(cd, GH621X_IRAM_CRC_EADDR_REG_ADDR, (patch_start_addr + patch_code_bytes_len - GH621X_REG_VAL_SIZE));
	ret += gh621x_reg_write(cd, GH621X_CRC16_CTRL_REG_ADDR, GH621X_CRC16_CTRL_ENABLE_VAL);
	if (ret) {
		gh_err("fail to init patch %d", ret);
		return ret;
	}

	while (patch_code_bytes_len_tmp) {
		valid_write_size = ((valid_block_size > patch_code_bytes_len_tmp) ? \
				(patch_code_bytes_len_tmp) : (valid_block_size));
		memcpy(patch_block, patch_block_src_addr, valid_write_size);
		ret = gh621x_write(cd, patch_block_dest_addr, patch_block, valid_write_size);
		if (ret) {
			gh_err("failed load patch addr 0x%x", patch_block_dest_addr);
			return ret;
		}
		patch_code_bytes_len_tmp -= valid_write_size;
		patch_block_dest_addr += valid_write_size;
		patch_block_src_addr += valid_write_size;
	}
	usleep_range(2000, 2100);; //Delay 2ms

	ret = gh621x_reg_read(cd, GH621X_CRC16_STATUS_REG_ADDR, &reg_data_tmp);
	if (ret) {
		gh_err("failed read crc status reg %d", ret);
		return ret;
	}
	gh621x_reg_write(cd, GH621X_CRC16_CTRL_REG_ADDR, GH621X_CRC16_CTRL_DISABLE_VAL);

	if (GH621X_CHECK_BIT_SET(reg_data_tmp, GH621X_CRC16_STATUS_ERR_BIT_VAL)) {
		gh_err("load patch fail, CRC16 Error");
		return -EFAULT;
	}

	if (GH621X_CHECK_BIT_SET(reg_data_tmp, GH621X_CRC16_STATUS_DONE_BIT_VAL)) {
		gh_info("load patch Success!");
		return 0;
	}

	gh_err("load patch fail, Unknow Error");
	return -EINVAL;
}

int gh621x_fw_update(struct gh_core_data *cd, u8 *config, int cfg_len, u8 *patch, int patch_len)
{
	int ret;
	u16 irq_status_tmp[GH621X_IRQ_STATUS_REG_MAX_COUNT];

	gh_info("%s in", __func__);

	ret = gh621x_sw_reset(cd);
	if (ret) {
		gh_err("failed reset ic before config/patch update");
		return ret;
	}

	ret += gh621x_regs_read(cd, GH621X_IRQ_HOST_STATUS_0_REG_ADDR, irq_status_tmp, GH621X_IRQ_STATUS_REG_MAX_COUNT); // clear irq status
	ret += gh621x_reg_write(cd, GH621X_I2C_MEM_EN_REG_ADDR, GH621X_I2C_MEM_EX_ENABLE_VAL); //I2C配置关
	if (ret) {
		gh_err("failed clear irq %d", ret);
		return ret;
	}

	ret = gh621x_load_config(cd, (gh621x_reg_t *)config, cfg_len/sizeof(gh621x_reg_t));
	if (ret) {
		gh_err("failed load config %d", ret);
		return ret;
	}

	ret = gh621x_regs_read(cd, GH621X_IRQ_HOST_STATUS_0_REG_ADDR, irq_status_tmp, GH621X_IRQ_STATUS_REG_MAX_COUNT); // clear irq status
        ret += gh621x_load_patch(cd, patch, patch_len);
        if (ret) {
            gh_err("fail to load patch %d", ret);
            return ret;
        }

	ret = gh621x_reg_write(cd, GH621X_I2C_MEM_EN_REG_ADDR, GH621X_I2C_MEM_ENABLE_VAL); //I2C配置开
	return ret;
}

int gh621x_work_start(struct gh_core_data *cd)
{
	int ret = 0;
	u16 reg_val = 0;

	ret = gh621x_reg_write(cd, GH621X_SAR_INIT_FLAG_REG_ADDR, GH621X_SAR_INIT_DONE_PATCH_VAL);
	ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_2_REG_ADDR, GH621X_RELEASE_MCU_VAL);
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &reg_val);
	ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, reg_val | GH621X_WORK_MODE_HIGH_VAL);
	if (ret)
		gh_err("failed start work %d", ret);

	cd->status = GH621X_STATUS_STARTED;
	return ret;
}

int gh621x_work_stop(struct gh_core_data *cd)
{
	int ret;

	ret = gh621x_reg_write(cd, GH621X_PRIVATE_REG_2_REG_ADDR, GH621X_MCU_HOLD_BIT_VAL);
	if (ret)
		gh_err("failed stop work %d", ret);

	return ret;
}

//Exit lowpower mode, only in this mode can be write&read reg from gh621x
int gh621x_exit_lowpower_mode(struct gh_core_data *cd)
{
	int ret = 0;
	u16 reg_val_tmp = 0;
	u16 reg_val_tmp2 = 0;

	gh_debug("exit lowpower mode");
	ret = gh621x_reg_read(cd, GH621X_PRIVATE_REG_3_REG_ADDR, &reg_val_tmp);
	if (ret) {
		gh_err("%s, failed read reg 0x%x", __func__, GH621X_PRIVATE_REG_3_REG_ADDR);
		return ret;
	}

	if (GH621X_CHECK_BIT_NOT_SET_BITMASK(reg_val_tmp, GH621X_FSM_SLP_STATUS_BIT_MSK_VAL, 0)) {
		if (GH621X_CHECK_BIT_NOT_SET(reg_val_tmp, GH621X_FSM_DSLP_STATUS_BIT_VAL)) {
			// Not in dlsp state
			ret = gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_WAKEUP_HOST);
		} else {
			// in dlsp state
			ret = gh621x_reg_read(cd, GH621X_PRIVATE_REG_2_REG_ADDR, &reg_val_tmp);
			ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &reg_val_tmp2);
			if ((GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp, GH621X_MCU_HOLD_BIT_VAL, 0)) 
				&& (GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp2, GH621X_WORK_MODE_HIGH_VAL, 0))) {
				ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_2_REG_ADDR, GH621X_MCU_HOLD_BIT_VAL); // hold mcu
			}
			ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_NORMAL_HOST);
		}
		usleep_range(1000, 1100);
	}
	if (ret)
		gh_debug("%s, failed, %d", __func__, ret);
	return ret;
}

//Enter lowpower mode
int gh621x_enter_lowpower_mode(struct gh_core_data *cd)
{
	int ret = 0;
	u16 reg_val_tmp = 0;
	u16 reg_val_tmp2 = 0;

	gh_debug("enter lowpower mode");
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_3_REG_ADDR, &reg_val_tmp);
	if (GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp, GH621X_FSM_SLP_STATUS_BIT_MSK_VAL, 0)) {
		// if not in sleep mode
		if (GH621X_CHECK_BIT_NOT_SET(reg_val_tmp, GH621X_FSM_NORMAL_STATUS_BIT_VAL)) {
			ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_SLEEP_HOST);
		} else {
			ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_DSLP_HOST);
			ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_2_REG_ADDR, &reg_val_tmp);
			ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &reg_val_tmp2);
			if ((GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp, GH621X_MCU_HOLD_BIT_VAL, 0)) 
				&& (GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp2, GH621X_WORK_MODE_HIGH_VAL, 0))) {
				ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_2_REG_ADDR, GH621X_RELEASE_MCU_VAL);
			}
		}
		usleep_range(1000, 1100);
	}

	if (ret)
		gh_err("%s, failed %d", __func__, ret);
	return ret;
}

//Exit sleep mode
int gh621x_enable_nosleep_mode(struct gh_core_data *cd)
{
	int ret = 0;
	u16 reg_val_tmp = 0;
	u16 reg_val_tmp2 = 0;

	gh_debug("enable nosleep mode");
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_2_REG_ADDR, &reg_val_tmp);
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &reg_val_tmp2);
	if (ret) {
		gh_err("%s, failed read PRIVATE_REG_2 or PRIVATE_REG_1, %d", __func__, ret);
		return ret;
	}

	if ((GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp, GH621X_MCU_HOLD_BIT_VAL, 0)) 
		&& (GH621X_CHECK_BIT_SET_BITMASK(reg_val_tmp2, GH621X_WORK_MODE_HIGH_VAL, 0))) {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, GH621X_PRIVATE_SP_ENABLE_NO_SLEEP_VAL);
	} else {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, GH621X_PRIVATE_ENABLE_NO_SLEEP_VAL);
	}

	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_3_REG_ADDR, &reg_val_tmp);
	if (GH621X_CHECK_BIT_NOT_SET(reg_val_tmp, GH621X_FSM_DSLP_STATUS_BIT_VAL)) {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_WAKEUP_HOST);
	} else {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_NORMAL_HOST);
	}
	usleep_range(1000, 1100);
	if (ret)
		gh_err("%s, failed enable nosleep mode, %d", __func__, ret);
	return ret;
}

//Enable sleep mode
int gh621x_enable_sleep_mode(struct gh_core_data *cd)
{
	int ret = 0;
	u16 prv_reg1 = 0, prv_reg2 = 0;

	gh_debug("enable sleep mode");
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_2_REG_ADDR, &prv_reg2);
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &prv_reg1);
	if (ret) {
		gh_err("%s failed read prv reg %d", __func__, ret);
		return ret;
	}
	if ((GH621X_CHECK_BIT_SET_BITMASK(prv_reg2, GH621X_MCU_HOLD_BIT_VAL, 0)) 
		&& (GH621X_CHECK_BIT_SET_BITMASK(prv_reg1, GH621X_WORK_MODE_HIGH_VAL, 0))) {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, GH621X_PRIVATE_SP_DISABLE_NO_SLEEP_VAL);
	} else {
		ret += gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, GH621X_PRIVATE_DISABLE_NO_SLEEP_VAL);
	}

	if (ret)
		gh_err("%s failed enable sleep mode, %d", __func__, ret);

	return ret;
}

//check gh621x still alive
int gh621x_active_confirm(struct gh_core_data *cd)
{
	int ret = 0;
	u16 reg_val = 0;

	ret = gh621x_reg_read(cd, GH621X_PRIVATE_REG_1_REG_ADDR, &reg_val);
	if (!ret && GH621X_CHECK_BIT_SET(reg_val, GH621X_WORK_MODE_HIGH_VAL)) {
		gh_debug("working");
		ret = gh621x_reg_read(cd, GH621X_CPU_ID_LO_REG_ADDR, &reg_val);
		if (!ret && reg_val == GH621X_CPU_ID_LO_REG_VAL) {
			return 0;
		}
		gh_info("no in working mode 0x%x, ret %d", reg_val, ret);
	}
	gh_info("active confirm failed %d, reg 0x%x", ret, reg_val);
	return -EFAULT;
}

/* @mode: 1 do reg read directly with wake up form sleep */
int gh621x_memory_read(struct gh_core_data *cd, u16 addr,
		u8 *data, unsigned int len, int mode)
{
	int ret;

	if (mode)
		return gh621x_read(cd, addr, data, len);

	/* exit no sleep before read */
	ret = gh621x_enable_nosleep_mode(cd);
	ret += gh621x_exit_lowpower_mode(cd);

	ret += gh621x_read(cd, addr, data, len);
	ret += gh621x_enable_sleep_mode(cd);
	return ret;
}

static int gh621x_memory_write(struct gh_core_data *cd, u16 addr,
		u8 *data, unsigned int len, int mode)
{
	int ret;

	if (mode)
		gh621x_write(cd, addr, data, len);

	ret = gh621x_enable_nosleep_mode(cd);
	ret += gh621x_exit_lowpower_mode(cd);

	ret += gh621x_write(cd, addr, data, len);
	ret += gh621x_enable_sleep_mode(cd);
	return ret;
}

#define GH621X_RAWDATA_SIZE (82)
#define SENSOR_CHANNEL_NUM   2

static void gh621x_rawdata_print(gh621x_rawdata_t *rawdata)
{
	int i;

        gh_info("preraw{d0=%d,d1=%d,d2=%d,d3=%d,d4=%d,d5=%d,d6=%d,d7=%d,d8=%d}",
                rawdata->pre_raw[0], rawdata->pre_raw[1],
                rawdata->pre_raw[2], rawdata->pre_raw[3],
		rawdata->pre_raw[4], rawdata->pre_raw[5],
		rawdata->pre_raw[6], rawdata->pre_raw[7], rawdata->pre_raw[8]);

	gh_info("useful{%d, %d, %d, %d, %d, %d, %d, %d, %d}",
                rawdata->useful[0], rawdata->useful[1],
                rawdata->useful[2], rawdata->useful[3],
		rawdata->useful[4], rawdata->useful[5],
		rawdata->useful[6], rawdata->useful[7], rawdata->useful[8]);

	gh_info("avg{%d, %d, %d, %d}", rawdata->avg[0], rawdata->avg[1], rawdata->avg[2], rawdata->avg[3]);
	gh_info("diff{%d, %d, %d, %d}", rawdata->diff[0], rawdata->diff[1], rawdata->diff[2], rawdata->diff[3]);

	// reuse avg field
	gh_info("cc_ref0(hex){%.4x, %.4x, %.4x, %.4x}", rawdata->avg[5], rawdata->avg[6], rawdata->avg[7], rawdata->avg[8]);
	// reuse diff field
	gh_info("cc_ref1(hex){%.4x, %.4x, %.4x, %.4x}", rawdata->diff[5], rawdata->diff[6], rawdata->diff[7], rawdata->diff[8]);

	for (i = 0; i < SENSOR_CHANNEL_NUM; i++) {
		gh_info("CS[%d] rawdata{sensor=%d, ref=%d,}; base=%d, diff=%d, prox_stat=%d",
			 i, rawdata->useful[i], rawdata->useful[SENSOR_CHANNEL_NUM + i],
			 rawdata->avg[i], rawdata->diff[i], (rawdata->prox_stat >> i) & 0x1);
	}
}

static int gh621x_rawdata_get(struct gh_core_data *cd, gh621x_rawdata_t *rawdata)
{
	int ret, i;
	u16 checksum;
	u16 buffer[GH621X_RAWDATA_SIZE];

	ret = gh621x_regs_read(cd, GH621X_SAR_RAWDATA_START_REG_ADDR, buffer, GH621X_RAWDATA_SIZE);
	if (ret) {
		gh_err("failed read rawdata %d", ret);
		return ret;
	}
	checksum = 0;
	for (i = 0; i < GH621X_RAWDATA_SIZE - 1; i++) {
		checksum += buffer[i];
	}
	if (checksum != buffer[GH621X_RAWDATA_SIZE - 1]) {
		gh_err("rawdata check failed 0x%x != 0x%x",
			 checksum, buffer[GH621X_RAWDATA_SIZE - 1]);
		return -EINVAL;
	}
	i = 0;
	rawdata->temp_cels = buffer[i];
	i += 1;
	rawdata->temp_raw = buffer[i] | (buffer[i + 1] << 16);
	i += 2;
	gh_be32_to_cpu_array(buffer + i, rawdata->pre_raw, MAX_CHANNEL_NUM);
	i += 2*MAX_CHANNEL_NUM;
	gh_be32_to_cpu_array(buffer + i, rawdata->useful, MAX_CHANNEL_NUM);
	i += 2*MAX_CHANNEL_NUM;
	gh_be32_to_cpu_array(buffer + i, rawdata->avg, MAX_CHANNEL_NUM);
	i += 2*MAX_CHANNEL_NUM;
	gh_be32_to_cpu_array(buffer + i, rawdata->diff, MAX_CHANNEL_NUM);
	i += 2*MAX_CHANNEL_NUM;
	rawdata->prox_stat = buffer[i];
	i += 1;
	rawdata->table_stat = buffer[i];
	i += 1;
	rawdata->body_stat = buffer[i];
	i += 1;
	rawdata->steady_stat = buffer[i];
	i += 1;
	rawdata->comp_stat = buffer[i];
	i += 1;
	rawdata->and_or_stat = buffer[i];
	return 0;
}

int gh621x_event_handler(struct gh_core_data *cd, struct gh_event_data *event_data)
{
	int ret;
	int i;
	u16 reg_tmp;
	u16 irq_status[GH621X_IRQ_STATUS_REG_MAX_COUNT] = {0};
	struct debug_data *debug_data = &cd->debug_data;

	ret = gh621x_enable_nosleep_mode(cd);
	ret += gh621x_exit_lowpower_mode(cd);
	ret += gh621x_active_confirm(cd);
	if (ret) {
		gh_err("failed wakeup ic %d", ret);
		return ret;
	}

	ret = gh621x_regs_read(cd, GH621X_IRQ_HOST_STATUS_0_REG_ADDR, irq_status, GH621X_IRQ_STATUS_REG_MAX_COUNT);
	if (ret) {
		gh_err("failed get irq status");
		goto err_irq;
	}

        gh_info("got int msg! status:%.4x, %.4x, %.4x, %.4x",
                        irq_status[0], irq_status[1], irq_status[2], irq_status[3]);
        if (GH621X_CHECK_BIT_SET(irq_status[0], GH621X_IRQ0_MSK_SAR_CHIP_RESET_BIT)) {
		gh_info("has reset");
		goto err_irq;
	}

	memcpy(debug_data->irq_status, irq_status, sizeof(irq_status));

	if (irq_status[0] & (GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT | GH621X_IRQ0_MSK_SAR_PROX_CLOSE_BIT
			     | GH621X_IRQ0_MSK_SAR_PROX_FAR_BIT)) {
		gh_info("fw_ver: 0x%.4x,0x%.4x,0x%.4x,0x%.4x",
			cd->fw_version[0], cd->fw_version[1], cd->fw_version[2], cd->fw_version[3]);
		memcpy(debug_data->fw_version, cd->fw_version, GH621X_FW_VERSION_MAX_COUNT * 2);

		if (!gh621x_regs_read(cd, GH621X_CCSEL_DATA_START_REG_ADDR, debug_data->ccsel_data, MAX_CHANNEL_NUM)) {
			gh_info("cc_sel(hex): %.4x,%.4x,%.4x,%.4x",
				debug_data->ccsel_data[0], debug_data->ccsel_data[1],
				debug_data->ccsel_data[2], debug_data->ccsel_data[3]);
			gh_info("cc_ref(hex): %.4x,%.4x,%.4x,%.4x",
				debug_data->ccsel_data[4], debug_data->ccsel_data[5],
				debug_data->ccsel_data[6], debug_data->ccsel_data[7]);
		}
		ret = gh621x_rawdata_get(cd, &debug_data->rawdata);
		if (ret) {
			gh_err("failed get rawdata %d", ret);
			return ret;
		}
		gh621x_rawdata_print(&debug_data->rawdata);
	}

	if(GH621X_CHECK_BIT_SET(irq_status[0], GH621X_IRQ0_MSK_SAR_PROX_CLOSE_BIT) || GH621X_CHECK_BIT_SET(irq_status[0], GH621X_IRQ0_MSK_SAR_PROX_FAR_BIT)){
		for (i = 0; i < SENSOR_CHANNEL_NUM; i++) {
			event_data->state[i+1] = (debug_data->rawdata.prox_stat >> i) & 0x1;
			gh_info("%s:state i=%d : State:%d\n",__func__,i,event_data->state[i+1]);
		}
	}


	if (GH621X_CHECK_BIT_SET(irq_status[0], GH621X_IRQ0_MSK_SAR_PROX_CLOSE_BIT)) // result close
		event_data->state[0] = STATE_CLOSE;
	else if (GH621X_CHECK_BIT_SET(irq_status[0], GH621X_IRQ0_MSK_SAR_PROX_FAR_BIT)) // result far
		event_data->state[0] = STATE_FAR;

	if (gh621x_reg_write(cd, GH621X_SAR_DATA_READ_DONE_REG_ADDR, GH621X_SAR_DATA_READ_DONE_WAIT_VAL))
		gh_err("failed write read data done reg");

	// TODO need confirm
	ret += gh621x_reg_read(cd, GH621X_PRIVATE_REG_3_REG_ADDR, &reg_tmp);
	if((reg_tmp & 0x1000) != 0x1000)
		// ic does not in work mode, restart work
		gh621x_reg_write(cd, 0x0302, 0x0001);

err_irq:
	if (gh621x_enable_sleep_mode(cd))
		gh_info("failed enable sleep mode");

	return ret;

}

int gh621x_mode_ctrl(struct gh_core_data *cd, int mode)
{
	int ret;
	u16 irq_flag = 0;

	ret = gh621x_enable_nosleep_mode(cd);
	ret += gh621x_exit_lowpower_mode(cd);
	ret += gh621x_reg_read(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, &irq_flag);
	if (ret) {
		gh_err("failed get irq flag %d", ret);
		goto err_out;
	}

	switch (mode) {
	case WORK_MODE_NORMAL: // normal work mode
		if (irq_flag & GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT) {
			irq_flag &= ~GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT;
			ret = gh621x_reg_write(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, irq_flag);
			gh_info("disable rawdata mode %d", ret);
		}
		break;
	case WORK_MODE_RAWDATA: // rawdata mode
		if (!(irq_flag & GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT)) {
			irq_flag |= GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT;
			ret = gh621x_reg_write(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, irq_flag);
			gh_info("enable rawdata mode %d", ret);
		}
		break;
	default:
		gh_err("unsupported mode %d", mode);
		ret = -EINVAL;
		break;
	}

err_out:
	if (gh621x_enable_sleep_mode(cd))
		gh_err("[mode ctrl] failed enable sleep mode");
	return ret;
}

static int gh621x_calibration(struct gh_core_data *cd)
{
	int ret;
	int cnt = 200;
	u16 irq_flag = 0, reg_val = 0;

	gh_irq_enable(cd, 0);
	ret = gh621x_enable_nosleep_mode(cd);
	ret += gh621x_exit_lowpower_mode(cd);

	/*2023/08/12 debug cali */
	ret += gh621x_active_confirm(cd);
	if (ret) {
		gh_err("failed wakeup ic %d", ret);
		//return ret;
	}

	/* read back irq flag for backup */
	ret += gh621x_reg_read(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, &irq_flag);
	/* enable autocc done irq */
	ret += gh621x_reg_write(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, GH621X_IRQ0_MSK_SAR_AUTOCC_DONE_BIT);
	/* trig autocc */
	ret += gh621x_reg_write(cd, GH621X_SAR_AUTOCC_TRG_CH_REG_ADDR, GH621X_SAR_AUTOCC_TRG_CH_FLAG_VAL | GH621X_SAR_AUTOCC_TRG_VAL);
	if (ret) {
		gh_err("failed trig autocc");
		goto err_out;
	}
	msleep(50); // 50ms delay
	while(cnt--) {
		ret = gh621x_reg_read(cd, GH621X_IRQ_HOST_STATUS_0_REG_ADDR, &reg_val);
		if (ret) {
			gh_err("failed read autocc done status, %d", ret);
			break;
		}
		if (cnt == 0) {
			gh_err("wait for autocc done timeout");
			ret = -EFAULT;
			goto err_out;
		}
		if (reg_val & GH621X_IRQ0_MSK_SAR_AUTOCC_DONE_BIT) {
			gh_info("get autocc done flag %d, 0x%x", cnt, reg_val);
			usleep_range(2000, 2100);
			//gh621x_reg_read(cd, GH621X_IRQ_HOST_STATUS_0_REG_ADDR, &reg_val);
			break;
		}
		usleep_range(10000, 11000);
	}

	if (!gh621x_reg_read(cd, GH621X_PRIVATE_REG_3_REG_ADDR, &reg_val)) {
		// TODO need confirm
		if (!(reg_val & 0x1000))
			gh621x_reg_write(cd, 0x0302, 0x0001);
	}

err_out:
	if (irq_flag) {
		/* backup irq flag */
		gh621x_reg_write(cd, GH621X_HOST_IRQ_CTRL_0_REG_ADDR, irq_flag);
	}
	if (gh621x_enable_sleep_mode(cd))
		gh_err("[calibration] failed enable sleep mode");
	gh_irq_enable(cd, 1);
	return ret;
}


#define   GH621X_FIX_FREQ_REG_ADDR                       (0x02C8)    /**< dummy reg 4 */

#define   GH621X_MCU_PD_BIT_SET_VAL                      (0x0004)    /**< mcu pd bit set value */
#define   GH621X_MCU_PD_BIT_RESET_VAL                    (0x0000)    /**< mcu pd bit reset value */
// enter deep sleep mode
int gh621x_suspend(struct gh_core_data *cd, enum power_mode mode)
{

	gh_debug("[suspend] try enter suspend %d", mode);
	gh_irq_enable(cd, 0);	

	return 0;
}

#if 0
static int gh621x_check_comm_status_and_wait_wakeup(struct gh_core_data *cd)
{
    int ret;
    u16 reg_val = 0;
    do {
        ret = gh621x_reg_read(cd, GH621X_CPU_ID_LO_REG_ADDR, &reg_val);
        if (reg_val == GH621X_CHIP_SLEEP_COMM_VAL)
        {
            gh621x_exit_lowpower_mode(cd);
            gh_info("[wakeup]chip is in sleep mode, wakeup chip retry");
        }
        else
        {
            if (reg_val != GH621X_CPU_ID_LO_REG_VAL)
            {
                gh_err("[wakeup]communicate unstable, need check");
                ret = -EFAULT;
            }
            break;
        }
    } while (1);

    return ret;
}

static void gh621x_set_work_mode_high(struct gh_core_data *cd)
{
	gh_info("[resume] set work mode high");
	gh621x_reg_write(cd, GH621X_PRIVATE_REG_0_REG_ADDR, GH621X_PRIVATE_CMD_DSLP_HOST);

	gh621x_reg_write(cd, GH621X_PRIVATE_REG_2_REG_ADDR, GH621X_RELEASE_MCU_VAL);

	usleep_range(1000, 1100);
	gh621x_reg_write(cd, GH621X_PRIVATE_REG_1_REG_ADDR, GH621X_WORK_MODE_HIGH_VAL);
}
#endif
int gh621x_resume(struct gh_core_data *cd)
{
	gh_debug("[resume] enter, status %d", cd->status);
	gh_irq_enable(cd, 1);
	return 0;
}

int gh621x_fw_ver(struct gh_core_data *cd, u16 *version)
{
	int ret;

	ret = gh621x_enable_nosleep_mode(cd);
	ret |= gh621x_exit_lowpower_mode(cd);
	ret |=  gh621x_regs_read(cd, GH621X_FW_VERSION_START_REG_ADDR, version, GH621X_FW_VERSION_MAX_COUNT);
	if (gh621x_enable_sleep_mode(cd))
		gh_err("[fw_ver] failed enable sleep mode");

	return ret;
}

struct gh_hw_ops gh621x_hw_ops = {
	.read = gh621x_memory_read,
	.write = gh621x_memory_write,
	.dev_confirm = gh621x_dev_confirm,
	.mode_ctrl = gh621x_mode_ctrl,
	.sw_reset = gh621x_sw_reset,
	.fw_update = gh621x_fw_update,
	.work_start = gh621x_work_start,
	.work_stop = gh621x_work_stop,
	.event_handler = gh621x_event_handler,
	.calibration = gh621x_calibration,
	.fw_ver = gh621x_fw_ver,
	.resume = gh621x_resume,
	.suspend = gh621x_suspend
};