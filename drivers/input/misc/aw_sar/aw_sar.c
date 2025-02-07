#include "./comm/aw_sar_chip_interface.h"
#include "aw_sar.h"

#define AW_SAR_I2C_NAME		"awinic_sar"
#define AW_SAR_DRIVER_VERSION	"v0.1.5"

#define AW_I2C_RW_RETRY_TIME_MIN		(2000)
#define AW_I2C_RW_RETRY_TIME_MAX		(3000)
#define AW_RETRIES						(5)

#define AW_SAR_AWRW_OffSET				(20)
#define AW_SAR_AWRW_DATA_WIDTH			(5)
#define AW_DATA_OffSET_2				(2)
#define AW_DATA_OffSET_3				(3)
#define AW_SAR_CH_MAX					(20)
#define AW_POWER_ON_SYSFS_DELAY_MS		(5000)
#define AW_SAR_OFFSET_LEN				(15)

static struct mutex aw_sar_lock;

static int32_t aw_sar_get_chip_info(struct aw_sar *p_sar);
static void aw_sar_sensor_free(struct aw_sar *p_sar);

//Because disable/enable_irq api Therefore, IRQ is embedded
void aw_sar_disable_irq(struct aw_sar *p_sar)
{
	if (p_sar->irq_init.host_irq_stat == IRQ_ENABLE) {
		disable_irq(p_sar->irq_init.to_irq);
		p_sar->irq_init.host_irq_stat = IRQ_DISABLE;
	}
}

void aw_sar_enable_irq(struct aw_sar *p_sar)
{
	if (p_sar->irq_init.host_irq_stat == IRQ_DISABLE) {
		enable_irq(p_sar->irq_init.to_irq);
		p_sar->irq_init.host_irq_stat = IRQ_ENABLE;
	}
}

//Chip logic part start
//Load default array function
static int32_t aw_sar_para_loaded_func(struct i2c_client *i2c,
					const struct aw_sar_para_load_t *para_load)
{
	int32_t i = 0;
	int32_t ret = 0;

	for (i = 0; i < para_load->reg_arr_len; i = i + 2) {
		ret = aw_sar_i2c_write(i2c, (uint16_t)para_load->reg_arr[i],
						para_load->reg_arr[i + 1]);
		if (ret != AW_OK) {
			return -AW_REG_LOAD_ERR;
		}
		AWLOGD(&i2c->dev,"reg_addr = 0x%04x, reg_data = 0x%08x",
				para_load->reg_arr[i],
				para_load->reg_arr[i + 1]);
	}

	AWLOGD(&i2c->dev, "para writen completely");

	return AW_OK;
}

//Mode setting function
static void aw_sar_mode_set_func(struct i2c_client *i2c, int to_irq, struct aw_sar_mode_set_t *mode_set_para,
				const struct aw_sar_mode_set_t *mode_set, uint8_t len)
{
	uint8_t i = 0;

	AWLOGD(&i2c->dev, "enter");

	for(i = 0; i < len; i++) {
		//AWLOGD(&i2c->dev, "i = %d mode_set:%d chip_id:%d", i, mode_set[i].chip_id, mode_set_para->chip_id);
		if ((mode_set[i].chip_mode.curr_mode == mode_set_para->chip_mode.curr_mode) &&
				(mode_set[i].chip_mode.last_mode == mode_set_para->chip_mode.last_mode) &&
				((mode_set[i].chip_id == AW_SAR_NONE_CHECK_CHIP) ||
					((mode_set[i].chip_id & mode_set_para->chip_id) != 0))) {
			if (mode_set[i].mode_switch_ops.enable_clock != NULL) {
				mode_set[i].mode_switch_ops.enable_clock(i2c);
			}
			if (mode_set[i].mode_switch_ops.rc_irqscr != NULL) {
				mode_set[i].mode_switch_ops.rc_irqscr(i2c);
			}
			if (mode_set[i].mode_switch_ops.mode_update != NULL) {
				AWLOGD(&i2c->dev, "set curr_mode:%d, ok", mode_set_para->chip_mode.curr_mode);
				mode_set[i].mode_switch_ops.mode_update(i2c);
			}
			break;
		}
	}
}

static int32_t aw_sar_check_init_over_irq_func(struct i2c_client *i2c,
					const struct aw_sar_init_over_irq_t *p_check_irq)
{
	int32_t ret = -AW_ERR;
	uint32_t irq_stat = 0;
	int16_t cnt = p_check_irq->wait_times;

	do {
		ret = aw_sar_i2c_read(i2c, p_check_irq->reg_irqsrc, &irq_stat);
		if (ret < 0) {
			return -AW_ERR;
		}
		if (((irq_stat >> p_check_irq->irq_offset_bit) & p_check_irq->irq_mask) == p_check_irq->irq_flag) {
			AWLOGD(&i2c->dev, "check_init_over_irqt ok!");
			return AW_OK;
		} else {
			AWLOGE(&i2c->dev, "init over irq no ok! val:0x%x cnt: %d", irq_stat, cnt);
		}
		mdelay(1);
	} while (cnt--);

	if (cnt < 0) {
		AWLOGE(&i2c->dev, "init over irq error!");
	}

	return -AW_ERR_IRQ_INIT_OVER;
}

static int32_t aw_sar_soft_reset_func(struct i2c_client *i2c, const struct aw_sar_soft_rst_t *p_soft_rst)
{
	int32_t ret = 0;

	AWLOGD(&i2c->dev, "enter");

	ret = aw_sar_i2c_write(i2c, p_soft_rst->reg_rst, p_soft_rst->reg_rst_val);
	if (ret < 0) {
		AWLOGE(&i2c->dev, "soft_reset error: %d", ret);
		return -AW_ERR;
	}

	msleep(p_soft_rst->delay_ms);

	return AW_OK;
}
//Chip logic part end

static int32_t aw_sar_parse_bin(const struct firmware *cont, struct aw_sar *p_sar)
{
	struct aw_bin *aw_bin = NULL;
	int32_t ret = -AW_ERR;

	if (!cont) {
		AWLOGE(p_sar->dev, "def_reg_bin request error!");
		return -AW_ERR;
	} else {
		AWLOGI(p_sar->dev, "def_reg_bin request successfully");
	}

	AWLOGD(p_sar->dev, "Bin file size: %d", (uint32_t)cont->size);

	aw_bin = kzalloc(cont->size + sizeof(struct aw_bin), GFP_KERNEL);
	if (!aw_bin) {
		kfree(aw_bin);
		release_firmware(cont);
		AWLOGE(p_sar->dev, "failed to allcating memory!");
		return -AW_ERR;
	}

	aw_bin->info.len = cont->size;
	memcpy(aw_bin->info.data, cont->data, cont->size);

	if (cont != NULL) {
		release_firmware(cont);
	}

	ret = aw_sar_parsing_bin_file(aw_bin);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "parse bin fail! ret = %d", ret);
		goto err;
	}

	//Write bin file execution process
	if (p_sar->load_bin.bin_opera_func != NULL) {
		ret = p_sar->load_bin.bin_opera_func(aw_bin, p_sar);
		if (ret != AW_OK) {
			AWLOGE(p_sar->dev, "load_bin_to_chip error!");
			if (p_sar->load_bin.bin_load_fail_opera_func != NULL) {
				ret = p_sar->load_bin.bin_load_fail_opera_func(aw_bin, p_sar);
				if (ret != AW_OK) {
					AWLOGE(p_sar->dev, "bin_load_fail_opera_func error!");
					goto err;
				} else {
					AWLOGD(p_sar->dev, "bin_load_fail_opera_func OK!");
				}
			} else {
				goto err;
			}
		}
	} else {
		AWLOGE(p_sar->dev, "bin_opera_func is null error!");
	}

	if (aw_bin != NULL) {
		kfree(aw_bin);
	}

	AWLOGE(p_sar->dev, "load_def_reg_bin ok!!!");

	return AW_OK;
err:
	if (aw_bin != NULL) {
		kfree(aw_bin);
	}
	return -AW_ERR;
}

int32_t aw_sar_load_bin_comm(struct aw_sar *p_sar)
{
	int32_t ret = -AW_ERR;
	const struct firmware *fw = NULL;

	AWLOGD(p_sar->dev, "name :%s", p_sar->load_bin.bin_name);

	ret = request_firmware(&fw, p_sar->load_bin.bin_name, p_sar->dev);
	AWLOGD(p_sar->dev, "ret = %d", ret);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "parse %s error!", p_sar->load_bin.bin_name);
		return -AW_ERR;
	} else {
		AWLOGD(p_sar->dev, "parse %s ok!", p_sar->load_bin.bin_name);
	}

	ret = aw_sar_parse_bin(fw, p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "reg_bin %s load error!", p_sar->load_bin.bin_name);
		return -AW_ERR;
	} else {
		AWLOGD(p_sar->dev, "reg_bin %s load ok!",p_sar->load_bin.bin_name);
	}

	return AW_OK;
}

int32_t aw_sar_parse_dts_comm(struct device *dev, struct device_node *np, struct aw_sar_dts_info *p_dts_info)
{
	int32_t val = 0;

	val = of_property_read_u32(np, "sar-num", &p_dts_info->sar_num);
	if (val != 0) {
		AWLOGE(dev, "multiple sar failed!");
		return -AW_ERR;
	} else {
		AWLOGI(dev, "sar num = %d", p_dts_info->sar_num);
	}

	p_dts_info->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (p_dts_info->irq_gpio < 0) {
		p_dts_info->irq_gpio = -1;
		AWLOGE(dev, "no irq gpio provided.");
		return -AW_ERR;
	} else {
		AWLOGI(dev, "irq gpio provided ok.");
	}

	val = of_property_read_u32(np, "channel_use_flag", &p_dts_info->channel_use_flag);
	if (val != 0) {
		AWLOGE(dev, "channel_use_flag failed!");
		return -AW_ERR;
	} else {
		AWLOGI(dev, "channel_use_flag = 0x%x", p_dts_info->channel_use_flag);
	}

	//GPIO is set as internal pull-up input
	p_dts_info->use_inter_pull_up = of_property_read_bool(np, "aw_sar,pin_set_inter_pull-up");
	AWLOGI(dev, "aw_sar,use_inter_pull_up = <%d>", p_dts_info->use_inter_pull_up);

	p_dts_info->use_pm = of_property_read_bool(np, "aw_sar,using_pm_ops");
	AWLOGI(dev, "aw_sar,pin_set_inter_pull-up = <%d>", p_dts_info->use_pm);

	p_dts_info->update_fw_flag = of_property_read_bool(np, "aw_sar,update_fw");
	p_dts_info->update_fw_flag = 0;
	AWLOGI(dev, "aw_sar,update_fw = <%d>", p_dts_info->update_fw_flag);

	return AW_OK;
}

static int32_t aw_sar_parse_dts(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	AWLOGD(p_sar->dev, "enter");

	ret = aw_sar_parse_dts_comm(p_sar->dev, p_sar->i2c->dev.of_node, &p_sar->dts_info);

	//Special requirements of SAR chip
	if (p_sar->p_sar_para->p_platform_config->p_add_parse_dts_fn != NULL) {
		ret |= p_sar->p_sar_para->p_platform_config->p_add_parse_dts_fn(p_sar);
	}

	return ret;
}

static irqreturn_t aw_sar_irq(int32_t irq, void *data)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;
	uint32_t irq_status = 0;

	//step1: read clear interrupt
	if (p_sar->p_sar_para->p_platform_config->p_irq_init->rc_irq_fn != NULL) {
		irq_status = p_sar->p_sar_para->p_platform_config->p_irq_init->rc_irq_fn(p_sar->i2c);
	}

	//step2: Read the status register for status reporting
	if (p_sar->p_sar_para->p_platform_config->p_irq_init->irq_spec_handler_fn != NULL) {
		p_sar->p_sar_para->p_platform_config->p_irq_init->irq_spec_handler_fn(irq_status, p_sar);
	}

	return IRQ_HANDLED;
}

static int32_t aw_sar_irq_init_comm(struct aw_sar *p_sar, const struct aw_sar_irq_init_t *p_irq_init)
{
	int32_t ret = 0;
	irq_handler_t thread_fn = p_irq_init->thread_fn;

	snprintf(p_sar->irq_init.label, sizeof(p_sar->irq_init.label),
				"aw_sar%d_gpio", p_sar->dts_info.sar_num);
	snprintf(p_sar->irq_init.dev_id, sizeof(p_sar->irq_init.dev_id),
				"aw_sar%d_irq", p_sar->dts_info.sar_num);

	if (gpio_is_valid(p_sar->dts_info.irq_gpio)) {
		p_sar->irq_init.to_irq = gpio_to_irq(p_sar->dts_info.irq_gpio);
		ret = devm_gpio_request_one(p_sar->dev,
						p_sar->dts_info.irq_gpio,
						p_irq_init->flags,
						p_sar->irq_init.label);
		if (ret) {
			AWLOGE(p_sar->dev,
				"request irq gpio failed, ret = %d", ret);
			ret = -AW_ERR;
		} else {
			if (thread_fn == NULL) {
				thread_fn = aw_sar_irq;
			}
			ret = devm_request_threaded_irq(p_sar->dev,
							p_sar->irq_init.to_irq,
							p_irq_init->handler,
							thread_fn,
							p_irq_init->irq_flags,
							p_sar->irq_init.dev_id,
							p_sar);
			if (ret != 0) {
				AWLOGE(p_sar->dev,
						"failed to request IRQ %d: %d",
						p_sar->irq_init.to_irq, ret);
				ret = -AW_ERR;
			} else {
				AWLOGI(p_sar->dev,
					"IRQ request successfully!");
				ret = AW_OK;
			}
		}
	} else {
			AWLOGE(p_sar->dev, "irq gpio invalid!");
			return -AW_ERR;
	}

	AWLOGI(p_sar->dev, "disable_irq");
	p_sar->irq_init.host_irq_stat = IRQ_DISABLE;
	disable_irq(p_sar->irq_init.to_irq);

	AWLOGD(p_sar->dev, "irq init success!");

	return AW_OK;
}

static int32_t aw_sar_irq_init(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "aw_sar_irq_init enter!");

	if (p_sar->p_sar_para->p_platform_config->p_irq_init == NULL) {
		AWLOGE(p_sar->dev, "AW_INVALID_PARA");
		return AW_INVALID_PARA;
	}

	if (p_sar->p_sar_para->p_platform_config->p_irq_init->p_irq_init_fn != NULL) {
		AWLOGE(p_sar->dev, "p_irq_init_fn");
		return p_sar->p_sar_para->p_platform_config->p_irq_init->p_irq_init_fn(p_sar);
	}

	return aw_sar_irq_init_comm(p_sar, p_sar->p_sar_para->p_platform_config->p_irq_init);
}

static void aw_sar_irq_free(struct aw_sar *p_sar)
{
	if ((p_sar->p_sar_para->p_platform_config != NULL) &&
		(p_sar->p_sar_para->p_platform_config->p_irq_init != NULL) &&
		(p_sar->p_sar_para->p_platform_config->p_irq_init->p_irq_deinit_fn != NULL)) {
		p_sar->p_sar_para->p_platform_config->p_irq_init->p_irq_deinit_fn(p_sar);
		AWLOGE(p_sar->dev, "AW_INVALID_PARA");
		return;
	}
	AWLOGE(p_sar->dev, "aw_sar_irq_free");

}

static int32_t aw_sar_input_init_comm(struct aw_sar *p_sar)
{
	uint32_t i = 0;
	int32_t ret = 0;

	p_sar->channels_arr = devm_kzalloc(p_sar->dev,
				sizeof(struct aw_channels_info) *
				p_sar->p_sar_para->ch_num_max,
				GFP_KERNEL);
	if (p_sar->channels_arr == NULL) {
		AWLOGE(p_sar->dev, "devm_kzalloc err");
		return -AW_ERR;
	}

	for (i = 0; i < p_sar->p_sar_para->ch_num_max; i++) {
		snprintf(p_sar->channels_arr[i].name,
				sizeof(p_sar->channels_arr->name),
				"aw_sar%d_ch%d",
				p_sar->dts_info.sar_num, i);

		p_sar->channels_arr[i].last_channel_info = 0;

		if ((p_sar->dts_info.channel_use_flag >> i) & 0x01) {
			p_sar->channels_arr[i].used = AW_TRUE;
			p_sar->channels_arr[i].input = input_allocate_device();
			if (p_sar->channels_arr[i].input == NULL) {
				return -AW_ERR;
			}
			p_sar->channels_arr[i].input->name = p_sar->channels_arr[i].name;
			input_set_abs_params(p_sar->channels_arr[i].input,
						ABS_DISTANCE, -1, 100, 0, 0);
			ret = input_register_device(p_sar->channels_arr[i].input);
			if (ret) {
				AWLOGE(p_sar->dev, "failed to register input device");
				return -AW_ERR;
			}
		} else {
			p_sar->channels_arr[i].used = AW_FALSE;
			p_sar->channels_arr[i].input = NULL;
		}
	}

	return AW_OK;
}

static int32_t aw_sar_input_init(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_platform_config->p_input_init_fn != NULL) {
		return p_sar->p_sar_para->p_platform_config->p_input_init_fn(p_sar);
	}

	return aw_sar_input_init_comm(p_sar);
}

static void aw_sar_input_free_comm(struct aw_sar *p_sar)
{
	uint8_t i = 0;

	for (i = 0; i < p_sar->p_sar_para->ch_num_max; i++) {
		if (p_sar->channels_arr[i].input != NULL) {
			input_unregister_device(p_sar->channels_arr[i].input);
			input_free_device(p_sar->channels_arr[i].input);
		}
	}
	AWLOGE(p_sar->dev, "aw_sar_input_free ok");
}

static void aw_sar_input_free(struct aw_sar *p_sar)
{
	if ((p_sar->p_sar_para->p_platform_config != NULL) &&
		(p_sar->p_sar_para->p_platform_config->p_input_deinit_fn != NULL)) {
		p_sar->p_sar_para->p_platform_config->p_input_deinit_fn(p_sar);
		AWLOGE(p_sar->dev, "AW_INVALID_PARA");
		return;
	}
	AWLOGE(p_sar->dev, "aw_sar_input_free_comm");

	aw_sar_input_free_comm(p_sar);
}

static int32_t aw_sar_check_init_over_irq_comm(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	AWLOGD(p_sar->dev, "enter");

	if (p_sar->p_sar_para->p_init_over_irq == NULL) {
		return AW_INVALID_PARA;
	}

	ret = aw_sar_check_init_over_irq_func(p_sar->i2c, p_sar->p_sar_para->p_init_over_irq);
	if (ret == -AW_ERR_IRQ_INIT_OVER) {
		AWLOGD(p_sar->dev, "ret is AW_ERR_IRQ_INIT_OVER");
		if (p_sar->p_sar_para->p_init_over_irq->p_get_err_type_fn != NULL) {
			//Consider the abnormality reasonable
			if (p_sar->p_sar_para->p_init_over_irq->p_get_err_type_fn(p_sar) == AW_OK) {
				AWLOGD(p_sar->dev, "get_err_type return ok");
				p_sar->fw_fail_flag = AW_TRUE;
				return AW_OK;
			}
		}
	}

	return ret;
}

//If there is no special operation on the chip, execute the common process
int32_t aw_sar_check_init_over_irq(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_init_over_irq == NULL) {
		return AW_INVALID_PARA;
	}

	if (p_sar->p_sar_para->p_init_over_irq->p_check_init_over_irq_fn != NULL) {
		return p_sar->p_sar_para->p_init_over_irq->p_check_init_over_irq_fn(p_sar);
	}

	return aw_sar_check_init_over_irq_comm(p_sar);
}

static int32_t aw_sar_chip_other_operation(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_other_operation != NULL) {
		return p_sar->p_sar_para->p_other_operation(p_sar);
	}

	return AW_OK;
}

static void aw_sar_chip_other_operation_free(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_other_opera_free != NULL) {
		p_sar->p_sar_para->p_other_opera_free(p_sar);
	}
}

int32_t aw_sar_soft_reset(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	//If a private interface is defined, the private interface is used
	if ((p_sar->p_sar_para->p_soft_rst != NULL) && 
		(p_sar->p_sar_para->p_soft_rst->p_soft_reset_fn != NULL)) {
		return p_sar->p_sar_para->p_soft_rst->p_soft_reset_fn(p_sar);
	}

	return aw_sar_soft_reset_func(p_sar->i2c,  p_sar->p_sar_para->p_soft_rst);
}

static int32_t aw_sar_check_chipid(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para == NULL) {
		return AW_INVALID_PARA;
	}

	if (p_sar->p_sar_para->p_check_chipid != NULL) {
		if (p_sar->p_sar_para->p_check_chipid->p_check_chipid_fn != NULL) {
			return p_sar->p_sar_para->p_check_chipid->p_check_chipid_fn(p_sar);
		}
	}

	return -AW_ERR;
}

int32_t aw_sar_load_def_reg_bin(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if ((p_sar->p_sar_para->p_reg_bin == NULL) ||
		(p_sar->p_sar_para->p_reg_bin->bin_name == NULL)) {
		AWLOGE(p_sar->dev, "p_reg_bin is NULL or bin_name is NULL error");
		p_sar->ret_val = AW_BIN_PARA_INVALID;
		return AW_INVALID_PARA;
	}

	snprintf(p_sar->load_bin.bin_name, sizeof(p_sar->load_bin.bin_name),
			"%s_%d.bin", p_sar->p_sar_para->p_reg_bin->bin_name, p_sar->dts_info.sar_num);

	p_sar->load_bin.bin_opera_func = p_sar->p_sar_para->p_reg_bin->bin_opera_func;

	return aw_sar_load_bin_comm(p_sar);
}

static int32_t aw_sar_para_loaded(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_reg_arr == NULL) {
		return AW_INVALID_PARA;
	}

	aw_sar_para_loaded_func(p_sar->i2c, p_sar->p_sar_para->p_reg_arr);

	return AW_OK;
}

static int32_t aw_sar_reg_update_boot_work(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if ((p_sar->p_sar_para->p_boot_bin == NULL) || (p_sar->p_sar_para->p_boot_bin->bin_name == NULL)) {
		return AW_INVALID_PARA;
	}

	snprintf(p_sar->load_bin.bin_name, sizeof(p_sar->load_bin.bin_name),
			"%s_%d.bin", p_sar->p_sar_para->p_boot_bin->bin_name, p_sar->dts_info.sar_num);

	p_sar->load_bin.bin_opera_func = p_sar->p_sar_para->p_boot_bin->bin_opera_func;

	return aw_sar_load_bin_comm(p_sar);
}

static int32_t aw_sar_update_fw_para(struct aw_sar *p_sar, const struct aw_sar_load_bin_t *p_bin)
{
	AWLOGD(p_sar->dev, "enter 0");
	if ((p_bin == NULL) || (p_bin->bin_name == NULL)) {
		return AW_INVALID_PARA;
	}
	AWLOGD(p_sar->dev, "enter 1");
	p_sar->load_bin.bin_opera_func = p_bin->bin_opera_func;
	p_sar->load_bin.bin_load_fail_opera_func = p_bin->bin_load_fail_opera;

	snprintf(p_sar->load_bin.bin_name, sizeof(p_sar->load_bin.bin_name),
			"%s_%d.bin", p_bin->bin_name, p_sar->dts_info.sar_num);
			
	AWLOGD(p_sar->dev, "enter 2:%s",p_sar->load_bin.bin_name);
	return AW_OK;
}

int32_t aw_sar_update_fw(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if (aw_sar_update_fw_para(p_sar, p_sar->p_sar_para->p_fw_bin) != AW_OK) {
		return -AW_ERR;
	}

	return aw_sar_load_bin_comm(p_sar);
}

static int32_t aw_sar_node_prox_update_fw(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if (aw_sar_update_fw_para(p_sar, p_sar->p_sar_para->p_prox_fw) != AW_OK) {
		return -AW_ERR;
	}

	return aw_sar_load_bin_comm(p_sar);
}

static int32_t aw_sar_node_reg_update_fw(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if (aw_sar_update_fw_para(p_sar, p_sar->p_sar_para->p_reg_fw)) {
		return -AW_ERR;
	}

	return aw_sar_load_bin_comm(p_sar);
}

static int32_t aw_sar_awrw_data_analysis(struct aw_sar *p_sar,
						const char *buf, uint8_t len)
{
	uint32_t i = 0 ;
	uint8_t data_temp[2] = { 0 };
	uint8_t index = 0;
	uint32_t tranfar_data_temp = 0;
	uint32_t theory_len = len * AW_SAR_AWRW_DATA_WIDTH + AW_SAR_AWRW_OffSET;
	uint32_t actual_len = strlen(buf);

	AWLOGD(p_sar->dev, "enter");

	if (theory_len != actual_len) {
		AWLOGD(p_sar->dev,
			"error theory_len = %d actual_len = %d",
					theory_len, actual_len);
		return -AW_ERR;
	}

	for (i = 0; i < len * AW_SAR_AWRW_DATA_WIDTH;
					i += AW_SAR_AWRW_DATA_WIDTH) {
		data_temp[0] = buf[AW_SAR_AWRW_OffSET + i + AW_DATA_OffSET_2];
		data_temp[1] = buf[AW_SAR_AWRW_OffSET + i + AW_DATA_OffSET_3];

		if (sscanf(data_temp, "%02x", &tranfar_data_temp) == 1) {
			p_sar->awrw_info.p_i2c_tranfar_data[index] =
						(uint8_t)tranfar_data_temp;
			AWLOGD(p_sar->dev, "tranfar_data = 0x%2x",
				p_sar->awrw_info.p_i2c_tranfar_data[index]);
		}
		index++;
	}

	return 0;
}

static int32_t aw_sar_awrw_write(struct aw_sar *p_sar, const char *buf)
{
	int32_t ret = 0;

	ret = aw_sar_awrw_data_analysis(p_sar, buf,
				p_sar->awrw_info.i2c_tranfar_data_len);
	if (ret == 0) {
		aw_sar_i2c_write_seq(p_sar->i2c, p_sar->awrw_info.p_i2c_tranfar_data,
					p_sar->awrw_info.i2c_tranfar_data_len);
	}

	return ret;
}

static int32_t aw_sar_awrw_read(struct aw_sar *p_sar, const char *buf)
{
	int32_t ret = 0;
	uint8_t *p_buf = p_sar->awrw_info.p_i2c_tranfar_data + p_sar->awrw_info.addr_len;
	uint32_t len = (uint16_t)(p_sar->awrw_info.data_len * p_sar->awrw_info.reg_num);

	ret = aw_sar_awrw_data_analysis(p_sar, buf, p_sar->awrw_info.addr_len);
	if (ret == 0) {
		ret = aw_sar_i2c_read_seq(p_sar->i2c,
					p_sar->awrw_info.p_i2c_tranfar_data,
					p_sar->awrw_info.addr_len,
					p_sar->awrw_info.p_i2c_tranfar_data + p_sar->awrw_info.addr_len,
					(uint16_t)(p_sar->awrw_info.data_len * p_sar->awrw_info.reg_num));
		if (ret != AW_OK) {
			memset(p_buf, 0xff, len);
		}
	}

	return ret;
}

static int32_t aw_sar_awrw_get_func(struct aw_sar *p_sar, char *buf)
{
	uint32_t len = 0;
	uint i = 0;

	if (p_sar->awrw_info.p_i2c_tranfar_data == NULL) {
		AWLOGE(p_sar->dev, "p_i2c_tranfar_data is NULL");
		return len;
	}

	if (p_sar->awrw_info.rw_flag == AW_SAR_PACKAGE_RD) {
		for (i = 0; i < p_sar->awrw_info.i2c_tranfar_data_len; i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02x,",
				p_sar->awrw_info.p_i2c_tranfar_data[i]);
		}
	} else {
		for (i = 0; i < (p_sar->awrw_info.data_len) *
						(p_sar->awrw_info.reg_num); i++) {
			len += snprintf(buf + len, PAGE_SIZE - len,
				"0x%02x,",
				p_sar->awrw_info.p_i2c_tranfar_data[p_sar->awrw_info.addr_len + i]);
		}
	}
	snprintf(buf + len - 1, PAGE_SIZE - len, "\n");

	devm_kfree(p_sar->dev, p_sar->awrw_info.p_i2c_tranfar_data);
	p_sar->awrw_info.p_i2c_tranfar_data = NULL;

	return len;
}

//Function: continuous read register interface
static ssize_t aw_sar_awrw_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	ssize_t ret = 0;

	mutex_lock(&aw_sar_lock);
	if ((p_sar->p_sar_para->p_aw_sar_awrw != NULL) &&
		(p_sar->p_sar_para->p_aw_sar_awrw->p_get_awrw_node_fn != NULL)) {
		ret = (ssize_t)p_sar->p_sar_para->p_aw_sar_awrw->p_get_awrw_node_fn(p_sar, buf);
		mutex_unlock(&aw_sar_lock);
		return ret;
	}

	ret = (ssize_t)aw_sar_awrw_get_func(p_sar, buf);

	mutex_unlock(&aw_sar_lock);

	return ret;
}

static int32_t aw_sar_awrw_handle(struct aw_sar *p_sar, const char *buf)
{
	int32_t ret = 0;

	p_sar->awrw_info.i2c_tranfar_data_len =
			p_sar->awrw_info.addr_len +
 			p_sar->awrw_info.data_len *
 			p_sar->awrw_info.reg_num;
	if (p_sar->awrw_info.p_i2c_tranfar_data != NULL) {
		devm_kfree(p_sar->dev, p_sar->awrw_info.p_i2c_tranfar_data);
		p_sar->awrw_info.p_i2c_tranfar_data = NULL;
	}

	p_sar->awrw_info.p_i2c_tranfar_data =
			devm_kzalloc(p_sar->dev,
			p_sar->awrw_info.i2c_tranfar_data_len, GFP_KERNEL);
	if (p_sar->awrw_info.p_i2c_tranfar_data == NULL) {
		AWLOGE(p_sar->dev, "devm_kzalloc error");
		return -AW_ERR;
	}

	if (p_sar->awrw_info.rw_flag == AW_SAR_I2C_WR) {
		ret = aw_sar_awrw_write(p_sar, buf);
		if (ret != 0)
			AWLOGE(p_sar->dev, "awrw_write error");
		if (p_sar->awrw_info.p_i2c_tranfar_data != NULL) {
			devm_kfree(p_sar->dev,
					p_sar->awrw_info.p_i2c_tranfar_data);
			p_sar->awrw_info.p_i2c_tranfar_data = NULL;
		}
	} else if (p_sar->awrw_info.rw_flag == AW_SAR_I2C_RD) {
		ret = aw_sar_awrw_read(p_sar, buf);
		if (ret != 0)
			AWLOGE(p_sar->dev, "awrw_read error");
	} else {
		return -AW_ERR;
	}

	return AW_OK;
}

static int32_t aw_sar_awrw_set_func(struct aw_sar *p_sar, const char *buf)
{
	uint32_t rw_flag = 0;
	uint32_t addr_bytes = 0;
	uint32_t data_bytes =  0;
	uint32_t package_nums = 0;
	uint32_t reg_num = 0;
	uint32_t i = 0;
	uint32_t j = 0;
	uint32_t addr_tmp = 0;
	uint32_t buf_index0 = 0;
	uint32_t buf_index1 = 0;
	uint32_t r_buf_len = 0;
	uint32_t tr_offset = 0;
	uint8_t addr[4] = { 0 };
	uint32_t theory_len = 0;
	uint32_t actual_len = 0;

	//step1: Parse frame header
	if (sscanf(buf, "0x%02x 0x%02x 0x%02x ", &rw_flag, &addr_bytes, &data_bytes) != 3) {
		AWLOGE(p_sar->dev, "sscanf0 parse error!");
		return -AW_ERR;
	}
	p_sar->awrw_info.rw_flag = (uint8_t)rw_flag;
	p_sar->awrw_info.addr_len = (uint8_t)addr_bytes;
	p_sar->awrw_info.data_len = (uint8_t)data_bytes;

	if (addr_bytes > 4) {
		return -AW_ERR;
		AWLOGE(p_sar->dev, "para error!");
	}

	if ((rw_flag == AW_SAR_I2C_WR) || (rw_flag == AW_SAR_I2C_RD)) {
		if (sscanf(buf + AW_SAR_OFFSET_LEN, "0x%02x ", &reg_num) != 1) {
			AWLOGE(p_sar->dev, "sscanf1 parse error!");
			return -AW_ERR;
		}
		p_sar->awrw_info.reg_num = (uint8_t)reg_num;
		aw_sar_awrw_handle(p_sar, buf);
	} else if (rw_flag == AW_SAR_PACKAGE_RD){
		//step2: Get number of packages
		if (sscanf(buf + AW_SAR_OFFSET_LEN, "0x%02x ", &package_nums) != 1) {
			AWLOGE(p_sar->dev, "sscanf2 parse error!");
			return -AW_ERR;
		}
		theory_len = AW_SAR_OFFSET_LEN + AW_SAR_AWRW_DATA_WIDTH +
				package_nums * (AW_SAR_AWRW_DATA_WIDTH + AW_SAR_AWRW_DATA_WIDTH * addr_bytes);
		actual_len = strlen(buf);
	//	AWLOGD(p_sar->dev, "theory_len:%d, actual_len:%d", theory_len, actual_len);
		if (theory_len != actual_len) {

			AWLOGE(p_sar->dev, "theory_len:%d, actual_len:%d error!", theory_len, actual_len);
			return -AW_ERR;
		}

		//step3: Get the size of read data and apply for space
	//	AWLOGE(p_sar->dev, "package_nums:%d", package_nums);
		for (i = 0; i < package_nums; i++) {
			buf_index0 = AW_SAR_OFFSET_LEN + AW_SAR_AWRW_DATA_WIDTH +
					(AW_SAR_AWRW_DATA_WIDTH * addr_bytes + AW_SAR_AWRW_DATA_WIDTH) * i;
			if (sscanf(buf + buf_index0, "0x%02x", &reg_num) != 1) {
				AWLOGE(p_sar->dev, "sscanf3 parse error!");
				return -AW_ERR;
			}
			//AWLOGE(p_sar->dev, "reg_num:%d", reg_num);
			r_buf_len += reg_num * data_bytes;
		}

	//	AWLOGD(p_sar->dev, "r_buf_len:%d", r_buf_len);
		p_sar->awrw_info.i2c_tranfar_data_len = r_buf_len;

		if (p_sar->awrw_info.p_i2c_tranfar_data != NULL) {
			devm_kfree(p_sar->dev, p_sar->awrw_info.p_i2c_tranfar_data);
			p_sar->awrw_info.p_i2c_tranfar_data = NULL;
		}
		p_sar->awrw_info.p_i2c_tranfar_data = devm_kzalloc(p_sar->dev,
						r_buf_len, GFP_KERNEL);
		if (p_sar->awrw_info.p_i2c_tranfar_data == NULL) {
			AWLOGE(p_sar->dev, "devm_kzalloc error");
			return -AW_ERR;
		}

		//step3: Resolve register address and read data in packets
		for (i = 0; i < package_nums; i++) {
			buf_index0 = AW_SAR_OFFSET_LEN + AW_SAR_AWRW_DATA_WIDTH +
				(AW_SAR_AWRW_DATA_WIDTH * addr_bytes + AW_SAR_AWRW_DATA_WIDTH) * i;
			if (sscanf(buf + buf_index0, "0x%02x", &reg_num) != 1) {
				AWLOGE(p_sar->dev, "sscanf4 parse error!");
				return -AW_ERR;
			}

			for (j = 0; j < addr_bytes; j += 1) {
				buf_index1 = buf_index0 + AW_SAR_AWRW_DATA_WIDTH + (j * AW_SAR_AWRW_DATA_WIDTH);
			//	AWLOGD(p_sar->dev, "buf_index1 = %d", buf_index1);
				if (sscanf(buf + buf_index1, "0x%02x", &addr_tmp) == 1) {
					addr[j] = (uint8_t)addr_tmp;
					//AWLOGD(p_sar->dev, "tranfar_data = 0x%2x", addr[j]);
				} else {
					AWLOGE(p_sar->dev, "sscanf5 parse error!");
					return -AW_ERR;
				}
			}
		//	AWLOGD(p_sar->dev, "tr_offset = %d", tr_offset);
			aw_sar_i2c_read_seq(p_sar->i2c,
					addr,
					addr_bytes,
					p_sar->awrw_info.p_i2c_tranfar_data + tr_offset,
					(uint16_t)(data_bytes * reg_num));
			tr_offset += data_bytes * reg_num;
		}
	}

	return AW_OK;
}

//Function: continuous write register interface
static ssize_t aw_sar_awrw_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	mutex_lock(&aw_sar_lock);

	if ((p_sar->p_sar_para->p_aw_sar_awrw != NULL) &&
		(p_sar->p_sar_para->p_aw_sar_awrw->p_set_awrw_node_fn != NULL)) {
		p_sar->p_sar_para->p_aw_sar_awrw->p_set_awrw_node_fn (p_sar, buf, count);
		mutex_unlock(&aw_sar_lock);
		return count;
	}

	aw_sar_awrw_set_func(p_sar, buf);

	mutex_unlock(&aw_sar_lock);

	return count;
}
//Print all readable register values
static ssize_t aw_sar_reg_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	ssize_t len = 0;
	uint16_t i = 0;
	uint32_t reg_data = 0;
	int32_t ret = 0;
	uint8_t reg_rd_access = 0;

	if (p_sar->p_sar_para->p_reg_list == NULL) {
		return len;
	}

	reg_rd_access = p_sar->p_sar_para->p_reg_list->reg_rd_access;

	for (i = 0; i < p_sar->p_sar_para->p_reg_list->reg_num; i++) {
		if (p_sar->p_sar_para->p_reg_list->reg_perm[i].rw & reg_rd_access) {
			ret = aw_sar_i2c_read(p_sar->i2c, p_sar->p_sar_para->p_reg_list->reg_perm[i].reg, &reg_data);
			if (ret < 0) {
				len += snprintf(buf + len, PAGE_SIZE - len,
						"i2c read error ret = %d\n", ret);
			}
			len+= snprintf(buf + len, PAGE_SIZE - len,
						"%x,%x\n",
						p_sar->p_sar_para->p_reg_list->reg_perm[i].reg,
						reg_data);
		}
	}
	AWLOGD(p_sar->dev, "len %d", (int)len);

	return len;
}

//Write register interface with write permission
static ssize_t aw_sar_reg_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	uint16_t i = 0;
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	uint32_t databuf[2] = { 0, 0 };
	uint8_t reg_wd_access = 0;

	if (p_sar->p_sar_para->p_reg_list == NULL) {
		AWLOGE(p_sar->dev, "AW_INVALID_PARA");
		return count;
	}

	reg_wd_access = p_sar->p_sar_para->p_reg_list->reg_wd_access;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) != 2)
		return count;

	for (i = 0; i < p_sar->p_sar_para->p_reg_list->reg_num; i++) {
		if ((uint16_t)databuf[0] == p_sar->p_sar_para->p_reg_list->reg_perm[i].reg) {
			if (p_sar->p_sar_para->p_reg_list->reg_perm[i].rw & reg_wd_access) {
				aw_sar_i2c_write(p_sar->i2c,
					(uint16_t)databuf[0], (uint32_t)databuf[1]);
			}
			break;
		}
	}

	return count;
}

//set chip Soft reset
static ssize_t aw_sar_soft_rst_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	uint32_t flag = 0;
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &flag) != 1) {
		AWLOGE(p_sar->dev, "sscanf parse err");
		return count;
	}

	if (flag == AW_TRUE) {
		aw_sar_soft_reset(p_sar);
	}

	return count;
}

static int32_t aw_sar_aot(struct aw_sar *p_sar)
{
	if (p_sar->p_sar_para->p_aot == NULL) {
		return AW_INVALID_PARA;
	}

	if (p_sar->p_sar_para->p_aot->p_set_aot_node_fn != NULL) {
		return p_sar->p_sar_para->p_aot->p_set_aot_node_fn(p_sar);
	}

	return aw_sar_i2c_write_bits(p_sar->i2c, p_sar->p_sar_para->p_aot->aot_reg,
					p_sar->p_sar_para->p_aot->aot_mask, p_sar->p_sar_para->p_aot->aot_flag);
}

//Perform Auto-Offset-Tuning (AOT)
static ssize_t aw_sar_aot_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	uint32_t cali_flag = 0;

	if (sscanf(buf, "%d", &cali_flag) != 1)
		return count;

	if (cali_flag == AW_TRUE) {
		aw_sar_aot(p_sar);
	} else {
		AWLOGE(p_sar->dev, "fail to set aot cali");
	}

	return count;
}

//update Register configuration and set the chip active mode
int32_t aw_sar_update_reg_set_func(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	aw_sar_load_def_reg_bin(p_sar);
	aw_sar_mode_set(p_sar, p_sar->p_sar_para->p_chip_mode->active);

	return AW_OK;
}

//Update register configuration
static ssize_t aw_sar_update_reg_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	uint32_t flag = 0;
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &flag) != 1) {
		AWLOGE(p_sar->dev, "sscanf parse error");
		return count;
	}

	if (flag == AW_TRUE) {
		mutex_lock(&aw_sar_lock);
		aw_sar_soft_reset(p_sar);
		aw_sar_update_reg_set_func(p_sar);
		mutex_unlock(&aw_sar_lock);
	}

	return count;
}

//get chip diff val
static ssize_t aw_sar_get_diff(struct aw_sar *p_sar, char *buf)
{
	uint32_t i = 0;
	int32_t ret = 0;
	ssize_t len = 0;
	uint32_t data = 0;
	int32_t diff_val = 0;
	const struct aw_sar_diff_t *diff = p_sar->p_sar_para->p_diff;

	if (p_sar->p_sar_para->p_diff == NULL) {
		return AW_INVALID_PARA;
	}

	//If a private interface is defined, the private interface is used
	if (p_sar->p_sar_para->p_diff->p_get_diff_node_fn != NULL) {
		return p_sar->p_sar_para->p_diff->p_get_diff_node_fn(p_sar, buf);
	}

	for (i = 0; i < p_sar->p_sar_para->ch_num_max; i++) {
		ret = aw_sar_i2c_read(p_sar->i2c, diff->diff0_reg + i * diff->diff_step, &data);
		if (ret != AW_OK) {
			AWLOGE(p_sar->dev, "read diff err: %d", ret);
			return -AW_ERR;
		}
		diff_val = (int32_t)data / (int32_t)diff->rm_float;
		len += snprintf(buf + len, PAGE_SIZE - len, "DIFF_CH%d = %d\n", i, diff_val);
	}

	return len;
}


//Print diff values of all channels of the chip.
static ssize_t aw_sar_diff_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	return aw_sar_get_diff(p_sar, buf);
}

void aw_sar_mode_set(struct aw_sar *p_sar, uint8_t curr_mode)
{
	struct aw_sar_mode_set_t mode_set_para;

	if (p_sar->p_sar_para->p_mode == NULL) {
		return;
	}

	//If a private interface is defined, the private interface is used
	if (p_sar->p_sar_para->p_mode->p_set_mode_node_fn != NULL) {
		p_sar->p_sar_para->p_mode->p_set_mode_node_fn(p_sar, curr_mode);
		return;
	}

	mode_set_para.chip_id = p_sar->chip_type;
	mode_set_para.chip_mode.curr_mode = curr_mode;
	mode_set_para.chip_mode.last_mode = p_sar->last_mode;

	aw_sar_mode_set_func(p_sar->i2c, p_sar->irq_init.to_irq, &mode_set_para,
		p_sar->p_sar_para->p_mode->mode_set_arr, p_sar->p_sar_para->p_mode->mode_set_arr_len);
	p_sar->last_mode = curr_mode;
}

//Set the chip to enter different modes
static ssize_t aw_sar_mode_operation_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	uint32_t mode = 0;

	if (sscanf(buf, "%d", &mode) != 1) {
		AWLOGE(p_sar->dev, "sscanf parse err");
		return count;
	}
	aw_sar_mode_set(p_sar, mode);

	return count;
}

//Get the current mode of the chip
static ssize_t aw_sar_mode_operation_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	if (p_sar->p_sar_para->p_mode == NULL) {
		return len;
	}

	if (p_sar->p_sar_para->p_mode->p_get_mode_node_fn != NULL) {
		len = p_sar->p_sar_para->p_mode->p_get_mode_node_fn(p_sar, buf);
	}

	return len;
}

//Print information related information
static ssize_t aw_sar_chip_info_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "driver version: %s\n", AW_SAR_DRIVER_VERSION);
	len += snprintf(buf + len, PAGE_SIZE - len, "reg_load_state: %d\n", p_sar->ret_val);

	if ((p_sar->p_sar_para->p_get_chip_info != NULL) &&
		(p_sar->p_sar_para->p_get_chip_info->p_get_chip_info_node_fn != NULL)) {
		p_sar->p_sar_para->p_get_chip_info->p_get_chip_info_node_fn(p_sar, buf, &len);
	}

	return len;
}

static ssize_t aw_sar_offset_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	ssize_t len = 0;

	if ((p_sar->p_sar_para->p_offset != NULL) &&
		(p_sar->p_sar_para->p_offset->p_get_offset_node_fn != NULL)) {
		len = (ssize_t)p_sar->p_sar_para->p_offset->p_get_offset_node_fn(p_sar, buf);
	}

	return len;
}

static DEVICE_ATTR(awrw, 0664, aw_sar_awrw_get, aw_sar_awrw_set);
static DEVICE_ATTR(reg, 0664, aw_sar_reg_get, aw_sar_reg_set);
static DEVICE_ATTR(soft_rst, 0664, NULL, aw_sar_soft_rst_set);
static DEVICE_ATTR(aot, 0664, NULL, aw_sar_aot_set);
static DEVICE_ATTR(update_reg, 0664, NULL, aw_sar_update_reg_set);
static DEVICE_ATTR(diff, 0664, aw_sar_diff_get, NULL);
static DEVICE_ATTR(mode_operation, 0664, aw_sar_mode_operation_get, aw_sar_mode_operation_set);
static DEVICE_ATTR(chip_info, 0664, aw_sar_chip_info_get, NULL);
static DEVICE_ATTR(offset, 0664, aw_sar_offset_get, NULL);

static struct attribute *aw_sar_attributes[] = {
	&dev_attr_awrw.attr,
	&dev_attr_reg.attr,
	&dev_attr_soft_rst.attr,
	&dev_attr_aot.attr,
	&dev_attr_update_reg.attr,
	&dev_attr_diff.attr,
	&dev_attr_mode_operation.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_offset.attr,
	NULL
};

const static struct attribute_group aw_sar_attribute_group = {
	.attrs = aw_sar_attributes,
};

//firmware upgrade through write register mode, and the operation is supported by flash chip
static ssize_t aw_sar_prot_update_fw_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);


	mutex_lock(&aw_sar_lock);
	aw_sar_disable_irq(p_sar);

	AWLOGD(p_sar->dev, "enter");
	p_sar->prot_update_state = aw_sar_node_prox_update_fw(p_sar);

	aw_sar_enable_irq(p_sar);
	mutex_unlock(&aw_sar_lock);

	return count;
}

//firmware upgrade throughr Write register mode,and the operation is supported by flash chip
static ssize_t aw_sar_reg_update_fw_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	mutex_lock(&aw_sar_lock);
	aw_sar_disable_irq(p_sar);

	AWLOGD(p_sar->dev, "enter");
	aw_sar_node_reg_update_fw(p_sar);

	aw_sar_enable_irq(p_sar);
	mutex_unlock(&aw_sar_lock);

	return count;
}

//boot upgrade throughr Write register mode,and the operation is supported by flash chip
static ssize_t aw_sar_reg_update_boot_set(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);

	if (p_sar->p_sar_para->p_boot_bin == NULL) {
		return count;
	}

	mutex_lock(&aw_sar_lock);
	aw_sar_disable_irq(p_sar);

	AWLOGD(p_sar->dev, "enter");

	aw_sar_reg_update_boot_work(p_sar);

	aw_sar_enable_irq(p_sar);
	mutex_unlock(&aw_sar_lock);

	return count;
}

//Print the protocol upgrade status. 0 is success, Not 0 is failure
//Print the current firmware version number
static ssize_t aw_sar_prot_update_fw_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw_sar *p_sar = dev_get_drvdata(dev);
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
		"protocol update state:%s!\n",
		(p_sar->prot_update_state == 0) ? "ok" : "error");
	if ((p_sar->p_sar_para->p_prox_fw != NULL) &&
		(p_sar->p_sar_para->p_prox_fw->p_get_prot_update_fw_node_fn != NULL)) {
		p_sar->p_sar_para->p_prox_fw->p_get_prot_update_fw_node_fn(p_sar, buf, &len);
	}

	return len;
}

static DEVICE_ATTR(prot_update_fw, 0664, aw_sar_prot_update_fw_get, aw_sar_prot_update_fw_set);
static DEVICE_ATTR(reg_update_fw, 0664, NULL, aw_sar_reg_update_fw_set);
static DEVICE_ATTR(reg_update_boot, 0664, NULL, aw_sar_reg_update_boot_set);

static struct attribute *aw_sar_update_attributes[] = {
	&dev_attr_prot_update_fw.attr,
	&dev_attr_reg_update_fw.attr,
	&dev_attr_reg_update_boot.attr,
	NULL
};

const static struct attribute_group aw_sar_update_attribute_group = {
	.attrs = aw_sar_update_attributes,
};

static void aw_sar_update_work(struct work_struct *work)
{
	int32_t ret = 0;
	struct aw_sar *p_sar = container_of(work, struct aw_sar, update_work.work);

	mutex_lock(&aw_sar_lock);

	if (p_sar->dts_info.update_fw_flag == true) {
		AWLOGI(p_sar->dev, "firmware upgrade start!");
		ret = aw_sar_update_fw(p_sar);
		if (ret != AW_OK) {
			AWLOGE(p_sar->dev, "protocol upgrade firmware error!");
			p_sar->ret_val = AW_PROT_UPDATE_ERR;
		} else {
			AWLOGI(p_sar->dev, "protocol upgrade firmware ok!");
		}
	}

	//2.Parse the bin file and load the register configuration
	ret = aw_sar_load_def_reg_bin(p_sar);
	if (ret != AW_OK) {
		p_sar->ret_val = AW_REG_LOAD_ERR;
		AWLOGE(p_sar->dev, "reg_bin load err!");
		aw_sar_para_loaded(p_sar);
	}

	//3.active chip
	aw_sar_mode_set(p_sar, p_sar->p_sar_para->p_chip_mode->init_mode);
	if (p_sar->irq_init.host_irq_stat == IRQ_DISABLE) {
		enable_irq(p_sar->irq_init.to_irq);
		p_sar->irq_init.host_irq_stat = IRQ_ENABLE;
	}

	mutex_unlock(&aw_sar_lock);

	return;
}

static void aw_sar_update(struct aw_sar *p_sar)
{
	AWLOGD(p_sar->dev, "enter");

	if (p_sar->p_sar_para->p_reg_bin == NULL) {
		return;
	}

	if (p_sar->p_sar_para->p_reg_bin->p_update_fn != NULL) {
		p_sar->p_sar_para->p_reg_bin->p_update_fn(p_sar);
	}

	INIT_DELAYED_WORK(&p_sar->update_work, aw_sar_update_work);
	schedule_delayed_work(&p_sar->update_work,
					msecs_to_jiffies(AW_POWER_ON_SYSFS_DELAY_MS));
}

static int32_t aw_sar_create_node(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	AWLOGD(p_sar->dev, "enter");

	i2c_set_clientdata(p_sar->i2c, p_sar);

	ret = sysfs_create_group(&p_sar->i2c->dev.kobj, &aw_sar_attribute_group);

	if (p_sar->dts_info.update_fw_flag == true) {
		ret |= sysfs_create_group(&p_sar->i2c->dev.kobj, &aw_sar_update_attribute_group);
	}

	//Special requirements of SAR chip
	if (p_sar->p_sar_para->p_platform_config->p_add_node_create_fn != NULL) {
		ret |= p_sar->p_sar_para->p_platform_config->p_add_node_create_fn(p_sar);
	}

	return ret;
}

static void aw_sar_node_free(struct aw_sar *p_sar)
{
	sysfs_remove_group(&p_sar->i2c->dev.kobj, &aw_sar_attribute_group);

	if (p_sar->dts_info.update_fw_flag == true) {
		sysfs_remove_group(&p_sar->i2c->dev.kobj, &aw_sar_update_attribute_group);
	}
	//Special requirements of SAR chip
	if ((p_sar->p_sar_para->p_platform_config != NULL) &&
		(p_sar->p_sar_para->p_platform_config->p_add_node_free_fn != NULL)) {
		p_sar->p_sar_para->p_platform_config->p_add_node_free_fn(p_sar);
	}
}

//The interrupt pin is set to internal pull-up start
void aw_sar_int_output(struct aw_sar *p_sar, int32_t level)
{
	pr_info("%s enter aw_sar int level:%d\n", __func__, level);
	if (level == 0) {
		if (p_sar->pinctrl.pinctrl) {
			pinctrl_select_state(p_sar->pinctrl.pinctrl,
						p_sar->pinctrl.int_out_low);
		} else {
			pr_info("%s Failed set int pin output low\n", __func__);
		}
	} else if (level == 1) {
		if (p_sar->pinctrl.pinctrl) {
			pinctrl_select_state(p_sar->pinctrl.pinctrl,
						p_sar->pinctrl.int_out_high);
		} else {
			pr_info("%s Failed set int pin output high\n", __func__);
		}
	}
}

static int32_t aw_sar_pinctrl_init(struct aw_sar *p_sar)
{
	struct aw_sar_pinctrl *pinctrl = &p_sar->pinctrl;
	uint8_t pin_default_name[50] = { 0 };
	uint8_t pin_output_low_name[50] = { 0 };
	uint8_t pin_output_high_name[50] = { 0 };

	AWLOGD(p_sar->dev, "enter");

	pinctrl->pinctrl = devm_pinctrl_get(p_sar->dev);
	if (IS_ERR_OR_NULL(pinctrl->pinctrl)) {
		pr_info("%s:No pinctrl found\n", __func__);
		pinctrl->pinctrl = NULL;
		return -EINVAL;
	}

	snprintf(pin_default_name, sizeof(pin_default_name),
					"aw_default_sar%d", p_sar->dts_info.sar_num);
	AWLOGD(p_sar->dev, "pin_default_name = %s", pin_default_name);
	pinctrl->default_sta = pinctrl_lookup_state(pinctrl->pinctrl,
							pin_default_name);
	if (IS_ERR_OR_NULL(pinctrl->default_sta)) {
		AWLOGE(p_sar->dev, "Failed get pinctrl state:default state");
		goto exit_pinctrl_init;
	}

	snprintf(pin_output_high_name, sizeof(pin_output_high_name),
				"aw_int_output_high_sar%d", p_sar->dts_info.sar_num);
	AWLOGD(p_sar->dev, "pin_output_high_name = %s", pin_output_high_name);
	pinctrl->int_out_high = pinctrl_lookup_state(pinctrl->pinctrl,
							pin_output_high_name);
	if (IS_ERR_OR_NULL(pinctrl->int_out_high)) {
		AWLOGE(p_sar->dev, "Failed get pinctrl state:output_high");
		goto exit_pinctrl_init;
	}

	snprintf(pin_output_low_name, sizeof(pin_output_low_name),
				"aw_int_output_low_sar%d", p_sar->dts_info.sar_num);
	AWLOGD(p_sar->dev, "pin_output_low_name = %s", pin_output_low_name);
	pinctrl->int_out_low = pinctrl_lookup_state(pinctrl->pinctrl,
							pin_output_low_name);
	if (IS_ERR_OR_NULL(pinctrl->int_out_low)) {
		AWLOGE(p_sar->dev, "Failed get pinctrl state:output_low");
		goto exit_pinctrl_init;
	}

	AWLOGD(p_sar->dev, "Success init pinctrl");

	return 0;

exit_pinctrl_init:
	devm_pinctrl_put(pinctrl->pinctrl);
	pinctrl->pinctrl = NULL;

	return -EINVAL;
}

static void aw_sar_pinctrl_deinit(struct aw_sar *p_sar)
{
	if (p_sar->pinctrl.pinctrl) {
		devm_pinctrl_put(p_sar->pinctrl.pinctrl);
	}
}
//The interrupt pin is set to internal pull-up end

//AW_SAR_REGULATOR_POWER_ON start
static int32_t aw_sar_regulator_power_init(struct aw_sar *p_sar)
{
	int32_t rc = 0;
	uint8_t vcc_name[20] = { 0 };

	AWLOGD(p_sar->dev, "aw_sar power init enter");

	if (p_sar->p_sar_para->p_platform_config->p_regulator_config == NULL) {
		return AW_INVALID_PARA;
	}

	snprintf(vcc_name, sizeof(vcc_name), "%s%d",
						p_sar->p_sar_para->p_platform_config->p_regulator_config->vcc_name,
						p_sar->dts_info.sar_num);
	AWLOGD(p_sar->dev, "vcc_name = %s", vcc_name);

	p_sar->vcc = regulator_get(p_sar->dev, vcc_name);
	if (IS_ERR(p_sar->vcc)) {
		rc = PTR_ERR(p_sar->vcc);
		AWLOGE(p_sar->dev, "regulator get failed vcc rc = %d", rc);
		return rc;
	}

	if (regulator_count_voltages(p_sar->vcc) > 0) {
		rc = regulator_set_voltage(p_sar->vcc,
					p_sar->p_sar_para->p_platform_config->p_regulator_config->min_uV,
					p_sar->p_sar_para->p_platform_config->p_regulator_config->max_uV);
		if (rc) {
			AWLOGE(p_sar->dev,
				"regulator set vol failed rc = %d", rc);
			goto reg_vcc_put;
		}
	}

	return rc;

reg_vcc_put:
	regulator_put(p_sar->vcc);
	return rc;
}

static void aw_sar_power_deinit(struct aw_sar *p_sar)
{
	if (p_sar->power_enable) {
		//Turn off the power output. However,
		//it may not be turned off immediately
		//There are scenes where power sharing can exist
		regulator_disable(p_sar->vcc);
		regulator_put(p_sar->vcc);
	}
}

static void aw_sar_power_enable(struct aw_sar *p_sar, bool on)
{
	int32_t rc = 0;

	AWLOGD(p_sar->dev, "aw_sar power enable enter");

	if (on) {
		rc = regulator_enable(p_sar->vcc);
		if (rc) {
			AWLOGE(p_sar->dev,
				"regulator_enable vol failed rc = %d", rc);
		} else {
			p_sar->power_enable = AW_TRUE;
			msleep(20);
		}
	} else {
		rc = regulator_disable(p_sar->vcc);
		if (rc) {
			AWLOGE(p_sar->dev,
				"regulator_disable vol failed rc = %d", rc);
		} else {
			p_sar->power_enable = AW_FALSE;
		}
	}
}

static int32_t regulator_is_get_voltage(struct aw_sar *p_sar)
{
	uint32_t cnt = 10;
	int32_t voltage_val = 0;

	AWLOGD(p_sar->dev, "enter");

	while(cnt--) {
		voltage_val = regulator_get_voltage(p_sar->vcc);
		AWLOGD(p_sar->dev, "aw_sar voltage is : %d uv", voltage_val);
		if (voltage_val >= p_sar->p_sar_para->p_platform_config->p_regulator_config->min_uV) {
			return AW_OK;
		}
		mdelay(1);
	}
	//Ensure that the chip initialization is completed
	msleep(20);

	return -AW_VERS_ERR;
}
//AW_SAR_REGULATOR_POWER_ON end

static void aw_sar_init_lock(struct aw_sar *p_sar)
{
	//Initialize lock, To protect the thread safety of updating bin file
	mutex_init(&aw_sar_lock);
	//Required for mode setting
	p_sar->last_mode = p_sar->p_sar_para->p_chip_mode->pre_init_mode;
	p_sar->fw_fail_flag = AW_FALSE;
	p_sar->ret_val = AW_OK;
}

static int32_t aw_sar_platform_rsc_init(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	if (p_sar->p_sar_para->p_platform_config == NULL) {
		return AW_INVALID_PARA;
	}

	//step 1.parsing dts data
	ret = aw_sar_parse_dts(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "parse dts error!");
		return ret;
	}

	//Initialization lock and some variables
	aw_sar_init_lock(p_sar);

	//The interrupt pin is set to internal pull-up and configured by DTS
	if (p_sar->dts_info.use_inter_pull_up == true) {
		ret = aw_sar_pinctrl_init(p_sar);
		if (ret < 0) {
			/* if define pinctrl must define the following state
			 * to let int-pin work normally: default, int_output_high,
			 * int_output_low, int_input
			 */
			AWLOGE(p_sar->dev, "Failed get wanted pinctrl state");
			goto err_pinctrl;
		}
		aw_sar_int_output(p_sar, 1);
	}

	//step 2.Create debug file node
	ret = aw_sar_create_node(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "create node error!");
		goto free_sysfs_nodes;
	}

	//step 3.Initialization interrupt
	ret = aw_sar_irq_init(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "interrupt initialization error!");
		goto free_irq;
	}

	//step 4.Initialization input Subsystem
	ret = aw_sar_input_init(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "input_init error!");
		goto free_input;
	}

	return AW_OK;

free_input:
	aw_sar_input_free(p_sar);
free_irq:
	aw_sar_irq_free(p_sar);
free_sysfs_nodes:
	aw_sar_node_free(p_sar);
err_pinctrl:
if (p_sar->dts_info.use_inter_pull_up == true) {
	aw_sar_pinctrl_deinit(p_sar);
}

	return -AW_ERR;
}

/**
 * @brief sar sensor initialization logic.
 *
 * @param p_sar  data stored in type 'struct aw_sar *'.
 * @return AW_OK 0 if init successed. others if unpack error.
 */
int32_t aw_sar_chip_init(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	//step 1.check chipid,Verify whether the chip communication is successful
	ret = aw_sar_check_chipid(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "check_chipid error!");
		goto communication_fail;
	}

	//step 2.Check chip initialization completed,
	ret = aw_sar_soft_reset(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "soft_reset error!");
		goto communication_fail;
	}

	ret = aw_sar_check_init_over_irq(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "check_init_over_irqt error!");
		goto communication_fail;
	}

	//step 3.chip other operation
	ret = aw_sar_chip_other_operation(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "chip_other_operation error!");
		goto free_other_opera;
	}

	//step 4.Parse the bin file, upgrade the firmware, and load the register prize
	aw_sar_update(p_sar);

	AWLOGD(p_sar->dev, "init success!");

	return AW_OK;
free_other_opera:
	aw_sar_chip_other_operation_free(p_sar);
communication_fail:

	return -AW_ERR;
}

static int32_t aw_sar_init(struct aw_sar *p_sar)
{
	int32_t ret = 0;

	//step 1: Platform resource initialization
	ret = aw_sar_platform_rsc_init(p_sar);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "platform_rsc_init error!");
		return -AW_ERR;
	}

	//step 2: Chip initialization
	ret = aw_sar_chip_init(p_sar);
	if (ret != AW_OK) {
		aw_sar_input_free(p_sar);
		aw_sar_irq_free(p_sar);
		aw_sar_node_free(p_sar);
		if (p_sar->dts_info.use_inter_pull_up == true) {
			aw_sar_pinctrl_deinit(p_sar);
		}
		return -AW_ERR;
	}

	return AW_OK;
}

static int32_t aw_sar_regulator_power(struct aw_sar *p_sar)
{
	struct aw_sar_dts_info *p_dts_info = &p_sar->dts_info;
	int32_t ret = 0;

	p_dts_info->use_regulator_flag = of_property_read_bool(p_sar->i2c->dev.of_node, "aw_sar,regulator-power-supply");
	AWLOGI(p_sar->dev, "regulator-power-supply = <%d>", p_dts_info->use_regulator_flag);

	//Configure the use of regulator power supply in DTS
	if (p_sar->dts_info.use_regulator_flag == true) {
		ret = aw_sar_regulator_power_init(p_sar);
		if (ret) {
			AWLOGE(p_sar->dev, "power init failed");
		} else {
			aw_sar_power_enable(p_sar, AW_TRUE);
		}
		ret = regulator_is_get_voltage(p_sar);
		if (ret != AW_OK) {
			AWLOGE(p_sar->dev, "get_voltage failed");
			aw_sar_power_deinit(p_sar);
		}
	}

	return ret;
}

/**
 * @brief Drive logic entry
 *
 */
static int32_t aw_sar_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	int32_t ret = 0;
	struct aw_sar *p_sar = NULL;
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	char temp[256] = {0};
	ssize_t len = PAGE_SIZE;
#endif
	pr_info("%s enter", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("check_functionality failed!\n");
		return -EIO;
	}

	p_sar = devm_kzalloc(&i2c->dev, sizeof(struct aw_sar), GFP_KERNEL);
	if (p_sar == NULL) {
		pr_err("p_sar failed to malloc memory!");
		ret = -AW_ERR;
		goto err_malloc;
	}

	p_sar->dev = &i2c->dev;
	p_sar->i2c = i2c;
	i2c_set_clientdata(i2c, p_sar);

	//1.Judge whether to use regular power supply. If yes, supply power
	ret = aw_sar_regulator_power(p_sar);
	if (ret != AW_OK) {
		AWLOGE(&i2c->dev, "regulator_power error!");
		goto err_get_voltage;
	}

	//2.Get chip initialization resources
	ret = aw_sar_get_chip_info(p_sar);
	if (ret != AW_OK) {
		AWLOGE(&i2c->dev, "chip_init error!");
		goto err_chip_init;
	}

	//3.Chip initialization process
	ret = aw_sar_init(p_sar);
	if (ret != AW_OK) {
		AWLOGE(&i2c->dev, "chip_init error!");
		goto err_chip_init;
	}
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	if ((p_sar->p_sar_para->p_get_chip_info != NULL) &&
		(p_sar->p_sar_para->p_get_chip_info->p_get_chip_info_node_fn != NULL)) {
		p_sar->p_sar_para->p_get_chip_info->p_get_chip_info_node_fn(p_sar, temp, &len);
	}
#endif
	AWLOGD(&i2c->dev, "probe success!");

	return 0;
err_chip_init:
	aw_sar_sensor_free(p_sar);
err_get_voltage:
if (p_sar->dts_info.use_regulator_flag == true) {
	aw_sar_power_deinit(p_sar);
}

err_malloc:
	return ret;
}

static int32_t aw_sar_i2c_remove(struct i2c_client *i2c)
{
	struct aw_sar *p_sar  = i2c_get_clientdata(i2c);

	aw_sar_chip_other_operation_free(p_sar);

	aw_sar_node_free(p_sar);

	aw_sar_irq_free(p_sar);

	aw_sar_input_free(p_sar);

	if (p_sar->dts_info.use_inter_pull_up == true) {
		aw_sar_pinctrl_deinit(p_sar);
	}
	if (p_sar->dts_info.use_regulator_flag == true) {
		aw_sar_power_deinit(p_sar);
	}

	aw_sar_sensor_free(p_sar);

	AWLOGI(p_sar->dev, "aw_sar_i2c_remove ok!");

	return 0;
}

static int aw_sar_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw_sar *p_sar = i2c_get_clientdata(client);

	AWLOGI(p_sar->dev, "enter");

	if (p_sar->dts_info.use_pm == true) {
		if ((p_sar->p_sar_para->p_platform_config == NULL) ||
			(p_sar->p_sar_para->p_platform_config->p_pm_chip_mode == NULL)) {
			return 0;
		}
		if (p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_suspend_fn) {
			p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_suspend_fn(p_sar);
			return 0;
		}
		aw_sar_mode_set(p_sar, p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->suspend_set_mode);
	}

	return 0;
}

static int aw_sar_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw_sar *p_sar = i2c_get_clientdata(client);

	AWLOGI(p_sar->dev, "enter");

	if (p_sar->dts_info.use_pm == true) {
		if ((p_sar->p_sar_para->p_platform_config == NULL) ||
			(p_sar->p_sar_para->p_platform_config->p_pm_chip_mode == NULL)) {
			return 0;
		}
		if (p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_resume_fn) {
			p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_resume_fn(p_sar);
			return 0;
		}
		aw_sar_mode_set(p_sar, p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->resume_set_mode);
	}
	return 0;
}

static void aw_sar_i2c_shutdown(struct i2c_client *i2c)
{
	struct aw_sar *p_sar  = i2c_get_clientdata(i2c);

	AWLOGI(p_sar->dev, "enter");

	if ((p_sar->p_sar_para->p_platform_config == NULL) ||
		(p_sar->p_sar_para->p_platform_config->p_pm_chip_mode == NULL)) {
		return;
	}

	if (p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_shutdown_fn) {
		p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->p_shutdown_fn(p_sar);
		return;
	}

	aw_sar_mode_set(p_sar, p_sar->p_sar_para->p_platform_config->p_pm_chip_mode->shutdown_set_mode);
}

static const struct dev_pm_ops aw_sar_pm_ops = {
	.suspend = aw_sar_suspend,
	.resume = aw_sar_resume,
};

static const struct of_device_id aw_sar_dt_match[] = {
	{ .compatible = "awinic,aw_sar" },
	{}
};

static const struct i2c_device_id aw_sar_i2c_id[] = {
	{ AW_SAR_I2C_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, aw_sar_i2c_id);

static struct i2c_driver aw_sar_i2c_driver = {
	.driver = {
		.name = AW_SAR_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw_sar_dt_match),
		.pm = &aw_sar_pm_ops,
	},
	.probe = aw_sar_i2c_probe,
	.remove = aw_sar_i2c_remove,
	.shutdown = aw_sar_i2c_shutdown,
	.id_table = aw_sar_i2c_id,
};

static int32_t __init aw_sar_i2c_init(void)
{
	int32_t ret = 0;

	pr_info("awinic sar driver version %s\n", AW_SAR_DRIVER_VERSION);

	ret = i2c_add_driver(&aw_sar_i2c_driver);
	if (ret) {
		pr_err("fail to add aw_sar device into i2c\n");
		return ret;
	}

	return 0;
}

late_initcall(aw_sar_i2c_init);
static void __exit aw_sar_i2c_exit(void)
{
	i2c_del_driver(&aw_sar_i2c_driver);
}
module_exit(aw_sar_i2c_exit);

/**
 * @brief Distinguish different chips by chip name and obtain relevant chip information
 *
 * @param p_sar Structure to be filled
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
static int32_t aw_sar_get_chip_info(struct aw_sar *p_sar)
{
	uint8_t i = 0;
	int32_t ret = 0;

	AWLOGD(p_sar->dev, "enter");

	for(i = 0; i < AW_SAR_DRIVER_MAX; i++) {
		if (g_aw_sar_driver_list[i].p_who_am_i != NULL) {
			ret = g_aw_sar_driver_list[i].p_who_am_i(p_sar);
			if (ret == AW_OK) {
				p_sar->curr_use_driver_type = g_aw_sar_driver_list[i].driver_type;
				if (g_aw_sar_driver_list[i].p_chip_init == NULL) {
					AWLOGE(p_sar->dev, "drvier:%d p_chip_init is null  error!", i);
					continue;
				}
				g_aw_sar_driver_list[i].p_chip_init(p_sar);
				AWLOGI(p_sar->dev, "current use drvier is :%d", g_aw_sar_driver_list[i].driver_type);
				return AW_OK;
			} else {
				AWLOGI(p_sar->dev, "drvier:%d Not supported", i);
			}
		}
	}

	return -AW_ERR;
}

/**
 * @brief Distinguish different chips by chip name and obtain relevant chip information
 *
 * @param p_sar Associated release resource information，Follow priv_data member
 * @return AW_OK 0 if init successed. -AW_ERR -1 if error.
 */
static void aw_sar_sensor_free(struct aw_sar *p_sar)
{
	if (g_aw_sar_driver_list[p_sar->curr_use_driver_type].p_chip_deinit != NULL) {
		g_aw_sar_driver_list[p_sar->curr_use_driver_type].p_chip_deinit(p_sar);
	}
}

MODULE_DESCRIPTION("AWINIC SAR Driver");

MODULE_LICENSE("GPL v2");
