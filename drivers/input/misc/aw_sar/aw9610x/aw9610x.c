#include "aw9610x.h"
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../../misc/hardware_info/hardware_info.h"
#endif
#include "hf_manager.h"

#define AW9610X_I2C_NAME "aw9610x_sar"
#define AW9610X_DRIVER_VERSION "v2.3.4"

#define AW9610X_RETRIES		(3)

static struct aw_sar *g_aw_sar = NULL;

//#define	AW_INPUT_TRIGGER_TH1
//#define	AW_INPUT_TRIGGER_TH2
//#define	AW_INPUT_TRIGGER_TH3

#ifdef UEVENT_REPORT
char *envp[2];
char envp_buff[128];
#endif

struct sar_sensor_dev {
	struct hf_device hf_dev;
	bool enable;
};

static const struct sensor_info sarsensor_support_list[] = {
	{
		.sensor_type = SENSOR_TYPE_NT_SAR,
		.gain = 1,
		.name = {'n', 't', '_', 's', 'a', 'r'},
		.vendor = {'n','t'},
	},
};

static struct sar_sensor_dev sar_sensor;

static void sar_sensor_report(struct aw_sar *p_sar)
{
	struct hf_manager *manager = sar_sensor.hf_dev.manager;
	struct hf_manager_event event;

	if (!sar_sensor.enable) {
		AWLOGE(g_aw_sar->dev, "sar_sensor_report sar sensor enable is false");
		return;
	}

	if (!manager) {
		AWLOGE(p_sar->dev, "sar_sensor_report hf manager is null");
		return;
	}

	if (!p_sar) {
		AWLOGE(p_sar->dev, "sar_sensor_report p_sar is null");
		return;
	}

	AWLOGE(p_sar->dev, "sar_sensor_report channel_0:%d, channel_3:%d",
		p_sar->channels_arr[0].last_channel_info, p_sar->channels_arr[3].last_channel_info);

	memset(&event, 0, sizeof(struct hf_manager_event));
	event.timestamp = ktime_get_boottime_ns();
	event.sensor_type = SENSOR_TYPE_NT_SAR;
	event.action = DATA_ACTION;
	event.word[0] = p_sar->channels_arr[0].last_channel_info;
	event.word[1] = p_sar->channels_arr[3].last_channel_info;
	manager->report(manager, &event);
	manager->complete(manager);
}

static int sar_sensor_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	AWLOGE(g_aw_sar->dev,"sar_sensor_enable sensor_type:%d en:%d\n", sensor_type, en);
	sar_sensor.enable = en ? true : false;
	return 0;
}

static int sar_sensor_batch(struct hf_device *hfdev, int sensor_type,
		int64_t delay, int64_t latency)
{
	AWLOGE(g_aw_sar->dev,"sar_sensor_batch sensor:%d delay:%lld latency:%lld\n", sensor_type,
			delay, latency);
	return 0;
}

static void sar_sensor_init(void)
{
	struct hf_device *dev = &sar_sensor.hf_dev;
	int ret;

	AWLOGE(g_aw_sar->dev, "sar_sensor_init");

	dev->dev_name = "nt_sar_dev";
	dev->device_poll = HF_DEVICE_IO_POLLING;
	dev->device_bus = HF_DEVICE_IO_ASYNC;
	dev->support_list = (struct sensor_info *)sarsensor_support_list;
	dev->support_size = ARRAY_SIZE(sarsensor_support_list);
	dev->enable = sar_sensor_enable;
	dev->batch = sar_sensor_batch;

	ret = hf_device_register_manager_create(dev);
	AWLOGE(g_aw_sar->dev, "sar_sensor_init register manager ret:%d", ret);
}
static int32_t aw9610x_baseline_filter(struct aw_sar *p_sar)
{
	uint8_t i = 0;
	uint32_t status0 = 0;
	uint32_t status1 = 0;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	aw_sar_i2c_read(p_sar->i2c, REG_STAT1, &status1);
	aw_sar_i2c_read(p_sar->i2c, REG_STAT0, &status0);

	for (i = 0; i < AW9610X_CHANNEL_MAX; i++) {
		if (((status1 >> i) & 0x01) == 1) {
			if (aw9610x->satu_flag[i] == 0) {
				aw_sar_i2c_read(p_sar->i2c, REG_BLFILT_CH0 + i * AW_CL1SPE_DEAL_OS, &aw9610x->satu_data[i]);
				aw_sar_i2c_write(p_sar->i2c, REG_BLFILT_CH0 + i * AW_CL1SPE_DEAL_OS, ((aw9610x->satu_data[i] | 0x1fc) & 0x3fffffff));
				aw9610x->satu_flag[i] = 1;
			}
		} else if (((status1 >> i) & 0x01) == 0) {
			if (aw9610x->satu_flag[i] == 1) {
				if (((status0 >> (i + 24)) & 0x01) == 0) {
					aw_sar_i2c_write(p_sar->i2c, REG_BLFILT_CH0 + i * AW_CL1SPE_DEAL_OS, aw9610x->satu_data[i]);
					aw9610x->satu_flag[i] = 0;
				}
			}
		}
	}

	return AW_OK;
}

static void aw9610x_saturat_release_handle(struct aw_sar *p_sar)
{
	uint32_t satu_irq = 0;
	uint8_t i = 0;
	uint32_t status0 = 0;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	AWLOGD(p_sar->dev, "enter");

	satu_irq = (aw9610x->irq_status >> 7) & 0x01;
	if (satu_irq == 1) {
		aw9610x_baseline_filter(p_sar);
	} else {
		aw_sar_i2c_read(p_sar->i2c, REG_STAT0, &status0);
		for (i = 0; i < AW9610X_CHANNEL_MAX; i++) {
			if (aw9610x->satu_flag[i] == 1) {
				if (((status0 >> (i + 24)) & 0x01) == 0) {
					aw_sar_i2c_write(p_sar->i2c, REG_BLFILT_CH0 + i * AW_CL1SPE_DEAL_OS, aw9610x->satu_data[i]);
					aw9610x->satu_flag[i] = 0;
				}
			}
		}
	}

	AWLOGI(p_sar->dev, "satu_irq handle over!");
}

static void aw9610x_irq_handle(struct aw_sar *p_sar)
{
	//uint8_t i = 0;
	uint32_t curr_status0 = 0;
	uint32_t curr_status3 = 0;
	uint32_t curr_status_val = 0;
	int32_t ret = 0;
	uint32_t data0 = 0;
	uint32_t data3 = 0;
	int32_t diff_val0 = 0;
	int32_t diff_val3 = 0;
	const struct aw_sar_diff_t *diff = p_sar->p_sar_para->p_diff;


	AWLOGD(p_sar->dev, "enter");

	aw_sar_i2c_read(p_sar->i2c, REG_STAT0, &curr_status_val);
	AWLOGD(p_sar->dev, "channel = 0x%08x", curr_status_val);

	if (p_sar->channels_arr == NULL) {
		AWLOGE(p_sar->dev, "input err!!!");
		return;
	}

	ret = aw_sar_i2c_read(p_sar->i2c, diff->diff0_reg, &data0);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "read diff err: %d", ret);
		return;
	}
	diff_val0 = (int32_t)data0 / (int32_t)diff->rm_float;

	ret = aw_sar_i2c_read(p_sar->i2c, diff->diff0_reg + 3 * diff->diff_step, &data3);
	if (ret != AW_OK) {
		AWLOGE(p_sar->dev, "read diff err: %d", ret);
		return;
	}
	diff_val3 = (int32_t)data3 / (int32_t)diff->rm_float;

	AWLOGD(p_sar->dev, "diff_val0 = %d,diff_val3 = %d", diff_val0,diff_val3);

#ifdef UEVENT_REPORT
	curr_status0 =
			(((uint8_t)(curr_status_val >> (24)) & 0x1))
#ifdef AW_INPUT_TRIGGER_TH1
			| (((uint8_t)(curr_status_val >> (16)) & 0x1) << 1)
#endif
#ifdef AW_INPUT_TRIGGER_TH2
			| (((uint8_t)(curr_status_val >> (8)) & 0x1) << 2)
#endif
#ifdef AW_INPUT_TRIGGER_TH3
			| (((uint8_t)(curr_status_val >> (0)) & 0x1) << 3)
#endif
			;

	curr_status3 =
				(((uint8_t)(curr_status_val >> (24 + 3)) & 0x1))
#ifdef AW_INPUT_TRIGGER_TH1
				| (((uint8_t)(curr_status_val >> (16 + 3)) & 0x1) << 1)
#endif
#ifdef AW_INPUT_TRIGGER_TH2
				| (((uint8_t)(curr_status_val >> (8 + 3)) & 0x1) << 2)
#endif
#ifdef AW_INPUT_TRIGGER_TH3
				| (((uint8_t)(curr_status_val >> (3)) & 0x1) << 3)
#endif
			;

	if(p_sar->channels_arr[0].last_channel_info != curr_status0 || p_sar->channels_arr[3].last_channel_info != curr_status3){
		snprintf(envp_buff,128,"ABS_DISTANCE=%d,%d,%d,%d",curr_status0,curr_status3,diff_val0,diff_val3);
		AWLOGD(p_sar->dev,"%s",envp_buff);
		envp[0] = envp_buff;
		envp[1] = NULL;
		kobject_uevent_env(&p_sar->dev->kobj, KOBJ_CHANGE, envp);
		p_sar->channels_arr[0].last_channel_info = curr_status0;
		p_sar->channels_arr[3].last_channel_info = curr_status3;
		sar_sensor_report(p_sar);
	}
#else
	for (i = 0; i < AW9610X_CHANNEL_MAX; i++) {
		curr_status =
			(((uint8_t)(curr_status_val >> (24 + i)) & 0x1))
#ifdef AW_INPUT_TRIGGER_TH1
			| (((uint8_t)(curr_status_val >> (16 + i)) & 0x1) << 1)
#endif
#ifdef AW_INPUT_TRIGGER_TH2
			| (((uint8_t)(curr_status_val >> (8 + i)) & 0x1) << 2)
#endif
#ifdef AW_INPUT_TRIGGER_TH3
			| (((uint8_t)(curr_status_val >> (i)) & 0x1) << 3)
#endif
			;
		//AWLOGI(p_sar->dev, "curr_state[%d] = 0x%x", i, curr_status);

		if (p_sar->channels_arr[i].used == AW_FALSE) {
			//AWLOGD(p_sar->dev, "channels_arr[%d] no user", i);
			continue;
		}
		if (p_sar->channels_arr[i].last_channel_info == curr_status) {
			continue;
		}

		switch (curr_status) {
		case AW9610X_FAR:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 0);
			break;
		case AW9610X_TRIGGER_TH0:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 1);
			break;
#ifdef AW_INPUT_TRIGGER_TH1
		case AW9610X_TRIGGER_TH1:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 2);
			break;
#endif
#ifdef AW_INPUT_TRIGGER_TH2
		case AW9610X_TRIGGER_TH2:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 3);
			break;
#endif
#ifdef AW_INPUT_TRIGGER_TH3
		case AW9610X_TRIGGER_TH3:
			input_report_abs(p_sar->channels_arr[i].input, ABS_DISTANCE, 4);
			break;
#endif
		default:
			AWLOGE(p_sar->dev, "error abs distance");
			return;
		}
		input_sync(p_sar->channels_arr[i].input);

		p_sar->channels_arr[i].last_channel_info = curr_status;
	}
#endif
}

static void aw9610x_version_aw9610x_private(struct aw_sar *p_sar)
{
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	AWLOGD(p_sar->dev, "AW9610X enter");

	if (aw9610x->satu_release == AW9610X_FUNC_ON)
		aw9610x_saturat_release_handle(p_sar);
}

static void aw9610x_irq_handle_func(uint32_t irq_status, void *data)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	AWLOGD(p_sar->dev, "enter");

	AWLOGI(p_sar->dev, "IRQSRC = 0x%x", irq_status);

	switch (aw9610x->vers) {
	case AW9610X:
		aw9610x_version_aw9610x_private(p_sar);
		break;
	case AW9610XA:
		break;
	default:
		break;
	}

	aw9610x_irq_handle(p_sar);
}

int32_t aw9610x_check_chipid(void *data)
{
	int32_t ret = -1;
	uint32_t reg_val = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;

	if (p_sar == NULL) {
		return -AW_BIN_PARA_INVALID;
	}

	ret = aw_sar_i2c_read(p_sar->i2c, REG_CHIPID, &reg_val);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read CHIP ID failed: %d", ret);
	} else {
		reg_val = reg_val >> 16;
	}

	if (reg_val == AW9610X_CHIP_ID) {
		AWLOGI(p_sar->dev, "aw9610x detected, 0x%04x", reg_val);
		return AW_OK;
	} else {
		AWLOGE(p_sar->dev, "unsupport dev, chipid is (0x%04x)", reg_val);
	}

	return -AW_ERR;
}

static const struct aw_sar_check_chipid_t g_aw9610x_check_chipid = {
	.p_check_chipid_fn = aw9610x_check_chipid,
};

static ssize_t aw9610x_operation_mode_get(void *data, char *buf)
{
	ssize_t len = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	if (p_sar->last_mode == AW9610X_ACTIVE_MODE)
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Active\n");
	else if (p_sar->last_mode == AW9610X_SLEEP_MODE)
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Sleep\n");
	else if ((p_sar->last_mode == AW9610X_DEEPSLEEP_MODE) && (aw9610x->vers == AW9610XA))
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: DeepSleep\n");
	else
		len += snprintf(buf + len, PAGE_SIZE - len, "operation mode: Unconfirmed\n");

	return len;
}

static void aw9610x_chip_info_get(void *data, char *buf, ssize_t *p_len)
{
	uint32_t reg_data = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "sar%d, aw9610x chip driver version %s\n", p_sar->dts_info.sar_num, AW9610X_DRIVER_VERSION);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "The driver supports UI\n");

	aw_sar_i2c_read(p_sar->i2c, REG_CHIPID, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "chipid is 0x%08x\n", reg_data);
	aw_sar_i2c_read(p_sar->i2c, REG_IRQEN, &reg_data);
	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "REG_HOSTIRQEN is 0x%08x\n", reg_data);

	*p_len += snprintf(buf + *p_len, PAGE_SIZE - *p_len, "chip_name:%s bin_prase_chip_name:%s\n",
							aw9610x->chip_name, aw9610x->chip_type);
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	strcpy(current_sarsensor_info.chip, aw9610x->chip_name);
	strcpy(current_sarsensor_info.vendor, "awinic_sar");
	strcpy(current_sarsensor_info.more, "sar");
#endif
}

static const struct aw_sar_get_chip_info_t g_aw9610x_get_chip_info = {
	.p_get_chip_info_node_fn = aw9610x_chip_info_get,
};

static void aw9610x_reg_version_comp(struct aw_sar *p_sar, struct aw_bin *aw_bin)
{
	uint8_t i = 0;
	uint32_t blfilt1_data = 0;
	uint32_t blfilt1_tmp = 0;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	if ((aw9610x->chip_name[7] == 'A') &&
		(aw_bin->header_info[0].chip_type[7] == '\0')) {
		AWLOGI(p_sar->dev, "enter");
		for (i = 0; i < 6; i++) {
			aw_sar_i2c_read(p_sar->i2c, REG_BLFILT_CH0 + (0x3c * i), &blfilt1_data);
			AWLOGD(p_sar->dev, "addr 0x%04x val: 0x%08x", REG_BLFILT_CH0 + (0x3c * i), blfilt1_data);
			blfilt1_tmp = (blfilt1_data >> 25) & 0x1;
			if (blfilt1_tmp == 1) {
				AWLOGD(p_sar->dev, "ablfilt1_tmp is 1");
				aw_sar_i2c_write_bits(p_sar->i2c, REG_BLRSTRNG_CH0 + (0x3c * i), ~(0x3f), 1 << i);
			}
		}
	}
}

static int32_t aw9610x_load_reg_bin(struct aw_bin *aw_bin, void *load_bin_para)
{
	int32_t ret = 0;

	struct aw_sar *p_sar = (struct aw_sar *)load_bin_para;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	snprintf(aw9610x->chip_type, sizeof(aw9610x->chip_type), "%s", aw_bin->header_info[0].chip_type);
	AWLOGE(p_sar->dev, "chip name: %s", aw9610x->chip_type);

	ret = aw_sar_load_reg(aw_bin, p_sar->i2c);
	aw9610x_reg_version_comp(p_sar, aw_bin);

	return ret;
}

static ssize_t aw9610x_get_self_cap_offset(void *data, char *buf)
{
	ssize_t len = 0;
	uint8_t i = 0;
	uint32_t reg_val = 0;
	uint32_t coff_data = 0;
	uint32_t coff_data_int = 0;
	uint32_t coff_data_dec = 0;
	uint8_t temp_data[20] = { 0 };
	struct aw_sar *p_sar = (struct aw_sar *)data;

	for (i = 0; i < AW9610X_CHANNEL_MAX; i++) {
		aw_sar_i2c_read(p_sar->i2c,
			REG_AFECFG1_CH0 + i * AW_CL1SPE_CALI_OS, &reg_val);
		coff_data = (reg_val >> 24) * 900 +
						((reg_val >> 16) & 0xff) * 13;
		coff_data_int = coff_data / 1000;
		coff_data_dec = coff_data % 1000;
		snprintf(temp_data, sizeof(temp_data), "%d.%d", coff_data_int,
								coff_data_dec);
		len += snprintf(buf+len, PAGE_SIZE-len,
				"PARASITIC_DATA_CH%d = %s pf\n", i, temp_data);
	}

	return len;
}

static const struct aw_sar_offset_t g_aw9610x_offset = {
	.p_get_offset_node_fn = aw9610x_get_self_cap_offset,
};

static uint32_t attr_buf[] = {
	8, 10,
	9, 100,
	10, 1000,
};

static void aw9610x_addrblock_load(struct aw_sar *p_sar, const char *buf)
{
	uint32_t addrbuf[4] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	uint32_t i = 0;
	struct aw9610x *aw9610x = p_sar->priv_data;
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;

	for (i = 0; i < addr_bytes; i++) {
		if (reg_num < attr_buf[1]) {
			temp_buf[0] = buf[attr_buf[0] + i * 5];
			temp_buf[1] = buf[attr_buf[0] + i * 5 + 1];
		} else if (reg_num >= attr_buf[1] && reg_num < attr_buf[3]) {
			temp_buf[0] = buf[attr_buf[2] + i * 5];
			temp_buf[1] = buf[attr_buf[2] + i * 5 + 1];
		} else if (reg_num >= attr_buf[3] && reg_num < attr_buf[5]) {
			temp_buf[0] = buf[attr_buf[4] + i * 5];
			temp_buf[1] = buf[attr_buf[4] + i * 5 + 1];
		}
		if (sscanf(temp_buf, "%02x", &addrbuf[i]) == 1)
 			aw9610x->aw_i2c_package.init_addr[i] = (uint8_t)addrbuf[i];
	}
}

static int32_t aw9610x_awrw_write_seq(struct aw_sar *p_sar)
{
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;
	uint8_t w_buf[228];
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t msg_cnt = 0;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint8_t *p_reg_data = aw9610x->aw_i2c_package.p_reg_data;
	uint32_t msg_idx = 0;

	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw9610x->aw_i2c_package.init_addr[msg_idx];
		AWLOGI(p_sar->dev, "w_buf_addr[%d] = 0x%02x", msg_idx, w_buf[msg_idx]);
	}

	msg_cnt = addr_bytes;
	for (msg_idx = 0; msg_idx < data_bytes * reg_num; msg_idx++) {
		w_buf[msg_cnt] = *p_reg_data++;
		msg_cnt++;
	}

	return aw_sar_i2c_write_seq(p_sar->i2c, w_buf, msg_cnt);
}

static void aw9610x_datablock_load(struct aw_sar *p_sar, const char *buf)
{
	uint32_t i = 0;
	uint8_t reg_data[220] = { 0 };
	uint32_t databuf[220] = { 0 };
	uint8_t temp_buf[2] = { 0 };
	struct aw9610x *aw9610x = p_sar->priv_data;
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;

	for (i = 0; i < data_bytes * reg_num; i++) {
		if (reg_num < attr_buf[1]) {
			temp_buf[0] = buf[attr_buf[0] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf[0] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf[1] && reg_num < attr_buf[3]) {
			temp_buf[0] = buf[attr_buf[2] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf[2] + (addr_bytes + i) * 5 + 1];
		} else if (reg_num >= attr_buf[3] && reg_num < attr_buf[5]) {
			temp_buf[0] = buf[attr_buf[4] + (addr_bytes + i) * 5];
			temp_buf[1] =
				buf[attr_buf[4] + (addr_bytes + i) * 5 + 1];
		}
		sscanf(temp_buf, "%02x", &databuf[i]);
		reg_data[i] = (uint8_t)databuf[i];
	}
	aw9610x->aw_i2c_package.p_reg_data = reg_data;
	aw9610x_awrw_write_seq(p_sar);
}

static int32_t aw9610x_awrw_read_seq(struct aw_sar *p_sar, uint8_t *reg_data)
{
	struct aw9610x *aw9610x= (struct aw9610x *)p_sar->priv_data;
	int32_t ret = 0;
	uint8_t w_buf[4];
	uint8_t buf[228];
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;
	uint8_t addr_bytes = aw9610x->aw_i2c_package.addr_bytes;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint32_t msg_idx = 0;
	uint32_t msg_cnt = data_bytes * reg_num;

	for (msg_idx = 0; msg_idx < addr_bytes; msg_idx++) {
		w_buf[msg_idx] = aw9610x->aw_i2c_package.init_addr[msg_idx];
		AWLOGD(p_sar->dev, "w_buf_addr[%d] = 0x%02x",
					msg_idx, w_buf[msg_idx]);
	}

	ret = aw_sar_i2c_read_seq(p_sar->i2c, w_buf, 2, (uint8_t *)buf, msg_cnt);

	for (msg_idx = 0; msg_idx < msg_cnt; msg_idx++) {
		reg_data[msg_idx] = buf[msg_idx];
		AWLOGD(p_sar->dev, "buf = 0x%02x", buf[msg_idx]);
	}

	return ret;
}

static ssize_t aw9610x_awrw_get(void *data, char *buf)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x= (struct aw9610x *)p_sar->priv_data;
	uint8_t reg_data[228] = { 0 };
	uint8_t i = 0;
	ssize_t len = 0;
	uint8_t reg_num = aw9610x->aw_i2c_package.reg_num;
	uint8_t data_bytes = aw9610x->aw_i2c_package.data_bytes;

	aw9610x_awrw_read_seq(p_sar, reg_data);
	for (i = 0; i < reg_num * data_bytes; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
						"0x%02x,", reg_data[i]);
	}

	snprintf(buf + len - 1, PAGE_SIZE - len, "\n");

	return len;
};

static ssize_t aw9610x_awrw_set(void *data, const char *buf, size_t count)
{
	uint32_t datatype[3] = { 0 };
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x= (struct aw9610x *)p_sar->priv_data;

	if (sscanf(buf, "%d %d %d", &datatype[0], &datatype[1], &datatype[2]) == 3) {
		aw9610x->aw_i2c_package.addr_bytes = (uint8_t)datatype[0];
		aw9610x->aw_i2c_package.data_bytes = (uint8_t)datatype[1];
		aw9610x->aw_i2c_package.reg_num = (uint8_t)datatype[2];

		aw9610x_addrblock_load(p_sar, buf);
		if (count > 7 + 5 * aw9610x->aw_i2c_package.addr_bytes)
			aw9610x_datablock_load(p_sar, buf);
	}

	return count;
}

static int32_t aw9610x_get_chip_version(void *data)
{
	uint32_t fw_ver = 0;
	int32_t ret = 0;
	uint32_t firmvers = 0;
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;

	aw_sar_i2c_read(p_sar->i2c, REG_FWVER, &firmvers);
	AWLOGD(p_sar->dev, "REG_FWVER = 0x%08x", firmvers);

	ret = aw_sar_i2c_read(p_sar->i2c, REG_FWVER2, &fw_ver);
	if (ret < 0) {
		AWLOGE(p_sar->dev, "read REG_FWVER2 err!");
		return -AW_ERR;
	}
	snprintf(aw9610x->chip_name, sizeof(aw9610x->chip_name), "AW9610X");
	p_sar->chip_type = AW_SAR_NONE_CHECK_CHIP;

	AWLOGD(p_sar->dev, "REG_FWVER2 : 0x%08x", fw_ver);
	if (fw_ver == AW_CHIP_AW9610XA) {
		aw9610x->vers = AW9610XA;
		memcpy(aw9610x->chip_name + strlen(aw9610x->chip_name), "A", 2);
		p_sar->chip_type = SAR_AW9610XA;
	} else {
		aw9610x->vers = AW9610X;
		p_sar->chip_type = SAR_AW9610X;
		aw9610x->chip_name[7] = '\0';
	}
	AWLOGI(p_sar->dev, "the IC is = %s", aw9610x->chip_name);

	return AW_OK;
}

#ifdef AW9610X_TVS_ABNORMAL_CAIL
static ssize_t aw9610x_set_aot(void *data)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;
	struct aw9610x *aw9610x = (struct aw9610x *)p_sar->priv_data;
	uint32_t i = 0;
	uint32_t irqen_reg_val = 0;
	uint32_t reg_val_tmp = 0;
	uint32_t max_delay_ms = AW9610X_AOT_OVER_DELAY_MAX_MS;
	uint32_t scan_over_cnt = 0;
	uint32_t ch_en = 0;
	uint32_t scan_over_en = 0;

	//1. disable chip irq
	aw_sar_i2c_read(p_sar->i2c, REG_IRQEN, &irqen_reg_val);
	aw_sar_i2c_write(p_sar->i2c, REG_IRQEN, AW_REG_IRQEN_CLOSE);

	//2. aot cail
	aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~(AW9610X_AOT_MASK << AW9610X_AOT_BIT),
						AW9610X_AOT_MASK << AW9610X_AOT_BIT);
	// aot over
	for (i = 0; i < max_delay_ms; i++) {
		aw_sar_i2c_read(p_sar->i2c, REG_IRQSRC, &reg_val_tmp);
		if (((reg_val_tmp >> AW_REG_IRQSRC_AOT_OVER_BIT) & 0x01) == 1) {
			AWLOGD(p_sar->dev, "aot over i = %d REG_IRQSRC:0X%X", i, reg_val_tmp);
			break;
		}
		AWLOGD(p_sar->dev, "aot over i = %d REG_IRQSRC:0X%X", i, reg_val_tmp);
		msleep(1);
	}

	//3. scan 8 cnt over
	aw_sar_i2c_read(p_sar->i2c, REG_SCANCTRL0, &ch_en);
	aw_sar_i2c_read(p_sar->i2c, REG_CHINTEN, &scan_over_en);
	if ((ch_en & AW9610X_AOT_MASK) != (scan_over_en & AW9610X_AOT_MASK)) {
		aw_sar_i2c_write_bits(p_sar->i2c, REG_CHINTEN, ~(AW9610X_AOT_MASK), ch_en & (AW9610X_AOT_MASK));
	}

	for (scan_over_cnt = 0; scan_over_cnt < AW9610X_AOT_SCAN_OVER_CNT; scan_over_cnt++) {
		for (i = 0; i < max_delay_ms; i++) {
			aw_sar_i2c_read(p_sar->i2c, REG_IRQSRC, &reg_val_tmp);
			if (((reg_val_tmp >> REG_IRQSRC_SCAN_OVER_BIT) & 0x01) == 1) {
				//AWLOGD(p_sar->dev, "scan %d cnt over REG_IRQSRC %d:0X%X", scan_over_cnt, i, reg_val_tmp);
				break;
			}
			msleep(1);
		}
	}
	if ((ch_en & AW9610X_AOT_MASK) != (scan_over_en & AW9610X_AOT_MASK)) {
		aw_sar_i2c_write_bits(p_sar->i2c, REG_CHINTEN, ~(AW9610X_AOT_MASK), ch_en & (AW9610X_AOT_MASK));
	}

	if (aw9610x->vers == AW9610XA) {
		//4. chip set sleep mode
		aw_sar_i2c_write(p_sar->i2c, REG_CMD, AW9610X_SLEEP_MODE);
	} else if (aw9610x->vers == AW9610X) {
		aw_sar_i2c_write(p_sar->i2c, REG_CMD, AW9610X_DEEPSLEEP_MODE); //set aw9610x sleep mode
	}
	for (i = 0; i < max_delay_ms; i++) {
		aw_sar_i2c_read(p_sar->i2c, REG_WST, &reg_val_tmp);
		if ((reg_val_tmp & 0xFF) == REG_REG_WST_SLEEP_MODE) {
			AWLOGD(p_sar->dev, "sleep i = %d REG_WST:0X%X", i, reg_val_tmp);
			break;
		}
		msleep(1);
	}

	//5. write baseline data
	for (i = 0; i < AW9610X_CHANNEL_MAX; i++) {
		aw_sar_i2c_read(p_sar->i2c, REG_COMP_CH0 + i * AW9610X_REG_OFFSET_STEP, &reg_val_tmp);
		aw_sar_i2c_write(p_sar->i2c, REG_BASELINE_CH0 + i *AW9610X_REG_OFFSET_STEP, reg_val_tmp);
	}

	//6. chip set active, irq recovery
	aw_sar_i2c_write(p_sar->i2c, REG_CMD, AW9610X_ACTIVE_MODE);
	aw_sar_i2c_write(p_sar->i2c, REG_IRQEN, irqen_reg_val);

	AWLOGD(p_sar->dev, "AW9610X_TVS_ABNORMAL_CAIL OVER!");

	return AW_OK;
}
#else
static ssize_t aw9610x_set_aot(void *data)
{
	struct aw_sar *p_sar = (struct aw_sar *)data;

	aw_sar_i2c_write_bits(p_sar->i2c, REG_SCANCTRL0, ~(AW9610X_AOT_MASK << AW9610X_AOT_BIT),
						(AW9610X_AOT_MASK) << AW9610X_AOT_BIT);
	return AW_OK;
}
#endif

static const struct aw_sar_aot_t g_aw9610x_aot = {
	.p_set_aot_node_fn = aw9610x_set_aot,
};

/**********************mode operation start*******************************/
static void aw9610x_enable_clock(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_OSCEN, AW9610X_CPU_WORK_MASK);
}

static uint32_t aw9610x_rc_irqscr(void *i2c)
{
	uint32_t val = 0;
	aw_sar_i2c_read(i2c, REG_IRQSRC, &val);
	return val;
}

//Note: TVS exceptions need to be handled after active
static void aw9610x_set_active_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9610X_ACTIVE_MODE);

#ifdef AW9610X_TVS_ABNORMAL_CAIL
	AWLOGD(g_aw_sar->dev, "AW9610X_TVS_ABNORMAL_CAIL");
	if (g_aw_sar != NULL) {
		aw9610x_set_aot(g_aw_sar);
	}
#endif
}

static void aw9610x_set_sleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9610X_SLEEP_MODE);
}

static void aw9610x_set_deepsleep_cmd(void *i2c)
{
	aw_sar_i2c_write(i2c, REG_CMD, AW9610X_DEEPSLEEP_MODE);
}

static const struct aw_sar_mode_set_t g_aw9610x_mode_set[] = {
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_ACTIVE_MODE,
			.last_mode = AW9610X_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = aw9610x_enable_clock,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_active_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_ACTIVE_MODE,
			.last_mode = AW9610X_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_active_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_ACTIVE_MODE,
			.last_mode = AW9610X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_active_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_SLEEP_MODE,
			.last_mode = AW9610X_DEEPSLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = aw9610x_enable_clock,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_sleep_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_SLEEP_MODE,
			.last_mode = AW9610X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_sleep_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA | SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_DEEPSLEEP_MODE,
			.last_mode = AW9610X_SLEEP_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = aw9610x_set_deepsleep_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610XA,
		.chip_mode = {
			.curr_mode = AW9610X_DEEPSLEEP_MODE,
			.last_mode = AW9610X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = aw9610x_rc_irqscr,
			.mode_update = aw9610x_set_deepsleep_cmd,
		},
	},
	{
		.chip_id = SAR_AW9610X,
		.chip_mode = {
			.curr_mode = AW9610X_DEEPSLEEP_MODE,
			.last_mode = AW9610X_ACTIVE_MODE,
		},
		.mode_switch_ops = {
			.enable_clock = NULL,
			.rc_irqscr = NULL,
			.mode_update = NULL,
		},
	},
};
/**********************mode operation end*******************************/

static const struct aw_sar_irq_init_t g_aw9610x_irq_init = {
	.flags = GPIOF_DIR_IN | GPIOF_INIT_HIGH,
	.irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.handler = NULL,
	.thread_fn = NULL,
	.rc_irq_fn = aw9610x_rc_irqscr,
	.irq_spec_handler_fn = aw9610x_irq_handle_func,
};

static const struct aw_sar_soft_rst_t g_aw9610x_soft_rst = {
	.reg_rst = REG_RESET,
	.reg_rst_val = 0,
	.delay_ms = 20,
};

static const struct aw_sar_init_over_irq_t g_aw9610x_init_over_irq = {
	.wait_times = 20,
	.daley_step = 1,
	.reg_irqsrc = REG_IRQSRC,
	.irq_offset_bit = 0,
	.irq_mask = 0x1,
	.irq_flag = 0x1,
};

static const struct aw_sar_load_bin_t g_aw9610x_load_reg_bin = {
	.bin_name = "aw9610x",
	.bin_opera_func = &aw9610x_load_reg_bin,
	.p_update_fn = NULL,
};

static const struct aw_sar_para_load_t g_aw9610x_reg_arr_para = {
	.reg_arr = aw9610x_reg_default,
	.reg_arr_len = (sizeof(aw9610x_reg_default) / sizeof(aw9610x_reg_default[0])),
};

static const struct aw_sar_diff_t g_aw9610x_diff = {
	.diff0_reg = REG_DIFF_CH0,
	.diff_step = 4,
	.rm_float = AW9610x_DATA_PROCESS_FACTOR,
};

static const struct aw_sar_mode_t g_aw9610x_mode = {
	.mode_set_arr = &g_aw9610x_mode_set[0],
	.mode_set_arr_len = sizeof(g_aw9610x_mode_set) / sizeof(g_aw9610x_mode_set[0]),
	.p_set_mode_node_fn = NULL,
	.p_get_mode_node_fn = aw9610x_operation_mode_get,
};

static const struct aw_sar_reg_list_t g_aw9610x_reg_list = {
	.reg_none_access = REG_NONE_ACCESS,
	.reg_rd_access = REG_RD_ACCESS,
	.reg_wd_access = REG_WR_ACCESS,
	.reg_perm = (struct aw_sar_reg_data *)&g_aw9610x_reg_access[0],
	.reg_num = sizeof(g_aw9610x_reg_access) / sizeof(g_aw9610x_reg_access[0]),
};

static const struct aw_sar_pm_t g_aw9610x_pm_chip_mode = {
	.suspend_set_mode = AW9610X_SLEEP_MODE,
	.resume_set_mode = AW9610X_ACTIVE_MODE,
	.shutdown_set_mode = AW9610X_SLEEP_MODE,
};

static const struct aw_sar_chip_mode_t g_aw9610x_chip_mode = {
	.init_mode = AW9610X_ACTIVE_MODE,
	.active = AW9610X_ACTIVE_MODE,
	.pre_init_mode = AW9610X_SLEEP_MODE,
};

static const struct aw_sar_regulator_config_t g_regulator_config = {
	.vcc_name = "vcc",
	.min_uV = AW9610X_SAR_VCC_MIN_UV,
	.max_uV = AW9610X_SAR_VCC_MAX_UV,
};

struct aw_sar_awrw_t g_aw9610x_awrw = {
	.p_set_awrw_node_fn = aw9610x_awrw_set,
	.p_get_awrw_node_fn = aw9610x_awrw_get,
};

static const struct aw_sar_platform_config g_aw9610x_platform_config = {
	.p_regulator_config = &g_regulator_config,
	.p_irq_init = &g_aw9610x_irq_init,
	.p_pm_chip_mode = &g_aw9610x_pm_chip_mode,
};

static const struct aw_sar_chip_config g_aw9610x_chip_config = {
	.ch_num_max = AW9610X_CHANNEL_MAX,

	.p_platform_config = &g_aw9610x_platform_config,

	.p_check_chipid = &g_aw9610x_check_chipid,
	.p_soft_rst = &g_aw9610x_soft_rst,
	.p_init_over_irq = &g_aw9610x_init_over_irq,
	.p_fw_bin = NULL,
	.p_reg_bin = &g_aw9610x_load_reg_bin,
	.p_chip_mode = &g_aw9610x_chip_mode,

	//Node usage parameters
	.p_reg_list = &g_aw9610x_reg_list,
	.p_reg_arr = &g_aw9610x_reg_arr_para,
	.p_aot = &g_aw9610x_aot,
	.p_diff = &g_aw9610x_diff,
	.p_offset = &g_aw9610x_offset,
	.p_mode = &g_aw9610x_mode,
	.p_prox_fw = NULL,
	.p_get_chip_info = &g_aw9610x_get_chip_info,
	.p_aw_sar_awrw = &g_aw9610x_awrw,
	.p_boot_bin = NULL,

	.p_other_operation = aw9610x_get_chip_version,
	.p_other_opera_free = NULL,
};

int32_t aw9610x_init(struct aw_sar *p_sar)
{
	if (p_sar == NULL) {
		AWLOGE(p_sar->dev, "para is NULL, error!");
		return -AW_ERR;
	}

	g_aw_sar = p_sar;

	p_sar->priv_data = devm_kzalloc(p_sar->dev, sizeof(struct aw9610x), GFP_KERNEL);
	if (p_sar->priv_data == NULL) {
		AWLOGE(p_sar->dev, "priv_data failed to malloc memory!");
		return -AW_ERR;
	}

	//Chip private function operation
	p_sar->p_sar_para = &g_aw9610x_chip_config;

	sar_sensor_init();
	return AW_OK;
}

void aw9610x_deinit(struct aw_sar *p_sar)
{
	if (p_sar->priv_data != NULL) {
		devm_kfree(p_sar->dev, p_sar->priv_data);
	}
}
