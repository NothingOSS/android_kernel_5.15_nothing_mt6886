#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/input.h>
#include <linux/ktime.h>
#include <linux/workqueue.h>
#include <linux/fb.h>
#include <linux/firmware.h>
#include "hf_manager.h"
#include "gh_core.h"
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../../misc/hardware_info/hardware_info.h"
#endif


#ifdef UEVENT_REPORT
char *envp[2];
char envp_buff[128];
#endif

#define POWER_ON_DELAY_MS 3
struct gh_core_data gh_cd;
extern struct gh_hw_ops gh621x_hw_ops;

#define GH621X_FW_FILE_NAME 		"gh621x_fw.bin"
#define GH621X_CFG_FILE_NAME  		"gh621x_cfg.bin"

int gh_tools_init(struct gh_core_data *cd);
int gh_tools_exit(struct gh_core_data *cd);
enum sensor_wakeup_mode {
	NON_WAKEUP_MODE,
	WAKEUP_MODE,
};

struct sar_sensor_dev{
	struct hf_device hf_dev;
	bool enable;
	int wakeup_mode;
	bool irq_wakeup;
};

static const struct sensor_info sarsensor_support_list[] = {
	{
		.sensor_type = SENSOR_TYPE_NT_SAR,
		.gain = 1,
		.name = {'S','A','R','-','g', 'h', '6', '2', '1', 'x'},
		.vendor = {'N','o','t','h','i','n','g'},
	},
};

static struct sar_sensor_dev sar_sensor = {
	.enable = false,
	.wakeup_mode = NON_WAKEUP_MODE,
	.irq_wakeup = false,
};

static void sar_sensor_report(struct gh_event_data *gh_event, struct gh_core_data *cd)
{
	struct hf_manager *manager = sar_sensor.hf_dev.manager;
	struct hf_manager_event event;

	if (!sar_sensor.enable) {
		gh_info("sar_sensor_report sar sensor enable is false\n");
		return;
	}

	if (!manager) {
		gh_info("sar_sensor_report hf manager is null\n");
		return;
	}

	if (!gh_event) {
		gh_info("sar_sensor_report gh_event is null\n");
		return;
	}

	if (!cd) {
		gh_info("sar_sensor_report cd is null\n");
		return;
	}

	gh_info("sar_sensor_report state1:%d, state2:%d\n", gh_event->state[1], gh_event->state[2]);

	gh_info("sar_sensor_report irq_status[0]:%u, diff[0]:%d, diff[1]:%d\n", cd->debug_data.irq_status[0],
			cd->debug_data.rawdata.diff[0], cd->debug_data.rawdata.diff[1]);

	gh_info("sar_sensor_report pre_raw[0]:%d, pre_raw[1]:%d, pre_raw[2]:%d, pre_raw[3]:%d\n", cd->debug_data.rawdata.pre_raw[0],
			cd->debug_data.rawdata.pre_raw[1], cd->debug_data.rawdata.pre_raw[2], cd->debug_data.rawdata.pre_raw[3]);

	gh_info("sar_sensor_report ccsel_data[0]:%d, ccsel_data[1]:%d, ccsel_data[4]:%d, ccsel_data[5]:%d\n", cd->debug_data.ccsel_data[0],
			cd->debug_data.ccsel_data[1], cd->debug_data.ccsel_data[4], cd->debug_data.ccsel_data[5]);

	memset(&event, 0, sizeof(struct hf_manager_event));
	event.timestamp = ktime_get_boottime_ns();
	event.sensor_type = SENSOR_TYPE_NT_SAR;
	event.action = DATA_ACTION;

	/* sar channel far/near state */
	event.word[0] = gh_event->state[1];
	event.word[1] = gh_event->state[2];

	/* sar irq state */
	event.word[2] = cd->debug_data.irq_status[0];

	/* sar pre_raw data */
	event.word[3] = cd->debug_data.rawdata.pre_raw[0];
	event.word[4] = cd->debug_data.rawdata.pre_raw[1];
	event.word[5] = cd->debug_data.rawdata.pre_raw[2];
	event.word[6] = cd->debug_data.rawdata.pre_raw[3];

	/* sar diff raw data */
	event.word[7] = cd->debug_data.rawdata.diff[0];
	event.word[8] = cd->debug_data.rawdata.diff[1];

	/* sar ccsel data */
	event.word[9] = cd->debug_data.ccsel_data[0];
	event.word[10] = cd->debug_data.ccsel_data[1];
	event.word[11] = cd->debug_data.ccsel_data[4];
	event.word[12] = cd->debug_data.ccsel_data[5];

	manager->report(manager, &event);
	manager->complete(manager);
}

static int sar_sensor_enable(struct hf_device *hfdev, int sensor_type, int en)
{
	gh_info("sar_sensor_enable sensor_type:%d en:%d\n", sensor_type, en);
	sar_sensor.enable = en ? true : false;
	return 0;
}

static int sar_sensor_batch(struct hf_device *hfdev, int sensor_type,
		int64_t delay, int64_t latency)
{
	gh_info("sar_sensor_batch sensor:%d delay:%lld latency:%lld\n", sensor_type,
			delay, latency);
	return 0;
}

static int sar_sensor_flush(struct hf_device *hfdev, int sensor_type)
{
	struct hf_manager *manager = sar_sensor.hf_dev.manager;
	struct hf_manager_event event;
	gh_info("sar_sensor_flush sensor:%d \n", sensor_type);
	event.timestamp = ktime_get_boottime_ns();
	event.sensor_type = SENSOR_TYPE_NT_SAR;
	event.action = FLUSH_ACTION;
	manager->report(manager, &event);
	manager->complete(manager);
	return 0;
}

static void sar_sensor_init(void)
{
	struct hf_device *dev = &sar_sensor.hf_dev;
	int ret;

	gh_info("sar_sensor_init");

	dev->dev_name = "nt_sar_dev";
	dev->device_poll = HF_DEVICE_IO_INTERRUPT;
	dev->device_bus = HF_DEVICE_IO_ASYNC;
	dev->support_list = (struct sensor_info *)sarsensor_support_list;
	dev->support_size = ARRAY_SIZE(sarsensor_support_list);
	dev->enable = sar_sensor_enable;
	dev->batch = sar_sensor_batch;
	dev->flush = sar_sensor_flush;

	ret = hf_device_register_manager_create(dev);
	gh_info("sar_sensor_init register manager ret:%d\n", ret);
}

static bool sar_is_wakeup()
{
	gh_info("sar_is_wakeup wakeup_mode:%d\n", sar_sensor.wakeup_mode);
	return (sar_sensor.wakeup_mode == WAKEUP_MODE);
}
static int gh_parse_dt(struct gh_core_data *cd)
{
	int ret;
	const char *name_tmp;
	struct device_node *np = cd->client->dev.of_node;

	ret = of_property_read_u32(np, "goodix,irq-flag",
				   &cd->irq_trig_type);
	if (ret) {
		gh_err("Failed get int-trigger-type from dts");
		return -EINVAL;
	}

	cd->irq_gpio = of_get_named_gpio(np, "goodix,irq-gpio", 0);
	if (!gpio_is_valid(cd->irq_gpio)) {
		gh_err("No valid irq gpio");
		cd->irq_gpio = 0;
		return -EINVAL;
	}

	cd->rst_gpio = of_get_named_gpio(np, "goodix,rst-gpio", 0);
	if (!gpio_is_valid(cd->rst_gpio)) {
		gh_info("No valid rst gpio");
		cd->rst_gpio = 0;
	}

	memset(cd->avdd_name, 0, sizeof(cd->avdd_name));
	ret = of_property_read_string(np, "goodix,avdd-name", &name_tmp);
	if (!ret) {
		gh_info("avdd name form dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(cd->avdd_name))
			strncpy(cd->avdd_name,
				name_tmp, sizeof(cd->avdd_name));
		else
			gh_info("invalied avdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(cd->avdd_name));
	}

	return 0;
}

static int gh_power_setup(struct gh_core_data *cd)
{
	int ret = 0;
	struct device *dev = &cd->client->dev;

	gh_info("Power init");
	if (!strlen(cd->avdd_name)) {
		gh_info("Avdd name is NULL");
		return 0;
	}
	cd->avdd = devm_regulator_get(dev, cd->avdd_name);
	if (IS_ERR_OR_NULL(cd->avdd)) {
		ret = PTR_ERR(cd->avdd);
		gh_err("Failed to get regulator avdd:%d", ret);
		cd->avdd = NULL;
	}
	return ret;
}

static int gh_gpio_setup(struct gh_core_data *cd)
{
	int ret;

	if (cd->irq_gpio) {
		ret = devm_gpio_request_one(&cd->client->dev, cd->irq_gpio,
				GPIOF_IN, "gh_irq_gpio");
		if (ret < 0) {
			gh_err("failed request irq gpio");
			return ret;
		}
	}

	if (cd->rst_gpio) {
		ret = devm_gpio_request_one(&cd->client->dev, cd->rst_gpio,
				GPIOF_OUT_INIT_LOW, "gh_rst_gpio");
		if (ret < 0) {
			gh_err("failed request rst gpio");
			return ret;
		}
	}
	return 0;
}

static int gh_power_on(struct gh_core_data *cd)
{
	int ret;

	gh_info("power on");
	if (cd->rst_gpio)
		gpio_direction_output(cd->rst_gpio, 1);
	if (cd->avdd) {
		ret = regulator_enable(cd->avdd);
		if (ret) {
			gh_err("failed enable avdd %d", ret);
			return ret;
		}
		gh_info("avdd power on");
		udelay(POWER_ON_DELAY_MS * 1000);
	}

	return 0;
}

static void gh_power_off(struct gh_core_data *cd)
{
	if (cd->rst_gpio)
		gpio_direction_output(cd->rst_gpio, 0);
	if (cd->avdd) {
		if (regulator_disable(cd->avdd)) {
			gh_err("failed disable avdd");
		}
	}
}


static int gh_fw_request(struct device *dev, const char *name, struct fw_data *fw_data)
{
	int ret;
	const struct firmware *firmware = NULL;

	ret = request_firmware(&firmware, name, dev);
	if (ret) {
		gh_err("failed get fw patch bin[%s] error:%d",
			name, ret);
		return ret;
	}

	if (firmware->size <= 0 || firmware->size > MAX_FW_FILE_SIZE) {
		gh_err("invalid fw file size %ld", firmware->size);
		release_firmware(firmware);
		return -EINVAL;
	}

	fw_data->data = kzalloc(firmware->size, GFP_KERNEL);
	if (!fw_data->data) {
		gh_err("failed alloc memory for fw");
		return -ENOMEM;
	}
	memcpy(fw_data->data, firmware->data, firmware->size);
	fw_data->size = firmware->size;
	gh_info("success get fw file %s", name);
	release_firmware(firmware);
	return 0;
}

static int gh_get_fw_data(struct gh_core_data *cd)
{
	int ret;

	/* get patch data from file */
	ret = gh_fw_request(&cd->client->dev, GH621X_FW_FILE_NAME, &cd->patch);
	if (ret) {
		gh_err("failed get fw data");
		cd->patch.size = 0;
		cd->patch.data = NULL;
		return ret;
	}
	/* get config data from file */
	ret = gh_fw_request(&cd->client->dev, GH621X_CFG_FILE_NAME, &cd->cfg);
	if (ret) {
		gh_err("failed get cfg data");
		cd->cfg.size = 0;
		cd->cfg.data = NULL;
		kfree(cd->patch.data);
		cd->patch.size = 0;
		return ret;
	}

	gh_info("success get fw %d &cfg %d", cd->patch.size, cd->cfg.size);
	return 0;
}

static void gh_release_fw_data(struct gh_core_data *cd)
{
	if (cd->cfg.size)
		kfree(cd->cfg.data);
	if (cd->patch.size)
		kfree(cd->patch.data);

	cd->cfg.size = 0;
	cd->cfg.data = NULL;
	cd->patch.size = 0;
	cd->patch.data = NULL;
}

#ifndef QCOM_SENSOR_CLASS
static int gh_input_dev_init(struct gh_core_data *cd)
{
	int ret;

	cd->input_dev = input_allocate_device();
	if (!cd->input_dev) {
		gh_err("failed alloc input dev");
		return -EINVAL;
	}
	cd->input_dev->name = "Gh621x";
	//__set_bit(EV_KEY, cd->input_dev->evbit);

	input_set_capability(cd->input_dev, EV_ABS, ABS_DISTANCE);

	input_set_abs_params(cd->input_dev, ABS_DISTANCE, 0, 2, 0, 0);

	ret = input_register_device(cd->input_dev);
	if (ret) {
		gh_err("Unable to register accel device");
		return ret;
	}

	gh_info("success init input device");
	return 0;
}

static void gh_input_dev_remove(struct gh_core_data *cd)
{
	input_unregister_device(cd->input_dev);
	input_free_device(cd->input_dev);
	cd->input_dev = NULL;
}
#endif

static ssize_t gh_version_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int count;
	u16 fw_ver[GH621X_FW_VERSION_MAX_COUNT] = {0};
	struct gh_core_data *cd = dev_get_drvdata(dev);

	count = snprintf(buf, PAGE_SIZE, "driver version:%d.%d.%s\n",
			DRIVER_VERSION_MAJOR, DRIVER_VERSION_MINOR, DRIVER_MAGIC_STR);

	ret = cd->hw_ops->fw_ver(cd, fw_ver);
	if (ret)
		count += snprintf(buf + count, PAGE_SIZE - count, "failed get fw version info\n");
	else
		count += snprintf(buf + count, PAGE_SIZE - count, "fw_ver: 0x%.4x,0x%.4x,0x%.4x,0x%.4x\n",
			fw_ver[0], fw_ver[1], fw_ver[2], fw_ver[3]);

	return count;
}

static ssize_t gh_log_level_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int level = -1;

	if (!buf || count <= 0)
		return -EINVAL;

	sscanf(buf, "%d", &level);

	if (level >=0) {
		gh_log_level_set(level);
		pr_info("[gh621x] set log level %d\n", level);
	} else {
		pr_err("[gh621x] invalid log level %d\n", level);
	}

	return count;
}

static ssize_t gh_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);

	if (!buf || count <= 0)
		return -EINVAL;

	gh_info("reset ic");
	cd->hw_ops->sw_reset(cd);
	return count;
}

int gh_irq_enable(struct gh_core_data *cd, bool enable)
{
	if (!cd->irq_num)
		return 0;

	if (enable && !atomic_cmpxchg(&cd->irq_enabled, 0, 1)) {
		enable_irq(cd->irq_num);
		gh_info("Irq enabled");
		return 0;
	}

	if (!enable && atomic_cmpxchg(&cd->irq_enabled, 1, 0)) {
		disable_irq(cd->irq_num);
		gh_info("Irq disabled");
		return 0;
	}
	gh_debug("warning: irq deepth in-balance!");
	return 0;
}

static ssize_t gh_irq_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);

	if (!buf || count <= 0)
		return -EINVAL;

	gh_info("set irq %s", buf[0] == '0' ? "disable" : "enable");
	gh_irq_enable(cd, buf[0] == '0' ? 0 : 1);
	return count;
}

static ssize_t gh_mode_ctrl_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);
	int mode;
	int ret;

	if (!buf || count <= 0)
		return -EINVAL;

	sscanf(buf, "%d", &mode);

	ret = cd->hw_ops->mode_ctrl(cd, mode);
	if (ret) {
		gh_err("failed set mode %d", mode);
		return -EFAULT;
	}
	gh_info("success set mode %d", mode);
	return count;
}

static ssize_t gh_manual_calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);
	int enable = 0;

	if (!buf || count <= 0)
		return -EINVAL;

	sscanf(buf, "%d", &enable);

	if (enable) {
		if(cd->hw_ops->calibration(cd))
			gh_err("failed trig calibration");
		else
			gh_info("success do calibration");
	} else {
		gh_err("invalid input");
	}
	return count;
}


/*reg read/write */
#define GH621X_REG_ADDR_LEN		2
#define GH621X_RW_MAX_LEN		64
static uint16_t rw_addr;
static uint32_t rw_len;
static uint8_t rw_flag;
static uint8_t store_buf[66];
static uint8_t show_buf[PAGE_SIZE];
static ssize_t gh_reg_rw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct gh_core_data *cd = dev_get_drvdata(dev);

	if (!rw_addr || !rw_len) {
		gh_err("address(0x%x) and length(%d) cann't be null",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		gh_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = cd->hw_ops->read(cd, rw_addr, show_buf, rw_len, 0);
	if (ret) {
		gh_err("failed read addr(%x) length(%d)", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
				"failed read addr(%x), len(%d)\n",
				rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n",
			rw_addr, rw_len, rw_len, show_buf);
}

static ssize_t gh_reg_rw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);
	char *pos = NULL, *token = NULL;
	long result = 0;
	int ret, i;

	if (!buf || !count) {
		gh_err("invalid params");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		gh_err("string must start with 'r/w'");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		gh_err("invalid address info");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			gh_err("failed get addr info");
			goto err_out;
		}
		rw_addr = (u16)result;
		gh_info("rw addr is 0x%x", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		gh_err("invalid length info");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			gh_err("failed get length info");
			goto err_out;
		}
		rw_len = (u32)result;
		gh_info("rw length info is %d", rw_len);
		if (rw_len > GH621X_RW_MAX_LEN) {
			gh_err("data len > %lu", GH621X_RW_MAX_LEN);
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	/* get write buf */
	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			gh_err("invalid data info");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				gh_err("failed get data[%d] info", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
			gh_info("get data[%d]=0x%x", i, store_buf[i]);
		}
	}

	/* i2c write ops */
	ret = cd->hw_ops->write(cd, rw_addr, store_buf, rw_len, 0);
	if (ret) {
		gh_err("failed write addr(%x) data %*ph", rw_addr, rw_len,
				store_buf);
		goto err_out;
	}
	gh_info("%s write to addr (%x) with data %*ph",
		"success", rw_addr, rw_len, store_buf);

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s",
		"invalid params, format{r/w:8240:length:[41:21:31]}");
	return -EINVAL;
}

static ssize_t gh_low_power_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gh_core_data *cd = dev_get_drvdata(dev);
	int mode;
	int ret;

	if (!buf || count <= 0)
		return -EINVAL;

	if (sar_is_wakeup()) {
		return count;
	}

	sscanf(buf, "%d", &mode);

	if (mode == 0) {
		// resume
		ret = cd->hw_ops->resume(cd);
	} else if (mode == 1) {
		// power down
		ret = cd->hw_ops->suspend(cd, GH621X_MCU_PD);
	} else if (mode == 2) {
		// deep sleep
		ret = cd->hw_ops->suspend(cd, GH621X_MCU_DONT_PD);
	} else {
		gh_err("unsupported");
		ret = -EINVAL;
	}


	gh_info("[low_power] ret %d, mode %d", ret, mode);
	return count;
}

static ssize_t gh_wakeup_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (!buf || count <= 0)
		return -EINVAL;

	sscanf(buf, "%d", &sar_sensor.wakeup_mode);
	gh_info("gh_wakeup_mode_store wakeup_mode %d\n", sar_sensor.wakeup_mode);
	return count;
}

static ssize_t gh_wakeup_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sar_sensor.wakeup_mode);
}

static DEVICE_ATTR(version_info, 0444, gh_version_info_show, NULL);
static DEVICE_ATTR(log_level, 0220, NULL, gh_log_level_store);
static DEVICE_ATTR(reset, 0220, NULL, gh_reset_store);
static DEVICE_ATTR(irq, 0220, NULL, gh_irq_store);
static DEVICE_ATTR(mode_ctrl, 0220, NULL, gh_mode_ctrl_store);
static DEVICE_ATTR(manual_calibration, 0220, NULL, gh_manual_calibration_store);
static DEVICE_ATTR(reg_rw, 0664, gh_reg_rw_show, gh_reg_rw_store);
static DEVICE_ATTR(low_power, 0220, NULL, gh_low_power_store);
static DEVICE_ATTR(wakeup_mode, 0660, gh_wakeup_mode_show, gh_wakeup_mode_store);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_version_info.attr,
	&dev_attr_log_level.attr,
	&dev_attr_reset.attr,
	&dev_attr_irq.attr,
	&dev_attr_mode_ctrl.attr,
	&dev_attr_manual_calibration.attr,
	&dev_attr_reg_rw.attr,
	&dev_attr_low_power.attr,
	&dev_attr_wakeup_mode.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static int gh_sysfs_init(struct device *dev)
{
	int ret;

	ret = sysfs_create_group(&dev->kobj, &sysfs_group);
	if (ret) {
		gh_err("failed create sysfs group");
		return ret;
	}
	gh_info("success create sysfs group");
	return ret;
}

static void gh_sysfs_remove(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &sysfs_group);
}

irqreturn_t gh_irq_thread(int irq, void *data)
{
	int ret;
	struct gh_core_data *cd = (struct gh_core_data *)data;
	struct gh_event_data event_data;
	static struct gh_event_data pre_event_data;

	ret = cd->hw_ops->event_handler(cd, &event_data);
	if (ret) {
		gh_err("failed handle irq");
		return IRQ_HANDLED;
	}
#ifdef UEVENT_REPORT
/*	if (event_data.state[0] != pre_event_data.state[0]) {*/
		snprintf(envp_buff,128,"ABS_DISTANCE=%d,%d,%d,%d",event_data.state[1],event_data.state[2],cd->debug_data.rawdata.diff[0],cd->debug_data.rawdata.diff[1]);
/*		if (event_data.state[0] == STATE_CLOSE)
			sprintf(envp_buff, "%s", "ABS_DISTANCE0=%d,ABS_DISTANCE2=%d",);
		else
			sprintf(envp_buff, "%s", "ABS_DISTANCE=1");
*/
		gh_info("envp_buff = %s\n",envp_buff);
		envp[0] = envp_buff;
		envp[1] = NULL;
		kobject_uevent_env(&cd->client->dev.kobj, KOBJ_CHANGE, envp);
/*	}*/
	pre_event_data = event_data;
	sar_sensor_report(&pre_event_data, cd);
#else
	if (event_data.state != pre_event_data.state) {
		if (event_data.state == STATE_CLOSE)
			input_report_abs(cd->input_dev, ABS_DISTANCE, STATE_CLOSE);
		else
			input_report_abs(cd->input_dev, ABS_DISTANCE, STATE_FAR);

		input_sync(cd->input_dev);
	}
	pre_event_data = event_data;
#endif
	return IRQ_HANDLED;
}

int gh_irq_setup(struct gh_core_data *cd)
{
	int ret;
	int irq = gpio_to_irq(cd->irq_gpio);

	gh_info("%s: irq %d, flag 0x%x", cd->client->name, irq,
			cd->irq_trig_type);
	ret = devm_request_threaded_irq(&cd->client->dev,
			irq, NULL,
			gh_irq_thread,
			IRQF_ONESHOT | cd->irq_trig_type,
			cd->client->name, &gh_cd);
	if (ret < 0) {
		gh_err("failed request irq %d", ret);
		return ret;
	}
	atomic_set(&cd->irq_enabled, 1);
	cd->irq_num = irq;

	return 0;
}

static int gh_later_init_thread(void *data)
{
	int ret;
	struct gh_core_data *cd = data;

	gh_info("later init work start");

	ret = gh_get_fw_data(cd);
	if (ret) {
		gh_err("failed get fw or config, %d", ret);
		return ret;
	}

	ret = cd->hw_ops->fw_update(cd, cd->cfg.data,
			 cd->cfg.size, cd->patch.data, cd->patch.size);
	if (ret) {
		gh_err("failed load fw and config %d", ret);
		goto err_fw_update;
	}

#ifdef QCOM_SENSOR_CLASS
#else
	ret = gh_input_dev_init(cd);
	if (ret) {
		gh_err("failed init input device");
		goto err_input_dev;
	}
#endif

	ret = gh_tools_init(cd);
	if (ret)
		goto err_tools;

	ret = gh_sysfs_init(&cd->client->dev);
	if (ret)
		goto err_sysfs;

	ret = cd->hw_ops->work_start(cd);
	if (ret) {
		gh_err("failed start work");
		goto err_work_start;
	}

	ret = cd->hw_ops->fw_ver(cd, cd->fw_version);
	if (ret) {
		gh_err("failed get fw version");
	} else {
		gh_info("fw_ver: 0x%.4x,0x%.4x,0x%.4x,0x%.4x",
			cd->fw_version[0], cd->fw_version[1], cd->fw_version[2], cd->fw_version[3]);
	}

	// if (cd->hw_ops->calibration(cd))
	// 	gh_err("failed do calibration[ignored]");
	// else
	// 	gh_info("calibration done");

	if (cd->hw_ops->mode_ctrl(cd, WORK_MODE_NORMAL))
		gh_err("failed set to normal mode");
	else
		gh_info("success set to normal mode");

	ret = gh_irq_setup(cd);
	if (ret) {
		gh_err("failed init irq %d", ret);
		goto err_irq;
	}

	cd->init_stage = CORE_INIT_STAGE2;
	return 0;

err_irq:
err_work_start:
	gh_sysfs_remove(&cd->client->dev);
err_sysfs:
	gh_tools_exit(cd);
err_tools:
#ifndef QCOM_SENSOR_CLASS
	gh_input_dev_remove(cd);
#endif
err_input_dev:
err_fw_update:
	gh_release_fw_data(cd);
	return ret;
}

static int gh_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct gh_core_data *cd;
	struct task_struct *init_thrd;

	gh_log_level_set(GH621X_LOG_LEVEL_INFO);
	gh_info("goodix sar driver v%d.%d_%s", DRIVER_VERSION_MAJOR,
		 DRIVER_VERSION_MINOR, DRIVER_MAGIC_STR);
	gh_info("I2C Address: 0x%02x", client->addr);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		gh_err("Failed check I2C functionality");
		return -ENODEV;
	}

	cd = &gh_cd;
	memset(cd, 0, sizeof(*cd));
	i2c_set_clientdata(client, cd);
	/* init core data */
	mutex_init(&cd->mutex);

	cd->client = client;
	cd->hw_ops = &gh621x_hw_ops;
	ret = gh_parse_dt(cd);
	if (ret) {
		gh_err("failed parse dts, %d", ret);
		return ret;
	}

	/* avdd setup */
	ret = gh_power_setup(cd);
	if (ret) {
		gh_err("failed get avdd regulator %d", ret);
		return ret;
	}

	/* setup gpio */
	ret = gh_gpio_setup(cd);
	if (ret) {
		gh_err("failed setup gpios %d", ret);
		goto err_gpio;
	}

	ret = gh_power_on(cd);
	if (ret) {
		gh_err("failed power on device %d", ret);
		goto err_power_on;
	}

	ret = cd->hw_ops->dev_confirm(cd);
	if (ret) {
		gh_err("dev confirm failed %d", ret);
		goto err_dev;
	}
	
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	strcpy(current_sarsensor_info.chip, "gh621x");
	strcpy(current_sarsensor_info.vendor, "goodix_sar");
	strcpy(current_sarsensor_info.more, "sar");
#endif

	init_thrd = kthread_run(gh_later_init_thread,
				cd, "gh_init_thread");
	if (IS_ERR_OR_NULL(init_thrd)) {
		gh_err("Failed to create init thread:%ld", PTR_ERR(init_thrd));
		return -EFAULT;
	}
	cd->init_stage = CORE_INIT_STAGE1;
	sar_sensor_init();
	gh_info("probe out");
	return 0;
err_dev:
err_gpio:
err_power_on:
	gh_power_off(cd);
	gh_info("probe failed");
	return ret;
}

static int gh_drv_remove(struct i2c_client *client)
{
	// TODO
	struct gh_core_data *cd = i2c_get_clientdata(client);

	if (cd->init_stage < CORE_INIT_STAGE2)
		return 0;

	gh_tools_exit(cd);
	gh_sysfs_remove(&cd->client->dev);
	gh_release_fw_data(cd);
#ifndef QCOM_SENSOR_CLASS
	gh_input_dev_remove(cd);
#endif

	cd->hw_ops->work_stop(cd);
	gh_power_off(cd);
	return 0;
}


#ifdef CONFIG_PM
int gh621x_suspend(struct gh_core_data *cd, enum power_mode mode);
int gh621x_resume(struct gh_core_data *cd);
/**
 * gh_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int gh_pm_suspend(struct device *dev)
{
	int ret;
	struct gh_core_data *cd = dev_get_drvdata(dev);
	if (sar_is_wakeup()) {
		if (atomic_read(&cd->irq_enabled)) {
			if (!enable_irq_wake(cd->irq_num)) {
				sar_sensor.irq_wakeup = true;
				gh_info("enable irq_num:%d wakeup\n", cd->irq_num);
			}
		}
		return 0;
	}
	// ret = cd->hw_ops->suspend(cd, GH621X_MCU_DONT_PD);
	ret = gh621x_suspend(cd, GH621X_MCU_DONT_PD);
	if (ret) {
		gh_err("failed enter suspend mode, %d", ret);
	} else {
		gh_info("success enter suspend");
	}
	return 0;
}

/**
 * gh_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int gh_pm_resume(struct device *dev)
{
	int ret;
	struct gh_core_data *cd = dev_get_drvdata(dev);
	if (sar_is_wakeup()) {
		if (sar_sensor.irq_wakeup) {
			if (!disable_irq_wake(cd->irq_num)) {
				sar_sensor.irq_wakeup = false;
				gh_info("disable irq_num:%d wakeup\n", cd->irq_num);
			}
		}
		return 0;
	}

	//ret = cd->hw_ops->resume(cd);
	ret = gh621x_resume(cd);
	if (ret) {
		gh_err("failed resume, %d", ret);
	} else {
		gh_info("success resume");
	}
	return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id gh_match_table[] = {
	{.compatible = "goodix,gh621x",},
	{ },
};
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gh_pm_suspend, gh_pm_resume)
};
#endif

static const struct i2c_device_id gh_device_id[] = {
	{ GH_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver gh_i2c_driver = {
	.probe		= gh_probe,
	.remove		= gh_drv_remove,
	.id_table	= gh_device_id,
	.driver = {
		.name	  = GH_I2C_NAME,
		.owner	  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = gh_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &dev_pm_ops,
#endif
	},
};

static int __init gh_init(void)
{
	gh_info("driver install");
	return i2c_add_driver(&gh_i2c_driver);
}

static void __exit gh_exit(void)
{
	gh_info("driver exit");
	i2c_del_driver(&gh_i2c_driver);
}

module_init(gh_init);
module_exit(gh_exit);

MODULE_DESCRIPTION("Goodix Graz Driver");
MODULE_LICENSE("GPL v2");
