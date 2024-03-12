#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/pinctrl/consumer.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include "charger_class.h"
#include "aw32280_reg.h"
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../misc/hardware_info/hardware_info.h"
#endif
/************************************************************************/
#define USING_BAT1			             (1)
#define USING_VOUT1			             (1)

#define AW32280_ROLE_STANDALONE      (0)
#define AW32280_ROLE_SLAVE           (1)
#define AW32280_ROLE_MASTER          (2)

#define READ_MSG_LEN                 (2)
#define WRITE_MSG_LEN                (1)
#define AW_I2C_RETRIES               (5)
#define AW_I2C_RETRY_DELAY           (2)

/************************************************************************/
enum ADC_CH{
    ADC_IBUS,
    ADC_VBUS,
    ADC_VUSB,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_TDIE,
    ADC_MAX_NUM,
};

enum charger_work_mode {
	CHARGER_FBPS = 0,
	CHARGER_F21SC,
	CHARGER_F41SC,
	CHARGER_RBPS,
	CHARGER_R12SC,
	CHARGER_R14SC,
};

enum aw_charge_interrupt_type {
	AW_USB_OVP = 0,
	AW_PSW_OVP,
	AW_BUS_OVP,
	AW_BAT1_OVP,
	AW_BAT2_OVP,
	AW_USB_OVP_ALM,
	AW_PSW_OVP_ALM,
	AW_USB_UVLO,
	AW_PSW_UVLO,
	AW_BUS_UVLO,
	AW_OUT_UVLO,
	AW_USB_PLUGIN,
	AW_PSW_PLUGIN,
	AW_BUS_PLUGIN,
	AW_BAT1_OTP,
	AW_DIE_OTP,
	AW_DIE_OTP_ALM,
	AW_IQ6Q8_OCP_PEAK,
	AW_BUS_OCP,
	AW_BUS_OCP_PEAK,
	AW_BAT1_OCP,
	AW_BAT2_OCP,
	AW_BUS_UCP,
	AW_BUS_RCP,
	AW_USB_VDROP_OVP,
	AW_PSW_VDROP_OVP,
	AW_VDROP_OVP,
	AW_VDROP_MIN,
	AW_OUT_OVP,
	AW_BUS_UVP,
	AW_WD_CNT_OVF,
};
/************************************************************************/
static const u32 aw32280_adc_accuracy_tbl[ADC_MAX_NUM] = {
160000,/* IBUS */
260000,/* VBUS */
260000,/* VUSB */
60000,/* VOUT */
50000,/* VBAT */
250000,/* IBAT */
7/* TDIE */
};
/************************************************************************/
static int aw32280_mode_data[] = {
    [AW32280_ROLE_STANDALONE] = AW32280_ROLE_STANDALONE,
    [AW32280_ROLE_SLAVE] = AW32280_ROLE_SLAVE,
    [AW32280_ROLE_MASTER] = AW32280_ROLE_MASTER,
};
/************************************************************************/
struct aw32280_cfg {
	const char *chg_name;

	int host_slave_sel;
	int parallel_mode;
	int work_mode;
	int psw_en;

	int f41sc_usb_ovp;
	int f21sc_usb_ovp;
	int fbps_usb_ovp;
	int r14sc_usb_ovp;
	int r12sc_usb_ovp;
	int rbps_usb_ovp;
	int f41sc_psw_ovp;
	int f21sc_psw_ovp;
	int fbps_psw_ovp;
	int r14sc_psw_ovp;
	int r12sc_psw_ovp;
	int rbps_psw_ovp;
};

struct aw32280 {
	struct device *dev;
	struct i2c_client *client;

	int revision;
	int mode;

	int irq_gpio;
	int irq;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool usb_present;
	bool charge_enabled;

	int aw_rst_gpio;
	int sc_en_gpio;
	int usb_en_gpio;
	int aw_lpm_gpio;
	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int ibat_curr;
	int ibus_curr;
	int die_temp;

	int skip_writes;
	int skip_reads;
	int init_finished;

	struct aw32280_cfg *cfg;
	struct charger_device *chg_dev;
	struct charger_properties 	chg_prop;
	struct power_supply_desc 	psy_desc;
	struct power_supply_config  psy_cfg;
	struct power_supply 		*fc2_psy;
};
/************************************************************************/
#define aw_err(fmt, ...)  pr_err("%s:" fmt,__func__, ##__VA_ARGS__);
#define aw_info(fmt, ...)  pr_info("%s:" fmt,__func__, ##__VA_ARGS__);
/***********************************************************************/
static int aw32280_write_block(struct aw32280 *aw ,u8 *value, u16 reg, u32 num_bytes)
{
	unsigned char i = 0;
	unsigned char *buf = NULL;
	unsigned char cnt = 0;
	int ret = -EINVAL;
	struct i2c_msg msg[WRITE_MSG_LEN];

	if (aw == NULL || value == NULL) {
		pr_err("aw32280 is null or value is null\n");
		return -EIO;
	}
	buf = devm_kzalloc(&aw->client->dev,	num_bytes + AW32280_REGISTER_ADDR_LENGTH, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg >> AW32280_LENGTH_OF_BYTE;
	buf[1] = reg & 0xff;
	for (i = 0; i < num_bytes; i++)
		buf[i + AW32280_REGISTER_ADDR_LENGTH] = value[i];

	msg[0].addr = aw->client->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = num_bytes + AW32280_REGISTER_ADDR_LENGTH;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_transfer(aw->client->adapter, msg, WRITE_MSG_LEN);
		if (ret != WRITE_MSG_LEN){
			aw_err("i2c try to write %d times", cnt + 1);
		}else{
			break;
		}
		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	devm_kfree(&aw->client->dev, buf);
	buf = NULL;
	/* i2c_transfer returns number of messages transferred */
	if (ret != WRITE_MSG_LEN) {
	aw_err("write_block failed[0x%04X]\n", reg);
	if (ret < 0)
		return ret;
	else
		return -EIO;
	} else {
		return 0;
	}
}
/*********************************************************************/
static int aw32280_read_block(struct aw32280 *aw ,u8 *value,u16 reg, u32 num_bytes)
{
	unsigned char cnt = 0;
	int ret = -EINVAL;
	unsigned char buf[AW32280_REGISTER_ADDR_LENGTH] = {0};
	struct i2c_msg msg[READ_MSG_LEN];

	if (aw == NULL || value == NULL) {
		pr_err("aw32280 is null or value is null\n");
		return -EIO;
	}
	buf[0] = reg >> AW32280_LENGTH_OF_BYTE;
	buf[1] = reg & 0xff;

	msg[0].addr = aw->client->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = AW32280_REGISTER_ADDR_LENGTH;

	msg[1].addr = aw->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = value;
	msg[1].len = num_bytes;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_transfer(aw->client->adapter, msg, READ_MSG_LEN);
		if (ret != READ_MSG_LEN){
			aw_err("i2c try to read %d times", cnt + 1);
		}else{
			break;
		}
		cnt++;
		mdelay(AW_I2C_RETRY_DELAY);
	}

	/* i2c_transfer returns number of messages transferred */
	if (ret != READ_MSG_LEN) {
		aw_err("read_block failed[0x%04X]\n", reg);
		if (ret < 0){
			return ret;
		}else{
			return -EIO;
		}
	} else {
		return 0;
	}
}
/*********************************************************************/
static int aw32280_read_byte(struct aw32280 *aw, u16 reg, u8 *data)
{
	int ret = -EINVAL;

	if (aw->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&aw->i2c_rw_lock);
	ret = aw32280_read_block(aw, data, reg, 1);
	mutex_unlock(&aw->i2c_rw_lock);

	return ret;
}
/*********************************************************************/
static int aw32280_write_byte(struct aw32280 *aw, u16 reg, u8 data)
{
	int ret = -EINVAL;

	if (aw->skip_writes)
		return 0;

	mutex_lock(&aw->i2c_rw_lock);
	ret = aw32280_write_block(aw, &data, reg, 1);
	mutex_unlock(&aw->i2c_rw_lock);

	return ret;
}
/*********************************************************************/
static int aw32280_read_bytes(struct aw32280 *aw, u16 reg, u8 *data, u8 len)
{
	int ret = -EINVAL;

	if (aw->skip_reads){
		*data = 0;
		return 0;
	}

	mutex_lock(&aw->i2c_rw_lock);
	ret = aw32280_read_block(aw, data, reg, len);
	mutex_unlock(&aw->i2c_rw_lock);

	return ret;
}
/*********************************************************************/
static int aw32280_write_bytes(struct aw32280 *aw, u16 reg, u8 *data, u8 len)
{
	int ret = -EINVAL;

	if (aw->skip_writes || (data == NULL))
		return 0;

	mutex_lock(&aw->i2c_rw_lock);
	ret = aw32280_write_block(aw, data, reg, len);
	mutex_unlock(&aw->i2c_rw_lock);

	return ret;
}
/*********************************************************************/
static int aw32280_write_bits(struct aw32280 *aw, u16 reg,
                    u8 mask, u8 *value)
{
	int ret = -EINVAL;
	u8 tmp = 0;
	u8 i = 0;
	u8 index = 0;
	u8 mask_value = 0;

	aw_info("enter\n");
	if(aw->skip_reads || aw->skip_writes)
		return 0;

	for (i = 0; i < AW32280_LENGTH_OF_BYTE; i++) {
		if (mask & (1 << i)) {
			mask_value |= value[index] << i;
			index++;
		}
	}

	ret = aw32280_read_byte(aw, reg, &tmp);
	if (ret) {
		aw_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		return 0;
	}
	aw_info("[0x%04X] old value is 0x%02X\n", reg, tmp);
	tmp &= ~mask;
	tmp |= mask_value;

	ret = aw32280_write_byte(aw, reg, tmp);
	if(ret)
		aw_err("Failed: reg=%02X, ret=%d\n", reg, ret);
	aw_info("[0x%04X] new value is 0x%02X\n", reg, tmp);

	return ret;
}
/*********************************************************************/
static int aw32280_write_mask(struct aw32280 *aw, u16 reg, u8 mask,u8 shift, u8 value)
{
	u8 val = 0;
	int ret = -EINVAL;

	ret = aw32280_read_byte(aw, reg, &val);
	if (ret < 0)
		return ret;
	aw_info("[0x%04X] old value is 0x%02X\n", reg, val);

	val &= ~mask;
	val |= ((value << shift) & mask);

	ret = aw32280_write_byte(aw, reg, val);
	if (ret < 0)
		return ret;
	aw_info("[0x%04X] new value is 0x%02X\n", reg, val);

	return ret;
}
/*********************************************************************/
static int aw32280_init_reg_value(struct aw32280 *aw,
		u16 reg, u8 mask,
		const char * const *property_str,
		u8 *default_init_value, int nums)
{
	int i = 0;
	int value = 0;
	int ret = -EINVAL;
	struct device_node *np = aw->dev->of_node;
	u8 bit_array[AW32280_LENGTH_OF_BYTE] = {0};

	for (i = 0; i < nums; i++) {
		ret = of_property_read_u32(np, *(property_str + i), &value);
		if (ret) {
			bit_array[i] = default_init_value[i];
			aw_info("%s is set as the default initial value\n",
			*(property_str + i));
		} else {
			bit_array[i] = (u8)value;
			aw_info("%s is %d\n", *(property_str + i),
			bit_array[i]);
		}
	}
	ret = aw32280_write_bits(aw, reg, mask, bit_array);

	return ret;
}
/***********************************************************************/
static int aw32280_restore_digital_reg(struct aw32280 *aw)
{
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_DA_ECO_SHIELD_REG,AW32280_ECO_FORCE_EN_CLOCK);
	ret |= aw32280_write_byte(aw, AW32280_DA_ECO_SHIELD_REG,AW32280_ECO_RESTORE);
	return ret;
}
/*********************************************************************/
static int aw32280_reg_reset(struct aw32280 *aw)
{
	u8 reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_GLB_SOFT_RST_CTRL_REG, AW32280_SOFT_RESET_VALUE1);
	ret |= aw32280_write_byte(aw, AW32280_GLB_SOFT_RST_CTRL_REG, AW32280_SOFT_RESET_VALUE2);
	if (ret) {
		aw_err("error: reg_reset write fail!\n");
		return ret;
	}
	/* Reset delay time */
	mdelay(10);
	ret = aw32280_read_byte(aw, AW32280_GLB_SOFT_RST_CTRL_REG, &reg);
	if (ret) {
		aw_err("error: reg_reset read fail!\n");
		return ret;
	}
	aw_info("reg_reset [0x%04X]=0x%02X\n",AW32280_GLB_SOFT_RST_CTRL_REG, reg);
	return ret;
}
/*********************************************************************/
static int aw32280_init_det_top_reg1(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,vout-ovp-int"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_DET_TOP_CFG_REG_1_REG,
	AW32280_DET_TOP_REG_1_MASK, property_str, default_value,
	ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask(struct aw32280 *aw)
{
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,sc-glb-others-mask",
		"aw,aw32280,sc-glb-mask",
		"aw,aw32280,psw-ovp-mask",
		"aw,aw32280,usb-ovp-mask",
		"aw,aw32280,glb-mask"
	};
	u8 default_value[] = {0, 0, 0, 0, 0};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_IRQ_MASK_REG,
	AW32280_IRQ_MASK_MASK, property_str, default_value,
	ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask0(struct aw32280 *aw)
{
	u8 default_value[] = {1, 0, 0};
	u8 bit_array[] = {1, 1, 1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,vusb-uvlo-mask",
		"aw,aw32280,vusb-ovp-mask",
		"aw,aw32280,vusb-ovp-alm-mask"
	};

	aw_info("enter\n");
	ret = aw32280_write_bits(aw, AW32280_IRQ_MASK_0_REG,
	AW32280_IRQ_MASK0_DEFAULT_MASK, bit_array);

	ret |= aw32280_init_reg_value(aw, AW32280_IRQ_MASK_0_REG,
	AW32280_IRQ_MASK0_MASK, property_str, default_value,
	ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask1(struct aw32280 *aw)
{
	u8 default_value[] = {1, 0, 0};
	u8 bit_array[] = {1, 1, 1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,vpsw-uvlo-mask",
		"aw,aw32280,vpsw-ovp-mask",
		"aw,aw32280,vpsw-ovp-alm-mask"
	};

	aw_info("enter\n");
	ret = aw32280_write_bits(aw, AW32280_IRQ_MASK_1_REG,
	AW32280_IRQ_MASK1_DEFAULT_MASK, bit_array);

	ret |= aw32280_init_reg_value(aw, AW32280_IRQ_MASK_1_REG,
	AW32280_IRQ_MASK1_MASK, property_str, default_value,
	ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask2(struct aw32280 *aw)
{
	u8 bit_array[] = {1, 1};
	u8 default_value[] = {0, 0, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,vout-ovp-mask",
		"aw,aw32280,vbus-ovp-mask",
		"aw,aw32280,fvbus-uvlo-mask"
	};

	aw_info("enter\n");
	ret = aw32280_write_bits(aw, AW32280_IRQ_MASK_2_REG,
	AW32280_IRQ_MASK2_DEFAULT_MASK, bit_array);

	ret |= aw32280_init_reg_value(aw, AW32280_IRQ_MASK_2_REG,
	AW32280_IRQ_MASK2_MASK, property_str, default_value,
	ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask3(struct aw32280 *aw)
{
	u8 bit_array[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,vdrop-ovp-mask",
		"aw,aw32280,vdrop-min-mask",
		"aw,aw32280,ocp4-peak-mask",
		"aw,aw32280,fibat2-ocp-mask",
		"aw,aw32280,fibat1-ocp-mask",
		"aw,aw32280,tbat2-otp-mask",
		"aw,aw32280,tbat1-otp-mask"
	};
	u8 default_value[] = {0, 0, 0, 0, 0, 0, 0};

	aw_info("enter\n");
	ret = aw32280_write_bits(aw, AW32280_IRQ_MASK_3_REG,
	AW32280_IRQ_MASK3_DEFAULT_MASK, bit_array);

	ret |= aw32280_init_reg_value(aw, AW32280_IRQ_MASK_3_REG,
					AW32280_IRQ_MASK3_MASK, property_str, default_value,
					ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask4(struct aw32280 *aw)
{
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,rvs-vout-uvlo-mask",
		"aw,aw32280,fvbat2-ovp-mask",
		"aw,aw32280,fvbat1-ovp-mask",
		"aw,aw32280,fvbus-uvp-mask",
		"aw,aw32280,ibus-rcp-mask",
		"aw,aw32280,ibus-ucp-mask",
		"aw,aw32280,ocp3-mask"
	};
	u8 default_value[] = {0, 0, 0, 0, 0, 0, 0};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_IRQ_MASK_4_REG,
				AW32280_IRQ_MASK4_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_irq_mask5(struct aw32280 *aw)
{
	u8 default_value[] = {0, 0, 0};
	u8 bit_array[] = {1, 1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,wd-cnt-ovf-mask",
		"aw,aw32280,tdie-otp-mask",
		"aw,aw32280,tdie-otp-alm-mask"
	};

	aw_info("enter\n");
	ret = aw32280_write_bits(aw, AW32280_IRQ_MASK_5_REG,
				AW32280_IRQ_MASK5_DEFAULT_MASK, bit_array);
	ret |= aw32280_init_reg_value(aw, AW32280_IRQ_MASK_5_REG,
				AW32280_IRQ_MASK5_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_hkadc_ctrl1(struct aw32280 *aw)
{
	u8 default_value[] = {0, 0};
	int ret = -EINVAL;
	int value = 0;
	static const char * const property_str[] = {
		"aw,aw32280,hkadc-seq-loop-en",
		"aw,aw32280,hkadc-cul-time"
	};
	struct device_node *np = aw->dev->of_node;

	aw_info("enter\n");
	ret = of_property_read_u32(np, "aw,aw32280,hkadc-avg-times", &value);
	if (ret) {
		value = 0;
		aw_info("hkadc-avg-times is set as the default value\n");
	} else {
		aw_info("hkadc-avg-times is %d\n", value);
	}

	ret = aw32280_write_mask(aw, AW32280_HKADC_HKADC_CTRL1_REG,
				AW32280_HKADC_AVG_TIMES_CONFIG_MASK,
				AW32280_HKADC_AVG_TIMES_CONFIG_SHIFT,
				(u8)value);
	ret |= aw32280_init_reg_value(aw, AW32280_HKADC_HKADC_CTRL1_REG,
				AW32280_HKADC_CTRL1_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_seq_ch_h(struct aw32280 *aw)
{
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,ibat2-adc-loop",
		"aw,aw32280,ibus-ref-adc-loop",
		"aw,aw32280,ibus-adc-loop",
		"aw,aw32280,tdie-adc-loop",
		"aw,aw32280,tbat1-adc-loop"
	};
	u8 default_value[] = {0, 0, 0, 0, 0};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_HKADC_HKADC_SEQ_CH_H_REG,
				AW32280_SEQ_CH_H_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_seq_ch_l(struct aw32280 *aw)
{
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,vusb-adc-loop",
		"aw,aw32280,vpsw-adc-loop",
		"aw,aw32280,vbus-adc-loop",
		"aw,aw32280,vout1-adc-loop",
		"aw,aw32280,vout2-adc-loop",
		"aw,aw32280,vbat1-adc-loop",
		"aw,aw32280,vbat2-adc-loop",
		"aw,aw32280,ibat1-adc-loop"
	};
	u8 default_value[] = {0, 0, 0, 0, 0, 0, 0, 0};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_HKADC_HKADC_SEQ_CH_L_REG,
				AW32280_SEQ_CH_L_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_ibat1_res_place_sel(struct aw32280 *aw)
{
	u8 default_value[] = {0};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,da-ibat1-res-place-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw,
				AW32280_HKADC_DA_IBAT1_RES_PLACE_SEL_REG,
				AW32280_DA_IBAT1_RES_PLACE_SEL_MASK,
				property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_ibat2_res_place_sel(struct aw32280 *aw)
{
	u8 default_value[] = {0};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,da-ibat2-res-place-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw,
				AW32280_HKADC_DA_IBAT2_RES_PLACE_SEL_REG,
				AW32280_DA_IBAT2_RES_PLACE_SEL_MASK,
				property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_pro_top_cfg_reg6(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,da-ibat-res-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_6_REG,
				AW32280_PRO_TOP_REG_6_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_vdrop_ovp_th(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,vdrop-vol-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_12_REG,
				AW32280_DA_SC_VDROP_OVP_SEL_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_ibat1_ocp_th(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,ibat1-vol-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_4_REG,
				AW32280_DA_IBAT1_OCP_SEL_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_ibat2_ocp_th(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,ibat2-vol-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_5_REG,
				AW32280_DA_IBAT2_OCP_SEL_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}/*********************************************************************/
static int aw32280_init_sc_vpsw_en(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,psw-en"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_GLB_SC_PSW_EN_REG,
				AW32280_SC_PSW_EN_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));

	return ret;
}

/*********************************************************************/
static int aw32280_init_sc_vpsw_uvlo_en(struct aw32280 *aw)
{
	u8 default_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,psw_uvlo_en"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_PSW_OVP_CFG_REG_0_REG,
				AW32280_PSW_UVLO_EN_MASK, property_str, default_value,
				ARRAY_SIZE(property_str));
	return ret;
}

/*********************************************************************/
static int aw32280_work_mode_set(struct aw32280 *aw,u8 mode)
{
	u8 reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_GLB_SC_SC_MODE_REG, mode);
	if (ret) {
		aw_err("error: sc_mode write fail!\n");
		return ret;
	}
	ret = aw32280_read_byte(aw, AW32280_GLB_SC_SC_MODE_REG, &reg);
	if (ret) {
		aw_err("error: sc_mode read fail!\n");
		return ret;
	}
	aw_info("sc_mode [0x%04X]=0x%02X\n", AW32280_GLB_SC_SC_MODE_REG, reg);

	return ret;
}
/*********************************************************************/
static int aw32280_config_usb_ovp(struct aw32280 *aw,u8 ovp_threshold)
{
	unsigned char reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_mask(aw, AW32280_ANA_USB_OVP_CFG_REG_2_REG,
				AW32280_USB_OVP_CONFIG_MASK, AW32280_USB_OVP_CONFIG_SHIFT,
				ovp_threshold);
	if (ret) {
		aw_err("error: usb_ovp_cfg write fail!\n");
		return ret;
	}
	ret = aw32280_read_byte(aw, AW32280_ANA_USB_OVP_CFG_REG_2_REG, &reg);
	if (ret) {
		aw_err("error: usb_ovp_cfg read fail!\n");
		return ret;
	}
	aw_info("usb_ovp_cfg [0x%04X]=0x%02X\n",AW32280_ANA_USB_OVP_CFG_REG_2_REG, reg);

	return ret;
}
/*********************************************************************/
static int aw32280_config_usb_ovp_mode(struct aw32280 *aw,u8 ovp_mode)
{
	u8 reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_GLB_SC_USB_MODE_REG, ovp_mode);
	if (ret) {
		aw_err("error: usb_ovp_mode write fail!\n");
		return ret;
	}
	ret = aw32280_read_byte(aw, AW32280_GLB_SC_USB_MODE_REG, &reg);
	if (ret) {
		aw_err("error: usb_ovp_mode read fail!\n");
		return ret;
	}
	aw_info("usb_ovp_mode [0x%04X]=0x%02X\n",AW32280_GLB_SC_USB_MODE_REG, reg);

	return ret;
}
/*********************************************************************/
static int aw32280_config_psw_ovp(struct aw32280 *aw,u8 ovp_threshold)
{
	u8 reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_mask(aw, AW32280_ANA_PSW_OVP_CFG_REG_2_REG,
				AW32280_PSW_OVP_CONFIG_MASK, AW32280_PSW_OVP_CONFIG_SHIFT,
				ovp_threshold);
	if (ret) {
		aw_err("error: psw_ovp_cfg write fail!\n");
		return ret;
	}
	ret = aw32280_read_byte(aw, AW32280_ANA_PSW_OVP_CFG_REG_2_REG, &reg);
	if (ret) {
		aw_err("error: psw_ovp_cfg read fail!\n");
		return ret;
	}
	aw_info("psw_ovp_cfg [0x%04X]=0x%02X\n",AW32280_ANA_PSW_OVP_CFG_REG_2_REG, reg);

	return ret;
}
/*********************************************************************/
static int aw32280_config_psw_ovp_mode(struct aw32280 *aw,u8 ovp_mode)
{
	u8 reg = 0;
	int ret = -EINVAL;

	aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_GLB_SC_PSW_MODE_REG,ovp_mode);
	if (ret) {
		aw_err("error: psw_ovp_mode write fail!\n");
		return ret;
	}
	ret = aw32280_read_byte(aw, AW32280_GLB_SC_PSW_MODE_REG, &reg);
	if (ret) {
		aw_err("error: psw_ovp_mode read fail!\n");
		return ret;
	}
	aw_info("psw_ovp_mode [0x%04X]=0x%02X\n",AW32280_GLB_SC_PSW_MODE_REG, reg);

	return ret;
}
/*********************************************************************/
static int aw32280_charge_mode_set(struct aw32280 *aw,u8 mode)
{
	u8 work_mode = 0;
	u8 usb_dir = 0;
	u8 psw_dir = 0;
	int usb_ovp = 0;
	int psw_ovp = 0;
	int ret = -EINVAL;

	aw_info("enter,mode:%d\n", mode);
	switch (mode) {
	case CHARGER_FBPS:
		work_mode = AW32280_SC_MODE_FBPS;
		usb_ovp = aw->cfg->fbps_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_FORWARD;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->fbps_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_FORWARD;
		}
	break;
	case CHARGER_F21SC:
		work_mode = AW32280_SC_MODE_F21SC;
		usb_ovp = aw->cfg->f21sc_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_FORWARD;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->f21sc_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_FORWARD;
		}
	break;
	case CHARGER_F41SC:
		work_mode = AW32280_SC_MODE_F41SC;
		usb_ovp = aw->cfg->f41sc_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_FORWARD;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->f41sc_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_FORWARD;
		}
	break;
	case CHARGER_RBPS:
		work_mode = AW32280_SC_MODE_RBPS;
		usb_ovp = aw->cfg->rbps_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_REVERSED;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->rbps_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_REVERSED;
		}
	break;
	case CHARGER_R12SC:
		work_mode = AW32280_SC_MODE_R12SC;
		usb_ovp = aw->cfg->r12sc_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_REVERSED;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->r12sc_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_REVERSED;
		}
	break;
	case CHARGER_R14SC:
		work_mode = AW32280_SC_MODE_R14SC;
		usb_ovp = aw->cfg->r14sc_usb_ovp;
		usb_dir = AW32280_SC_USB_OVP_MODE_REVERSED;
		if (aw->cfg->psw_en) {
			psw_ovp = aw->cfg->r14sc_psw_ovp;
			psw_dir = AW32280_SC_PSW_OVP_MODE_REVERSED;
		}
	break;
	default:
		return ret;
	}
	aw->mode = work_mode;
	ret = aw32280_work_mode_set(aw, work_mode);
	ret |= aw32280_config_usb_ovp(aw, usb_ovp);
	ret |= aw32280_config_usb_ovp_mode(aw, usb_dir);
	if (aw->cfg->psw_en) {
		ret |= aw32280_config_psw_ovp(aw, psw_ovp);
		ret |= aw32280_config_psw_ovp_mode(aw, psw_dir);
	}

	return ret;
}
/*********************************************************************/
static int aw32280_init_det_top_cfg_reg2(struct aw32280 *aw)
{
	/*
	* The voltage detection range must be set to
	* 3v-22v(da-adc-det-sel = 1)
	* It is not recommended to modify.
	*/
	u8 default_init_value[] = {1};
	int ret = -EINVAL;
	static const char * const property_str[] = {"aw,aw32280,da-adc-det-sel"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_DET_TOP_CFG_REG_2_REG,
				AW32280_DA_ADC_DET_SEL_MASK, property_str, default_init_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_pro_top_reg3(struct aw32280 *aw)
{
	/* Recommend setting SC_PRO_TOP_CFG_REG_3 = 0x0F */
	u8 default_init_value[] = {1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,scbus-vdrop-ovp-int",
		"aw,aw32280,scbus-vdrop-min-int"
	};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_3_REG,
				AW32280_PRO_TOP_REG_3_MASK, property_str, default_init_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_pro_top_reg0(struct aw32280 *aw)
{
	/* Recommend setting SC_PRO_TOP_CFG_REG_0 = 0xA4 */
	u8 default_init_value[] = {1, 0};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,sc-ibus-ucp-int",
		"aw,aw32280,fwd-ibat-ocp-int"
	};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_0_REG,
				AW32280_PRO_TOP_REG_0_MASK, property_str, default_init_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_det_top_reg0(struct aw32280 *aw)
{
	/* Recommend setting SC_DET_TOP_CFG_REG_0 = 0xFF */
	u8 default_init_value[] = {1, 1, 1, 1, 1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,sc-vbus-ovp-int",
		"aw,aw32280,rvs-vout-uvlo-int",
		"aw,aw32280,fwd-vbus-uvp-int",
		"aw,aw32280,fwd-vbus-uvlo-int",
		"aw,aw32280,fwd-vbat2-ovp-int",
		"aw,aw32280,fwd-vbat1-ovp-int"};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_SC_DET_TOP_CFG_REG_0_REG,
				AW32280_DET_TOP_REG_0_MASK, property_str, default_init_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_init_usb_ovp_reg0(struct aw32280 *aw)
{
	/* Recommend setting USB_OVP_CFG_REG_0 = 0x3F */
	u8 default_init_value[] = {1, 1};
	int ret = -EINVAL;
	static const char * const property_str[] = {
		"aw,aw32280,usb-vdrop-ovp-int",
		"aw,aw32280,usb-ovp-pro-int"
	};

	aw_info("enter\n");
	ret = aw32280_init_reg_value(aw, AW32280_ANA_USB_OVP_CFG_REG_0_REG,
				AW32280_USB_OVP_REG_0_MASK, property_str, default_init_value,
				ARRAY_SIZE(property_str));

	return ret;
}
/*********************************************************************/
static int aw32280_work_mode_reg_init(struct aw32280 *aw)
{
	int ret = -EINVAL;

	aw_info("enter\n");
	/* Set the voltage detection range to 3v-22v */
	ret = aw32280_init_det_top_cfg_reg2(aw);
	/* Configure sc2_iq8_ocp_peak as 21.6A */
	ret |= aw32280_write_mask(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_8_REG,
				AW32280_SC2_IQ8_OCP_PEAK_SEL_CONFIG_MASK,
				AW32280_SC2_IQ8_OCP_PEAK_SEL_CONFIG_SHIFT,
				AW32280_SC2_IQ8_OCP_PEAK_INIT_VAL);
	/*
	* Configure sc1_iq8_ocp_peak as 21.6A.
	* sc1_iq8_ocp_peak_sel[4:0] =
	* {0x032D[5], 0x032D[7], 0x032E[1], 0x032E[3], 0x032D[3]}.
	*/
	ret |= aw32280_write_byte(aw, AW32280_ANA_SC_DET_TOP_CFG_REG_4_REG,
				AW32280_SC_DET_TOP_CFG_REG4_INIT_VAL);
	ret |= aw32280_write_byte(aw, AW32280_ANA_SC_DET_TOP_CFG_REG_5_REG,
				AW32280_SC_DET_TOP_CFG_REG5_INIT_VAL);
	/* Initialize interrupt protection. */
	ret |= aw32280_init_pro_top_reg3(aw);
	ret |= aw32280_init_pro_top_reg0(aw);
	ret |= aw32280_init_det_top_reg0(aw);
	ret |= aw32280_init_usb_ovp_reg0(aw);
	/* Enable comparator hysteresis function of vdrop_ovp/vdrop_min */
	ret |= aw32280_write_byte(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_2_REG,
				AW32280_VDROP_PRO_COMPARATOR_HYSTERESIS_EN);

	return ret;
}
/*********************************************************************/
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
static void aw32280_fill_hardwareinfo(int id)
{
	sprintf(current_cp_info.chip,"AW32280");
	sprintf(current_cp_info.id,"0x%04x",id);
	strcpy(current_cp_info.vendor, "Awinic");
	sprintf(current_cp_info.more, "charger pump");
}
#endif
/*********************************************************************/
static int aw32280_detect_device(struct aw32280 *aw)
{
	int i = 0;
	int ret = -EINVAL;
	u8 reg_val[AW32280_CHIP_ID_NUMS] = {0};
	u8 chip_id_array[AW32280_CHIP_ID_NUMS] = {AW32280_CHIP_ID0,
																	AW32280_CHIP_ID1,
																	AW32280_CHIP_ID2,
																	AW32280_CHIP_ID3};

	aw_info("enter\n");
	ret = aw32280_read_bytes(aw, AW32280_GLB_CHIP_ID_0_REG, reg_val, AW32280_CHIP_ID_NUMS);
	if (ret < 0) {
		aw_err("error: chip id read fail\n");
		return ret;
	}

	for (i = 0; i < AW32280_CHIP_ID_NUMS; i++) {
		aw_info("Chip_id_array[%d]=0x%x\n",i,reg_val[i]);
		if (reg_val[i] != chip_id_array[i])
			return -EPERM;
	}
	aw->revision = AW32280_CHIP_ID3;
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	aw32280_fill_hardwareinfo(aw->revision);
#endif
	return 0;
}
/*********************************************************************/
static int aw32280_parse_dt(struct aw32280 *aw, struct device *dev)
{
	int ret = -EINVAL;
	int usb_ovp_cfgs[6] = {0};
	int psw_ovp_cfgs[6] = {0};
	struct device_node *np = dev->of_node;

	aw->cfg = devm_kzalloc(dev, sizeof(struct aw32280_cfg), GFP_KERNEL);
	if (!aw->cfg)
		return -ENOMEM;

	ret = of_property_read_string(np, "chg_name", &aw->cfg->chg_name);
	if(ret) {
		aw_err("%s no chg name, use default\n", __func__);
		aw->cfg->chg_name = "primary_dvchg";
	}

	ret = of_property_read_u32(np, "aw,aw32280,host-slave-sel",&aw->cfg->host_slave_sel);
	if (ret) {
		aw_err("host-slave-sel dts read failed\n");
		return ret;
	}
	ret = of_property_read_u32(np, "aw,aw32280,parallel-mode", &aw->cfg->parallel_mode);
	if (ret) {
		aw_err("parallel-mode dts read failed\n");
		return ret;
	}
	ret = of_property_read_u32(np, "aw,aw32280,work-mode", &aw->cfg->work_mode);
	if (ret) {
		aw_err("work-mode dts read failed\n");
		return ret;
	}
	ret = of_property_read_u32(np, "aw,aw32280,psw-en", &aw->cfg->psw_en);
	if (ret) {
		aw_err("psw_en dts read failed\n");
		return ret;
	}

	ret = of_property_read_u32_array(np, "aw,aw32280,usb-ovp-cfgs", usb_ovp_cfgs, ARRAY_SIZE(usb_ovp_cfgs));
	if (ret) {
		aw_err("usb-ovp-cfgs dts read failed\n");
		return ret;
	}
	aw->cfg->f41sc_usb_ovp = usb_ovp_cfgs[0];
	aw->cfg->f21sc_usb_ovp = usb_ovp_cfgs[1];
	aw->cfg->fbps_usb_ovp = usb_ovp_cfgs[2];
	aw->cfg->r14sc_usb_ovp = usb_ovp_cfgs[3];
	aw->cfg->r12sc_usb_ovp = usb_ovp_cfgs[4];
	aw->cfg->rbps_usb_ovp = usb_ovp_cfgs[5];

	aw_info("usb-ovp-cfgs = (%d, %d, %d, %d, %d, %d)",
	usb_ovp_cfgs[0], usb_ovp_cfgs[1], usb_ovp_cfgs[2],
	usb_ovp_cfgs[3], usb_ovp_cfgs[4], usb_ovp_cfgs[5]);

	if(aw->cfg->psw_en)	{
		ret = of_property_read_u32_array(np, "aw,aw32280,psw-ovp-cfgs", psw_ovp_cfgs, ARRAY_SIZE(psw_ovp_cfgs));
		if (ret) {
			aw_err("psw-ovp-cfgs dts read failed\n");
		} else {
			aw->cfg->f41sc_psw_ovp = psw_ovp_cfgs[0];
			aw->cfg->f21sc_psw_ovp = psw_ovp_cfgs[1];
			aw->cfg->fbps_psw_ovp = psw_ovp_cfgs[2];
			aw->cfg->r14sc_psw_ovp = psw_ovp_cfgs[3];
			aw->cfg->r12sc_psw_ovp = psw_ovp_cfgs[4];
			aw->cfg->rbps_psw_ovp = psw_ovp_cfgs[5];
			aw_info("psw-ovp-cfgs = (%d, %d, %d, %d, %d, %d)",
			psw_ovp_cfgs[0], psw_ovp_cfgs[1],
			psw_ovp_cfgs[2], psw_ovp_cfgs[3],
			psw_ovp_cfgs[4], psw_ovp_cfgs[5]);
		}
	}

	aw->irq_gpio = of_get_named_gpio(np, "aw,aw32280,irq-gpio", 0);
	if (!gpio_is_valid(aw->irq_gpio)) {
		aw_err("fail to valid irq_gpio : %d\n", aw->irq_gpio);
		return -EINVAL;
	}

	aw->aw_rst_gpio = of_get_named_gpio(np, "aw,aw32280,rst-gpio", 0);
	if (!gpio_is_valid(aw->aw_rst_gpio)) {
		aw_err("fail to valid aw_rst_gpio : %d\n", aw->aw_rst_gpio);
		return -EINVAL;
	}
	gpio_set_value(aw->aw_rst_gpio, 1);

	aw->aw_lpm_gpio = of_get_named_gpio(np, "aw,aw32280,aw-lpm-gpio", 0);
	if (!gpio_is_valid(aw->aw_lpm_gpio)) {
		aw_err("fail to valid aw_lpm_gpio : %d\n", aw->aw_lpm_gpio);
		return -EINVAL;
	}
	gpio_set_value(aw->aw_lpm_gpio, 0);

	aw->sc_en_gpio = of_get_named_gpio(np, "aw,aw32280,sc-en-gpio", 0);
	if (!gpio_is_valid(aw->sc_en_gpio)) {
		aw_err("fail to valid gpio : %d\n", aw->sc_en_gpio);
		return -EINVAL;
	}
	gpio_set_value(aw->sc_en_gpio, 1);

	aw->usb_en_gpio = of_get_named_gpio(np, "aw,aw32280,usb-en-gpio", 0);
	if (!gpio_is_valid(aw->usb_en_gpio)) {
		aw_err("fail to valid gpio : %d\n", aw->usb_en_gpio);
		return -EINVAL;
	}
	gpio_set_value(aw->usb_en_gpio, 0);
	return 0;
}
/*********************************************************************/
static int aw32280_init_device(struct aw32280 *aw)
{
	int ret = -EINVAL;

	aw_info("enter\n");
	/* Host or slave select */
	ret = aw32280_write_mask(aw, AW32280_ANA_SC_TOP_CFG_REG_4_REG,
				AW32280_SC_HOST_SEL_MASK, AW32280_SC_HOST_SEL_SHIFT,
				aw->cfg->host_slave_sel);
	/* set parallel mode or standalone mode */
	ret |= aw32280_write_mask(aw, AW32280_ANA_SC_TOP_CFG_REG_6_REG,
				AW32280_SC_PARALLEL_SEL_MASK,
				AW32280_SC_PARALLEL_SEL_SHIFT,
				aw->cfg->parallel_mode);

	aw32280_init_det_top_reg1(aw);

	/* interrupt mask configuration */
	ret |= aw32280_init_irq_mask(aw);
	ret |= aw32280_init_irq_mask0(aw);
	ret |= aw32280_init_irq_mask1(aw);
	ret |= aw32280_init_irq_mask2(aw);
	ret |= aw32280_init_irq_mask3(aw);
	ret |= aw32280_init_irq_mask4(aw);
	ret |= aw32280_init_irq_mask5(aw);
	/* ADC configuration */
	ret |= aw32280_init_hkadc_ctrl1(aw);
	ret |= aw32280_init_seq_ch_h(aw);
	ret |= aw32280_init_seq_ch_l(aw);
	/* Sampling resistor configuration. */

	ret |= aw32280_init_ibat1_res_place_sel(aw);
	ret |= aw32280_init_ibat2_res_place_sel(aw);
	ret |= aw32280_init_pro_top_cfg_reg6(aw);

	/*set threshold*/
	ret |= aw32280_init_vdrop_ovp_th(aw);
	ret |= aw32280_init_sc_vpsw_en(aw);

	/*set ibat threshod*/
	ret |= aw32280_init_ibat1_ocp_th(aw);
	ret |= aw32280_init_ibat2_ocp_th(aw);

	/*set int enable*/
	ret |= aw32280_init_sc_vpsw_en(aw);
	ret |= aw32280_init_sc_vpsw_uvlo_en(aw);

	/*charging mode configuration*/
	ret |= aw32280_charge_mode_set(aw, aw->cfg->work_mode);
	ret |= aw32280_work_mode_reg_init(aw);
	if (ret)
		aw_err("reg_init fail\n");
	return ret;
}
/*********************************************************************/
static void aw32280_sc_en_pin_config(struct aw32280 *aw, u8 value)
{
	gpio_set_value(aw->sc_en_gpio, value);
}
/*********************************************************************/
static int aw32280_sc_enable(struct aw32280 *aw, u8 value)
{
	u8 work_mode = 0;
	int ret = 0;
	aw_info("enter\n");
	ret = aw32280_read_byte(aw, AW32280_GLB_SC_SC_MODE_REG, &work_mode);
	/* if work mode is reverse */
	if (work_mode > CHARGER_F41SC) {
		if (value != 0) {
			ret |= aw32280_write_mask(aw,
						AW32280_ANA_USB_OVP_CFG_REG_1_REG,
						AW32280_CAP_DISCHARGE_ACTIVE_MASK,
						AW32280_CAP_DISCHARGE_ACTIVE_SHIFT, 1);
			mdelay(400);
			ret |= aw32280_write_mask(aw,
						AW32280_ANA_USB_OVP_CFG_REG_1_REG,
						AW32280_CAP_DISCHARGE_ACTIVE_MASK,
						AW32280_CAP_DISCHARGE_ACTIVE_SHIFT, 0);
		}
		aw32280_sc_en_pin_config(aw, value);
	} else { /* work mode is forward */
		aw32280_sc_en_pin_config(aw, value);
		if (value != 0) {
			ret |= aw32280_write_mask(aw,
						AW32280_ANA_USB_OVP_CFG_REG_1_REG,
						AW32280_CAP_DISCHARGE_ACTIVE_MASK,
						AW32280_CAP_DISCHARGE_ACTIVE_SHIFT, 1);
			mdelay(400);
			ret |= aw32280_write_mask(aw,
						AW32280_ANA_USB_OVP_CFG_REG_1_REG,
						AW32280_CAP_DISCHARGE_ACTIVE_MASK,
						AW32280_CAP_DISCHARGE_ACTIVE_SHIFT, 0);
		}
	}
	ret |= aw32280_write_byte(aw, AW32280_GLB_SC_SC_EN_REG, value);
	return ret;
}
/*********************************************************************/
static int aw32280_enable_charge(struct aw32280 *aw, u8 value)
{
	u8 work_mode = 0;
	int ret = -EINVAL;

	ret = aw32280_read_byte(aw, AW32280_GLB_SC_SC_MODE_REG,&work_mode);
	/* work mode is forward */
	if (work_mode < CHARGER_RBPS) {
		/*
		* set vbat ovp deglitch time to 0us.
		* da_fwd_vbat1_ovp_dbt = 0 and da_fwd_vbat2_ovp_dbt = 0
		*/
		ret |= aw32280_write_mask(aw,
					AW32280_ANA_SC_DET_TOP_CFG_REG_2_REG,
					AW32280_DA_FWD_VBAT1_OVP_DBT_MASK,
					AW32280_DA_FWD_VBAT1_OVP_DBT_SHIFT,
					AW32280_VBAT_OVP_DEGLITCH_TIME_0US);
		ret |= aw32280_write_mask(aw,
					AW32280_ANA_SC_DET_TOP_CFG_REG_2_REG,
					AW32280_DA_FWD_VBAT2_OVP_DBT_MASK,
					AW32280_DA_FWD_VBAT2_OVP_DBT_SHIFT,
					AW32280_VBAT_OVP_DEGLITCH_TIME_0US);
	}

	ret |= aw32280_sc_enable(aw, value);
	mdelay(100);
	/* work mode is forward */
	if (work_mode < CHARGER_RBPS) {
		/*
		* set vbat ovp deglitch time to 128us.
		* da_fwd_vbat1_ovp_dbt = 1 and da_fwd_vbat2_ovp_dbt = 1
		*/
		ret |= aw32280_write_mask(aw,
					AW32280_ANA_SC_DET_TOP_CFG_REG_2_REG,
					AW32280_DA_FWD_VBAT1_OVP_DBT_MASK,
					AW32280_DA_FWD_VBAT1_OVP_DBT_SHIFT,
					AW32280_VBAT_OVP_DEGLITCH_TIME_128US);
		ret |= aw32280_write_mask(aw,
					AW32280_ANA_SC_DET_TOP_CFG_REG_2_REG,
					AW32280_DA_FWD_VBAT2_OVP_DBT_MASK,
					AW32280_DA_FWD_VBAT2_OVP_DBT_SHIFT,
					AW32280_VBAT_OVP_DEGLITCH_TIME_128US);
	}
	/* Enable SC IBUS reverse OCP */
	ret |= aw32280_write_mask(aw,
				AW32280_ANA_SC_PRO_TOP_CFG_REG_0_REG,
				AW32280_PRO_TOP_REG_0_IBUS_RCP_MASK,
				AW32280_PRO_TOP_REG_0_IBUS_RCP_SHIFT,
				value);

	return ret;
}
/********************************************************************/
static int aw32280_enable_chg(struct charger_device *chg_dev, bool en)
{
	struct aw32280 *aw = charger_get_data(chg_dev);

	aw_info("%s: %s\n", __func__, en ? "enable" : "disable");
	return aw32280_enable_charge(aw, !!en);
}
/*********************************************************************/
static int aw32280_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	int ret = -EINVAL;
	u8 reg = 0;
	struct aw32280 *aw = charger_get_data(chg_dev);

	ret = aw32280_read_byte(aw, AW32280_GLB_SC_SC_EN_REG, &reg);
	if (ret) {
		aw_err("charge_enable read fail!\n");
		return ret;
	}
	*en = reg;
	aw_info("charge_enable [0x%04X]=0x%02X\n",AW32280_GLB_SC_SC_EN_REG, reg);
	return ret;
}
/*********************************************************************/
static inline enum ADC_CH to_aw32280_adc(enum adc_channel chan)
{
	switch (chan) {
	case ADC_CHANNEL_VBUS:
		return ADC_VBUS;
	case ADC_CHANNEL_VBAT:
		return ADC_VBAT;
	case ADC_CHANNEL_IBUS:
		return ADC_IBUS;
	case ADC_CHANNEL_IBAT:
		return ADC_IBAT;
	case ADC_CHANNEL_TEMP_JC:
		return ADC_TDIE;
	case ADC_CHANNEL_VOUT:
		return ADC_VOUT;
	default:
		break;
	}
	return ADC_MAX_NUM;
}
/*********************************************************************/
static int aw32280_hkadc_get_seq_loop(struct aw32280 *aw,u8 *seq_loop)
{
	u8 val = 0;
	int ret = -EINVAL;

	//aw_info("enter\n");
	ret = aw32280_read_byte(aw, AW32280_HKADC_HKADC_CTRL1_REG, &val);
	if (ret == 0)
	*seq_loop = (val & AW32280_HKADC_SEQ_LOOP_MASK) >> AW32280_HKADC_SEQ_LOOP_SHIFT;

	return ret;
}
/*********************************************************************/
static int aw32280_hkadc_en(struct aw32280 *aw, int enable)
{
	u8 value = enable ?	AW32280_HKADC_ENABLE : AW32280_HKADC_DISABLE;
	int ret = -EINVAL;

	//aw_info("enter\n");
	ret = aw32280_write_byte(aw, AW32280_HKADC_HKADC_EN_REG, value);
	return ret;
}
/*********************************************************************/
static int aw32280_get_hkadc_en(struct aw32280 *aw,u8 *enable)
{
	int ret = -EINVAL;

	//aw_info("enter\n");
	ret = aw32280_read_byte(aw, AW32280_HKADC_HKADC_EN_REG, enable);
	return ret;
}
/*********************************************************************/
static int aw32280_hkadc_switch_start(struct aw32280 *aw)
{
	u8 seq_loop = 0;
	int ret = -EINVAL;

	//aw_info("enter\n");
	ret = aw32280_hkadc_en(aw, AW32280_HKADC_ENABLE);
	ret |= aw32280_hkadc_get_seq_loop(aw, &seq_loop);
	ret |= aw32280_write_byte(aw, AW32280_HKADC_HKADC_START_REG,AW32280_HKADC_START);
	msleep(15);
	/* loop convert mode */
	if (seq_loop)
		ret |= aw32280_write_byte(aw, AW32280_HKADC_HKADC_RD_SEQ_REG,AW32280_HKADC_START);
	return ret;
}
/*********************************************************************/
static int aw32280_get_adc_data(struct aw32280 *aw, enum ADC_CH _chan,int *value)
{
	u8 reg_high = 0;
	u8 reg_low = 0;
	u8 seq_loop = 0;
	u8 hkadc_en = 0;
	u16 ch_h = 0;
	u16 ch_l = 0;
	int ret = -EINVAL;

	ret = aw32280_hkadc_get_seq_loop(aw, &seq_loop);
	ret |= aw32280_get_hkadc_en(aw, &hkadc_en);

	if (hkadc_en && seq_loop)
		ret |= aw32280_write_byte(aw, AW32280_HKADC_HKADC_RD_SEQ_REG,AW32280_HKADC_START);
	else
		ret |= aw32280_hkadc_switch_start(aw);

	switch(_chan){
	case ADC_IBAT:
		#ifdef USING_BAT1
		ch_h = AW32280_HKADC_IBAT1_ADC_H_REG;
		ch_l = AW32280_HKADC_IBAT1_ADC_L_REG;
		#else
		ch_h = AW32280_HKADC_IBAT2_ADC_H_REG;
		ch_l = AW32280_HKADC_IBAT2_ADC_L_REG;
		#endif
		break;
	case ADC_IBUS:
		ch_h = AW32280_HKADC_IBUS_ADC_H_REG;
		ch_l = AW32280_HKADC_IBUS_ADC_L_REG;
		break;
	case ADC_VBUS:
		ch_h = AW32280_HKADC_VBUS_ADC_H_REG;
		ch_l = AW32280_HKADC_VBUS_ADC_L_REG;
		break;
	case ADC_VUSB:
		ch_h = AW32280_HKADC_VUSB_ADC_H_REG;
		ch_l = AW32280_HKADC_VUSB_ADC_L_REG;
		break;
	case ADC_VBAT:
		#ifdef USING_BAT1
		ch_h = AW32280_HKADC_VBAT1_ADC_H_REG;
		ch_l = AW32280_HKADC_VBAT1_ADC_L_REG;
		#else
		ch_h = AW32280_HKADC_VBAT2_ADC_H_REG;
		ch_l = AW32280_HKADC_VBAT2_ADC_L_REG;
		#endif
		break;
	case ADC_VOUT:
		#ifdef USING_VOUT1
		ch_h = AW32280_HKADC_VOUT1_ADC_H_REG;
		ch_l = AW32280_HKADC_VOUT1_ADC_L_REG;
		#else
		ch_h = AW32280_HKADC_VOUT2_ADC_H_REG;
		ch_l = AW32280_HKADC_VOUT2_ADC_L_REG;
		#endif
		break;
	case ADC_TDIE:
		ch_h = AW32280_HKADC_TDIE_ADC_H_REG;
		ch_l = AW32280_HKADC_TDIE_ADC_L_REG;
		break;
	default:
		break;
	}
	ret |= aw32280_read_byte(aw, ch_h, &reg_high);
	ret |= aw32280_read_byte(aw, ch_l, &reg_low);
	if (seq_loop == 0)
		ret |= aw32280_hkadc_en(aw, AW32280_HKADC_DISABLE);
	if (ret)
		return ret;

	aw_info("ADC_CH [%d] : ADC_H [0x%04X]=0x%02X, ADC_L [0x%04X]=0x%02X\n",_chan,ch_h, reg_high, ch_l, reg_low);
	if(_chan == ADC_IBAT){
		*value = ((reg_high << AW32280_LENGTH_OF_BYTE) + reg_low) * 12500 / 4096;
	}else if(_chan == ADC_TDIE){
		*value = 25;
	}else{
		*value = ((reg_high << AW32280_LENGTH_OF_BYTE) + reg_low)*1000;
	}

	aw_info("get_adc is %d (ma/mv)\n", (*value)/1000);
	return ret;
}
/*********************************************************************/
static int aw32280_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	int ret;
	struct aw32280 *aw = charger_get_data(chg_dev);
	enum ADC_CH _chan = to_aw32280_adc(chan);

	if (_chan == ADC_MAX_NUM)
		return -EINVAL;

	ret = aw32280_get_adc_data(aw, _chan, max);
	if (ret < 0)
		return ret;

	if (min != max)
		*min = *max;

	return ret;
}
/*********************************************************************/
static int aw32280_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
	enum ADC_CH _chan = to_aw32280_adc(chan);

	if (_chan == ADC_MAX_NUM)
		return -EINVAL;

	*min = *max = aw32280_adc_accuracy_tbl[_chan];
	return 0;
}
/*********************************************************************/
static int aw32280_dump_registers(struct charger_device *chg_dev)
{
	u8 val = 0;
	u16 i = 0;
	u8 ret = 0;
	struct aw32280 *aw = charger_get_data(chg_dev);

	for (i = 0; i < AW32280_REG_MAX; i++) {
		if (!(aw32280_reg_access[i] & REG_RD_ACCESS))
			continue;
		ret = aw32280_read_byte(aw, i, &val);
		if (ret) {
			aw_err("error: read register fail\n");
			return -1;
		}
		aw_err("aw32280_reg[0x%04X] = 0x%02X\n", i, val);
	}
	return 0;
}
/*********************************************************************/
static int aw32280_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	struct aw32280 *aw = charger_get_data(chg_dev);

	aw_info("enter\n")
	if(uV/1000 > 15000)
		aw32280_charge_mode_set(aw,CHARGER_F41SC);
	else
		aw32280_charge_mode_set(aw,CHARGER_F21SC);
	return 0;
	return 0;
}
/*********************************************************************/
static int aw32280_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	struct aw32280 *aw = charger_get_data(chg_dev);
	int ret = 0;
	u8 value = 0;

	aw_info("%d uV\n", uV)
	value = (uV/1000 - 4500)/100;
	#ifdef USING_BAT1
	ret = aw32280_write_byte(aw, AW32280_ANA_DA_FWD_VBAT1_OVP_SEL_REG, value);
	#else
	ret = aw32280_write_byte(aw, AW32280_ANA_DA_FWD_VBAT2_OVP_SEL_REG, value);
	#endif
	return ret;
}
/*********************************************************************/
static int aw32280_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	struct aw32280 *aw = charger_get_data(chg_dev);
	int ret = 0;
	u8 value = 0;

	aw_info(" %d uA\n", uA)
	value = (uA/1000 - 5200)/400;
	ret = aw32280_write_byte(aw, AW32280_ANA_SC_PRO_TOP_CFG_REG_11_REG, value);
	return ret;
}

/*********************************************************************/
static int aw32280_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	return 0;
}
/*********************************************************************/
static int aw32280_init_chip(struct charger_device *chg_dev)
{
	int ret = 0;
	struct aw32280 *aw = charger_get_data(chg_dev);

	aw_info("enter\n")
	ret = aw32280_init_device(aw);
	return ret;
}
/*********************************************************************/
static int aw32280_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}
/*********************************************************************/
static int aw32280_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}
/*********************************************************************/
static int aw32280_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}
/*********************************************************************/
static int aw32280_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}
/*********************************************************************/
static int aw32280_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	return 0;
}
/*********************************************************************/
static int aw32280_get_cp_status(struct charger_device *chg_dev, u32 evt)
{
	int val = 0;
	struct aw32280 *aw = charger_get_data(chg_dev);

	switch(evt){
	case CP_DEV_REVISION:
		val = aw->revision;
		break;
	case CP_DEV_OVPGATE:
		val = !gpio_get_value(aw->usb_en_gpio);
		break;
	case CP_DEV_WPCGATE:
		val = 0;
		break;
	case CP_DEV_WORKMODE:
		if(aw->mode == CHARGER_F41SC)
			val = 4;
		else if(aw->mode == CHARGER_F21SC)
			val = 2;
		else
			val = 1;
		break;
	default:
		break;
	}
	return val;
}
/*********************************************************************/
static int aw32280_set_mode(struct charger_device *chg_dev, bool div2)
{
	int ret = 0;
	struct aw32280 *aw = charger_get_data(chg_dev);

	if(div2)
		ret = aw32280_charge_mode_set(aw,CHARGER_F21SC);
	else
		ret = aw32280_charge_mode_set(aw,CHARGER_F41SC);
	return ret;
}
/*********************************************************************/
static const struct charger_ops aw32280_chg_ops = {
	.enable = aw32280_enable_chg,
	.is_enabled = aw32280_is_chg_enabled,
	.get_adc = aw32280_get_adc,
	.set_vbusovp = aw32280_set_vbusovp,
	.set_ibusocp = aw32280_set_ibusocp,
	.set_vbatovp = aw32280_set_vbatovp,
	.set_ibatocp = aw32280_set_ibatocp,
	.init_chip = aw32280_init_chip,
	.set_vbatovp_alarm = aw32280_set_vbatovp_alarm,
	.reset_vbatovp_alarm = aw32280_reset_vbatovp_alarm,
	.set_vbusovp_alarm = aw32280_set_vbusovp_alarm,
	.reset_vbusovp_alarm = aw32280_reset_vbusovp_alarm,
	.is_vbuslowerr = aw32280_is_vbuslowerr,
	.get_adc_accuracy = aw32280_get_adc_accuracy,
	.dump_registers = aw32280_dump_registers,
	.get_cp_status	= aw32280_get_cp_status,
	.set_operation_mode = aw32280_set_mode,
};
/*********************************************************************
*********************************************************************/
static int aw32280_register_chgdev(struct aw32280 *aw)
{
	aw_info("chg name : %s\n", aw->cfg->chg_name);

	aw->chg_prop.alias_name = aw->cfg->chg_name;
	aw->chg_dev = charger_device_register(aw->cfg->chg_name, aw->dev,
								aw, &aw32280_chg_ops,
								&aw->chg_prop);
	if (!aw->chg_dev)
		return -EINVAL;
	return 0;
}
/*********************************************************************/
static int aw32280_set_present(struct aw32280 *aw, bool present)
{
	aw->usb_present = present;

	if (present)
		aw32280_init_device(aw);
	return 0;
}

/*********************************************************************/
static ssize_t aw32280_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	u8 val = 0;
	u16 i = 0;
	ssize_t len = 0;
	int ret = -EINVAL;
	struct aw32280 *aw = dev_get_drvdata(dev);

	for (i = 0; i < AW32280_REG_MAX; i++) {
		if (!(aw32280_reg_access[i] & REG_RD_ACCESS))
			continue;
		ret = aw32280_read_byte(aw, i, &val);
		if (ret) {
			aw_err("error: read register fail\n");
			return len;
		}
		len += snprintf(buf + len, PAGE_SIZE - len,	"reg[0x%04X] = 0x%02X\n", i, val);
	}
	return len;
}
/*********************************************************************/
static ssize_t aw32280_store_registers(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw32280 *aw = dev_get_drvdata(dev);
	int ret = -EINVAL;
	u16 reg;
	u8 val;

	if(sscanf(buf, "%x %x", &reg, &val) == 2){
		if (aw32280_reg_access[reg] & REG_WR_ACCESS) {
			ret = aw32280_write_byte(aw, (u16)reg, (u8)val);
			if (ret)
				aw_info("error: write register fail\n");
		}else{
			aw_info("no permission to write 0x%04X register\n",reg);
		}
	}
	return count;
}

/*********************************************************************/
static DEVICE_ATTR(registers, 0660, aw32280_show_registers, aw32280_store_registers);
/*********************************************************************/
static void aw32280_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}
/*********************************************************************/
static enum power_supply_property aw32280_charger_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_TEMP,
};
/*********************************************************************/
static int aw32280_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
	struct aw32280 *aw = power_supply_get_drvdata(psy);
	int result;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		aw32280_is_chg_enabled(aw->chg_dev, &aw->charge_enabled);
		val->intval = aw->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = aw->usb_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = aw32280_get_adc_data(aw, ADC_VBUS, &result);
		if (!ret)
		aw->vbus_volt = result;
		val->intval = aw->vbus_volt;
		break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = aw32280_get_adc_data(aw, ADC_IBUS, &result);
		if (!ret)
		aw->ibus_curr = result;
		val->intval = aw->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = aw32280_get_adc_data(aw, ADC_VBAT, &result);
		if (!ret)
			aw->vbat_volt = result;
		val->intval = aw->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = aw32280_get_adc_data(aw, ADC_IBAT, &result);
		if (!ret)
			aw->ibat_curr = result;
		val->intval = aw->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = aw32280_get_adc_data(aw, ADC_TDIE, &result);
		if (!ret)
			aw->die_temp = result;
		val->intval = aw->die_temp;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/*********************************************************************/
static int aw32280_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
	struct aw32280 *aw = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		aw32280_enable_charge(aw, val->intval);
		aw32280_is_chg_enabled(aw->chg_dev, &aw->charge_enabled);
		aw_info("POWER_SUPPLY_PROP_ONLINE: %s\n",	val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		aw32280_set_present(aw, !!val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/*********************************************************************/
static int aw32280_charger_is_writeable(struct power_supply *psy,
                    enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}
/*********************************************************************/
static int aw32280_psy_register(struct aw32280 *aw)
{
	aw->psy_cfg.drv_data = aw;
	aw->psy_cfg.of_node = aw->dev->of_node;
	aw->psy_desc.name = "aw32280-standalone";
	aw->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	aw->psy_desc.properties = aw32280_charger_props;
	aw->psy_desc.num_properties = ARRAY_SIZE(aw32280_charger_props);
	aw->psy_desc.get_property = aw32280_charger_get_property;
	aw->psy_desc.set_property = aw32280_charger_set_property;
	aw->psy_desc.property_is_writeable = aw32280_charger_is_writeable;
	aw->psy_cfg.drv_data = aw;
	aw->fc2_psy = devm_power_supply_register(aw->dev,
								&aw->psy_desc, &aw->psy_cfg);
	if (IS_ERR(aw->fc2_psy)) {
		aw_err("failed to register fc2_psy\n");
		return PTR_ERR(aw->fc2_psy);
	}
	aw_info("%s power supply register successfully\n", aw->psy_desc.name);
	return 0;
}
/*********************************************************************/
static void aw32280_check_fault_status(struct aw32280 *aw)
{
	int ret = -EINVAL;
	int interrupt_type = 0;
	u8 irq_flag_val[6] = {0};
	unsigned long evt = CHARGER_DEV_NOTIFY_BATPRO_DONE + 1;

	mutex_lock(&aw->data_lock);
	ret = aw32280_read_bytes(aw, AW32280_IRQ_FLAG_0_REG, irq_flag_val, ARRAY_SIZE(irq_flag_val));
	if (ret < 0) {
		aw_err("read interrupt register error\n");
		mutex_unlock(&aw->data_lock);
		return;
	}

	if (irq_flag_val[0] & AW32280_VUSB_OVP_FLAG_MASK) {
		aw_info("USB OVP happened\n");
		interrupt_type = AW_USB_OVP;
	} else if (irq_flag_val[1] & AW32280_VPSW_OVP_FLAG_MASK) {
		aw_info("PSW OVP happened\n");
		interrupt_type = AW_PSW_OVP;
	} else if (irq_flag_val[2] & AW32280_VBUS_OVP_FLAG_MASK) {
		aw_info("BUS OVP happened\n");
		interrupt_type = AW_BUS_OVP;
		evt = CHARGER_DEV_NOTIFY_VBUS_OVP;
	} else if (irq_flag_val[4] & AW32280_FWD_VBAT1_OVP_FLAG_MASK) {
		aw_info("BAT1 OVP happened\n");
		interrupt_type = AW_BAT1_OVP;
		evt =  CHARGER_DEV_NOTIFY_BAT_OVP;
	} else if (irq_flag_val[4] & AW32280_FWD_VBAT2_OVP_FLAG_MASK) {
		aw_info("BAT2 OVP happened\n");
		interrupt_type = AW_BAT2_OVP;
	} else if (irq_flag_val[0] & AW32280_VUSB_OVP_ALM_FLAG_MASK) {
		aw_info("USB OVP ALM happened\n");
		interrupt_type = AW_USB_OVP_ALM;
	} else if (irq_flag_val[1] & AW32280_VPSW_OVP_ALM_FLAG_MASK) {
		aw_info("PSW OVP ALM happened\n");
		interrupt_type = AW_PSW_OVP_ALM;
	} else if (irq_flag_val[0] & AW32280_VUSB_UVLO_FLAG_MASK) {
		aw_info("USB UVLO happened\n");
		interrupt_type = AW_USB_UVLO;
	} else if (irq_flag_val[1] & AW32280_VPSW_UVLO_FLAG_MASK) {
		aw_info("PSW UVLO happened\n");
		interrupt_type = AW_PSW_UVLO;
	} else if (irq_flag_val[2] & AW32280_FWD_VBUS_UVLO_FLAG_MASK) {
		aw_info("BUS UVLO happened\n");
		interrupt_type = AW_BUS_UVLO;
	} else if (irq_flag_val[4] & AW32280_RVS_VOUT_UVLO_FLAG_MASK) {
		aw_info("OUT UVLO happened\n");
		interrupt_type = AW_OUT_UVLO;
	} else if (irq_flag_val[0] & AW32280_VUSB_PLUGIN_FLAG_MASK) {
		aw_info("USB PLUGIN happened\n");
		interrupt_type = AW_USB_PLUGIN;
	} else if (irq_flag_val[1] & AW32280_VPSW_PLUGIN_FLAG_MASK) {
		aw_info("PSW PLUGIN happened\n");
		interrupt_type = AW_PSW_PLUGIN;
	} else if (irq_flag_val[2] & AW32280_VBUS_PLUGIN_FLAG_MASK) {
		aw_info("BUS PLUGIN happened\n");
		interrupt_type = AW_BUS_PLUGIN;
	} else if (irq_flag_val[3] & AW32280_TBAT1_OTP_FLAG_MASK) {
		aw_info("BAT1 OTP happened\n");
		interrupt_type = AW_BAT1_OTP;
	} else if (irq_flag_val[5] & AW32280_TDIE_OTP_FLAG_MASK) {
		aw_info("DIE OTP happened\n");
		interrupt_type = AW_DIE_OTP;
	} else if (irq_flag_val[5] & AW32280_TDIE_OTP_ALM_FLAG_MASK) {
		aw_info("DIE OTP ALM happened\n");
		interrupt_type = AW_DIE_OTP_ALM;
	} else if (irq_flag_val[3] & AW32280_IQ6Q8_OCP_PEAK_FLAG_MASK) {
		aw_info("IQ6Q8 OCP PEAK happened\n");
		interrupt_type = AW_IQ6Q8_OCP_PEAK;
	} else if (irq_flag_val[4] & AW32280_IBUS_OCP_FLAG_MASK) {
		aw_info("BUS OCP happened\n");
		interrupt_type = AW_BUS_OCP;
		evt = CHARGER_DEV_NOTIFY_IBUSOCP;
	} else if (irq_flag_val[3] & AW32280_IBUS_OCP_PEAK_FLAG_MASK) {
		aw_info("BUS OCP PEAK happened\n");
		interrupt_type = AW_BUS_OCP_PEAK;
	} else if (irq_flag_val[3] & AW32280_FWD_IBAT1_OCP_FLAG_MASK) {
		aw_info("BAT1 OCP happened\n");
		interrupt_type = AW_BAT1_OCP;
		evt = CHARGER_DEV_NOTIFY_IBATOCP;
	} else if (irq_flag_val[3] & AW32280_FWD_IBAT2_OCP_FLAG_MASK) {
		aw_info("BAT2 OCP happened\n");
		interrupt_type = AW_BAT2_OCP;
	} else if (irq_flag_val[4] & AW32280_IBUS_UCP_FLAG_MASK) {
		aw_info("BUS UCP happened\n");
		interrupt_type = AW_BUS_UCP;
	} else if (irq_flag_val[4] & AW32280_IBUS_RCP_FLAG_MASK) {
		aw_info("BUS RCP happened\n");
		interrupt_type = AW_BUS_RCP;
	} else if (irq_flag_val[0] & AW32280_FWD_USB_VDROP_OVP_FLAG_MASK) {
		aw_info("USB VDROP OVP happened\n");
		interrupt_type = AW_USB_VDROP_OVP;
	} else if (irq_flag_val[1] & AW32280_FWD_PSW_VDROP_OVP_FLAG_MASK) {
		aw_info("PSW VDROP OVP happened\n");
		interrupt_type = AW_PSW_VDROP_OVP;
	} else if (irq_flag_val[3] & AW32280_VDROP_OVP_FLAG_MASK) {
		aw_info("VDROP OVP happened\n");
		interrupt_type = AW_VDROP_OVP;
	} else if (irq_flag_val[3] & AW32280_VDROP_MIN_FLAG_MASK) {
		aw_info("VDROP MIN happened\n");
		interrupt_type = AW_VDROP_MIN;
	} else if (irq_flag_val[2] & AW32280_VOUT_OVP_FLAG_MASK) {
		aw_info("OUT OVP happened\n");
		interrupt_type = AW_OUT_OVP;
		evt = CHARGER_DEV_NOTIFY_VOUTOVP;
	} else if (irq_flag_val[4] & AW32280_FWD_VBUS_UVP_FLAG_MASK) {
		aw_info("BUS UVP happened\n");
		interrupt_type = AW_BUS_UVP;
	} else if (irq_flag_val[5] & AW32280_WD_CNT_OVF_FLAG_MASK) {
		aw_info("WD CNT OVF happened\n");
		interrupt_type = AW_WD_CNT_OVF;
	}
	if(evt <= CHARGER_DEV_NOTIFY_BATPRO_DONE)
		charger_dev_notify(aw->chg_dev, evt);
	mutex_unlock(&aw->data_lock);
}
/*********************************************************************/
static int aw32280_clear_irq(struct aw32280 *aw)
{
	int ret = -EINVAL;
	u8 irq_value[AW32280_IRQ_REG_NUMS];

	aw_info("enter\n");
	memset(irq_value, AW32280_CLEAR_IRQ_REG_VALUE, AW32280_IRQ_REG_NUMS);
	ret = aw32280_write_bytes(aw, AW32280_IRQ_FLAG_REG, irq_value, AW32280_IRQ_REG_NUMS);
	return ret;
}
/*********************************************************************/
static irqreturn_t aw32280_charger_interrupt(int irq, void *dev_id)
{
	struct aw32280 *aw = dev_id;
#if 1
	mutex_lock(&aw->irq_complete);
	aw->irq_waiting = true;
	if (!aw->resume_completed) {
		dev_dbg(aw->dev, "IRQ triggered before device-resume\n");
		if (!aw->irq_disabled) {
			disable_irq_nosync(irq);
			aw->irq_disabled = true;
		}
		mutex_unlock(&aw->irq_complete);
		return IRQ_HANDLED;
	}
	aw->irq_waiting = false;
	aw32280_check_fault_status(aw);
	aw32280_clear_irq(aw);
	mutex_unlock(&aw->irq_complete);
#endif
	power_supply_changed(aw->fc2_psy);
	return IRQ_HANDLED;
}
/*********************************************************************/
static int aw32280_irq_register(struct aw32280 *aw)
{
	int ret;
	struct device_node *node = aw->dev->of_node;

	if (!node) {
		aw_err("device tree node missing\n");
		return -EINVAL;
	}

	if (gpio_is_valid(aw->irq_gpio)) {
		ret = gpio_request_one(aw->irq_gpio, GPIOF_DIR_IN,"aw32280_irq");
		if (ret) {
			aw_err("failed to request aw32280_irq\n");
			return -EINVAL;
		}
		aw->irq = gpio_to_irq(aw->irq_gpio);
		if (aw->irq < 0) {
			aw_err("failed to gpio_to_irq\n");
			return -EINVAL;
		}
	} else {
		aw_err("irq gpio not provided\n");
		return -EINVAL;
	}

	if (aw->irq) {
		if (aw->mode == AW32280_ROLE_STANDALONE) {
			ret = devm_request_threaded_irq(&aw->client->dev, aw->irq,
						NULL, aw32280_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"aw32280 standalone irq", aw);
		} else if (aw->mode == AW32280_ROLE_MASTER) {
			ret = devm_request_threaded_irq(&aw->client->dev, aw->irq,
						NULL, aw32280_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"aw32280 master irq", aw);
		} else {
			ret = devm_request_threaded_irq(&aw->client->dev, aw->irq,
						NULL, aw32280_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"aw32280 slave irq", aw);
		}
		if (ret < 0) {
			aw_err("request irq for irq=%d failed, ret =%d\n",aw->irq, ret);
			return ret;
		}
		enable_irq_wake(aw->irq);
	}
	return ret;
}
/*********************************************************************/
static void determine_initial_status(struct aw32280 *aw)
{
	if (aw->client->irq)
		aw32280_charger_interrupt(aw->client->irq, aw);
}
/*********************************************************************/
static struct of_device_id aw32280_charger_match_table[] = {
	{
		.compatible = "aw,aw32280-standalone",
		.data = &aw32280_mode_data[AW32280_ROLE_STANDALONE],
	},
	{
		.compatible = "aw,aw32280-master",
		.data = &aw32280_mode_data[AW32280_ROLE_MASTER],
	},
	{
		.compatible = "aw,aw32280-slave",
		.data = &aw32280_mode_data[AW32280_ROLE_SLAVE],
	},
	{},
};
/*********************************************************************/
static int aw32280_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
	struct aw32280 *aw;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret = 0;

	aw = devm_kzalloc(&client->dev, sizeof(struct aw32280), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;

	aw->dev = &client->dev;
	aw->client = client;
	mutex_init(&aw->i2c_rw_lock);
	mutex_init(&aw->data_lock);
	mutex_init(&aw->charging_disable_lock);
	mutex_init(&aw->irq_complete);
	aw->resume_completed = true;
	aw->irq_waiting = false;
	match = of_match_node(aw32280_charger_match_table, node);
	if (match == NULL) {
		aw_err("device tree match not found!\n");
		goto err_1;
	}
	aw->mode =  *(int *)match->data;
	ret = aw32280_parse_dt(aw, &client->dev);
	if (ret){
		goto err_1;
	}
	ret = aw32280_restore_digital_reg(aw);
	if (ret) {
		pr_err("error: failed to restore digital register\n");
		goto err_1;
	}
	/* Wait for the digital register to be restored. */
	mdelay(5);
	ret = aw32280_detect_device(aw);
	if (ret) {
		aw_err("No aw32280 Device found!\n");
		goto err_1;
	}
	i2c_set_clientdata(client, aw);
	aw32280_create_device_node(&(client->dev));
	ret = aw32280_reg_reset(aw);
	if (ret) {
		aw_err("aw32280 reg reset fail\n");
		goto err_1;
	}
	ret = aw32280_init_device(aw);
	if (ret) {
		aw_err("Failed to init device\n");
		goto err_1;
	}

	ret = aw32280_psy_register(aw);
	if (ret)
		goto err_2;

	ret = aw32280_irq_register(aw);
	if (ret < 0)
		goto err_2;

	ret = aw32280_register_chgdev(aw);
	if (ret < 0) {
		aw_err("reg chgdev fail(%d)\n", ret);
		goto err_3;
	}

	device_init_wakeup(aw->dev, 1);
	determine_initial_status(aw);
	aw->init_finished = true;
	aw32280_dump_registers(aw->chg_dev);
	aw_info("aw32280 probe sucessfully\n!");
	return 0;
err_3:
	charger_device_unregister(aw->chg_dev);
err_2:
	power_supply_unregister(aw->fc2_psy);
err_1:
	mutex_destroy(&aw->i2c_rw_lock);
	mutex_destroy(&aw->data_lock);
	mutex_destroy(&aw->charging_disable_lock);
	mutex_destroy(&aw->irq_complete);
	if (gpio_is_valid(aw->aw_lpm_gpio))
		gpio_set_value(aw->aw_lpm_gpio, 1);
	aw_info("aw32280 probe fail\n!");
	return ret;
}
/*********************************************************************/
static inline bool is_device_suspended(struct aw32280 *aw)
{
	return !aw->resume_completed;
}
/*********************************************************************/
static int aw32280_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw32280 *aw = i2c_get_clientdata(client);

	mutex_lock(&aw->irq_complete);
	aw->resume_completed = false;
	mutex_unlock(&aw->irq_complete);
	aw_err("Suspend successfully!");

	return 0;
}
/*********************************************************************/
static int aw32280_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw32280 *aw = i2c_get_clientdata(client);

	if (aw->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}
/*********************************************************************/
static int aw32280_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct aw32280 *aw = i2c_get_clientdata(client);

	mutex_lock(&aw->irq_complete);
	aw->resume_completed = true;
	if (aw->irq_waiting) {
		aw->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&aw->irq_complete);
		aw32280_charger_interrupt(client->irq, aw);
	} else {
		mutex_unlock(&aw->irq_complete);
	}

	power_supply_changed(aw->fc2_psy);
	aw_err("Resume successfully!");
	return 0;
}
/*********************************************************************/
static int aw32280_charger_remove(struct i2c_client *client)
{
	struct aw32280 *aw = i2c_get_clientdata(client);

	aw32280_hkadc_en(aw, AW32280_HKADC_DISABLE);
	power_supply_unregister(aw->fc2_psy);
	charger_device_unregister(aw->chg_dev);
	mutex_destroy(&aw->charging_disable_lock);
	mutex_destroy(&aw->data_lock);
	mutex_destroy(&aw->i2c_rw_lock);
	mutex_destroy(&aw->irq_complete);
	aw->init_finished = false;
	return 0;
}
/*********************************************************************/
static void aw32280_charger_shutdown(struct i2c_client *client)
{
	struct aw32280 *aw = i2c_get_clientdata(client);

	aw32280_hkadc_en(aw, AW32280_HKADC_DISABLE);
}
/*********************************************************************/
static const struct dev_pm_ops aw32280_pm_ops = {
	.resume     = aw32280_resume,
	.suspend_noirq = aw32280_suspend_noirq,
	.suspend    = aw32280_suspend,
};
/*********************************************************************/
static const struct i2c_device_id aw32280_charger_id[] = {
	{"aw32280-standalone", AW32280_ROLE_STANDALONE},
	{},
};
/*********************************************************************/
static struct i2c_driver aw32280_charger_driver = {
	.driver     = {
		.name   = "aw32280-charger",
		.owner  = THIS_MODULE,
		.of_match_table = aw32280_charger_match_table,
		.pm = &aw32280_pm_ops,
	},
	.id_table   = aw32280_charger_id,
	.probe      = aw32280_charger_probe,
	.remove     = aw32280_charger_remove,
	.shutdown   = aw32280_charger_shutdown,
};
module_i2c_driver(aw32280_charger_driver);
MODULE_DESCRIPTION("awinic aw32280 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
