/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2022 Southchip Semiconductor Technology(Shanghai) Co., Ltd.
 */

#define pr_fmt(fmt)	"[sc8562] %s: " fmt, __func__

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
#include "sc8562_reg.h"
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
#include "../../misc/hardware_info/hardware_info.h"
#endif
/*********************************************************************/
enum ADC_CH{
    ADC_IBUS,
    ADC_VBUS,
    ADC_VUSB,
    ADC_VWPC,
    ADC_VOUT,
    ADC_VBAT,
    ADC_IBAT,
    ADC_RSV,
    ADC_TDIE,
    ADC_MAX_NUM,
};

enum {
	SC8562_STDALONE,
	SC8562_SLAVE,
	SC8562_MASTER,
};

enum {
	SC8562_ROLE_STDALONE,
	SC8562_ROLE_SLAVE,
	SC8562_ROLE_MASTER,
};

struct sc8562_cfg {
	const char *chg_name;
	bool bat_ovp_disable;
	bool bat_ocp_disable;
	bool bus_ocp_disable;
	bool bus_ucp_disable;
	bool pmid2out_ovp_disable;
	bool pmid2out_uvp_disable;
	bool vbat_ovp_alarm_disable;
	bool bus_ocp_alarm_disable;
	int bat_ovp_th;
	int bat_ocp_th;
	int bat_ovp_alm_th;
	int bus_ovp_th;
	int bus_ocp_th;
	int usb_ovp_th;
	int wpc_ovp_th;
	int out_ovp_th;
	int bus_ocp_alm_th;
	int pmid2out_ovp_th;
	int pmid2out_uvp_th;
	int tsbat_dis;
	int sense_r_mohm;
};
struct sc8562 {
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
	bool batt_present;
	bool vbus_present;
	bool vusb_present;
	bool vwpc_present;
	bool usb_present;
	bool qb_enabled;
	bool charge_enabled;	/* Register bit status */
	bool ovpgate_state;
	bool wpcgate_state;
	int work_mode;
	int vbus_error;
	int rst_gpio;
	int sc_en_gpio;
	int sc_lpm_gpio;
	int ovp_en_gpio;
	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vusb_volt;
	int vwpc_volt;
	int ibat_curr;
	int ibus_curr;
	int bat_temp;
	int die_temp;
	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool usb_ovp_fault;
	bool wpc_ovp_fault;
	bool bus_ocp_fault;
	bool bus_ucp_fault;
	bool bat_ovp_alarm;
	bool bus_ocp_alarm;
	int chg_ma;
	int chg_mv;
	int charge_state;
	struct sc8562_cfg *cfg;
	int skip_writes;
	int skip_reads;
	struct charger_device *chg_dev;
	struct charger_properties chg_prop;
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *fc2_psy;
};
/*********************************************************************/
static const u32 sc8562_adc_accuracy_tbl[ADC_MAX_NUM] = {
150000,/* IBUS */
100000,/* VBUS */
100000,/* VAC */
100000,/* WPC */
22000,/* VOUT */
22000,/* VBAT */
250000,/* IBAT */
250000,/* RSV */
7/* TDIE */
};

static int sc8562_mode_data[] = {
	[SC8562_STDALONE] = SC8562_ROLE_STDALONE,
	[SC8562_MASTER] = SC8562_ROLE_MASTER,
	[SC8562_SLAVE] = SC8562_ROLE_SLAVE,
};
/*********************************************************************/
#define	VOUT_INSERT         				BIT(3)
#define	VBUS_INSERT         				BIT(2)
#define VWPC_INSERT         				BIT(1)
#define VUSB_INSERT         				BIT(0)

#define	BAT_OVP_FAULT_SHIFT         (0)
#define	BAT_OCP_FAULT_SHIFT         (1)
#define	USB_OVP_FAULT_SHIFT         (2)
#define	WPC_OVP_FAULT_SHIFT         (3)
#define	BUS_OCP_FAULT_SHIFT         (4)
#define	BUS_UCP_FAULT_SHIFT         (5)
#define BAT_OVP_ALARM_SHIFT         (0)
#define BUS_OCP_ALARM_SHIFT         (1)
#define SC_I2C_RETRIES              (5)
#define SC_I2C_RETRY_DELAY          (50)
/*********************************************************************/
#define sc_err(fmt, ...)   pr_err("%s:" fmt,__func__, ##__VA_ARGS__);
#define sc_info(fmt, ...)  pr_info("%s:" fmt,__func__, ##__VA_ARGS__);
#define sc_dbg(fmt, ...)   pr_debug("%s:" fmt,__func__, ##__VA_ARGS__);
/************************************************************************/
static int __sc8562_read_byte(struct sc8562 *sc, u8 reg, u8 *data)
{
	s32 ret = 0;

	ret = i2c_smbus_read_byte_data(sc->client, reg);
	if (ret < 0) {
		sc_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}
	*data = (u8)ret;
	return ret;
}
/*********************************************************************/
static int __sc8562_write_byte(struct sc8562 *sc, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(sc->client, reg, val);
	if (ret < 0) {
		sc_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",val, reg, ret);
		return ret;
	}
	return ret;
}
/*********************************************************************/
static int sc8562_read_byte(struct sc8562 *sc, u8 reg, u8 *data)
{
	int ret;

	if (sc->skip_reads) {
		*data = 0;
		return 0;
	}
	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8562_read_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}
/*********************************************************************/
static int sc8562_write_byte(struct sc8562 *sc, u8 reg, u8 data)
{
	int ret;

	if (sc->skip_writes)
		return 0;
	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8562_write_byte(sc, reg, data);
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}
/*********************************************************************/
static int sc8562_update_bits(struct sc8562*sc, u8 reg,
                    u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if (sc->skip_reads || sc->skip_writes)
		return 0;

	mutex_lock(&sc->i2c_rw_lock);
	ret = __sc8562_read_byte(sc, reg, &tmp);
	if (ret < 0) {
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __sc8562_write_byte(sc, reg, tmp);
	if (ret < 0)
		sc_err("Failed: reg=%02X, ret=%d\n", reg, ret);
out:
	mutex_unlock(&sc->i2c_rw_lock);
	return ret;
}

/*********************************************************************/
static int sc8562_enable_batovp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BAT_OVP_ENABLE;
	else
		val = SC8562_BAT_OVP_DISABLE;
	val <<= SC8562_BAT_OVP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_01,	SC8562_BAT_OVP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_batovp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8562_BAT_OVP_BASE)
		threshold = SC8562_BAT_OVP_BASE;
	val = (threshold - SC8562_BAT_OVP_BASE) / SC8562_BAT_OVP_LSB;
	val <<= SC8562_BAT_OVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_01,	SC8562_BAT_OVP_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_batocp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BAT_OCP_ENABLE;
	else
		val = SC8562_BAT_OCP_DISABLE;
	val <<= SC8562_BAT_OCP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_02,	SC8562_BAT_OCP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_batocp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8562_BAT_OCP_BASE)
		threshold = SC8562_BAT_OCP_BASE;

	val = (threshold - SC8562_BAT_OCP_BASE) / SC8562_BAT_OCP_LSB;
	val <<= SC8562_BAT_OCP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_02,	SC8562_BAT_OCP_MASK, val);
	return ret;
}
/*********************************************************************/

static int sc8562_set_usbovp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold == 6500)
		val = SC8562_USB_OVP_6PV5;
	else
		val = (threshold - SC8562_USB_OVP_BASE) / SC8562_USB_OVP_LSB;

	val <<= SC8562_USB_OVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_03,	SC8562_USB_OVP_MASK, val);
	return ret;
}
/*********************************************************************/

static int sc8562_set_wpcovp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold == 6500)
		val = SC8562_WPC_OVP_6PV5;
	else
		val = (threshold - SC8562_WPC_OVP_BASE) / SC8562_WPC_OVP_LSB;

	val <<= SC8562_WPC_OVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_04,	SC8562_WPC_OVP_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_busovp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (sc->work_mode == SC8562_FORWARD_4_1_CHARGER_MODE
		|| sc->work_mode == SC8562_REVERSE_1_4_CONVERTER_MODE) {
		if (threshold < 14000)
			threshold = 14000;
		else if (threshold > 22000)
			threshold = 22000;
		val = (threshold - SC8562_BUS_OVP_41MODE_BASE) / SC8562_BUS_OVP_41MODE_LSB;
	}	else if (sc->work_mode == SC8562_FORWARD_2_1_CHARGER_MODE
		|| sc->work_mode == SC8562_REVERSE_1_2_CONVERTER_MODE) {
		if (threshold < 7000)
			threshold = 7000;
		else if (threshold > 11000)
			threshold = 11000;
		val = (threshold - SC8562_BUS_OVP_21MODE_BASE) / SC8562_BUS_OVP_21MODE_LSB;
	}else {
		if (threshold < 3500)
			threshold = 3500;
		else if (threshold > 5500)
			threshold = 5500;
		val = (threshold - SC8562_BUS_OVP_11MODE_BASE) / SC8562_BUS_OVP_11MODE_LSB;
	}

	val <<= SC8562_BUS_OVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_05, SC8562_BUS_OVP_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_outovp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < 4800)
		threshold = 4800;
	val = (threshold - SC8562_OUT_OVP_BASE) / SC8562_OUT_OVP_LSB;
	val <<= SC8562_OUT_OVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_05,	SC8562_OUT_OVP_MASK, val);
	return ret;
}
/*********************************************************************/

static int sc8562_enable_busocp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BUS_OCP_ENABLE;
	else
		val = SC8562_BUS_OCP_DISABLE;
	val <<= SC8562_BUS_OCP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_06,	SC8562_BUS_OCP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_busocp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8562_BUS_OCP_BASE)
		threshold = SC8562_BUS_OCP_BASE;
	val = (threshold - SC8562_BUS_OCP_BASE) / SC8562_BUS_OCP_LSB;
	val <<= SC8562_BUS_OCP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_06,	SC8562_BUS_OCP_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_busucp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BUS_UCP_ENABLE;
	else
		val = SC8562_BUS_UCP_DISABLE;
	val <<= SC8562_BUS_UCP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_07,	SC8562_BUS_UCP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_pmid2outovp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_PMID2OUT_OVP_ENABLE;
	else
		val = SC8562_PMID2OUT_OVP_DISABLE;
	val <<= SC8562_PMID2OUT_OVP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_08,	SC8562_PMID2OUT_OVP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_pmid2outuvp(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_PMID2OUT_UVP_ENABLE;
	else
		val = SC8562_PMID2OUT_UVP_DISABLE;
	val <<= SC8562_PMID2OUT_UVP_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_09,	SC8562_PMID2OUT_UVP_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_pmid2outuvp_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8562_PMID2OUT_UVP_BASE)
		threshold = SC8562_PMID2OUT_UVP_BASE;
	val = (threshold - SC8562_PMID2OUT_UVP_BASE) / SC8562_PMID2OUT_UVP_LSB;
	val <<= SC8562_PMID2OUT_UVP_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_09,	SC8562_PMID2OUT_UVP_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_tsbat_flt_dis(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_TSBAT_FLT_DIS_DISABLE;
	else
		val = SC8562_TSBAT_FLT_DIS_ENABLE;
	val <<= SC8562_TSBAT_FLT_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_70,	SC8562_TSBAT_FLT_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_charge(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_CHG_ENABLE;
	else
		val = SC8562_CHG_DISABLE;
	val <<= SC8562_CHG_EN_SHIFT;
	sc_err("sc8562 charger %s\n", enable == false ? "disable" : "enable");
	ret = sc8562_update_bits(sc, SC8562_REG_0B, SC8562_CHG_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_acdrv_manual(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_ACDRV_MANUAL_MODE;
	else
		val = SC8562_ACDRV_AUTO_MODE;
	val <<= SC8562_ACDRV_MANUAL_EN_SHIFT;

	sc_err("sc8562 acdrv_manual %s\n", enable == false ? "Auto mode" : "Manual mode");
	ret = sc8562_update_bits(sc, SC8562_REG_0B, SC8562_ACDRV_MANUAL_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_wpcgate(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_WPCGATE_ENABLE;
	else
		val = SC8562_WPCGATE_DISABLE;
	val <<= SC8562_WPCGATE_EN_SHIFT;
	sc->wpcgate_state = enable;
	sc_err("sc8562 wpc gate %s\n", enable == false ? "disable" : "enable");
	ret = sc8562_update_bits(sc, SC8562_REG_0B,	SC8562_WPCGATE_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_ovpgate(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_OVPGATE_ENABLE;
	else
		val = SC8562_OVPGATE_DISABLE;
	val <<= SC8562_OVPGATE_EN_SHIFT;
	sc->ovpgate_state = enable;
	sc_err("sc8562 ovp gate %s\n", enable == false ? "disable" : "enable");
	ret = sc8562_update_bits(sc, SC8562_REG_0B,	SC8562_OVPGATE_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_ss_timeout(struct sc8562 *sc, int timeout)
{
	int ret;
	u8 val;

	switch (timeout) {
	case 0:
		val = SC8562_SS_TIMEOUT_DISABLE;
		break;
	case 40:
		val = SC8562_SS_TIMEOUT_40MS;
		break;
	case 80:
		val = SC8562_SS_TIMEOUT_80MS;
		break;
	case 320:
		val = SC8562_SS_TIMEOUT_320MS;
		break;
	case 1280:
		val = SC8562_SS_TIMEOUT_1280MS;
		break;
	case 5120:
		val = SC8562_SS_TIMEOUT_5120MS;
		break;
	case 20480:
		val = SC8562_SS_TIMEOUT_20480MS;
		break;
	case 81920:
		val = SC8562_SS_TIMEOUT_81920MS;
		break;
	default:
		val = SC8562_SS_TIMEOUT_DISABLE;
		break;
	}
	val <<= SC8562_SS_TIMEOUT_SET_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0D,	SC8562_SS_TIMEOUT_SET_MASK,	val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_wdt(struct sc8562 *sc, int ms)
{
	int ret;
	u8 val;

	switch (ms) {
	case 0:
		val = SC8562_WD_TIMEOUT_DISABLE;
		break;
	case 200:
		val = SC8562_WD_TIMEOUT_0P2S;
		break;
	case 500:
		val = SC8562_WD_TIMEOUT_0P5S;
		break;
	case 1000:
		val = SC8562_WD_TIMEOUT_1S;
		break;
	case 5000:
		val = SC8562_WD_TIMEOUT_5S;
		break;
	case 30000:
		val = SC8562_WD_TIMEOUT_30S;
		break;
	case 100000:
		val = SC8562_WD_TIMEOUT_100S;
		break;
	case 255000:
		val = SC8562_WD_TIMEOUT_255S;
		break;
	default:
		val = SC8562_WD_TIMEOUT_DISABLE;
		break;
	}

	val <<= SC8562_WD_TIMEOUT_SET_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0D,	SC8562_WD_TIMEOUT_SET_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_sense_resistor(struct sc8562 *sc, int r_mohm)
{
	int ret;
	u8 val;

	if (r_mohm == 1)
		val = SC8562_IBAT_SNS_RES_1MHM;
	else if (r_mohm == 2)
		val = SC8562_IBAT_SNS_RES_2MHM;
	else
		return -EINVAL;

	val <<= SC8562_IBAT_SNS_RES_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0E,	SC8562_IBAT_SNS_RES_MASK,	val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_reg_reset(struct sc8562 *sc)
{
	int ret;
	u8 val = 1;

	val = SC8562_REG_RESET;
	val <<= SC8562_REG_RST_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0E,	SC8562_REG_RST_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_operation_mode(struct sc8562 *sc, int operation_mode)
{
	int ret;
	u8 val = 1;

	switch (operation_mode) {
	case SC8562_FORWARD_4_1_CHARGER_MODE:
		val = SC8562_FORWARD_4_1_CHARGER_MODE;
		break;
	case SC8562_FORWARD_2_1_CHARGER_MODE:
		val = SC8562_FORWARD_2_1_CHARGER_MODE;
		break;
	case SC8562_FORWARD_1_1_CHARGER_MODE:
	case SC8562_FORWARD_1_1_CHARGER_MODE1:
		val = SC8562_FORWARD_1_1_CHARGER_MODE;
		break;
	case SC8562_REVERSE_1_4_CONVERTER_MODE:
		val = SC8562_REVERSE_1_4_CONVERTER_MODE;
		break;
	case SC8562_REVERSE_1_2_CONVERTER_MODE:
		val = SC8562_REVERSE_1_2_CONVERTER_MODE;
		break;
	case SC8562_REVERSE_1_1_CONVERTER_MODE:
	case SC8562_REVERSE_1_1_CONVERTER_MODE1:
		val = SC8562_REVERSE_1_1_CONVERTER_MODE;
		break;
	default:
		sc_err("sc8562 set operation mode fail : not have this mode!\n");
		return -1;
		break;
	}
	sc->work_mode = operation_mode;
	sc_info("sc->work_mode:%d\n", sc->work_mode);
	val <<= SC8562_MODE_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0E,	SC8562_MODE_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_adc(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_ADC_ENABLE;
	else
		val = SC8562_ADC_DISABLE;
	val <<= SC8562_ADC_EN_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_15,	SC8562_ADC_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_adc_scanrate(struct sc8562 *sc, bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = SC8562_ADC_RATE_ONESHOT;
	else
		val = SC8562_ADC_RATE_CONTINOUS;
	val <<= SC8562_ADC_RATE_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_15,	SC8562_ADC_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_get_adc_data(struct sc8562 *sc, int channel,  int *result)
{
	int ret;
	u8 val_l, val_h;
	int val;

	if(channel >= ADC_MAX_NUM) return 0;

	ret = sc8562_read_byte(sc, SC8562_REG_17 + (channel << 1), &val_h);
	ret = sc8562_read_byte(sc, SC8562_REG_17 + (channel << 1) + 1, &val_l);

	if (ret < 0)
		return ret;
	val = (val_h << 8) | val_l;
	if(channel == ADC_IBUS)				val = val * SC8562_IBUS_ADC_LSB;
	else if(channel == ADC_VBUS)		val = val * SC8562_VBUS_ADC_LSB;
	else if(channel == ADC_VUSB)		val = val * SC8562_VUSB_ADC_LSB;
	else if(channel == ADC_VWPC)		val = val * SC8562_VWPC_ADC_LSB;
	else if(channel == ADC_VOUT)		val = val * SC8562_VOUT_ADC_LSB;
	else if(channel == ADC_VBAT)		val = val * SC8562_VBAT_ADC_LSB;
	else if(channel == ADC_IBAT)		val = val * SC8562_IBAT_ADC_LSB;
	else if(channel == ADC_TDIE)		val = 25;
	*result = val;
	return ret;
}
/*********************************************************************/
static int sc8562_set_adc_scan(struct sc8562 *sc, int channel, bool enable)
{
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;
	if (channel == ADC_IBUS) {
		reg = SC8562_REG_15;
		shift = SC8562_IBUS_ADC_DIS_SHIFT;
		mask = SC8562_IBUS_ADC_DIS_MASK;
	} else {
		reg = SC8562_REG_16;
		shift = 8 - channel;
		mask = 1 << shift;
	}
	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;
	ret = sc8562_update_bits(sc, reg, mask, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_batovp_alarm(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BAT_OVP_ALM_ENABLE;
	else
		val = SC8562_BAT_OVP_ALM_DISABLE;
	val <<= SC8562_BAT_OVP_ALM_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_6C,	SC8562_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}
/*********************************************************************/

static int sc8562_set_batovp_alarm_th(struct sc8562 *sc, int threshold)
{
	int ret;
	u8 val;

	if (threshold < SC8562_BAT_OVP_ALM_BASE)
		threshold = SC8562_BAT_OVP_ALM_BASE;

	val = (threshold - SC8562_BAT_OVP_ALM_BASE) / SC8562_BAT_OVP_ALM_LSB;
	val <<= SC8562_BAT_OVP_ALM_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_6C,	SC8562_BAT_OVP_ALM_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_busocp_alarm(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_BUS_OCP_ALM_ENABLE;
	else
		val = SC8562_BUS_OCP_ALM_DISABLE;

	val <<= SC8562_BUS_OCP_ALM_DIS_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_6D,	SC8562_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_batovp_alarm_int_mask(struct sc8562 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8562_read_byte(sc, SC8562_REG_6C, &val);
	if (ret < 0)
		return ret;
	val |= (mask << SC8562_BAT_OVP_ALM_MASK_SHIFT);
	ret = sc8562_write_byte(sc, SC8562_REG_6C, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_busocp_alarm_int_mask(struct sc8562 *sc, u8 mask)
{
	int ret;
	u8 val;

	ret = sc8562_read_byte(sc, SC8562_REG_6D, &val);
	if (ret < 0)
		return ret;
	val |= (mask << SC8562_BUS_OCP_ALM_MASK_SHIFT);
	ret = sc8562_write_byte(sc, SC8562_REG_6D, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_parallel_func(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_SYNC_FUNCTION_ENABLE;
	else
		val = SC8562_SYNC_FUNCTION_DISABLE;

	val <<= SC8562_SYNC_FUNCTION_EN_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0E, SC8562_SYNC_FUNCTION_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_enable_config_func(struct sc8562 *sc, bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = SC8562_SYNC_CONFIG_MASTER;
	else
		val = SC8562_SYNC_CONFIG_SLAVE;
	val <<= SC8562_SYNC_MASTER_EN_SHIFT;
	ret = sc8562_update_bits(sc, SC8562_REG_0E,	SC8562_SYNC_MASTER_EN_MASK, val);
	return ret;
}
/*********************************************************************/
static int sc8562_set_work_mode(struct sc8562 *sc)
{
	int ret;

	if (sc->mode == SC8562_ROLE_STDALONE) {
		ret = sc8562_enable_parallel_func(sc, false);
		if (ret) {
			sc_err("Failed to write sync function register\n");
			return ret;
		}
	}else if (sc->mode == SC8562_ROLE_MASTER) {
		ret = sc8562_enable_parallel_func(sc, true);
		if (ret) {
			sc_err("Failed to write sync function register\n");
			return ret;
		}
		ret = sc8562_enable_config_func(sc, true);
		if (ret) {
			sc_err("Failed to write sync config register\n");
			return ret;
		}
	}else{
		ret = sc8562_enable_parallel_func(sc, true);
		if (ret) {
			sc_err("Failed to write sync function register\n");
			return ret;
		}
		ret = sc8562_enable_config_func(sc, false);
		if (ret) {
			sc_err("Failed to write sync config register\n");
			return ret;
		}
	}
	sc_info("work mode:%s\n", sc->mode == SC8562_ROLE_STDALONE ? "Standalone" :
					(sc->mode == SC8562_ROLE_SLAVE ? "Slave" : "Master"));
	return ret;
}
/*********************************************************************/
static int sc8562_check_vbus_error_status(struct sc8562 *sc)
{
	int ret;
	u8 data;

	ret = sc8562_read_byte(sc, SC8562_REG_0A, &data);
	if(ret < 0){
		sc_err("vbus error >>>>%02x\n", data);
		sc->vbus_error = data;
	}
	return ret;
}
/*********************************************************************/
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
static void sc8562_fill_hardwareinfo(int id)
{
	sprintf(current_cp_info.chip,"SC8562");
	sprintf(current_cp_info.id,"0x%04x",id);
	strcpy(current_cp_info.vendor, "Southchip");
	sprintf(current_cp_info.more, "charger pump");
}
#endif
/*********************************************************************/
static int sc8562_detect_device(struct sc8562 *sc)
{
	int ret;
	u8 data;
	u8 cnt = 0;

	while (cnt++ < SC_I2C_RETRIES) {
		ret = sc8562_read_byte(sc, SC8562_REG_6E, &data);
		if (ret > 0) {
			break;
		}
		mdelay(SC_I2C_RETRY_DELAY);
		sc_info("CHIP_ID :cnt %d\n", cnt);
	}
	if (data != SC8562_DEVICE_ID)
		return -ENAVAIL;
	sc->revision = SC8562_DEVICE_ID;
#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_INFO)
	sc8562_fill_hardwareinfo(SC8562_DEVICE_ID);
#endif
	sc_info("CHIP_ID : %02x\n", SC8562_DEVICE_ID);
	return 0;
}
/*********************************************************************/
static int sc8562_parse_dt(struct sc8562 *sc, struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node;

	sc->cfg = devm_kzalloc(dev, sizeof(struct sc8562_cfg),
	            GFP_KERNEL);
	if (!sc->cfg)
		return -ENOMEM;
	ret = of_property_read_string(np, "chg_name", &sc->cfg->chg_name);
	if(ret) {
		sc_err("%s no chg name, use default\n", __func__);
		sc->cfg->chg_name = "primary_dvchg";
	}
	sc->cfg->bat_ovp_disable = of_property_read_bool(np,
	    "sc,sc8562,bat-ovp-disable");
	sc->cfg->bat_ocp_disable = of_property_read_bool(np,
	    "sc,sc8562,bat-ocp-disable");
	sc->cfg->bus_ocp_disable = of_property_read_bool(np,
	    "sc,sc8562,bus-ocp-disable");
	sc->cfg->bus_ucp_disable = of_property_read_bool(np,
	    "sc,sc8562,bus-ucp-disable");
	sc->cfg->pmid2out_ovp_disable = of_property_read_bool(np,
	    "sc,sc8562,pmid2out-ovp-disable");
	sc->cfg->pmid2out_uvp_disable = of_property_read_bool(np,
	    "sc,sc8562,pmid2out-uvp-disable");
	sc->cfg->vbat_ovp_alarm_disable = of_property_read_bool(np,
	"sc,sc8562,bat-ovp-alarm-disable");
	sc->cfg->bus_ocp_alarm_disable = of_property_read_bool(np,
	"sc,sc8562,bus-ocp-alarm-disable");

	sc->rst_gpio = of_get_named_gpio(np, "sc,sc8562,rst-gpio", 0);
	if (!gpio_is_valid(sc->rst_gpio)) {
		sc_err("fail to valid gpio : %d\n", sc->rst_gpio);
		return -EINVAL;
	}
	gpio_set_value(sc->rst_gpio, 1);

	sc->irq_gpio = of_get_named_gpio(np, "sc,sc8562,irq-gpio", 0);
	if (!gpio_is_valid(sc->irq_gpio)) {
		sc_err("fail to valid gpio : %d\n", sc->irq_gpio);
		return -EINVAL;
	}

	sc->sc_lpm_gpio = of_get_named_gpio(np, "sc,sc8562,sc-lpm-gpio", 0);
	if (!gpio_is_valid(sc->sc_lpm_gpio)) {
		sc_err("fail to valid gpio : %d\n", sc->sc_lpm_gpio);
		return -EINVAL;
	}
	gpio_set_value(sc->sc_lpm_gpio, 1);

	sc->sc_en_gpio = of_get_named_gpio(np, "sc,sc8562,sc-en-gpio", 0);
	if (!gpio_is_valid(sc->sc_en_gpio)) {
		sc_err("fail to valid gpio : %d\n", sc->sc_en_gpio);
		return -EINVAL;
	}
	gpio_set_value(sc->sc_en_gpio, 1);

	sc->ovp_en_gpio = of_get_named_gpio(np, "sc,sc8562,ovp-en-gpio", 0);
	if (!gpio_is_valid(sc->ovp_en_gpio)) {
		sc_err("fail to valid gpio : %d\n", sc->ovp_en_gpio);
		return -EINVAL;
	}
	gpio_set_value(sc->ovp_en_gpio, 0);

	ret = of_property_read_u32(np, "sc,sc8562,bat-ovp-threshold",
				&sc->cfg->bat_ovp_th);
	if (ret) {
		sc_err("failed to read bat-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,bat-ocp-threshold",
				&sc->cfg->bat_ocp_th);
	if (ret) {
		sc_err("failed to read bat-ocp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,bat-ovp-alarm-threshold",
				&sc->cfg->bat_ovp_alm_th);
	if (ret) {
		sc_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,bus-ovp-threshold",
				&sc->cfg->bus_ovp_th);
	if (ret) {
		sc_err("failed to read bus-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,bus-ocp-threshold",
				&sc->cfg->bus_ocp_th);
	if (ret) {
		sc_err("failed to read bus-ocp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,usb-ovp-threshold",
				&sc->cfg->usb_ovp_th);
	if (ret) {
		sc_err("failed to read usb-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,wpc-ovp-threshold",
				&sc->cfg->wpc_ovp_th);
	if (ret) {
		sc_err("failed to read wpc-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,out-ovp-threshold",
				&sc->cfg->out_ovp_th);
	if (ret) {
		sc_err("failed to read out-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,bus-ocp-alarm-threshold",
				&sc->cfg->bus_ocp_alm_th);
	if (ret) {
		sc_err("failed to read sc8562,bus-ocp-alarm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,pmid2out-ovp-threshold",
				&sc->cfg->pmid2out_ovp_th);
	if (ret) {
		sc_err("failed to read sc8562,pmid2out-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,pmid2out-uvp-threshold",
				&sc->cfg->pmid2out_uvp_th);
	if (ret) {
		sc_err("failed to read sc8562,pmid2out-uvp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,tsbat_dis",
				&sc->cfg->tsbat_dis);
	if (ret) {
		sc_err("failed to read sc8562,tsbat_dis\n");
		return ret;
	}

	ret = of_property_read_u32(np, "sc,sc8562,sense-r-mohm",
				&sc->cfg->sense_r_mohm);
	if (ret) {
		sc_err("failed to read sc8562,sense-r-mohm\n");
		return ret;
	}

	return 0;
}
/*********************************************************************/
static int sc8562_init_protection(struct sc8562 *sc)
{
	int ret;

	ret = sc8562_enable_batovp(sc, !sc->cfg->bat_ovp_disable);
	sc_info("%s bat ovp %s\n",sc->cfg->bat_ovp_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_batocp(sc, !sc->cfg->bat_ocp_disable);
	sc_info("%s bat ocp %s\n",sc->cfg->bat_ocp_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_busocp(sc, !sc->cfg->bus_ocp_disable);
					sc_info("%s bus ocp %s\n",sc->cfg->bus_ocp_disable ? "disable" : "enable",
	!ret ? "successfullly" : "failed");

	ret = sc8562_enable_busucp(sc, !sc->cfg->bus_ucp_disable);
	sc_info("%s bus ucp %s\n",sc->cfg->bus_ucp_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_pmid2outovp(sc, !sc->cfg->pmid2out_ovp_disable);
	sc_info("%s pmid2out ovp %s\n",sc->cfg->pmid2out_ovp_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_pmid2outuvp(sc, !sc->cfg->pmid2out_uvp_disable);
	sc_info("%s pmid2out uvp %s\n",sc->cfg->pmid2out_uvp_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_batovp_alarm(sc, !sc->cfg->vbat_ovp_alarm_disable);
	sc_info("%s bat ovp alarm %s\n",sc->cfg->vbat_ovp_alarm_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_enable_busocp_alarm(sc, !sc->cfg->bus_ocp_alarm_disable);
	sc_info("%s bus ocp alarm %s\n",sc->cfg->bus_ocp_alarm_disable ? "disable" : "enable",
					!ret ? "successfullly" : "failed");

	ret = sc8562_set_batovp_th(sc, sc->cfg->bat_ovp_th);
	sc_info("set bat ovp th %d %s\n", sc->cfg->bat_ovp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_batocp_th(sc, sc->cfg->bat_ocp_th);
	sc_info("set bat ocp threshold %d %s\n", sc->cfg->bat_ocp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_batovp_alarm_th(sc, sc->cfg->bat_ovp_alm_th);
	sc_info("set bat ovp alarm threshold %d %s\n", sc->cfg->bat_ovp_alm_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_busovp_th(sc, sc->cfg->bus_ovp_th);
	sc_info("set bus ovp threshold %d %s\n", sc->cfg->bus_ovp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_busocp_th(sc, sc->cfg->bus_ocp_th);
	sc_info("set bus ocp threshold %d %s\n", sc->cfg->bus_ocp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_usbovp_th(sc, sc->cfg->usb_ovp_th);
	sc_info("set usb ovp threshold %d %s\n", sc->cfg->usb_ovp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_wpcovp_th(sc, sc->cfg->wpc_ovp_th);
	sc_info("set wpc ovp threshold %d %s\n", sc->cfg->wpc_ovp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_outovp_th(sc, sc->cfg->out_ovp_th);
	sc_info("set out ovp threshold %d %s\n", sc->cfg->out_ovp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_pmid2outuvp_th(sc, sc->cfg->pmid2out_uvp_th);
	sc_info("set pmid2out uvp threshold %d %s\n", sc->cfg->pmid2out_uvp_th,!ret ? "successfully" : "failed");

	ret = sc8562_set_tsbat_flt_dis(sc, sc->cfg->tsbat_dis);
	sc_info("set tsbat %d %s\n", sc->cfg->tsbat_dis,!ret ? "successfully" : "failed");
	return ret;
}
/*********************************************************************/
static int sc8562_get_ovpgate_state(struct sc8562 *sc, bool *enable)
{
	int ret;
	u8 val;

	ret = sc8562_read_byte(sc, SC8562_REG_0F, &val);
	if (ret < 0) {
		sc_err("sc8562 get ovpgate state fail\n");
		return -1;
	}
	*enable = !!(val & SC8562_OVPGATE_STAT_MASK);
	return ret;
}
/*********************************************************************/
static int sc8562_get_wpcgate_state(struct sc8562 *sc, bool *enable)
{
	int ret;
	u8 val;

	ret = sc8562_read_byte(sc, SC8562_REG_0F, &val);
	if (ret < 0) {
		sc_err("sc8562 get ovpgate state fail\n");
		return -1;
	}
	*enable = !!(val & SC8562_WPCGATE_STAT_MASK);
	return ret;
}
/*********************************************************************/
static int sc8562_init_adc(struct sc8562 *sc)
{
	sc8562_set_adc_scanrate(sc, false);
	sc8562_set_adc_scan(sc, ADC_IBUS, true);
	sc8562_set_adc_scan(sc, ADC_VBUS, true);
	sc8562_set_adc_scan(sc, ADC_VUSB, true);
	sc8562_set_adc_scan(sc, ADC_VWPC, true);
	sc8562_set_adc_scan(sc, ADC_VOUT, true);
	sc8562_set_adc_scan(sc, ADC_VBAT, true);
	sc8562_set_adc_scan(sc, ADC_IBAT, true);
	sc8562_set_adc_scan(sc, ADC_TDIE, true);
	sc8562_enable_adc(sc, true);
	return 0;
}
/*********************************************************************/
static int sc8562_init_int_src(struct sc8562 *sc)
{
	int ret;
	/*TODO:be careful ts bus and ts bat alarm bit mask is in
	*	fault mask register, so you need call
	*	sc8562_set_fault_int_mask for tsbus and tsbat alarm
	*/
	ret = sc8562_set_batovp_alarm_int_mask(sc, SC8562_BAT_OVP_ALM_NOT_MASK);
	if (ret < 0) {
		sc_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
	ret = sc8562_set_busocp_alarm_int_mask(sc, SC8562_BUS_OCP_ALM_NOT_MASK);
	if (ret < 0) {
		sc_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
	return ret;
}
/*********************************************************************/
static int sc8562_init_device(struct sc8562 *sc)
{
	sc8562_set_reg_reset(sc);
	sc8562_set_wdt(sc, 0);// disable watch dog
	sc8562_enable_acdrv_manual(sc, false);//ac drive auto mode
	sc8562_enable_wpcgate(sc, false);
	sc8562_enable_ovpgate(sc, true);
	sc8562_set_ss_timeout(sc, 5120);
	sc8562_set_sense_resistor(sc, sc->cfg->sense_r_mohm);
	sc8562_init_protection(sc);
	sc8562_init_adc(sc);
	sc8562_init_int_src(sc);
	sc8562_set_operation_mode(sc, SC8562_FORWARD_4_1_CHARGER_MODE);
	return 0;
}
/*********************************************************************/
static void sc8562_sc_en_pin_config(struct sc8562 *sc, u8 value)
{
	gpio_set_value(sc->sc_en_gpio, value);
}
/***********************MTK CHARGER OPS API START************************/

static int sc8562_enable_chg(struct charger_device *chg_dev, bool en)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	sc_info("%s: %s\n", __func__, en ? "enable" : "disable");
	sc8562_sc_en_pin_config(sc,(u8)en);
	return sc8562_enable_charge(sc, !!en);
}
/*********************************************************************/
static int sc8562_check_charge_enabled(struct sc8562 *sc, bool *enabled)
{
	int ret;
	u8 val;

	ret = sc8562_read_byte(sc, SC8562_REG_0B, &val);
	if (!ret)
		*enabled = !!(val & SC8562_CHG_EN_MASK);
	return ret;
}
/*********************************************************************/
static int sc8562_is_chg_enabled(struct charger_device *chg_dev, bool *en)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	return sc8562_check_charge_enabled(sc, en);
}
/*********************************************************************/
static inline enum ADC_CH to_sc8562_adc(enum adc_channel chan)
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
static int sc8562_get_adc(struct charger_device *chg_dev, enum adc_channel chan,
			  int *min, int *max)
{
	int ret;
	struct sc8562 *sc = charger_get_data(chg_dev);
	enum ADC_CH _chan = to_sc8562_adc(chan);

	if (_chan == ADC_MAX_NUM)
		return -EINVAL;
	ret = sc8562_get_adc_data(sc, _chan, max);
	if (ret < 0)
		return ret;
	if (min != max)
		*min = *max;
	sc_info("get_adc: chan:%d ,value:%d (ma/mv)\n",_chan, *max/1000);
	return ret;
}
/*********************************************************************/
static int sc8562_get_adc_accuracy(struct charger_device *chg_dev,
				   enum adc_channel chan, int *min, int *max)
{
	enum ADC_CH _chan = to_sc8562_adc(chan);

	if (_chan == ADC_MAX_NUM)
		return -EINVAL;
	*min = *max = sc8562_adc_accuracy_tbl[_chan];
	return 0;
}
/*********************************************************************/
static int sc8562_set_vbusovp(struct charger_device *chg_dev, u32 uV)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	sc_info("%s: %d uV\n", __func__, uV)
	return sc8562_set_busovp_th(sc, uV / 1000);
}
/*********************************************************************/
static int sc8562_set_ibusocp(struct charger_device *chg_dev, u32 uA)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	sc_info("%s: %d uA\n", __func__, uA)
	return sc8562_set_busocp_th(sc, uA / 1000);
}
/*********************************************************************/
static int sc8562_set_vbatovp(struct charger_device *chg_dev, u32 uV)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	sc_info("%s: %d uV\n", __func__, uV)
	return sc8562_set_batovp_th(sc, uV / 1000);
}
/*********************************************************************/
static int sc8562_set_ibatocp(struct charger_device *chg_dev, u32 uA)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	sc_info("%s: %d uA\n", __func__, uA)
	return sc8562_set_batocp_th(sc, uA / 1000);
}
/*********************************************************************/
static int sc8562_init_chip(struct charger_device *chg_dev)
{
	struct sc8562 *sc = charger_get_data(chg_dev);

	return sc8562_init_device(sc);
}
/*********************************************************************/
static int sc8562_set_vbatovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}
/*********************************************************************/
static int sc8562_reset_vbatovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}
/*********************************************************************/
static int sc8562_set_vbusovp_alarm(struct charger_device *chg_dev, u32 uV)
{
	return 0;
}
/*********************************************************************/
static int sc8562_reset_vbusovp_alarm(struct charger_device *chg_dev)
{
	return 0;
}
/*********************************************************************/
static int sc8562_is_vbuslowerr(struct charger_device *chg_dev, bool *err)
{
	int ret;
	struct sc8562 *sc = charger_get_data(chg_dev);

	ret = sc8562_check_vbus_error_status(sc);
	if (ret < 0)
		return ret;
	sc_info("%s: reg val = 0x%02x\n", __func__, sc->vbus_error);
	*err = !!(sc->vbus_error & SC8562_VBUS_ERRORLO_STAT_MASK);
	return ret;
}
/*********************************************************************/
static int sc8562_dump_registers(struct charger_device *chg_dev)
{
	int ret;
	u8 addr;
	u8 val;
	struct sc8562 *sc = charger_get_data(chg_dev);

	for (addr = 0x0; addr <= 0x6e; addr++) {
		if (addr <= 0x29 || addr >= 0x6c) {
			ret = sc8562_read_byte(sc, addr, &val);
			if (ret) {
				sc_info("sc8562_reg[0x%02X] = 0x%02X\n", addr, val);
			}
		}
	}
	return 0;
}
/*********************************************************************/
static int sc8562_get_cp_status(struct charger_device *chg_dev, u32 evt)
{
	int ret = 0;
	int val = 0;
	struct sc8562 *sc = charger_get_data(chg_dev);

	switch(evt){
	case CP_DEV_REVISION:
		val = sc->revision;
		break;
	case CP_DEV_OVPGATE:
		ret = sc8562_get_ovpgate_state(sc, &sc->ovpgate_state);
		if (!ret)
			val = sc->ovpgate_state;
		break;
	case CP_DEV_WPCGATE:
		ret = sc8562_get_wpcgate_state(sc, &sc->wpcgate_state);
		if (!ret)
			val = sc->wpcgate_state;
		break;
	case CP_DEV_WORKMODE:
		if(sc->work_mode == SC8562_FORWARD_4_1_CHARGER_MODE)
			val = 4;
		else if(sc->work_mode == SC8562_FORWARD_2_1_CHARGER_MODE)
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
static int sc8562_set_mode(struct charger_device *chg_dev, bool div2)
{
	int ret = 0;

	struct sc8562 *sc = charger_get_data(chg_dev);
	if(div2)
		ret = sc8562_set_operation_mode(sc,SC8562_FORWARD_2_1_CHARGER_MODE);
	else
		ret = sc8562_set_operation_mode(sc,SC8562_FORWARD_4_1_CHARGER_MODE);
	return ret;
}
/*********************************************************************/
static const struct charger_ops sc8562_chg_ops = {
	.enable = sc8562_enable_chg,
	.is_enabled = sc8562_is_chg_enabled,
	.get_adc = sc8562_get_adc,
	.set_vbusovp = sc8562_set_vbusovp,
	.set_ibusocp = sc8562_set_ibusocp,
	.set_vbatovp = sc8562_set_vbatovp,
	.set_ibatocp = sc8562_set_ibatocp,
	.init_chip = sc8562_init_chip,
	.set_vbatovp_alarm = sc8562_set_vbatovp_alarm,
	.reset_vbatovp_alarm = sc8562_reset_vbatovp_alarm,
	.set_vbusovp_alarm = sc8562_set_vbusovp_alarm,
	.reset_vbusovp_alarm = sc8562_reset_vbusovp_alarm,
	.is_vbuslowerr = sc8562_is_vbuslowerr,
	.get_adc_accuracy = sc8562_get_adc_accuracy,
	.dump_registers = sc8562_dump_registers,
	.get_cp_status	= sc8562_get_cp_status,
	.set_operation_mode = sc8562_set_mode,
};
/***********************MTK CHARGER OPS API END************************/
static int sc8562_register_chgdev(struct sc8562 *sc)
{
	sc_info("%s: chg name : %s\n", __func__, sc->cfg->chg_name);

	sc->chg_prop.alias_name = sc->cfg->chg_name;
	sc->chg_dev = charger_device_register(sc->cfg->chg_name, sc->dev,sc, &sc8562_chg_ops,	&sc->chg_prop);
	if (!sc->chg_dev)
		return -EINVAL;
	return 0;
}
/*********************************************************************/
static int sc8562_set_present(struct sc8562 *sc, bool present)
{
	sc->usb_present = present;

	if (present)
		sc8562_init_device(sc);
	return 0;
}
/*********************************************************************/
static ssize_t sc8562_show_registers(struct device *dev,
                struct device_attribute *attr, char *buf)
{
	struct sc8562 *sc = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[400];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "sc8562");
	for (addr = 0x0; addr <= 0x6e; addr++) {
		if (addr <= 0x29 || addr >= 0x6c) {
			ret = sc8562_read_byte(sc, addr, &val);
			if (ret) {
				len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[%.2X] = 0x%.2x\n", addr, val);
				memcpy(&buf[idx], tmpbuf, len);
				idx += len;
			}
		}
	}
	return idx;
}
/*********************************************************************/
static ssize_t sc8562_store_registers(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct sc8562 *sc = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && (reg <= 0x29 || (reg >= 0x6c && reg <= 0x6e)))
		sc8562_write_byte(sc, (unsigned char)reg, (unsigned char)val);
	return count;
}
/*********************************************************************/
static DEVICE_ATTR(registers, 0660, sc8562_show_registers, sc8562_store_registers);
/*********************************************************************/
static void sc8562_create_device_node(struct device *dev)
{
	device_create_file(dev, &dev_attr_registers);
}
/*********************************************************************/
static enum power_supply_property sc8562_charger_props[] = {
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
static int sc8562_charger_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
	struct sc8562 *sc = power_supply_get_drvdata(psy);
	int result;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		sc8562_check_charge_enabled(sc, &sc->charge_enabled);
		val->intval = sc->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sc->usb_present;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sc8562_get_adc_data(sc, ADC_VBUS, &result);
		if (!ret)
			sc->vbus_volt = result;
		val->intval = sc->vbus_volt;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sc8562_get_adc_data(sc, ADC_IBUS, &result);
		if (!ret)
			sc->ibus_curr = result;
		val->intval = sc->ibus_curr;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = sc8562_get_adc_data(sc, ADC_VBAT, &result);
		if (!ret)
			sc->vbat_volt = result;
		val->intval = sc->vbat_volt;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = sc8562_get_adc_data(sc, ADC_IBAT, &result);
		if (!ret)
			sc->ibat_curr = result;
		val->intval = sc->ibat_curr;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sc8562_get_adc_data(sc, ADC_TDIE, &result);
		if (!ret)
			sc->die_temp = result;
		val->intval = sc->die_temp;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/*********************************************************************/
static int sc8562_charger_set_property(struct power_supply *psy,
                    enum power_supply_property prop,
                    const union power_supply_propval *val)
{
	struct sc8562 *sc = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		sc8562_enable_charge(sc, val->intval);
		sc8562_check_charge_enabled(sc, &sc->charge_enabled);
		sc_info("POWER_SUPPLY_PROP_ONLINE: %s\n",	val->intval ? "enable" : "disable");
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		sc8562_set_present(sc, !!val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/*********************************************************************/
static int sc8562_charger_is_writeable(struct power_supply *psy,
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
static int sc8562_psy_register(struct sc8562 *sc)
{
	sc->psy_cfg.drv_data = sc;
	sc->psy_cfg.of_node = sc->dev->of_node;
	sc->psy_desc.name = "sc8562-standalone";
	sc->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
	sc->psy_desc.properties = sc8562_charger_props;
	sc->psy_desc.num_properties = ARRAY_SIZE(sc8562_charger_props);
	sc->psy_desc.get_property = sc8562_charger_get_property;
	sc->psy_desc.set_property = sc8562_charger_set_property;
	sc->psy_desc.property_is_writeable = sc8562_charger_is_writeable;
	sc->psy_cfg.drv_data = sc;
	sc->fc2_psy = devm_power_supply_register(sc->dev,	&sc->psy_desc, &sc->psy_cfg);
	if (IS_ERR(sc->fc2_psy)) {
		sc_err("failed to register fc2_psy\n");
		return PTR_ERR(sc->fc2_psy);
	}
	sc_info("%s power supply register successfully\n", sc->psy_desc.name);
	return 0;
}
/*********************************************************************/
static void sc8562_check_fault_status(struct sc8562 *sc)
{
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	unsigned long evt = CHARGER_DEV_NOTIFY_BATPRO_DONE + 1;

	mutex_lock(&sc->data_lock);
	ret = sc8562_read_byte(sc, SC8562_REG_01, &flag);
	if (!ret)
		sc->bat_ovp_fault = !!(flag & SC8562_BAT_OVP_FLAG_MASK);
	if(sc->bat_ovp_fault){
		sc_err("BAT OVP happened\n");
		evt =  CHARGER_DEV_NOTIFY_BAT_OVP;
	}
	ret = sc8562_read_byte(sc, SC8562_REG_02, &flag);
	if (!ret)
		sc->bat_ocp_fault = !!(flag & SC8562_BAT_OCP_FLAG_MASK);
	if(sc->bat_ocp_fault){
		sc_err("BAT OCP happened\n");
		evt =  CHARGER_DEV_NOTIFY_IBATOCP;
	}
	ret = sc8562_read_byte(sc, SC8562_REG_03, &flag);
	if (!ret)
		sc->usb_ovp_fault = !!(flag & SC8562_USB_OVP_FLAG_MASK);
	if(sc->usb_ovp_fault){
		sc_err("USB OVP happened\n");
		evt = CHARGER_DEV_NOTIFY_VBUS_OVP;
	}
	ret = sc8562_read_byte(sc, SC8562_REG_04, &flag);
	if (!ret)
		sc->wpc_ovp_fault = !!(flag & SC8562_WPC_OVP_FLAG_MASK);
	if(sc->wpc_ovp_fault){
		sc_err("WPC OVP happened\n");
	}
	ret = sc8562_read_byte(sc, SC8562_REG_06, &flag);
	if (!ret)
		sc->bus_ocp_fault = !!(flag & SC8562_BUS_OCP_FLAG_MASK);
	if(sc->bus_ocp_fault){
		sc_err("BUS OCP happened\n");
		evt = CHARGER_DEV_NOTIFY_IBUSOCP;
	}
	ret = sc8562_read_byte(sc, SC8562_REG_07, &flag);
	if (!ret)
		sc->bus_ucp_fault = !!(flag & SC8562_BUS_UCP_FALL_FLAG_MASK);
	if(sc->bus_ucp_fault){
		sc_err("BUS UCP happened\n");
	}
	ret = sc8562_read_byte(sc, SC8562_REG_0A, &stat);
	if (!ret && (stat & SC8562_USB_OVP_STAT_MASK))
		sc_err("FAULT_STAT REG0A = 0x%02X\n", stat);
	ret = sc8562_read_byte(sc, SC8562_REG_10, &stat);
	if (!ret && (stat & SC8562_WPC_OVP_STAT_MASK))
		sc_err("FAULT_STAT REG10 = 0x%02X\n", stat);
	if(evt <= CHARGER_DEV_NOTIFY_BATPRO_DONE)
		charger_dev_notify(sc->chg_dev, evt);
	mutex_unlock(&sc->data_lock);
}
/*********************************************************************/
static irqreturn_t sc8562_charger_interrupt(int irq, void *dev_id)
{
	struct sc8562 *sc = dev_id;

	sc_dbg("INT OCCURED\n");
	#if 1
	mutex_lock(&sc->irq_complete);
	sc->irq_waiting = true;
	if (!sc->resume_completed) {
		dev_dbg(sc->dev, "IRQ triggered before device-resume\n");
		if (!sc->irq_disabled) {
			disable_irq_nosync(irq);
			sc->irq_disabled = true;
		}
		mutex_unlock(&sc->irq_complete);
		return IRQ_HANDLED;
	}
	sc->irq_waiting = false;
	#if 1
	sc8562_check_fault_status(sc);
	#endif
	mutex_unlock(&sc->irq_complete);
	#endif
	power_supply_changed(sc->fc2_psy);
	return IRQ_HANDLED;
}
/*********************************************************************/
static int sc8562_irq_register(struct sc8562 *sc)
{
	int ret;
	struct device_node *node = sc->dev->of_node;

	if (!node) {
		sc_err("device tree node missing\n");
		return -EINVAL;
	}

	if (gpio_is_valid(sc->irq_gpio)) {
		ret = gpio_request_one(sc->irq_gpio, GPIOF_DIR_IN,"sc8562_irq");
		if (ret) {
			sc_err("failed to request sc8562_irq\n");
			return -EINVAL;
		}
		sc->irq = gpio_to_irq(sc->irq_gpio);
		if (sc->irq < 0) {
			sc_err("failed to gpio_to_irq\n");
			return -EINVAL;
		}
	} else {
		sc_err("irq gpio not provided\n");
		return -EINVAL;
	}

	if (sc->irq) {
		if (sc->mode == SC8562_ROLE_STDALONE) {
			ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
						NULL, sc8562_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"sc8562 standalone irq", sc);
		} else if (sc->mode == SC8562_ROLE_MASTER) {
			ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
						NULL, sc8562_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"sc8562 master irq", sc);
		} else {
			ret = devm_request_threaded_irq(&sc->client->dev, sc->irq,
						NULL, sc8562_charger_interrupt,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"sc8562 slave irq", sc);
		}
		if (ret < 0) {
			sc_err("request irq for irq=%d failed, ret =%d\n",
			sc->irq, ret);
			return ret;
		}
		enable_irq_wake(sc->irq);
	}
	return ret;
}
/*********************************************************************/
static void determine_initial_status(struct sc8562 *sc)
{
	if (sc->client->irq)
		sc8562_charger_interrupt(sc->client->irq, sc);
}
/*********************************************************************/
static struct of_device_id sc8562_charger_match_table[] = {
	{
		.compatible = "sc,sc8562-standalone",
		.data = &sc8562_mode_data[SC8562_STDALONE],
	},
	{}
};
/*********************************************************************/
static int sc8562_charger_probe(struct i2c_client *client,
                    const struct i2c_device_id *id)
{
	struct sc8562 *sc;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret = 0;

	sc = devm_kzalloc(&client->dev, sizeof(struct sc8562), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = &client->dev;
	sc->client = client;
	mutex_init(&sc->i2c_rw_lock);
	mutex_init(&sc->data_lock);
	mutex_init(&sc->charging_disable_lock);
	mutex_init(&sc->irq_complete);
	sc->resume_completed = true;
	sc->irq_waiting = false;
	match = of_match_node(sc8562_charger_match_table, node);
	if (match == NULL) {
		sc_err("device tree match not found!\n");
		goto err_0;
	}
	sc->mode =  *(int *)match->data;
	ret = sc8562_detect_device(sc);
	if (ret) {
		sc_err("No sc8562 device found!\n");
		goto err_0;
	}
	ret = sc8562_parse_dt(sc, &client->dev);
	if (ret){
		goto err_1;
	}
	i2c_set_clientdata(client, sc);
	sc8562_create_device_node(&(client->dev));
	sc8562_set_work_mode(sc);
	ret = sc8562_init_device(sc);
	if (ret) {
		sc_err("Failed to init device\n");
		goto err_1;
	}
	ret = sc8562_psy_register(sc);
	if (ret){
		goto err_2;
	}
	ret = sc8562_irq_register(sc);
	if (ret < 0)
		goto err_2;
	ret = sc8562_register_chgdev(sc);
	if (ret < 0) {
		sc_err("%s reg chgdev fail(%d)\n", __func__, ret);
		goto err_3;
	}
	device_init_wakeup(sc->dev, 1);
	determine_initial_status(sc);
	sc8562_dump_registers(sc->chg_dev);
	sc_info("sc8562 probe successfully\n!");
	return 0;
err_3:
	charger_device_unregister(sc->chg_dev);
err_2:
	power_supply_unregister(sc->fc2_psy);
err_1:
	if (gpio_is_valid(sc->sc_lpm_gpio))
		gpio_set_value(sc->sc_lpm_gpio, 0);
err_0:
	mutex_destroy(&sc->i2c_rw_lock);
	mutex_destroy(&sc->data_lock);
	mutex_destroy(&sc->charging_disable_lock);
	mutex_destroy(&sc->irq_complete);
	sc_info("sc8562 probe fail\n!");
	return ret;
}
/*********************************************************************/
static inline bool is_device_suspended(struct sc8562 *sc)
{
	return !sc->resume_completed;
}
/*********************************************************************/
static int sc8562_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8562 *sc = i2c_get_clientdata(client);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = false;
	mutex_unlock(&sc->irq_complete);
	sc_err("Suspend successfully!");
	return 0;
}
/*********************************************************************/
static int sc8562_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8562 *sc = i2c_get_clientdata(client);

	if (sc->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}
/*********************************************************************/
static int sc8562_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sc8562 *sc = i2c_get_clientdata(client);

	mutex_lock(&sc->irq_complete);
	sc->resume_completed = true;
	if (sc->irq_waiting){
		sc->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&sc->irq_complete);
		sc8562_charger_interrupt(client->irq, sc);
	} else {
		mutex_unlock(&sc->irq_complete);
	}

	power_supply_changed(sc->fc2_psy);
	sc_err("Resume successfully!");
	return 0;
}
/*********************************************************************/
static int sc8562_charger_remove(struct i2c_client *client)
{
	struct sc8562 *sc = i2c_get_clientdata(client);

	sc8562_enable_adc(sc, false);
	power_supply_unregister(sc->fc2_psy);
	charger_device_unregister(sc->chg_dev);
	mutex_destroy(&sc->charging_disable_lock);
	mutex_destroy(&sc->data_lock);
	mutex_destroy(&sc->i2c_rw_lock);
	mutex_destroy(&sc->irq_complete);
	return 0;
}
/*********************************************************************/
static void sc8562_charger_shutdown(struct i2c_client *client)
{
	struct sc8562 *sc = i2c_get_clientdata(client);

	sc8562_enable_adc(sc, false);
}
/*********************************************************************/
static const struct dev_pm_ops sc8562_pm_ops = {
	.resume     = sc8562_resume,
	.suspend_noirq = sc8562_suspend_noirq,
	.suspend    = sc8562_suspend,
};
/*********************************************************************/
static const struct i2c_device_id sc8562_charger_id[] = {
	{"sc8562-standalone", SC8562_ROLE_STDALONE},
	{},
};
/*********************************************************************/
static struct i2c_driver sc8562_charger_driver = {
	.driver     = {
		.name   = "sc8562-charger",
		.owner  = THIS_MODULE,
		.of_match_table = sc8562_charger_match_table,
		.pm = &sc8562_pm_ops,
	},
	.id_table   = sc8562_charger_id,
	.probe      = sc8562_charger_probe,
	.remove     = sc8562_charger_remove,
	.shutdown   = sc8562_charger_shutdown,
};
module_i2c_driver(sc8562_charger_driver);
MODULE_DESCRIPTION("Southchip SC8562 Charge Pump Driver");
MODULE_LICENSE("GPL v2");
