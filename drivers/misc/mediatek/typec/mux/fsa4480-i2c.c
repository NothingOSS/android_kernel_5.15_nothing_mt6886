// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/mfd/fsa4480-i2c.h>
#include "tcpm.h"

#define FSA4480_I2C_NAME	"fsa4480-driver"

#define HL5280_DEVICE_INFO_VAL           0x49
#define ASW5480_DEVICE_INFO_VAL          0x59
#define WAS4780_DEVICE_INFO_VAL          0x11

/* Registers Map */
#define FSA4480_DEVICE_ID                 0x00
#define FSA4480_SWITCH_SETTINGS           0x04
#define FSA4480_SWITCH_CONTROL            0x05
#define FSA4480_SWITCH_STATUS0            0x06
#define FSA4480_SWITCH_STATUS1            0x07
#define FSA4480_SLOW_L                    0x08
#define FSA4480_SLOW_R                    0x09
#define FSA4480_SLOW_MIC                  0x0A
#define FSA4480_SLOW_SENSE                0x0B
#define FSA4480_SLOW_GND                  0x0C
#define FSA4480_DELAY_L_R                 0x0D
#define FSA4480_DELAY_L_MIC               0x0E
#define FSA4480_DELAY_L_SENSE             0x0F
#define FSA4480_DELAY_L_AGND              0x10
#define FSA4480_FUN_EN                    0x12
#define FSA4480_JACK_STATUS               0x17
#define FSA4480_DET_INT_FLAG              0x18
#define FSA4480_RESET                     0x1E
#define FSA4480_CURRENT_SOURCE_SETTING    0x1F

#define FSA4480_SRC_100 0x01
#define FSA4480_SRC_400 0x04
#define FSA4480_SRC_700 0x07

#define FSA4480_HEADSET_P 0x0A
#define FSA4480_HEADSET_N 0x11

/* external accdet callback wrapper */
#define EINT_PIN_PLUG_OUT       (0)
#define EINT_PIN_PLUG_IN        (1)
void accdet_eint_callback_wrapper(unsigned int plug_status);

// #undef dev_dbg
// #define dev_dbg dev_info

enum switch_vendor {
	FSA4480 = 0,
	HL5280,
	ASW5480,
	WAS4780,
};

struct fsa4480_priv {
	struct regmap *regmap;
	struct device *dev;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	atomic_t usbc_mode;
	struct work_struct usbc_analog_work;
	struct blocking_notifier_head fsa4480_notifier;
	struct mutex notification_lock;
	unsigned int hs_det_pin;
	enum switch_vendor vendor;
	bool plug_state;
};

struct fsa4480_reg_val {
	uint8_t reg;
	uint8_t val;
};

struct fsa4480_priv *g_fsa_priv = NULL;

static const struct regmap_config fsa4480_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = FSA4480_CURRENT_SOURCE_SETTING,
};

static const struct fsa4480_reg_val fsa_reg_i2c_defaults[] = {
	{FSA4480_SLOW_L, 0x00},
	{FSA4480_SLOW_R, 0x00},
	{FSA4480_SLOW_MIC, 0x00},
	{FSA4480_SLOW_SENSE, 0x00},
	{FSA4480_SLOW_GND, 0x00},
	{FSA4480_DELAY_L_R, 0x00},
	{FSA4480_DELAY_L_MIC, 0x00},
	{FSA4480_DELAY_L_SENSE, 0x00},
	{FSA4480_DELAY_L_AGND, 0x09},
	{FSA4480_SWITCH_SETTINGS, 0x98},
};

static void fsa4480_usbc_update_settings(struct fsa4480_priv *fsa_priv,
		u32 switch_control, u32 switch_enable)
{
	if (!fsa_priv->regmap) {
		dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
		return;
	}

	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x80);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, switch_control);
	/* FSA4480 chip hardware requirement */
	usleep_range(50, 55);
	regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, switch_enable);
	/* Following NT2 phone */
	if (fsa_priv->vendor == WAS4780) {
		usleep_range(200, 205);
	}
}

static void fsa4480_autoset_switch(struct fsa4480_priv *fsa_priv)
{
	u32 rc, reg;
	u8 i;

	if (!fsa_priv->regmap) {
	        dev_err(fsa_priv->dev, "%s: regmap invalid\n", __func__);
	        return;
	}
	/*start auto dectection*/
	regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x09);
	/*checking auto detection finish*/
	for (i = 0; i < 10; i++) {
		usleep_range(1000, 1050);
		regmap_read(fsa_priv->regmap, FSA4480_DET_INT_FLAG, &rc);
		regmap_read(fsa_priv->regmap, FSA4480_FUN_EN, &reg);
		if (((reg & 0x01) == 0) || (rc & 0x04)) {
				dev_err(fsa_priv->dev, "%s: auto_det success\n", __func__);
				break;
		}
		if (i == 9) {
			regmap_read(fsa_priv->regmap, FSA4480_CURRENT_SOURCE_SETTING, &rc);
			if (rc == FSA4480_SRC_100) {
				dev_err(fsa_priv->dev, "%s: auto_det fail\n", __func__);
				return;
			}
			regmap_write(fsa_priv->regmap, FSA4480_CURRENT_SOURCE_SETTING, FSA4480_SRC_100);
			fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
			regmap_write(fsa_priv->regmap, FSA4480_FUN_EN, 0x09);
			i = 0;
		}
	}
	//for codec 3/4 pole earphone setting
	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &rc);
	dev_err(fsa_priv->dev, "%s:auto_det reg_0x07 = 0x%#x\n",__func__, rc);
	//manual set switch
	if ((rc != FSA4480_HEADSET_P) && (rc != FSA4480_HEADSET_N)) {
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_SETTINGS, 0x9F);
		rc = (rc & FSA4480_HEADSET_P) ? 0x00 : 0x07;
		regmap_write(fsa_priv->regmap, FSA4480_SWITCH_CONTROL, rc);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &rc);
		dev_err(fsa_priv->dev, "%s: manual_set reg_0x07 = 0x%#x\n",__func__, rc);
	}
}

static int fsa4480_usbc_event_changed(struct notifier_block *nb,
					  unsigned long evt, void *ptr)
{
	struct fsa4480_priv *fsa_priv =
			container_of(nb, struct fsa4480_priv, pd_nb);
	struct device *dev;
	struct tcp_notify *noti = ptr;

	if (!fsa_priv)
		return -EINVAL;

	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	if (fsa_priv->vendor == HL5280) {
		dev_info(dev, "%s: switch chip is HL5280\n", __func__);
	}
	else if (fsa_priv->vendor == ASW5480) {
		dev_info(dev, "%s: switch chip is ASW5480\n", __func__);
	}
	else if (fsa_priv->vendor == WAS4780) {
		dev_info(dev, "%s: switch chip is WAS4780\n", __func__);
	}

	dev_info(dev, "%s: typeC event: %d\n", __func__, evt);

	switch (evt) {
	case TCP_NOTIFY_TYPEC_STATE:
		dev_info(dev, "%s: old_state: %d, new_state: %d\n",
			__func__, noti->typec_state.old_state, noti->typec_state.new_state);
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_AUDIO) {
			/* AUDIO plug in */
			dev_info(dev, "%s: audio plug in\n", __func__);
			fsa_priv->plug_state = true;
			dev_dbg(dev, "%s: tcpc polarity = %d\n", __func__, noti->typec_state.polarity);
			pm_stay_awake(fsa_priv->dev);
			schedule_work(&fsa_priv->usbc_analog_work);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			/* AUDIO plug out */
			dev_err(dev, "%s: audio plug out\n", __func__);
			fsa_priv->plug_state = false;
			pm_stay_awake(fsa_priv->dev);
			schedule_work(&fsa_priv->usbc_analog_work);
		}
		else {
			dev_dbg(dev, "%s: ignore tcpc non-audio notification\n", __func__);
		}
		break;
	default:
		break;
	};

	return NOTIFY_OK;
}

static int fsa4480_usbc_analog_setup_switches(struct fsa4480_priv *fsa_priv)
{
	struct device *dev;
	unsigned int switch_status  = 0;
	unsigned int jack_status    = 0;
	int i = 0, reg_val = 0;

	if (!fsa_priv)
		return -EINVAL;
	dev = fsa_priv->dev;
	if (!dev)
		return -EINVAL;

	mutex_lock(&fsa_priv->notification_lock);

	dev_info(dev, "%s: plug_state %d\n", __func__, fsa_priv->plug_state);
	if (fsa_priv->plug_state) {
		/* activate switches */
		fsa4480_usbc_update_settings(fsa_priv, 0x00, 0x9F);
		fsa4480_autoset_switch(fsa_priv);
		dev_info(dev, "%s: set reg[0x%x] done.\n", __func__, FSA4480_FUN_EN);

		accdet_eint_callback_wrapper(EINT_PIN_PLUG_IN);

		regmap_read(fsa_priv->regmap, FSA4480_JACK_STATUS, &jack_status);
		dev_info(dev, "%s: jack status: 0x%x.\n", __func__, jack_status);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS0, &switch_status);
		dev_info(dev, "%s: switch status0: 0x%x.\n", __func__, switch_status);
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);
		dev_info(dev, "%s: switch status1: 0x%x.\n", __func__, switch_status);
	} else {
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		accdet_eint_callback_wrapper(EINT_PIN_PLUG_OUT);
	}
	/*test*/
	for (i = 0; i <= 0x1f; i++) {
		regmap_read(fsa_priv->regmap, i, &reg_val);
		dev_info(dev, "%s: read reg: %x value: 0x%x\n", __func__, i, reg_val);
	}
	mutex_unlock(&fsa_priv->notification_lock);
	return 0;
}

/*
 * fsa4480_reg_notifier - register notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on success, or error code
 */
int fsa4480_reg_notifier(struct notifier_block *nb,
			 struct device_node *node)
{
	int rc = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	rc = blocking_notifier_chain_register
				(&fsa_priv->fsa4480_notifier, nb);
	if (rc)
		return rc;

	/*
	 * as part of the init sequence check if there is a connected
	 * USB C analog adapter
	 */
	dev_dbg(fsa_priv->dev, "%s: verify if USB adapter is already inserted\n",
		__func__);
	rc = fsa4480_usbc_analog_setup_switches(fsa_priv);

	return rc;
}
EXPORT_SYMBOL(fsa4480_reg_notifier);

/*
 * fsa4480_unreg_notifier - unregister notifier block with fsa driver
 *
 * @nb - notifier block of fsa4480
 * @node - phandle node to fsa4480 device
 *
 * Returns 0 on pass, or error code
 */
int fsa4480_unreg_notifier(struct notifier_block *nb,
				 struct device_node *node)
{
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	return blocking_notifier_chain_unregister
					(&fsa_priv->fsa4480_notifier, nb);
}
EXPORT_SYMBOL(fsa4480_unreg_notifier);

static int fsa4480_validate_display_port_settings(struct fsa4480_priv *fsa_priv)
{
	u32 switch_status = 0;

	regmap_read(fsa_priv->regmap, FSA4480_SWITCH_STATUS1, &switch_status);

	if ((switch_status != 0x23) && (switch_status != 0x1C)) {
		dev_err(fsa_priv->dev, "%s: AUX SBU1/2 switch status is invalid = %u\n",
				__func__, switch_status);
		return -EIO;
	}

	return 0;
}
/*
 * fsa4480_switch_event - configure FSA switch position based on event
 *
 * @node - phandle node to fsa4480 device
 * @event - fsa_function enum
 *
 * Returns int on whether the switch happened or not
 */
int fsa4480_switch_event(struct device_node *node,
			 enum fsa_function event)
{
	int switch_control = 0;
	struct i2c_client *client = of_find_i2c_device_by_node(node);
	struct fsa4480_priv *fsa_priv;

	if (!client)
		return -EINVAL;

	fsa_priv = (struct fsa4480_priv *)i2c_get_clientdata(client);
	if (!fsa_priv)
		return -EINVAL;
	if (!fsa_priv->regmap)
		return -EINVAL;

	pr_info("%s - switch event: %d\n", __func__, event);
	switch (event) {
	case FSA_MIC_GND_SWAP:
		regmap_read(fsa_priv->regmap, FSA4480_SWITCH_CONTROL,
				&switch_control);
		if ((switch_control & 0x07) == 0x07)
			switch_control = 0x0;
		else
			switch_control = 0x7;
		fsa4480_usbc_update_settings(fsa_priv, switch_control, 0x9F);
		break;
	case FSA_USBC_ORIENTATION_CC1:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_ORIENTATION_CC2:
		fsa4480_usbc_update_settings(fsa_priv, 0x78, 0xF8);
		return fsa4480_validate_display_port_settings(fsa_priv);
	case FSA_USBC_DISPLAYPORT_DISCONNECTED:
		fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(fsa4480_switch_event);

static int fsa4480_parse_dt(struct fsa4480_priv *fsa_priv,
	struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	int ret = 0;

	if (dNode == NULL) {
		pr_err("%s: device node is NULL\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void fsa4480_usbc_analog_work_fn(struct work_struct *work)
{
	struct fsa4480_priv *fsa_priv =
		container_of(work, struct fsa4480_priv, usbc_analog_work);

	if (!fsa_priv) {
		pr_err("%s: fsa container invalid\n", __func__);
		return;
	}
	fsa4480_usbc_analog_setup_switches(fsa_priv);
	pm_relax(fsa_priv->dev);
}

static void fsa4480_update_reg_defaults(struct fsa4480_priv *fsa_priv)
{
	u8 i;

	if (fsa_priv->vendor == WAS4780) {
		regmap_write(fsa_priv->regmap, FSA4480_RESET, 0x01);
		msleep(1);
	}
	for (i = 0; i < ARRAY_SIZE(fsa_reg_i2c_defaults); i++)
		regmap_write(fsa_priv->regmap, fsa_reg_i2c_defaults[i].reg,
				   fsa_reg_i2c_defaults[i].val);
}

static ssize_t fregdump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, rc, ret = 0;
	struct fsa4480_priv *fsa_priv = g_fsa_priv;

	if (!fsa_priv) {
		pr_err("%s: fsa priv invalid\n", __func__);
		return ret;
	}
	mutex_lock(&fsa_priv->notification_lock);
	for (i = 0 ; i <= 0x1F; i++) {
		ret = regmap_read(fsa_priv->regmap, (uint8_t)i, &rc);
		dev_info(fsa_priv->dev, "read was4780 Reg_0x%02x=0x%02x\n", (uint8_t)i, rc);
	}
	mutex_unlock(&fsa_priv->notification_lock);

	return ret;
}
DEVICE_ATTR(fregdump, S_IRUGO, fregdump_show, NULL);

static int fsa4480_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct fsa4480_priv *fsa_priv;
	int rc = 0;
	unsigned int reg_value = 0;
	uint8_t tcpc_attach_state = 0;

	fsa_priv = devm_kzalloc(&i2c->dev, sizeof(*fsa_priv),
				GFP_KERNEL);
	if (!fsa_priv)
		return -ENOMEM;

	fsa_priv->dev = &i2c->dev;

	fsa4480_parse_dt(fsa_priv, &i2c->dev);

	fsa_priv->regmap = devm_regmap_init_i2c(i2c, &fsa4480_regmap_config);
	if (IS_ERR_OR_NULL(fsa_priv->regmap)) {
		dev_err(fsa_priv->dev, "%s: Failed to initialize regmap: %d\n",
			__func__, rc);
		if (!fsa_priv->regmap) {
			rc = -EINVAL;
			goto err_data;
		}
		rc = PTR_ERR(fsa_priv->regmap);
		goto err_data;
	}

	regmap_read(fsa_priv->regmap, FSA4480_DEVICE_ID, &reg_value);
	if (HL5280_DEVICE_INFO_VAL == reg_value) {
		fsa_priv->vendor = HL5280;
		dev_info(fsa_priv->dev, "%s: switch chip is HL5280\n", __func__);
	}
	else if (ASW5480_DEVICE_INFO_VAL == reg_value) {
		fsa_priv->vendor = ASW5480;
		dev_info(fsa_priv->dev, "%s: switch chip is ASW5480\n", __func__);
	}
	else if (WAS4780_DEVICE_INFO_VAL == reg_value) {
		fsa_priv->vendor = WAS4780;
		dev_info(fsa_priv->dev, "%s: switch chip is WAS4780\n", __func__);
	}
	else {
		fsa_priv->vendor = FSA4480;
		dev_info(fsa_priv->dev, "%s: switch chip is FSA4480 or other[0x%x]\n", __func__, reg_value);
	}

	fsa4480_update_reg_defaults(fsa_priv);

	fsa_priv->plug_state = false;
	fsa_priv->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!fsa_priv->tcpc_dev) {
		pr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		goto err_data;
	}

	fsa_priv->pd_nb.notifier_call = fsa4480_usbc_event_changed;
	fsa_priv->pd_nb.priority = 0;
	rc = register_tcp_dev_notifier(fsa_priv->tcpc_dev, &fsa_priv->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (rc < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		goto err_data;
	}

	mutex_init(&fsa_priv->notification_lock);
	i2c_set_clientdata(i2c, fsa_priv);

	INIT_WORK(&fsa_priv->usbc_analog_work,
		  fsa4480_usbc_analog_work_fn);

	fsa_priv->fsa4480_notifier.rwsem =
		(struct rw_semaphore)__RWSEM_INITIALIZER
		((fsa_priv->fsa4480_notifier).rwsem);
	fsa_priv->fsa4480_notifier.head = NULL;

	device_create_file(fsa_priv->dev, &dev_attr_fregdump);
	/* In case of audio accessory pluged in before this driver probe */
	tcpc_attach_state = tcpm_inquire_typec_attach_state(fsa_priv->tcpc_dev);
	if (unlikely(tcpc_attach_state == TYPEC_ATTACHED_AUDIO)) {
		dev_info(fsa_priv->dev, "%s: audio plug in before probe\n", __func__);
		fsa_priv->plug_state = true;
		pm_stay_awake(fsa_priv->dev);
		schedule_work(&fsa_priv->usbc_analog_work);
	}

	g_fsa_priv = fsa_priv;
	return 0;

err_data:
	devm_kfree(&i2c->dev, fsa_priv);
	return rc;
}

static int fsa4480_remove(struct i2c_client *i2c)
{
	struct fsa4480_priv *fsa_priv =
			(struct fsa4480_priv *)i2c_get_clientdata(i2c);

	if (!fsa_priv)
		return -EINVAL;

	fsa4480_usbc_update_settings(fsa_priv, 0x18, 0x98);
	cancel_work_sync(&fsa_priv->usbc_analog_work);
	pm_relax(fsa_priv->dev);
	mutex_destroy(&fsa_priv->notification_lock);
	dev_set_drvdata(&i2c->dev, NULL);
	device_remove_file(fsa_priv->dev, &dev_attr_fregdump);

	return 0;
}

static const struct of_device_id fsa4480_i2c_dt_match[] = {
	{ .compatible = "onsemi,fsa4480-i2c",},
	{ .compatible = "willsemi,was4780-i2c",},
	{ .compatible = "halomicro,hl5280-i2c",},
	{}
};

static struct i2c_driver fsa4480_i2c_driver = {
	.driver = {
		.name = FSA4480_I2C_NAME,
		.of_match_table = fsa4480_i2c_dt_match,
	},
	.probe = fsa4480_probe,
	.remove = fsa4480_remove,
};

static int __init fsa4480_init(void)
{
	int rc;

	rc = i2c_add_driver(&fsa4480_i2c_driver);
	if (rc)
		pr_err("fsa4480: Failed to register I2C driver: %d\n", rc);

	return rc;
}

static void __exit fsa4480_exit(void)
{
	i2c_del_driver(&fsa4480_i2c_driver);
}

late_initcall_sync(fsa4480_init);
module_exit(fsa4480_exit);

MODULE_DESCRIPTION("FSA4480 I2C driver");
MODULE_LICENSE("GPL v2");
