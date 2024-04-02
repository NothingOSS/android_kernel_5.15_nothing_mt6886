#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <tcpm.h>
#include <linux/alarmtimer.h>
#if IS_ENABLED(CONFIG_PROC_FS)
#include <linux/proc_fs.h>
#endif
#include "mtk_charger.h"
#include "mtk_battery.h"
#include "mtk_pe5.h"
#include "mtk_pps.h"
#include "nt_usb_ts.h"
#include "nt_chg.h"

static int _uA_to_mA(int uA)
{
	if (uA == -1)
		return -1;
	else
		return uA / 1000;
}

const char * const POWER_SUPPLY_USB_TYPE_TEXT[] = {
	[POWER_SUPPLY_USB_TYPE_UNKNOWN]		= "Unknown",
	[POWER_SUPPLY_USB_TYPE_SDP]		= "SDP",
	[POWER_SUPPLY_USB_TYPE_DCP]		= "DCP",
	[POWER_SUPPLY_USB_TYPE_CDP]		= "CDP",
	[POWER_SUPPLY_USB_TYPE_ACA]		= "ACA",
	[POWER_SUPPLY_USB_TYPE_C]		= "C",
	[POWER_SUPPLY_USB_TYPE_PD]		= "PD",
	[POWER_SUPPLY_USB_TYPE_PD_DRP]		= "PD_DRP",
	[POWER_SUPPLY_USB_TYPE_PD_PPS]		= "PD_PPS",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID]	= "BrickID",
	[POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID+1]	= "PE",
};

static struct mtk_battery *get_battery_entry(void)
{
	struct mtk_gauge *gauge;
	struct power_supply *psy;
	static struct mtk_battery *gm;

	if (gm == NULL) {
		psy = power_supply_get_by_name("mtk-gauge");
		if (psy == NULL) {
			pr_err("[%s]psy is not rdy\n", __func__);
			return NULL;
		}

		gauge = (struct mtk_gauge *)power_supply_get_drvdata(psy);
		if (gauge == NULL) {
			pr_err("[%s]mtk_gauge is not rdy\n", __func__);
			return NULL;
		}
		gm = gauge->gm;
	}
	return gm;
}

static int check_chg_status(struct nt_chg_info *nci)
{
	int vchr = 0;
	struct mtk_charger *info;

	if (!nci || !nci->info)
		return 0;
	info = nci->info;

	if ((!nci->typec_attach) || (info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN))
		return 0;

	charger_dev_get_vbus(info->chg1_dev, &vchr);
	pr_info("[%s]vbus : %d,{%d ~ %d}\n", __func__, vchr , (nci->is_hvcharger ? HVDCP_SW_VBUSOV_UV : info->data.max_charger_voltage), info->data.min_charger_voltage);

	if (vchr > (nci->is_hvcharger ? HVDCP_SW_VBUSOV_UV : info->data.max_charger_voltage)) {
		return NT_NOTIFY_CHARGER_OVER_VOL;
	} else if (vchr <= info->data. min_charger_voltage) {
		return NT_NOTIFY_CHARGER_LOW_VOL;
	}
	return 0;
}

static int check_dvchg_status(void)
{
	struct chg_alg_device *alg = NULL;
	struct pe50_algo_info *info = NULL;
	struct pps_algo_info *info2 = NULL;

	alg = get_chg_alg_by_name("pe5");
	if(alg){
		info = chg_alg_dev_get_drvdata(alg);
		if(info && info->data){
			return info->data->notify;
		}
	}
	alg = get_chg_alg_by_name("pps");
	if(alg){
		info2 = chg_alg_dev_get_drvdata(alg);
		if(info2 && info2->data){
			return info2->data->notify;
		}
	}
	return 0;
}

static unsigned int check_abnormal_status(struct nt_chg_info *nci)
{
	u8 i;
	int evt;
	struct sw_jeita_data *sw_jeita;
	struct mtk_battery *gm = get_battery_entry();
	struct mtk_charger *info;
	struct battery_data *bat_data;
	unsigned int notify_code = 0;
	enum nt_charge_abnormal_type status;
	int usb_temp_status = 0;

	if (!nci || !gm)
		return 0;

	info = nci->info;

	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}
	/* check usb temperature status*/
	usb_temp_status = get_usb_ts_check_state();
	if(usb_temp_status){
		notify_code |= NT_NOTIFY_USB_TEMP_ABNORMAL;
	}else{
		notify_code &= ~NT_NOTIFY_USB_TEMP_ABNORMAL;
	}
	sw_jeita = &info->sw_jeita;
	/*check vbus*/
	status = check_chg_status(nci);
	if(status){
		notify_code |= status;
	}else{
		notify_code &= ~NT_NOTIFY_CHARGER_OVER_VOL;
		notify_code &= ~NT_NOTIFY_CHARGER_LOW_VOL;
	}
	if((nci->typec_attach)&&(!info->is_charging))
		notify_code |= NT_NOTIFY_CHGING_CURRENT;
	else
		notify_code &= ~NT_NOTIFY_CHGING_CURRENT;

	if((nci->typec_attach)&&(info->safety_timeout))
		notify_code |= NT_NOTIFY_CHGING_OVERTIME;
	else
		notify_code &= ~NT_NOTIFY_CHGING_OVERTIME;

	/*check batt exist*/
	bat_data = &gm->bs_data;
	if(!bat_data->bat_present)
		notify_code |= NT_NOTIFY_BAT_NOT_CONNECT;
	else
		notify_code &= ~NT_NOTIFY_BAT_NOT_CONNECT;

	/*check batt full*/
	if(bat_data->bat_capacity == 100)
		notify_code |= NT_NOTIFY_BAT_FULL;
	else
		notify_code &= ~NT_NOTIFY_BAT_FULL;

	/*check batt temp*/
	if(sw_jeita->sm == TEMP_ABOVE_T4){
		notify_code |= NT_NOTIFY_BAT_OVER_TEMP;
		notify_code &= ~NT_NOTIFY_BAT_LOW_TEMP;
	}else if(sw_jeita->sm == TEMP_BELOW_T0){
		notify_code &= ~NT_NOTIFY_BAT_OVER_TEMP;
		notify_code |= NT_NOTIFY_BAT_LOW_TEMP;
	}else{
		notify_code &= ~NT_NOTIFY_BAT_OVER_TEMP;
		notify_code &= ~NT_NOTIFY_BAT_LOW_TEMP;
	}

	/*check batt vol*/
	if(sw_jeita->cv){
		if((bat_data->bat_batt_vol >= sw_jeita->cv))
			notify_code |= NT_NOTIFY_BAT_OVER_VOL;
		else
			notify_code &= ~NT_NOTIFY_BAT_OVER_VOL;
	}else{
		/*T2-T3*/
		if((bat_data->bat_batt_vol >= info->data.battery_cv))
			notify_code |= NT_NOTIFY_BAT_OVER_VOL;
		else
			notify_code &= ~NT_NOTIFY_BAT_OVER_VOL;
	}

	/*check ieoc*/
	if(gm->b_EOC)
		notify_code |= NT_NOTIFY_CHARGER_TERMINAL;
	else
		notify_code &= ~NT_NOTIFY_CHARGER_TERMINAL;
	/*check pump status*/
	evt = check_dvchg_status();
	if(evt > EVT_HARDRESET){
		notify_code |= NT_NOTIFY_CHARGE_PUMP_ERR;
	}else if(evt && evt < EVT_HARDRESET){
		notify_code &= ~NT_NOTIFY_CHARGE_PUMP_ERR;
	}

	pr_info("%s: [nt_abnormal_status] : %d \n",__func__, notify_code);
	for(i = 0;i < 20; i++){
		if(g_abnormal_info[i].type & notify_code){
			pr_info("[nt_abnormal_status] : { %s }",g_abnormal_info[i].item_text);
		}
	}
	return notify_code;
}

static int cooling_state_to_charger_limit(struct nt_chg_info *nci)
{
	int fcc = -1;
	int lst_rnd_alg_idx = -1;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;
	struct charger_data *pdata, *pdata_dvchg;
	struct mtk_charger *info;
	struct chg_limit_setting *setting;
	
	if (!nci){
		pr_info("%s: nci is null!\n", __func__);
		return -EINVAL;
	}
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}
	mutex_lock(&nci->proc_lock);
	lst_rnd_alg_idx = info->lst_rnd_alg_idx;
	pr_err("%s:lst_rnd_alg_idx :%d\n", __func__,lst_rnd_alg_idx);
	if(lst_rnd_alg_idx != -1){
		alg = info->alg[lst_rnd_alg_idx];
		is_fast_charge = true;
	}else{
		is_fast_charge = false;
	}
	fcc = nci->fcc;
	pdata = &info->chg_data[CHG1_SETTING];
	pdata_dvchg = &info->chg_data[DVCHG1_SETTING];
	setting = &info->setting;
	nci->cp_workmode = charger_dev_get_cp_status(nci->info->dvchg1_dev, CP_DEV_WORKMODE);
	if(is_fast_charge && alg){
		switch(alg->alg_id){
		case PE5_ID:
			if(fcc > 0){
				pdata_dvchg->thermal_input_current_limit = (fcc*1000)/nci->cp_workmode + nci->adc_compensation;
			}else{
				pdata_dvchg->thermal_input_current_limit = -1;
			}
			break;
		case PPS_ID:
			if(fcc > 0){
				pdata_dvchg->thermal_input_current_limit = (fcc*1000)/nci->cp_workmode + nci->adc_compensation;
			}else{
				pdata_dvchg->thermal_input_current_limit = -1;
			}
			break;
		case PE2_ID:
			if((fcc > 0) && (fcc < (nci->sc_charger_current/1000))){
				pdata->thermal_charging_current_limit = fcc*1000;
			}else{
				pdata->thermal_charging_current_limit = -1;
			}
			break;
		case PDC_ID:
			if((fcc > 0) && (fcc < (nci->pd_charger_current/1000))){
				pdata->thermal_charging_current_limit = fcc*1000;
			}else{
				pdata->thermal_charging_current_limit = -1;
			}
			break;
		default:
			break;
		}
		pr_info("%s: fcc_store %d ma,fast_charge = %d ,alg_id = %d \n", __func__, fcc, is_fast_charge,alg->alg_id);
	}else{
		if((fcc > 0) && (fcc <= (nci->ac_charger_current/1000))){
			pdata->thermal_charging_current_limit = fcc*1000;
		}else{
			pdata->thermal_charging_current_limit = -1;
		}
		pr_info("%s: fcc_store %d ma\n", __func__, fcc);
	}

	if(fcc == 0){
		info->cmd_discharging = true;
	}else{
		info->cmd_discharging = false;
	}
	mutex_unlock(&nci->proc_lock);
	return 0;
}

static int chg_check_vbus(struct nt_chg_info *nci)
{
	int vchr = 0,ret = 0;

	if (nci == NULL)
		return 0;
	if(nci->info == NULL)
		return 0;
	ret = charger_dev_get_vbus(nci->info->chg1_dev, &vchr);
	if(ret < 0){
		pr_err("%s: get vbus failed: %d\n", __func__, ret);
	}else
		vchr /= 1000;
	return vchr;
}
#if  0
static int chg_check_ibat(struct nt_chg_info *nci)
{
	int ret = 0;
	int ibat = 0;

	if (nci == NULL)
		return 0;
	if(nci->info == NULL)
		return 0;

	ret = charger_dev_get_ibat(nci->info->chg1_dev, &ibat);
	if (ret < 0)
		pr_err("%s: get ibat failed: %d\n", __func__, ret);

	return ibat / 1000;
}
#endif

static int chg_check_ibus(struct nt_chg_info *nci)
{
	bool en = false;
	int ret = 0;
	int ibus = 0;
	int dvchg = 0;

	if (nci == NULL)
		return 0;
	if(nci->info == NULL)
		return 0;

	ret = charger_dev_get_ibus(nci->info->chg1_dev, &ibus);
	if (ret < 0)
		pr_err("%s: get ibus failed: %d\n", __func__, ret);

	ret = charger_dev_is_enabled(nci->info->dvchg1_dev, &en);
	if (ret >= 0 && en) {
		ret = charger_dev_get_adc(nci->info->dvchg1_dev,
					  ADC_CHANNEL_IBUS, &dvchg, &dvchg);
		if (ret >= 0) {
			ibus += dvchg;
		}
	}
	return ibus;
}

static int chg_check_battery_info(struct nt_chg_info *nci,enum power_supply_property psp)
{
	union power_supply_propval prop;
	struct power_supply *bat_psy = NULL;
	int ret = 0;

	if (nci == NULL)
		return 0;
	if(nci->info == NULL)
		return 0;

	bat_psy = nci->info->bat_psy;

	if (bat_psy == NULL || IS_ERR(bat_psy)) {
		pr_err("%s retry to get bat_psy\n", __func__);
		bat_psy = devm_power_supply_get_by_phandle(&nci->info->pdev->dev, "gauge");
		nci->info->bat_psy = bat_psy;
	}

	if (bat_psy == NULL || IS_ERR(bat_psy))
		pr_err("%s Couldn't get bat_psy\n", __func__);
	else{
		ret = power_supply_get_property(bat_psy, psp, &prop);
		switch(psp){
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		case POWER_SUPPLY_PROP_CAPACITY:
			ret = prop.intval;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = prop.intval / 1000;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			ret = prop.intval / 10;
			break;
		default:
			break;
		}
	}
	return ret;
}

static int chg_check_pmic_info(struct nt_chg_info *nci,enum power_supply_property psp)
{
	union power_supply_propval prop = {0};
	struct power_supply *bc12_psy = NULL;

	if (!nci || !nci->info)
		return -EINVAL;

	bc12_psy = nci->info->bc12_psy;

	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		pr_err("%s: retry to get bc12_psy\n", __func__);
		bc12_psy = power_supply_get_by_name("primary_chg");
	}

	if (bc12_psy == NULL || IS_ERR(bc12_psy)) {
		pr_err("%s: Couldn't get bc12_psy\n", __func__);
	} else {
		power_supply_get_property(bc12_psy,	psp, &prop);
	}
	pr_info("%s: psp:%d - %d \n", __func__, psp, prop.intval);
	return prop.intval;
}

static int check_chg_type(struct nt_chg_info *nci)
{
	int i,ret,type = 0;
	struct chg_alg_device *alg = NULL;
	struct mtk_charger *info = NULL;

	if(nci == NULL){
		pr_err("%s:nci is null!!\n", __func__);
		return 0;
	}
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}
	type = chg_check_pmic_info(nci,POWER_SUPPLY_PROP_USB_TYPE);
	if(nci->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO){
		type = nci->chg_type = POWER_SUPPLY_USB_TYPE_PD_PPS;
	}else if(nci->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30 ||
		 nci->info->pd_type == MTK_PD_CONNECT_PE_READY_SNK){
		type = nci->chg_type = POWER_SUPPLY_USB_TYPE_PD;
	}else {
		/*PE : POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID+1 = 10*/
		if((nci->chg_type != POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID+1) && nci->typec_attach){
			for (i = 0; i < MAX_ALG_NO; i++) {
				alg = info->alg[i];
				if (alg == NULL || (alg->alg_id != PE2_ID))
					continue;

				if (info->enable_fast_charging_indicator &&
					((alg->alg_id & info->fast_charging_indicator) == 0))
					continue;

				ret = chg_alg_is_algo_ready(alg);
				if (ret == ALG_RUNNING) {
					type = POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID+1;
					nci->chg_type = type;
					break;
				}
			}
			if(i == MAX_ALG_NO)
				nci->chg_type = type;
		}else{
			type = nci->chg_type;
		}
	}
	return type;
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct nt_chg_info *nci = container_of(nb, struct nt_chg_info, pd_nb);

	switch (event) {
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
		    (noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)) {
			pr_info("%s:[NT] USB Plug in, pol = %d\n", __func__,	noti->typec_state.polarity);
			nci->typec_attach = true;
			nci->chg_type = 0;
			nci->chg_vol_max = 0;
			nci->chg_icl_max = 0;
			_wake_up_nt_charger(nci);
			nci->pre_input_current_limmit = -1;
			nci->pre_aicl_limmit = -1;
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO)
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_info("%s:[NT] USB Plug out\n", __func__);
			nci->typec_attach = false;
			nci->chg_type = 0;
			nci->chg_vol_max = 0;
			nci->chg_icl_max = 0;
			_wake_up_nt_charger(nci);
			nci->pre_input_current_limmit = -1;
			nci->pre_aicl_limmit = -1;
		}
		break;
	case TCP_NOTIFY_WD0_STATE:
		nci->wd0_state = noti->wd0_state.wd0;
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

#if IS_ENABLED(CONFIG_PROC_FS)
static int usb_charger_en_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	seq_printf(m, "%d\n", nci->cmd_discharging);
	return 0;
}

static ssize_t usb_charger_en_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int i,cmd_discharging,len;
	char desc[32];
	struct chg_alg_device *alg;
	struct mtk_charger *info;
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}
	if(sscanf(desc, "%d", &cmd_discharging) == 1){
		nci->cmd_discharging = cmd_discharging;
		if (cmd_discharging == 1) {
			for (i = 0; i < MAX_ALG_NO; i++) {
				alg = info->alg[i];
				if (alg == NULL)
					continue;
				chg_alg_stop_algo(alg);
			}
			info->cmd_discharging = true;
			charger_dev_enable(info->chg1_dev, false);
			charger_dev_do_event(info->chg1_dev, EVENT_DISCHARGE, 0);
		} else if (cmd_discharging == 0) {
			info->cmd_discharging = false;
			charger_dev_enable(info->chg1_dev, true);
			charger_dev_do_event(info->chg1_dev,EVENT_RECHARGE, 0);
		}
		pr_info("%s: cmd_discharging=%d\n",__func__, cmd_discharging);
	}
	return count;
}
PROC_FOPS_RW(usb_charger_en);

static int temp_proc_show(struct seq_file *m, void *v)
{
	return 0;
}
PROC_FOPS_RO(temp);

static int voltage_adc_proc_show(struct seq_file *m, void *v)
{
	int ret = 0;
	union power_supply_propval prop;
	struct power_supply *batpsy = power_supply_get_by_name("battery");

	if(batpsy){
		ret = power_supply_get_property(batpsy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		ret = prop.intval;
	}
	pr_info("%s: bat vol = %d uv!\n",__func__, ret);
	seq_printf(m, "%d\n", ret);
	return 0;
}
PROC_FOPS_RO(voltage_adc);

static int usb_temp_proc_show(struct seq_file *m, void *v)
{
	int temp = 0;
	struct nt_chg_info *nci = m->private;

	if(nci == NULL){
		pr_err("%s:nci is null!!\n", __func__);
		return 0;
	}
	temp = get_usb_temperature();
	seq_printf(m, "%d\n", temp);
	return 0;
}
PROC_FOPS_RO(usb_temp);

static int usb_real_type_proc_show(struct seq_file *m, void *v)
{
	int type = 0;
	struct nt_chg_info *nci = m->private;

	if(nci == NULL){
		pr_err("%s:nci is null!!\n", __func__);
		return 0;
	}
	if (nci->is_hvcharger) {
		seq_printf(m, "%s\n", "HVDCP");
	} else {
		type = check_chg_type(nci);
		seq_printf(m, "%s\n", POWER_SUPPLY_USB_TYPE_TEXT[type]);
	}
	return 0;
}
PROC_FOPS_RO(usb_real_type);

static int nt_abnormal_status_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	if (!nci)
		return -EINVAL;

	seq_printf(m, "%d\n", nci->notify_code);
	return 0;
}
PROC_FOPS_RO(nt_abnormal_status);

static int charge_pump_enable_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	int ret;
	bool en = false;

	if (!nci || !nci->info)
		return -EINVAL;

	if(nci->info->dvchg1_dev){
		ret = charger_dev_is_enabled(nci->info->dvchg1_dev, &en);
		if (ret >= 0 && en) {
			pr_info("%s: CP is enabled!\n",__func__);
		}else{
			pr_info("%s: CP is disabled or not \n",__func__);
		}
	}
	seq_printf(m, "%d\n", en);
	return 0;
}
PROC_FOPS_RO(charge_pump_enable);

static int is_aging_test_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	seq_printf(m, "%d\n", nci->is_aging_test);
	return 0;
}
static ssize_t is_aging_test_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int is_aging_test,len;
	char desc[32];
	struct mtk_charger *info;
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}

	if(sscanf(desc, "%d", &is_aging_test) == 1){
		nci->is_aging_test = is_aging_test;
		if(is_aging_test)
			info->aging_mode = true;
		else
			info->aging_mode = false;
		pr_info("%s: is_aging_test = %d\n",__func__, is_aging_test);
	}
	return count;
}
PROC_FOPS_RW(is_aging_test);

static int nt_data_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	seq_printf(m, "%d %d %d %d %d %d %d %d %d %d\n", nci->nt_data[0], nci->nt_data[1],
		nci->nt_data[2], nci->nt_data[3],nci->nt_data[4], nci->nt_data[5],
		nci->nt_data[6], nci->nt_data[7],nci->nt_data[8], nci->nt_data[9]);
	return 0;
}

static ssize_t nt_data_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int len;
	char desc[32];
	struct usb_temp_parameter *usb_temp_parameter;
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (count <= 0)
		return -EINVAL;
	usb_temp_parameter = get_usb_temp_parameter();

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';
	if (!nci || !nci->info)
		return -EINVAL;
	len = sscanf(desc,"%d %d %d %d %d %d %d %d %d %d",
		&(nci->nt_data[0]),&(nci->nt_data[1]),&(nci->nt_data[2]),&(nci->nt_data[3]),
		&(nci->nt_data[4]),&(nci->nt_data[5]),&(nci->nt_data[6]),&(nci->nt_data[7]),
		&(nci->nt_data[8]),&(nci->nt_data[9]));

	if(len == 10){
		if(nci->nt_data[0] == ONLINE_PARAM_USB_TEMP){
			usb_temp_parameter->sleep_ms = nci->nt_data[1];
			usb_temp_parameter->usb_temp = nci->nt_data[2];
			usb_temp_parameter->usb_battery_gap_temp = nci->nt_data[3];
			usb_temp_parameter->usb_temp_raise_time = nci->nt_data[4];
			usb_temp_parameter->usb_temp_raise_degree = nci->nt_data[5];
			usb_temp_parameter->usb_temp_print_log_enable = nci->nt_data[6];
			set_usb_temp_parameter(usb_temp_parameter);
		}else{
			pr_err("%s data_0 is false\n", __func__);
		}
	} else {
		pr_err("%s failed\n", __func__);
	}
	return count;
}
PROC_FOPS_RW(nt_data);

static int scenario_fcc_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	if(nci){
		pr_info("%s: fcc_show %d ma\n",__func__, nci->fcc);
		seq_printf(m, "%d\n", nci->fcc);
	}
	return 0;
}
static ssize_t scenario_fcc_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int fcc = 0;
	int len = 0;
	char desc[32];
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (count <= 0){
		pr_info("%s: count <= 0\n", __func__);
		return -EINVAL;
	}
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)){
		pr_info("%s: copy frome user failed!\n", __func__);
		return -EFAULT;
	}
	desc[len] = '\0';
	if(sscanf(desc, "%d", &fcc)){
		nci->fcc = fcc;
		pr_info("%s: fcc %d ma\n",__func__, nci->fcc);
		cooling_state_to_charger_limit(nci);
	}
	return count;
}
PROC_FOPS_RW(scenario_fcc);

static int typec_cc_orientation_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;
	int ret = -1;

	ret = tcpm_inquire_cc_polarity(nci->tcpc_dev);
	pr_info("%s: cc_polarity %d \n", __func__,ret+1);
	seq_printf(m, "%d\n", ret+1);
	return 0;
}
PROC_FOPS_RO(typec_cc_orientation);

static int ship_mode_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: ship_mode_en %d \n", __func__,nci->ship_mode_en);
	seq_printf(m, "%d\n", nci->ship_mode_en);
	return 0;
}

static ssize_t ship_mode_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned long magic = 0;
	int len = 0;
	char desc[32];
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';
	if(sscanf(desc, "%d", &magic)){
		pr_err("%s:magic :%ld\n", __func__,magic);
		if(magic == SHIP_MODE_MAGIC)
			nci->ship_mode_en = true;
		else
			nci->ship_mode_en = false;
	}
	return count;
}
PROC_FOPS_RW(ship_mode);

static int real_soc_proc_show(struct seq_file *m, void *v)
{
	int soc = 0;
	struct mtk_battery *gm = get_battery_entry();

	if (!gm)
		return -EINVAL;
	soc = gm->soc;
	seq_printf(m, "%d\n", soc);
	return 0;
}
PROC_FOPS_RO(real_soc);

static int nt_otg_enable_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: otg_enable: %d \n", __func__,nci->otg_enable);
	seq_printf(m, "%d\n", nci->otg_enable);
	return 0;
}

static ssize_t nt_otg_enable_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int otg_enable, len;
	int ret = 0;
	char desc[32];
	struct mtk_charger *info;
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}

	if(sscanf(desc, "%d", &otg_enable) == 1){
		if(otg_enable == OTG_ENABLE_MODE) {
			ret = tcpm_typec_change_role_postpone(nci->tcpc_dev,
							TYPEC_ROLE_DRP, true);
		} else if (otg_enable == OTG_DISABLE_MODE) {
			ret = tcpm_typec_change_role_postpone(nci->tcpc_dev,
							TYPEC_ROLE_SNK, true);
		} else {
			otg_enable = OTG_DISABLE_MODE;
		}
		nci->otg_enable = otg_enable;

		pr_info("%s: otg_enable:%d %s\n",__func__,
			otg_enable, !ret ? "successfully" : "failed");
	}

	return count;
}
PROC_FOPS_RW(nt_otg_enable);

static int chg_data_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", PROJECT_DATA_ID);
	return 0;
}
PROC_FOPS_RO(chg_data_id);

static ssize_t handle_fake_value(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos, enum nt_fake_value type)
{
	int value, len;
	char desc[32];
	struct mtk_battery *gm = get_battery_entry();
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	if(!sscanf(desc, "%d", &value)){
		pr_info("%s: write fake value failed! \n", __func__);
		return len;
	}
	switch(type){
		case FAKE_SOC:
			nci->fake_soc = value;
			gm->fixed_uisoc = value;
			pr_info("%s: fake_soc: %d \n", __func__,nci->fake_soc);
		break;
		case FAKE_TBAT:
			nci->fake_tbat = value;
			gm->fixed_bat_tmp = value;
			pr_info("%s: fake_tbat: %d \n", __func__,nci->fake_tbat);
		break;
		case FAKE_IBAT:
			nci->fake_ibat = value;
			gm->fixed_bat_i = value;
			pr_info("%s: fake_ibat: %d \n", __func__,nci->fake_ibat);
		break;
		case FAKE_VBAT:
			nci->fake_vbat = value;
			gm->fixed_bat_v = value;
			pr_info("%s: fake_vbat: %d \n", __func__,nci->fake_vbat);
		break;
		case FAKE_TUSB:
			nci->fake_tusb = value;
			set_usb_temperature(value);
			pr_info("%s: fake_tusb: %d \n", __func__,nci->fake_tusb);
		break;
		default:
			break;
	}

	return count;
}

static int fake_soc_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: fake_soc: %d \n", __func__,nci->fake_soc);
	seq_printf(m, "%d\n", nci->fake_soc);
	return 0;
}

static ssize_t fake_soc_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	return handle_fake_value(file, buffer, count , pos, FAKE_SOC);
}
PROC_FOPS_RW(fake_soc);

static int fake_tbat_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: fake_tbat: %d \n", __func__,nci->fake_tbat);
	seq_printf(m, "%d\n", nci->fake_tbat);
	return 0;
}

static ssize_t fake_tbat_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	return handle_fake_value(file, buffer, count , pos, FAKE_TBAT);
}
PROC_FOPS_RW(fake_tbat);

static int fake_ibat_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: fake_ibat: %d \n", __func__,nci->fake_ibat);
	seq_printf(m, "%d\n", nci->fake_ibat);
	return 0;
}

static ssize_t fake_ibat_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	return handle_fake_value(file, buffer, count , pos, FAKE_IBAT);
}
PROC_FOPS_RW(fake_ibat);

static int fake_vbat_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: fake_vbat: %d \n", __func__,nci->fake_vbat);
	seq_printf(m, "%d\n", nci->fake_vbat);
	return 0;
}

static ssize_t fake_vbat_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	return handle_fake_value(file, buffer, count , pos, FAKE_VBAT);
}
PROC_FOPS_RW(fake_vbat);

static int fake_tusb_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: fake_tusb: %d \n", __func__,nci->fake_tusb);
	seq_printf(m, "%d\n", nci->fake_tusb);
	return 0;
}

static ssize_t fake_tusb_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	return handle_fake_value(file, buffer, count , pos, FAKE_TUSB);
}
PROC_FOPS_RW(fake_tusb);

static void handle_cam_evt(struct file *file, int cam_on_off)
{
	struct mtk_charger *info;
	struct charger_data *pdata;
	struct nt_chg_info *nci = PDE_DATA(file_inode(file));

	if (!nci){
		pr_err("%s: nci is NULL! \n", __func__);
		return;
	}
	info = nci->info;
	if(!info){
		pr_err("%s: info is NULL! \n", __func__);
		return;
	}

	pdata = &info->chg_data[CHG2_SETTING];
	if(cam_on_off == NT_CAM_ON){
		nci->pre_nt_cam = NT_CAM_OFF;
		nci->cam_on_off = NT_CAM_ON;
		cancel_delayed_work_sync(&nci->cam_delay_work);
		queue_delayed_work(nci->cam_workqueue,
					   &nci->cam_delay_work,
					   msecs_to_jiffies(500));
	}else if(cam_on_off == NT_CAM_OFF){
		nci->pre_nt_cam = NT_CAM_ON;
		nci->cam_on_off = NT_CAM_OFF;
		info->enable_hv_charging = true;
		nci->cam_lmt = -1;
		pdata->thermal_input_current_limit = -1;
	}
	pr_err("%s: pre_nt_cam : %d, cam_on_off : %d, enable_hv_charging : %d, cam_lmt %d\n",
		__func__,nci->pre_nt_cam , cam_on_off, info->enable_hv_charging, nci->cam_lmt);
	return;
}

static int camera_on_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: camera_on: %d \n", __func__,nci->cam_on_off);
	seq_printf(m, "%d\n", nci->cam_on_off);
	return 0;
}

static ssize_t camera_on_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int camera_on = 0;
	int len = 0;
	char desc[32];

	if (count <= 0){
		pr_info("%s: count <= 0\n", __func__);
		return -EINVAL;
	}
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)){
		pr_info("%s: copy frome user failed!\n", __func__);
		return -EFAULT;
	}
	desc[len] = '\0';
	if(sscanf(desc, "%d", &camera_on)){
		pr_info("%s: camera_on %d ma\n",__func__, camera_on);
		handle_cam_evt(file,camera_on);
	}
	return count;
}
PROC_FOPS_RW(camera_on);

static int nt_cp_ctrl_proc_show(struct seq_file *m, void *v)
{
	pr_info("%s: g_nt_cp_ctrl: %d \n", __func__,g_nt_cp_ctrl);
	seq_printf(m, "%d\n", g_nt_cp_ctrl);
	return 0;
}

static ssize_t nt_cp_ctrl_proc_write(struct file *file,
	const char __user *buffer, size_t count, loff_t *pos)
{
	int len = 0;
	char desc[32];
	struct chg_alg_device *alg = NULL;
	struct pe50_algo_info *info = NULL;

	if (count <= 0){
		pr_info("%s: count <= 0\n", __func__);
		return -EINVAL;
	}
	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len)){
		pr_info("%s: copy frome user failed!\n", __func__);
		return -EFAULT;
	}
	desc[len] = '\0';
	if(sscanf(desc, "%d", &g_nt_cp_ctrl)){
		pr_info("%s: camera_on %d ma\n",__func__, g_nt_cp_ctrl);
	}
	alg = get_chg_alg_by_name("pe5");
	if(alg){
		info = chg_alg_dev_get_drvdata(alg);
		if(info && info->data){
			info->data->g_nt_cp_ctrl = g_nt_cp_ctrl;
		}
	}
	return count;
}
PROC_FOPS_RW(nt_cp_ctrl);

static int nt_qmax_proc_show(struct seq_file *m, void *v)
{
	int ret = 0;
	union power_supply_propval prop;
	struct power_supply *batpsy = power_supply_get_by_name("battery");

	if(batpsy){
		ret = power_supply_get_property(batpsy, POWER_SUPPLY_PROP_CHARGE_FULL, &prop);
		ret = prop.intval;
	}
	pr_info("%s: qmax = %d uAh!\n",__func__, ret);
	seq_printf(m, "%d\n", ret);
	return 0;
}
PROC_FOPS_RO(nt_qmax);

static int nt_quse_proc_show(struct seq_file *m, void *v)
{
	int ret = -1;
	struct mtk_gauge *gauge;
	static struct mtk_battery *gm;
	//union power_supply_propval prop;
	struct power_supply *gaugepsy = power_supply_get_by_name("mtk-gauge");

	if(gaugepsy) {
			gauge = (struct mtk_gauge *)power_supply_get_drvdata(gaugepsy);
			if (gauge) {
				gm = gauge->gm;
				ret = gm->nt_quse;
			}
	}
	pr_info("%s: quse = %d uAh!\n",__func__, ret);
	seq_printf(m, "%d\n", ret);
	return 0;
}
PROC_FOPS_RO(nt_quse);

static int nt_resistance_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", -100);
	return 0;
}
PROC_FOPS_RO(nt_resistance);

static int maxchargingvoltage_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: maxchargingvoltage: %d \n", __func__,nci->chg_vol_max);
	seq_printf(m, "%d\n", nci->chg_vol_max);
	return 0;
}

PROC_FOPS_RO(maxchargingvoltage);

static int maxchargingcurrent_proc_show(struct seq_file *m, void *v)
{
	struct nt_chg_info *nci = m->private;

	pr_info("%s: maxchargingcurrent: %d \n", __func__,nci->chg_icl_max);
	seq_printf(m, "%d\n", nci->chg_icl_max);
	return 0;
}

PROC_FOPS_RO(maxchargingcurrent);


const struct nt_proc entries[] = {
	PROC_ENTRY(usb_charger_en),
	PROC_ENTRY(voltage_adc),
	PROC_ENTRY(usb_temp),
	PROC_ENTRY(usb_real_type),
	PROC_ENTRY(nt_abnormal_status),
	PROC_ENTRY(charge_pump_enable),
	PROC_ENTRY(is_aging_test),
	PROC_ENTRY(nt_data),
	PROC_ENTRY(scenario_fcc),
	PROC_ENTRY(typec_cc_orientation),
	PROC_ENTRY(ship_mode),
	PROC_ENTRY(real_soc),
	PROC_ENTRY(nt_otg_enable),
	PROC_ENTRY(chg_data_id),
	PROC_ENTRY(fake_soc),
	PROC_ENTRY(fake_tbat),
	PROC_ENTRY(fake_ibat),
	PROC_ENTRY(fake_vbat),
	PROC_ENTRY(fake_tusb),
	PROC_ENTRY(camera_on),
	PROC_ENTRY(nt_cp_ctrl),
	PROC_ENTRY(nt_qmax),
	PROC_ENTRY(nt_quse),
	PROC_ENTRY(nt_resistance),
	PROC_ENTRY(maxchargingvoltage),
	PROC_ENTRY(maxchargingcurrent),
};
#endif

void _wake_up_nt_charger(struct nt_chg_info *nci)
{
	unsigned long flags;

	if (nci == NULL)
		return;
	spin_lock_irqsave(&nci->slock, flags);
	if (!nci->charger_wakelock->active)
		__pm_stay_awake(nci->charger_wakelock);
	spin_unlock_irqrestore(&nci->slock, flags);
	nci->charger_thread_timeout = true;
	wake_up_interruptible(&nci->wait_que);
}

static enum power_supply_property nt_charger_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *nt_charger_supplied_to[] = {
	"battery"
};

static int nt_psy_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct nt_chg_info *nci;
	struct mtk_charger *info;
	struct charger_device *chg;
	struct charger_device *dvchg;
	int ret = 0;
	struct chg_alg_device *alg = NULL;

	nci = (struct nt_chg_info *)power_supply_get_drvdata(psy);
	if (nci == NULL) {
		pr_err("%s: get nci failed\n", __func__);
		return -EINVAL;
	}
	if (nci->info == NULL) {
		pr_err("%s: get info failed\n", __func__);
		return -EINVAL;
	}
	info = nci->info;

	pr_debug("%s: psp:%d\n", __func__, psp);

	if (info->chg1_dev != NULL &&	 info->dvchg1_dev != NULL){
		chg = info->chg1_dev;
		dvchg = info->dvchg1_dev;
	}else {
		pr_err("%s: fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = false;
		alg = get_chg_alg_by_name("pe5");
		if (alg == NULL)
			pr_err("get pe5 fail\n");
		else {
			ret = chg_alg_is_algo_ready(alg);
			if (ret == ALG_RUNNING)
				val->intval = true;
			else{
				alg = get_chg_alg_by_name("pps");
				if (alg == NULL)
					pr_err("get pps fail\n");
				else {
					ret = chg_alg_is_algo_ready(alg);
					if (ret == ALG_RUNNING)
						val->intval = true;
				}
			}
		}
		break;
	default:
		break;
	}
	return 0;
}

static void nt_charger_external_power_changed(struct power_supply *psy)
{
	struct nt_chg_info *nci;

	nci = (struct nt_chg_info *)power_supply_get_drvdata(psy);
	if (nci == NULL) {
		pr_notice("%s: failed to get nci\n", __func__);
		return;
	}
	_wake_up_nt_charger(nci);
}

static enum alarmtimer_restart nt_charger_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct nt_chg_info *nci =	container_of(alarm, struct nt_chg_info, charger_timer);

	if (nci->is_suspend == false) {
		_wake_up_nt_charger(nci);
		pr_err("[%s][%d]\n",__func__,__LINE__);
	} else {
		__pm_stay_awake(nci->charger_wakelock);
		pr_err("[%s][%d]\n",__func__,__LINE__);
	}
	return ALARMTIMER_NORESTART;
}

static void nt_charger_start_timer(struct nt_chg_info *nci)
{
	struct timespec64 end_time, time_now;
	ktime_t ktime, ktime_now;
	int ret = 0;

	ret = alarm_try_to_cancel(&nci->charger_timer);
	if (ret < 0) {
		pr_err("%s: callback was running, skip timer\n", __func__);
		return;
	}

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	if(nci->typec_attach){
		nci->polling_interval = CHG_THREAD_INTERVAL_A;
	}else{
		nci->polling_interval = CHG_THREAD_INTERVAL_D;
	}
	end_time.tv_sec = time_now.tv_sec + nci->polling_interval;
	end_time.tv_nsec = time_now.tv_nsec + 0;
	nci->endtime = end_time;
	ktime = ktime_set(nci->endtime.tv_sec, nci->endtime.tv_nsec);

	pr_info("%s: alarm timer start [%d][%d]: %ld %ld\n", __func__,nci->typec_attach,
		nci->is_suspend, nci->endtime.tv_sec, nci->endtime.tv_nsec);
	alarm_start(&nci->charger_timer, ktime);
}

static void nt_charger_init_timer(struct nt_chg_info *nci)
{
	alarm_init(&nci->charger_timer, ALARM_BOOTTIME, nt_charger_alarm_timer_func);
	nt_charger_start_timer(nci);
}

#ifdef CONFIG_PM
static int nt_charger_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	ktime_t ktime_now;
	struct timespec64 now;
	struct nt_chg_info *nci;

	nci = container_of(notifier, struct nt_chg_info, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		nci->is_suspend = true;
		pr_err("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		nci->is_suspend = false;
		pr_err("%s: enter PM_POST_SUSPEND\n", __func__);
		ktime_now = ktime_get_boottime();
		now = ktime_to_timespec64(ktime_now);

		if (timespec64_compare(&now, &nci->endtime) >= 0 &&
			nci->endtime.tv_sec != 0 &&
			nci->endtime.tv_nsec != 0) {
			pr_err("%s: alarm timeout, wake up charger\n",
				__func__);
			__pm_relax(nci->charger_wakelock);
			nci->endtime.tv_sec = 0;
			nci->endtime.tv_nsec = 0;
			_wake_up_nt_charger(nci);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif /* CONFIG_PM */
static int chg_check_status(enum power_supply_property psp)
{
	int ret = 0;
	union power_supply_propval prop;
	struct power_supply *psy = NULL;

	psy = power_supply_get_by_name("aw32280-standalone");
	if (psy == NULL || IS_ERR(psy)) {
		psy = power_supply_get_by_name("sc8562-standalone");
	}
	if (psy == NULL || IS_ERR(psy)) {
		pr_err("%s Couldn't get psy\n", __func__);
	}else{
		ret = power_supply_get_property(psy, psp, &prop);
		ret = prop.intval;
	}
	return ret;
}

static void nt_update_wakeup_status(struct nt_chg_info *nci)
{
	int vbus = 0;
	int Vinput_present = 0;
	static int Vinput_present_pre;
	bool boost_en = false;

	boost_en = get_usb_otg_status();
	vbus = chg_check_vbus(nci);
	Vinput_present = (vbus > PLUGIN_VOLTAGE);
	if (Vinput_present_pre != Vinput_present){
		Vinput_present_pre = Vinput_present;
		if (Vinput_present && (boost_en == false)) {
			if (nci->chg_wake){
				pr_info("chg_wake stay_awake");
				__pm_stay_awake(nci->chg_wake);
			}
		} else {
			if (nci->chg_wake) {
				pr_info("chg_wake relax");
				__pm_relax(nci->chg_wake);
			}
			nci->is_hvcharger = false;
		}
	}
}

static int ibus_accuracy_compensation(struct nt_chg_info *nci , int id)
{
	if (!nci){
		pr_info("%s: nci is null!\n", __func__);
		return -EINVAL;
	}
	/*Compensation only for aw32280*/
	switch(id){
		case AW32280_CHIP_ID3:
			if(nci->fcc > ABOVE_IBAT_COMPENSATION)
				nci->adc_compensation = ADC_ACCURACY_COMPENSATION*1000;
			else
				nci->adc_compensation = 0;
		break;
		case SC8562_DEVICE_ID:
			nci->adc_compensation = 0;
		break;
		default:
			break;
	}
	return 0;
}

static int check_aging_mode_status(struct nt_chg_info *nci)
{
	struct mtk_charger *info;
	struct charger_data *pdata;

	if (!nci){
		pr_info("%s: nci is null!\n", __func__);
		return -EINVAL;
	}
	info = nci->info;
	if(info == NULL){
		pr_err("%s:info is null!!\n", __func__);
		return 0;
	}
	pdata = &info->chg_data[CHG1_SETTING];
	if(info->aging_mode){
		if(pdata->input_current_limit <= 1500000)
			pdata->input_current_limit = 1500000;
	}
	return 0;
}

static int nt_ctrl_charge(struct nt_chg_info *nci)
{
	struct mtk_charger *info;
	struct charger_data *pdata;
	struct mtk_battery *gm = get_battery_entry();
	struct battery_data *bat_data;
	struct chg_alg_device *alg;
	int i;
	int val = 0;
	int need_limmit = nci->cam_on_off;

	if(!gm){
		pr_err("%s: gm is NULL!\n",	__func__);
		return -1;
	}
	bat_data = &gm->bs_data;
	if(!bat_data){
		pr_err("%s: bat_data is NULL!\n",	__func__);
		return -1;
	}
	info = nci->info;
	if(!info){
		pr_err("%s: info is NULL!\n",	__func__);
		return -1;
	}
	pdata = &info->chg_data[CHG2_SETTING];
	if(bat_data->bat_capacity < NT_CAM_LMT_BAT_LEVEL){
		pr_info("%s: BAT level is less than %d\n",	__func__, NT_CAM_LMT_BAT_LEVEL);
		info->enable_hv_charging = true;
		nci->cam_lmt = -1;
		pdata->thermal_input_current_limit = -1;
		return EXIT_WQ;
	}
	pr_info("%s: need_limmit :%d ,pre_nt_cam : %d ,nt_ctrl_count : %d, nt_ctrl_timeout : %d \n",
		__func__, need_limmit, nci->pre_nt_cam, nt_ctrl_count, nt_ctrl_timeout);

	if (nci->pre_nt_cam == NT_CAM_OFF) {
		nt_ctrl_count = 0;
		nt_ctrl_timeout = false;
	}

	if (nci->pre_nt_cam == NT_CAM_ON) {
		nt_ctrl_count++;
	}

	if (nt_ctrl_count > NT_COUNT_FIVE_MINUTE){
		if (nt_ctrl_timeout == false) {
			info->enable_hv_charging = true;
			nci->cam_lmt = -1;
			pdata->thermal_input_current_limit = -1;
		}
		nt_ctrl_timeout = true;
		return EXIT_WQ;
	}

	if (nci->pre_nt_cam == need_limmit) {
		return RERUN_WQ;
	}

	nci->pre_nt_cam = need_limmit;

	if (need_limmit == NT_CAM_ON) {
		pdata->thermal_input_current_limit = NT_CAM_LMT_IBUS;
		for (i = 0; i < MAX_ALG_NO; i++) {
			alg = info->alg[i];
			if (alg == NULL)
				continue;

			chg_alg_get_prop(alg, ALG_MAX_VBUS, &val);
			if (val > 5000 && chg_alg_is_algo_running(alg))
				chg_alg_stop_algo(alg);

			pr_info("%s: Stop hv charging. en_hv:%d alg:%s alg_vbus:%d\n",
				__func__, info->enable_hv_charging,	dev_name(&alg->dev), val);
		}
		charger_dev_set_input_current(info->chg1_dev, NT_CAM_LMT_IBUS);
		charger_dev_enable(info->chg1_dev, true);
		info->enable_hv_charging = false;
		nci->cam_lmt = NT_CAM_LMT_IBUS;
		nt_ctrl_count++;
		return RERUN_WQ;
	} else if (need_limmit == NT_CAM_OFF){
		info->enable_hv_charging = true;
		nci->cam_lmt = -1;
		pdata->thermal_input_current_limit = -1;
		charger_dev_set_input_current(info->chg1_dev, NT_CAM_RESTORE_IBUS);
		nt_ctrl_count = 0;
		return EXIT_WQ;
	}
	return RERUN_WQ;
}

static void nt_cam_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct nt_chg_info *nci;
	int ret;

	delay_work = to_delayed_work(work);
	nci = container_of(delay_work, struct nt_chg_info, cam_delay_work);
	ret = nt_ctrl_charge(nci);
	if(ret > 0){
		queue_delayed_work(nci->cam_workqueue,
				   &nci->cam_delay_work,
				   msecs_to_jiffies((nci->cam_on_off == NT_CAM_ON) ? NT_CAM_ON_INTERVAL_MS : NT_CAM_POLL_INTERVAL_MS));
	}
}

static int nt_charger_routine_thread(void *arg)
{
	int ret  = 0;
	u32 val  = 0;
	int fcc = 0,fv = 0,icl = 0,ibus = 0,chg_type = 0,real_cap = 0,usb_sus = 0,boost_cc = 0,boost_cv = 0;
	int pump = 0, htemp_charge = 0,battTemp = 0,plugGpio = 0,chg_status = 0,AICR = 0,CV = 0, MIVR = 0, ICHG  = 0;
	char BattInit = 1, soc_time = 0;
	static int soc = 0, soc_pre = 0;
	bool pump_en = false, chg_en = false,boost_en = false,ovp = false, wpc = false, wls_sus = false;
	unsigned long flags = 0;
	static struct charger_data *pdata = NULL;
	struct nt_chg_info *nci = arg;
	char keyinfo_buffer_temp[DUMP_MSG_BUF_SIZE] = "\0";

	while (1) {
		ret = wait_event_interruptible(nci->wait_que,
			(nci->charger_thread_timeout == true));
		if (ret < 0) {
			pr_err("%s: wait event been interrupted(%d)\n", __func__, ret);
			continue;
		}

		mutex_lock(&nci->charger_lock);
		spin_lock_irqsave(&nci->slock, flags);
		if (!nci->charger_wakelock->active)
			__pm_stay_awake(nci->charger_wakelock);
		spin_unlock_irqrestore(&nci->slock, flags);
		nci->charger_thread_timeout = false;
		check_chg_type(nci);
		nci->usbTemp = get_usb_temperature();
		nci->notify_code = check_abnormal_status(nci);
		pump = charger_dev_get_cp_status(nci->info->dvchg1_dev, CP_DEV_REVISION);
		ibus_accuracy_compensation(nci,pump);
		cooling_state_to_charger_limit(nci);
		ovp = charger_dev_get_cp_status(nci->info->dvchg1_dev, CP_DEV_OVPGATE);
		wpc = charger_dev_get_cp_status(nci->info->dvchg1_dev, CP_DEV_WPCGATE);
		charger_dev_is_enabled(nci->info->dvchg1_dev, &pump_en);
		charger_dev_is_enabled(nci->info->chg1_dev, &chg_en);

		fcc = chg_check_battery_info(nci,POWER_SUPPLY_PROP_CURRENT_NOW);
		fv = nci->info->data.battery_cv;
		pdata = &nci->info->chg_data[CHG1_SETTING];
		if(pdata)
			icl = _uA_to_mA(pdata->input_current_limit);
		ibus = chg_check_ibus(nci);
		if (ibus > nci->chg_icl_max) {
			nci->chg_icl_max = ibus;
		}
		chg_type = chg_check_pmic_info(nci,POWER_SUPPLY_PROP_TYPE);
		soc_pre = soc;
		soc = chg_check_battery_info(nci,POWER_SUPPLY_PROP_CAPACITY);
		real_cap = chg_check_battery_info(nci,POWER_SUPPLY_PROP_CHARGE_COUNTER);
		htemp_charge = chg_check_status(POWER_SUPPLY_PROP_TEMP);
		plugGpio = nci->wd0_state;
		battTemp = chg_check_battery_info(nci,POWER_SUPPLY_PROP_TEMP);
		chg_status = chg_check_pmic_info(nci,POWER_SUPPLY_PROP_STATUS);
		boost_en = get_usb_otg_status();
		charger_dev_get_input_current(nci->info->chg1_dev,&AICR);
		charger_dev_get_constant_voltage(nci->info->chg1_dev,&CV);
		charger_dev_get_mivr(nci->info->chg1_dev,&MIVR);
		charger_dev_get_charging_current(nci->info->chg1_dev,&ICHG);
		charger_dev_get_boost_current_limit(nci->info->chg1_dev, &boost_cc);
		charger_dev_get_boost_voltage_limit(nci->info->chg1_dev, &val);
		boost_cv = ((val & BOOST_CV_MAX) - BOOST_CV_MIN)*BOOST_CV_OFFSET + BOOST_CV_BASE;
		check_aging_mode_status(nci);
		scnprintf(keyinfo_buffer_temp,DUMP_MSG_BUF_SIZE - 1,"pump:0x%x,pump_en:%d,ovp:%d,wpc:%d, fcc:%d,fv:%d,icl:%d,ibus_ma:%d,usb_sus:%d,wls_sus:%d,BattInit:%d, chg_type:%d,soc:%d,soc_time:%d,soc_pre:%d,real_cap:%d,htemp_charge:%d,plug_in:%d,plugGpio:%d,usbTemp:%d,battTemp:%d,CHG_EN:%x,CHG_STATUS:%x,AICL:%d,AICR:%d,CV:%d,MIVR:%d,ICHG:%d,BOOST_CV:%d,BOOST_EN:%d,BOOST_CC:%d,aging_mode:%d,NT_CHG_TYPE:%s,CAM_LMT:{[ON_OFF]%d,[HV_CHG]%d,[LMT_C]%d,g_nt_cp_ctrl:%d,hvcharger:%d} \n",\
			pump,pump_en,ovp,wpc,\
			fcc,fv/1000,icl,ibus,usb_sus,wls_sus,\
			BattInit,chg_type,soc,soc_time,soc_pre,real_cap,\
			htemp_charge,nci->typec_attach,plugGpio,nci->usbTemp,battTemp,\
			chg_en,chg_status,_uA_to_mA(pdata->input_current_limit_by_aicl),\
			_uA_to_mA(AICR),CV/1000,MIVR/1000,_uA_to_mA(ICHG),boost_cv,boost_en,\
			boost_cc,nci->info->aging_mode,POWER_SUPPLY_USB_TYPE_TEXT[nci->chg_type],\
			nci->cam_on_off,nci->info->enable_hv_charging,nci->cam_lmt, g_nt_cp_ctrl,nci->is_hvcharger);
		pr_info("%s \n",keyinfo_buffer_temp);
		pr_info("[NT]lst_rnd_alg_idx : %d\n",nci->info->lst_rnd_alg_idx);
		if (nci->charger_thread_polling == true)
			nt_charger_start_timer(nci);
		spin_lock_irqsave(&nci->slock, flags);
		__pm_relax(nci->charger_wakelock);
		spin_unlock_irqrestore(&nci->slock, flags);
		nt_update_wakeup_status(nci);
		mutex_unlock(&nci->charger_lock);
	}
	return 0;
}
static void nt_chg_parse_dt(struct nt_chg_info *nci, struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val;

	/* single charger */
	if (of_property_read_u32(np, "sc_input_current", &val) >= 0)
		nci->sc_input_current = val;
	else {
		pr_notice("use default SC_INPUT_CURRENT\n");
		nci->sc_input_current = 3200000;
	}

	if (of_property_read_u32(np, "sc_charger_current", &val) >= 0)
		nci->sc_charger_current = val;
	else {
		pr_notice("use default SC_CHARGING_CURRENT\n");
		nci->sc_charger_current = 3200000;
	}

	if (of_property_read_u32(np, "pd_input_current", &val) >= 0)
		nci->sc_input_current = val;
	else {
		pr_notice("use default PD_INPUT_CURRENT\n");
		nci->pd_input_current = 3200000;
	}

	if (of_property_read_u32(np, "pd_charger_current", &val) >= 0)
		nci->pd_charger_current = val;
	else {
		pr_notice("use default PD_CHARGER_CURRENT\n");
		nci->pd_charger_current = 3200000;
	}

	if (of_property_read_u32(np, "ac_charger_input_current", &val) >= 0)
		nci->ac_charger_input_current = val;
	else {
		pr_notice("use default AC_CHARGER_INPUT_CURRENT\n");
		nci->ac_charger_input_current = 3200000;
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) >= 0) {
		nci->ac_charger_current = val;
	} else {
		pr_notice("use default AC_CHARGER_CURRENT\n");
		nci->ac_charger_current = 2050000;
	}
	nci->cp_workmode = CP_WORKMODE_DEF;
	nci->fcc = -1;
	pr_notice("nt_chg_parse_dt:pe2{%d,%d},dcp{%d,%d}\n",nci->sc_input_current,
		nci->sc_charger_current,nci->ac_charger_input_current,
		nci->ac_charger_current);
}

static int nt_chg_probe(struct platform_device *pdev)
{
	struct nt_chg_info *nci;
	int ret;
	int i;
	char *name = NULL;
	struct proc_dir_entry *entry;

	nci = devm_kzalloc(&pdev->dev, sizeof(*nci), GFP_KERNEL);
	if (!nci)
		return -ENOMEM;

	nci->dev = &pdev->dev;
	platform_set_drvdata(pdev, nci);
	nt_chg_parse_dt(nci,&pdev->dev);
	mutex_init(&nci->charger_lock);
	mutex_init(&nci->proc_lock);
	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",	"nt_chg suspend wakelock");
	nci->charger_wakelock = wakeup_source_register(NULL, name);
	nci->chg_wake = wakeup_source_register(NULL,"NT_CHG_WAKE");
	spin_lock_init(&nci->slock);
	init_waitqueue_head(&nci->wait_que);
	nci->polling_interval = CHG_THREAD_INTERVAL_A;
	nci->ship_mode_en = false;
	nci->otg_enable = OTG_DISABLE_MODE;
	nci->charger_thread_polling = true;
	nci->pre_input_current_limmit = -1;
	nci->pre_aicl_limmit = -1;
	nci->fake_soc = FAKE_BATT_MAGIC;
	nci->fake_tbat = FAKE_BATT_MAGIC;
	nci->fake_ibat = FAKE_BATT_MAGIC;
	nci->fake_vbat = FAKE_BATT_MAGIC;
	nci->fake_tusb = FAKE_BATT_MAGIC;
	nci->cam_lmt = -1;
	nci->pre_nt_cam = -1;
	nci->cam_on_off = NT_CAM_OFF;
	nci->is_hvcharger = false;
	nci->chg_vol_max = 0;
	nci->chg_icl_max = 0;
	nt_charger_init_timer(nci);

	nci->chgpsy = power_supply_get_by_name("mtk-master-charger");
	if (IS_ERR_OR_NULL(nci->chgpsy)) {
		pr_err("%s: Failed to get charger psy\n", __func__);
		return PTR_ERR(nci->chgpsy );
	}

	nci->info =(struct mtk_charger *)power_supply_get_drvdata(nci->chgpsy);
	if (nci->info == NULL) {
		pr_err("%s:get charger device fail\n", __func__);
		return -ENODEV;
	}

	nci->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (nci->tcpc_dev == NULL) {
		pr_err("%s:get tcpc device type_c_port0 fail\n", __func__);
		return -ENODEV;
	}

	nci->pd_nb.notifier_call = pd_tcp_notifier_call;

	ret = register_tcp_dev_notifier(nci->tcpc_dev, &nci->pd_nb, TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		pr_err("%s: register tcpc notifer fail\n", __func__);
		return -EINVAL;
	}

	nci->psy_desc.name = "nt-chg";
	nci->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
	nci->psy_desc.properties = nt_charger_psy_properties;
	nci->psy_desc.num_properties = ARRAY_SIZE(nt_charger_psy_properties);
	nci->psy_desc.get_property = nt_psy_charger_get_property;
	nci->psy_desc.external_power_changed = nt_charger_external_power_changed;
	nci->psy_cfg.drv_data = nci;
	nci->psy_cfg.supplied_to = nt_charger_supplied_to;
	nci->psy_cfg.num_supplicants = ARRAY_SIZE(nt_charger_supplied_to);
	nci->psy = power_supply_register(&pdev->dev, &nci->psy_desc, &nci->psy_cfg);

#if IS_ENABLED(CONFIG_PROC_FS)
	chg_proc_dir =  proc_mkdir("charger", NULL);
	if(!chg_proc_dir){
		pr_err("proc dir creates failed !!\n");
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create_data(entries[i].name, 0660, chg_proc_dir, entries[i].fops, nci))
			pr_info("%s: create /proc/charger/%s failed\n",__func__, entries[i].name);
	}

	chg_usb_proc_dir =  proc_mkdir("usb", chg_proc_dir);
	if(!chg_usb_proc_dir){
		pr_err("proc dir creates failed !!\n");
		ret =  -ENODEV;
	}

	entry = proc_create_data("temp", 0660, chg_usb_proc_dir, &temp_proc_fops, nci);
	if(!entry) {
		pr_err("proc file creates failed !!\n");
		ret =  -ENODEV;
		goto fail_procfs;
	}
#endif
#ifdef CONFIG_PM
	if (register_pm_notifier(&nci->pm_notifier)) {
		pr_err("%s: register pm failed\n", __func__);
		return -ENODEV;
	}
	nci->pm_notifier.notifier_call = nt_charger_pm_event;
#endif /* CONFIG_PM */
 	nci->cam_workqueue = create_singlethread_workqueue("nt_cam_work");
 	INIT_DELAYED_WORK(&nci->cam_delay_work, nt_cam_work);
	kthread_run(nt_charger_routine_thread, nci, "nt_chg_thread");
	pr_info("%s: successfully\n", __func__);
	return ret;
#if IS_ENABLED(CONFIG_PROC_FS)
fail_procfs:
	remove_proc_subtree("charger",NULL);
#endif
	return ret;
}

static int nt_chg_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	return 0;
}
static void nt_chg_shutdown(struct platform_device *pdev)
{
	struct mtk_charger *info;
	struct nt_chg_info *nci = platform_get_drvdata(pdev);

	if(nci){
		info = nci->info;
		if(info && nci->ship_mode_en){
			charger_dev_enable_ship_mode(info->chg1_dev,true);
			msleep(3000);
		}
	}
}
static const struct of_device_id __maybe_unused nt_chg_of_id[] = {
	{ .compatible = "Nothing,nt_chg"},
	{}
};
MODULE_DEVICE_TABLE(of, nt_chg_of_id);

static struct platform_driver nt_chg_driver = {
	.driver = {
		.name = "nt_chg",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(nt_chg_of_id),
	},
	.probe = nt_chg_probe,
	.remove = nt_chg_remove,
	.shutdown = nt_chg_shutdown,
};

static int __init nt_chg_init(void)
{
	return platform_driver_register(&nt_chg_driver);
}
device_initcall_sync(nt_chg_init);

static void __exit nt_chg_exit(void)
{
	platform_driver_unregister(&nt_chg_driver);
}
module_exit(nt_chg_exit);

MODULE_DESCRIPTION("NT CHARGER MANAGER Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(NT_CHG_DRV_VERSION);
