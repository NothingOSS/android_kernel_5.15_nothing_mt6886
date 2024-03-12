#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/poll.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include<linux/timer.h>
#include<linux/jiffies.h>

//#include "lcm_drv.h"
#include "hardware_info.h"
//#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_LPDDR_INFO)
//#include "prize_custom_memory_list.h"
//#else
//#include "prize_custom_memory.h"
//#endif
#include <linux/fs.h>

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/fb.h>
#include <drm/drm_panel.h>
#include <linux/notifier.h>

//#include "../../../../gpu/drm/mediatek/mediatek_v2/mtk_disp_notify.h"

#define DEBUG_ON		0
#define HW_PRINT(fmt,arg...)           printk("[HW_INFO] "fmt"\n",##arg)
#define HW_ERROR(fmt,arg...)          printk("[HW_INFO] ERROR:"fmt"\n",##arg)
#define HW_DEBUG(fmt,arg...)          do{\
	if(DEBUG_ON)\
		printk("[HW_INFO] [%d]"fmt"\n",__LINE__, ##arg);\
		}while(0)


//#if defined(CONFIG_MTK_AUXADC)
//extern int IMM_get_adc_channel_num(char *channel_name, int len);
//extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
//#endif
//#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
//extern int lcm_auxadc_get_lcm_v(void);
//#endif

int len = 0;
//static struct notifier_block fb_nb;
#if defined(CONFIG_MTK_FB)
struct tag_video_lfb {
        u64 fb_base;
        u32 islcmfound;
        u32 fps;
        u32 vram;
        char lcmname[1];        /* this is the minimum size */
};
#endif

struct class *hardware_info_class;
struct hardware_info current_tof_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_lcm_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_camera_info[5] =
{
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
};
struct hardware_info current_tp_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_nfc_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_alsps_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_gsensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_gyroscope_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_msensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_barosensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_sarsensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_wireless_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_fingerprint_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_flash_lpddr_info =
{
	"unknow","unknow","unknow","unknow",
};
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
struct hardware_info current_battery_info =
{
	"unknow","unknow","unknow","unknow","unknow",
};
//EXPORT_SYMBOL_GPL(current_battery_info);
#endif
struct hardware_info current_coulo_info =
{
	"unknow","unknow","unknow","unknow",
};
//EXPORT_SYMBOL_GPL(current_coulo_info);
struct hardware_info current_mmc_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_line_motor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_cp_info =
{
	"unknow","unknow","unknow","unknow",
};
EXPORT_SYMBOL_GPL(current_line_motor_info);

EXPORT_SYMBOL_GPL(current_lcm_info);
EXPORT_SYMBOL_GPL(current_camera_info);
EXPORT_SYMBOL_GPL(current_tp_info);
//EXPORT_SYMBOL_GPL(current_nfc_info);
EXPORT_SYMBOL_GPL(current_alsps_info);
EXPORT_SYMBOL_GPL(current_gsensor_info);
EXPORT_SYMBOL_GPL(current_gyroscope_info);
EXPORT_SYMBOL_GPL(current_msensor_info);
EXPORT_SYMBOL_GPL(current_barosensor_info);
//EXPORT_SYMBOL_GPL(current_fingerprint_info);
//EXPORT_SYMBOL_GPL(current_flash_lpddr_info);
EXPORT_SYMBOL_GPL(current_sarsensor_info);
//EXPORT_SYMBOL_GPL(current_tof_info);
EXPORT_SYMBOL_GPL(current_cp_info);
//mt_battery_meter.h
/*static void dev_get_current_tof_info(char *buf)
{
    char *p = buf;
	HW_PRINT("hardware_info_tof");

	if(strcmp(current_tof_info.chip,"unknow") == 0)
	 	return ;

	     p += sprintf(p, "[TOF]:\n");
         p += sprintf(p, "  chip:%s\n", current_tof_info.chip);
         p += sprintf(p, "  id:%s\n", current_tof_info.id);
         p += sprintf(p, "  vendor:%s\n",current_tof_info.vendor);
         p += sprintf(p, "  more:%s\n", current_tof_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}*/

static void  dev_get_current_line_motor_info(char *buf)
{
    char *p = buf;
	HW_PRINT("dev_get_current_line_motor_info");
	if(strcmp(current_line_motor_info.chip,"unknow") == 0)
	    return ;

	p += sprintf(p, "\n[line_motor]:\n");
	p += sprintf(p, "  chip:%s\n", current_line_motor_info.chip);
	p += sprintf(p, "  id:%s\n", current_line_motor_info.id);
	p += sprintf(p, "  vendor:%s\n",current_line_motor_info.vendor);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
extern unsigned int get_lcm_id(void);
extern void lcm_get_hardware_info(struct hardware_info *hwinfo);
static void dev_get_current_lcm_info(char *buf)
{
    char *p = buf;
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)||IS_ENABLED(CONFIG_MTK_AUXADC)
	int ret = 0;
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)||defined(CONFIG_SC27XX_ADC)||defined(CONFIG_MTK_AUXADC)
	int lcm_volt = 0;
#endif
#if defined(CONFIG_MTK_AUXADC)
	int lcm_volt_ch = 0;
#endif
#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	const char *compatible_str = NULL;
#elif defined(CONFIG_MTK_FB)
	struct device_node *chosen_node;
	struct tag_video_lfb *videolfb_tag = NULL;
	unsigned long size = 0;
#endif
	static unsigned int lcm_id = 0;
	static unsigned int lcm_id_fail = 0;

	HW_PRINT("hardware_info_lcm");
	if (!lcm_id_fail)
	{
		lcm_id = get_lcm_id();
		sprintf(current_lcm_info.id,"0x%06x",lcm_id);
		if (lcm_id == 0)
		{
			lcm_id_fail = 1;
		}
	}
	lcm_get_hardware_info(&current_lcm_info);

	if(strcmp(current_lcm_info.chip,"unknow") == 0){
	#if IS_ENABLED(CONFIG_DRM_MEDIATEK)
		dsi_node = of_find_compatible_node(NULL,NULL,"mediatek,dsi0");
		if (!IS_ERR_OR_NULL(dsi_node)){
			endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
			if (endpoint) {
				remote_node = of_graph_get_remote_port_parent(endpoint);
				if (!remote_node) {
					HW_PRINT("No panel connected");
					//return -ENODEV;
				}else{
					ret = of_property_read_string(remote_node, "compatible", &compatible_str);
					if (!ret){
						snprintf(&current_lcm_info.chip[0], sizeof(current_lcm_info.chip), compatible_str);
					}
				}
			}
		}else{
			HW_PRINT("get dsi0 node fail");
		}
	#elif defined(CONFIG_MTK_FB)
		chosen_node = of_find_node_by_path("/chosen");
		if (!chosen_node){
			chosen_node = of_find_node_by_path("/chosen@0");
		}
		if (chosen_node){
			videolfb_tag = (struct tag_video_lfb *)of_get_property(chosen_node,"atag,videolfb", (int *)&size);
			if (videolfb_tag) {
				snprintf(&current_lcm_info.chip[0], sizeof(current_lcm_info.chip), videolfb_tag->lcmname);
			}else{
				return;
			}
		}else{
			return;
		}
	#else
	 	return;
	#endif
	}


	p += sprintf(p, "[LCM]:\n");
	p += sprintf(p, "  chip:%s\n", current_lcm_info.chip);
	if(strcmp(current_lcm_info.id,"unknow") != 0){
		p += sprintf(p, "  id:%s\n", current_lcm_info.id);
	}
	if(strcmp(current_lcm_info.vendor,"unknow") != 0){
		p += sprintf(p, "  vendor:%s\n",current_lcm_info.vendor);
	}
	p += sprintf(p, "  more:%s", current_lcm_info.more);

#if defined(CONFIG_MTK_AUXADC)
	lcm_volt_ch = IMM_get_adc_channel_num("ADC_LCM_VOLTAGE",strlen("ADC_LCM_VOLTAGE"));
	if (lcm_volt_ch >= 0){
		ret = IMM_GetOneChannelValue_Cali(lcm_volt_ch, &lcm_volt);
		if (!ret) {
			p += sprintf(p, "  %duV", lcm_volt);
		}
	}
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)||defined(CONFIG_SC27XX_ADC)
	lcm_volt = lcm_auxadc_get_lcm_v();
	if (lcm_volt >= 0){
		p += sprintf(p, "  %dmV", lcm_volt);
	}
#endif

	p += sprintf(p, "\n");

	len += (p - buf);
	HW_PRINT("%s",buf);
}

struct hardware_info current_imgsensor_info[5] =
{
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
};
EXPORT_SYMBOL_GPL(current_imgsensor_info);

void camera_get_hardware_info(struct hardware_info *hwinfo)
{
	int i;
	if (hwinfo != NULL) {
	    for(i = 0;i < 5;i++) {
	        strcpy(hwinfo[i].chip, current_imgsensor_info[i].chip);
	        strcpy(hwinfo[i].id, current_imgsensor_info[i].id);
	        strcpy(hwinfo[i].vendor, current_imgsensor_info[i].vendor);
	        strcpy(hwinfo[i].more, current_imgsensor_info[i].more);
	    }
	}
}

static void dev_get_current_camera_info(char *buf)
{
     char *p = buf;
	 HW_PRINT("dev_get_current_camera_info");

	 camera_get_hardware_info(&current_camera_info[0]);
	 if(strcmp(current_camera_info[0].chip,"unknow") != 0)
	 {
	    p += sprintf(p, "\n[Main Camera]:\n");
	    p += sprintf(p, "  chip:%s\n", current_camera_info[0].chip);
	    p += sprintf(p, "  id:%s\n", current_camera_info[0].id);
	    p += sprintf(p, "  vendor:%s\n",current_camera_info[0].vendor);
	    p += sprintf(p, "  more:%s\n", current_camera_info[0].more);
	 }

	 if(strcmp(current_camera_info[1].chip,"unknow") != 0)
	 {
	    p += sprintf(p, "\n[Sub Camera]:\n");
	    p += sprintf(p, "  chip:%s\n", current_camera_info[1].chip);
	    p += sprintf(p, "  id:%s\n", current_camera_info[1].id);
	    p += sprintf(p, "  vendor:%s\n",current_camera_info[1].vendor);
	    p += sprintf(p, "  more:%s\n", current_camera_info[1].more);
	 }

	 if(strcmp(current_camera_info[2].chip,"unknow") != 0)
	 {
	    p += sprintf(p, "\n[Main2 Camera]:\n");
	    p += sprintf(p, "  chip:%s\n", current_camera_info[2].chip);
	    p += sprintf(p, "  id:%s\n", current_camera_info[2].id);
	    p += sprintf(p, "  vendor:%s\n",current_camera_info[2].vendor);
	    p += sprintf(p, "  more:%s\n", current_camera_info[2].more);
	 }

	 if(strcmp(current_camera_info[3].chip,"unknow") != 0)
	 {
	    p += sprintf(p, "\n[Main3 Camera]:\n");
	    p += sprintf(p, "  chip:%s\n", current_camera_info[3].chip);
	    p += sprintf(p, "  id:%s\n", current_camera_info[3].id);
	    p += sprintf(p, "  vendor:%s\n",current_camera_info[3].vendor);
	    p += sprintf(p, "  more:%s\n", current_camera_info[3].more);
	 }

	 if(strcmp(current_camera_info[4].chip,"unknow") != 0)
	 {
	    p += sprintf(p, "\n[Main4 Camera]:\n");
	    p += sprintf(p, "  chip:%s\n", current_camera_info[4].chip);
	    p += sprintf(p, "  id:%s\n", current_camera_info[4].id);
	    p += sprintf(p, "  vendor:%s\n",current_camera_info[4].vendor);
	    p += sprintf(p, "  more:%s\n", current_camera_info[4].more);
	 }
	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
static void  dev_get_current_tp_info(char *buf)
{
    char *p = buf;
    HW_PRINT("dev_get_current_tp_info");
    if(strcmp(current_tp_info.chip,"unknow") == 0)
	    return ;

    p += sprintf(p, "\n[Touch Panel]:\n");
	p += sprintf(p, "  chip:%s\n", current_tp_info.chip);
	p += sprintf(p, "  id:%s\n", current_tp_info.id);
	p += sprintf(p, "  vendor:%s\n",current_tp_info.vendor);
	p += sprintf(p, "  more:%s\n", current_tp_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
extern void stnfc_get_hardware_info(struct hardware_info *hwinfo);
static void  dev_get_current_nfc_info(char *buf)
{

	char *p = buf;
	HW_PRINT("dev_get_current_nfc_info");
	stnfc_get_hardware_info(&current_nfc_info);
	if(strcmp(current_nfc_info.chip,"unknow") == 0)
	 	return ;


	 p += sprintf(p, "\n[Nfc]:\n");
	 p += sprintf(p, "  chip:%s\n", current_nfc_info.chip);
	 p += sprintf(p, "  id:%s\n", current_nfc_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_nfc_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_nfc_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);

}
static void dev_get_current_alsps_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_alsps_info");
	if(strcmp(current_alsps_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[ALS/PS]:\n");
	p += sprintf(p, "  chip:%s\n", current_alsps_info.chip);
	// p += sprintf(p, "  id:%s\n", current_alsps_info.id);
	p += sprintf(p, "  vendor:%s\n",current_alsps_info.vendor);
	p += sprintf(p, "  more:%s\n", current_alsps_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}

static void dev_get_current_gsensor_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_gsensor_info");
	if(strcmp(current_gsensor_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[G-sensor]:\n");
	p += sprintf(p, "  chip:%s\n", current_gsensor_info.chip);
	// p += sprintf(p, "  id:%s\n", current_gsensor_info.id);
	p += sprintf(p, "  vendor:%s\n",current_gsensor_info.vendor);
	p += sprintf(p, "  more:%s\n", current_gsensor_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
static void dev_get_current_gyroscope_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_gyroscope_info");
	if(strcmp(current_gyroscope_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[Gyroscope]:\n");
	p += sprintf(p, "  chip:%s\n", current_gyroscope_info.chip);
	// p += sprintf(p, "  id:%s\n", current_gyroscope_info.id);
	p += sprintf(p, "  vendor:%s\n",current_gyroscope_info.vendor);
	p += sprintf(p, "  more:%s\n", current_gyroscope_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
static void dev_get_current_msensor_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_msensor_info");
	if(strcmp(current_msensor_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[M-sensor]:\n");
	p += sprintf(p, "  chip:%s\n", current_msensor_info.chip);
	// p += sprintf(p, "  id:%s\n", current_msensor_info.id);
	p += sprintf(p, "  vendor:%s\n",current_msensor_info.vendor);
	p += sprintf(p, "  more:%s\n", current_msensor_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
static void dev_get_current_barosensor_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_barosensor_info");
	if(strcmp(current_barosensor_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[BARO-sensor]:\n");
	p += sprintf(p, "  chip:%s\n", current_barosensor_info.chip);
	//p += sprintf(p, "  id:%s\n", current_barosensor_info.id);
	p += sprintf(p, "  vendor:%s\n",current_barosensor_info.vendor);
	p += sprintf(p, "  more:%s\n", current_barosensor_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}

static void dev_get_current_sarsensor_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_sarsensor_info");
	if(strcmp(current_sarsensor_info.chip,"unknow") == 0)
		return;

	p += sprintf(p, "\n[SAR-sensor]:\n");
	p += sprintf(p, "  chip:%s\n", current_sarsensor_info.chip);
	//p += sprintf(p, "  id:%s\n", current_sarsensor_info.id);
	p += sprintf(p, "  vendor:%s\n",current_sarsensor_info.vendor);
	p += sprintf(p, "  more:%s\n", current_sarsensor_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}

//static void dev_get_current_fingerprint_info(char *buf)
//{
//     char *p = buf;
//	HW_PRINT("dev_get_current_fingerprint_info");
//	if(strcmp(current_fingerprint_info.chip,"unknow") == 0)
//		return ;
//
//
//	 p += sprintf(p, "\n[Fingerprint]:\n");
//	 p += sprintf(p, "  chip:%s\n", current_fingerprint_info.chip);
//	 p += sprintf(p, "  id:%s\n", current_fingerprint_info.id);
//	 p += sprintf(p, "  vendor:%s\n",current_fingerprint_info.vendor);
//	 p += sprintf(p, "  more:%s\n", current_fingerprint_info.more);
//
//	 len += (p - buf);
//	 HW_PRINT("%s",buf);
//}

//#if IS_ENABLED(CONFIG_PRIZE_HARDWARE_LPDDR_INFO)
//int get_lpddr_used_index(void)
//{
//	struct device_node * of_chosen = NULL;
//	char *bootargs = NULL;
//	char *ptr;
//	int lpddr_index = 0;
//	char *saved_command_line = vmalloc(600 * sizeof(char));
//
//	of_chosen = of_find_node_by_path("/chosen");
//	if (of_chosen) {
//		bootargs = (char *)of_get_property(of_chosen, "bootargs", NULL);
//		if (!bootargs) {
//			printk("%s: failed to get bootargs\n", __func__);
//		} else {
//			strncpy(saved_command_line, bootargs, 600);
//			printk("%s: bootargs: %s\n", __func__, bootargs);
//		}
//	} else {
//		printk("%s: failed to get /chosen \n", __func__);
//	}
//
//	ptr = strstr(saved_command_line, "lpddr_used_index=");
//	if(ptr == NULL)
//		return -1;
//	ptr += strlen("lpddr_used_index=");
//	lpddr_index = simple_strtol(ptr, NULL, 10);
//	return lpddr_index;
//}
//#endif

extern void ufs_get_hardware_info(struct hardware_info *hwinfo);
static void dev_get_current_flash_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_flash_lpddr_info");
	ufs_get_hardware_info(&current_flash_lpddr_info);
	if (strcmp(current_flash_lpddr_info.chip, "unknow") == 0)
		return;

	p += sprintf(p, "\n[flash]:\n");
	p += sprintf(p, "  chip:%s\n",    current_flash_lpddr_info.chip);
	p += sprintf(p, "  id:%s\n",      current_flash_lpddr_info.id);
	p += sprintf(p, "  vendor:%s\n",  current_flash_lpddr_info.vendor);
	// p += sprintf(p, "  more:%s\n",    current_flash_lpddr_info.more);

	len += (p - buf);
	HW_PRINT("%s", buf);
}

//#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
//static void dev_get_current_battery_info(char *buf)
//{
//     char *p = buf;
//	HW_PRINT("dev_get_current_battery_info");
//	//if(strcmp(current_battery_info.chip,"unknow") == 0)
//	//	return ;
//
//
//	 p += sprintf(p, "\n[battery]:\n");
//	 p += sprintf(p, " batt_vendor:%s\n",current_battery_info.batt_versions);
//	 p += sprintf(p, " Q_MAX_POS_50:%s\n",current_battery_info.Q_MAX_POS_50);
//	 p += sprintf(p, " Q_MAX_POS_25:%s\n",current_battery_info.Q_MAX_POS_25);
//	 p += sprintf(p, " Q_MAX_POS_10:%s\n",current_battery_info.Q_MAX_POS_10);
//	 p += sprintf(p, " Q_MAX_POS_0:%s\n",current_battery_info.Q_MAX_POS_0);
//
//
//	 len += (p - buf);
//	 HW_PRINT("%s",buf);
//}
//#endif
//
//static void dev_get_current_coulo_info(char *buf)
//{
//     char *p = buf;
//	HW_PRINT("dev_get_current_coulo_info");
//	if(strcmp(current_coulo_info.chip,"unknow") == 0)
//		return ;
//
//
//	 p += sprintf(p, "\n[coulo]:\n");
//	 p += sprintf(p, "  chip:%s\n", current_coulo_info.chip);
//	 p += sprintf(p, "  id:%s\n", current_coulo_info.id);
//	 p += sprintf(p, "  vendor:%s\n",current_coulo_info.vendor);
//	 p += sprintf(p, "  more:%s\n", current_coulo_info.more);
//
//	 len += (p - buf);
//	 HW_PRINT("%s",buf);
//}

extern int sec_schip_enabled(void);
static void dev_get_efuse_status(char *buf)
{
    char *p = buf;
    HW_PRINT("dev_get_efuse_status");
	//if(strcmp(current_battery_info.chip,"unknow") == 0)
	//	return ;
	p += sprintf(p, "\n[EFUSE]:\n");
	if(sec_schip_enabled())
		p += sprintf(p, " Status: eFuse blown!\n");
	else
		p += sprintf(p, " Status: eFuse not blown!\n");

	len += (p - buf);
	HW_PRINT("%s",buf);
}
//#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
//static void dev_get_wireless_version(char *buf)
//{
//     char *p = buf;
//	HW_PRINT("dev_get_wireless_version");
//	if(strcmp(current_wireless_info.chip,"unknow") == 0)
//		return ;
//
//
//	 p += sprintf(p, "\n[wireless]:\n");
//	 p += sprintf(p, "  chip:%s\n", current_wireless_info.chip);
//	 p += sprintf(p, "  id:%s\n", current_wireless_info.id);
//	 p += sprintf(p, "  vendor:%s\n",current_wireless_info.vendor);
//	 p += sprintf(p, "  more:%s\n", current_wireless_info.more);
//
//	 len += (p - buf);
//	 HW_PRINT("%s",buf);
//}
//#endif
static void dev_get_current_cp_info(char *buf)
{
	char *p = buf;
	HW_PRINT("dev_get_current_cp_info");
	if(strcmp(current_gsensor_info.chip,"unknow") == 0)
		return ;

	p += sprintf(p, "\n[CP]:\n");
	p += sprintf(p, "  chip:%s\n", current_cp_info.chip);
	p += sprintf(p, "  id:%s\n", current_cp_info.id);
	p += sprintf(p, "  vendor:%s\n",current_cp_info.vendor);
	p += sprintf(p, "  more:%s\n", current_cp_info.more);

	len += (p - buf);
	HW_PRINT("%s",buf);
}
static ssize_t hardware_info_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("hardware_info_store buf:%s,size:%d=======",buf,(int)size);
	return size;
}

//extern char *saved_command_line;
/*int get_lpddr_emmc_used_index(void)
{
     char *ptr;
     int lpddr_index=0;
     ptr=strstr(saved_command_line,"lpddr_used_index=");
     if(ptr==NULL)
	     return -1;
     ptr+=strlen("lpddr_used_index=");
     lpddr_index=simple_strtol(ptr,NULL,10);
     return lpddr_index;
} */

/*static void dev_get_current_flash_lpddr_index_info(char *buf)
{

	 char *p = buf;
	 int flash_lpddr_index =-1;

#if defined(CONFIG_MTK_UFS_SUPPORT)
	const char *life_time;
	const char *pre_eol_info;
	char *path = "/memory";
	struct device_node *dt_node;
	int LifeEstA = 0,LifeEstB = 0,PreEOL = 0;
	int ret;

	dt_node = of_find_node_by_path(path);
	if (dt_node) {
		if (of_property_read_string(dt_node, "life_time", &life_time) == 0){
			ret = sscanf(life_time,"%x %x",&LifeEstA,&LifeEstB);
			if (ret != 2) {
				HW_PRINT("read all health_status fail \n");
			}
			HW_PRINT("of_property_read_string LifeEstA=%x LifeEstB=%x\n",LifeEstA,LifeEstB);
		}

		if (of_property_read_string(dt_node, "pre_eol_info", &pre_eol_info) == 0){
			ret = sscanf(pre_eol_info,"%x",&PreEOL);
			if (ret != 1) {
				HW_PRINT("read all health_status fail \n");
			}
			HW_PRINT("of_property_read_string  PreEOL=%x \n",PreEOL);
		}
	}else{
		HW_PRINT("of_find_node_by_path fail \n");
	}
#endif

	HW_PRINT("dev_get_flash_info");

//	if(strcmp(current_alsps_info.chip,"unknow") == 0)
	 //	return ;
     flash_lpddr_index=get_lpddr_emmc_used_index();
	 p += sprintf(p, "\n[flash]:\n");
	 p += sprintf(p, " %s\n",Cust_emmc_support[flash_lpddr_index]);

#if defined(CONFIG_MTK_UFS_SUPPORT)
	p += sprintf(p, "LifeEstA=0x%02x LifeEstB=0x%02x\n",LifeEstA,LifeEstB);
	 p += sprintf(p, "EOL=0x%02X\n", PreEOL);
#else
	 p += sprintf(p, "LifeEstA=%s LifeEstB=%s\n",current_mmc_info.chip,current_mmc_info.vendor);
	 p += sprintf(p, "EOL=%s\n",current_mmc_info.id);
#endif


	 len += (p - buf);
	 HW_PRINT("%s",buf);
}*/

/*static void dev_get_AudioParam_version_info(char *buf)
{

	   char *p = buf;
	   struct file *fp = NULL;
	   mm_segment_t fs;
	   loff_t pos;
	   char databuf[100]={0};
	   int ret = -1;

	   HW_PRINT("hardware_info_store hello enter\n");
	   fp = filp_open("/vendor/etc/audio_param/AudioParamVersionInfo.txt", O_RDONLY, 0664);
	   if (IS_ERR(fp)){
			HW_ERROR("open AudioParamVersionInfo.txt file error\n");
			return;
	   }
	   fs = get_fs();
	   set_fs(KERNEL_DS);
	   pos =0;
	   ret = vfs_read(fp, databuf, sizeof(databuf), &pos);
	   HW_PRINT("hardware_info_store read ret: %d\n",ret);
	   filp_close(fp,NULL);
	   set_fs(fs);

	   p += sprintf(p, "\n[AudioParamVersionInfo]:\n");
	   p += sprintf(p, "%s\n", databuf);

	   len += (p - buf);
	   HW_PRINT("%s",buf);
}*/

static ssize_t hardware_info_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	len = 0;
	HW_PRINT("hardware_info_show=======");
	dev_get_current_lcm_info(buf + len);

	dev_get_current_camera_info(buf + len);

	dev_get_current_tp_info(buf + len);

	dev_get_current_nfc_info(buf + len);

	dev_get_current_alsps_info(buf + len);

	dev_get_current_gsensor_info(buf + len);

	dev_get_current_gyroscope_info(buf + len);

	dev_get_current_msensor_info(buf + len);
	dev_get_current_barosensor_info(buf + len);

	dev_get_current_sarsensor_info(buf + len);

	//dev_get_current_fingerprint_info(buf + len);
	//dev_get_current_coulo_info(buf + len);
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
	//dev_get_current_battery_info(buf + len);
#endif
	dev_get_efuse_status(buf + len);
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
	//dev_get_wireless_version(buf + len);
#endif

//	dev_get_current_flash_lpddr_index_info(buf + len);

	dev_get_current_flash_info(buf + len);
//	dev_get_AudioParam_version_info(buf + len);
	//dev_get_current_tof_info(buf + len);
	dev_get_current_line_motor_info(buf + len);
	dev_get_current_cp_info(buf + len);
	return len;
}

static DEVICE_ATTR(hw_info_read, 0664, hardware_info_show, hardware_info_store);

#if IS_ENABLED(CONFIG_PRIZE_TYPEC_POSITIVE_NEGATIVE)
extern int prize_otg_detection(void);
static ssize_t otgtypec_detection_show(struct device *dev, struct device_attribute *attr,char *buf)
{
    int otgdetection = -1;
	len = 0;
	HW_PRINT("otgtypec_detection_show=======otgdetection=%d, len=%d\n",otgdetection,len);
	otgdetection = prize_otg_detection();
	return sprintf(buf, "%d\n", otgdetection);
}

static ssize_t otgtypec_detection_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("otgtypec_detection_store buf:%s,size:%d=======",buf,(int)size);
	return size;
}
static DEVICE_ATTR(otg_detection_read, 0664, otgtypec_detection_show, otgtypec_detection_store);

#if IS_ENABLED(CONFIG_PRIZE_TYPEC_CHG_DET)
extern int prize_typec_charge_det(void);
static ssize_t typec_charge_detection_show(struct device *dev, struct device_attribute *attr,char *buf)
{
int typeccharge_det = -1;
	len = 0;
	HW_PRINT("otgtypec_detection_show=======typeccharge_det=%d, len=%d\n",typeccharge_det,len);
    typeccharge_det = prize_typec_charge_det();
	return sprintf(buf, "%d\n", typeccharge_det);
}

static ssize_t typec_charge_detection_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("otgtypec_detection_store buf:%s,size:%d=======",buf,(int)size);
	return size;
}
static DEVICE_ATTR(typec_charge_det, 0664, typec_charge_detection_show, typec_charge_detection_store);
#endif
#endif

/*static int drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
		struct fts_ts_data *ts = container_of(self, struct fts_ts_data, fb_notif);
	    int *evdata = (int *)data;

	if (ts && data) {
		printk("drm_notifier_callback IN");
		if (event == MTK_DISP_EARLY_EVENT_BLANK) {
			if (*evdata == MTK_DISP_BLANK_POWERDOWN) {
				fts_ts_suspend(ts->dev);
			}
		} else if (event == MTK_DISP_EVENT_BLANK) {
			if (*evdata == MTK_DISP_BLANK_UNBLANK) {
				queue_work(ts->ts_workqueue, &ts->resume_work);
			}
		}
		printk("drm_notifier_callback OUT");
	} else {
		printk("ft8722 touch IC can not suspend or resume");
		return -1;
	}

	return 0;

}*/

/*static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data){
	struct fb_event *fb_event = data;
	struct fb_var_screeninfo *scr_info = NULL;

	//if (event != FB_EVENT_FB_REGISTERED){
	//	return 0;
	//}

	if (fb_event->info){
		scr_info = &fb_event->info->var;
		snprintf(&current_lcm_info.more[0], sizeof(current_lcm_info.more),
				"%dx%d", scr_info->xres, scr_info->yres);
	}

	return 0;
}*/

static int __init hardware_info_dev_init(void) {

	struct device *hardware_info_dev;
	//int ret = 0;

	hardware_info_class = class_create(THIS_MODULE, "hw_info");

	if (IS_ERR(hardware_info_class)) {
		HW_ERROR("Failed to create class(hardware_info)!");
		return PTR_ERR(hardware_info_class);
	}

	hardware_info_dev = device_create(hardware_info_class, NULL, 0, NULL, "hw_info_data");
	if (IS_ERR(hardware_info_dev))
		HW_ERROR("Failed to create hardware_info_dev device");

	if (device_create_file(hardware_info_dev, &dev_attr_hw_info_read) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_hw_info_read.attr.name);

#if IS_ENABLED(CONFIG_PRIZE_TYPEC_POSITIVE_NEGATIVE)
	if (device_create_file(hardware_info_dev, &dev_attr_otg_detection_read) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_otg_detection_read.attr.name);

#if IS_ENABLED(CONFIG_PRIZE_TYPEC_CHG_DET)
	if (device_create_file(hardware_info_dev, &dev_attr_typec_charge_det) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_typec_charge_det.attr.name);
#endif
#endif

	//fb_nb.notifier_call = drm_notifier_callback;//fb_notifier_callback;
	//ret = mtk_disp_notifier_register("hardware_info", &fb_nb);
   //  if (ret)
    //        HW_ERROR("register fb client fail\n");
	//if (fb_register_client(&fb_nb)){
	//	HW_ERROR("register fb client fail\n");
	//}

	HW_PRINT("hardware_info initialized ok ");
	return 0;
}

static void __exit hardware_info_dev_exit(void)
{
	class_destroy(hardware_info_class);
}

subsys_initcall_sync(hardware_info_dev_init);
module_exit(hardware_info_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lixuefeng <lixuefeng@boruizhiheng.com>");
MODULE_DESCRIPTION("show hardware info Driver");
//MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

