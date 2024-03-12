

#ifndef _HARDWARE_INFO_H_
#define _HARDWARE_INFO_H_


struct hardware_info{
	unsigned char chip[32];
	unsigned char vendor[32];
	unsigned char id[32];
	unsigned char more[64];
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
	unsigned char batt_versions[32];
	unsigned char Q_MAX_POS_50[32];
	unsigned char Q_MAX_POS_25[32];
	unsigned char Q_MAX_POS_10[32];
	unsigned char Q_MAX_POS_0[32];
#endif
};

extern struct hardware_info current_line_motor_info;
extern struct hardware_info current_tp_info;
extern struct hardware_info current_alsps_info;
extern struct hardware_info current_gsensor_info;
extern struct hardware_info current_msensor_info;
extern struct hardware_info current_gyroscope_info;
extern struct hardware_info current_barosensor_info;
extern struct hardware_info current_sarsensor_info;
extern struct hardware_info current_cp_info;
extern struct hardware_info current_camera_info[5];
extern struct hardware_info current_imgsensor_info[5];
/*extern struct hardware_info current_lcm_info;
extern struct hardware_info current_fingerprint_info;
extern struct hardware_info current_coulo_info;
extern struct hardware_info current_flash_lpddr_info;*/

#endif /* _HARDWARE_INFO_H_ */
