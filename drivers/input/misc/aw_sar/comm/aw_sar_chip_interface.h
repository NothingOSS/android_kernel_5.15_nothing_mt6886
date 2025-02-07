#ifndef _SAR_SUPPORT_CHIP_H_
#define _SAR_SUPPORT_CHIP_H_
#include "aw_sar_type.h"

enum aw_sar_driver_list_t {
	AW_SAR_AW9610X,
//	AW_SAR_AW9620X,
//	AW_SAR_AW963XX,

	AW_SAR_DRIVER_MAX,
};

int32_t aw9610x_check_chipid(void *data);
int32_t aw9610x_init(struct aw_sar *p_sar);
void aw9610x_deinit(struct aw_sar *p_sar);

//int32_t aw9620x_check_chipid(void *data);
//int32_t aw9620x_init(struct aw_sar *p_sar);
//void aw9620x_deinit(struct aw_sar *p_sar);

//int32_t aw963xx_check_chipid(void *data);
//int32_t aw963xx_init(struct aw_sar *p_sar);
//void aw963xx_deinit(struct aw_sar *p_sar);


static const struct aw_sar_driver_type g_aw_sar_driver_list[] = {
	{ AW_SAR_AW9610X, aw9610x_check_chipid, aw9610x_init, aw9610x_deinit },
//	{ AW_SAR_AW9620X, aw9620x_check_chipid, aw9620x_init, aw9620x_deinit },
//	{ AW_SAR_AW963XX, aw963xx_check_chipid, aw963xx_init, aw963xx_deinit },
};


#endif