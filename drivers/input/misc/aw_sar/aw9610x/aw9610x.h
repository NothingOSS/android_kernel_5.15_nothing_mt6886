#ifndef _AW9610X_H_
#define _AW9610X_H_
#include "../comm/aw_sar_type.h"

//#define AW9610X_TVS_ABNORMAL_CAIL
#define AW9610X_AOT_SCAN_OVER_CNT (32)

#define AW9610X_CHIP_ID				(0xa961)
#define AW9610x_DATA_PROCESS_FACTOR		(1024)
#define AW_CHIP_AW9610XA			(0x03000b00)
#define AW9610X_CPU_WORK_MASK			(1)

#define AW9610X_SAR_VCC_MIN_UV			(1700000)
#define AW9610X_SAR_VCC_MAX_UV			(3600000)

#define AW_REG_IRQEN_CLOSE					(0)
#define AW_REG_IRQSRC_AOT_OVER_BIT			(3)
#define REG_IRQSRC_SCAN_OVER_BIT			(4)
#define REG_REG_WST_SLEEP_MODE				(0x3)
#define AW9610X_AOT_OVER_DELAY_MAX_MS		(6000)
#define AW9610X_AOT_MASK					(0x3f)
#define AW9610X_AOT_BIT						(8)
#define AW9610X_REG_OFFSET_STEP				(4)

enum aw9610x_sar_vers {
	AW9610X = 2,
	AW9610XA = 6,
	AW9610XB = 0xa,
};

enum aw9610x_operation_mode {
	AW9610X_ACTIVE_MODE = 1,
	AW9610X_SLEEP_MODE,
	AW9610X_DEEPSLEEP_MODE,
	AW9610XB_DEEPSLEEP_MODE,
};

/**********************************************
*spereg addr offset
**********************************************/
enum aw9610x_spereg_addr_offset {
	AW_CL1SPE_CALI_OS = 20,
	AW_CL1SPE_DEAL_OS = 60,
	AW_CL2SPE_CALI_OS = 4,
	AW_CL2SPE_DEAL_OS = 4,
};


/**********************************************
 *the flag of i2c read/write
 **********************************************/
enum aw9610x_function_flag {
	AW9610X_FUNC_OFF,
	AW9610X_FUNC_ON,
};

/**********************************************
* mutiple sar define
**********************************************/
enum aw9610x_multiple_sar {
	AW_SAR0,
	AW_SAR1,
	AW_SAR_MAX,
};

#define AW9610X_CHANNEL_MAX	(6)
#define UEVENT_REPORT


enum aw9610x_irq_trigger_position {
	AW9610X_FAR,
	AW9610X_TRIGGER_TH0,
	AW9610X_TRIGGER_TH1 = 0x03,
	AW9610X_TRIGGER_TH2 = 0x07,
	AW9610X_TRIGGER_TH3 = 0x0f,
};

struct aw_i2c_package {
	uint8_t addr_bytes;
	uint8_t data_bytes;
	uint8_t reg_num;
	uint8_t init_addr[4];
	uint8_t *p_reg_data;
};

struct aw9610x {
	uint8_t vers;
	uint8_t channel;
	uint32_t irq_status;
	uint8_t chip_name[9];
	uint8_t chip_type[9];
	bool satu_release;

	struct aw_i2c_package aw_i2c_package;

	uint8_t satu_flag[6];
	uint32_t satu_data[6];
};

/********************************************
* Register List
********************************************/
#define AFE_BASE_ADDR					(0x0000)
#define DSP_BASE_ADDR					(0x0000)
#define STAT_BASE_ADDR					(0x0000)
#define SFR_BASE_ADDR					(0x0000)
#define DATA_BASE_ADDR					(0x0000)
/* registers list */
//AW96105A_UI_REGISTER_AFE MAP
#define REG_SCANCTRL0					((0x0000) + AFE_BASE_ADDR)
#define REG_SCANCTRL1					((0x0004) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH0					((0x0010) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH0					((0x0014) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH0					((0x001C) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH0					((0x0020) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH1					((0x0024) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH1					((0x0028) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH1					((0x0030) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH1					((0x0034) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH2					((0x0038) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH2					((0x003C) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH2					((0x0044) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH2					((0x0048) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH3					((0x004C) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH3					((0x0050) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH3					((0x0058) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH3					((0x005C) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH4					((0x0060) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH4					((0x0064) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH4					((0x006C) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH4					((0x0070) + AFE_BASE_ADDR)
#define REG_AFECFG0_CH5					((0x0074) + AFE_BASE_ADDR)
#define REG_AFECFG1_CH5					((0x0078) + AFE_BASE_ADDR)
#define REG_AFECFG3_CH5					((0x0080) + AFE_BASE_ADDR)
#define REG_AFECFG4_CH5					((0x0084) + AFE_BASE_ADDR)

/* registers list */
//AW96105A_UI_REGISTER_DSP MAP
#define REG_DSPCFG0_CH0					((0x00A0) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH0					((0x00A4) + DSP_BASE_ADDR)
#define REG_BLFILT_CH0					((0x00A8) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH0				((0x00B0) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH0				((0x00B4) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH0					((0x00B8) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH0					((0x00BC) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH0					((0x00C0) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH0					((0x00C4) + DSP_BASE_ADDR)
#define REG_STDDET_CH0					((0x00C8) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH0				((0x00CC) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH0				((0x00D0) + DSP_BASE_ADDR)
#define REG_DATAOFFSET_CH0				((0x00D4) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH0					((0x00D8) + DSP_BASE_ADDR)
#define REG_DSPCFG0_CH1					((0x00DC) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH1					((0x00E0) + DSP_BASE_ADDR)
#define REG_BLFILT_CH1					((0x00E4) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH1				((0x00EC) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH1				((0x00F0) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH1					((0x00F4) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH1					((0x00F8) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH1					((0x00FC) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH1					((0x0100) + DSP_BASE_ADDR)
#define REG_STDDET_CH1					((0x0104) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH1				((0x0108) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH1				((0x010C) + DSP_BASE_ADDR)
#define REG_RAWOFFSET_CH1				((0x0110) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH1					((0x0114) + DSP_BASE_ADDR)
#define REG_DSPCFG0_CH2					((0x0118) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH2					((0x011C) + DSP_BASE_ADDR)
#define REG_BLFILT_CH2					((0x0120) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH2				((0x0128) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH2				((0x012C) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH2					((0x0130) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH2					((0x0134) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH2					((0x0138) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH2					((0x013C) + DSP_BASE_ADDR)
#define REG_STDDET_CH2					((0x0140) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH2				((0x0144) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH2				((0x0148) + DSP_BASE_ADDR)
#define REG_DATAOFFSET_CH2				((0x014C) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH2					((0x0150) + DSP_BASE_ADDR)
#define REG_DSPCFG0_CH3					((0x0154) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH3					((0x0158) + DSP_BASE_ADDR)
#define REG_BLFILT_CH3					((0x015C) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH3				((0x0164) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH3				((0x0168) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH3					((0x016C) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH3					((0x0170) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH3					((0x0174) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH3					((0x0178) + DSP_BASE_ADDR)
#define REG_STDDET_CH3					((0x017C) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH3				((0x0180) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH3				((0x0184) + DSP_BASE_ADDR)
#define REG_DATAOFFSET_CH3				((0x0188) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH3					((0x018C) + DSP_BASE_ADDR)
#define REG_DSPCFG0_CH4					((0x0190) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH4					((0x0194) + DSP_BASE_ADDR)
#define REG_BLFILT_CH4					((0x0198) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH4				((0x01A0) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH4				((0x01A4) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH4					((0x01A8) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH4					((0x01AC) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH4					((0x01B0) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH4					((0x01B4) + DSP_BASE_ADDR)
#define REG_STDDET_CH4					((0x01B8) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH4				((0x01BC) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH4				((0x01C0) + DSP_BASE_ADDR)
#define REG_DATAOFFSET_CH4				((0x01C4) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH4					((0x01C8) + DSP_BASE_ADDR)
#define REG_DSPCFG0_CH5					((0x01CC) + DSP_BASE_ADDR)
#define REG_DSPCFG1_CH5					((0x01D0) + DSP_BASE_ADDR)
#define REG_BLFILT_CH5					((0x01D4) + DSP_BASE_ADDR)
#define REG_PROXCTRL_CH5				((0x01DC) + DSP_BASE_ADDR)
#define REG_BLRSTRNG_CH5				((0x01E0) + DSP_BASE_ADDR)
#define REG_PROXTH0_CH5					((0x01E4) + DSP_BASE_ADDR)
#define REG_PROXTH1_CH5					((0x01E8) + DSP_BASE_ADDR)
#define REG_PROXTH2_CH5					((0x01EC) + DSP_BASE_ADDR)
#define REG_PROXTH3_CH5					((0x01F0) + DSP_BASE_ADDR)
#define REG_STDDET_CH5					((0x01F4) + DSP_BASE_ADDR)
#define REG_INITPROX0_CH5				((0x01F8) + DSP_BASE_ADDR)
#define REG_INITPROX1_CH5				((0x01FC) + DSP_BASE_ADDR)
#define REG_DATAOFFSET_CH5				((0x0200) + DSP_BASE_ADDR)
#define REG_AOTTAR_CH5					((0x0204) + DSP_BASE_ADDR)
#define REG_ATCCR0					((0x0208) + DSP_BASE_ADDR)
#define REG_ATCCR1					((0x020C) + DSP_BASE_ADDR)

/* registers list */
//AW96105A_UI_REGISTER_STAT MAP
#define REG_FWVER					((0x0088) + STAT_BASE_ADDR)
#define REG_WST						((0x008C) + STAT_BASE_ADDR)
#define REG_STAT0					((0x0090) + STAT_BASE_ADDR)
#define REG_STAT1					((0x0094) + STAT_BASE_ADDR)
#define REG_STAT2					((0x0098) + STAT_BASE_ADDR)
#define REG_CHINTEN					((0x009C) + STAT_BASE_ADDR)

/* registers list */
//AW96105A_UI_REGISTER_SFR MAP
#define REG_CMD						((0xF008) + SFR_BASE_ADDR)
#define REG_IRQSRC					((0xF080) + SFR_BASE_ADDR)
#define REG_IRQEN					((0xF084) + SFR_BASE_ADDR)
#define REG_I2CADDR					((0xF0F0) + SFR_BASE_ADDR)
#define REG_OSCEN					((0xFF00) + SFR_BASE_ADDR)
#define REG_RESET					((0xFF0C) + SFR_BASE_ADDR)
#define REG_CHIPID					((0xFF10) + SFR_BASE_ADDR)

/* registers list */
//AW96105A_UI_REGISTER_DATA MAP
#define REG_COMP_CH0					((0x0210) + DATA_BASE_ADDR)
#define REG_COMP_CH1					((0x0214) + DATA_BASE_ADDR)
#define REG_COMP_CH2					((0x0218) + DATA_BASE_ADDR)
#define REG_COMP_CH3					((0x021C) + DATA_BASE_ADDR)
#define REG_COMP_CH4					((0x0220) + DATA_BASE_ADDR)
#define REG_COMP_CH5					((0x0224) + DATA_BASE_ADDR)
#define REG_BASELINE_CH0				((0x0228) + DATA_BASE_ADDR)
#define REG_BASELINE_CH1				((0x022C) + DATA_BASE_ADDR)
#define REG_BASELINE_CH2				((0x0230) + DATA_BASE_ADDR)
#define REG_BASELINE_CH3				((0x0234) + DATA_BASE_ADDR)
#define REG_BASELINE_CH4				((0x0238) + DATA_BASE_ADDR)
#define REG_BASELINE_CH5				((0x023C) + DATA_BASE_ADDR)
#define REG_DIFF_CH0					((0x0240) + DATA_BASE_ADDR)
#define REG_DIFF_CH1					((0x0244) + DATA_BASE_ADDR)
#define REG_DIFF_CH2					((0x0248) + DATA_BASE_ADDR)
#define REG_DIFF_CH3					((0x024C) + DATA_BASE_ADDR)
#define REG_DIFF_CH4					((0x0250) + DATA_BASE_ADDR)
#define REG_DIFF_CH5					((0x0254) + DATA_BASE_ADDR)
#define REG_FWVER2					((0x0410) + DATA_BASE_ADDR)

struct aw_reg_data {
	unsigned char rw;
	unsigned short reg;
};
/********************************************
 * Register Access
 *******************************************/
#define REG_NONE_ACCESS					(0)
#define REG_RD_ACCESS					(1 << 0)
#define REG_WR_ACCESS					(1 << 1)

static const struct aw_reg_data g_aw9610x_reg_access[] = {
	//AFE MAP
	{ .reg = REG_SCANCTRL0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_SCANCTRL1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG0_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG1_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG3_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AFECFG4_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//DSP MAP
	{ .reg = REG_DSPCFG0_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DATAOFFSET_CH0,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH0,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG0_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_RAWOFFSET_CH1,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH1,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG0_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DATAOFFSET_CH2,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH2,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG0_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DATAOFFSET_CH3,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH3,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG0_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DATAOFFSET_CH4,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH4,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG0_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DSPCFG1_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLFILT_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXCTRL_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_BLRSTRNG_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH0_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH1_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH2_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_PROXTH3_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_STDDET_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX0_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_INITPROX1_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_DATAOFFSET_CH5,		.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_AOTTAR_CH5,			.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_ATCCR0,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_ATCCR1,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//STAT MAP
	{ .reg = REG_FWVER,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_WST,					.rw = REG_RD_ACCESS, },
	{ .reg = REG_STAT0,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_STAT1,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_STAT2,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_CHINTEN,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },

	//SFR MAP
	{ .reg = REG_CMD,					.rw = REG_NONE_ACCESS, },
	{ .reg = REG_IRQSRC,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_IRQEN,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_I2CADDR,				.rw = REG_RD_ACCESS, },
	{ .reg = REG_OSCEN,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_RESET,				.rw = REG_RD_ACCESS | REG_WR_ACCESS, },
	{ .reg = REG_CHIPID,				.rw = REG_RD_ACCESS, },

	//DATA MAP
	{ .reg = REG_COMP_CH0,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_COMP_CH1,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_COMP_CH2,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_COMP_CH3,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_COMP_CH4,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_COMP_CH5,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH0,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH1,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH2,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH3,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH4,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_BASELINE_CH5,		.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH0,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH1,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH2,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH3,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH4,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_DIFF_CH5,			.rw = REG_RD_ACCESS, },
	{ .reg = REG_FWVER2,				.rw = REG_RD_ACCESS, },
};


/******************************************************
* Register Detail
******************************************************/
const static uint32_t aw9610x_reg_default[] = {
	0x0000, 0x00003f3f,
	0x0004, 0x00000064,
	0x0008, 0x0017c11e,
	0x000c, 0x05000000,
	0x0010, 0x00093ffd,
	0x0014, 0x19240009,
	0x0018, 0xd81c0207,
	0x001c, 0xff000000,
	0x0020, 0x00241900,
	0x0024, 0x00093ff7,
	0x0028, 0x58020009,
	0x002c, 0xd81c0207,
	0x0030, 0xff000000,
	0x0034, 0x00025800,
	0x0038, 0x00093fdf,
	0x003c, 0x7d3b0009,
	0x0040, 0xd81c0207,
	0x0044, 0xff000000,
	0x0048, 0x003b7d00,
	0x004c, 0x00093f7f,
	0x0050, 0xe9310009,
	0x0054, 0xd81c0207,
	0x0058, 0xff000000,
	0x005c, 0x0031e900,
	0x0060, 0x00093dff,
	0x0064, 0x1a0c0009,
	0x0068, 0xd81c0207,
	0x006c, 0xff000000,
	0x0070, 0x000c1a00,
	0x0074, 0x80093fff,
	0x0078, 0x043d0009,
	0x007c, 0xd81c0207,
	0x0080, 0xff000000,
	0x0084, 0x003d0400,
	0x00a0, 0xe6400000,
	0x00a4, 0x00000000,
	0x00a8, 0x010408d2,
	0x00ac, 0x00000000,
	0x00b0, 0x00000000,
	0x00b8, 0x00005fff,
	0x00bc, 0x00000000,
	0x00c0, 0x00000000,
	0x00c4, 0x00000000,
	0x00c8, 0x00000000,
	0x00cc, 0x00000000,
	0x00d0, 0x00000000,
	0x00d4, 0x00000000,
	0x00d8, 0x00000000,
	0x00dc, 0xe6447800,
	0x00e0, 0x78000000,
	0x00e4, 0x010408d2,
	0x00e8, 0x00000000,
	0x00ec, 0x00000000,
	0x00f4, 0x00005fff,
	0x00f8, 0x00000000,
	0x00fc, 0x00000000,
	0x0100, 0x00000000,
	0x0104, 0x00000000,
	0x0108, 0x00000000,
	0x010c, 0x02000000,
	0x0110, 0x00000000,
	0x0114, 0x00000000,
	0x0118, 0xe6447800,
	0x011c, 0x78000000,
	0x0120, 0x010408d2,
	0x0124, 0x00000000,
	0x0128, 0x00000000,
	0x0130, 0x00005fff,
	0x0134, 0x00000000,
	0x0138, 0x00000000,
	0x013c, 0x00000000,
	0x0140, 0x00000000,
	0x0144, 0x00000000,
	0x0148, 0x02000000,
	0x014c, 0x00000000,
	0x0150, 0x00000000,
	0x0154, 0xe6447800,
	0x0158, 0x78000000,
	0x015c, 0x010408d2,
	0x0160, 0x00000000,
	0x0164, 0x00000000,
	0x016c, 0x00005fff,
	0x0170, 0x00000000,
	0x0174, 0x00000000,
	0x0178, 0x00000000,
	0x017c, 0x00000000,
	0x0180, 0x00000000,
	0x0184, 0x02000000,
	0x0188, 0x00000000,
	0x018c, 0x00000000,
	0x0190, 0xe6447800,
	0x0194, 0x78000000,
	0x0198, 0x010408d2,
	0x019c, 0x00000000,
	0x01a0, 0x00000000,
	0x01a8, 0x00005fff,
	0x01ac, 0x00000000,
	0x01b0, 0x00000000,
	0x01b4, 0x00000000,
	0x01b8, 0x00000000,
	0x01bc, 0x00000000,
	0x01c0, 0x02000000,
	0x01c4, 0x00000000,
	0x01c8, 0x00000000,
	0x01cc, 0xe6407800,
	0x01d0, 0x78000000,
	0x01d4, 0x010408d2,
	0x01d8, 0x00000000,
	0x01dc, 0x00000000,
	0x01e4, 0x00005fff,
	0x01e8, 0x00000000,
	0x01ec, 0x00000000,
	0x01f0, 0x00000000,
	0x01f4, 0x00000000,
	0x01f8, 0x00000000,
	0x01fc, 0x02000000,
	0x0200, 0x00000000,
	0x0204, 0x00000000,
	0x0208, 0x00000008,
	0x020c, 0x0000000d,
	0x41fc, 0x00000000,
	0x4400, 0x00000000,
	0x4410, 0x00000000,
	0x4420, 0x00000000,
	0x4430, 0x00000000,
	0x4440, 0x00000000,
	0x4450, 0x00000000,
	0x4460, 0x00000000,
	0x4470, 0x00000000,
	0xf080, 0x00003018,
	0xf084, 0x00000fff,
	0xf800, 0x00000000,
	0xf804, 0x00002e00,
	0xf8d0, 0x00000001,
	0xf8d4, 0x00000000,
	0xff00, 0x00000301,
	0xff0c, 0x01000000,
	0xffe0, 0x00000000,
	0xfff4, 0x00004011,
	0x0090, 0x00000000,
	0x0094, 0x00000000,
	0x0098, 0x00000000,
	0x009c, 0x3f3f3f3f,
};

#endif
