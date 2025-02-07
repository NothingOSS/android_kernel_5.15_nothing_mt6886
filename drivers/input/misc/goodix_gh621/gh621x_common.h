#pragma once
/**
 * @copyright (c) 2003 - 2021, Goodix Co., Ltd. All rights reserved.
 *
 * @file
 *
 * @brief   gh621x Chip reg define & other common define
 *
 * @version ref gh621x.h
 *
 * @author  Goodix Iot Team
 *
 */

#define GH621X_LOAD_REG_PATCH

/// null val
#define   GH621X_PTR_NULL                                ((void *) 0)

/// get high byte from word
#define   GH621X_GET_HIGH_BYTE_FROM_WORD(sValue)        ((u8)(((sValue) >> 8) & 0xFFU))

/// get low byte from word
#define   GH621X_GET_LOW_BYTE_FROM_WORD(sValue)         ((u8)((sValue) & 0xFFU))

/// get high word from dword
#define   GH621X_GET_HIGH_WORD_FROM_DWORD(unValue)       ((u16)(((unValue) >> 16) & 0x0000FFFFU))

/// get low word from dword
#define   GH621X_GET_LOW_WORD_FROM_DWORD(unValue)        ((u16)((unValue) & 0x0000FFFFU))

/// makeup word from bytes
#define   GH621X_MAKEUP_WORD(uchHighByte, uchLowByte)    ((u16)(((((u16)(uchHighByte)) << 8)& 0xFF00) |\
                                                                            (((u16)(uchLowByte))& 0x00FF)))

#define   GH621X_MAKEUP_DWORD2(usHighWord, usLowWord)    (((((u32)(usHighWord)) << 16) & 0xFFFF0000U) |\
                                                                            (((u32)(usLowWord)) & 0x0000FFFFU))

#define I2C_WRITE_READ_REG_ADDR_LEN                      (2)     /**< i2c write/read reg buffer len */
#define I2C_WRITE_READ_REG_DATA_LEN                      (2)     /**< i2c write/read reg buffer len */

#define   GH621X_I2C_MEM_EN_REG_ADDR                     (0x0072)    /**< mem enable */

#define   GH621X_CHIP_ID_0_REG_ADDR                      (0x0200)    /**< chip id 0 */
#define   GH621X_CHIP_ID_1_REG_ADDR                      (0x0202)    /**< chip id 1 */
#define   GH621X_CHIP_ID_2_REG_ADDR                      (0x0204)    /**< chip id 2 */
#define   GH621X_SYS_CTRL_REG_ADDR                       (0x0208)    /**< system ctrl reg */

#define   GH621X_PRIVATE_REG_0_REG_ADDR                  (0x6000)    /**< private reg 0 */
#define   GH621X_PRIVATE_REG_1_REG_ADDR                  (0x6002)    /**< private reg 1 */
#define   GH621X_PRIVATE_REG_2_REG_ADDR                  (0x6004)    /**< private reg 2 */
#define   GH621X_PRIVATE_REG_3_REG_ADDR                  (0x6006)    /**< private reg 3 */

#define   GH621X_CPU_ID_LO_REG_ADDR                      (0x0004)    /**< cpu id lo reg */
#define   GH621X_IRQ_HOST_STATUS_0_REG_ADDR              (0x0288)    /**< irq status for host reg */

#define   GH621X_LOWPOWER_CTRL_REG_ADDR                  (0x0256)    /**< lowpower ctrl reg */

#define   GH621X_INSTRUCTIONS_CHIP_INIED_REG_ADDR        (0x02DC)    /**< instructions chip inited reg */

#define   GH621X_SAR_DATA_READ_DONE_REG_ADDR             (0x0BF2)    /**< sar data read done flag reg */
#define   GH621X_SAR_INIT_FLAG_REG_ADDR                  (0x02E6)    /**< sar init flag reg */

#define   GH621X_HOST_IRQ_CTRL_0_REG_ADDR                (0x0270)    /**< host irq ctrl 0 reg */
#define   GH621X_HOST_IRQ_CTRL_1_REG_ADDR                (0x0272)    /**< host irq ctrl 1 reg */
#define   GH621X_HOST_IRQ_CTRL_2_REG_ADDR                (0x0274)    /**< host irq ctrl 2 reg */
#define   GH621X_HOST_IRQ_CTRL_3_REG_ADDR                (0x0276)    /**< host irq ctrl 3 reg */

#define   GH621X_MODE_CTRL_ENABLE_REG_ADDR               (0x0300)    /**< mode ctrl enable reg */
#define   GH621X_MODE_CTRL_STATUS_REG_ADDR               (0x0328)    /**< mode ctrl status reg */

#define   GH621X_CRC16_CTRL_REG_ADDR                     (0x0050)    /**< crc16 ctrl reg */
#define   GH621X_CRC16_STATUS_REG_ADDR                   (0x0054)    /**< crc16 status reg */
#define   GH621X_CRC_PATTERN_REG_ADDR                    (0x0068)    /**< crc16 parttern reg */
#define   GH621X_IRAM_CRC_SADDR_REG_ADDR                 (0x006C)    /**< crc start addr reg */
#define   GH621X_IRAM_CRC_EADDR_REG_ADDR                 (0x006E)    /**< crc end addr reg */

#define   GH621X_SAMPLING_DONE_DATA_0_REG_ADDR           (0x073C)    /**< sampling done data 0 reg */
#define   GH621X_SAMPLING_DONE_DATA_1_REG_ADDR           (0x0396)    /**< sampling done data 1 reg */

#define   GH621X_FW_VERSION_START_REG_ADDR               (0x0954)    /**< FW version start reg */

#define   GH621X_SAR_RAWDATA_START_REG_ADDR              (0x0B30)    /**< sar rawdata start reg */

#define   GH621X_CCSEL_DATA_START_REG_ADDR               (0x0BDE)    /**< tk ied rawdata start reg */

#define   GH621X_SAR_AUTOCC_TRG_CH_REG_ADDR              (0x0BF0)    /**< sar autocc trg &ch_en reg */

// top ctrl block, start addr: 0x0000

/* utils reg macro */
#define   GH621X_REG_ADDR_SIZE                           (0x0002)    /**< reg addr size */
#define   GH621X_REG_VAL_SIZE                            (0x0002)    /**< reg val size */
#define   GH621X_REG_ADDR_EVEN_FIXED                     (0xFFFE)    /**< reg addr even fixed */
#define   GH621X_REG_ADDR_MAX                            (0xFFFF)    /**< reg addr max */

#define   GH621X_IRQ_STATUS_REG_MAX_COUNT                (4)         /**< irq status reg max count */
#define   GH621X_FW_VERSION_MAX_COUNT                    (4)

#define   GH621X_INSTRUCTIONS_CHIP_INIED_VAL             (0xA5A5)    /**< instructions chip inited val */

#define   GH621X_SAR_RAWDATA_REG_MAX_COUNT               (81)        /**< sar rawdata reg max count */
#define   GH621X_SAR_RAWDATA_EXCHANGE_8BITS_LEN          (161)       /**< sar rawdata 8bits len(116 + (94*2-116)/2) */
#define   GH621X_SAR_RAWDATA_START_16BITS_DATA_COUNT     (2)          /**< sar rawdata start 16bits data count(1*2) */
#define   GH621X_SAR_RAWDATA_32BITS_DATA_COUNT           (148)        /**< sar rawdata 32bits data count(29*4) */
#define   GH621X_SAR_RAWDATA_16BITS_DATA_COUNT           (10)        /**< sar rawdata 16bits data count(5*2) */
#define   GH621X_SAR_RAWDATA_DATA_COUNT                  (65)        /**< sar rawdata count */

#define   GH621X_SAR_RAWDATA_PROX_TEMPDATA_COUNT          (1)         /**< sar rawdata prox temp-rawdata count */
#define   GH621X_SAR_RAWDATA_PROX_TEMPDATA_INDEX          (0)         /**< sar rawdata prox temp-rawdata index */

#define   GH621X_SAR_RAWDATA_PROX_PRERAW_COUNT          (9)         /**< sar rawdata prox pre-rawdata count */
#define   GH621X_SAR_RAWDATA_PROX_PRERAW_INDEX          (1)         /**< sar rawdata prox pre-rawdata index */

#define   GH621X_SAR_RAWDATA_PROX_RAWDATA_COUNT          (9)         /**< sar rawdata prox rawdata count */
#define   GH621X_SAR_RAWDATA_PROX_RAWDATA_INDEX          (10)         /**< sar rawdata prox rawdata index */
#define   GH621X_SAR_RAWDATA_PROX_REF_RAWDATA_INDEX      (15)         /**< sar rawdata prox ref rawdata index */

#define   GH621X_SAR_RAWDATA_PROX_BASE_COUNT             (5)         /**< sar rawdata prox base count */
#define   GH621X_SAR_RAWDATA_PROX_BASE_INDEX             (19)         /**< sar rawdata prox base count */

#define   GH621X_SAR_RAWDATA_PROX_DIFF_COUNT             (5)         /**< sar rawdata prox diff count */
#define   GH621X_SAR_RAWDATA_PROX_DIFF_INDEX             (24)         /**< sar rawdata prox diff count */

#define   GH621X_SAR_RAWDATA_PROX_STAT_COUNT             (5)         /**< sar rawdata prox stat count */
#define   GH621X_SAR_RAWDATA_PROX_STAT_INDEX             (29)         /**< sar rawdata prox stat count */

#define   GH621X_SAR_RAWDATA_PROX_BODY_STAT_COUNT        (5)         /**< sar rawdata prox body stat count */
#define   GH621X_SAR_RAWDATA_PROX_BODY_STAT_INDEX        (34)         /**< sar rawdata prox body stat count */

#define   GH621X_SAR_RAWDATA_PROX_TABLE_STAT_COUNT       (5)         /**< sar rawdata prox table stat count */
#define   GH621X_SAR_RAWDATA_PROX_TABLE_STAT_INDEX       (39)         /**< sar rawdata prox table stat count */

#define   GH621X_SAR_RAWDATA_PROX_STEADY_STAT_COUNT      (9)         /**< sar rawdata prox steady stat count */
#define   GH621X_SAR_RAWDATA_PROX_STEADY_STAT_INDEX      (44)         /**< sar rawdata prox steady stat count */

#define   GH621X_SAR_RAWDATA_PROX_COMP_STAT_COUNT        (9)         /**< sar rawdata prox comp stat count */
#define   GH621X_SAR_RAWDATA_PROX_COMP_STAT_INDEX        (53)         /**< sar rawdata prox comp stat count */

#define   GH621X_SAR_RAWDATA_PROX_OR_STAT_COUNT          (1)         /**< sar rawdata prox or stat count */
#define   GH621X_SAR_RAWDATA_PROX_OR_STAT_INDEX          (62)         /**< sar rawdata prox or stat count */

#define   GH621X_SAR_RAWDATA_PROX_AND_STAT_COUNT         (1)         /**< sar rawdata prox and stat count */
#define   GH621X_SAR_RAWDATA_PROX_AND_STAT_INDEX         (63)         /**< sar rawdata prox and stat count */

#define   GH621X_SAR_RAWDATA_PROX_TEMP_CHANGE_COUNT      (1)         /**< sar rawdata prox temp change coun */
#define   GH621X_SAR_RAWDATA_PROX_TEMP_CHANGE_INDEX      (64)         /**< sar rawdata prox temp change count */

#define   GH621X_TK_IED_CAP_RAWDATA_NUM			         (9)         /**< ied tk cap rawdata num */
#define   GH621X_TK_IED_REFRESH_DATA_NUM			     (1)         /**< ied tk refresh data num */
#define   GH621X_TK_IED_ACC_DATA__NUM			         (3)         /**< ied tk acc data num */
#define   GH621X_TK_IED_TEMP_DATA_NUM			         (2)         /**< ied tk temp data num */
#define   GH621X_TK_IED_RES_DATA_NUM			         (2)         /**< ied tk res data num */
#define   GH621X_TK_IED_STATUS_DATA_NUM            		 (1)         /**< ied tk rawdata reg max count 15 * 2 */


/* Register bit definition */
#define   GH621X_CPU_ID_LO_REG_VAL                       (0x080A)    /**< cpu id lo value */
#define   GH621X_SAR_RAWDATA_CHECKSUM_REG_ADDR           (0x0BD2)    /**< sar rawdata checksum reg */

#define   GH621X_PRIVATE_CMD_SLEEP_HOST                  (0x0001)    /**< private reg 0,cmd_sleep_host */
#define   GH621X_PRIVATE_CMD_DSLP_HOST                   (0x0002)    /**< private reg 0,cmd_dslp_host */
#define   GH621X_PRIVATE_CMD_WAKEUP_HOST                 (0x0004)    /**< private reg 0,cmd_wakeup_host */
#define   GH621X_PRIVATE_CMD_NORMAL_HOST                 (0x0008)    /**< private reg 0,cmd_normal_host */
#define   GH621X_PRIVATE_CMD_AUTOCC_HOST                 (0x0010)    /**< private reg 0,cmd_autocc_host */
#define   GH621X_PRIVATE_CMD_RESET_HOST                  (0x0020)    /**< private reg 0,cmd_reset_host */
#define   GH621X_PRIVATE_WDT_TIMER_CLR                   (0x0040)    /**< private reg 0,wdt_timer_clr */

#define   GH621X_PRIVATE_ENABLE_NO_SLEEP_VAL             (0x0003)    /**< private reg 1,enable no sleep */
#define   GH621X_PRIVATE_SP_ENABLE_NO_SLEEP_VAL          (0x0002)    /**< private reg 1,enable no sleep, when suspend */
#define   GH621X_PRIVATE_DISABLE_NO_SLEEP_VAL            (0x0001)    /**< private reg 1,disable no sleep */
#define   GH621X_PRIVATE_SP_DISABLE_NO_SLEEP_VAL         (0x0000)    /**< private reg 1,disable no sleep, when suspend*/

#define   GH621X_MCU_PD_BIT_SET_VAL                      (0x0004)    /**< mcu pd bit set value */
#define   GH621X_MCU_PD_BIT_RESET_VAL                    (0x0000)    /**< mcu pd bit reset value */

#define   GH621X_SAR_IRQ_0_WITHOUT_PROX_DONE_VAL         (0x9801)    /**< irq ctrl 0 val without prox done value */
#define   GH621X_SAR_IRQ_0_WITH_PROX_DONE_VAL            (0xD801)    /**< irq ctrl 0 val with prox done value */

#define   GH621X_TK_IED_IRQ_2_VAL                        (0xFFFD)    /**< irq ctrl 2 val */
#define   GH621X_TK_IED_IRQ_3_WITHOUT_RAWDATA_VAL        (0x0004)    /**< irq ctrl 3 val without rawdata */
#define   GH621X_TK_IED_IRQ_3_WITH_RAWDATA_VAL           (0x0404)    /**< irq ctrl 3 val with rawdata */

#define   GH621X_CRC16_CTRL_ENABLE_VAL                   (0x0001)    /**< crc16 ctrl enable value */
#define   GH621X_CRC16_CTRL_DISABLE_VAL                  (0x0000)    /**< crc16 ctrl disable value */

#define   GH621X_CRC16_STATUS_ERR_BIT_VAL                (0x0002)    /**< crc16 status error bit value */
#define   GH621X_CRC16_STATUS_DONE_BIT_VAL               (0x0001)    /**< crc16 status done bit value */

#define   GH621X_I2C_MEM_EX_ENABLE_VAL                   (0x0001)    /**< mem ex enable value */
#define   GH621X_I2C_MEM_ENABLE_VAL                      (0x0002)    /**< mem enable value */
#define   GH621X_I2C_MEM_DISABLE_VAL                     (0x0000)    /**< mem disable value */

#define   GH621X_RELEASE_MCU_VAL                         (0x0000)    /**< release mcu value */
#define   GH621X_MCU_HOLD_BIT_VAL                        (0x0001)    /**< mcu hold bit value */

#define   GH621X_WORK_MODE_HIGH_VAL                      (0x0001)    /**< work mode high value */
#define   GH621X_WORK_MODE_LOW_VAL                       (0x0000)    /**< work mode low value */

#define   GH621X_FSM_SLP_STATUS_BIT_MSK_VAL              (0x0C00)    /**< fsm slp(sleep/dsleep) bit msk value */
#define   GH621X_FSM_NORMAL_STATUS_BIT_VAL               (0x0200)    /**< fsm normal status bit value */
#define   GH621X_FSM_DSLP_STATUS_BIT_VAL                 (0x0800)    /**< fsm dslp status bit value */

#define   GH621X_SAR_DATA_READ_DONE_DIRECT_VAL           (0xA5A5)    /**< sar data read done direct flag value */
#define   GH621X_SAR_DATA_READ_DONE_WAIT_VAL             (0xA5A0)    /**< sar data read done wait time flag value*/

#define   GH621X_SAR_INIT_DONE_ROM_VAL                   (0xA500)    /**< sar init done & rom run value */
#define   GH621X_SAR_INIT_DONE_PATCH_VAL                 (0xA5AA)    /**< sar init done & patch run value */

#define   GH621X_CHECK_ROM_RUN_BIT_VAL                   (0x0001)    /**< check rom run bit value */

#define   GH621X_CHIP_SLEEP_COMM_VAL                     (0x55AA)    /**< chip sleep communicate value */

#define   GH621X_MODE_CTRL_ENABLE_ALL_OFF_VAL            (0x0000)    /**< mode ctrl enable all off value */

#define   GH621X_PRIVATE_REG_3_FSM_BIT_VAL               (0x1E00)    /**< fsm bit value */

#define   GH621X_SAR_AUTOCC_TRG_VAL                      (0x001F)    /**< autocc trg value */

#define   GH621X_SAR_AUTOCC_TRG_CH_FLAG_VAL              (0xA800)    /**< sar autocc trg flag val */

/* reg bit field index */
#define   GH621X_PRIVATE_REG_3_FSM_LOW_INDEX             (9)         /**< fsm val LSB index */
#define   GH621X_PRIVATE_REG_3_FSM_HIGH_INDEX            (12)        /**< fsm val MSB index */


/* patch code index */
#define  GH621X_PATCH_CODE_BIN_HEADER_LEN           (8u)    /**< bin header len */
#define  GH621X_PATCH_CODE_PROJECT_ID_H_INDEX       (0u)    /**< project id h index */
#define  GH621X_PATCH_CODE_PROJECT_ID_L_INDEX       (1u)    /**< project id l index */
#define  GH621X_PATCH_CODE_CHIP_TYPE_H_INDEX        (2u)    /**< chip type h index */
#define  GH621X_PATCH_CODE_CHIP_TYPE_L_INDEX        (3u)    /**< chip type l index */
#define  GH621X_PATCH_CODE_VERSION_1_INDEX          (4u)    /**< version 1 index */
#define  GH621X_PATCH_CODE_VERSION_2_INDEX          (5u)    /**< version 2 index */
#define  GH621X_PATCH_CODE_VERSION_3_INDEX          (6u)    /**< version 3 index */
#define  GH621X_PATCH_CODE_VERSION_4_INDEX          (7u)    /**< version 4 index */
#define  GH621X_PATCH_CODE_START_ADDR_H_INDEX       (GH621X_PATCH_CODE_BIN_HEADER_LEN + 0u) /**< start addr h index */
#define  GH621X_PATCH_CODE_START_ADDR_L_INDEX       (GH621X_PATCH_CODE_BIN_HEADER_LEN + 1u) /**< start addr l index */
#define  GH621X_PATCH_CODE_CRC_H_INDEX              (GH621X_PATCH_CODE_BIN_HEADER_LEN + 2u) /**< crc h index */
#define  GH621X_PATCH_CODE_CRC_L_INDEX              (GH621X_PATCH_CODE_BIN_HEADER_LEN + 3u) /**< crc l index */
#define  GH621X_PATCH_CODE_END_ADDR_H_INDEX         (GH621X_PATCH_CODE_BIN_HEADER_LEN + 4u) /**< end addr h index */
#define  GH621X_PATCH_CODE_END_ADDR_L_INDEX         (GH621X_PATCH_CODE_BIN_HEADER_LEN + 5u) /**< end addr l index */
#define  GH621X_PATCH_CODE_REAL_DATA_INDEX          (GH621X_PATCH_CODE_BIN_HEADER_LEN + 6u) /**< real data index */

/// patch code block buffer size
// #define  GH621X_PATCH_CODE_BLOCK_BUFFER_SIZE        (GH621X_MAX_BLOCK_SIZE_SUPPORT / GH621X_REG_VAL_SIZE)

#define   GH621X_CHECK_BIT_SET(x, b)                 (((x) & (b)) == (b))  /**< macro of check bits set */
#define   GH621X_CHECK_BIT_NOT_SET(x, b)             (((x) & (b)) != (b))  /**< macro of check bits not set */
#define   GH621X_CHECK_BIT_SET_BITMASK(x, m, b)      (((x) & (m)) == (b))  /**< macro of check bits set with bitmask */
#define   GH621X_CHECK_BIT_NOT_SET_BITMASK(x, m, b)  (((x) & (m)) != (b))  /**< macro of check bits not set */

#define   GH621X_IRQ0_MSK_SAR_CHIP_RESET_BIT         (0x0001)       /**< sar chip reset */
#define   GH621X_IRQ0_MSK_SAR_AUTOCC_DONE_BIT        (0x0080)       /**< sar autocc done */
#define   GH621X_IRQ0_MSK_SAR_PROX_CLOSE_BIT         (0x0800)       /**< sar prox close */
#define   GH621X_IRQ0_MSK_SAR_PROX_FAR_BIT           (0x1000)       /**< sar prox far */
#define   GH621X_IRQ0_MSK_SAR_PROX_DONE_BIT          (0x4000)       /**< sar prox done */
#define   GH621X_IRQ0_MSK_SAR_DEFAULT_CFG_DONE_BIT   (0x8000)       /**< sar load default cfg done */
#define   GH621X_IRQ3_MSK_SAR_WAIT_CFG_TIMEOUT_BIT   (0x2000)       /**< sar wait cfg timeout */
#define   GH621X_IRQ3_MSK_SAR_AUTOCC_FAIL_BIT        (0x4000)       /**< sar autocc fail */
#define   GH621X_IRQ3_MSK_SAR_AUTOCC_NOT_FOUND_BIT   (0x8000)       /**< sar autocc not found */

#define   GH621X_MAX_BLOCK_SIZE_SUPPORT                 (254)

#define   GH621x_SAR_RAWDATA_TEMPCELS_16BIT_INDEX              (0)
#define   GH621x_SAR_RAWDATA_TEMPCELS_16BIT_COUNT              (1)

#define   GH621x_SAR_RAWDATA_TEMPRAW_16BIT_INDEX               (1)
#define   GH621x_SAR_RAWDATA_TEMPRAW_16BIT_COUNT               (2)

#define   GH621x_SAR_RAWDATA_PRERAW_16BIT_INDEX                (3)
#define   GH621x_SAR_RAWDATA_PRERAW_16BIT_COUNT                (18)

#define   GH621x_SAR_RAWDATA_USEFUL_16BIT_INDEX                (21)
#define   GH621x_SAR_RAWDATA_USEFUL_16BIT_COUNT                (18)

#define   GH621x_SAR_RAWDATA_AVG_16BIT_INDEX                   (39)
#define   GH621x_SAR_RAWDATA_AVG_16BIT_COUNT                   (18)

#define   GH621x_SAR_RAWDATA_DIFF_16BIT_INDEX                  (57)
#define   GH621x_SAR_RAWDATA_DIFF_16BIT_COUNT                  (18)

#define   GH621x_SAR_RAWDATA_STATE_16BIT_INDEX                 (75)
#define   GH621x_SAR_RAWDATA_STATE_16BIT_COUNT                 (2)

typedef struct _gh621x_reg_t
{
    u16 addr;     /**< register address */
    u16 value;     /**< register val */
} gh621x_reg_t;
