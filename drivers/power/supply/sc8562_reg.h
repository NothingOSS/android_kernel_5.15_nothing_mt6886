#ifndef __SC8562_HEADER__
#define __SC8562_HEADER__

/* Register 00h */
#define SC8562_REG_00                       (0x00)

/* Register 01h */
#define SC8562_REG_01                       (0x01)
#define SC8562_BAT_OVP_DIS_MASK             (0x80)
#define SC8562_BAT_OVP_DIS_SHIFT            (7)
#define SC8562_BAT_OVP_ENABLE               (0)
#define SC8562_BAT_OVP_DISABLE              (1)

#define SC8562_BAT_OVP_MASK_MASK            (0x40)
#define SC8562_BAT_OVP_MASK_SHIFT           (6)
#define SC8562_BAT_OVP_NOT_MASK             (0)
#define SC8562_BAT_OVP_IS_MASK              (1)

#define SC8562_BAT_OVP_FLAG_MASK            (0x20)
#define SC8562_BAT_OVP_FLAG_SHIFT           (5)

#define SC8562_BAT_OVP_MASK                 (0x1f)
#define SC8562_BAT_OVP_SHIFT                (0)
#define SC8562_BAT_OVP_BASE                 (4450)
#define SC8562_BAT_OVP_LSB                  (25)

/* Register 02h */
#define SC8562_REG_02                       (0x02)
#define	SC8562_BAT_OCP_DIS_MASK             (0x80)
#define	SC8562_BAT_OCP_DIS_SHIFT            (7)
#define SC8562_BAT_OCP_ENABLE               (0)
#define SC8562_BAT_OCP_DISABLE              (1)

#define SC8562_BAT_OCP_MASK_MASK            (0x20)
#define SC8562_BAT_OCP_MASK_SHIFT           (5)
#define SC8562_BAT_OCP_NOT_MASK             (0)
#define SC8562_BAT_OCP_IS_MASK              (1)

#define SC8562_BAT_OCP_FLAG_MASK            (0x10)
#define SC8562_BAT_OCP_FLAG_SHIFT           (4)

#define SC8562_BAT_OCP_MASK                 (0x0F)
#define SC8562_BAT_OCP_SHIFT                (0)
#define SC8562_BAT_OCP_BASE                 (6000)
#define SC8562_BAT_OCP_LSB                  (500)

/* Register 03h */
#define SC8562_REG_03                       (0x03)
#define SC8562_OVPGATE_ON_DG_MASK           (0x80)
#define SC8562_OVPGATE_ON_DG_SHIFT          (7)
#define SC8562_OVPGATE_ON_DG_20MS           (0)
#define SC8562_OVPGATE_ON_DG_128MS          (1)

#define SC8562_USB_OVP_MASK_MASK            (0x40)
#define SC8562_USB_OVP_MASK_SHIFT           (6)
#define SC8562_USB_OVP_NOT_MASK             (0)
#define SC8562_USB_OVP_IS_MASK              (1)

#define SC8562_USB_OVP_FLAG_MASK            (0x20)
#define SC8562_USB_OVP_FLAG_SHIFT           (5)

#define SC8562_USB_OVP_STAT_MASK            (0x10)
#define SC8562_USB_OVP_STAT_SHIFT           (4)

#define SC8562_USB_OVP_MASK                 (0x0F)
#define SC8562_USB_OVP_SHIFT                (0)
#define SC8562_USB_OVP_BASE                 (11000)
#define SC8562_USB_OVP_LSB                  (1000)
#define SC8562_USB_OVP_6PV5                 (0x0F)

/* Register 04h */
#define SC8562_REG_04                       (0x04)
#define SC8562_WPCGATE_ON_DG_MASK           (0x80)
#define SC8562_WPCGATE_ON_DG_SHIFT          (7)
#define SC8562_WPCGATE_ON_DG_20MS           (0)
#define SC8562_WPCGATE_ON_DG_128MS          (1)

#define SC8562_WPC_OVP_MASK_MASK            (0x40)
#define SC8562_WPC_OVP_MASK_SHIFT           (6)
#define SC8562_WPC_OVP_NOT_MASK             (0)
#define SC8562_WPC_OVP_IS_MASK              (1)

#define SC8562_WPC_OVP_FLAG_MASK            (0x20)
#define SC8562_WPC_OVP_FLAG_SHIFT           (5)

#define SC8562_WPC_OVP_STAT_MASK            (0x10)
#define SC8562_WPC_OVP_STAT_SHIFT           (4)

#define SC8562_WPC_OVP_MASK                 (0x0F)
#define SC8562_WPC_OVP_SHIFT                (0)
#define SC8562_WPC_OVP_BASE                 (11000)
#define SC8562_WPC_OVP_LSB                  (1000)
#define SC8562_WPC_OVP_6PV5                 (0x0F)

/* Register 05h */
#define SC8562_REG_05                       (0x05)
#define SC8562_BUS_OVP_MASK                 (0xFC)
#define SC8562_BUS_OVP_SHIFT                (2)
#define SC8562_BUS_OVP_41MODE_BASE          (14000)
#define SC8562_BUS_OVP_41MODE_LSB           (200)
#define SC8562_BUS_OVP_21MODE_BASE          (7000)
#define SC8562_BUS_OVP_21MODE_LSB           (100)
#define SC8562_BUS_OVP_11MODE_BASE          (3500)
#define SC8562_BUS_OVP_11MODE_LSB           (50)

#define SC8562_OUT_OVP_MASK                 (0x03)
#define SC8562_OUT_OVP_SHIFT                (0)
#define SC8562_OUT_OVP_BASE                 (4800)
#define SC8562_OUT_OVP_LSB                  (200)

/* Register 06h */
#define SC8562_REG_06                       (0x06)
#define SC8562_BUS_OCP_DIS_MASK             (0x80)
#define SC8562_BUS_OCP_DIS_SHIFT            (7)
#define SC8562_BUS_OCP_ENABLE               (0)
#define SC8562_BUS_OCP_DISABLE              (1)

#define SC8562_BUS_OCP_MASK_MASK            (0x40)
#define SC8562_BUS_OCP_MASK_SHIFT           (6)
#define SC8562_BUS_OCP_NOT_MASK             (0)
#define SC8562_BUS_OCP_IS_MASK              (1)

#define SC8562_BUS_OCP_FLAG_MASK            (0x20)
#define SC8562_BUS_OCP_FLAG_SHIFT           (5)

#define SC8562_BUS_OCP_MASK                 (0x1F)
#define SC8562_BUS_OCP_SHIFT                (0)
#define SC8562_BUS_OCP_BASE                 (2500)
#define SC8562_BUS_OCP_LSB                  (125)

/* Register 07h */
#define SC8562_REG_07                       (0x07)
#define SC8562_BUS_UCP_DIS_MASK             (0x80)
#define SC8562_BUS_UCP_DIS_SHIFT            (7)
#define SC8562_BUS_UCP_ENABLE               (0)
#define SC8562_BUS_UCP_DISABLE              (1)

#define SC8562_BUS_UCP_FALL_DG_MASK         (0x30)
#define SC8562_BUS_UCP_FALL_DG_SHIFT        (4)
#define SC8562_BUS_UCP_FALL_DG_8US          (0)
#define SC8562_BUS_UCP_FALL_DG_5MS          (1)
#define SC8562_BUS_UCP_FALL_DG_20MS         (2)
#define SC8562_BUS_UCP_FALL_DG_50MS         (3)

#define SC8562_BUS_UCP_RISE_MASK_MASK       (0x08)
#define SC8562_BUS_UCP_RISE_MASK_SHIFT      (3)
#define SC8562_BUS_UCP_RISE_NOT_MASK        (0)
#define SC8562_BUS_UCP_RISE_MASK            (1)

#define SC8562_BUS_UCP_RISE_FLAG_MASK       (0x04)
#define SC8562_BUS_UCP_RISE_FLAG_SHIFT      (2)

#define SC8562_BUS_UCP_FALL_MASK_MASK       (0x02)
#define SC8562_BUS_UCP_FALL_MASK_SHIFT      (1)
#define SC8562_BUS_UCP_FALL_NOT_MASK        (0)
#define SC8562_BUS_UCP_FALL_MASK            (1)

#define SC8562_BUS_UCP_FALL_FLAG_MASK       (0x01)
#define SC8562_BUS_UCP_FALL_FLAG_SHIFT      (0)

/* Register 08h */
#define SC8562_REG_08                       (0x08)
#define SC8562_PMID2OUT_OVP_DIS_MASK        (0x80)
#define SC8562_PMID2OUT_OVP_DIS_SHIFT       (7)
#define	SC8562_PMID2OUT_OVP_ENABLE          (0)
#define	SC8562_PMID2OUT_OVP_DISABLE         (1)

#define SC8562_PMID2OUT_OVP_MASK_MASK       (0x10)
#define SC8562_PMID2OUT_OVP_MASK_SHIFT      (4)
#define SC8562_PMID2OUT_OVP_NOT_MASK        (0)
#define SC8562_PMID2OUT_OVP_IS_MASK         (1)

#define SC8562_PMID2OUT_OVP_FLAG_MASK       (0x08)
#define SC8562_PMID2OUT_OVP_FLAG_SHIFT      (3)

#define SC8562_PMID2OUT_OVP_MASK            (0x07)
#define SC8562_PMID2OUT_OVP_SHIFT           (0)
#define SC8562_PMID2OUT_OVP_BASE            (200)
#define SC8562_PMID2OUT_OVP_LSB             (50)

/* Register 09h */
#define SC8562_REG_09												(0x09)
#define SC8562_PMID2OUT_UVP_DIS_MASK        (0x80)
#define SC8562_PMID2OUT_UVP_DIS_SHIFT       (7)
#define	SC8562_PMID2OUT_UVP_ENABLE          (0)
#define	SC8562_PMID2OUT_UVP_DISABLE         (1)

#define SC8562_PMID2OUT_UVP_MASK_MASK       (0x10)
#define SC8562_PMID2OUT_UVP_MASK_SHIFT      (4)
#define SC8562_PMID2OUT_UVP_NOT_MASK        (0)
#define SC8562_PMID2OUT_UVP_IS_MASK         (1)

#define SC8562_PMID2OUT_UVP_FLAG_MASK       (0x08)
#define SC8562_PMID2OUT_UVP_FLAG_SHIFT      (3)

#define SC8562_PMID2OUT_UVP_MASK            (0x07)
#define SC8562_PMID2OUT_UVP_SHIFT           (0)
#define SC8562_PMID2OUT_UVP_BASE            (50)
#define SC8562_PMID2OUT_UVP_LSB             (50)

/* Register 0Ah */
#define SC8562_REG_0A                       (0x0A)
#define SC8562_POR_FLAG_MASK                (0x80)
#define SC8562_POR_FLAG_SHIFT               (7)

#define SC8562_ACRB_WPC_STAT_MASK           (0x40)
#define SC8562_ACRB_WPC_STAT_SHIFT          (6)
#define SC8562_WPCGATE_TIED_TO_GND          (0)
#define SC8562_WPCGATE_NOT_TIED_TO_GND      (1)

#define SC8562_ACRB_USB_STAT_MASK           (0x20)
#define SC8562_ACRB_USB_STAT_SHIFT          (5)
#define SC8562_OVPGATE_TIED_TO_GND          (0)
#define SC8562_OVPGATE_NOT_TIED_TO_GND      (1)

#define SC8562_VBUS_ERRORLO_STAT_MASK       (0x10)
#define SC8562_VBUS_ERRORLO_STAT_SHIFT      (4)

#define SC8562_VBUS_ERRORHI_STAT_MASK       (0x08)
#define SC8562_VBUS_ERRORHI_STAT_SHIFT      (3)

#define SC8562_QB_ON_STAT_MASK              (0x04)
#define SC8562_QB_ON_STAT_SHIFT             (2)
#define SC8562_QB_OFF                       (0)
#define SC8562_QB_ON                        (1)

#define SC8562_CP_SWITCHING_STAT_MASK       (0x02)
#define SC8562_CP_SWITCHING_STAT_SHIFT      (1)
#define SC8562_CP_NOT_SWITCHING             (0)
#define SC8562_CP_SWITCHING                 (1)

#define SC8562_PIN_DIAG_FALL_FLAG_MASK      (0x01)
#define SC8562_PIN_DIAG_FALL_FLAG_SHIFT     (0)

/* Register 0Bh */
#define SC8562_REG_0B                       (0x0B)
#define SC8562_CHG_EN_MASK                  (0x80)
#define SC8562_CHG_EN_SHIFT                 (7)
#define SC8562_CHG_ENABLE                   (1)
#define SC8562_CHG_DISABLE                  (0)

#define SC8562_QB_EN_MASK                   (0x40)
#define SC8562_QB_EN_SHIFT                  (6)
#define SC8562_QB_ENABLE                    (1)
#define SC8562_QB_DISABLE                   (0)

#define SC8562_ACDRV_MANUAL_EN_MASK         (0x20)
#define SC8562_ACDRV_MANUAL_EN_SHIFT        (5)
#define SC8562_ACDRV_AUTO_MODE              (0)
#define SC8562_ACDRV_MANUAL_MODE            (1)

#define SC8562_WPCGATE_EN_MASK              (0x10)
#define SC8562_WPCGATE_EN_SHIFT             (4)
#define SC8562_WPCGATE_ENABLE               (1)
#define SC8562_WPCGATE_DISABLE              (0)

#define SC8562_OVPGATE_EN_MASK              (0x08)
#define SC8562_OVPGATE_EN_SHIFT             (3)
#define SC8562_OVPGATE_ENABLE               (1)
#define SC8562_OVPGATE_DISABLE              (0)

#define SC8562_VBUS_PD_EN_MASK              (0x04)
#define SC8562_VBUS_PD_EN_SHIFT             (2)
#define SC8562_VBUS_PD_ENABLE               (1)
#define SC8562_VBUS_PD_DISABLE              (0)

#define SC8562_VWPC_PD_EN_MASK              (0x02)
#define SC8562_VWPC_PD_EN_SHIFT             (1)
#define SC8562_VWPC_PD_ENABLE               (1)
#define SC8562_VWPC_PD_DISABLE              (0)

#define SC8562_VUSB_PD_EN_MASK              (0x01)
#define SC8562_VUSB_PD_EN_SHIFT             (0)
#define SC8562_VUSB_PD_ENABLE               (1)
#define SC8562_VUSB_PD_DISABLE              (0)

/* Register 0Ch */
#define SC8562_REG_0C                       (0x0C)
#define SC8562_FSW_SET_MASK                 (0xF8)
#define SC8562_FSW_SET_SHIFT                (3)
#define SC8562_FSW_SET_300K                 (0)
#define SC8562_FSW_SET_325K                 (1)
#define SC8562_FSW_SET_350K                 (2)
#define SC8562_FSW_SET_375K                 (3)
#define SC8562_FSW_SET_400K                 (4)
#define SC8562_FSW_SET_425K                 (5)
#define SC8562_FSW_SET_450K                 (6)
#define SC8562_FSW_SET_475K                 (7)
#define SC8562_FSW_SET_500K                 (8)
#define SC8562_FSW_SET_525K                 (9)
#define SC8562_FSW_SET_550K                 (10)
#define SC8562_FSW_SET_575K                 (11)
#define SC8562_FSW_SET_600K                 (12)
#define SC8562_FSW_SET_625K                 (13)
#define SC8562_FSW_SET_650K                 (14)
#define SC8562_FSW_SET_675K                 (15)
#define SC8562_FSW_SET_700K                 (16)
#define SC8562_FSW_SET_725K                 (17)
#define SC8562_FSW_SET_750K                 (18)
#define SC8562_FSW_SET_775K                 (19)
#define SC8562_FSW_SET_800K                 (20)
#define SC8562_FSW_SET_825K                 (21)
#define SC8562_FSW_SET_850K                 (22)
#define SC8562_FSW_SET_875K                 (23)
#define SC8562_FSW_SET_900K                 (24)
#define SC8562_FSW_SET_925K                 (25)
#define SC8562_FSW_SET_950K                 (26)
#define SC8562_FSW_SET_975K                 (27)
#define SC8562_FSW_SET_1000K                (28)
#define SC8562_FSW_SET_1025K                (29)
#define SC8562_FSW_SET_1050K                (30)
#define SC8562_FSW_SET_1075K                (31)

#define SC8562_FREQ_SHIFT_MASK              (0x04)
#define SC8562_FREQ_SHIFT_SHIFT             (2)
#define SC8562_FSW_NORMANL                  (0)
#define SC8562_FSW_SPREAD                   (1)

#define SC8562_SYNC_MASK                    (0x03)
#define SC8562_SYNC_SHIFT                   (0)
#define SC8562_SYNC_90DEG                   (0)
#define SC8562_SYNC_60DEG                   (1)
#define SC8562_SYNC_120DEG                  (2)
#define SC8562_SYNC_NO_SHIFT                (3)

/* Register 0Dh */
#define SC8562_REG_0D                       (0x0D)
#define SC8562_SS_TIMEOUT_SET_MASK          (0x38)
#define SC8562_SS_TIMEOUT_SET_SHIFT         (3)
#define SC8562_SS_TIMEOUT_DISABLE           (0)
#define SC8562_SS_TIMEOUT_40MS              (1)
#define SC8562_SS_TIMEOUT_80MS              (2)
#define SC8562_SS_TIMEOUT_320MS             (3)
#define SC8562_SS_TIMEOUT_1280MS            (4)
#define SC8562_SS_TIMEOUT_5120MS            (5)
#define SC8562_SS_TIMEOUT_20480MS           (6)
#define SC8562_SS_TIMEOUT_81920MS           (7)

#define SC8562_WD_TIMEOUT_SET_MASK          (0x07)
#define SC8562_WD_TIMEOUT_SET_SHIFT         (0)
#define SC8562_WD_TIMEOUT_DISABLE           (0)
#define SC8562_WD_TIMEOUT_0P2S              (1)
#define SC8562_WD_TIMEOUT_0P5S              (2)
#define SC8562_WD_TIMEOUT_1S                (3)
#define SC8562_WD_TIMEOUT_5S                (4)
#define SC8562_WD_TIMEOUT_30S               (5)
#define SC8562_WD_TIMEOUT_100S              (6)
#define SC8562_WD_TIMEOUT_255S              (7)

/* Register 0Eh */
#define SC8562_REG_0E                       (0x0E)
#define SC8562_SYNC_FUNCTION_EN_MASK        (0x80)
#define SC8562_SYNC_FUNCTION_EN_SHIFT       (7)
#define SC8562_SYNC_FUNCTION_DISABLE        (0)
#define SC8562_SYNC_FUNCTION_ENABLE         (1)

#define SC8562_SYNC_MASTER_EN_MASK          (0x40)
#define SC8562_SYNC_MASTER_EN_SHIFT         (6)
#define SC8562_SYNC_CONFIG_SLAVE            (0)
#define SC8562_SYNC_CONFIG_MASTER           (1)

#define SC8562_VBAT_OVP_DG_MASK             (0x20)
#define SC8562_VBAT_OVP_DG_SHIFT            (5)
#define SC8562_VBAT_OVP_NO_DG               (0)
#define SC8562_VBAT_OVP_DG_10US             (1)

#define SC8562_IBAT_SNS_RES_MASK            (0x10)
#define SC8562_IBAT_SNS_RES_SHIFT           (4)
#define SC8562_IBAT_SNS_RES_1MHM            (0)
#define SC8562_IBAT_SNS_RES_2MHM            (1)

#define SC8562_REG_RST_MASK                 (0x08)
#define SC8562_REG_RST_SHIFT                (7)
#define SC8562_REG_NO_RESET                 (0)
#define SC8562_REG_RESET                    (1)

#define SC8562_MODE_MASK                    (0x07)
#define SC8562_MODE_SHIFT                   (0)
#define SC8562_FORWARD_4_1_CHARGER_MODE     (0)
#define SC8562_FORWARD_2_1_CHARGER_MODE     (1)
#define SC8562_FORWARD_1_1_CHARGER_MODE     (2)
#define SC8562_FORWARD_1_1_CHARGER_MODE1    (3)
#define SC8562_REVERSE_1_4_CONVERTER_MODE   (4)
#define SC8562_REVERSE_1_2_CONVERTER_MODE   (5)
#define SC8562_REVERSE_1_1_CONVERTER_MODE   (6)
#define SC8562_REVERSE_1_1_CONVERTER_MODE1  (7)

/* Register 0Fh */
#define SC8562_REG_0F                       (0x0F)
#define SC8562_OVPGATE_STAT_MASK            (0x80)
#define SC8562_OVPFATE_STAT_SHIFT           (7)
#define SC8562_OVPFATE_OFF                  (0)
#define SC8562_OVPFATE_ON                   (1)

#define SC8562_WPCGATE_STAT_MASK            (0x40)
#define SC8562_WPCFATE_STAT_SHIFT           (6)
#define SC8562_WPCFATE_OFF                  (0)
#define SC8562_WPCFATE_ON                   (1)

/* Register 10h */
#define SC8562_REG_10                       (0x10)
#define SC8562_VOUT_OK_REV_STAT_MASK        (0x20)
#define SC8562_VOUT_OK_REV_STAT_SHIFT       (5)

#define SC8562_VOUT_OK_CHG_STAT_MASK        (0x10)
#define SC8562_VOUT_OK_CHG_STAT_SHIFT       (4)

#define SC8562_VOUT_INSERT_STAT_MASK        (0x08)
#define SC8562_VOUT_INSERT_STAT_SHIFT       (3)

#define SC8562_VBUS_PRESENT_STAT_MASK       (0x04)
#define SC8562_VBUS_PRESENT_STAT_SHIFT      (2)

#define SC8562_VWPC_INSERT_STAT_MASK        (0x02)
#define SC8562_VWPC_INSERT_STAT_SHIFT       (1)

#define SC8562_VUSB_INSERT_STAT_MASK        (0x01)
#define SC8562_VUSB_INSERT_STAT_SHIFT       (0)

/* Register 11h */
#define SC8562_REG_11                       (0x11)
#define SC8562_VOUT_OK_REV_FLAG_MASK        (0x20)
#define SC8562_VOUT_OK_REV_FLAG_SHIFT       (5)

#define SC8562_VOUT_OK_CHG_FLAG_MASK        (0x10)
#define SC8562_VOUT_OK_CHG_FLAG_SHIFT       (4)

#define SC8562_VOUT_INSERT_FLAG_MASK        (0x08)
#define SC8562_VOUT_INSERT_FLAG_SHIFT       (3)

#define SC8562_VBUS_PRESENT_FLAG_MASK       (0x04)
#define SC8562_VBUS_PRESENT_FLAG_SHIFT      (2)

#define SC8562_VWPC_INSERT_FLAG_MASK        (0x02)
#define SC8562_VWPC_INSERT_FLAG_SHIFT       (1)

#define SC8562_VUSB_INSERT_FLAG_MASK        (0x01)
#define SC8562_VUSB_INSERT_FLAG_SHIFT       (0)

/* Register 12h */
#define SC8562_REG_12                       (0x12)
#define SC8562_VOUT_OK_REV_MASK_MASK        (0x20)
#define SC8562_VOUT_OK_REV_MASK_SHIFT       (5)
#define SC8562_VOUT_OK_REV_NOT_MASK         (0)
#define SC8562_VOUT_OK_REV_MASK             (1)

#define SC8562_VOUT_OK_CHG_MASK_MASK        (0x10)
#define SC8562_VOUT_OK_CHG_MASK_SHIFT       (4)
#define SC8562_VOUT_OK_CHG_NOT_MASK         (0)
#define SC8562_VOUT_OK_CHG_MASK             (1)

#define SC8562_VOUT_INSERT_MASK_MASK        (0x08)
#define SC8562_VOUT_INSERT_MASK_SHIFT       (3)
#define SC8562_VOUT_INSERT_NOT_MASK         (0)
#define SC8562_VOUT_INSERT_MASK             (1)

#define SC8562_VBUS_PRESENT_MASK_MASK       (0x04)
#define SC8562_VBUS_PRESENT_MASK_SHIFT      (2)
#define SC8562_VBUS_PRESENT_NOT_MASK        (0)
#define SC8562_VBUS_PRESENT_MASK            (1)

#define SC8562_VWPC_INSERT_MASK_MASK        (0x02)
#define SC8562_VWPC_INSERT_MASK_SHIFT       (1)
#define SC8562_VWPC_INSERT_NOT_MASK         (0)
#define SC8562_VWPC_INSERT_MASK             (1)

#define SC8562_VUSB_INSERT_MASK_MASK        (0x01)
#define SC8562_VUSB_INSERT_MASK_SHIFT       (0)
#define SC8562_VUSB_INSERT_NOT_MASK         (0)
#define SC8562_VUSB_INSERT_MASK             (1)

/* Register 13h */
#define SC8562_REG_13                       (0x13)
#define SC8562_TSBAT_FLT_FLAG_MASK          (0x80)
#define SC8562_TSBAT_FLT_FLAG_SHIFT         (7)

#define SC8562_TSHUT_FLAG_MASK              (0x40)
#define SC8562_TSHUT_FLAG_SHIFT             (6)

#define SC8562_SS_TIMEOUT_FLAG_MASK         (0x20)
#define SC8562_SS_TIMEOUT_FLAG_SHIFT        (5)

#define SC8562_WD_TIMEOUT_FLAG_MASK         (0x10)
#define SC8562_WD_TIMEOUT_FLAG_SHIFT        (4)

#define SC8562_CONV_OCP_FLAG_MASK           (0x08)
#define SC8562_CONV_OCP_FLAG_SHIFT          (3)

#define SC8562_SS_FAIL_FLAG_MASK            (0x04)
#define SC8562_SS_FAIL_FLAG_SHIFT           (2)

#define SC8562_VBUS_OVP_FLAG_MASK           (0x02)
#define SC8562_VBUS_OVP_FLAG_SHIFT          (1)

#define SC8562_VOUT_OVP_FLAG_MASK           (0x01)
#define SC8562_VOUT_OVP_FLAG_SHIFT          (0)

/* Register 14h */
#define SC8562_REG_14                       (0x14)
#define SC8562_TSBAT_FLT_MASK_MASK          (0x80)
#define SC8562_TSBAT_FLT_MASK_SHIFT         (7)
#define SC8562_TSBAT_FLT_NOT_MASK           (0)
#define SC8562_TSBAT_FLT_MASK               (1)

#define SC8562_TSHUT_MASK_MASK              (0x40)
#define SC8562_TSHUT_MASK_SHIFT             (6)
#define SC8562_TSHUT_NOT_MASK               (0)
#define SC8562_TSHUT_MASK                   (1)

#define SC8562_SS_TIMEOUT_MASK_MASK         (0x20)
#define SC8562_SS_TIMEOUT_MASK_SHIFT        (5)
#define SC8562_SS_TIMEOUT_NOT_MASK          (0)
#define SC8562_SS_TIMEOUT_MASK              (1)

#define SC8562_WD_TIMEOUT_MASK_MASK         (0x10)
#define SC8562_WD_TIMEOUT_MASK_SHIFT        (4)
#define SC8562_WD_TIMEOUT_NOT_MASK          (0)
#define SC8562_WD_TIMEOUT_MASK              (1)

#define SC8562_CONV_OCP_MASK_MASK           (0x08)
#define SC8562_CONV_OCP_MASK_SHIFT          (3)
#define SC8562_CONV_OCP_NOT_MASK            (0)
#define SC8562_CONV_OCP_MASK                (1)

#define SC8562_SS_FAIL_MASK_MASK            (0x04)
#define SC8562_SS_FAIL_MASK_SHIFT           (2)
#define SC8562_SS_FAIL_NOT_MASK             (0)
#define SC8562_SS_FAIL_MASK                 (1)

#define SC8562_VBUS_OVP_MASK_MASK           (0x02)
#define SC8562_VBUS_OVP_MASK_SHIFT          (1)
#define SC8562_VBUS_OVP_NOT_MASK            (0)
#define SC8562_VBUS_OVP_MASK                (1)

#define SC8562_VOUT_OVP_MASK_MASK           (0x01)
#define SC8562_VOUT_OVP_MASK_SHIFT          (0)
#define SC8562_VOUT_OVP_NOT_MASK            (0)
#define SC8562_VOUT_OVP_MASK                (1)

/* Register 15h */
#define SC8562_REG_15                       (0x15)
#define SC8562_ADC_EN_MASK                  (0x80)
#define SC8562_ADC_EN_SHIFT                 (7)
#define SC8562_ADC_DISABLE                  (0)
#define SC8562_ADC_ENABLE                   (1)

#define SC8562_ADC_RATE_MASK                (0x40)
#define SC8562_ADC_RATE_SHIFT               (6)
#define SC8562_ADC_RATE_CONTINOUS           (0)
#define SC8562_ADC_RATE_ONESHOT             (1)

#define SC8562_ADC_DONE_STAT_MASK           (0x20)
#define SC8562_ADC_DONE_STAT_SHIFT          (5)

#define SC8562_ADC_DONE_FLAG_MASK           (0x10)
#define SC8562_ADC_DONE_FALG_SHIFT          (4)

#define SC8562_ADC_DONE_MASK_MASK           (0x08)
#define SC8562_ADC_DONE_MASK_SHIFT          (3)
#define SC8562_ADC_DONE_NOT_MASK            (0)
#define SC8562_ADC_DONE_MASK                (1)

#define SC8562_IBUS_ADC_DIS_MASK            (0x01)
#define SC8562_IBUS_ADC_DIS_SHIFT           (0)
#define SC8562_IBUS_ADC_ENABLE              (0)
#define SC8562_IBUS_ADC_DISABLE             (1)

/* Register 16h */
#define SC8562_REG_16                       (0x16)
#define SC8562_VBUS_ADC_DIS_MASK            (0x80)
#define SC8562_VBUS_ADC_DIS_SHIFT           (7)
#define SC8562_VBUS_ADC_ENABLE              (0)
#define SC8562_VBUS_ADC_DISABLE             (1)

#define SC8562_VUSB_ADC_DIS_MASK            (0x40)
#define SC8562_VUSB_ADC_DIS_SHIFT           (6)
#define SC8562_VUSB_ADC_ENABLE              (0)
#define SC8562_VUSB_ADC_DISABLE             (1)

#define SC8562_VWPC_ADC_DIS_MASK            (0x20)
#define SC8562_VWPC_ADC_DIS_SHIFT           (5)
#define SC8562_VWPC_ADC_ENABLE              (0)
#define SC8562_VWPC_ADC_DISABLE             (1)

#define SC8562_VOUT_ADC_DIS_MASK            (0x10)
#define SC8562_VOUT_ADC_DIS_SHIFT           (4)
#define SC8562_VOUT_ADC_ENABLE              (0)
#define SC8562_VOUT_ADC_DISABLE             (1)

#define SC8562_VBAT_ADC_DIS_MASK            (0x08)
#define SC8562_VBAT_ADC_DIS_SHIFT           (3)
#define SC8562_VBAT_ADC_ENABLE              (0)
#define SC8562_VBAT_ADC_DISABLE             (1)

#define SC8562_IBAT_ADC_DIS_MASK            (0x04)
#define SC8562_IBAT_ADC_DIS_SHIFT           (2)
#define SC8562_IBAT_ADC_ENABLE              (0)
#define SC8562_IBAT_ADC_DISABLE             (1)

#define SC8562_TSBAT_ADC_DIS_MASK           (0x02)
#define SC8562_TSBAT_ADC_DIS_SHIFT          (1)
#define SC8562_TSBAT_ADC_ENABLE             (0)
#define SC8562_TSBAT_ADC_DISABLE            (1)

#define SC8562_TDIE_ADC_DIS_MASK            (0x01)
#define SC8562_TDIE_ADC_DIS_SHIFT           (0)
#define SC8562_TDIE_ADC_ENABLE              (0)
#define SC8562_TDIE_ADC_DISABLE             (1)

/* Register 17h */
#define SC8562_REG_17                       (0x17)
#define SC8562_IBUS_POL_H_MASK              (0x0F)
#define SC8562_IBUS_ADC_LSB                 (15625 / 10 )

/* Register 18h */
#define SC8562_REG_18                       (0x18)
#define SC8562_IBUS_POL_L_MASK              (0xFF)

/* Register 19h */
#define SC8562_REG_19                       (0x19)
#define SC8562_VBUS_POL_H_MASK              (0x0F)
#define SC8562_VBUS_ADC_LSB                 (6250)

/* Register 1Ah */
#define SC8562_REG_1A                       (0x1A)
#define SC8562_VBUS_POL_L_MASK              (0xFF)

/* Register 1Bh */
#define SC8562_REG_1B                       (0x1B)
#define SC8562_VUSB_POL_H_MASK              (0x0F)
#define SC8562_VUSB_ADC_LSB                 (6250) 

/* Register 1Ch */
#define SC8562_REG_1C                       (0x1C)
#define SC8562_VUSB_POL_L_MASK              (0xFF)

/* Register 1Dh */
#define SC8562_REG_1D D                     (0x1D)
#define SC8562_VWPC_POL_H_MASK              (0x0F)
#define SC8562_VWPC_ADC_LSB                 (6250)

/* Register 1Eh */
#define SC8562_REG_1E                       (0x1E)
#define SC8562_VWPC_POL_L_MASK              (0xFF)

/* Register 1Fh */
#define SC8562_REG_1F                       (0x1F)
#define SC8562_VOUT_POL_H_MASK              (0x0F)
#define SC8562_VOUT_ADC_LSB                 (1250)

/* Register 20h */
#define SC8562_REG_20                       (0x20)
#define SC8562_VOUT_POL_L_MASK              (0x0F)

/* Register 21h */
#define SC8562_REG_21                       (0x21)
#define SC8562_VBAT_POL_H_MASK              (0x0F)
#define SC8562_VBAT_ADC_LSB                 (1250)

/* Register 22h */
#define SC8562_REG_22                       (0x22)
#define SC8562_VBAT_POL_L_MASK              (0xFF)

/* Register 23h */
#define SC8562_REG_23                       (0x23)
#define SC8562_IBAT_POL_H_MASK              (0x0F)
#define SC8562_IBAT_ADC_LSB                 (3125)

/* Register 24h */
#define SC8562_REG_24                       (0x24)
#define SC8562_IBAT_POL_L_MASK              (0xFF)

/* Register 25h */
#define SC8562_REG_25                       (0x25)
#define SC8562_TSBAT_POL_H_MASK             (0x03)
#define SC8562_TSBAT_ADC_LSB                (9766 / 100000)

/* Register 26h */
#define SC8562_REG_26                       (0x26)
#define SC8562_TSBAT_POL_L_MASK             (0xFF)

/* Register 27h */
#define SC8562_REG_27                       (0x27)
#define SC8562_TDIE_POL_H_MASK              (0x01)
#define SC8562_TDIE_ADC_LSB                 (5 / 10)

/* Register 28h */
#define SC8562_REG_28                       (0x28)
#define SC8562_TDIE_POL_L_MASK              (0xFF)

/* Register 29h */
#define SC8562_REG_29                       (0x29)
#define SC8562_TSBAT_FLT1_MASK              (0xFF)
#define SC8562_TSBAT_FLT1_SHIFT             (0)
#define SC8562_TSBAT_FLT1_BASE              (0)
#define SC8562_TSBAT_FLT1_LSB               (19531 / 100000)

/* Register 6Ch */
#define SC8562_REG_6C                       (0x6C)
#define SC8562_BAT_OVP_ALM_DIS_MASK         (0x80)
#define SC8562_BAT_OVP_ALM_DIS_SHIFT        (7)
#define SC8562_BAT_OVP_ALM_ENABLE           (0)
#define SC8562_BAT_OVP_ALM_DISABLE          (1)

#define SC8562_BAT_OVP_ALM_MASK_MASK        (0x40)
#define SC8562_BAT_OVP_ALM_MASK_SHIFT       (6)
#define SC8562_BAT_OVP_ALM_NOT_MASK         (0)
#define SC8562_BAT_OVP_ALM_IS_MASK          (1)

#define SC8562_BAT_OVP_ALM_FLAG_MASK        (0x20)
#define SC8562_BAT_OVP_ALM_FLAG_SHIFT       (5)

#define SC8562_BAT_OVP_ALM_MASK             (0x1F)
#define SC8562_BAT_OVP_ALM_SHIFT            (0)
#define SC8562_BAT_OVP_ALM_BASE             (4450)
#define SC8562_BAT_OVP_ALM_LSB              (25)

/* Register 6Dh */
#define SC8562_REG_6D                       (0x6D)
#define SC8562_BUS_OCP_ALM_DIS_MASK         (0x80)
#define SC8562_BUS_OCP_ALM_DIS_SHIFT        (7)
#define SC8562_BUS_OCP_ALM_ENABLE           (0)
#define SC8562_BUS_OCP_ALM_DISABLE          (1)

#define SC8562_BUS_OCP_ALM_MASK_MASK        (0x40)
#define SC8562_BUS_OCP_ALM_MASK_SHIFT       (6)
#define SC8562_BUS_OCP_ALM_NOT_MASK         (0)
#define SC8562_BUS_OCP_ALM_IS_MASK          (1)

#define SC8562_BUS_OCP_ALM_FLAG_MASK        (0x20)
#define SC8562_BUS_OCP_ALM_FLAG_SHIFT       (5)

#define SC8562_BUS_OCP_ALM_MASK             (0x1F)
#define SC8562_BUS_OCP_ALM_SHIFT            (0)
#define SC8562_BUS_OCP_ALM_BASE             (2500)
#define SC8562_BUS_OCP_ALM_LSB              (125)

/* Register 6Eh */
#define SC8562_REG_6E                       (0x6E)
#define SC8562_DEVICE_ID                    (0x61)

/* Register 70h */
#define SC8562_REG_70                       (0x70)
#define SC8562_TSBAT_FLT_DIS_MASK         	(0x08)
#define SC8562_TSBAT_FLT_DIS_SHIFT        	(3)
#define SC8562_TSBAT_FLT_DIS_ENABLE         (0)
#define SC8562_TSBAT_FLT_DIS_DISABLE        (1)
#endif