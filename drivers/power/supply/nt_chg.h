#define NT_CHG_DRV_VERSION	"1.0.0_NT"

/*ma*/
#define ADC_ACCURACY_COMPENSATION        (100)
#define SC8562_DEVICE_ID                 (0x61)
#define AW32280_CHIP_ID3                 (0x36)
#define CP_WORKMODE_DEF                  (4)
#define ABOVE_IBAT_COMPENSATION          (2600)
#define PLUGIN_VOLTAGE                   (2500)
#define PROJECT_DATA_ID                  (2)
#define NT_CAM_ON                        (1000)
#define NT_CAM_OFF                       (9000)
#define NT_CAM_LMT_BAT_LEVEL             (15)
#define NT_CAM_LMT_IBUS                  (300000)
#define NT_CAM_RESTORE_IBUS              (3200000)
#define NT_CAM_ON_INTERVAL_MS            (2000)
#define NT_CAM_POLL_INTERVAL_MS          (10000)
#define NT_COUNT_FIVE_MINUTE             (150)
static int nt_ctrl_count = 0;
static bool nt_ctrl_timeout = false;
#if IS_ENABLED(CONFIG_PROC_FS)
struct nt_proc {
	const char *name;
	const struct proc_ops *fops;
};
int  g_nt_cp_ctrl  = -400;

#define PROC_FOPS_RO(name)																						\
static int name ## _proc_open(struct inode *inode, struct file *file)	\
{                                                                     \
	return single_open(file, name ## _proc_show, PDE_DATA(inode));   		\
}                                                                     \
static const struct proc_ops name ## _proc_fops = {               		\
	.proc_open = name ## _proc_open,                                    \
	.proc_read = seq_read,                                              \
	.proc_lseek = seq_lseek,                                            \
	.proc_release = single_release,                                     \
}

#define PROC_FOPS_RW(name)																						\
static int name ## _proc_open(struct inode *inode, struct file *file) \
{																																			\
	return single_open(file, name ## _proc_show, PDE_DATA(inode));			\
}																																			\
static const struct proc_ops name ## _proc_fops = {										\
	.proc_open           = name ## _proc_open,													\
	.proc_read           = seq_read,																		\
	.proc_lseek         = seq_lseek,																		\
	.proc_release        = single_release,															\
	.proc_write          = name ## _proc_write,													\
}

#define PROC_ENTRY(name)	{__stringify(name), &name ## 	_proc_fops}
#endif

enum cam_wq_t {
	EXIT_WQ = 0,
	RERUN_WQ,
};

enum nt_fake_value {
	FAKE_SOC = 0,
	FAKE_TBAT,
	FAKE_VBAT,
	FAKE_IBAT,
	FAKE_TUSB,
	FAKE_TYPE_MAX,
};

enum nt_charge_abnormal_type {
	NT_NOTIFY_USB_TEMP_ABNORMAL = 1 << 0,
	NT_NOTIFY_CHARGER_OVER_VOL = 1 << 1,
	NT_NOTIFY_CHARGER_LOW_VOL = 1 << 2,
	NT_NOTIFY_BAT_OVER_TEMP = 1 << 3,
	NT_NOTIFY_BAT_LOW_TEMP = 1 << 4,
	NT_NOTIFY_BAT_NOT_CONNECT = 1 << 5,
	NT_NOTIFY_BAT_OVER_VOL = 1 << 6,
	NT_NOTIFY_BAT_FULL = 1 << 7,
	NT_NOTIFY_CHGING_CURRENT = 1 << 8,
	NT_NOTIFY_CHGING_OVERTIME = 1 << 9,
	NT_NOTIFY_BAT_FULL_PRE_HIGH_TEMP = 1 << 10,
	NT_NOTIFY_BAT_FULL_PRE_LOW_TEMP = 1 << 11,
	NT_NOTIFY_BAT_FULL_THIRD_BATTERY = 1 << 12,
	NT_NOTIFY_CHARGE_PUMP_ERR = 1 << 13,
	NT_NOTIFY_WLS_TX_FOD_ERR_CODE2 = 1 << 14,
	NT_NOTIFY_SHORT_C_BAT_FULL_ERR_CODE3 = 1 << 15,
	NT_NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE4 = 1 << 16,
	NT_NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE5 = 1 << 17,
	NT_NOTIFY_CHARGER_TERMINAL = 1 << 18,
	NT_NOTIFY_GAUGE_I2C_ERR = 1 << 19,
};

struct notify_info {
	enum nt_charge_abnormal_type type;
	const char *item_text;
};

enum {
	STAT_SLEEP,
	STAT_VBUS_RDY,
	STAT_TRICKLE,
	STAT_PRE,
	STAT_FAST,
	STAT_EOC,
	STAT_BKGND,
	STAT_DONE,
	STAT_FAULT,
	STAT_OTG = 15,
	STAT_MAX,
};

enum nt_online_param_type {
	ONLINE_PARAM_USB_TEMP = 1,
};

struct nt_chg_info {
	struct device *dev;
	struct power_supply *chgpsy;
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *psy;
	/* system lock */
	spinlock_t slock;
	struct wakeup_source *charger_wakelock;
	struct wakeup_source *chg_wake;
	struct mutex charger_lock;
	struct mutex proc_lock;

	/* thread related */
	wait_queue_head_t  wait_que;
	bool charger_thread_timeout;
	unsigned int polling_interval;
	bool charger_thread_polling;

	/*cam delay work*/
	struct workqueue_struct *cam_workqueue;
	struct delayed_work cam_delay_work;
	/* alarm timer */
	struct alarm charger_timer;
	struct timespec64 endtime;
	bool is_suspend;
	struct notifier_block pm_notifier;

	bool wd0_state;

	struct mtk_charger *info;
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	int typec_attach;
	int cmd_discharging;
	int fcc;
	int is_aging_test;
	bool is_hvcharger;
	int sc_input_current;
	int sc_charger_current;
	int ac_charger_current;
	int ac_charger_input_current;
	int pd_input_current;
	int pd_charger_current;
	bool ship_mode_en;
	int otg_enable;
	int usbTemp;
	unsigned int notify_code;
	unsigned int nt_data[10];
	int chg_type;
	int adc_compensation;
	int cp_workmode;

	int fake_soc;
	int fake_tbat;
	int fake_vbat;
	int fake_ibat;
	int fake_tusb;

	int cam_on_off;
	int pre_nt_cam;
	int cam_lmt;
};

static struct notify_info g_abnormal_info[] = {
	{NT_NOTIFY_USB_TEMP_ABNORMAL ,	"[usb temp abnormal]"},
	{NT_NOTIFY_CHARGER_OVER_VOL ,	"[charger over vol]"},
	{NT_NOTIFY_CHARGER_LOW_VOL ,	"[charger low vol]"},
	{NT_NOTIFY_BAT_OVER_TEMP ,	"[bat over temp]"},
	{NT_NOTIFY_BAT_LOW_TEMP ,	"[bat low temp]"},
	{NT_NOTIFY_BAT_NOT_CONNECT ,	"[bat not connect]"},
	{NT_NOTIFY_BAT_OVER_VOL ,	"[bat over ovl]"},
	{NT_NOTIFY_BAT_FULL ,	"[bat full]"},
	{NT_NOTIFY_CHGING_CURRENT ,	"[charging current]"},
	{NT_NOTIFY_CHGING_OVERTIME ,	"[charging over time]"},
	{NT_NOTIFY_BAT_FULL_PRE_HIGH_TEMP ,	"[bat full high temp]"},
	{NT_NOTIFY_BAT_FULL_PRE_LOW_TEMP ,	"[bat full low temp]"},
	{NT_NOTIFY_BAT_FULL_THIRD_BATTERY ,	"[bat full third battery]"},
	{NT_NOTIFY_CHARGE_PUMP_ERR ,	"[charger pump err]"},
	{NT_NOTIFY_WLS_TX_FOD_ERR_CODE2 ,	"[wls tx fod err code2]"},
	{NT_NOTIFY_SHORT_C_BAT_FULL_ERR_CODE3 ,	"[short c bat full err code3]"},
	{NT_NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE4 ,	"[short c bat dynamic err code4]"},
	{NT_NOTIFY_SHORT_C_BAT_DYNAMIC_ERR_CODE5 ,	"[short c bat dynamic err code5]"},
	{NT_NOTIFY_CHARGER_TERMINAL ,	"[charger terminal]"},
	{NT_NOTIFY_GAUGE_I2C_ERR ,	"[fg i2c err]"},
};

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

#define DUMP_MSG_BUF_SIZE		  (512)
/*Attach 10s*/
#define CHG_THREAD_INTERVAL_A	  (10)
/*Dettach 2min*/
#define CHG_THREAD_INTERVAL_D	  (120)
#define BOOST_CV_MIN			  (20)
#define BOOST_CV_MAX			  (46)
#define BOOST_CV_BASE			  (4850)
#define BOOST_CV_OFFSET			  (25)
#define SHIP_MODE_MAGIC			  (1)
#define OTG_ENABLE_MODE			  (3)
#define OTG_DISABLE_MODE		  (4)

#define HVDCP_SW_VBUSOV_UV		14000*1000

#if IS_ENABLED(CONFIG_PROC_FS)
static struct proc_dir_entry *chg_proc_dir;
static struct proc_dir_entry *chg_usb_proc_dir;
#endif
struct nt_chg_info *g_nt_chg = NULL;

