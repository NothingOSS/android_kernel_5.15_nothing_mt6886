#ifndef _SAR_TYPE_H_
#define _SAR_TYPE_H_

#include "aw_sar_comm_interface.h"

typedef int32_t (*aw_sar_chip_other_operation_t)(void *data);
typedef void (*aw_sar_chip_other_opera_free_t)(void *data);

enum aw_i2c_flags {
	AW_SAR_I2C_WR,
	AW_SAR_I2C_RD,
	AW_SAR_PACKAGE_RD,
};

typedef int32_t (*aw_sar_bin_opera_t)(struct aw_bin *aw_bin, void *load_bin_para);
typedef int32_t (*aw_sar_bin_load_fail_opera_t)(struct aw_bin *aw_bin, void *load_bin_para);

struct aw_sar_get_chip_info_t {
	void (*p_get_chip_info_node_fn)(void *data, char *buf, ssize_t *p_len);
};

struct aw_sar_load_bin_t {
	const uint8_t *bin_name;
	aw_sar_bin_opera_t bin_opera_func;
	aw_sar_bin_load_fail_opera_t bin_load_fail_opera;

	void (*p_get_prot_update_fw_node_fn)(void *data, char *buf, ssize_t *p_len);

	/*Perform different operations to update parameters*/
	int32_t (*p_update_fn)(void *data);
};

struct aw_sar_reg_data {
	unsigned char rw;
	unsigned short reg;
};

struct aw_sar_awrw_t {
	ssize_t (*p_set_awrw_node_fn)(void *data, const char *buf, size_t count);
	ssize_t (*p_get_awrw_node_fn)(void *data, char *buf);
};

struct aw_sar_reg_list_t {
	uint8_t reg_none_access;
	uint8_t reg_rd_access;
	uint8_t reg_wd_access;
	const struct aw_sar_reg_data *reg_perm;
	uint32_t reg_num;
};

typedef void (*aw_sar_update_work_t)(struct work_struct *work);
struct aw_sar_update_static_t {
	aw_sar_update_work_t update_work_func;
	uint32_t delay_ms;
};

typedef irqreturn_t (*aw_sar_irq_t)(int irq, void *data);
typedef uint32_t (*sar_rc_irqscr_t)(void *i2c);
//If the return value is 1, there is an initialization completion interrupt; if the return value is 0, there is no
typedef uint32_t (*aw_sar_is_init_over_irq)(uint32_t irq_status);
typedef void (*aw_sar_irq_spec_handler_t)(uint32_t irq_status, void *data);

struct aw_sar_check_chipid_t {
	/*Read chipid and check chipid, Must be implemented externally*/
	int32_t (*p_check_chipid_fn)(void *data);
};

struct aw_sar_irq_init_t {
	unsigned long flags;
	unsigned long irq_flags;
	irq_handler_t handler;
	irq_handler_t thread_fn;
	/*Interrupt processing parameters*/
	sar_rc_irqscr_t rc_irq_fn;
	//aw_sar_is_init_over_irq is_init_over_irq_fn;
	aw_sar_irq_spec_handler_t irq_spec_handler_fn;

	/*Use a different initialization interrupt to initialize the operation*/
	int32_t (*p_irq_init_fn)(void *data);
	/*Release interrupt resource*/
//	void const (*p_irq_deinit_fn)(void *data);
	int (*p_irq_deinit_fn)(void *data);
};

struct aw_sar_pm_t {
	uint32_t suspend_set_mode;
	uint32_t resume_set_mode;
	uint32_t shutdown_set_mode;
	//system api
	int32_t (*p_suspend_fn)(void *data);
	int32_t (*p_resume_fn)(void *data);
	int32_t (*p_shutdown_fn)(void *data);
};

struct aw_sar_chip_mode_t {
	uint32_t init_mode;
	uint32_t active;
	uint32_t pre_init_mode;
};

struct aw_sar_regulator_config_t {
	//Note that "_sar_num" after VCC name is defined by SAR C auto add
	const uint8_t *vcc_name;
	int32_t min_uV;
	int32_t max_uV;
};

struct aw_channels_info {
	uint16_t used;
	uint32_t last_channel_info;
	struct input_dev *input;
	uint8_t name[20];
};

struct aw_sar_dts_info {
	uint32_t sar_num;
	int32_t irq_gpio;
	uint32_t channel_use_flag;
	bool use_regulator_flag;
	bool use_inter_pull_up;
	bool use_pm;
	bool update_fw_flag;
};

struct aw_sar_irq_init_comm_t {
	int32_t to_irq;
	uint8_t host_irq_stat;
	void *data;
	uint8_t label[30];
	uint8_t dev_id[30];
};

struct aw_sar_load_bin_comm_t {
	uint8_t bin_name[30];
	aw_sar_bin_opera_t bin_opera_func;
	aw_sar_bin_load_fail_opera_t bin_load_fail_opera_func;
};

struct aw_awrw_info {
	uint8_t rw_flag;
	uint8_t addr_len;
	uint8_t data_len;
	uint8_t reg_num;
	uint32_t i2c_tranfar_data_len;
	uint8_t *p_i2c_tranfar_data;
};

/****************mode set start******************/
typedef void (*sar_enable_clock_t)(void *i2c);
typedef void (*sar_operation_irq_t)(int to_irq);
typedef void (*sar_mode_update_t)(void *i2c);

struct aw_sar_mode_switch_ops {
	sar_enable_clock_t enable_clock;
	sar_rc_irqscr_t rc_irqscr;
	sar_mode_update_t mode_update;
};

struct aw_sar_chip_mode {
	uint8_t curr_mode;
	uint8_t last_mode;
};

struct aw_sar_mode_set_t {
	uint8_t chip_id;
	struct aw_sar_chip_mode chip_mode;
	struct aw_sar_mode_switch_ops mode_switch_ops;
};

struct aw_sar_mode_t {
	const struct aw_sar_mode_set_t *mode_set_arr;
	uint8_t mode_set_arr_len;
	ssize_t (*p_set_mode_node_fn)(void *data, uint8_t curr_mode);
	ssize_t (*p_get_mode_node_fn)(void *data, char *buf);
};
/********************mode set end****************/

struct aw_sar_init_over_irq_t {
	int16_t wait_times;
	uint8_t daley_step;
	uint32_t reg_irqsrc;
	uint32_t irq_offset_bit;
	uint32_t irq_mask;
	uint32_t irq_flag;
	/*Perform different verification initialization to complete the interrupt operation*/
	int32_t (*p_check_init_over_irq_fn)(void *data);
	/*When initialization fails, get the corresponding error type and apply it to the chip with flash*/
	int32_t (*p_get_err_type_fn)(void *data);
};

struct aw_sar_soft_rst_t {
	uint16_t reg_rst;
	uint32_t reg_rst_val;
	uint32_t delay_ms;
	/*Perform different soft reset operations*/
	int32_t (*p_soft_reset_fn)(void *data);
};

struct aw_sar_aot_t {
	uint32_t aot_reg;
	uint32_t aot_mask;
	uint32_t aot_flag;
	ssize_t (*p_set_aot_node_fn)(void *data);
};

struct aw_sar_diff_t {
	uint16_t diff0_reg;
	uint16_t diff_step;
	//Data format:S21.10, Floating point types generally need to be removed
	uint32_t rm_float;
	
	ssize_t (*p_get_diff_node_fn)(void *data, char *buf);
};

struct aw_sar_offset_t {
	ssize_t (*p_get_offset_node_fn)(void *data, char *buf);
};

struct aw_sar_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *default_sta;
	struct pinctrl_state *int_out_high;
	struct pinctrl_state *int_out_low;
};

//update reg node
struct aw_sar_para_load_t {
	const uint32_t *reg_arr;
	uint32_t reg_arr_len;
};

struct aw_sar_platform_config {
	/*The chip needs to parse more DTS contents for addition*/
	int32_t (*p_add_parse_dts_fn)(void *data);

	const struct aw_sar_regulator_config_t *p_regulator_config;

	/*The chip needs to add more nodes*/
	int32_t (*p_add_node_create_fn)(void *data);
	/*Release the added node*/
	int32_t (*p_add_node_free_fn)(void *data);

	/*Use a different initialization interrupt to initialize the operation*/
	int32_t (*p_input_init_fn)(void *data);
	/*Release input resource*/
	int32_t (*p_input_deinit_fn)(void *data);

	//The parameters passed in are required for interrupt initialization
	const struct aw_sar_irq_init_t *p_irq_init;

	//The chip is set to different modes in different power management interfaces
	const struct aw_sar_pm_t *p_pm_chip_mode;
};

/*Parameter passed in by different SAR sensors.
 It must be implemented in each sensor code.
 If it is not necessary that some members can be assigned null,
 the corresponding function will not be implemented
*/
struct aw_sar_chip_config {
	uint8_t ch_num_max; //Number of channels of the chip

	//Chip related platform content configuration
	const struct aw_sar_platform_config *p_platform_config;
	//Parameters required for verification of chipid
	const struct aw_sar_check_chipid_t *p_check_chipid;
	//Parameters required for soft reset
	const struct aw_sar_soft_rst_t *p_soft_rst;
	//Verify the parameters required to initialize a complete interrupt
	const struct aw_sar_init_over_irq_t *p_init_over_irq;
	//Parameters required for load boot bin file,
	//If the chip does not have flash, please ignore and assign the value to null
	const struct aw_sar_load_bin_t *p_fw_bin;
	//Parameters required for load register bin file
	const struct aw_sar_load_bin_t *p_reg_bin;
	//The mode set before and after the initialization of the chip
	const struct aw_sar_chip_mode_t *p_chip_mode;

	//Node usage parameters
	//Register permission table
	const struct aw_sar_reg_list_t *p_reg_list;
	//Default register table
	const struct aw_sar_para_load_t *p_reg_arr;
	//Parameters required for set Auto-Offset-Tuning(aot)
	const struct aw_sar_aot_t *p_aot;
	//Parameters required for get chip diff val
	const struct aw_sar_diff_t *p_diff;
	//Parameters required for get chip offset val
	const struct aw_sar_offset_t *p_offset;
	//Set the parameters of different working modes of the chip
	const struct aw_sar_mode_t *p_mode;
	//Upgrading firmware using the debug node protocol
	const struct aw_sar_load_bin_t *p_prox_fw;
	//Upgrading firmware using the debug node reg
	const struct aw_sar_load_bin_t *p_reg_fw;
	//Obtain the necessary information of the chip
	const struct aw_sar_get_chip_info_t *p_get_chip_info;
	//Continuous read/write register interface
	const struct aw_sar_awrw_t *p_aw_sar_awrw;
	//Parameters required for load boot bin file,
	//If the chip does not have flash, please ignore and assign the value to null
	const struct aw_sar_load_bin_t *p_boot_bin;

	/*Other operations during initialization, Add according to different usage*/
	aw_sar_chip_other_operation_t p_other_operation;
	/*If requested by resources, please release*/
	aw_sar_chip_other_opera_free_t p_other_opera_free;
};

struct aw_sar {
	struct i2c_client *i2c;
	struct device *dev;
	struct regulator *vcc;
	struct delayed_work update_work;
	//Set pin pull-up mode
	struct aw_sar_pinctrl pinctrl;

	uint8_t chip_type;
	uint8_t chip_name[20];

	bool power_enable;
	bool fw_fail_flag;
	uint8_t last_mode;
	uint8_t ret_val;
	uint8_t curr_use_driver_type;
	int32_t prot_update_state;

	//Parameters related to platform logic
	struct aw_sar_dts_info dts_info;
	struct aw_sar_load_bin_comm_t load_bin;
	struct aw_sar_irq_init_comm_t irq_init;
	struct aw_awrw_info awrw_info;
	struct aw_channels_info *channels_arr;

	//Private arguments required for public functions
	const struct aw_sar_chip_config *p_sar_para;
	//Private arguments required for private functions
	void *priv_data;
};

//Determine whether the chip exists by verifying chipid
typedef int32_t (*aw_sar_who_am_i_t)(void *data);
typedef int32_t (*aw_sar_chip_init_t)(struct aw_sar *p_sar);
typedef void (*aw_sar_chip_deinit_t)(struct aw_sar *p_sar);

struct aw_sar_driver_type {
	uint8_t driver_type;
	aw_sar_who_am_i_t p_who_am_i;
	aw_sar_chip_init_t p_chip_init;
	aw_sar_chip_deinit_t p_chip_deinit;
};

#endif
