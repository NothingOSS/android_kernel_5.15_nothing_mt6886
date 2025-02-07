#ifndef AW_SAR_H_
#define AW_SAR_H_

void aw_sar_disable_irq(struct aw_sar *p_sar);
void aw_sar_enable_irq(struct aw_sar *p_sar);

int32_t aw_sar_soft_reset(struct aw_sar *p_sar);
int32_t aw_sar_check_init_over_irq(struct aw_sar *p_sar);
int32_t aw_sar_update_fw(struct aw_sar *p_sar);
int32_t aw_sar_load_def_reg_bin(struct aw_sar *p_sar);
void aw_sar_mode_set(struct aw_sar *p_sar, uint8_t curr_mode);
int32_t aw_sar_update_reg_set_func(struct aw_sar *p_sar);
#endif