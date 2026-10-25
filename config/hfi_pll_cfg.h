#ifndef CONFIG_HFI_PLL_CFG_H
#define CONFIG_HFI_PLL_CFG_H

typedef struct {
    float kp;
    float ki;
    float w_max;
    float int_max;
    float eps_dead;
    float init_off_rad;
} hfi_pll_cfg_t;

extern const hfi_pll_cfg_t g_hfi_pll_cfg;

#endif /* CONFIG_HFI_PLL_CFG_H */
