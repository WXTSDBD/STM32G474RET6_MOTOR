# 宏归属表：`motor_params_m1.h`（2026-10-07）

**性质**：只读扫描产物，**不改固件**。分类为名称启发初值，搬宏前须人工复核。

## 1. 总量

| 项 | 数量 |
|---|---:|
| 文件行数 | 3378 |
| `#define` 行数 | 1003 |
| `#define` 唯一宏名 | 591 |
| `#undef` 行数 | 182 |
| `#undef` 波及的唯一宏名 | 102 |
| 条件编译（`#if/#ifdef/#ifndef/#elif`，密度口径） | 692 |
| 条件编译密度 | 20.5% |
| 全部预处理条件行（含 `#else/#endif`） | 1366 |
| `#error` | 85 |
| `_Static_assert`（本文件内） | 0 |

## 2. 定义风格（地雷在 `undef_force`）

| 风格 | 宏数 | 含义 |
|---|---:|---|
| `ifndef_default` | 353 | profile 可覆盖 |
| `bare` | 98 | 直接 `#define`，覆盖难 |
| `undef_force` | 102 | `#undef` 后强写，**include 顺序敏感** |
| `mixed` | 38 | 多种风格并存 |

## 3. 启发分类（初值）

| 归属猜想 | 宏数 | 建议 |
|---|---:|---|
| `tune_param` | 150 | 先归 B 类再定 |
| `capability` | 140 | 保留宏（能力裁剪） |
| `experiment` | 140 | → atrb / exp / profile |
| `other` | 123 | 先归 B 类再定 |
| `profile_select` | 21 | 保留（bringup_active） |
| `board_hw` | 11 | → 板级 desc 或保留宏 |
| `nameplate` | 6 | → `motor_cfg_t`（A，表+undef 收口后） |

## 4. 建议刀序（硬前置）

1. **本表**（已生成）人工抽查 nameplate / undef_force 两栏。
2. **收口 `#undef` 强制项**（优先「有读取点」的 99 个），按批改，勿一次清完 182 行。
3. 再搬 **铭牌 → `motor_cfg_t`**。
4. 实验注册表 E0 **另刀**，不与宏账绑在一起。

## 5. 铭牌候选（启发）

| 宏 | 行 | 风格 | 读取文件数 |
|---|---:|---|---:|
| `M1_POLE_PAIRS` | 16 | bare | 6 |
| `M1_RS_OHM` | 42 | bare | 9 |
| `M1_LD_H` | 43 | bare | 2 |
| `M1_LQ_H` | 44 | bare | 2 |
| `M1_VBUS_V` | 760 | bare | 2 |
| `M1_DEADTIME_NS` | 2753 | bare | 0 |

## 6. 高风险：`undef_force` 且已被代码引用（Top）

| 宏 | 读取文件数 | 引用次数 | 归属猜想 | Top 文件 |
|---|---:|---:|---|---|
| `M1_OPEN_UD_PRE_ID_CAL_ENABLE` | 13 | 42 | capability | motor/motor_open_sweep.c:9,motor/deadband/deadband_flow.c:8,debug/app_uart_dma_d |
| `M1_OPEN_UQ_PRE_ID_CAL_ENABLE` | 13 | 38 | capability | motor/motor_open_sweep.c:8,motor/deadband/deadband_flow.c:7,debug/app_uart_dma_d |
| `M1_VOFA_IDENT_DUMP_ENABLE` | 11 | 15 | capability | motor/telem_ident_dump.c:3,motor/deadband/deadband_id_cal.c:3,motor/deadband/dea |
| `M1_DEADBAND_ENABLE` | 10 | 11 | capability | motor/deadband/deadband_service.c:2,motor/motor_open_sweep.c:1,config/profiles/m |
| `M1_TELEM_BRINGUP_K` | 9 | 22 | experiment | debug/app_uart_dma_debug.c:6,config/profiles/m1_bode_id_fc1000.profile.h:2,confi |
| `M1_TELEM_BRINGUP_DECIMATION` | 9 | 19 | experiment | debug/app_uart_dma_debug.c:3,config/profiles/m1_bode_id_fc1000.profile.h:2,confi |
| `M1_VOFA_UNIFIED_12CH` | 9 | 17 | experiment | config/profiles/m1_bode_id_fc1000.profile.h:2,config/profiles/m1_flux_id_1000rpm |
| `M1_DEADBAND_NVM_ON_BOOT` | 9 | 10 | capability | motor/deadband/deadband_service.c:2,motor/deadband/deadband_id_cal.c:1,config/pr |
| `M1_PLL_ENABLE` | 7 | 32 | capability | motor/motor_current.c:20,debug/app_uart_dma_debug.c:4,config/profiles/m1_hfi_sta |
| `M1_CLOSURE_BRINGUP` | 7 | 14 | experiment | config/profiles/m1_bode_id_fc1000.profile.h:2,config/profiles/m1_flux_id_1000rpm |
| `M1_VOFA_PLL_CH8_11` | 6 | 11 | experiment | config/profiles/m1_flux_id_1000rpm.profile.h:2,config/profiles/m1_obs_veq_1000rp |
| `M1_VOFA_SPEED_CH8_11` | 6 | 11 | experiment | config/profiles/m1_flux_id_1000rpm.profile.h:2,config/profiles/m1_obs_veq_1000rp |
| `M1_LD_LQ_MULTI_ANGLE_ENABLE` | 5 | 21 | capability | motor/deadband/deadband_id_cal.c:12,motor/ident/ld_lq_ident.c:5,debug/app_uart_d |
| `M1_SPEED_OMEGA_RAMP_ENABLE` | 5 | 20 | capability | motor/motor_outer_loop.c:12,config/profiles/m1_hfi_standstill.profile.h:2,config |
| `M1_VOFA_LUT_DUMP_ENABLE` | 5 | 12 | capability | motor/deadband/deadband_id_cal.c:4,motor/telem_lut_dump.c:3,motor/deadband/deadb |
| `M1_LD_LQ_IDENT_F2_ENABLE` | 4 | 15 | capability | motor/ident/ld_lq_ident.c:8,motor/telem_ident_dump.c:4,motor/telem_ident_dump.h: |
| `M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE` | 4 | 14 | capability | motor/ident/ld_lq_ident.c:11,motor/motor_current.c:1,motor/motor_foc_loop.c:1,mo |
| `M1_IDENT_OVERRIDE_LIMITS` | 4 | 6 | tune_param | motor/motor_foc_loop.c:2,config/profiles/m1_bode_id_fc1000.profile.h:2,motor/dea |
| `M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE` | 3 | 14 | capability | motor/deadband/deadband_cal.c:9,motor/deadband/deadband.c:4,motor/deadband/deadb |
| `M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD` | 3 | 10 | other | motor/deadband/deadband_flow.c:6,motor/deadband/deadband_id_cal.c:3,motor/deadba |
| `M1_POS_OMEGA_MAX_RPM` | 3 | 9 | tune_param | motor/motor_outer_loop.c:5,config/profiles/m1_sensed_pos_mit.profile.h:2,config/ |
| `M1_POS_KP_RPM_PER_RAD` | 3 | 8 | tune_param | motor/motor_outer_loop.c:4,config/profiles/m1_sensed_pos_mit.profile.h:2,config/ |
| `M1_DEADBAND_I_ZERO_DISABLE` | 3 | 5 | other | motor/deadband/deadband.c:2,motor/deadband/deadband_cal.c:2,motor/telem_lut_dump |
| `M1_IDENT_BODE_AXIS_ID` | 3 | 5 | other | motor/ident/ident_module.c:2,config/profiles/m1_bode_id_fc1000.profile.h:2,debug |
| `M1_IDENT_BODE_BANDS` | 3 | 5 | tune_param | motor/ident/ident_flow.c:2,config/profiles/m1_bode_id_fc1000.profile.h:2,motor/i |
| `M1_IDENT_BODE_LUT_ENABLE` | 3 | 4 | capability | config/profiles/m1_bode_id_fc1000.profile.h:2,motor/ident/ident_flow.c:1,motor/i |
| `M1_LD_LQ_IDENT_ANGLE_COUNT` | 2 | 12 | other | motor/ident/ld_lq_ident.c:7,motor/deadband/deadband_id_cal.c:5 |
| `M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE` | 2 | 11 | capability | motor/deadband/deadband_id_cal.c:10,debug/app_uart_dma_debug.c:1 |
| `M1_IDENT_STEP_OFF_ROUNDS` | 2 | 7 | other | motor/ident/ident_flow.c:5,config/profiles/m1_bode_id_fc1000.profile.h:2 |
| `M1_OPEN_PRE_ID_LADDER_AB_ENABLE` | 2 | 7 | capability | motor/motor_open_sweep.c:6,motor/deadband/deadband_flow.c:1 |
| `M1_IDENT_STEP_BANDS` | 2 | 6 | tune_param | motor/ident/ident_flow.c:3,motor/ident/ident_module.c:3 |
| `M1_IDENT_STEP_I0_A` | 2 | 6 | tune_param | motor/ident/ident_module.c:5,motor/deadband/deadband_flow.c:1 |
| `M1_LD_LQ_IDENT_F_FINE_HZ` | 2 | 6 | tune_param | motor/ident/ld_lq_ident.c:5,motor/ident/ld_lq_ident.h:1 |
| `M1_LD_LQ_IQ_BIAS_N` | 2 | 6 | other | motor/ident/ld_lq_ident.c:3,motor/ident/ld_lq_ident.h:3 |
| `M1_IDENT_STEP_FIXED_ROUNDS` | 2 | 5 | other | motor/ident/ident_flow.c:3,config/profiles/m1_bode_id_fc1000.profile.h:2 |
| `M1_LD_LQ_IDENT_F_COARSE_HZ` | 2 | 4 | tune_param | motor/ident/ld_lq_ident.c:3,motor/ident/ld_lq_ident.h:1 |
| `M1_LD_LQ_ID_BIAS_N` | 2 | 4 | other | motor/ident/ld_lq_ident.h:3,motor/ident/ld_lq_ident.c:1 |
| `M1_OPEN_UD_AFTER_ID_CAL` | 2 | 4 | other | motor/deadband/deadband_flow.c:3,motor/deadband/deadband_id_cal.c:1 |
| `M1_IDENT_BODE_CYCLES_HI` | 2 | 3 | other | config/profiles/m1_bode_id_fc1000.profile.h:2,motor/ident/ident_module.c:1 |
| `M1_IDENT_BODE_CYCLES_PER_FREQ` | 2 | 3 | other | config/profiles/m1_bode_id_fc1000.profile.h:2,motor/ident/ident_module.c:1 |
| … | | | | 另有 59 项见 CSV |

明细 CSV：[`宏归属表_motor_params_m1_20261007.csv`](./宏归属表_motor_params_m1_20261007.csv)

生成命令：`python tools/scan_motor_params_macros.py`

---

红线：未记账前禁止批量删宏。`#error` 是资产，不是垃圾。
