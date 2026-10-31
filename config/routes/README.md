# config/routes — S4b route 数据表

本目录是「宏 → 实验表」阶段里 **8 份 profile → 8 条 route** 的落点
（另加 id=9 的 PASS0_RS 标定档）。每条 `r_*.c` 打包
`plant + gains + ops 指针`；数值从对应 `config/profiles/*.profile.h`
与 `motor_params_m1.h` 默认展开抄出，不许臆造。

## 状态（2026-10-28）

- **已全部进 MDK 工程**：`r_*.c`（id 1..9）均在 `uvprojx`；现网
  `M1_ACTIVE_ROUTE_ID=9` 只引用 `g_route_id_cal_pass0_rs`，其余由链接器剔除。
- **选路**：`bringup_active.h` 设 `M1_ACTIVE_ROUTE_ID` + `BRINGUP_MODE`；
  `M1_USE_*_PROFILE` 由 `config/route_use_from_id.h` 按 id 推导；
  `motor_route_for_boot()` 编译期互斥选表。
- **ops**：除角度外，`outer/ff/iref/telem` 大多仍为 `NULL`（表尚未落地）。
- **PI**：可读 `ctx->gains` / `ctx->plant`（bind 注入）。

## profile ↔ route

| route_id | profile | route 文件 | 符号 | 角度 | outer_mode |
|---:|---|---|---|---|---|
| 1 | `m1_hfi_standstill.profile.h` | `r_hfi_standstill.c` | `g_route_hfi_standstill` | `g_angle_hfi` | SPEED |
| 2 | `m1_sensed_pos_mit.profile.h` | `r_sensed_pos_mit.c` | `g_route_sensed_pos_mit` | `g_angle_enc` | MIT |
| 3 | `m1_sensorless_pos_mit.profile.h` | `r_sensorless_pos_mit.c` | `g_route_sensorless_pos_mit` | `g_angle_hfi` | MIT |
| 4 | `m1_if_100rpm.profile.h` | `r_if_100rpm.c` | `g_route_if_100rpm` | `NULL`（无 I-f ops） | SPEED |
| 5 | `m1_speed_1000rpm.profile.h` | `r_speed_1000rpm.c` | `g_route_speed_1000rpm` | `g_angle_enc` | SPEED |
| 6 | `m1_obs_veq_1000rpm.profile.h` | `r_obs_veq_1000rpm.c` | `g_route_obs_veq_1000rpm` | `g_angle_enc` | SPEED |
| 7 | `m1_flux_id_1000rpm.profile.h` | `r_flux_id_1000rpm.c` | `g_route_flux_id_1000rpm` | `g_angle_enc` | SPEED |
| 8 | `m1_bode_id_fc1000.profile.h` | `r_bode_id_fc1000.c` | `g_route_bode_id_fc1000` | `NULL` | DISABLED |
| 9 | （bringup `ID_CAL_PASS0_RS`） | `r_id_cal_pass0_rs.c` | `g_route_id_cal_pass0_rs` | `g_angle_hfi` | DISABLED |

共用铭牌：`config/plant/m1_n5065.c`（`g_plant_m1_n5065`），已进 MDK；
各 `r_*.c` 一律 `&g_plant_m1_n5065`，不再本地 static 副本。

## 共用 plant（全 profile 一致）

| 字段 | 值 | 宏来源 |
|---|---:|---|
| rs_ohm | 0.122 | `M1_RS_OHM` |
| ld_h | 59e-6 | `M1_LD_H` |
| lq_h | 87e-6 | `M1_LQ_H` |
| kt_nm_a | 0.063 | `M1_MIT_KT_NM_A` |
| psi / J / B | 0 | 现网无宏 |
| pole_pairs | 7 | `M1_POLE_PAIRS` |
| vbus_v | 24.0 | `M1_VBUS_V` |
| deadtime_s | 5.91e-7 | `M1_DEADTIME_NS`=591 |

## 电流环 PI（除 Bode 限幅外各档相同）

由 `M1_PI_FC_HZ=1000`、`Ld/Lq/Rs`、`M1_CTRL_TS_S=50e-6` 展开：

| 项 | 值 |
|---|---:|
| iq.kp | 0.54663712（`Lq·ωc`） |
| id.kp | 0.37070793（`Ld·ωc`） |
| iq/id.ki | 0.03832743（`Rs·ωc·Ts`） |
| iq/id.fc_hz | 1000 |
| Vlim（有速度环） | 13.856406（`Vbus·√3/3`） |
| Vlim（Bode/CLOSURE） | 6.0 |

## 各档 gains 差异（速度/位置/斜坡/限流）

| route | spd.kp | spd.ki | spd.i_lim | pos.kp | ramp / decel | iq/id.i_lim |
|---|---:|---:|---:|---:|---:|---:|
| hfi_standstill | 0.005 | 6.5e-6 | 11 | 80（默认） | 500 / 800 | 11 |
| sensed_pos_mit | 0.030 | 0.004 | 5 | 600 | 500 / 500 | 5 |
| sensorless_pos_mit | 0.005 | 6.5e-6 | 5 | 20 | 0 / 0 | 5 |
| if_100rpm | 0.0008 | 5e-5 | 3.5 | 80（默认） | 200 / 200 | 11 |
| speed_1000rpm | 0.015 | 0.002 | 11 | 80（默认） | 500 / 500 | 11 |
| obs_veq_1000rpm | 0.015 | 0.002 | 11 | 80（默认） | 500 / 500 | 11 |
| flux_id_1000rpm | 0.015 | 0.002 | 11 | 80（默认） | 500 / 500 | 11 |
| bode_id_fc1000 | 0 | 0 | 0 | 0 | 0 / 0 | 0.5 |

未单独定义 `M1_SPEED_OMEGA_RAMP_DECEL_RPM_S` 的档，decel 与 accel 同值
（与外环「无 route 时 decel=accel」回退一致）。仅 HFI 档为 800。
