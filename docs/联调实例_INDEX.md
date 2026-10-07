# 联调实例总索引

**维护规则**：每新增/签收一个实验，在本表加一行；细节只写各实例 RUNBOOK，本表不堆参数。

**架构说明**：[Bringup联调实例架构设计.md](Bringup联调实例架构设计.md)

---

## 状态图例

| 标记 | 含义 |
|------|------|
| ✅ | 已签收 |
| ⚠️ | 有条件签收 / bringup 可用未产品签收 |
| 📝 | RUNBOOK 已有，待签收 |
| 🔧 | 仅日报/碎片记录，待整理 |
| 🚧 | 设计中 |

---

## 电流环 / 参数辨识

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `bode_current_limit` | CURRENT | — | 多档 fc 汇总 | — | ✅ | **总参考**：[电流环 Bode 扫频与带宽上限](总结_电流环Bode扫频与带宽上限_2026-09-09.md)（\(f_s/5\)≈4 kHz；推荐 fc=1500） |
| `bode_iq_off` | CURRENT | BOOT_ONCE | `M1_BRINGUP_MODE_BODE_OFF_ONLY` | 🔧 | ✅ | **主档** `0042/0044/0045`（−3dB≈2128 Hz；相位−90°≈1.57 kHz）。早期 `2340` 等仅过程稿 |
| `bode_id_off` | CURRENT | BOOT_ONCE | `bringup_active.h` + `profiles/m1_bode_id_fc1000.profile.h` | 🔧 | ✅ | **主档** `2149/2152`（7/7）/`1726–28`（9/9 5kHz 表，−3dB≈2137 Hz）；见 [总总结](总结_电流环Bode扫频与带宽上限_2026-09-09.md) |
| `bode_id_fc800` | CURRENT | BOOT_ONCE | 同上，`M1_PI_FC_HZ=800`，69 点→5 kHz | 🔧 | ✅ | **主档** `2124/2125/2127`（−3dB≈1059 Hz）；见总总结 |
| `bode_id_fc1500` | CURRENT | BOOT_ONCE | 同上，`M1_PI_FC_HZ=1500`，69 点→5 kHz | 🔧 | ✅ | **主档** `1638/1643/1644`（−3dB≈3875 Hz，−90°≈2011 Hz）；见 [fc1500 报告](验收报告_BODE_ID_fc1500_5kHz_2026-09-09.md) |
| `bode_id_fc1600` | CURRENT | BOOT_ONCE | 同上，`M1_PI_FC_HZ=1600` | 🔧 | ✅ | **主档** `1705/1708/1709`（−3dB≈4057 Hz，Mp≈1.30，踩线）；见 [fc1600 报告](验收报告_BODE_ID_fc1600_5kHz_2026-09-09.md) |
| `ident_iq_step` | CURRENT | BOOT_RECIPE | `M1_BRINGUP_MODE_IDENT_IQ_STEP` | 🔧 | ✅ | IDENT 1604：0.5–2 A 有条件通过（单次长录波多轮） |
| `rs_ld_lq_only` | PARAM | BOOT_ONCE | `M1_BRINGUP_MODE_RS_LD_LQ_ONLY` | 🔧 | ✅ | `VOFA+CSV/20260708/` |
| `iq_probe_off` | CURRENT | MANUAL | `M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY` | — | 🔧 | — |
| `id_cal_pass0_build` | DEADBAND | BOOT_RECIPE | `M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD` | 🔧 | ⚠️ | Pass0 流水线 |

---

## 速度环

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `speed_ident_v12` | SPEED | BOOT_ONCE | `M1_BRINGUP_MODE_SPEED_IDENT` | [操作说明](联调实例_SPEED_IDENT_操作说明.md) | ⚠️ v1.2 | `120009`/`120011`（0.6 N·m） |
| `speed_profile_ladder` | SPEED | BOOT_ONCE | `NORMAL` + `M1_SPEED_PROFILE_ENABLE` | — | 🔧 | 7/3 阶梯 |

---

## 观测器旁路（有感对照，不进 Park）

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `obs_veq_bypass` | OBS | BOOT_ONCE | `SPEED_IDENT` + `M1_USE_OBS_VEQ_PROFILE` / `m1_obs_veq_1000rpm` | 🔧 | ⚠️ Veq+atan PASS | [总结_OBS_VEQ](总结_OBS_VEQ旁路观测_2026-09-10.md)（1325/1337） |
| `obs_smo_bypass` | OBS | BOOT_ONCE | `SPEED_IDENT` + `m1_speed_1000rpm`（Veq+SMO） | 🔧 | ❌ SMO+atan FAIL | 三包 [1956/2009/2012](分析报告_OBS_SMO_三包复测_1956_2009_2012_2026-09-10.md)；**改 PLL**：[决策总结](总结_OBS_SMO旁路与换PLL决策_2026-09-10.md) |
| `obs_emf_pll` | OBS | BOOT_ONCE | `SPEED_IDENT` + `m1_speed_1000rpm`（角+速软切+LPF前馈） | 🔧 | ⚠️ **中高速有条件签收** | **签收总结**：[总结_OBS_SMO中高速软切签收_2026-09-11](总结_OBS_SMO中高速软切签收_2026-09-11.md)；过程：[无感软切与LPF相位前馈](分析报告_无感软切与LPF相位前馈_2026-09-11.md)（1548）；探底 `1654`；旁路对照 `2048` |

---

## I/F → SMO 无感（中高速）

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `if_smo_mid_v1` | OBS/SPEED | BOOT_ONCE | `M1_USE_IF_100_PROFILE` → `m1_if_100rpm` | 🔧 | ⚠️ **中高速 V1 结档** | **结档**：[SMO V1 与 HFI 开干](结档_SMO中高速无感V1与HFI开干_2026-09-12.md)；半成功：[限权变速](分析报告_SMO无感限权变速半成功_2026-09-12.md)；DIR_SEQ 主档 **`2154`**；同向阶跃 `1640/1702/1714` |

---

## 位置环

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `pos_step_v1` | POSITION | RUNTIME_ARM* | `NORMAL` + `M1_DB_BRINGUP_SPEED_OFF` + `M1_POS_STEP_TEST_ENABLE` | 🚧 | ⚠️ | 7/5 三环日报 |
| `pos_mit_signoff_v1` | POSITION/MIT | BOOT_ONCE | `M1_USE_SENSED_POS_MIT_PROFILE=1` + `M1_OUTER_EXPT = M1_OUTER_EXPT_SIGNOFF` | [规划 §13](规划_位置与MIT实验签收_2026-10-06.md) / [签收_P3](签收_位置环与MIT_P3_2026-10-06.md) | ⚠️ **有条件签收**（P3；P-C 单帧 Iq 毛刺不改增益） | 金样 `VOFA+CSV/20261006/vofa+202610062336.csv`；hex `MDK-ARM/_pkg/20261006_P3_sat_signoff/` |
| `sensorless_pos_mit_smoke` | POSITION/MIT | BOOT_ONCE | `M1_USE_SENSORLESS_POS_MIT_PROFILE=1`（`bringup_active.h`） | [总结 §无感位控冒烟](总结_代码现状与问题分析_2026-10-07.md) | 🔧 **冒烟**（关超速守卫档，非产品签收） | 过程包见 `_pkg/20261006_*sensorless*`；日常回归用有感 `regress_sensed_short` |

\* 现状上电自动 arm；目标改为独立 profile `M1_INSTANCE_POS_STEP`（见架构设计 §10）。

---

## 产品 / 日常

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 备注 |
|---------|-----|------|------------------|---------|------|------|
| `normal_product` | PRODUCT | MANUAL | `M1_BRINGUP_MODE_NORMAL` | 🚧 | 🚧 | ESO/阻抗主战场 |
| `normal_db_off` | SPEED | MANUAL | `NORMAL` + deadband OFF | — | 🔧 | 速度环死区 A/B |

---

## 规划中

| 实例 ID | 域 | 依赖基线 | 说明 |
|---------|-----|----------|------|
| `hfi_low_speed` | OBS/STARTUP | `m1_hfi_standstill` | `M1_HFI_GATE`：1…8；捕获对照见 [9/27 报告](报告_HFI无感捕获现状与对照实验计划_2026-09-27.md)。下一枪 C0=GATE=3。[规划](规划_HFI四步重验收与后续计划_2026-09-26.md) |
| `if_smo_full_pi` | SPEED | `if_smo_mid_v1` | SMO 回填：`open_seq=248` 满权威+正常 PI（不挡 HFI） |
| `eso_disturb_reject` | SPEED | `normal_product` | ESO 阶跃 / 0.6 N·m 对比 |
| `impedance_joint_z` | POSITION | `normal_product` | 关节阻抗 Z 扫频/阶跃 |

---

## 快速切换

**只改** `config/bringup_active.h`（选 `M1_BRINGUP_MODE`）。

| 实验 | 模式 | 附加 |
|------|------|------|
| **HFI 四步（编号见规划）** | `SPEED_IDENT` | **GATE=6=S3c1**（对照 Park=enc 爬）；回退 `5/4/3` |
| **I/F→SMO 中高速 V1** | `NORMAL`/`SPEED_IDENT` 以 bringup 为准 | `M1_USE_IF_100_PROFILE=1` → `m1_if_100rpm`（DIR_SEQ±1000；结档见上） |
| Id Bode 签收 | `BODE_ID_OFF_ONLY` | `profiles/m1_bode_id_fc1000.profile.h` |
| 有感 1000 rpm 估 ψf | `SPEED_IDENT` | `M1_USE_FLUX_ID_PROFILE=1` → `m1_flux_id_1000rpm.profile.h` |
| 有感 1000 rpm + Veq 旁路 | `SPEED_IDENT` | `M1_USE_OBS_VEQ_PROFILE=1` → `m1_obs_veq_1000rpm.profile.h`（FLUX 须 0） |
| Veq+SMO 旁路阶梯（cycle） | `SPEED_IDENT` | `M1_USE_SPEED_1000_PROFILE=1` → `m1_speed_1000rpm.profile.h`（与 OBS_VEQ/FLUX/IF 互斥） |
| 日常 | `NORMAL` | FLUX/OBS/SPEED1000/IF 均 0 |

```c
/* bringup_active.h 示例：Veq 旁路 */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT
#define M1_USE_FLUX_ID_PROFILE  0
#define M1_USE_OBS_VEQ_PROFILE  1
```

VOFA OBS_VEQ×12（D=2→10 kHz）：iα iβ uα uβ eα eβ θ̂ θenc θerr \|e\| ωe ψinst。  
0910 Veq 总结：[总结_OBS_VEQ旁路观测_2026-09-10](总结_OBS_VEQ旁路观测_2026-09-10.md)。  
SMO 三包后 **改 EMF-PLL**：[总结_OBS_SMO旁路与换PLL决策_2026-09-10](总结_OBS_SMO旁路与换PLL决策_2026-09-10.md)。

---

## Keil Target（规划）

| Target 名 | 预定义宏 |
|-----------|----------|
| `MOTOR_NORMAL` | `M1_BRINGUP_MODE=0` |
| `MOTOR_SPEED_IDENT` | `M1_BRINGUP_MODE=8` |
| `MOTOR_BODE_IQ` | `M1_BRINGUP_MODE=9` |
| `MOTOR_POS_STEP` | `M1_INSTANCE_POS_STEP=1`（未来） |

---

**最后更新**：2026-10-07（补无感位控冒烟一行；架构接缝包 2–4 / 台架开关迁 `bringup_bench.h`）
