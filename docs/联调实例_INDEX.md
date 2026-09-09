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

## 位置环

| 实例 ID | 域 | 触发 | 配置入口（现状） | RUNBOOK | 验收 | 主录波 |
|---------|-----|------|------------------|---------|------|--------|
| `pos_step_v1` | POSITION | RUNTIME_ARM* | `NORMAL` + `M1_DB_BRINGUP_SPEED_OFF` + `M1_POS_STEP_TEST_ENABLE` | 🚧 | ⚠️ | 7/5 三环日报 |

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
| `eso_disturb_reject` | SPEED | `normal_product` | ESO 阶跃 / 0.6 N·m 对比 |
| `impedance_joint_z` | POSITION | `normal_product` | 关节阻抗 Z 扫频/阶跃 |

---

## 快速切换

**只改** `config/bringup_active.h`（选 `M1_BRINGUP_MODE`）。

Id Bode 签收配方（fc=1000、69 点→5 kHz）在：

`config/profiles/m1_bode_id_fc1000.profile.h`

```c
/* bringup_active.h 示例 */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_NORMAL
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_ID_OFF_ONLY  /* Id Bode */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_OFF_ONLY     /* Iq Bode */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT
```

---

## Keil Target（规划）

| Target 名 | 预定义宏 |
|-----------|----------|
| `MOTOR_NORMAL` | `M1_BRINGUP_MODE=0` |
| `MOTOR_SPEED_IDENT` | `M1_BRINGUP_MODE=8` |
| `MOTOR_BODE_IQ` | `M1_BRINGUP_MODE=9` |
| `MOTOR_POS_STEP` | `M1_INSTANCE_POS_STEP=1`（未来） |

---

**最后更新**：2026-09-09
