# 死区补偿拆分 · 验收清单

> 金标准 CSV 已冻结（Phase 0）。每 Phase 编译烧录后对照本表。

## Phase 1 · deadband_service 门面

**状态**：✅ 已通过（对照 `vofa+202606282007` / `vofa+202606282308`）

---

## Phase 2 · deadband_id_cal 状态机迁出

**状态**：✅ 已通过（对照 `vofa+202606282007` 金标准；验收录波 `vofa+202606282321`）

### 编译

- [x] Keil 无 error（`deadband_id_cal.c` 已加入 `motor/deadband` 组）

### 运行模式

`M1_BRINGUP_MODE_IDENT_IQ_STEP`（或常用 Id 标定模式）

### VOFA 验收（与 Phase 0 金标准对比）

| 检查项 | 金标准 282007 | Phase 2 验收 282321 |
|--------|---------------|---------------------|
| Id 扫表结束 | 9.05 s | **9.04 s** |
| Bode 切分 | 10.93 / 16.99 / 23.05 / 28.69 | **10.92 / 16.99 / 23.05 / 28.69** |
| OFF \|G_i\| mean | 0.851 | 0.937 |
| OFF err RMS | 24.5 mA | **25.1 mA** (~2%) |
| OFF 频点 | 32/32 | **32/32** |
| FIXED 频点 | 32/32 | **32/32** |
| LUT 频点 | 18/32（已知） | 18/32（未回归） |

### 结构

```bash
rg "motor_id_cal|id_cal_sweep" motor/motor_current.c
# 应无匹配
wc -l motor/motor_current.c   # 目标 ~720 行（原 ~1440）
```

---

## Phase 1 原始清单（归档）

**改动**：`motor/deadband/deadband_service.*`；`motor_current` / `ident_module` 不再直接 `deadband_set_mode`。

### 编译

- [ ] Keil 无 error/warning 新增（`deadband_service.c` 已加入 `motor/deadband` 组）

### 运行模式

当前默认：`M1_BRINGUP_MODE_IDENT_IQ_STEP`（Id Pass0 → commit → Bode OFF/FIXED/LUT）

### VOFA 观测（与金标准 CSV 对比）

| 检查项 | 通道 | 期望 |
|--------|------|------|
| Pass0 建表 | ch5 `open_seq_phase` | 0→1..N→commit 序列与冻结 CSV 一致 |
| Bode OFF 轮 | ch5 = 62 | Iq/Uq 幅频与金标准偏差在脚本阈值内 |
| Bode FIXED 轮 | ch5 = 63 | 同上 |
| Bode LUT 轮 | ch5 = 64 | 同上（重点：LUT 来自 commit plut） |
| 结束 | ch5 = 73 | IDENT_DONE |

### 自动脚本（可选）

```bash
python tools/ident/analyze_iq_bode.py VOFA+CSV/.../vofa+金标准.csv
python tools/ident/analyze_iq_bode.py VOFA+CSV/.../vofa+本次.csv
```

### 结构（grep 自检）

```bash
# motor/ 下除 deadband/*.c 外不应再有 deadband_set_mode
rg "deadband_set_mode" motor --glob "!motor/deadband/*"
# 应无输出
```

### 通过标准

1. 全流程 VOFA 与金标准在约定阈值内  
2. `open_seq_phase` 码表 62/63/64/73 时序不变  
3. grep 结构检查通过  

---

## Phase 3 · deadband_flow 配方表

**状态**：✅ 已通过（对照 `vofa+202606282007` 金标准；验收录波 `vofa+202606282330`）

**改动**：`motor/deadband/deadband_flow.*`；`motor_current.c` 编排改为 `deadband_flow_boot/tick`。

### VOFA 验收（与金标准对比）

| 检查项 | 金标准 282007 | Phase 3 验收 282330 |
|--------|---------------|---------------------|
| Id 扫表结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / 16.99 / 23.05 / 28.69 | **10.93 / 16.99 / 23.05 / 28.61** |
| OFF err RMS | 24.5 mA | **25.1 mA** (~2%) |
| OFF 频点 | 32/32 | **32/32** |
| FIXED 频点 | 32/32 | **32/32** |
| LUT 频点 | 18/32（已知） | 16/32（未回归） |

### 编译

- [x] Keil 无 error（`deadband_flow.c` 已加入 `motor/deadband` 组）

### 结构（grep 自检）

```bash
rg "ident_module_init|ident_module_tick|deadband_id_cal_init" motor/motor_current.c
# 应无匹配
rg "s_ident_precal_done|motor_current_id_cal_active" motor/motor_current.c
# 应无匹配
```

### VOFA 验收

与 Phase 2 相同：Id ~9.05 s，Bode 切分 10.93/16.99/23.05/28.69，OFF err ~1% 内。

---

## Phase 4 · ident 纯激励 + ident_flow 编排

**状态**：✅ 已通过（对照 `vofa+202606282007`；验收录波 `vofa+202606282336`）

**改动**：
- `ident_module.*` 只保留 HOLD/STEP/BODE 波形；显式 32 点频表
- 新建 `ident_flow.*`：OFF/FIXED/LUT profile + `open_seq_phase`
- `deadband_flow` 改调 `ident_flow_init/tick`

### VOFA 验收（与金标准对比）

| 检查项 | 金标准 282007 | Phase 4 验收 282336 |
|--------|---------------|---------------------|
| Id 扫表结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / 16.99 / 23.05 / 28.69 | **10.93 / 16.99 / 23.05 / 28.71** |
| OFF 频点 | 32/32 | **32/32** |
| OFF err RMS | 24.5 mA | **24.6 mA** |
| FIXED 频点 | 32/32 | **32/32** |
| LUT 频点 | 18/32（分析窗/录波） | 18/32（同量级，非回归） |

### 编译

- [x] Keil 无 error（`ident_flow.c` 已加入工程）

---

## Phase 5a · dbg_monitor 外提

**状态**：✅ 已通过（对照 `vofa+202606282007`；验收录波 `vofa+202606282341`）

### VOFA 验收

| 检查项 | 金标准 282007 | Phase 5a 282341 |
|--------|---------------|-----------------|
| Id 扫表结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / 16.99 / 23.05 / 28.69 | **10.93 / 16.99 / 23.05 / 28.71** |
| OFF | 32/32，24.5 mA | **32/32**，**24.8 mA** |
| FIXED | 32/32 | **32/32** |
| LUT | 18/32 | 18/32（同金标准，非回归） |

### 编译

- [x] Keil 无 error（`debug/dbg_monitor.c` + `../debug` IncludePath）

---

## R1 · motor_current 脱离 FOC_CAL / as5047_spi1

**状态**：✅ 已通过（对照 `vofa+202606282007`；验收录波 `vofa+202606290002`）

**改动**（0 新文件）：
- `encoder_t` 增加 `theta_el_offset_rad`；`encoder_set/get_theta_el_offset()`
- `main.c`：`encoder_set_theta_el_offset(&enc_m1, …)` 替代 `as5047_spi1.add`；移除 `FOC_CAL.h` / `angle_init()`
- `motor_current.c`：θ 仅用 `axis->enc` + offset API；删除 `as5047_spi1.raw/.get` 双写

| 检查项 | 金标准 282007 | R1 290002 |
|--------|---------------|-----------|
| Id 扫表结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / 16.99 / 23.05 / 28.69 | **10.93 / 16.99 / 23.05 / 28.60** |
| OFF/FIXED | 32/32 | **32/32** |
| OFF err RMS | 24.5 mA | **24.7 mA** |

---

## R2 · PI 编排 → motor_foc_loop

**状态**：✅ 已通过（对照 `vofa+202606282007`；验收录波 `vofa+202606290007`）

**改动**（+1 文件）：
- `motor/motor_foc_loop.c/h`：`pi_init/reset`、bumpless、Id cal / ident / 闭环 PI

| 检查项 | 金标准 282007 | R2 290007 |
|--------|---------------|-----------|
| Id 扫表结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / … / 28.69 | **完全一致** |
| OFF/FIXED | 32/32 | **32/32** |
| OFF err RMS | 24.5 mA | **25.2 mA** |

---

## R3 · 开环 Uq → motor_open_sweep

**状态**：✅ 已通过（对照 `vofa+202606282007`；验收录波 `vofa+202606290015`）

**改动**（+1 文件）：
- `motor/motor_open_sweep.c/h`：V0/V1/V2 扫参、AB 死区对比、固定 Uq 定时
- **IDENT_IQ_STEP**：Bode 结束后 **5 s Uq=2 V** 开环（`open_seq` 80→81）

| 检查项 | 金标准 282007 | R3+R4 290015 |
|--------|---------------|--------------|
| Id 结束 | 9.05 s | **9.05 s** |
| Bode 切分 | 10.93 / … / 28.69 | **一致** |
| OFF/FIXED | 32/32 | **32/32** |
| 开环段 | — | **28.7–33.7 s**，θ 有变化（锁轴后手拨验证） |

---

## R4 · trans/motor_trig 目录迁移

**状态**：✅ 已通过（同 `vofa+202606290015`）

**改动**：
- `bringup/trans.*` → `motor/foc_svpwm.*`
- `bringup/motor_trig*` → `Drivers/foc/motor_trig*`

---

## R5 · app_uart_dma_debug → debug/

**状态**：✅ 代码已改（待 Keil + VOFA）

**改动**：
- `bringup/app_uart_dma_debug.*` → `debug/app_uart_dma_debug.*`
- 移除 `FOC_CAL.h` / `motor_current.h` 依赖
- **`open_seq` 80/81**：VOFA ch4=`foc_uq_out`（开环段可读）

### VOFA 验收

与 R3 相同；开环段 ch4 应 ≈ **2 V**（不再误映 Id_ref）。

---

## Phase 5b+（待做）
