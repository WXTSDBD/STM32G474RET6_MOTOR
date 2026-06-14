# ADC 采样与 config 层部署计划

> **日期：** 2026-06-09  
> **状态：** 已评审（架构对齐 encoder 五层；Step 1.1 待落地）  
> **平台：** STM32G474RET6 @ 160 MHz，Keil AC6，FreeRTOS + VOFA  
> **关联：** [FOC后续实施计划.md](./FOC后续实施计划.md)、[电流环部署计划_Step1.md](./电流环部署计划_Step1.md)、[SPI编码器架构实现说明.md](./SPI编码器架构实现说明.md)

---

## 一、文档目的

本文说明 **ADC 电流采样模块**（`adc_sample`）与 **config 层**（`bsp_axes` + `bridge_cubemx`）的 **如何部署**：目录结构、文件清单、init 顺序、Keil 配置、验收标准，以及后续 CANopen / OTA / OLED 等域 **如何只增不改地接入**。

**本阶段范围（Step 1.1～1.2）：**

- 零电流偏置标定（discard 20 + **递推增量均值** 100 帧，方案 B：仅 TIM8 Base + CH4）
- ADC2 三 rank 注入采样（`T8_CC4` 触发）
- **不**做 OPAMP/ADC1/3/5 路径
- **不**删 `FOC_CAL`（并存过渡，后续绞杀删除）
- **不**上 YAML 设备树 / 完整 BSP devtree

---

## 二、架构原则（部署时必须遵守）

### 2.1 三层分工

```text
bringup/adc_sample/     模块 schema + 标定/读数逻辑（不含 M1/M2、不含 &hadc2）
config/bsp_axes.*       轴级 binding 表 + init 顺序（统一管理，非大总管 struct）
config/bridge_cubemx.*  唯一填写 CubeMX HAL 句柄（&hadc2、&htim8 等）
board/board_encoder_m1  encoder 硬件绑定不变；config 只引用 &enc_m1
Core/Src/main.c         薄集成：MX_*_Init → bsp_init → cal → PWM → RTOS
```

### 2.2 四条铁律（避免后续重构）

| # | 规则 |
|---|------|
| 1 | 驱动层（`adc_sample.c`）**不出现** `hadc2`、`htim8` 等 CubeMX 符号 |
| 2 | **全工程仅** `bridge_cubemx.c` 填写 HAL 指针（encoder board 除外） |
| 3 | **offset 写入 `adc_cfg.ch[i].offset`**（标定结果）；`adc_sample_t` 存 raw/ia/ib/ic 与标定过程状态 |
| 4 | CANopen / OTA / OLED **不得**并入 `bsp_axis_t` 或 `adc_sample_config_t` |

### 2.3 与 encoder 对齐

| encoder（已有） | ADC（本计划） |
|----------------|---------------|
| `encoder_spi_bus_t` schema | `adc_sample_config_t` schema |
| `board_encoder_m1.c` binding | `bsp_axes.c` + `bridge_cubemx.c` |
| `encoder_kick(&enc_m1)` | `adc_sample_on_injected(&bsp_axis(BSP_AXIS_M1)->adc, hadc)` |

---

## 三、硬件与 CubeMX 前提（M1 当前）

| 项目 | 配置 |
|------|------|
| 电流 ADC | **ADC2** 注入 3 rank：`IN9`(PC3) / `IN1`(PA0) / `IN5`(PC4) → JDR1/2/3 |
| 采样触发 | **TIM8 CH4** 上升沿 `T8_CC4` |
| 电机 PWM | **TIM8** CH1/2/3 + 互补 N，20 kHz 中心对齐 |
| 编码器 | **SPI1** / `enc_m1`（本计划不改动） |
| HAL 芯片校准 | `HAL_ADCEx_Calibration_Start(&hadc2)`（已有，与零偏标定不同） |

**方案 B 标定：** 标定阶段只开 TIM8 **Base + CH4**，**不开** CH1～3 PWM，**不开** `Base_Start_IT`（避免 `setPhaseVoltage` / encoder kick 干扰零偏）。

### 3.1 零偏标定算法（递推增量均值）

与「先累加 `cal_sum` 再除以 N」的批处理平均 **数学等价**，实现上在 JEOC 里 **来一帧更新一帧**，标定结束 `offset` 已收敛，无需最后再除。

**参数（默认）：**

| 参数 | 值 | 含义 |
|------|-----|------|
| `discard` | 20 | 丢弃 TIM 刚启动的前 20 帧（不进均值） |
| `samples` | 100 | 参与递推的有效帧数 |
| `timeout_ms` | 50 | 等待 `discard + samples` 帧的超时 |

**每帧 JEOC（标定态 `cal_active=1`）：**

```text
cal_done++                                    /* 总帧计数 */
若 cal_done <= discard：return                /* 去掉不稳定数据 */
k = cal_done - discard                        /* 有效帧序号 1..samples */
对每相 i：
  offset[i] += (raw[i] - offset[i]) / k       /* int32 递推增量均值 */
```

**标定前后：**

- 标定开始：`offset[i] = 0`（或先赋首帧 raw，实现可选；当前从 0 递推）
- 标定结束：`cal_done >= discard + samples` 时 **停止 TIM8 Base+CH4**，`offset` 即为零电流偏置
- 运行时：`adc_sample_update` 做 `(raw - offset) * scale`

**不用 EMA（`offset += α*(raw-offset)`）：** 一次性上电标定要用 **有限 N 帧的均值**，不用常驻低通。

**与 HAL 校准区别：** `HAL_ADCEx_Calibration_Start` 修正 ADC 内部；本节修正 **零电流 shunt 偏置点**（你 Watch 里 ~2058 那一档）。

---

## 四、新建文件清单

### 4.1 模块层

| 文件 | 职责 |
|------|------|
| `bringup/adc_sample/adc_sample.h` | `adc_sample_config_t`、`adc_sample_t`、API 声明 |
| `bringup/adc_sample/adc_sample.c` | 读 JDR、SCAN 收帧、方案 B 标定、`update`→安培 |

**对外 API（定版，后续只增不改）：**

```c
void adc_sample_init(adc_sample_t *s, const adc_sample_config_t *cfg);
bool adc_sample_calibrate_offset(adc_sample_t *s, uint16_t discard, uint16_t n, uint32_t timeout_ms);
void adc_sample_on_injected(adc_sample_t *s, ADC_HandleTypeDef *hadc);
void adc_sample_update(adc_sample_t *s);
void adc_sample_get_abc(const adc_sample_t *s, float *ia, float *ib, float *ic);
```

### 4.2 config 层

| 文件 | 职责 |
|------|------|
| `config/bsp_axes.h` | `bsp_axis_id_t`、`bsp_axis_t`、`bsp_init()`、`bsp_axis()`、标定入口 |
| `config/bsp_axes.c` | M1 binding 实装，M2 `enabled=false` 占位 |
| `config/bridge_cubemx.h` | bridge 声明（可选） |
| `config/bridge_cubemx.c` | `&hadc2`、`&htim8` 等指针填入 config |

**M1 静态 binding（写在 `bsp_axes.c`，无 HAL 指针）：**

```text
topo:     ADC_SAMPLE_TOPO_SCAN
channels: JDR1 / JDR2 / JDR3
trigger:  TIM8 CH4
pwm_tim:  TIM8（标定时禁止开 CH1～3）
scale:    1.0f（Step 1.2 再填真实 A/LSB）
enc:      &enc_m1（extern，init 仍调 board_encoder_m1_init）
```

**`bsp_axis_t` 建议字段（留扩展，防重构）：**

```c
typedef struct {
    bool enabled;
    adc_sample_t adc;
    encoder_t *enc;
    TIM_HandleTypeDef *pwm_tim;
    void *motor_ctx;   /* 以后 motor_context_t*，Step 1 置 NULL */
} bsp_axis_t;
```

---

## 五、修改现有文件

### 5.1 `Core/Src/main.c`

**`USER CODE BEGIN 2` init 顺序：**

```text
1. （可选）HAL_OPAMP_Start — Step 1.1 可忽略 OPAMP
2. HAL_ADCEx_Calibration_Start(&hadc2)      ← 芯片 ADC 校准（已有）
3. HAL_ADCEx_InjectedStart_IT(&hadc2)
4. __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC)
5. board_encoder_m1_init()                 ← 或并入 bsp_init 内调用
6. bsp_init()                                ← bridge + adc_sample_init
7. bsp_axis_adc_calibrate_zero(BSP_AXIS_M1)  ← 方案 B，阻塞 ~6 ms
8. telem / FDCAN 等（与现序一致）
9. HAL_TIM_PWM_Start(htim8, CH1～3 + N)    ← 标定之后
10. HAL_TIM_PWM_Start(htim8, CH4)
11. HAL_TIM_Base_Start_IT(&htim8)
12. osKernelStart()
```

**`HAL_ADCEx_InjectedConvCpltCallback`（hadc2 分支）：**

```c
adc_sample_on_injected(&bsp_axis(BSP_AXIS_M1)->adc, hadc);
/* 过渡期：继续写 adc_read[3..5]、dbg.adc_reg[] 供 VOFA */
```

**注意：** 标定函数执行期间 **不得** 已启动 `HAL_TIM_Base_Start_IT(&htim8)`。

### 5.2 Keil `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`

| 操作 | 内容 |
|------|------|
| 添加源文件 | `bringup/adc_sample/adc_sample.c`、`config/bsp_axes.c`、`config/bridge_cubemx.c` |
| Include Path | `../bringup/adc_sample`、`../config`（按工程相对路径调整） |
| 编译 | Rebuild，确认无 `hadc2` 重复定义或链接错误 |

---

## 六、部署步骤（推荐顺序）

### Step A — 骨架与编译

1. 创建 `bringup/adc_sample/`、`config/` 目录及上述 4～6 个文件  
2. 实现空壳 API + M1 binding 表；**Step A 即把 `bridge_cubemx.c` 中 M1 的 `&hadc2`、`&htim8` 填齐**（即使尚未跑标定，避免 A→B 中断后遗漏）  
3. 加入 Keil 工程，**先不调用标定**，确认编译链接通过  

### Step B — 接入 ISR（无标定）

1. main 中 `bsp_init()`，JEOC 转发 `adc_sample_on_injected`  
2. 开 TIM8（含 CH4 + Base_IT），VOFA 看 `dbg.adc_reg[0..2]` 是否 20 kHz 更新  
3. 确认与 encoder 开环 **共存**（`isr_delta` 仍健康）  

### Step C — 零偏标定（Step 1.1）

1. 在 **PWM CH1～3 与 `Base_IT` 启动前** 调用 `bsp_axis_adc_calibrate_zero(M1, 20, 100, 50)`  
2. 标定期间仅 TIM8 **Base + CH4**；电机 **可不转**  
3. 算法见 §3.1（discard 20 + 递推 100 帧）；标定结束 `offset[i]` ≈ 空载 raw（~2048 一带）  
4. 验收：`raw[i] - offset[i]` 接近 0（±几十 LSB）；再开 PWM / VOFA 验证  

### Step D — 标度（Step 1.2）

1. 在 binding 或后续 `motor_params_default.h` 填 `scale`（A/LSB）  
2. 调用 `adc_sample_update`，验收 Ia+Ib+Ic ≈ 0（空载）  

### Step E — 文档与遗留

1. 联调日报补充标定前后 `adc_reg`、offset 数值  
2. `FOC_CAL` 暂保留；遥测可逐步改绑 `adc_sample_get_abc`  

---

## 七、验收标准

### Step 1.1（零偏）

| 项 | 标准 |
|----|------|
| 标定返回值 | `bsp_axis_adc_calibrate_zero(M1)` == true，< 50 ms |
| offset | 三路为稳定常数，彼此接近（视硬件偏置） |
| 空载 | PWM 开、无电流指令时，`raw[i] - offset[i]` 在 ±几十 LSB 内 |
| 架构 | `adc_sample.c` 无 `hadc2`；bridge 外无第二处 HAL 绑定点 |
| 性能 | TIM8 UP `isr_delta` 相对合并前无异常劣化 |

### Step 1.2（标度）

| 项 | 标准 |
|----|------|
| 三相和 | Ia + Ib + Ic ≈ 0（空载，误差 < 5% 量程） |
| VOFA | 可观察 Ia/Ib/Ic（float） |

---

## 八、后续扩展（只增不改）

### 8.1 本仓库内

| 需求 | 部署方式 | 是否改 adc_sample API |
|------|----------|------------------------|
| 启用 M2 | `bsp_axes.c` 填 `s_axes[M2]` + bridge 加 TIM1/hadc | 否 |
| 电流环 | 新增 `motor/motor_current.c`，ADC2 回调 `get_abc` | 否 |
| 删 FOC_CAL | main/遥测去 `#include FOC_CAL.h`，删文件 | 否 |
| motor 参数默认 | 新增 `motor/motor_params_default.h` | 否 |
| NVS 存 offset | 新增 `adc_sample_load/save_offset` | **增** API |

### 8.2 其它域（并列 config，不并入 axes）

```text
config/comm_canopen_m1.h    → comm_canopen_init()    bridge 填 &hfdcan1
config/ota_layout_m1.h      → ota_init()             与 comm 互不 include
config/ui_oled_m1.h         → ui_oled_init()         bridge 填 &hi2c1
app_init.c（可选）          聚合各域 init 顺序
```

**统一模板：** `drivers/xxx`（schema）+ `config/xxx_m1.h`（binding）+ `bridge`（HAL 指针）+ `xxx_init()`。

---

## 九、风险与对策

| 风险 | 对策 |
|------|------|
| 标定时开了 motor PWM | 严格 init 顺序；标定只用 Base+CH4 |
| offset 写在 const 表 | 写入 `adc_cfg.ch[i].offset`（`bsp_axis` 内运行时配置） |
| `bsp_axes` 膨胀塞 CAN/OLED | 禁止；各域独立 config 文件 |
| CubeMX Generate 改 handle 名 | 只改 `bridge_cubemx.c` |
| PER_PHASE（ADC1/3/5） | schema 预留 `topo`；M1 用 SCAN |

---

## 十、与 FOC 路线图关系

```text
本文（Step 1.1～1.2）──→ 电流环 Step 1.3～1.6 ──→ Step 2 参数 / PI
         │                                              │
         └─ adc_sample 定版 API ─────────────────────────┘
```

详见 [FOC后续实施计划.md](./FOC后续实施计划.md) §三 Step 1。

---

## 十一、Checklist（部署完成打勾）

- [ ] `bringup/adc_sample/adc_sample.c/h` 已加入 Keil  
- [ ] `config/bsp_axes.c/h`、`config/bridge_cubemx.c` 已加入 Keil  
- [ ] Include Path 已配置  
- [ ] main init 顺序：InjectedStart → bsp_init → **cal** → PWM → Base_IT  
- [ ] JEOC 已转发 `adc_sample_on_injected`  
- [ ] 标定 success，offset 已记录  
- [ ] VOFA `adc_reg` / raw−offset 已截图存档  
- [ ] 联调日报已更新  

---

**修订记录**

| 日期 | 说明 |
|------|------|
| 2026-06-09 | 初版：ADC 采样 + config/bridge 部署计划（架构评审通过） |
| 2026-06-09 | 零偏标定改为 discard + **递推增量均值**（§3.1），替代 cal_sum 批处理 |
