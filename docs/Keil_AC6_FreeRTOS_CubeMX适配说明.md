# Keil AC6 + FreeRTOS + CubeMX 适配说明

本文档说明本工程为何需要 `tools/` 下的后处理脚本，以及日常开发与换工程时的固定流程。**请与 `tools/README.md` 一并提交 git。**

---

## 背景：为什么 CubeMX 生成的工程在 Keil 里编不过

| 项目 | CubeMX 默认（MDK-ARM V5.32） | 本工程实际使用 |
|------|------------------------------|----------------|
| 编译器 | 面向 AC5 习惯 | **Keil AC6**（ArmClang V6.x） |
| FreeRTOS 端口 | `portable/RVDS/ARM_CM4F` | 必须 **`portable/GCC/ARM_CM4F`** |
| `portmacro.h` | 使用 `__forceinline`、`__asm` 等 AC5 语法 | AC6 不识别 → 报 `__forceinline` 等错误 |
| `SystemCoreClock` | 写在 `#if defined(__GNUC__)` 里 | **AC6 不定义 `__GNUC__`** → `port.c` 报未声明 |
| FPU | `configENABLE_FPU 0` | GCC CM4F 端口需要 **`configENABLE_FPU 1`** |

CubeMX **每次 Generate** 都会：

- 把 Keil 工程指回 **RVDS**
- **删除** `Middlewares/.../GCC/`（只保留 RVDS）
- 可能清空部分 USER CODE（脚本会补回关键项）

因此不能依赖「人生成一次就永久正确」，需要 **Generate 后自动跑脚本**。

---

## 日常流程（固定三步）

```
1. 关闭 Keil
2. STM32CubeMX → GENERATE CODE
3. 打开 Keil → Rebuild（建议先 Clean Targets）
```

CubeMX 在 **Project Manager → Code Generator → After Code Generation** 中应配置为：

```
tools\fix_keil_ac6_freertos.bat
```

脚本在 Generate **结束后**自动执行，无需手改 `uvprojx` 或 `FreeRTOSConfig.h`。

---

## 脚本做什么（`tools/fix_keil_ac6_freertos.ps1`）

| 步骤 | 操作 |
|------|------|
| 1 | 从 `tools/freertos_port/GCC/ARM_CM4F/` 复制 `port.c`、`portmacro.h` 到 `Middlewares/.../GCC/ARM_CM4F/` |
| 2 | 修改 `MDK-ARM/STM32G474RET6_MOTOR.uvprojx`：include 路径与 `port.c` 由 **RVDS → GCC** |
| 3 | 去掉 `uvprojx` 的 **UTF-8 BOM**（否则 Keil 报 *Cannot read project file*） |
| 4 | 在 `Core/Inc/FreeRTOSConfig.h` 的 **USER CODE** 中写入 `extern SystemCoreClock`、`configENABLE_FPU 1`（若缺失） |
| 5 | 从 Define 中移除 `__CC_ARM`（若存在） |

**脚本故意不做的事**（避免与 CubeMX 冲突）：

- 不修改 Keil Before Make 钩子
- 不改 `bringup`、CMSIS-DSP 等自定义工程组
- 不大改 `.mxproject`

---

## 常见现象与处理

### CubeMX 弹窗：`MDK-ARM V5.32 project generation have a problem`

- **含义**：C 源码已生成，但 CubeMX **没能完整合并** Keil 的 `.uvprojx`。
- **处理**：点 OK 继续；确认 After 脚本已跑；Keil **Rebuild**。以 **0 Error** 为准，不必追求弹窗消失。

### Keil：`Cannot read project file ... uvprojx`

- **常见原因**：`uvprojx` 带 UTF-8 BOM（旧版脚本或工具写入）。
- **处理**：双击 `tools\fix_keil_ac6_freertos.bat`，再开 Keil。

### 编译：`unknown type name '__forceinline'` / 路径含 `RVDS`

- **原因**：脚本未跑或 After 路径未配置。
- **处理**：双击 `tools\fix_keil_ac6_freertos.bat` → Rebuild；检查 CubeMX After 是否指向上述 bat。

### 编译：`use of undeclared identifier 'SystemCoreClock'`（仅 `port.c`）

- **原因**：`FreeRTOSConfig.h` USER CODE Includes 缺少声明（AC6 不走 `__GNUC__` 分支）。
- **处理**：再跑一次脚本；或检查 USER CODE Includes 是否有 `extern uint32_t SystemCoreClock;`。

### `main.c`：`redefinition of HAL_TIM_PeriodElapsedCallback`

- **原因**：在 `USER CODE 4` 里写了一份，CubeMX 又生成了一份。
- **规范**：FOC 等逻辑只放在 CubeMX 生成函数内的 **`USER CODE BEGIN Callback 1`**；`USER CODE 4` 仅保留 ADC 等其它回调。

---

## 新工程 / 新电脑迁移

1. 将本仓库 **`tools/` 整个目录** 复制到新工程根目录（与 `.ioc` 同级）。
2. CubeMX **After Code Generation** 设为 `tools\fix_keil_ac6_freertos.bat`，保存 `.ioc`。
3. 确认 Keil 使用 **Compiler V6（AC6）**，C/C++ Define 中 **无** `__CC_ARM`。
4. 若工程名不是 `STM32G474RET6_MOTOR`，需改脚本内 `uvprojx` 路径（或后续改为自动搜索 `MDK-ARM\*.uvprojx`）。
5. 按「日常流程」Generate → Rebuild 验证。

**建议提交 git 的内容：**

- `tools/`（含 `freertos_port/`）
- `STM32G474RET6_MOTOR.ioc`（含 UAScriptAfterPath）
- 本文档与 `tools/README.md`

---

## 何时需要 CubeMX Generate

| 需要 Generate | 不需要 Generate |
|---------------|-----------------|
| 改引脚、时钟、新增外设 | 只改 `main.c`、FOC、遥测、通信逻辑 |
| 改 FreeRTOS 任务 / 中断优先级 | 只改 `bringup/` 下应用代码 |
| 改 DMA / UART 等 CubeMX 配置 | 调试参数、VOFA 通道等 |

应用层开发：**直接 Keil Rebuild** 即可。

---

## 相关文件索引

| 路径 | 说明 |
|------|------|
| `tools/fix_keil_ac6_freertos.bat` | CubeMX After 入口 |
| `tools/fix_keil_ac6_freertos.ps1` | 实际修补逻辑 |
| `tools/freertos_port/GCC/ARM_CM4F/` | GCC 端口永久备份 |
| `MDK-ARM/STM32G474RET6_MOTOR.uvprojx` | Keil 工程（Generate 后由脚本改为 GCC） |
| `Core/Inc/FreeRTOSConfig.h` | USER CODE 由脚本维护 AC6 相关项 |
| `Core/Src/main.c` | TIM 回调：FOC 在 Callback 1 |

---

## 修订记录

| 日期 | 说明 |
|------|------|
| 2026-06-07 | 初版：RVDS→GCC 脚本、BOM 修复、FreeRTOSConfig USER CODE 自动补丁 |
