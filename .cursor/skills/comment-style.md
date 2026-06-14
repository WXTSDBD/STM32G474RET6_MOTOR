# 注释规范

## 适用范围

`bringup/`、`config/`、`board/`、`drivers/`、`motor/`、`platform/` 的新增与较大改动。

**不适用**：CubeMX 生成的 `Core/Src/*.c`（勿手改）、第三方 Middleware。

## 铁律

1. **文件头 `@file` + `@brief`**。3-6 行说清这个文件干什么、不该干什么。架构约束写进注释——driver 不含板级 HAL 句柄、bridge 是唯一填 HAL 指针的地方。driver 和 bridge 的注释互相对上。

2. **`.c` 里每个函数加 `@brief`**。一行说清做什么。`static` 函数也要。

3. **多参数或有坑的函数加 `@param` + `@note`**。`@note` 写调用顺序限制、ISR 安全性、和 CubeMX Generate 的关系——这些比参数说明更重要。

4. **不用废话注释。** 注释解释「为什么 / 边界 / 顺序」，不重复「是什么」——代码已经告诉了你做了什么。
   ❌ "循环遍历 channels"
   ✅ "前 discard 帧丢弃，避免 TIM 刚启动采样不稳定"

5. **不用英文写注释。** 除 `@file/@brief/@param/@return/@note/@see` 标签和符号名。

6. **映射关系用注释列表。** switch/结构体做硬件绑定的地方，在文件头或函数前写映射表。
   ```
   当前映射：
     M1 — hadc2 JDR1/2/3，TIM8 CH4 触发，TIM8 PWM
     M2 — 占位；TIM1，ADC 待定
   ```

7. **改 binding/bridge 映射时同步改注释。** 防文档漂移。

## .h / .c 分工

- **.h**：文件头 + 类型/枚举一行说明（调用方必须知道的语义）。不写函数 `@brief`（除非 inline 且实现在 .h）。
- **.c**：所有函数 `@brief` / `@param` / `@return`。不重复 .h 已有的类型说明。

## 金样参照

- `bringup/adc_sample/adc_sample.c` — 函数注释密度 + `@note` 写法
- `config/bridge_cubemx.c` — 映射表注释 + "CubeMX 句柄变化只需改本文件"
- `config/bsp_axes.c` — init 顺序注释
