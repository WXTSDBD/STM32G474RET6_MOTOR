# 注释规范

自研 C 的注释以 [docs/代码注释规范.md](../../docs/代码注释规范.md) 为准。本文只留执行时不能违反的几条。

## 适用范围

`bringup/`、`config/`、`board/`、`Drivers/`（自研）、`motor/`、`platform/` 的新增与补注释。

不适用：CubeMX 生成的 `Core/Src/*.c`、`Core/Inc/*.h`，第三方 Middleware。

## 执行要点

1. `.c` 和 `.h` 都要有文件头：`@file`、`@date`、`@brief`，以及职责边界。`@date` 是文件头写入日，后补旧文件要注明不是诞生日期。写入后不改 `@date`。
2. 函数的 `/** */` 只写在 `.c` 的定义上（含 `static`）。`.h` 函数声明不写注释。唯一例外是实现就在 `.h` 里的 `static inline`。
3. 同一约束只写一处。节拍、中断号、调错会打电机的限制只写在该 `.h` 的文件头。`.c` 的 `@note` 只写函数内部顺序、profile 关系和调用方看不见的状态，不复述节拍。
4. 新文件和正在改的函数按规范 §5 给每个函数写 `@brief`。旧文件按 §12，先文件头、枚举和热路径约束，不要一次铺满所有 `static`。
5. 宏第一刀只写仍被读到的。故障门、注入幅值、PLL 死区在 `hfi_sqwave.c`，不在 `motor_params_m1.h`。`motor_params_m1.h` 不要铺完整文件，只写其中仍被读到的（如发布门槛、速度反馈）。没有引用点的不写「改了会怎样」。头文件保护宏和纯别名不写。
6. 文件级变量、结构体成员、以及要写的宏，说明用途和单位。
7. 正文中文，源文件 UTF-8 无 BOM。块注释正文不得再出现 `/*`（armclang `-Wcomment`，Warning 加 1，0 Warning 验收失败）。电压写 ud/uq，不写 `*ud`。不用 `Get-Content`/`Set-Content` 整文件改写，不引入字节 `0x85`。
8. 实验编号、录波文件名、施工计划不进注释。补 `hfi_sqwave` 的阶段名对着源码里的 `IDLE/MOVE/SETTLE/MEAS/LOG/DONE/CRAWL/RUN`，不照抄规范样例里的 HOLD/CAPTURE。
9. 纯注释提交不改逻辑、不改宏数值、不动故障门和 `INIT_FROM_ENC`，不算解冻。Size 必须与动手前 `rebuild_log.txt` 的 `Program Size` 逐字相同；改过的翻译单元 `-E -dM` diff 为 0。不要用 `-E -dD` 当通过条件（行号标记会漂）。只有动到 `s_eps`、AUTH、ATAN2 或 12 通道映射才重录金样。
10. 改硬件映射时，文件头里的映射表一起改。
