/**
 * @file telem_lut_dump.h
 * @brief 标定结束后经 VOFA JustFloat 突发 LUT 表（头帧 + 数据 + 尾帧）。
 *
 * 需 M1_VOFA_LUT_DUMP_ENABLE=1 且 M1_ID_LOCK_CAL_SWEEP=1；否则 API 为空 stub。
 */

#ifndef TELEM_LUT_DUMP_H
#define TELEM_LUT_DUMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** ch0 魔数：LUT 头帧 */
#define M1_VOFA_LUT_MAGIC_HDR     (-888888.0f)
/** ch0 魔数：LUT 尾帧 */
#define M1_VOFA_LUT_MAGIC_TAIL    (-999999.0f)
/** ch4 协议版本：见下表；ch5 随 proto 携带 runtime meta */
#define M1_VOFA_LUT_PROTO_VER     (3.0f)

/*
 * LUT 突发头帧（telem_lut_dump）— CSV 搜 I0≈−888888：
 *   I1=len, I2=Rs, I3=outlier, I4=proto, I5=meta
 * 数据帧：amp0,val0, amp1,val1, amp2,val2（phase 域，commit 后）
 * 尾帧：I0=−999999, I1=len, I2=sum(vals)
 *
 * proto 3.0 phase ×0.866 | 3.1 +LOW_FLAT | 3.2 +I_ZERO_OFF | 3.3 ch5=APPLY_MIN_A
 * proto 3.4 geo 双角样本池建 phase 表，ch5=geo_sample_len
 * proto 3.5 geo 分相三表 A/B/C，ch5=geo_sample_len；数据帧 amp_a,val_a..amp_c,val_c
 * 表来源：Pass0 geo 样本池 或 capture → ×0.866 → 本突发
 */

/** 标定 DONE 后调用一次，启动突发发送状态机 */
void telem_lut_dump_arm(void);

/** 1=突发进行中（HDR/DATA/TAIL），发完前勿进入 Pass1/Iq */
uint8_t telem_lut_dump_is_busy(void);

/**
 * @brief 若处于 LUT 突发，填充下一帧前 lut_dump_ch 个通道（通常 6）
 * @return 1=vals 有效（替代正常 telem）；0=未突发或已发完
 */
int telem_lut_dump_next(float vals[], uint8_t lut_dump_ch);

#ifdef __cplusplus
}
#endif

#endif
