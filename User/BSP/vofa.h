#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "servo_mapping.h"   // 为了使用 moterMapHeader

#define VOFA_CH_COUNT 6U

typedef struct
{
    float fdata[VOFA_CH_COUNT];
    uint8_t tail[4];
} VOFA_JustFloatFrame_t;

/* 发送 6 通道浮点数据到 VOFA */
void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5);

/* 直接发送 MoterMap 中的 j0~j5 */
void VOFA_SendMotorMap(const moterMapHeader *map);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */