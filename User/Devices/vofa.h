#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "struct_typedef.h"

#define VOFA_CH_COUNT 6U
#define VOFA_AI_POWER_MOTOR_COUNT 4U
#define VOFA_AI_POWER_CH_COUNT 9U
#define VOFA_AI_CSV_BUFFER_SIZE 512U
#define VOFA_ENABLE_CSV_TEXT 0

typedef struct
{
    float fdata[VOFA_CH_COUNT];
    uint8_t tail[4];
} VOFA_JustFloatFrame_t;

typedef struct
{
    float fdata[VOFA_AI_POWER_CH_COUNT];
    uint8_t tail[4];
} VOFA_AiPowerJustFloatFrame_t;

/* 发送 6 通道浮点数据到 VOFA */
void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5);

typedef struct
{
    uint32_t t_ms;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 wheel_speed_set[4];
    fp32 motor_speed[4];
    fp32 model_current[4];
    fp32 give_current[4];
    fp32 set_power;
    fp32 buffer_energy;
    fp32 pm01_v_out;
    fp32 pm01_i_out;
    fp32 pm01_temp;
    fp32 pm01_p_out;
    fp32 k_label;
    fp32 s_label[4];
} VOFA_AiPowerCsv_t;

#if VOFA_ENABLE_CSV_TEXT
void VOFA_SendAiPowerCsvHeader(void);
void VOFA_SendAiPowerCsv(const VOFA_AiPowerCsv_t *log, uint8_t motor_idx);
#endif
void VOFA_SendAiPowerJustFloat(const VOFA_AiPowerCsv_t *log, uint8_t motor_idx);


#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
