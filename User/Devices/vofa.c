#include "vofa.h"
#include "usart.h"
#if VOFA_ENABLE_CSV_TEXT
#include <stdio.h>
#include <string.h>
#endif

static VOFA_JustFloatFrame_t s_vofa_frame =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

static VOFA_AiPowerJustFloatFrame_t s_vofa_ai_power_frame =
{
    .tail = {0x00, 0x00, 0x80, 0x7F}
};

#if VOFA_ENABLE_CSV_TEXT
static uint8_t s_vofa_csv_buf[VOFA_AI_CSV_BUFFER_SIZE];

static void VOFA_SendCsvBuffer(int len)
{
    if (len <= 0)
    {
        return;
    }

    if (len > (int)sizeof(s_vofa_csv_buf))
    {
        len = (int)sizeof(s_vofa_csv_buf);
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    HAL_UART_Transmit_DMA(&huart1, s_vofa_csv_buf, (uint16_t)len);
}
#endif

void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5)
{
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    s_vofa_frame.fdata[0] = ch0;
    s_vofa_frame.fdata[1] = ch1;
    s_vofa_frame.fdata[2] = ch2;
    s_vofa_frame.fdata[3] = ch3;
    s_vofa_frame.fdata[4] = ch4;
    s_vofa_frame.fdata[5] = ch5;

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&s_vofa_frame, sizeof(s_vofa_frame));
}

#if VOFA_ENABLE_CSV_TEXT
void VOFA_SendAiPowerCsvHeader(void)
{
    static const char header[] =
        "vx_set,vy_set,wz_set,"
        "wheel_speed_set,"
        "motor_speed,"
        "model_current,"
        "give_current,"
        "set_power,"
        "pm01_p_out\r\n";

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    memcpy(s_vofa_csv_buf, header, sizeof(header) - 1U);
    HAL_UART_Transmit_DMA(&huart1, s_vofa_csv_buf, (uint16_t)(sizeof(header) - 1U));
}

void VOFA_SendAiPowerCsv(const VOFA_AiPowerCsv_t *log, uint8_t motor_idx)
{
    int len;

    if (log == NULL)
    {
        return;
    }

    if (motor_idx >= VOFA_AI_POWER_MOTOR_COUNT)
    {
        return;
    }

    len = snprintf((char *)s_vofa_csv_buf,
                   sizeof(s_vofa_csv_buf),
                   "%.4f,%.4f,%.4f,"
                   "%.4f,"
                   "%.4f,"
                   "%.2f,"
                   "%.2f,"
                   "%.2f,"
                   "%.2f\r\n",
                   log->vx_set,
                   log->vy_set,
                   log->wz_set,
                   log->wheel_speed_set[motor_idx],
                   log->motor_speed[motor_idx],
                   log->model_current[motor_idx],
                   log->give_current[motor_idx],
                   log->set_power,
                   log->pm01_p_out);

    VOFA_SendCsvBuffer(len);
}
#endif

void VOFA_SendAiPowerJustFloat(const VOFA_AiPowerCsv_t *log, uint8_t motor_idx)
{
    float *ch = s_vofa_ai_power_frame.fdata;

    if (log == NULL)
    {
        return;
    }

    if (motor_idx >= VOFA_AI_POWER_MOTOR_COUNT)
    {
        return;
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    ch[0] = log->vx_set;
    ch[1] = log->vy_set;
    ch[2] = log->wz_set;
    ch[3] = log->wheel_speed_set[motor_idx];
    ch[4] = log->motor_speed[motor_idx];
    ch[5] = log->model_current[motor_idx];
    ch[6] = log->give_current[motor_idx];
    ch[7] = log->set_power;
    ch[8] = log->pm01_p_out;

    HAL_UART_Transmit_DMA(&huart1,
                          (uint8_t *)&s_vofa_ai_power_frame,
                          sizeof(s_vofa_ai_power_frame));
}
