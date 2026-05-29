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
        "t_ms,"
        "vx_set,vy_set,wz_set,"
        "wheel_speed_set_0,wheel_speed_set_1,wheel_speed_set_2,wheel_speed_set_3,"
        "motor_speed_0,motor_speed_1,motor_speed_2,motor_speed_3,"
        "model_current_0,model_current_1,model_current_2,model_current_3,"
        "give_current_0,give_current_1,give_current_2,give_current_3,"
        "set_power,buffer_energy,"
        "pm01_v_out,pm01_i_out,pm01_temp,pm01_p_out,"
        "k_label,"
        "s_label_0,s_label_1,s_label_2,s_label_3\r\n";

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    memcpy(s_vofa_csv_buf, header, sizeof(header) - 1U);
    HAL_UART_Transmit_DMA(&huart1, s_vofa_csv_buf, (uint16_t)(sizeof(header) - 1U));
}

void VOFA_SendAiPowerCsv(const VOFA_AiPowerCsv_t *log)
{
    int len;

    if (log == NULL)
    {
        return;
    }

    len = snprintf((char *)s_vofa_csv_buf,
                   sizeof(s_vofa_csv_buf),
                   "%lu,"
                   "%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.4f,%.4f,%.4f,%.4f,"
                   "%.2f,%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,%.2f,"
                   "%.2f,%.2f,"
                   "%.2f,%.2f,%.2f,%.2f,"
                   "%.4f,"
                   "%.4f,%.4f,%.4f,%.4f\r\n",
                   (unsigned long)log->t_ms,
                   log->vx_set,
                   log->vy_set,
                   log->wz_set,
                   log->wheel_speed_set[0],
                   log->wheel_speed_set[1],
                   log->wheel_speed_set[2],
                   log->wheel_speed_set[3],
                   log->motor_speed[0],
                   log->motor_speed[1],
                   log->motor_speed[2],
                   log->motor_speed[3],
                   log->model_current[0],
                   log->model_current[1],
                   log->model_current[2],
                   log->model_current[3],
                   log->give_current[0],
                   log->give_current[1],
                   log->give_current[2],
                   log->give_current[3],
                   log->set_power,
                   log->buffer_energy,
                   log->pm01_v_out,
                   log->pm01_i_out,
                   log->pm01_temp,
                   log->pm01_p_out,
                   log->k_label,
                   log->s_label[0],
                   log->s_label[1],
                   log->s_label[2],
                   log->s_label[3]);

    VOFA_SendCsvBuffer(len);
}
#endif

void VOFA_SendAiPowerJustFloat(const VOFA_AiPowerCsv_t *log)
{
    float *ch = s_vofa_ai_power_frame.fdata;

    if (log == NULL)
    {
        return;
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    ch[0] = (float)log->t_ms;
    ch[1] = log->vx_set;
    ch[2] = log->vy_set;
    ch[3] = log->wz_set;
    ch[4] = log->wheel_speed_set[0];
    ch[5] = log->wheel_speed_set[1];
    ch[6] = log->wheel_speed_set[2];
    ch[7] = log->wheel_speed_set[3];
    ch[8] = log->motor_speed[0];
    ch[9] = log->motor_speed[1];
    ch[10] = log->motor_speed[2];
    ch[11] = log->motor_speed[3];
    ch[12] = log->model_current[0];
    ch[13] = log->model_current[1];
    ch[14] = log->model_current[2];
    ch[15] = log->model_current[3];
    ch[16] = log->give_current[0];
    ch[17] = log->give_current[1];
    ch[18] = log->give_current[2];
    ch[19] = log->give_current[3];
    ch[20] = log->set_power;
    ch[21] = log->buffer_energy;
    ch[22] = log->pm01_v_out;
    ch[23] = log->pm01_i_out;
    ch[24] = log->pm01_temp;
    ch[25] = log->pm01_p_out;
    ch[26] = log->k_label;
    ch[27] = log->s_label[0];
    ch[28] = log->s_label[1];
    ch[29] = log->s_label[2];
    ch[30] = log->s_label[3];

    HAL_UART_Transmit_DMA(&huart1,
                          (uint8_t *)&s_vofa_ai_power_frame,
                          sizeof(s_vofa_ai_power_frame));
}
