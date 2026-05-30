#include "vofa.h"
#include "bsp_fdcan.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "pm01_api.h"
#include "referee.h"
#include "shoot_task.h"
#include "usart.h"
#include <math.h>
#if VOFA_ENABLE_CSV_TEXT
#include <stdio.h>
#include <string.h>
#endif

#ifndef VOFA_SERVICE_CHASSIS_MOTOR_INDEX
#define VOFA_SERVICE_CHASSIS_MOTOR_INDEX 1U
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

static void VOFA_Send6(float ch0, float ch1, float ch2, float ch3, float ch4, float ch5)
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

void VOFA_SendChassisMotorMeasure(uint8_t motor_idx)
{
    const motor_measure_t *motor;

    if (motor_idx >= VOFA_CHASSIS_MOTOR_COUNT)
    {
        return;
    }

    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    motor = &CHASSIS_MOTOR_MEASURE[motor_idx];

    s_vofa_frame.fdata[0] = (float)motor->ecd;
    s_vofa_frame.fdata[1] = (float)motor->speed_rpm;
    s_vofa_frame.fdata[2] = (float)motor->given_current;
    s_vofa_frame.fdata[3] = chassis_move.ai_predicted_power ;
    s_vofa_frame.fdata[4] = (float)motor->last_ecd;
    s_vofa_frame.fdata[5] = PowerLimit.P_origin;

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

void VOFA_SendChassisAiPowerJustFloat(uint8_t motor_idx)
{
    VOFA_AiPowerCsv_t log;
    static uint32_t last_send_ms = 0U;
    uint32_t now_ms;

    if (motor_idx >= VOFA_AI_POWER_MOTOR_COUNT)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - last_send_ms) < CHASSIS_AI_LOG_PERIOD_MS)
    {
        return;
    }
    last_send_ms = now_ms;

    log.t_ms = now_ms;
    log.vx_set = chassis_move.vx_set;
    log.vy_set = chassis_move.vy_set;
    log.wz_set = chassis_move.wz_set;
    log.set_power = PowerLimit.set_power;
    log.buffer_energy = (fp32)get_buffer_energy();
    log.pm01_v_out = (fp32)pm01_od.v_out * 0.01f;
    log.pm01_i_out = (fp32)pm01_od.i_out * 0.01f;
    log.pm01_temp = (fp32)pm01_od.temp;
    log.pm01_p_out = (fp32)pm01_od.p_out * 0.01f;
    log.k_label = PowerLimit.K_Reduction;

    for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
    {
        fp32 model_current = chassis_move.model_3508_out[i];
        fp32 give_current = (fp32)chassis_move.chassis_3508[i].give_current;

        log.wheel_speed_set[i] = chassis_move.chassis_3508[i].speed_set;
        log.motor_speed[i] = chassis_move.chassis_3508[i].speed;
        log.model_current[i] = model_current;
        log.give_current[i] = give_current;
        log.s_label[i] = (fabsf(model_current) > 1.0f) ? (give_current / model_current) : 0.0f;
    }

    VOFA_SendAiPowerJustFloat(&log, motor_idx);
}

void VOFA_SendGimbalFric(void)
{
    float current_avg;

    current_avg = (shoot_task_control.fric1.give_current_a +
                   shoot_task_control.fric2.give_current_a +
                   shoot_task_control.fric3.give_current_a) / 3.0f;

    VOFA_Send6(shoot_task_control.fric1.speed_rpm,
               shoot_task_control.fric2.speed_rpm,
               shoot_task_control.fric3.speed_rpm,
               current_avg,
               shoot_task_control.bullet_speed_min_avg_rpm,
               shoot_task_control.estimated_bullet_speed_mps);
}

void VOFA_SendGimbalYaw(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->absolute_angle_set,
               yaw->absolute_angle,
               yaw->gyro,
               yaw->current_set);
}

void VOFA_SendGimbalPitch(void)
{
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->absolute_angle_set,
               pitch->absolute_angle,
               pitch->gyro,
               pitch->current_set);
}

void VOFA_SendGimbalYawPitchHalf(void)
{
    const gimbal_motor_t *yaw = &gimbal_control.gimbal_yaw_motor;
    const gimbal_motor_t *pitch = &gimbal_control.gimbal_pitch_motor;

    VOFA_Send6(yaw->relative_angle_set,
               yaw->relative_angle,
               yaw->static_friction_comp,
               pitch->relative_angle_set,
               pitch->relative_angle,
               pitch->static_friction_comp);
}

void VOFA_SendGimbalStrum(void)
{
    const MITMeasure_t *strum = &MIT_MOTOR_MEASURE[SHOOT_STRUM_MIT_INDEX];

    VOFA_Send6(strum->fdb.pos,
               strum->set.POS,
               strum->fdb.vel,
               strum->fdb.tor,
               strum->set.TOR,
               strum->fdb.t_motor);
}

void VOFA_ServiceSend(void)
{
    VOFA_SendChassisMotorMeasure(VOFA_SERVICE_CHASSIS_MOTOR_INDEX);
}
