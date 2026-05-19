#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "adrc.h"
#include "bsp_fdcan.h"
#include "project_config.h"
#include "remote_control.h"

/* ========================= shoot task 基础配置 ========================= */
#ifndef SHOOT_TASK_INIT_TIME
#define SHOOT_TASK_INIT_TIME 200U         // 任务启动延时，单位 ms
#endif

#ifndef SHOOT_CONTROL_TIME
#define SHOOT_CONTROL_TIME 1U             // shoot 控制周期，单位 ms
#endif

#ifndef SHOOT_RC_MODE_CHANNEL
#define SHOOT_RC_MODE_CHANNEL 1           // 遥控器 shoot 模式通道
#endif

/* ========================= 摩擦轮目标与保护配置 ========================= */
#ifndef SHOOT_FRIC_TARGET_SPEED_RPM
#define SHOOT_FRIC_TARGET_SPEED_RPM 3500  // 三路摩擦轮统一目标转速，单位 rpm
#endif

#ifndef SHOOT_FRIC_WHEEL_RADIUS_M
#define SHOOT_FRIC_WHEEL_RADIUS_M 0.03f   // 摩擦轮半径，单位 m
#endif

#ifndef SHOOT_FRIC_MAX_CURRENT
#define SHOOT_FRIC_MAX_CURRENT 5       // 三路摩擦轮电流限幅，单位 A
#endif

#ifndef SHOOT_FRIC_CURRENT_CMD_FULL_SCALE
#define SHOOT_FRIC_CURRENT_CMD_FULL_SCALE 16384.0f
#endif

#ifndef SHOOT_FRIC_CURRENT_FULL_SCALE_A
#define SHOOT_FRIC_CURRENT_FULL_SCALE_A 20.0f
#endif

#ifndef SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A
#define SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A 0.3f
#endif

#ifndef SHOOT_FRIC_REDUCTION_RATIO
#define SHOOT_FRIC_REDUCTION_RATIO (3591.0f / 187.0f)
#endif

#ifndef SHOOT_FRIC_FEEDBACK_RANGE_RPM
#define SHOOT_FRIC_FEEDBACK_RANGE_RPM 7000 // 速度反馈量程估计，供 ADRC 参数设计参考
#endif

#ifndef SHOOT_FRIC_FDB_TIMEOUT
#define SHOOT_FRIC_FDB_TIMEOUT 100U       // 反馈超时保护，单位 ms
#endif

#ifndef SHOOT_FRIC_TEMP_LIMIT
#define SHOOT_FRIC_TEMP_LIMIT 80U         // 电机温度保护阈值
#endif

/* ========================= fric1 ADRC 参数 =========================
 * B0 越小，给出的补偿电流通常越大；太小会更躁、更容易过冲
 * RESPONSE_TIME_S 越小，速度环越激进，恢复更快；太小会更容易不稳
 * OBSERVER_RATIO 越大，扰动观测越敏感；太大更容易把噪声当扰动
 * OUTPUT_RATE_LIMIT 越大，电流爬升越快；太大时电流尖峰更明显
 */
#ifndef SHOOT_FRIC1_B0
#define SHOOT_FRIC1_B0 15000.0f
#endif

#ifndef SHOOT_FRIC1_RESPONSE_TIME_S
#define SHOOT_FRIC1_RESPONSE_TIME_S 0.01469442f
#endif

#ifndef SHOOT_FRIC1_OBSERVER_RATIO
#define SHOOT_FRIC1_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC1_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC1_OUTPUT_RATE_LIMIT 300
#endif

/* ========================= fric2 ADRC 参数 ========================= */
#ifndef SHOOT_FRIC2_B0
#define SHOOT_FRIC2_B0 15000.0f
#endif

#ifndef SHOOT_FRIC2_RESPONSE_TIME_S
#define SHOOT_FRIC2_RESPONSE_TIME_S 0.01469442f
#endif

#ifndef SHOOT_FRIC2_OBSERVER_RATIO
#define SHOOT_FRIC2_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC2_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC2_OUTPUT_RATE_LIMIT 300
#endif

/* ========================= fric3 ADRC 参数 ========================= */
#ifndef SHOOT_FRIC3_B0
#define SHOOT_FRIC3_B0 15000.0f
#endif

#ifndef SHOOT_FRIC3_RESPONSE_TIME_S
#define SHOOT_FRIC3_RESPONSE_TIME_S 0.01469442f
#endif

#ifndef SHOOT_FRIC3_OBSERVER_RATIO
#define SHOOT_FRIC3_OBSERVER_RATIO 2.5f
#endif

#ifndef SHOOT_FRIC3_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC3_OUTPUT_RATE_LIMIT 300
#endif

/* ========================= ADRC 非线性项配置 ========================= */
#ifndef SHOOT_FRIC_ERROR_LINEAR_ZONE
#define SHOOT_FRIC_ERROR_LINEAR_ZONE 120  // fal 线性区间，增大后小误差段更平缓
#endif

#ifndef SHOOT_FRIC_ALPHA1
#define SHOOT_FRIC_ALPHA1 0.5f            // 控制律 fal 指数
#endif

#ifndef SHOOT_FRIC_ALPHA2
#define SHOOT_FRIC_ALPHA2 0.25f           // ESO fal 指数
#endif

/* ========================= 掉速后固定前馈补偿配置 ========================= */
#ifndef SHOOT_FRIC_FF_ENABLE
#define SHOOT_FRIC_FF_ENABLE 1            // 1: 使能掉速触发前馈 0: 关闭
#endif

#ifndef SHOOT_FRIC1_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC1_FF_TRIGGER_DROP_RPM 300.0f // fric1 单拍掉速触发阈值
#endif

#ifndef SHOOT_FRIC1_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC1_FF_MIN_SPEED_RATIO 0.85f   // fric1 进入稳速区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC1_FF_CURRENT
#define SHOOT_FRIC1_FF_CURRENT 1        // fric1 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC1_FF_DURATION_MS
#define SHOOT_FRIC1_FF_DURATION_MS 22U     // fric1 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC1_FF_COOLDOWN_MS
#define SHOOT_FRIC1_FF_COOLDOWN_MS 20U     // fric1 前馈冷却时间，单位 ms
#endif

#ifndef SHOOT_FRIC2_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC2_FF_TRIGGER_DROP_RPM 300.0f // fric2 单拍掉速触发阈值
#endif

#ifndef SHOOT_FRIC2_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC2_FF_MIN_SPEED_RATIO 0.85f   // fric2 进入稳速区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC2_FF_CURRENT
#define SHOOT_FRIC2_FF_CURRENT 1        // fric2 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC2_FF_DURATION_MS
#define SHOOT_FRIC2_FF_DURATION_MS 22U     // fric2 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC2_FF_COOLDOWN_MS
#define SHOOT_FRIC2_FF_COOLDOWN_MS 20U     // fric2 前馈冷却时间，单位 ms
#endif

#ifndef SHOOT_FRIC3_FF_TRIGGER_DROP_RPM
#define SHOOT_FRIC3_FF_TRIGGER_DROP_RPM 300.0f // fric3 单拍掉速触发阈值
#endif

#ifndef SHOOT_FRIC3_FF_MIN_SPEED_RATIO
#define SHOOT_FRIC3_FF_MIN_SPEED_RATIO 0.85f   // fric3 进入稳速区后才允许触发前馈
#endif

#ifndef SHOOT_FRIC3_FF_CURRENT
#define SHOOT_FRIC3_FF_CURRENT 1        // fric3 固定前馈电流，单位 A
#endif

#ifndef SHOOT_FRIC3_FF_DURATION_MS
#define SHOOT_FRIC3_FF_DURATION_MS 22U     // fric3 前馈持续时间，单位 ms
#endif

#ifndef SHOOT_FRIC3_FF_COOLDOWN_MS
#define SHOOT_FRIC3_FF_COOLDOWN_MS 20U     // fric3 前馈冷却时间，单位 ms
#endif

/* ========================= 三路摩擦轮安装方向 ========================= */
#ifndef SHOOT_FRIC1_DIRECTION
#define SHOOT_FRIC1_DIRECTION -1          // fric1 实际安装方向
#endif

#ifndef SHOOT_FRIC2_DIRECTION
#define SHOOT_FRIC2_DIRECTION 1           // fric2 实际安装方向
#endif

#ifndef SHOOT_FRIC3_DIRECTION
#define SHOOT_FRIC3_DIRECTION 1           // fric3 实际安装方向
#endif

typedef enum
{
    SHOOT_TASK_STOP = 0,
    SHOOT_TASK_READY_FRIC,
} shoot_task_mode_e;

typedef struct
{
    const motor_measure_t *measure;
    adrc_type_def speed_adrc;
    float speed_rpm;
    float speed_mps;
    float speed_set_rpm;
    float last_speed_rpm;
    float direction;
    uint16_t ff_ticks;
    uint16_t ff_cooldown_ticks;
    int16_t ff_current;              // 前馈补偿电流，单位 A
    int16_t give_current;            // 最终发送给电机的电流，单位 mA
    int16_t given_current;           // 电机反馈中的电流原始值
    float give_current_a;
    float given_current_a;
    float give_input_torque_nm;
    float given_input_torque_nm;
} shoot_task_motor_t;

typedef struct
{
    shoot_task_mode_e mode;
    shoot_task_mode_e last_mode;
    const RC_ctrl_t *rc;
    bool friction_enable;
    shoot_task_motor_t fric1;
    shoot_task_motor_t fric2;
    shoot_task_motor_t fric3;
} shoot_task_control_t;

extern shoot_task_control_t shoot_task_control;

void shoot_task_init(void);
void shoot_task_loop(void);

#endif
