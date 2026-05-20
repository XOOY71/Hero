#include "shoot_task.h"

#include <string.h>

#include "fdcan.h"
#include "gimbal_behaviour.h"

/* 摩擦轮电机 CAN 电流指令 ID。 */
#define SHOOT_FRICTION_CMD_ID 0x200U
#define SHOOT_FRIC_RPM_TO_MPS (2.0f * PI * SHOOT_FRIC_WHEEL_RADIUS_M / 60.0f)
#define SHOOT_FRIC_MA_PER_A 1000.0f

extern motor_measure_t DJI_MOTOR_MEASURE[8];

/* 射击任务全局控制实例。 */
shoot_task_control_t shoot_task_control;

static void shoot_task_init_control(shoot_task_control_t *control);
static void shoot_task_set_mode(shoot_task_control_t *control);
static void shoot_task_update_feedback(shoot_task_control_t *control);
static void shoot_task_control_friction(shoot_task_control_t *control);
static void shoot_task_stop_friction(shoot_task_control_t *control);
static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current, int16_t fric3_current);
static void shoot_task_motor_init(shoot_task_motor_t *motor,
                                  const motor_measure_t *measure,
                                  float direction,
                                  float b0,
                                  float response_time_s,
                                  float observer_ratio,
                                  float output_rate_limit);
static void shoot_task_motor_reset(shoot_task_motor_t *motor);
static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor);
static float shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm);
static float shoot_task_limit_current_a(float current_a);
static int16_t shoot_task_current_a_to_current_ma(float current_a);
static float shoot_task_current_ma_to_current_a(int16_t current_ma);
static int16_t shoot_task_current_ma_to_esc_cmd(int16_t current_ma);
static float shoot_task_feedback_cmd_to_current_a(int16_t current_cmd);
static float shoot_task_current_a_to_input_torque_nm(float current_a);
static void shoot_task_motor_update_current_physics(shoot_task_motor_t *motor);
static void shoot_task_motor_finalize_current(shoot_task_motor_t *motor);
static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now);
static bool shoot_task_motor_should_trigger_feedforward(const shoot_task_motor_t *motor,
                                                        float trigger_drop_rpm,
                                                        float min_speed_ratio);
static void shoot_task_motor_apply_feedforward(shoot_task_motor_t *motor);
static void shoot_task_update_history(shoot_task_control_t *control);
static void shoot_task_update_bullet_speed_estimate(shoot_task_control_t *control);
static bool shoot_task_should_start_bullet_speed_estimate(const shoot_task_control_t *control);
static void shoot_task_start_bullet_speed_estimate(shoot_task_control_t *control);
static uint16_t shoot_task_ms_to_ticks(uint16_t ms);
static float shoot_task_avg3(float a, float b, float c);
static float shoot_task_min_float(float a, float b);

/**
 * @brief 初始化射击任务控制对象。
 *
 * 完成遥控器绑定、模式清零以及三路摩擦轮电机控制器初始化。
 */
void shoot_task_init(void)
{
    shoot_task_init_control(&shoot_task_control);
}

/**
 * @brief 射击任务周期循环。
 *
 * 每个控制周期依次完成模式判定、反馈刷新以及摩擦轮启停控制。
 */
void shoot_task_loop(void)
{
    /* 先根据遥控和上层状态机决定当前工作模式。 */
    shoot_task_set_mode(&shoot_task_control);
    /* 再读取电机最新转速反馈，供后续闭环控制使用。 */
    shoot_task_update_feedback(&shoot_task_control);

    if (shoot_task_control.mode == SHOOT_TASK_READY_FRIC)
    {
        /* 就绪状态下闭环控制三路摩擦轮目标转速。 */
        shoot_task_control_friction(&shoot_task_control);
    }
    else
    {
        /* 非就绪状态立即停止摩擦轮输出。 */
        shoot_task_stop_friction(&shoot_task_control);
    }

    /* 保存本周期模式，用于下周期判断是否发生模式切换。 */
    shoot_task_control.last_mode = shoot_task_control.mode;
}

/**
 * @brief 初始化射击控制结构体。
 * @param control 射击任务控制对象指针。
 */
static void shoot_task_init_control(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    /* 清零运行时状态，避免使用未初始化数据。 */
    memset(control, 0, sizeof(*control));
    /* 获取遥控器数据入口。 */
    control->rc = get_remote_control_point();
    control->mode = SHOOT_TASK_STOP;
    control->last_mode = SHOOT_TASK_STOP;

    /* 绑定三路摩擦轮电机反馈并初始化各自速度控制器。 */
    shoot_task_motor_init(&control->fric1,
                          &DJI_MOTOR_MEASURE[0],
                          SHOOT_FRIC1_DIRECTION,
                          SHOOT_FRIC1_B0,
                          SHOOT_FRIC1_RESPONSE_TIME_S,
                          SHOOT_FRIC1_OBSERVER_RATIO,
                          SHOOT_FRIC1_OUTPUT_RATE_LIMIT);
    shoot_task_motor_init(&control->fric2,
                          &DJI_MOTOR_MEASURE[1],
                          SHOOT_FRIC2_DIRECTION,
                          SHOOT_FRIC2_B0,
                          SHOOT_FRIC2_RESPONSE_TIME_S,
                          SHOOT_FRIC2_OBSERVER_RATIO,
                          SHOOT_FRIC2_OUTPUT_RATE_LIMIT);
    shoot_task_motor_init(&control->fric3,
                          &DJI_MOTOR_MEASURE[2],
                          SHOOT_FRIC3_DIRECTION,
                          SHOOT_FRIC3_B0,
                          SHOOT_FRIC3_RESPONSE_TIME_S,
                          SHOOT_FRIC3_OBSERVER_RATIO,
                          SHOOT_FRIC3_OUTPUT_RATE_LIMIT);
}

/**
 * @brief 根据遥控器输入和云台状态更新射击模式。
 * @param control 射击任务控制对象指针。
 */
static void shoot_task_set_mode(shoot_task_control_t *control)
{
    /* 记录上一拍键值，用于检测按键上升沿。 */
    static uint16_t last_key_value = 0U;
    uint16_t pressed_keys;
    int shoot_switch;

    if (control == NULL || control->rc == NULL)
    {
        return;
    }

    /* 仅保留本周期新按下的按键。 */
    pressed_keys = (uint16_t)(control->rc->key.v & (uint16_t)(~last_key_value));
    last_key_value = control->rc->key.v;
    /* 读取射击三档开关状态。 */
    shoot_switch = control->rc->rc.s[SHOOT_RC_MODE_CHANNEL];

    if ((pressed_keys & KEY_PRESSED_OFFSET_R) != 0U)
    {
        /* R 键上升沿使能摩擦轮。 */
        control->friction_enable = true;
    }

    if ((pressed_keys & KEY_PRESSED_OFFSET_G) != 0U)
    {
        /* G 键上升沿关闭摩擦轮。 */
        control->friction_enable = false;
    }

    if (switch_is_down(shoot_switch))
    {
        /* 遥控拨杆下档强制开启摩擦轮。 */
        control->friction_enable = true;
    }
    else if (switch_is_mid(shoot_switch))
    {
        /* 遥控拨杆中档强制关闭摩擦轮。 */
        control->friction_enable = false;
    }

    if (gimbal_cmd_to_shoot_stop())
    {
        /* 上层要求停射时，射击模块进入停止态。 */
        control->mode = SHOOT_TASK_STOP;
    }
    else if (control->friction_enable)
    {
        /* 已使能摩擦轮时进入摩擦轮预备状态。 */
        control->mode = SHOOT_TASK_READY_FRIC;
    }
    else
    {
        control->mode = SHOOT_TASK_STOP;
    }
}

/**
 * @brief 刷新三路摩擦轮转速反馈。
 * @param control 射击任务控制对象指针。
 */
static void shoot_task_update_feedback(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    if (control->fric1.measure != NULL)
    {
        /* 按配置方向统一转速正负号，便于后续控制器复用。 */
        control->fric1.speed_rpm = (float)control->fric1.measure->speed_rpm * control->fric1.direction;
        control->fric1.speed_mps = control->fric1.speed_rpm * SHOOT_FRIC_RPM_TO_MPS;
    }

    if (control->fric2.measure != NULL)
    {
        control->fric2.speed_rpm = (float)control->fric2.measure->speed_rpm * control->fric2.direction;
        control->fric2.speed_mps = control->fric2.speed_rpm * SHOOT_FRIC_RPM_TO_MPS;
    }

    if (control->fric3.measure != NULL)
    {
        control->fric3.speed_rpm = (float)control->fric3.measure->speed_rpm * control->fric3.direction;
        control->fric3.speed_mps = control->fric3.speed_rpm * SHOOT_FRIC_RPM_TO_MPS;
    }

    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);
    shoot_task_motor_update_current_physics(&control->fric3);
}

/**
 * @brief 摩擦轮闭环控制。
 * @param control 射击任务控制对象指针。
 */
static void shoot_task_control_friction(shoot_task_control_t *control)
{
    uint32_t now;

    if (control == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    if (!shoot_task_motor_ready(&control->fric1, now) ||
        !shoot_task_motor_ready(&control->fric2, now) ||
        !shoot_task_motor_ready(&control->fric3, now))
    {
        /* 任一路反馈超时或过温时，整体停机保护。 */
        shoot_task_stop_friction(control);
        return;
    }

    if (control->last_mode != SHOOT_TASK_READY_FRIC)
    {
        /* 刚切入摩擦轮工作态时热启动重置 ADRC，减小切换冲击。 */
        shoot_task_motor_hot_reset(&control->fric1);
        shoot_task_motor_hot_reset(&control->fric2);
        shoot_task_motor_hot_reset(&control->fric3);
        control->fric1.ff_ticks = 0U;
        control->fric1.ff_cooldown_ticks = 0U;
        control->fric1.ff_current = 0;
        control->fric2.ff_ticks = 0U;
        control->fric2.ff_cooldown_ticks = 0U;
        control->fric2.ff_current = 0;
        control->fric3.ff_ticks = 0U;
        control->fric3.ff_cooldown_ticks = 0U;
        control->fric3.ff_current = 0;
        shoot_task_update_history(control);
    }

    /* 三路摩擦轮统一给定目标转速。 */
    control->fric1.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;
    control->fric2.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;
    control->fric3.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;

    /* 计算各电机闭环输出电流，单位 A。 */
    control->fric1.give_current_a = shoot_task_motor_calc(&control->fric1, control->fric1.speed_set_rpm);
    control->fric2.give_current_a = shoot_task_motor_calc(&control->fric2, control->fric2.speed_set_rpm);
    control->fric3.give_current_a = shoot_task_motor_calc(&control->fric3, control->fric3.speed_set_rpm);

    if ((control->fric1.ff_cooldown_ticks == 0U) &&
        shoot_task_motor_should_trigger_feedforward(&control->fric1,
                                                    SHOOT_FRIC1_FF_TRIGGER_DROP_RPM,
                                                    SHOOT_FRIC1_FF_MIN_SPEED_RATIO))
    {
        control->fric1.ff_current = SHOOT_FRIC1_FF_CURRENT;
        control->fric1.ff_ticks = SHOOT_FRIC1_FF_DURATION_MS;
        control->fric1.ff_cooldown_ticks = SHOOT_FRIC1_FF_COOLDOWN_MS;
    }

    if ((control->fric2.ff_cooldown_ticks == 0U) &&
        shoot_task_motor_should_trigger_feedforward(&control->fric2,
                                                    SHOOT_FRIC2_FF_TRIGGER_DROP_RPM,
                                                    SHOOT_FRIC2_FF_MIN_SPEED_RATIO))
    {
        control->fric2.ff_current = SHOOT_FRIC2_FF_CURRENT;
        control->fric2.ff_ticks = SHOOT_FRIC2_FF_DURATION_MS;
        control->fric2.ff_cooldown_ticks = SHOOT_FRIC2_FF_COOLDOWN_MS;
    }

    if ((control->fric3.ff_cooldown_ticks == 0U) &&
        shoot_task_motor_should_trigger_feedforward(&control->fric3,
                                                    SHOOT_FRIC3_FF_TRIGGER_DROP_RPM,
                                                    SHOOT_FRIC3_FF_MIN_SPEED_RATIO))
    {
        control->fric3.ff_current = SHOOT_FRIC3_FF_CURRENT;
        control->fric3.ff_ticks = SHOOT_FRIC3_FF_DURATION_MS;
        control->fric3.ff_cooldown_ticks = SHOOT_FRIC3_FF_COOLDOWN_MS;
    }

    if (control->fric1.ff_ticks > 0U)
    {
        shoot_task_motor_apply_feedforward(&control->fric1);
        control->fric1.ff_ticks--;
    }
    if (control->fric2.ff_ticks > 0U)
    {
        shoot_task_motor_apply_feedforward(&control->fric2);
        control->fric2.ff_ticks--;
    }
    if (control->fric3.ff_ticks > 0U)
    {
        shoot_task_motor_apply_feedforward(&control->fric3);
        control->fric3.ff_ticks--;
    }

    if (control->fric1.ff_cooldown_ticks > 0U)
    {
        control->fric1.ff_cooldown_ticks--;
    }
    if (control->fric2.ff_cooldown_ticks > 0U)
    {
        control->fric2.ff_cooldown_ticks--;
    }
    if (control->fric3.ff_cooldown_ticks > 0U)
    {
        control->fric3.ff_cooldown_ticks--;
    }

    shoot_task_motor_finalize_current(&control->fric1);
    shoot_task_motor_finalize_current(&control->fric2);
    shoot_task_motor_finalize_current(&control->fric3);

    /* 将三路电流打包后通过 CAN 下发。 */
    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);
    shoot_task_motor_update_current_physics(&control->fric3);

    shoot_task_send_friction_current(control->fric1.give_current,
                                     control->fric2.give_current,
                                     control->fric3.give_current);
    shoot_task_update_bullet_speed_estimate(control);
    shoot_task_update_history(control);
}

/**
 * @brief 停止摩擦轮并清理控制输出。
 * @param control 射击任务控制对象指针。
 */
static void shoot_task_stop_friction(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    /* 清零目标转速与输出电流。 */
    control->fric1.speed_set_rpm = 0.0f;
    control->fric2.speed_set_rpm = 0.0f;
    control->fric3.speed_set_rpm = 0.0f;
    control->fric1.give_current = 0;
    control->fric2.give_current = 0;
    control->fric3.give_current = 0;
    control->fric1.give_current_a = 0.0f;
    control->fric2.give_current_a = 0.0f;
    control->fric3.give_current_a = 0.0f;
    shoot_task_motor_update_current_physics(&control->fric1);
    shoot_task_motor_update_current_physics(&control->fric2);
    shoot_task_motor_update_current_physics(&control->fric3);
    control->fric1.ff_ticks = 0U;
    control->fric1.ff_cooldown_ticks = 0U;
    control->fric1.ff_current = 0;
    control->fric2.ff_ticks = 0U;
    control->fric2.ff_cooldown_ticks = 0U;
    control->fric2.ff_current = 0;
    control->fric3.ff_ticks = 0U;
    control->fric3.ff_cooldown_ticks = 0U;
    control->fric3.ff_current = 0;
    control->bullet_speed_est_active = false;
    control->bullet_speed_est_ticks = 0U;

    if (control->last_mode != SHOOT_TASK_STOP)
    {
        /* 从运行态切回停止态时重置控制器内部状态。 */
        shoot_task_motor_reset(&control->fric1);
        shoot_task_motor_reset(&control->fric2);
        shoot_task_motor_reset(&control->fric3);
    }

    /* 向驱动发送零电流，确保摩擦轮停机。 */
    shoot_task_send_friction_current(0, 0, 0);
}

/**
 * @brief 发送三路摩擦轮电流指令。
 * @param fric1_current 摩擦轮 1 电流，单位 mA。
 * @param fric2_current 摩擦轮 2 电流，单位 mA。
 * @param fric3_current 摩擦轮 3 电流，单位 mA。
 */
static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current, int16_t fric3_current)
{
    uint8_t data[8];
    int16_t fric1_cmd = shoot_task_current_ma_to_esc_cmd(fric1_current);
    int16_t fric2_cmd = shoot_task_current_ma_to_esc_cmd(fric2_current);
    int16_t fric3_cmd = shoot_task_current_ma_to_esc_cmd(fric3_current);

    /* 按高字节在前的格式打包三路 16 位电调原始电流命令。 */
    data[0] = (uint8_t)((uint16_t)fric1_cmd >> 8);
    data[1] = (uint8_t)fric1_cmd;
    data[2] = (uint8_t)((uint16_t)fric2_cmd >> 8);
    data[3] = (uint8_t)fric2_cmd;
    data[4] = (uint8_t)((uint16_t)fric3_cmd >> 8);
    data[5] = (uint8_t)fric3_cmd;
    data[6] = 0U;
    data[7] = 0U;

    /* 通过 FDCAN2 向摩擦轮电调广播控制帧。 */
    canx_send_data(&hfdcan2, SHOOT_FRICTION_CMD_ID, data, 8U);
}

/**
 * @brief 初始化单个摩擦轮电机控制对象。
 * @param motor 电机控制对象指针。
 * @param measure 电机反馈数据指针。
 * @param direction 电机方向系数，通常为 1 或 -1。
 */
static void shoot_task_motor_init(shoot_task_motor_t *motor,
                                  const motor_measure_t *measure,
                                  float direction,
                                  float b0,
                                  float response_time_s,
                                  float observer_ratio,
                                  float output_rate_limit)
{
    adrc_param_t param;

    if (motor == NULL)
    {
        return;
    }

    /* 清零电机运行状态并绑定反馈源。 */
    memset(motor, 0, sizeof(*motor));
    motor->measure = measure;
    motor->direction = direction;

    /* 配置速度环 ADRC 参数。 */
    param.sample_time_s = (float)SHOOT_CONTROL_TIME * 0.001f;
    param.b0 = b0;
    param.controller_bandwidth = 5.0f / response_time_s;
    param.observer_bandwidth_ratio = observer_ratio;
    param.tracking_gain = 0.0f;
    param.max_out = SHOOT_FRIC_MAX_CURRENT;
    param.output_rate_limit = output_rate_limit;
    param.error_linear_zone = SHOOT_FRIC_ERROR_LINEAR_ZONE;
    param.alpha1 = SHOOT_FRIC_ALPHA1;
    param.alpha2 = SHOOT_FRIC_ALPHA2;

    /* 初始化并复位速度控制器。 */
    ADRC_init(&motor->speed_adrc, &param);
    ADRC_reset(&motor->speed_adrc, 0.0f, 0.0f);
}

/**
 * @brief 常规复位摩擦轮 ADRC 控制器。
 * @param motor 电机控制对象指针。
 */
static void shoot_task_motor_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    /* 以当前转速为初值复位观测器，避免停机后状态残留。 */
    ADRC_reset(&motor->speed_adrc, motor->speed_rpm, 0.0f);
}

/**
 * @brief 热启动复位摩擦轮 ADRC 控制器。
 * @param motor 电机控制对象指针。
 */
static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    /* 带入当前速度、目标速度和当前输出，实现平滑切入闭环。 */
    ADRC_hot_reset(&motor->speed_adrc,
                   motor->speed_rpm,
                   SHOOT_FRIC_TARGET_SPEED_RPM,
                   motor->give_current_a * motor->direction);
}

static float shoot_task_limit_current_a(float current_a)
{
    if (current_a > SHOOT_FRIC_MAX_CURRENT)
    {
        current_a = SHOOT_FRIC_MAX_CURRENT;
    }
    else if (current_a < -SHOOT_FRIC_MAX_CURRENT)
    {
        current_a = -SHOOT_FRIC_MAX_CURRENT;
    }

    return current_a;
}

static int16_t shoot_task_current_a_to_current_ma(float current_a)
{
    float current_ma;

    current_ma = current_a * SHOOT_FRIC_MA_PER_A;
    return (int16_t)((current_ma >= 0.0f) ? (current_ma + 0.5f) : (current_ma - 0.5f));
}

static float shoot_task_current_ma_to_current_a(int16_t current_ma)
{
    return (float)current_ma / SHOOT_FRIC_MA_PER_A;
}

static int16_t shoot_task_current_ma_to_esc_cmd(int16_t current_ma)
{
    const float current_a = shoot_task_current_ma_to_current_a(current_ma);
    const float current_cmd = (current_a / SHOOT_FRIC_CURRENT_FULL_SCALE_A) *
                              SHOOT_FRIC_CURRENT_CMD_FULL_SCALE;

    return (int16_t)((current_cmd >= 0.0f) ? (current_cmd + 0.5f) : (current_cmd - 0.5f));
}

static float shoot_task_feedback_cmd_to_current_a(int16_t current_cmd)
{
    return ((float)current_cmd / SHOOT_FRIC_CURRENT_CMD_FULL_SCALE) * SHOOT_FRIC_CURRENT_FULL_SCALE_A;
}

static float shoot_task_current_a_to_input_torque_nm(float current_a)
{
    const float output_torque_nm = current_a * SHOOT_FRIC_OUTPUT_TORQUE_CONSTANT_NM_PER_A;

    return output_torque_nm / SHOOT_FRIC_REDUCTION_RATIO;
}

static void shoot_task_motor_update_current_physics(shoot_task_motor_t *motor)
{
    int16_t given_current = 0;

    if (motor == NULL)
    {
        return;
    }

    if (motor->measure != NULL)
    {
        given_current = motor->measure->given_current;
    }

    motor->given_current = given_current;
    motor->given_current_a = shoot_task_feedback_cmd_to_current_a(given_current);
    motor->give_input_torque_nm = shoot_task_current_a_to_input_torque_nm(motor->give_current_a);
    motor->given_input_torque_nm = shoot_task_current_a_to_input_torque_nm(motor->given_current_a);
}

static void shoot_task_motor_finalize_current(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->give_current_a = shoot_task_limit_current_a(motor->give_current_a);
    motor->give_current = shoot_task_current_a_to_current_ma(motor->give_current_a);
}

/**
 * @brief 计算单个摩擦轮所需输出电流。
 * @param motor 电机控制对象指针。
 * @param target_speed_rpm 目标转速，单位 rpm。
 * @return 电流输出值，单位 A。
 */
static float shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm)
{
    float current_output_a;

    if (motor == NULL)
    {
        return 0;
    }

    /* 先按统一正方向计算控制量，再恢复到电机实际安装方向。 */
    current_output_a = ADRC_Calc(&motor->speed_adrc, motor->speed_rpm, target_speed_rpm);
    current_output_a *= motor->direction;

    return current_output_a;
}

static bool shoot_task_motor_should_trigger_feedforward(const shoot_task_motor_t *motor,
                                                        float trigger_drop_rpm,
                                                        float min_speed_ratio)
{
#if (SHOOT_FRIC_FF_ENABLE == 0)
    (void)motor;
    (void)trigger_drop_rpm;
    (void)min_speed_ratio;
    return false;
#else
    float min_speed;
    float speed_drop;

    if (motor == NULL)
    {
        return false;
    }

    min_speed = SHOOT_FRIC_TARGET_SPEED_RPM * min_speed_ratio;
    speed_drop = motor->prev_speed_rpm - motor->speed_rpm;

    return ((motor->prev_speed_rpm >= min_speed) &&
            (speed_drop >= trigger_drop_rpm));
#endif
}

static void shoot_task_motor_apply_feedforward(shoot_task_motor_t *motor)
{
#if (SHOOT_FRIC_FF_ENABLE != 0)
    if (motor == NULL)
    {
        return;
    }

    {
        motor->give_current_a += (float)motor->ff_current * motor->direction;
        motor->give_current_a = shoot_task_limit_current_a(motor->give_current_a);
    }
#else
    (void)motor;
#endif
}


static void shoot_task_update_history(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->fric1.prev_speed_rpm = control->fric1.last_speed_rpm;
    control->fric2.prev_speed_rpm = control->fric2.last_speed_rpm;
    control->fric3.prev_speed_rpm = control->fric3.last_speed_rpm;
    control->fric1.last_speed_rpm = control->fric1.speed_rpm;
    control->fric2.last_speed_rpm = control->fric2.speed_rpm;
    control->fric3.last_speed_rpm = control->fric3.speed_rpm;
}

static void shoot_task_update_bullet_speed_estimate(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    if (!control->bullet_speed_est_active &&
        shoot_task_should_start_bullet_speed_estimate(control))
    {
        shoot_task_start_bullet_speed_estimate(control);
    }

    if (!control->bullet_speed_est_active)
    {
        return;
    }

    control->bullet_speed_min_fric1_rpm =
        shoot_task_min_float(control->bullet_speed_min_fric1_rpm,
                             control->fric1.speed_rpm);
    control->bullet_speed_min_fric2_rpm =
        shoot_task_min_float(control->bullet_speed_min_fric2_rpm,
                             control->fric2.speed_rpm);
    control->bullet_speed_min_fric3_rpm =
        shoot_task_min_float(control->bullet_speed_min_fric3_rpm,
                             control->fric3.speed_rpm);

    if (control->bullet_speed_est_ticks > 0U)
    {
        control->bullet_speed_est_ticks--;
    }

    if (control->bullet_speed_est_ticks == 0U)
    {
        float speed_drop_rpm;

        control->bullet_speed_min_avg_rpm =
            shoot_task_avg3(control->bullet_speed_min_fric1_rpm,
                            control->bullet_speed_min_fric2_rpm,
                            control->bullet_speed_min_fric3_rpm);
        speed_drop_rpm =
            control->bullet_speed_start_avg_rpm -
            control->bullet_speed_min_avg_rpm;
        if (speed_drop_rpm < 0.0f)
        {
            speed_drop_rpm = 0.0f;
        }

        control->estimated_bullet_speed_mps =
            speed_drop_rpm * SHOOT_BULLET_SPEED_EST_COEFF_MPS_PER_RPM;
        control->bullet_speed_est_active = false;
    }
}

static bool shoot_task_should_start_bullet_speed_estimate(const shoot_task_control_t *control)
{
    float stable_speed_avg_rpm;

    if (control == NULL)
    {
        return false;
    }

    stable_speed_avg_rpm =
        shoot_task_avg3(control->fric1.last_speed_rpm,
                        control->fric2.last_speed_rpm,
                        control->fric3.last_speed_rpm);
    if (stable_speed_avg_rpm <
        SHOOT_FRIC_TARGET_SPEED_RPM * SHOOT_BULLET_SPEED_EST_MIN_SPEED_RATIO)
    {
        return false;
    }

    return (((control->fric1.last_speed_rpm - control->fric1.speed_rpm) >=
             SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM) ||
            ((control->fric2.last_speed_rpm - control->fric2.speed_rpm) >=
             SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM) ||
            ((control->fric3.last_speed_rpm - control->fric3.speed_rpm) >=
             SHOOT_BULLET_SPEED_EST_TRIGGER_DROP_RPM));
}

static void shoot_task_start_bullet_speed_estimate(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->bullet_speed_est_active = true;
    control->bullet_speed_est_ticks =
        shoot_task_ms_to_ticks(SHOOT_BULLET_SPEED_EST_WINDOW_MS);
    control->bullet_speed_start_avg_rpm =
        shoot_task_avg3(control->fric1.last_speed_rpm,
                        control->fric2.last_speed_rpm,
                        control->fric3.last_speed_rpm);
    control->bullet_speed_min_fric1_rpm = control->fric1.speed_rpm;
    control->bullet_speed_min_fric2_rpm = control->fric2.speed_rpm;
    control->bullet_speed_min_fric3_rpm = control->fric3.speed_rpm;
    control->bullet_speed_min_avg_rpm =
        shoot_task_avg3(control->bullet_speed_min_fric1_rpm,
                        control->bullet_speed_min_fric2_rpm,
                        control->bullet_speed_min_fric3_rpm);
}

static uint16_t shoot_task_ms_to_ticks(uint16_t ms)
{
    uint16_t ticks;

    ticks = (uint16_t)((ms + SHOOT_CONTROL_TIME - 1U) / SHOOT_CONTROL_TIME);
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

static float shoot_task_avg3(float a, float b, float c)
{
    return (a + b + c) / 3.0f;
}

static float shoot_task_min_float(float a, float b)
{
    return (a < b) ? a : b;
}

static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now)
{
    if (motor == NULL || motor->measure == NULL)
    {
        return false;
    }

    /* 反馈超时说明电机离线或总线异常。 */
    if ((now - motor->measure->last_fdb_time) > SHOOT_FRIC_FDB_TIMEOUT)
    {
        return false;
    }

    /* 电机温度超限时禁止继续运行。 */
    if (motor->measure->temperate >= SHOOT_FRIC_TEMP_LIMIT)
    {
        return false;
    }

    return true;
}
