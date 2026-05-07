#include "shoot_task.h"

#include <string.h>

#include "fdcan.h"
#include "gimbal_behaviour.h"

#define SHOOT_FRICTION_CMD_ID 0x200U

extern motor_measure_t DJI_MOTOR_MEASURE[8];

shoot_task_control_t shoot_task_control;

static void shoot_task_init_control(shoot_task_control_t *control);
static void shoot_task_set_mode(shoot_task_control_t *control);
static void shoot_task_update_feedback(shoot_task_control_t *control);
static void shoot_task_control_friction(shoot_task_control_t *control);
static void shoot_task_stop_friction(shoot_task_control_t *control);
static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current, int16_t fric3_current);
static void shoot_task_motor_init(shoot_task_motor_t *motor, const motor_measure_t *measure, float direction);
static void shoot_task_motor_reset(shoot_task_motor_t *motor);
static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor);
static int16_t shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm);
static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now);

void shoot_task_init(void)
{
    shoot_task_init_control(&shoot_task_control);
}

void shoot_task_loop(void)
{
    shoot_task_set_mode(&shoot_task_control);
    shoot_task_update_feedback(&shoot_task_control);

    if (shoot_task_control.mode == SHOOT_TASK_READY_FRIC)
    {
        shoot_task_control_friction(&shoot_task_control);
    }
    else
    {
        shoot_task_stop_friction(&shoot_task_control);
    }

    shoot_task_control.last_mode = shoot_task_control.mode;
}

static void shoot_task_init_control(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    memset(control, 0, sizeof(*control));
    control->rc = get_remote_control_point();
    control->mode = SHOOT_TASK_STOP;
    control->last_mode = SHOOT_TASK_STOP;

    shoot_task_motor_init(&control->fric1, &DJI_MOTOR_MEASURE[0], SHOOT_FRIC1_DIRECTION);
    shoot_task_motor_init(&control->fric2, &DJI_MOTOR_MEASURE[1], SHOOT_FRIC2_DIRECTION);
    shoot_task_motor_init(&control->fric3, &DJI_MOTOR_MEASURE[2], SHOOT_FRIC3_DIRECTION);
}

static void shoot_task_set_mode(shoot_task_control_t *control)
{
    static uint16_t last_key_value = 0U;
    uint16_t pressed_keys;
    int shoot_switch;

    if (control == NULL || control->rc == NULL)
    {
        return;
    }

    pressed_keys = (uint16_t)(control->rc->key.v & (uint16_t)(~last_key_value));
    last_key_value = control->rc->key.v;
    shoot_switch = control->rc->rc.s[SHOOT_RC_MODE_CHANNEL];

    if ((pressed_keys & KEY_PRESSED_OFFSET_R) != 0U)
    {
        control->friction_enable = true;
    }

    if ((pressed_keys & KEY_PRESSED_OFFSET_G) != 0U)
    {
        control->friction_enable = false;
    }

    if (switch_is_down(shoot_switch))
    {
        control->friction_enable = true;
    }
    else if (switch_is_mid(shoot_switch))
    {
        control->friction_enable = false;
    }

    if (gimbal_cmd_to_shoot_stop())
    {
        control->mode = SHOOT_TASK_STOP;
    }
    else if (control->friction_enable)
    {
        control->mode = SHOOT_TASK_READY_FRIC;
    }
    else
    {
        control->mode = SHOOT_TASK_STOP;
    }
}

static void shoot_task_update_feedback(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    if (control->fric1.measure != NULL)
    {
        control->fric1.speed_rpm = (float)control->fric1.measure->speed_rpm * control->fric1.direction;
    }

    if (control->fric2.measure != NULL)
    {
        control->fric2.speed_rpm = (float)control->fric2.measure->speed_rpm * control->fric2.direction;
    }

    if (control->fric3.measure != NULL)
    {
        control->fric3.speed_rpm = (float)control->fric3.measure->speed_rpm * control->fric3.direction;
    }
}

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
        shoot_task_stop_friction(control);
        return;
    }

    if (control->last_mode != SHOOT_TASK_READY_FRIC)
    {
        shoot_task_motor_hot_reset(&control->fric1);
        shoot_task_motor_hot_reset(&control->fric2);
        shoot_task_motor_hot_reset(&control->fric3);
    }

    control->fric1.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;
    control->fric2.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;
    control->fric3.speed_set_rpm = SHOOT_FRIC_TARGET_SPEED_RPM;

    control->fric1.give_current = shoot_task_motor_calc(&control->fric1, control->fric1.speed_set_rpm);
    control->fric2.give_current = shoot_task_motor_calc(&control->fric2, control->fric2.speed_set_rpm);
    control->fric3.give_current = shoot_task_motor_calc(&control->fric3, control->fric3.speed_set_rpm);

    shoot_task_send_friction_current(control->fric1.give_current,
                                     control->fric2.give_current,
                                     control->fric3.give_current);
}

static void shoot_task_stop_friction(shoot_task_control_t *control)
{
    if (control == NULL)
    {
        return;
    }

    control->fric1.speed_set_rpm = 0.0f;
    control->fric2.speed_set_rpm = 0.0f;
    control->fric3.speed_set_rpm = 0.0f;
    control->fric1.give_current = 0;
    control->fric2.give_current = 0;
    control->fric3.give_current = 0;

    if (control->last_mode != SHOOT_TASK_STOP)
    {
        shoot_task_motor_reset(&control->fric1);
        shoot_task_motor_reset(&control->fric2);
        shoot_task_motor_reset(&control->fric3);
    }

    shoot_task_send_friction_current(0, 0, 0);
}

static void shoot_task_send_friction_current(int16_t fric1_current, int16_t fric2_current, int16_t fric3_current)
{
    uint8_t data[8];

    data[0] = (uint8_t)((uint16_t)fric1_current >> 8);
    data[1] = (uint8_t)fric1_current;
    data[2] = (uint8_t)((uint16_t)fric2_current >> 8);
    data[3] = (uint8_t)fric2_current;
    data[4] = (uint8_t)((uint16_t)fric3_current >> 8);
    data[5] = (uint8_t)fric3_current;
    data[6] = 0U;
    data[7] = 0U;

    canx_send_data(&hfdcan2, SHOOT_FRICTION_CMD_ID, data, 8U);
}

static void shoot_task_motor_init(shoot_task_motor_t *motor, const motor_measure_t *measure, float direction)
{
    adrc_param_t param;

    if (motor == NULL)
    {
        return;
    }

    memset(motor, 0, sizeof(*motor));
    motor->measure = measure;
    motor->direction = direction;

    param.sample_time_s = (float)SHOOT_CONTROL_TIME * 0.001f;
    param.b0 = SHOOT_FRIC_B0;
    param.controller_bandwidth = 5.0f / SHOOT_FRIC_RESPONSE_TIME_S;
    param.observer_bandwidth_ratio = SHOOT_FRIC_OBSERVER_RATIO;
    param.tracking_gain = 0.0f;
    param.max_out = SHOOT_FRIC_MAX_CURRENT;
    param.output_rate_limit = SHOOT_FRIC_OUTPUT_RATE_LIMIT;
    param.error_linear_zone = SHOOT_FRIC_ERROR_LINEAR_ZONE;
    param.alpha1 = SHOOT_FRIC_ALPHA1;
    param.alpha2 = SHOOT_FRIC_ALPHA2;

    ADRC_init(&motor->speed_adrc, &param);
    ADRC_reset(&motor->speed_adrc, 0.0f, 0.0f);
}

static void shoot_task_motor_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    ADRC_reset(&motor->speed_adrc, motor->speed_rpm, 0.0f);
}

static void shoot_task_motor_hot_reset(shoot_task_motor_t *motor)
{
    if (motor == NULL)
    {
        return;
    }

    ADRC_hot_reset(&motor->speed_adrc,
                   motor->speed_rpm,
                   SHOOT_FRIC_TARGET_SPEED_RPM,
                   (float)motor->give_current * motor->direction);
}

static int16_t shoot_task_motor_calc(shoot_task_motor_t *motor, float target_speed_rpm)
{
    float current_output;

    if (motor == NULL)
    {
        return 0;
    }

    current_output = ADRC_Calc(&motor->speed_adrc, motor->speed_rpm, target_speed_rpm);
    current_output *= motor->direction;

    if (current_output > SHOOT_FRIC_MAX_CURRENT)
    {
        current_output = SHOOT_FRIC_MAX_CURRENT;
    }
    else if (current_output < -SHOOT_FRIC_MAX_CURRENT)
    {
        current_output = -SHOOT_FRIC_MAX_CURRENT;
    }

    return (int16_t)current_output;
}

static bool shoot_task_motor_ready(const shoot_task_motor_t *motor, uint32_t now)
{
    if (motor == NULL || motor->measure == NULL)
    {
        return false;
    }

    if ((now - motor->measure->last_fdb_time) > SHOOT_FRIC_FDB_TIMEOUT)
    {
        return false;
    }

    if (motor->measure->temperate >= SHOOT_FRIC_TEMP_LIMIT)
    {
        return false;
    }

    return true;
}
