#include "chassis_ai_power_predict.h"
#include "NanoEdgeAI.h"

#define CHASSIS_AI_CURRENT_CMD_TO_A (20.0f / 16384.0f)
#define CHASSIS_AI_AXIS_COUNT 2U

#if NEAI_INPUT_AXIS_NUMBER != CHASSIS_AI_AXIS_COUNT
#error "NanoEdgeAI axis count must be 2: current_a, speed_rpm."
#endif

static float s_input_signal[NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];
static fp32 s_predicted_power = 0.0f;
static uint16_t s_sample_count = 0U;
static uint8_t s_ready = 0U;
static uint8_t s_initialized = 0U;
static int s_last_state = NEAI_NOT_INITIALIZED;

static fp32 chassis_ai_current_cmd_to_a(int16_t current_cmd)
{
    return (fp32)current_cmd * CHASSIS_AI_CURRENT_CMD_TO_A;
}

void chassis_ai_power_predict_init(void)
{
    enum neai_state state;

    state = neai_extrapolation_init();
    s_last_state = (int)state;
    if (state == NEAI_OK)
    {
        s_initialized = 1U;
        s_sample_count = 0U;
        s_ready = 0U;
        s_predicted_power = 0.0f;
    }
}

void chassis_ai_power_predict_update(const chassis_move_t *chassis_move, uint8_t motor_idx)
{
    const chassis_motor_t *motor;
    uint16_t offset;
    fp32 current_a;
    fp32 speed_rpm;
    float prediction = 0.0f;
    enum neai_state state;

    if (chassis_move == NULL || motor_idx >= CHASSIS_MODULE_NUM)
    {
        s_last_state = (int)NEAI_INVALID_PARAM;
        return;
    }

    if (s_initialized == 0U)
    {
        chassis_ai_power_predict_init();
        if (s_initialized == 0U)
        {
            return;
        }
    }

    motor = &chassis_move->chassis_3508[motor_idx];
    current_a = chassis_ai_current_cmd_to_a(motor->give_current);
    speed_rpm = 0.0f;
    if (motor->chassis_motor_measure != NULL)
    {
        speed_rpm = (fp32)motor->chassis_motor_measure->speed_rpm;
    }

    offset = (uint16_t)(s_sample_count * CHASSIS_AI_AXIS_COUNT);
    s_input_signal[offset] = current_a;
    s_input_signal[offset + 1U] = speed_rpm;
    s_sample_count++;

    if (s_sample_count < NEAI_INPUT_SIGNAL_LENGTH)
    {
        return;
    }

    s_sample_count = 0U;
    state = neai_extrapolation(s_input_signal, &prediction);
    s_last_state = (int)state;
    if (state == NEAI_OK)
    {
        s_predicted_power = (fp32)prediction;
        s_ready = 1U;
    }
}

fp32 chassis_ai_power_predict_get_power(void)
{
    return s_predicted_power;
}

uint8_t chassis_ai_power_predict_is_ready(void)
{
    return s_ready;
}

int chassis_ai_power_predict_get_state(void)
{
    return s_last_state;
}

uint16_t chassis_ai_power_predict_get_sample_count(void)
{
    return s_sample_count;
}
