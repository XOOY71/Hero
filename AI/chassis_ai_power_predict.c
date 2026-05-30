#include "chassis_ai_power_predict.h"
#include "NanoEdgeAI.h"

#if NEAI_INPUT_AXIS_NUMBER != CHASSIS_AI_POWER_AXIS_COUNT
#error "NanoEdgeAI axis count must be 2: current_a, speed_rad_s."
#endif

chassis_ai_power_predict_control_t chassis_ai_power_predict_control =
{
    .last_state = NEAI_NOT_INITIALIZED,
};

static void chassis_ai_power_predict_sum(void)
{
    fp32 total = 0.0f;

    for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
    {
        total += chassis_ai_power_predict_control.predicted_power[i];
    }

    chassis_ai_power_predict_control.total_predicted_power = total;
}

static void chassis_ai_power_predict_push_sample(uint8_t motor_idx, fp32 current_a, fp32 speed_rad_s)
{
    float *signal = chassis_ai_power_predict_control.input_signal[motor_idx];
    uint16_t sample_count = chassis_ai_power_predict_control.sample_count[motor_idx];
    uint16_t offset;

    if (sample_count < NEAI_INPUT_SIGNAL_LENGTH)
    {
        offset = (uint16_t)(sample_count * CHASSIS_AI_POWER_AXIS_COUNT);
        chassis_ai_power_predict_control.sample_count[motor_idx]++;
    }
    else
    {
        for (uint16_t i = 0U; i < (NEAI_INPUT_SIGNAL_LENGTH - 1U) * CHASSIS_AI_POWER_AXIS_COUNT; i++)
        {
            signal[i] = signal[i + CHASSIS_AI_POWER_AXIS_COUNT];
        }
        offset = (uint16_t)((NEAI_INPUT_SIGNAL_LENGTH - 1U) * CHASSIS_AI_POWER_AXIS_COUNT);
    }

    signal[offset] = current_a;
    signal[offset + 1U] = speed_rad_s;
}

void chassis_ai_power_predict_init(void)
{
    enum neai_state state;

    state = neai_extrapolation_init();
    chassis_ai_power_predict_control.last_state = (int)state;
    if (state == NEAI_OK)
    {
        chassis_ai_power_predict_control.initialized = 1U;
        chassis_ai_power_predict_control.total_predicted_power = 0.0f;

        for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
        {
            chassis_ai_power_predict_control.sample_count[i] = 0U;
            chassis_ai_power_predict_control.ready[i] = 0U;
            chassis_ai_power_predict_control.predicted_power[i] = 0.0f;
        }
    }
}

void chassis_ai_power_predict_update(const chassis_move_t *chassis_move, uint8_t motor_idx)
{
    const chassis_motor_t *motor;
    fp32 current_a;
    fp32 speed_rad_s;
    float prediction = 0.0f;
    enum neai_state state;

    if (chassis_move == NULL || motor_idx >= CHASSIS_MODULE_NUM)
    {
        chassis_ai_power_predict_control.last_state = (int)NEAI_INVALID_PARAM;
        return;
    }

    if (chassis_ai_power_predict_control.initialized == 0U)
    {
        chassis_ai_power_predict_init();
        if (chassis_ai_power_predict_control.initialized == 0U)
        {
            return;
        }
    }

    motor = &chassis_move->chassis_3508[motor_idx];
    current_a = motor->given_current_a;
    speed_rad_s = motor->speed_rad_s;

    chassis_ai_power_predict_push_sample(motor_idx, current_a, speed_rad_s);

    if (chassis_ai_power_predict_control.sample_count[motor_idx] < NEAI_INPUT_SIGNAL_LENGTH)
    {
        return;
    }

    state = neai_extrapolation(chassis_ai_power_predict_control.input_signal[motor_idx], &prediction);
    chassis_ai_power_predict_control.last_state = (int)state;
    if (state == NEAI_OK)
    {
        chassis_ai_power_predict_control.predicted_power[motor_idx] = (fp32)prediction;
        chassis_ai_power_predict_control.ready[motor_idx] = 1U;
        chassis_ai_power_predict_sum();
    }
}

fp32 chassis_ai_power_predict_get_power(void)
{
    return chassis_ai_power_predict_control.total_predicted_power;
}

uint8_t chassis_ai_power_predict_is_ready(void)
{
    for (uint8_t i = 0U; i < CHASSIS_MODULE_NUM; i++)
    {
        if (chassis_ai_power_predict_control.ready[i] == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

int chassis_ai_power_predict_get_state(void)
{
    return chassis_ai_power_predict_control.last_state;
}

uint16_t chassis_ai_power_predict_get_sample_count(void)
{
    return chassis_ai_power_predict_control.sample_count[0];
}
