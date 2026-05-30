#ifndef CHASSIS_AI_POWER_PREDICT_H
#define CHASSIS_AI_POWER_PREDICT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "NanoEdgeAI.h"
#include "chassis_task.h"
#include "struct_typedef.h"

#define CHASSIS_AI_POWER_AXIS_COUNT 2U

typedef struct
{
    float input_signal[CHASSIS_MODULE_NUM][NEAI_INPUT_SIGNAL_LENGTH * NEAI_INPUT_AXIS_NUMBER];
    fp32 predicted_power[CHASSIS_MODULE_NUM];
    fp32 total_predicted_power;
    uint16_t sample_count[CHASSIS_MODULE_NUM];
    uint8_t ready[CHASSIS_MODULE_NUM];
    uint8_t initialized;
    int last_state;
} chassis_ai_power_predict_control_t;

extern chassis_ai_power_predict_control_t chassis_ai_power_predict_control;

void chassis_ai_power_predict_init(void);
void chassis_ai_power_predict_update(const chassis_move_t *chassis_move, uint8_t motor_idx);
fp32 chassis_ai_power_predict_get_power(void);
uint8_t chassis_ai_power_predict_is_ready(void);
int chassis_ai_power_predict_get_state(void);
uint16_t chassis_ai_power_predict_get_sample_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_AI_POWER_PREDICT_H */
