#ifndef CHASSIS_AI_POWER_PREDICT_H
#define CHASSIS_AI_POWER_PREDICT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "chassis_task.h"
#include "struct_typedef.h"

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
