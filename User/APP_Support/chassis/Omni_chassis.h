#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H

#include "chassis_task.h"

void chassis_init(chassis_move_t *chassis_move_init);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_set_contorl(chassis_move_t *chassis_move_control);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void chassis_send_cmd(chassis_move_t *chassis_move_send);

#endif
