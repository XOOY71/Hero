#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "adrc.h"
#include "bsp_fdcan.h"
#include "project_config.h"
#include "remote_control.h"

#ifndef SHOOT_TASK_INIT_TIME
#define SHOOT_TASK_INIT_TIME 200U
#endif

#ifndef SHOOT_CONTROL_TIME
#define SHOOT_CONTROL_TIME 1U
#endif

#ifndef SHOOT_RC_MODE_CHANNEL
#define SHOOT_RC_MODE_CHANNEL 1
#endif

#ifndef SHOOT_FRIC_TARGET_SPEED_RPM
#define SHOOT_FRIC_TARGET_SPEED_RPM 5730
#endif

#ifndef SHOOT_FRIC_WHEEL_RADIUS_M
#define SHOOT_FRIC_WHEEL_RADIUS_M 0.02f
#endif

#ifndef SHOOT_FRIC_MAX_CURRENT
#define SHOOT_FRIC_MAX_CURRENT 16000
#endif

#ifndef SHOOT_FRIC_OUTPUT_RATE_LIMIT
#define SHOOT_FRIC_OUTPUT_RATE_LIMIT 40000
#endif

#ifndef SHOOT_FRIC_RESPONSE_TIME_S
#define SHOOT_FRIC_RESPONSE_TIME_S 0.05f
#endif

#ifndef SHOOT_FRIC_FEEDBACK_RANGE_RPM
#define SHOOT_FRIC_FEEDBACK_RANGE_RPM 7000
#endif

#ifndef SHOOT_FRIC_B0
#define SHOOT_FRIC_B0 30
#endif

#ifndef SHOOT_FRIC_OBSERVER_RATIO
#define SHOOT_FRIC_OBSERVER_RATIO 4
#endif

#ifndef SHOOT_FRIC_ERROR_LINEAR_ZONE
#define SHOOT_FRIC_ERROR_LINEAR_ZONE 80
#endif

#ifndef SHOOT_FRIC_ALPHA1
#define SHOOT_FRIC_ALPHA1 0.5f
#endif

#ifndef SHOOT_FRIC_ALPHA2
#define SHOOT_FRIC_ALPHA2 0.25f
#endif

#ifndef SHOOT_FRIC_FDB_TIMEOUT
#define SHOOT_FRIC_FDB_TIMEOUT 100U
#endif

#ifndef SHOOT_FRIC_TEMP_LIMIT
#define SHOOT_FRIC_TEMP_LIMIT 80U
#endif

#ifndef SHOOT_FRIC1_DIRECTION
#define SHOOT_FRIC1_DIRECTION 1
#endif

#ifndef SHOOT_FRIC2_DIRECTION
#define SHOOT_FRIC2_DIRECTION -1
#endif

#ifndef SHOOT_FRIC3_DIRECTION
#define SHOOT_FRIC3_DIRECTION 1
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
    float direction;
    int16_t give_current;
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
