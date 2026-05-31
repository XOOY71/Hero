#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
#include "pid.h"
#include "project_config.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define CHASSIS_TASK_INIT_TIME      357
#define CHASSIS_X_CHANNEL          1
#define CHASSIS_Y_CHANNEL          0
#define CHASSIS_WZ_CHANNEL         2
#define CHASSIS_MODE_CHANNEL       0

#define CHASSIS_VX_RC_SEN          0.004f
#define CHASSIS_VY_RC_SEN          0.004f

#define CHASSIS_ACCEL_X_NUM        0.1666666667f
#define CHASSIS_ACCEL_Y_NUM        0.3333333333f

#define CHASSIS_RC_DEADLINE        25

#define CHASSIS_CONTROL_TIME_MS    2
#define CHASSIS_CONTROL_TIME       0.002f
#define CHASSIS_CONTROL_FREQUENCE  500.0f

#define CHASSIS_AI_LOG_ENABLE      0
#define CHASSIS_AI_LOG_PERIOD_MS   10U

#define CHASSIS_FRONT_KEY          KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY           KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY           KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY          KEY_PRESSED_OFFSET_D

#define MAX_WHEEL_SPEED            3.0f

#define MAX_6020_CAN_CURRENT       16000.0f
#define MAX_3508_CAN_CURRENT       16000.0f
#define CHASSIS_CURRENT_CMD_FULL_SCALE 16384.0f
#define CHASSIS_CURRENT_FULL_SCALE_A   20.0f
#define CHASSIS_CURRENT_CMD_TO_A       (CHASSIS_CURRENT_FULL_SCALE_A / CHASSIS_CURRENT_CMD_FULL_SCALE)

#define M3508_RR                   19.20320855f
#define GM6020_Angle_Ratio         1303.63813886f
#define Wheel_Radius               0.0567f
#define Wheel_Perimeter            0.35625661f
#define MPS_to_RPM                 3234.16461897f
#define RPM_to_Icmd                1.55125592f
#define CHASSIS_RPM_TO_RAD_PER_SEC 0.104719755f

#define ROBOT_MASS                 11.0f
#define M3508_TORQUE_CONSTANT      0.3f
#define M3508_REDUCTION_RATIO      19.2032f
#define CHASSIS_EFFICIENCY         0.92f
#define CONTROL_PERIOD_MODEL       0.02f
#define CHASSIS_MAX_ACCEL          5.0f
#define CHASSIS_MAX_JERK           50.0f
#define M3508_MAX_CONT_TORQUE      2.8f
#define FRICTION_SPEED_BAND        0.1f
#define FRICTION_LINEAR_GAIN       4000.0f
#define FRICTION_CONSTANT_CURRENT  400.0f
#define TRACTION_ZERO_FORCE_THRESHOLD 1.0f
#define SPEED_HOLD_ERROR_THRESHOLD 0.1f
#define SPEED_HOLD_KP              600.0f

#define CHASSIS_ACCEL_FILTER_TAU   0.02f
#define CHASSIS_SPEED_PI_KP        1200.0f
#define CHASSIS_SPEED_PI_KI        40.0f
#define CHASSIS_SPEED_PI_MAX_OUT   2500.0f
#define CHASSIS_SPEED_PI_MAX_IOUT  800.0f
#define CHASSIS_FF_VISCOUS_GAIN    400.0f
#define CHASSIS_FF_COULOMB_CURRENT FRICTION_CONSTANT_CURRENT
#define CHASSIS_FF_COULOMB_SPEED_EPS 0.08f
#define CHASSIS_FF_STATIC_CURRENT  120.0f
#define CHASSIS_FF_STATIC_SPEED_EPS 0.05f
#define CHASSIS_WZ_MAX_SPEED       0.10f
#define CHASSIS_WZ_MAX_ACCEL       0.30f
#define CHASSIS_WZ_MAX_JERK        3.0f

#define M3505_MOTOR_SPEED_PID_KP       30000.0f
#define M3505_MOTOR_SPEED_PID_KI       10.0f
#define M3505_MOTOR_SPEED_PID_KD       0.0f
#define M3505_MOTOR_SPEED_RUN_MAX_OUT  5734
#define M3505_MOTOR_SPEED_PID_MAX_OUT  16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 100.0f

#define GM6020_MOTOR_ANGLE_PID_KP       160.0f
#define GM6020_MOTOR_ANGLE_PID_KI       0.01f
#define GM6020_MOTOR_ANGLE_PID_KD       0.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_OUT  120.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_IOUT 10.0f

#define YAW_RETURN_PID_KP       800.0f
#define YAW_RETURN_PID_KI       0.08f
#define YAW_RETURN_PID_KD       20.0f
#define YAW_RETURN_PID_MAX_OUT  600.0f
#define YAW_RETURN_PID_MAX_IOUT 60.0f

#define GM6020_MOTOR_SPEED_PID_KP       85.0f
#define GM6020_MOTOR_SPEED_PID_KI       0.0f
#define GM6020_MOTOR_SPEED_PID_KD       80.0f
#define GM6020_MOTOR_SPEED_PID_MAX_OUT  16000.0f
#define GM6020_MOTOR_SPEED_PID_MAX_IOUT 0.0f

#define CHASSIS_FOLLOW_GIMBAL_PID_KP       0.08f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI       0.002f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD       0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT  0.08f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.005f

#ifndef myabs
#define myabs(x) (((x) >= 0) ? (x) : (-(x)))
#endif

#define rc_deadband_limit(input, output, dealine)       \
{                                                       \
    if ((input) > (dealine) || (input) < -(dealine))    \
    {                                                   \
        (output) = (input);                             \
    }                                                   \
    else                                                \
    {                                                   \
        (output) = 0;                                   \
    }                                                   \
}

typedef enum
{
    CHASSIS_VECTOR_NO_MOVE = 0,
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_SPIN,
    CHASSIS_VECTOR_RETURN,
} chassis_mode_e;

typedef struct
{
    const MOTOR_MEASURE_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_rad_s;
    fp32 speed_set;
    fp32 angle;
    fp32 angle_set;
    int16_t give_current;
    fp32 given_current_a;
} chassis_motor_t;

typedef struct
{
    fp32 now[CHASSIS_MODULE_NUM];
    fp32 last[CHASSIS_MODULE_NUM];
    fp32 initial[CHASSIS_MODULE_NUM];
} wheel_angle_offset_t;

typedef struct
{
    const RC_ctrl_t *chassis_RC;
    const gimbal_motor_t *chassis_yaw_motor;
    const gimbal_motor_t *chassis_pitch_motor;
    const fp32 *chassis_INS_angle;

    chassis_mode_e chassis_mode;
    chassis_mode_e last_chassis_mode;

    chassis_motor_t chassis_3508[CHASSIS_MODULE_NUM];
    wheel_angle_offset_t wheel_angle_offset;

    fp32 model_3508_out[CHASSIS_MODULE_NUM];
    fp32 model_accel[CHASSIS_MODULE_NUM];
    fp32 model_last_speed_set[CHASSIS_MODULE_NUM];
    fp32 speed_pi_iout[CHASSIS_MODULE_NUM];
    fp32 ai_predicted_power;

    pid_type_def chassis_angle_pid;
    pid_type_def chas_return_pid;

    uint8_t chassis_return_flag;
    chassis_mode_e chassis_return_record;

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;

    fp32 vx;
    fp32 vy;
    fp32 wz;
    fp32 vx_set;
    fp32 vy_set;
    fp32 wz_set;
    fp32 last_vx_set;
    fp32 last_vy_set;
    fp32 last_wz_set;
    fp32 vx_plan;
    fp32 vy_plan;
    fp32 wz_plan;
    fp32 vx_plan_accel;
    fp32 vy_plan_accel;
    fp32 wz_plan_accel;
    fp32 return_wz_set;
    fp32 chassis_relative_angle;
    fp32 chassis_relative_angle_set;
    fp32 gimbal_radian_of_ecd;

    uint8_t gimbal_behaviour;
    uint8_t gimbal_shoot_mode;

    fp32 vx_max_speed;
    fp32 vx_min_speed;
    fp32 vy_max_speed;
    fp32 vy_min_speed;
    fp32 chassis_yaw;
    fp32 chassis_pitch;
    fp32 chassis_roll;
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

extern chassis_move_t chassis_move;
extern fp32 yaw_set;

#endif
