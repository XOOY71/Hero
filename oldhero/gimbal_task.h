/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "struct_typedef.h"
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
	
#if ROBOT_TYPE	== Sentinel_robot
	  MITMeasure_t *gimbal_DM4310_measure;  //包含目标速度值的设定，所以没有const
#endif	
	
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
	
#if ROBOT_TYPE == Sentinel_robot	
	  gimbal_PID_t gimbal_DM4310_absolute_angle_pid;
#endif	
	
    pid_type_def gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle ; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;
	  fp32 radian_of_ecd;

} gimbal_motor_t;	


typedef struct
{
//    fp32 max_yaw = 3.0;
//    fp32 min_yaw = -3.0;
//    fp32 max_pitch = 0.261750996;
//    fp32 min_pitch = 0.82528168;
//    uint16_t max_yaw_ecd = 0x0DE0;
//    uint16_t min_yaw_ecd = 0x1CE5;
//    uint16_t max_pitch_ecd = 0x19B7;
//    uint16_t min_pitch_ecd = 0x1C36;
	  fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;


#if ROBOT_GIMBAL  ==   yaw_pitch_direct || ROBOT_GIMBAL  ==   yaw_pitch_linkage

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
}gimbal_control_t;

#elif ROBOT_GIMBAL   ==   double_yaw_pitch

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;


#elif ROBOT_GIMBAL   ==   multi_axis_robotic_arm

 typedef struct
{
    const MITMeasure_t *motor_measure;
    //电机在线标志位,0为离线
    uint8_t online;
    //电机最大角度
    fp32 max_angle;
    //电机最小角度
    fp32 min_angle;
    //电机初始位置
    fp32 zero_offset;
}arm_joint_t;

typedef struct
{
    //错误代码，不为0马上进无力模式
    uint8_t error_code;
    //初始化标志位
    uint8_t init_flag;
    //初始化完成时间
    uint32_t init_time;
    const RC_ctrl_t *gimbal_rc_ctrl;

		//自定义控制器发送的数组指针,后续需要在串口文件中处理
		const fp32 *custom_ctrl;

    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;

		//不用，闲置
    gimbal_motor_t gimbal_pitch_motor;

    //机械臂的各个臂的重量和长度参数，0是长度，1是重量
    fp32 multi_arm_params[MOTOR_NUM][2];
    //计算出机械臂发送的速度\位置\力矩
    fp32 multi_arm_set[MOTOR_NUM][3];
		fp32 multi_arm_cmd[MOTOR_NUM][3];
    arm_joint_t joint_motor[MOTOR_NUM];
    gimbal_motor_t gimbal_yaw_motor;
    
    gimbal_step_cali_t gimbal_cali;
    // 0为角度环KP，1为速度环KD
    pid_type_def joint_pid[MOTOR_NUM];
} gimbal_control_t;

#endif

extern gimbal_control_t gimbal_control;

extern int16_t yaw_can_set_current, pitch_can_set_current, shoot_can_set_current;
//让云台保持在固定的值
extern fp32 yaw_target ;
extern fp32 yaw_curret ;
extern uint8_t auto_aim_flag ;
extern fp32 aim_flag;
extern pid_type_def AIM;


extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);
extern void gimbal_task(void const *pvParameters);
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
extern fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
extern void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
extern void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
extern void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
extern void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
extern void gimbal_PID_clear(gimbal_PID_t *pid_clear);
extern fp32 gimbal_PID_Calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
extern void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);


#endif
