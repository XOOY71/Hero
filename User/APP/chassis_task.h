/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.h
  * @brief     	底盘控制任务
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/
	
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "hwt_imu.h"

#define CHASSIS_TASK_INIT_TIME 357	//任务开始空闲一段时间

#define CHASSIS_X_CHANNEL			1		//前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL			0		//左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL		2		//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_MODE_CHANNEL 	0		//选择底盘状态 开关通道号

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 	0.004f	//0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 	0.004f	//0.006f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//摇杆死区
#define CHASSIS_RC_DEADLINE 25

#define CHASSIS_CONTROL_TIME_MS 		2				//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME				0.001f	//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_FREQUENCE		500.0f	//底盘任务控制频率

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY		KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY 		KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY 		KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY		KEY_PRESSED_OFFSET_D

#define MAX_WHEEL_SPEED							3.0f	//单个底盘电机最大速度

/* 底盘6020和3508最大can发送电流值 */
#define MAX_6020_CAN_CURRENT 16000.0f
#define MAX_3508_CAN_CURRENT 16000.0f

/* 舵轮解算用到的数据 */
#define M3508_RR						19.20320855f				//M3508电机减速比(M3508 Reduction Ratio)：3591 / 187
#define GM6020_Angle_Ratio	1303.63813886f			//把计算出的弧度Radian转换为GM6020电机角度Angle的系数：8191 / (2 * PI)
#define Wheel_Radius				0.0567f							//车轮的半径，单位：米
#define Wheel_Perimeter			0.35625661f					//车轮周长：2.0f * PI * Wheel_Radius，单位：米
#define MPS_to_RPM					3234.16461897f			//把M3508线速度m/s -> 转速r/min：60.0f * M3508_RR / Wheel_Perimeter
#define RPM_to_Icmd					1.55125592f					//把M3508电机的转速RPM -> 控制电流Icmd(适用于开环)：(187 /3591) * (16384 / 550)

/* 基于物理模型的控制参数 */
#define ROBOT_MASS                      11.0f     // 机器人质量 (kg)
#define M3508_TORQUE_CONSTANT           0.3f        // 3508电机力矩常数 (Nm/A)
#define M3508_REDUCTION_RATIO           19.2032f       // 电机减速比
#define CHASSIS_EFFICIENCY              0.92f       // 机械效率
#define CONTROL_PERIOD_MODEL            0.02           // 加速度计算周期 (s)   对应加速度的值 error/T
#define CHASSIS_MAX_ACCEL               5.0f        // 最大加速度 (m/s^2)
#define CHASSIS_MAX_JERK                50.0f       // 最大加加速度 (m/s^3)
#define M3508_MAX_CONT_TORQUE           2.8f        // 3508最大连续输出力矩 (Nm)
#define FRICTION_SPEED_BAND             0.1f        // 摩擦补偿速度过渡范围 (m/s)
#define FRICTION_LINEAR_GAIN            4000.0f     // 过渡范围内的补偿增益
#define FRICTION_CONSTANT_CURRENT       400.0f      // 常态摩擦补偿电流 (abs(v) > 0.1)
#define TRACTION_ZERO_FORCE_THRESHOLD   1.0f        // 牵引力近零判断阈值 (N)               还没调试
#define SPEED_HOLD_ERROR_THRESHOLD      0.1f        // 小P启用误差阈值 (m/s)
#define SPEED_HOLD_KP                   600.0f      // 小P补偿增益（辅助平衡摩擦力）

/* 底盘3508电机PID */	/***************待定***************/
#define M3505_MOTOR_SPEED_PID_KP 				30000.0f
#define M3505_MOTOR_SPEED_PID_KI 				10.0f
#define M3505_MOTOR_SPEED_PID_KD				0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 	16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT	100.0f

/* 底盘6020电机PID */	/***************待定***************/
#define GM6020_MOTOR_ANGLE_PID_KP 				160.0f
#define GM6020_MOTOR_ANGLE_PID_KI 				0.01f
#define GM6020_MOTOR_ANGLE_PID_KD					0.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_OUT 		120.0f
#define GM6020_MOTOR_ANGLE_PID_MAX_IOUT		10.0f


//YAW回正-+-*-98077777777777777777777077��式PID
#define YAW_RETURN_PID_KP 				800.0f
#define YAW_RETURN_PID_KI 				0.08f
#define YAW_RETURN_PID_KD					20.0f
#define YAW_RETURN_PID_MAX_OUT 		600.0f
#define YAW_RETURN_PID_MAX_IOUT		60.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define GM6020_MOTOR_SPEED_PID_KP        85.0f
#define GM6020_MOTOR_SPEED_PID_KI        0.0f
#define GM6020_MOTOR_SPEED_PID_KD        80.0f
#define GM6020_MOTOR_SPEED_PID_MAX_OUT   16000.0f
#define GM6020_MOTOR_SPEED_PID_MAX_IOUT  0.0f

/* 底盘旋转跟随PID */	/***************待定***************/
#define CHASSIS_FOLLOW_GIMBAL_PID_KP					0.08f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI					0.002f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD					0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT			0.08f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT		0.005f


#ifndef myabs
	#define myabs(x) (((x) >= 0) ? (x) : (-(x)))
#endif


#define rc_deadband_limit(input, output, dealine)	  \
{                                                		\
	if ((input) > (dealine) || (input) < -(dealine))	\
	{                                             		\
		(output) = (input);                          		\
	}                                             		\
	else                                          		\
	{                                             		\
		(output) = 0;                          					\
	}                                          				\
}

typedef enum
{
	CHASSIS_VECTOR_NO_MOVE = 0,					//底盘不动
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //底盘会跟随云台相对角度
  CHASSIS_VECTOR_SPIN,								//底盘有旋转速度控制
	CHASSIS_VECTOR_RETURN  							//底盘回正
} chassis_mode_e;

typedef struct
{
  const MOTOR_MEASURE_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
	fp32 angle;
	fp32 angle_set;
  int16_t give_current;
	
} chassis_motor_t;

typedef struct
{
	fp32 now[4], last[4], initial[4];
}	wheel_angle_offset_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;              	//底盘使用的遥控器指针
	const gimbal_motor_t *chassis_yaw_motor;  	//底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
	const gimbal_motor_t *chassis_pitch_motor;	//底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
	const fp32 *chassis_INS_angle;            	//取陀螺仪解算出的欧拉角指针
	chassis_mode_e chassis_mode;              	//底盘控制状态机
	chassis_mode_e last_chassis_mode;         	//底盘上次控制状态机
	chassis_motor_t chassis_3508[4];						//底盘3508电机数据
	chassis_motor_t chassis_6020[4];						//底盘6020电机数据
	wheel_angle_offset_t wheel_angle_offset;		//底盘6020电机角度偏移
	fp32 model_3508_out[4];                       // 3508物理模型输出电流控制值
	fp32 model_accel[4];                          // 3508物理模型平滑加速度
	pid_type_def chas_6020_angle_pid[4];        //底盘6020电机角度pid
	pid_type_def chas_6020_speed_pid[4];        //底盘6020电机角度pid
	pid_type_def chassis_angle_pid;           	//底盘跟随角度pid
	pid_type_def chas_return_pid;        				//底盘回正模式pid
	uint8_t chassis_return_flag;								//底盘回正标志
	chassis_mode_e chassis_return_record;				//底盘回正记录
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;  	//使用一阶低通滤波减缓设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;  	//使用一阶低通滤波减缓设定值
	
	fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set;											//底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set;											//底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;											//底盘设定旋转角速度，逆时针为正，单位 rad/s
	fp32 last_vx_set;									//底盘上一次前后速度
	fp32 last_vy_set;									//底盘上一次左右速度
	fp32 last_wz_set;									//底盘上一次旋转速度
	fp32 return_wz_set;								//底盘回正时设置的角速度，逆时针为正，单位 rad/s
	fp32 chassis_relative_angle;			//底盘与云台的相对角度，单位 rad
	fp32 chassis_relative_angle_set;	//底盘相对云台控制角度设置
	fp32 gimbal_radian_of_ecd;				//云台yaw电机反馈的弧度值
	uint8_t gimbal_behaviour;					//云台当前行为模式
	uint8_t gimbal_shoot_mode;				//云台当前射击模式
	
	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
	fp32 vx_min_speed;  //后退方向最大速度 单位m/s
	fp32 vy_max_speed;  //左方向最大速度 单位m/s
	fp32 vy_min_speed;  //右方向最大速度 单位m/s
	fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
	fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
	fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
	
} chassis_move_t;


extern void chassis_task(void const *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

//底盘运动数据
extern chassis_move_t chassis_move;
extern fp32 yaw_set;


#endif
