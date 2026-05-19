/**
  ****************************(C) COPYRIGHT 2025 PRINTK****************************
  * @file       robot_param.h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     3-26-2025     	滕勇军          1. done
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
	
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "struct_typedef.h"

/******** 不要的记得注释掉，不要改成0 ********/

/*
*兵种定义：
 @param  	Hero_robot
					Infantry_robot
					Balance_robot
					Sentinel_robot
					Engineer_robot
*/

//英雄宏定义
//#define Hero_robot 1

//普通步兵宏定义
#define Infantry_robot 1

////平衡步兵宏定义
//#define Balance_robot 1

////哨兵宏定义
//#define Sentinel_robot 1

////工程宏定义
//#define Engineer_robot 1

/*
*底盘种类定义：
 @param  	Mecanum wheel
					Omni_wheel
					Balance_wheel
					steering_wheel
*/

////麦轮底盘
//#define Mecanum_wheel 1 

////全向轮底盘
//#define Omni_wheel 1

//舵轮底盘
#define steering_wheel 1

////平衡轮足底盘
//#define Balance_wheel 1


/*
*云台种类定义：
 @param  还没想好，就先欠着
*/



////超电定义
//#define Super_cap 1


//英雄
#ifdef Hero_robot
	#ifdef steering_wheel
		//底盘尺寸参数：长40，宽35
		#define HALF_LENGTH			20.0f
		#define HALF_WIDTH			17.5f
		
		//舵轮初始角度
		#define CHASSIS_6020_INIT_ANGLE_0		(-0.14f)
		#define CHASSIS_6020_INIT_ANGLE_1		(-0.62f)
		#define CHASSIS_6020_INIT_ANGLE_2		( 0.88f)
		#define CHASSIS_6020_INIT_ANGLE_3		( 0.20f)
		
		//底盘回正偏移
		#define CHASSIS_RETURN_TARGET											( 0.72f)
		#define CHASSIS_RETURN_OFFSET											( 1.15f)
		//底盘跟随云台yaw偏移
		#define CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET					(-2.12f)
		//底盘旋转模式偏移
		#define CHASSIS_SPIN_OFFSET												(-1.60f)
		
		//舵轮位置，更改此宏定义可以交换不同轮子的行为
		#define WHEEL_RF		3
		#define WHEEL_LF		0
		#define WHEEL_LB		1
		#define WHEEL_RB		2

		//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
		#define CHASSIS_WZ_RC_SEN			4.352e-5f
		
		//底盘小陀螺速度
		#define CHASSIS_SPIN_SPEED		0.075f
		
		//底盘运动过程最大前进和横移速度
		#define NORMAL_MAX_CHASSIS_SPEED_X	2.5f
		#define NORMAL_MAX_CHASSIS_SPEED_Y	2.5f
		
	#endif
#endif

//步兵
#ifdef Infantry_robot
	#ifdef steering_wheel
		//底盘尺寸参数：长29.803，宽29.803
		#define HALF_LENGTH			14.9015f
		#define HALF_WIDTH			14.9015f
		
		//舵轮初始角度
		#define CHASSIS_6020_INIT_ANGLE_0		( 1.30f)
		#define CHASSIS_6020_INIT_ANGLE_1		( 2.14f)
		#define CHASSIS_6020_INIT_ANGLE_2		( 1.67f)
		#define CHASSIS_6020_INIT_ANGLE_3		( 2.13f)
		
		//云台yaw电机反馈值取反
		#define GIMBAL_YAW_RADIAN_REVERSE						1
		
		//底盘回正偏移
		#define CHASSIS_RETURN_TARGET											(-3.13f)
		#define CHASSIS_RETURN_OFFSET											( 0.00f)
		//底盘跟随云台yaw偏移
		#define CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET					( 0.00f)
		//底盘旋转模式偏移
		#define CHASSIS_SPIN_OFFSET												( 0.50f)
		
		//舵轮位置，更改此宏定义可以交换不同轮子的行为
		#define WHEEL_RF		1
		#define WHEEL_LF		0
		#define WHEEL_LB		3
		#define WHEEL_RB		2

		//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
		#define CHASSIS_WZ_RC_SEN		4.945e-5f
		
		//底盘小陀螺速度
		#define CHASSIS_SPIN_SPEED		0.08f
		
		//底盘运动过程最大前进和横移速度
		#define NORMAL_MAX_CHASSIS_SPEED_X	2.6f	
		#define NORMAL_MAX_CHASSIS_SPEED_Y	2.6f
		
	#endif
#endif

//哨兵
#ifdef Sentinel_robot
	#ifdef steering_wheel
		//底盘尺寸参数：长37.2，宽37.2
		#define HALF_LENGTH			18.6f
		#define HALF_WIDTH			18.6f
		
		//舵轮初始角度
		#define CHASSIS_6020_INIT_ANGLE_0		( 4.98f)
		#define CHASSIS_6020_INIT_ANGLE_1		(-2.80f)
		#define CHASSIS_6020_INIT_ANGLE_2		( 1.82f)
		#define CHASSIS_6020_INIT_ANGLE_3		(-0.81f)
		
		//底盘回正偏移
		#define CHASSIS_RETURN_TARGET											(-0.96f)
		#define CHASSIS_RETURN_OFFSET											( 0.00f)
		//底盘跟随云台yaw偏移
		#define CHASSIS_FOLLOW_GIMBAL_YAW_OFFSET					(	0.92f)
		//底盘旋转模式偏移
		#define CHASSIS_SPIN_OFFSET												( 1.20f)
		
		//舵轮位置，更改此宏定义可以交换不同轮子的行为
		#define WHEEL_RF		0
		#define WHEEL_LF		3
		#define WHEEL_LB		2
		#define WHEEL_RB		1

		//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
		#define CHASSIS_WZ_RC_SEN		4.945e-5f		//1.789e-5f
		
		
	#endif
#endif

#endif
