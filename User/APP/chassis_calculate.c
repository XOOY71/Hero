/** 
	*	@file       chassis_calculate.c
  * @brief      底盘运动学正/逆解算
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

#include "arm_math.h"
#include "bsp_usart.h"
#include "chassis_calculate.h"
#include "chassis_task.h"
#include "CAN_receive.h"
#include "robot_param.h"
#include "user_lib.h"
#include <math.h>
#include "stdlib.h"


/**
  * @brief          底盘舵向电机角度初始化
  * @retval         none
  */
void chassis_wheel_angle_offset_init(void)
{
	chassis_move.wheel_angle_offset.now[0] = chassis_move.wheel_angle_offset.initial[0] = chassis_move.wheel_angle_offset.initial[0] = CHASSIS_6020_INIT_ANGLE_0;
	chassis_move.wheel_angle_offset.now[1] = chassis_move.wheel_angle_offset.initial[1] = chassis_move.wheel_angle_offset.initial[1] = CHASSIS_6020_INIT_ANGLE_1;
	chassis_move.wheel_angle_offset.now[2] = chassis_move.wheel_angle_offset.initial[2] = chassis_move.wheel_angle_offset.initial[2] = CHASSIS_6020_INIT_ANGLE_2;
	chassis_move.wheel_angle_offset.now[3] = chassis_move.wheel_angle_offset.initial[3] = chassis_move.wheel_angle_offset.initial[3] = CHASSIS_6020_INIT_ANGLE_3;
}

/**
  * @brief          旋转变换函数(二维)
  * @param[in]			旋转角度
	* @param[in]			需要旋转变换的向量
  * @retval         none
  */
void vector_rotate(fp32 angle, fp32 *vector)
{
	if(vector == NULL) return;
	
	//定义临时变量
	fp32 x_temp = vector[0];
	
	//角度规范化
	angle = rad_format(angle);
	
	//计算旋转矩阵
	fp32 cos = arm_cos_f32(angle), sin = arm_sin_f32(angle);
	
	//计算旋转变换后的向量														//旋转矩阵为：
	vector[0] = cos * x_temp - sin * vector[1];       //{{cos, -sin},
	vector[1] = sin * x_temp + cos * vector[1];       // {sin,  cos}}
}

/**
  * @brief          运动学逆解算纵享丝滑控制策略
  * @param[in&out]  wheel_angle     舵轮的目标角度
  * @param[in&out]  wheel_speed     舵轮的目标速度
  * @retval         none
  */
static void smooth_control(fp32 *wheel_angle, fp32 *wheel_speed)
{
	fp32 factor = 0.50f;

	// 定义当前的舵轮角度、舵轮目标角度与当前角度差
	fp32 angle_delta[4], Current_angle[4];
	for(uint8_t i = 0; i < 4; i++)
	{
		// 获取舵轮当前的角度
		Current_angle[i] = chassis_move.chassis_6020[i].angle;

		// 计算舵轮目标角度与当前角度差，并规范化到[-π, π]范围
		angle_delta[i] = rad_format(wheel_angle[i] - Current_angle[i]);

		// 就近转位策略
		if(angle_delta[i] > PI * factor)				//如果角度差大于90度
		{
			wheel_angle[i] -= PI;       					//目标角度减180度
			wheel_speed[i] = -wheel_speed[i];			//速度反向
		}
		else if(angle_delta[i] < -PI * factor)  //如果角度差小于-90度
		{
			wheel_angle[i] += PI;        					//目标角度加180度
			wheel_speed[i] = -wheel_speed[i]; 		//速度反向
		}

		// 确保最终角度在合理范围内
		wheel_angle[i] = rad_format(wheel_angle[i]);
	}
}

//fp32 offset[4] = {1.30f, 2.14f, 1.67f, 2.13f};
fp32 offset = 0;

/**
  * @brief          运动学逆解算
  * @param[in]      vx_set  相对云台设置的x方向速度分量
  * @param[in]      vy_set  相对云台设置的y方向速度分量  
  * @param[in]      wz_set  底盘旋转速度
  * @param[out]     wheel_angle     舵轮目标角度指针
  * @param[out]     wheel_speed     舵轮目标速度指针
  * @retval         none
  */
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed)
{
	if((wheel_angle == NULL) || (wheel_speed == NULL)) return;

	// 计算各轮子到旋转中心的距离和角度
	// 对于矩形底盘，每个轮子的旋转分量需要单独计算
	
	// 计算各轮速度分量并求模
	fp32 vx_total[4], vy_total[4];
	
	// WHEEL_LF (1号电机) - 左前: 位置(-HALF_LENGTH, HALF_WIDTH)
	vx_total[WHEEL_LF] = vx_set - wz_set * HALF_WIDTH;   // -w × ry
	vy_total[WHEEL_LF] = vy_set + wz_set * HALF_LENGTH;  // +w × rx
	arm_sqrt_f32(vx_total[WHEEL_LF] * vx_total[WHEEL_LF] + vy_total[WHEEL_LF] * vy_total[WHEEL_LF], &wheel_speed[WHEEL_LF]);
	
	// WHEEL_LB (2号电机) - 左后: 位置(-HALF_LENGTH, -HALF_WIDTH)
	vx_total[WHEEL_LB] = vx_set + wz_set * HALF_WIDTH;   // -w × (-ry) = +w × ry
	vy_total[WHEEL_LB] = vy_set + wz_set * HALF_LENGTH;  // +w × rx
	arm_sqrt_f32(vx_total[WHEEL_LB] * vx_total[WHEEL_LB] + vy_total[WHEEL_LB] * vy_total[WHEEL_LB], &wheel_speed[WHEEL_LB]);
	
	// WHEEL_RB (3号电机) - 右后: 位置(HALF_LENGTH, -HALF_WIDTH)
	vx_total[WHEEL_RB] = vx_set + wz_set * HALF_WIDTH;   // -w × (-ry) = +w × ry
	vy_total[WHEEL_RB] = vy_set - wz_set * HALF_LENGTH;  // +w × (-rx) = -w × rx
	arm_sqrt_f32(vx_total[WHEEL_RB] * vx_total[WHEEL_RB] + vy_total[WHEEL_RB] * vy_total[WHEEL_RB], &wheel_speed[WHEEL_RB]);
	
	// WHEEL_RF (4号电机) - 右前: 位置(HALF_LENGTH, HALF_WIDTH)
	vx_total[WHEEL_RF] = vx_set - wz_set * HALF_WIDTH;   // -w × ry
	vy_total[WHEEL_RF] = vy_set - wz_set * HALF_LENGTH;  // +w × (-rx) = -w × rx
	arm_sqrt_f32(vx_total[WHEEL_RF] * vx_total[WHEEL_RF] + vy_total[WHEEL_RF] * vy_total[WHEEL_RF], &wheel_speed[WHEEL_RF]);
	
	// 计算各轮角度
	wheel_angle[WHEEL_LF] = atan2(vy_total[WHEEL_LF], vx_total[WHEEL_LF]) - chassis_move.wheel_angle_offset.now[WHEEL_LF] + offset;
	wheel_angle[WHEEL_LB] = atan2(vy_total[WHEEL_LB], vx_total[WHEEL_LB]) - chassis_move.wheel_angle_offset.now[WHEEL_LB] + offset;
	wheel_angle[WHEEL_RB] = atan2(vy_total[WHEEL_RB], vx_total[WHEEL_RB]) - chassis_move.wheel_angle_offset.now[WHEEL_RB] + offset;
	wheel_angle[WHEEL_RF] = atan2(vy_total[WHEEL_RF], vx_total[WHEEL_RF]) - chassis_move.wheel_angle_offset.now[WHEEL_RF] + offset;
	
	// 角度规范化
	for(uint8_t i = 0; i < 4; i++)
	{
		wheel_angle[i] = rad_format(wheel_angle[i]);
	}
	
	smooth_control(wheel_angle, wheel_speed);
}

/**
  * @brief          运动学正解算
  * @param[in]			wheel_angle		舵轮当前角度指针
  * @param[in]			wheel_speed		舵轮当前速度指针
  * @param[out]			vx	相对云台的当前x方向速度分量
  * @param[out]			vy	相对云台的当前y方向速度分量
  * @param[out]			wz	底盘当前的旋转速度
  * @retval         none
  */
//void chas_for_cal(fp32 *wheel_angle, fp32 *wheel_speed, fp32 *vx, fp32 *vy, fp32 *wz)
//{
//	if((wheel_angle == NULL) || (wheel_speed == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL)) return;
//	
//	//运动学解算前的准备
//	pre_chas_cal();
//	
//	//定义各轮子的总速度分量、角速度分量、底盘的实际速度向量分量
//	fp32 vx_total[4], vy_total[4], wz_x, vx_actual, vy_actual;
//	
//	for(uint8_t i = 0; i < 4; i ++)
//	{
//		//计算各轮子的总速度分量
//		vx_total[i] = wheel_speed[i] * arm_cos_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
//		vy_total[i] = wheel_speed[i] * arm_sin_f32(rad_format(wheel_angle[i]) - chassis_yaw + wheel_angle_offset[i]);
//	}
//	
//	//解方程求出vx_actual、vy_actual、wz_x (注: 解方程时假定角速度为逆时针方向)
//	vx_actual = ((vx_total[WHEEL_LF] + vx_total[WHEEL_RB]) / 2.0f + (vx_total[WHEEL_LB] + vx_total[WHEEL_RF]) / 2.0f) / 2.0f;
//	vy_actual = ((vy_total[WHEEL_LF] + vy_total[WHEEL_RB]) / 2.0f + (vy_total[WHEEL_LB] + vy_total[WHEEL_RF]) / 2.0f) / 2.0f;
//	wz_x = ((vx_total[WHEEL_LF] - vx_total[WHEEL_RB]) / 2.0f + (vx_total[WHEEL_LB] - vx_total[WHEEL_RF]) / 2.0f) / 2.0f;
//	
//	//旋转变换, 将速度向量由底盘坐标系转换为云台坐标系
////	*vx = vx_actual * for_matrix[0][0] + vy_actual * for_matrix[0][1];
////	*vy = vx_actual * for_matrix[1][0] + vy_actual * for_matrix[1][1];
//	
//	//不进行旋转旋转变换, 直接得到底盘坐标系下的速度向量, 用于打滑控制计算
//	*vx = vx_actual;
//	*vy = vy_actual;
//	
//	//根据角速度分量计算角速度,并将其从 m/s 转换为 rad/s
//	*wz = wz_x * 1.41421356f / MOTOR_TO_CENTER;
//}
