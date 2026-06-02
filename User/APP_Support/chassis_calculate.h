/** 
	*	@file       chassis_calculate.h
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

/****** 坐标轴示意图 ******

							 0^ x
						\		|
						 \	|<-------- 芯片板子放在此处
							\ |
			y				 \|
PI/2	<―――――――――― -PI/2
								|`
								|  `
								|    `
							PI|-PI   `

角度范围: [-PI, PI]
两个角度分别∈(0, PI/2)和(-PI/2, -PI)
***************************/

#ifndef __CHASSIS_CALCULATE_H
#define __CHASSIS_CALCULATE_H

#include "main.h"
#include "chassis_task.h"

#define GM6020_ECD_to_RAD 	7.67084032e-4f	//把GM6020的编码器值转换为弧度: 2 * PI / 8191
#define MOTOR_TO_CENTER 		0.27f						//车轮到整车中心的距离，待定

void chassis_wheel_angle_offset_init(void);
void vector_rotate(fp32 angle, fp32 *vector);
void slip_control(chassis_move_t *chassis_move);
fp32 chassis_limit_wz_by_lateral_accel(fp32 vx, fp32 vy, fp32 wz);
void chas_inv_cal(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 *wheel_angle, fp32 *wheel_speed);
void chas_for_cal(fp32 *wheel_angle, fp32 *wheel_speed, fp32 *vx, fp32 *vy, fp32 *wz);


#endif
