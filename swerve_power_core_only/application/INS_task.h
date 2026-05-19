/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.h
  * @brief      主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
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

#ifndef INS_Task_H
#define INS_Task_H
#include "struct_typedef.h"


#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET  		0		//偏航角(Yaw)
#define INS_PITCH_ADDRESS_OFFSET		1		//俯仰角(Pitch)  
#define INS_ROLL_ADDRESS_OFFSET 		2		//翻滚角(Roll)

#define INS_GYRO_X_ADDRESS_OFFSET		0		//X轴角速度
#define INS_GYRO_Y_ADDRESS_OFFSET		1		//Y轴角速度
#define INS_GYRO_Z_ADDRESS_OFFSET		2		//Z轴角速度

#define INS_ACCEL_X_ADDRESS_OFFSET	0		//X轴加速度
#define INS_ACCEL_Y_ADDRESS_OFFSET	1		//Y轴加速度  
#define INS_ACCEL_Z_ADDRESS_OFFSET	2		//Z轴加速度

#define INS_MAG_X_ADDRESS_OFFSET		0		//X轴磁场强度
#define INS_MAG_Y_ADDRESS_OFFSET		1		//Y轴磁场强度
#define INS_MAG_Z_ADDRESS_OFFSET		2		//Z轴磁场强度


extern void INS_task(void const *pvParameters);
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
extern const fp32 *get_INS_quat_point(void);
extern fp32 *get_INS_angle_point(void);
extern fp32 *get_gyro_data_point(void);
extern const fp32 *get_accel_data_point(void);
extern const fp32 *get_mag_data_point(void);


#endif
