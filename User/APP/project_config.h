#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H
//#include "project_config.h"
#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * Global project constants
 * ========================================================= */
 /* 代码类型 调试或者发布*/
#define debug   0
#define release 1
/* 超级电容开关 */
#define Cap_off 0X00
#define Cap_on  0X01

#define ROBOT_MODE        debug
#define ROBOT_CAP         Cap_off
/*云台*/
#define PITCH_GYRO_ABSOLUTE_PID_KP					15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI					0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD					0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 		10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT		0.0f

#define YAW_GYRO_ABSOLUTE_PID_KP      			56.0f
#define YAW_GYRO_ABSOLUTE_PID_KI      			0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD      			0.5f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 			10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT			0.0f
			
#define PITCH_ENCODE_RELATIVE_PID_KP				38.0f
#define PITCH_ENCODE_RELATIVE_PID_KI				0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD				0.3f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT	  10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f
				
#define YAW_ENCODE_RELATIVE_PID_KP 					28.0f
#define YAW_ENCODE_RELATIVE_PID_KI 					0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 					0.5f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT			10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT		0.0f

#define FEEDFORWARD_GAIN        						0.00f
#define PITCH_SPEED_PID_KP      						3000.0f
#define PITCH_SPEED_PID_KI      						80.0f
#define PITCH_SPEED_PID_KD      						0.0f
#define PITCH_SPEED_PID_MAX_OUT 						30000.0f
#define PITCH_SPEED_PID_MAX_IOUT						10000.0f

#define YAW_SPEED_PID_KP      							4500.0f
#define YAW_SPEED_PID_KI      							80.0f
#define YAW_SPEED_PID_KD      							0.0f
#define YAW_SPEED_PID_MAX_OUT 							30000.0f
#define YAW_SPEED_PID_MAX_IOUT							10000.0f

//电机输出方向取反，顺带可以解决电机颤抖的bug
#define YAW_CURRENT_SET_POLARITY					(-1)
#define PITCH_CURRENT_SET_POLARITY				( 1)

#define GIMBAL_ANGLE_Z_RC_SEN         0.000002f
#define GIMBAL_TASK_INIT_TIME         200
#define YAW_CHANNEL                   2
#define PITCH_CHANNEL                 3
#define GIMBAL_MODE_CHANNEL           0
#define WZ_CHANNEL                    2
#define TURN_KEYBOARD                 KEY_PRESSED_OFFSET_F
#define TURN_SPEED                    0.04f
#define TEST_KEYBOARD                 KEY_PRESSED_OFFSET_B
#define RC_DEADBAND                   10
#define YAW_RC_SEN                    -0.000005f
#define PITCH_RC_SEN                  -0.000006f
#define YAW_MOUSE_SEN                 0.00006f
#define PITCH_MOUSE_SEN               0.00006f
#define YAW_ENCODE_SEN                0.01f
#define PITCH_ENCODE_SEN              0.01f
#define GIMBAL_CONTROL_TIME           1
#define GIMBAL_TEST_MODE              0
#define HALF_ECD_RANGE                4096
#define ECD_RANGE                     8191
#define GIMBAL_INIT_ANGLE_ERROR       0.1f
#define GIMBAL_INIT_STOP_TIME         100
#define GIMBAL_INIT_TIME              6000
#define GIMBAL_CALI_REDUNDANT_ANGLE   0.1f
#define GIMBAL_INIT_PITCH_SPEED       0.004f
#define GIMBAL_INIT_YAW_SPEED         0.005f
#define GIMBAL_CALI_MOTOR_SET         8000
#define GIMBAL_CALI_STEP_TIME         2000
#define GIMBAL_CALI_GYRO_LIMIT        0.1f
#define GIMBAL_CALI_PITCH_MAX_STEP    1
#define GIMBAL_CALI_PITCH_MIN_STEP    2
#define GIMBAL_CALI_YAW_MAX_STEP      3
#define GIMBAL_CALI_YAW_MIN_STEP      4
#define GIMBAL_CALI_START_STEP        GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP          5
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

#define INIT_YAW_SET    							0.0f
#define INIT_PITCH_SET  							0.0f

/*串口*/
#define USART_RX_BUF_LENGHT     			64
#define REFEREE_FIFO_BUF_LENGTH 			1024
#define REF_PROTOCOL_FRAME_MAX_SIZE 	192


#define DM_YAW_CAN_ID									0X01
#define DM_PIT_CAN_ID									0X02

#define DM_YAW_MASTER_ID							0X51
#define DM_PIT_MASTER_ID							0X52


#define CAN_FRIC1_ID									0X201
#define CAN_FRIC2_ID									0X202
#define CAN_FRIC3_ID									0X203
#define CAN_STRUM_ID									0X204


/* =========================================================
 * Compiler / utility macros
 * ========================================================= */


#ifdef __cplusplus
}
#endif

#endif /* PROJECT_CONFIG_H */
