/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "bsp_rng.h"
#include "bsp_usart.h"
#include "detect_task.h"

//电机数据读取
#define get_motor_measure(ptr, data)                                \
	{                                                                 \
		(ptr)->last_ecd = (ptr)->ecd;                                   \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
		(ptr)->temperate = (data)[6];                                   \
	}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

	
#ifdef chassis_board
static CAN_TxHeaderTypeDef  chassis_tx_message;
//static MOTOR_MEASURE_t motor_chas[8];	//底盘上有8个电机
MOTOR_MEASURE_t motor_chas[8];	//底盘上有8个电机
	
/**
  * @brief          HAL库CAN回调函数,接收底盘电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	switch (rx_header.StdId)
	{
		case CAN1_3508_M1_ID:
		case CAN1_3508_M2_ID:
		case CAN1_3508_M3_ID:
		case CAN1_3508_M4_ID:
		case CAN2_6020_M1_ID:
		case CAN2_6020_M2_ID:
		case CAN2_6020_M3_ID:
		case CAN2_6020_M4_ID:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - CAN1_3508_M1_ID;
			get_motor_measure(&motor_chas[i], rx_data);
			detect_hook(CHASSIS_MOTOR1_TOE + i);
			break;
		}
		case 0x600:
		case 0x601:
		case 0x602:
		case 0x603:
		case 0x610:
		case 0x611:
		case 0x612:
		case 0x613:
		{
			pm01_response_handle(&rx_header, rx_data);
			break;
		}
		default: break;
	}
}

/**
  * @brief          发送底盘3508电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_CHAS_3508(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN1_CHAS_3508_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&CHASSIS_3508_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	//usart1_tx_dma_enable( chassis_can_send_data, 8);
}

/**
  * @brief          发送底盘6020电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      motor5: (0x205) 6020电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor6: (0x206) 6020电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor7: (0x207) 6020电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor8: (0x208) 6020电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_CHAS_6020(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN2_CHAS_6020_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = motor5 >> 8;
	chassis_can_send_data[1] = motor5;
	chassis_can_send_data[2] = motor6 >> 8;
	chassis_can_send_data[3] = motor6;
	chassis_can_send_data[4] = motor7 >> 8;
	chassis_can_send_data[5] = motor7;
	chassis_can_send_data[6] = motor8 >> 8;
	chassis_can_send_data[7] = motor8;

	HAL_CAN_AddTxMessage(&CHASSIS_6020_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	//usart1_tx_dma_enable( chassis_can_send_data, 8);
}

/**
  * @brief          返回底盘3508和6020电机数据指针
  * @param[in]      i: 电机编号,范围[0,7]
  * @retval         电机数据指针
  */
MOTOR_MEASURE_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chas[i & 0x07];
}
#endif // DEBUG


//#ifdef gimbal_board
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static MOTOR_MEASURE_t motor_gimb[3];	//云台上有5个电机(没算那两个摩擦轮电机)

/**
  * @brief          HAL库CAN回调函数,接收云台电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	CAN_RxHeaderTypeDef rx_header;
//	uint8_t rx_data[8];
//	
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//	
//	switch (rx_header.StdId)
//	{
//    case CAN1_PLUCK_ID:  // 0x203 -> i = 0 (拨弹电机)
//    case CAN2_YAW_ID:    // 0x205 -> i = 2 → 调整后 i = 1 (YAW 电机)
//    case CAN2_PIT_ID:    // 0x206 -> i = 3 → 调整后 i = 2 (PITCH 电机)
//		{
//			static uint8_t i = 0;
//			i = rx_header.StdId - CAN1_PLUCK_ID;
//			if((i == 2) | (i == 3)) i -= 1;		//由于2006的ID为0x203,而6020电机的ID为0x205和0x206
//			get_motor_measure(&motor_gimb[i], rx_data);
//			detect_hook(PLUCK_MOTOR_TOE + i);
//			break;
//		}
//		
//		default: break;
//	}
//}

/**
  * @brief          发送云台3508电机和2006电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      fric1	(0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      fric2	(0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      pluck	(0x203) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      rev		保留
  * @retval         none
  */
void CAN_cmd_GIMB_FP(int16_t fric1, int16_t fric2, int16_t pluck, int16_t rev)
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN1_GIMB_3508_2006_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = fric1 >> 8;
	gimbal_can_send_data[1] = fric1;
	gimbal_can_send_data[2] = fric2 >> 8;
	gimbal_can_send_data[3] = fric2;
	gimbal_can_send_data[4] = pluck >> 8;
	gimbal_can_send_data[5] = pluck;
	gimbal_can_send_data[6] = rev >> 8;
	gimbal_can_send_data[7] = rev;
	HAL_CAN_AddTxMessage(&GIMBAL_3508_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	//usart1_tx_dma_enable(gimbal_can_send_data, 8);
}

/**
  * @brief          发送云台6020电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw		(0x205) 6020电机控制电流, 范围 [-16384,16384]
  * @param[in]      pitch	(0x206) 6020电机控制电流, 范围 [-16384,16384]
  * @param[in]      rev		保留
  * @param[in]      rev		保留
  * @retval         none
  */
void CAN_cmd_GIMB_6020(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN1_GIMB_3508_2006_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = yaw >> 8;
	gimbal_can_send_data[1] = yaw;
	gimbal_can_send_data[2] = pitch >> 8;
	gimbal_can_send_data[3] = pitch;
	gimbal_can_send_data[4] = rev1 >> 8;
	gimbal_can_send_data[5] = rev1;
	gimbal_can_send_data[6] = rev2 >> 8;
	gimbal_can_send_data[7] = rev2;
	HAL_CAN_AddTxMessage(&GIMBAL_6020_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	//usart1_tx_dma_enable(gimbal_can_send_data, 8);
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
MOTOR_MEASURE_t *get_pluck_motor_measure_point(void)
{
	return &motor_gimb[2];
}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
MOTOR_MEASURE_t *get_yaw_gimbal_motor_measure_point(void)
{
	return &motor_gimb[0];
}

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
MOTOR_MEASURE_t *get_pitch_gimbal_motor_measure_point(void)
{
	return &motor_gimb[1];
}
//#endif // DEBUG
