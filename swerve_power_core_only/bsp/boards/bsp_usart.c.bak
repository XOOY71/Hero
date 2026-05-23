#include "auto_aim.h"
#include "bsp_buzzer.h"
#include "bsp_usart.h"
#include "chassis_power_control.h"
#include "main.h"
#include "referee_usart_task.h"
#include "remote_control.h"
#include "robot_param.h"
#include "usb_task.h"
#include <stdarg.h>
#include <stdio.h>
#include "string.h"
#include "video_transmission_module.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern VTM_Data_t vtm_data;
extern RC_ctrl_t rc_ctrl;
extern const uint8_t RC_TYPE;

#ifdef Sentinel_robot
extern RC_ctrl_t rc_ctrl;
#endif

//#define DMA_FLAG_TCIF5 ((uint32_t)0x20000800)
uint8_t gimbal_data[GIMBAL_DATA_LENGTH] = {0};
uint8_t usart1_buf[2][USART_BUF_LENGHT];//设置双缓冲区
uint8_t data_send_from_pc[ USART_BUF_LENGHT/2] = {0};
uint8_t data_send_from_pc6[ USART_BUF_LENGHT ] = {0};
uint8_t data_send_from_chassis[ USART_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//请求重新发送数据

void usart1_init(void)
{
	//使能DMA串口接收和发送
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
	
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);	//接收中断
	
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	
	while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF7);

	hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
	//内存缓冲区1
	hdma_usart1_rx.Instance->M0AR = (uint32_t)(usart1_buf[0]);
	//内存缓冲区2
	hdma_usart1_rx.Instance->M1AR = (uint32_t)(usart1_buf[1]);
	//数据长度3
	__HAL_DMA_SET_COUNTER(&hdma_usart1_rx, USART_BUF_LENGHT);
	//使能双缓冲区
	SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
	
	//使能DMA
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
	
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
}

void usart1_receive(void){
	// 开始DMA接收
	HAL_UART_Receive_DMA(&huart1, usart1_buf[0], USART_BUF_LENGHT/2);
}

void usart1_tx_dma_init(void)
{
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
	hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
	hdma_usart1_tx.Instance->NDTR = 0;
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

	hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

	__HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

	hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

	__HAL_DMA_ENABLE(&hdma_usart6_tx);
}

//底盘、云台的数据接收
void uart1_data_receive(void)
{
	static uint16_t this_time_rx_len = 0;
	
	if(USART1->SR & UART_FLAG_IDLE)
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);

		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = USART_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

			hdma_usart1_rx.Instance->NDTR = USART_BUF_LENGHT;

			//设定缓冲区1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
			
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			
			#ifdef chassis_board
				memcpy(gimbal_data, usart1_buf[0], GIMBAL_DATA_LENGTH);
				//处理来自云台发送的信息
				gimbal_to_chassis(gimbal_data);
				
				if(RC_TYPE == 1)
				{
					sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
				}
			#endif
		 }
		else{
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = USART_BUF_LENGHT - hdma_usart1_rx.Instance->NDTR;

			hdma_usart1_rx.Instance->NDTR = USART_BUF_LENGHT;

			//设定缓冲区0
			DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
			
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);
			
			#ifdef chassis_board
				memcpy(gimbal_data, usart1_buf[1], GIMBAL_DATA_LENGTH);
				//处理来自云台发送的信息
				gimbal_to_chassis(gimbal_data);
			#endif
		}
	}
}

void usart6_init(uint8_t *rx0_buf, uint8_t *rx1_buf, uint16_t dma_buf_num)
{
	//使能DMA串口接收和发送
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	
	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_rx);
	}

	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
	//内存缓冲区1
	hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx0_buf);
	//内存缓冲区2
	hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx1_buf);
	//数据长度
	__HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

	//使能双缓冲区
	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

	//使能DMA
	__HAL_DMA_ENABLE(&hdma_usart6_rx);

	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);

	while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}

	hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);
}

//串口6的回调函数
void usart6_rx_complete_callback(uint8_t *buf, uint16_t len) {
	// 先验证数据包格式是否正确
  // 拷贝接收到的数据到data_send_from_pc
  if (buf[0] == 's' || buf[9] == 'e') {
		memcpy(data_send_from_pc6, buf, len);
		auto_aim(data_send_from_pc6);
  }
  else{
    usart6_tx_dma_enable(restart_array,len);
  }
}

//这些变量用于下面的函数
fp32 GIMBAL_INS_yaw, GIMBAL_INS_pitch, GIMBAL_INS_roll;

//底盘处理云台发来的数据
//目前发送给底盘的数据：yaw电机由编码值转换的弧度值、imu的yaw值、pitch值、roll值、云台行为模式、射击模式
void gimbal_to_chassis(uint8_t *gimbal_data)
{
	static fp32_to_bytes motor_yaw_now;			//yaw电机由编码值转换的弧度值
	static fp32_to_bytes INS_yaw_now;				//存储INS指针, 用于记录云台imu的yaw值
	static fp32_to_bytes INS_pitch_now;			//存储INS指针, 用于记录云台imu的pitch值
	static fp32_to_bytes INS_roll_now;			//存储INS指针, 用于记录云台imu的roll值
	
#ifdef Sentinel_robot	//云台板通过板件通信控制底盘，相关指令
	static int_to_bytes order_now[4];
	static uint8_t mode_now[2];
#endif	
	
	static uint8_t gimbal_behaviour_now;		//云台行为模式
	static uint8_t gimbal_shoot_mode_now;		//射击模式
	
//	//底盘板通信失败保障
//	static fp32 last_gimbal_radian_of_ecd = 0;	//上一次的yaw电机弧度值
//	static uint8_t error_count = 0;							//错误计数
	
	motor_yaw_now.bytes[0] = gimbal_data[1];
	motor_yaw_now.bytes[1] = gimbal_data[2];
	motor_yaw_now.bytes[2] = gimbal_data[3];
	motor_yaw_now.bytes[3] = gimbal_data[4];
	
	if(motor_yaw_now.fp32 >= PI){ motor_yaw_now.fp32 = PI; }
	else if(motor_yaw_now.fp32 < -PI){ motor_yaw_now.fp32 = -PI; }
	
	chassis_move.gimbal_radian_of_ecd = motor_yaw_now.fp32;
	
	#if GIMBAL_YAW_RADIAN_REVERSE
		chassis_move.gimbal_radian_of_ecd = -chassis_move.gimbal_radian_of_ecd;
	#endif

	INS_yaw_now.bytes[0] = gimbal_data[5];
	INS_yaw_now.bytes[1] = gimbal_data[6];
	INS_yaw_now.bytes[2] = gimbal_data[7];
	INS_yaw_now.bytes[3] = gimbal_data[8];
	GIMBAL_INS_yaw = INS_yaw_now.fp32;

	INS_pitch_now.bytes[0] = gimbal_data[9];
	INS_pitch_now.bytes[1] = gimbal_data[10];
	INS_pitch_now.bytes[2] = gimbal_data[11];
	INS_pitch_now.bytes[3] = gimbal_data[12];
	GIMBAL_INS_pitch = INS_pitch_now.fp32;
	
	INS_roll_now.bytes[0] = gimbal_data[13];
	INS_roll_now.bytes[1] = gimbal_data[14];
	INS_roll_now.bytes[2] = gimbal_data[15];
	INS_roll_now.bytes[3] = gimbal_data[16];
	GIMBAL_INS_roll = INS_roll_now.fp32;
	
	gimbal_behaviour_now = gimbal_data[17];
	chassis_move.gimbal_behaviour = gimbal_behaviour_now;
	
	gimbal_shoot_mode_now = gimbal_data[18];
	chassis_move.gimbal_shoot_mode = gimbal_shoot_mode_now;
	
	uint8_t* vtm_data_ptr = (uint8_t*)&vtm_data;
	for(uint8_t i = 0; i <= 25; i ++)
	{
		vtm_data_ptr[2 + i] = gimbal_data[19 + i];
	}
	
#ifdef Sentinel_robot
	order_now[0].bytes[0] = gimbal_data[19];
	order_now[0].bytes[1] = gimbal_data[20];
	rc_ctrl.rc.ch[0] = order_now[0].int16;
	
	order_now[1].bytes[0] = gimbal_data[21];
	order_now[1].bytes[1] = gimbal_data[22];
	rc_ctrl.rc.ch[1] = -order_now[1].int16;
	
	order_now[2].bytes[0] = gimbal_data[23];
	order_now[2].bytes[1] = gimbal_data[24];
	rc_ctrl.rc.ch[2] = -order_now[2].int16;
	
	order_now[3].bytes[0] = gimbal_data[25];
	order_now[3].bytes[1] = gimbal_data[26];
	rc_ctrl.rc.ch[3] = order_now[3].int16;
	
	mode_now[0] = gimbal_data[27];
	rc_ctrl.rc.s[0] = mode_now[0];
	mode_now[1] = gimbal_data[28];
	rc_ctrl.rc.s[1] = mode_now[1];
#endif
	
//	if(chassis_move.chassis_mode == CHASSIS_VECTOR_SPIN)
//	{
//		if((last_gimbal_radian_of_ecd - chassis_move.gimbal_radian_of_ecd == 0))
//		{
//			if(error_count < 50)
//			{
//				error_count ++;
//			}
//			else
//			{
//				error_count = 0;
//				usart1_init();
//				usart1_receive();
//				usart1_tx_dma_init();
//			}
//		}
//		else{ error_count = 0; }
//	}
//	last_gimbal_radian_of_ecd = chassis_move.gimbal_radian_of_ecd;
}

/****************** 串口调试 ******************/
/**
 * @brief 类似printf的串口打印函数，通过USART6的DMA发送
 * @param format 格式化字符串
 * @param ... 可变参数列表
 * @retval 发送的字节数，-1表示出错
 */
//int uart6_printf(const char *format, ...)
//{
//    va_list args;
//    uint16_t len;
//    static uint8_t print_buf[256];  // 静态缓冲区存储格式化后的字符串

//    // 检查输入参数合法性
//    if (format == NULL)
//    {
//        return -1;
//    }

//    // 初始化可变参数列表
//    va_start(args, format);

//    // 将格式化字符串写入缓冲区
//    len = vsnprintf((char *)print_buf, 256, format, args);

//    // 结束可变参数列表
//    va_end(args);

//    // 检查是否超出缓冲区大小
//    if (len < 0 || len >= 256)
//    {
//        return -1;  // 格式化失败或缓冲区溢出
//    }

//    // 通过DMA发送格式化后的字符串
//    usart6_tx_dma_enable(print_buf, len);

//    return len;  // 返回发送的字节数
//}

// 使用示例

//if(tag_delay < 100) //UART6串口调试，减缓发送频率，避免消息被覆盖
//{
//	tag_delay ++;
//}
//else
//{
//	tag_delay = 0;
//	uart6_printf("%.4f,%.4f,%.4f\r\n", angle_delta[0],wheel_angle[0],current_angle[0]);
//}
/****************** 串口调试 ******************/
