#include "bsp_usart.h"

#include "usart.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "hwt_imu.h"
//串口5，接收遥控数据


//串口1，接收裁判系统数据
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];


uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;

//串口7 板间通讯
//static uint8_t gimbal_data[USART_RX_BUF_LENGHT]={0};
uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区
uint8_t data_send_from_pc[ USART_RX_BUF_LENGHT] = {0};
uint8_t data_send_from_chassis[ USART_RX_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//请求重新发送数据

//串口10 云台板与图传通信, 用于绘制UI
uint8_t usart10_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区

//串口1发送函数
void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart1, pData, Size, 50);
}

//串口7发送函数
void USART7_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart7, pData, Size, 10);
}

//串口10发送函数
//void USART10_Transmit(uint8_t *pData, uint16_t Size)
//{
//	HAL_UART_Transmit(&huart10, pData, Size, 50);
//}

////串口10中断发送函数
//void USART10_Transmit_IT(uint8_t *pData, uint16_t Size)
//{
//	HAL_UART_Transmit_IT(&huart10, pData, Size);
//}

gimbal_data_t gimbal;



float yaw_motor_relative_angle, GIMBAL_INS_yaw;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7) {
		// 处理接收到的数据
		// 例如，将接收到的数据存入缓冲区或触发某种事件

		// 继续接收下一个数据块
	}
	else if (huart->Instance == USART10) {

	}
}
/*空闲中断回调*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{

	}
	if(huart->Instance == UART5)
	{
		if (Size <= RC_FRAME_LENGTH)
		{
			sbus_to_rc(remote_buff, &rc_ctrl, &remoter);
		}
		else if(Size > RC_FRAME_LENGTH) // 错误处理
		{	
			memset(remote_buff, 0, SBUS_RX_BUF_NUM);				   
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM);
	}
	if(huart->Instance == UART7)
	{
		hwt101_rx_parse(usart7_buf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7_buf, USART_RX_BUF_LENGHT);
	}

	if(huart->Instance == USART10)
	{
		hwt906_rx_parse(usart10_buf, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART_RX_BUF_LENGHT);
	}
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[0], USART_RX_BUF_LENGHT); // 接收发生错误后重启
		memset(usart1_buf, 0,  USART_RX_BUF_LENGHT * 2);		
	}
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM); // 接收发生错误后重启
		memset(remote_buff, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存		
	}
	if(huart->Instance == UART7)
	{
		memset(usart7_buf, 0, USART_RX_BUF_LENGHT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, usart7_buf, USART_RX_BUF_LENGHT);
	}
	if(huart->Instance == USART10)
	{
		memset(usart10_buf, 0, USART_RX_BUF_LENGHT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, usart10_buf, USART_RX_BUF_LENGHT);

	}
}

/* 转发 TX 完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* removed: uart_dma_tx_cplt_handler */
}

