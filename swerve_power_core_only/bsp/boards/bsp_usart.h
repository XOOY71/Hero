#ifndef BSP_USART_H
#define BSP_USART_H


#include "struct_typedef.h"
#include "referee_usart_task.h"
#include "robot_param.h"

#define USART_BUF_LENGHT			90

#ifdef Sentinel_robot
	#define GIMBAL_DATA_LENGTH		30
#else
	#define GIMBAL_DATA_LENGTH		46
#endif

// 定义一个联合类型，用于将浮点数转换为字节
typedef union {
    float fp32;
    uint8_t bytes[4];
	
} fp32_to_bytes;
// 定义一个联合类型，用于将16位整数转换为字节
typedef union {
    int16_t int16;
    uint8_t bytes[2];
	
} int_to_bytes;

extern fp32 chassis_INS_yaw, chassis_INS_pitch, chassis_spin_speed;
extern uint8_t usart1_buf[2][USART_BUF_LENGHT];
extern void usart1_init(void);
extern void uart1_data_receive(void);
extern void usart1_receive(void);
extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart1_rx_complete_callback(uint8_t *buf, uint16_t len);
extern void usart6_init(uint8_t *rx0_buf, uint8_t *rx1_buf, uint16_t dma_buf_num);
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);
extern void usart6_rx_complete_callback(uint8_t *buf, uint16_t len);
extern void chassis_to_gimbal(uint8_t *gimbal_data);
extern void gimbal_to_chassis(uint8_t *gimbal_data);
extern int uart6_printf(const char *format, ...);


#endif
