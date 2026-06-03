#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"
#include "project_config.h"
#include "protocol.h"

extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

void RefereeUsartTask_Init(void);
void referee_usart_task(void const *argument);

#endif
