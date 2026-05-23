/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee.h
  * @brief     	裁判系统 接收到的数据的转移 与 某些数据获取
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

#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
#include "protocol.h"


extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
uint8_t get_robot_id(void);
uint16_t get_shooter_barrel_heat_limit(void);
uint16_t get_chassis_power_limit(void);
uint16_t get_buffer_energy(void);
uint16_t get_shooter_17mm_heat(void);
uint16_t get_shooter_42mm_heat(void);

#endif
