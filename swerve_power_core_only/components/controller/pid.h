/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
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
	
#ifndef PID_H
#define PID_H

#include "struct_typedef.h"

enum PID_MODE
{
	PID_USUAL = 0,
	PID_DELTA
};

typedef struct
{
	uint8_t mode;
	//PID 三参数
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;

	fp32 max_out;  //最大输出
	fp32 max_iout; //最大积分输出

	fp32 set;
	fp32 fdb;

	fp32 out;
	fp32 Pout;
	fp32 Iout;
	fp32 Dout;
	fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
	fp32 error[3]; //误差项 0最新 1上一次 2上上次
	
} pid_type_def;


extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
extern void PID_clear(pid_type_def *pid);


#endif
