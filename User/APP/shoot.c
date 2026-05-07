//#include "bsp_laser.h"
#include "arm_math.h"
#include "bsp_PWM.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "main.h"
#include "pid.h"
#include "referee.h"
#include "shoot.h"
#include <stdbool.h>


#define shoot_fric1_on(shoot)		fric1_on((shoot))		//摩擦轮1宏定义
#define shoot_fric2_on(shoot)		fric2_on((shoot))		//摩擦轮2宏定义
#define shoot_fric_off()				fric_off()					//关闭两个摩擦轮

//#define shoot_laser_on()    laser_on()      //激光开启宏定义
//#define shoot_laser_off()   laser_off()     //激光关闭宏定义

//#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)		//微动开关IO


__weak void fric1_on(shoot_control_t *shoot);
__weak void fric2_on(shoot_control_t *shoot);
__weak void fric_off(void);
__weak void shoot_set_mode(void);
__weak void shoot_feedback_update(void);
__weak void trigger_motor_turn_back(void);
__weak void shoot_bullet_control(void);
__weak void shoot_init(void);


shoot_control_t shoot_control;			//射击数据

uint8_t shoot_mode_choose = 0;
uint8_t shoot_mode_choose_flag = 0;

fp32 continue_trigger_speed = CONTINUE_TRIGGER_SPEED;

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
//	/*********************** 模式选择调试 ************************/
//	//	typedef enum
//	//	{
//	//		SHOOT_STOP = 0,
//	//		SHOOT_READY_FRIC,
//	//		SHOOT_READY_BULLET,
//	//		SHOOT_BULLET,
//	//		SHOOT_CONTINUE_BULLET,
//	//	} shoot_mode_e;
//	
//	if(shoot_mode_choose_flag == 1)
//	{
//		switch(shoot_mode_choose)
//		{
//			case(0):{ shoot_control.shoot_mode = SHOOT_STOP       		  ; break; }
//			case(1):{ shoot_control.shoot_mode = SHOOT_READY_FRIC       ; break; }
//			case(2):{ shoot_control.shoot_mode = SHOOT_READY_BULLET     ; break; }
//			case(3):{ shoot_control.shoot_mode = SHOOT_BULLET           ; break; }
//			case(4):{ shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET  ; break; }
//			default:{ shoot_control.shoot_mode = SHOOT_STOP       		  ; break; }
//		}
//		shoot_mode_choose_flag = 0;
//	}
//	/***********************************************************/
	
	shoot_set_mode();        //设置状态机	
	shoot_feedback_update(); //更新数据
	
	if( shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R || (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_on();
	}
	else if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_G || (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && switch_is_mid(shoot_control.shoot_rc->rc.s[GIMBAL_MODE_CHANNEL])) ){
		Bombbay_off();
	}

	//射击停止状态设置拨弹盘目标速度为0
	if (shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.speed_set = 0.0f;
	}
	//摩擦轮准备状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
	{
		shoot_control.speed_set = 0.0f;
	}
	//可以发射状态设置拨弹盘目标速度为0
	else if (shoot_control.shoot_mode == SHOOT_READY_BULLET)
	{
		shoot_control.speed_set = 0.0f;
	}
	//单发状态执行单发函数
	else if (shoot_control.shoot_mode == SHOOT_BULLET)
	{
		shoot_bullet_control();
	}
	//连发状态设置拨弹盘转速为定值
	else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
	{
		shoot_control.speed_set = continue_trigger_speed;
		trigger_motor_turn_back();
	}
	
	PID_Calc(&shoot_control.trigger_motor_speed_pid, shoot_control.speed, shoot_control.speed_set);
	
	#if ROBOT_TYPE == Sentinel_robot		
		shoot_control.given_current = (int16_t)shoot_control.speed_set;        //哨兵,直接用目标值,暂时将就
	#else			
		shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_speed_pid.out);
	#endif
	
	//遥控器中档禁止发弹
	if(gimbal_cmd_to_shoot_stop())
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	
	if(shoot_control.shoot_mode == SHOOT_STOP)
	{
		shoot_control.given_current = 0;
		fric_off();
	}
	else
	{
		shoot_control.shoot_time = HAL_GetTick();
	}
	
	//计算摩擦轮电机的电流值
	shoot_fric1_on(&shoot_control);
	shoot_fric2_on(&shoot_control);
	
	//发送摩擦轮电机的电流值
	if(shoot_control.fric1.shoot_motor_measure != NULL && shoot_control.fric2.shoot_motor_measure != NULL)
	{
		//过热保护，摩擦轮过烫时禁止运行摩擦轮
		if(shoot_control.fric1.shoot_motor_measure->temperate < 80.0f && \
			 shoot_control.fric2.shoot_motor_measure->temperate < 80.0f && \
			 shoot_control.shoot_mode > SHOOT_STOP )
		{
			#if ROBOT_FRICTION == friction_3508
				CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current, 0, 0);
			#elif ROBOT_FRICTION == friction_double
				CAN_cmd_friction(shoot_control.fric1.give_current, shoot_control.fric2.give_current, shoot_control.fric1_.give_current, shoot_control.fric2_.give_current);
			#endif
		}
		else
		{
			CAN_cmd_friction(0,0,0,0);
		}
	}
	else
	{
		CAN_cmd_friction(0,0,0,0);
	}
	
	//拨弹盘的控制电流值
	return shoot_control.given_current;
}

//射击热量检测
void shoot_calories_detect()
{		
	shoot_control.last_calories = shoot_control.calories;
	shoot_control.calories += shoot_control.damn_calories;
	shoot_control.shoot_delay = HAL_GetTick() - shoot_control.shoot_time;
}
