#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"
#include "stdbool.h"

#define hcan_t FDCAN_HandleTypeDef


typedef struct
{
	__packed struct{
    //原始数据
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    //计算后的数据
    float pos;
    float vel;
    float tor; //电机反馈的力矩
    float Kp;
    float Kd;
    float t_mos; //mos温度
    float t_motor; //电机温度
		uint32_t last_fdb_time; //电机反馈时间
	}fdb;
	__packed struct
	{
		float KP;
		float KD;
		float POS;
		float VEL;
		float TOR;
	}set;
	__packed struct
	{
	float	P_min;
	float	P_max;
	float	V_min;
	float	V_max;
	float	KP_min;
	float	KP_max;
	float	KD_min;
	float	KD_max;
	float	T_min;
	float	T_max;
	}param;
} MITMeasure_t;

typedef struct {
	
	__IO bool rxFrameFlag;
}CAN_t;
// 声明全局CAN结构体
extern __IO CAN_t can;

/* 错误状态枚举（可根据需要扩展） */
typedef enum {
    CAN_ERROR_NONE         = 0x00,
    CAN_ERROR_WARNING      = 0x01,   // 错误警告
    CAN_ERROR_PASSIVE      = 0x02,   // 错误被动
    CAN_ERROR_BUS_OFF      = 0x04,   // 总线关闭
    CAN_ERROR_PROTOCOL_ARB = 0x08,   // 协议错误（仲裁阶段）
    CAN_ERROR_PROTOCOL_DATA= 0x10,   // 协议错误（数据阶段）
    CAN_ERROR_STUFF        = 0x20,   // 填充错误
    CAN_ERROR_FORM         = 0x40,   // 格式错误
    CAN_ERROR_ACK          = 0x80,   // 应答错误
    CAN_ERROR_CRC          = 0x100,  // CRC错误
	CAN_ERROR_SEND		   = 0x200,  //发送失败
} CAN_ErrorStatus;
//rm motor data
//dji电机结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		uint32_t last_fdb_time;
} motor_measure_t;

/* 声明全局错误状态变量 */
extern __IO CAN_ErrorStatus can_error_status;


void bsp_can_init(void);
void can1_filter_init(void);
void can2_filter_init(void);
void can3_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcan1_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan2_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
uint8_t fdcan3_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

//void fdcan2_rx_callback(void);
//void fdcan3_rx_callback(void);
 uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq);
void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_ENABLE(FDCAN_HandleTypeDef *hcan, uint16_t id);
void Motor_MIT_MODE(FDCAN_HandleTypeDef *hcan, uint16_t id);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);

#endif /* __BSP_FDCAN_H_ */
