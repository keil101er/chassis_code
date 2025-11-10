/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef DM_8009_DRV_H
#define DM_8009_DRV_H

#include "struct_typedef.h"

#include "main.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;




#define MIT_MODE 		0x000
#define POS_MODE		0x100
#define SPEED_MODE		0x200

#define DM_JOINT_CAN hcan1
#define GIMBAL_CAN hcan2


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define P_MIN2 -12.5f
#define P_MAX2 12.5f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -10.0f
#define T_MAX2 10.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;



typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t;

typedef struct
{
	uint16_t mode;
	float wheel_T;//轮毂电机的输出扭矩，单位为N
	
	motor_fbpara_t para;	
}Wheel_Motor_t;



extern int float_to_uint(float x_float, float x_min, float x_max, int bits);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);




extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);


extern void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);


extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);


extern void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);


extern void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id);


extern void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);





#endif

