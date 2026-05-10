/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control.底盘功率控制
  * @note       this is only controling 80 w power, mainly limit motor current set.
  *             if power limit is 40w, reduce the value JUDGE_TOTAL_CURRENT_LIMIT
  *             and POWER_CURRENT_LIMIT, and chassis max speed (include max_vx_speed, min_vx_speed)
  *             只控制80w功率，主要通过控制电机电流设定值,如果限制功率是40w，减少
  *             JUDGE_TOTAL_CURRENT_LIMIT和POWER_CURRENT_LIMIT的值，还有底盘最大速度
  *             (包括max_vx_speed, min_vx_speed)
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "controller.h"
#include <math.h>
#include "CAN_receive.h"

extern robot_status_t robot_state;
extern supercap_rx_msg_t supercap_rx_msg;
#define POWER_LIMIT (robot_state.chassis_power_limit)
#define WARNING_POWER 40.0f
#define WARNING_POWER_BUFF 50.0f

#define NO_JUDGE_TOTAL_CURRENT_LIMIT 64000.0f // 16000 * 4,
#define BUFFER_TOTAL_CURRENT_LIMIT 16000.0f
#define POWER_TOTAL_CURRENT_LIMIT 20000.0f

/**
 * @brief          limit the power, mainly limit motor current
 * @param[in]      chassis_power_control: chassis data
 * @retval         none
 */
/**
 * @brief          限制功率，主要限制电机电流
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */

fp32 total_current_limit = 0.0f;


extern RC_ctrl_t rc_ctrl;
float initial_give_power[4]; // initial power from PID calculation
float initial_total_power = 0;
float total_power = 0;
fp32 chassis_power_buffer = 0.0f;
void chassis_power_control(chassis_t *chassis_power_control)
{

	uint16_t max_power_limit = POWER_LIMIT; // hardcoded for test
	fp32 chassis_max_power = 0.0f;
	fp32 input_power = 0.0f;
	fp32 scaled_give_power[2] = {0.0f};
	fp32 command_current[2] = {0.0f};
	const fp32 current_to_torque[2] = {0.000396211f, 0.000366211f};

	fp32 chassis_power = 0.0f;
	
	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	fp32 a = 1.23e-07f;
	fp32 k2 = 1.453e-07f;
	fp32 constant = 0.5f;

	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
	PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);

	// input_power = max_power_limit - chassis_power_control->buffer_pid.out;
	// if (input_power < 45.0f)
	// {
	// 	input_power = 45.0f;
	// }
	if(supercap_is_online())
	{
		if(supercap_rx_msg.chassis_power_limit > 30.0f + max_power_limit)
		{
			input_power = 30.0f + max_power_limit;
		}
		else if(supercap_rx_msg.chassis_power_limit < 45)
		{
			input_power = 45.0f;
		}
		else
		{
			input_power = supercap_rx_msg.chassis_power_limit;
		}
	}
	else
	{
		input_power = max_power_limit - chassis_power_control->buffer_pid.out;
	}
	chassis_max_power = input_power;
	initial_total_power = 0.0f;

	// 首先获取所有初始电机的电机功率和电机总功率
	for (uint8_t i = 0; i < 2; i++) // first get all the initial motor power and total motor power
	{
		command_current[i] = chassis_power_control->wheel_motor[i].wheel_T / current_to_torque[i];
		initial_give_power[i] = fabsf(command_current[i]) * toque_coefficient *
								fabsf((fp32)chassis_power_control->wheel_motor[i].speed_rpm) +
								k2 * chassis_power_control->wheel_motor[i].speed_rpm *
								chassis_power_control->wheel_motor[i].speed_rpm +
								a * command_current[i] * command_current[i] +
								constant;

		if (initial_give_power[i] < 0.0f)
		{
			initial_give_power[i] = 0.0f;
		}

		initial_total_power += initial_give_power[i];
		continue;

		//		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
		//								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
		//								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;
		//
		//
		initial_give_power[i] = abs(chassis_power_control->wheel_motor[i].given_current)   // 读到电机的转矩电流反馈乘以
									* toque_coefficient									   // 转矩系数
									* abs(chassis_power_control->wheel_motor[i].speed_rpm) // 乘以转速
								+ k2 * chassis_power_control->wheel_motor[i].speed_rpm	   // k2乘以转速的二次方
									  * chassis_power_control->wheel_motor[i].speed_rpm +
								a * chassis_power_control->wheel_motor[i].given_current // 加a乘以电流转矩的二次方
									* chassis_power_control->wheel_motor[i].given_current +
								constant; // 加常量

		//		* toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
		//								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
		//								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

		if (initial_give_power < 0) // 不包含负功率暂时
			continue;
		initial_total_power += initial_give_power[i]; // 总功率
	}
	// initial_total_power=supercap_rx_msg.chassis_power;
	// total_power=initial_total_power;
	// if(supercap_is_online())
	// {
	// 	total_power=supercap_rx_msg.chassis_power;
	// }
	// else
	// {
		total_power=initial_total_power;
	// }
	if (total_power> chassis_max_power) // 判断是否大于最大功率
	{
		fp32 direct_power_scale = chassis_max_power / total_power;
		if (direct_power_scale < 0.0f)
		{
			direct_power_scale = 0.0f;
		}

		for (uint8_t i = 0; i < 2; i++)
		{
			chassis_power_control->wheel_motor[i].wheel_T *= direct_power_scale;
			chassis_power_control->wheel_motor[i].given_current =
				(int16_t)(chassis_power_control->wheel_motor[i].wheel_T / current_to_torque[i]);
		}
		return;
	}
}
