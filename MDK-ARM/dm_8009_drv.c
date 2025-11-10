#include "dm_8009_drv.h"

#include "cmsis_os.h"

#include "main.h"


#include "detect_task.h"






static CAN_TxHeaderTypeDef  joint_tx_message;



/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}






void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}

void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode)
{
  motor->mode=mode;
  motor->para.id=id;
}




/**
************************************************************************
* @brief:      	dm4310_fbdata: 获取DM4310电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/

/*************************************************需要修改测试****************************/
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{//返回的数据有8个字节
	  motor->para.id = (rx_data[0])&0x0F;
	  motor->para.state = (rx_data[0])>>4;
	  motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	  motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	  motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
	  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
	  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  // (-10.0,10.0)
	  motor->para.Tmos = (float)(rx_data[6]);
	  motor->para.Tcoil = (float)(rx_data[7]);
	}
}









void enable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
//	uint16_t id = motor_id + mode_id;
//	
	uint32_t send_mail_box;
    joint_tx_message.StdId = motor_id+mode_id;
    joint_tx_message.IDE = CAN_ID_STD;
    joint_tx_message.RTR = CAN_RTR_DATA;
    joint_tx_message.DLC = 0x08;
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;

    HAL_CAN_AddTxMessage(&DM_JOINT_CAN, &joint_tx_message, data, &send_mail_box);
	
}


void disable_motor_mode(CAN_HandleTypeDef *hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
//	uint16_t id = motor_id + mode_id;
//	
	uint32_t send_mail_box;
    joint_tx_message.StdId = motor_id+mode_id;
    joint_tx_message.IDE = CAN_ID_STD;
    joint_tx_message.RTR = CAN_RTR_DATA;
    joint_tx_message.DLC = 0x08;
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;

    HAL_CAN_AddTxMessage(&DM_JOINT_CAN, &joint_tx_message, data, &send_mail_box);
	
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT模式下的电机控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef *hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data[8];
	uint32_t send_mail_box;
    joint_tx_message.StdId = motor_id;
    joint_tx_message.IDE = CAN_ID_STD;
    joint_tx_message.RTR = CAN_RTR_DATA;
    joint_tx_message.DLC = 0x08;
	
	
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	
	
	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
    HAL_CAN_AddTxMessage(&DM_JOINT_CAN, &joint_tx_message, data, &send_mail_box);
	
	
	
	
}


