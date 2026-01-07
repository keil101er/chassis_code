/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "pid.h"
#include "chassisR_task.h"
#include "chassisL_task.h"

#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮
#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) //I7



shoot_control_t shoot_control;          //射击数据

//***************裁判系统数据获取*****************************************
extern power_heat_data_t power_heat_data_t1;
//**************裁判系统数据获取*****************************************
extern robot_status_t robot_state;




extern chassis_t chassis_move_balance;



// *************初始化射击模块****************
void shoot_init(void)
{
	//初始化拨弹盘pid
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    //*************一开始射击模式***8设置为 停止模式***********
    shoot_control.shoot_mode = SHOOT_STOP;
	//读取遥控器的指针
    shoot_control.shoot_rc = get_remote_control_point();
	//获取2006电机数据反馈
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();//获取拨弹轮
	
    //拨弹轮初始化
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);

    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
	
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
	
    shoot_control.key_time = 0;
    shoot_control.press_l_time = 0;
    shoot_control.key = 0;

    SHOOT_ON_KEYBOARD;
}

// 射击数据更新
static void shoot_feedback_update(void)
{
	static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
	
	    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度，转子转一圈ecd_count加减1；
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }
//防止ecd_count值溢出，当转子旋转18圈后重置为-17；反方向同理，因为电机的范围用的是-pi到pi，
    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
	
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	
	    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }
    //射击开关下档时间计时（连发模式计时）
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }
	
	
}

// 堵转倒转处理
static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
          shoot_control.speed_set = -20;
//					shoot_control.speed_set = 0.0;
    }
    else//如果卡弹时间>700，拨弹开启反转
    {
        shoot_control.speed_set = 20;//开启反转
	   	//shoot_control.speed_set = 0.0;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)//根据拨弹轮速度判断是否卡弹
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;//恢复时间
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}


//// 射击控制，控制拨弹电机角度，完成一次发射
//static void shoot_bullet_control(void)
//{
//    //每次拨动 1/4PI的角度
//    if (shoot_control.move_flag == 0)
//    {
//        shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
//        shoot_control.move_flag = 1;
//    }
//    if(shoot_control.key == SWITCH_TRIGGER_OFF)
//    {

//        shoot_control.shoot_mode = SHOOT_DONE;
//    }
//    //到达角度判断
//    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
//    {
//        //没到达一直设置旋转角度
//        shoot_control.trigger_speed_set = TRIGGER_SPEED;
//        trigger_motor_turn_back();
//    }
//    else
//    {
//        shoot_control.move_flag = 0;
//    }
//}


//************************************************************************
// 射击状态机设置
static void shoot_set_mode(void)
{
	static int8_t last_s = RC_SW_UP;
    //上拨判断， 一次开启，再次关闭
	//波上去一次
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.fire_mode == FIRE_STOP) )
    {
     //shoot_control.shoot_mode = SHOOT_READY_BULLET;
		 shoot_control.fire_mode = FIREING;  //射击 //开摩擦轮
		shoot_control.shoot_send_flag = 1;
		
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.fire_mode != FIRE_STOP))
    {
       //shoot_control.shoot_mode = SHOOT_STOP;
	            shoot_control.fire_mode = FIRE_STOP; //停止射击
				shoot_control.shoot_send_flag = 0;
    }

        //下拨进入射击状态
        //if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])&&shoot_control.fire_mode == FIREING )
		if (shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL] ==2 )
        {
          shoot_control.shoot_mode = 1;//开拨弹
        }
		else if(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL] ==3)
		{
		    shoot_control.shoot_mode = 0;
						
		}
		
		last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];//存储遥控器上一次的状态
	
				
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//q键按下一下开启
	if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && 
        !(shoot_control.last_key & SHOOT_ON_KEYBOARD)) 
    {
       	
		shoot_control.fire_mode = FIREING;  //射击 //开摩擦轮
		shoot_control.shoot_send_flag = 1;
    
	}
    //e键按下一下关闭
    if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && 
        !(shoot_control.last_key & SHOOT_OFF_KEYBOARD)) 
    {
	            shoot_control.fire_mode = FIRE_STOP; //停止射击
				shoot_control.shoot_send_flag = 0;   
	}
	
	 // 保存按键状态
    shoot_control.last_key = shoot_control.shoot_rc->key.v; 		
				
				
				
				
				
				
				
				
	/////////////////////////////////////////////////////////////////////////////////
    //读取热量限制
//    get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);
    
//    //进行热量限制
//	if( !toe_is_error(REFEREE_TOE) && (shoot_control.heat + 60 > shoot_control.heat_limit) )
//    {
//		if(shoot_control.shoot_mode == SHOOT_READY_BULLET)
//        {
//            shoot_control.shoot_mode =SHOOT_STOP;
//        }
//    }
//////////////////////////////////////////////////////////////////////

    //如果云台状态是 无力状态，就关闭射击
    if (chassis_move_balance.start_flag ==0)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
		shoot_control.given_current =0;
		shoot_control.fire_mode = FIRE_STOP;
		shoot_control.shoot_send_flag = 0;
    }

}




extern int board_receive_data[8];

// 射击循环**********************************************************************************
//返回的值为given_current



int16_t shoot_control_loop(void)
{
    shoot_set_mode();  //shoot_ready_buu
    shoot_feedback_update();
//	if(shoot_control.shoot_mode == SHOOT_READY_BULLET)
//	{    
//	//设置拨弹轮的拨动速度,并开启堵转反转处理
//	shoot_control.speed_set = READY_TRIGGER_SPEED;  //拨弹盘准备速度
//	trigger_motor_turn_back();
//	shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
//	shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
//}

	
			//以下都会进行
    if(shoot_control.shoot_mode == 0)
    {
        //shoot_laser_off();
        shoot_control.given_current = 0;
    }
	//****************SHOOT_READY_BULLET**************************
    if(shoot_control.shoot_mode == 1)
    {
	shoot_control.speed_set = -20;  //拨弹盘准备速度
	trigger_motor_turn_back();
	shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
	shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
		
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
		//测试
		//shoot_control.given_current = 300;
    }

	
	   get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);

	
	    if ((power_heat_data_t1.shooter_17mm_1_barrel_heat + BULLET_HEAT_BEST > robot_state.shooter_barrel_heat_limit)) 
	{

		shoot_control.given_current=0;
    }
		

	
    return shoot_control.given_current;
}    