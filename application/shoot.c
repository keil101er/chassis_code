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
#include "CANdata_analysis.h"

#define shoot_fric_off() fric_off()   // 关闭两个摩擦轮
#define shoot_laser_on() laser_on()   // 激光开启宏定义
#define shoot_laser_off() laser_off() // 激光关闭宏定义
// 微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // I7

uint8_t shoot_single_flag = 0;
uint16_t shoot_single_cnt = 0;
uint8_t auto_shoot_busy = 0; // 上位机自动开火忙标志：1=正在执行单发，0=空闲可接收新命令
uint16_t auto_shoot_cnt = 0; // 控制弹频
float relative_angle = 0.0f;
shoot_control_t shoot_control; // 射击数据
extern k_b;
//***************裁判系统数据获取*****************************************
extern power_heat_data_t power_heat_data_t1;
//**************裁判系统数据获取*****************************************
extern robot_status_t robot_state;
// 云台数据获取
extern c_fbpara_t C_data;
extern float gimbal_mode;

extern chassis_t chassis_move_balance;
extern uint8_t RC_KEY_flag;
float last_trigger_set_speed;
// *************初始化射击模块****************
void shoot_init(void)
{
    // 初始化拨弹盘pid
    static const fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    static const fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    //*************一开始射击模式设置为 停止模式***********
    shoot_control.shoot_mode = SHOOT_STOP;
    // 读取遥控器的指针
    shoot_control.shoot_rc = get_remote_control_point();
    // 获取2006电机数据反馈
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point(); // 获取拨弹轮

    // 拨弹轮速度初始化
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    // 拨弹轮角度初始化
    PID_init(&shoot_control.trigger_Angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT);
    shoot_control.ecd_count = 0;
    shoot_control.angle = -shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;

    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.pre_filter_angle = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.press_l_time = 0;
    shoot_control.key = 0;

    SHOOT_ON_KEYBOARD;
    shoot_control.fire_mode = FIRE_STOP;
    shoot_control.shoot_send_flag = 0;
}

// 射击数据更新
static void shoot_feedback_update(void)
{
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    // 拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度，转子转一圈ecd_count加减1；
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }
    // 防止ecd_count值溢出，当转子旋转18圈后重置为-17；反方向同理，因为电机的范围用的是-pi到pi，
    //测试
    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    // 计算输出轴角度
    shoot_control.angle = -(shoot_control.ecd_count * (ECD_RANGE + 1) + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    // shoot_control.angle=0.3*shoot_control.pre_filter_angle+0.7*shoot_control.angle;
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;

    // 长按计时(键鼠模式)
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
    // 射击开关下档时间计时(遥控器模式)
    if (shoot_control.shoot_rc->rc.ch[4] > 600)
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME && (shoot_control.shoot_mode != 1))
        {
            shoot_control.rc_s_time++;
        }
    }
    // if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    // {

    //     if (shoot_control.rc_s_time < RC_S_LONG_TIME)
    //     {
    //         shoot_control.rc_s_time++;
    //     }
    // }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    relative_angle = rad_format(shoot_control.set_angle - shoot_control.angle);
}

// 堵转倒转处理
static void trigger_motor_turn_back(void)
{
    last_trigger_set_speed = shoot_control.speed_set;
    if (shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set =TRIGGER_SPEED;
        //shoot_control.speed_set = 0.0;
    }
    else // 如果卡弹时间>700，拨弹开启反转
    {
        shoot_control.speed_set = -TRIGGER_SPEED; // 开启反转
        // shoot_control.speed_set = 0.0;
    }

    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME) // 根据拨弹轮速度判断是否卡弹
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0; // 恢复时间
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
    
    if(last_trigger_set_speed == -shoot_control.speed_set)
    {
        PID_clear(&shoot_control.trigger_motor_pid);
    }
}

// 射击控制，控制拨弹电机角度，完成一次发射
static void shoot_bullet_control(void)
{
    // 每次拨动 1/10PI的角度
    shoot_control.set_angle = rad_format(shoot_control.angle + PI_TEN);
    shoot_control.move_flag = 1;
    shoot_single_cnt = 0;
    auto_shoot_cnt = 0;
    PID_clear(&shoot_control.trigger_Angle_pid);
    PID_clear(&shoot_control.trigger_motor_pid);

    //    if(shoot_control.key == SWITCH_TRIGGER_ON)
    //    {

    //        shoot_control.shoot_mode = SHOOT_DONE; 
    //    }
}

//************************************************************************
// 射击状态机设置
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;
    if(RC_KEY_flag)
    {
        if(shoot_control.shoot_mode != 2)
        {
        if (shoot_control.press_l_time >= PRESS_LONG_TIME)
        {
            shoot_control.shoot_mode = 1; // 连发
        }
        else if (shoot_control.press_l_time == 0 && shoot_control.last_press_l == 1)
        {
            shoot_control.shoot_mode = 2; // 单发
        }
        }
    }
    // 下拨进入射击状态
    else
    {
        //if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL])&&shoot_control.fire_mode == FIREING )
        if(shoot_control.shoot_mode != 2)
        {
            if (shoot_control.rc_s_time>=RC_S_LONG_TIME)
            {
              shoot_control.shoot_mode = 1;//连发
              shoot_control.rc_s_time=0;
            }
            else if((shoot_control.rc_s_time > 0 ) && (shoot_control.rc_s_time < RC_S_LONG_TIME)
                    && (shoot_control.shoot_rc->rc.ch[4] <= 600) )
            {
                shoot_control.shoot_mode = 2;//单发 Single shot
                shoot_control.rc_s_time=0;
            }
        }
    }
    // 上位机自动开火：仅在空闲时接受新的开火命令
    if (C_data.MODE == 1 && gimbal_mode == 2 )
    {
        shoot_control.shoot_mode = 1; // 触发连发
    }
    else if (C_data.MODE == 0 && gimbal_mode == 2 )
    {                               
        shoot_control.shoot_mode = 0;
        shoot_control.given_current = 0;
        shoot_control.speed_set = 0.0f;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 上拨判断， 一次开启，再次关闭
    // 波上去一次
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.fire_mode == FIRE_STOP))
    {
        // shoot_control.shoot_mode = SHOOT_READY_BULLET;
        shoot_control.fire_mode = FIREING;  //开摩擦轮
        shoot_control.shoot_send_flag = 1;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.fire_mode != FIRE_STOP))
    {
        // shoot_control.shoot_mode = SHOOT_STOP;
        shoot_control.fire_mode = FIRE_STOP; // 停止射击
        shoot_control.shoot_send_flag = 0;
    }
    if(RC_KEY_flag)
    // if (shoot_control.shoot_rc->rc.s[1] == 3)
    {
        // q键按下一下开启
        if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) &&
            !(shoot_control.last_key & SHOOT_ON_KEYBOARD))
        {
            shoot_control.fire_mode = FIREING; // 射击开摩擦轮
            shoot_control.shoot_send_flag = 1;
        }
        // e键按下一下关闭
        else if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) &&
                 !(shoot_control.last_key & SHOOT_OFF_KEYBOARD))
        {
            shoot_control.fire_mode = FIRE_STOP; // 停止射击
            shoot_control.shoot_send_flag = 0;
        }
    }
    // 保存按键状态
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]; // 存储遥控器上一次的状态
    shoot_control.last_key = shoot_control.shoot_rc->key.v;

    /////////////////////////////////////////////////////////////////////////////////
    // 读取热量限制
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

    // 如果云台状态是 无力状态，就关闭射击
    //  if (chassis_move_balance.start_flag ==0)
    //  {
    //      shoot_control.shoot_mode = SHOOT_STOP;
    //  	shoot_control.given_current =0;
    //  	shoot_control.fire_mode = FIRE_STOP;
    //  	shoot_control.shoot_send_flag = 0;
    //  }
}

extern int board_receive_data[8];

// 射击循环**********************************************************************************
// 返回的值为given_current

int16_t shoot_control_loop(void)
{
    shoot_set_mode(); // shoot_ready_buu
    shoot_feedback_update();
    //	if(shoot_control.shoot_mode == SHOOT_READY_BULLET)
    //	{
    //	//设置拨弹轮的拨动速度,并开启堵转反转处理
    //	shoot_control.speed_set = READY_TRIGGER_SPEED;  //拨弹盘准备速度
    //	trigger_motor_turn_back();
    //	shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
    //	shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    //}
    float heat_remain = (float)robot_state.shooter_barrel_heat_limit -
    (float)power_heat_data_t1.shooter_17mm_barrel_heat;
    if ((power_heat_data_t1.shooter_17mm_barrel_heat + BULLET_HEAT_BEST > robot_state.shooter_barrel_heat_limit))
    {
        shoot_control.speed_set = 0.0f;  //拨弹盘准备速度
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
    }
    else
    {
    // 以下都会进行
    if (shoot_control.shoot_mode == 0)
    {
        // shoot_laser_off();
        shoot_control.given_current = 0;
    }
    //****************SHOOT_READY_BULLET**************************
    else if (shoot_control.shoot_mode == 1)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        // 键鼠模式
        if(gimbal_mode !=2)
        {
            if(RC_KEY_flag)
            {

                if (shoot_control.press_l)
                {
                if (heat_remain <= BULLET_HEAT_BEST)
                {
                    shoot_control.speed_set = 0.0f;
                }
                else if (heat_remain >= BULLET_HIGH_SPEED_HEAT_BEST)
                {
                    shoot_control.speed_set = TRIGGER_SPEED;
                }
                else
                {
                    shoot_control.speed_set = -7.0f;
                }
                trigger_motor_turn_back();
                // 计算拨弹轮电机PID
                PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
                shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
                }
                else
                {
                    shoot_control.shoot_mode = 0;
                    shoot_control.given_current = 0;
                    shoot_control.speed_set = 0.0f;
                }
            }
            else
            {
                if(shoot_control.shoot_rc->rc.ch[4] >500)
                {
                if (heat_remain <= BULLET_HEAT_BEST)
                {
                    shoot_control.speed_set = 0.0f;
                }
                else if (heat_remain >= BULLET_HIGH_SPEED_HEAT_BEST)
                {
                    shoot_control.speed_set = TRIGGER_SPEED;
                }
                else
                {
                    shoot_control.speed_set = -7.0f;
                }
                    trigger_motor_turn_back();
                    //计算拨弹轮电机PID
                    PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
                    shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
                }
                else
                {
                    shoot_control.shoot_mode=0;
                    shoot_control.given_current = 0;
                    shoot_control.speed_set =0.0f;
                }
            }
        }
        else
        {
            if(C_data.MODE == 1 && !k_b)
            {
                if (heat_remain <= BULLET_HEAT_BEST)
                {
                    shoot_control.speed_set = 0.0f;
                }
                else if (heat_remain >= BULLET_HIGH_SPEED_HEAT_BEST)
                {
                    shoot_control.speed_set = TRIGGER_SPEED;
                }
                else
                {
                    shoot_control.speed_set = -7.0f;
                }
                trigger_motor_turn_back();
                // 计算拨弹轮电机PID
                PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
                shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
            }
            else
            {
                shoot_control.shoot_mode = 0;
                shoot_control.given_current = 0;
                shoot_control.speed_set = 0.0f;                
            }
        }
    }
    else if (shoot_control.shoot_mode == 2)
    {
        if (shoot_single_flag == 0)
        {
            shoot_bullet_control();
            shoot_single_flag++;
        }
        if (shoot_single_flag == 1)
        {
            // if (((fabs(rad_format(shoot_control.set_angle-shoot_control.angle))>0.05f)||fabs(shoot_control.speed)>0.5f)&& shoot_single_cnt<1500)
            if (fabs(rad_format(shoot_control.set_angle - shoot_control.angle)) > 0.05f && shoot_single_cnt < 200)
            {
                shoot_single_cnt++;
                // 没到达一直设置旋转角度
                //  shoot_control.speed_set = TRIGGER_SPEED;
                //  trigger_motor_turn_back();
                // 计算拨弹轮电机PID
                shoot_control.speed_set = -PID_angle_calc(&shoot_control.trigger_Angle_pid, shoot_control.angle, shoot_control.set_angle);
                PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
                shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
            }
            else
            {
                // shoot_control.given_current = 0;
                auto_shoot_cnt++;
                shoot_control.given_current = 0;
                // if (fabs(shoot_control.speed) > 0.5f)
                // {
                //     PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, 0);
                //     shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
                // }
                // else
                // {
                //     shoot_control.given_current = 0;
                // }
                if (auto_shoot_cnt >=10)
                {
                    shoot_control.given_current = 0;
                    auto_shoot_cnt = 0;
                    shoot_single_cnt = 0;
                    shoot_control.shoot_mode = 0;
                    shoot_single_flag = 0;
                    shoot_control.speed_set = 0.0f;
                    auto_shoot_busy = 0; // 单发完成，解锁，允许接收下一次上位机命令
                }
            }
        }
    }
    // 自瞄模式控制开火
    //  if(C_data.MODE==1&&gimbal_mode==2)
    //  {

    // }
    // else if(C_data.MODE==0&&gimbal_mode==2)
    // {
    //      shoot_control.shoot_mode=0;
    //      shoot_control.given_current = 0;
    //      shoot_control.speed_set =0.0f;
    // }

    // if(C_data.MODE==1&&gimbal_mode==2)
    // {
    //     shoot_control.speed_set = -20;  //拨弹盘准备速度
    //     trigger_motor_turn_back();
    //     //计算拨弹轮电机PID
    //     PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
    //     shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
    // }
    // else if(C_data.MODE==0&&gimbal_mode==2)
    // {
    //     shoot_control.shoot_mode=0;
    //     shoot_control.given_current = 0;
    //     shoot_control.speed_set =0.0f;
    // }
    get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);

    //暂时注释

    }
    return shoot_control.given_current;
}
