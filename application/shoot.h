/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
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

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

#define BULLET_HEAT_BEST  35

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             100
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              150
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
// m/s  diameter 60mm
#define FRIC_RPM_TO_SPEED           0.00314159265358793f
//拨弹速度
#define TRIGGER_SPEED               (-20.0f)
#define CONTINUE_TRIGGER_SPEED      (10.0f)
#define READY_TRIGGER_SPEED         (20.0f)


//子弹速度
#define BULLET_SPEED         (20.0f)


#define KEY_OFF_JUGUE_TIME          500 
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡弹时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.5f
#define BLOCK_TIME                  300
#define REVERSE_TIME                400
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.6f

//拨弹轮电机角度PID
#define TRIGGER_ANGLE_PID_KP        10.0f
#define TRIGGER_ANGLE_PID_KI        0.0f
#define TRIGGER_ANGLE_PID_KD        1.0f

//拨弹轮电机速度PID
#define TRIGGER_SPEED_PID_KP        60.0f
#define TRIGGER_SPEED_PID_KI        0.3f
#define TRIGGER_SPEED_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  9000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 2000.0f

#define TRIGGER_READY_PID_MAX_OUT   9000.0f
#define TRIGGER_READY_PID_MAX_IOUT  2000.0f

#define TRIGGER_ANGLE_PID_MAX_OUT  10.0f
#define TRIGGER_ANGLE_PID_MAX_IOUT 1.0f

#define FRIC_SPEED_PID_KP        6000.0f
#define FRIC_SPEED_PID_KI        18.0f
#define FRIC_SPEED_PID_KD        -1.0f
#define FRIC_PID_MAX_OUT         16300.0f
#define FRIC_PID_MAX_IOUT        10000.0f

#define SHOOT_HEAT_REMAIN_VALUE     20

typedef enum
{
    SHOOT_STOP = 0,//射击停止
	
    SHOOT_READY_FRIC,//摩擦轮准备
	
    SHOOT_READY_BULLET = 1,//拨弹盘准备
	
    SHOOT_READY,//射击准备
	
    SHOOT_BULLET,//射击
	
    SHOOT_CONTINUE_BULLET,//持续射击
	
    SHOOT_DONE,//射击完成
} shoot_mode_e;

typedef enum
{
		FIRE_STOP,
		FIREING
	
} fire_mode_e;



typedef struct
{
    shoot_mode_e shoot_mode;
	fire_mode_e fire_mode;
	
	
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
	const motor_measure_t *fricL_motor_measure;
    const motor_measure_t *fricR_motor_measure;
//    ramp_function_source_t fric1_ramp;
//    uint16_t fric_pwm1;
//    ramp_function_source_t fric2_ramp;
//    uint16_t fric_pwm2;
    pid_type_def trigger_Angle_pid;
    pid_type_def trigger_motor_pid;
	pid_type_def fric_motor_L_pid;
    pid_type_def fric_motor_R_pid;
	pid_type_def self_explosion_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
	fp32 fricL_speed;
    fp32 fricR_speed;
	fp32 fric_set;
    fp32 angle;
    fp32 pre_filter_angle;
    fp32 set_angle;
    int16_t given_current;
	int16_t fric_l_current;
    int16_t fric_r_current;
    int8_t ecd_count;
	uint8_t fric_enabled;     //摩擦轮开启判断
    uint8_t last_key;
    uint8_t shoot_key;
    uint8_t tright_mode;
    uint32_t shoot_key_time;
     uint8_t t_m;
   
    bool_t press_l;
//    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
	
uint8_t	shoot_send_flag;
	
	
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#endif
