#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm_8009_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "user_lib.h"
// Remote control status macro (essentially the position of elements in the array)
//遥控器状态宏 （本质是在数组中元素的位置）

//in the beginning of task ,wait a time
//任务开始后空闲一段时间
#define CHASSIS_TASK_INIT_TIME 1000

//the channel of choosing chassis mode
////选择底盘状态的开关通道号
#define CHASSIS_MODE_CHANNEL 0



#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S

#define CHASSIS_LEG_KEY KEY_PRESSED_OFFSET_CTRL 









//reducation of 3508 motor
//m3508电机的减速比
#define M3508_MOTOR_REDUCATION 15.764705882f

//m3508 rpm change to chassis speed   576.096  3.14 *07425 = 0.233145
////m3508转子转速(rpm)转化成底盘速度(m/s)的比例，c=pi*r/(30*k)，k为电机减速比
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

//m3508 rpm change to motor angular velocity
//m3508转子转速(rpm)转换为输出轴角速度(rad/s)的比例
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

//m3508 current change to motor torque
//m3508转矩电流(-16384~16384)转为成电机输出转矩(N.m)的比例
//c=20/16384*0.3  
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f




#define ROLL_PID_KP 40.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.1f

//#define ROLL_PID_KP 25.0f
//#define ROLL_PID_KI 0.0f 
//#define ROLL_PID_KD 1.0f
#define ROLL_PID_MAX_OUT  100.0f
#define ROLL_PID_MAX_IOUT 0.0f



//#define TP_PID_KP 0.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 0.0f
//adjust roll before
//调节roll之前
//#define TP_PID_KP 15.0f
//#define TP_PID_KI 0.0f 
//#define TP_PID_KD 0.8f

////#define TP_PID_KP 10.0f
////#define TP_PID_KI 0.0f 
////#define TP_PID_KD 0.8f
//#define TP_PID_MAX_OUT  5.0f
//#define TP_PID_MAX_IOUT 0.0f


//#define TURN_PID_KP 5.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 0.8f
////#define TURN_PID_KP 0.0f
////#define TURN_PID_KI 0.0f 
////#define TURN_PID_KD 0.0f
////Rated torque of the hub motor
////轮毂电机的额定扭矩
//#define TURN_PID_MAX_OUT  3.0f
//#define TURN_PID_MAX_IOUT 0.0f



#define TP_PID_KP 25.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 8.0f
#define TP_PID_MAX_OUT  5.0f
#define TP_PID_MAX_IOUT 0.0f


//#define TURN_PID_KP 6.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 200.0f
#define TURN_PID_KP 15.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 2.5f
//Rated torque of the wheel hub motor
//轮毂电机的额定扭矩
#define TURN_PID_MAX_OUT  5.0f
#define TURN_PID_MAX_IOUT 0.0f



#define WZ_PID_KP 1.0f
#define WZ_PID_KI 0.0f 
#define WZ_PID_KD 0.8f
//#define TURN_PID_KP 0.0f
//#define TURN_PID_KI 0.0f 
//#define TURN_PID_KD 0.0f
//Rated torque of the wheel hub motor
//轮毂电机的额定扭矩
#define WZ_PID_MAX_OUT  1.5f
#define WZ_PID_MAX_IOUT 0.0f











//Chassis motor speed loop PID
//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 18000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f  
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f



#define POWER_PID_KP 9.0f
#define POWER_PID_KI 0.001f
#define POWER_PID_KD -0.08f
#define POWER_PID_MAX_OUT 10.0f
#define POWER_PID_MAX_IOUT 0.2f



//Chassis control mode
//底盘控制模式
typedef enum
{
  CHASSIS_REMOTE_MODE,  //遥控模式 Remote control mode
  CHASSIS_BALANCE_MODE, //平衡模式 Balance mode
  CHASSIS_DOWN_MODE,    //DOWN模式 
} chassis_mode_e;


typedef struct
{
	//关节电机参数 Joint motor parameters
 	Joint_Motor_t joint_motor[5];
	//轮毂电机参数 Wheel motor parameters
    //Wheel_Motor_t wheel_motor[2];
    //应该是电池 Battery
	float vbus;
	uint8_t vbus_mode; // 1-For 4s 
										//	2-For 6s

	float v_set;//目标速度设定值m/s Target speed set value m/s
	float x_set;//目标位置设定值m Target position set value m
	float target_v;
	float target_x;

	float turn_set;//目标yaw角设定值 Target yaw angle set value

	float roll_set;	//目标roll角设定值 Target roll angle set value
	float roll_target;
	
	
	float roll_x;
	float phi_set;
	float theta_set;

	float target_leg;	
	float leg_set;//期望腿长，单位是m Expected leg length, unit is m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s Velocity of the chassis after filtering, unit is m/s
	float v_filter2;
	
	float v_wheel;
	float x_filter;//滤波后的车体位置，单位是m Position of the chassis after filtering, unit is m
	float x_filter2;//滤波后的车体位置，单位是m Position of the chassis after filtering, unit is m

	
	float v;
	float x;
		
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差 Leg angle error

	float turn_T;//yaw角补偿 Yaw angle compensation
	float roll_f0;//roll角补偿 Roll angle compensation

	float leg_tp;//防劈叉补偿 Anti-split compensation

	uint8_t start_flag;//启动标志 Start flag

	uint8_t jump_flag_r;//右腿跳跃标志 Right leg jump flag
	uint8_t jump_flag_l;//左腿跳跃标志 Left leg jump flag

	uint8_t prejump_flag;//预跳跃标志 Pre-jump flag
	uint8_t recover_flag;//一种情况下的倒地自起标志 Recovery flag
	uint8_t help_jump_flag;
	uint8_t	recover_time;


    const RC_ctrl_t *chassis_RC;  //获取遥控器指令 Get remote control commands
/*	typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[2];
        } rc;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        } mouse;
        __packed struct
        {
                uint16_t v;
        } key;

} RC_ctrl_t;
*/
	chassis_mode_e chassis_mode;               //底盘控制模式状态机 Chassis Control Mode State Machine
    chassis_mode_e last_chassis_mode;          //底盘上次控制模式状态机 Last Chassis Control Mode State Machine
	
    motor_measure_t wheel_motor[5];
    uint8_t DUBS_ON;   
  
    pid_type_def buffer_pid;
    pid_type_def motor_speed_pid[2];  
	
	float yaw_motor_angle;	
	float relative_angle;
	
	motor_measure_t motor_chassis[5];
	
	float Wz_set;  //目标偏航角速度设定值 Target yaw angular velocity set value
	float Wz;
	float Wz_target;
	uint8_t w_flag;	     //底盘是否跟随云台标志 Chassis whether to follow the gimbal
	uint8_t move_flag;
	
} chassis_t;

extern void ChassisR_task(void);
extern void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legr,pid_type_def *wheel);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins);
extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void mySaturate(float *in,float min,float max);
extern void Pensation_init(pid_type_def *roll,pid_type_def *Tp,pid_type_def *turn,pid_type_def *wz);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,INS_t *ins,float *LQR_K,pid_type_def *leg);
//extern void chassis_set_mode(chassis_t *chassis_move_mode);
extern void chassis_recover(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins,float *LQR_K,pid_type_def *leg);
extern uint8_t recover_detect(chassis_t *chassis);
extern void control_motor(chassis_t *chassis );

#endif





