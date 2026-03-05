/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief      该任务是对机体运动速度估计，用于抑制打滑
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
#define VELOCITY_EPSILON 1e-4f	
#include "observe_task.h"
#include "kalman_filter.h"
#include "cmsis_os.h"
extern float pre_v;
KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体
extern uint8_t filter_flag;
float vaEstimateKF_F[4] = {1.0f, 0.001f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

//float vaEstimateKF_Q[4] = {10.0f, 0.0f, 
//                           0.0f, 2000.0f};    // Q矩阵初始值   表示系统模型的不确定性
////差不多但是扭
//float vaEstimateKF_R[4] = {2000.0f, 0.0f, //表示传感器测量的不确定性。R 越大，滤波器对测量的信任度越低
//                            0.0f,  0.05f}; 	
						   
float vaEstimateKF_Q[4] = {1.0f, 0.0f, 
                           0.0f, 1.0f};    // Q矩阵初始值   表示系统模型的不确定性   Q 越大，滤波器对模型的信任度越低。

float vaEstimateKF_R[4] = {120.0f, 0.0f, //表示传感器测量的不确定性。R 越大，滤波器对测量的信任度越低
                            0.0f,  120.0f}; 					   
////						   
						   						   														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
														 															 
extern INS_t INS;		
extern chassis_t chassis_move_balance;																 															 
extern uint8_t Rc_flag;																 
extern vmc_leg_t right;			
extern vmc_leg_t left;

float v_l=0.0f;
float v_r=0.0f;
float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//任务周期是3ms	
																 
void observe_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
	static float last_rc=0;
//		
	xvEstimateKF_Init(&vaEstimateKF);

  while(1)
	{  
		
		chassis_move_balance.wheel_motor[0].vel = chassis_move_balance.wheel_motor[0].speed_rpm * 0.00664267f;  //角速度
		chassis_move_balance.wheel_motor[1].vel = chassis_move_balance.wheel_motor[1].speed_rpm * 0.00664267f;  //角速度
		
		//计算出右轮的角速度和速度
		// wr= chassis_move_balance.wheel_motor[0].vel-INS.Gyro[0]+right.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		// vrb=wr*0.0755f+right.L0*right.d_theta*arm_cos_f32(right.theta)+right.d_L0*arm_sin_f32(right.theta);//机体b系的速度
		// //计算出左轮的角速度和速度
		// wl= chassis_move_balance.wheel_motor[1].vel+INS.Gyro[0]+left.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		// vlb=wl*0.0755f+left.L0*left.d_theta*arm_cos_f32(left.theta)+left.d_L0*arm_sin_f32(left.theta);//机体b系的速度
		//  aver_v=(vrb-vlb)/2.0f;
        //  xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_b[1],aver_v);
		pre_v=chassis_move_balance.v_filter2;
		chassis_move_balance.v_filter = 
		                                // aver_v;
		                               ((chassis_move_balance.wheel_motor[0].vel) - (chassis_move_balance.wheel_motor[1].vel))*(0.0755f)/2 ;
		v_r=chassis_move_balance.wheel_motor[0].vel*0.0755f;
		v_l=-chassis_move_balance.wheel_motor[1].vel*0.0755f;			
		xvEstimateKF_Update(&vaEstimateKF,-INS.MotionAccel_b[0],chassis_move_balance.v_filter);
			chassis_move_balance.v_filter2=vel_acc[0];//得到卡尔曼滤波后的速度
			// if(chassis_move_balance.v_filter2<0.010f&&chassis_move_balance.v_filter2>-0.010f)
			// {
			// 	chassis_move_balance.v_filter2=0.0f;
			// }
		//位控
		//  chassis_move_balance.x_filter=chassis_move_balance.x_filter+chassis_move_balance.v_filter*((float)3/1000.0f);
		    chassis_move_balance.x_filter=chassis_move_balance.x_filter+chassis_move_balance.v_filter*((float)2.5/1000.0f);
		    // if(chassis_move_balance.x_filter<1.0f&&chassis_move_balance.x_filter>-1.0f&&filter_flag==1)
			// {
			// 	chassis_move_balance.x_filter=0.0f;
			// 	filter_flag = 0;
			// }
//回撤补偿           
			// if(chassis_move_balance.v_filter2<0.3f&&chassis_move_balance.v_filter2>-0.3f&&filter_flag == 1)
			// {
			// 	chassis_move_balance.x_filter=0.0f;
			// 	chassis_move_balance.x_set=0.55f;
			// 	// chassis_move_balance.x_set=0.7f;
			// chassis_move_balance.x_set=chassis_move_balance.x_set+(left.L0*sinf(left.theta)+right.L0*sinf(right.theta))/2.0f; 
			// 	filter_flag = 0;
			// }            			
            // if(filter_flag == 0&&last_rc!=0&&(float)chassis_move_balance.chassis_RC->rc.ch[1]==0)
			// {
			// 	filter_flag = 1;
			// }
            // last_rc = (float)chassis_move_balance.chassis_RC->rc.ch[1];

//			osDelay(OBSERVE_TIME);
		
//		chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed))/2 ;
//	    chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg))/2 ;
	
//		chassis_move_balance.v_filter = (vrb-vlb)/2.0f;
	
//	    chassis_move_balance.wheel_motor[0].speed = 0.0004998609952f * chassis_move_balance.wheel_motor[0].speed_rpm;
//	    chassis_move_balance.wheel_motor[1].speed = 0.0004998609952f * chassis_move_balance.wheel_motor[1].speed_rpm;
//    	chassis_move_balance.v_filter = ((chassis_move_balance.wheel_motor[0].speed) - (chassis_move_balance.wheel_motor[1].speed))/2 ;	
////		
//		chassis_move_balance.v_wheel = ((chassis_move_balance.wheel_motor[0].vel) - (chassis_move_balance.wheel_motor[1].vel))*(0.0755f)/2 ;	
//		
//		xvEstimateKF_Update(&vaEstimateKF,-INS.MotionAccel_b[0],chassis_move_balance.v_wheel);
//		
		//原地自转的过程中v_filter和x_filter应该都是为0
//		chassis_move_balance.v_filter=aver_v;//得到卡尔曼滤波后的速度
	
    //    chassis_move_balance.x_filter2 =chassis_move_balance.x_filter*2.5;

//if(fabs(chassis_move_balance.v_filter2)>0.3){

//chassis_move_balance.move_flag=1;

//}

if(chassis_move_balance.w_flag==1){
	
		chassis_move_balance.v_filter=0;
		chassis_move_balance.x_filter=0;
	
}
//	//如果想直接用轮子速度，不做融合的话可以这样
//	chassis_move_balance.v_filter=(chassis_move_balance.wheel_motor[0].vel-chassis_move_balance.wheel_motor[1].vel)*(-0.0755f)/2.0f;
//	//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
//	chassis_move_balance.x_filter=chassis_move_balance.x_filter+chassis_move_balance.v_filter*((float)OBSERVE_TIME/1000.0f);
//
if(pre_v < -VELOCITY_EPSILON && chassis_move_balance.v_filter2 >= -VELOCITY_EPSILON && Rc_flag == 1)
{  
	if(chassis_move_balance.chassis_RC->rc.ch[1]==0)
    {
		Rc_flag = 3;
	}
	else
	{
		Rc_flag = 0;
	}
}
else if(pre_v > VELOCITY_EPSILON && chassis_move_balance.v_filter2 <= VELOCITY_EPSILON && Rc_flag == 2)
{
	if(chassis_move_balance.chassis_RC->rc.ch[1]==0)
    {
		Rc_flag = 3;
	}
	else
	{
		Rc_flag = 0;
	}
}
				 
	osDelay(OBSERVE_TIME);

	}
}

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维 
	//memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F)); 将vaEstimateKF_F开始复制大小为第三个参数复制到第一个参数到
	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
	memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

fp32  RAMP_float( fp32  final, fp32  now, fp32  ramp )
{
	  fp32  buffer = 0;
		
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}
