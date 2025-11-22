/****************************LIFT*******************************/
/****************************CAN2*******************************/
//左下角的DM4310贴纸标签为6


#include "chassisL_task.h"
#include "can.h"
#include "shoot.h"
#include "cmsis_os.h"


vmc_leg_t left;
      
	  
float LQR_K_L[12]={  
//  -3.3259  , -0.2272,   -1.2959,   -1.2122 ,   3.3133,    0.3249,
//    2.5219 ,   0.2142 ,   1.8118 ,   1.6073 ,   8.2857 ,   0.4180
-11.0224 , -2.664 ,-11.6436,	-20.2754 , 30.2361 , 3.3863,
10.2554 ,  0.9963 ,   1.0764  ,  3.2532 , 40.8928  , 3.5782		
// 短距离不会晃动，有扰动后疯狂抖动
// 偏航
//   -7.9542 ,  -0.7620 ,  -2.3764  , -3.8669 ,   9.5258,    1.4091,
//   11.9374  ,  1.0310 ,   1.4607  ,  1.0218  , 3.6514 ,   1.3526	
//	
	
   
};

extern float Poly_Coefficient[12][4];

extern chassis_t chassis_move_balance;

float jump_time_l;
extern float jump_time_r;	
extern shoot_control_t shoot_control;          //射击数据
pid_type_def LegL_Pid;	

pid_type_def jump_pid_L;//跳跃pid
float leg_l_pid_int=0;		//腿长pid误差积分
float jump_pid_i_L=0;		//跳跃pid积分分量
const static float jump_pid[3] = {600.0f,0.0f,100.0f};
// {450.0f,0.0f,500.0f};//{LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
float jumpF0_L=17.2f;//左腿跳跃初始力

extern INS_t INS;
uint32_t CHASSL_TIME=1;	
int16_t	  shoot_can_set_current;
	  
void ChassisL_task(void)
{
  while(INS.ins_flag==0)//等待IMU初始化完成
	{
	  osDelay(1);	
	}	
		shoot_control.shoot_send_flag = 0;

	
        ChassisL_init(&chassis_move_balance,&left,&LegL_Pid);// 初始化左边两个关节电机和左边轮毂电机的 id 和控制模式、初始化腿部
		PID_init(&jump_pid_L, PID_POSITION, jump_pid, 90.0f, 0.0f);
		    //shoot init
            // 射击初始化
            shoot_init();

	while(1) 
	{	
	
		chassisL_feedback_update(&chassis_move_balance,&left,&INS);// 更新数据
		
		shoot_can_set_current = shoot_control_loop();
		
		chassisL_control_loop(&chassis_move_balance,&left,&INS,LQR_K_L,&LegL_Pid);//控制计算
	  
//		shoot_can_set_current = shoot_control_loop();        //射击任务控制循环，fire_key有
//			
//			
		if(chassis_move_balance.start_flag==1)	
	
	{
		
	mit_ctrl(&hcan2,0x08, 0.0f, 0.0f,0.0f, 0.8f,left.torque_set[1]);//left.torque_set[1]
	osDelay(CHASSL_TIME);
	mit_ctrl(&hcan2,0x06, 0.0f, 0.0f,0.0f, 0.8f,left.torque_set[0]);
	osDelay(CHASSL_TIME);

	chassis_move_balance.wheel_motor[1].given_current = chassis_move_balance.wheel_motor[1].wheel_T/0.000366211f;
			
	CAN_cmd_gimbal(-chassis_move_balance.wheel_motor[1].given_current,0,shoot_can_set_current,0);
	osDelay(CHASSL_TIME);	
////		
//		
//		mit_ctrl(&hcan2,0x08, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
//		osDelay(CHASSL_TIME);
//		mit_ctrl(&hcan2,0x06, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
//		osDelay(CHASSL_TIME);		  
//		CAN_cmd_gimbal(0,0,0,0);
//		osDelay(CHASSL_TIME);		
//		
//		CAN_cmd_gimbal(0,0,shoot_can_set_current,0);
//		osDelay(CHASSL_TIME);	
		
	 }
	
	 	else if(chassis_move_balance.start_flag==0)	
		{
			
		    mit_ctrl(&hcan2,0x08, 0.0f, 0.0f,0.0f, 0.8f,0.0f);//left.torque_set[1]
			osDelay(CHASSL_TIME);
			mit_ctrl(&hcan2,0x06, 0.0f, 0.0f,0.0f, 0.8f,0.0f);
			osDelay(CHASSL_TIME);
			CAN_cmd_gimbal(0,0,0,0);
			osDelay(CHASSL_TIME);
			
		}
		

//		if(chassis_move_balance.start_flag==1)	{
//			
//			
//			
////测试
//		shoot_control.speed_set = -READY_TRIGGER_SPEED;
//		PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
//        shoot_can_set_current = (int16_t)(shoot_control.trigger_motor_pid.out);
//			
//       CAN_cmd_gimbal(0,0,shoot_can_set_current,0);
//	   osDelay(CHASSL_TIME);
//		}
	}
}



void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,pid_type_def *legl)
{
    const static float legl_pid[3] = 
	                                // {0,0,0};
	                                {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};

	joint_motor_init(&chassis->joint_motor[2],6,MIT_MODE);//发送id为6
	joint_motor_init(&chassis->joint_motor[3],8,MIT_MODE);//发送id为8
	
	VMC_init(vmc);//给杆长赋值

	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//初始化腿部pid

     //拨弹轮初始化
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
 
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan2,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
	  enable_motor_mode(&hcan2,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	  osDelay(1);
	}

}

void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
{
    vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
		
	chassis->myPithL=0.0f-ins->Pitch;
	chassis->myPithGyroL=0.0f-ins->Gyro[0];
	
	chassis->wheel_motor[1].vel = chassis->wheel_motor[1].speed_rpm * CHASSIS_MOTOR_RPM_TO_OMG_SEN; //角速度
	chassis->wheel_motor[1].speed = 0.0004998609952f * chassis->wheel_motor[1].speed_rpm;  //速度
	chassis->wheel_motor[1].wheel_T = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * chassis->wheel_motor[1].given_current;
	
	chassis->yaw_motor_angle = motor_ecd_to_angle_change(chassis->motor_chassis[4].ecd,0);
	
//	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();//获取拨弹轮
	
	static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (chassis->wheel_motor[3].speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;
	
}

uint8_t pre_left_flag=0;
extern uint8_t right_flag;
uint8_t left_flag;
extern float mg;
extern float jump_time_r;
uint8_t land_l_flag=0; //左腿落地标志
extern uint8_t land_r_flag; //右腿落地标志
void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,pid_type_def *leg)
{
	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*3.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001秒
	
	
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	

//	}
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
	}
	
//	     chassis->wheel_motor[1].wheel_T= ( LQR_K[0]*(vmcl->theta-0.0f)
//	                                    +LQR_K[1]*(vmcl->d_theta-0.0f)
//										+LQR_K[2]*(chassis->x_set-chassis->x)
//										+LQR_K[3]*(chassis->v_set-chassis->v)
////										+LQR_K[2]*(chassis->x_set-chassis->x_filter)
////										+LQR_K[3]*(0.4f*chassis->v_set-chassis->v_filter2)
//										+LQR_K[4]*(chassis->myPithL-(-0.01025f)) 
//										+LQR_K[5]*(chassis->myPithGyroL-0.01f));
//											
//											
//	
//	     vmcl->Tp=(     +LQR_K[6]*(vmcl->theta-0.0f)
//						+LQR_K[7]*0.7*(vmcl->d_theta-0.0f)
//						+LQR_K[8]*(chassis->x_set-chassis->x)
//						+LQR_K[9]*(chassis->v_set-chassis->v)
////						+LQR_K[8]*(chassis->x_set-chassis->x_filter)
////						+LQR_K[9]*(0.4f*chassis->v_set-chassis->v_filter2)
//						+LQR_K[10]*(chassis->myPithL-(-0.01025f))
//						+LQR_K[11]*(chassis->myPithGyroL-0.01f));	
   
    // chassis->wheel_motor[0].wheel_T =   0;
	chassis->wheel_motor[1].wheel_T=   ( LQR_K[0]*(vmcl->theta-0.0f)
	                                    +LQR_K[1]*(vmcl->d_theta-0.0f)
//										+LQR_K[2]*(chassis->x_set-chassis->x)
//										+LQR_K[3]*(chassis->v_set-chassis->v)
                                        // +LQR_K[2]*0.0f
										+LQR_K[2]*((chassis->x_set)-chassis->x_filter)
										+LQR_K[3]*((chassis->v_set)-chassis->v_filter2)
										+LQR_K[4]*(chassis->myPithL-(-0.0f)) 
										+LQR_K[5]*(chassis->myPithGyroL-0.0f));
											
											


	     vmcl->Tp=(      LQR_K[6]*(vmcl->theta-0.0f)
						+LQR_K[7]*(vmcl->d_theta-0.0f)
//						+LQR_K[8]*(chassis->x_set-chassis->x)
//						+LQR_K[9]*(chassis->v_set-chassis->v)
						// +LQR_K[8]*0.0f
						+LQR_K[8]*((chassis->x_set)-chassis->x_filter)
						+LQR_K[9]*(chassis->v_set-chassis->v_filter2)
						+LQR_K[10]*(chassis->myPithL-(-0.0f))
						+LQR_K[11]*(chassis->myPithGyroL-0.0f));		


	    vmcl->Tp=vmcl->Tp+chassis->leg_tp;//髋关节输出力矩=LQR得到的+防劈叉补偿
//  	vmcl->Tp=vmcl->Tp;
         //vmcl->Tp=0.0f;
         //chassis->wheel_motor[1].wheel_T = 0.0f;
	    chassis->wheel_motor[1].wheel_T = chassis->wheel_motor[1].wheel_T-chassis->turn_T;	//轮毂电机输出力矩

		mySaturate(&chassis->wheel_motor[1].wheel_T,-4.2f,4.2f);
		if(chassis->help_jump_flag ==1)
		{
		if(chassis->jump_flag_l==1)
		{
			vmcl->F0=jumpF0_L/arm_cos_f32(vmcl->theta)+PID_calc(&jump_pid_L,vmcl->L0,chassis->leg_set)+chassis->roll_f0;
		}
		else if(chassis->jump_flag_l==3)
		{
			vmcl->F0=jumpF0_L/arm_cos_f32(vmcl->theta)+jump_pid_i_L+PID_calc(&jump_pid_L,vmcl->L0,chassis->leg_set);
		}
		else if(chassis->jump_flag_l==0)
		{
			vmcl->F0=11.2f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set)+chassis->roll_f0;
		}			
		else
		{
			vmcl->F0=jumpF0_L/arm_cos_f32(vmcl->theta)+PID_calc(&jump_pid_L,vmcl->L0,chassis->leg_set);
		}

		}
		else{
			vmcl->F0=17.2f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set)+chassis->roll_f0;//前馈+pd	
		}
		
		
		//vmcl->F0=0.0f;
		// float Poly_Coefficient[12][4] = {-421.75465, 404.14066, -163.18492, 1.52481, -100.33598, 88.97437, -33.12351, 0.46394, -13.79006, 20.45913, -5.20346, -6.44626, 0.23096, 8.70198, -1.16290, -7.84332, -145.33854, 158.48464, -89.03226, 32.92067, -27.19789, 19.76695, -10.47246, 4.95830, 615.75677, -496.49431, 155.68024, -1.46787, 129.15010, -93.95712, 23.77051, 0.02808, 207.43929, -152.95834, 33.54335, 1.53506, 211.73999, -148.21559, 29.04610, 2.57854, 88.69753, -151.25099, 93.98502, 36.97904, -21.28958, 10.81717, 4.85406, 2.35990};
		//拟合矩阵
		//		vmcl->F0=17.55f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set)+chassis->roll_f0;


		//		vmcl->F0=11.2f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd

//起跳阶段
 	if(chassis->chassis_RC->rc.s[1] ==1)
 {
 		if(chassis->chassis_RC->rc.ch[4] >500){
 		chassis->help_jump_flag =1;
 		 }
 //压缩阶段
 		 if(chassis->jump_flag_l==0&& chassis->help_jump_flag ==1)
 		{
			
 		 chassis->leg_set = 0.13f;
		jumpF0_L=17.2f;
 		 if(vmcl->L0<0.15f)
 		 {
 		  jump_time_l++;
 		 }
 		 if(jump_time_l>=25&&jump_time_l>=25)
 		 {  
 			 jump_time_l=0;
 			 jump_time_r=0;
 			 chassis->jump_flag_l=1;
 			 chassis->jump_flag_r=1;//压缩完毕进入上升加速阶段
 		 }			 
 		}

//上升加速阶段	
 		else if(chassis->jump_flag_l==1&& chassis->help_jump_flag ==1)
 		{

 			chassis->leg_set = 0.32f;
			jumpF0_L=20.0f;
 			//  if(vmcl->L0>0.13f)
 			//  {
 			// 	jump_time_l++;
 			//  }
			jump_time_l++;
 			 if(jump_time_l>=30&&jump_time_r>=30)
 			 {  
 				 jump_time_l=0;
 				  jump_time_r=0;
 				 chassis->jump_flag_l=2;
 				 chassis->jump_flag_r=2;//上升完毕进入缩腿阶段
 			 }
 		}
 //缩腿阶段
 		else if(chassis->jump_flag_l==2&& chassis->help_jump_flag ==1)
 		{
 		 	chassis->leg_set = 0.13f;
			jumpF0_L=0.0f;
 			chassis->theta_set=0.0f;
 			chassis->x_filter=0.0f;
 			chassis->x_set=chassis->x_filter+0.3f;
 		//   if(vmcl->L0<0.22f)
 		//   {
 		// 	 jump_time_l++;
 		//   }
		jump_time_l++;
 		  if(jump_time_l>=40&&jump_time_r>=40)
 		  { 
 			 jump_time_l=0;
 			 jump_time_r=0;
 			//  chassis->leg_set=0.15f;
 			//  chassis->last_leg_set=0.15f;
 			//  chassis->jump_flag_l=0;
 			//  chassis->jump_flag_r=0;
 			//  chassis->help_jump_flag = 0;
			 chassis->jump_flag_l=3;
 			 chassis->jump_flag_r=3;
			
 		  }
 		}
		 //落地阶段
		else if(chassis->jump_flag_l==3&& chassis->help_jump_flag ==1)
		{
			 jumpF0_L=11.2f;	
			chassis->leg_set = 0.2f;
			// leg_l_pid_int+=chassis->leg_set - vmcl->L0;
			// mySaturate(&leg_l_pid_int,-2.0f,0.0f);
			// jump_pid_i_L=leg_l_pid_int*15.0f;

			// 测试代码
			// if(pre_left_flag==1&&left_flag==0)
			// {
			// 	land_l_flag=1;
			// }
			// if(land_r_flag==1&&land_l_flag==1)
			// {
			// 	jumpF0_L=17.2f; //着陆时加重力前馈
			// }
			jump_time_l++;
			if(jump_time_l>=85&&jump_time_r>=85)
			{
				jump_time_l=0;
				jump_time_r=0;
				chassis->last_leg_set=0.2f;
				chassis->jump_flag_l=0;		//缩腿完毕
				chassis->jump_flag_r=0;
				chassis->help_jump_flag = 0;
				land_r_flag=0;
				land_l_flag=0;
			}
		}
 	else
 	{
 		vmcl->F0=17.2f/arm_cos_f32(vmcl->theta)+PID_calc(leg,vmcl->L0,chassis->leg_set)+chassis->roll_f0;
 	}
 }	

// 	 vmcl->F0=PID_calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd
 	pre_left_flag=left_flag;
	 left_flag=ground_detectionL(vmcl,ins);//右腿离地检测

	 if(chassis->recover_flag==0)
	 {
//倒地自起不需要检测是否离地	 
//		if((right_flag==1&&left_flag==1&&vmcl->leg_flag==0&&chassis->jump_flag_l!=1&&chassis->jump_flag!=1&&chassis->jump_flag_l!=2&&chassis->jump_flag!=2)

//			||chassis->jump_flag_l==3)
		if(right_flag==1&&left_flag==1&&vmcl->leg_flag==0)
		{  
			 //当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				chassis->wheel_motor[1].wheel_T=0.0f;
				vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);

				chassis->x_filter=0.0f;
				chassis->x_set=chassis->x_filter;
				chassis->turn_set=chassis->total_yaw;
				vmcl->Tp=vmcl->Tp+chassis->leg_tp;	
		}
		else
		{
			//没有离地
			vmcl->leg_flag=0;//置为0
	
//不跳跃的时候需要roll轴补偿
			if(chassis->jump_flag_l==0)
			{
				
				vmcl->F0=vmcl->F0+chassis->roll_f0;//roll轴补偿取反然后加上去					
			}
		}
	 }

	 else if(chassis->recover_flag==1)
	 {
		 vmcl->Tp=0.0f;
		 vmcl->F0=0.0f;
	 }

	mySaturate(&vmcl->F0,-80.0f,100.0f);//限幅 

	VMC_calc_2(vmcl);//计算期望的关节输出力矩

//跳跃的时候需要更大扭矩
// 	// if(chassis->jump_flag_l==1||chassis->jump_flag_l==2||chassis->jump_flag_l==3)
// 	// {
// 	// 	mySaturate(&vmcl->torque_set[1],-18.0f,18.0f);	
// 	// 	mySaturate(&vmcl->torque_set[0],-18.0f,18.0f);	
// 	// }

//不跳跃的时候最大为额定扭矩
	// else
	// {
    mySaturate(&vmcl->torque_set[1],-18.0f,18.0f);	
	mySaturate(&vmcl->torque_set[0],-18.0f,18.0f);	
	// }		

}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
/**
  * @brief          计算ecd和offset_ecd之间的相对角度
  * @param[in]      ecd: 当前电机编码器值
  * @param[in]      offset_ecd: 云台偏移编码器值
  * @retval         相对角度，单位rad
  */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}
