/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "VMC_calc.h"
#include "test_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "stdio.h"
#include "string.h"
#include "chassisR_task.h"

uint8_t Txcplt_flag=1;
extern UART_HandleTypeDef huart1;
extern chassis_t chassis_move_balance;
extern uint8_t Rc_flag;
char debug_info[50];
static void buzzer_warn_error(uint8_t num);

const error_t *error_list_test_local;
extern vmc_leg_t right;
extern vmc_leg_t left;
extern uint8_t jump_status;

float F_r=0,F_l=0;
/**
  * @brief          test task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          test任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
 //蓝牙调试信息输出任务 Bluetooth debugging information output task
 void Buletooth_debug_task(void)
{
    
       //sprintf(debug_info,"%.1f,%.2f,%.2f,%.2f,%d\n",chassis_move_balance.v_set,chassis_move_balance.v_filter2,chassis_move_balance.x_set,chassis_move_balance.x_filter,Rc_flag);
    //    sprintf(debug_info,"%.2f,%.2f,%d\n",right.F0,right.L0,jump_status);
    F_r=right.torque_set[1]-right.torque_set[0];
    F_l=left.torque_set[1]-left.torque_set[0];
    sprintf(debug_info,"%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",left.L0,right.L0,F_r,F_l,right.F0,jump_status);
       //发送完成标志位
		if(Txcplt_flag==1)
		{
        HAL_UART_Transmit_DMA(&huart1,(uint8_t *)debug_info,strlen(debug_info));
		Txcplt_flag=0;
		}			
}
void test_task(void const * argument)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
    error_list_test_local = get_error_list_point();

    while(1)
    {
        error = 0;
        //find error
        //发现错误
        for(error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if(error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        //no error, stop buzzer
        //没有错误, 停止蜂鸣器
        if(error == 0 && last_error != 0)
        {
            buzzer_off();
        }
        //have error
        //有错误
        if(error)
        {
            buzzer_warn_error(error_num+1);
        }

        last_error = error;
        //Buletooth_debug_task();
        osDelay(10);
    }
}


/**
  * @brief          make the buzzer sound
  * @param[in]      num: the number of beeps 
  * @retval         none
  */
/**
  * @brief          使得蜂鸣器响
  * @param[in]      num:响声次数
  * @retval         none
  */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if(show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if(show_num == 0)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if(tick < 50)
        {
            buzzer_off();
        }
        else if(tick < 100)
        {
            buzzer_on(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //发送完成置标志位
    if (huart==&huart1){ // 检查是否是我们关心的UART
        Txcplt_flag=1;
    }
}

