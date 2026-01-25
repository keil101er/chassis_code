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
#include "shoot.h"
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
extern uint8_t land_flag;
extern float aver_fnr;
extern float aver_fn;
float F_r=0,F_l=0;
extern float theat_set;
extern int16_t shoot_can_set_current;
extern shoot_control_t shoot_control;  
extern float relative_angle;


// STP-23激光测距传感器接收相关变量 / STP-23 Laser Sensor Receive Variables
uint8_t STP23_Receive_buf[1];                    // USART1接收中断数据缓冲区 / USART1 Receive Buffer
LiDARFrameTypeDef STP23_Pack_Data;               // 激光雷达数据帧 / LiDAR Frame Data
uint16_t stp23_receive_cnt = 0;                  // 成功接收数据帧计数 / Successfully Received Frame Count
uint16_t stp23_distance = 0;                     // 距离(mm) / Distance (mm)
uint16_t stp23_temperature = 0;                  // 温度值 / Temperature
uint16_t stp23_start_angle = 0;                  // 起始角度(0.01度) / Start Angle
uint16_t stp23_end_angle = 0;                    // 结束角度(0.01度) / End Angle
uint16_t stp23_timestamp = 0;                    // 时间戳(ms) / Timestamp



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
    sprintf(debug_info,"%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",shoot_can_set_current,shoot_control.speed,shoot_control.speed_set,shoot_control.angle,shoot_control.set_angle,relative_angle);
    //sprintf(debug_info,"%.2f,%.2f\n",left.theta,right.theta);
    //  sprintf(debug_info,"%.2f,%.2f,%.2f,%.2f\n",chassis_move_balance.v_filter2,chassis_move_balance.x_set,chassis_move_balance.x_filter,theat_set);
    //sprintf(debug_info,"%.2f,%.2f,%d,%d\n",right.F0,right.L0,jump_status,chassis_move_balance.joint_motor[0].para.state);
    // F_r=right.torque_set[1]-right.torque_set[0];
    // F_l=left.torque_set[1]-left.torque_set[0];
     //sprintf(debug_info,"%.2f,%.2f,%.2f,%.2f,%d\n",right.torque_set[0],right.torque_set[1],right.L0,right.F0,jump_status);
    // sprintf(debug_info,"%.2f,%.2f,%.2f,%.2f,%.2f\n",right.phi1,right.phi2,right.phi3,right.phi4,right.L0);
    //sprintf(debug_info,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",left.L0,right.L0,F_r,F_l,aver_fn,aver_fnr,jump_status);
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
//    HAL_UART_Receive_IT(&huart1,STP23_Receive_buf,sizeof(STP23_Receive_buf));
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
        Buletooth_debug_task();
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

// CRC8校验表 / CRC8 Checksum Table
static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

/**
  * @brief          CRC8校验计算函数 / CRC8 Checksum Calculation
  * @param[in]      p: 数据指针 / Data Pointer
  * @param[in]      len: 数据长度 / Data Length
  * @retval         CRC8校验值 / CRC8 Value
  */
uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

/**
  * @brief          USART接收中断回调函数 / USART Receive Interrupt Callback
  * @param[in]      huart: UART句柄 / UART Handle
  * @retval         none
  */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     static uint8_t state = 0;       // 状态位 / State
//     static uint8_t crc = 0;         // CRC校验值 / CRC Value
//     static uint8_t cnt = 0;         // 统计一帧12个点的计数 / Count for 12 points in one frame
//     uint8_t temp_data;

//     // USART1 - STP-23激光测距传感器数据接收 / STP-23 Laser Sensor Data Reception
//     if(huart->Instance == USART1)
//     {
//         temp_data = STP23_Receive_buf[0];

//         // 解析状态机 / State Machine for Parsing
//         if(state > 5)  // 数据点解析(state 6-41) / Parse measurement points
//         {
//             if(state < 42)  // 12个点,每个点3字节 / 12 points, 3 bytes each
//             {
//                 if(state % 3 == 0)  // 距离值低字节 / Distance LSB
//                 {
//                     STP23_Pack_Data.point[cnt].distance = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                 }
//                 else if(state % 3 == 1)  // 距离值高字节 / Distance MSB
//                 {
//                     STP23_Pack_Data.point[cnt].distance = ((uint16_t)temp_data << 8) + STP23_Pack_Data.point[cnt].distance;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                 }
//                 else  // 信号强度 / Intensity
//                 {
//                     STP23_Pack_Data.point[cnt].intensity = temp_data;
//                     cnt++;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                 }
//             }
//             else  // 结束角度、时间戳、CRC校验 / End Angle, Timestamp, CRC
//             {
//                 switch(state)
//                 {
//                     case 42:  // 结束角度低字节 / End Angle LSB
//                         STP23_Pack_Data.end_angle = (uint16_t)temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                         break;

//                     case 43:  // 结束角度高字节 / End Angle MSB
//                         STP23_Pack_Data.end_angle = ((uint16_t)temp_data << 8) + STP23_Pack_Data.end_angle;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                         break;

//                     case 44:  // 时间戳低字节 / Timestamp LSB
//                         STP23_Pack_Data.timestamp = (uint16_t)temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                         break;

//                     case 45:  // 时间戳高字节 / Timestamp MSB
//                         STP23_Pack_Data.timestamp = ((uint16_t)temp_data << 8) + STP23_Pack_Data.timestamp;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                         break;

//                     case 46:  // CRC8校验 / CRC8 Checksum
//                         STP23_Pack_Data.crc8 = temp_data;
//                         if(STP23_Pack_Data.crc8 == crc)  // 校验成功 / CRC OK
//                         {
//                             STP23_Data_Process();  // 数据处理 / Data Processing
//                             stp23_receive_cnt++;   // 成功接收计数 / Success Count
//                         }
//                         // 复位状态 / Reset State
//                         crc = 0;
//                         state = 0;
//                         cnt = 0;
//                         break;

//                     default: break;
//                 }
//             }
//         }
//         else  // 协议头解析(state 0-5) / Parse header
//         {
//             switch(state)
//             {
//                 case 0:  // 起始符 0x54 / Header
//                     if(temp_data == HEADER)
//                     {
//                         STP23_Pack_Data.header = temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     }
//                     else
//                     {
//                         state = 0;
//                         crc = 0;
//                     }
//                     break;

//                 case 1:  // 版本长度 0x2C / VerLen
//                     if(temp_data == VERLEN)
//                     {
//                         STP23_Pack_Data.ver_len = temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     }
//                     else
//                     {
//                         state = 0;
//                         crc = 0;
//                     }
//                     break;

//                 case 2:  // 温度值低字节 / Temperature LSB
//                     STP23_Pack_Data.temperature = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     break;

//                 case 3:  // 温度值高字节 / Temperature MSB
//                     STP23_Pack_Data.temperature = ((uint16_t)temp_data << 8) + STP23_Pack_Data.temperature;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     break;

//                 case 4:  // 起始角度低字节 / Start Angle LSB
//                     STP23_Pack_Data.start_angle = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     break;

//                 case 5:  // 起始角度高字节 / Start Angle MSB
//                     STP23_Pack_Data.start_angle = ((uint16_t)temp_data << 8) + STP23_Pack_Data.start_angle;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xff];
//                     break;

//                 default: break;
//             }
//         }

//         // 重新开启USART1接收中断 / Restart USART1 receive interrupt
//         HAL_UART_Receive_IT(&huart1, STP23_Receive_buf, sizeof(STP23_Receive_buf));
//     }
// }

/**
  * @brief          STP-23数据处理函数(对12个点求平均) / STP-23 Data Processing (Average of 12 points)
  * @param[in]      none
  * @retval         none
  */
void STP23_Data_Process(void)
{
    uint8_t i;
    uint16_t count = 0;
    uint32_t sum = 0;

    // 对12个点取平均 / Average 12 points
    for(i = 0; i < POINT_PER_PACK; i++)
    {
        if(STP23_Pack_Data.point[i].distance != 0)  // 去除距离为0的点 / Remove points with distance 0
        {
            count++;
            sum += STP23_Pack_Data.point[i].distance;
        }
    }

    // 计算平均距离 / Calculate average distance
    if(count != 0)
    {
        stp23_distance = sum / count;
    }

    // 保存其他数据 / Save other data
    stp23_temperature = STP23_Pack_Data.temperature;
    stp23_start_angle = STP23_Pack_Data.start_angle;
    stp23_end_angle = STP23_Pack_Data.end_angle;
    stp23_timestamp = STP23_Pack_Data.timestamp;
}