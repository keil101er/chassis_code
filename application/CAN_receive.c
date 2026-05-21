/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      CAN receive callbacks and transmit helpers.
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"
#include "main.h"
#include "chassisR_task.h"
#include "dm_8009_drv.h"
#include "detect_task.h"
#include "CANdata_analysis.h"
#include "shoot.h"
#include "string.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static void supercap_rx_parse(const uint8_t rx_data[8]);
static uint16_t supercap_clamp_power_limit(uint16_t power_limit);
static uint16_t supercap_clamp_energy_buffer(uint16_t energy_buffer);

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

motor_measure_t motor_chassis[7];

extern chassis_t chassis_move_balance;
extern shoot_control_t shoot_control;
extern chassis_t chassis_move_balance;

CAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];
// uint16_t auto_cnt = 0;

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

supercap_tx_msg_t supercap_tx_msg;
supercap_rx_msg_t supercap_rx_msg;
static uint32_t supercap_last_rx_tick = 0U;

static uint16_t supercap_clamp_power_limit(uint16_t power_limit)
{
    if (power_limit < SUPERCAP_POWER_LIMIT_MIN)
    {
        return SUPERCAP_POWER_LIMIT_MIN;
    }
    if (power_limit > SUPERCAP_POWER_LIMIT_MAX)
    {
        return SUPERCAP_POWER_LIMIT_MAX;
    }
    return power_limit;
}

static uint16_t supercap_clamp_energy_buffer(uint16_t energy_buffer)
{
    if (energy_buffer > SUPERCAP_ENERGY_BUFFER_MAX)
    {
        return SUPERCAP_ENERGY_BUFFER_MAX;
    }
    return energy_buffer;
}

static void supercap_rx_parse(const uint8_t rx_data[8])
{
    // 港科超电回包:
    // byte0 errorCode
    // byte1-4 chassisPower(float, little-endian)
    // byte5-6 chassisPowerLimit(uint16_t, little-endian)
    // byte7 capEnergyPercent
    supercap_rx_msg.error_code = rx_data[0];
    memcpy(&supercap_rx_msg.chassis_power, &rx_data[1], 4);
    supercap_rx_msg.chassis_power_limit = (uint16_t)(rx_data[5] | (rx_data[6] << 8));
    supercap_rx_msg.cap_energy_percent_raw = rx_data[7];
    supercap_last_rx_tick = HAL_GetTick();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1)
    {
        switch (rx_header.StdId)
        {
        case CAN_SUPERCAP_RX_ID:
            supercap_rx_parse(rx_data);
            break;

        case 0x15:
            C_fbdata2(&C_data, rx_data, 8);
            // if (C_data.MODE == 1)
            // {
            //     auto_cnt++;
            // }
            break;

        case 3:
            dm4310_fbdata1(&chassis_move_balance.joint_motor[0], rx_data, 8);
            break;

        case 4:
            dm4310_fbdata1(&chassis_move_balance.joint_motor[1], rx_data, 8);
            break;

        case 9:
            dm4310_fbdata1(&chassis_move_balance.joint_motor[4], rx_data, 8);

            if (chassis_move_balance.joint_motor[4].para.pos >= 6.25f &&
                chassis_move_balance.joint_motor[4].para.pos <= 12.5f)
            {
                chassis_move_balance.joint_motor[4].para.pos -= 6.25f;
            }
            if (chassis_move_balance.joint_motor[4].para.pos <= 0.0f &&
                chassis_move_balance.joint_motor[4].para.pos > -6.25f)
            {
                chassis_move_balance.joint_motor[4].para.pos += 6.25f;
            }
            if (chassis_move_balance.joint_motor[4].para.pos <= -6.25f &&
                chassis_move_balance.joint_motor[4].para.pos >= -12.5f)
            {
                chassis_move_balance.joint_motor[4].para.pos += 12.5f;
            }

            chassis_move_balance.motor_chassis[4].last_ecd = chassis_move_balance.motor_chassis[4].ecd;
            chassis_move_balance.motor_chassis[4].ecd =
                chassis_move_balance.joint_motor[4].para.pos * 8192 / 6.25f;
            break;

        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            static uint8_t i = 0;
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&chassis_move_balance.wheel_motor[i], rx_data);
            break;
        }

        default:
            break;
        }
    }

    if (hcan == &hcan2)
    {
        switch (rx_header.StdId)
        {
        case 3:
            dm4310_fbdata1(&chassis_move_balance.joint_motor[2], rx_data, 8);
            break;

        case 4:
            dm4310_fbdata1(&chassis_move_balance.joint_motor[3], rx_data, 8);
            break;

        case Wheel_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t mm = 0;
            mm = rx_header.StdId - 0x204;
            get_motor_measure(&chassis_move_balance.wheel_motor[mm], rx_data);
            break;
        }

        default:
            break;
        }
    }
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;

    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (uint8_t)(yaw >> 8);
    gimbal_can_send_data[1] = (uint8_t)yaw;
    gimbal_can_send_data[2] = (uint8_t)(pitch >> 8);
    gimbal_can_send_data[3] = (uint8_t)pitch;
    gimbal_can_send_data[4] = (uint8_t)(shoot >> 8);
    gimbal_can_send_data[5] = (uint8_t)shoot;
    gimbal_can_send_data[6] = (uint8_t)(rev >> 8);
    gimbal_can_send_data[7] = (uint8_t)rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;

    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0U;
    chassis_can_send_data[1] = 0U;
    chassis_can_send_data[2] = 0U;
    chassis_can_send_data[3] = 0U;
    chassis_can_send_data[4] = 0U;
    chassis_can_send_data[5] = 0U;
    chassis_can_send_data[6] = 0U;
    chassis_can_send_data[7] = 0U;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1)
{
    uint32_t send_mail_box;

    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (uint8_t)(motor1 >> 8);
    chassis_can_send_data[1] = (uint8_t)motor1;
    chassis_can_send_data[2] = 0U;
    chassis_can_send_data[3] = 0U;
    chassis_can_send_data[4] = 0U;
    chassis_can_send_data[5] = 0U;
    chassis_can_send_data[6] = 0U;
    chassis_can_send_data[7] = 0U;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_supercap(uint8_t enable_dcdc, uint8_t system_restart, uint16_t power_limit, uint16_t energy_buffer)
{
    uint32_t send_mail_box;

    power_limit = supercap_clamp_power_limit(power_limit);
    energy_buffer = supercap_clamp_energy_buffer(energy_buffer);

    supercap_tx_msg.enable_dcdc = (enable_dcdc != 0U) ? 1U : 0U;
    supercap_tx_msg.system_restart = (system_restart != 0U) ? 1U : 0U;
    supercap_tx_msg.referee_power_limit = power_limit;
    supercap_tx_msg.referee_energy_buffer = energy_buffer;

    chassis_tx_message.StdId = CAN_SUPERCAP_TX_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (uint8_t)((supercap_tx_msg.enable_dcdc & 0x01U) |
                                         ((supercap_tx_msg.system_restart & 0x01U) << 1));
    chassis_can_send_data[1] = (uint8_t)(supercap_tx_msg.referee_power_limit & 0xFFU);
    chassis_can_send_data[2] = (uint8_t)((supercap_tx_msg.referee_power_limit >> 8) & 0xFFU);
    chassis_can_send_data[3] = (uint8_t)(supercap_tx_msg.referee_energy_buffer & 0xFFU);
    chassis_can_send_data[4] = (uint8_t)((supercap_tx_msg.referee_energy_buffer >> 8) & 0xFFU);
    chassis_can_send_data[5] = 0U;
    chassis_can_send_data[6] = 0U;
    chassis_can_send_data[7] = 0U;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);

    supercap_tx_msg.system_restart = 0U;
}

const supercap_tx_msg_t *get_supercap_tx_data_point(void)
{
    return &supercap_tx_msg;
}

const supercap_rx_msg_t *get_supercap_rx_data_point(void)
{
    return &supercap_rx_msg;
}

uint8_t supercap_is_online(void)
{
    if (supercap_last_rx_tick == 0U)
    {
        return 0U;
    }
    return ((HAL_GetTick() - supercap_last_rx_tick) <= SUPERCAP_OFFLINE_TIMEOUT_MS) ? 1U : 0U;
}

uint32_t get_supercap_last_rx_tick(void)
{
    return supercap_last_rx_tick;
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[6];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &chassis_move_balance.wheel_motor[3];
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
