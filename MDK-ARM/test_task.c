#include "test_task.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "VMC_calc.h"
#include "bsp_buzzer.h"
#include "chassisR_task.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "main.h"
#include "referee.h"
#include "shoot.h"
#include "ui_g.h"
#include "ui_interface.h"
#include "remote_control.h"
#include "CAN_receive.h"

uint8_t Txcplt_flag = 1;

extern UART_HandleTypeDef huart1;
extern chassis_t chassis_move_balance;
extern shoot_control_t shoot_control;
extern vmc_leg_t right;
extern robot_status_t robot_state;

char debug_info[50];
static void buzzer_warn_error(uint8_t num);
static void ui_update_overlay(void);
static float ui_clamp_f32(float value, float min_value, float max_value);
static uint32_t ui_wrap_angle_u32(int32_t angle_deg);
const error_t *error_list_test_local;
static void ui_reinit_overlay(void);

uint8_t STP23_Receive_buf[1];
LiDARFrameTypeDef STP23_Pack_Data;
uint16_t stp23_receive_cnt = 0;
uint16_t stp23_distance = 0;
uint16_t stp23_temperature = 0;
uint16_t stp23_start_angle = 0;
uint16_t stp23_end_angle = 0;
uint16_t stp23_timestamp = 0;
extern float total_power;
extern supercap_rx_msg_t supercap_rx_msg;
void Buletooth_debug_task(void)
{
    snprintf(debug_info, sizeof(debug_info), "%d,%.2f,%.2f\n",
             shoot_control.fric_enabled, chassis_move_balance.leg_set, right.L0);

    if (Txcplt_flag == 1U)
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)debug_info, strlen(debug_info));
        Txcplt_flag = 0;
    }
}

static void ui_reinit_overlay(void)
{
    ui_delete_layer(2, 0);
    osDelay(80);
    ui_init_g();
}

void test_task(void const *argument)
{
    static uint8_t error;
    static uint8_t last_error;
    static uint8_t error_num;
    static uint8_t ui_initialized = 0;
    static uint8_t ui_init_in_progress = 0;
    static uint8_t ui_fragile_refresh_tick = 0;
    static uint16_t last_key_v = 0;

    (void)argument;
    error_list_test_local = get_error_list_point();
    HAL_UART_Receive_IT(&huart1, STP23_Receive_buf, sizeof(STP23_Receive_buf));

    while (1)
    {
        error = 0;
        for (error_num = 0; error_num < REFEREE_TOE; error_num++)
        {
            if (error_list_test_local[error_num].error_exist)
            {
                error = 1;
                break;
            }
        }

        if ((error == 0U) && (last_error != 0U))
        {
            buzzer_off();
        }

        if (error != 0U)
        {
            buzzer_warn_error(error_num + 1U);
        }

        last_error = error;

        if (get_robot_id() != 0U)              
        {
            uint16_t key_v = chassis_move_balance.chassis_RC->key.v;
            uint8_t g_key_pressed = (uint8_t)((key_v & KEY_PRESSED_OFFSET_G) != 0U);
            uint8_t g_key_pressed_last = (uint8_t)((last_key_v & KEY_PRESSED_OFFSET_G) != 0U);

            ui_self_id = get_robot_id();

            if ((ui_initialized == 0U) || (g_key_pressed != 0U && g_key_pressed_last == 0U))
            {
                ui_reinit_overlay();
                ui_initialized = 1;
                ui_init_in_progress = 1U;
                ui_fragile_refresh_tick = 0U;
            }
            else if (ui_init_in_progress != 0U)
            {
                ui_update_overlay();
                if (ui_init_step_g() != 0U)
                {
                    ui_init_in_progress = 0U;
                    ui_fragile_refresh_tick = 0U;
                }
            }
            else
            {
                ui_update_overlay();
                ui_update_g();

                ui_fragile_refresh_tick++;
                if (ui_fragile_refresh_tick >= 20U)
                {
                    ui_refresh_fragile_create_g();
                    ui_fragile_refresh_tick = 0U;
                }
            }

            last_key_v = key_v;
        }

        osDelay(40);
    }
}

static void ui_update_overlay(void)
{
    static float display_leg_length = 0.0f;
    static float display_power = 0.0f;
    static uint8_t display_value_initialized = 0U;
    const float body_center_x = 1571.5f;
    const float body_center_y = 441.0f;
    const float body_half_length = 74.5f;
    const float wheel_radius = 24.0f;
    const float leg_visual_length_min = wheel_radius;
    const float leg_visual_length_max = 100.0f;
    const float leg_length_min_m = 0.12f;
    const float leg_length_max_m = 0.32f;
    const float chassis_arc_center_x = 960.0f;
    const float chassis_arc_center_y = 540.0f;
    const float supercap_bar_left = 710.0f;
    const float supercap_bar_width = 499.0f;
    const float rad_to_deg = 57.2957795f;
    const int32_t chassis_arc_half_span_deg = 30;

    float chassis_power = 0.0f;
    float buffer_energy = 0.0f;
    float leg_length = chassis_move_balance.leg_set;
    float body_angle = chassis_move_balance.myPithR;
    float leg_angle = right.theta;
    float relative_angle = -chassis_move_balance.relative_angle;
    float leg_length_ratio;
    float leg_visual_length;
    float body_dx;
    float body_dy;
    float wheel_center_x;
    float wheel_center_y;
    float supercap_ratio;
    uint32_t supercap_bar_color = 2U;
    float relative_angle_deg;
    uint8_t show_fire;
    uint8_t show_gyro;
    uint8_t show_enable;

    get_chassis_power_and_buffer(&chassis_power, &buffer_energy);
    chassis_power = total_power;
    if (right.L0 > 0.05f)
    {
        leg_length = right.L0;
    }

    display_leg_length = leg_length;
    display_power = chassis_power;
    // display_leg_length=supercap_rx_msg.cap_energy_percent_raw;
    // display_power=supercap_rx_msg.chassis_power_limit;

    display_leg_length = ui_clamp_f32(display_leg_length, leg_length_min_m, leg_length_max_m);
    // display_leg_length = ui_clamp_f32(display_leg_length, 0.0f, 9999.0f);
    display_power = ui_clamp_f32(display_power, 0.0f, 9999.0f);

    ui_g_Ungroup_leg_value->number = (int32_t)lroundf(display_leg_length * 1000.0f);
    ui_g_Ungroup_power_value->number = (int32_t)lroundf(display_power * 1000.0f);

    body_angle = ui_clamp_f32(body_angle, -0.6f, 0.6f);
    leg_angle = ui_clamp_f32(leg_angle, -0.8f, 0.8f);
    relative_angle = ui_clamp_f32(relative_angle, -3.1415926f, 3.1415926f);
    leg_length_ratio = (display_leg_length - leg_length_min_m) / (leg_length_max_m - leg_length_min_m);
    leg_length_ratio = ui_clamp_f32(leg_length_ratio, 0.0f, 1.0f);
    leg_visual_length = leg_visual_length_min +
                        leg_length_ratio * (leg_visual_length_max - leg_visual_length_min);

    body_dx = cosf(body_angle) * body_half_length;
    body_dy = sinf(body_angle) * body_half_length;

    ui_g_Ungroup_car_body_line->start_x = (uint32_t)lroundf(body_center_x - body_dx);
    ui_g_Ungroup_car_body_line->start_y = (uint32_t)lroundf(body_center_y - body_dy);
    ui_g_Ungroup_car_body_line->details_d = (uint32_t)lroundf(body_center_x + body_dx);
    ui_g_Ungroup_car_body_line->details_e = (uint32_t)lroundf(body_center_y + body_dy);

    ui_g_Ungroup_leg_line->start_x = (uint32_t)lroundf(body_center_x);
    ui_g_Ungroup_leg_line->start_y = (uint32_t)lroundf(body_center_y);

    wheel_center_x = body_center_x - sinf(leg_angle) * leg_visual_length;
    wheel_center_y = body_center_y - cosf(leg_angle) * leg_visual_length;

    ui_g_Ungroup_leg_line->details_d = (uint32_t)lroundf(wheel_center_x);
    ui_g_Ungroup_leg_line->details_e = (uint32_t)lroundf(wheel_center_y);

    ui_g_Ungroup_wheel_round->start_x = (uint32_t)lroundf(wheel_center_x);
    ui_g_Ungroup_wheel_round->start_y = (uint32_t)lroundf(wheel_center_y);
    ui_g_Ungroup_wheel_round->details_c = (uint32_t)lroundf(wheel_radius);

    relative_angle_deg = -relative_angle * rad_to_deg;
    ui_g_Ungroup_chassis_dirct->start_x = (uint32_t)lroundf(chassis_arc_center_x);
    ui_g_Ungroup_chassis_dirct->start_y = (uint32_t)lroundf(chassis_arc_center_y);
    ui_g_Ungroup_chassis_dirct->details_a =
        ui_wrap_angle_u32((int32_t)lroundf(relative_angle_deg) - chassis_arc_half_span_deg);
    ui_g_Ungroup_chassis_dirct->details_b =
        ui_wrap_angle_u32((int32_t)lroundf(relative_angle_deg) + chassis_arc_half_span_deg);
    ui_g_Ungroup_chassis_dirct->details_d = 90U;
    ui_g_Ungroup_chassis_dirct->details_e = 90U;

    supercap_ratio = ui_clamp_f32((float)supercap_rx_msg.cap_energy_percent_raw / 245.0f, 0.0f, 1.0f);
    if (supercap_rx_msg.cap_energy_percent_raw < 50U)
    {
        supercap_bar_color = 4U;
    }
    else if (supercap_rx_msg.cap_energy_percent_raw < 150U)
    {
        supercap_bar_color = 1U;
    }
    else
    {
        supercap_bar_color = 2U;
    }

    ui_g_Ungroup_supercap_capacity_bar->details_d =
        (uint32_t)lroundf(supercap_bar_left + (supercap_ratio * supercap_bar_width));
    ui_g_Ungroup_supercap_capacity_bar->details_e = 80U;
    ui_g_Ungroup_supercap_capacity_bar->color = supercap_bar_color;

    show_fire = (uint8_t)((shoot_control.fire_mode == FIREING) || (shoot_control.fric_enabled != 0U));
    show_gyro = (uint8_t)(chassis_move_balance.w_flag != 0U);
    show_enable = (uint8_t)((chassis_move_balance.start_flag != 0U) &&
                            (robot_state.power_management_chassis_output != 0U));

    ui_g_Ungroup_Fire_round->color = show_fire ? 2U : 4U;
    ui_g_Ungroup_W_round->color = show_gyro ? 2U : 4U;
    ui_g_Ungroup_enable_round->color = show_enable ? 2U : 4U;
}

static float ui_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint32_t ui_wrap_angle_u32(int32_t angle_deg)
{
    while (angle_deg < 0)
    {
        angle_deg += 360;
    }

    while (angle_deg >= 360)
    {
        angle_deg -= 360;
    }

    return (uint32_t)angle_deg;
}

static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    static uint8_t tick = 0;

    if ((show_num == 0U) && (stop_num == 0U))
    {
        show_num = num;
        stop_num = 100;
    }
    else if (show_num == 0U)
    {
        stop_num--;
        buzzer_off();
    }
    else
    {
        tick++;
        if (tick < 50U)
        {
            buzzer_off();
        }
        else if (tick < 100U)
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

uint8_t CalCRC8(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        crc = CrcTable[(crc ^ *p++) & 0xff];
    }

    return crc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t state = 0;
    static uint8_t crc = 0;
    static uint8_t cnt = 0;
    uint8_t temp_data;

    if (huart->Instance == USART1)
    {
        temp_data = STP23_Receive_buf[0];

        if (state > 5U)
        {
            if (state < 42U)
            {
                if ((state % 3U) == 0U)
                {
                    STP23_Pack_Data.point[cnt].distance = (uint16_t)temp_data;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                }
                else if ((state % 3U) == 1U)
                {
                    STP23_Pack_Data.point[cnt].distance =
                        ((uint16_t)temp_data << 8) + STP23_Pack_Data.point[cnt].distance;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                }
                else
                {
                    STP23_Pack_Data.point[cnt].intensity = temp_data;
                    cnt++;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                }
            }
            else
            {
                switch (state)
                {
                case 42:
                    STP23_Pack_Data.end_angle = (uint16_t)temp_data;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                    break;
                case 43:
                    STP23_Pack_Data.end_angle =
                        ((uint16_t)temp_data << 8) + STP23_Pack_Data.end_angle;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                    break;
                case 44:
                    STP23_Pack_Data.timestamp = (uint16_t)temp_data;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                    break;
                case 45:
                    STP23_Pack_Data.timestamp =
                        ((uint16_t)temp_data << 8) + STP23_Pack_Data.timestamp;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                    break;
                case 46:
                    STP23_Pack_Data.crc8 = temp_data;
                    if (STP23_Pack_Data.crc8 == crc)
                    {
                        STP23_Data_Process();
                        stp23_receive_cnt++;
                    }
                    crc = 0;
                    state = 0;
                    cnt = 0;
                    break;
                default:
                    break;
                }
            }
        }
        else
        {
            switch (state)
            {
            case 0:
                if (temp_data == HEADER)
                {
                    STP23_Pack_Data.header = temp_data;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                }
                else
                {
                    state = 0;
                    crc = 0;
                }
                break;
            case 1:
                if (temp_data == VERLEN)
                {
                    STP23_Pack_Data.ver_len = temp_data;
                    state++;
                    crc = CrcTable[(crc ^ temp_data) & 0xff];
                }
                else
                {
                    state = 0;
                    crc = 0;
                }
                break;
            case 2:
                STP23_Pack_Data.temperature = (uint16_t)temp_data;
                state++;
                crc = CrcTable[(crc ^ temp_data) & 0xff];
                break;
            case 3:
                STP23_Pack_Data.temperature =
                    ((uint16_t)temp_data << 8) + STP23_Pack_Data.temperature;
                state++;
                crc = CrcTable[(crc ^ temp_data) & 0xff];
                break;
            case 4:
                STP23_Pack_Data.start_angle = (uint16_t)temp_data;
                state++;
                crc = CrcTable[(crc ^ temp_data) & 0xff];
                break;
            case 5:
                STP23_Pack_Data.start_angle =
                    ((uint16_t)temp_data << 8) + STP23_Pack_Data.start_angle;
                state++;
                crc = CrcTable[(crc ^ temp_data) & 0xff];
                break;
            default:
                break;
            }
        }

        HAL_UART_Receive_IT(&huart1, STP23_Receive_buf, sizeof(STP23_Receive_buf));
    }
}

void STP23_Data_Process(void)
{
    uint8_t count = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < POINT_PER_PACK; i++)
    {
        if (STP23_Pack_Data.point[i].distance != 0U)
        {
            count++;
            sum += STP23_Pack_Data.point[i].distance;
        }
    }

    if (count != 0U)
    {
        stp23_distance = (uint16_t)(sum / count);
    }

    stp23_temperature = STP23_Pack_Data.temperature;
    stp23_start_angle = STP23_Pack_Data.start_angle;
    stp23_end_angle = STP23_Pack_Data.end_angle;
    stp23_timestamp = STP23_Pack_Data.timestamp;
}
