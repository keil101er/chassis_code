#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
	RED_MISSILE     = 8,
	RED_RADAR       = 9,
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STNDARD_3  = 105,
    BLUE_AEARIAL    = 106,
    BLUE_SENTRY     = 107,
	BLUE_MISSILE    = 108,
	BLUE_RADAR      = 109,
} robot_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;

typedef __packed struct //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
	uint64_t SyncTimeStamp; 
} game_status_t;

typedef __packed struct //0x0002
{
    uint8_t winner;
} game_result_t;

typedef __packed struct //0x0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

typedef __packed struct //0x0101
{
     uint32_t event_data;
} event_data_t;

typedef __packed struct //0x0102
{
	uint8_t reserved; 
    uint8_t supply_robot_id; 
    uint8_t supply_projectile_step; 
    uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t; 


typedef __packed struct //0x0103
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x0104
{
    uint8_t level; 
    uint8_t offending_robot_id; 
    uint8_t count; 
} referee_warning_t;

typedef __packed struct //0x0201
{
	uint8_t robot_id; 
    uint8_t robot_level; 
    uint16_t current_HP;  
    uint16_t maximum_HP; 
    uint16_t shooter_barrel_cooling_value; 
    uint16_t shooter_barrel_heat_limit; 
    uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1; 
    uint8_t power_management_chassis_output : 1;  
    uint8_t power_management_shooter_output : 1; 
} robot_status_t; 

typedef __packed struct //0x0202
{
	uint16_t chassis_voltage; 
    uint16_t chassis_current; 
    float chassis_power; 
    uint16_t buffer_energy; 
    uint16_t shooter_17mm_1_barrel_heat; 
    uint16_t shooter_17mm_2_barrel_heat; 
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t; 

typedef __packed struct //0x0203
{
    float x;
    float y;
	float angle;
} robot_pos_t; 

typedef __packed struct //0x0204
{
	uint8_t recovery_buff;  
    uint8_t cooling_buff;  
    uint8_t defence_buff;  
    uint8_t vulnerability_buff; 
    uint16_t attack_buff; 
} buff_t; 

typedef __packed struct //0x0205
{ 
	uint8_t airforce_status; 
	uint8_t time_remain; 
} air_support_data_t; 

typedef __packed struct //0x0206
{
    uint8_t armor_id : 4; 
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;  
    uint8_t shooter_number; 
    uint8_t launching_frequency;  
    float initial_speed; 
} shoot_data_t;

typedef __packed struct //0x0208
{
    uint16_t projectile_allowance_17mm; 
    uint16_t projectile_allowance_42mm;  
    uint16_t remaining_gold_coin;
} projectile_allowance_t;

typedef __packed struct //0x0301
{
    uint16_t data_cmd_id; 
    uint16_t sender_id; 
    uint16_t receiver_id; 
    uint8_t user_data[];
} robot_interaction_data_t; 

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;






extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);
extern void get_game_state(uint8_t *game_progress, uint16_t *stage_remain_time);
extern void get_ext_robot_command_date(float *coordinate_x, float  *coordinate_y,uint8_t *commd_keyboard);
extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);
extern void get_chassis_max_power(uint16_t *max_power_limit);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);
#endif
