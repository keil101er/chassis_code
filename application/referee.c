#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


frame_header_struct_t referee_receive_header;//
frame_header_struct_t referee_send_header;

game_status_t game_state;//
game_result_t game_result;//
game_robot_HP_t robot_HP_t;//血量

event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
referee_warning_t referee_warning_t1;


robot_status_t robot_state;
power_heat_data_t power_heat_data_t1;
robot_pos_t game_robot_pos_t;
buff_t buff_musk_t;
air_support_data_t robot_energy_t;
hurt_data_t robot_hurt_t;
shoot_data_t shoot_data_t1;
projectile_allowance_t bullet_remaining_t;
robot_interaction_data_t student_interactive_data_t;
ext_robot_command_t ext_robot_command;


void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
   
    memset(&game_state, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&robot_HP_t, 0, sizeof(game_robot_HP_t));


    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t1, 0, sizeof(referee_warning_t));


    memset(&robot_state, 0, sizeof(robot_status_t));
    memset(&power_heat_data_t1, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data_t1, 0, sizeof(shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(projectile_allowance_t));
    memset(&ext_robot_command, 0, sizeof(ext_robot_command));

    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));



}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index, sizeof(game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&robot_HP_t, frame + index, sizeof(game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t1, frame + index, sizeof(referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t1, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t1, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(projectile_allowance_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(student_interactive_data_t));
        }
        break;
		//		 case YUNTAISHOU_DATE:
//        {
//             memcpy(&ext_robot_command, frame + index, sizeof(ext_robot_command));
//        }
        break;
		
        default:
        {
            break;
        }
    }
}

void get_chassis_max_power(uint16_t *max_power_limit)
{
	*max_power_limit = robot_state.chassis_power_limit;
}

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t1.chassis_power;
    *buffer = power_heat_data_t1.buffer_energy;
    //*buffer =70;*power=20;
}


uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_barrel_cooling_value;
    *heat0 = power_heat_data_t1.shooter_17mm_2_barrel_heat;
}


void get_ext_robot_command_date(float *coordinate_x, float  *coordinate_y,uint8_t *commd_keyboard)
{
    *coordinate_x = ext_robot_command.target_position_x;
    *coordinate_y = ext_robot_command.target_position_y;
	  *commd_keyboard=ext_robot_command.commd_keyboard;		
	
//	 *coordinate_x =1.223;
//    *coordinate_y =1.2223;
//	  *commd_keyboard='A';		
	
	
	
	
}




void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_barrel_cooling_value;
    *heat1 = power_heat_data_t1.shooter_17mm_2_barrel_heat;
}
 void get_game_state(uint8_t *game_progress, uint16_t *stage_remain_time)
{
    *game_progress =game_state.game_progress ;
    *stage_remain_time = game_state.stage_remain_time;
}

