#ifndef UI_G_H
#define UI_G_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[7];
extern uint8_t ui_g_dirty_figure[7];
extern ui_interface_string_t ui_g_now_strings[5];
extern uint8_t ui_g_dirty_string[5];
extern uint8_t ui_g_max_send_count[12];

#define ui_g_Ungroup_leg_value ((ui_interface_number_t *)&ui_g_now_figures[0])
#define ui_g_Ungroup_car_line_left ((ui_interface_line_t *)&ui_g_now_figures[1])
#define ui_g_Ungroup_car_line_right ((ui_interface_line_t *)&ui_g_now_figures[2])
#define ui_g_Ungroup_car_body_line ((ui_interface_line_t *)&ui_g_now_figures[3])
#define ui_g_Ungroup_leg_line ((ui_interface_line_t *)&ui_g_now_figures[4])
#define ui_g_Ungroup_wheel_round ((ui_interface_round_t *)&ui_g_now_figures[5])
#define ui_g_Ungroup_power_value ((ui_interface_number_t *)&ui_g_now_figures[6])

#define ui_g_Ungroup_fire_mode (&ui_g_now_strings[0])
#define ui_g_Ungroup_w_flag (&ui_g_now_strings[1])
#define ui_g_Ungroup_mode_rc_1 (&ui_g_now_strings[2])
#define ui_g_Ungroup_mode_rc_0 (&ui_g_now_strings[3])
#define ui_g_Ungroup_power_text (&ui_g_now_strings[4])

void ui_init_g(void);
void ui_update_g(void);
void ui_force_refresh_g(void);
void ui_refresh_string_create_g(void);

#endif
