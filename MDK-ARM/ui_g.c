#include "ui_g.h"

#include <string.h>

#define TOTAL_FIGURE 12
#define TOTAL_STRING 3

typedef enum
{
    UI_G_INIT_STAGE_STATIC = 0,
    UI_G_INIT_STAGE_MOVE,
    UI_G_INIT_STAGE_STATE_DATA,
    UI_G_INIT_STAGE_STRING_F,
    UI_G_INIT_STAGE_STRING_R,
    UI_G_INIT_STAGE_STRING_E,
    UI_G_INIT_STAGE_DONE
} ui_g_init_stage_e;

static const uint8_t ui_g_init_static_indices[2] = {1, 2};
static const uint8_t ui_g_init_move_indices[4] = {3, 4, 5, 10};
static const uint8_t ui_g_init_state_data_indices[6] = {0, 6, 7, 8, 9, 11};
static const uint8_t ui_g_runtime_move_indices[4] = {3, 4, 5, 10};
static const uint8_t ui_g_runtime_data_indices[3] = {0, 6, 11};
static const uint8_t ui_g_runtime_state_indices[3] = {7, 8, 9};

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    2, 1, 1, 2, 2, 2,
    2, 3, 3, 3, 2, 2,
    1, 1, 1,
};

static ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
static ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
static ui_g_init_stage_e ui_g_init_stage = UI_G_INIT_STAGE_DONE;

static void ui_init_text(ui_interface_string_t *obj, uint32_t color, uint32_t x, uint32_t y,
                         uint32_t width, uint32_t font_size, const char *text)
{
    memset(obj, 0, sizeof(*obj));
    obj->figure_type = UI_FIGURE_TYPE_STRING;
    obj->operate_type = 1;
    obj->layer = 0;
    obj->color = color;
    obj->start_x = x;
    obj->start_y = y;
    obj->width = width;
    obj->font_size = font_size;
    strncpy(obj->string, text, sizeof(obj->string) - 1U);
    obj->str_length = (uint32_t)strlen(obj->string);
}

static void ui_prepare_names(void)
{
    uint32_t idx;
    uint32_t string_name;

    for (idx = 0U; idx < TOTAL_FIGURE; idx++)
    {
        ui_g_now_figures[idx].figure_name[0] = (uint8_t)((idx >> 16) & 0xFFU);
        ui_g_now_figures[idx].figure_name[1] = (uint8_t)((idx >> 8) & 0xFFU);
        ui_g_now_figures[idx].figure_name[2] = (uint8_t)(idx & 0xFFU);
    }

    for (idx = 0U; idx < TOTAL_STRING; idx++)
    {
        string_name = idx + TOTAL_FIGURE;
        ui_g_now_strings[idx].figure_name[0] = (uint8_t)((string_name >> 16) & 0xFFU);
        ui_g_now_strings[idx].figure_name[1] = (uint8_t)((string_name >> 8) & 0xFFU);
        ui_g_now_strings[idx].figure_name[2] = (uint8_t)(string_name & 0xFFU);
    }
}

static void ui_send_figure_subset(const uint8_t *indices, int count, uint32_t operate_type)
{
    ui_interface_figure_t figures[7];
    uint8_t dirty[7];
    int i;

    memset(figures, 0, sizeof(figures));
    memset(dirty, 0, sizeof(dirty));

    for (i = 0; i < count; i++)
    {
        figures[i] = ui_g_now_figures[indices[i]];
        figures[i].operate_type = operate_type;
        dirty[i] = 1U;
    }

    ui_scan_and_send(figures, dirty, 0, 0, count, 0);
}

static void ui_send_single_string(const ui_interface_string_t *obj, uint32_t operate_type)
{
    ui_string_frame.option = *obj;
    ui_string_frame.option.operate_type = operate_type;
    ui_proc_string_frame(&ui_string_frame);
    print_message((const uint8_t *)&ui_string_frame, sizeof(ui_string_frame));
}

static void ui_mark_all_update_mode(void)
{
    int i;

    for (i = 0; i < TOTAL_FIGURE; i++)
    {
        ui_g_now_figures[i].operate_type = 2U;
        ui_g_last_figures[i] = ui_g_now_figures[i];
        ui_g_dirty_figure[i] = 0U;
    }

    for (i = 0; i < TOTAL_STRING; i++)
    {
        ui_g_now_strings[i].operate_type = 2U;
        ui_g_last_strings[i] = ui_g_now_strings[i];
        ui_g_dirty_string[i] = 0U;
    }
}

static void ui_refresh_dirty_figures(const uint8_t *indices, int count)
{
    int i;
    uint8_t figure_index;

    for (i = 0; i < count; i++)
    {
        figure_index = indices[i];
        if (memcmp(&ui_g_now_figures[figure_index], &ui_g_last_figures[figure_index],
                   sizeof(ui_g_now_figures[figure_index])) != 0)
        {
            ui_g_dirty_figure[figure_index] = ui_g_max_send_count[figure_index];
            ui_g_last_figures[figure_index] = ui_g_now_figures[figure_index];
        }
    }
}

static uint8_t ui_has_dirty_figures(const uint8_t *indices, int count)
{
    int i;

    for (i = 0; i < count; i++)
    {
        if (ui_g_dirty_figure[indices[i]] > 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static void ui_send_dirty_figures(const uint8_t *indices, int count)
{
    ui_interface_figure_t figures[7];
    uint8_t dirty[7];
    uint8_t selected_indices[7];
    int selected_count = 0;
    int i;
    uint8_t figure_index;

    memset(figures, 0, sizeof(figures));
    memset(dirty, 0, sizeof(dirty));
    memset(selected_indices, 0, sizeof(selected_indices));

    for (i = 0; i < count; i++)
    {
        figure_index = indices[i];
        if (ui_g_dirty_figure[figure_index] == 0U)
        {
            continue;
        }

        figures[selected_count] = ui_g_now_figures[figure_index];
        dirty[selected_count] = ui_g_dirty_figure[figure_index];
        selected_indices[selected_count] = figure_index;
        selected_count++;
    }

    if (selected_count == 0)
    {
        return;
    }

    ui_scan_and_send(figures, dirty, 0, 0, selected_count, 0);

    for (i = 0; i < selected_count; i++)
    {
        ui_g_dirty_figure[selected_indices[i]] = dirty[i];
    }
}

void ui_init_g(void)
{
    memset(ui_g_now_figures, 0, sizeof(ui_g_now_figures));
    memset(ui_g_dirty_figure, 0, sizeof(ui_g_dirty_figure));
    memset(ui_g_now_strings, 0, sizeof(ui_g_now_strings));
    memset(ui_g_dirty_string, 0, sizeof(ui_g_dirty_string));
    memset(ui_g_last_figures, 0, sizeof(ui_g_last_figures));
    memset(ui_g_last_strings, 0, sizeof(ui_g_last_strings));

    ui_g_Ungroup_leg_value->figure_type = UI_FIGURE_TYPE_NUMBER;
    ui_g_Ungroup_leg_value->operate_type = 1;
    ui_g_Ungroup_leg_value->layer = 0;
    ui_g_Ungroup_leg_value->color = 6;
    ui_g_Ungroup_leg_value->start_x = 1661;
    ui_g_Ungroup_leg_value->start_y = 391;
    ui_g_Ungroup_leg_value->width = 2;
    ui_g_Ungroup_leg_value->font_size = 20;

    ui_g_Ungroup_car_line_left->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_car_line_left->operate_type = 1;
    ui_g_Ungroup_car_line_left->layer = 0;
    ui_g_Ungroup_car_line_left->color = 3;
    ui_g_Ungroup_car_line_left->start_x = 684;
    ui_g_Ungroup_car_line_left->start_y = 127;
    ui_g_Ungroup_car_line_left->width = 2;
    ui_g_Ungroup_car_line_left->details_d = 844;
    ui_g_Ungroup_car_line_left->details_e = 603;

    ui_g_Ungroup_car_line_right->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_car_line_right->operate_type = 1;
    ui_g_Ungroup_car_line_right->layer = 0;
    ui_g_Ungroup_car_line_right->color = 3;
    ui_g_Ungroup_car_line_right->start_x = 1238;
    ui_g_Ungroup_car_line_right->start_y = 109;
    ui_g_Ungroup_car_line_right->width = 2;
    ui_g_Ungroup_car_line_right->details_d = 1078;
    ui_g_Ungroup_car_line_right->details_e = 602;

    ui_g_Ungroup_car_body_line->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_car_body_line->operate_type = 1;
    ui_g_Ungroup_car_body_line->layer = 0;
    ui_g_Ungroup_car_body_line->color = 8;
    ui_g_Ungroup_car_body_line->start_x = 1497;
    ui_g_Ungroup_car_body_line->start_y = 441;
    ui_g_Ungroup_car_body_line->width = 3;
    ui_g_Ungroup_car_body_line->details_d = 1646;
    ui_g_Ungroup_car_body_line->details_e = 441;

    ui_g_Ungroup_leg_line->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_leg_line->operate_type = 1;
    ui_g_Ungroup_leg_line->layer = 0;
    ui_g_Ungroup_leg_line->color = 1;
    ui_g_Ungroup_leg_line->start_x = 1567;
    ui_g_Ungroup_leg_line->start_y = 340;
    ui_g_Ungroup_leg_line->width = 3;
    ui_g_Ungroup_leg_line->details_d = 1567;
    ui_g_Ungroup_leg_line->details_e = 442;

    ui_g_Ungroup_wheel_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_wheel_round->operate_type = 1;
    ui_g_Ungroup_wheel_round->layer = 0;
    ui_g_Ungroup_wheel_round->color = 5;
    ui_g_Ungroup_wheel_round->start_x = 1567;
    ui_g_Ungroup_wheel_round->start_y = 339;
    ui_g_Ungroup_wheel_round->width = 5;
    ui_g_Ungroup_wheel_round->details_c = 24;

    ui_g_Ungroup_energr_buffer->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_energr_buffer->operate_type = 1;
    ui_g_Ungroup_energr_buffer->layer = 0;
    ui_g_Ungroup_energr_buffer->color = 2;
    ui_g_Ungroup_energr_buffer->start_x = 710;
    ui_g_Ungroup_energr_buffer->start_y = 80;
    ui_g_Ungroup_energr_buffer->width = 15;
    ui_g_Ungroup_energr_buffer->details_d = 1209;
    ui_g_Ungroup_energr_buffer->details_e = 80;

    ui_g_Ungroup_Fire_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_Fire_round->operate_type = 1;
    ui_g_Ungroup_Fire_round->layer = 0;
    ui_g_Ungroup_Fire_round->color = 4;
    ui_g_Ungroup_Fire_round->start_x = 225;
    ui_g_Ungroup_Fire_round->start_y = 814;
    ui_g_Ungroup_Fire_round->width = 6;
    ui_g_Ungroup_Fire_round->details_c = 30;

    ui_g_Ungroup_W_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_W_round->operate_type = 1;
    ui_g_Ungroup_W_round->layer = 0;
    ui_g_Ungroup_W_round->color = 4;
    ui_g_Ungroup_W_round->start_x = 223;
    ui_g_Ungroup_W_round->start_y = 726;
    ui_g_Ungroup_W_round->width = 6;
    ui_g_Ungroup_W_round->details_c = 30;

    ui_g_Ungroup_enable_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_enable_round->operate_type = 1;
    ui_g_Ungroup_enable_round->layer = 0;
    ui_g_Ungroup_enable_round->color = 4;
    ui_g_Ungroup_enable_round->start_x = 223;
    ui_g_Ungroup_enable_round->start_y = 644;
    ui_g_Ungroup_enable_round->width = 6;
    ui_g_Ungroup_enable_round->details_c = 30;

    ui_g_Ungroup_chassis_dirct->figure_type = UI_FIGURE_TYPE_ARC;
    ui_g_Ungroup_chassis_dirct->operate_type = 1;
    ui_g_Ungroup_chassis_dirct->layer = 0;
    ui_g_Ungroup_chassis_dirct->color = 6;
    ui_g_Ungroup_chassis_dirct->start_x = 960;
    ui_g_Ungroup_chassis_dirct->start_y = 540;
    ui_g_Ungroup_chassis_dirct->width = 8;
    ui_g_Ungroup_chassis_dirct->details_a = 330;
    ui_g_Ungroup_chassis_dirct->details_b = 30;
    ui_g_Ungroup_chassis_dirct->details_d = 90;
    ui_g_Ungroup_chassis_dirct->details_e = 90;

    ui_g_Ungroup_power_value->figure_type = UI_FIGURE_TYPE_NUMBER;
    ui_g_Ungroup_power_value->operate_type = 1;
    ui_g_Ungroup_power_value->layer = 0;
    ui_g_Ungroup_power_value->color = 5;
    ui_g_Ungroup_power_value->start_x = 171;
    ui_g_Ungroup_power_value->start_y = 580;
    ui_g_Ungroup_power_value->width = 2;
    ui_g_Ungroup_power_value->font_size = 20;

    ui_init_text(ui_g_Ungroup_fire_mode, 3, 216, 830, 3, 30, "F");
    ui_init_text(ui_g_Ungroup_w_flag, 3, 212, 743, 3, 30, "R");
    ui_init_text(ui_g_Ungroup_Enable_flag, 3, 212, 663, 3, 30, "E");

    ui_prepare_names();
    ui_g_init_stage = UI_G_INIT_STAGE_STATIC;
}

uint8_t ui_init_step_g(void)
{
    if (ui_g_init_stage == UI_G_INIT_STAGE_DONE)
    {
        return 1U;
    }

    switch (ui_g_init_stage)
    {
    case UI_G_INIT_STAGE_STATIC:
        ui_send_figure_subset(ui_g_init_static_indices, 2, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_MOVE;
        break;

    case UI_G_INIT_STAGE_MOVE:
        ui_send_figure_subset(ui_g_init_move_indices, 4, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_STATE_DATA;
        break;

    case UI_G_INIT_STAGE_STATE_DATA:
        ui_send_figure_subset(ui_g_init_state_data_indices, 6, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_STRING_F;
        break;

    case UI_G_INIT_STAGE_STRING_F:
        ui_send_single_string(ui_g_Ungroup_fire_mode, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_STRING_R;
        break;

    case UI_G_INIT_STAGE_STRING_R:
        ui_send_single_string(ui_g_Ungroup_w_flag, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_STRING_E;
        break;

    case UI_G_INIT_STAGE_STRING_E:
        ui_send_single_string(ui_g_Ungroup_Enable_flag, 1U);
        ui_mark_all_update_mode();
        ui_g_init_stage = UI_G_INIT_STAGE_DONE;
        return 1U;

    default:
        ui_mark_all_update_mode();
        ui_g_init_stage = UI_G_INIT_STAGE_DONE;
        return 1U;
    }

    return 0U;
}

void ui_update_g(void)
{
    static uint8_t runtime_slot = 0U;
    uint8_t has_state = 0U;
    uint8_t has_move = 0U;
    uint8_t has_data = 0U;

    ui_refresh_dirty_figures(ui_g_runtime_state_indices, 3);
    ui_refresh_dirty_figures(ui_g_runtime_move_indices, 4);
    ui_refresh_dirty_figures(ui_g_runtime_data_indices, 3);

    has_state = ui_has_dirty_figures(ui_g_runtime_state_indices, 3);
    has_move = ui_has_dirty_figures(ui_g_runtime_move_indices, 4);
    has_data = ui_has_dirty_figures(ui_g_runtime_data_indices, 3);

    switch (runtime_slot)
    {
    case 0:
        if (has_data != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
        }
        else if (has_move != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
        }
        else if (has_state != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
        }
        break;

    case 1:
        if (has_move != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
        }
        else if (has_data != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
        }
        else if (has_state != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
        }
        break;

    case 2:
        if (has_data != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
        }
        else if (has_move != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
        }
        else if (has_state != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
        }
        break;

    case 3:
        if (has_move != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
        }
        else if (has_data != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
        }
        else if (has_state != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
        }
        break;

    default:
        if (has_state != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
        }
        else if (has_data != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
        }
        else if (has_move != 0U)
        {
            ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
        }
        break;
    }

    runtime_slot++;
    if (runtime_slot >= 5U)
    {
        runtime_slot = 0U;
    }
}

void ui_force_refresh_g(void)
{
    int i;

    for (i = 0; i < TOTAL_FIGURE; i++)
    {
        ui_g_dirty_figure[i] = ui_g_max_send_count[i];
    }

    ui_send_dirty_figures(ui_g_runtime_state_indices, 3);
    ui_send_dirty_figures(ui_g_runtime_move_indices, 4);
    ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
}

void ui_refresh_fragile_create_g(void)
{
    static uint8_t refresh_cursor = 0U;
    uint8_t figure_index_list[1];

    switch (refresh_cursor)
    {
    case 0:
        figure_index_list[0] = 0U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 1:
        figure_index_list[0] = 11U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 2:
        figure_index_list[0] = 6U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 3:
        figure_index_list[0] = 10U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 4:
        figure_index_list[0] = 7U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 5:
        figure_index_list[0] = 8U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 6:
        figure_index_list[0] = 9U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 7:
        ui_send_single_string(ui_g_Ungroup_fire_mode, 1U);
        break;

    case 8:
        ui_send_single_string(ui_g_Ungroup_w_flag, 1U);
        break;

    default:
        ui_send_single_string(ui_g_Ungroup_Enable_flag, 1U);
        break;
    }

    refresh_cursor++;
    if (refresh_cursor >= 10U)
    {
        refresh_cursor = 0U;
    }
}
