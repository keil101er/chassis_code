#include "ui_g.h"

#include <string.h>

#define TOTAL_FIGURE 14
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

static const uint8_t ui_g_init_static_indices[4] = {1, 2, 6, 9};
static const uint8_t ui_g_init_move_indices[5] = {3, 4, 5, 7, 8};
static const uint8_t ui_g_init_state_data_indices[5] = {0, 10, 11, 12, 13};
static const uint8_t ui_g_runtime_move_indices[4] = {3, 4, 5, 7};
static const uint8_t ui_g_runtime_data_indices[3] = {0, 8, 13};
static const uint8_t ui_g_runtime_state_indices[3] = {10, 11, 12};

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

uint8_t ui_g_max_send_count[TOTAL_FIGURE + TOTAL_STRING] = {
    2, 1, 1, 2, 2, 2, 1,
    2, 2, 1, 3, 3, 3, 2,
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
    ui_g_Ungroup_leg_value->start_x = 1556;
    ui_g_Ungroup_leg_value->start_y = 397;
    ui_g_Ungroup_leg_value->width = 2;
    ui_g_Ungroup_leg_value->font_size = 20;

    ui_g_Ungroup_car_line_left->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_car_line_left->operate_type = 1;
    ui_g_Ungroup_car_line_left->layer = 0;
    ui_g_Ungroup_car_line_left->color = 3;
    ui_g_Ungroup_car_line_left->start_x = 675;
    ui_g_Ungroup_car_line_left->start_y = 126;
    ui_g_Ungroup_car_line_left->width = 2;
    ui_g_Ungroup_car_line_left->details_d = 835;
    ui_g_Ungroup_car_line_left->details_e = 602;

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
    ui_g_Ungroup_car_body_line->start_x = 1695;
    ui_g_Ungroup_car_body_line->start_y = 450;
    ui_g_Ungroup_car_body_line->width = 3;
    ui_g_Ungroup_car_body_line->details_d = 1844;
    ui_g_Ungroup_car_body_line->details_e = 450;

    ui_g_Ungroup_leg_line->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_leg_line->operate_type = 1;
    ui_g_Ungroup_leg_line->layer = 0;
    ui_g_Ungroup_leg_line->color = 1;
    ui_g_Ungroup_leg_line->start_x = 1770;
    ui_g_Ungroup_leg_line->start_y = 450;
    ui_g_Ungroup_leg_line->width = 3;
    ui_g_Ungroup_leg_line->details_d = 1770;
    ui_g_Ungroup_leg_line->details_e = 474;

    ui_g_Ungroup_wheel_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_wheel_round->operate_type = 1;
    ui_g_Ungroup_wheel_round->layer = 0;
    ui_g_Ungroup_wheel_round->color = 7;
    ui_g_Ungroup_wheel_round->start_x = 1770;
    ui_g_Ungroup_wheel_round->start_y = 474;
    ui_g_Ungroup_wheel_round->width = 5;
    ui_g_Ungroup_wheel_round->details_c = 24;

    ui_g_Ungroup_chassis->figure_type = UI_FIGURE_TYPE_RECT;
    ui_g_Ungroup_chassis->operate_type = 1;
    ui_g_Ungroup_chassis->layer = 0;
    ui_g_Ungroup_chassis->color = 2;
    ui_g_Ungroup_chassis->start_x = 1737;
    ui_g_Ungroup_chassis->start_y = 485;
    ui_g_Ungroup_chassis->width = 3;
    ui_g_Ungroup_chassis->details_d = 1797;
    ui_g_Ungroup_chassis->details_e = 565;

    ui_g_Ungroup_car_head->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_car_head->operate_type = 1;
    ui_g_Ungroup_car_head->layer = 0;
    ui_g_Ungroup_car_head->color = 8;
    ui_g_Ungroup_car_head->start_x = 1767;
    ui_g_Ungroup_car_head->start_y = 525;
    ui_g_Ungroup_car_head->width = 3;
    ui_g_Ungroup_car_head->details_d = 1767;
    ui_g_Ungroup_car_head->details_e = 585;

    ui_g_Ungroup_energe_buffer->figure_type = UI_FIGURE_TYPE_LINE;
    ui_g_Ungroup_energe_buffer->operate_type = 1;
    ui_g_Ungroup_energe_buffer->layer = 0;
    ui_g_Ungroup_energe_buffer->color = 2;
    ui_g_Ungroup_energe_buffer->start_x = 710;
    ui_g_Ungroup_energe_buffer->start_y = 120;
    ui_g_Ungroup_energe_buffer->width = 15;
    ui_g_Ungroup_energe_buffer->details_d = 1209;
    ui_g_Ungroup_energe_buffer->details_e = 120;

    ui_g_Ungroup_energe_buffer_Rect->figure_type = UI_FIGURE_TYPE_RECT;
    ui_g_Ungroup_energe_buffer_Rect->operate_type = 1;
    ui_g_Ungroup_energe_buffer_Rect->layer = 0;
    ui_g_Ungroup_energe_buffer_Rect->color = 8;
    ui_g_Ungroup_energe_buffer_Rect->start_x = 710;
    ui_g_Ungroup_energe_buffer_Rect->start_y = 120;
    ui_g_Ungroup_energe_buffer_Rect->width = 1;
    ui_g_Ungroup_energe_buffer_Rect->details_d = 1210;
    ui_g_Ungroup_energe_buffer_Rect->details_e = 135;

    ui_g_Ungroup_w_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_w_round->operate_type = 1;
    ui_g_Ungroup_w_round->layer = 0;
    ui_g_Ungroup_w_round->color = 3;
    ui_g_Ungroup_w_round->start_x = 115;
    ui_g_Ungroup_w_round->start_y = 759;
    ui_g_Ungroup_w_round->width = 6;
    ui_g_Ungroup_w_round->details_c = 30;

    ui_g_Ungroup_enable_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_enable_round->operate_type = 1;
    ui_g_Ungroup_enable_round->layer = 0;
    ui_g_Ungroup_enable_round->color = 3;
    ui_g_Ungroup_enable_round->start_x = 116;
    ui_g_Ungroup_enable_round->start_y = 682;
    ui_g_Ungroup_enable_round->width = 6;
    ui_g_Ungroup_enable_round->details_c = 30;

    ui_g_Ungroup_fiic_round->figure_type = UI_FIGURE_TYPE_CIRCLE;
    ui_g_Ungroup_fiic_round->operate_type = 1;
    ui_g_Ungroup_fiic_round->layer = 0;
    ui_g_Ungroup_fiic_round->color = 3;
    ui_g_Ungroup_fiic_round->start_x = 115;
    ui_g_Ungroup_fiic_round->start_y = 836;
    ui_g_Ungroup_fiic_round->width = 6;
    ui_g_Ungroup_fiic_round->details_c = 30;

    ui_g_Ungroup_power_value->figure_type = UI_FIGURE_TYPE_NUMBER;
    ui_g_Ungroup_power_value->operate_type = 1;
    ui_g_Ungroup_power_value->layer = 0;
    ui_g_Ungroup_power_value->color = 5;
    ui_g_Ungroup_power_value->start_x = 53;
    ui_g_Ungroup_power_value->start_y = 616;
    ui_g_Ungroup_power_value->width = 3;
    ui_g_Ungroup_power_value->font_size = 25;

    ui_init_text(ui_g_Ungroup_fire_mode, 1, 105, 854, 3, 30, "F");
    ui_init_text(ui_g_Ungroup_w_flag, 3, 103, 777, 3, 30, "R");
    ui_init_text(ui_g_Ungroup_enable_flag, 2, 103, 701, 3, 30, "E");

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
        ui_send_figure_subset(ui_g_init_static_indices, 4, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_MOVE;
        break;

    case UI_G_INIT_STAGE_MOVE:
        ui_send_figure_subset(ui_g_init_move_indices, 5, 1U);
        ui_g_init_stage = UI_G_INIT_STAGE_STATE_DATA;
        break;

    case UI_G_INIT_STAGE_STATE_DATA:
        ui_send_figure_subset(ui_g_init_state_data_indices, 5, 1U);
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
        ui_send_single_string(ui_g_Ungroup_enable_flag, 1U);
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
    ui_refresh_dirty_figures(ui_g_runtime_state_indices, 3);
    ui_send_dirty_figures(ui_g_runtime_state_indices, 3);

    ui_refresh_dirty_figures(ui_g_runtime_move_indices, 4);
    ui_send_dirty_figures(ui_g_runtime_move_indices, 4);

    ui_refresh_dirty_figures(ui_g_runtime_data_indices, 3);
    ui_send_dirty_figures(ui_g_runtime_data_indices, 3);
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
        figure_index_list[0] = 13U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 2:
        figure_index_list[0] = 7U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 3:
        figure_index_list[0] = 8U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 4:
        figure_index_list[0] = 10U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 5:
        figure_index_list[0] = 11U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 6:
        figure_index_list[0] = 12U;
        ui_send_figure_subset(figure_index_list, 1, 1U);
        break;

    case 7:
        ui_send_single_string(ui_g_Ungroup_fire_mode, 1U);
        break;

    case 8:
        ui_send_single_string(ui_g_Ungroup_w_flag, 1U);
        break;

    default:
        ui_send_single_string(ui_g_Ungroup_enable_flag, 1U);
        break;
    }

    refresh_cursor++;
    if (refresh_cursor >= 10U)
    {
        refresh_cursor = 0U;
    }
}
