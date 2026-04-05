#include "ui_interface.h"

#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "protocol.h"
#include "usart.h"

uint16_t ui_self_id = 0;
static uint8_t ui_seq = 0;

ui_string_frame_t ui_string_frame;
ui_1_frame_t ui_1_frame;
ui_2_frame_t ui_2_frame;
ui_5_frame_t ui_5_frame;
ui_7_frame_t ui_7_frame;

static void ui_fill_header(ui_frame_header_t *header, uint16_t data_length, uint16_t sub_id)
{
    header->SOF = HEADER_SOF;
    header->length = data_length;
    header->seq = ui_seq++;
    header->crc8 = 0;
    header->cmd_id = STUDENT_INTERACTIVE_DATA_CMD_ID;
    header->sub_id = sub_id;
    header->send_id = ui_self_id;
    header->recv_id = ui_self_id + 256U;
    append_CRC8_check_sum((uint8_t *)header, REF_PROTOCOL_HEADER_SIZE);
}

void print_message(const uint8_t *message, uint16_t length)
{
    if (ui_self_id == 0U)
    {
        return;
    }

    HAL_UART_Transmit(&huart6, (uint8_t *)message, length, 100);
    osDelay(12);
}

void ui_proc_1_frame(ui_1_frame_t *msg)
{
    ui_fill_header(&msg->header, 6U + sizeof(msg->data), 0x0101);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_proc_2_frame(ui_2_frame_t *msg)
{
    ui_fill_header(&msg->header, 6U + sizeof(msg->data), 0x0102);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_proc_5_frame(ui_5_frame_t *msg)
{
    ui_fill_header(&msg->header, 6U + sizeof(msg->data), 0x0103);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_proc_7_frame(ui_7_frame_t *msg)
{
    ui_fill_header(&msg->header, 6U + sizeof(msg->data), 0x0104);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_proc_string_frame(ui_string_frame_t *msg)
{
    msg->option.str_length = (uint32_t)strlen(msg->option.string);
    ui_fill_header(&msg->header, 6U + sizeof(msg->option), 0x0110);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_proc_delete_frame(ui_delete_frame_t *msg)
{
    ui_fill_header(&msg->header, 8U, 0x0100);
    append_CRC16_check_sum((uint8_t *)msg, sizeof(*msg));
}

void ui_delete_layer(uint8_t delete_type, uint8_t layer)
{
    ui_delete_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.delete_type = delete_type;
    frame.layer = layer;
    ui_proc_delete_frame(&frame);
    print_message((const uint8_t *)&frame, sizeof(frame));
}

static void ui_send_figure_pack(const ui_interface_figure_t *figures, int pack_size)
{
    if (pack_size == 7)
    {
        memcpy(ui_7_frame.data, figures, sizeof(ui_interface_figure_t) * 7U);
        ui_proc_7_frame(&ui_7_frame);
        print_message((const uint8_t *)&ui_7_frame, sizeof(ui_7_frame));
    }
    else if (pack_size == 5)
    {
        memcpy(ui_5_frame.data, figures, sizeof(ui_interface_figure_t) * 5U);
        ui_proc_5_frame(&ui_5_frame);
        print_message((const uint8_t *)&ui_5_frame, sizeof(ui_5_frame));
    }
    else if (pack_size == 2)
    {
        memcpy(ui_2_frame.data, figures, sizeof(ui_interface_figure_t) * 2U);
        ui_proc_2_frame(&ui_2_frame);
        print_message((const uint8_t *)&ui_2_frame, sizeof(ui_2_frame));
    }
    else
    {
        memcpy(ui_1_frame.data, figures, sizeof(ui_interface_figure_t));
        ui_proc_1_frame(&ui_1_frame);
        print_message((const uint8_t *)&ui_1_frame, sizeof(ui_1_frame));
    }
}

void ui_scan_and_send(const ui_interface_figure_t *ui_now_figures, uint8_t *ui_dirty_figure,
                      const ui_interface_string_t *ui_now_strings, uint8_t *ui_dirty_string,
                      int total_figures, int total_strings)
{
    int dirty_figure_count = 0;
    int sent_in_group = 0;
    int pack_size = 0;
    int remain = 0;
    int i = 0;
    ui_interface_figure_t figure_pack[7];

    if (ui_self_id == 0U)
    {
        return;
    }

    for (i = 0; i < total_figures; i++)
    {
        if (ui_dirty_figure[i] > 0U)
        {
            dirty_figure_count++;
        }
    }

    for (i = 0; i < total_figures; i++)
    {
        if (ui_dirty_figure[i] == 0U)
        {
            continue;
        }

        if (sent_in_group == 0)
        {
            remain = dirty_figure_count;
            if (remain > 5)
            {
                pack_size = 7;
            }
            else if (remain > 2)
            {
                pack_size = 5;
            }
            else if (remain > 1)
            {
                pack_size = 2;
            }
            else
            {
                pack_size = 1;
            }

            memset(figure_pack, 0, sizeof(figure_pack));
        }

        figure_pack[sent_in_group] = ui_now_figures[i];
        sent_in_group++;
        dirty_figure_count--;
        ui_dirty_figure[i]--;

        if (sent_in_group == pack_size)
        {
            ui_send_figure_pack(figure_pack, pack_size);
            sent_in_group = 0;
        }
    }

    if (sent_in_group > 0)
    {
        for (i = sent_in_group; i < pack_size; i++)
        {
            figure_pack[i].operate_type = 0;
        }
        ui_send_figure_pack(figure_pack, pack_size);
    }

    for (i = 0; i < total_strings; i++)
    {
        if (ui_dirty_string[i] == 0U)
        {
            continue;
        }

        ui_string_frame.option = ui_now_strings[i];
        ui_proc_string_frame(&ui_string_frame);
        print_message((const uint8_t *)&ui_string_frame, sizeof(ui_string_frame));
        ui_dirty_string[i]--;
    }
}
