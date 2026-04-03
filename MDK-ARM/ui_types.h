#ifndef UI_TYPES_H
#define UI_TYPES_H

#include <stdint.h>

#if defined(__GNUC__) || defined(__CC_ARM)
#define UI_PACKED __attribute__((packed))
#else
#error "Unsupported compiler for UI packed structures"
#endif

#define UI_FIGURE_TYPE_LINE   0
#define UI_FIGURE_TYPE_RECT   1
#define UI_FIGURE_TYPE_CIRCLE 2
#define UI_FIGURE_TYPE_ELLIPSE 3
#define UI_FIGURE_TYPE_ARC    4
#define UI_FIGURE_TYPE_NUMBER 5
#define UI_FIGURE_TYPE_FLOAT  6
#define UI_FIGURE_TYPE_STRING 7

typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} UI_PACKED ui_interface_figure_t;

typedef ui_interface_figure_t ui_interface_line_t;
typedef ui_interface_figure_t ui_interface_rect_t;
typedef ui_interface_figure_t ui_interface_round_t;
typedef ui_interface_figure_t ui_interface_ellipse_t;
typedef ui_interface_figure_t ui_interface_arc_t;

typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t font_size : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    int32_t number;
} UI_PACKED ui_interface_number_t;

typedef struct
{
    uint8_t figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t font_size : 9;
    uint32_t str_length : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
    char string[30];
} UI_PACKED ui_interface_string_t;

typedef struct
{
    uint8_t SOF;
    uint16_t length;
    uint8_t seq;
    uint8_t crc8;
    uint16_t cmd_id;
    uint16_t sub_id;
    uint16_t send_id;
    uint16_t recv_id;
} UI_PACKED ui_frame_header_t;

typedef struct
{
    ui_frame_header_t header;
    ui_interface_figure_t data[1];
    uint16_t crc16;
} UI_PACKED ui_1_frame_t;

typedef struct
{
    ui_frame_header_t header;
    ui_interface_figure_t data[2];
    uint16_t crc16;
} UI_PACKED ui_2_frame_t;

typedef struct
{
    ui_frame_header_t header;
    ui_interface_figure_t data[5];
    uint16_t crc16;
} UI_PACKED ui_5_frame_t;

typedef struct
{
    ui_frame_header_t header;
    ui_interface_figure_t data[7];
    uint16_t crc16;
} UI_PACKED ui_7_frame_t;

typedef struct
{
    ui_frame_header_t header;
    ui_interface_string_t option;
    uint16_t crc16;
} UI_PACKED ui_string_frame_t;

typedef struct
{
    ui_frame_header_t header;
    uint8_t delete_type;
    uint8_t layer;
    uint16_t crc16;
} UI_PACKED ui_delete_frame_t;

extern ui_string_frame_t ui_string_frame;
extern ui_1_frame_t ui_1_frame;
extern ui_2_frame_t ui_2_frame;
extern ui_5_frame_t ui_5_frame;
extern ui_7_frame_t ui_7_frame;

#endif
