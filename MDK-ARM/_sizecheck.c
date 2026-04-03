#include <stdio.h>
#include "ui_types.h"
int main(){
printf("figure=%zu\n", sizeof(ui_interface_figure_t));
printf("number=%zu\n", sizeof(ui_interface_number_t));
printf("string=%zu\n", sizeof(ui_interface_string_t));
printf("frame1=%zu\n", sizeof(ui_1_frame_t));
printf("frames=%zu\n", sizeof(ui_string_frame_t));
return 0;
}
