#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"
#include "referee.h"


//#define FRIC_UP 1390
//#define FRIC_DOWN 1310
#define FRIC_OFF 1000
#define FRIC_DOWN (robot_state.shooter_id1_17mm_speed_limit<=17?1460:1510)//1525)//<=17?1510:1560)
                                                                       //Êµ²âµ¯ËÙ14.9/17.5
#define FRIC_UP (FRIC_DOWN+80)

extern robot_status_t robot_state;

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif

