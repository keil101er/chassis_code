#ifndef CAN_ANALYSIS_H
#define CAN_ANALYSIS_H

#include "struct_typedef.h"

typedef struct 
{

	uint8_t  MODE;
	uint8_t  MODE_STOOT;
	int16_t  pitch_ch;
	int16_t  yaw_ch;
	float    received_yaw;
	float    received_upyaw;
	uint8_t  received_v;
}c_fbpara_t;

extern c_fbpara_t  C_data;

extern void C_fbdata(c_fbpara_t *motor, uint8_t *rx_data,uint32_t data_len);
extern void C_fbdata1(c_fbpara_t *motor, uint8_t *rx_data,uint32_t data_len);
#endif
