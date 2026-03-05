#include "CANdata_analysis.h"
#include "main.h"


#define FLOAT_MAX 3.5f
#define FLOAT_MIN -3.5f
c_fbpara_t  C_data;


void C_fbdata(c_fbpara_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{
		
        motor->MODE = rx_data[0]/10;
		motor->MODE_STOOT = rx_data[0]%10;
		motor->pitch_ch = (rx_data[1]<<8)|rx_data[2]; 
		motor->yaw_ch = (rx_data[3]<<8)|rx_data[4]; 
	    int16_t compressed_data = (int16_t)((rx_data[5] << 8) | rx_data[6]);
	    motor->received_yaw = ((float)compressed_data / 32767.0f) * FLOAT_MAX;
		motor->received_v = rx_data[7];
	}
}

void C_fbdata1(c_fbpara_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{
		int16_t compressed_data = (int16_t)((rx_data[0] << 8) | rx_data[1]);
		motor->received_upyaw = ((float)compressed_data / 32767.0f) * FLOAT_MAX;	
		
	}
}
void C_fbdata2(c_fbpara_t *motor, uint8_t *rx_data,uint32_t data_len)
{ 
	if(data_len==8)
	{	
		motor->MODE=rx_data[0];
	}
}
