#include "as5600.h"
#include "i2c.h"
float angle_origanl=0;
int16_t AS5600_read_twobytes(void)
{
	int16_t retVal=-1;
	uint8_t low_data=0;
	uint8_t high_data=0;
	HAL_I2C_Mem_Read(&hi2c1,0x36<<1,0x0d ,I2C_MEMADD_SIZE_8BIT, &low_data, 1, 50);
	HAL_I2C_Mem_Read(&hi2c1,0x36<<1,0x0c ,I2C_MEMADD_SIZE_8BIT, &high_data, 1, 50);
	retVal = (high_data << 8) | low_data;
//	angle_origanl=retVal*0.08789;
	return retVal;
}