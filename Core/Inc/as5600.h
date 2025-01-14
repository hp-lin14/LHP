#ifndef _AS5600_H
#define _AS5600_H

#include "main.h"

typedef enum {
   _raw_ang_hi=0x0c,
	_raw_ang_lo=0x0d,
	_ams5600_Address=0x36<<1  //原7bit地址0x36，8位要向左移动一位
	
}AS5600_Register_e;
int16_t AS5600_read_twobytes(void);
#endif