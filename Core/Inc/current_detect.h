#ifndef _CURRENT_DETECT_H
#define _CURRENT_DETECT_H
#include "main.h"
#define PI					3.14159265358979f
struct CurrentDetect{
	float I_a;
	float I_b;
	float I_c;
	float U_a;
	float U_b;
	float U_c;
};
struct CurrentDetect GetPhaseCurrent(void);
#endif