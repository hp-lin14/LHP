#ifndef _CURRENT_DETECT_H
#define _CURRENT_DETECT_H
#include "main.h"

#include "as5600.h"
#include "usart.h"
#include "adc.h"
#define PI 3.14159265358979323846
#define _ADC_CONV    0.00080586f
#define gain -50.0f
#define res 0.01f

union M0_ADC_UAB
{
	float M0_U;
	uint8_t M0_U_ARR[4];
};
union M1_ADC_UAB
{
	float M1_U;
	uint8_t M1_U_ARR[4];
};
typedef struct {
	float I_a;
	float I_b;
	float I_c;
	float U_a;
	float U_b;
	float U_c;
}CurrentDetect;

CurrentDetect M0_GetPhaseCurrent(void);
CurrentDetect M1_GetPhaseCurrent(void);
void send_vofa(void);
#endif