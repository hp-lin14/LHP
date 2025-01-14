#ifndef _FOC_MAIN_H
#define _FOC_MAIN_H
#include "current_detect.h"
#include "math.h"
#include "as5600.h"
typedef struct  
{
	float target;
	float angle_target;

	float kp;
	float ki;
	float kd;
	float spd_kp;
	float spd_ki;
	float spd_kd;
	
	float measure;
	float angle_measure;
	float err;
	float last_err;
	float next_err;
	float angle_err;
	
	float pout;
	float iout;
	float dout;
	
	float speed_pout;
	float speed_iout;
	float speed_dout;
	float output;//本次输出
	float angle_output;
	float last_output;//上次输出
	
	float MAXout;//输出限幅
	float IntegralLimit;//积分限幅
}PID_TypeDef;

struct clark_Transformation{
	float Ualpha;
	float Ubeta;
};
float current_loop(int8_t Uq,PID_TypeDef*pid, CurrentDetect *UABC);
struct clark_Transformation anipark(float Uq);
CurrentDetect SVPWM_ZeroSequence(struct clark_Transformation*clark);
void SetPWM(CurrentDetect UABC);
float get_electric_angle(void);
#endif