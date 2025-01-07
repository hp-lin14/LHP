#ifndef _FOC_H
#define _FOC_H
#include "main.h"
#include "current_detect.h"
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
float clark_park(struct CurrentDetect UABC);
void SetPWM(struct CurrentDetect UABC);
struct CurrentDetect SVPWM_ZeroSequence(struct clark_Transformation*clark);
float current_loop(uint8_t Uq,PID_TypeDef*pid,struct CurrentDetect UABC);
struct clark_Transformation anipark(float Uq);
void send_vofa(void);
#endif
