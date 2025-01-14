#include "foc_main.h"
#include <stdio.h>
#define VDC 12.0f
#define PWM_PERIOD 1200
#define Uqmax 6.24//12*0.9/sqrt(3)

float ele_angle=0;
float get_electric_angle(void)
{
		ele_angle=AS5600_read_twobytes()*0.001533*4;   //读取电角度
    return ele_angle;
}
float clark_park(CurrentDetect *UABC)
{
	//克拉克变化
	float Ualpha=UABC->U_a;
	float Ubeta=sqrt(3)/3*(2*UABC->U_b+UABC->U_a);
	//帕克变化
	float Ud = Ualpha * cos(ele_angle) + Ubeta * sin(ele_angle);
  float Uq = -Ualpha * sin(ele_angle) + Ubeta * cos(ele_angle);
//	printf("ele_angle:%f\r\n",ele_angle);
	return Uq;
}
float current_loop(int8_t Uq,PID_TypeDef*pid, CurrentDetect *UABC)
{
	//电流环PI控制
	pid->target=Uq;
	pid->measure=clark_park(UABC);
	pid->err=pid->target-pid->measure;
	
	 pid->pout=pid->kp*pid->err;
	 pid->iout+=pid->ki*pid->err;
	
	 pid->output=pid->pout+pid->iout;
	if(pid->output>Uqmax)
	{
		pid->output=Uqmax;
	}
	if(pid->output<-Uqmax)
	{
		pid->output=-Uqmax;
	}
	return pid->output;
}
struct clark_Transformation anipark(float Uq)
{
		struct clark_Transformation clark;
		clark.Ualpha = -Uq*sin(ele_angle);
		clark.Ubeta= Uq*cos(ele_angle);
		return clark;
	
}
CurrentDetect SVPWM_ZeroSequence(struct clark_Transformation*clark) {
    float Va_star, Vb_star, Vc_star;
    float V_zero;
		static  CurrentDetect UABC;
    // 1. 反 Clarke 变换：计算初始三相电压
    Va_star = clark->Ualpha;
    Vb_star = -0.5f * clark->Ualpha + (sqrtf(3) / 2.0f) *clark->Ubeta;
    Vc_star = -0.5f * clark->Ualpha - (sqrtf(3) / 2.0f) * clark->Ubeta;
   
    // 2. 计算零序分量 V0
    float V_max = fmaxf(fmaxf(Va_star, Vb_star), Vc_star);
    float V_min = fminf(fminf(Va_star, Vb_star), Vc_star);
    V_zero = (V_max + V_min) / 2.0f;

    // 3. 注入零序分量，调整三相电压
    UABC.U_a= Va_star - V_zero;
    UABC.U_b= Vb_star - V_zero;
    UABC.U_c= Vc_star - V_zero;
		return UABC;
}
void SetPWM( CurrentDetect UABC) {
	  
    // 归一化电压值到PWM占空比
    int dutyA = (int)((UABC.U_a / VDC+0.5f ) * PWM_PERIOD);
    int dutyB = (int)((UABC.U_b / VDC+0.5f) * PWM_PERIOD);
    int dutyC = (int)((UABC.U_c / VDC+0.5f) * PWM_PERIOD);

    // 设置PWM寄存器 (示例)

    TIM1->CCR1 = dutyA;
    TIM1->CCR2 = dutyB;
    TIM1->CCR3 = dutyC;


		

}