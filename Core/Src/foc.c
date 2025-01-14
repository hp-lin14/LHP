#include "foc.h"
#include "math.h"
#include "usart.h"
#include "tim.h"
#define VDC 24
#define PWM_PERIOD 8000
#define PHASE_SHIFT_ANGLE (float)(220.0f/360.0f*2.0f*PI)
#define Uqmax 12.47//24*0.9/sqrt(3)
extern PID_TypeDef pid;
extern float HallTheta;
extern float uq_target;
union duty
{
	float Duty;
	uint8_t DUTY_ARR[4];
};
union target_measure
{
	float data;
	uint8_t ARR[4];
};
union duty DUTYA;
union duty DUTYB;
union duty DUTYC;
union target_measure pid_target;
union target_measure pid_measure;
union target_measure pid_out;
unsigned char nail[4]={0x00,0x00,0x80,0x7F};

float  Ia,Ib,Ic=0;
uint8_t HallReadTemp = 0;
extern float HallTemp;
extern float HallThetaAdd;
extern float HallTheta;
extern float HallSpeed;
float clark_park(struct CurrentDetect UABC)
{
	//克拉克变化
	float Ualpha=UABC.U_a;
	float Ubeta=sqrt(3)/3*(2*UABC.U_b+UABC.U_a);
	//帕克变化
	float Ud = Ualpha * cos(HallTheta) +Ubeta* sin(HallTheta);
  float Uq = -Ualpha * sin(HallTheta) + Ubeta * cos(HallTheta);
	return Uq;
}
float current_loop(uint8_t Uq,PID_TypeDef*pid,struct CurrentDetect UABC)
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
		clark.Ualpha = -Uq*sin(HallTheta);
		clark.Ubeta= Uq*cos(HallTheta);
		return clark;
	
}
struct CurrentDetect SVPWM_ZeroSequence(struct clark_Transformation*clark) {
    float Va_star, Vb_star, Vc_star;
    float V_zero;
		struct CurrentDetect UABC;
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
void SetPWM(struct CurrentDetect UABC) {
	  
    // 归一化电压值到PWM占空比
    int dutyA = (int)((UABC.U_a / VDC + 0.5f) * PWM_PERIOD);
    int dutyB = (int)((UABC.U_b / VDC + 0.5f) * PWM_PERIOD);
    int dutyC = (int)((UABC.U_c / VDC + 0.5f) * PWM_PERIOD);

    // 设置PWM寄存器 (示例)

    TIM1->CCR1 = dutyA;
    TIM1->CCR2 = dutyB;
    TIM1->CCR3 = dutyC;


		

}
void send_vofa(void)
{
		DUTYA.Duty=TIM1->CCR1;
		DUTYB.Duty=TIM1->CCR2;
		DUTYC.Duty=TIM1->CCR3;
		pid_target.data=uq_target;
		pid_measure.data=pid.measure;
//		pid_out.data=pid.output;
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&DUTYA.DUTY_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&DUTYB.DUTY_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&DUTYC.DUTY_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&pid_target.ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&pid_measure.ARR[i],1,10);
		
	}
//		for(int i=0;i<4;i++)
//	{
//		HAL_UART_Transmit(&huart3,&pid_out.ARR[i],1,10);
//		
//	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart3,&nail[i],1,10);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
	if(htim == &htim4)
	{
		HallTemp = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);;
		HallThetaAdd = (PI/3)/(HallTemp/3200000)/10000;
		HallSpeed = (PI/3)/(HallTemp/3200000)*30/(2*PI);
		HallReadTemp = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
		HallReadTemp |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)<<1;
		HallReadTemp |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)<<2;
		if(HallReadTemp==0x05)
    {
      HallTheta = 0.0f+PHASE_SHIFT_ANGLE+PI/6;
    }
    else if(HallReadTemp==0x04)
    {
      HallTheta = (PI/3.0f)+PHASE_SHIFT_ANGLE+PI/6;
    }
    else if(HallReadTemp==0x06)
    {
      HallTheta = (PI*2.0f/3.0f)+PHASE_SHIFT_ANGLE+PI/6;
    }
    else if(HallReadTemp==0x02)
    {
      HallTheta = PI+PHASE_SHIFT_ANGLE+PI/6;
    }
    else if(HallReadTemp==0x03)
    {
      HallTheta = (PI*4.0f/3.0f)+PHASE_SHIFT_ANGLE+PI/6;
    }
    else if(HallReadTemp==0x01)
    {
      HallTheta = (PI*5.0f/3.0f)+PHASE_SHIFT_ANGLE+PI/6;
    }
    if(HallTheta<0.0f)
    {
      HallTheta += 2.0f*PI;
    }
    else if(HallTheta>(2.0f*PI))
    {
      HallTheta -= 2.0f*PI;
    }

	}
}