#include "current_detect.h"
#include "adc.h"
#include "foc.h"
uint8_t ADC_OFFSET=0;
float offset_ia = 0.0, offset_ib,offset_ic = 0.0;
float HallTemp = 0;
float HallThetaAdd = 0;
float HallTheta = 0;
float HallSpeed = 0;
float uq_target=2;
volatile  float Samp_volts[3];
struct CurrentDetect current;
struct CurrentDetect UABC;
struct clark_Transformation clark_change;
struct CurrentDetect UABC_set;
PID_TypeDef pid;

void DriftOffsets()
{
	uint16_t detect_rounds = 10;
	for(int i = 0; i < detect_rounds; i++)
	{
		offset_ia += Samp_volts[0];
		offset_ib += Samp_volts[1];
		offset_ic += Samp_volts[2];
	}
	offset_ia = offset_ia / detect_rounds;
	offset_ib = offset_ib / detect_rounds;
	offset_ic = offset_ic / detect_rounds;
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

		if(hadc == &hadc1)
		{
				HallTheta = HallTheta + HallThetaAdd;
				if(HallTheta<0.0f)
				{
					HallTheta += 2.0f*PI;
				}
				else if(HallTheta>(2.0f*PI))
				{
					HallTheta -= 2.0f*PI;
				}
			Samp_volts[0]=hadc1.Instance->JDR1;
			Samp_volts[0]=(Samp_volts[0] - 0x7ef)*0.02197f;
			Samp_volts[1]=hadc1.Instance->JDR2;
			Samp_volts[1]=(Samp_volts[1]-0x7f5)*0.02197f;
			Samp_volts[2]=hadc2.Instance->JDR1;
			Samp_volts[2]=(Samp_volts[2]-0x7e8)*0.02197f;
			float Uq_pid_result=0;
			UABC=GetPhaseCurrent();
			Uq_pid_result=current_loop(uq_target,&pid,UABC);
			clark_change=anipark(Uq_pid_result);
			UABC_set=SVPWM_ZeroSequence(&clark_change);
			SetPWM(UABC_set);
			send_vofa();
		}
	if(ADC_OFFSET==0)
	{
		DriftOffsets();
		ADC_OFFSET=1;
	}

}
          
struct CurrentDetect GetPhaseCurrent()
{
		current.I_a = (Samp_volts[0] - offset_ia);
		current.I_b = (Samp_volts[1] - offset_ib);
		current.I_c=	(Samp_volts[2] - offset_ic);
		
		current.U_a = (Samp_volts[0] - offset_ia)*0.005f;
		current.U_b = (Samp_volts[1] - offset_ib)*0.005f;
		current.U_c = (Samp_volts[2] - offset_ic)*0.005f;
	return current;
}
