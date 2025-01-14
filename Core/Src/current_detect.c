#include "current_detect.h"
#include "foc_main.h"
#include <stdio.h>
uint8_t ADC_OFFSET=0;
float M0_offset_ia = 0.0f, M0_offset_ib=0.0f,M1_offset_ia =0.0f,M1_offset_ib=0.0f;
volatile  float Samp_volts[4];

 CurrentDetect M0_phase_current;
 CurrentDetect M1_phase_current;
 CurrentDetect UABC_set;
struct clark_Transformation clark_change;
union M0_ADC_UAB M0_UA;
union M0_ADC_UAB M0_UB;

union M1_ADC_UAB M1_UA;
union M1_ADC_UAB M1_UB;
unsigned char nail[4]={0x00,0x00,0x80,0x7F};
float Uq_result=0;
extern float ele_angle;
PID_TypeDef pid;
void DriftOffsets()
{
	uint16_t detect_rounds = 1000;
	for(int i = 0; i < detect_rounds; i++)
	{
		M0_offset_ia += Samp_volts[0];
		M0_offset_ib += Samp_volts[1];

		M1_offset_ia += Samp_volts[2];
		M1_offset_ib += Samp_volts[3];
	}
		M0_offset_ia = M0_offset_ia / detect_rounds;
		M0_offset_ib = M0_offset_ib / detect_rounds;

    M1_offset_ia = M1_offset_ia / detect_rounds;
    M1_offset_ib = M1_offset_ib / detect_rounds;
	
}
 //获取M0电机三相电流值         
CurrentDetect M0_GetPhaseCurrent(void)
{
		static  CurrentDetect current;
		current.I_a= (Samp_volts[0] - M0_offset_ia)/gain/res;
		current.I_b = (Samp_volts[1] - M0_offset_ib)/gain/res;

		current.U_a = (Samp_volts[0] - M0_offset_ia)/gain/res;
		current.U_b = (Samp_volts[1] - M0_offset_ib)/gain/res;
//		printf("M0_offset_ia:%f\r\n",M0_offset_ia);
//		printf("U_a:%f\r\n",current.U_a);
		
		return current;
}
//获取M1电机三相电流值
CurrentDetect M1_GetPhaseCurrent(void)
{
		static  CurrentDetect current;
		current.I_a= (Samp_volts[2] - M0_offset_ia)/gain/res;
		current.I_b = (Samp_volts[3] - M0_offset_ib)/gain/res;

		current.U_a = (Samp_volts[2] - M0_offset_ia)/gain;
		current.U_b = (Samp_volts[3] - M0_offset_ib)/gain;
		return current;
}
void send_vofa(void)
{
//	angle=AS5600_read_twobytes()*0.08789;
  	M0_UA.M0_U = TIM1->CCR1;
		M0_UB.M0_U = TIM1->CCR2;
  	M1_UA.M1_U = pid.target;
		M1_UB.M1_U = pid.measure;
//  	M0_UA.M0_U = M0_phase_current.U_a;
//		M0_UB.M0_U = M0_phase_current.U_b;
//		M1_UA.M1_U=pid.output;
//	  M1_UA.M1_U = M1_phase_current.U_a;
//	  M1_UB.M1_U = M1_phase_current.U_b;
//	M1_UB.M1_U =angle;
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart1,&M0_UA.M0_U_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart1,&M0_UB.M0_U_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart1,&M1_UA.M1_U_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart1,&M1_UB.M1_U_ARR[i],1,10);
		
	}
		for(int i=0;i<4;i++)
	{
		HAL_UART_Transmit(&huart1,&nail[i],1,10);
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);

		if(hadc == &hadc1)
		{
            //M0
			Samp_volts[0]=hadc1.Instance->JDR1;
			Samp_volts[0]=(Samp_volts[0])*_ADC_CONV;
			Samp_volts[1]=hadc1.Instance->JDR2;
			Samp_volts[1]=(Samp_volts[1])*_ADC_CONV;
            //M1
			Samp_volts[2]=hadc1.Instance->JDR3;
			Samp_volts[2]=(Samp_volts[2])*_ADC_CONV;
			Samp_volts[3]=hadc1.Instance->JDR4;
			Samp_volts[3]=(Samp_volts[3])*_ADC_CONV;

		if(ADC_OFFSET==1)
		{
				get_electric_angle();
				M0_phase_current=M0_GetPhaseCurrent();
				M1_phase_current=M1_GetPhaseCurrent();
			
				Uq_result=current_loop(-3,&pid,&M0_phase_current);
				clark_change=anipark(Uq_result);
				UABC_set=SVPWM_ZeroSequence(&clark_change);
				SetPWM(UABC_set);
				send_vofa();
		}
            
		}
        if(ADC_OFFSET==0)
        {
            DriftOffsets();
            ADC_OFFSET=1;
					 
        }

}

