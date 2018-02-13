#include "stm32f10x.h"
#include "stm32_delay.h"
//#include "stm32f10x_conf.h"
//#include "string.h"
void ADC_init(void)
	
{
ADC1->SMPR1=0x00000000; //Sample time bits 3 bits for one channel, 17 channel
ADC1->SMPR2=0x00000000;	//Sample time bits 3 bits for one channel, 17 channel
ADC1->LTR=0x00000000;		//ADC_LTR low level  0..11
ADC1->HTR=0x000007ff;		//ADC_HTR High level 0..11
ADC1->JOFR1=0x00000000;	//data offset register 0..11
ADC1->CR2=0x00000001;		//CONTROL register 
_delay_ms(1);
ADC1->CR2=0x00000001;
ADC1->SMPR2=0x00000000;

ADC2->SMPR1=0x00000000; //Sample time bits 3 bits for one channel, 17 channel
ADC2->SMPR2=0x00000000;	//Sample time bits 3 bits for one channel, 17 channel
ADC2->LTR=0x00000000;		//ADC_LTR low level  0..11
ADC2->HTR=0x000007ff;		//ADC_HTR High level 0..11
ADC2->JOFR1=0x00000000;	//data offset register 0..11
ADC2->CR2=0x00000001;		//CONTROL register 
_delay_ms(1);
ADC2->CR2=0x00000001;
ADC2->SMPR2=0x00000000;
ADC2->SQR3=0x00000001;  //ADC_SQR3 naznachit 1 kanalom port 1
	
//ADC1->CR1=0x00030000;
}
uint32_t  ADC_mV(uint32_t ADC_in_mV)						 //funkcia perescheta rez. ADC v mV
{
	uint32_t ADC_out_mV=(int)((ADC_in_mV)*820)/1000; //pereschet znachenia s okrugleniem do 1 mV 
	return (ADC_out_mV);
	
}

