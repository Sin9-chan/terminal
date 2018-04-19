#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stdio.h"
#include "string.h"
#include "DMA_init.h"


extern volatile uint8_t Receive_Buffer[64];
extern volatile uint32_t Receive_length ;
extern volatile uint32_t length ;
uint8_t Send_Buffer[64];
uint8_t x=0x33;
uint8_t Transmit_Buffer[127];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
int STROB=0;
int cnt_pix=0;

void SetSysClockTo72(void)
{
	ErrorStatus HSEStartUpStatus;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig( RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
    	//FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);

        /* Flash 2 wait state */
        //FLASH_SetLatency( FLASH_Latency_2);

        /* HCLK = SYSCLK */
        RCC_HCLKConfig( RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config( RCC_HCLK_Div1);

        /* PCLK1 = HCLK/2 */
        RCC_PCLK1Config( RCC_HCLK_Div2);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        /* Enable PLL */
        RCC_PLLCmd( ENABLE);

        /* Wait till PLL is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock configuration.
     User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}
void TIM2_init (void)
{
		TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		GPIO_InitTypeDef  GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_SetBits(GPIOB, GPIO_Pin_11);
	
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 1;
    TIMER_InitStructure.TIM_Period = 1000;
    TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM4_IRQn Interrupt */
   /* NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);*/
	
		TIM2->CCER |= TIM_CCER_CC4E;
		TIM2->CCMR2|= TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
		TIM2->CCR4 = 300;
}		

void TIM3_init (void)
{
		TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;
		
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		GPIO_InitTypeDef  GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA, GPIO_Pin_6);
	
    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler = 1;
    TIMER_InitStructure.TIM_Period = 120;
    TIM_TimeBaseInit(TIM3, &TIMER_InitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
 
    /* NVIC Configuration */
    /* Enable the TIM3_IRQn Interrupt */
   /* NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);*/
		
		TIM3->CCER |= TIM_CCER_CC1E;
		//TIM3->CCMR2|= TIM_CCMR2_OC1M_0 | TIM_CCMR2_OC1M_1 | TIM_CCMR2_OC1M_2;
		TIM3->CCR1 = 60;
}		
int main(void)
{
  Set_System();
  SetSysClockTo72();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
	ADC_DMA_init();
	TIM2_init();
	TIM3_init();
	
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
	
	sprintf((char*)Transmit_Buffer, "ADC=%d",  ADCBuffer[0]);
	
	
  while (1)
  {
		cnt_pix=0;
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		//if (STROB)
	//	GPIO_SetBits(GPIOA, GPIO_Pin_2); else
		//GPIO_ResetBits(GPIOA, GPIO_Pin_2);
    if (bDeviceState == CONFIGURED)
    {
      CDC_Receive_DATA();
      // Check to see if we have data yet
      if (Receive_length != 0) //Receive_length != 0
      {
    	  // If received symbol '1' then LED turn on, else LED turn off
    	  if (Receive_Buffer[0]=='1') {
						
						
						TIM2->CR1 |= TIM_CR1_CEN;
								while (cnt_pix<127)
								{	
									cnt_pix++;
								if (STROB==3)
								{GPIOA->ODR = 0x0004;
								Transmit_Buffer[0]=ADCBuffer[0];
								Transmit_Buffer[1]=ADCBuffer[0]>>8;
								CDC_Send_DATA ((uint8_t*)Transmit_Buffer,0x02);
								GPIOA->ODR = 0x0000;}
								}
						TIM2->CNT = 0;
						TIM2->CR1 &= ~TIM_CR1_CEN;
						STROB=0;
								
    	  }
    	  /*else {
					GPIO_SetBits(GPIOA, GPIO_Pin_2);
					if (STROB)
    		  sprintf((char*)Transmit_Buffer, "%d",  ADCBuffer[0]);
					else sprintf((char*)Transmit_Buffer, "ADC=%d",  ADCBuffer[0]);
					TIM2->CNT = 0;
					TIM2->CR1 &= ~TIM_CR1_CEN;
    	  }*/

    	  // Echo
    	  if (packet_sent == 1) {
					
    		 // CDC_Send_DATA ((uint8_t*)Transmit_Buffer,0x04);
    	  }
    	  Receive_length = 0;
      }
    }

  }
}


/*void TIM2_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
        {
            
            TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
            //GPIOA->ODR ^= GPIO_Pin_4;	 115n
						if (STROB==3)
						STROB=0; else STROB++;
						//GPIOA->ODR = 0x0010;			 //17n
						//GPIOA->ODR = 0x0000;
					
        }
}*/



