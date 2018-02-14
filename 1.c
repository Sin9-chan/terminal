#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "stm32_delay.h"
#include "stm32f10x_conf.h"
#include "string.h"

#include "stm32_delay.h"		//include _delay_us and _delay_ms
#include "UART_init.h"
#include "ADC_init.h"
#include "LCD_init.h"
#include "LCD2_init.h"
 
#define RX_BUF_SIZE 80



char RX_FLAG_END_LINE = 0;
char RXi;
char RXii;
char RXc;
char RX_BUF[RX_BUF_SIZE] = {'\0'};
char buffer[80] = {'\0'};
char RX_BUF_ADD[RX_BUF_SIZE] = {'\0'};
 

uint32_t  ADC_out1;
uint32_t  ADC_out2;
uint32_t  ADC_out1_V;
uint32_t  ADC_out2_V;
uint32_t  ADC_out1_mV;
uint32_t  ADC_out2_mV;
char RX_BUF_TXT;


 
void clear_RXBuffer(void) {
    for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
        RX_BUF[RXi] = '\0';
				RXi = 0;
}
 
 
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
            RXc = USART_ReceiveData(USART1);
            RX_BUF[RXi] = RXc;
			RX_BUF_ADD[RXi] = RXc;
            RXi++;
 
            if (RXc != 13) {
                if (RXi > RX_BUF_SIZE-1) {
                    clear_RXBuffer();
                }
            }
            else {
                RX_FLAG_END_LINE = 1;
            }
 
            //Echo
            USART_SendData(USART1, RXc);
    }
}
 
void USARTSend(  char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
 
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

		 long C1=12255;
int main(void)
{
		// Set System clock
		SystemInit();
		SetSysClockTo72();
		DelayInit();
		usart_init();
		ADC_init();
		//i2c_init();
		//LCD2_init();
		// lcd_i2c_init();
		LCD_W(" SEREGA MOLODEC {} ,. >< <> &%$#sdfsdgsgs1241515adhadhadhadhdf@");
		LCD_DEC(C1);
		_delay_ms(2000);
		LCD_CLEAR();
		_delay_ms(2000);
		LCD_ADR (0x54);
		// line 1: 0x80
		// line 2: 0x40
		// line 3: 0x94
		// line 4: 0x54
	 
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
		/* Initialize LED which connected to PC13 */
		GPIO_InitTypeDef  GPIO_InitStructure;
		// Enable PORTC Clock
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
		/* Configure the GPIO_LED pin */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Set C13 to Low level ("0")

		// Initialize USART
		usart_init();
		USARTSend(" Hello.\r\nUSART1 is ready.\r\n");
 
    while (1)
    {		//RX_BUF_TXT=RX_BUF;
        if (RX_FLAG_END_LINE == 1) {
            // Reset END_LINE Flag
            RX_FLAG_END_LINE = 0;
 
            USARTSend("\r\nI has received a line:\r\n");
            USARTSend(RX_BUF);
            USARTSend("\r\n");
 
			if (strncmp(RX_BUF, "ON\r", 2) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"ON\"!!!\r\n");
				GPIO_ResetBits(GPIOC, GPIO_Pin_13);			
				}
			if (strncmp(RX_BUF, "OFF\r", 3) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"OFF\"!!!\r\n");
				GPIO_SetBits(GPIOC, GPIO_Pin_13);
				}
			if (strncmp(RX_BUF, "LCD_CLEAR\r", 8) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"LCD clear\"!!!\r\n");
				LCD_CLEAR();
			}
			if (strncmp(RX_BUF, "PRINT CHAR", 10) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"PRINT\"!!!\r\n");
				LCD_CLEAR();
				for(RXii=0;RXii<RX_BUF_SIZE-11;RXii++)
				{
					buffer[RXii]=RX_BUF_ADD[RXii+11];
				}
				USARTSend(buffer);
				LCD_W("You wrote: ");
				LCD_W(buffer);
				
            }
			if (strncmp(RX_BUF, "PRINT NUM", 9) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"PRINT\"!!!\r\n");
				LCD_CLEAR();
				for(RXii=0;RXii<RX_BUF_SIZE-10;RXii++)
				{
					buffer[RXii]=RX_BUF_ADD[RXii+10];
				}
				USARTSend(buffer);
				long int num;
				sscanf(buffer, "%ld", &num);
				LCD_W("You wrote: ");
				LCD_DEC(num);
				
            }
			if (strncmp(RX_BUF, "BATA_V_HATE\r", 8) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"BATA_V_HATE\"!!!\r\n");
				LCD_CLEAR();
				ADC_out1=(int)ADC_out1/10;
				ADC_out2=(int)ADC_out2/10;
				ADC_out1_V=ADC_out1/1000;
				ADC_out2_V=ADC_out2/1000;
				ADC_out1_mV=1000+555;
				ADC_out2_mV=1000+555;
				LCD_W("VOLTAGE     CURRENT ");
				LCD_W("                     ");
				LCD_DEC(ADC_out1_mV);
				LCD_W("       ");
				LCD_DEC(RXc);
				for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
				{LCD_DEC(RX_BUF[RXi]);}
				}
			if (strncmp(RX_BUF, "lcd_i2c_clear\r", 10) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"lcd_i2c_clear\"!!!\r\n");
				lcd_i2c_clear();
			}
			if (strncmp(RX_BUF, "ADC\r", 8) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"ADC\"!!!\r\n");
				lcd_i2c_clear();
				lcd_i2c_w(RX_BUF);
			}
			if (strncmp(RX_BUF, "TXT\r", 8) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"TXT\"!!!\r\n");
				lcd_i2c_clear();
				lcd_i2c_w(RX_BUF);
			}

			if (strncmp(RX_BUF, "lcd_i2c_adc\r", 10) == 0) {
				USARTSend("\r\nTHIS IS A COMMAND \"lcd_i2c_adc\"!!!\r\n");

				ADC_out1=(int)ADC_out1/10;
				ADC_out2=(int)ADC_out2/10;
				ADC_out1_V=ADC_out1/1000;
				ADC_out2_V=ADC_out2/1000;
				ADC_out1_mV=ADC_out1%1000+555;
				ADC_out2_mV=ADC_out2%1000+555;
				lcd_i2c_command(0x02);
				lcd_i2c_w("VOLTAGE  CURRENT");
				lcd_i2c_adr(0x40);
				lcd_i2c_dec(ADC_out1_V);
				lcd_i2c_w(",");

				if (ADC_out1_mV<10) 
				{lcd_i2c_w("00");
				lcd_i2c_dec(ADC_out1_mV);
				} 
				if ((ADC_out1_mV<100)&(ADC_out1_mV>9))
				{lcd_i2c_w("0");
				lcd_i2c_dec(ADC_out1_mV);
				} 
				if	(ADC_out1_mV>99)	{lcd_i2c_dec(ADC_out1_mV);}
				lcd_i2c_w("kV");
				lcd_i2c_w("  ");
				lcd_i2c_dec(ADC_out2_V);
				lcd_i2c_w(",");

				if (ADC_out2_mV<10) 
				{lcd_i2c_w("00");
				lcd_i2c_dec(ADC_out2_mV);
				} 
				if ((ADC_out2_mV<100)&(ADC_out2_mV>9))
				{lcd_i2c_w("0");
				lcd_i2c_dec(ADC_out2_mV);
				} 
				if	(ADC_out2_mV>99)	lcd_i2c_dec(ADC_out2_mV);
				//ADC1->CR2=0x0007;
				lcd_i2c_w("A ");
				ADC_out1=0;
				ADC_out2=0;
				//Averaging
				for (int i=0;i<10;i++)			
				{	

				ADC1->CR2=0x00000001;									//mesure start ADC1
				ADC2->CR2=0x00000001;									//mesure start ADC2
				_delay_ms(1);
				ADC_out1= ADC_out1+ADC_mV(ADC1->DR);
				ADC_out2= ADC_out2+ADC_mV(ADC2->DR);
				}//end Averaging
				}
 
            clear_RXBuffer();
        }
    }
}

