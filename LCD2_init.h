#include "stm32f10x.h"
#include "stm32_delay.h"
#include "string.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_conf.h"

// PORT A0...A7 DATA
// PORT B12-E; B13-RW; B14-RS;
//

	void LCD_DATA(char DATA);
	void LCD_W(char word[]);
	
void LCD2_init(void)	//init I2C
{	

GPIO_InitTypeDef gpio;
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//enable clock on port B (for SDA, SCL)
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	//enable clock on port B (for SDA, SCL)
	
	gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4 | GPIO_Pin_3 | GPIO_Pin_2 | GPIO_Pin_1 | GPIO_Pin_0 ;	// DATA LCD 
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;			//Alternate function open drain
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);								//Initialize port A with this settings

	gpio.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;	// PORT B12-E; B13-RW; B14-RS;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;			
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);								//Initialize port B with this settings

	_delay_ms(15);
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_Write(GPIOA, 0x48);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		_delay_ms(5);

	GPIO_Write(GPIOA, 0x48);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(500);

	GPIO_Write(GPIOA, 0x52);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(500);
	
	GPIO_Write(GPIOA, 0x08);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(500);
	

	GPIO_Write(GPIOA, 0x0F);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(200);
	
		GPIO_Write(GPIOA, 0x01);
	_delay_us(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_ms(5);
	
		GPIO_Write(GPIOA, 0x3C);
	_delay_us(80);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(10);
	
	

	LCD_W(" SUN HUN CHEN         TEHNOLODGY");
	_delay_ms(4000);
	
	GPIO_Write(GPIOA, 0x01);
	_delay_us(80);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_ms(5);
	
	
}
	void LCD_DATA(char DATA)					//funkcia zagruzki adresa simvola
	{
		GPIO_Write(GPIOA, DATA);
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
	_delay_us(80);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		
	}
	
	void LCD_ADR(char ADR)					//funkcia zagruzki adresa kursora
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_Write(GPIOA, ADR);
	_delay_us(80);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_us(20);
	
	// line 1: 0x80
	// line 2: 0x40
	// line 3: 0x94
	// line 4: 0x54
}
	void LCD_CLEAR(void)	   				//funkcia ochistki LCD
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_Write(GPIOA, 0x01);
	_delay_us(80);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	_delay_ms(2);
}
	void LCD_W(char word[])					//funkcia zagruzki simvolov
{
int i;	//letter counter
int j;
i=strlen(word);	//kolvo simvolov

for (j=0;j<i;j++)
	{
	
	LCD_DATA(word[j]);
	}

}
void LCD_DEC(long cel)						//funkcia vivoda dec chisel
{
char wor[33];		//array for number word (max 33 letters)
char row[33];		//redirected word array
char a=0x30;		//address of 0
char ost=cel%10;	//ostatok from 1st divis
int j=0;
wor[j]=ost;	//word[0]=ost=4
while (cel>=10){	//1. 32>10	2.return
ost=cel%10;		//1. ost=2
cel=cel/10;		//1. cel=3
wor[j]=ost;	//word[1]=ost=2
j++;			//j=2
}//end while
if(j>0) wor[j]=cel;	//word[2]=cel=3
for (int i=0;i<j+1;i++){

char buf=wor[i];
row[j-i]=buf;	//redirect array
}//end for
for (int i=0;i<j+1;i++){
row[i] = row[i]+a;	//get symbol address
LCD_DATA(row[i]);
}//end for
}	
