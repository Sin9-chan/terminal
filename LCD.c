void i2c_init(void)	//init I2C
{
	//Structures rename
	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef  i2c;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);		//ENABLE clock on I2C1 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	//enable clock on port B (for SDA, SCL)
	
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	//Choose 6,7 pins for SCL and SDA
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;			//Alternate function open drain
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);			//Initialize port B with this settings
	
	i2c.I2C_ClockSpeed=400000;			//Speed 400kHz
	i2c.I2C_Mode = I2C_Mode_I2C;		//I2C in I2c mode (not SMBus)
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;	//2 or 16/9
	i2c.I2C_OwnAddress1 = i2c_adr_w;				//RANDOM
	i2c.I2C_Ack = I2C_Ack_Enable;			//Enable Ack
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;	//7 or 10- bit address
	//Enable I2C
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &i2c);					//Initialize I2C with this settings

}

//Send command to PCF8574
void lcd_i2c_command(char lcd_command)
{
	while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until line is not busy
	I2C_GenerateSTART(I2C1,ENABLE);								//Generate START														
		
	while(!(I2C1->SR2 & I2C_SR2_MSL)){}	//wait when MSL=1 (the same as SB=1)					EV5
		
(void) I2C1->SR1;				//READ SR1	
I2C1->DR = i2c_adr_w;		//send address (SB should be = 0)

	while(!(I2C1->SR1 & I2C_SR1_ADDR))				//wait until ADDR=1											EV6
{if(I2C1->SR1 & I2C_SR1_AF) GPIOC->ODR|=0x2000;	}		//if AF flag is set? set PC13

(void) I2C1->SR1;				//READ SR1
(void) I2C1->SR2;				//READ SR2		to clear ADDR flag
		
	while(!(I2C1->SR1 & I2C_SR1_TXE))	{}			//wait until TxE = 1										EV8_1
		I2C1->DR = (shhbyte(lcd_command))|(0x0C);	//send high half-byte of data to TWDR, E_/, BCKL on
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (shhbyte(lcd_command))|(0x08);	//send high half-byte of data to TWDR, E\_ BCKL on
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (slhbyte(lcd_command))|(0x0C);	//send low half-byte of data to TWDR, E_/ BCKL on
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (slhbyte(lcd_command))|(0x08);	//send low half-byte of data to TWDR, E\_ BCKL on
		
	while(!(I2C1->SR1 & I2C_SR1_BTF)) {}		//wait until BTF = 1  (byte transmitted, the same as TxE)		EV8_2
		
	I2C_GenerateSTOP(I2C1,ENABLE);					//Generate STOP
	while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until BUSY = 0

_delay_ms(1);
}

//Send data to PCF8574
void lcd_i2c_data(char lcd_data)
{
	while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until line is not busy
	I2C_GenerateSTART(I2C1,ENABLE);								//Generate START														
		
	while(!(I2C1->SR2 & I2C_SR2_MSL)){}	//wait when MSL=1 (the same as SB=1)					EV5
		
(void) I2C1->SR1;				//READ SR1	
I2C1->DR = i2c_adr_w;		//send address (SB should be = 0)

	while(!(I2C1->SR1 & I2C_SR1_ADDR))				//wait until ADDR=1											EV6
{if(I2C1->SR1 & I2C_SR1_AF) GPIOC->ODR|=0x2000;	}		//if AF flag is set? set PC13

(void) I2C1->SR1;				//READ SR1
(void) I2C1->SR2;				//READ SR2		to clear ADDR flag
		
	while(!(I2C1->SR1 & I2C_SR1_TXE))	{}			//wait until TxE = 1										EV8_1
		I2C1->DR = (shhbyte(lcd_data))|(0x0D);	////high half-byte to TWDR, E_/, BCKL on, RS=1
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (shhbyte(lcd_data))|(0x09);	//send high half-byte of data to TWDR, E\_ BCKL on RS=1
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (slhbyte(lcd_data))|(0x0D);	//send low half-byte of data to TWDR, E_/ BCKL on RS=1
		
	while(!(I2C1->SR1 & I2C_SR1_TXE)) {}		//wait until TXE = 1 
		I2C1->DR = (slhbyte(lcd_data))|(0x09);	//send low half-byte of data to TWDR, E\_ BCKL on RS=1
		
	while(!(I2C1->SR1 & I2C_SR1_BTF)) {}		//wait until BTF = 1  (byte transmitted, the same as TxE)		EV8_2
		
	I2C_GenerateSTOP(I2C1,ENABLE);					//Generate STOP
	while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until BUSY = 0

//_delay_us(100);
}
//set cursor to address
void lcd_i2c_adr(char lcd_adr)
{
	lcd_i2c_command(lcd_adr | 0x80);
}

//Clear screen
void lcd_i2c_clear(void)
{
	lcd_i2c_command(0x01);
	_delay_ms(1);
}
//Write a word to the lcd
void lcd_i2c_w(char word[])
{
int i;	//letter counter
int j;
i=strlen(word);

for (j=0;j<i;j++){
	
	lcd_i2c_data(word[j]);
	if (j==15)			//if number of letters >16
	lcd_i2c_command(0xC0);	//to the new line
	}
	
}
//Write a number to i2c_lcd in dec 
void lcd_i2c_dec(long cel)
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
lcd_i2c_data(row[i]);
}//end for
}	

//write number in hex format
void lcd_i2c_hex(long cel)
{
char wor[33];		//array for number word (max 33 letters)
char row[33];		//redirected word array
char O=0x30;		//address of 0
char A=0x37;		//address of A
char ost=cel%16;	//ostatok from 1st divis
int j=0;			//counter for number of divisions
wor[j]=ost;	
while (cel>=16){	
ost=cel%16;		
cel=cel/16;		
wor[j]=ost;	
j++;			
}//end while
if(j>0) wor[j]=cel;	//if more than 1 division occured
for (int i=0;i<j+1;i++){

char buf=wor[i];	//buffer for array redirection
row[j-i]=buf;	//redirect array
}//end for
for (int i=0;i<j+1;i++){

if (row[i]<10) row[i] = row[i]+O;	//get symbol address 0..9
else row[i] = row[i]+A;			//get symbol address A..F
lcd_i2c_data(row[i]);
}//end for
}	

//Init LCD I2C
void lcd_i2c_init(void)
{
	_delay_ms(1);	//wait 20 ms
	
		while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until line is not busy
	I2C_GenerateSTART(I2C1,ENABLE);								//Generate START														
		
	while(!(I2C1->SR2 & I2C_SR2_MSL)){}	//wait when MSL=1 (the same as SB=1)					EV5
		
(void) I2C1->SR1;				//READ SR1	
I2C1->DR = i2c_adr_w;		//send address (SB should be = 0)
																																	
	while(!(I2C1->SR1 & I2C_SR1_ADDR))				//wait until ADDR=1											EV6
{if(I2C1->SR1 & I2C_SR1_AF) GPIOC->ODR|=0x2000;	}		//if AF flag is set? set PC13
																																	
(void) I2C1->SR1;				//READ SR1
(void) I2C1->SR2;				//READ SR2		to clear ADDR flag
			
	while(!(I2C1->SR1 & I2C_SR1_TXE))	{}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30|(0x04));							//send 0011 to TWDR, E_/	
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30&(~0x04));								//send 0011 to TWDR, E\_
	_delay_ms(2);	//wait 20 ms
																																	
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30|(0x04));							//send 0011 to TWDR, E_/
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30&(~0x04));							//send 0011 to TWDR, E\_
	_delay_ms(2);	//wait 20 ms
																																	
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30|(0x04));							//send 0011 to TWDR, E_/
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x30&(~0x04));								//send 0011 to TWDR, E\_
					
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x20|(0x04));								//send 0010 to TWDR, E_/
	while(!(I2C1->SR1 & (I2C_SR1_TXE))){}		//Wait until TxE = 1 (Data reg empty)
	I2C1->DR =(0x20&(~0x04));								//send 0010 to TWDR, E\_
																																	
	while(!(I2C1->SR1 & (I2C_SR1_BTF))){}		//Wait until BTF = 1 End of transmission
	I2C_GenerateSTOP(I2C1,ENABLE);					//Generate STOP
	while(I2C1->SR2 & I2C_SR2_BUSY){}			//wait until BUSY = 0
																																	
		
	lcd_i2c_command(0x28);	//4-bit, 2 lines, 5x8 font
	lcd_i2c_command(0x08);	//disp off, cursor off
	lcd_i2c_command(0x01);	//clear disp
	lcd_i2c_command(0x06);	//right move 
	lcd_i2c_command(0x02);	//ret home
	lcd_i2c_command(0x0F);	//disp on
	lcd_i2c_adr(0x40);			//drop line

}
