/*
0x05 0x04 0x00 0x01 0x00 0x01 0x61 0x8E
MODBUS RTU slave (USART)
04 READ INPUT REGISTER
https://www.modbustools.com/modbus.html#function04

Step 7.
- USART2 interrupt, ok
- wrong slave address, ok
- read 7 bytes from usartx, ok
- crc check, ok
- reads sensor, ok
- gives resonse to master, ok
- Reads ADC and sends data to master
- USART1
*/

/* Includes */

#include "stm32l1xx.h"
#define HSI_VALUE    ((uint32_t)16000000)
#include "nucleo152start.h"
#include <stdio.h>
#include <stdlib.h>

#define SLAVE_ADDRESS 0x05
#define INPUT_REGISTER 0x01

/* Private typedef */
/* Private define  */
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
void delay_Ms(int delay);
void USART1_write(char data);
void USART2_write(char data);
void USART1_Init(void);
void USART2_Init(void);
void timer11_init(void);
void wrong_slave_address(void);
char USART1_read();
char USART2_read();
unsigned short int CRC16(char *nData,unsigned short int wLength);
void read_7_bytes_from_usartx(char *received_frame);
int read_sensor(int input_address);
void respond_frame(int sensor_value);
char mFlag=0;

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
int main(void)
{
	__disable_irq();			//global disable IRQs, M3_Generic_User_Guide p135.
	USART1_Init();
	USART2_Init();

	/* Configure the system clock to 32 MHz and update SystemCoreClock */
	SetSysClock();
	SystemCoreClockUpdate();

	/* TODO - Add your application code here */

	USART2->CR1 |= 0x0020;			//enable RX interrupt
	NVIC_EnableIRQ(USART2_IRQn); 	//enable interrupt in NVIC
	__enable_irq();					//global enable IRQs, M3_Generic_User_Guide p135

	RCC->AHBENR|=1; 				//GPIOA ABH bus clock ON. p154
	GPIOA->MODER&=~0x00000C00;		//clear (input reset state for PA5). p184
	GPIOA->MODER|=0x400; 			//GPIOA pin 5 to output. p184
	GPIOA->MODER|=0x3;			//PA0 analog (A0)

	GPIOA->MODER&=~0x00003000;		//clear (input reset state for PA6). p184
	GPIOA->MODER|=0x1000;			//GPIOA pin 6 to output. p184

	  //setup ADC1. p272
	  RCC->APB2ENR|=0x00000200;		//enable ADC1 clock
	  ADC1->SQR5=0;					//conversion sequence starts at ch0
	  ADC1->CR2=0;					//bit 1=0: Single conversion mode, bit 11=0 align right
	  ADC1->SMPR3=7;				//384 cycles sampling time for channel 0 (longest)
	  ADC1->CR1&=~0x03000000;		//resolution 12-bit
	  ADC1->CR2|=1;					//bit 0, ADC on/off (1=on, 0=off)


	char received_frame[8]={0};
  /* Infinite loop */
	unsigned short int crc=0; //16 bits
	char crc_high_byte=0;
	char crc_low_byte=0;
	char buf[100];

	while (1)
  {

	   if(mFlag==1) //correct slave address
	   {
		   read_7_bytes_from_usartx(&received_frame[1]);
		   received_frame[0]=SLAVE_ADDRESS;
		   crc=CRC16(received_frame,6);
		   crc_high_byte=crc>>8|crc_high_byte; //high byte
		   crc_low_byte=crc|crc_low_byte; //low byte


		   if((received_frame[7]==crc_high_byte)&&(received_frame[6]==crc_low_byte)) //CRC check
		   {
			   if(received_frame[3]==INPUT_REGISTER) //input register check and can be added more registers
			   {
				   int sensor_value=read_sensor(INPUT_REGISTER);
				   respond_frame(sensor_value); //no multiplication by x10 because ADC value quite big 300-1000
			   }
		   }
		   else
		   {
		   }
		   crc=0;
		   crc_low_byte=0;
		   crc_high_byte=0;
		   mFlag=0;
		   //GPIOA->ODR|=0x20;				//0010 0000 or bit 5. p186
		   USART2->CR1 |= 0x0020;			//enable RX interrupt
	   }
	   else if(mFlag==2) //wrong slave address
	   {
		   wrong_slave_address();
	   }
  }
  return 0;
}

void delay_Ms(int delay)
{
	int i=0;
	for(; delay>0;delay--)
		for(i=0;i<2460;i++); 	//measured with oscilloscope
}

void USART1_Init(void)
{
	RCC->APB2ENR|=(1<<14);	 	//set bit 14 (USART1 EN) p.156
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[1]=0x00000700;	//GPIOx_AFRL p.189,AF7 p.177
	GPIOA->AFR[1]|=0x00000070;	//GPIOx_AFRL p.189,AF7 p.177
	GPIOA->MODER|=0x00080000; 	//MODER2=PA9(TX)D8 to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00200000; 	//MODER2=PA10(RX)D2 to mode 10=alternate function mode. p184

	USART1->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART1->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART1->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART1->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART2_Init(void)
{
	RCC->APB1ENR|=0x00020000; 	//set bit 17 (USART2 EN)
	RCC->AHBENR|=0x00000001; 	//enable GPIOA port clock bit 0 (GPIOA EN)
	GPIOA->AFR[0]=0x00000700;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->AFR[0]|=0x00007000;	//GPIOx_AFRL p.188,AF7 p.177
	GPIOA->MODER|=0x00000020; 	//MODER2=PA2(TX) to mode 10=alternate function mode. p184
	GPIOA->MODER|=0x00000080; 	//MODER2=PA3(RX) to mode 10=alternate function mode. p184

	USART2->BRR = 0x00000D05;	//9600 BAUD and crystal 32MHz. p710, D05
	USART2->CR1 = 0x00000008;	//TE bit. p739-740. Enable transmit
	USART2->CR1 |= 0x00000004;	//RE bit. p739-740. Enable receiver
	USART2->CR1 |= 0x00002000;	//UE bit. p739-740. Uart enable
}

void USART1_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART1->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART1->DR=(data);			//p739
}

void USART2_write(char data)
{
	//wait while TX buffer is empty
	while(!(USART2->SR&0x0080)){} 	//TXE: Transmit data register empty. p736-737
		USART2->DR=(data);			//p739
}

void USART2_IRQHandler(void)
{
	char received_slave_address=0;

	if(USART2->SR & 0x0020) 		//if data available in DR register. p737
	{
		received_slave_address=USART2->DR;
	}
	if(received_slave_address==SLAVE_ADDRESS) //if we have right address
	{
		mFlag=1;

	}
	else
	{
		mFlag=2;


	}
	USART2->CR1 &= ~0x0020;			//disable RX interrupt

}

unsigned short int CRC16 (char *nData,unsigned short int wLength)
{
static const unsigned short int wCRCTable[] = {
0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 };

unsigned char nTemp;
unsigned short int wCRCWord = 0xFFFF;

   while (wLength--)
   {
      nTemp = *nData++ ^ wCRCWord;
      wCRCWord >>= 8;
      wCRCWord ^= wCRCTable[nTemp];
   }
   return wCRCWord;

}

char USART1_read()
{
	char data=0;
									//wait while RX buffer is data is ready to be read
	while(!(USART1->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART1->DR;			//p739
		return data;
}

char USART2_read()
{
	char data=0;
									//wait while RX buffer is data is ready to be read
	while(!(USART2->SR&0x0020)){} 	//Bit 5 RXNE: Read data register not empty
		data=USART2->DR;			//p739
		return data;
}

void wrong_slave_address(void)
{
	   USART2->CR1 &= ~0x00000004;		//RE bit. p739-740. Disable receiver
	   delay_Ms(10); 					//time=1/9600 x 10 bits x 7 byte = 7,29 ms
	   USART2->CR1 |= 0x00000004;		//RE bit. p739-740. Enable receiver
	   USART2->CR1 |= 0x0020;			//enable RX interrupt
	   mFlag=0;
}


void read_7_bytes_from_usartx(char *received_frame)
{
	char frame[8]={0};
	char i=0;

	   while(i<7)
	   {
	   *received_frame=USART2_read();
	   frame[i]=*received_frame;
	   received_frame++;
	   i++;
	   }
/*
	int i=0;

		for(i = 0; i < 7; i++)
		{
			*(received_frame+i) = USART2_read();
		}
*/
}

int read_sensor(int input_address)
{
//	char buf[100];
//	  int result=0;
		  int result=0;
		//if(input_address==0x01)
		  ADC1->CR2|=0x40000000;	//start conversion
		  while(!(ADC1->SR & 2)){}	//wait for conversion complete
		  result=ADC1->DR;			//read conversion result

//			result=0.0001*sensor_value*sensor_value-0.0749*result+2.4828;

//		  int len=0;
//		  while(buf[len]!='\0')
//		  len++;
//
//			for(int i=0;i<len;i++)
//			{
//				USART2_write(buf[i]);
//			}
//
//			USART2_write('\n');
//			USART2_write('\r');

			delay_Ms(1000);
			return result;
}

void respond_frame(int sensor_value)
{
	GPIOA->ODR|=0x20;				//led on, transmitting mode
	//example response should be like this: 0104020B057E03
	char respond_frame[7]={SLAVE_ADDRESS,0x04,0x02,0,0,0,0};
	char sensor_high_bits=0;
	char sensor_low_bits=0;
	char crc_high_byte=0;
	char crc_low_byte=0;
	unsigned short int crc=0; //16 bits

	sensor_high_bits=(sensor_value>>8)|sensor_high_bits;
	sensor_low_bits=sensor_value|sensor_low_bits;
	respond_frame[3]=sensor_high_bits;
	respond_frame[4]=sensor_low_bits;
	crc=CRC16(respond_frame,5);
	crc_high_byte=(crc>>8)|crc_high_byte; //high byte
	crc_low_byte=crc|crc_low_byte; //low byte

	respond_frame[6]=crc_high_byte;
	respond_frame[5]=crc_low_byte;

	for(int i=0;i<7;i++)
	{
		USART2_write(respond_frame[i]);
	}
	GPIOA->ODR&=~0x20;				//led off, receiving mode

}
