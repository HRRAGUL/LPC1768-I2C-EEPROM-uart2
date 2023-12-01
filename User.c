#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <RTL.h>
#include <Net_Config.h>
#include <LPC17xx.h>                    /* LPC17xx definitions               */

#include <core_cm3.h>
//#define  FOSC    12000000
//#define  FCCLK   (FOSC*8)
//#define  FPCLK   (FCCLK/4)
#include "User.h"
#include "HTTP.h"
#include "Client.h"
#include "Server.h"
#include "Announce.h"
#include "I2C_EEPROM.h"
#include "USER_EEPROM.h"
#include "HTTP_CGI.h"
#include "Iiot.h"
#include "RTC.h"
#include "Modbus_Server.h"
#include "UART.h"
#include "ADC.h"
#include "GPIO.h"
#include "SPI.h"
#include "TIMER.h" 
#include "SSP.h"
#include "TDSXGL.h"	
#include "TDSXGA.h"

#define BIT(n) 1<<n
#define TRUE 1
#define FALSE 0
uint8_t GC_RecieveBuff[GK_RECEIVE_LENGTH]={0},GC_RecieveBuff1[GK_RECEIVE_LENGTH]={0},GC_RX_Flag=0,GC_RX_Flag1=0;
//static uint8_t  GC_ArrayPutPtr=0;//GC_ArrayPutPtr1=0;
extern unsigned char GC_IECEnableFlag_GA,GC_IECEnableFlag_GL,GC_Ticker30Sec_Flag;
extern uint64_t GL_Ticker30Sec;
uint8_t G_TxBuff[40]={0}; 
uint8_t G_TxBuff1[40]={0}; 
uint16_t AXE410_ModRtuCRC(uint8_t *,uint16_t);
static uint32_t GLI_Pclk,GLI_Fdiv;
 uint8_t Byte=0,i=0;
unsigned char bb=0;
char a[]="\n\rApplied Embedded";
uint16_t sec,min,hou,day,mon,year;
unsigned char str3[150];
char on1=1,off1=1, on2=1,off2=1;
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AXE410_I2C0Initiation(void)
{
	LPC_SC->PCONP |= BIT(7);	// Powerup I2C0
	LPC_SC->PCLKSEL0 |= (0x01<<14);	// Set PCLK to I2C0 block
	
	/* set PIO0.27 and PIO0.28 to I2C0 SDA and SCL */
  /* function to 01 on both SDA and SCL. */
  LPC_PINCON->PINSEL1 &= ~((0x03<<22)|(0x03<<24));
  LPC_PINCON->PINSEL1 |= ((0x01<<22)|(0x01<<24));
	
  /*--- Clear flags ---*/	
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC | GK_I2C_CONCLR_I2ENC;
  LPC_I2C0->I2CONSET = GK_I2C_CONSET_I2EN;	// Set I2EN
	
	LPC_I2C0->I2SCLL = 125;	// PClk is 100 Mhz. To get 400Khz 100Mhz/250. This 250 is shared by two registers
	LPC_I2C0->I2SCLH = 125;
}
uint8_t AXE410_I2CWaitStatus(uint8_t u8Status)
{
	uint32_t LLI_WaitCount = 0;
	while(LLI_WaitCount < GK_MAX_TIMEOUT)
	{
		if(LPC_I2C0->I2CONSET & 8)
		{
			if(LPC_I2C0->I2STAT == u8Status)
			{
				 LLI_WaitCount = 0;
				return TRUE;
			}
		}
		 LLI_WaitCount++;
	}
	return FALSE;
}

uint8_t AXE410_I2CByteWriteEEPROM(uint16_t Address,uint8_t Data)
{
	// Address set
	AXE410_I2CSetAddressEEPROM(Address);	
	
	// Data write
	LPC_I2C0->I2DAT = Data;
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x28))	// Data has been transmitted, Ack has been received
		return FALSE;
	
	// I2C stop
	AXE410_I2C0Stop();
	
	return TRUE;
}
 
uint8_t AXE410_I2CByteReadEEPROM(uint16_t Address)
{
	uint8_t LC_Data;
	if(!AXE410_I2CSetAddressEEPROM(Address))
		return FALSE;
	
	if(!AXE410_I2C0Restart())
		return FALSE;
		
	LPC_I2C0->I2DAT = GK_I2C_EEPROM_DEVICE_ADDR|0x01;
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x40))	// Ready for data byte
		return FALSE;
	
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x58))	// data byte received and NACK returned
		return FALSE;		
	LC_Data = (uint8_t)LPC_I2C0->I2DAT;
	AXE410_I2C0Stop();	// Stop I2C Communication
	
	return LC_Data;
}

uint8_t AXE410_I2CSetAddressEEPROM(uint16_t u16StartAddr)
{	
	AXE410_I2CStart(0);	// Set I2C start
	if(!AXE410_I2CWaitStatus(0x08))	// Ready for device addr
		return FALSE;
			
	LPC_I2C0->I2DAT = 0xA0;	// Slave addr
	LPC_I2C0->I2CONCLR = 0x2C;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x18))	// ready for data byte
		return FALSE;
			
	// Transmit start address 1st byte
	LPC_I2C0->I2DAT = ((u16StartAddr & 0xFF00)>>8)&0xFF;
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x28))	// Data has been transmitted, Ack has been received
		return FALSE;
		
	// Transmit start address 2nd byte
	LPC_I2C0->I2DAT = (u16StartAddr & 0x00FF);
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
	if(!AXE410_I2CWaitStatus(0x28))	// Data has been transmitted, Ack has been received
		return FALSE;
	
	return TRUE;
}
void AXE410_I2CStart(uint8_t PortNo)
{
	uint32_t LLI_Delay;
	LPC_I2C0->I2CONSET = GK_I2C_CONSET_STA;
	while(!(LPC_I2C0->I2CONSET & 0x20))
	{
		if(LLI_Delay < GK_MAX_TIMEOUT)
		{
			LLI_Delay++;
		}
		else 
			break;
	}
}
void AXE410_I2C0Stop(void)
{
	LPC_I2C0->I2CONSET = GK_I2C_CONSET_STO;	// Set I2C Stop 
  LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_AAC | GK_I2C_CONCLR_SIC | GK_I2C_CONCLR_STAC;	// Clear all except I2EN
}

uint8_t AXE410_I2C0Restart(void)
{	
	LPC_I2C0->I2CONCLR = GK_I2C_CONCLR_SIC;	// Clear I2C SI
	LPC_I2C0->I2CONSET = GK_I2C_CONSET_STO;	// Set I2C Stop
		
	// Read Data Bytes 
	LPC_I2C0->I2CONSET = GK_I2C_CONSET_STA;	// Start I2C
	if(!AXE410_I2CWaitStatus(0x08))	// Ready for device addr
		return FALSE;
	return TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AXE410_UART2Init(void)
{
	
		LPC_SC->PCONP 		|= 1<<24;	// Set UART1 Power Control Bit
	  LPC_SC->PCLKSEL0 		|= (0x01<<16);	// Set Uart clock is Pclk/4
	//	LPC_PINCON->PINSEL1 &= ~(0xFF << 18); // Clear bits 18-25
   // LPC_PINCON->PINSEL1 |= (0x10 << 18); 
	  LPC_PINCON->PINSEL0 |= (0x00500000);	// Set P0.0 as a TXD3,P0.1 as RXD3
	  LPC_PINCON->PINMODE0|= (0x00<<11);	// P0.0,P0.1 pin pull up is enabled
		LPC_PINCON->PINMODE0|= (0x00<<10);
	  LPC_GPIO0->FIODIR 	|= (1<<10);	// Set P0.0 pin as output,P0.1 pin as input
		LPC_GPIO0->FIODIR 	|= (0<<11);
	//  LPC_GPIO0->FIOCLR    = (0xFFFFFFFF);	// Clear output states
		LPC_GPIO0->FIOCLR	= (0xFFFFFFFF);	// Clear output states
	// UART control registers
	//0  LPC_UART3->FCR = 0x07;	// Uart access enabled. Tx,Rx FIFO got reset.
	  LPC_UART2->LCR = 0x83;	// 8data bits,1stop bit,DLA enabled
	  GLI_Pclk = SystemCoreClock/4;	// SystemCoreClock division value depands on PCLKSEL register value
	  GLI_Fdiv = (GLI_Pclk/16)/9600;
	  LPC_UART2->DLM = GLI_Fdiv/256;
	  LPC_UART2->DLL = GLI_Fdiv%256;
	
	  LPC_UART2->LCR = 0x03;	// Disable DLA
	  LPC_UART2->THR = 0x00;
	  LPC_UART2->FCR = 0x07;
	  NVIC_EnableIRQ(UART2_IRQn);
	  NVIC_SetPriority(UART2_IRQn,1);
	  LPC_UART2->IER = 1;	// Set UART RX interrupt*/
		
}



void AXE410_Uart2Tx(uint8_t Value)
{
	while(!(LPC_UART2->LSR & 0x20));	// wait until TSR get empty
	LPC_UART2->THR = Value;	// Fill Transmit Holding Register with out data
}


//////////////////////////////////////////////////////////
void UART2_IRQHandler(void)
{ // uint32_t LLI_i;	
	while(LPC_UART2->LSR&0x01)	// Wait until data gets receive
	{
		Byte=LPC_UART2->RBR;
	 
		
}
}



int main(void)
{
  AXE410_UserIoInit();
	AXE410_UART2Init();
  AXE410_I2C0Initiation();
  bb=	AXE410_I2CByteReadEEPROM(5);
  uint32_t LLI_i;
  for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);	
  AXE410_Uart2Tx(bb);
	for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);	
 while (1) 
{
		 
	for(i=0x41;i<0x5B;i++)
  {
   AXE410_I2CByteWriteEEPROM(5,i);
	uint32_t LLI_i;
  for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);	
  AXE410_Uart2Tx(i);
	for(LLI_i=0;LLI_i<5000000;LLI_i++);for(LLI_i=0;LLI_i<5000000;LLI_i++);	
	LPC_GPIO2->FIOSET = GK_USER_LED4_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
	LPC_GPIO2->FIOCLR = GK_USER_LED4_PIN;
	for(LLI_i=0;LLI_i<5000000;LLI_i++);
	}
	
}
}
