#include "stm32f4xx.h"
#include "define.h"
#include "usart.h"
#include "main.h"


unsigned char DataUsart1[9];
long DataYAW=0,TempYaw=0;
int GyroHeading,ThresholdGyro=0;

void KirimData1(unsigned char x){
		USART_SendData(USART1,x);
		while(!(USART1->SR & USART_FLAG_TXE)){}
		USART1->SR &= ~USART_FLAG_TXE;
}

void KirimData2(unsigned char x){
		USART_SendData(USART2,x);
		while(!(USART2->SR & USART_FLAG_TXE)){}
		USART2->SR &= ~USART_FLAG_TXE;
}

void KirimData3(unsigned char x){   
	//USART3->CR1 |= 0x00000000;					//disable RX interupt 
	USART3->DR = x;
	while(!(USART3->SR & USART_FLAG_TXE)){}
  USART3->SR &= ~USART_FLAG_TXE; 
	//USART3->CR1 |= 0x00000020;					//enable RX interupt
}

void SetPwm(unsigned char ID,int speed){
	//Kiri E1 Kanan E2 Belakang E3
	int xx,dataH,dataL;
	xx = speed+2000;
	dataH=(xx>>8)&0xFF;
	dataL=(xx & 0xFF);
	KirimData3(ID);
	KirimData3(dataH);
	KirimData3(dataL);
//	KirimData3((xx >> 8)&0xFF);
//	KirimData3(xx & 0xFF);
}

void SetDynamixel(unsigned char ID,unsigned char Address,unsigned int data){
	unsigned char x;
	
	KirimData2(0xFF);						//header 1	
	KirimData2(0xFF);						//header 2
	KirimData2(ID);							//ID		     
	KirimData2(5);							//Panjang Paket (length)
	KirimData2(Dxl_write_data);	//INS Write	
	KirimData2(Address);				//Address
	KirimData2(data & 0xFF);		//Data Low
	KirimData2((data>>8));			//Data high
	x=ID+5+/*3*/Dxl_write_data+Address+(data&0xFF)+(data>>8);
	KirimData2(~x);							//ceck sum
	delay_us(5);
	KirimData2(0xFF);						//header 1	
	KirimData2(0xFF);						//header 2
	KirimData2(ID);							//ID		     
	KirimData2(5);							//Panjang Paket (length)
	KirimData2(Dxl_write_data);	//INS Write	
	KirimData2(Address);				//Address
	KirimData2(data & 0xFF);		//Data Low
	KirimData2((data>>8));			//Data high
	x=ID+5+/*3*/Dxl_write_data+Address+(data&0xFF)+(data>>8);
	KirimData2(~x);							//ceck sum
	delay_us(5);
}


void USART1_IRQHandler(void){  
	static unsigned char Ucount2=0;
		
	volatile unsigned int IIR;
  IIR = USART1->SR;
  if (IIR & USART_FLAG_RXNE) {                
		USART1->SR &= ~USART_FLAG_RXNE;	          
	}		
	
	DataUsart1[Ucount2] = USART1->DR;	
		
	if(DataUsart1[0]!=0xAA){		
		DataUsart1[0]=0;
		Ucount2=0;
	}
	else{
		Ucount2++; 
		if(Ucount2==7){
			TempYaw=(DataUsart1[1]<<8 | DataUsart1[2]);
			if(TempYaw>32768){DataYAW=-1*(TempYaw-65535)/100;}
			else 						 {DataYAW=-1*TempYaw/100;}			
			Ucount2=0;
		}
	}
	GyroHeading =  (DataYAW) - ThresholdGyro;
	
	if(GyroHeading > 180) GyroHeading -= 360;
	else if(GyroHeading < -180) GyroHeading += 360;

}
