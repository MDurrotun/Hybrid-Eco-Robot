#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

GPIO_InitTypeDef	GPIO_InitStructure;
USART_InitTypeDef	USART_InitStructure;
NVIC_InitTypeDef	NVIC_InitStrcuture;
TIM_TimeBaseInitTypeDef	TIM_InitStructure;
TIM_OCInitTypeDef	TIM_OCInitStructure;
TIM_ICInitTypeDef	TIM_ICInitStructure;
ADC_InitTypeDef		ADC_InitStructure;
DMA_InitTypeDef	DMA_InitStructure;

uint8_t Tx3Buffer[24];

void KonfigurasiGPIO(){
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
		
		//-------------------------Konfigurasi LCD----------------------------
		GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4 | GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		
}

void AllGPIO(){	
		/*
		INPUT
						LIMIT
							D14  D15  A8
						BUTTON
							E12  E8  E7  B0  B1  C5
						PROXYMITY
							B13  C2
						US ECHO
							E2  E5  C15
			
		OUTPUT
						US TRIGGER
							E3  E4  C14  
				
	  SERIAL
						DYNAMIXEL 
							D6  D5  
						IMU 
							B7  B6
		
		ADC
						ARUS 
							C3	  
		*/
		
			//Menu
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOB, &GPIO_InitStructure);	
							
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOD, &GPIO_InitStructure);	

						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOE, &GPIO_InitStructure);	
				
			//Limit
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOE, &GPIO_InitStructure);

						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 ;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOB, &GPIO_InitStructure);	
			
			//US
						GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_4;
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_Init(GPIOC, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_5;
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_Init(GPIOA, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_5;
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_Init(GPIOC, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_4;
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_Init(GPIOA, &GPIO_InitStructure);
						
			// ADC
						GPIO_InitStructure.GPIO_Pin		= GPIO_Pin_3|GPIO_Pin_2;
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AN;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_Init(GPIOC, &GPIO_InitStructure);
						
			// SERIAL
						//--------------------usart 1
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_Init(GPIOB, &GPIO_InitStructure);
						
						//--------------------usart 2
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_Init(GPIOA, &GPIO_InitStructure);	
						
						//--------------------usart 3
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_Init(GPIOD, &GPIO_InitStructure);
				
			// EDF
						GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;	
						GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
						GPIO_Init(GPIOA, &GPIO_InitStructure);
						GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
		
			//Motor Master
			
						GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
						GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_Init(GPIOC, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_14 | GPIO_Pin_15;
						GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_Init(GPIOB, &GPIO_InitStructure);

			//Motor Slave

						GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_6 | GPIO_Pin_7;
						GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_Init(GPIOD, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
						GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_Init(GPIOC, &GPIO_InitStructure);
						
						GPIO_InitStructure.GPIO_Pin		=	GPIO_Pin_2;
						GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_OType	=	GPIO_OType_PP;
						GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
						GPIO_InitStructure.GPIO_PuPd	=	GPIO_PuPd_UP;
						GPIO_Init(GPIOE, &GPIO_InitStructure);
			//PNUMATIK
				
						GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
						GPIO_Init(GPIOE, &GPIO_InitStructure);
						
			 //ROTARI
						//Configure pins timer 1  GPIOE11 dan GPIOE9
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOE, &GPIO_InitStructure);

							//Configure pins timer 4  GPIOD12 dan GPIOD13
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOD, &GPIO_InitStructure);
						
							//Configure pins timer 5  GPIOA1 dan GPIOA0
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOA, &GPIO_InitStructure);

						//Configure pins timer 3  GPIOB4 dan GPIOB5
						GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
						GPIO_Init(GPIOB, &GPIO_InitStructure);
						
					//-----------------------------PNUMATIK
						GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
						GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
						GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
						GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
						GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
						GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
}

void PwmMotor(){
	
	
	
}

void ServoEDF(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	TIM_InitStructure.TIM_Period 				= 999;				// 50Hz
	TIM_InitStructure.TIM_Prescaler 		= 1679;				
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM13, &TIM_InitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);						

	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_Cmd(TIM13, ENABLE);	
}


void RotaryTIM1(void){
	//enable clock rotary 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	
	
	//connect TIM  ke GPIO
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);   //TIM1_CH1 
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);  //TIM1_CH2
	
	//Debounce filter
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; 
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	
	//************ SMCR -> Mode pada encoder
	TIM1->SMCR = 3;          // Encoder mode 3 -> TI12
	//************ CCER -> edge pembacaan pulsa
	// 0x00 ->rising edge polarity // 0x02-> falling edge polarity
  TIM1->CCER = 0x00;           
	//************ ARR -> counter start (aoutoreload)
  TIM1->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
	//************ CCMR1 Counter mode
  TIM1->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
	
	
	//enable counter
	TIM_Cmd(TIM1, ENABLE);
}

void RotaryTIM4(){
	//enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//connect GPIO ke Tim
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);  //TIM4_CH1
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);  //TIM4_CH2
	
	//Debounce filter
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; 
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	//************ SMCR -> Mode pada encoder
	TIM4->SMCR = 3;          // Encoder mode 3 -> TI12
	//************ CCER -> edge pembacaan pulsa
	// 0x00 ->rising edge polarity // 0x02-> falling edge polarity
  TIM4->CCER = 0x00;           
	//************ ARR -> counter start (aoutoreload)
  TIM4->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
	//************ CCMR1 Counter mode
  TIM4->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
	
	TIM_Cmd(TIM4, ENABLE);
}


void RotaryTIM5(){
	//enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); 
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);   //TIM5_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);   //TIM5_CH2
	
	//Debounce filter
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; 
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);

	//************ SMCR -> Mode pada encoder
	TIM5->SMCR = 3;          // Encoder mode 3 / TI12
	
	//************ CCER -> edge pembacaan pulsa
	// 0 ->rising edge polarity // 0x02-> falling edge polarity
  TIM5->CCER = 0;           
	//************ ARR -> counter start (aoutoreload)
  TIM5->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
	//************ CCMR1 Counter mode
  TIM5->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
		
	//enable counter
	TIM_Cmd(TIM5, ENABLE); 
}


void RotaryTIM3(){
	//enable clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);   //TIM3_CH1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);   //TIM3_CH2
	
	//Debounce filter
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1; 
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter=0x0F;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//************ SMCR -> Mode pada encoder
	TIM3->SMCR = 3;          // Encoder mode 3 / TI12
	
	//************ CCER -> edge pembacaan pulsa
	// 0 ->rising edge polarity // 0x02-> falling edge polarity
  TIM3->CCER = 0;           
	//************ ARR -> counter start (aoutoreload)
  TIM3->ARR = 0xFFFF;      // count from 0-ARR or ARR-0
	//************ CCMR1 Counter mode
  TIM3->CCMR1 = 0xC1C1;    // f_DTS/16, N=8, IC1->TI1, IC2->TI2
	
	//enable counter
	TIM_Cmd(TIM3, ENABLE); 
}

void Timer(){		// clock source = 168MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	
	TIM_InitStructure.TIM_Period 				= 1000-1;	
	TIM_InitStructure.TIM_Prescaler 		= 168-1;					
	TIM_InitStructure.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_InitStructure.TIM_RepetitionCounter =0;
	
	TIM_TimeBaseInit(TIM10, &TIM_InitStructure);
	TIM_Cmd(TIM10,ENABLE);
	TIM_SetCounter(TIM10,0);
	//TIM_ARRPreloadConfig(TIM10,ENABLE);	
}

void TIM6_Init(){		// clock source = 84MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_InitStructure.TIM_Period 				= 65535-1;	
	TIM_InitStructure.TIM_Prescaler 		= 84-1;					
	TIM_InitStructure.TIM_ClockDivision = 0;//TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_InitStructure.TIM_RepetitionCounter =0;
	
	TIM_TimeBaseInit(TIM6, &TIM_InitStructure);
	TIM_Cmd(TIM6,ENABLE);
	TIM_SetCounter(TIM6,0);
	
}
void ADCConfig(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution				=	ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigConv	=	ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ScanConvMode			=	DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode= DISABLE;
	ADC_InitStructure.ADC_NbrOfConversion		=	0;
	ADC_StructInit(&ADC_InitStructure);
	ADC_Init(ADC1,&ADC_InitStructure);
	ADC_Cmd(ADC1,ENABLE);

}

uint16_t ReadADC(unsigned char channel){
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_28Cycles);
  ADC_SoftwareStartConv(ADC1);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  return ADC_GetConversionValue(ADC1);
}

void SetUSART1(int baudrate){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	USART_OverSampling8Cmd(USART1,DISABLE);
	USART_InitStructure.USART_BaudRate 		= baudrate;
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits		= USART_StopBits_1;
	USART_InitStructure.USART_Parity			= USART_Parity_No;
  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
}

//void DMA_USART3(){
//	DMA_DeInit(DMA1_Stream3);
//	
//	DMA_InitStructure.DMA_Channel	=	DMA_Channel_4;
//	DMA_InitStructure.DMA_DIR			=	DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_Memory0BaseAddr	=	(uint32_t) Tx3Buffer;
//	DMA_InitStructure.DMA_BufferSize			=	24;
//	
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
//	
//	DMA_Init(DMA1_Stream3,&DMA_InitStructure);
//	
//	DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);
//	
//	DMA_Cmd(DMA1_Stream3,ENABLE);
//}

void SetUSART2(int baudrate){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	USART_OverSampling8Cmd(USART2,DISABLE);
	USART_InitStructure.USART_BaudRate 		= baudrate;
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits		= USART_StopBits_1;
	USART_InitStructure.USART_Parity			= USART_Parity_No;
  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_Init(USART2,&USART_InitStructure);
	//USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART2,ENABLE);
}


void SetUSART3(int baudrate){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	USART_OverSampling8Cmd(USART3,DISABLE);
	USART_InitStructure.USART_BaudRate 		= baudrate;
	USART_InitStructure.USART_WordLength	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits		= USART_StopBits_1;
	USART_InitStructure.USART_Parity			= USART_Parity_No;
  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl	= USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure);
	//USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);	
	USART_Cmd(USART3,ENABLE);
}


void NVICConfig(){
		
		NVIC_InitStrcuture.NVIC_IRQChannel										= USART1_IRQn;
		NVIC_InitStrcuture.NVIC_IRQChannelPreemptionPriority	=	2;
		NVIC_InitStrcuture.NVIC_IRQChannelSubPriority					= 3;
		NVIC_InitStrcuture.NVIC_IRQChannelCmd									= ENABLE;
		NVIC_Init(&NVIC_InitStrcuture);
	
}
