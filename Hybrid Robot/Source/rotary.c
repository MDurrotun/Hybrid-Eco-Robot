#include "stm32f4xx.h"
#include "define.h"


int	Data1,Data2,Data3,Counter1=0,Counter2=0,Counter3=0;
float	RKanan,RKiri,RLift;

void BacaRotaryBawah(){
		Data1 			= 	TIM4->CNT;
		TIM4->CNT		=		32767;
		Counter1		+=	(Data1-32767);
	
		Data2				= 	TIM5->CNT;
		TIM5->CNT		=		32767;
		Counter2		+=	(Data2-32767);
}
void ResetRotaryBawah(){
		TIM4->CNT		= 	32767;
		Data1				=		TIM4->CNT;
		Counter1		=		0;
		RKanan			=		0;

		TIM5->CNT		=		32767;
		Data2				=		TIM5->CNT;
		Counter2		=		0;
		RKiri				= 	0;
}
void BacaLift(){	
		Data3				= 	TIM1->CNT;
		TIM1->CNT		= 	32767;
		Counter3		-=	Data3-32767;		
}
void ResetLift(){
		TIM1->CNT		=	32767;
		Data3				=	TIM1->CNT;
		Counter3		=	0;
		RLift				= 0;

}
