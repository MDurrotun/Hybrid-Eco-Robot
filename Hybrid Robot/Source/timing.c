#include "stm32f4xx.h"
#include "main.h"
#include "define.h"
#include "motor.h"


uint16_t tick,tik,detik,timelift,timslider,pwmtik;
unsigned char slider;
void 	SysTick_Handler(){
	TimingDelay_Decrement();
	tik++;
	pwmtik++;
	timelift++;
	timslider++;
	if(tik % 100 	==0) tick++;
	if(tik % 1000 ==0) detik++;
	if(slider==MASUK){
			if(timslider<400)LenganMasuk();
			else 			 LenganStop();
	}
}


