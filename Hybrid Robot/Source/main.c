#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "define.h"
#include "lcd.h"
#include "konfigurasi.h"
#include "usart.h"
#include "global.h"
#include "motor.h"
#include "rotary.h"
#include "move.h"
#include "red.h"
#include "blue.h"
#include "stdio.h"
#include "main.h" 
#include "eeprom.h"

/*
			0 derajat Servo = pwm 90;
			90 derjata Servo = pwm 48;
			max =112
*/

static __IO uint32_t TimingDelay;
uint16_t CurrentADC=0,ADCGripper=2118,ThresholdUS,PosServo=1600,PosDynamixelEDF=200,DistStart=298,ThresUSTiang=27,tutupdyx=279,bukadyx=512;
float  JarakUS1,JarakUS2,JarakUS3,corr;
unsigned char EDFCalib=0,select=0,USWall=0,flags=0,flagstr=0,flagnaik=0,flagbtn=0,flagtiang=0,flaglift=0;
int DataADC,i,n,heads,dist,SudutZigzag=92,headtres=0,MinEDF=35,a=0,SetPointGyro;
void delay_ms(__IO uint32_t nTime); 
void delay_us(__IO uint32_t xx);
int SetPointGyroWall;


void TampilInt(float w, float x, float y, float z){
	lcd_gotoxy(0,0);
	lcd_int16(w);
	lcd_gotoxy(7,0);
	lcd_int16(x);
	lcd_gotoxy(14,0);
	lcd_int16(GyroHeading);
	
	lcd_gotoxy(0,1);
	lcd_int16(y);
	lcd_gotoxy(7,1);
	lcd_int16(z);
}

//TampilFloat(Mki,Mka,Distance,RLift);
void TampilFloat(float w, float x, float y, float z){
	lcd_gotoxy(0,0);
	lcd_float6(w);
	lcd_gotoxy(7,0);
	lcd_float6(x);
	//lcd_gotoxy(14,0);
	//lcd_int16(GyroHeading);
	
	lcd_gotoxy(0,1);
	lcd_float6(y);
	lcd_gotoxy(7,1);
	lcd_float6(z);
}

void WriteEEPROM(uint16_t data1){
		FLASH_Unlock();
		EE_Init();
		EE_WriteVariable(VirtAddVarTab[0],data1);
}
uint16_t ReadEEPROM(){
	uint16_t data1;
	EE_ReadVariable(VirtAddVarTab[0],&data1);
	
	return data1;
}
void AdjustServo(){
			i=600;
			while(1){
				if(!KuningKanan) i+=10;
				else if(!KuningKiri) i-=10;
				delay_ms(100);
				lcd_gotoxy(0,0);
				lcd_int16(i);
				SetDynamixel(2,30,i);
			}
}
void BacaUS1(){	
	
			US1On;	delay_us(15);			US1Off;			
			//for(i=0;i<=10;i++){	}
			TIM6->CNT = 0;	
			while(!EUS1 && TIM6->CNT<=20000){}	
			TIM6->CNT = 0;		
			while(EUS1 && TIM6->CNT<=3500)	{	}
			JarakUS1=TIM6->CNT;	
			JarakUS1=	JarakUS1 * 0.1657f;
			
			if(JarakUS1>350) JarakUS1=350; 
}

void BacaUS2(){
			
			US3On;	delay_us(15);			US3Off;			
			//for(i=0;i<=10;i++){	}
			TIM6->CNT = 0;	
			while(!EUS3 && TIM6->CNT<=20000){}	
			TIM6->CNT = 0;			
			while(EUS3 && TIM6->CNT<=3500)	{	}
			JarakUS2=TIM6->CNT;
			JarakUS2=JarakUS2 * 0.1657f;
			if(JarakUS2>350) JarakUS2=350;
}

void KalibrasiIMU(){
			for(i=0;i<100;i++)ThresholdGyro += DataYAW;
			ThresholdGyro = ThresholdGyro/100;
}


void SiapPropeler90(int speed){ //speed
		//SetDynamixel(1,32,1023);
		SetDynamixel(2,30,PosServo-300); //setengah
		SetDynamixel(1,30,765);  //buka separo
		//SetDynamixel(1,30,450);  //buka separo
}

void SiapPropeler90Tutup(int speed){ // 250
		//SetDynamixel(1,32,1023);
		SetDynamixel(2,30,PosServo-300);
		SetDynamixel(1,30,135);
		//SetDynamixel(1,30,135);
}

void SiapPropeler(){
		SetDynamixel(1,32,1023);
		SetDynamixel(2,30,PosServo+120);
		SetDynamixel(1,30,765);
}

void SiapPropeler160(int speed){
		SetDynamixel(1,32,1023);
		SetDynamixel(2,30,PosServo-830);
		SetDynamixel(1,30,140);	
}

void PasangPropeler(){
		//SetDynamixel(1,32,1023);
		SetDynamixel(2,30,PosServo-1010);
		SetDynamixel(1,30,135);	//jepit
}

void PasangPropeler2(){
		//SetDynamixel(1,32,1023);		
		SetDynamixel(1,30,600);		//agak buka
		SetDynamixel(2,30,PosServo-1110);	
}

void LepasPropeler(){
			//SetDynamixel(1,32,1023);
			SetDynamixel(1,30,765);
			delay_ms(100);
			SetDynamixel(2,30,PosServo-600);
}


void TangkapPropeler2(){
	int k; // 2300
	for(k=0;k<=-330;k--){
		SetDynamixel(1,32,1023);
		SetDynamixel(1,30,765);
		SetDynamixel(2,30,PosServo+k);
		delay_ms(1);
	}
	for(k=0;k>=130;k++){
		SetDynamixel(1,32,1023);
		SetDynamixel(1,30,765);
		SetDynamixel(2,30,PosServo+k);
		delay_ms(1);
	}
	for(k=0;k<=-330;k--){
		SetDynamixel(1,32,1023);
		SetDynamixel(1,30,765);
		SetDynamixel(2,30,PosServo+k);
		delay_ms(1);
	}

	SetDynamixel(1,32,1023);
	SetDynamixel(1,30,140);
	
	SetDynamixel(2,30,PosServo-550);	
}
void XTangkapPropeler2(){
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,90);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);

			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
			SetDynamixel(1,32,1023);
			SetDynamixel(2,32,90);
			SetDynamixel(2,30,PosServo+130);
			SetDynamixel(1,30,140);
}
void TangkapPropeler1(){	
	int k;
	//for(i=0;i<10;i++)
	//for(i=10;i>0;i--)
	for(k=0;k>=-330;k-=15){
		//SetDynamixel(1,32,1023);
		SetDynamixel(1,30,765);
		SetDynamixel(2,30,PosServo+k);
		delay_us(1);
	}
	for(k=-330;k<=0;k+=20){
		//SetDynamixel(1,32,1023);
		SetDynamixel(1,30,765);
		SetDynamixel(2,30,PosServo+k);
		delay_us(1);
	}
	for(k=0;k>=-600;k-=10){
		//SetDynamixel(1,32,1023);
		if(k>-500) SetDynamixel(1,30,765); //buka
		else 			SetDynamixel(1,30,300);	
		SetDynamixel(2,30,PosServo+k);
		delay_us(1);
	}

	//SetDynamixel(1,32,1023);
	SetDynamixel(1,30,140);  //nutup	
	SetDynamixel(2,30,PosServo-750);
	delay_ms(100);
//		SetDynamixel(1,32,1023);
//		SetDynamixel(2,32,90);
//		SetDynamixel(2,30,PosServo+130);
//		SetDynamixel(1,30,765);
//		SetDynamixel(1,32,1023);
//		SetDynamixel(2,32,90);
//		SetDynamixel(2,30,PosServo+130);
//		SetDynamixel(1,30,765);
}
void XTangkapPropeler1(){
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);

		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
		SetDynamixel(1,32,1023);
		SetDynamixel(2,32,90);
		SetDynamixel(2,30,PosServo+130);
		SetDynamixel(1,30,765);
}

void DynamixelEDF(int32_t theta){
	//	SetDynamixel(3,32,512);
		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
//		SetDynamixel(3,32,512);
//		SetDynamixel(3,30,PosDynamixelEDF+theta);
}

void HoldLift(){
				BacaLift();
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<48.5f) LiftNaik();
				else if(RLift>=50 || !LimAtas) LiftStop();
					
}

void TiupEDF(int servo,int edf,int delay){
		//1 delay = 100 ms
				timelift=0;
				PwmEDF=edf;
				DynamixelEDF(servo);
				MotorStop();
				LiftStop();
				JepitStop();
				while(timelift <= delay){
						BacaLift();
						RLift = (float) -(Counter3*R3_tik);
						if(RLift<48.5f) LiftNaik();
						else if(RLift>=51) LiftStop();
					
						if(!LimAtas)	LiftStop();
				}
}


//int exp_filter(int data){
//		static float x=0,y=0,last_y=0;
//    x=(float)data;
//    y=last_y+(0.888)*(x-last_y);
//    last_y=y;
//    return (int)y;
//}

void ChaiYoMerah(int theta,int mode){
	
int flag=0,a,tanda=0,kp=4,sudut=0,flagLengan=0;
	lcd_clear();
	
	if(mode==RETRY){
			kp=3;
			sudut=0;
	}
// %JEPIT PROPELER DIKIT LIFT NAIK
	SetDynamixel(2,30,3300); SetDynamixel(2,30,3300); SetDynamixel(2,30,3300);
	LiftNaik();
	
	// %JEPIT TIANG LIFT NAIK
	CurrentADC=0;
	JepitTutup();				
	KiriMaju(420);KananMaju(410);	delay_ms(200);	
	CurrentADC=ReadADC(13);
	lcd_clear();
	while(CurrentADC<ADCGripper){	
			//CurrentADC = exp_filter(ReadADC(13));
			CurrentADC = ReadADC(13);
			if(!LimAtas){ LiftStop();}
			else if(LimAtas ) {LiftNaik(); }		
			lcd_gotoxy(0,0); lcd_int16(CurrentADC);
	} 
	JepitStop();
	MotorStop();
	
	// %JEPIT ERAT PROPELER LIFT NAIK
	SetDynamixel(2,30,3350); SetDynamixel(2,30,3350); SetDynamixel(2,30,3350);

	// %MENUJU PUCUK LIFT NAIK
	PemanjatNaik();			
	
	tik=0;
	while(tik<=400){
			Ena1_0;Dir1_1; //Lengan masuk
			
			if(LimAtas && LimNaik && LimTutup) 			{LiftNaik(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(LimAtas && LimNaik && !LimTutup){LiftNaik(); PemanjatNaik(); LenganStop(); ;}
			else if(LimAtas && !LimNaik && LimTutup){LiftNaik(); PemanjatStop(); Ena1_0;Dir1_1;}
			else if(LimAtas && !LimNaik && !LimTutup){LiftNaik(); PemanjatStop(); LenganStop();}
			else if(!LimAtas && LimNaik && LimTutup){LiftStop(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(!LimAtas && LimNaik && !LimTutup){LiftStop(); PemanjatNaik(); LenganStop();}
			else if(!LimAtas && !LimNaik && LimTutup){LiftStop(); PemanjatStop(); Ena1_0;Dir1_1;}
			else 																		 {LiftStop(); PemanjatStop(); LenganStop();}
	}	

	Ena1_0;Dir1_1; //LenganMasuk
	flagnaik=0;	
	while(1){		
			if(LimAtas && LimNaik && LimTutup) 			{LiftNaik(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(LimAtas && LimNaik && !LimTutup){LiftNaik(); PemanjatNaik(); LenganStop(); flagLengan=1;}
			else if(LimAtas && !LimNaik && LimTutup){LiftNaik(); PemanjatStop(); Ena1_0;Dir1_1; break;}
			else if(LimAtas && !LimNaik && !LimTutup){LiftNaik(); PemanjatStop(); LenganStop();flagLengan=1; break;}
			else if(!LimAtas && LimNaik && LimTutup){LiftStop(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(!LimAtas && LimNaik && !LimTutup){LiftStop(); PemanjatNaik(); LenganStop();flagLengan=1;}
			else if(!LimAtas && !LimNaik && LimTutup){LiftStop(); PemanjatStop(); Ena1_0;Dir1_1; break;}
			else 																		 {LiftStop(); PemanjatStop(); LenganStop();flagLengan=1; break;}
			
			ClimbControl(kp,0,theta,mode);
	}
		
	//LenganStop();
	PwmEDF=50;
	PemanjatStop();
	
	// %SAMPAI ATAS PELUK ERAT
	JepitTutup(); delay_ms(150); JepitStop();
	LiftStop();
	
	// %Masuk sampai limit(Koreksi)
	while(flagLengan==0){
			Ena1_0;Dir1_1; //Lengan Masuk
			if(!LimTutup){LenganStop(); flagLengan=1;}
	}
		
	// %PASANG PROPELER
	tik=0;
	while(1){  
		if(tik<=200) 			{
			Ena1_0;Dir1_1;
			if(LimAtas && LimNaik ) 		{LiftNaik(); PemanjatNaik();}
			else if(!LimAtas && LimNaik){LiftStop(); PemanjatNaik();}
			else if(LimAtas && !LimNaik){LiftNaik(); PemanjatStop();}
			else 												{LiftStop(); PemanjatStop();}	
		}	
		else if(tik>200)	{LenganStop();PemanjatStop(); LiftStop();break;}	
		
	}
	
	delay_ms(400);
//	for(a=3300; a>=3250; a-=5){
//			SetDynamixel(2,30,a);
//	}	
//	
	
	// %LEPAS PROPELER DIKIT
	SetDynamixel(2,30,3250); SetDynamixel(2,30,3250); SetDynamixel(2,30,3250);

	// %LIFT TURUN
	ResetLift();
	tik=0;
	while(tik<=500){
			BacaLift();
			RLift = (float) -(Counter3*R3_tik);
			if(RLift>-20)LiftTurun();
			else if(RLift<=-10&&RLift>=-11) LiftStop();
			else LiftNaik();		
	}
	
	// %LEPAS PROPELER BANYAK
	SetDynamixel(2,30,2900);
	LiftStop();
	
	// %Tarik Lengan Penjepit Keluar
	tik=0;
	while(tik<=300){
			Ena1_0;Dir1_0;
	}
	
	LenganStop();
	while(1){LiftStop();}
}

void ChaiYoBiru(int theta,int mode){
	
	int flag=0,a,tanda=0,kp=2,flagLengan=0,tandaLimit;
	lcd_clear();
	
	if(mode==RETRY){
			kp=2;
			theta=-2;
	}
	
	// %JEPIT PROPELER DIKIT LIFT NAIK
	SetDynamixel(2,30,3300); SetDynamixel(2,30,3300); SetDynamixel(2,30,3300);
	LiftNaik();
	
	// %JEPIT TIANG LIFT NAIK
	CurrentADC=0;
	JepitTutup();				
	KiriMaju(320);KananMaju(310);	delay_ms(200);	
	CurrentADC=ReadADC(13);
	lcd_clear();
	while(CurrentADC<ADCGripper){	
			//CurrentADC = exp_filter(ReadADC(13));
			CurrentADC = ReadADC(13);
			if(!LimAtas){ LiftStop();}
			else if(LimAtas ) {LiftNaik(); }		
			lcd_gotoxy(0,0); lcd_int16(CurrentADC);
	} 
	JepitStop();
	MotorStop();
	
	// %JEPIT ERAT PROPELER LIFT NAIK
	SetDynamixel(2,30,3400); SetDynamixel(2,30,3400); SetDynamixel(2,30,3400);

	// %MENUJU PUCUK LIFT NAIK
	PemanjatNaik();			
	
	tik=0;
	while(tik<=400){
			Ena1_0;Dir1_1; //Lengan masuk
			
			if(LimAtas && LimNaik && LimTutup) 			{LiftNaik(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(LimAtas && LimNaik && !LimTutup){LiftNaik(); PemanjatNaik(); LenganStop();}
			else if(LimAtas && !LimNaik && LimTutup){LiftNaik(); PemanjatStop(); Ena1_0;Dir1_1;}
			else if(LimAtas && !LimNaik && !LimTutup){LiftNaik(); PemanjatStop(); LenganStop();}
			else if(!LimAtas && LimNaik && LimTutup){LiftStop(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(!LimAtas && LimNaik && !LimTutup){LiftStop(); PemanjatNaik(); LenganStop();}
			else if(!LimAtas && !LimNaik && LimTutup){LiftStop(); PemanjatStop(); Ena1_0;Dir1_1;}
			else 																		 {LiftStop(); PemanjatStop(); LenganStop();}
	}	

	Ena1_0;Dir1_1; //LenganMasuk
	flagnaik=0;	
	while(1){		
			if(LimAtas && LimNaik && LimTutup) 			{LiftNaik(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(LimAtas && LimNaik && !LimTutup){LiftNaik(); PemanjatNaik(); LenganStop(); flagLengan=1;}
			else if(LimAtas && !LimNaik && LimTutup){LiftNaik(); PemanjatStop(); Ena1_0;Dir1_1; break;}
			else if(LimAtas && !LimNaik && !LimTutup){LiftNaik(); PemanjatStop(); LenganStop();flagLengan=1; break;}
			else if(!LimAtas && LimNaik && LimTutup){LiftStop(); PemanjatNaik(); Ena1_0;Dir1_1;}
			else if(!LimAtas && LimNaik && !LimTutup){LiftStop(); PemanjatNaik(); LenganStop();flagLengan=1;}
			else if(!LimAtas && !LimNaik && LimTutup){LiftStop(); PemanjatStop(); Ena1_0;Dir1_1; break;}
			else 																		 {LiftStop(); PemanjatStop(); LenganStop();flagLengan=1; break;}
			
		ClimbControlBiruReverse(kp,0,theta,mode);
	}		
		
	//LenganStop();
	PwmEDF=50;
	PemanjatStop();
	
	// %SAMPAI ATAS PELUK ERAT
	JepitTutup(); delay_ms(150); JepitStop();
	LiftStop();
	
	// %Masuk sampai limit(Koreksi)
	while(flagLengan==0){
			Ena1_0;Dir1_1; //Lengan Masuk
			if(!LimTutup){LenganStop(); flagLengan=1;}
	}

	// %PASANG PROPELER
	if(LimTutup)tandaLimit=300;
	else tandaLimit=200;
	
	tik=0;
	while(1){  
		if(tik<=tandaLimit){ 
			Ena1_0;Dir1_1;
			if(LimAtas && LimNaik ) 		{LiftNaik(); PemanjatNaik();}
			else if(!LimAtas && LimNaik){LiftStop(); PemanjatNaik();}
			else if(LimAtas && !LimNaik){LiftNaik(); PemanjatStop();}
			else 												{LiftStop(); PemanjatStop();}	
		}		
		else if(tik>tandaLimit)	{LenganStop();PemanjatStop(); LiftStop();break;}	
		
	}
	
	delay_ms(400);
	
	// %LEPAS PROPELER DIKIT
	SetDynamixel(2,30,3250); SetDynamixel(2,30,3250); SetDynamixel(2,30,3250);

	// %LIFT TURUN
	ResetLift();
	tik=0;
	while(tik<=500){
			BacaLift();
			RLift = (float) -(Counter3*R3_tik);
			if(RLift>-20)LiftTurun();
			else if(RLift<=-10&&RLift>=-11) LiftStop();
			else LiftNaik();		
	}
	
	// %LEPAS PROPELER BANYAK
	SetDynamixel(2,30,2900);
	LiftStop();
	
	// %Tarik Lengan Penjepit Keluar
	tik=0;
	while(tik<=300){
			Ena1_0;Dir1_0;
	}
	
	LenganStop();
	while(1){LiftStop();}
}

void ChaiYoRetryMerah(){
					KalibrasiIMU();
					ChaiYoMerah(0,RETRY);
}


void ChaiYoRetryBiru(){
					KalibrasiIMU();
					ChaiYoBiru(-2,RETRY);
}

void RedChaiyo(){
		int flag=0;
		lcd_clear();
		LenganStop();
		while(LimJepit&&flag==0){
			ClrB(8);SetB(12);
			if(!LimJepit){LenganStop(); flag=1;}
			lcd_gotoxy(0,0);
			lcd_string("1");
		}
		
		SetB(8);
		LenganStop();
		SetDynamixel(2,30,tutupdyx);
		delay_ms(50);
		
					LiftNaik();
		
					CurrentADC=0;
					JepitTutup();
					KiriMaju(320);KananMaju(310);
					delay_ms(100);					
					CurrentADC=ReadADC(13);
					lcd_clear();
					while(CurrentADC<ADCGripper){	
							CurrentADC = ReadADC(13);
							if(!LimAtas) LiftStop();
					}
					
					lcd_gotoxy(0,0);
					lcd_uint16(CurrentADC);
					LiftStop();
					JepitStop();
					MotorStop();

					PemanjatNaik();											// Pemanjat Terus
					flagnaik=0;
					tik=0;
					while(LimNaik && flagnaik==0){
						if(tik<350)			{ClrB(8);SetB(12);}
						else if(tik>350)			{SetB(8);}
						else 					{SetB(8);}
						HoldLift();
						if(LimAtas) {LiftNaik();}
						else				{LiftStop();}
						//ClimbControl(8,0,theta);
						if(!LimNaik)flagnaik=1;
					}
					
						if(flagnaik==1){
						HoldLift();
						PwmEDF=50;
						PemanjatStop();
					}
					PemanjatStop();
//					LenganMasuk();
//					SetDynamixel(2,30,bukadyx);
//					LenganKeluar();

}

int main(void){
		SysTick_Config(SystemCoreClock / 1000);
		KonfigurasiGPIO();
		
		AllGPIO();
		slider=4;
		LiftStop();
		LenganStop();
		NVICConfig();
		PwmMotor();
		ServoEDF();
		RotaryTIM1();
		RotaryTIM3();
		RotaryTIM4();
		RotaryTIM5();
		ResetRotaryBawah();
		ResetLift();
		Timer();
		SetUSART1(115200);
		SetUSART2(19230);
		SetUSART3(57613);
		KirimData2(0xA5);
		KirimData2(0x52);
		ADCConfig();
		STOP;

		lcd_init();
		lcd_gotoxy(0,0);
		lcd_string("BISMILLAH PENSAE");
		lcd_gotoxy(0,1);
		lcd_string("BISMILLAH JUARA");
		delay_ms(1000);
		lcd_clear();
				
		 //-125 eco lama //30 eco baru
		DynamixelEDF(-50);
		LiftStop();
		JepitStop();
		delay_ms(100);
		TIM6_Init();
		SetDynamixel(2,30,3250); //3210 //3270
		SetDynamixel(2,30,3250);
		SetDynamixel(2,30,3250);
		
		while(1){
			for(n=0;n<5000;n++) while(KuningKanan && MerahTengah && KuningKiri && LimTurun && LimStop && AdjustDx){
				BacaUS1();
				BacaUS2();
				Odometry();
				BacaLift();
				
				
				lcd_gotoxy(0,0);	lcd_uint8(JarakUS1);
				lcd_gotoxy(4,0);	lcd_uint8(JarakUS2);
				lcd_gotoxy(8,0); lcd_int8(DataYAW);
				lcd_gotoxy(0,1);	lcd_int8(RKanan);
				lcd_gotoxy(4,1);	lcd_int8(RKiri);
				lcd_gotoxy(8,1);	lcd_int8(0-Counter3);
				lcd_gotoxy(14,0);	lcd_bin1(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11));
				lcd_gotoxy(13,0);	lcd_bin1(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10));
				lcd_gotoxy(12,0);	lcd_bin1(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15));
				lcd_gotoxy(15,0);	lcd_bin1(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_14));
				
				lcd_gotoxy(12,0);	lcd_bin1(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12));		
				lcd_gotoxy(13,1);	lcd_bin1(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13));
				lcd_gotoxy(14,1);	lcd_bin1(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8));
				lcd_gotoxy(15,1);	lcd_bin1(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_15));
				
				KananStop();
				KiriStop();
				LiftStop();
				JepitStop();
				PemanjatStop();
				
				
				if(!LimBuka) LenganKeluar();
				else	 				LenganStop();
				
			
	}			
			for(n=0;n<5000;n++) while(!KuningKanan || !MerahTengah || !KuningKiri || !LimTurun || !LimStop || !AdjustDx ){
				if(!KuningKanan){while(!KuningKanan){} select=1; }
				else if(!MerahTengah){while(!MerahTengah){}	select=2; 	}
				else if(!KuningKiri){	while(!KuningKiri){}select=3;}
				else if(!LimTurun){while(!LimTurun){}		select=4;}
				else if(!LimStop)	{while(!LimStop){}		select=5;}
				else if(!AdjustDx) {while(!AdjustDx){}	select=6;}
				}	

			if(select==1 && !ToggleTombol){
				lcd_clear();
				ResetLift();
				ResetOdometry();
				KalibrasiIMU();
				StartMerah();
			}
			else if(select==2 && !ToggleTombol){ 
				lcd_clear();
				ResetLift();
				ResetOdometry();
				KalibrasiIMU();
				RetryZigzagMerah();
			}
			else if(select==3 && !ToggleTombol){
				lcd_clear();
				ResetLift();
				ResetOdometry();
				KalibrasiIMU();
				RetryTiangMerah();
			}
			else if(select==1 && ToggleTombol){ //biru

					lcd_clear();
					ResetLift();
					ResetOdometry();
					KalibrasiIMU();
					StartBiru();
			}
			else if(select==2 && ToggleTombol){ //biru
					lcd_clear();
					ResetLift();
					ResetOdometry();
					KalibrasiIMU();
					RetryZigzagBiru();
			}
			else if(select==3 && ToggleTombol){ //biru
					lcd_clear();
					ResetLift();
					ResetOdometry();
					KalibrasiIMU();
					RetryTiangBiru();
			}
			
			else if(select==4 && !ToggleTombol){
					lcd_clear();
					ResetLift();
					ResetOdometry();
					KalibrasiIMU();
					ChaiYoRetryMerah();
			}
			else if(select==4 && ToggleTombol){
				lcd_clear();
				ResetLift();
				ResetOdometry();
				KalibrasiIMU();
				ChaiYoRetryBiru();
			}
			else if(select==5 && (!ToggleTombol || ToggleTombol)){

				JepitBuka();
				delay_ms(50);
				JepitStop();
				
			}
			else if(select==6 && (!ToggleTombol || ToggleTombol)){
				JepitTutup();
				delay_ms(50);
				JepitStop();
			}
//			else if(select==7 && !ToggleTombol){
//				lcd_clear();
//				ResetLift();
//				ResetOdometry();
//				KalibrasiIMU();
//				PasangPropeler2();
//				StartMerah2();
//			}
//			else if(select==7 && ToggleTombol){
//				lcd_clear();
//				ResetLift();
//				ResetOdometry();
//				KalibrasiIMU();
//				PasangPropeler2();
//				StartBiru2();		
//			}
		}

}


void delay_ms(__IO uint32_t nTime){ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void TimingDelay_Decrement(void){
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void delay_us(__IO uint32_t xx){  
	xx*= delay_const;	
   while(xx--);
}
