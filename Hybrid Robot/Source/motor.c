#include "stm32f4xx.h"
#include "define.h"
#include "lcd.h"
#include "usart.h"
//Motor Kiri 150



void LenganStop(){
		Ena1_1;Dir1_1;
}

void LenganMasuk(){
		if(LimBuka) {Ena1_0;Dir1_1;}
		else 				{Ena1_1;Dir1_1;}		
}

void LenganKeluar(){
		if(LimTutup) {Ena1_0;Dir1_0;}
		else 				 {Ena1_1;Dir1_1;}
}


void LiftStop(){
		Ena2_1; Dir2_1;
}

void LiftTurun(){
	if(LimLengan){
		Ena2_0;	Dir2_1;

	}
	else	LiftStop();
}

void LiftNaik(){
	if(LimAtas){
		Ena2_0;	Dir2_0;
	}
	else	LiftStop();
}


void JepitBuka(){

		SetC(7);
		ClrC(6);
		
}

void JepitTutup(){
		ClrC(7);
		SetC(6);
		
}

void JepitStop(){
		SetC(7);
		SetC(6);
		
}	

void PemanjatNaik(){
		ClrC(9);
		SetC(8);
		ClrB(14);
		SetB(15);
}
void PemanjatTurun(){
		SetC(9);
		ClrC(8);
		SetB(14);
		ClrB(15);
}
void PemanjatStop(){
		SetC(9);
		SetC(8);
		SetB(14);
		SetB(15);
}

void KananMundur(int speed){
	//	int ConvSpeed = (int) speed;
		SetPwm(0xE1,speed);
}

void KananMaju(int speed){
//		int ConvSpeed = (int) speed;
		SetPwm(0xE1,-1*speed);
}


void KiriMaju(int speed){
	//	int ConvSpeed = (int) speed;
		SetPwm(0xE2,speed);
}

void KiriMundur(int speed){
//		int ConvSpeed = (int) speed;
		SetPwm(0xE2,-1*speed);
}

void KiriStop(){
		SetPwm(0xE2,0);
}

void KananStop(){
		SetPwm(0xE1,0);
}


void MotorStop(){
			KiriStop();
			KananStop();
			KiriStop();
			KananStop();
			KiriStop();
			KananStop();
}


