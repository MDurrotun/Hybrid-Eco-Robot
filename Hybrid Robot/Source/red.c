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
#include "main.h"
#include "stdio.h"


void RetryZigzagMerah(){	
		headtres=90;
		MoveGyroRe2(120,340,3,1900,625,80);
		ResetOdometry();	
		MoveGyroRe2(80,50,0,300,50,10);
		MotorStop();
		LiftStop();
	
		TiupEDF(300,MinEDF+25,500);
		TiupEDF(300,MinEDF+30,500);
		TiupEDF(300,MinEDF+35,500);
		TiupEDF(300,MinEDF+75,1500);
		
		DynamixelEDF(250);		
		ResetOdometry();
		OdoJalanEDFBlue(-10,100,900,0,MinEDF+75,44,250,250,20,NORMAL);
	
		PwmEDF=50;
		ResetOdometry();
		GyroUSJalanMerah();
		
		PwmEDF=50;
		headtres=90;
		LiftStop();		
		DynamixelEDF(-80);

		ResetOdometry();	
		Gyro2(70,110,-75+headtres,1900,60,NOSOFT); //100
		
		ResetOdometry();	
		Gyro2(70,110,-65+headtres,1900,90,NOSOFT); 
		
		ResetOdometry();	
		Gyro2(70,110,-55+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-45+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-35+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-20+headtres,1900,80,NOSOFT);
		
		ResetOdometry();
		Gyro2(70,110,0+headtres,1900,90,NOSOFT);		

		ResetOdometry();
		ResetLift();
		
		MoveTiangUSRed(1,ThresUSTiang,1200); //500							
		LiftStop();	
		
		ResetLift();
		MotorStop();MotorStop();MotorStop();
		ChaiYoMerah(90,NORMAL);	
}


void RetryTiangMerah(){
		headtres=90;
		MoveGyroRe2(120,340,3,1900,645,80);
		ResetOdometry();	
		MoveGyroRe2(80,50,0,300,50,10);
		MotorStop();
		LiftStop();
	
//		TiupEDF(300,MinEDF+30,1000);
//		TiupEDF(300,MinEDF+40,1000);
//		TiupEDF(300,MinEDF+75,1000);
		
		DynamixelEDF(250);		
		ResetOdometry();
		OdoJalanEDFBlue(-10,100,900,0,50,44,250,250,20,RETRY);
	
		PwmEDF=50;
		ResetOdometry();
		GyroUSJalanMerah();
		
		PwmEDF=50;
		headtres=90;
		LiftStop();		
		DynamixelEDF(-80);

		ResetOdometry();	
		Gyro2(70,110,-75+headtres,1900,60,NOSOFT); //100
		
		ResetOdometry();	
		Gyro2(70,110,-65+headtres,1900,90,NOSOFT); 
		
		ResetOdometry();	
		Gyro2(70,110,-55+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-45+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-35+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-20+headtres,1900,80,NOSOFT);
		
		ResetOdometry();
		Gyro2(70,110,0+headtres,1900,90,NOSOFT);		

		ResetOdometry();
		ResetLift();
		
		MoveTiangUSRed(1,ThresUSTiang,1200); //500							
		LiftStop();	
		
		ResetLift();
		MotorStop();MotorStop();MotorStop();
		ChaiYoMerah(90,NORMAL);		
}


void StartMerah(){
		lcd_clear();
		tanda=0;

		MoveGyro(50,180,0,1900,DistStart-50);
		MotorStop();

		timslider=0;
		slider=MASUK;
		delay_ms(200);
	
		PwmEDF=MinEDF+40;
		DynamixelEDF(-20); 
	
//		BelokGyro(-40,550);
//		PwmEDF=MinEDF+40;
//		DynamixelEDF(0); 		

		BelokGyro(-90,550);
		MotorStop();

	  ResetOdometry();
		OdoJalan(-15,87,1700,0,MinEDF+42,-10,5);
	
		ResetOdometry();
		OdoJalan4(-19,55,1700,0,MinEDF+42,-65,15); // HILL 1
//		OdoJalan4(-8,55,1400,0,MinEDF+30,30,17); // HILL 1
	
		ResetOdometry();
		OdoJalan4(-24,60,1700,0,MinEDF+42,-65,25); // HILL   40
//			OdoJalan4(-10,60,1400,0,MinEDF+30,10,17); // HILL 

		ResetOdometry();
		OdoJalan4(-30,105,1500,0,MinEDF+40,-30,30);  //42
//	OdoJalan4(-35,112,1190,0,MinEDF+26,13,23); // LANDAI 1

		ResetOdometry();
		OdoJalan4(-22,100,1300,0,MinEDF+37,-20,37); // HILL 2  //900 35

//		OdoJalan4(-43,117,900,0,MinEDF+27,18,30); // HILL 2  //900		
		ResetOdometry();
		OdoJalan4(-4,173,1200,0,MinEDF+35,-40,40); // Landai 3 //850 31
		
		ResetOdometry();
		OdoJalanEDF(20,190,1200,0,MinEDF+65,44,100,370,20); //55
		
		MotorStop();LiftStop();
		
		lcd_clear();
		UbahServoMerah(310);	
		
		PwmEDF=MinEDF+45;
		delay_ms(300);
		
		PwmEDF=MinEDF+75;
		delay_ms(1100);
		
		DynamixelEDF(300);
		PwmEDF=MinEDF+75;
		delay_ms(500);
		
		ResetOdometry();
		GyroUSJalanMerah();
		
		ResetOdometry();	
		Gyro2(70,110,-85+headtres,1900,80,NOSOFT);

		ResetOdometry();	
		Gyro2(70,110,-75+headtres,1900,85,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-65+headtres,1900,90,NOSOFT); 
		
		ResetOdometry();	
		Gyro2(70,110,-55+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-45+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-35+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro2(70,110,-20+headtres,1900,80,NOSOFT);
		
		ResetOdometry();
		Gyro2(70,110,0+headtres,1900,60,NOSOFT);

		ResetOdometry();
		ResetLift();
		
		MoveTiangUSRed(1,ThresUSTiang,1200); //500							
		LiftStop();	
		
//	MotorStop();LiftStop();PwmEDF=50;
//	while(1){MotorStop();LiftStop();PwmEDF=50;}
				
/*		
		flags=0;
		while((GyroHeading >= 5 || GyroHeading <= -5) && flags<=2){
				if(GyroHeading >= 5){
						flags++;
						ResetOdometry();
						KiriMundur(600);			KananStop();
						dist=0;
						while(dist>= - 10){
							Odometry();
							dist = (int) RKiri;
							lcd_gotoxy(0,0);
							lcd_int16(GyroHeading);
						}
						KiriMaju(400);			KananMaju(470);
						while(LimTiang){lcd_gotoxy(0,0);lcd_int16(GyroHeading);}
				}
				else if(GyroHeading <= -5) {
						flags++;
						ResetOdometry();
						dist=0;
						KiriStop();				KananMundur(670);
						while(dist>= - 10){
							Odometry();
							dist = (int) RKanan; //+ RKiri)/2;
							lcd_gotoxy(0,0);
							lcd_int16(GyroHeading);	
						}
						KiriMaju(400);			KananMaju(470);
						while(LimTiang){lcd_gotoxy(0,0);lcd_int16(GyroHeading);}
				} 
			}
*/			
			
			ResetLift();
			MotorStop();MotorStop();MotorStop();
			ChaiYoMerah(0,NORMAL);			
}


