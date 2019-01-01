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


void StartBiru(){
		headtres = 180;
		lcd_clear();
		tanda=0;
	
		MoveGyro(100,180,0,1900,DistStart-40);
		MotorStop();MotorStop();MotorStop();
	
		timslider=0;
		slider=MASUK;
		delay_ms(200);
	
		PwmEDF=MinEDF+50;
		DynamixelEDF(-20);
		BelokGyro(90,550);	

		ResetOdometry();	
		OdoJalan(3,110,1700,0,MinEDF+37,-40,5);  // 3.93 4
	
		ResetOdometry();
		OdoJalan3(10,44,1700,0,MinEDF+37,-75,10); // 54 HILL 1 x12
		
		ResetOdometry();
		OdoJalan3(15,61,1700,0,MinEDF+37,-70,20); // HILL  x22
		
		ResetOdometry();	
		OdoJalan3(50,115,1600,0,MinEDF+40,-85,25); // 1500 LANDAI 1 
		
		ResetOdometry();
		OdoJalan3(25,117,1600,0,MinEDF+40,-10,35); // 1500 HILL
		
		//ResetOdometry();
		//OdoJalan3(-30,170,1300,0,MinEDF+35,15,40); // Landai 2 32 s50
		
		// %LANDAI 2
		ResetOdometry();
		OdoJalan3(-5,70,1300,0,MinEDF+35,40,40); 
		ResetOdometry();
		OdoJalan3(-20,110,1400,0,MinEDF+30,60,40); //E35
		
		// %HILL 3 (TERAKHIR)				
		ResetOdometry();
		OdoJalanEDFBlue(0,80,900,0,MinEDF+65,44,100,140,20,NORMAL); // -60 . 85 s200	
		ResetOdometry();
		OdoJalanEDFBlue(-30,100,600,0,MinEDF+65,44,140,220,20,NORMAL); // -60 . 85
		
		lcd_clear();
		UbahServoBiru(260);	
	
		// %TIUP ECO DI ZIGZAG	
		MotorStop();LiftStop();	
		
		//DynamixelEDF(230);		
		PwmEDF=MinEDF+75;
		delay_ms(400);		
		
		for(i=0;i<30;i++){
				BacaUS1(); 
		}	
		
		PwmEDF=MinEDF+75;
		delay_ms(800);
		
		DynamixelEDF(250);
		PwmEDF=MinEDF+75;
		delay_ms(250);
		
		DynamixelEDF(250);		
		
		ResetOdometry();
		if(JarakUS1 >250)		OdoJalanEDFBlue(25,70,900,0,MinEDF+75,44,250,250,20,NORMAL); // -60 . 85 x-15
		else								OdoJalanEDFBlue(15,75,900,0,MinEDF+75,44,250,250,20,NORMAL); // -60 . 85 x-10 y90
		
		// %MEMPET DINDING KIRI
		PwmEDF=50;
		ResetOdometry();
		GyroUSJalanBiru();

		// %MELINGKAR
		PwmEDF=50;
		headtres=0;		
		DynamixelEDF(-80);
		
		ResetOdometry();	
		Gyro(70,100,80+headtres,1900,90,NOSOFT); 
		
		ResetOdometry();	
		Gyro(70,100,70+headtres,1900,85,NOSOFT); 
		
		ResetOdometry();	
		Gyro(70,100,60+headtres,1900,80,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,50+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,35+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,20+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,0+headtres,1900,90,NOSOFT);
		
		ResetOdometry();
		ResetLift();
		MoveTiangUS(2,ThresUSTiang-3,1200);
		
//		MotorStop();LiftStop();PwmEDF=50;
//		while(1){MotorStop();LiftStop();PwmEDF=50;}
		
		ResetLift();
		MotorStop();
		ChaiYoBiru(-2,NORMAL);
}

void RetryZigzagBiru(){				
		MoveGyroRe2(120,340,-2,1900,630,80);
		ResetOdometry();	
		MoveGyroRe2(80,50,0,300,50,10);
		MotorStop();
	
		TiupEDF(300,MinEDF+30,1000);
		TiupEDF(300,MinEDF+40,1000);
		TiupEDF(300,MinEDF+75,1000);
	
		DynamixelEDF(250);		
		ResetOdometry();
		OdoJalanEDFBlue(15,120,900,0,MinEDF+75,44,250,250,20,NORMAL);
	
		// %MEMPET DINDING KIRI
		PwmEDF=50;
		ResetOdometry();
		GyroUSJalanBiru();
	
		// %MELINGKAR
		PwmEDF=50;
		headtres=-90;		
		DynamixelEDF(-80);
		
		ResetOdometry();	
		Gyro(70,100,70+headtres,1900,100,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,60+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,50+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,40+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,20+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,0+headtres,1900,85,NOSOFT);
		
		ResetOdometry();
		ResetLift();
		MoveTiangUS(2,ThresUSTiang-3,1200);
		
//		MotorStop();LiftStop();PwmEDF=50;
//		while(1){MotorStop();LiftStop();PwmEDF=50;}
		
		ResetLift();
		MotorStop();
		ChaiYoBiru(-90,NORMAL);
}

void RetryTiangBiru(){
		MoveGyroRe2(120,340,-2,1900,653,80);
		ResetOdometry();	
		MoveGyroRe2(80,50,0,300,50,10);
		MotorStop();
	
//		TiupEDF(300,MinEDF+30,1000);
//		TiupEDF(300,MinEDF+40,1000);
//		TiupEDF(300,MinEDF+75,1000);
//	
		DynamixelEDF(250);		
		ResetOdometry();
		OdoJalanEDFBlue(20,120,900,0,50,44,250,250,20,RETRY);
	
		// %MEMPET DINDING KIRI
		PwmEDF=50;
		ResetOdometry();
		GyroUSJalanBiru();
	
		// %MELINGKAR
		PwmEDF=50;
		headtres=-90;		
		DynamixelEDF(-80);
		
		ResetOdometry();	
		Gyro(70,100,70+headtres,1900,100,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,60+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,50+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,40+headtres,1900,90,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,20+headtres,1900,95,NOSOFT);
		
		ResetOdometry();	
		Gyro(70,100,0+headtres,1900,85,NOSOFT);
		
		ResetOdometry();
		ResetLift();
		MoveTiangUS(2,ThresUSTiang-3,1200);
		
//		MotorStop();LiftStop();PwmEDF=50;
//		while(1){MotorStop();LiftStop();PwmEDF=50;}
		
		ResetLift();
		MotorStop();
		ChaiYoBiru(-92,NORMAL);

}
