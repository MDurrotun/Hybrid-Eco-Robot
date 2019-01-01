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

void StartBiru(){
		lcd_clear();
		KalibrasiIMU();
		LepasPropeler();
		OdoJalan(5,325,1200,50,50,110,0);
		MotorStop();
		delay_ms(300);
		PwmEDF=81;
		PwmServo=112;
		BelokGyro(90,250);
		MotorStop();
		ResetOdometry();
		OdoJalan(10,85,600,0,81,90,5);
		MotorStop();
		LiftStop();
		PwmServo=90;
		PwmEDF=80;
		delay_ms(200);
		ResetOdometry();
		OdoJalan2(12,55,800,0,68,95,14); // HILL 1
		ResetOdometry();
		BacaUS2();
		corr=0;
		corr = (JarakUS2/10)-18;
		OdoJalan2(12-corr,60,800,0,68,95,14); // HILL 
		ResetOdometry();
		BacaUS2();
		corr = (JarakUS2/10)-15;
		OdoJalan2(35-corr,115,800,0,77,95,20); // LANDAI 1
		MotorStop();
		LiftStop();
		PwmServo=97;
		PwmEDF=68;
		delay_ms(200);
		ResetOdometry();
		BacaUS2();
		corr = (JarakUS2/10)-18;
		OdoJalan2(35-corr,115,800,0,66,95,31); // HILL 2
		ResetOdometry();
		BacaUS2();
		corr = (JarakUS2/10)-20;
		OdoJalan2(10-corr,170,600,0,71,87,40); // Landai 2
		ResetOdometry();
		BacaUS2();
		corr = (JarakUS2/10)-18;
		OdoJalan2(18-corr,135,800,0,67,87,45); // Hill 3
		ResetOdometry();
		BacaUS2();
		corr = (JarakUS2/10)-18;
		OdoJalan2(20-corr,110,150,0,67,45,49); // Zigzag
		PwmEDF=67;
		MotorStop();
		delay_ms(300);
		lcd_clear();
		if(GyroHeading < 90){
					while(GyroHeading < 90 ){
						KiriMaju(40+170);
						KananMundur(40+270);
						lcd_gotoxy(15,1);
						lcd_string("1");						
						lcd_gotoxy(0,1);
						lcd_int16(GyroHeading);
					}
						
		}
		else if(GyroHeading > 90){
					while(GyroHeading > 90){
						KiriMundur(40+170);
						KananMaju(40+270);
						lcd_gotoxy(15,1);
						lcd_string("2");
						lcd_gotoxy(0,1);
						lcd_int16(GyroHeading);
		
					}
		}
		//
		MotorStop();
		delay_ms(100);
		TiupEDF(42,70,700);
		TiupEDF(48,74,1500);
		TiupEDF(51,78,500);
		PwmEDF=50;
		ResetOdometry();
		GyroEDF(45,50,90,800,230);
		LiftStop();
		ResetOdometry();	
		Gyro(18,5,70,1100,245);
		ResetOdometry();	
		Gyro(18,5,30,1100,195);
		MotorStop();
		delay_ms(100);
		BelokGyro(0,100);
		SiapPropeler();
		delay_ms(50);
		ResetOdometry();
		ResetLift();
		MoveTiang(2,25,10,1000);
		LiftStop();
		MotorStop();
		ChaiYo();
}