#include "stm32f4xx.h"
#include "math.h"
#include "define.h"
#include "rotary.h"
#include "motor.h"
#include "global.h"
#include "main.h"
#include "lcd.h"
#include "main.h"
#include "konfigurasi.h"
#include "usart.h"
#include "main.h"


	/*
			Motor Kiri Jalan , PWM>=200
			Motor Kanan Jalan , PWM >=270
	
	*/
float	Distance,Xpos,Ypos,Heading,target_bearing,XposGyro,YposGyro,GyroTheta;
float Theta;
int tanda=0;


void UbahServoMerah(int SudutServo){
		int error,servo;
	
		error = GyroHeading-(-90);
		servo = SudutServo-(error*2);
		DynamixelEDF(servo);
	
		lcd_gotoxy(0,0);
		lcd_int16(error);
	
		lcd_gotoxy(0,1);
		lcd_int16(servo);
	
		lcd_gotoxy(10,0);
		lcd_int16(GyroHeading);
}

void UbahServoBiru(int SudutServo){
		int error,servo;
	
		error = GyroHeading-(90);
		servo = SudutServo+(error*3);
		DynamixelEDF(servo);
	
		lcd_gotoxy(0,0);
		lcd_int16(error);
	
		lcd_gotoxy(0,1);
		lcd_int16(servo);
	
		lcd_gotoxy(10,0);
		lcd_int16(GyroHeading);
}

void SiapLengan(){
		while(LimLengan){
				LiftTurun();
		lcd_gotoxy(0,0);
		lcd_string("Lift Turun");
		}
		LiftStop();
		ResetLift();
		lcd_gotoxy(0,0);
		lcd_string("Lift Siap");
}

void Odometry(){ 
	BacaRotaryBawah();
	RKanan=(float)-(Counter1*R1_tik);
	RKiri=(float)(Counter2*R2_tik);
	
	Distance=((RKanan)+(RKiri))/2;
	Theta = (RKiri-RKanan) / wheelbase;	
	
	if(Theta>3.14f){Theta-=6.28f;}
	else if(Theta<-3.14f){Theta+=6.28f;}
		
	Heading= Theta*RADS;	
	Xpos = Distance * (sin(Theta));
	Ypos = Distance * (cos(Theta));	
}

void Gyrodometry(){ 
	BacaRotaryBawah();
	RKanan=(float)-(Counter1*R1_tik);
	RKiri=(float)-(Counter2*R2_tik);
	
	Distance=((RKanan)+(RKiri))/2;
	GyroTheta		=	GyroHeading / RADS;
	
	if(GyroTheta>3.14f){GyroTheta-=6.28f;}
	else if(GyroTheta<-3.14f){GyroTheta+=6.28f;}
		
	XposGyro = Distance * (sin(GyroTheta));
	YposGyro = Distance * (cos(GyroTheta));	
	
}

void ResetOdometry(void){
	ResetRotaryBawah();
	RKanan=0;
	RKiri=0;
	Distance=0;
	Theta = 0;
	Heading= 0;
	Xpos = 0;
	Ypos = 0;
}

void BelokKiri(int sudut,int speed){
	Odometry();
	KananMaju(speed);
	KiriMundur(speed);
	while(Heading>sudut){
			Odometry();			
			lcd_gotoxy(0,0);
			lcd_float6(Heading);
	}
	KananMundur(speed*0.3f);
	KiriMaju(speed*0.35f);
	delay_ms(300);
	KananStop();
	KiriStop();
}
void BelokKanan(int sudut,int speed){
	Odometry();
	KananMundur(speed);
	KiriMaju(speed);
	while(Heading<sudut){
			Odometry();			
			lcd_gotoxy(0,0);
			lcd_float6(Heading);
	}
	KananStop();
	KiriStop();
}

void BelokGyro(int sudut,int speed){
	static int xxkec;
	
	if(sudut>=0){
		while(GyroHeading<=(sudut)){
				//lcd_gotoxy(0,0);
				//lcd_uint16(GyroHeading;	
				xxkec = speed * ((GyroHeading - sudut) / sudut);
				if(xxkec<350)xxkec=350;
				KananMundur(xxkec+270);
				KiriMaju(xxkec+200);
				
				if(GyroHeading>60 && GyroHeading<=65) 	DynamixelEDF(-20);
		}
		
		KiriMundur((speed*0.8f)+200);
		KananMaju((speed*0.8f)+270);
		delay_ms(10);
		MotorStop();
		MotorStop();
		MotorStop();
		delay_ms(5);
	}
	else if(sudut<0){
		while(GyroHeading>(sudut)){
//				KiriMundur(speed+200);
//				KananMaju(speed+270);
				//lcd_gotoxy(0,0);
				//lcd_uint16(GyroHeading);
				xxkec = speed * ((GyroHeading - sudut) / sudut);
				if(xxkec<350)xxkec=350;
				KananMaju(xxkec+270);
				KiriMundur(xxkec+200);
			
			 if(GyroHeading<-60 && GyroHeading>=-65) 	DynamixelEDF(0);

		}
		KiriMaju((speed*0.8f)+220);
		KananMundur((speed*0.8f)+290);
		delay_ms(10);
		MotorStop();
		MotorStop();
		MotorStop();
		delay_ms(5);
	}
	else{
			MotorStop();MotorStop();MotorStop();

	}
}

void BelokGyros(int sudut,int speed){
	unsigned char flagsudut=0;
	if(sudut>=0){
		while(GyroHeading<=(sudut-30) && flagsudut!=1){
				KananMundur(speed*0.8);
				KiriMaju(speed);
				//lcd_gotoxy(0,0);
				//lcd_uint16(GyroHeading);
			//if(GyroHeading>=sudut-45) speed-=100;
		  if(GyroHeading>40 && GyroHeading<=45) 	DynamixelEDF(30);


		}
		
		KiriMundur(speed);
		KananMaju(speed);
		delay_ms(120);
		MotorStop();
		delay_ms(200);
	}
	else if(sudut<0){
		while(GyroHeading>(sudut) && flagsudut!=1){
				KiriMundur(speed);
				KananMaju(speed*1.2);
				//lcd_gotoxy(0,0);
				//lcd_uint16(GyroHeading);
			 //if(GyroHeading>=-sudut+45) speed-=100;
			 if(GyroHeading<-40 && GyroHeading>=-45) 	DynamixelEDF(0);

		}
		KiriMaju(speed);
		KananMundur(speed);
		delay_ms(100);
		MotorStop();
		delay_ms(200);
	}
	else{
			MotorStop();
	}
	
}

void OdoJalan(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);

					
    while(target_distance >= 20)
    {   
				Odometry();
				BacaLift();
			
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	      if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 2;
        }         
				
        Steering =ErrF * 35 + dErrF * 55;   //5020
	
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF< 1)    TargetSpeedF = 1;
				

        Mka = (int)(TargetSpeedF - Steering)*0.85;
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-Speed) Mka= -Speed/2;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-Speed) Mki=-Speed/2;
					
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				else if (!LimAtas) LiftStop();
				
				PwmEDF = edf;
				
				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(Mki,Mka,Distance,RLift);
				//lcd_gotoxy(14,0);
				//lcd_int16(GyroHeading);
				//TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}



void OdoJalanEDF(float Xt,float Yt,int Speed,int PStop,int edf,int Katrol,int awal, int akhir, int jarak)
{   
		unsigned char flag=1;
		int target_distance=0,us,servoDX,DistanceAwal;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DistanceAwal=target_distance;
		DynamixelEDF(awal);
		BacaUS2();
		pwmtik=0;	
		lcd_clear();
    while((target_distance >= 20))
    {   
				//BacaUS2();
				Odometry();
				BacaLift();
			
//				us = (int) JarakUS2/10;
//				if(us>17) {flag=1;}
//				else {flag=0;}
			
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
				
				if(target_distance<100){
							Speed = 450;  //250
 //						if(Speed<=200)Speed=200;
//						else Speed-= 100; 
				} //////////////////
				
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        			
        Steering =ErrF * 43 + dErrF * 55;   //5020

        Mka = (int)(Speed - Steering)*0.95f;
        Mki = (int)(Speed + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-Speed) Mka= -Speed/2;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-Speed) Mki=-Speed/2;
															
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				else if (!LimAtas) LiftStop();
				
				PwmEDF = edf;				
				//servoDX = ((akhir-awal)/(jarak-DistanceAwal))*(target_distance-DistanceAwal)+awal;
				//if(servoDX>=awal) DynamixelEDF(servoDX);
				
				if(target_distance<50) 			  DynamixelEDF(akhir);				
				else if(target_distance<100)  DynamixelEDF(awal+40);
				else													DynamixelEDF(awal);
				
//				lcd_gotoxy(0,0);
//				lcd_int16(servoDX);
				
				//TampilFloat(Mka,Mki,target_distance,Steering);
			}				
}

void OdoJalanEDFBlue(float Xt,float Yt,int Speed,int PStop,int edf,int Katrol,int awal, int akhir, int jarak,int mode)
{   
		unsigned char flag=1;
		int target_distance=0,us,servoDX,DistanceAwal;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0,xMki=1;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DistanceAwal=target_distance;
		DynamixelEDF(awal);
		pwmtik=0;	
		lcd_clear();
    while((target_distance >= 20))
    {   
				Odometry();
				BacaLift();
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
					
				ErrF = heading_error;
				dErrF = ErrF-LastErrF;
				LastErrF = ErrF;
        
				if(target_distance<70) {
					//if(Speed>=300)Speed -= 100;
					//else 
						Speed=300;
					
					if(mode==RETRY)PwmEDF=50;
					else {PwmEDF = MinEDF+37;}
					
					Steering =ErrF * 3 + dErrF * 5;   //20 35				
				}
				
				else {
					Speed=Speed;  

					if(mode==RETRY)PwmEDF=50;
					else PwmEDF = edf;
						
					Steering =ErrF * 15 + dErrF * 5;   //15 35				
				}
				Mka = (int)(Speed - Steering);
				Mki = (int)(Speed + Steering);
								
				if(Mka>=1329) Mka=1329;
				else if(Mka<-Speed) Mka= -Speed/2;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-Speed) Mki=-Speed/2;
															
				if(Mka>3){KananMaju(Mka+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				else if (!LimAtas) LiftStop();
				
								
				if(target_distance<50) 			  DynamixelEDF(akhir);				
				else if(target_distance<100)  DynamixelEDF(awal+((akhir-awal)/2));
				else													DynamixelEDF(awal);				
				//TampilFloat(Mka,Mki,target_distance,Steering);
			}			
}

void OdoJalan3(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=500,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);					
    while(target_distance >= 20 && flag==0)
    {   
				Odometry();
				BacaUS2();							
				BacaLift();
				PwmEDF = edf;
				
				if(((JarakUS2/10)-20)<0){
					x = (Xt - (((JarakUS2/10)-20)/2)) - Xpos;				
				}
				else {
					x = (Xt - ((JarakUS2/10)-20)) - Xpos;				
				}
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 60;
        } 

        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
								
				if(Speed<300) Steering =ErrF * 10 + dErrF * 5; //.15  10
				//else     Steering =ErrF * 32 + dErrF * 6;   		
				else     Steering =ErrF * 20 + dErrF * 30;   
				
      
				
        Mka = (int)(TargetSpeedF - Steering)*0.68f;
        Mki = (int)(TargetSpeedF + Steering);

				if(Mka>=1729) Mka=1729;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1729) Mki=1729;
				else if(Mki<-3) Mki=-3;
				
//				if(Mka>=1999) Mka=1999;
//				else if(Mka<-3) Mka= -3;
//				
//				if(Mki>=1999) Mki=1999;
//				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(RLift,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}


void CobaOdoJalan3(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);					
		pwmtik=0;
    while(target_distance >= 20 && flag==0)
    {   
				Odometry();
				BacaUS1();							
				BacaLift();
				PwmEDF = edf;
				
				x = (Xt + (((JarakUS1/10)-20)/2)) - Xpos;				
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
				
        if(ErrF>40)         {Speed += (float)( - ErrF)/100;}
        else if(ErrF<-40)   {Speed += (float)(  ErrF)/100;}
				else{
					if(target_distance < PStop)  Speed -= 25;
				}
					
				Steering =ErrF * 15 + dErrF * 7;   //50 20.  35 35
				
        //Mka = (int)(TargetSpeedF - Steering)*0.85f;
				Mka = (int)(Speed - Steering);
        Mki = (int)(Speed + Steering);

				
				if(Mka>=1999) Mka=1999;
				else if(Mka<0) Mka= 0;
				
				if(Mki>=1999) Mki=1999;
				else if(Mki<0) Mki=0;
														
				if(pwmtik % 15 ==0){
						if(Mka>0){KananMaju(Mka);}
						//else if(Mka<0){KananMundur((Mka*=-1));}
						else KananStop();
			
						if(Mki>0){KiriMaju(Mki);}
						//else if(Mki<0){KiriMundur((Mki*=-1));}
						else KiriStop();
				}				
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(JarakUS2,Mka,Xpos,Ypos);
					

				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void OdoJalan3Rev(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=500,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);					
    while(target_distance >= 20 && flag==0)
    {   
				Odometry();
				BacaUS2();							
				BacaLift();
				PwmEDF = edf;
				
				if(((JarakUS2/10)-20)<0){
					x = (Xt - (((JarakUS2/10)-20)/2)) - Xpos;				
				}
				else {
					x = (Xt - ((JarakUS2/10)-20)) - Xpos;				
				}
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 60;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
								
				if(Speed<300) Steering =ErrF * 15 + dErrF * 6;
				else     Steering =ErrF * 32 + dErrF * 6;   //50 20.  35 35
				//else     Steering =ErrF * 25 + dErrF * 10;   //50 20.  35 35
				
      
				
        Mka = (int)(TargetSpeedF - Steering)*0.68f;
        Mki = (int)(TargetSpeedF + Steering);

				if(Mka>=1729) Mka=1729;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1729) Mki=1729;
				else if(Mki<-3) Mki=-3;
				
//				if(Mka>=1999) Mka=1999;
//				else if(Mka<-3) Mka= -3;
//				
//				if(Mki>=1999) Mki=1999;
//				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(JarakUS2,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void OdoJalan4(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=500,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo); DynamixelEDF(servo); DynamixelEDF(servo);					
    while(target_distance >= 20 && flag==0)
    {   
				Odometry();
				BacaUS1();							
				BacaLift();
				PwmEDF = edf;
				
				x = (Xt + (((JarakUS1/10)-20)/2)) - Xpos;				
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 25;
					  else                         TargetSpeedF += 60;

            //else                         TargetSpeedF += 40;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				
				if(Speed<300) Steering =ErrF * 15 + dErrF * 6;
				else     Steering =ErrF * 32 + dErrF * 6;   //50 20.  35 35
				
        Mka = (int)(TargetSpeedF - Steering)*0.85f;
        Mki = (int)(TargetSpeedF + Steering);

				if(Mka>=1729) Mka=1729;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1729) Mki=1729;
				else if(Mki<-3) Mki=-3;
				
//				if(Mka>=1999) Mka=1999;
//				else if(Mka<-3) Mka= -3;
//				
//				if(Mki>=1999) Mki=1999;
//				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();;
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(JarakUS2,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void CobaOdoJalan4(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);					
		//pwmtik=0;
    while(target_distance >= 10 && flag==0)
    {   
				BacaUS1();		
				Odometry();						
				BacaLift();
				PwmEDF = edf;			
				x = (Xt + (((JarakUS1/10)-20)/2)) - Xpos;				
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
				
        if(ErrF>40)         {Speed += (float)( - ErrF)/100;}
        else if(ErrF<-40)   {Speed += (float)(  ErrF)/100;}
				else{
					if(target_distance < PStop)  Speed -= 25;
				}
					
				Steering =ErrF * 35 + dErrF * 25;   //50 20.  35 35
				
        //Mka = (int)(TargetSpeedF - Steering)*0.85f;
				Mka = (int)(Speed - Steering);
        Mki = (int)(Speed + Steering)*0.7f;

				
				if(Mka>=1999) Mka=1999;
				else if(Mka<0) Mka= 0;
				
				if(Mki>=1999) Mki=1999;
				else if(Mki<0) Mki=0;
														
				//if(pwmtik >= 5){
						pwmtik=0;
					
						if(Mka>0){KananMaju(Mka);}
						//else if(Mka<0){KananMundur((Mka*=-1));}
						else KananStop();
			
						if(Mki>0){KiriMaju(Mki);}
						//else if(Mki<0){KiriMundur((Mki*=-1));}
						else KiriStop();
						
				//}				
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(JarakUS2,target_distance,Xpos,Ypos);
					

				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}


void OdoJalan4Rev(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=180,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);					
    while(target_distance >= 20 && flag==0)
    {   
				Odometry();
				BacaUS1();							
				BacaLift();
				PwmEDF = edf;
				
				if(target_distance>40){
					x = (Xt + (((JarakUS1/10)-20)/2)) - Xpos;				
				}
				else{
					x = (Xt + (((JarakUS1/10)-20)/4)) - Xpos;
				}
				
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error ;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 40;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				
				if(Speed<300) Steering =ErrF * 15 + dErrF * 6;
				else     Steering =ErrF * 32 + dErrF * 6;   //50 20.  35 35
				
        Mka = (int)(TargetSpeedF - Steering)*0.85f;
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				
				if (!LimAtas) LiftStop();

				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(JarakUS2,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}



void OdoJalan5(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		unsigned char flag=0;
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=180,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		flag=0;
		DynamixelEDF(servo);

    while(target_distance >= 20 && flag==0)
    {   
				PwmEDF = edf;
				Odometry();
				BacaLift();
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 50;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				
				
        Steering =ErrF * 35 + dErrF * 35;   //5020
				
        Mka = (int)(TargetSpeedF - Steering)*0.94;
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+270);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<22)LiftNaik();
				else if(target_distance<80 && target_distance>60 && RLift<27   ){
						LiftNaik();BacaLift();
				}
				else if(RLift>=22 || flag==1) LiftStop();
				else if (!LimAtas) LiftStop();
				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				TampilFloat(Mki,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void OdoJalan2(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=180,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
		DynamixelEDF(servo);
			
    while(target_distance >= 20)
    {   
				PwmEDF = edf;

				//PwmServo = servo;
				Odometry();
				BacaLift();
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 40;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				
				
        Steering =ErrF * 35 + dErrF * 35;   //5020
				
        Mka = (int)(TargetSpeedF - Steering);
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+300);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+300);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				else if (!LimAtas) LiftStop();

				
				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				//TampilFloat(Mki,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void OdoHillZigzag(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=180,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		Odometry();
		LiftStop();
	
    x = Xt - Xpos;
    y = Yt - Ypos;
		target_distance = (int)(sqrt((x*x)+(y*y)));
					
    while(target_distance >= 20)
    {   
				PwmEDF = edf;
				//PwmServo = servo;
				if(target_distance>=70) 	DynamixelEDF(20);
				else if(target_distance<=70) DynamixelEDF(100);

				Odometry();
				BacaLift();
        x = Xt - Xpos;
        y = Yt - Ypos;
			
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
			
				heading_error = target_bearing - Heading;
				if (heading_error > 180.0f) heading_error -= 360.0f;
				else if (heading_error < -180.0f) heading_error += 360.0f;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 30;
        } 

					
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				
				
        Steering =ErrF * 35 + dErrF * 35;   //5020
				
        Mka = (int)(TargetSpeedF - Steering);
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
										
				if(Mka>3){KananMaju(Mka+270);}
				//else if(Mka<-3){KananMundur((Mka*=-1)+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+200);}
				//else if(Mki<-3){KiriMundur((Mki*=-1)+200);}
				else KiriStop();
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				else if (!LimAtas) LiftStop();

				
				if(!MerahTengah){STOP;PwmEDF=50;while(1){}}
				//TampilFloat(Mki,Mka,Xpos,Ypos);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void GyrodoJalan(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol)
{   
		int target_distance=0;
		static int Mka,Mki;
    float Steering =0,TargetSpeedF=0,LastErrF=0,ErrF=0,dErrF=0,x=0,y=0,heading_error=0;
		LiftStop();
		Gyrodometry();			
		x = Xt - XposGyro;
    y = Yt - YposGyro;
		target_distance = (int)(sqrt((x*x)+(y*y)));
    while(target_distance >= 20)
    {   
				Gyrodometry();
				BacaLift();
        x = Xt - XposGyro;
        y = Yt - YposGyro;
			
				
				if (x > 0.00001f)        target_bearing = 90.0f - (180/PI) * atan(y/x);
				else if (x < -0.00001f)  target_bearing = -90.0f -(180/PI) * atan(y/x);
				else {target_bearing=0;}
				
				
				heading_error = target_bearing - GyroHeading;
				//if (heading_error > 180.0) heading_error -= 360.0;
				//else if (heading_error < -180.0) heading_error += 360.0;
    
				target_distance = (int)(sqrt((x*x)+(y*y)));
        ErrF = heading_error;
        dErrF = ErrF-LastErrF;
        LastErrF = ErrF;
        
	     	if(ErrF>40)         {TargetSpeedF -=  ErrF/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else if(ErrF<-40)	  {TargetSpeedF -=  (ErrF*-1)/100; if(TargetSpeedF<=235) TargetSpeedF=235;}
				else{   
            if(target_distance < PStop)  TargetSpeedF -= 30;
            else                         TargetSpeedF += 3;
        }         
				
        Steering =ErrF * 40 + dErrF * 70;   //5020
	
        if((int)TargetSpeedF> Speed )   TargetSpeedF = (float)Speed;
        else if((int)TargetSpeedF<= 2)    TargetSpeedF = 2;
				

        Mka = (int)(TargetSpeedF - Steering);
        Mki = (int)(TargetSpeedF + Steering);
						
				if(Mka>=1329) Mka=1329;
				else if(Mka<-3) Mka= -3;
				
				if(Mki>=1399) Mki=1399;
				else if(Mki<-3) Mki=-3;
					
				if(Mka>3){KananMaju(Mka+270);}
				else KananStop();
	
				if(Mki>3){KiriMaju(Mki+200);}
				else KiriStop();
				
				/*
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<Katrol)LiftNaik();
				else if(RLift>=Katrol) LiftStop();
				*/

				/*
				PwmEDF = edf;
				PwmServo = servo;
				*/
				
				if(!MerahTengah){STOP;while(1){}}
				TampilInt(x,y,target_distance,YposGyro);
				//	TampilFloat(Xpos,Ypos,Xt,Yt);
				//TampilFloat(X,Mki,target_distance,Steering);
		}
}

void PanjatTiang(){
//				JepitTutup();
//				delay_ms(300);
//				while(ReadADC(13)<2027){
//				
//				}
//				JepitStop();
				PemanjatNaik();
				while(LimNaik){}
				PemanjatStop();
}

void ClimbControl(int Kp,int Kd,int sudut,int mode){
		int Error,DError,LastError,speed,PD,speedMAX;
							
		Error			=	-GyroHeading+sudut;
		
		DError		=	Error - LastError;
		LastError = Error;
		PD				=	(Error*Kp) + (DError*Kd);
		speed			=	50 - PD;
		
		if(mode==RETRY)speedMAX=70;
		else speedMAX=90;
	
		if(speed>=speedMAX) speed=speedMAX;
		DynamixelEDF(550);
		PwmEDF=speed;	
	
//		if(!LimTutup){LenganStop();}
//		else{	Ena1_0;Dir1_1;}
		
}

void ClimbControlBiru(int Kp,int Kd,int sudut,int mode){
		int Error,DError,LastError,speed,PD,speedMAX;
		
		
		Error			=	sudut+GyroHeading;
		
		DError		=	LastError - Error;
		LastError = Error;
	
		PD				=	(Error*Kp) + (DError*Kd);
		speed			=	50 + PD;

		if(mode==RETRY)speedMAX=75;
		else speedMAX=110;
	
		if(speed>=speedMAX) speed=speedMAX;
		PwmEDF=speed;	
		if(!LimTutup){LenganStop();}
		DynamixelEDF(30);
}

void ClimbControlBiruReverse(int Kp,int Kd,int sudut,int mode){
		int Error,DError,LastError,speed,PD,speedMAX;
		
		
		Error			=	-sudut+GyroHeading;;
		
		DError		=	LastError - Error;
		LastError = Error;
	
		PD				=	(Error*Kp) + (DError*Kd);
		speed			=	50 - PD;

		if(mode==RETRY)speedMAX=80;
		else speedMAX=85;
	
		if(speed>=speedMAX) speed=speedMAX;
		DynamixelEDF(550);
		PwmEDF=speed;	
}



void MoveGyroRe(int Kp, int Kd, int sudut, int kec,int jarak,int PStop){
	int Error,LastError,dError,PID,kec_ki,kec_ka,SoftStart=50,Target=0;
	unsigned char flaglift=0;
	Distance=0;
	Target = jarak - Distance;
	while(Distance<jarak){
		Target = jarak - Distance;
		
		Odometry();
		BacaLift();
		
		if(Target < PStop ){
				if(SoftStart<= 400) SoftStart=400;
				else SoftStart-=20;
		}
		else SoftStart+=4;
		
		if(SoftStart>=kec) SoftStart=kec;
		else if(SoftStart<=5) SoftStart=5;
		
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = SoftStart - PID;
		kec_ka = SoftStart + PID;
		
		if(!MerahTengah){MotorStop();LiftStop();while(1){}}
	
		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka-90);
		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
				
		RLift = (float) - (Counter3*R3_tik);
		if(RLift<49 && flaglift==1)LiftNaik();
		else {
				LiftStop();
				if(Target<350)flaglift=1;
				else flaglift=0;
		}
//		else if(RLift>=49) {LiftStop();flaglift=1;}
//		else if (!LimAtas) {LiftStop();flaglift=1;}
			
		TampilInt(GyroHeading,Target,SoftStart,RLift);
	}
}

void MoveGyroRe2(int Kp, int Kd, int sudut, int kec,int jarak,int PStop){
	int Error,LastError,dError,PID,kec_ki,kec_ka,SoftStart=50,Target=0;
	unsigned char flaglift=0;
	Distance=0;
	Target = jarak - Distance;
	while(Distance < jarak){
		Target = jarak - Distance;
		
		Odometry();
		BacaLift();
		
		if(Target < PStop ){
				if(SoftStart<= 400) SoftStart=400;
				else SoftStart-=20;
		}
		else SoftStart+=50;
		
		if(SoftStart>=kec) SoftStart=kec;
		else if(SoftStart<=5) SoftStart=5;
		
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = SoftStart - PID;
		kec_ka = (SoftStart + PID)*0.75f;
		
	
		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur(kec_ka*-1);
		//else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
				
		RLift = (float) - (Counter3*R3_tik);
		if(RLift<=50 && flaglift==1)LiftNaik();
		else {
				LiftStop();
				if(Target<350)flaglift=1;
				else flaglift=0;
		}
//		else if(RLift>=49) {LiftStop();flaglift=1;}
//		else if (!LimAtas) {LiftStop();flaglift=1;}
		DynamixelEDF(310);	
		TampilInt(GyroHeading,Target,SoftStart,RLift);
	}
}
void MoveGyroRe3(int Kp, int Kd, int sudut, int kec,int PStop){
	int Error,LastError,dError,PID,kec_ki,kec_ka,SoftStart=50,Target=0;
	unsigned char flaglift=0;
	BacaUS2();
	while(JarakUS2<=350){
		BacaUS2();
		BacaLift();
		
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = SoftStart - PID;
		kec_ka = (SoftStart + PID)*0.93;
		
	
		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur(kec_ka*-1);
		//else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
				
		RLift = (float) - (Counter3*R3_tik);
		if(RLift<49 && flaglift==1)LiftNaik();
		else {
				LiftStop();
				if(Target<350)flaglift=1;
				else flaglift=0;
		}
//		else if(RLift>=49) {LiftStop();flaglift=1;}
//		else if (!LimAtas) {LiftStop();flaglift=1;}
		DynamixelEDF(310);	
		TampilInt(GyroHeading,Target,SoftStart,RLift);
	}
}



//void MoveGyro(int Kp, int Kd, int sudut, int kec,int jarak){
//	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1599,SoftStart=50,target=0;
//	Distance=0;
//	while(Distance<jarak){
//		Odometry();
//		target = jarak - Distance;
//				
//		if(SoftStart<kec) SoftStart+=4;
//				
//		else SoftStart=kec;
//		
//		Error = GyroHeading - sudut; // -90-(-90)=0
//		dError = Error - LastError;
//		LastError = Error;
//		
//		PID = (Error*Kp)+(dError*Kd);
//		
//		kec_ki = SoftStart - PID;
//		kec_ka = SoftStart + PID;
//		
//		if(!MerahTengah){MotorStop();while(1){}}
//		
//		if(kec_ki>=kec_max) kec_ki=kec_max;
//		if(kec_ka>=kec_max) kec_ka=kec_max;

//		if(kec_ki>kec) KiriMaju(kec_ki);
//		else if(kec_ki<0) KiriMundur(-1*kec_ki);
//		else if(kec_ki<200) KiriMundur(kec_ki);
//		else KiriMaju(kec_ki);
//		
//		if(kec_ka>kec) KananMaju(kec_ka-90);
//		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
//		else if(kec_ka<270) KananMundur(kec_ka);
//		else KananMaju(kec_ka);
//				
//		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
//	}
//}
void MoveGyro(int Kp, int Kd, int sudut, int kec,int jarak){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1900,SoftStart=600,target=0;
	Distance=0;

	while(Distance<jarak){
		Odometry();
		LiftStop();
		JepitStop();
		
		target = jarak - Distance;
				
		if(target <= (0.1f * jarak)) 	SoftStart-=10;
		else{ 													
				if(SoftStart>=kec) SoftStart=kec;
				else					  	 SoftStart+=4;
		}
			
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = SoftStart - PID;
		kec_ka =(int) (SoftStart + PID) * 0.85f;
		
		if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		else if(kec_ki<50) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka-90);
		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		else if(kec_ka<60) KananMundur(kec_ka);
		else KananMaju(kec_ka);
				
		//TampilInt(GyroHeading,Distance,kec_ka,kec_ki);
		TampilFloat(kec_ki,kec_ka,Distance,Distance);
	}

}
void MoveGyroLurus(int Kp, int Kd, int sudut, int kec){
	int Error,LastError,dError,PID,kec_ki,kec_ka;
	static int kec_max=400,xDist;
	
	Odometry();	
	xDist=(int) Distance; 
	if(xDist%=5 && kec_max>kec)kec_max+=50;
	
	Error = GyroHeading - sudut; 
	dError = Error - LastError;
	LastError = Error;
		
	PID = (Error*Kp)+(dError*Kd);
		
	kec_ki = kec_max - PID;
	kec_ka =(int) (kec_max + PID) * 0.9f;
				
	if(kec_ki>kec_max) KiriMaju(kec_max);
	else if(kec_ki<0) KiriMundur(-1*kec_ki);
	else KiriMaju(kec_ki);
		
	if(kec_ka>kec_max) KananMaju(kec_max);
	else if(kec_ka<0) KananMundur(kec_ka*-1);
	else KananMaju(kec_ka);			 
	
}

void MoveGyro2(int Kp, int Kd, int sudut, int kec,int jarak){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1990,SoftStart=50;
	int aaa=0;
	Distance=0;
	while(Distance<jarak){
		Odometry();
		
		aaa=(jarak-Distance)/jarak *100;
		if(aaa<30) aaa=40;
		if(aaa>=30) aaa=100;
		
		if(SoftStart<kec) SoftStart+=4;
		else SoftStart=kec;
				
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = SoftStart - PID;
		kec_ka = SoftStart + PID*0.94f;
		
		kec_ki=(kec_ki/100) * aaa;
		kec_ka=(kec_ka/100) * aaa;
		
		if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur((kec_ka*-1));
		//else if(kec_ka<0) KananMundur(kec_ka);
		else KananMaju(kec_ka);
				
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}
void GyroTiang(int Kp, int Kd, int sudut, int kec){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1100;
	unsigned char flag=0;
	while(LimTiang && flag==0){
		Odometry();
		Error = GyroHeading - sudut; // -90-(-90)=0
		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = kec + PID;
		
		if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka-90);
		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
		
		if(!LimTiang) flag=1;
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void GyroUS(int Kp, int Kd, int sudut, int kec){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1100,flag=0,i=0;
	
	BacaUS2();
	i = JarakUS2/10;
	
	while(i>25&&flag==0){
		BacaUS2();
	  i = JarakUS2/10;
		Odometry();
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = kec + PID;
		
		if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka-90);
		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
			
		if(i<=25){flag=1;}
		
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
	MotorStop();
}

void Gyro(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1900,target=0;
	unsigned char limit=1;
	Distance=0;
	while(Distance<jarak){
		LiftStop();
		JepitStop();
		Odometry();
				
		target = jarak - Distance;
		if(target == (0.1*jarak) && brake!=NOSOFT) kec -=20;
		else kec = kec;
		
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = (kec + PID)*0.8;
		//if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur((kec_ka*-1));
		else KananMaju(kec_ka);
		
		if(LimLengan) limit=1;
		if(!LimLengan)limit=0;
		
		if(limit==1) LiftTurun();
		else if(limit==0) LiftStop();
		
		if(LimTutup) LenganKeluar();
		else				 LenganStop();
		
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void GyroEDF(int Kp, int Kd, int sudut, int kec,int jarak){
	unsigned char limit=0;
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1900,target=0,Sampling=0,LastDistance=50;
	Distance=0;
	PwmEDF=50;
	lcd_clear();
	while(Distance<jarak){
		Odometry();
		LiftStop();
		JepitStop();
		PwmEDF=50;
		
		if(Distance<=80 ){
				kec+=2;
				//DynamixelEDF(300);
				//PwmEDF=MinEDF+75;
		}		
		else{
				kec += 4;
				if(LimLengan) limit=1;
				//DynamixelEDF(-80);
				//PwmEDF=50;
		}
		
		if(!LimLengan){limit=0;}
		
		if(LimTutup) LenganKeluar();
		else				 LenganStop();

		if(limit==1) LiftTurun();
		else if(limit==0) LiftStop();
		
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = (kec + PID)*0.85;
		
		//if(!MerahTengah){MotorStop();LiftStop();while(1){}}
		

		if(kec_ki>kec_max) KiriMaju(kec_max);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec_max) KananMaju(kec_max);
		else if(kec_ka<0) KananMundur(kec_ka*-1);
		//else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
		
		lcd_gotoxy(0,0);
		lcd_string("Gyro EDF");
		
		//TampilInt(GyroHeading,Error,kec_ka,kec_ki);
			

	}
}

void GyroPutar(int Kp, int Kd, int edf, int servo, int Katrol, int sudut, int kec,int jarak,unsigned char brake){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1000,target=0;
	Distance=0;
	LiftStop();
	
	DynamixelEDF(servo); 
	while(Distance<jarak){
		DynamixelEDF(servo); 
		Odometry();
		BacaLift();
				
//		target = jarak - Distance;
//		if(target == (0.1*jarak) && brake!=NOSOFT) kec -=20;
//		else kec = kec;
		
		if(brake==SOFT&&Distance<=20){ kec+=3;}
		else kec+=3;
		
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = (kec + PID)*0.8;
		//if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur((kec_ka*-1));
		else KananMaju(kec_ka);	
		
		RLift = (float) -(Counter3*R3_tik);
		if(RLift<Katrol)LiftNaik();
		else if(RLift>=Katrol) LiftStop();
		else if (!LimAtas) LiftStop();
		
		PwmEDF = edf;
		
		//else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		//else if(kec_ka<270) KananMundur(kec_ka);
		
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void Gyro2(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1900,target=0;
	unsigned char limit=1;
	Distance=0;
	lcd_clear();
	while(Distance<jarak){
		LiftStop();
		Odometry();
		PwmEDF=50;
		JepitStop();
		
		if(LimLengan) limit=1;
		else if(!LimLengan)limit=0;
		
		if(limit==1) LiftTurun();
		else if(limit==0) LiftStop();
		
		target = jarak - Distance;
		if(target == (0.1*jarak) && brake!=NOSOFT) kec -=20;
		else kec = kec;
		
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = (kec + PID)*0.85;
		
		//if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur((kec_ka*-1));
			
		//else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		//else if(kec_ka<270) KananMundur(kec_ka);
		else KananMaju(kec_ka);
		
		if(LimTutup) LenganKeluar();
		else				 LenganStop();
		
		lcd_gotoxy(0,0);
		lcd_string("Gyro 2");
		//TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void GyroHill(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake, int edf, int servo, int katrol){
	int Error,LastError,dError,PID,kec_ki,kec_ka,kec_max=1700,target=0,Last_Distance=0,Result=0;
	unsigned char limit=1;
	Distance=0;
	DynamixelEDF(servo);
	
	while(Distance<jarak){
		LiftStop();
		Odometry();
		PwmEDF=edf;
		JepitStop();
		BacaLift();
		
//		Result = Distance - Last_Distance;
//		if(Result==10){
//				Result=0;
//				Last_Distance = Distance;
//				if(servo<=300) servo+=50;
//				DynamixelEDF(servo); DynamixelEDF(servo); DynamixelEDF(servo);
//		}
		
		if(servo<=350){
				servo+=2;
				DynamixelEDF(servo);
		}
		
		if(Distance<=30){sudut = -110;}
				
		target = jarak - Distance;
		if(target == (0.1*jarak) && brake!=NOSOFT) kec -=20;
		else kec = kec;
		
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		PID = (Error*Kp)+(dError*Kd);
		
		kec_ki = kec - PID;
		kec_ka = (kec + PID)*0.85;
		
		//if(!MerahTengah){MotorStop();while(1){}}
		
		if(kec_ki>=kec_max) kec_ki=kec_max;
		if(kec_ka>=kec_max) kec_ka=kec_max;

		if(kec_ki>kec) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		//else if(kec_ki<200) KiriMundur(kec_ki);
		else KiriMaju(kec_ki);
		
		if(kec_ka>kec) KananMaju(kec_ka);
		else if(kec_ka<0) KananMundur((kec_ka*-1));
		else KananMaju(kec_ka);
		
		RLift = (float) -(Counter3*R3_tik);
		if(RLift<katrol)LiftNaik();
		else if(RLift>=katrol) LiftStop();
				
		if (!LimAtas) LiftStop();
		
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void MoveTiang2(unsigned char trace,int threshold,int dista,int kec){
		int kec_ka,kec_ki,PDWall,kp=0,kd=0;
		int ErrorWall,DErrorWall;
		int LastErrorWall,SpeedMax=800;
		unsigned char flg=0;
		uint16_t us;	
		Distance=0;
		kp=22;
		kd=15;
		while(Distance<dista && flg==0){
				Odometry();
				BacaLift();
				if(trace==1) {
						BacaUS2();
						us = JarakUS2;
				}
				else if(trace==2) {
						BacaUS1();
						us = JarakUS1;
				}
				

				
				ErrorWall			=	(us/10) - threshold;
				if(ErrorWall > 25) {ErrorWall=25;	kp=15; kd=7;}
				else if(ErrorWall>=20 && ErrorWall<25){kp=15; kd=7;}
				else if(ErrorWall>=15 && ErrorWall<20){kp=21; kd=14;}
				else if(ErrorWall>=10 && ErrorWall<15){kp=25; kd=21;}
				else if(ErrorWall>=5  && ErrorWall<10){kp=27; kd=40;}
				else{
						kp=30;
						kd=60;
				}
				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				

				kec_ka				= (kec + PDWall)*0.95;
				kec_ki				=	kec - PDWall;

				if(kec_ki>=SpeedMax) kec_ki=SpeedMax;
				if(kec_ka>=SpeedMax) kec_ka=SpeedMax;	
				
				if(kec_ki>kec) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				else if(kec_ki<200) KiriMundur(kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>kec) KananMaju(kec_ka-90);
				else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
				else if(kec_ka<270) KananMundur(kec_ka);
				else KananMaju(kec_ka);
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<45)LiftNaik();
				else if(RLift>=45) LiftStop();
				
				if(dista<Distance) flg=1;
				if(!MerahTengah){MotorStop();LiftStop();while(1){}}
					
				TampilInt(JarakUS2,GyroHeading,kec_ki,kec_ka);
		}
}


void MoveTiang(unsigned char trace,int threshold,int dista,int kec){
		int kec_ka,kec_ki,PDWall,kp=0,kd=0;
		int ErrorWall,DErrorWall;
		int LastErrorWall,SpeedMax=800;
		unsigned char flg=0;
		uint16_t us;	
		Distance=0;
		kp=22;
		kd=15;
		while(Distance<dista && flg==0){
				Odometry();
				BacaLift();
				if(trace==1) {
						BacaUS2();
						us = JarakUS2;
				}
				else if(trace==2) {
						BacaUS1();
						us = JarakUS1;
				}
				
				ErrorWall			=	(us/10) - threshold;
				if(ErrorWall > 25) {ErrorWall=25;	kp=15; kd=7;}
				else if(ErrorWall>=20 && ErrorWall<25){kp=15; kd=7;}
				else if(ErrorWall>=15 && ErrorWall<20){kp=21; kd=14;}
				else if(ErrorWall>=10 && ErrorWall<15){kp=25; kd=21;}
				else if(ErrorWall>=5  && ErrorWall<10){kp=27; kd=40;}
				else{
						kp=30;
						kd=60;
				}
				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				
					kec_ka				= (kec - PDWall)*0.95;
					kec_ki				=	(kec + PDWall);

				if(kec_ki>=SpeedMax) kec_ki=SpeedMax;
				if(kec_ka>=SpeedMax) kec_ka=SpeedMax;	
				
				if(kec_ki>kec) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				//else if(kec_ki<200) KiriMundur(kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>kec) KananMaju(kec_ka-90);
				else if(kec_ka<0) KananMundur((kec_ka*-1));
				//else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
				//else if(kec_ka<270) KananMundur(kec_ka);
				else KananMaju(kec_ka);
				
				RLift = (float) -(Counter3*R3_tik);
				if(RLift<45)LiftNaik();
				else if(RLift>=45) LiftStop();
				
				if(dista<Distance) flg=1;
				if(!MerahTengah){MotorStop();LiftStop();while(1){}}
					
				TampilInt(JarakUS2,JarakUS1,kec_ki,kec_ka);
		}
}


void MoveTiangUS(unsigned char trace,int threshold,int kec){
		int kec_ka,kec_ki,kp=0,kd=0;
		unsigned char limit=0;
		float PDWall,ErrorWall=0,DErrorWall=0,LastErrorWall=0;
		int SpeedMax=1900;
		unsigned char flg=0;
		float us;	
		Distance=0;
		kp=20;
		kd=40;
	
		SetDynamixel(2,30,3100); SetDynamixel(2,30,3100); SetDynamixel(2,30,3100);
		while(LimTiang && flg==0){
				Odometry();
				LiftStop();
				JepitStop();
			
				BacaLift();
				if(trace==1) {
						BacaUS2();
						us =  JarakUS2;
				}
				else if(trace==2) {
						BacaUS1();
						us = JarakUS1;
				}
				
				ErrorWall			=	(us/10) - threshold;
				
				if(ErrorWall >= 25) {		
						//ErrorWall=25;	kp=1; kd=10;	
						KiriMaju(600);			KananMaju(400);
				}
				else{
						if(Distance>=90)																																	{kp=40 ; kd = 55;}
						else if((ErrorWall>=10 && ErrorWall<15) || (ErrorWall<=-10 && ErrorWall>-15))			{kp=45; kd=50;}
						else if((ErrorWall>=5  && ErrorWall<10) || (ErrorWall<=-5  && ErrorWall>-10))			{kp=40; kd=40;}
						else if((ErrorWall>=0  && ErrorWall<5) || (ErrorWall<=0  && ErrorWall>-5))				{kp=35; kd=35;}
						else  																																						{kp=40; kd=80;}
						
				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				

				kec_ka				= (int) (kec - PDWall)*0.90f;
				kec_ki				=	(int) kec + PDWall;

				if(kec_ki>=SpeedMax) kec_ki=SpeedMax;
				if(kec_ka>=SpeedMax) kec_ka=SpeedMax;	
				
				if(kec_ki>kec) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				//else if(kec_ki<200) KiriMundur(kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>kec) KananMaju(kec_ka);
				else if(kec_ka<0) KananMundur(kec_ka*-1);
				//else if(kec_ka<270) KananMundur(kec_ka);
				else KananMaju(kec_ka);
				
//				RLift = (float) -(Counter3*R3_tik);
//				if(RLift<50)LiftNaik();
//				else if(RLift>=50 || !LimAtas) LiftStop();
				if(LimTutup) LenganKeluar();
				else				 LenganStop();
		
				if(Distance>=30){kec=350;}
				
				if(!LimTiang){ flg=1; MotorStop(); }
				if(!MerahTengah){MotorStop();LiftStop();while(1){}}
					
				TampilInt(JarakUS2,GyroHeading,kec_ki,kec_ka);
				}
		}
}
void MoveTiangUSRed(unsigned char trace,int threshold,int kec){
		int kec_ka,kec_ki,kp=0,kd=0;
		float PDWall,ErrorWall=0,DErrorWall=0,LastErrorWall=0;
		int SpeedMax=kec+300;//,SoftStart=300,kecs=0;
		unsigned char flg=0;
		float us;	
		kp=20;
		kd=40;
		Distance=0;
	
	 SetDynamixel(2,30,3100); SetDynamixel(2,30,3100); SetDynamixel(2,30,3100); 
	
		while(LimTiang && flg==0){
				Odometry();
				BacaLift();
				LiftStop();
				PwmEDF=50;
				JepitStop();
			
				if(trace==1) {
						BacaUS2();
						us =  JarakUS2;
				}
				else if(trace==2) {
						BacaUS1();
					
						us = JarakUS1;
				}
				
//				if(SoftStart<kec) SoftStart+=30;
//				else							 SoftStart=kec;
				
				//kecs=SoftStart;
				
				ErrorWall			=	(us/10) - threshold;
//				if(ErrorWall > 25) {ErrorWall=25;	kp=15; kd=7;}
//				else if(ErrorWall>=20 && ErrorWall<25){kp=15; kd=7;}
//				else if(ErrorWall>=15 && ErrorWall<20){kp=21; kd=12;}
//				else if(ErrorWall>=10 && ErrorWall<15){kp=25; kd=15;}
//				else if(ErrorWall>=5  && ErrorWall<10){kp=27; kd=17;}
//				else{
//						kp=22;
//						kd=4;
//				}
				if(ErrorWall >= 25) {		
						//ErrorWall=25;	kp=1; kd=10;	
						KiriMaju(400);			KananMaju(600);
				}
				else{
						if(Distance>=50)																																	{kp=40 ; kd = 70;}
						else if((ErrorWall>=10 && ErrorWall<15) || (ErrorWall<=-10 && ErrorWall>-15))			{kp=45; kd=50;}
						else if((ErrorWall>=5  && ErrorWall<10) || (ErrorWall<=-5  && ErrorWall>-10))			{kp=40; kd=40;}
						else if((ErrorWall>=0  && ErrorWall<5) || (ErrorWall<=0  && ErrorWall>-5))				{kp=35; kd=35;}
						else  																																						{kp=25; kd=60;}
						//					else  																																						{kp=40; kd=80;}

					
//						if(Distance>=155)																																		{kp=40 ; kd = 55;}
//						else if((ErrorWall>=10 && ErrorWall<15) || (ErrorWall<=-10 && ErrorWall>-15))			{kp=35; kd=50;}
//						else if((ErrorWall>=5  && ErrorWall<10) || (ErrorWall<=-5  && ErrorWall>-10))			{kp=33; kd=40;}
//						else if((ErrorWall>=0  && ErrorWall<5) || (ErrorWall<=0  && ErrorWall>-5))				{kp=30; kd=35;}
//						else  																																						{kp=35; kd=70;}
				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				
				kec_ka				= (int) (kec + PDWall);

//				kec_ka				= (int) (kec + PDWall)*0.95f;
				kec_ki				=	(int) kec - PDWall;

				if(kec_ki>=SpeedMax) kec_ki=SpeedMax;
				if(kec_ka>=SpeedMax) kec_ka=SpeedMax;	
				
				if(kec_ki>kec) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				//else if(kec_ki<200) KiriMundur(kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>kec) KananMaju(kec_ka);
				else if(kec_ka<0) KananMundur(kec_ka*-1);
				//else if(kec_ka<270) KananMundur(kec_ka);
				else KananMaju(kec_ka);
			}
//				RLift = (float) -(Counter3*R3_tik);
//				if(RLift<45)LiftNaik();
//				else if(RLift>=45) LiftStop();
			
				if(LimTutup) LenganKeluar();
				else				 LenganStop();
		
				if(Distance>=45){kec=400;}
				
				if(!LimTiang){ flg=1; MotorStop();}
//				if(!LimLengan) {LiftStop();}
//				if(LimLengan) {LiftTurun();}
				if(!MerahTengah){MotorStop();LiftStop();while(1){}}
					
				if(Distance>=60){
						if(kec<=420)kec=420;
						else kec-=25;
				}
					
				TampilInt(JarakUS1,GyroHeading,kec_ki,kec_ka);
		}
}

void USRed(int threshold,int servo,int kec, int jarak){
		int kec_ka,kec_ki,kp=0,kd=0;
		float PDWall,ErrorWall=0,DErrorWall=0,LastErrorWall=0;
		int SpeedMax=300,xDist;
		unsigned char flg=0;
		float us;	
		kp=20;
		kd=40;
		Distance=0;
		DynamixelEDF(servo);
		lcd_clear();
		BacaUS2();
		while(Distance<=jarak){
				Odometry();
				BacaLift(); 
				
				BacaUS2();
				
				xDist=(int) Distance;  
				if(xDist%10==0 && SpeedMax<kec) SpeedMax+=100; 
			
				ErrorWall			=	(JarakUS2/10) - threshold;
				
				if((ErrorWall>=10 && ErrorWall<15) || (ErrorWall<=-10 && ErrorWall>-15))					{kp=40; kd=80;}
				else if((ErrorWall>=5  && ErrorWall<10) || (ErrorWall<=-5  && ErrorWall>-10))			{kp=60; kd=100;}
				else if((ErrorWall>=0  && ErrorWall<5) || (ErrorWall<=0  && ErrorWall>-5))				{kp=60; kd=100;}
				else  																																						{kp=20; kd=40;}																																				{kp=40; kd=80;}

				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				
				kec_ka				= (int) (SpeedMax + PDWall);
				kec_ki				=	(int) SpeedMax - PDWall;
 
				if(kec_ki>SpeedMax) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>SpeedMax) KananMaju(kec_ka);
				else if(kec_ka<0) KananMundur(kec_ka*-1);
				else KananMaju(kec_ka);				 
		}
}


void USBlue(int threshold,int servo,int kec, int jarak){
		int kec_ka,kec_ki,kp=0,kd=0;
		float PDWall,ErrorWall=0,DErrorWall=0,LastErrorWall=0;
		int SpeedMax=300,xDist;
		unsigned char flg=0;
		float us;	
		kp=20;
		kd=40;
		Distance=0;
		DynamixelEDF(servo);
		lcd_clear();
		BacaUS1();
		while(Distance<=jarak){
				Odometry();
				BacaLift();
				
				BacaUS1();
				
				xDist=(int) Distance;  
				if(xDist%10==0 && SpeedMax<kec) SpeedMax+=150;
			
				ErrorWall			=	(JarakUS1/10) - threshold;
				
				if((ErrorWall>=10 && ErrorWall<15) || (ErrorWall<=-10 && ErrorWall>-15))					{kp=40; kd=80;}
				else if((ErrorWall>=5  && ErrorWall<10) || (ErrorWall<=-5  && ErrorWall>-10))			{kp=60; kd=100;}
				else if((ErrorWall>=0  && ErrorWall<5) || (ErrorWall<=0  && ErrorWall>-5))				{kp=60; kd=100;}
				else  																																						{kp=20; kd=40;}																																				{kp=40; kd=80;}

				DErrorWall		=	ErrorWall - LastErrorWall;
				LastErrorWall	=	ErrorWall;
				
				PDWall				=	ErrorWall*kp	+ DErrorWall*kd;
				
				kec_ka				= (int) (SpeedMax - PDWall);
				kec_ki				=	(int) SpeedMax + PDWall;
 
				if(kec_ki>SpeedMax) KiriMaju(kec_ki);
				else if(kec_ki<0) KiriMundur(-1*kec_ki);
				else KiriMaju(kec_ki);
				
				if(kec_ka>SpeedMax) KananMaju(kec_ka);
				else if(kec_ka<0) KananMundur(kec_ka*-1);
				else KananMaju(kec_ka);				 
		}
}

void GyroUSJalanMerah(){
		int XGyro,i,us;
	
		for(i=0; i<100; i++)XGyro = GyroHeading;		 
		Distance=0; 		
		while(1){				 
			MoveGyroLurus(20,40,XGyro,700);					
			BacaUS2();  if(JarakUS2<280) break;
		}
		LiftStop();
		PwmEDF=50;	
		USRed(24,-80,1700,120);		
}



void GyroUSJalanBiru(){
		int XGyro,i,us,flg=0;
	
		for(i=0; i<100; i++) XGyro = GyroHeading;		
		Distance=0;		
		while(flg==0){
				MoveGyroLurus(20,40,XGyro,700);				 
				BacaUS1(); if(JarakUS1<280){ flg=1; break;}
		}				
		LiftStop();//PwmEDF=50;						 
		USBlue(24,-80,1700,130);
		PwmEDF=50;						 
}

void HeadingLOCK(int sudut){
	int Error,LastError,dError,Gain,kec_ki,kec_ka,kec_max=500;
	int LimitSpeedMax=kec_max,LimitSpeedMin=-kec_max;
	while(GyroHeading!=sudut){
		Error = GyroHeading - sudut; // -90-(-90)=0

		dError = Error - LastError;
		LastError = Error;
		
		Gain = (Error*80)+(dError*0);
		
		if(Gain > LimitSpeedMax){Gain = LimitSpeedMax;}
		if(Gain < LimitSpeedMin){Gain = LimitSpeedMin;}
	
		kec_ki=-Gain;
		kec_ka=Gain;
		if(!MerahTengah){MotorStop();while(1){}}
				
			
		if(kec_ki>0) KiriMaju(kec_ki);
		else if(kec_ki<0) KiriMundur(-1*kec_ki);
		else KiriMaju(5);
		
		if(kec_ka>0) KananMaju(kec_max-90);
		else if(kec_ka<0) KananMundur((kec_ka*-1)+90);
		else KiriMaju(5);
			
		TampilInt(GyroHeading,Error,kec_ka,kec_ki);
	}
}

void Parkir(){
			PwmEDF=50;
			MotorStop();
			LiftStop();
		
			while(1){
		}
}

