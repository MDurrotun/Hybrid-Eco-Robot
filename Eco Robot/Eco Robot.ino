#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "pt.h"
#include <Dynamixel_Serial.h>       // Library needed to control Dynamixal servo
LiquidCrystal lcd(13, 12, 6, 5, 4, 3);

#define SERVO_ID 0x01               // ID of which we will set Dynamixel too 
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 1000000  // Baud rate speed which the Dynamixel will be set too (57600)
#define CW CSERVO-1020         // lowest clockwise angle is 1, as when set to 0 it set servo to wheel mode
#define CCW CSERVO+1020        // Highest anit-clockwise angle is 0XFFF, as when set to 0 it set servo to wheel mode
#define CSERVO 3000//1860
#define Jarak TCNT5
#define Phi (float) 3.1415926535897932384626433832795

static struct pt pt1, pt2;
unsigned char buffer;
int pidx;

///////////////////////////UNDEFINED///////////////////////////
char state, flag;
float Speed_avg;
int flagSens, Dist, kondisi, condition, konsmv=100,konsb=-35, konsb2=5, konsbx=-40, 
konsby=-200, konsm=20, konsmx=-30, konsmy=-180, zag=0 ;//my80 380
unsigned char Re_buf[8],count=0;
///////////////////////////UNDEFINED///////////////////////////

/////////////////////////////Button///////////////////////////
const int buttonPin1 = 23;    
const int buttonPin2 = 25;
const int buttonPin3 = 27;
const int buttonPin4 = 35; 
const int buttonPin5 = 43; 
const int togglePin = 49;
const int autoKiPin = 20;
const int autoKaPin = 21;
const int autoKi2Pin = 50;
const int autoKa2Pin = 52;
const int autoKiFPin = 8;
const int autoKaFPin = 9;

int button;
int buttonState1 = 0;  
int buttonState2 = 0;
int buttonState3 = 0;
int buttonState4 = 0;  
int buttonState5 = 0;
int toggleState = 0;
int autoKiState = 0;
int autoKaState = 0;
int autoKi2State = 0;
int autoKa2State = 0;
int autoKiFState = 0;
int autoKaFState = 0;
int buzzerState = 0;

void setupRotary(){
  TCCR5A=0x00;
  TCCR5B=0x06;
  TCNT5H=0x00;
  TCNT5L=0x00;
  ICR5H=0x00;
  ICR5L=0x00;
  OCR5AH=0x00;
  OCR5AL=0x00;
  OCR5BH=0x00;
  OCR5BL=0x00;
  OCR5CH=0x00;
  OCR5CL=0x00;
}

void Button(){
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);
  buttonState4 = digitalRead(buttonPin4);
  buttonState5 = digitalRead(buttonPin5);
  toggleState = digitalRead(togglePin);
  autoKiState = digitalRead(autoKiPin);
  autoKaState = digitalRead(autoKaPin);
  autoKi2State = digitalRead(autoKi2Pin);
  autoKa2State = digitalRead(autoKa2Pin);
  autoKiFState = digitalRead(autoKiFPin);
  autoKaFState = digitalRead(autoKaFPin);
}

void setupButton(){
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  pinMode(buttonPin5, INPUT_PULLUP);
  pinMode(togglePin, INPUT_PULLUP);
  pinMode(autoKiPin, INPUT_PULLUP);
  pinMode(autoKaPin, INPUT_PULLUP);
  pinMode(autoKi2Pin, INPUT_PULLUP);
  pinMode(autoKa2Pin, INPUT_PULLUP);
  pinMode(autoKiFPin, INPUT_PULLUP);
  pinMode(autoKaFPin, INPUT_PULLUP);
}

void ButtonState(){
  Button();
  if(buttonState1 == 0) {button=1;}        
  if(buttonState2 == 0) {button=2;}
  if(buttonState3 == 0) {button=3;}
  if(buttonState4 == 0) {button=4;}        
  if(buttonState5 == 0) {button=5;}
}

/////////////////////////////Button/////////////////////
///////////////////////////Sensor Garis/////////////////
int XSensor, Data, Status2, Status3, DataIMU,
StatusSensor, Kalibrasi;
int s[16];
unsigned int DataR, DataB, DataG;
/////////////////////////variable PID///////////////////
int Error, DError, LastError, IError, Flag=0;
int PID, posServo, Sudut, Kec;
float Kp, Kd, Ki;
///////////////////////////Sensor Garis/////////////////

///////////////////////////Rotary///////////////////////
int Speed, Jarak_1, JarakX=0, Posisi;

///////////////////////////Rotary///////////////////////

///////////////////////////IMU/////////////////////////
int YPR[3],yaw=0,roll=0,thresholdYaw=0,thresholdRoll=0,
sudutYaw=0,sudutRoll=0;
unsigned char counter=0,sign=0;
int buff[8];
void setupIMU(){
  delay(100);
  Serial1.write(0XA5);
  Serial1.write(0X52);
  Serial1.write(0XA5);
  Serial1.write(0X52);
}
///////////////////////////IMU//////////////////////////

///////////////////////////Serial///////////////////////
void setupSerial(){
  Serial1.begin(115200);    
  Serial2.begin(38400); 
}
///////////////////////////Serial///////////////////////

///////////////////////////Servo///////////////////////
int servoPos = CSERVO;
void setupDynamixel(){
  delay(1000);
  Dynamixel.begin(SERVO_SET_Baudrate, SERVO_ControlPin);              // We now need to set Ardiuno to the new Baudrate speed 
  Dynamixel.setMode(SERVO_ID, SERVO, CW, CCW);    // set mode to SERVO and set angle limits
  Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);  
}
///////////////////////////Servo///////////////////////

void kalibrasi(){
////////////////////////Kalibrasi//////////////////////
    Kalibrasi=0;
    while(Kalibrasi==0){
      lcd.setCursor(0,0);lcd.print(">>>Kalibrasi<<<<");
      lcd.setCursor(0,1);lcd.print("Bismillah Sukses");
      button=0;
      ButtonState(); 
      if(button==5){Kalibrasi=1;} 
    }
    lcd.setCursor(0,0);lcd.print("Kalibrasi Sensor");
    lcd.setCursor(0,1);lcd.print("   Warna Hijau   ");
    Serial2.write(0xB0);
    delay(500);
    while(Kalibrasi==1){
      button=0;
      ButtonState();
      if(button==5){
        Kalibrasi=2;
        Serial2.write(0xB0);
      }  
    }
    delay(300);
    lcd.setCursor(0,1);lcd.print("  Warna Orange  ");
    while(Kalibrasi==2){
      button=0;
      ButtonState();
      if(button==5){
        Kalibrasi=3;
        Serial2.write(0xB0);        
      }
    }
    delay(300);
    lcd.setCursor(0,1);
    lcd.print("  Warna Biru   ");
    while(Kalibrasi==3){
      button=0;
      ButtonState();
      if(button==5){
        Kalibrasi=4;
        Serial2.write(0xB0);        
      }
    }      
    delay(300);
    if(Kalibrasi==4){
      lcd.setCursor(0,1);
      lcd.print("Kalibrasi Sukses");
      lcd.setCursor(0,0);
      lcd.print(" Alhamdulillah  ");
      delay(500);
      lcd.clear();
      Kalibrasi=0;      
    }
 }

///////////////////////////////Kalibrasi//////////////////////////



void setup() {
  setupSerial();
  setupButton();
  setupDynamixel();
  setupRotary();
  Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
  PT_INIT(&pt1);  // initialise the two
  PT_INIT(&pt2); 
  pinMode(22,OUTPUT);
  lcd.begin(16, 2);
  delay(100);
  beep(50);
  beep(50);
  beep(50);
  lcd.setCursor(0,0);
  lcd.print(" PENSAE Chai-yo ");
  lcd.setCursor(0,1);
  lcd.print("Bismillah Juara1");
  delay(200);
  setupIMU();
  lcd.clear();
}

static int protothread1(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { 
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    bacaSensor();
    sudutIMU();
  }
  PT_END(pt);
}

static int protothread2(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    pid();
  }
  PT_END(pt);
}
void beep(unsigned char delayms){
  digitalWrite(22, HIGH);
  delay(delayms);
  digitalWrite(22, LOW);
  delay(delayms); 
}
///////////////////////////Sensor Garis///////////////////////
void ambilSensor(){
    while (Serial2.available()>1) {  
     buffer=Serial2.read(); 
      switch(Status2){
        case 0: if(buffer==0x81) Status2=1; else Status2=0; break;
        case 1: if(buffer==0xC1) Status2=2; else Status2=0; break;  
        case 2: Data=buffer;Status2=3; break;
        case 3: DataR=buffer;Status2=4; break;
        case 4: DataB=buffer;Status2=5; break;
        case 5: DataG=buffer;Status2=0; break;
      }
    XSensor = Data;
    if(DataG > DataR){StatusSensor=1;}
    else{StatusSensor=2;} 
    if(XSensor & 0x01 == 1) Flag=0;
    if(XSensor>>7 & 0x01 == 1) Flag=1;   
    }
}
void bacaSensor(){
  ambilSensor();
  int aa=0,bb=8;
  XSensor=Data;  
  for(aa=0;aa<8;aa++){
    bb--;
    lcd.setCursor(aa,1);
    s[aa]=(XSensor >> bb & 0x01);
    lcd.print(s[aa]);
 }
}
////////////////////////////IMU////////////////////////////
void bacaIMU(){
    while (Serial1.available()) {   
    Re_buf[count]=(unsigned char)Serial1.read();
    if(count==0&&Re_buf[0]!=0xAA) return;        
    count++;       
    if(count==8) 
    {    
       count=0;
       sign=1;
    }      
  }    
  if(sign){  
     sign=0;
     if(Re_buf[0]==0xAA && Re_buf[7]==0x55)
     {           
        YPR[0]=-1*(Re_buf[1]<<8|Re_buf[2])/100;
        YPR[1]=-1*(Re_buf[3]<<8|Re_buf[4])/100;
        YPR[2]=-1*(Re_buf[5]<<8|Re_buf[6])/100; 
    }
  }     
}

void kalibIMU(){
  yaw=0;
  for(int x=0; x<100; x++){
    yaw += YPR[0];
  }
  thresholdYaw = yaw/100;
  writeEEPROM();
}

void sudutIMU(){
  bacaIMU();
  readEEPROM();
  sudutYaw = YPR[0] - thresholdYaw;
//  sudutRoll = YPR[2] - thresholdRoll;
  
  if(sudutYaw > 180){sudutYaw -= 360;}
  else if(sudutYaw < -180){sudutYaw += 360;}
  
//  if(sudutRoll > 180){sudutRoll -= 360;}
//  else if(sudutRoll < -180){sudutRoll += 360;}
}
void writeEEPROM(){
    if(thresholdYaw<0){
        thresholdYaw *= -1;
        EEPROM.write(0,thresholdYaw);
        EEPROM.write(1,1);
    }
    else{
        EEPROM.write(0,thresholdYaw); 
        EEPROM.write(1,2);
    }
}

void readEEPROM(){
    flag = EEPROM.read(1);
    if(flag==1){
        thresholdYaw = EEPROM.read(0);     
        thresholdYaw *= -1;
    }
    else if(flag==2){
        thresholdYaw = EEPROM.read(0); 
    }
}

void pid(){
  switch(pidx){
    case 0: pidnaik(); break;
    case 1: pidturun(); break;
    case 2: pidnaik2(); break;
    case 3: pidturun2(); break;
    case 4: pidnaik3(); break;
    case 5: pidturun3(); break; 
    case 6: pidturun4(); break;   
  }
}

void pidnaik(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
  switch(XSensor){
      case 0b00000001: Error = -4;  break; 
      case 0b00000011: Error = -3;  break;
      case 0b00000111: Error = -3;  break; 
      case 0b00000010: Error = -2;  break; 
      case 0b00000110: Error = -2;  break; 
      case 0b00001110: Error = -2;  break;
      case 0b00000100: Error = -1;  break; 
      case 0b00001100: Error = -1;  break;
      case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 1;  break;
      case 0b00100000: Error = 1;  break;
      case 0b01110000: Error = 2;  break;
      case 0b01100000: Error = 2;  break;
      case 0b01000000: Error = 2;  break;
      case 0b11100000: Error = 3;  break;
      case 0b11000000: Error = 3;  break;
      case 0b10000000: Error = 4;  break;      
      case 0b00000000: if(Flag==1){Error=6;}
                       else if(Flag==0){Error=-6;} break;
  }  
  if(Error<3 && Error >-3)  {Kp =2;  Kd =25;  Ki =1;}   //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =3;  Kd =40;  Ki =1;}   //Konstanta ketika Error besar
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=1 && Error>=-1) IError=0;
  
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);
    
  if(PID>60)          PID=60;                   // pembatas nilai PID 
  else if(PID<-60)    PID=-60;  
    
  posServo+=PID;                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+800)    posServo=CW+800;      // pembatas sudut ster 800
  if(posServo>=CCW-800)   posServo=CCW-800;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(Jarak);
  lcd.setCursor(8,0);    lcd.print(StatusSensor);
  Dynamixel.servo(SERVO_ID,posServo,0x3FF);      
}

void pidnaik2(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
  switch(XSensor){
      case 0b00000001: Error = -4;  break; 
      case 0b00000011: Error = -3;  break;
      case 0b00000111: Error = -3;  break; 
      case 0b00000010: Error = -2;  break; 
      case 0b00000110: Error = -2;  break; 
      case 0b00001110: Error = -2;  break;
      case 0b00000100: Error = -1;  break; 
      case 0b00001100: Error = -1;  break;
      case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 1;  break;
      case 0b00100000: Error = 1;  break;
      case 0b01110000: Error = 2;  break;
      case 0b01100000: Error = 2;  break;
      case 0b01000000: Error = 2;  break;
      case 0b11100000: Error = 3;  break;
      case 0b11000000: Error = 3;  break;
      case 0b10000000: Error = 4;  break;      
      case 0b00000000: if(Flag==1){Error=6;}
                       else if(Flag==0){Error=-6;} break;
  }  
  if(Error<3 && Error >-3)  {Kp =4;  Kd =10;  Ki =1;}   //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =5;  Kd =25;  Ki =1;}   //Konstanta ketika Error besar
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=1 && Error>=-1) IError=0;
  
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);
    
  if(PID>60)          PID=60;                   // pembatas nilai PID 
  else if(PID<-60)    PID=-60;  
    
  posServo+=PID;                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+200)    posServo=CW+200;      // pembatas sudut ster 800
  if(posServo>=CCW-200)   posServo=CCW-200;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(Jarak);
  lcd.setCursor(13,0);    lcd.print(StatusSensor);
  Dynamixel.servo(SERVO_ID,posServo,0x3FF);      
}

void pidnaik3(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
  switch(XSensor){
      case 0b00000001: Error = -4;  break; 
      case 0b00000011: Error = -3;  break;
      case 0b00000111: Error = -3;  break; 
      case 0b00000010: Error = -2;  break; 
      case 0b00000110: Error = -2;  break; 
      case 0b00001110: Error = -2;  break;
      case 0b00000100: Error = -1;  break; 
      case 0b00001100: Error = -1;  break;
      case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 1;  break;
      case 0b00100000: Error = 1;  break;
      case 0b01110000: Error = 2;  break;
      case 0b01100000: Error = 2;  break;
      case 0b01000000: Error = 2;  break;
      case 0b11100000: Error = 3;  break;
      case 0b11000000: Error = 3;  break;
      case 0b10000000: Error = 4;  break;      
      case 0b00000000: if(Flag==1){Error=6;}
                       else if(Flag==0){Error=-6;} break;
  }  
  if(Error<3 && Error >-3)  {Kp =4;  Kd =10;  Ki =1;}   //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =5;  Kd =25;  Ki =1;}   //Konstanta ketika Error besar
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=1 && Error>=-1) IError=0;
  
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);
    
  if(PID>200)          PID=200;                   // pembatas nilai PID 
  else if(PID<-200)    PID=-200;  
    
  posServo+=PID;                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+200)    posServo=CW+200;      // pembatas sudut ster 800
  if(posServo>=CCW-200)   posServo=CCW-200;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(Jarak);
  lcd.setCursor(13,0);    lcd.print(StatusSensor);
  Dynamixel.servo(SERVO_ID,posServo,0x3FF);      
}

void pidturun(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
     switch(XSensor){
      case 0b00000001: Error = -9;  break; 
      case 0b00000011: Error = -8;  break;
      //case 0b00000111: Error = -5;  break; 
      case 0b00000010: Error = -5;  break; 
      case 0b00000110: Error = -4;  break; 
      //case 0b00001110: Error = -3;  break;
      case 0b00000100: Error = -2;  break; 
      case 0b00001100: Error = -2;  break;
      //case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      //case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 2;  break;
      case 0b00100000: Error = 2;  break;
      //case 0b01110000: Error = 3;  break;
      case 0b01100000: Error = 4;  break;
      case 0b01000000: Error = 5;  break;
      //case 0b11100000: Error = 5;  break;
      case 0b11000000: Error = 8;  break;
      case 0b10000000: Error = 9;  break; 
      case 0xF8|0xFC|0xFE|0xF3|0xF6|
           0xF2|0x76|0x36|0x6E|0x6F|
           0xE3|0xB1|0xB8|0xBC|0xBE|
           0x1E|0x3E|0x7E|0x0F|0x1F|
           0x3F|0x7F|0x0D|0x1D|0x3D|
           0x7D|0xFF: Error=0;break;
      //case 0b00000101: Error = -6;  break;
      //case 0b00001101: Error = -6;  break;
      //case 0b00001001: Error = -6;  break;
      //case 0b00011001: Error = -6;  break;
      
      //case 0b10100000: Error = 6;  break;
      //case 0b10110000: Error = 6;  break;
      //case 0b10010000: Error = 6;  break;
      //case 0b10011000: Error = 6;  break;
      
      case 0b00000000: if(Flag==1){Error=10;}
                       else if(Flag==0){Error=-10;} break;
      default: Error = 0; break;
  }  
  Error-=3;
  if(Error<5 && Error >-5)  {Kp =3.5;  Kd =9.3;  Ki=0; Kec=1000;}//3.5 9.3  10 //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =3.3;  Kd =9.3;  Ki=0; Kec=700;}//2.3 9 13//Konstanta ketika Error besar

  if(Error==10 || Error==-10) {Ki =0.05; Kec=1000;}
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=4 && Error>=-4) IError=0;
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);

  //if(Error<0)       { Kec = -1*Error*250;}
  //else if(Error>=0) { Kec = Error*250;} 

  if(Kec>1023) Kec = 1023;
  
  if(PID>100)          PID=100;                   // pembatas nilai PID 
  else if(PID<-100)    PID=-100;  
    
  posServo+=(PID);                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+500)    posServo=CW+500;      // pembatas sudut ster
  if(posServo>=CCW-500)   posServo=CCW-500;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(posServo);
  lcd.setCursor(8,0);    lcd.print(PID);
  if(Error<5 && Error >-5){ Dynamixel.servo(SERVO_ID,posServo,Kec); }
  else                      Dynamixel.servo(SERVO_ID,posServo,Kec);
}



void pidturun2(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
     switch(XSensor){
      case 0b00000001: Error = -9;  break; 
      case 0b00000011: Error = -8;  break;
      //case 0b00000111: Error = -5;  break; 
      case 0b00000010: Error = -5;  break; 
      case 0b00000110: Error = -4;  break; 
      //case 0b00001110: Error = -3;  break;
      case 0b00000100: Error = -2;  break; 
      case 0b00001100: Error = -2;  break;
      //case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      //case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 2;  break;
      case 0b00100000: Error = 2;  break;
      //case 0b01110000: Error = 3;  break;
      case 0b01100000: Error = 4;  break;
      case 0b01000000: Error = 5;  break;
      //case 0b11100000: Error = 5;  break;
      case 0b11000000: Error = 8;  break;
      case 0b10000000: Error = 9;  break; 
      case 0xF8|0xFC|0xFE|0xF3|0xF6|
           0xF2|0x76|0x36|0x6E|0x6F|
           0xE3|0xB1|0xB8|0xBC|0xBE|
           0x1E|0x3E|0x7E|0x0F|0x1F|
           0x3F|0x7F|0x0D|0x1D|0x3D|
           0x7D|0xFF: Error=0;break;
      //case 0b00000101: Error = -6;  break;
      //case 0b00001101: Error = -6;  break;
      //case 0b00001001: Error = -6;  break;
      //case 0b00011001: Error = -6;  break;
      
      //case 0b10100000: Error = 6;  break;
      //case 0b10110000: Error = 6;  break;
      //case 0b10010000: Error = 6;  break;
      //case 0b10011000: Error = 6;  break;
      
      case 0b00000000: if(Flag==1){Error=10;}
                       else if(Flag==0){Error=-10;} break;
      default: Error = 0; break;
  }  
  if(Error<5 && Error >-5)  {Kp =3.5;  Kd =16;  Ki =0;}//3 20  10 //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =3.8;  Kd =18;  Ki =0;}//3.3 23 13//Konstanta ketika Error besar
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=1 && Error>=-1) IError=0;
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);

  if(Error<0)       { Kec = -1*Error*250;}
  else if(Error>=0) { Kec = Error*250;} 

  if(Kec>1023) Kec = 1023;
  
  if(PID>100)          PID=100;                   // pembatas nilai PID 
  else if(PID<-100)    PID=-100;  
    
  posServo+=(PID);                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+500)    posServo=CW+500;      // pembatas sudut ster
  if(posServo>=CCW-500)   posServo=CCW-500;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(posServo);
  lcd.setCursor(8,0);    lcd.print(PID);
  if(Error<5 && Error >-5){ Dynamixel.servo(SERVO_ID,posServo,Kec); }
  else                      Dynamixel.servo(SERVO_ID,posServo,Kec);
}

void pidturun3(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
     switch(XSensor){
      case 0b00000001: Error = -9;  break; 
      case 0b00000011: Error = -8;  break;
      //case 0b00000111: Error = -6;  break; 
      case 0b00000010: Error = -6;  break; 
      case 0b00000110: Error = -4;  break; 
      //case 0b00001110: Error = -3;  break;
      case 0b00000100: Error = -2;  break; 
      case 0b00001100: Error = -2;  break;
      //case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      //case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 1;  break;
      case 0b00100000: Error = 2;  break;
      //case 0b01110000: Error = 3;  break;
      case 0b01100000: Error = 4;  break;
      case 0b01000000: Error = 5;  break;
      //case 0b11100000: Error = 5;  break;
      case 0b11000000: Error = 8;  break;
      case 0b10000000: Error = 9;  break; 

      case 0b10000001: Error = 10;  break;
      case 0b11000001: Error = 10;  break;
      case 0b01000001: Error = 10;  break;
      case 0b01100001: Error = 10;  break;
      case 0b00100001: Error = 10;  break;
      case 0b00001001: Error = 10;  break;
      case 0b00011001: Error = 10;  break;
//      
//      case 0b10100000: Error = 6;  break;
//      case 0b10110000: Error = 6;  break;
//      case 0b10010000: Error = 6;  break;
//      case 0b10011000: Error = 6;  break;
      
      case 0b00000000: if(Flag==1){Error=10;}
                       else if(Flag==0){Error=-10;} break;
                       //default: Error = 0; break;
  }  
  Error+=1;
  if(Error<5 && Error >-5)  {Kp =5.9;  Kd =9.3;  Ki=0; Kec=800;}//3 20  10 //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =4.7;  Kd =10;  Ki=0; Kec=600;}//3.3 23 13//Konstanta ketika Error besar

  if(Error==10 || Error==-10) {Ki =0.02; Kec=1000;}
  
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=4 && Error>=-4) IError=0;
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);

 // if(Error<0)       { Kec = -1*Error*200;}
 // else if(Error>=0) { Kec = Error*200;} 

  if(Kec>1023) Kec = 1023;
  
  if(PID>100)          PID=100;                   // pembatas nilai PID 
  else if(PID<-100)    PID=-100;  
    
  posServo+=(PID);                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+500)    posServo=CW+500;      // pembatas sudut ster
  if(posServo>=CCW-500)   posServo=CCW-500;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(posServo);
  lcd.setCursor(8,0);    lcd.print(PID);
  if(Error<5 && Error >-5){ Dynamixel.servo(SERVO_ID,posServo,Kec); }
  else                      Dynamixel.servo(SERVO_ID,posServo,Kec);
}

void pidturun4(){
  static unsigned char pilih=0;
  if(XSensor & 0x01 == 1) Flag=0;
  if(XSensor>>7 & 0x01 == 1) Flag=1;
     switch(XSensor){
      case 0b00000001: Error = -9;  break; 
      case 0b00000011: Error = -8;  break;
      //case 0b00000111: Error = -5;  break; 
      case 0b00000010: Error = -5;  break; 
      case 0b00000110: Error = -4;  break; 
      //case 0b00001110: Error = -3;  break;
      case 0b00000100: Error = -2;  break; 
      case 0b00001100: Error = -2;  break;
      //case 0b00011100: Error = -1;  break;
      case 0b00001000: Error = 0;  break; 
      case 0b00011000: Error = 0;  break;
      case 0b00010000: Error = 0;  break;
      //case 0b00111000: Error = 1;  break;
      case 0b00110000: Error = 2;  break;
      case 0b00100000: Error = 2;  break;
      //case 0b01110000: Error = 3;  break;
      case 0b01100000: Error = 4;  break;
      case 0b01000000: Error = 5;  break;
      //case 0b11100000: Error = 5;  break;
      case 0b11000000: Error = 8;  break;
      case 0b10000000: Error = 9;  break; 
      case 0xF8|0xFC|0xFE|0xF3|0xF6|
           0xF2|0x76|0x36|0x6E|0x6F|
           0xE3|0xB1|0xB8|0xBC|0xBE|
           0x1E|0x3E|0x7E|0x0F|0x1F|
           0x3F|0x7F|0x0D|0x1D|0x3D|
           0x7D|0xFF: Error=0;break;
      //case 0b00000101: Error = -6;  break;
      //case 0b00001101: Error = -6;  break;
      //case 0b00001001: Error = -6;  break;
      //case 0b00011001: Error = -6;  break;
      
      //case 0b10100000: Error = 6;  break;
      //case 0b10110000: Error = 6;  break;
      //case 0b10010000: Error = 6;  break;
      //case 0b10011000: Error = 6;  break;
      
      case 0b00000000: if(Flag==1){Error=10;}
                       else if(Flag==0){Error=-10;} break;
      default: Error = 0; break;
  }  
  //Error+=3;
  if(Error<5 && Error >-5)  {Kp =3.5;  Kd =9.3;  Ki=0; Kec=1000;}//3.5 9.3  10 //Konstanta ketika garis disekitar tengah sensor 
  else                      {Kp =3.3;  Kd =9.3;  Ki=0; Kec=700;}//2.3 9 13//Konstanta ketika Error besar

  if(Error==10 || Error==-10) {Ki =0.05; Kec=1000;}
  
  DError = Error-LastError;
  LastError = Error;
  IError+=Error*0.1;
  if(Error<=4 && Error>=-4) IError=0;
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);

  //if(Error<0)       { Kec = -1*Error*250;}
  //else if(Error>=0) { Kec = Error*250;} 

  if(Kec>1023) Kec = 1023;
  
  if(PID>100)          PID=100;                   // pembatas nilai PID 
  else if(PID<-100)    PID=-100;  
    
  posServo+=(PID);                                // PID sebagai nilai yang diakumulasikan untuk nilai proses (ster)
  if(posServo<=CW+500)    posServo=CW+500;      // pembatas sudut ster
  if(posServo>=CCW-500)   posServo=CCW-500;     // pembatas sudut ster
   
  lcd.setCursor(0,0);    lcd.print(posServo);
  lcd.setCursor(8,0);    lcd.print(PID);
  if(Error<5 && Error >-5){ Dynamixel.servo(SERVO_ID,posServo,Kec); }
  else                      Dynamixel.servo(SERVO_ID,posServo,Kec);
}


void SetServo(float DisMax, int ServoNow, float ServoTarget){
  float Val,Place=0;
  Jarak=0;
  Dynamixel.servo(SERVO_ID,ServoNow,0x3FF);
  while(Place<DisMax){     
    Place+=Jarak;    
    Jarak=0;
    Val = (float)Place/DisMax * (ServoTarget-ServoNow) + ServoNow;    
    posServo=(int)Val;  
         
    if(posServo>CCW)      posServo=CCW;
    else if(posServo<CW)  posServo=CW;

    Dynamixel.servo(SERVO_ID,posServo,0x3FF);
     
    protothread1(&pt1, 10);
    protothread2(&pt2, 10);
    pidx=0;
  }
  Dynamixel.servo(SERVO_ID,posServo,0x3FF);   
}

void Heading(int sudut){
  protothread1(&pt1, 10);
  Kp =17; 
  Kd =30;
  Ki=0;
  
  Error = sudutYaw-sudut;  
  DError = Error-LastError;  
  LastError = Error;
  IError+=Error*0.2;
  if(Error<2 && Error>-2) IError=0;
  else if(IError>100) IError=100;
  else if(IError<-100) IError=-100;
  
  PID =(Error*Kp)  + (DError*Kd) + (IError*Ki);
  
  posServo=(int)(CSERVO+PID);
  if(posServo<=CW+400)posServo=CW+400;
  if(posServo>=CCW+400)posServo=CCW+400;
  
  Dynamixel.servo(SERVO_ID,posServo,0x3FF);  
}

void cekSensor(){
  if(autoKiFState==0){lcd.setCursor(11,0);lcd.print("1");}
  else{lcd.setCursor(11,0);lcd.print("0");}
  if(autoKaFState==0){lcd.setCursor(12,0);lcd.print("1");}
  else{lcd.setCursor(12,0);lcd.print("0");}  
  if(autoKaState==0){lcd.setCursor(13,0);lcd.print("1");}
  else{lcd.setCursor(13,0);lcd.print("0");} 
  lcd.setCursor(15,0);lcd.print(StatusSensor);
}

void loop(){
  //Serial2.write(0xC0);
  Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
  ButtonState();
//========================== START BIRU =======================
  if(!toggleState){
    protothread1(&pt1, 10);
    cekSensor();
    lcd.setCursor(0,0);
    lcd.print("BIRU");
    lcd.setCursor(9,1);lcd.print(YPR[0]);
    lcd.setCursor(13,1);lcd.print(sudutYaw);
    lcd.setCursor(6,0);lcd.print(Jarak);
    if(button==4){
      ButtonState();
      lcd.setCursor(0,0);
      lcd.print("Kalibrasi IMU");
      delay(300);  
      kalibIMU();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Alhamdulillah");
      delay(400);
      lcd.clear();
      button=0;
    }
    if(button==5){
      kalibrasi();
    }
//========================== START BIRU 1 ========================         
    if(button==1){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START BIRU 1  ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);
      lcd.clear(); 
    }
// ============================ BUTTON 1 =========================    
    while(button==1){
      Jarak=0;
      while(Jarak<=7000){//12000
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;
      }
      while(StatusSensor!=1){
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;  
      }
      Jarak=0;
      while(Jarak<=1150){//1450
        digitalWrite(22, LOW);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;
      }
// ================= BELOK 90 DERAJAT ===========================
      //digitalWrite(22, HIGH);           
      //Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);   //        condong kiri      450
      //Jarak=0;
      SetServo(1000,CSERVO,CSERVO+500);//1560          //        belok kanan     
      SetServo(40,CSERVO+500,CSERVO-300);          //        banting kiri    
      //Dynamixel.servo(SERVO_ID,CSERVO+500,0x3FF);   
      SetServo(200,CSERVO+500,CSERVO+510);              //        belok kanan       1000, 
      //Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);       //        center
// ================= BACA ISLAND ================================
      protothread1(&pt1, 10); 
      Jarak=0; int kk=0;
      while(Jarak<300){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;    
        ButtonState();
        //if(autoKiFState==0) kk++;
      } 
      //digitalWrite(22, LOW);
      ButtonState();
      for(int kk=0;kk<300;kk++) while(autoKiFState==1){     
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2; 
        ButtonState();      
      }
// ================= LEPAS ISLAND =============================
      int YawNow=0;
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=200){//190       
        ButtonState();
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
      }
      YawNow=sudutYaw;
      if(YawNow>20)YawNow=20;
      
// ================= BELOK IMU ================================   
      int xPlace=0;
      float xVal=0;
      
      Jarak=0;
      digitalWrite(22, HIGH);
      while(sudutYaw>10){
        protothread1(&pt1, 10);
        //Dynamixel.servo(SERVO_ID,CSERVO+400,0x3FF);
        
        if(xPlace<350) xPlace=Jarak; 
        xVal = (float)xPlace/350 * ((CSERVO+300)-(CSERVO-(125*22/YawNow))) + (CSERVO-(125*22/YawNow));    
        posServo=(int)xVal;  
         
        if(posServo>CCW)      posServo=CCW;
        else if(posServo<CW)  posServo=CW;

         Dynamixel.servo(SERVO_ID,posServo,0x3FF);
         lcd.setCursor(1,12);lcd.print(xPlace);
      }   
      digitalWrite(22, LOW);
// ================= COUNTER ISLAND =========================== 
      Jarak=0;
      while(Jarak<=800){
        Heading(0);
      }
// ================= BACA ISLAND 2 =========================== 
      ButtonState();
      for(int kk=0;kk<100;kk++) while(autoKaFState==1){     
        digitalWrite(22, HIGH);
        Heading(0);
        ButtonState();      
      }  
// ================= LEPAS ISLAND 2 =========================== 
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=140){       
        Heading(0);
      }  
      Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
      SetServo(250,CSERVO,CSERVO-300);  
// ====================== TURUN =============================== 
      Jarak=0;
      while(Jarak<=500){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=4;        
      }
      Jarak=0;
      while(Jarak<=7500){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=3;        
      }
      digitalWrite(22, HIGH);
      while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=6;
      }
      
// ====================== TIANG =============================== 
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<500;i++){
            protothread1(&pt1, 10);
            int sterYaw=85-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
        }
        kondisi=3;
      }
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CCW,0x3FF);
      }                 
// ========================== BUTTON 1 =========================
    }
//******************** START BIRU 1 SELESAI ********************
 
//========================  START BIRU 2 ======================= 
    if(button==2){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START BIRU 2  ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);      
      lcd.clear(); 
    }
// ========================== BUTTON 2 =========================     
    while(button==2){
// ================= BACA ISLAND ================================
      ButtonState();
      for(int kk=0;kk<300;kk++) while(autoKiFState==1){     
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2; 
        ButtonState();      
      }
// ================= LEPAS ISLAND =============================
      int YawNow=0;
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=150){//190       
        ButtonState();
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
      }
      YawNow=sudutYaw;
      if(YawNow>20)YawNow=20;
      
// ================= BELOK IMU ================================   
      int xPlace=0;
      float xVal=0;
      
      Jarak=0;
      digitalWrite(22, HIGH);
      while(sudutYaw>10){
        protothread1(&pt1, 10);
        //Dynamixel.servo(SERVO_ID,CSERVO+400,0x3FF);
        
        if(xPlace<350) xPlace=Jarak; 
        xVal = (float)xPlace/350 * ((CSERVO+300)-(CSERVO-(125*22/YawNow))) + (CSERVO-(125*22/YawNow));    
        posServo=(int)xVal;  
         
        if(posServo>CCW)      posServo=CCW;
        else if(posServo<CW)  posServo=CW;

         Dynamixel.servo(SERVO_ID,posServo,0x3FF);
         lcd.setCursor(1,12);lcd.print(xPlace);
      }   
      digitalWrite(22, LOW);
// ================= COUNTER ISLAND =========================== 
      Jarak=0;
      while(Jarak<=1300){
        Heading(0);
        ButtonState();
      }
// ================= BACA ISLAND 2 =========================== 
      ButtonState();
      for(int kk=0;kk<100;kk++) while(autoKaFState==1){     
        digitalWrite(22, HIGH);
        Heading(0);
        ButtonState();      
      }  
// ================= LEPAS ISLAND 2 =========================== 
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=140){       
        Heading(0);
      }  
      Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
      SetServo(250,CSERVO,CSERVO-300);  
// ====================== TURUN =============================== 
      Jarak=0;
      while(Jarak<=8000){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=3;        
      }
      while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=5;
      }
// ====================== TIANG =============================== 
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<400;i++){
            protothread1(&pt1, 10);
            int sterYaw=85-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
        }
        kondisi=3;
      }
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CCW,0x3FF);
      }          
// ========================== BUTTON 2 =========================      
    }
//********************* START BIRU 2 SELESAI *******************

//========================  START BIRU 3 =======================
    if(button==3){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START BIRU 3  ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);   
      lcd.clear();    
    }
// ========================== BUTTON 3 =========================     
    while(button==3){
// ====================== TURUN ===============================  
      Jarak=0;
      while(Jarak<=8000){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=3;        
      }  
    while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=3;
      }
// ====================== TIANG ===============================         
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<500;i++){
            protothread1(&pt1, 10);
            int sterYaw=85-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
        }
        kondisi=3;
      }
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CCW,0x3FF);
      }     
// ====================== TIANG =============================== 
//      ButtonState();
//      for(int kk=0;kk<100;kk++) while(autoKaState==1){     
//        digitalWrite(22, HIGH);
//        //Heading(0);
//        protothread1(&pt1, 10);
//        protothread2(&pt2, 10);
//        pidx=3;
//        ButtonState();      
//      }  
//      kondisi=2;
//      while(kondisi==2){
//        for(int i=0;i<7000;i++){
//          protothread1(&pt1, 10);
//          protothread2(&pt2, 10);
//          pidx=3;
//        }
//        kondisi=3;
//      }
//      Jarak=0;
//      while(Jarak<330){
//        protothread1(&pt1, 10);
//        protothread2(&pt2, 10);
//        pidx=3;
//      }
//      kondisi=2;
//      while(kondisi==2){
//        //Dynamixel.servo(SERVO_ID,CCW-500,0x3FF);
//        //for(int i=0;i<13000;i++){}
//        protothread1(&pt1, 10);
//        protothread2(&pt2, 10);
//        pidx=3;
//        delay(1000);
//        kondisi=3;
//      }
//      while(kondisi==3){
//        Dynamixel.servo(SERVO_ID,CCW,0x3FF);        
//      }
// ========================== BUTTON 3 =========================           
    }
//********************* START BIRU 3 SELESAI *******************    
  }
//**************************BIRU SELESAI************************ 


//========================== START MERAH =======================
  if(toggleState){
    protothread1(&pt1, 10);
    cekSensor();
    lcd.setCursor(0,0);
    lcd.print("MERAH");
    lcd.setCursor(9,1);lcd.print(YPR[0]);
    lcd.setCursor(13,1);lcd.print(sudutYaw);
    lcd.setCursor(6,0);lcd.print(Jarak);
    if(button==4){
      ButtonState();
      lcd.setCursor(0,0);
      lcd.print("Kalibrasi IMU");
      delay(300);  
      kalibIMU();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Alhamdulillah");
      delay(400);
      lcd.clear();
      button=0;
    }
    if(button==5){
      kalibrasi();
    }
//========================== START MERAH 1 ========================         
    if(button==1){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START MERAH 1 ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);
      lcd.clear(); 
    }
// ============================ BUTTON 1 =========================    
    while(button==1){
      Jarak=0;
      while(Jarak<=7000){//12000
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;
      }
      protothread1(&pt1, 10); 
      while(StatusSensor!=1){
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;  
      }
      Jarak=0;
      while(Jarak<=1250){//1350
        digitalWrite(22, LOW);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=0;
      }
// ================= BELOK 90 DERAJAT ===========================
      //digitalWrite(22, HIGH);   
      //Dynamixel.servo(SERVO_ID,CSERVO-300,0x3FF);   //        condong kiri      450
      Jarak=0;
      SetServo(1060,CSERVO,CSERVO-480);          //  1160      belok kanan     
      SetServo(50,CSERVO-480,CSERVO+300);          //  770      banting kiri    
      SetServo(400,CSERVO-350,CSERVO-400); //480 480             //        belok kanan       1000, 
      //Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);       //        center
// ================= BACA ISLAND ================================
      Jarak=0;
      while(Jarak<300){
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;    
      }  
      //protothread1(&pt1, 10); 
      //digitalWrite(22, LOW);
      ButtonState();
      for(int kk=0;kk<100;kk++) while(autoKaFState==1){     
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
        ButtonState();      
      }
// ================= LEPAS ISLAND =============================
      int YawNow=0;
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=180){//300       
        ButtonState();
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
      }
      YawNow=sudutYaw;
      if(YawNow<-20)YawNow=-20;      
// ================= BELOK IMU ================================   
//      while(sudutYaw<-10){
//        protothread1(&pt1, 10);
//        Dynamixel.servo(SERVO_ID,CSERVO-450,0x3FF); 
//      }   
      
      int xPlace=0;
      float xVal=0;
      
      Jarak=0;
      digitalWrite(22, HIGH);
      while(sudutYaw<-10){
        protothread1(&pt1, 10);
        //Dynamixel.servo(SERVO_ID,CSERVO+400,0x3FF);
        
        if(xPlace<350) xPlace=Jarak; 
        xVal = (float)xPlace/400 * ((CSERVO-400)-(CSERVO-(125*22/YawNow))) + (CSERVO-(125*22/YawNow));    
        posServo=(int)xVal;  
         
        if(posServo>CCW)      posServo=CCW;
        else if(posServo<CW)  posServo=CW;

         Dynamixel.servo(SERVO_ID,posServo,0x3FF);
         lcd.setCursor(1,12);lcd.print(xPlace);
      }   
      digitalWrite(22, LOW);      
// ================= COUNTER ISLAND =========================== 
      Jarak=0;
      while(Jarak<=1000){
        Heading(0);
      }
// ================= BACA ISLAND 2 =========================== 
      ButtonState();
      digitalWrite(22, HIGH);
      for(int kk=0;kk<100;kk++) while(autoKiFState==1){     
        Heading(0);
        ButtonState();      
      }  
// ================= LEPAS ISLAND 2 =========================== 
      
      Jarak=0;
      while(Jarak<=60){       
        Heading(0);
      }  
      Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
      SetServo(250,CSERVO,CSERVO+350); 
// ====================== TURUN ===============================
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=500){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=4;          
      }
      Jarak=0;
      while(Jarak<=6500){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=3;          
      }
      digitalWrite(22, HIGH);      
      while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=1;
      }
// ====================== TIANG ===============================         
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<300;i++){
          protothread1(&pt1, 10);
          //protothread2(&pt2, 10);
          //pidx=5;
          /*if(sudutYaw>=-92 && sudutYaw<=-88){
            Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
          }
          else{*/
            int sterYaw=-85-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
          //}
        }
        kondisi=3;
        
      }
      //digitalWrite(22, LOW);
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CW,0x3FF);
      }                 
// ========================== BUTTON 1 =========================
    }
//******************** START MERAH 1 SELESAI ********************
 
//========================  START MERAH 2 ======================= 
    if(button==2){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START MERAH 2 ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);      
      lcd.clear(); 
    }
// ========================== BUTTON 2 =========================     
    while(button==2){
// ================= BACA ISLAND ================================
      //protothread1(&pt1, 10); 
      //digitalWrite(22, LOW);
      ButtonState();
      for(int kk=0;kk<100;kk++) while(autoKaFState==1){     
        digitalWrite(22, HIGH);
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
        ButtonState();      
      }
// ================= LEPAS ISLAND =============================
      int YawNow=0;
      digitalWrite(22, LOW);
      Jarak=0;
      while(Jarak<=150){//300       
        ButtonState();
        protothread1(&pt1, 10); //SENSOR
        protothread2(&pt2, 10); //PID
        pidx=2;
      }
      YawNow=sudutYaw;
      if(YawNow<-20)YawNow=-20;      
// ================= BELOK IMU ================================   
//      while(sudutYaw<-10){
//        protothread1(&pt1, 10);
//        Dynamixel.servo(SERVO_ID,CSERVO-450,0x3FF); 
//      }   
      
      int xPlace=0;
      float xVal=0;
      
      Jarak=0;
      digitalWrite(22, HIGH);
      while(sudutYaw<-10){
        protothread1(&pt1, 10);
        //Dynamixel.servo(SERVO_ID,CSERVO+400,0x3FF);
        
        if(xPlace<350) xPlace=Jarak; 
        xVal = (float)xPlace/400 * ((CSERVO-400)-(CSERVO-(125*22/YawNow))) + (CSERVO-(125*22/YawNow));    
        posServo=(int)xVal;  
         
        if(posServo>CCW)      posServo=CCW;
        else if(posServo<CW)  posServo=CW;

         Dynamixel.servo(SERVO_ID,posServo,0x3FF);
         //lcd.setCursor(1,12);lcd.print(xPlace);
      }   
      digitalWrite(22, LOW);      
// ================= COUNTER ISLAND =========================== 
      Jarak=0;
      while(Jarak<=1150){
        Heading(0);
      }
// ================= BACA ISLAND 2 =========================== 
      ButtonState();
      for(int kk=0;kk<100;kk++) while(autoKiFState==1){     
        digitalWrite(22, HIGH);
        Heading(0);
        ButtonState();      
      }  
// ================= LEPAS ISLAND 2 =========================== 
      Jarak=0;
      while(Jarak<=100){       
        Heading(0);
      }  
      Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
      SetServo(250,CSERVO,CSERVO+350); 
// ====================== TURUN ===============================
      digitalWrite(22, LOW);
      
      Jarak=0;
      while(Jarak<=500){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=4;          
      }
      Jarak=0;
      while(Jarak<=6500){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=1;          
      }
      while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=1;
      }
// ====================== TIANG ===============================         
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<300;i++){
          protothread1(&pt1, 10);
          //protothread2(&pt2, 10);
          //pidx=5;
          /*if(sudutYaw>=-92 && sudutYaw<=-88){
            Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
          }
          else{*/
            int sterYaw=-80-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
          //}
        }
        kondisi=3;
        
      }
      //digitalWrite(22, LOW);
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CW,0x3FF);
      }         
// ========================== BUTTON 2 =========================      
    }
//********************* START MERAH 2 SELESAI *******************

//========================  START BIRU 3 =======================
    if(button==3){
      Serial2.write(0xC0);
      lcd.clear();                //0123456789012345
      lcd.setCursor(0,0);lcd.print("  START MERAH 3 ");
      lcd.setCursor(0,0);lcd.print("BISMILLAH CHAIYO");
      delay(300);     
      lcd.clear();  
    }
// ========================== BUTTON 3 =========================     
    while(button==3){
// ====================== TURUN ===============================  
      Jarak=0;
      while(Jarak<7000){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=1;
      }
      while((XSensor & 0xF8) != 0xF8 && (XSensor & 0xFC) != 0xFC && (XSensor & 0xFE) != 0xFE
            && (XSensor & 0xF3) != 0xF3 && (XSensor & 0xF6) != 0xF2 && (XSensor & 0x76) != 0x76
            && (XSensor & 0x36) != 0x36 && (XSensor & 0x6E) != 0x6E && (XSensor & 0x6F) != 0x6F
            && (XSensor & 0xE3) != 0xE3 && (XSensor & 0xB1) != 0xB1
            && (XSensor & 0xB8) != 0xB8 && (XSensor & 0xBC) != 0xBC && (XSensor & 0xBE) != 0xBE
            && (XSensor & 0x1E) != 0x1E && (XSensor & 0x3E) != 0x3E && (XSensor & 0x7E) != 0x7E
            && (XSensor & 0x0F) != 0x0F && (XSensor & 0x1F) != 0x1F && (XSensor & 0x3F) != 0x3F 
            && (XSensor & 0x7F) != 0x7F && (XSensor & 0x0D) != 0x0D && (XSensor & 0x1D) != 0x1D 
            && (XSensor & 0x3D) != 0x3D && (XSensor & 0x7D) != 0x7D && (XSensor & 0xFF) != 0xFF){
        protothread1(&pt1, 10);
        protothread2(&pt2, 10);
        pidx=1;
        digitalWrite(22, HIGH);
     }
// ====================== TIANG =============================== 
      kondisi=2;
      while(kondisi==2){
        for(int i=0;i<300;i++){
          protothread1(&pt1, 10);
          //protothread2(&pt2, 10);
          //pidx=5;
          /*if(sudutYaw>=-92 && sudutYaw<=-88){
            Dynamixel.servo(SERVO_ID,CSERVO,0x3FF);
          }
          else{*/
            int sterYaw=-80-sudutYaw;
            Dynamixel.servo(SERVO_ID,CSERVO-(sterYaw*10),0x3FF);
          //}
        }
        kondisi=3;
        
      }
      while(kondisi==3){
        Dynamixel.servo(SERVO_ID,CW,0x3FF);
      } 
  //  ==================== BUTTON 3 =========================           
    }
//********************* START MERAH 3 SELESAI *******************    
  }
//**************************MERAH SELESAI************************ 

//**************************** LOOP ****************************
 }  

