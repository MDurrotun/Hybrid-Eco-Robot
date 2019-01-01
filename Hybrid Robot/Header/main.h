#ifndef __MAIN_H
#define __MAIN_H

extern void TimingDelay_Decrement(void);
extern void delay_ms(__IO uint32_t nTime);
extern void delay_us(__IO uint32_t xx);
extern void SudutIMU(void);
extern unsigned char EDFCalib;
extern void TampilInt(float w, float x, float y, float z);
extern void TampilFloat(float w, float x, float y, float z);
extern void BacaUS1(void);
extern void BacaUS2(void);
extern void BacaUS3(void);
extern void menu(void);
extern void TangkapPropeler1(void);
extern void TangkapPropeler2(void);
extern void SiapPropeler(void);
extern void SiapPropeler90(int speed);
extern void LepasPropeler(void);
extern void PasangPropeler1(void);
extern void DynamixelEDF(int32_t theta);
extern void ChaiYoMerah(int theta,int mode);
extern void ChaiYoBiru(int theta,int modex);
extern void TiupEDF(int servo,int edf,int delay);
extern uint16_t ThresholdUS,CurrentADC,DistStart,ThresUSTiang,timelift;
extern float  JarakUS1,JarakUS2,JarakUS3,corr;
extern int DataADC,i,n,heads,dist,SudutZigzag,headtres,MinEDF,a;
extern uint16_t CurrentADC,ThresholdUS,PosServo,PosDynamixelEDF,tutupdyx,bukadyx;
extern unsigned char EDFCalib,select,USWall,flags,flagstr,flagnaik,flagbtn,flagtiang,flaglift;

#endif
