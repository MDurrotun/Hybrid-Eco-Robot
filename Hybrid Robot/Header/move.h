#ifndef __MOVE_H
#define __MOVE_H

extern void Odometry(void);
extern void ResetOdometry(void);
extern void GyrodoJalan(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan2(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan3(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan3Rev(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan4(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void CobaOdoJalan4(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void CobaOdoJalan3(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan4Rev(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoJalan5(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void OdoHillZigzag(float Xt,float Yt,int Speed,int PStop,int edf,int servo,int Katrol);
extern void BelokKiri(int sudut,int speed);
extern void BelokKanan(int sudut,int speed);
extern void BelokGyro(int sudut,int speed);
extern void BelokGyros(int sudut,int speed);
extern void Parkir(void);
extern void PanjatTiang(void);
extern void Gyrodometry(void);
extern void Gyro(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake);
extern void Gyro2(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake);
extern void GyroEDF(int Kp, int Kd, int sudut, int kec,int jarak);
extern void MoveTiang(unsigned char trace,int threshold,int heading,int kec);
extern void MoveTiangUS(unsigned char trace,int threshold,int kec);
extern void MoveTiangUSRed(unsigned char trace,int threshold,int kec);
extern void ClimbControl(int Kp,int Kd,int sudut, int mode);
extern void ClimbControlBiru(int Kp,int Kd,int sudut,int mode);
extern void HeadingLOCK(int sudut);
extern void MoveTiang2(unsigned char trace,int threshold,int dista,int kec);
extern void GyroTiang(int Kp, int Kd, int sudut, int kec);
extern void MoveGyro(int Kp, int Kd, int sudut, int kec,int jarak);
extern void MoveGyro2(int Kp, int Kd, int sudut, int kec,int jarak);
extern void MoveGyroRe(int Kp, int Kd, int sudut, int kec,int jarak,int Soft);
extern void MoveGyroRe2(int Kp, int Kd, int sudut, int kec,int jarak,int Soft);
extern void MoveGyroRe3(int Kp, int Kd, int sudut, int kec,int PStop);
extern void GyroPutar(int Kp, int Kd, int edf, int servo, int Katrol, int sudut, int kec,int jarak,unsigned char brake);
extern void GyroHill(int Kp, int Kd, int sudut, int kec,int jarak,unsigned char brake, int edf, int servo, int katrol);
extern void OdoJalanEDF(float Xt,float Yt,int Speed,int PStop,int edf,int Katrol,int awal, int akhir, int jarak);
extern void USRed(int threshold,int edf,int kec, int jarak);
extern void USBlue(int threshold,int servo,int kec, int jarak);
extern void ClimbControlBiruReverse(int Kp,int Kd,int sudut,int mode);
extern void MoveGyroLurus(int Kp, int Kd, int sudut, int kec);
extern void UbahServoMerah(int SudutServo);
extern void UbahServoBiru(int SudutServo);
extern void GyroUSJalanMerah();
extern void GyroUSJalanBiru();
extern void OdoJalanEDFBlue(float Xt,float Yt,int Speed,int PStop,int edf,int Katrol,int awal, int akhir, int jarak, int mode);
#endif