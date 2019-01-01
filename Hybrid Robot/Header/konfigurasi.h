#ifndef __KONFIGURASI_H
#define	__KONFIGURASI_H

extern void	KonfigurasiGPIO(void);
extern void AllGPIO(void);
extern void	PwmMotor(void);
extern void ServoEDF(void);
extern void RotaryTIM1(void);
extern void RotaryTIM3(void);
extern void RotaryTIM4(void);
extern void RotaryTIM5(void);
extern void	ADCConfig(void);
extern uint16_t ReadADC(unsigned char channel);
extern void Timer(void);
extern void TIM6_Init(void);
extern void SetUSART1(int baudrate);
extern void SetUSART2(int baudrate);
extern void SetUSART3(int baudrate);
extern void NVICConfig(void);
#endif
