#ifndef __USART_H
#define __USART_H

extern void KirimData1(unsigned char x);
extern void KirimData2(unsigned char x);
extern void KirimData3(unsigned char x);
extern void SetDynamixel(unsigned char ID,unsigned char Address,unsigned int data);
extern void SetPwm(unsigned char ID,int speed);

#endif
