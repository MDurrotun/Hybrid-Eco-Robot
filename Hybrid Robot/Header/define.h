#ifndef	__DEFINE_H
#define	__DEFINE_H

#define SetBit(x,y) 	x->BSRRL = (1<<y);
#define ClrBit(x,y) 	x->BSRRH  = (1<<y);
#define Pin(Port,x)	    (Port->IDR&(1<<x))
//membuat regx 1
#define SetA(y) 		GPIOA->BSRRL = (1<<y);
#define SetB(y) 		GPIOB->BSRRL = (1<<y);
#define SetC(y) 		GPIOC->BSRRL = (1<<y);
#define SetD(y) 		GPIOD->BSRRL = (1<<y);
#define SetE(y) 		GPIOE->BSRRL = (1<<y);
#define SetF(y) 		GPIOF->BSRRL = (1<<y);
#define SetG(y) 		GPIOG->BSRRL = (1<<y);
//membuat regx 0
#define ClrA(y) 		GPIOA->BSRRH  = (1<<y);
#define ClrB(y) 		GPIOB->BSRRH  = (1<<y);
#define ClrC(y) 		GPIOC->BSRRH  = (1<<y);
#define ClrD(y) 		GPIOD->BSRRH  = (1<<y);
#define ClrE(y) 		GPIOE->BSRRH  = (1<<y);
#define ClrF(y) 		GPIOF->BSRRH  = (1<<y);
#define ClrG(y) 		GPIOG->BSRRH  = (1<<y);
#define	posisi			lcd_gotoxy(0,0);

#define delay_const	(uint32_t) (336000000 / 15574784)

//dinamixel
#define Dxl_read_data			2
#define Dxl_write_data		3
#define Dxl_reg_write			4
#define Dxl_action				5
#define Dxl_reset					6
#define Dxl_sync_write		0x83



#define	PwmEDF		TIM13->CCR1

//=======US Depan Kiri======// 
#define	US1On				SetC(4)
#define EUS1				Pin(GPIOC,5)
#define US1Off			ClrC(4)

//=======US Belakang Kiri======//
#define	US3On				SetA(5)
#define	EUS3				Pin(GPIOA,4)
#define	US3Off			ClrA(5)


#define	Dir1_1			SetD(7)
#define	Ena1_1			SetD(6)
#define	Dir1_0			ClrD(7)
#define	Ena1_0			ClrD(6)

#define	Dir2_1			SetE(2)
#define	Ena2_1			SetC(13)
#define	Dir2_0			ClrE(2)
#define	Ena2_0			ClrC(13)


#define ToggleTombol		Pin(GPIOB,1)
#define KuningKiri			Pin(GPIOB,0)
#define KuningKanan			Pin(GPIOD,11)
#define MerahTengah			Pin(GPIOE,7)

#define	AdjustDx				Pin(GPIOE,8)
#define	LimJepit					Pin(GPIOE,10)
#define LimStop				Pin(GPIOE,13)
#define	LimTurun				Pin(GPIOE,12)
#define LimAtas					Pin(GPIOE,15)

#define LimLengan					Pin(GPIOE,14)
#define	LimTiang				Pin(GPIOB,11)
#define	LimNaik			Pin(GPIOB,10)  //limit jepit
#define	LimTutup				Pin(GPIOB,13)  //limit tutup jepit
#define LimBuka				Pin(GPIOB,12)

#define STOP	{SetPwm(0xE1,0);SetPwm(0xE2,0);ClrC(9);ClrC(8);ClrC(7);ClrC(6);ClrB(14);ClrB(15);ClrB(8);ClrB(9);SetC(13);SetD(6);PwmEDF=50;}

//========ROTARI========//
#define PI					3.142857f
#define Rka					1.860f//1.79f
#define Rki					1.965f//1.795f
//#define Rka					1.913f//1.79f
//#define Rki					1.922f//1.795f
#define	R2					1.10f
#define R1_tik			(0.5f*(PI*Rka)/100)
#define	R2_tik			(0.5f*(PI*Rki)/100)
#define R3_tik			(0.5f*(PI*R2)/100)
#define RADS				57.2957795f
#define wheelbase		34.4f



#define RETRY 	0
#define NORMAL 	1

#define	MASUK		0
#define	KELUAR	1

#define	SOFT		0
#define	NOSOFT	1

#define	MKiri		0xE1
#define	MKanan	0xE2

#endif
