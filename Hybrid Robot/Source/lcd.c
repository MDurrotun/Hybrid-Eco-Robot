//BISMILLAAHIRROHMAANIRROHIIM

#include "stm32f4xx.h"
#include "define.h"
#include "main.h"

#include <stdio.h>
char data_f[7];

//*** <<< Use Configuration Wizard in Context Menu >>> ***

//
//=========================================================================== LCD Text Configuration
// <e0> Text LCD Configuration
//   <o1.0..2> LCD DATA PORT
//     <i> Default: PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o2.0..3> Data Position  (LCD D4-D7) 
//     <i> Default: Data = Bit 0-3
//        <0=> Data = Bit 0-3
//        <1=> Data = Bit 1-4  
//        <2=> Data = Bit 2-5
//        <3=> Data = Bit 3-6
//        <4=> Data = Bit 4-7
//        <5=> Data = Bit 5-8
//        <6=> Data = Bit 6-9
//        <7=> Data = Bit 7-10
//        <8=> Data = Bit 8-11 
//        <9=> Data = Bit 9-12
//        <10=> Data = Bit 10-13
//        <11=> Data = Bit 11-14
//        <12=> Data = Bit 12-15
//   <o3.0..2> LCD RS PORT
//     <i> Default: RS_PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o4.0..3> LCD RS BIT
//     <i> Default: RS_BIT = Bit 4 
//		  <0=> RS = Bit 0
//		  <1=> RS = Bit 1
//		  <2=> RS = Bit 2
//		  <3=> RS = Bit 3
//		  <4=> RS = Bit 4
//		  <5=> RS = Bit 5
//		  <6=> RS = Bit 6
//		  <7=> RS = Bit 7
//		  <8=> RS = Bit 8
//		  <9=> RS = Bit 9
//		  <10=> RS = Bit 10
//		  <11=> RS = Bit 11
//		  <12=> RS = Bit 12
//		  <13=> RS = Bit 13
//		  <14=> RS = Bit 14
//		  <15=> RS = Bit 15
//   <o5.0..2> LCD Enable PORT
//     <i> Default: Enable_PORT = GPIOB
//        <0=> GPIOA (sometimes used for ADC)
//        <1=> GPIOB (set as the alternate function of JTAG pin)
//        <2=> GPIOC (only for Chip 64 pin and above)
//        <3=> GPIOD (only for Chip 100 pin and above)
//        <4=> GPIOE (only for Chip 100 pin and above)
//        <5=> GPIOF (only for Chip 100 pin and above)
//        <6=> GPIOG (only for Chip 144 pin and above)
//   <o6.0..3> LCD Enable BIT
//     <i> Default: Enable_BIT = Bit 5 
//		  <0=> Enable = Bit 0
//		  <1=> Enable = Bit 1
//		  <2=> Enable = Bit 2
//		  <3=> Enable = Bit 3
//		  <4=> Enable = Bit 4
//		  <5=> Enable = Bit 5
//		  <6=> Enable = Bit 6
//		  <7=> Enable = Bit 7
//		  <8=> Enable = Bit 8
//		  <9=> Enable = Bit 9
//		  <10=> Enable = Bit 10
//		  <11=> Enable = Bit 11
//		  <12=> Enable = Bit 12
//		  <13=> Enable = Bit 13
//		  <14=> Enable = Bit 14
//		  <15=> Enable = Bit 15
// </e> End Text LCD Configuration


//*** <<< end of configuration section >>>    ***


#define __LCD_CHAR            		 	1                //  
#define __LCD_DATA_PORT          	0x000000003              //  port
#define __LCD_DATA_BIT          	0x000000000              //  posisi Data
#define __LCD_RS_PORT            	0x000000003              //  PORT RS
#define __LCD_RS_BIT            	0x000000004              //  posisi Bit RS
#define __LCD_EN_PORT            	0x000000003              //  PORT Enable
#define __LCD_EN_BIT            	0x000000005              //  posisi Bit Enable


// Deklarasikan Port sambungan LCD 
#if __LCD_CHAR
// inisalisasi LCD karakter 
    #if ((__LCD_DATA_PORT & 7)==0)	
		#define LCD_PORT   	GPIOA->ODR
	#elif ((__LCD_DATA_PORT & 7)==1)
		#define LCD_PORT   	GPIOB->ODR
	#elif ((__LCD_DATA_PORT & 7)==2)
		#define LCD_PORT   	GPIOC->ODR
	#elif ((__LCD_DATA_PORT & 7)==3)
		#define LCD_PORT   	GPIOD->ODR
	#elif ((__LCD_DATA_PORT & 7)==4)
		#define LCD_PORT   	GPIOE->ODR
	#elif ((__LCD_DATA_PORT & 7)==5)
		#define LCD_PORT   	GPIOF->ODR
    #elif ((__LCD_DATA_PORT & 7)==6)
		#define LCD_PORT   	GPIOG->ODR
    #endif	

	#if ((__LCD_RS_PORT & 7)==0)	
		#define LCD_RS		GPIOA
	#elif ((__LCD_RS_PORT & 7)==1)
		#define LCD_RS   	GPIOB
	#elif ((__LCD_RS_PORT & 7)==2)
		#define LCD_RS   	GPIOC
	#elif ((__LCD_RS_PORT & 7)==3)
		#define LCD_RS   	GPIOD
	#elif ((__LCD_RS_PORT & 7)==4)
		#define LCD_RS   	GPIOE
	#elif ((__LCD_RS_PORT & 7)==5)
		#define LCD_RS   	GPIOF
    #elif ((__LCD_RS_PORT & 7)==6)
		#define LCD_RS   	GPIOG
    #endif	

	#if ((__LCD_EN_PORT & 7)==0)	
		#define LCD_EN   	GPIOA
	#elif ((__LCD_EN_PORT & 7)==1)
		#define LCD_EN   	GPIOB
	#elif ((__LCD_EN_PORT & 7)==2)
		#define LCD_EN   	GPIOC
	#elif ((__LCD_EN_PORT & 7)==3)
		#define LCD_EN   	GPIOD
	#elif ((__LCD_EN_PORT & 7)==4)
		#define LCD_EN   	GPIOE
	#elif ((__LCD_EN_PORT & 7)==5)
		#define LCD_EN   	GPIOF
    #elif ((__LCD_EN_PORT & 7)==6)
		#define LCD_EN   	GPIOG
    #endif	
//
									  

//#include <stm32f10x_lib.h>

#define lcd_wait_const 	336000000 / 8769227
#define lcd_delay		75000	 

void wait_lcd(unsigned long int xx)
{  xx*= lcd_wait_const;	
   while(xx--);
}

void lcd_en_clk(void)
{		SetBit(LCD_EN,__LCD_EN_BIT);				// set enable
		wait_lcd(10);					 			// tunggu 2us
		ClrBit(LCD_EN,__LCD_EN_BIT);				// reset enable 
} 

void lcd_ins(unsigned short xx)
{		ClrBit(LCD_RS,__LCD_RS_BIT);				// set enable
		LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data
		lcd_en_clk();
}

void lcd_ins2(unsigned short xx)
{		SetBit(LCD_RS,__LCD_RS_BIT);				// set enable
		LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data
		lcd_en_clk();
}

void lcd_cmd (unsigned char cmd)
{		lcd_ins ((cmd>>4) & 0x0F);	// kirim nibble high	
		lcd_ins (cmd & 0x0F);		// kirim nibble low
		wait_lcd(50);				// tunggu 0.05ms
}

void lcd_data (unsigned char dat)
{		lcd_ins2 ((dat>>4) & 0x0F);	// kirim nibble high	
		lcd_ins2 (dat & 0x0F);		// kirim nibble low
		wait_lcd(50);				// tunggu 0.05ms
}
					  
void lcd_reset(void)
{		LCD_PORT |= (0xF<<__LCD_DATA_BIT);
		SetBit(LCD_RS,__LCD_RS_BIT);
		SetBit(LCD_EN,__LCD_EN_BIT);
		wait_lcd(20000);			// tunggu 20ms
		lcd_ins(3);					// reset #1
		wait_lcd(15000);			// tunggu 15ms
		lcd_ins(3);					// reset #2
		wait_lcd(5000);				// tunggu 5ms
		lcd_ins(3);					// reset #3
		wait_lcd(5000);				// tunggu 5ms
		lcd_ins(2);					// set data transfer 4 bit						
		wait_lcd(5000);				// tunggu 5ms
}

void lcd_init(void)
{		lcd_reset();
		lcd_cmd(0x28);				//LCD yang digunakan  = data 4 bit, 2 baris, 5x7 dots
		wait_lcd(1000);		
		lcd_cmd(0x0c);				//Display ON cursor OFF
		wait_lcd(1000);		
		lcd_cmd(0x06);				//Set entry mode - auto increement 
		wait_lcd(1000);		
		lcd_cmd(0x01);
		wait_lcd(2000);		
		lcd_cmd(0x80);
		wait_lcd(2000);		
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{	ClrBit(LCD_RS,__LCD_RS_BIT);	
	lcd_cmd(0x80 | (y*0x40) + x);				
}

void lcd_clear(void)
{	 lcd_cmd(1);delay_us(1);}

// menampilkan string ke lCD

void lcd_string(const char *xx)
{	while(*xx) lcd_data(*xx++);
}

void lcd_Test(unsigned char xx)
{	ClrBit(LCD_EN,__LCD_EN_BIT);				// set enable
	SetBit(LCD_RS,__LCD_RS_BIT);				// set enable
    LCD_PORT  = (LCD_PORT & ~(0xF << __LCD_DATA_BIT)) | (xx<<__LCD_DATA_BIT);	// kirim data	
}


// menampilkan data integer ke LCD
void lcd_uint32(unsigned short xx)
{  lcd_data(xx/10000 + 0x30);
   lcd_data((xx%10000)/1000 + 0x30);
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint16(unsigned short xx)
{  lcd_data(xx/10000 + 0x30);
   lcd_data((xx%10000)/1000 + 0x30);
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint8(unsigned short xx)
{  //lcd_data(xx/10000 + 0x30);
   //lcd_data((xx%10000)/1000 + 0x30);
   lcd_data((xx%1000)/100 + 0x30);
   lcd_data((xx%100)/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint2(unsigned short xx)
{  
   lcd_data(xx/10 + 0x30);
   lcd_data(xx%10 + 0x30);
}

void lcd_uint1(unsigned short xx)
{  
   lcd_data(xx%10 + 0x30);
}

void lcd_float4(float xx)
{
	sprintf(data_f,"%10.8f",xx);
	lcd_data(data_f[0]);
	lcd_data(data_f[1]);
	lcd_data(data_f[2]);
	lcd_data(data_f[3]);

}

void lcd_float6(float xx)
{
	sprintf(data_f,"%10.8f",xx);
	lcd_data(data_f[0]);
	lcd_data(data_f[1]);
	lcd_data(data_f[2]);
	lcd_data(data_f[3]);
	lcd_data(data_f[4]);
	lcd_data(data_f[5]);
}

void lcd_int16 (short int xx) 
{  if(xx<0)
   {   
   	   lcd_data('-');
       xx=-xx;
   }
   else lcd_data(' ');	      
   lcd_uint16(xx); 
}

void lcd_int8 (short int xx) 
{  if(xx<0)
   {   
   	   lcd_data('-');
       xx=-xx;
   }
   else lcd_data(' ');	      
   lcd_uint8(xx); 
}

// menampilkan data dalam bentuk hexa desimal

void lcd_hex (unsigned char xx)
{  if (xx<10) lcd_data(xx + 48);	   	// menampilkan angka
   else       lcd_data(xx + 55);		// menampilkan huruf				   	
} 

void lcd_bin (unsigned char xx)
{  unsigned char i;
   for(i=0;i<16;i++)
   {	if (!(xx&(1<<i))) 	lcd_data('0');
   	else         		lcd_data('1');
   }
} 

void lcd_bin1 (unsigned char xx)
{  
   	if (!(xx)) 	lcd_data('0');
   	else         		lcd_data('1');
   
} 

void lcd_hex8 (unsigned char xx)
{  lcd_hex(xx>>4);
   lcd_hex(xx & 15);
} 


void lcd_hex16(unsigned short int xx) 
{  lcd_hex((xx>>12) & 0xF);
   lcd_hex((xx>>8) & 0xF);
   lcd_hex((xx>>4) & 0xF);
   lcd_hex(xx & 0xF);
}

void lcd_hex32(unsigned long int xx) 
{  lcd_hex16((xx>>16) & 0xFFFF);
   lcd_hex16(xx & 0xFFFF);
}		  

// animasi tampilan string dari LCD

void lcd_tengah(unsigned char kolom, const char *xx, unsigned char x)
{   	unsigned char baseline;
	unsigned char i,j;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	kolom/=2;
	for(i=0;i<kolom;i++)
	{ j = kolom-1-i;
	  lcd_cmd(baseline + j);
	  lcd_data(*(xx+j));
	  j = kolom+i;
	  lcd_cmd(baseline + j);
	  lcd_data(*(xx+j));
	  wait_lcd(lcd_delay);
	}
}

void lcd_hapus_tengah(unsigned char kolom, unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	kolom/=2;
	for(i=0;i<kolom;i++)
	{ lcd_cmd(baseline + kolom -1 - i);
	  lcd_data(' ');
	  lcd_cmd(baseline + kolom + i);
	  lcd_data(' ');
	  wait_lcd(lcd_delay);
	}
}

void lcd_pinggir(unsigned char kolom, const char *xx, unsigned char x)
{	unsigned char baseline;
	unsigned char i,j;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	for(i=0;i<kolom/2;i++)
	{ j = i;
	  lcd_cmd(baseline + j);
	  lcd_data(*(xx+j));
	  j = kolom-1-i;
	  lcd_cmd(baseline + j);
	  lcd_data(*(xx+j));
	  wait_lcd(lcd_delay);
	}
}

void lcd_hapus_pinggir(unsigned char kolom, unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	for(i=0;i<kolom/2;i++)
	{ lcd_cmd(baseline + i);
	  lcd_data(' ');
	  lcd_cmd(baseline + kolom - 1 - i);
	  lcd_data(' ');
	  wait_lcd(lcd_delay);
	}
}


void lcd_kiri(unsigned char kolom, const char *xx, unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	for(i=0;i<kolom;i++)
	{ lcd_cmd(baseline + i);
	  lcd_data(*(xx+i));
	  wait_lcd(lcd_delay);
	}
}

void lcd_hapus_kiri(unsigned char kolom, unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    	if (x)    baseline = 0x0C0;
	else      baseline = 0x080;
	for(i=0;i<kolom;i++)
	{ lcd_cmd(baseline + i);
	  lcd_data(' ');
	  wait_lcd(lcd_delay);
	}
}

void lcd_kanan(unsigned char kolom, const char *xx, unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    if (x)    baseline = 0x0C0 + kolom - 1;
	else      baseline = 0x080 + kolom - 1;
	for(i=0;i<kolom;i++)
	{ lcd_cmd(baseline - i);
	  lcd_data(*(xx+kolom-1-i));
	  wait_lcd(lcd_delay);
	}
}

void lcd_hapus_kanan(unsigned char kolom,unsigned char x)
{	unsigned char baseline;
	unsigned char i;
    if (x)    baseline = 0x0C0 + kolom - 1;
	else      baseline = 0x080 + kolom - 1;
	for(i=0;i<kolom;i++)
	{ lcd_cmd(baseline - i);
	  lcd_data(' ');
	  wait_lcd(lcd_delay);
	}
}


#endif
