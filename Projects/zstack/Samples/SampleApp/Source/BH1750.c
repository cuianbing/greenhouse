#include <ioCC2530.h>
#include "OnBoard.h"


#define uint unsigned int 
#define uchar unsigned char
#define ushort unsigned short


#define st(x)      do { x } while (__LINE__ == -1)
#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin## = val; )
#define HAL_IO_GET(port, pin)   HAL_IO_GET_PREP( port,pin)
#define HAL_IO_GET_PREP(port, pin)   ( P##port##_##pin)
 
 
#define LIGHT_SCK_0()         HAL_IO_SET(1,3,0)
#define LIGHT_SCK_1()         HAL_IO_SET(1,3,1)
#define LIGHT_DTA_0()         HAL_IO_SET(1,1,0)
#define LIGHT_DTA_1()         HAL_IO_SET(1,1,1)
 
#define LIGHT_DTA()          HAL_IO_GET(1,1)
#define LIGHT_SCK()          HAL_IO_GET(1,3)
 
#define SDA_W() (P1DIR |=(1 << 1)  )
#define SDA_R() (P1DIR &=~(1 << 1) )
 
                       
#define LIGHT_INIT()  do{P1SEL &= ~0x08;P1DIR |=0x08;P1_3 = 1;P1SEL &= ~0x02;P1DIR |= 0x02;P1_1 = 1;}while(0);


void halMcuWaitUs(uint usec);
void halMcuWaitMs(uint msec);
void delay_us();
void delay_5us();
void delay_10us();
void delay_nms(int n);
ushort get_light(void);

void halMcuWaitUs(uint usec)
{
    while(usec--)
    {
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
    }
}
//ÒÔmsÑÓÊ±
void halMcuWaitMs(uint msec)
{
    while(msec--)
        halMcuWaitUs(1000);
}
 

void delay_nms(int n)
{
  halMcuWaitMs(n);
}
/****************************/
static void start_i2c(void)
{
  SDA_W() ;
  LIGHT_DTA_1();//
  LIGHT_SCK_1() ;//
  MicroWait(1);
  LIGHT_DTA_0() ;
  MicroWait(1);
  LIGHT_SCK_0() ;
  MicroWait(1);
  //delay()  ;
}
 
static void stop_i2c(void)
{
  SDA_W() ;
  LIGHT_DTA_0() ;
   MicroWait(1);
  LIGHT_SCK_1() ;
   MicroWait(1);
  LIGHT_DTA_1() ;
  MicroWait(1);
  LIGHT_SCK_0() ;
  MicroWait(1);  
}
static char i2c_send(uchar val)                 
{
        int i;
        char error=0;
        SDA_W();
        for(i=0x80;i>0;i/=2)
		{
			if(val&i)
				LIGHT_DTA_1();
			else
				LIGHT_DTA_0();
			 MicroWait(1);
			LIGHT_SCK_1() ; 
			 MicroWait(1);
			LIGHT_SCK_0() ;
			 MicroWait(1);					
		}
        LIGHT_DTA_1();
        SDA_R();
         MicroWait(1);
        //delay_us();
        LIGHT_SCK_1() ; 
         MicroWait(1);
        if(LIGHT_DTA())
            error=1;
         MicroWait(1);
        LIGHT_SCK_0() ;
        return error;
        
}
static char i2c_read(char ack)
{
        int i;
        char val=0;
        LIGHT_DTA_1();
        //SDA_R();
        for(i=0x80;i>0;i/=2)
                {
                        
                        LIGHT_SCK_1() ;
                        MicroWait(1);
                        SDA_R();
                        //SDA_W();
                        //LIGHT_DTA_0();
                        //LIGHT_DTA_0() ;
                        
                        //delay_us();
                        if(LIGHT_DTA())
                                val=(val|i);
                         MicroWait(1);
                        //SDA_R();
                        LIGHT_SCK_0() ;
                         MicroWait(1);
                        
                        
                }
        SDA_W();
        if(ack)
                LIGHT_DTA_0();
        else
                LIGHT_DTA_1();
         MicroWait(1);
        LIGHT_SCK_1() ;
         MicroWait(1);
        LIGHT_SCK_0() ;
        LIGHT_DTA_1();
        return val;
        
}
ushort get_light(void)
{        
        uchar ack1=1;
        uchar ack2=1;
        uchar ack3=1;
        uchar ack4=1;
        uchar ack5=1;
        uchar ack6=1;
        uchar ack7=1;
        
        uchar t0;
        uchar t1;
        ushort t;
 
        P1DIR |= (1 << 1);
        delay_nms(200);
 
		start_i2c();
		ack1=i2c_send(0x46);
		if(ack1)
				return 255;
		ack2=i2c_send(0x01);
		if(ack2)
				return 254;
		stop_i2c();           //init
		start_i2c();
		ack3=i2c_send(0x46);
		if(ack3)
				return 253;
		ack4=i2c_send(0x01);
		if(ack4)
				return 252;
		stop_i2c();//power
		start_i2c();
		ack5=i2c_send(0x46);
		if(ack5)
				return 251;
		ack6=i2c_send(0x10);
		if(ack6)
				return 250;
		stop_i2c();                     
        delay_nms(1500);
        start_i2c();
        
		ack7=i2c_send(0x47);
		if(ack7)
				return 249;
                        
        t0 = i2c_read(1);
        t1 = i2c_read(0);
        stop_i2c();
        t =  ((short)t0)<<8;
        t |= t1;
        return t;
}
 
