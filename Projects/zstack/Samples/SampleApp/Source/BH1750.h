#ifndef __BH1750_H
#define __BH1750_H

#define uint unsigned int 
#define uchar unsigned char
#define ushort unsigned short

extern ushort get_light(void);
extern void halMcuWaitMs(uint msec);
extern void delay_nms(int n);
extern void halMcuWaitUs(uint usec);
extern void delay_us();
extern void delay_5us();
extern void delay_10us();
extern void delay_nms(int n);
#endif // __BH1750_H
