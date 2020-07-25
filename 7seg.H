#ifndef _7seg_H_
#define _7seg_H_

#include <lpc214x.h>

 void delay_ms2(unsigned int j);
 unsigned char getAlphaCode(unsigned char alphachar);
 void alphadisp7seg(char *buf);
 
#endif
