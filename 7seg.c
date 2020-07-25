#include "7seg.H"

void delay_ms2(unsigned int j)
{
	unsigned int x, i;
	for(i = 0; i < j; i++)
	{
		for(x = 0; x < 10000; x++);
	}
}

unsigned char getAlphaCode(unsigned char alphachar)
{
	switch(alphachar)
	{
		case '0': return 0xc0;
		case '1': return 0xf9;
		case '2': return 0xa4;
		case '3': return 0xb0;
		case '4': return 0x99;
		case '5': return 0x92;
		case '6': return 0x82;
		case '7': return 0xf8;
		case '8': return 0x80;
		case '9': return 0x90;
		case '.': return 0x7f;
		case ' ': return 0xff;
		default : break;
	}
	return 0xff;
}

void alphadisp7seg(char *buf)
{
	unsigned char i, j;
	unsigned char seg7_data, temp = 0;
	for(i = 0; i < 5; i++)
	{
		seg7_data = getAlphaCode(*(buf + i));
		for(j = 0; j < 8; j++)
		{
			temp = seg7_data & 0x80;
			if(temp == 0x80)
				IO0SET |= 1 << 19;
			else
				IO0CLR |= 1 << 19;
			
			IO0SET |= 1 << 20;
			delay_ms2(1);
			IO0CLR |= 1 << 20;
			seg7_data = seg7_data << 1;
		}
	}
	
	IO0SET |= 1 << 30;
	delay_ms2(1);
	IO0CLR |= 1 << 30;
	return;
}
