/*
Animal Health Monitoring System
Used LM35, 1157, 4118, LED, LCD, Internal RTC, GPIO, Timer0 & Timer1, UART0
*/

#include <lpc214x.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "LCD.H"
#include "7seg.H"
#include "uart.H"

#define HR_SENSOR (IO0PIN & 1 << 5)
#define PIR (IO1PIN & 1 << 20)
	
typedef struct
{
	unsigned char sec;
  unsigned char min;
  unsigned char hour;
  unsigned char weekDay;
  unsigned char date;
  unsigned char month;
  unsigned int year;  
}rtc_t;

void SystemInit(void);//initialize CCLK and PCLK
void Board_Init(void);//initialize GPIO
void RTC_Init(void);
void timer1_Init(void);// generates interrupt every 1sec
void delay(int cnt);
void RTC_SetDateTime(rtc_t *rtc);
void RTC_GetDateTime(rtc_t *rtc);
void runDCMotor(unsigned int direction,unsigned int speed);
unsigned int adc(int no,int ch);// to read LDR(AD1.3),LM35(AD1.5)
void serialPrint(unsigned val);//print int on serialport
void serialPrintStr(char * buf);//print string on serialport
void beep(unsigned int val);
int readSensor(int sen_no);
void hrsense(void);
void GPS(void);
void GPS_Init(void);
void get_Time(void);
void get_Latitude(uint16_t Latitude_Pointer);
void get_Longitude(uint16_t Longitude_Pointer);
void get_Altitude(uint16_t Altitude_Pointer);


//global variables
rtc_t rtc; // declare a variable to store date,time

unsigned  int beatms = 0, temp = 0, tempc = 0, tempf = 32, count = 0, count1 = 0;
float bpm;
char buf[50];
char sbuffer[30], ch;
unsigned char pos;
char Latitude_Buffer[15],Longitude_Buffer[15],Time_Buffer[15],Altitude_Buffer[8];
char iir_val[10];
char GGA_String[150];
uint8_t GGA_Comma_Pointers[20];
char GGA[3];
volatile uint16_t GGA_Index, CommaCounter;
bool IsItGGAString;	
//unsigned char read1, read2, read3;


unsigned  int x = 0;
// ISR Routine to blink LED D7 to indicate project working
__irq   void  Timer1_ISR(void)
 {
	x = ~x;//x ^ 1;
  if (x)   
    IO0SET  =  1u << 31;   //P0.31  =  1
  else   
    IO0CLR =   1u <<31;   // P0.31  = 0	 
	T1IR  =  0x01; // clear match0 interrupt, and get ready for the next interrupt
  VICVectAddr = 0x00000000 ; //End of interrupt 
}
 
__irq void UART0_Interrupt(void)
{
	int iir_value;
	char received_char;
	iir_value = U0IIR;
		received_char = U0RBR;
			if( received_char == '$' )
			{
				GGA_Index = 0;
				CommaCounter = 0;
				IsItGGAString = false;
			}
			else if( IsItGGAString == true )	/* If $GPGGA string */
			{
				if ( received_char == ',' )	
				{
					GGA_Comma_Pointers[CommaCounter++] = GGA_Index;	/* Store locations of commas in the string in a buffer */
				}
				GGA_String[GGA_Index++] = received_char;	/* Store the $GPGGA string in a buffer */
			}
			else if( ( GGA[0] == 'G' ) && ( GGA[1] == 'G' ) && ( GGA[2] == 'A' ) )	/* If GGA string received */
			{
				IsItGGAString = true;
				GGA[0] = GGA[1] = GGA[2] = 0;
			}
			else	/* Store received character */
			{
				GGA[0] = GGA[1];
				GGA[1] = GGA[2];
				GGA[2] = received_char;
			}
	VICVectAddr = 0x00;
} 
 
int main() 
{
    unsigned char msg[100];
	  char *buff;
	  unsigned int count2 = 0; 
  
 // initialize the peripherals & the board GPIO
	
    	  Board_Init();
    	  SystemInit();
	  UART1_init();
	  RTC_Init();
	  timer1_Init(); 
	  LCD_Reset();
		LCD_Init();
	
	  // set date & time to 7thApril 2020,10:00:00am

		
	
    rtc.hour = 17;rtc.min =  00;rtc.sec =  00;//5:00:00pm
    rtc.date = 20;rtc.month = 04;rtc.year = 2020;//15th April 2020
    RTC_SetDateTime(&rtc);  // comment this line after first use
	
	  GPS_Init();
		UART0_init();
	 
    while(1)
    {
      RTC_GetDateTime(&rtc);//get current date & time stamp
			sprintf((char *)msg,"time:%2d:%2d:%2d  Date:%2d/%2d/%2d \x0d\xa",(uint16_t)rtc.hour,(uint16_t)rtc.min,(uint16_t)rtc.sec,(uint16_t)rtc.date,(uint16_t)rtc.month,(uint16_t)rtc.year);
			UART0_SendString((char *)msg);
			// use the time stored in the variable rtc for date & time stamping
			LCD_CmdWrite(0x80); LCD_DisplayString((char*)msg);
			delay_ms(500);
			
			hrsense();
			delay_ms(250);
			temp    = adc(1,4);//readTemp();
			tempc = (temp * 3.3 * 100) / 1024;
		  tempf = ((tempc*9)/5 + 32);
			
		  if(tempf > 99)
			{
				count1++;
			}
			
			if(count1 > 10)
			{
				count1 = 0;
				beep(750);
				delay_ms(500);
				beep(0);
			}
			
			sprintf((char *)buf,"Body temperature in oC, oF :%5d, %5d \x0d\xa",tempc, tempf);
			UART0_SendString(buf);
			LCD_CmdWrite(0xc0);		LCD_DisplayString((char *)buf);	
			delay_ms(500);
			
			/*ch = 0x00;
			while(ch != 0x0A)
			{
				while((U0LSR & (0x01<<0))== 0x00){}; // wait till, a character (8bit) is received from PC
				ch = U0RBR;
				//store serial data to buffer
				sbuffer[pos] = ch;
				pos++;
	    }
			
			// if received character is <LF>, 0x0A, 10 then process buffer
			pos = 0; // buffer position reset for next reading
					
			// extract data from serial buffer to 8 bit integer value
			// convert data from ASCII to decimal
      read1 = ((sbuffer[1]-'0')*100) + ((sbuffer[2]-'0')*10) +(sbuffer[3]-'0');
      read2 = ((sbuffer[6]-'0')*100) + ((sbuffer[7]-'0')*10) +(sbuffer[8]-'0');
      read3 = ((sbuffer[11]-'0')*100) + ((sbuffer[12]-'0')*10) +(sbuffer[13]-'0');
					
			// Value of variables will be between 0-255
			sprintf((char *)buf,"Blood pressure is:%5d / %5d \x0d\xa",read1, read2);
			LCD_CmdWrite(0x94);		LCD_DisplayString((char *)buf);
			sprintf((char *)buf,"Pulse is:%5d \x0d\xa",read3);
			LCD_CmdWrite(0xD4);		LCD_DisplayString((char *)buf);
					
			if(read1 < 90 || read1 > 119 || read2 < 60 || read2 > 79)
			{
				//Abnormal blood pressure
				beep(512);
				delay_ms(1000);
				beep(0);
			}
					
			if(read3 > 200)
			{
				//Abnormal heart rate
				beep(1023);
				delay_ms(1000);
				beep(0);
			}*/
			
			
			
			if(PIR == 0) //humans detected, probably poachers
			{
				sprintf((char *)msg,"Intruder detected nearby \x0d\xa");
				UART0_SendString((char *)msg);
				count2++;
				if(count2 > 50)
				{
					beep(1023);
					delay_ms(500);
					beep(0);
					count2 = 0;
				}
			}
			
			//GPS();
			
			//Sending data to RPi
			
			sprintf((char *)buf,"%5f \x0d\xa",bpm);
			UART1_SendString((char *)buf);
			sprintf((char *)buf,"%5d \x0d\xa",tempc);
			UART1_SendString((char *)buf);
			sprintf((char *)buf,"%5d \x0d\xa",tempf);
			UART1_SendString((char *)buf);
			
			
				

			//To display animal body temperature and heart rate on 7 segment display
			/*sprintf((char *)buff,"%5f \x0d\xa",bpm);
			alphadisp7seg(&buff[0]);
			delay(2000);
			sprintf((char *)buff,"%5d \x0d\xa",tempc);
			alphadisp7seg(&buff[0]);
			delay(2000);
			sprintf((char *)buff,"%5d \x0d\xa",tempf);
			alphadisp7seg(&buff[0]);*/
			
    }
}

void Board_Init(void)
{
	
	IO0DIR |= 1 << 11; // RELAY IS CONNECTED TO P0.11
	IO0DIR |= 1U << 31 | 0x00FF0000 | 1U << 30; // to set P0.16 to P0.23 as o/ps ,
  	IO1DIR |= 1U << 25 | 1U << 24;	              // to set P1.25 as o/p used for EN, P1.24 to enable sensor
                                    // make D7 Led (P0.31) on off for testing	
	PINSEL1 |= 0x00080000;			//P0.25 as DAC output: option 3 (bits 18, 19) for buzzer
}
unsigned int adc(int no,int ch)
{
  // adc(1,4) for temp sensor LM35, digital value will increase as temp increases
	// adc(1,3) for proximity sensor 
	// adc(1,2) for trimpot - digital value changes as the pot rotation
	unsigned int val;
	PINSEL0 |=  0x0F300000;   
	/* Select the P0.13 AD1.4 for ADC function */
  /* Select the P0.12 AD1.3 for ADC function */
	/* Select the P0.10 AD1.2 for ADC function */
  switch (no)        //select adc
    {
        case 0: AD0CR   = 0x00200600 | (1<<ch); //select channel
                AD0CR  |= (1<<24) ;            //start conversion
                while (( AD0GDR &  ( 1U << 31 )) == 0);
                val = AD0GDR;
                break;
 
        case 1: AD1CR = 0x00200600  | ( 1 << ch );       //select channel
                AD1CR |=  ( 1 << 24 ) ;                              //start conversion
                while (( AD1GDR & (1U << 31)) == 0);
                val = AD1GDR;
                break;
    }
    val = (val  >>  6) & 0x03FF;         // bit 6:15 is 10 bit AD value
    return  val;
}

void RTC_Init(void)
{
   //enable clock and select external 32.768KHz
	   CCR = ((1 << 0 ) | (1<<4));//D0 - 1 enable, 0 disable
} 														// D4 - 1 external clock,0 from PCLK

// SEC,MIN,HOUR,DOW,DOM,MONTH,YEAR are defined in LPC214x.h
void RTC_SetDateTime(rtc_t *rtc)//to set date & time
{
     SEC   =  rtc->sec;       // Update sec value
     MIN   =  rtc->min;       // Update min value
     HOUR  =  rtc->hour;      // Update hour value 
     DOW   =  rtc->weekDay;   // Update day value 
     DOM   =  rtc->date;      // Update date value 
     MONTH =  rtc->month;     // Update month value
     YEAR  =  rtc->year;      // Update year value
}

void RTC_GetDateTime(rtc_t *rtc)
{
     rtc->sec     = SEC ;       // Read sec value
     rtc->min     = MIN ;       // Read min value
     rtc->hour    = HOUR;       // Read hour value 
     rtc->weekDay = DOW;      // Read day value 
     rtc->date    = DOM;       // Read date value 
     rtc->month   = MONTH;       // Read month value
     rtc->year    = YEAR;       // Read year value

}
void SystemInit(void)
{
   PLL0CON = 0x01; 
   PLL0CFG = 0x24; 
   PLL0FEED = 0xAA; 
   PLL0FEED = 0x55; 
   while( !( PLL0STAT & 0x00000400 ))
   { ; }
   PLL0CON = 0x03;
   PLL0FEED = 0xAA;  // lock the PLL registers after setting the required PLL
   PLL0FEED = 0x55;
   VPBDIV = 0x00;      // PCLK is 1/4 as CCLK i.e 15Mhz  
}


void timer1_Init()
{
	T1TCR = 0X00;
	T1MCR = 0X03;  //011 
	T1MR0 = 150000;
	T1TC  = 0X00;
	VICIntEnable = 0x0000020;  //00100000 Interrupt Souce No:5, D5=1
	VICVectAddr5 = (unsigned long)Timer1_ISR;  // set the timer ISR vector address
	VICVectCntl5 = 0x0000025;  // set the channel,D5=1,D4-D0->channelNo-5
	T1TCR = 0X01;
}


void delay(int cnt)
{
	T0MR0 = 1000;//14926; // some arbitrary count for delay
	T0MCR = 0x0004; // set Timer 0 Stop after Match
	while(cnt--)
	{
		T0TC = 0X00;
	  T0TCR = 1; // start the timer (enbale)
		while(!(T0TC == T0MR0)){};// wait for the match
	  T0TCR = 2;// stop the timer		
	}
}

void beep(unsigned int val)
{
	DACR = ((1 << 16) | (val << 6));
	delay_ms(500);
	DACR = ((1 << 16) | (1023 << 6));
	delay_ms(1);
}

void GPS(void)
{
	    		UART0_SendString(GGA_String);
			UART0_SendString("\r\n");
			UART0_SendString("UTC Time : ");
			get_Time();
			UART0_SendString(Time_Buffer);
			UART1_SendString(Time_Buffer);
			UART0_SendString("\r\n");
			UART0_SendString("Latitude : ");
			get_Latitude(GGA_Comma_Pointers[0]);
			UART0_SendString(Latitude_Buffer);
			UART1_SendString(Latitude_Buffer);
			UART0_SendString("\r\n");
			UART0_SendString("Longitude : ");
			get_Longitude(GGA_Comma_Pointers[2]);
			UART0_SendString(Longitude_Buffer);
	    		UART1_SendString(Longitude_Buffer);
			UART0_SendString("\r\n");
			UART0_SendString("Altitude : ");
			get_Altitude(GGA_Comma_Pointers[7]);
			UART0_SendString(Altitude_Buffer);
	    		UART1_SendString(Altitude_Buffer);
			UART0_SendString("\r\n");		
			UART0_SendString("\r\n");
			memset(GGA_String, 0 , 150);
			memset(Latitude_Buffer, 0 , 15);
			memset(Longitude_Buffer, 0 , 15);
			memset(Time_Buffer, 0 , 15);
			memset(Altitude_Buffer, 0 , 8);
}

void GPS_Init(void)
{
	  	IsItGGAString = false;
	  	GGA_Index = 0;
		memset(GGA_String, 0 , 150);
	  	memset(Latitude_Buffer, 0 , 15);
	  	memset(Longitude_Buffer, 0 , 15);
		memset(Time_Buffer, 0 , 15);
		memset(Altitude_Buffer, 0 , 8);
		VICVectAddr0 = (unsigned) UART0_Interrupt;	// UART0 ISR Address 
		VICVectCntl0 = 0x00000026;	// Enable UART0 IRQ slot 
		VICIntEnable = 0x00000040;	// Enable UART0 interrupt 
		VICIntSelect = 0x00000000;	// UART0 configured as IRQ
}

void get_Time(void)
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	
	uint8_t time_index=0;
	uint8_t index;
	uint16_t hour, min, sec;
	uint32_t Time_value;

	/* parse Time in GGA string stored in buffer */
	for(index = 0; GGA_String[index]!=','; index++)
	{		
		Time_Buffer[time_index] = GGA_String[index];
		time_index++;
	}	
	Time_value = atol(Time_Buffer);               /* convert string to integer */
	hour = (Time_value / 10000);                  /* extract hour from integer */
	min = (Time_value % 10000) / 100;             /* extract minute from integer */
	sec = (Time_value % 10000) % 100;             /* extract second from integer*/

	sprintf(Time_Buffer, "%d:%d:%d", hour,min,sec);
	
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void get_Latitude(uint16_t Latitude_Pointer)
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	
	uint8_t lat_index = 0;
	uint8_t index = (Latitude_Pointer+1);
	
	/* parse Latitude in GGA string stored in buffer */
	for(;GGA_String[index]!=',';index++)
	{
		Latitude_Buffer[lat_index]= GGA_String[index];
		lat_index++;
	}
	
	float lat_decimal_value, lat_degrees_value;
	int32_t lat_degrees;
	lat_decimal_value = atof(Latitude_Buffer);	/* Latitude in ddmm.mmmm */       
	
	/* convert raw latitude into degree format */
	lat_decimal_value = (lat_decimal_value/100);	/* Latitude in dd.mmmmmm */
	lat_degrees = (int)(lat_decimal_value);	/* dd of latitude */
	lat_decimal_value = (lat_decimal_value - lat_degrees)/0.6;	/* .mmmm/0.6 (Converting minutes to eequivalent degrees) */ 
	lat_degrees_value = (float)(lat_degrees + lat_decimal_value);	/* Latitude in dd.dddd format */
	
	sprintf(Latitude_Buffer, "%f", lat_degrees_value);
	
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void get_Longitude(uint16_t Longitude_Pointer)
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	
	uint8_t long_index = 0;
	uint8_t index = (Longitude_Pointer+1);
	
	/* parse Longitude in GGA string stored in buffer */
	for(;GGA_String[index]!=',';index++)
	{
		Longitude_Buffer[long_index]= GGA_String[index];
		long_index++;
	}
	
	float long_decimal_value, long_degrees_value;
	int32_t long_degrees;
	long_decimal_value = atof(Longitude_Buffer);	/* Longitude in dddmm.mmmm */
	
	/* convert raw longitude into degree format */
	long_decimal_value = (long_decimal_value/100);	/* Longitude in ddd.mmmmmm */
	long_degrees = (int)(long_decimal_value);	/* ddd of Longitude */
	long_decimal_value = (long_decimal_value - long_degrees)/0.6;	/* .mmmmmm/0.6 (Converting minutes to eequivalent degrees) */
	long_degrees_value = (float)(long_degrees + long_decimal_value);	/* Longitude in dd.dddd format */
	
	sprintf(Longitude_Buffer, "%f", long_degrees_value);
	
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void get_Altitude(uint16_t Altitude_Pointer)
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	
	uint8_t alt_index = 0;
	uint8_t index = (Altitude_Pointer+1);
	
	/* parse Altitude in GGA string stored in buffer */
	for(;GGA_String[index]!=',';index++)
	{
		Altitude_Buffer[alt_index]= GGA_String[index];
		alt_index++;
	}
	
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void hrsense(void)
{
	while(HR_SENSOR==0)	// wait for high pulse from sensor
			{;}
			delay_ms(10); // 10ms delay so that it does not listen to any noise
			beatms = 10; // start counting beatms from 10ms since we have delay after pulse
			while(HR_SENSOR==1)// wait until signal is high
			{
				delay_ms(1); //wait 1msec
				beatms++; //keep incrementing counter each 1ms
			}
			while(HR_SENSOR==0) //keep looping till signal goes back high, wait for next
			{	
				delay_ms(1); //wait 1msec
				beatms++; //keep incrementing counter each 1ms
			}
			// beatms variable will now have time in ms between two high edge pulses

			bpm = (float)60000/beatms; // see document of #1157 for this calculation
			if(bpm > 200)
			{
				sprintf(buf, "Irregular Heartbeat: %0.0f BPM\n",bpm);
				UART0_SendString(buf);
				LCD_CmdWrite(0x80);		LCD_DisplayString((char *)buf);
				count++;		//If more than 5 invalid signals, beep
				if(count > 5)
				{
					count = 0;
					beep(1023);
					delay_ms(500);
					beep(0);
				}
			} 
			else 
			{
				sprintf (buf, "%0.0f BPM\n", bpm); // Display reading in BPM
				UART0_SendString(buf);
				LCD_CmdWrite(0x80);		LCD_DisplayString((char *)buf);
			}
}



