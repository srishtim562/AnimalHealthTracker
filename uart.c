#include "uart.H"

void UART1_init(void)		//For RPi
{
	PINSEL0 |= 1 << 18 | 1 << 16;					//Option 01 to configure P0.9 as RXD1 and P0.8 as TXD1 for raspberry pi
	U1LCR = 0x83;   /* 8 bits, no Parity, 1 Stop bit    */
  U1DLM = 0x01; U1DLL = 0x6e; // 9600 baud rate,PCLK = 15MHz
	U1FDR = 0xf1;
  U1LCR = 0x03;  /* DLAB = 0                         */
  U1FCR = 0x07;  /* Enable and reset TX and RX FIFO. */
}

void UART0_init(void)
{
	PINSEL0 = PINSEL0 | 0x00000005;	/* Enable UART0 Rx0 and Tx0 pins of UART0 */
	U0LCR = 0x83;	/* DLAB = 1, 1 stop bit, 8-bit character length */
	U0DLM = 0x00;	/* For baud rate of 9600 with Pclk = 15MHz */
	U0DLL = 0x61;	/* We get these values of U0DLL and U0DLM from formula */
	U0LCR = 0x03; /* DLAB = 0 */
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void UART0_TxChar(char ch) /* A function to send a byte on UART0 */
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	U0THR = ch;
	while( (U0LSR & 0x40) == 0 );	/* Wait till THRE bit becomes 1 which tells that transmission is completed */
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void UART0_SendString(char* str) /* A function to send string on UART0 */
{
	U0IER = 0x00000000; /* Disable RDA interrupts */
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		UART0_TxChar(str[i]);
		i++;
	}
	U0IER = 0x00000001; /* Enable RDA interrupts */
}

void UART1_TxChar(char ch) /* A function to send a byte on UART0 */
{
	U1IER = 0x00000000; /* Disable RDA interrupts */
	U1THR = ch;
	while( (U1LSR & 0x40) == 0 );	/* Wait till THRE bit becomes 1 which tells that transmission is completed */
	U1IER = 0x00000001; /* Enable RDA interrupts */
}

void UART1_SendString(char* str) /* A function to send string on UART1 */
{
	U1IER = 0x00000000; /* Disable RDA interrupts */
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		UART1_TxChar(str[i]);
		i++;
	}
	U1IER = 0x00000001; /* Enable RDA interrupts */
}
