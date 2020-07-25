#ifndef _uart_H_
#define _uart_H_

#include <lpc214x.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

void UART1_init(void);
void UART0_init(void);
void UART0_TxChar(char ch);
void UART0_SendString(char* str);
void UART1_TxChar(char ch);
void UART1_SendString(char* str);

#endif
