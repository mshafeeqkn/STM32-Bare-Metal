#ifndef __UART_H__
#define __UART_H__

#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#define UART_TX_ENABLE           0x01
#define UART_RX_ENABLE           0x02

extern void uart1_send_string(const char *format, ...);
extern void uart1_setup(uint8_t uart_mode);
extern void uart1_get_string(char *buff, size_t len);

#endif
