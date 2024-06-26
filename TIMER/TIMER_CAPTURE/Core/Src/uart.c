/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : USART1 implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "uart.h"

/**
 * @brief Send a byte through the UART1
 *
 * @param byte to be sent
 */
static void uart1_send_byte(uint8_t ch) {
    while((USART1->SR & USART_SR_TXE) == 0) {}
    USART1->DR = ch;
}

/**
 * @brief Send formatted string through UART1
 *
 * @param format - formatted string
 */
void uart1_send_string(const char *format, ...) {
    va_list args;
    char buffer[128];
    size_t i, len;

    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    len = strlen(buffer);
    for(i = 0; i < len; i++) {
        uart1_send_byte(buffer[i]);
    }

    // Wait until transmission complted
    while((USART1->SR & USART_SR_TC) == 0) {}
}

/**
 * @brief Setup the UART1 for transmit or recieve
 *
 * @param uart_mode - UART_TX_ENABLE, UART_RX_ENABLE
 *                    or both
 */
void uart1_setup(uint8_t uart_mode) {
    uint16_t uart1_cr1_flags = 0;

    // Enable clock to USART1, Alternate function IO and GPIOA
    RCC->APB2ENR |= (RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN);

    // Baud rate 2400 @ 8MHz clock frequency
    USART1->BRR = 0xD05;
    if(0 != uart_mode) {
        // Global UART enable if either Tx or Rx enabled.
        uart1_cr1_flags |= USART_CR1_UE;

        // Enable Transmit mode if required
        if(uart_mode & UART_TX_ENABLE) {
            uart1_cr1_flags |= USART_CR1_TE;

            // Configure PA9 as output; 10MHz max; push pull
            GPIOA->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
            GPIOA->CRH |= (GPIO_CRH_MODE9_0 | GPIO_CRH_CNF9_1);
        }

        // Enable Receive mode if required
        if(uart_mode & UART_RX_ENABLE) {
            uart1_cr1_flags |= USART_CR1_RE;

            // Configure PA10 input pull-up/pull-down
            GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
            GPIOA->CRH |= GPIO_CRH_CNF10_1;
        }

    }

    // Enable UART and required mode
    USART1->CR1 |= uart1_cr1_flags;
}

/**
 * @brief receive a byte from UART1
 */
uint8_t uart1_get_byte() {
    uint8_t data;

    // Wait until new data arrive
    while((USART1->SR & USART_SR_RXNE) == 0) {}

    // Take the data;
    data = USART1->DR;

    // CLear the received data flag
    USART1->SR &= ~USART_SR_RXNE;

    return data;
}

/**
 * @brief The function to read the line of data from UART1
 * The function will return if any \r char detected or
 * len number of charector received.
 *
 * @param buff - The pointer to the buffer
 * @param len - Maximum buffer length
 */
void uart1_get_string(char *buff, size_t len) {
    int i = 0;
    do {
        buff[i] = uart1_get_byte();
        // Receive till the enter key is pressed
        if('\r' == buff[i]) {
            break;
        }
        i++;
    // Loop until the buffer is full
    } while(i < len);
}
