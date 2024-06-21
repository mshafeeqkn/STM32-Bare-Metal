
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "uart.h"

#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

static uint16_t adc_data_buff[3];

typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

void delay(uint32_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

void turn_led_on(LedState_t state) {
    if(state == TURN_TOGGLE){
        GPIOC->ODR ^= GPIO_ODR_ODR13;
    } else if(state == TURN_ON) {
        GPIOC->ODR &= ~(GPIO_ODR_ODR13);
    } else {
        GPIOC->ODR |= GPIO_ODR_ODR13;
    }
}

static void adc1_setup() {
    // Enable clock for GPIOA & ADC1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                     // Analog input
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                     // ADC peripheral

    // GPIOA1, GPIOA2 and GPIOA3 as analog input mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
    GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
    GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);

    ADC1->CR1 |= ADC_CR1_SCAN;                              // Enable scan mode

    ADC1->CR2 |= ( ADC_CR2_CONT   |                         // Continuous mode
                   ADC_CR2_DMA    |                         // Enable DMA
                   // Right align by default
                   ADC_CR2_EXTSEL |                         // Start conversion by setting SWSTART bit
                   ADC_CR2_EXTTRIG );                       // Start conversion on external trigger

    ADC1->SQR1 |= (2 << ADC_SQR1_L_Pos);                    // 2 for 3 channels

    ADC1->SQR3 = ( (ADC_SQR3_SQ1_0) |                       // 1st conversion - channel 1
                   (ADC_SQR3_SQ2_1) |                       // 2nd conversion - channel 2
                   (ADC_SQR3_SQ3_0 | ADC_SQR3_SQ3_1) );     // 3rd Conversion - channel 3

    // Document RM0008 Section 11.3.1
    // The ADC can be powered-on by setting the ADON bit in the
    // ADC_CR2 register. When the ADON bit is set for the first
    // time, it wakes up the ADC from Power Down mode. Conversion
    // starts when ADON bit is set for a second time by software
    // after ADC power-up time
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(100);

    // Caliberate ADC after each power up
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);
}

static void dma_setup() {
    // Enable clock for DMA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;;                     // Analog input

    // Here the data is read from the ADC (Peripheral) to a (variable) memory
    DMA1_Channel1->CCR &= ~(DMA_CCR_DIR);

    DMA1_Channel1->CCR |= (DMA_CCR_CIRC    |               // Enable ciruclar buffer
                           // DMA_CCR_PINC |               // Peripheral address is always constant
                           DMA_CCR_MINC    |               // Enable peripheral address increment
                           DMA_CCR_PSIZE_0 |               // Peripheral data size 16 bit
                           DMA_CCR_MSIZE_0 );              // Memory data size 16 bit

    DMA1_Channel1->CNDTR = 3;                              // Data size 3 (3 ADC channels)
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_data_buff;
    DMA1_Channel1->CCR |= DMA_CCR_EN;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    uart1_setup(UART_TX_ENABLE);

    uart1_send_string("ADC1 DMA\r\n");

    // Enable clock for GPIOC
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;     // Built-in LED

    // Configure built in LED as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    adc1_setup();
    dma_setup();

    // Start ADC conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Turn LED off
    TOGGLE_LED();

    while (1) {
        // uart1_send_string("ADC1 value = %d;%d;%d\r\n", adc_data_buff[0], adc_data_buff[1], adc_data_buff[2]);
        for(uint8_t i = 0; i < 3; i++) {
            TOGGLE_LED();
            delay(adc_data_buff[i]);
            TOGGLE_LED();
            delay(adc_data_buff[i]);
        }
    }
}

