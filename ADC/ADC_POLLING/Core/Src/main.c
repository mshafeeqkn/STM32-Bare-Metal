
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


#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)


#define UART_TX_ENABLE           0x01
#define UART_RX_ENABLE           0x02

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

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
    uint16_t adcDelay1 = 0xFFF;
    uint16_t adcDelay2 = 0xFFF;

    uint16_t onTime = 0;
    uint16_t offTime = 0;
    uint8_t dutyCycle = 0;

    // Enable clock for GPIOA, GPIOC & ADC1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure built in LED as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    // GPIOA1 analog input mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
    GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);

    // Document RM0008 Section 11.3.1
    // The ADC can be powered-on by setting the ADON bit in the
    // ADC_CR2 register. When the ADON bit is set for the first
    // time, it wakes up the ADC from Power Down mode. Conversion
    // starts when ADON bit is set for a second time by software
    // after ADC power-up time (giving ~100ms)
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(100);
    ADC1->CR2 |= ADC_CR2_ADON;

    // Caliberate ADC after each power up
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    // Turn LED off
    TOGGLE_LED();

    while (1) {
        // Get ADC value from channel 1
        ADC1->SQR3 = 1;                     // Select channel
        ADC1->CR2 |= ADC_CR2_ADON;          // Enable ADC
        while(!(ADC1->SR & ADC_SR_EOC));    // Wait until the End Of Conversion flag is set
        adcDelay1 = ADC1->DR;               // Get the data from ADC Data Register

        // Get ADC value from channel 2
        ADC1->SQR3 = 2;
        ADC1->CR2 |= ADC_CR2_ADON;
        while(!(ADC1->SR & ADC_SR_EOC));
        adcDelay2 = ADC1->DR;

        dutyCycle = adcDelay1 * 100 / 3600;

        // Work-around to keep dutycycle not more than 100%
        // Because in some case, the adcDelay1 is going more
        // than 3600.
        if(dutyCycle > 100) {
            dutyCycle = 100;
        }

        // Calculate the LED on and off time
        onTime = (adcDelay2 * dutyCycle) / 100;
        offTime = adcDelay2 - onTime;

        // Toggle
        TOGGLE_LED();
        delay(onTime);
        TOGGLE_LED();
        delay(offTime);
    }
}

