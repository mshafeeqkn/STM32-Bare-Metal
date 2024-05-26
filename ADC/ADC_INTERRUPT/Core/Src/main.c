
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

static volatile uint8_t loopCount = 0;

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

void ADC1_2_IRQHandler() {
    uint16_t adcDelay = ADC1->DR;
    loopCount = adcDelay * 60 / 3500;
}

int main(void)
{
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

    // Select first channel
    ADC1->SQR3 = 1;

    // Enable end of conversion interrupt
    __disable_irq();
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_EnableIRQ(ADC1_2_IRQn);
    __enable_irq();

    // Turn LED off
    // TOGGLE_LED();

    while (1) {
        ADC1->SR &= ~(ADC_SR_EOC);
        for(int i = 0; i < loopCount; i++) {
            if(i < 2) {
                TOGGLE_LED();
            }

            delay(50);
            ADC1->CR2 |= ADC_CR2_ADON;
        }

        // if the above loop not executed (when loopCount is 0), the
        // conversion won't start. So start the conversion explicitly
        if(loopCount <= 1) {
            ADC1->CR2 |= ADC_CR2_ADON;
        }
    }
}

