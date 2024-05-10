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

#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)


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
    uint8_t i;
    const uint8_t repeat = 3;

    // Enable clock for GPIOC and GPIOB peripherals
    RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN);

    // Configure PC13 pin as output push-pull maximum speed 10MHz
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    // Configure PB0 pin as input pull up or pull down
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOB->CRL |= GPIO_CRL_CNF0_1;

    // PBO as pull up
    GPIOB->ODR |= GPIO_ODR_ODR0;

    // By default the ouptu will be high, turn it off
    TURN_OFF_LED();

    while (1) {
        // Switch is released by default. On pressing switch
        // the input will become 0 and exit the loop.
        while(GPIOB->IDR & GPIO_IDR_IDR0) {}

        // Wait here as long as switch kept as pressed
        while(!(GPIOB->IDR & GPIO_IDR_IDR0)) {}

        // Blink LED `repeat` times
        for(i = 0; i < repeat; i++) {
            TURN_ON_LED();
            delay(500);
            TURN_OFF_LED();
            delay(500);
        }
    }
}

