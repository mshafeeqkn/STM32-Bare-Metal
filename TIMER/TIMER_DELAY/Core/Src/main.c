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

// Macro defines
#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

/**
 * Enumerations
 */
typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

/**
 * @brief Trun the built in LED on/off/toggle
 *
 * @param state - TURN_ON / TURN_OFF / TURN_TOGGLE
 */
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
 * @brief Configure the system clock as 8MHz using
 * external crystal oscillator.
 */
void config_sys_clock() {
    // Enable HSE (High-Speed External) oscillator
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);  // Wait for HSE to be ready

    // Select HSE as the system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;  // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSE;  // Set SW bits to select HSE as system clock

    // Wait until HSE is used as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
}

/**
 * @brief Setup the TIM1 for 1 min delay
 */
void setup_timer_1() {
    // Enable system clock to TIM1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set pre-scalar 8000; so each count will take
    // 1ms delay since we are using 8MHz clock
    TIM1->PSC = 7999;

    // Count 0 - 59999 (both inclusive - so total 60000)
    TIM1->ARR = 59999;

    // Clear the counter register as 0
    TIM1->CNT = 0;

    // Clear the overflow flag
    TIM1->SR &= ~(TIM_SR_UIF);

    // Enable the timer.
    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();

    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    setup_timer_1();

    while(1) {
        // Wait until the overflow flag is set.
        while( (TIM1->SR & TIM_SR_UIF) == 0) {}
        TIM1->SR &= ~(TIM_SR_UIF);
        TOGGLE_LED();
    }
}
