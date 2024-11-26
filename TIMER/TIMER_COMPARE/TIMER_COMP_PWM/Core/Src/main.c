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
 * @brief Setup Timer1 in compare mode.
 */
void setup_timer_1_compare() {
    // Enable clock for Timer1 and GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    // Set PA8 as alternate function output push-pull
    GPIOA->CRH &= ~(GPIO_CRH_MODE8|GPIO_CRH_CNF8);
    GPIOA->CRH |= GPIO_CRH_MODE8_0;
    GPIOA->CRH |= GPIO_CRH_CNF8_1;

    // Total 4 sec delay for on and off LED
    TIM1->PSC = 499;
    TIM1->CCMR1 = 32767;    // no matter the value set here.
                            // because the match will happen
                            // once in every 0-65535 only.

    // OC1REF should not clear; preload disable; fast disable
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE);

    // Toggle on comapre match between TIM1_CNT and TIM1_CCMR1
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0);

    // Enable compare output
    TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);

    // Capture compare enable
    TIM1->CCER |= TIM_CCER_CC1E;

    // Enable PA8 (OC) if corresponding OCxE is set in the CCER register
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start the timer
    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
    setup_timer_1_compare();
    while(1) {}
}

