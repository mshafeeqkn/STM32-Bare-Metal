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
#include "uart.h"

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
    RCC->CFGR &= ~RCC_CFGR_SW;              // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSE;           // Set SW bits to select HSE as system clock

    // Wait until HSE is used as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
}

void setup_timer_1_capture() {
    // Configure GPIOA8 pin as input
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;               // Enable clock for Port A
    GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8);  // Set PA8 as input
    GPIOA->CRH |= GPIO_CRH_CNF8_1;                    // Set PA8 as pull-up/pull-down
    GPIOA->ODR &= ~(GPIO_ODR_ODR8);                   // Set PA8 as pull down;

    // Enable TIMER1
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;               // Enable clock for TIM1
    TIM1->PSC = 0x3FFF;                               // set prescalar
    TIM1->ARR = 0xFFFF;                               // set maximum count
    TIM1->CR1 |=  TIM_CR1_CEN;                        // Start the timer.

    // Select filter trigger
    TIM1->CCMR1 &= ~(TIM_CCMR1_IC1F);                 // Sampling at internal clock and wait unitl 8 cycles
    TIM1->CCMR1 |= (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);   // to ensure the low/high state
 
    // Select edge and channel
    TIM1->CCMR1 |= TIM_CCMR1_CC1S_0;                  // CC1 configured as input, IC1 mapped with TI1 (TIM1_CH1)
    TIM1->CCMR1 &= ~(TIM_CCMR1_IC1PSC);               // No prescalar. capture on every event on the TI
    TIM1->CCER &= ~(TIM_CCER_CC1P);                   // Raising edge of IC1.
    TIM1->CCER |= TIM_CCER_CC1E;                      // Capture enabled for channel 1
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    uint16_t data;

    uart1_setup(UART_TX_ENABLE);
    config_sys_clock();

    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    setup_timer_1_capture();
    uart1_send_string("Setup timer 1 capture ... Done\r\n");

    while(1) {
        // Wait until the overflow flag is set.
        while( (TIM1->SR & TIM_SR_CC1IF) == 0) {}
        TIM1->SR &= ~(TIM_SR_CC1IF);
        data = TIM1->CCR1;
        uart1_send_string("Captured data ... 0x%x\r\n", data);
        TOGGLE_LED();
    }
}
