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
#include "usb.h"
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
#if 0
    // Enable HSE (High-Speed External) oscillator
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);  // Wait for HSE to be ready

    // Select HSE as the system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;  // Clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSE;  // Set SW bits to select HSE as system clock

    // Wait until HSE is used as the system clock source
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);

#else
    // Enable the HSE
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    // Set flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    // Disable the PLL
    RCC->CR &= ~RCC_CR_PLLON;
    while(RCC->CR & RCC_CR_PLLRDY);

    // Configure the PLL
    RCC->CFGR &= ~RCC_CFGR_PLLMULL;
    RCC->CFGR |= RCC_CFGR_PLLMULL9;
    RCC->CFGR |= RCC_CFGR_PLLSRC;

    // Re-enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Set the PLL as system clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while(0 == (RCC->CFGR & RCC_CFGR_SWS_PLL));

    // Configure AHB and APB prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;            // AHB prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;           // APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;           // APB2 prescaler

    // Update the global variables with
    // new clock source
    SystemCoreClockUpdate();
#endif
}

void config_1sec_timer1() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->PSC = 1199;
    TIM1->ARR = 59999;
    TIM1->CNT = 0;
    TIM1->SR &= ~(TIM_SR_UIF);
    TIM1->CR1 |= TIM_CR1_CEN;
}

void config_debug_led() {
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= (GPIO_CRH_MODE13_0 | GPIO_CRH_MODE13_1);

    turn_led_on(0);
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 7200; i++)
        __asm__("nop");  // No operation, just delay
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
    config_debug_led();
    uart1_setup(UART_TX_ENABLE);
    init_usb();
    uart1_send_string("Setup done\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");

    while(1) {
        delay_ms(100);
        usb_send_data();
    }
}
