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

void delay(uint32_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

volatile uint8_t expire_20ms = 0;

void TIM3_IRQHandler() {
    TIM3->SR &= ~(TIM_SR_UIF);
    expire_20ms = 1;
    // TOGGLE_LED();
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

static void setup_debug_led() {
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)
    // TOGGLE_LED();
}

/**
 * @brief Setup the TIM3 for 20ms delay
 */
void setup_timer_1() {
    // Enable system clock to TIM3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Set pre-scalar 8000; so each count will take
    // 1ms delay since we are using 8MHz clock
    TIM3->PSC = 7999;

    // Count 0 - 999 (both inclusive - so total 1000)
    TIM3->ARR = 19;

    // Clear the counter register as 0
    TIM3->CNT = 0;

    // Clear the overflow flag
    TIM3->SR &= ~(TIM_SR_UIF);
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
}

void configure_adc() {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR2 |= ADC_CR2_ADON;
    delay(100);
    ADC1->CR2 |= ADC_CR2_ADON;

    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    ADC1->SQR3 = 1;                  
}

uint16_t get_adc_value() {
    uint16_t adcDelay;

    ADC1->CR2 |= ADC_CR2_ADON;       
    while(!(ADC1->SR & ADC_SR_EOC)); 
    adcDelay = ADC1->DR;            
    return adcDelay;
}

void get_dc_value() {
    uint32_t adc_sum = 0;
    uint16_t rep_count = 0;
    uint16_t avg_adc;

    expire_20ms = 0;

    // Enable the timer.
    TIM3->CR1 |= TIM_CR1_CEN;
    while(expire_20ms == 0) {
        adc_sum += get_adc_value();
        rep_count++;
    }

    avg_adc = adc_sum / rep_count;
    uart1_send_string("avarage DC = %u", avg_adc);

    TOGGLE_LED();
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
    uart1_setup(UART_TX_ENABLE);
    setup_debug_led();
    setup_timer_1();
    configure_adc();

    get_dc_value();

    while(1) {
    }
}
