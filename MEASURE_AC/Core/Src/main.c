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
#include <math.h>

// Macro defines
#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

#define SENSITIVITY              880.0
#define VREF                     3.3
#define ADC_RES                  4095.0

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
    TIM3->CR1 &= ~(TIM_CR1_CEN);
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

/**
 * @breif Configure ADC1 channel A0 for voltage
 * reading
 */
void configure_adc() {
    // Enable clock for GPIOA and ADC1, configure
    // the A0 pin for analog input
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Enable the ADC and calibrate it.
    ADC1->CR2 |= ADC_CR2_ADON;
    delay(100);
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    // Number of channel = 1 L-bits in ADC1->SQR3 is
    // 0 (default). The channel to be converted is A0,
    // so SQ1 ADC->SQR1 register is 0 (default)
}

/**
 * Get ADC value
 */
uint16_t get_adc_value() {
    uint16_t adc_val;

    // Enable the ADC and wait until the conversion
    // completed.
    ADC1->CR2 |= ADC_CR2_ADON;       
    while(!(ADC1->SR & ADC_SR_EOC));
    adc_val = ADC1->DR;            
    return adc_val;
}

/**
 * Get DC offset of the analog signal by taking
 * the avarage of each sample.
 */
uint16_t get_zero_point() {
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

    return avg_adc;
}


/**
 * Calculate the RMS value of the AC
 * RMS = avarage of square of each sample
 * Calculate the RMS multiple times and take
 * the avarage to get more accurate value.
 */
float get_rms_voltage(uint8_t loop) {
    uint16_t dc_volt = get_zero_point();
    uint32_t adc_sq_sum = 0;
    uint16_t rep_count = 0;
    int16_t v_now, adc_val;
    float rms_volt = 0.0f;

    // Calculate 'loop' times
    for(uint8_t j = 0; j < loop; j++) {
        expire_20ms = 0;
        TIM3->CR1 |= TIM_CR1_CEN;
        while(expire_20ms == 0) {
            adc_val = get_adc_value();
            v_now = adc_val  - dc_volt;
            adc_sq_sum += (v_now * v_now);
            rep_count++;
        }
        rms_volt += sqrt(adc_sq_sum/rep_count) / ADC_RES * VREF * SENSITIVITY;
    }

    return rms_volt/loop;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    float rms;
    config_sys_clock();
    uart1_setup(UART_TX_ENABLE);
    setup_debug_led();
    setup_timer_1();
    configure_adc();

    while(1) {
        rms = get_rms_voltage(5);
        uart1_send_string(">> %d\r\n", (int)rms);
        delay(1000);
    }
}
