
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc.c
  * @brief          : ADC related functions
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
#include "adc.h"

uint16_t i = 0;

void ADC1_2_IRQHandler() {
    ADC1->SR &= ~(ADC_SR_EOC);
    // uint16_t adcDelay = ADC1->DR;
    turn_led_on(2);
}

static void configure_CC1_event() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1->ARR = 8000 - 1;
    TIM1->PSC = 1000 - 1;
    TIM1->CCR1 = 7999;
    // TIM1->CCMR1 |= ()
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; // Set OC1M to 110: PWM Mode 1
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for CC1
    TIM1->EGR |= TIM_EGR_UG; // Generate an update event to apply changes

    TIM1->CR1 |= TIM_CR1_CEN; // enable timer
}

void adc_init(void) {
    // Enable clock for GPIOA  & ADC1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // GPIOA1 analog input mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);

    // Document RM0008 Section 11.3.1
    // The ADC can be powered-on by setting the ADON bit in the
    // ADC_CR2 register. When the ADON bit is set for the first
    // time, it wakes up the ADC from Power Down mode. Conversion
    // starts when ADON bit is set for a second time by software
    // after ADC power-up time (giving ~100ms)
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 800000; ++i) { __NOP(); }
    ADC1->CR2 |= ADC_CR2_ADON;

    // Caliberate ADC after each power up
    ADC1->CR2 |= ADC_CR2_CAL;
    while(ADC1->CR2 & ADC_CR2_CAL);

    // Start conversion on external event.
    ADC1->CR2 |= ADC_CR2_EXTTRIG;
    // We are taking CC1 event of timer 1 as external event (default)
    // set EXTSEL of ADC_CR2 as 0
    configure_CC1_event();

    // Select first channel
    ADC1->SQR3 = 1;

    // Enable end of conversion interrupt
    __disable_irq();
    ADC1->CR1 |= ADC_CR1_EOCIE;
    NVIC_SetPriority(ADC1_2_IRQn, 1); // Set priority
    NVIC_EnableIRQ(ADC1_2_IRQn);
    __enable_irq();
}

void adc_start() {
    ADC1->CR2 |= ADC_CR2_ADON;
}
