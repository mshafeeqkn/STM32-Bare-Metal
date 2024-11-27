
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
#include "timer1.h"
#include "main.h"

uint16_t i = 0;

extern void turn_led_on(LedState_t state);

void ADC1_2_IRQHandler() {
    (void)ADC1->DR;
    turn_led_on(2);
}

void adc_init(void) {
    // Enable clock for GPIOA & ADC1 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1 clock

    // Set ADC clock prescaler to divide by 6 (72 MHz / 6 = 12 MHz)
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    // Configure PA0 as analog input
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);

    // Power on ADC
    (void)ADC1->DR;
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 800000; ++i) { __NOP(); } // Startup delay
    ADC1->CR2 |= ADC_CR2_ADON;

    // Perform ADC calibration
    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL); // Wait for calibration

    // Enable external trigger and select timer event (EXTSEL = 000 -> TIM1 CC1 event)
    ADC1->CR2 |= ADC_CR2_EXTTRIG;
    ADC1->CR2 |= ADC_CR2_EXTSEL;

    // Select channel 0 (PA0)
    ADC1->SQR3 = 0;

    // Enable End of Conversion (EOC) interrupt
    ADC1->CR1 |= ADC_CR1_EOCIE;

    // Configure NVIC for ADC1 interrupt
    uint32_t prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(prioritygroup, 10, 0));
    NVIC_EnableIRQ(ADC1_2_IRQn);
}

void adc_start() {
    ADC1->CR2 |= ADC_CR2_ADON;
}
