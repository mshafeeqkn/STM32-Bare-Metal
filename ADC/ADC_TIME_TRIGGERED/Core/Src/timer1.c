/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : timer1.c
  * @brief          : Timer CC1
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
#include "timer1.h"

void TIM1_CC_IRQHandler() {
    // ADC1->CR2 |= ADC_CR2_ADON;
    TIM1->SR &= ~TIM_SR_CC1IF;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

// This function create 1 sec delay using Timer1 CC1, am I missing anything to trigger the ADC using timer1 cc1?
void configure_timer1_cc1() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    TIM1->ARR = 7199;
    TIM1->PSC = 9999;
    TIM1->CNT = 0;

    TIM1->DIER |= TIM_DIER_CC1IE;

    uint32_t prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(TIM1_CC_IRQn, NVIC_EncodePriority(prioritygroup, 10, 0));
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CR1 |= TIM_CR1_CEN;
}



