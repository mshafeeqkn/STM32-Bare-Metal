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
#include "i2c_master.h"

// Macro defines
#define CHIP_ADDR                0x50

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


int main(void) {
    config_sys_clock();

    i2c_init();
    write_eeprom_data(CHIP_ADDR, 0x00, (uint8_t*)"This is a very long string used to test the full memory size of the EEPROM which has the memory size of 512 byte of memory but I am not sure how it works and how to fix this again some dummy data to fill the maximum length of the full memory address right? ok bye again a plenty of data to test the second part of the memory which is automatically updating or not we need to test. This is the aim of the whole program. Next part is we wil write a protram to read the data from the eeprom but why this is not andyyyyy", 512);

    while(1) {
    }
}

