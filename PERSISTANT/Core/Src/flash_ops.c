/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : flash_ops.c
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
#include "flash_ops.h"


void flash_write_struct(uint32_t address, flash_data_t *data) {
    uint16_t *ptr = (uint16_t *)data;
    // Unlock Flash
    flash_unlock();

    // Erase the page
    flash_erase_page(address);

    // Write the structure data
    for(int i = 0; i < sizeof(flash_data_t); i++) {
        flash_program_half_word(address + (i * 2), *(ptr + i));
    }

    // Lock Flash
    flash_lock();
}

void flash_unlock(void) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
}

void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_page(uint32_t page_address) {
    // Ensure no operation is ongoing
    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = page_address;
    FLASH->CR |= FLASH_CR_STRT;

    while (FLASH->SR & FLASH_SR_BSY);

    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR |= FLASH_SR_EOP;
    }

    FLASH->CR &= ~FLASH_CR_PER;
}

void flash_program_half_word(uint32_t address, uint16_t data) {
    // Ensure no operation is ongoing
    while (FLASH->SR & FLASH_SR_BSY);

    FLASH->CR |= FLASH_CR_PG;
    *(__IO uint16_t*)address = data;

    while (FLASH->SR & FLASH_SR_BSY);

    if (FLASH->SR & FLASH_SR_EOP) {
        FLASH->SR |= FLASH_SR_EOP;
    }

    FLASH->CR &= ~FLASH_CR_PG;
}

uint32_t read_32bit_data(uint32_t address) {
    uint32_t ret = *(__IO uint16_t*)address;
    ret |= ((*(__IO uint16_t*)(address + 2)) << 16);
    return ret;
}

void flash_read_struct(uint32_t address, flash_data_t *data) {
    data->alarm_time = read_32bit_data(address);
    data->run_time = read_32bit_data(address+4);
    data->sample = read_32bit_data(address+8);
}
