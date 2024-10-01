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
#ifndef __MAIN_H__
#define __MAIN_H__

#include "main.h"

void i2c_init();

void i2c_start();

void i2c_addr(uint8_t addr, uint8_t is_read);

void i2c_send_byte(uint8_t byte);

void i2c_stop();

void write_eeprom_data(uint8_t chip, uint8_t addr, uint8_t* data, uint16_t len);
void read_eeprom_data(uint8_t chip, uint8_t addr, uint8_t* data, uint16_t len);

#endif
