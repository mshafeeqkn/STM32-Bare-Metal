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
static void delay(uint32_t tick) {
    for (volatile uint32_t i = 0; i < tick; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

void i2c_init() {
    // Enable clock for GPIOB, Alternate IO
    // and I2C peripherals
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB6 and PB7 as output mode with max speed 10MHz
    // with alternate function as open-drain
    GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0);
    GPIOB->CRL |= (GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0);

    // Reset I2C module and wait until BUSY
    // flag get cleared
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    while((I2C1->SR2 & I2C_SR2_BUSY));

    // APB clock frequency as 8MHz
    I2C1->CR2 |= 0x08;

    // 100KHz normal mode; clock time period = 10uS
    // Clock speed 8MHz; time period = 125nS
    // Assuem i2c dutycycle 0.5, I2C high time = 5uS = 5000ns
    // Number of system clock required = 5000/125 = 40d = 0x28
    I2C1->CCR |= 0x28;

    // Rising edge maximum time = 1000ns;
    // Number of system clock 1000/125 + 1 = 8 + 1 = 9
    I2C1->TRISE |= 0x09;

    // Enable I2C peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_start() {
    // Generate start condition, wait until the SB bit set
    // clear the SB flag by reading SR1 register
    I2C1->CR1 |= I2C_CR1_START;
    while((I2C1->SR1 & I2C_SR1_SB) == 0);
    (void)I2C1->SR1;
}

void i2c_addr(uint8_t addr, uint8_t is_read) {
    // Set ACK flag for receive data
    if(is_read) {
        I2C1->CR1 |= I2C_CR1_ACK;
    }

    // Write address into the data register with write/read flag
    // Wait until the ADDR flag set and clear the ADDR flag by
    // reading SR1 and SR2.
    I2C1->DR = (addr << 1) | is_read;
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

void i2c_send_byte(uint8_t byte) {
    // Wait until the transmit buffer empty
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = byte;
    // Wait until transmit finished
    while(!(I2C1->SR1 & I2C_SR1_BTF));
}

void i2c_stop() {
    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
}

uint8_t i2c_recv_byte() {
    while(!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

void read_eeprom_data(uint8_t chip, uint8_t addr, uint8_t* data, uint16_t len) {
    int i = 0;
    i2c_start();
    i2c_addr(chip, 0);
    i2c_send_byte(addr);
    i2c_start();
    i2c_addr(chip, 1);
    while(len--) {
        if(len == 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;
            I2C1->CR1 |= I2C_CR1_STOP;
        }
        data[i++] = i2c_recv_byte();
    }
    i2c_stop();
}

void write_eeprom_data(uint8_t chip, uint8_t addr, uint8_t* data, uint16_t len) {
    uint16_t i = 0;
    uint16_t end_addr = addr | 0x0F;
    uint16_t start = 0;
    uint16_t end = end_addr - addr + 1;

    while(len) {
        i2c_start();
        i2c_addr(chip, 0);
        i2c_send_byte(addr);

        for(i = start; i < end; i++) {
            i2c_send_byte(data[i]);
            len--;
        }

        i2c_stop();
        addr += (end - start);
        start += (end - start);
        if(len > 16) {
            end += 16;
        } else {
            end += len;
        }

        if(addr == 0 && chip == 0x50) {
            chip++;
        }

        delay(3700);
    }
}

