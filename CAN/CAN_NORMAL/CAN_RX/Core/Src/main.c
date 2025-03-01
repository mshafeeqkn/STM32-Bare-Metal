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
 * @brief Configure the system clock as 72MHz using
 * external crystal oscillator.
 */
void config_sys_clock() {
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
    while(RCC_CFGR_SWS_PLL != (RCC->CFGR & RCC_CFGR_SWS_PLL));

    // Configure AHB and APB prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;            // AHB prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;           // APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;           // APB2 prescaler

    // Update the global variables with
    // new clock source
    SystemCoreClockUpdate();
}

void config_debug_led() {
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)

    // TURN_OFF_LED();
}

void delay_ms(uint16_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

static void can_init() {
    // Enable clock for CAN and port B
    // We are using alternate pins for CAN since
    // the default pin may interfer with the USB
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // PB8 - CAN RX and PB9 - CAN TX
    // Remap CAN pins
    // CAN RX should be configured as floating
    // input pin. All pins are floating input mode
    // by default.
    // CAN TX pin should be configured as alternate
    // function output push-pull. High speed
    GPIOB->CRH &= ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9);
    GPIOB->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9);

    // Remap the CAN pins
    AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

    __disable_irq();
    // Enable FIFO full interrupt enable bit
    CAN1->IER |= CAN_IER_FMPIE1;

    // Enable interrupt for CAN receive
    // priority_group = NVIC_GetPriorityGrouping();
    // priority_encoded = NVIC_EncodePriority(priority_group, 0, 0);
    // NVIC_SetPriority(CAN1_RX1_IRQn, priority_encoded);
    NVIC_EnableIRQ(CAN1_RX1_IRQn);

    __enable_irq();

    // Request CAN initialization
    CAN1->MCR |= CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) == 0);

    // Exit from sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while((CAN1->MSR & CAN_MSR_SLAK) != 0);

    // Disable automatic re-transmission
    CAN1->MCR |= CAN_MCR_NART;

    // Configure Bit timing register
    // disable silent and loopback mode (ie normal mode)
    CAN1->BTR &= ~(CAN_BTR_SILM | CAN_BTR_LBKM);

    // Sync jump width as 1 time quanta, SJW = 0
    CAN1->BTR &= ~(CAN_BTR_SJW);

    // Time segment 2 - 1tq
    CAN1->BTR &= ~(CAN_BTR_TS2);

    // Time segment 1 - 2tq
    CAN1->BTR &= ~(CAN_BTR_TS1);
    CAN1->BTR |= CAN_BTR_TS1_0;

    // Set prescalar 72
    CAN1->BTR |= (72 - 1);
}

void can_start() {
    // Request to leave init state
    CAN1->MCR &= ~(CAN_MCR_INRQ);
    while((CAN1->MSR & CAN_MSR_INAK) != 0);
}

void can_send_byte(uint8_t byte) {
    uint32_t tsr = CAN1->TSR;
    uint8_t tx_mailbox;

    if((0 != (tsr & CAN_TSR_TME0)) ||
       (0 != (tsr & CAN_TSR_TME1)) ||
       (0 != (tsr & CAN_TSR_TME2))) {
        tx_mailbox = ((tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos);
        CAN1->sTxMailBox[tx_mailbox].TIR = (0x6A5 << CAN_TI0R_STID_Pos);
        CAN1->sTxMailBox[tx_mailbox].TDTR = 8;
        CAN1->sTxMailBox[tx_mailbox].TDHR = byte | byte << 8 | byte << 16 | byte << 24;
        CAN1->sTxMailBox[tx_mailbox].TDLR = byte | byte << 8 | byte << 16 | byte << 24;
        CAN1->sTxMailBox[tx_mailbox].TIR |= CAN_TI0R_TXRQ;
    }
}

void CAN1_RX1_IRQHandler(void) {
    if((CAN1->RF1R & CAN_RF1R_FMP1) == 0) {
        return;
    }

    if(((CAN_RDH1R_DATA7 & CAN1->sFIFOMailBox[1].RDHR) >> CAN_RDH0R_DATA7_Pos) == 'A') {
        TURN_ON_LED();
    } else {
        TURN_OFF_LED();
    }

    CAN1->RF1R |= CAN_RF1R_RFOM1;
}

void can_config_filter() {
    // Initialization mode for filter 1
    CAN1->FMR |= CAN_FMR_FINIT;

    // De-activate 0th filter bank
    CAN1->FA1R &= ~(CAN_FA1R_FACT0);

    // Configure 32-bit scale for filter 0
    CAN1->FS1R |= CAN_FS1R_FSC0;

    // Configure filters
    CAN1->sFilterRegister[0].FR1 = ((0x6A5 << 5) << 16);
    CAN1->sFilterRegister[0].FR2 = (0x7FF << 16);

    // Filter mode ID mask by default

    // Assign FIFO1 for filter-0
    CAN1->FFA1R |= CAN_FFA1R_FFA0;

    // Enable filter
    CAN1->FA1R |= CAN_FA1R_FACT0;

    // Leave init mode
    CAN1->FMR &= ~(CAN_FMR_FINIT);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
    config_debug_led();

    can_init();
    can_start();
    can_config_filter();

    while(1) {
    }
}
