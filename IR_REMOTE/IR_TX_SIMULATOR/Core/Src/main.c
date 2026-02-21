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

#define TURN_ON_LED()            turn_led_on(TURN_ON)
#define TURN_OFF_LED()           turn_led_on(TURN_OFF)
#define TOGGLE_LED()             turn_led_on(TURN_TOGGLE)

typedef enum {
    TURN_OFF,
    TURN_ON,
    TURN_TOGGLE
} LedState_t;

typedef enum {
    PKT_WAITING,
    PKT_STARTED,
    PKT_RECEIVED,
    PKT_PROCESSING
} PktStatus_t;

static uint32_t delays[] = {
    65000, 18000, 9000,
    1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125,
    1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125,
    1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125,
    1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125, 1125,
    1125
};
const    uint32_t num_delays    = sizeof(delays) / sizeof(delays[0]);
volatile uint32_t delay_idx     = 1;
volatile uint16_t edge_index    = 0;
volatile uint8_t  discard_count = 5;
volatile PktStatus_t pkt_stat   = PKT_WAITING;

volatile uint16_t offsets[75];

void delay(uint32_t ms) {
    // Simple delay function (not accurate, just for demonstration)
    for (volatile uint32_t i = 0; i < ms * 1000; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

void turn_led_on(LedState_t state) {
    if(state == TURN_TOGGLE){
        GPIOC->ODR ^= GPIO_ODR_ODR13;
    } else if(state == TURN_ON) {
        GPIOC->ODR &= ~(GPIO_ODR_ODR13);
    } else {
        GPIOC->ODR |= GPIO_ODR_ODR13;
    }
}

inline void start_ir_simulator() {
    TIM2->CR1 |= TIM_CR1_CEN;    // Start timer
}

inline void stop_ir_simulator() {
    TIM2->CR1 &= ~(TIM_CR1_CEN);            // Stop timer
}

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
    while(0 == (RCC->CFGR & RCC_CFGR_SWS_PLL));

    // Configure AHB and APB prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;            // AHB prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;           // APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;           // APB2 prescaler

    // Update the global variables with
    // new clock source
    SystemCoreClockUpdate();
}


void encode_nec_data(uint8_t addr, uint8_t cmd) {
    uint32_t full_data = (addr << 0) | ((uint8_t)~addr << 8) | (cmd << 16) | ((uint8_t)~cmd << 24);
    uint8_t index = 4;
    while(full_data) {
        if(full_data & 0x01)
            delays[index] = 3375;
        full_data >>= 1;
        index += 2;
    }
}

void TIM2_IRQHandler(void) {
    uint32_t next_delay;

    if (TIM2->SR & TIM_SR_CC1IF) {
        TIM2->SR &= ~TIM_SR_CC1IF;  // Clear flag
        next_delay = delays[delay_idx];
        TIM2->CCR1 += next_delay;
        delay_idx = (delay_idx + 1) % num_delays;  // Repeat sequence
    }
}

void configure_timer2() {
    // 2. Enable clocks
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // GPIOA clock

    // 3. PA0 (TIM2_CH1): Alternate push-pull, max speed 50 MHz
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);  // Clear
    GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1; // 50 MHz AF PP
    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;              // No remap

    // 2. NVIC: Enable TIM2 IRQ (IRQn=28)
    NVIC->ISER[0] |= (1 << 28);

    // 4. TIM2: 36000 sys ticks = 0.1 ms @72MHz
    TIM2->PSC  = 35;                // 36 -> 0.1us tick
    TIM2->ARR  = 0xFFFF;
    TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) | (0x3 << 4);  // OC1M=011 toggle
    TIM2->CCER |= TIM_CCER_CC1E;    // CH1 enable
    TIM2->DIER |= TIM_DIER_CC1IE;   // CC1 interrupt enable

    TIM2->CNT  = 0;
    TIM2->CCR1 = delays[edge_index];  // 500ms
    delay_idx  = 1;

    start_ir_simulator();
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {                   // EXTI1 pending?
        EXTI->PR = EXTI_PR_PR1;                     // Clear flag (write 1)[web:152]
        offsets[edge_index] = TIM3->CNT;
        TIM3->CNT = 0;

        if(discard_count > 0) {
            discard_count--;
            return;
        }

        if(offsets[edge_index] > 8950 && offsets[edge_index] < 9100) {
            pkt_stat = PKT_STARTED;
        }

        if(pkt_stat != PKT_STARTED) {
            return;
        }

        if(edge_index > 65) {
            stop_ir_simulator();
            pkt_stat = PKT_RECEIVED;
        }
        edge_index++;
    }
}

void configure_exti() {
    // 1. Clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // 2. NVIC: EXTI1_IRQn=7
    NVIC->IP[7] = (0 << 4);         // Priority 0
    NVIC->ISER[0] |= (1 << 7);      // Enable IRQ 7

    // 3. PA1: Input floating (or pull-up/down as needed)
    GPIOA->CRL &= ~GPIO_CRL_MODE1;  // Input mode (00)
    GPIOA->CRL |= GPIO_CRL_CNF1_0;  // Floating input (01)

    // 4. EXTI1: Both edges
    AFIO->EXTICR[0] &= ~(0xF << 4); // EXTI1=PA1 (0000)[web:152]
    EXTI->IMR |= EXTI_IMR_MR1;      // Unmask EXTI1
    EXTI->RTSR |= EXTI_RTSR_TR1;    // Rising edge
    EXTI->FTSR |= EXTI_FTSR_TR1;    // Falling edge
}

void configure_timer3() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // Critical!

    TIM3->PSC = 71;             // 72MHz / 72 = 1MHz → 1µs/tick
    TIM3->ARR = 0xFFFF;
    TIM3->CR1 |= TIM_CR1_CEN;   // Free run
}

uint32_t get_nec_data() {
    uint32_t ret    = 0;
    uint8_t bit_pos = 0;
    uint32_t tmp    = 0;

    for(uint8_t i = 3; i <= 65; i += 2, bit_pos++) {
        if(offsets[i] > 1000) {
            tmp |= (1 << bit_pos);
        }

        ret = (tmp << 16) | (tmp >> 16);
    }
    return ret;    
}

uint8_t is_nec_data_valid(uint32_t data) {
    uint8_t xor = ((data >> 0) & 0xFF) ^ ((data >> 8) & 0xFF) ^
           ((data >> 0) & 0xFF) ^ ((data >> 8) & 0xFF);
    return xor == 0;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    config_sys_clock();
#ifdef DEBUG_UART
    uart1_setup(UART_TX_ENABLE);
#endif
    configure_exti();
    encode_nec_data(0x12, 0x44);

    // Enable clock for GPIOC and GPIOB peripherals
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure PC13 pin as output push-pull maximum speed 10MHz
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    // By default the output will be high, turn it off
    TURN_OFF_LED();

    configure_timer3();
    configure_timer2();

    while (1) {
        delay(1000);
        if(pkt_stat == PKT_RECEIVED) {
            pkt_stat = PKT_PROCESSING;
            uint32_t data = get_nec_data();
            if(is_nec_data_valid(data)) {
#ifdef DEBUG_UART
                uart1_send_string("Data: %08X", data);
#endif
                if(data == 0xED12BB44) {
                    TURN_ON_LED();
                }
            }
            pkt_stat = PKT_WAITING;
        }
        while(1);
    }
}
