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


void delay(uint32_t tick) {
    for (volatile uint32_t i = 0; i < tick; ++i) {
        __NOP();  // No operation (compiler barrier)
    }
}

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

void gpio_init() {
    // Enable clock for GPIOC peripheral
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // Configure GPIO pin as output
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // Clear configuration
    GPIOC->CRH |= GPIO_CRH_MODE13_0;  // Set pin mode to general purpose output (max speed 10 MHz)
    TURN_OFF_LED();

}

void i2c_slave_init(uint8_t addr) {
    uint32_t prioritygroup;

    // Enable clock for Port B, I2C, Alternate function IO
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure the I2C pins as open-drain output
    GPIOB->CRL |= (GPIO_CRL_CNF6 | GPIO_CRL_MODE6_0);
    GPIOB->CRL |= (GPIO_CRL_CNF7 | GPIO_CRL_MODE7_0);

    // Disable the peripheral enable bit
    I2C1->CR1 &= ~(I2C_CR1_PE);

    // Reset I2C
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // Configure internal clock, raise time
    I2C1->CR2 |= 0x08;
    I2C1->TRISE = 0x9;

    // Clock control register for 100KHz I2C frequency
    // at system clock frequency of 8MHz
    I2C1->CCR = 0x28;

    // Configure the slave address; as per the datasheet
    // bit 14 should be set.
    I2C1->OAR1 = 0x4000 | (addr << 1);

    // Enable the peripheral enable bit and ACK
    I2C1->CR1 |= I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR2 |= I2C_CR2_ITEVTEN;

    // Configure the NVIC
    prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(prioritygroup, 10, 0));
    NVIC_EnableIRQ(I2C1_EV_IRQn);
}
void i2c_slave_listen() {
    // Wait until any master sent our address
    // through the I2C interface. Clear the flag
    // if the address matched.
    while(!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

uint8_t i2c_slave_recv_byte() {
    // Receive 1 byte from the master.
    while(!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

void i2c_slave_send_byte(uint8_t byte) {
    while(!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = byte;
}

void i2c_slave_send_finish() {
    while(!(I2C1->SR1 & I2C_SR1_AF));
    I2C1->SR1 &= ~I2C_SR1_AF;
}

static volatile uint8_t off_time, on_time;
void I2C1_EV_IRQHandler() {
    if((I2C1->SR1 & I2C_SR1_ADDR) != 0) {
        (void)I2C1->SR1;
        (void)I2C1->SR2;
        if(I2C1->SR2 & I2C_SR2_TRA) {
            i2c_slave_send_byte('A');
            i2c_slave_send_byte('B');
            i2c_slave_send_finish();
        } else {
            on_time = i2c_slave_recv_byte();
            off_time = i2c_slave_recv_byte();
        }
    }

    if (I2C1->SR1 & I2C_SR1_STOPF) {
        (void)I2C1->SR1;
        I2C1->CR1 |= I2C_CR1_PE;
    }
}

int main(void) {
    uint8_t addr = 0x14;
    config_sys_clock();
    gpio_init();
    i2c_slave_init(addr);

    while(1) {
        TURN_ON_LED();
        delay(40000 * on_time);
        TURN_OFF_LED();
        delay(40000 * off_time);
    }
}

