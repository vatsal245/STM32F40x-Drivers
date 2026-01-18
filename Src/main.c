#include <stdint.h>

// Base Addresses for Peripherals
#define RCC_BASE      0x40023800
#define GPIOA_BASE    0x40020000

// Register Offsets
#define RCC_AHB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_ODR     (*(volatile uint32_t *)(GPIOA_BASE + 0x14))

// Bit Definitions
#define GPIOAEN       (1 << 0)      // GPIOA clock enable is bit 0
#define PIN8          (1 << 8)
#define MODER8_OUT    (1 << 16)     // Setting bit 16 to 1 (01 = Output mode)

void delay(void) {
    for (int i = 0; i < 500000; i++); // Simple software delay
}

int main(void) {
    // 1. Enable Clock for GPIOA
    RCC_AHB1ENR |= GPIOAEN;

    // 2. Set GPIOA Pin 8 to Output Mode
    // First, clear the bits for pin 8 (bits 17:16)
    GPIOA_MODER &= ~(3 << 16);
    // Set bit 16 to 1 for General Purpose Output
    GPIOA_MODER |= MODER8_OUT;

    while (1) {
        // 3. Toggle Pin 8 using XOR
        GPIOA_ODR ^= PIN8;

        delay();
    }
}
