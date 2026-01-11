/*
 * stm32f407xx.h
 *
 *  Created on: 02-Jan-2026
 *      Author: vatsal
 */

/* [Note:] This header file will have addresses of all the base addresses we will need to create drivers.
 * these addresses will be stored as macros
 */
#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>
#define __vo volatile

/*FLASH, RAMs and ROM base addresses*/
#define FLASH_BASEADDR			0x80000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM_BASEADDR			0x1FFF0000U

/* Base addresses for Peripheral Bus Addresses*/

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERI_BASEADDR		PERIPH_BASEADDR
#define APB2PERI_BASEADDR		0x40010000U
#define AHB1PERI_BASEADDR		0x40020000U
#define AHB2PERI_BASEADDR		0x50000000U

/* Base Addresses of all GPIOs. Note that all these are hanging on the AHB1 bus */

#define GPIOA_BASEADDR			(AHB1PERI_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERI_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERI_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERI_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERI_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERI_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERI_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERI_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERI_BASEADDR + 0x2000)
#define RCC_BASEADDR			(AHB1PERI_BASEADDR + 0x3800)
/*Base Addresses for peripherals on the APB1 BUS(I2C, UART,SPI)*/

#define I2C1_BASEADDR			(APB1PERI_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERI_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERI_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERI_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERI_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERI_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERI_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERI_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERI_BASEADDR + 0x5000)

/* Base Addresses for peripherals on the APB2 BUS */

#define SPI1_BASEADDR			(APB2PERI_BASEADDR + 0x3000)
#define USART1_BASEADDR			(APB2PERI_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERI_BASEADDR + 0x1400)
#define EXTI_BASEADDR			(APB2PERI_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERI_BASEADDR + 0x3800)

/*Peripheral Register definition structure*/

typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;

} GPIO_RegDef_t;

/*Type cast the base addresses for easier usage*/
#define GPIOA					((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t *) GPIOI_BASEADDR)


/* Type cast for GPIO register*/
#define GPIO					((RCC_RegDef_t *) RCC_BASEADDR)
/*RCC Register definition structure*/

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t Reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t Reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t Reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t Reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t  Reserved4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t Reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t Reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

} RCC_RegDef_t;

/* Type Cast for RCC register*/
#define RCC					((RCC_RegDef_t *) RCC_BASEADDR)

/* Clock Enable function macros for GPIOs*/

#define GPIOA_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<0))
#define GPIOB_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<1))
#define GPIOC_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<2))
#define GPIOD_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<3))
#define GPIOE_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<4))
#define GPIOF_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<5))
#define GPIOG_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<6))
#define GPIOH_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<7))
#define GPIOI_PCLK_EN()		((RCC -> AHB1ENR) |= (1<<8))

/* Clock Disable function macros for GPIOs*/

#define GPIOA_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<0))
#define GPIOB_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<1))
#define GPIOC_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<2))
#define GPIOD_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<3))
#define GPIOE_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<4))
#define GPIOF_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<5))
#define GPIOG_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<6))
#define GPIOH_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<7))
#define GPIOI_PCLK_DN()		((RCC -> AHB1ENR) &= ~(1<<8))

/* Clock Enable function macros for I2Cs*/

#define I2C1_PCLK_EN()		((RCC -> APB1ENR) |= (1<<21))
#define I2C2_PCLK_EN()		((RCC -> APB1ENR) |= (1<<22))
#define I2C3_PCLK_EN()		((RCC -> APB1ENR) |= (1<<23))

/* Clock Disable function macros for I2Cs*/

#define I2C1_PCLK_DN()		((RCC -> APB1ENR) &= ~(1<<21))
#define I2C2_PCLK_DN()		((RCC -> APB1ENR) &= ~(1<<22))
#define I2C3_PCLK_DN()		((RCC -> APB1ENR) &= ~(1<<23))

/* Clock Enable function macros for SPIs*/

#define SPI1_PCLK_EN()		((RCC -> APB2ENR) |= (1<<12))
#define SPI2_PCLK_EN()		((RCC -> APB1ENR) |= (1<<14))
#define SPI3_PCLK_EN()		((RCC -> APB1ENR) |= (1<<15))

/* Clock Disable function macros for SPIs*/

#define SPI1_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<12))
#define SPI2_PCLK_DN()		((RCC -> APB1ENR) &= ~(1<<14))
#define SPI3_PCLK_DN()		((RCC -> APB1ENR) &= ~(1<<15))

/*  Clock Enable function macros for USASRTs */

#define USART1_PCLK_EN()		((RCC -> APB2ENR) |= (1<<4))
#define USART2_PCLK_EN()		((RCC -> APB2ENR) |= (1<<17))
#define USART3_PCLK_EN()		((RCC -> APB2ENR) |= (1<<18))
#define USART6_PCLK_EN()		((RCC -> APB2ENR) |= (1<<5))


/*  Clock Disable function macros for USASRTs */

#define USART1_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<4))
#define USART2_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<17))
#define USART3_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<18))
#define USART6_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<5))

/*  Clock Enable function macros for SPIs */

#define UART1_PCLK_EN()			((RCC -> APB1ENR) |= (1<<19))
#define UART2_PCLK_EN()			((RCC -> APB1ENR) |= (1<<20))


/*  Clock Disable function macros for SPIs */

#define UART1_PCLK_DN()			((RCC -> APB1ENR) &= ~(1<<19))
#define UART2_PCLK_DN()			((RCC -> APB1ENR) &= ~(1<<20))

/* CLock Enable Function macros for SYSCFG */

#define SYSCFG_PCLK_EN()		((RCC -> APB2ENR) |= (1<<14))

/* CLock Disable Function macros for SYSCFG */

#define SYSCFG_PCLK_DN()		((RCC -> APB2ENR) &= ~(1<<14))

#endif /* STM32F407XX_H_ */
