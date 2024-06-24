/*
 * stm32f429.h
 *
 *  Created on: Jun 22, 2024
 *      Author: Brian Wu
 * MCU Specific Header File used for defining
 * memory addresses, peripheral addresses, and other
 * macros that correspond to the STM32F429XX MCUs.
 */

#ifndef INC_STM32F429_H_
#define INC_STM32F429_H_

#include <stdint.h> // For int sizes

/*
 * Size of SRAM and Flash for STM32F429XX
 */

#define SRAM1_SIZE			( (112)* (1024) )
#define SRAM2_SIZE			( (16) * (1024) )
#define SRAM3_SIZE			( (64) * (1024) )
#define FLASH_SIZE			( (2) * (1024) )
#define ROM_SIZE			( (30) * (1024))
/*
 * Base addresses for SRAM and FLASH Memory
 */

#define	SRAM1_BASE_ADDR		0x20000000U
#define	SRAM2_BASE_ADDR		SRAM1_BASE_ADDR + SRAM1_SIZE
#define SRAM3_BASE_ADDR		0x20020000U
#define	FLASH_BASE_ADDR		0x08000000U
#define	ROM_BASE_ADDR		0x1FFF0000U // System Memory

/*
 *	AHB & APBH Base Addresses
 */

#define PERIPH_BASE_ADDR	0x40000000U
#define APB1_BASE_ADDR		0x40000000U
#define APB2_BASE_ADDR		0x40010000U
#define AHB1_BASE_ADDR		0x40020000U
#define AHB2_BASE_ADDR		0x50000000U
#define ABH3_BASE_ADDR 		0xA0000000U



/*
 *	AHB1 Peripheral Base Addresses
 */
#define	GPIOA_BASE_ADDR		(AHB1_BASE_ADDR + 0x0000)
#define	GPIOB_BASE_ADDR		(AHB1_BASE_ADDR + 0x0400)
#define	GPIOC_BASE_ADDR		(AHB1_BASE_ADDR + 0x0800)
#define	GPIOD_BASE_ADDR		(AHB1_BASE_ADDR + 0x0C00)
#define	GPIOE_BASE_ADDR		(AHB1_BASE_ADDR + 0x1000)
#define	GPIOF_BASE_ADDR		(AHB1_BASE_ADDR + 0x1400)
#define	GPIOG_BASE_ADDR		(AHB1_BASE_ADDR + 0x1800)
#define	GPIOH_BASE_ADDR		(AHB1_BASE_ADDR + 0x1C00)
#define	GPIOI_BASE_ADDR		(AHB1_BASE_ADDR + 0x2000)
#define RCC_BASE_ADDR		(AHB1_BASE_ADDR + 0x3800)

/*
 * APB1 Peripheral Base Addresses
 */
#define	SPI2_BASE_ADDR		(APB1_BASE_ADDR + 0x3800)
#define	SPI3_BASE_ADDR		(APB1_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR	(APB1_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR	(APB1_BASE_ADDR + 0x4800)

#define UART4_BASE_ADDR		(APB1_BASE_ADDR + 0x4c00)
#define UART5_BASE_ADDR		(APB1_BASE_ADDR + 0x5000)

#define	I2C1_BASE_ADDR		(APB1_BASE_ADDR + 0x5400)
#define	I2C2_BASE_ADDR		(APB1_BASE_ADDR + 0x5800)
#define	I2C3_BASE_ADDR		(APB1_BASE_ADDR + 0x5C00)

/*
 *	APB2 Peripheral Base Addresses
 */

#define	USART1_BASE_ADDR		(APB2_BASE_ADDR + 0x1000)
#define	USART6_BASE_ADDR		(APB2_BASE_ADDR + 0x1400)
#define	SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0x3000)
#define	EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0x3C00)
#define	SYSCFG_BASE_ADDR		(APB2_BASE_ADDR + 0x3800)

/****************************************** Peripheral Register Struct Definitions **********************************/


#define __vo volatile // volatile shorthand

typedef struct {
	__vo uint32_t 	CR;				// RCC clock control register 					Addr Offset:	0x00
	__vo uint32_t 	PLLCFGR;		// RCC PLL configuration register				Addr Offset:	0x04
	__vo uint32_t 	CFGR;			// RCC clock configuration register				Addr Offset:	0x08
	__vo uint32_t 	CIR;			// RCC clock interrupt register					Addr Offset:	0x0C
	__vo uint32_t 	AHB1RSTR;		// RCC AHB1 peripheral reset register			Addr Offset:	0x10
	__vo uint32_t 	AHB2RSTR;		// RCC AHB2 peripheral reset register			Addr Offset:	0x14
	__vo uint32_t 	AHB3RSTR;		// RCC AHB3 peripheral reset register			Addr Offset:	0x18
	const uint32_t 	RES_1C;			// Reserved Address Space						Addr Offset: 	0x1C
	__vo uint32_t 	APB1RSTR;		// RCC APB1 peripheral reset register			Addr Offset:	0x20
	__vo uint32_t 	APB2RSTR;		// RCC APB2 peripheral reset register			Addr Offset:	0x24
	const uint32_t 	RES_28;			// Reserved Address Space						Addr Offset: 	0x28
	const uint32_t 	RES_2C;			// Reserved Address Space						Addr Offset: 	0x2C
	__vo uint32_t 	AHB1ENR;		// RCC AHB1 peripheral clock enable register	Addr Offset:	0x30
	__vo uint32_t 	AHB2ENR;		// RCC AHB2 peripheral clock enable register	Addr Offset:	0x34
	__vo uint32_t 	AHB3ENR;		//RCC AHB3 peripheral clock enable register		Addr Offset:	0x38
	const uint32_t 	RES_3C;			// Reserved Address Space						Addr Offset: 	0x3C
	__vo uint32_t 	APB1ENR;		// RCC APB1 peripheral clock enable register	Addr Offset:	0x40
	__vo uint32_t 	APB2ENR;		// RCC APB2 peripheral clock enable register	Addr Offset:	0x44
	const uint32_t 	RES_48;			// Reserved Address Space						Addr Offset: 	0x48
	const uint32_t 	RES_4C;			// Reserved Address Space						Addr Offset: 	0x4C
	__vo uint32_t 	AHB1LPENR;		// RCC AHB1 peripheral clock enable in low power mode register	Addr Offset:	0x50
	__vo uint32_t 	AHB2LPENR;		// RCC AHB2 peripheral clock enable in low power mode register	Addr Offset:	0x54
	__vo uint32_t 	AHB3LPENR;		// RCC AHB3 peripheral clock enable in low power mode register	Addr Offset:	0x58
	const uint32_t 	RES_5C;			// Reserved Address Space										Addr Offset: 	0x5C
	__vo uint32_t 	APB1LPENR;		// RCC APB1 peripheral clock enable in low power mode register	Addr Offset:	0x60
	__vo uint32_t 	APB2LPENR;		// RCC APB2 peripheral clock enabled in low power mode			Addr Offset:	0x64
	const uint32_t 	RES_68;			// Reserved Address Space						Addr Offset: 	0x68
	const uint32_t 	RES_6C;			// Reserved Address Space						Addr Offset: 	0x6C
	__vo uint32_t 	BDCR;			// RCC Backup domain control register			Addr Offset:	0x70
	__vo uint32_t 	CSR;			// RCC clock control & status register			Addr Offset:	0x74
	const uint32_t 	RES_78;			// Reserved Address Space						Addr Offset: 	0x78
	const uint32_t 	RES_7C;			// Reserved Address Space						Addr Offset: 	0x7C
	__vo uint32_t 	SSCGR;			// RCC spread spectrum clock generation register				Addr Offset:	0x80
	__vo uint32_t 	PLLI2SCFGR;		// RCC PLLI2S configuration register			Addr Offset:	0x84
} RCC_RegDef_t;

typedef struct {
	__vo uint32_t MODER;	// GPIO port mode register					Addr Offset:	0x00
	__vo uint32_t OTYPER;	// GPIO port output type register			Addr Offset:	0x04
	__vo uint32_t OSPEEDR;	// GPIO port output speed register			Addr Offset:	0x08
	__vo uint32_t PUPDR;	// GPIO port pull-up/pull-down register		Addr Offset:	0x0C
	__vo uint32_t IDR;		// GPIO port input data register			Addr Offset:	0x10
	__vo uint32_t ODR;		// GPIO port output data register			Addr Offset:	0x14
	__vo uint32_t BSRR;		// GPIO port bit set/reset register			Addr Offset:	0x18
	__vo uint32_t LCKR;		// GPIO port configuration lock register	Addr Offset:	0x1C
	__vo uint32_t AFR[2];	// AFR[0] : AFR Low, AFR[1] AFR High		Addr Offset:	0x20, 0x24
							// GPIO alternate function low register
} GPIO_RegDef_t;

/*
 * Peripheral Definitions 	(Peripheral Base Addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA				((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB				((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC				((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD				((GPIO_RegDef_t *) GPIOD_BASE_ADDR)
#define GPIOE				((GPIO_RegDef_t *) GPIOE_BASE_ADDR)
#define GPIOF				((GPIO_RegDef_t *) GPIOF_BASE_ADDR)
#define GPIOG				((GPIO_RegDef_t *) GPIOG_BASE_ADDR)
#define GPIOH				((GPIO_RegDef_t *) GPIOH_BASE_ADDR)
#define GPIOI				((GPIO_RegDef_t *) GPIOI_BASE_ADDR)

#define RCC					((RCC_RegDef_t *) RCC_BASE_ADDR)



/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define	GPIOA_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define	GPIOB_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define	GPIOC_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define	GPIOD_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define	GPIOE_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define	GPIOF_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define	GPIOG_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define	GPIOH_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define	GPIOI_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 8))



/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 21)
#define I2C2_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 22)
#define I2C3_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 23)

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI2_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 14)
#define SPI3_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 15)

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_P_CLK_EN()			(RCC->APB2ENR) |= (1 << 4)
#define USART6_P_CLK_EN()			(RCC->APB2ENR) |= (1 << 5)
/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_P_CLK_EN()			(RCC->APB2ENR) |= (1 << 14)


/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define	GPIOA_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define	GPIOB_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define	GPIOC_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define	GPIOD_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define	GPIOE_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define	GPIOF_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define	GPIOG_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define	GPIOH_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define	GPIOI_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))



/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 21)
#define I2C2_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 22)
#define I2C3_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 23)

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI2_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 14)
#define SPI3_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 15)

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 4)
#define USART6_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 5)
/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 14)
#endif /* INC_STM32F429_H_ */






