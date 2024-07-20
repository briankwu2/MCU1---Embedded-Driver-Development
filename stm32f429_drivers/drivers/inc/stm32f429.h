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

/*********************************START: Processor Specific Details ***************************/


/**
 * 
 * ARM Cortex M-4 Processor NVIC ISERx Register Addresess
 * Only need ISER0-ISER2 as there exist IRQs from 0-90 in the STM32F42X controllers
 */

#define NVIC_ISER0		((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*) 0xE000E108)

#define NVIC_ICER0		((__vo uint32_t*) 0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*) 0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*) 0xE000E188)

#define NVIC_PR_BASE_ADDR ((__vo uint32_t*) 0xE000E400) 
#define NO_PR_BITS_IMPLEMENTED		4

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
#define	GPIOJ_BASE_ADDR		(AHB1_BASE_ADDR + 0x2400)
#define	GPIOK_BASE_ADDR		(AHB1_BASE_ADDR + 0x2800)

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

/**
 * RCC Peripheral Register Struct
 */
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
	__vo uint32_t 	PLLSAICFGR;		// RCC PLL configuration register			Addr Offset:	0x88
	__vo uint32_t 	DCKCFGR;		// RCC Dedicated Clock Configuration Register			Addr Offset:	0x8C
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

typedef struct {
	__vo uint32_t IMR;		// Interrupt Mask Register				Addr Offset: 0x00
	__vo uint32_t EMR;		// Event Mask Register 					Addr Offset: 0x04
	__vo uint32_t RTSR;		// Rising Trigger Selection Register 	Addr Offset: 0x08
	__vo uint32_t FTSR;		// Fall Trigger Selection Register		Addr Offset: 0x0C
	__vo uint32_t SWIER;	//	Software Interrupt Event Reg		Addr Offset: 0x10
	__vo uint32_t PR;		// Pending Register 					Addr Offset: 0x14

} EXTI_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;		// Memory Remap Register				Addr Offset: 0x00
	__vo uint32_t PMC;			// Peripheral Mode Configuration		Addr Offset: 0x04
	__vo uint32_t EXTICR[4];	// EXTI Configuration Register			Addr Offset: 0x08
	__vo uint32_t CMPCR;		// Compensation Cell Control Register	Addr Offset: 0x0C
} SYSCFG_RegDef_t;


typedef struct {
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;


/*
 * GPIO Base Addresses (Peripheral Base Addresses typecasted to xxx_RegDef_t)
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
#define GPIOJ				((GPIO_RegDef_t *) GPIOJ_BASE_ADDR)
#define GPIOK				((GPIO_RegDef_t *) GPIOK_BASE_ADDR)



/**
 *	SPI Register Base Addresses
 */
#define SPI1	((SPI_RegDef_t *) SPI1_BASE_ADDR)
#define SPI2	((SPI_RegDef_t *) SPI2_BASE_ADDR)
#define SPI3	((SPI_RegDef_t *) SPI3_BASE_ADDR)

/**
 * Configuration Register Base Addresses
 */
#define RCC					((RCC_RegDef_t *) RCC_BASE_ADDR)
#define EXTI				((EXTI_RegDef_t*) EXTI_BASE_ADDR)
#define SYSCFG 				((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDR)

/********************************** Macros *********************************/
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
#define	GPIOJ_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 9))
#define	GPIOK_P_CLK_EN()		(RCC->AHB1ENR |= (1 << 10))



/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 21)
#define I2C2_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 22)
#define I2C3_P_CLK_EN()			(RCC->APB1ENR) |= (1 << 23)

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_P_CLK_EN()			(RCC->APB2ENR) |= (1 << 12)
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
 * Clock Disable Macros for GPIOx Peripherals
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
#define	GPIOJ_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 9))
#define	GPIOK_P_CLK_DI()		(RCC->AHB1ENR &= ~(1 << 10))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 21)
#define I2C2_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 22)
#define I2C3_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 23)

/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 12)
#define SPI2_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 14)
#define SPI3_P_CLK_DI()			(RCC->APB1ENR) &= ~(1 << 15)

/*
 * Clock Disable Macros for USARTx Peripherals
 */
#define USART1_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 4)
#define USART6_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 5)
/*
 * Clock Disable Macros for SYSCFG Peripherals
 */
#define SYSCFG_P_CLK_DI()			(RCC->APB2ENR) &= ~(1 << 14)
#endif /* INC_STM32F429_H_ */


/****************************************** RESET MACROS ***************************************** */
/*
 * Macros to reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET()			do{(RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); } while(0)



/**
 * Macros to reset SPI Peripherals 
 * 
 */

#define SPI1_REG_RESET()			do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)
/***********************************/
// Generic Macros

#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_SET        SET
#define FLAG_RESET      RESET 


// IRQ Numbers for EXTI Interrupts
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23	
#define IRQ_NO_EXTI15_10	40		
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

// Rest will be filled if needed, but this is for example
#define NVIC_IRQ_PRI0	0
#define NVIC_IRQ_PRI15	15	


/*********************************
 * Bit Positions 
 *********************************/

/*********************************
 * SPI Peripheral
 *********************************/

// SPI_CR1 Register Bit Positions
#define SPI_CR1_CPHA		0   // Clock Phase
#define SPI_CR1_CPOL		1   // Clock Polarity
#define SPI_CR1_MSTR		2 	// Master Selection
#define SPI_CR1_BR			3	// Baud Rate Control
#define SPI_CR1_SPE			6   // SPI Enable
#define SPI_CR1_LSB			7   // Frame Format
#define SPI_CR1_SSI			8   // Internal Slave Select
#define SPI_CR1_SSM			9   // Software Slave Management
#define SPI_CR1_RX_ONLY 	10  // Receive Only
#define SPI_CR1_DFF			11  // Data Frame Format
#define SPI_CR1_CRC_NEXT	12  // CRC Transfer Next
#define SPI_CR1_CRC_EN		13  // Hardware CRC Calculation Enable
#define SPI_CR1_BIDI_OE		14  // Output Enable in Bi-directional Mode
#define SPI_CR1_BIDI_MODE	15  // Bi-directional Data Mode Enable

// SPI_CR2 Register Bit Positions
#define SPI_CR2_RXDMAEN		0   // RX Buffer DMA Enable
#define SPI_CR2_TXDMAEN		1   // TX Buffer DMA Enable 
#define SPI_CR2_SSOE		2 	// SS Output Enable 
#define SPI_CR2_FRF			4   // Frame Format
#define SPI_CR2_ERRIE		5   // Error Interrupt Enable
#define SPI_CR2_RXNEIE		6   // RX Buffer Not Empty Interrupt Enable 
#define SPI_CR2_TXEIE		7   // TX Buffer Empty Interrupt Enable

// SPI_SR Regster Bit Positions
#define SPI_SR_RXNE		    0   // Receive Buffer Not Empty
#define SPI_SR_TXE 		    1   // Transmit Buffer Empty
#define SPI_SR_CHSIDE		2 	// Channel Side
#define SPI_SR_UDR			3	// Underrun Flag
#define SPI_SR_CRC_ERR		4   // CRC Error Flag
#define SPI_SR_MODF		    5   // Mode Fault
#define SPI_SR_OVR			6   // Overrun Flag
#define SPI_SR_BSY			7   // Busy Flag
#define SPI_SR_FRE 	        8   // Frame Format Error Flag