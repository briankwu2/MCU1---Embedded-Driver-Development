/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
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

/* File for manually activating peripherals through the use of special registers.
 *
 */


// Macros --------------------------------------

// ADC Register Definitions
#define ADC_BASE_ADDR 			0x40012000UL
#define ADC_CR1_REG_OFFSET 		0x04UL
#define ADC_CR1_REG_ADDR		(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)


// RCC Register Definitions
#define RCC_BASE_ADDR  			0x40023800UL
#define RCC_APB2_REG_OFFSET		0x44UL
#define RCC_APB2_REG_ADDR 		(RCC_BASE_ADDR + RCC_APB2_REG_OFFSET)

// ---------------------------------------------
#include <stdint.h>

int main(void)
{
	/*
	 * Goal: Activate "Scan Mode" for the ADC peripheral.
	 * Method:
	 * - Enable the peripheral clock by using an RCC register, and set the 8th bit to enable ADC1
	 * - Set the 8th bit of the ADC_CR1 register
	 */

	uint32_t *pAPB2Reg = (uint32_t *) RCC_APB2_REG_ADDR;
	*pAPB2Reg |= (1 << 8); // Set the 8th bit


	uint32_t *pAdcCr1Reg = (uint32_t *) ADC_CR1_REG_ADDR;
	*pAdcCr1Reg |= (1 << 8); // Set the 8th bit

	for(;;);
}




















