/*
 * stm32f429_gpio_driver.h
 *
 *  Created on: Jun 24, 2024
 *      Author: viruple
 */

#ifndef INC_STM32F429_GPIO_DRIVER_H_
#define INC_STM32F429_GPIO_DRIVER_H_
#include "stm32f429.h"





typedef struct {
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOPType;
	uint8_t	GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle Structure for GPIO Pin
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;	// This holds the base address of the GPIO Port to which the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;






/*************************************************** Function Prototypes ***************************************/

/***************************************************************************
 * 			APIs supported by this driver
 *		For more information check the API function definitions
 ***************************************************************************/

/*
 * Peripheral Clock Set-Up
 */
void GPIO_PeriClockControl(void);

/*
 * Init and DeInit
 */
void GPIO_Init(void);
void GPIO_DeInit(void);

/*
 * Data Read and Write
 */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);

/*
 *	IRQ Config and ISR Handling
 */
void GPIO_ToggleOutputPin(void);
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);



#endif /* INC_STM32F429_GPIO_DRIVER_H_ */