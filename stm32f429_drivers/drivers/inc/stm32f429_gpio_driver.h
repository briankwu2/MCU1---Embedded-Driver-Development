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
	uint8_t	GPIO_pinNumber;
	uint8_t	GPIO_PinMode; 		// Possible Values from @GPIO_PIN_MODES
	uint8_t	GPIO_PinSpeed;		// Possible Values from @GPIO_OUTPUT_SPEEDS
	uint8_t	GPIO_PinPuPdControl;// Possible Values from @GPIO_PUPD_MODES
	uint8_t	GPIO_PinOPType;		// Possible Values from @GPIO_OUTPUT_MODES
	uint8_t	GPIO_PinAltFunMode;	// Possible Values from
} GPIO_PinConfig_t;

/*
 * Handle Structure for GPIO Pin
 */

typedef struct {
	GPIO_RegDef_t *pGPIOx;	// This holds the base address of the GPIO Port to which the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;


/**
 * @GPIO_PIN_NUMBERS
 * 
 */
#define GPIO_PIN_NO_0		0	
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2	
#define GPIO_PIN_NO_3		3	
#define GPIO_PIN_NO_4		4	
#define GPIO_PIN_NO_5		5	
#define GPIO_PIN_NO_6		6	
#define GPIO_PIN_NO_7		7	
#define GPIO_PIN_NO_8		8	
#define GPIO_PIN_NO_9		9	
#define GPIO_PIN_NO_10		10	
#define GPIO_PIN_NO_11		11	2
#define GPIO_PIN_NO_12		12	
#define GPIO_PIN_NO_13		13	
#define GPIO_PIN_NO_14		14	
#define GPIO_PIN_NO_15		15

/** @GPIO_PIN_MODES 
 * GPIO Input Possible Modes
 * 
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4 // Interrupt - Falling Trigger
#define GPIO_MODE_IT_RT		5 // Interrupt - Rising Trigger
#define GPIO_MODE_IT_RFT	6 // Interrupt - Rising & Falling Trigger

/** @GPIO_OUTPUT_MODES
 * GPIO Output Possible Modes
 */

#define GPIO_OP_TYPE_PP		0 // Push-Pull Type
#define GPIO_OP_TYPE_OD		1 // Open-Drain Type

/** @GPIO_OUTPUT_SPEEDS
 *	GPIO Pin Possible Output Speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/** @GPIO_PUPD_MODES
 *	GPIO Pin Pull-Up and Pull-Down Configuration Macros 
 */
#define GPIO_NO_PUPD		0 // No Pull-Up or Pull-Down
#define GPIO_PIN_PU			1 // Pull-Up
#define GPIO_PIN_PD			2 // Pull-Down

/*************************************************** Function Prototypes ***************************************/

/***************************************************************************
 * 			APIs supported by this driver
 *		For more information check the API function definitions
 ***************************************************************************/

/*
 * Peripheral Clock Set-Up
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDI);

/*
 * Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
/*
 *	IRQ Config and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDI);
void GPIO_IRQHandling(uint8_t pinNumber);



#endif /* INC_STM32F429_GPIO_DRIVER_H_ */
