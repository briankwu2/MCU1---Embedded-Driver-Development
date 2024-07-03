/*
 * stm32f429_gpio_driver.h
 *
 *  Created on: Jun 24, 2024
 *      Author: viruple
 */

#ifndef INC_STM32F429_GPIO_DRIVER_H_
#define INC_STM32F429_GPIO_DRIVER_H_
#include "stm32f429.h"

/**
 * 
 */
typedef enum {
	GPIO_PIN_NO_0,
	GPIO_PIN_NO_1,
	GPIO_PIN_NO_2,
	GPIO_PIN_NO_3,	
	GPIO_PIN_NO_4,	
	GPIO_PIN_NO_5,	
	GPIO_PIN_NO_6,	
	GPIO_PIN_NO_7,	
	GPIO_PIN_NO_8,	
	GPIO_PIN_NO_9,	
	GPIO_PIN_NO_10,		
	GPIO_PIN_NO_11,	
	GPIO_PIN_NO_12,		
	GPIO_PIN_NO_13,		
	GPIO_PIN_NO_14,		
	GPIO_PIN_NO_15
} GPIO_PIN_NUMBER;

/** @GPIO_PIN_MODES 
 * GPIO Input Possible Modes
 * 
 */

typedef enum {
	GPIO_MODE_IN,
	GPIO_MODE_OUT,
	GPIO_MODE_ALTFN,
	GPIO_MODE_ANALOG,
	GPIO_MODE_IT_FT,		 // Interrupt - Falling Trigger
	GPIO_MODE_IT_RT,		 // Interrupt - Rising Trigger
	GPIO_MODE_IT_RFT		 // Interrupt - Rising & Falling Trigger
} GPIO_PIN_MODE;

/** @GPIO_OUTPUT_MODE
 * GPIO Output Possible Modes
 */

typedef enum {
	GPIO_OP_TYPE_PP,	 // Push-Pull Type
	GPIO_OP_TYPE_OD	 // Open-Drain Type
} GPIO_OUTPUT_MODE;

/** @GPIO_OUTPUT_SPEED
 *	GPIO Pin Possible Output Speeds
 */
typedef enum {
	GPIO_SPEED_LOW,
	GPIO_SPEED_MEDIUM,	
	GPIO_SPEED_FAST,
	GPIO_SPEED_HIGH
} GPIO_OUTPUT_SPEED;

/** @GPIO_PUPD_MODE
 *	GPIO Pin Pull-Up and Pull-Down Configuration Macros 
 */
typedef enum {
	GPIO_NO_PUPD,		 // No Pull-Up or Pull-Down
	GPIO_PIN_PU, 		 // Pull-Up
	GPIO_PIN_PD			 // Pull-Down
} GPIO_PUPD_MODE;


typedef struct {
	GPIO_PIN_NUMBER GPIO_pinNumber;
	GPIO_PIN_MODE GPIO_PinMode; 		// Possible values from @GPIO_PIN_MODE
	GPIO_OUTPUT_SPEED GPIO_PinSpeed;		// Possible values from @GPIO_OUTPUT_SPEED
	GPIO_PUPD_MODE GPIO_PinPuPdControl;// Possible values from @GPIO_PUPD_MODE
	GPIO_OUTPUT_MODE GPIO_PinOPType;		// Possible values from @GPIO_OUTPUT_MODE
	uint16_t GPIO_PinAltFunMode;	// Possible values from (WIP)
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
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
/*
 *	IRQ Config and ISR Handling
 */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDI); 
void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint32_t IRQPriority); 
void GPIO_IRQHandling(uint8_t pinNumber);
uint8_t getPortCode(GPIO_RegDef_t* pGPIOx);

#endif /* INC_STM32F429_GPIO_DRIVER_H_ */
