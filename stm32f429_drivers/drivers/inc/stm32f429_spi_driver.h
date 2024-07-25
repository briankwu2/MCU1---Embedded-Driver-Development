/*
 * stm32f429_spi_driver.h
 *
 *  Created on: Jul 3, 2024
 *      Author: viruple
 */

#ifndef INC_STM32F429_SPI_DRIVER_H_
#define INC_STM32F429_SPI_DRIVER_H_

#include "stm32f429.h"

/******************************** MACRO DEFINITIONS FOR SPI ***************/

/**
 * @brief Configures whether or not the device is a Master or Slave in the SPI.
 * SPI_DeviceMode Values
 * 
 */
typedef enum {
	SPI_DEVICE_MODE_SLAVE = 0,
	SPI_DEVICE_MODE_MASTER
} SPI_DEVICE_MODE;

/**
 * @brief Configuration for Full Duplex, Half Duplex, Simplex modes
 * SPI_BusConfig Values
 */
typedef enum {
	SPI_BUS_CONFIG_FD = 1,
	SPI_BUS_CONFIG_HD,
	SPI_BUS_CONFIG_SIMPLEX_TXONLY,
	SPI_BUS_CONFIG_SIMPLEX_RXONLY,

} SPI_BUSCONFIG;


/**
 * @brief Speed of to SCLK used to drive how fast
 * data is transferred in and out
 * SPI_SCLKSpeed Values
 * Calculated as Peripheral Bus Clk Speed / DIVx
 */
typedef enum {
	SPI_SCLKSPEED_DIV2 = 0,
	SPI_SCLKSPEED_DIV4, 
	SPI_SCLKSPEED_DIV8,
	SPI_SCLKSPEED_DIV16, 
	SPI_SCLKSPEED_DIV32,
	SPI_SCLKSPEED_DIV64,
	SPI_SCLKSPEED_DIV128,
	SPI_SCLKSPEED_DIV256,
} SPI_SCLKSPEED;

/**
 * @brief Data Frame Format for the TX and RX Buffer
 * 
 * SPI_DFF Values 
 */
typedef enum {
	SPI_DFF_8BITS = 0,
	SPI_DFF_16BITS
} SPI_DFF;

/**
 * @brief Clock Polarity. Defined as what value does the line idle at.
 * e.g. Idles at 0, until data is transferred if LOW
 * 		Idles at 1, until data is transferred if HIGH
 * SPI_CPOL Values 
 */
typedef enum {
	SPI_CPOL_LOW = 0,
	SPI_CPOL_HIGH,
} SPI_CPOL;

/**
 * @brief Clock Phase. Describes when the data is captured.
 * 
 * SPI_CPHA Values
 */
typedef enum {
	SPI_CPHA_FIRST = 0, // Data captured on the first clock edge
	SPI_CPHA_SECOND // Data captured on the second clock edge
} SPI_CPHA;

// SPI_SSM Values

/**
 * @brief Software Slave Management. Manages whether or not the slave
 * chip select is received through the SSI register bit or the physical pin.
 * SPI_SSM Values
 */
typedef enum {
	SPI_SSM_DI = 0,
	SPI_SSM_EN
} SPI_SSM;

/**
 * @brief Slave Select Internal - Bit used to assign to the Slave Select pin.
 * Only used if SSM is enabled.
 * 
 */
typedef enum {
	SPI_SSI_LOW = 0,
	SPI_SSI_HIGH
} SPI_SSI;


// SPI Related Flags & Macros
#define SPI_FLAG_TXE		(1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE		(1 << SPI_SR_RXNE)
#define SPI_FLAG_BSY		(1 << SPI_SR_BSY)

// SPI Application States
#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

// SPI Events
#define SPI_EVENT_TX_COMPLETE	1
#define SPI_EVENT_RX_COMPLETE	2
#define SPI_EVENT_OVR_ERR		3

/***************************************** SPI Configuration Structs *******************************************/
typedef struct {
	SPI_DEVICE_MODE SPI_DeviceMode;
	SPI_BUSCONFIG SPI_BusConfig;
	SPI_SCLKSPEED SPI_SCLKSpeed;
	SPI_DFF SPI_DFF;
	SPI_CPOL SPI_CPOL;
	SPI_CPHA SPI_CPHA;
	SPI_SSM SPI_SSM;
	SPI_SSI SPI_SSI;
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;	
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

/*********************************************************************************************************
 * 										APIs supported by this driver
 * 					For more information see function definitions in source file
 *********************************************************************************************************/

// Peripheral Clock Setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDI);
// Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// Data Send and Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len); // Interrupt enabled versions
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len); // Interrupt enabled versions

// IRQ Configuration and ISR Handling
void SPI_IRQInteruptConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDI); 
void SPI_IRQPriorityConfig(uint32_t IRQNumber, uint32_t IRQPriority); 
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

// Peripheral Control API
uint8_t SPI_getStatusFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t* pSPIHandle);
void SPI_CloseReception(SPI_Handle_t* pSPIHandle);

// Application Callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


#endif /* INC_STM32F429_SPI_DRIVER_H_ */

