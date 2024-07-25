#include "stm32f429_spi_driver.h"
#include "stm32f429_gpio_driver.h"
#include "string.h"

/* Alternate Function Mapping for SPI1

PA4 -> SPI1_NSS
PA5 -> SPI1_SCLK
PA6 -> SPI1_MISO
PA7 -> SPI1_MOSI
AF5
*/

void SPI_GPIOInits(void) {
	GPIO_Handle_t SPIPins;
	GPIO_PinConfig_t PinConfig = SPIPins.GPIO_PinConfig;
	SPIPins.pGPIOx = GPIOA;
	PinConfig.GPIO_PinAltFunMode = 5;
	PinConfig.GPIO_PinOPType =  GPIO_OP_TYPE_PP;
	PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// NSS
	PinConfig.GPIO_pinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

	// SCLK
	PinConfig.GPIO_pinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	// MISO
	PinConfig.GPIO_pinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	// MOSI 
	PinConfig.GPIO_pinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

} 

void SPI1_Inits(void) {
	SPI_Handle_t SPI1_Handle;
	SPI_Config_t SPIConfig = SPI1_Handle.SPIConfig;
	
	SPI1_Handle.pSPIx = SPI1;
	SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIConfig.SPI_SCLKSpeed = SPI_SCLKSPEED_DIV2;
	SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIConfig.SPI_CPHA = SPI_CPHA_FIRST;
	SPIConfig.SPI_SSM = SPI_SSM_EN;
	SPIConfig.SPI_SSI = SPI_SSI_HIGH;

	SPI_Init(&SPI1_Handle);

}

int main(void) {
	char user_data[] = "Hello World";

	// Initialize the GPIO Pins for the SPI1 Pins
	SPI_GPIOInits();

	// Initialize the SPI2 Peripheral Parameters
	SPI1_Inits();

	// Enable the SPI2 Peripheral
	SPI_PeripheralControl(SPI1, ENABLE);
	SPI_SendData(SPI1, (uint8_t *) user_data, strlen(user_data));

	while(1);

	return 0;

}
