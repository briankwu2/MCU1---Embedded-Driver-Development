// 001-2LED_Toggle
/*
    Toggles on an on-board LED using the GPIO_Drivers built from scratch

    Exercises in this code:
    3 - Toggles an on-board LED using an on-board button
*/

#include "stm32f429_gpio_driver.h"
#define ENABLE_COMPILATION 0     // Change to 1 and other "main" files to compile this one

#if ENABLE_COMPILATION
	#define LED_GREEN       GPIO_PIN_NO_13
	#define LED_ORANGE      GPIO_PIN_NO_14
	#define USER_BUTTON_PIN GPIO_PIN_NO_0

	void delay(void) {
		for (uint32_t i = 0; i < 500000; i++);
	}

    int main(void) 
    {

        // GPIO Handle for on board green LED
        // Pin Name PG13 - GPIO Port A, Pin 13

        GPIO_Handle_t gpio_led_green;
        gpio_led_green.pGPIOx = GPIOG;
        gpio_led_green.GPIO_PinConfig.GPIO_pinNumber = LED_GREEN;
        gpio_led_green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
        gpio_led_green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
        gpio_led_green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-Pull Configuration
        gpio_led_green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No Pull-up or Pull-down resistor

        // GPIO Handle for User Button
        // Pin Name PA0 - GPIO Port A, Pin 0 
        GPIO_Handle_t gpio_board_button;
        gpio_board_button.pGPIOx = GPIOA;
        gpio_board_button.GPIO_PinConfig.GPIO_pinNumber = USER_BUTTON_PIN;
        gpio_board_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
        gpio_board_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
        gpio_board_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
        gpio_board_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // Activates Pull-Down Resistor

        // Must Enable clock before initializing and using some peripheral
        GPIO_PeriClockControl(GPIOG, ENABLE);
        GPIO_PeriClockControl(GPIOA, ENABLE);
        GPIO_Init(&gpio_led_green);
        GPIO_Init(&gpio_board_button);

        while (1) {
            if (GPIO_ReadFromInputPin(GPIOA, USER_BUTTON_PIN)) {
                GPIO_ToggleOutputPin(GPIOG, LED_GREEN);
                delay();
            } else {
                // Do Nothing
            }
        }


        return 0;

    }
#endif

