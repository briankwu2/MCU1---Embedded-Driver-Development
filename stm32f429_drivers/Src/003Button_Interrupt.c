#include "stm32f429_gpio_driver.h"
#define ENABLE_COMPILATION 1     // Change to 1 and other "main" files to compile this one

/**
 *  Brief Wiring Explanation:
 *      An external LED should be connected to pin PA6 (with appropriate resistor and connected to ground)
 *      An external button should be connected to ground and other side to the pin PC4. Note that the
 *      internal pull-up resistor is on, so there is no need for an external pull-up resistor.
 *      The rail should be connected to 5V and GND respectively on the breadboard.
 * 
 */
#if ENABLE_COMPILATION
	#define EXTERNAL_LED GPIO_PIN_NO_6
    #define EXTERNAL_BUTTON GPIO_PIN_NO_4

	void delay(void) {
		for (uint32_t i = 0; i < 500000; i++);
	}

    int main(void) 
    {

        // GPIO Handle for LED Output 
        // Pin Name EXTERNAL_LED - GPIO Port A, Pin 6 

        GPIO_Handle_t gpio_led_red;
        gpio_led_red.pGPIOx = GPIOA;
        gpio_led_red.GPIO_PinConfig.GPIO_pinNumber = EXTERNAL_LED;
        gpio_led_red.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
        gpio_led_red.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
        gpio_led_red.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // Push-Pull Configuration
        gpio_led_red.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // No Pull-up or Pull-down resistor

        // GPIO Handle for External Button
        // Pin Name PC4
        GPIO_Handle_t gpio_button;
        gpio_button.pGPIOx = GPIOC;
        gpio_button.GPIO_PinConfig.GPIO_pinNumber = EXTERNAL_BUTTON;
        gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
        gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
        gpio_button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
        gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Activates Pull-Up Resistor

        // Must Enable clock before initializing and using some peripheral
        GPIO_PeriClockControl(GPIOC, ENABLE);
        GPIO_PeriClockControl(GPIOA, ENABLE);
        GPIO_Init(&gpio_led_red);
        GPIO_Init(&gpio_button);

        // IRQ Configurations
        GPIO_IRQPriorityConfig(IRQ_NO_EXTI4, NVIC_IRQ_PRI15);
        GPIO_IRQInteruptConfig(IRQ_NO_EXTI4, ENABLE);

        while (1);

        return 0;

    }
#endif

void EXTI4_IRQHandler(void) {
    GPIO_IRQHandling(EXTERNAL_BUTTON); // Requires pin number
    GPIO_ToggleOutputPin(GPIOA, EXTERNAL_LED);
    delay(); // For debouncing
}



