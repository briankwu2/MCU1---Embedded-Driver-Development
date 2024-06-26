// 001LED_Toggle
/*
    Toggles on an on-board LED using the GPIO_Drivers built from scratch
*/

#include "stm32f429_gpio_driver.h"


#define LED_GREEN       GPIO_PIN_NO_13
#define LED_ORANGE      GPIO_PIN_NO_14

void delay(void) {
    for (uint32_t i = 0; i < 500000; i++);
}
int main(void) 
{
    GPIO_Handle_t gpio_led_green;
    gpio_led_green.pGPIOx = GPIOG;
    gpio_led_green.GPIO_PinConfig.GPIO_pinNumber = LED_GREEN;
    gpio_led_green.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led_green.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led_green.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpio_led_green.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Handle_t gpio_led_orange;
    gpio_led_orange.pGPIOx = GPIOG;
    gpio_led_orange.GPIO_PinConfig.GPIO_pinNumber = LED_ORANGE;
    gpio_led_orange.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpio_led_orange.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpio_led_orange.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpio_led_orange.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    // Must Enable clock before initializing and using some peripheral
    GPIO_PeriClockControl(GPIOG, ENABLE);
    GPIO_Init(&gpio_led_green);
    GPIO_Init(&gpio_led_orange);

    while (1) {
        GPIO_ToggleOutputPin(GPIOG, LED_GREEN);
        GPIO_ToggleOutputPin(GPIOG, LED_ORANGE);
        delay();
    }



    return 0;

}
