#include "stm32f429_gpio_driver.h"

/****************************************************************************************************
 * Peripheral Clock Set-Up
 ****************************************************************************************************/

/**
 * @brief This function enables or disables peripheral clock for the given GPIO Port
 * 
 * @param pGPIOx - Address to GPIOx Peipheral
 * @param EnOrDI - Enable or Disable
 * 
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDI) {
    if (EnOrDI == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_P_CLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_P_CLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_P_CLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_P_CLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_P_CLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_P_CLK_EN();
        } else if (pGPIOx == GPIOG) {
            GPIOG_P_CLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_P_CLK_EN();
        } else if (pGPIOx == GPIOI) {
            GPIOI_P_CLK_EN();
        } else if (pGPIOx == GPIOJ) {
            GPIOJ_P_CLK_EN();
        } else if (pGPIOx == GPIOK) {
            GPIOK_P_CLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_P_CLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_P_CLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_P_CLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_P_CLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_P_CLK_DI();
        } else if (pGPIOx == GPIOF) {
            GPIOF_P_CLK_DI();
        } else if (pGPIOx == GPIOG) {
            GPIOG_P_CLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_P_CLK_DI();
        } else if (pGPIOx == GPIOI) {
            GPIOI_P_CLK_DI();
        } else if (pGPIOx == GPIOJ) {
            GPIOJ_P_CLK_DI();
        } else if (pGPIOx == GPIOK) {
            GPIOK_P_CLK_DI();
        }

    }
}


/*******************************************************************************************************
 * Init and DeInit
 ******************************************************************************************************/


/**
 * @brief Initializes the GPIO peripheral using the given handle
 * 
 * @param pGPIOHandle - Handle of the given GPIO Peripheral with desired configurations
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    // Inner enum to make some logic easier

    //1. Configure the Mode of the GPIO Pin
    uint32_t temp = 0; // Temp Register for holding configuration values to set to the real register
    uint32_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber;
    GPIO_PIN_MODE pinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;

    // Non-Interrupt Modes
    if (pinMode <= GPIO_MODE_ANALOG) {

        // Changed for better readability
        // temp = pinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber)

        // "2 *" is for the 2 bits per pin used to set the mods
        temp = pinMode << (2 * pinNumber); // Shifts the pinMode to apply to the correct pin. 

        // Set the real register
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pinNumber); // Resets
        pGPIOHandle->pGPIOx->MODER |= temp; // Sets
        temp = 0;


    } else { // Interrupt Modes

        // 1. Configure the Edge Detection
        /*
            The pin number is the same as the specific EXTIx line. It is muxed, and the GPIO port is selected later
        */
        if (pinMode >= GPIO_MODE_ANALOG) { // If interrupt option mode
            switch (pinMode) {
                case GPIO_MODE_IT_FT:
                    EXTI->FTSR |= (1 << pinNumber);
                    EXTI->RTSR &= ~(1 << pinNumber);
                    break;
                case GPIO_MODE_IT_RT:
                    EXTI->FTSR &= (1 << pinNumber);
                    EXTI->RTSR |= ~(1 << pinNumber);
                    break;
                case GPIO_MODE_IT_RFT:
                    EXTI->FTSR |= (1 << pinNumber);
                    EXTI->RTSR |= ~(1 << pinNumber);
                    break;
                default:
                    break;
            }

        }
        // 2. Configure the GPIO Port Selection in SYSCFG_EXTICR
        uint8_t SYSCFGIndex = pinNumber / 4;
        uint8_t SYSCFGOffset = pinNumber % 4;
        uint8_t portCode = getPortCode(pGPIOHandle->pGPIOx);
        SYSCFG_P_CLK_EN();

        SYSCFG->EXTICR[SYSCFGIndex] |= portCode << SYSCFGOffset;

        // 3. Enable the EXTI Interrupt Delivery using IMR
        EXTI->IMR |= (1 << pinNumber); //
        

    }

    //2. Configure the Speed
    uint32_t pinSpeed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
    // "2 *" is for the 2 bits per pin used to set the mods
    temp = pinSpeed << (2 * pinNumber); // Shifts the pinMode to apply to the correct pin. 
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pinNumber); // Resets
    pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Set the real register
    temp = 0;


    //3. Configure the Pullup-Pulldown Settings
    uint32_t pinPUPD = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
    // "2 *" is for the 2 bits per pin used to set the mods
    temp = pinPUPD << (2 * pinNumber); // Shifts the pinMode to apply to the correct pin. 
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pinNumber); // Resets
    pGPIOHandle->pGPIOx->PUPDR |= temp; // Set the real register
    temp = 0;

    //4. Configure the Output Type
    uint32_t pinOPType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
    temp = pinOPType << pinNumber; // Shifts the pinMode to apply to the correct pin.
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pinNumber); // Resets
    pGPIOHandle->pGPIOx->OTYPER |= temp; // Set the real register
    temp = 0;

    //5. Configure the Alternate Functionality
    if (pinMode == GPIO_MODE_ALTFN) {
        // Configure the Alt Function Registers

        // Calculate the position of the pin's register
        uint32_t AFRIndex = pinNumber / 8;
        uint32_t AFROffset = 4 * (pinNumber % 8); // "4 *" as each pin takes 4 bits
        uint32_t AFRvalue = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;
        temp = AFRvalue << AFROffset;
        pGPIOHandle->pGPIOx->AFR[AFRIndex] &= ~(0x3 << pinNumber); // Resets
        pGPIOHandle->pGPIOx->AFR[AFRIndex] |= temp;
    }

}

/**
 * @brief Deinitializes the GPIO peripheral and resets to default values
 * 
 * @param pGPIOx Address to the GPIO Peripheral
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_P_CLK_EN();
    } else if (pGPIOx == GPIOB) {
        GPIOB_P_CLK_EN();
    } else if (pGPIOx == GPIOC) {
        GPIOC_P_CLK_EN();
    } else if (pGPIOx == GPIOD) {
        GPIOD_P_CLK_EN();
    } else if (pGPIOx == GPIOE) {
        GPIOE_P_CLK_EN();
    } else if (pGPIOx == GPIOF) {
        GPIOF_P_CLK_EN();
    } else if (pGPIOx == GPIOG) {
        GPIOG_P_CLK_EN();
    } else if (pGPIOx == GPIOH) {
        GPIOH_P_CLK_EN();
    } else if (pGPIOx == GPIOI) {
        GPIOI_P_CLK_EN();
    } else if (pGPIOx == GPIOJ) {
        GPIOJ_P_CLK_EN();
    } else if (pGPIOx == GPIOK) {
        GPIOK_P_CLK_EN();
    }
}


/********************************************************************************************************
 * Data Read and Write
 ********************************************************************************************************/

/**
 * @brief Reads from the GPIO input pin specified
 * 
 * @param pGPIOx - Address to the GPIO Peripheral
 * @param pinNumber - Pin to read
 * @return uint8_t - Returns a bit from the pin 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
    uint8_t value;
    value = (uint8_t) ((pGPIOx->IDR >> pinNumber) & 0x00000001); // Gets the (pinNumber)'th data bit from the IDR
    return value;
}

/**
 * @brief Reads from the GPIO Input Port (all 16 bits at once)
 * 
 * @param pGPIOx - Address to the GPIO Peripheral 
 * @return uint16_t - Returns the 16 bit wide register for all 16 pins
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value;
    value = (uint16_t) (pGPIOx->IDR); // Gets the whole port
    return value;
}

/**
 * @brief Writes to the GPIO output pin specified
 * 
 * @param pGPIOx - Address of the GPIOx Port
 * @param pinNumber - The specific pin number of the GPIOx port
 * @param value - The bit to be written on that pin
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
    if (value == GPIO_PIN_SET) {
        pGPIOx->ODR |= (1 << pinNumber);
    } else {
        pGPIOx->ODR &= ~(1 << pinNumber);
    }

}


/**
 * @brief Writes to the whole GPIOx Output Port Register
 * 
 * @param pGPIOx - Address of the GPIOx Port
 * @param value - 16 bit value to be written to the register
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
    pGPIOx->ODR = value;
}


/**
 * @brief Toggles an output pin for a specific pin for a GPIOx port
 * 
 * @param pGPIOx - Address of the GPIOx Port
 * @param pinNumber - Specific pin of the GPIOx Port
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
    /*
        How XOR'ing Works
        A = 0b1010
        Want to toggle the 2nd bit of A
        End result -> 0b1000 (turns 1 into 0)
        Logic is, if 2nd bit == 1, then turn to 0
                  if 2nd bit == 0, then turn to 1
        1010 ^ 0010
        Any bit encountered with a 0 will stay the same!
        (given that left bit is the original number and right is the flag to toggle)
        1 ^ 0 = 1
        0 ^ 0 = 0
        However any bit encountered with a 1 will toggle
        1 ^ 1 = 0
        1 ^ 0 = 1
    */
    pGPIOx->ODR ^= (1 << pinNumber); // Toggles the (pinNumber)th bit on the ODR
}

/**************************************************************************************************
 *	IRQ Config and ISR Handling
 **************************************************************************************************/

/**
 * @brief Configures the IRQ specified
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 * @param EnOrDI Enable or Disable
 */
void GPIO_IRQInteruptConfig(uint8_t IRQNumber, uint8_t EnOrDI) {
    if (EnOrDI == ENABLE) {
        if (IRQNumber >= 0 && IRQNumber < 31) {
            *NVIC_ISER0 |= (1 << IRQNumber);
        } if (IRQNumber >= 32 && IRQNumber < 63) {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } if (IRQNumber >= 64 && IRQNumber < 95) {
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        }
        
    } else if (EnOrDI == DISABLE) {
        if (IRQNumber >= 0 && IRQNumber < 31) {
            *NVIC_ICER0 |= (1 << IRQNumber);
        } if (IRQNumber >= 32 && IRQNumber < 63) {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } if (IRQNumber >= 64 && IRQNumber < 95) {
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
        }
    }
}

void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint32_t IRQPriority) {
    uint8_t IPRx = IRQNumber / 4;  // Interupt Priority Register index, where x is the index
    uint8_t IPROffset = ((IRQNumber % 4) * 8) + (8 - NO_PR_BITS_IMPLEMENTED) ; // The bit offset for the Interupt Priority Register

    *(NVIC_PR_BASE_ADDR + IPRx) = (IRQPriority << IPROffset);
}

/**
 * @brief Specifies 
 * 
 * @param pinNumber 
 */
void GPIO_IRQHandling(uint8_t pinNumber) {
    // Clears the EXTI Pending Register corresponding to the pin number
    if(EXTI->PR & (1 << pinNumber)) {

        EXTI->PR |= (1 << pinNumber); // Clear by setting 1

    }
}



/**
 * @brief Helper Functions
 * 
 */

uint8_t getPortCode(GPIO_RegDef_t* pGPIOx) {
    typedef enum {
        PORT_GPIOA = 0,
        PORT_GPIOB,
        PORT_GPIOC,
        PORT_GPIOD,
        PORT_GPIOE,
        PORT_GPIOF,
        PORT_GPIOG,
        PORT_GPIOH,
        PORT_GPIOI,
        PORT_GPIOJ,
        PORT_GPIOK
    } GPIO_PORT;

    GPIO_PORT portCode;

    if (pGPIOx == GPIOA) { 
        portCode = PORT_GPIOA;
    } else if (pGPIOx == GPIOB) {
        portCode = PORT_GPIOB;
    } else if (pGPIOx == GPIOC) {
        portCode = PORT_GPIOC;
    } else if (pGPIOx == GPIOD) {
        portCode = PORT_GPIOD;
    } else if (pGPIOx == GPIOE) {
        portCode = PORT_GPIOE;
    } else if (pGPIOx == GPIOF) {
        portCode = PORT_GPIOF;
    } else if (pGPIOx == GPIOG) {
        portCode = PORT_GPIOG;
    } else if (pGPIOx == GPIOH) {
        portCode = PORT_GPIOH;
    } else if (pGPIOx == GPIOI) {
        portCode = PORT_GPIOI;
    } else if (pGPIOx == GPIOJ) {
        portCode = PORT_GPIOJ;
    } else if (pGPIOx == GPIOK) {
        portCode = PORT_GPIOK;
    }

    return ((uint8_t) portCode);
}
