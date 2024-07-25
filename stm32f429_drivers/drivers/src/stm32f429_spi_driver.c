#include "stm32f429_spi_driver.h"

// Auxillary Function Prototypes
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

// Peripheral Clock Setup
/**
 * @brief Enables or disables the peripheral clock for SPI Peripherals 
 * 
 * @param pGPIOx 
 * @param EnorDI 
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI) { 
    switch (EnorDI) {
        case ENABLE:
            if (pSPIx == SPI1) {
                SPI1_P_CLK_EN();
            } else if (pSPIx == SPI2) {
                SPI2_P_CLK_EN();
            } else if (pSPIx == SPI3) {
                SPI3_P_CLK_EN();
            }
            break;
        case DISABLE:
            if (pSPIx == SPI1) {
                SPI1_P_CLK_DI();
            } else if (pSPIx == SPI2) {
                SPI2_P_CLK_DI();
            } else if (pSPIx == SPI3) {
                SPI3_P_CLK_DI();
            }
            break;
        default:
            break;
    }
}

// Init and De-Init

/**
 * @brief Initializes the SPI peripheral with the appropriate configurations
 * 
 * @param pSPIHandle Used to identify which SPIx port, and the configurations used to set
 * the SPI
 */
void SPI_Init(SPI_Handle_t *pSPIHandle) { 
    SPI_RegDef_t *pSPIx = pSPIHandle->pSPIx; 
    SPI_Config_t SPIConfig = pSPIHandle->SPIConfig;
    uint32_t temp = 0;


    // Enable SPI Peripheral Clock
    SPI_PeriClockControl(pSPIx, ENABLE);

    // CR1 Register Configuration

    // Device Mode
    temp |= (SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
    
    // Bus Config
    switch(SPIConfig.SPI_BusConfig) {
        case SPI_BUS_CONFIG_FD:
            temp &= ~(1 << SPI_CR1_BIDI_MODE); // 2 Line Unidirectional Option
            break;
        case SPI_BUS_CONFIG_HD:
            temp |= (1 << SPI_CR1_BIDI_MODE); // 1 Line Bidirectional Option
            break;
        case SPI_BUS_CONFIG_SIMPLEX_TXONLY:
            temp |= (1 << SPI_CR1_BIDI_MODE); // 1 Line Bidirectional Option
            temp |= (1 << SPI_CR1_BIDI_OE); // Transmit Only
            break;
        case SPI_BUS_CONFIG_SIMPLEX_RXONLY:
            temp |= (1 << SPI_CR1_BIDI_MODE); // 1 Line Bidirectional Option
            temp &= ~(1 << SPI_CR1_BIDI_OE); // Receive Only
            break;
        default:
            break;
    }

    // SCLK Speed Mode
    temp |= (SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR); // Sets 3 Bits

    // SPI Data Frame Format
    temp |= (SPIConfig.SPI_DFF << SPI_CR1_DFF);

    // SPI CPOL
    temp |= (SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

    // SPI CPHA
    temp |= (SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

    // SPI SSM
    temp |= (SPIConfig.SPI_SSM << SPI_CR1_SSM);

    temp |= (SPIConfig.SPI_SSI << SPI_CR1_SSI); 

    pSPIx->CR1 = temp; // Set the Register with configuration values

    
}

/**
 * @brief De-initializes the SPIx port. Uses a register to reset the SPI port to reset state.
 * 
 * @param pSPIx - SPIx Port Address
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) { 
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    }

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDI) {
    switch(EnOrDI) {
        case ENABLE:
            pSPIx->CR1 |= (1 << SPI_CR1_SPE);
            break;
        case DISABLE:
            pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
            break;
        default:
            break;
    }
}

// Data Send and Receive

/**
 * @brief Sends an data over SPI. Is a blocking function.
 * 
 * @param pSPIx - Contains the SPI port to send data over 
 * @param pTxBuffer - Pointer to buffer of data to send
 * @param len - length of the data
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) { 
    uint8_t TxEmpty;
    uint8_t numBytes;
    uint16_t DFFStatus = (pSPIx->CR1 >> SPI_CR1_DFF) & (0x0001);

    if (DFFStatus == 0) {
        numBytes = 1;
    } else if (DFFStatus == 1) {
        numBytes = 2;
    }

    while (len > 0) {
        TxEmpty = SPI_getStatusFlag(pSPIx, SPI_FLAG_TXE);
        while (!TxEmpty); // Block until TX Buffer is Empty

        // Load the data register with the buffer

        if (DFFStatus == 0) {
            pSPIx->DR = *pTxBuffer;
            (uint16_t *) pTxBuffer++;
        } else if (DFFStatus == 1) {

            pSPIx->DR = *((uint16_t*) pTxBuffer); // Collects 2 bytes
            pTxBuffer++;
        }

        len -= numBytes;
    }
}

/**
 * @brief Receives data through SPI. Is a blocking function.
 * 
 * @param pSPIx - Contains the SPI Port to receive data over
 * @param pRxBuffer - Pointer to the buffer of data to receive
 * @param len - length of the data
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) { 

    uint8_t RXNotEmpty;
    uint8_t DFFStatus = (pSPIx->CR1 >> SPI_CR1_DFF) & 0x01;

    while (len > 0) {
        RXNotEmpty = SPI_getStatusFlag(pSPIx, SPI_FLAG_RXNE);
        while (!RXNotEmpty);

        if (DFFStatus == 0) { // 8-bit DFF
            *pRxBuffer = pSPIx->DR;
            pRxBuffer++;
            len -= 1;
        } else if (DFFStatus == 1) {
            *((uint16_t*) pRxBuffer) = pSPIx->DR;
            (uint16_t *) pRxBuffer++;
            len -= 2;
        }
    }
}


/**
 * @brief Enables SPI data to be sent using SPI interrupt handler.
 * Does not actually send data using this function, but enables the interrupt handler to.
 * 
 * @param pSPIHandle 
 * @param pTxBuffer 
 * @param len 
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len) {
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_BUSY_IN_TX) {
        // 1. Save the TX Buffer address and len in some global variable
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = len;

        // 2. Mark the SPI state as busy in transmission
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE is set in the status register
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

    }

    return state;

}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len) {
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_BUSY_IN_RX) {
        // 1. Save the TX Buffer address and len in some global variable
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = len;

        // 2. Mark the SPI state as busy in transmission
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE is set in the status register
        pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

    }

    return state;
}

// IRQ Configuration and ISR Handling
/**
 * @brief Initializes and configures the interrupt from SPI
 * 
 * @param IRQNumber - IRQ Number of the SPI port
 * @param EnOrDI - Enable or Disable
 */
void SPI_IRQInteruptConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDI) { 

    // Enable the IRQ Number corresponding to the SPI peripheral
    if (EnOrDI == ENABLE) {
        if (pSPIx == SPI1) {
            *NVIC_ISER1 |= (1 << (IRQ_NO_SPI1 % 32));
        } else if (pSPIx == SPI2) {
            *NVIC_ISER1 |= (1 << (IRQ_NO_SPI2 % 32));
        } else if (pSPIx == SPI3) {
            *NVIC_ISER1 |= (1 << (IRQ_NO_SPI3 % 32));
        }

        // Enable all types of interrupts for SPI using the peripheral register
        pSPIx->CR2 |= (1 << SPI_CR2_TXEIE); // TX Buffer Empty Interrupt
        pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE); // RX Buffer Not Empty Interrupt
        pSPIx->CR2 |= (1 << SPI_CR2_ERRIE); // Error Interrupt 

    } else if (EnOrDI == DISABLE) {
        if (pSPIx == SPI1) {
            *NVIC_ISER1 &= ~(1 << (IRQ_NO_SPI1 % 32));
        } else if (pSPIx == SPI2) {
            *NVIC_ISER1 &= ~(1 << (IRQ_NO_SPI2 % 32));
        } else if (pSPIx == SPI3) {
            *NVIC_ISER1 &= ~(1 << (IRQ_NO_SPI3 % 32));
        }
        // Disables all types of interrupts for SPI using the peripheral register
        pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); // TX Buffer Empty Interrupt
        pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); // RX Buffer Not Empty Interrupt
        pSPIx->CR2 &= ~(1 << SPI_CR2_ERRIE); // Error Interrupt 
    }
        
} 

/**
 * @brief Configures the priority of the SPI Interrupt
 * 
 * @param IRQNumber - IRQ Number of the SPI Port
 * @param IRQPriority - Priority to assign
 */
void SPI_IRQPriorityConfig(uint32_t IRQNumber, uint32_t IRQPriority) { 
    uint8_t IPRx = IRQNumber / 4;  // Interupt Priority Register index, where x is the index
    uint8_t IPROffset = ((IRQNumber % 4) * 8) + (8 - NO_PR_BITS_IMPLEMENTED) ; // The bit offset for the Interupt Priority Register

    *(NVIC_PR_BASE_ADDR + IPRx) = (IRQPriority << IPROffset);
} 

/**
 * @brief Function that handles the SPI interrupt
 * 
 * @param pSPIHandle
 */
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle) { 

    // Check for interrupt reason
    uint8_t TXEmpty, TXInterruptEnable;

    TXEmpty = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
    TXInterruptEnable = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

    if (TXEmpty && TXInterruptEnable) {
        // Handle TXE     
        spi_txe_interrupt_handle(pSPIHandle);
    }

    uint8_t RXEmpty, RXInterruptEnable;

    RXEmpty = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
    RXInterruptEnable = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

    if (RXEmpty && RXInterruptEnable) {
        // Handle RXE     
        spi_rxne_interrupt_handle(pSPIHandle);
    }

    // Error Interrupt reasons

    // Check for OVR Flag
    uint8_t OVR, ERRIE;

    OVR = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
    ERRIE = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

    if (OVR && ERRIE) {
        // Handle RXE     

        spi_ovr_err_interrupt_handle(pSPIHandle);
    }
}

/**
 * @brief Auxilary function used to get the status of a flag from
 * the Status Register
 * 
 * @param pSPIx 
 * @param FlagName - Macro used to identify the mask of the flag
 * @return uint8_t 
 */
uint8_t SPI_getStatusFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName) {
    if (pSPIx->SR & FlagName) {
        return FLAG_SET;
    } else {
        return FLAG_RESET;
    }
}

/**
 * @brief Helper function to handle sending 1/2 bytes of data through the SPIx channel.
 * Calls the callback function to allow user to respond after all data is transmitted.
 * 
 * @param pSPIHandle
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    uint8_t TxEmpty;
    uint8_t numBytes;
    SPI_RegDef_t *pSPIx = pSPIHandle->pSPIx;
    uint8_t *pTxBuffer = pSPIHandle->pTxBuffer;

    uint16_t DFFStatus = (pSPIx->CR1 >> SPI_CR1_DFF) & (0x0001);

    if (DFFStatus == 0) {
        numBytes = 1;
    } else if (DFFStatus == 1) {
        numBytes = 2;
    }

    if (pSPIHandle->TxLen != 0) {
        // Load the data register with the buffer

        if (DFFStatus == 0) {
            pSPIx->DR = *pTxBuffer;
            (uint16_t *) pTxBuffer++;
        } else if (DFFStatus == 1) {

            pSPIx->DR = *((uint16_t*) pTxBuffer); // Collects 2 bytes
            pTxBuffer++;
        }

        pSPIHandle->TxLen -= numBytes;

    } else if (pSPIHandle->TxLen == 0) {
        // End TX Transmission and turn interrupts for TX off
        SPI_CloseTransmission(pSPIx);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLETE);
        // Function to alert application that TX has completed
    }
}
/**
 * @brief Helper function to handle receiving 1/2 bytes of data through the SPIx channel.
 * Calls the callback function to allow user to respond after all data is transmitted.
 * @param pSPIHandle 
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
    uint8_t RXNotEmpty;
    SPI_RegDef_t *pSPIx = pSPIHandle->pSPIx;
    uint8_t *pRxBuffer = pSPIHandle->pRxBuffer;
    uint16_t DFFStatus = (pSPIx->CR1 >> SPI_CR1_DFF) & (0x0001);

    if (pSPIHandle->RxLen != 0) {
        if (DFFStatus == 0) { // 8-bit DFF
            *pRxBuffer = pSPIx->DR;
            pRxBuffer++;
            pSPIHandle->RxLen -= 1;
        } else if (DFFStatus == 1) { // 16-bit DFF
            *((uint16_t*) pRxBuffer) = pSPIx->DR;
            (uint16_t *) pRxBuffer++;
            pSPIHandle->RxLen -= 2;
        }
    } else if (pSPIHandle->RxLen == 0) {
        // End RX Transmission and turn interrupts for RX off
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
        // Function to alert application that RX has completed
    }
}


/**
 * @brief Handles the OVR error. Reads from the specified registers in order to reset.
 * User application callback must implement and resolve data issues.
 * 
 * @param pSPIHandle 
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle) {

    // Clear the OVR Flag
    uint8_t tempReg = 0;
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX && pSPIHandle->RxState != SPI_BUSY_IN_RX) {
        tempReg = pSPIHandle->pSPIx->DR;
        tempReg = pSPIHandle->pSPIx->SR;
    }
    
    // Inform the application
    (void) tempReg; // To make the compiler happy
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/**
 * @brief Auxillary API to clear the OVR flag.
 * 
 * @param pSPIx 
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) {
    uint8_t tempReg;
    tempReg = pSPIx->DR;
    tempReg = pSPIx->SR;
    (void) tempReg;
}

/**
 * @brief Auxillary API to close the SPI Transmission.
 * 
 * @param pSPIHandle 
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuffer = NULL;
    pSPIHandle->TxLen = 0;
    pSPIHandle->TxState = SPI_READY;

}

/**
 * @brief Auxillary API to close the SPI Reception.
 * 
 * @param pSPIHandle 
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle) {
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}


/**
 * @brief Overrideable function used in applications to perform a response to an event
 * within the interrupt function.
 * Implemented by user-application.
 * 
 * @param pSPIHandle 
 * @param AppEv Application Event Macro
 * @return __weak 
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv) {

}