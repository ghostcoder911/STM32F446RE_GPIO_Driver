/**
 ******************************************************************************
 * @file           : stm32f446xx_gpio_driver.c
 * @author         : Neeraj
 * @brief          : GPIO Driver Implementation for STM32F446RE
 ******************************************************************************
 */

#include "stm32f446xx_gpio_driver.h"

/******************************************************************************************
 * @fn              - GPIO_PeriClockControl
 *
 * @brief           - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]       - Base address of the GPIO peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
        if(pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if(pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if(pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if(pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if(pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if(pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if(pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if(pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}

/******************************************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - This function initializes the GPIO pin
 *
 * @param[in]       - GPIO Handle containing base address and pin configuration
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    // Enable the peripheral clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    // 1. Configure the mode of GPIO pin
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // Non-interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
        pGPIOHandle->pGPIOx->MODER |= temp; // Set
    }
    else
    {
        // Interrupt mode - will be implemented later
        // For now, we'll just configure as input mode
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // Configure the FTSR (Falling Trigger Selection Register)
            // Will be implemented when EXTI driver is added
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // Configure the RTSR (Rising Trigger Selection Register)
            // Will be implemented when EXTI driver is added
        }
        else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // Configure both FTSR and RTSR
            // Will be implemented when EXTI driver is added
        }
    }

    temp = 0;

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    temp = 0;

    // 3. Configure the pull-up/pull-down settings
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clear
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    temp = 0;

    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    temp = 0;

    // 5. Configure the alternate functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1, temp2;

        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clear
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

/******************************************************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - This function de-initializes the GPIO port (resets all registers)
 *
 * @param[in]       - Base address of the GPIO peripheral
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if(pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if(pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if(pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if(pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if(pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if(pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if(pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if(pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
}

/******************************************************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - This function reads the value from a GPIO input pin
 *
 * @param[in]       - Base address of the GPIO peripheral
 * @param[in]       - Pin number
 *
 * @return          - 0 or 1
 *
 * @Note            - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

/******************************************************************************************
 * @fn              - GPIO_ReadFromInputPort
 *
 * @brief           - This function reads the value from a GPIO input port
 *
 * @param[in]       - Base address of the GPIO peripheral
 *
 * @return          - 16-bit port value
 *
 * @Note            - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

/******************************************************************************************
 * @fn              - GPIO_WriteToOutputPin
 *
 * @brief           - This function writes a value to a GPIO output pin
 *
 * @param[in]       - Base address of the GPIO peripheral
 * @param[in]       - Pin number
 * @param[in]       - Value to be written (SET or RESET)
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/******************************************************************************************
 * @fn              - GPIO_WriteToOutputPort
 *
 * @brief           - This function writes a value to a GPIO output port
 *
 * @param[in]       - Base address of the GPIO peripheral
 * @param[in]       - 16-bit value to be written to the port
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/******************************************************************************************
 * @fn              - GPIO_ToggleOutputPin
 *
 * @brief           - This function toggles a GPIO output pin
 *
 * @param[in]       - Base address of the GPIO peripheral
 * @param[in]       - Pin number
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/******************************************************************************************
 * @fn              - GPIO_IRQConfig
 *
 * @brief           - This function configures the interrupt
 *
 * @param[in]       - IRQ number
 * @param[in]       - IRQ priority
 * @param[in]       - ENABLE or DISABLE
 *
 * @return          - none
 *
 * @Note            - This will be implemented when we add NVIC configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{
    // Will be implemented later when we add NVIC driver
}

/******************************************************************************************
 * @fn              - GPIO_IRQHandling
 *
 * @brief           - This function handles the interrupt
 *
 * @param[in]       - Pin number
 *
 * @return          - none
 *
 * @Note            - This will be implemented when we add interrupt handling
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    // Will be implemented later when we add interrupt handling
}


