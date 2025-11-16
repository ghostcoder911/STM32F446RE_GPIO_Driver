# STM32F446xx GPIO Driver Development - Part 2
## Continuation: Driver Implementation and Complete Examples

**This is Part 2 of the technical reference. Read Part 1 first for architecture and register details.**

---

# Chapter 5 (Continued): GPIO Driver Implementation - Output Mode

## 5.3 GPIO Driver Header File - stm32f446xx_gpio_driver.h

**Purpose:** Define driver API and configuration structures

### Step 5.3.1: Header Guards and Includes

```c
/**
 ******************************************************************************
 * @file           : stm32f446xx_gpio_driver.h
 * @brief          : GPIO Driver Header for STM32F446xx
 * @reference      : RM0390
 ******************************************************************************
 */

#ifndef STM32F446XX_GPIO_DRIVER_H
#define STM32F446XX_GPIO_DRIVER_H

#include "stm32f446xx.h"

```

### Step 5.3.2: GPIO Pin Number Definitions

**Reference:** RM0390 Section 8 (GPIO has 16 pins per port)

```c
/**
 * @brief GPIO Pin Numbers
 * Each GPIO port has 16 pins numbered 0-15
 */
#define GPIO_PIN_0           0
#define GPIO_PIN_1           1
#define GPIO_PIN_2           2
#define GPIO_PIN_3           3
#define GPIO_PIN_4           4
#define GPIO_PIN_5           5      /* ← LED on Nucleo board (LD2) */
#define GPIO_PIN_6           6
#define GPIO_PIN_7           7
#define GPIO_PIN_8           8
#define GPIO_PIN_9           9
#define GPIO_PIN_10          10
#define GPIO_PIN_11          11
#define GPIO_PIN_12          12
#define GPIO_PIN_13          13     /* ← User button on Nucleo board */
#define GPIO_PIN_14          14
#define GPIO_PIN_15          15
```

### Step 5.3.3: GPIO Pin Mode Definitions

**Reference:** RM0390 Section 8.4.1 (MODER register)

```c
/**
 * @brief GPIO Pin Modes
 * Reference: RM0390 Section 8.4.1 (GPIOx_MODER)
 * 
 * These values are written to MODER register bits
 */
#define GPIO_MODE_INPUT      0      /* Input mode (reset state) */
#define GPIO_MODE_OUTPUT     1      /* General purpose output mode */
#define GPIO_MODE_ALTFN      2      /* Alternate function mode */
#define GPIO_MODE_ANALOG     3      /* Analog mode */

/**
 * @brief GPIO Interrupt Modes
 * These are custom values for interrupt configuration
 * (Not direct register values)
 */
#define GPIO_MODE_IT_FT      4      /* Input with interrupt, falling edge trigger */
#define GPIO_MODE_IT_RT      5      /* Input with interrupt, rising edge trigger */
#define GPIO_MODE_IT_RFT     6      /* Input with interrupt, rising/falling edge trigger */
```

### Step 5.3.4: GPIO Output Type Definitions

**Reference:** RM0390 Section 8.4.2 (OTYPER register)

```c
/**
 * @brief GPIO Output Types
 * Reference: RM0390 Section 8.4.2 (GPIOx_OTYPER)
 */
#define GPIO_OP_TYPE_PP      0      /* Push-pull (reset state) */
#define GPIO_OP_TYPE_OD      1      /* Open-drain */
```

### Step 5.3.5: GPIO Speed Definitions

**Reference:** RM0390 Section 8.4.3 (OSPEEDR register)

```c
/**
 * @brief GPIO Output Speed
 * Reference: RM0390 Section 8.4.3 (GPIOx_OSPEEDR)
 */
#define GPIO_SPEED_LOW       0      /* Low speed */
#define GPIO_SPEED_MEDIUM    1      /* Medium speed */
#define GPIO_SPEED_FAST      2      /* Fast speed (High speed) */
#define GPIO_SPEED_HIGH      3      /* High speed (Very high speed) */
```

### Step 5.3.6: GPIO Pull-up/Pull-down Definitions

**Reference:** RM0390 Section 8.4.4 (PUPDR register)

```c
/**
 * @brief GPIO Pull-up/Pull-down Configuration
 * Reference: RM0390 Section 8.4.4 (GPIOx_PUPDR)
 */
#define GPIO_NO_PUPD         0      /* No pull-up, no pull-down */
#define GPIO_PULL_UP         1      /* Pull-up */
#define GPIO_PULL_DOWN       2      /* Pull-down */
```

### Step 5.3.7: GPIO Configuration Structures

```c
/**
 * @brief GPIO Pin Configuration Structure
 * This structure contains all settings for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;         /*!< Pin number (0-15) from @GPIO_PIN_x */
    uint8_t GPIO_PinMode;           /*!< Pin mode from @GPIO_MODE_x */
    uint8_t GPIO_PinSpeed;          /*!< Output speed from @GPIO_SPEED_x */
    uint8_t GPIO_PinPuPdControl;    /*!< Pull-up/down from @GPIO_PUPD_x */
    uint8_t GPIO_PinOPType;         /*!< Output type from @GPIO_OP_TYPE_x */
    uint8_t GPIO_PinAltFunMode;     /*!< Alternate function (0-15), used if mode is ALTFN */
} GPIO_PinConfig_t;

/**
 * @brief GPIO Handle Structure
 * This structure contains the GPIO port and pin configuration
 */
typedef struct
{
    GPIO_TypeDef *pGPIOx;           /*!< Base address of GPIO port (GPIOA, GPIOB, etc.) */
    GPIO_PinConfig_t GPIO_PinConfig;/*!< GPIO pin configuration settings */
} GPIO_Handle_t;
```

### Step 5.3.8: Function Prototypes

```c
/**
 * @brief GPIO Driver API Functions
 * 
 * These functions provide a complete GPIO driver interface
 * All functions are documented in the .c file
 */

/*
 * Peripheral Clock Control
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_TypeDef *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* STM32F446XX_GPIO_DRIVER_H */
```

## 5.4 GPIO Driver Implementation File - stm32f446xx_gpio_driver.c

### Step 5.4.1: File Header and Includes

```c
/**
 ******************************************************************************
 * @file           : stm32f446xx_gpio_driver.c
 * @brief          : GPIO Driver Implementation for STM32F446xx
 * @reference      : RM0390
 ******************************************************************************
 */

#include "stm32f446xx_gpio_driver.h"
```

### Step 5.4.2: Peripheral Clock Control Function

**Reference:** RM0390 Section 6.3.12 (RCC_AHB1ENR)

```c
/**
 * @brief  Enables or disables peripheral clock for GPIO port
 * @param  pGPIOx: Pointer to GPIO port base address
 * @param  EnorDi: ENABLE or DISABLE macro
 * @retval None
 * 
 * @details This function controls the GPIO peripheral clock through RCC.
 *          Reference: RM0390 Section 6.3.12
 *          
 * Clock must be enabled before any GPIO configuration or operation.
 * 
 * Implementation:
 * - Sets/clears corresponding bit in RCC_AHB1ENR register
 * - Each GPIO port has dedicated bit (0-7 for GPIOA-GPIOH)
 */
void GPIO_PeriClockControl(GPIO_TypeDef *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        /* Enable clock by setting bit in RCC_AHB1ENR */
        if (pGPIOx == GPIOA)
        {
            RCC_GPIOA_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOB)
        {
            RCC_GPIOB_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOC)
        {
            RCC_GPIOC_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOD)
        {
            RCC_GPIOD_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOE)
        {
            RCC_GPIOE_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOF)
        {
            RCC_GPIOF_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOG)
        {
            RCC_GPIOG_CLK_ENABLE();
        }
        else if (pGPIOx == GPIOH)
        {
            RCC_GPIOH_CLK_ENABLE();
        }
    }
    else
    {
        /* Disable clock by clearing bit in RCC_AHB1ENR */
        if (pGPIOx == GPIOA)
        {
            RCC_GPIOA_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOB)
        {
            RCC_GPIOB_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOC)
        {
            RCC_GPIOC_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOD)
        {
            RCC_GPIOD_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOE)
        {
            RCC_GPIOE_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOF)
        {
            RCC_GPIOF_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOG)
        {
            RCC_GPIOG_CLK_DISABLE();
        }
        else if (pGPIOx == GPIOH)
        {
            RCC_GPIOH_CLK_DISABLE();
        }
    }
}

/*
 * Execution Flow:
 * 
 * 1. Function receives GPIO port pointer and enable/disable command
 * 2. Compares port pointer with known GPIO addresses
 * 3. Calls appropriate clock enable/disable macro
 * 4. Macro sets/clears bit in RCC_AHB1ENR register
 * 
 * Example for GPIOA:
 * GPIO_PeriClockControl(GPIOA, ENABLE);
 * → Calls RCC_GPIOA_CLK_ENABLE()
 * → Expands to: RCC->AHB1ENR |= (1U << 0)
 * → Sets bit 0 of register at 0x40023830
 * → GPIOA clock is now enabled
 */
```

### Step 5.4.3: GPIO Initialization Function

**Reference:** RM0390 Section 8.4 (GPIO registers)

```c
/**
 * @brief  Initializes GPIO pin according to specified parameters
 * @param  pGPIOHandle: Pointer to GPIO handle structure
 * @retval None
 * 
 * @details Configures all GPIO registers for the specified pin.
 *          Must be called after enabling peripheral clock.
 *          
 * Configured Registers:
 * - MODER: Pin mode (input/output/altfn/analog)
 * - OTYPER: Output type (push-pull/open-drain)
 * - OSPEEDR: Output speed
 * - PUPDR: Pull-up/pull-down
 * - AFR: Alternate function (if applicable)
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;  // Temporary variable for register manipulation
    
    /* STEP 1: Enable Peripheral Clock */
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
    
    /* STEP 2: Configure Pin Mode */
    /**
     * Reference: RM0390 Section 8.4.1 (GPIOx_MODER)
     * 
     * Each pin uses 2 bits in MODER register
     * Bit position = 2 * pin_number
     * 
     * Mode values:
     * 00 = Input
     * 01 = Output
     * 10 = Alternate Function
     * 11 = Analog
     */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* Non-interrupt mode */
        
        // Calculate bit position (each pin uses 2 bits)
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        
        // Clear the 2 bits for this pin
        // 0x3 = 0b11 (2 bits mask)
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        
        // Set the new mode
        pGPIOHandle->pGPIOx->MODER |= temp;
        
        /*
         * Example: Configure Pin 5 as Output (mode = 1)
         * 
         * Bit position = 2 * 5 = 10
         * temp = 1 << 10 = 0x00000400
         * 
         * Clear: MODER &= ~(0x3 << 10)
         *        MODER &= ~(0x00000C00)
         *        MODER &= 0xFFFFF3FF
         *        Clears bits [11:10]
         * 
         * Set: MODER |= 0x00000400
         *      Sets bits [11:10] to 01 (output mode)
         */
    }
    else
    {
        /* Interrupt mode - will be configured in Chapter 8 */
        /* This section handles GPIO_MODE_IT_FT, GPIO_MODE_IT_RT, GPIO_MODE_IT_RFT */
        /* Requires EXTI and SYSCFG configuration */
    }
    
    temp = 0;
    
    /* STEP 3: Configure Output Speed */
    /**
     * Reference: RM0390 Section 8.4.3 (GPIOx_OSPEEDR)
     * 
     * Each pin uses 2 bits
     * Speed values:
     * 00 = Low speed
     * 01 = Medium speed
     * 10 = Fast speed (High speed)
     * 11 = High speed (Very high speed)
     */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    // Clear bits
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    // Set speed
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    
    temp = 0;
    
    /* STEP 4: Configure Pull-up/Pull-down */
    /**
     * Reference: RM0390 Section 8.4.4 (GPIOx_PUPDR)
     * 
     * Each pin uses 2 bits
     * Values:
     * 00 = No pull-up, no pull-down
     * 01 = Pull-up
     * 10 = Pull-down
     * 11 = Reserved
     */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    // Clear bits
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    
    // Set pull-up/down
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    
    temp = 0;
    
    /* STEP 5: Configure Output Type */
    /**
     * Reference: RM0390 Section 8.4.2 (GPIOx_OTYPER)
     * 
     * Each pin uses 1 bit
     * Values:
     * 0 = Push-pull
     * 1 = Open-drain
     */
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    
    // Clear bit
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    
    // Set output type
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    
    temp = 0;
    
    /* STEP 6: Configure Alternate Function (if mode is ALTFN) */
    /**
     * Reference: RM0390 Section 8.4.9 and 8.4.10 (GPIOx_AFRL and GPIOx_AFRH)
     * 
     * AFR[0] = AFRL (pins 0-7)
     * AFR[1] = AFRH (pins 8-15)
     * 
     * Each pin uses 4 bits (alternate function 0-15)
     */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t temp1, temp2;
        
        // Determine which AFR register to use
        temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;   // 0 for pins 0-7, 1 for pins 8-15
        temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;   // Position within AFR register
        
        // Clear 4 bits
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
        
        // Set alternate function
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
        
        /*
         * Example: Configure Pin 7 as AF5
         * 
         * temp1 = 7 / 8 = 0 (use AFR[0] = AFRL)
         * temp2 = 7 % 8 = 7 (bit position 28-31)
         * 
         * Clear: AFR[0] &= ~(0xF << 28)
         * Set: AFR[0] |= (5 << 28)
         */
    }
}

/*
 * Complete Example: Initialize PA5 as Output for LED
 * 
 * GPIO_Handle_t led;
 * led.pGPIOx = GPIOA;
 * led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
 * led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
 * led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
 * led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
 * led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
 * 
 * GPIO_Init(&led);
 * 
 * Register State After Init:
 * RCC->AHB1ENR:     bit 0 = 1 (GPIOA clock enabled)
 * GPIOA->MODER:     bits [11:10] = 01 (output)
 * GPIOA->OTYPER:    bit 5 = 0 (push-pull)
 * GPIOA->OSPEEDR:   bits [11:10] = 10 (fast)
 * GPIOA->PUPDR:     bits [11:10] = 00 (no pull)
 */
```

### Step 5.4.4: GPIO De-initialization Function

**Reference:** RM0390 Section 6.3.7 (RCC_AHB1RSTR)

```c
/**
 * @brief  De-initializes GPIO port (resets to default state)
 * @param  pGPIOx: Pointer to GPIO port base address
 * @retval None
 * 
 * @details Resets the entire GPIO port to default state.
 *          All pin configurations are lost.
 *          Reference: RM0390 Section 6.3.7
 *          
 * Process:
 * 1. Set reset bit in RCC_AHB1RSTR
 * 2. Clear reset bit to release reset
 * 3. All port registers return to reset values
 */
void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        RCC_GPIOA_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        RCC_GPIOB_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        RCC_GPIOC_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        RCC_GPIOD_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        RCC_GPIOE_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        RCC_GPIOF_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        RCC_GPIOG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        RCC_GPIOH_RESET();
    }
}

/*
 * Usage: GPIO_DeInit(GPIOA);
 * 
 * Effect:
 * - GPIOA->MODER returns to 0xA8000000
 * - GPIOA->OTYPER returns to 0x00000000
 * - GPIOA->OSPEEDR returns to 0x00000000
 * - GPIOA->PUPDR returns to 0x64000000
 * - All other registers reset
 * 
 * Note: Clock remains enabled
 */
```

### Step 5.4.5: GPIO Write Functions

**Reference:** RM0390 Section 8.4.6 and 8.4.7 (ODR and BSRR)

```c
/**
 * @brief  Writes value to output pin
 * @param  pGPIOx: Pointer to GPIO port
 * @param  PinNumber: Pin number (0-15)
 * @param  Value: GPIO_PIN_SET or GPIO_PIN_RESET
 * @retval None
 * 
 * @details Sets or clears single output pin.
 *          Reference: RM0390 Section 8.4.6 (GPIOx_ODR)
 *          
 * Uses ODR (Output Data Register):
 * - Writing 1 sets pin HIGH (3.3V)
 * - Writing 0 sets pin LOW (0V)
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        /* Set the pin HIGH */
        pGPIOx->ODR |= (1 << PinNumber);
        
        /*
         * Example: Set Pin 5 HIGH
         * ODR |= (1 << 5)
         * ODR |= 0x00000020
         * 
         * Before: ODR = 0x00000000
         * After:  ODR = 0x00000020
         * Pin 5 is now HIGH (LED ON if connected)
         */
    }
    else
    {
        /* Set the pin LOW */
        pGPIOx->ODR &= ~(1 << PinNumber);
        
        /*
         * Example: Set Pin 5 LOW
         * ODR &= ~(1 << 5)
         * ODR &= ~0x00000020
         * ODR &= 0xFFFFFFDF
         * 
         * Before: ODR = 0x00000020
         * After:  ODR = 0x00000000
         * Pin 5 is now LOW (LED OFF if connected)
         */
    }
}

/**
 * @brief  Writes value to entire GPIO port
 * @param  pGPIOx: Pointer to GPIO port
 * @param  Value: 16-bit value for all pins
 * @retval None
 * 
 * @details Writes to all 16 pins simultaneously.
 *          Lower 16 bits of ODR are written.
 */
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
    
    /*
     * Example: Set pattern on GPIOA
     * GPIO_WriteToOutputPort(GPIOA, 0xAAAA);
     * 
     * Binary: 1010 1010 1010 1010
     * Pin 15: HIGH
     * Pin 14: LOW
     * Pin 13: HIGH
     * Pin 12: LOW
     * ... alternating pattern
     * Pin 0: LOW
     */
}

/**
 * @brief  Toggles output pin state
 * @param  pGPIOx: Pointer to GPIO port
 * @param  PinNumber: Pin number (0-15)
 * @retval None
 * 
 * @details Toggles pin state using XOR operation.
 *          If pin is LOW, makes it HIGH.
 *          If pin is HIGH, makes it LOW.
 *          
 * Uses XOR operation:
 * - 0 XOR 1 = 1 (LOW becomes HIGH)
 * - 1 XOR 1 = 0 (HIGH becomes LOW)
 */
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
    
    /*
     * Example: Toggle Pin 5
     * 
     * If ODR bit 5 = 0:
     *   ODR ^= (1 << 5)
     *   0 XOR 1 = 1
     *   LED turns ON
     * 
     * If ODR bit 5 = 1:
     *   ODR ^= (1 << 5)
     *   1 XOR 1 = 0
     *   LED turns OFF
     * 
     * This is perfect for blinking!
     */
}

/*
 * Alternative Implementation using BSRR (Better for concurrent access):
 * 
 * void GPIO_WriteToOutputPin_BSRR(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value)
 * {
 *     if (Value == GPIO_PIN_SET)
 *     {
 *         // Set pin using BSRR (atomic operation)
 *         pGPIOx->BSRR = (1 << PinNumber);
 *     }
 *     else
 *     {
 *         // Reset pin using BSRR (atomic operation)
 *         pGPIOx->BSRR = (1 << (PinNumber + 16));
 *     }
 * }
 * 
 * void GPIO_ToggleOutputPin_BSRR(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
 * {
 *     if (pGPIOx->ODR & (1 << PinNumber))
 *     {
 *         // Currently HIGH, set it LOW
 *         pGPIOx->BSRR = (1 << (PinNumber + 16));
 *     }
 *     else
 *     {
 *         // Currently LOW, set it HIGH
 *         pGPIOx->BSRR = (1 << PinNumber);
 *     }
 * }
 */
```

---

# Chapter 6: GPIO Driver Implementation - Input Mode

## 6.1 Input Mode Overview

**Reference:** RM0390 Section 8.3

GPIO pins can be configured as inputs to read digital signals from:
- Buttons/switches
- Sensors
- Communication interfaces
- Other digital devices

### Input Signal Characteristics

**Reference:** RM0390 Section 5.3.13 (I/O pin characteristics)

| Parameter | Min | Typical | Max | Unit |
|-----------|-----|---------|-----|------|
| Input LOW voltage (VIL) | -0.3 | - | 0.35×VDD | V |
| Input HIGH voltage (VIH) | 0.65×VDD | - | VDD+0.3 | V |
| Hysteresis | - | 200 | - | mV |
| Input leakage current | - | - | ±0.2 | μA |

For VDD = 3.3V:
- VIL max = 1.16V
- VIH min = 2.15V

## 6.2 GPIO Read Functions

**Reference:** RM0390 Section 8.4.5 (IDR register)

```c
/**
 * @brief  Reads value from input pin
 * @param  pGPIOx: Pointer to GPIO port
 * @param  PinNumber: Pin number (0-15)
 * @retval Pin state: 0 (LOW) or 1 (HIGH)
 * 
 * @details Reads single bit from IDR register.
 *          Reference: RM0390 Section 8.4.5 (GPIOx_IDR)
 *          
 * IDR Register:
 * - Read-only register
 * - Bit 0-15: Input data for pins 0-15
 * - Bit 16-31: Reserved (always 0)
 * 
 * Reading Process:
 * 1. Read entire IDR register (32 bits)
 * 2. Shift right by pin number
 * 3. Mask to get only LSB
 */
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    
    // Read IDR, shift to get pin bit, mask to get only that bit
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    
    return value;
    
    /*
     * Example: Read Pin 13 (user button on Nucleo)
     * 
     * Assume IDR = 0x00002000 (bit 13 set, others clear)
     * Binary: 00000000 00000000 00100000 00000000
     * 
     * Step 1: Shift right by 13
     * (0x00002000 >> 13) = 0x00000001
     * Binary: 00000000 00000000 00000000 00000001
     * 
     * Step 2: Mask with 0x1
     * 0x00000001 & 0x1 = 0x1
     * 
     * Return: 1 (button is pressed/released depending on circuit)
     */
}

/**
 * @brief  Reads entire GPIO port
 * @param  pGPIOx: Pointer to GPIO port
 * @retval 16-bit value representing all pin states
 * 
 * @details Reads all 16 pins simultaneously.
 *          Each bit represents one pin state.
 */
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx)
{
    uint16_t value;
    
    // Read lower 16 bits of IDR
    value = (uint16_t)pGPIOx->IDR;
    
    return value;
    
    /*
     * Example: Read entire GPIOC
     * 
     * If GPIOC->IDR = 0x00002001
     * Binary: 00000000 00000000 00100000 00000001
     * 
     * Return: 0x2001
     * Interpretation:
     * - Pin 0: HIGH
     * - Pins 1-12: LOW
     * - Pin 13: HIGH
     * - Pins 14-15: LOW
     */
}
```

## 6.3 Button Input Example

### Hardware Setup on Nucleo F446RE

**Reference:** Nucleo-64 User Manual (UM1724)

The user button (B1) on Nucleo board:
- Connected to: PC13
- Configuration: Active LOW (pressed = 0, released = 1)
- Internal: Pull-up resistor (external on board)

### Button Input Configuration

```c
/**
 * @brief Configure PC13 for button input
 * 
 * Button Schematic:
 *      3.3V
 *        │
 *     [10kΩ]  ← External pull-up on board
 *        │
 *        ├───── PC13
 *        │
 *     [Button]
 *        │
 *       GND
 * 
 * When button not pressed: PC13 = HIGH (1)
 * When button pressed: PC13 = LOW (0)
 */

GPIO_Handle_t button;

// Configure port
button.pGPIOx = GPIOC;

// Configure pin
button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Not critical for input
button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // External pull-up present
button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;   // Don't care for input

// Initialize
GPIO_Init(&button);

/*
 * Register State After Init:
 * GPIOC->MODER: bits [27:26] = 00 (input mode)
 * GPIOC->PUPDR: bits [27:26] = 00 (no internal pull-up/down)
 */
```

### Reading Button State

```c
/**
 * @brief Button polling example
 */
void button_polling_example(void)
{
    // Read button state
    uint8_t button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13);
    
    if (button_state == 0)
    {
        // Button is pressed (active LOW)
        // Do something (turn LED on, etc.)
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    }
    else
    {
        // Button is not pressed
        // Do something else (turn LED off, etc.)
        GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

/**
 * @brief Button with debouncing
 * 
 * Debouncing: Mechanical switches bounce (make/break contact multiple times)
 * Solution: Add delay after detecting state change
 */
uint8_t button_read_debounced(void)
{
    static uint8_t button_state = 1;  // Previous state (not pressed)
    uint8_t current_state;
    
    // Read current state
    current_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13);
    
    // Check if state changed
    if (current_state != button_state)
    {
        // Wait for bounce to settle (typically 10-50ms)
        delay_ms(20);
        
        // Read again
        current_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13);
        
        // Update state if still different
        if (current_state != button_state)
        {
            button_state = current_state;
        }
    }
    
    return button_state;
}
```

## 6.4 Complete Input Example: Button-Controlled LED

```c
/**
 * @brief Complete example: Button controls LED
 * 
 * Hardware:
 * - Button: PC13 (active LOW)
 * - LED: PA5 (active HIGH)
 * 
 * Behavior:
 * - Button pressed → LED ON
 * - Button released → LED OFF
 */

#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

int main(void)
{
    GPIO_Handle_t led, button;
    
    /* Configure LED (PA5) as output */
    led.pGPIOx = GPIOA;
    led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    
    GPIO_Init(&led);
    
    /* Configure button (PC13) as input */
    button.pGPIOx = GPIOC;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
    button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // External pull-up
    
    GPIO_Init(&button);
    
    /* Main loop */
    while (1)
    {
        // Read button (active LOW)
        if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_13) == 0)
        {
            // Button pressed → LED ON
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
        else
        {
            // Button released → LED OFF
            GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    }
    
    return 0;
}

/*
 * Execution Flow:
 * 
 * 1. Initialize LED output and button input
 * 2. Loop continuously:
 *    a. Read PC13 (button) from IDR register
 *    b. If 0 (pressed): Write 1 to PA5 (LED ON)
 *    c. If 1 (released): Write 0 to PA5 (LED OFF)
 * 
 * Register Operations Per Loop:
 * - Read: GPIOC->IDR (address 0x40020810)
 * - Write: GPIOA->ODR (address 0x40020014)
 */
```

---

# Chapter 7: GPIO Alternate Functions

## 7.1 Alternate Function Overview

**Reference:** RM0390 Section 8.3

GPIO pins have multiple functions:
- Default: General Purpose I/O
- Alternate: Special peripheral functions (UART, SPI, I2C, etc.)

Each pin can be configured for up to 16 alternate functions (AF0-AF15).

## 7.2 Alternate Function Table

**Reference:** RM0390 Table 12 (Alternate function mapping)

### Common Alternate Functions for PA5 (Our LED Pin)

| AF Number | Function | Description |
|-----------|----------|-------------|
| AF0 | - | Not used |
| AF1 | TIM2_CH1 | Timer 2 Channel 1 |
| AF2 | TIM8_CH1N | Timer 8 Channel 1 complementary |
| AF3 | - | Not used |
| AF4 | - | Not used |
| AF5 | SPI1_SCK | SPI1 Clock |
| AF6 | - | Not used |
| AF7 | - | Not used |
| AF8 | - | Not used |

### Example: Common USART2 Pins

| Pin | Function | AF Number |
|-----|----------|-----------|
| PA2 | USART2_TX | AF7 |
| PA3 | USART2_RX | AF7 |

## 7.3 Configuring Alternate Functions

**Reference:** RM0390 Section 8.4.9 and 8.4.10 (AFR registers)

### AFR Register Structure

```
GPIO has two AFR registers:
- AFR[0] = AFRL (Alternate Function Low Register) - Pins 0-7
- AFR[1] = AFRH (Alternate Function High Register) - Pins 8-15

Each pin uses 4 bits (can select AF0-AF15)

AFRL (AFR[0]) Layout:
Bit:  31-28  27-24  23-20  19-16  15-12  11-8   7-4    3-0
      Pin7   Pin6   Pin5   Pin4   Pin3   Pin2   Pin1   Pin0

AFRH (AFR[1]) Layout:
Bit:  31-28  27-24  23-20  19-16  15-12  11-8   7-4    3-0
      Pin15  Pin14  Pin13  Pin12  Pin11  Pin10  Pin9   Pin8
```

### Configuration Example: USART2

```c
/**
 * @brief Configure PA2 as USART2_TX (AF7)
 * 
 * PA2 Alternate Function 7 = USART2_TX
 * Reference: RM0390 Table 12
 */

GPIO_Handle_t usart_pin;

usart_pin.pGPIOx = GPIOA;
usart_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
usart_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;  // ← Alternate function mode
usart_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
usart_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
usart_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;
usart_pin.GPIO_PinConfig.GPIO_PinAltFunMode = 7;  // ← AF7 for USART2_TX

GPIO_Init(&usart_pin);

/*
 * Register Operations:
 * 
 * 1. MODER: bits [5:4] = 10 (alternate function)
 * 2. AFR[0]: bits [11:8] = 0111 (AF7)
 * 
 * Pin 2 uses AFRL (AFR[0])
 * Bit position = 4 * 2 = 8
 * 
 * GPIOA->AFR[0] &= ~(0xF << 8);  // Clear bits [11:8]
 * GPIOA->AFR[0] |= (7 << 8);     // Set to AF7
 * 
 * Result: PA2 is now controlled by USART2 peripheral
 */
```

## 7.4 Complete SPI Example

```c
/**
 * @brief Configure SPI1 pins
 * 
 * SPI1 Pins:
 * - PA5: SPI1_SCK (AF5) - Serial Clock
 * - PA6: SPI1_MISO (AF5) - Master In Slave Out
 * - PA7: SPI1_MOSI (AF5) - Master Out Slave In
 * 
 * Reference: RM0390 Table 12
 */

void SPI1_GPIO_Init(void)
{
    GPIO_Handle_t spi_pins;
    
    // Common configuration for all SPI pins
    spi_pins.pGPIOx = GPIOA;
    spi_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    spi_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;  // AF5 for SPI1
    spi_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    spi_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    spi_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    
    // Configure SCK (PA5)
    spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&spi_pins);
    
    // Configure MISO (PA6)
    spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
    GPIO_Init(&spi_pins);
    
    // Configure MOSI (PA7)
    spi_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&spi_pins);
}

/*
 * After initialization:
 * - PA5, PA6, PA7 are controlled by SPI1 peripheral
 * - Cannot use these pins for regular GPIO operations
 * - SPI1 peripheral controls pin states during communication
 */
```

---

# Chapter 8: GPIO Interrupts and EXTI

## 8.1 External Interrupt Overview

**Reference:** RM0390 Section 12 (EXTI)

STM32F446xx has 23 external interrupt/event lines (EXTI0-EXTI22).
- EXTI0-EXTI15: Connected to GPIO pins
- EXTI16-EXTI22: Connected to internal sources

### GPIO Interrupt Architecture

```
GPIO Pin → EXTI Line → NVIC → CPU

Example: Button on PC13
PC13 → EXTI13 → NVIC IRQ40 → EXTI15_10_IRQHandler()
```

## 8.2 EXTI Configuration Registers

**Reference:** RM0390 Section 12.3

### Key EXTI Registers

| Register | Address | Purpose |
|----------|---------|---------|
| EXTI_IMR | 0x40013C00 | Interrupt mask register |
| EXTI_RTSR | 0x40013C08 | Rising trigger selection |
| EXTI_FTSR | 0x40013C0C | Falling trigger selection |
| EXTI_PR | 0x40013C14 | Pending register |

### SYSCFG_EXTICR Registers

**Reference:** RM0390 Section 9.2.3

Before using EXTI, must configure SYSCFG to select which GPIO port connects to EXTI line.

```
SYSCFG_EXTICR[0]: EXTI0-EXTI3
SYSCFG_EXTICR[1]: EXTI4-EXTI7
SYSCFG_EXTICR[2]: EXTI8-EXTI11
SYSCFG_EXTICR[3]: EXTI12-EXTI15

Example: Configure EXTI13 for GPIOC (PC13)
SYSCFG_EXTICR[3] bits [7:4] = 0010 (port C)
```

## 8.3 IRQ Numbers

**Reference:** RM0390 Table 62 (Vector table)

| EXTI Line | IRQ Number | Handler Function |
|-----------|------------|------------------|
| EXTI0 | 6 | EXTI0_IRQHandler |
| EXTI1 | 7 | EXTI1_IRQHandler |
| EXTI2 | 8 | EXTI2_IRQHandler |
| EXTI3 | 9 | EXTI3_IRQHandler |
| EXTI4 | 10 | EXTI4_IRQHandler |
| EXTI5-9 | 23 | EXTI9_5_IRQHandler |
| EXTI10-15 | 40 | EXTI15_10_IRQHandler |

## 8.4 GPIO Interrupt Functions Implementation

This implementation is complex and requires additional registers. For completeness, here's a skeleton:

```c
/**
 * @brief Configure GPIO interrupt
 * @param IRQNumber: IRQ number from vector table
 * @param EnorDi: ENABLE or DISABLE
 * @retval None
 * 
 * @details Enables/disables interrupt in NVIC
 *          Reference: ARM Cortex-M4 Generic User Guide
 *          
 * NVIC Registers:
 * - NVIC_ISER0-NVIC_ISER2: Interrupt set-enable registers
 * - NVIC_ICER0-NVIC_ICER2: Interrupt clear-enable registers
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            // ISER0
            *((volatile uint32_t*)0xE000E100) |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            // ISER1
            *((volatile uint32_t*)0xE000E104) |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            // ISER2
            *((volatile uint32_t*)0xE000E108) |= (1 << (IRQNumber % 32));
        }
    }
    else
    {
        // Similar for ICER registers (clear-enable)
    }
}

/**
 * @brief Configure interrupt priority
 * @param IRQNumber: IRQ number
 * @param IRQPriority: Priority value (0-15)
 * @retval None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    // NVIC Priority registers (IPR)
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    
    uint8_t shift_amount = (8 * iprx_section) + (8 - 4);  // Only upper 4 bits used
    
    *(((volatile uint32_t*)0xE000E400) + iprx) |= (IRQPriority << shift_amount);
}

/**
 * @brief Handle GPIO interrupt
 * @param PinNumber: Pin that triggered interrupt
 * @retval None
 * 
 * @details Must be called from IRQ handler.
 *          Clears pending bit in EXTI_PR register.
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    // Clear EXTI pending bit
    if (*((volatile uint32_t*)0x40013C14) & (1 << PinNumber))
    {
        // Clear by writing 1
        *((volatile uint32_t*)0x40013C14) |= (1 << PinNumber);
    }
}
```

---

# Chapter 9: Complete LED Blinking Application

## 9.1 Project Structure

```
HelloWorld/
├── Inc/
│   ├── stm32f446xx.h
│   └── stm32f446xx_gpio_driver.h
├── Src/
│   ├── main.c
│   ├── stm32f446xx_gpio_driver.c
│   └── syscalls.c
└── Startup/
    └── startup_stm32f446retx.s
```

## 9.2 Complete main.c Implementation

```c
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : LED Blinking Application
 * @hardware       : STM32 Nucleo-F446RE
 * @led            : LD2 connected to PA5
 * @reference      : RM0390
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

/**
 * @brief Simple delay function
 * @param None
 * @retval None
 * 
 * @details Blocking delay using empty loop.
 *          At 16 MHz HSI clock:
 *          - Each iteration ≈ 4-5 cycles
 *          - 500,000 iterations ≈ 500ms
 *          
 * Note: Not accurate, depends on compiler optimization.
 *       For production, use SysTick or hardware timer.
 */
void delay(void)
{
    for (uint32_t i = 0; i < 500000; i++);
}

/**
 * @brief Main function
 * @retval Never returns
 * 
 * @details Configures PA5 as output and blinks LED continuously.
 *          
 * Hardware Setup:
 * - Board: STM32 Nucleo-F446RE
 * - LED: LD2 (green LED on PA5)
 * - LED Type: Active HIGH (PA5=1 → LED ON)
 * 
 * Execution Flow:
 * 1. Create GPIO configuration structure
 * 2. Configure PA5 settings
 * 3. Initialize GPIO (writes to registers)
 * 4. Infinite loop: toggle LED and delay
 */
int main(void)
{
    /* Print startup message (appears in SWV ITM console) */
    printf("LED Blinking Application Started\n");
    printf("Target: STM32F446RE Nucleo Board\n");
    printf("LED: PA5 (LD2)\n\n");
    
    /* Create GPIO handle structure */
    GPIO_Handle_t GpioLed;
    
    /* Configure GPIO settings for LED */
    GpioLed.pGPIOx = GPIOA;  // Use Port A
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;  // Pin 5
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;  // Output mode
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;  // Fast speed (not critical for LED)
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;  // Push-pull output
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;  // No pull-up/down (output)
    
    /* Initialize GPIO - This writes to hardware registers */
    GPIO_Init(&GpioLed);
    
    printf("GPIO Initialized Successfully\n");
    printf("Starting LED blink...\n\n");
    
    /* Infinite loop */
    while (1)
    {
        /* Toggle LED state */
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
        
        /* Wait ~500ms */
        delay();
        
        /*
         * What happens each iteration:
         * 
         * 1. GPIO_ToggleOutputPin():
         *    - Reads GPIOA->ODR register
         *    - XORs bit 5
         *    - Writes back to GPIOA->ODR
         *    - LED changes state
         * 
         * 2. delay():
         *    - CPU executes empty loop 500,000 times
         *    - Takes approximately 500ms
         * 
         * 3. Loop repeats
         * 
         * Result: LED blinks at ~1 Hz (500ms ON, 500ms OFF)
         */
    }
    
    return 0;  // Never reached
}

/*
 * Complete Register State During Execution:
 * 
 * After GPIO_Init():
 * ==================
 * RCC->AHB1ENR = 0x00000001
 *   - Bit 0 = 1: GPIOA clock enabled
 * 
 * GPIOA->MODER = 0xA8000400
 *   - Bits [11:10] = 01: Pin 5 output mode
 *   - Other bits: Default or debug configuration
 * 
 * GPIOA->OTYPER = 0x00000000
 *   - Bit 5 = 0: Push-pull output
 * 
 * GPIOA->OSPEEDR = 0x00000800
 *   - Bits [11:10] = 10: Fast speed
 * 
 * GPIOA->PUPDR = 0x64000000
 *   - Bits [11:10] = 00: No pull-up/down
 * 
 * During Execution:
 * =================
 * GPIOA->ODR toggles between:
 *   - 0x00000020 (bit 5 = 1, LED ON)
 *   - 0x00000000 (bit 5 = 0, LED OFF)
 * 
 * Memory Accesses Per Blink Cycle:
 * =================================
 * 1. Read GPIOA->ODR (0x40020014)
 * 2. XOR with 0x00000020
 * 3. Write GPIOA->ODR (0x40020014)
 * 4. Execute 500,000 empty loops
 * 5. Repeat
 */
```

## 9.3 Build and Debug Process

### Compile Command (Example for ARM GCC)

```bash
# Compile driver
arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -O0 -g3 \
  -I./Inc \
  -o stm32f446xx_gpio_driver.o \
  ./Src/stm32f446xx_gpio_driver.c

# Compile main
arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -O0 -g3 \
  -I./Inc \
  -o main.o \
  ./Src/main.c

# Link
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O0 -g3 \
  -T STM32F446RETX_FLASH.ld \
  -o HelloWorld.elf \
  main.o stm32f446xx_gpio_driver.o syscalls.o startup.o
```

### Debug Verification Points

1. **After GPIO_Init():**
   - Set breakpoint after `GPIO_Init(&GpioLed);`
   - Check RCC->AHB1ENR bit 0 = 1
   - Check GPIOA->MODER bits [11:10] = 01
   - Check GPIOA->OSPEEDR bits [11:10] = 10

2. **During Toggle:**
   - Set breakpoint in `GPIO_ToggleOutputPin()`
   - Watch GPIOA->ODR register
   - Should toggle between 0x00000020 and 0x00000000

3. **Hardware Verification:**
   - LED should blink at ~1 Hz
   - Measure PA5 with oscilloscope: square wave, ~1 Hz
   - Voltage: 0V to 3.3V transitions

---

# Chapter 10: Advanced Topics and Best Practices

## 10.1 Performance Optimization

### Using BSRR Instead of ODR

**Better Performance:**

```c
// Instead of:
GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);  // Read-modify-write

// Use:
if (GPIOA->ODR & (1 << 5))
{
    GPIOA->BSRR = (1 << (5 + 16));  // Atomic reset
}
else
{
    GPIOA->BSRR = (1 << 5);  // Atomic set
}
```

**Benefits:**
- Atomic operation (no read-modify-write)
- Safe in interrupt context
- Slightly faster execution

### Register Access Optimization

```c
// Cache pointer for repeated access
GPIO_TypeDef *port = GPIOA;
uint8_t pin = GPIO_PIN_5;

// Multiple operations on same port
port->BSRR = (1 << pin);         // Set
delay();
port->BSRR = (1 << (pin + 16));  // Reset
```

## 10.2 Power Optimization

### Minimize Enabled Peripherals

```c
// Only enable clocks for used peripherals
RCC_GPIOA_CLK_ENABLE();  // Need GPIOA
// Don't enable unused ports (GPIOB, C, D, etc.)
```

### Use Appropriate GPIO Speed

```c
// For LED (low frequency), use low speed
led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;  // Saves power
```

### Disable Unused Pins

```c
// Configure unused pins as analog to minimize power
GPIO_Handle_t unused_pin;
unused_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
```

## 10.3 Error Handling

### Parameter Validation

```c
/**
 * @brief Safe GPIO write with parameter checking
 */
void GPIO_WriteToOutputPin_Safe(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    // Validate parameters
    if (pGPIOx == NULL) return;
    if (PinNumber > 15) return;
    if (Value > 1) return;
    
    // Validate port address
    if (pGPIOx != GPIOA && pGPIOx != GPIOB && pGPIOx != GPIOC &&
        pGPIOx != GPIOD && pGPIOx != GPIOE && pGPIOx != GPIOF &&
        pGPIOx != GPIOG && pGPIOx != GPIOH)
    {
        return;  // Invalid port
    }
    
    // Perform operation
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}
```

## 10.4 Timing Considerations

### Accurate Delays Using SysTick

```c
/**
 * @brief Initialize SysTick for 1ms interrupts
 * @param clock_freq: System clock frequency in Hz
 */
void SysTick_Init(uint32_t clock_freq)
{
    // SysTick Reload Value Register
    SysTick->LOAD = (clock_freq / 1000) - 1;  // 1ms period
    
    // Clear current value
    SysTick->VAL = 0;
    
    // Enable SysTick with processor clock
    SysTick->CTRL = 0x7;  // Enable, interrupt enable, processor clock
}

volatile uint32_t msTicks = 0;

void SysTick_Handler(void)
{
    msTicks++;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}
```

## 10.5 Multi-tasking Considerations

### Critical Sections

```c
/**
 * @brief Atomic GPIO toggle
 */
void GPIO_Toggle_Atomic(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    // Disable interrupts
    __disable_irq();
    
    // Critical section
    pGPIOx->ODR ^= (1 << PinNumber);
    
    // Re-enable interrupts
    __enable_irq();
}
```

---

# Appendix A: Register Quick Reference

## GPIO Registers (Base + Offset)

| Register | Offset | Description | Access |
|----------|--------|-------------|--------|
| MODER | 0x00 | Port mode register | R/W |
| OTYPER | 0x04 | Port output type register | R/W |
| OSPEEDR | 0x08 | Port output speed register | R/W |
| PUPDR | 0x0C | Port pull-up/pull-down register | R/W |
| IDR | 0x10 | Port input data register | R |
| ODR | 0x14 | Port output data register | R/W |
| BSRR | 0x18 | Port bit set/reset register | W |
| LCKR | 0x1C | Port configuration lock register | R/W |
| AFRL | 0x20 | Alternate function low register | R/W |
| AFRH | 0x24 | Alternate function high register | R/W |

## RCC Registers (Relevant for GPIO)

| Register | Address | Bit(s) | Function |
|----------|---------|--------|----------|
| AHB1ENR | 0x40023830 | 0 | GPIOA clock enable |
| AHB1ENR | 0x40023830 | 1 | GPIOB clock enable |
| AHB1ENR | 0x40023830 | 2 | GPIOC clock enable |
| AHB1ENR | 0x40023830 | 3-7 | GPIOD-H clock enable |
| AHB1RSTR | 0x40023810 | 0-7 | GPIO reset bits |

## Common Values

| Parameter | Value | Description |
|-----------|-------|-------------|
| GPIO_MODE_INPUT | 0 | Input mode |
| GPIO_MODE_OUTPUT | 1 | Output mode |
| GPIO_MODE_ALTFN | 2 | Alternate function |
| GPIO_MODE_ANALOG | 3 | Analog mode |
| GPIO_OP_TYPE_PP | 0 | Push-pull |
| GPIO_OP_TYPE_OD | 1 | Open-drain |
| GPIO_SPEED_LOW | 0 | Low speed |
| GPIO_SPEED_MEDIUM | 1 | Medium speed |
| GPIO_SPEED_FAST | 2 | Fast speed |
| GPIO_SPEED_HIGH | 3 | High speed |

---

# Appendix B: Code Templates

## Template 1: Basic Output Pin

```c
GPIO_Handle_t output_pin;

output_pin.pGPIOx = GPIOx;  // Replace x with A,B,C...
output_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_x;  // Replace x with 0-15
output_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
output_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
output_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
output_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

GPIO_Init(&output_pin);
```

## Template 2: Basic Input Pin

```c
GPIO_Handle_t input_pin;

input_pin.pGPIOx = GPIOx;  // Replace x with A,B,C...
input_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_x;  // Replace x with 0-15
input_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
input_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
input_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PULL_UP;  // or GPIO_PULL_DOWN

GPIO_Init(&input_pin);
```

## Template 3: Alternate Function Pin

```c
GPIO_Handle_t altfn_pin;

altfn_pin.pGPIOx = GPIOx;  // Replace x with A,B,C...
altfn_pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_x;  // Replace x with 0-15
altfn_pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
altfn_pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
altfn_pin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
altfn_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
altfn_pin.GPIO_PinConfig.GPIO_PinAltFunMode = AFx;  // Replace x with 0-15

GPIO_Init(&altfn_pin);
```

---

# Appendix C: Troubleshooting Guide

## LED Not Blinking

### Check 1: Clock Enabled?
```c
// Add after GPIO_Init() and check in debugger
if (RCC->AHB1ENR & (1 << 0))
{
    printf("GPIOA clock is enabled\n");
}
else
{
    printf("ERROR: GPIOA clock NOT enabled!\n");
}
```

### Check 2: Pin Configured as Output?
```c
// Check MODER register
uint32_t moder_value = (GPIOA->MODER >> 10) & 0x3;
if (moder_value == 1)
{
    printf("PA5 is output mode\n");
}
else
{
    printf("ERROR: PA5 not in output mode! Value: %lu\n", moder_value);
}
```

### Check 3: ODR Toggling?
```c
// In main loop, before delay
printf("ODR = 0x%08lX\n", GPIOA->ODR);
// Should alternate between having bit 5 set/clear
```

## Compilation Errors

### "undefined reference to GPIO_Init"
**Cause:** Driver .c file not added to build  
**Solution:** Add `stm32f446xx_gpio_driver.c` to project sources

### "stm32f446xx.h: No such file"
**Cause:** Include path not set  
**Solution:** Add `Inc/` folder to include paths

## Runtime Issues

### Hard Fault
**Cause:** Usually null pointer or invalid address  
**Debug:** Enable fault handlers and check:
- Fault Status Registers
- Program Counter at fault
- Stack trace

### Nothing Works
**Check:**
1. Power connected?
2. ST-Link connected?
3. Correct target selected?
4. Flash programmed successfully?
5. Debugger shows code running?

---

**End of Technical Reference Guide**

For questions or clarifications, refer to:
- RM0390 Reference Manual
- STM32F446RE Datasheet
- ARM Cortex-M4 Technical Reference Manual
- Nucleo-64 User Manual (UM1724)

