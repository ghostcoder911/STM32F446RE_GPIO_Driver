/**
 ******************************************************************************
 * @file           : stm32f446xx.h
 * @author         : Neeraj
 * @brief          : STM32F446RE Device Header File
 ******************************************************************************
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

#include <stdint.h>

/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR          0x08000000U
#define SRAM1_BASEADDR          0x20000000U
#define SRAM2_BASEADDR          0x2001C000U
#define ROM_BASEADDR            0x1FFF0000U
#define SRAM                    SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE             0x40000000U
#define APB1PERIPH_BASE         PERIPH_BASE
#define APB2PERIPH_BASE         0x40010000U
#define AHB1PERIPH_BASE         0x40020000U
#define AHB2PERIPH_BASE         0x50000000U

/*
 * Base addresses of peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR          (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASE + 0x1C00)

#define RCC_BASEADDR            (AHB1PERIPH_BASE + 0x3800)

/*
 * Peripheral register definition structures
 */

/*
 * GPIO Peripheral Register Definition
 */
typedef struct
{
    volatile uint32_t MODER;        /* GPIO port mode register,                   Address offset: 0x00 */
    volatile uint32_t OTYPER;       /* GPIO port output type register,            Address offset: 0x04 */
    volatile uint32_t OSPEEDR;      /* GPIO port output speed register,           Address offset: 0x08 */
    volatile uint32_t PUPDR;        /* GPIO port pull-up/pull-down register,      Address offset: 0x0C */
    volatile uint32_t IDR;          /* GPIO port input data register,             Address offset: 0x10 */
    volatile uint32_t ODR;          /* GPIO port output data register,            Address offset: 0x14 */
    volatile uint32_t BSRR;         /* GPIO port bit set/reset register,          Address offset: 0x18 */
    volatile uint32_t LCKR;         /* GPIO port configuration lock register,     Address offset: 0x1C */
    volatile uint32_t AFR[2];       /* AFR[0]: GPIO alternate function low register, AFR[1]: GPIO alternate function high register, Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

/*
 * RCC Peripheral Register Definition
 */
typedef struct
{
    volatile uint32_t CR;           /* RCC clock control register,                          Address offset: 0x00 */
    volatile uint32_t PLLCFGR;      /* RCC PLL configuration register,                      Address offset: 0x04 */
    volatile uint32_t CFGR;         /* RCC clock configuration register,                    Address offset: 0x08 */
    volatile uint32_t CIR;          /* RCC clock interrupt register,                        Address offset: 0x0C */
    volatile uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register,                  Address offset: 0x10 */
    volatile uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register,                  Address offset: 0x14 */
    volatile uint32_t AHB3RSTR;     /* RCC AHB3 peripheral reset register,                  Address offset: 0x18 */
    uint32_t RESERVED0;             /* Reserved, 0x1C */
    volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register,                  Address offset: 0x20 */
    volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register,                  Address offset: 0x24 */
    uint32_t RESERVED1[2];          /* Reserved, 0x28-0x2C */
    volatile uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register,           Address offset: 0x30 */
    volatile uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register,           Address offset: 0x34 */
    volatile uint32_t AHB3ENR;      /* RCC AHB3 peripheral clock enable register,           Address offset: 0x38 */
    uint32_t RESERVED2;             /* Reserved, 0x3C */
    volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register,           Address offset: 0x40 */
    volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register,           Address offset: 0x44 */
    uint32_t RESERVED3[2];          /* Reserved, 0x48-0x4C */
    volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
    volatile uint32_t AHB2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
    volatile uint32_t AHB3LPENR;    /* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
    uint32_t RESERVED4;             /* Reserved, 0x5C */
    volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
    volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enabled in low power mode register, Address offset: 0x64 */
    uint32_t RESERVED5[2];          /* Reserved, 0x68-0x6C */
    volatile uint32_t BDCR;         /* RCC Backup domain control register,                  Address offset: 0x70 */
    volatile uint32_t CSR;          /* RCC clock control & status register,                 Address offset: 0x74 */
    uint32_t RESERVED6[2];          /* Reserved, 0x78-0x7C */
    volatile uint32_t SSCGR;        /* RCC spread spectrum clock generation register,       Address offset: 0x80 */
    volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register,                   Address offset: 0x84 */
    volatile uint32_t PLLSAICFGR;   /* RCC PLLSAI configuration register,                   Address offset: 0x88 */
    volatile uint32_t DCKCFGR;      /* RCC Dedicated Clocks configuration register,         Address offset: 0x8C */
    volatile uint32_t CKGATENR;     /* RCC clocks gated enable register,                    Address offset: 0x90 */
    volatile uint32_t DCKCFGR2;     /* RCC Dedicated Clocks configuration register 2,       Address offset: 0x94 */
} RCC_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Generic Macros
 */
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET

#endif /* STM32F446XX_H_ */


