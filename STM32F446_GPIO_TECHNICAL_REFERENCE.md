# STM32F446xx GPIO Driver Development
## Complete Technical Reference Guide with RM0390 Citations

**Document Version:** 1.0  
**Target Device:** STM32F446RET6  
**Reference Manual:** RM0390 Rev 8  
**Author:** Technical Documentation  
**Date:** November 14, 2025  

---

## Document Organization

This comprehensive technical guide is organized into the following chapters:

- **[Chapter 1: STM32F446xx Architecture Overview](#chapter-1-stm32f446xx-architecture-overview)**
- **[Chapter 2: Memory Organization and Addressing](#chapter-2-memory-organization-and-addressing)**
- **[Chapter 3: Clock Control and RCC](#chapter-3-clock-control-and-rcc)**
- **[Chapter 4: GPIO Register Reference](#chapter-4-gpio-register-reference)**
- **[Chapter 5: GPIO Driver Implementation - Output Mode](#chapter-5-gpio-driver-implementation---output-mode)**
- **[Chapter 6: GPIO Driver Implementation - Input Mode](#chapter-6-gpio-driver-implementation---input-mode)**
- **[Chapter 7: GPIO Alternate Functions](#chapter-7-gpio-alternate-functions)**
- **[Chapter 8: GPIO Interrupts and EXTI](#chapter-8-gpio-interrupts-and-exti)**
- **[Chapter 9: Complete LED Blinking Application](#chapter-9-complete-led-blinking-application)**
- **[Chapter 10: Advanced Topics and Best Practices](#chapter-10-advanced-topics-and-best-practices)**
- **[Appendix A: Register Quick Reference](#appendix-a-register-quick-reference)**
- **[Appendix B: Code Templates](#appendix-b-code-templates)**
- **[Appendix C: Troubleshooting Guide](#appendix-c-troubleshooting-guide)**

---

# Chapter 1: STM32F446xx Architecture Overview

## 1.1 Device Overview

**Reference:** RM0390 Section 2.1

The STM32F446xx is a high-performance ARM Cortex-M4 32-bit RISC core operating at up to 180 MHz frequency. The device incorporates:

- **Core:** ARM Cortex-M4 with FPU
- **Flash Memory:** Up to 512 KB
- **SRAM:** 128 KB (split into SRAM1 and SRAM2)
- **GPIO Ports:** 8 ports (GPIOA-GPIOH) with up to 16 pins each
- **Operating Voltage:** 1.7V to 3.6V

### Device Features Summary

| Feature | Specification |
|---------|--------------|
| CPU Frequency | Up to 180 MHz |
| Flash Memory | 512 KB |
| SRAM1 | 112 KB |
| SRAM2 | 16 KB |
| GPIO Pins | Up to 82 I/O ports |
| Operating Temp | -40°C to +85°C (105°C junction) |

## 1.2 System Architecture

**Reference:** RM0390 Section 2.2

The STM32F446xx architecture is based on a multi-AHB bus matrix that provides:

```
┌─────────────────────────────────────────────────────────────┐
│                   ARM Cortex-M4 Core                        │
│              (with FPU and Memory Protection Unit)           │
└────────┬────────────────────────────────────┬───────────────┘
         │                                    │
    ┌────┴────┐                          ┌────┴────┐
    │  ICode  │                          │  DCode  │
    │   Bus   │                          │   Bus   │
    └────┬────┘                          └────┬────┘
         │                                    │
         └──────────┬─────────────────────────┘
                    │
         ┌──────────┴──────────┐
         │  AHB Bus Matrix     │
         └──────────┬──────────┘
                    │
      ┌─────────────┼─────────────┐
      │             │             │
┌─────┴─────┐ ┌────┴────┐  ┌────┴────┐
│ AHB1 Bus  │ │ AHB2 Bus│  │ AHB3 Bus│
└─────┬─────┘ └────┬────┘  └─────────┘
      │            │
      │            └──── GPIOB-GPIOH
      │
      └──── GPIOA, RCC, DMA, etc.
```

### 1.2.1 Bus Architecture Details

**AHB1 Bus (Advanced High-performance Bus 1)**
- Base Address: 0x40020000
- Contains: GPIO ports, RCC, DMA, CRC
- Maximum Speed: 180 MHz

**AHB2 Bus**
- Base Address: 0x50000000
- Contains: USB OTG FS, Camera interface
- Maximum Speed: 180 MHz

**APB1 Bus (Advanced Peripheral Bus 1)**
- Base Address: 0x40000000
- Contains: Timers, USART, I2C, SPI
- Maximum Speed: 45 MHz (APB1)

**APB2 Bus**
- Base Address: 0x40010000
- Contains: SYSCFG, EXTI, ADC, SPI1, USART1
- Maximum Speed: 90 MHz (APB2)

## 1.3 Clock System Overview

**Reference:** RM0390 Section 6

The STM32F446xx has a flexible clock system with multiple clock sources:

```
Clock Tree (Simplified for GPIO):

    ┌──────────────┐
    │  HSI (16MHz) │──────┐
    │   (Internal) │      │
    └──────────────┘      │
                          ├──→ System Clock Mux
    ┌──────────────┐      │
    │  HSE (8MHz)  │──────┤      ↓
    │   (External) │      │   SYSCLK
    └──────────────┘      │      │
                          │      ├──→ AHB Prescaler
    ┌──────────────┐      │      │         │
    │  PLL (Main)  │──────┘      │         ↓
    │   (180MHz)   │             │     HCLK (AHB Clock)
    └──────────────┘             │         │
                                 │         ├──→ AHB1 Peripheral Clock
                                 │         │         │
                                 │         │         ├──→ GPIOA
                                 │         │         ├──→ GPIOB
                                 │         │         ├──→ GPIOC
                                 │         │         ├──→ ...
                                 │         │         └──→ RCC
                                 │         │
                                 │         └──→ APB1/APB2 Prescaler
                                 │
                                 └──→ APB Peripheral Clocks
```

### Default Clock Configuration

After reset, the system uses:
- **Clock Source:** HSI (High-Speed Internal) = 16 MHz
- **SYSCLK:** 16 MHz
- **HCLK:** 16 MHz
- **All peripheral clocks:** DISABLED (must be enabled manually)

---

# Chapter 2: Memory Organization and Addressing

## 2.1 Complete Memory Map

**Reference:** RM0390 Section 2.3

The STM32F446xx has a 4 GB address space organized as follows:

```
Memory Map (32-bit Address Space):

0xFFFFFFFF ┌─────────────────────────────────────┐
           │  Vendor-specific Memory             │
0xE0100000 ├─────────────────────────────────────┤
           │  Cortex-M4 Internal Peripherals    │
           │  - NVIC, SCB, SysTick, ITM, etc.   │
0xE0000000 ├─────────────────────────────────────┤
           │  External Device                     │
0xA0000000 ├─────────────────────────────────────┤
           │  External RAM                        │
0x60000000 ├─────────────────────────────────────┤
           │  Peripherals                         │
           │  ┌──────────────────────┐           │
0x50000000 │  │  AHB2 Peripherals    │           │
           │  └──────────────────────┘           │
0x40020000 │  ┌──────────────────────┐           │
           │  │  AHB1 Peripherals    │◄────┐     │
           │  │  - GPIOA: 0x40020000 │     │     │
           │  │  - GPIOB: 0x40020400 │     │     │
           │  │  - GPIOC: 0x40020800 │     │     │
           │  │  - GPIOD: 0x40020C00 │     │     │
           │  │  - GPIOE: 0x40021000 │     │     │
           │  │  - GPIOF: 0x40021400 │     │     │
           │  │  - GPIOG: 0x40021800 │     │     │
           │  │  - GPIOH: 0x40021C00 │     │     │
           │  │  - RCC:   0x40023800 │     │ Focus Area
           │  └──────────────────────┘     │     │
0x40010000 │  ┌──────────────────────┐     │     │
           │  │  APB2 Peripherals    │     │     │
           │  │  - SYSCFG, EXTI, etc.│◄────┘     │
           │  └──────────────────────┘           │
0x40000000 │  ┌──────────────────────┐           │
           │  │  APB1 Peripherals    │           │
           │  └──────────────────────┘           │
           └─────────────────────────────────────┘
0x20000000 ┌─────────────────────────────────────┐
           │  SRAM (128 KB)                      │
           │  - SRAM1: 0x20000000 (112 KB)      │
           │  - SRAM2: 0x2001C000 (16 KB)       │
0x1FFF0000 ├─────────────────────────────────────┤
           │  System Memory (Boot ROM)           │
0x08000000 ├─────────────────────────────────────┤
           │  Flash Memory (512 KB)              │
0x00000000 └─────────────────────────────────────┘
```

## 2.2 GPIO Port Address Calculation

**Reference:** RM0390 Section 2.3, Table 1

Each GPIO port occupies 1024 bytes (0x400) in the address space:

### GPIO Base Addresses

```c
#define GPIOA_BASE    0x40020000
#define GPIOB_BASE    0x40020400  // GPIOA_BASE + 0x400
#define GPIOC_BASE    0x40020800  // GPIOA_BASE + 0x800
#define GPIOD_BASE    0x40020C00  // GPIOA_BASE + 0xC00
#define GPIOE_BASE    0x40021000  // GPIOA_BASE + 0x1000
#define GPIOF_BASE    0x40021400  // GPIOA_BASE + 0x1400
#define GPIOG_BASE    0x40021800  // GPIOA_BASE + 0x1800
#define GPIOH_BASE    0x40021C00  // GPIOA_BASE + 0x1C00
```

### Address Offset Pattern

```
Formula: GPIOx_BASE = GPIOA_BASE + (port_number × 0x400)

Where port_number:
  A = 0, B = 1, C = 2, D = 3, E = 4, F = 5, G = 6, H = 7

Example: GPIOC address
  = 0x40020000 + (2 × 0x400)
  = 0x40020000 + 0x800
  = 0x40020800
```

## 2.3 GPIO Register Memory Layout

**Reference:** RM0390 Section 8.4

Within each GPIO port's 1024-byte space, registers are organized as follows:

```
GPIO Port Memory Organization (Example: GPIOA at 0x40020000):

Offset    Register   Size    Description
────────────────────────────────────────────────────────────
0x00      MODER      32-bit  Port mode register
0x04      OTYPER     32-bit  Port output type register
0x08      OSPEEDR    32-bit  Port output speed register
0x0C      PUPDR      32-bit  Port pull-up/pull-down register
0x10      IDR        32-bit  Port input data register
0x14      ODR        32-bit  Port output data register
0x18      BSRR       32-bit  Port bit set/reset register
0x1C      LCKR       32-bit  Port configuration lock register
0x20      AFRL       32-bit  Alternate function low register
0x24      AFRH       32-bit  Alternate function high register
0x28-0x3FF Reserved  -       Reserved space
```

### Register Address Calculation Example

```c
// To access GPIOA ODR (Output Data Register):
// Base: 0x40020000
// Offset: 0x14
// Full Address: 0x40020014

#define GPIOA_ODR    (*((volatile uint32_t*)0x40020014))

// Or using structure:
typedef struct {
    volatile uint32_t MODER;    // 0x00
    volatile uint32_t OTYPER;   // 0x04
    volatile uint32_t OSPEEDR;  // 0x08
    volatile uint32_t PUPDR;    // 0x0C
    volatile uint32_t IDR;      // 0x10
    volatile uint32_t ODR;      // 0x14
    // ...
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef*)0x40020000)

// Access: GPIOA->ODR
```

## 2.4 RCC Register Memory Layout

**Reference:** RM0390 Section 6.3

The Reset and Clock Control (RCC) peripheral is located at 0x40023800:

```
RCC Memory Organization (Base: 0x40023800):

Offset    Register     Description
───────────────────────────────────────────────────────────
0x00      CR          Clock control register
0x04      PLLCFGR     PLL configuration register
0x08      CFGR        Clock configuration register
0x0C      CIR         Clock interrupt register
0x10      AHB1RSTR    AHB1 peripheral reset register
0x14      AHB2RSTR    AHB2 peripheral reset register
0x18      AHB3RSTR    AHB3 peripheral reset register
0x1C      Reserved    -
0x20      APB1RSTR    APB1 peripheral reset register
0x24      APB2RSTR    APB2 peripheral reset register
0x28-2C   Reserved    -
0x30      AHB1ENR     AHB1 peripheral clock enable ◄── Key for GPIO
0x34      AHB2ENR     AHB2 peripheral clock enable
0x38      AHB3ENR     AHB3 peripheral clock enable
0x3C      Reserved    -
0x40      APB1ENR     APB1 peripheral clock enable
0x44      APB2ENR     APB2 peripheral clock enable ◄── Key for SYSCFG/EXTI
... (more registers)
```

---

# Chapter 3: Clock Control and RCC

## 3.1 Understanding Peripheral Clocks

**Reference:** RM0390 Section 6.3.12

### Why Clock Enable is Required

Every peripheral in the STM32F446xx is connected to a clock gate. By default, all peripheral clocks are **DISABLED** after reset to save power. Before using any peripheral, its clock MUST be enabled.

```
Power Saving Mechanism:

    RCC Clock Control
          │
          ├──► [Gate]──► GPIOA (DISABLED by default)
          ├──► [Gate]──► GPIOB (DISABLED by default)
          ├──► [Gate]──► GPIOC (DISABLED by default)
          └──► [Gate]──► ...

After Enabling GPIOA Clock:
    
    RCC Clock Control
          │
          ├──► [OPEN]──► GPIOA (Clock flowing ✓)
          ├──► [Gate]──► GPIOB (Still disabled)
          └──► [Gate]──► GPIOC (Still disabled)
```

### Power Consumption Impact

| State | Power Impact |
|-------|--------------|
| All GPIOs enabled | ~1-2 mA |
| Only required GPIO enabled | Minimal |
| Unused peripherals disabled | Maximum power savings |

## 3.2 RCC_AHB1ENR Register

**Reference:** RM0390 Section 6.3.12

### Register Description

- **Name:** RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
- **Address:** 0x40023830
- **Reset Value:** 0x00100000
- **Access:** Read/Write

### Bit Field Diagram

```
RCC_AHB1ENR (Address: 0x40023830)

Bit:  31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
      [Reserved                                              ][OTGHSULPISMEN]
      
Bit:  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      [--][--][--][--][--][--][--][DMA2EN][DMA1EN][CRCEN][--][GPIOH][GPIOG][GPIOF]
                                                      [GPIOE][GPIOD][GPIOC][GPIOB][GPIOAEN]
                                                                                        ↑
                                                                    Focus: Bits 0-7 (GPIO clocks)
```

### Key Bits for GPIO (Bits 0-7)

| Bit | Name | Description | Reset Value |
|-----|------|-------------|-------------|
| 7 | GPIOHEN | GPIOH clock enable | 0 |
| 6 | GPIOGEN | GPIOG clock enable | 0 |
| 5 | GPIOFEN | GPIOF clock enable | 0 |
| 4 | GPIOEEN | GPIOE clock enable | 0 |
| 3 | GPIODEN | GPIOD clock enable | 0 |
| 2 | GPIOCEN | GPIOC clock enable | 0 |
| 1 | GPIOBEN | GPIOB clock enable | 0 |
| 0 | GPIOAEN | GPIOA clock enable | 0 |

**Bit Behavior:**
- 0: Clock disabled (default)
- 1: Clock enabled

## 3.3 Enabling GPIO Clocks - Code Implementation

### Step 3.3.1: Direct Register Access

```c
// Enable GPIOA clock by setting bit 0
*((volatile uint32_t*)0x40023830) |= (1 << 0);

// Enable GPIOB clock by setting bit 1
*((volatile uint32_t*)0x40023830) |= (1 << 1);

// Enable GPIOC clock by setting bit 2
*((volatile uint32_t*)0x40023830) |= (1 << 2);
```

**Analysis:**
- Address 0x40023830 is RCC_AHB1ENR
- `|=` operator: OR-equals, sets bit without affecting others
- `(1 << 0)` creates bitmask: 0b00000001

### Step 3.3.2: Using Defined Macros (Better Approach)

```c
// Define RCC base address
#define RCC_BASE      0x40023800UL

// Define register structure
typedef struct {
    volatile uint32_t CR;           // Offset 0x00
    volatile uint32_t PLLCFGR;      // Offset 0x04
    volatile uint32_t CFGR;         // Offset 0x08
    volatile uint32_t CIR;          // Offset 0x0C
    volatile uint32_t AHB1RSTR;     // Offset 0x10
    volatile uint32_t AHB2RSTR;     // Offset 0x14
    volatile uint32_t AHB3RSTR;     // Offset 0x18
    uint32_t RESERVED0;             // Offset 0x1C
    volatile uint32_t APB1RSTR;     // Offset 0x20
    volatile uint32_t APB2RSTR;     // Offset 0x24
    uint32_t RESERVED1[2];          // Offset 0x28-0x2C
    volatile uint32_t AHB1ENR;      // Offset 0x30  ◄── This is what we need
    // ... more registers
} RCC_TypeDef;

// Create pointer to RCC
#define RCC ((RCC_TypeDef*)RCC_BASE)

// Enable clocks using structure
RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA
RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB
RCC->AHB1ENR |= (1 << 2);  // Enable GPIOC
```

### Step 3.3.3: Create Helper Macros (Best Practice)

```c
// Clock Enable Macros
#define RCC_GPIOA_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 0))
#define RCC_GPIOB_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 1))
#define RCC_GPIOC_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 2))
#define RCC_GPIOD_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 3))
#define RCC_GPIOE_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 4))
#define RCC_GPIOF_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 5))
#define RCC_GPIOG_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 6))
#define RCC_GPIOH_CLK_ENABLE()    (RCC->AHB1ENR |= (1 << 7))

// Clock Disable Macros
#define RCC_GPIOA_CLK_DISABLE()   (RCC->AHB1ENR &= ~(1 << 0))
#define RCC_GPIOB_CLK_DISABLE()   (RCC->AHB1ENR &= ~(1 << 1))
// ... similar for other ports

// Usage in code
int main(void) {
    RCC_GPIOA_CLK_ENABLE();  // Enable GPIOA clock
    // Now GPIOA is ready to use
}
```

### Step 3.3.4: Function-Based Approach (Driver Implementation)

```c
/**
 * @brief  Enables or disables peripheral clock for GPIOx
 * @param  GPIOx: Pointer to GPIO port (GPIOA, GPIOB, etc.)
 * @param  State: ENABLE or DISABLE
 * @retval None
 */
void GPIO_ClockControl(GPIO_TypeDef *GPIOx, uint8_t State)
{
    if (State == ENABLE)
    {
        if (GPIOx == GPIOA)
            RCC->AHB1ENR |= (1 << 0);
        else if (GPIOx == GPIOB)
            RCC->AHB1ENR |= (1 << 1);
        else if (GPIOx == GPIOC)
            RCC->AHB1ENR |= (1 << 2);
        else if (GPIOx == GPIOD)
            RCC->AHB1ENR |= (1 << 3);
        else if (GPIOx == GPIOE)
            RCC->AHB1ENR |= (1 << 4);
        else if (GPIOx == GPIOF)
            RCC->AHB1ENR |= (1 << 5);
        else if (GPIOx == GPIOG)
            RCC->AHB1ENR |= (1 << 6);
        else if (GPIOx == GPIOH)
            RCC->AHB1ENR |= (1 << 7);
    }
    else
    {
        if (GPIOx == GPIOA)
            RCC->AHB1ENR &= ~(1 << 0);
        // ... similar for disable
    }
}

// Usage
GPIO_ClockControl(GPIOA, ENABLE);   // Enable GPIOA clock
GPIO_ClockControl(GPIOB, DISABLE);  // Disable GPIOB clock
```

## 3.4 RCC_AHB1RSTR Register (Reset Control)

**Reference:** RM0390 Section 6.3.7

### Register Description

- **Name:** RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
- **Address:** 0x40023810
- **Reset Value:** 0x00000000
- **Access:** Read/Write

### Purpose

Used to reset peripherals to their default state. Useful for:
- Clearing configuration errors
- Reinitializing peripherals
- Handling error conditions

### Bit Fields for GPIO

| Bit | Name | Description |
|-----|------|-------------|
| 7 | GPIOHRST | GPIOH reset |
| 6 | GPIOGRST | GPIOG reset |
| 5 | GPIOFRST | GPIOF reset |
| 4 | GPIOERST | GPIOE reset |
| 3 | GPIODRST | GPIOD reset |
| 2 | GPIOCRST | GPIOC reset |
| 1 | GPIOBRST | GPIOB reset |
| 0 | GPIOARST | GPIOA reset |

### Reset Procedure

**Reference:** RM0390 Section 6.3.7

To properly reset a peripheral:
1. Set the corresponding bit to 1 (enable reset)
2. Clear the bit to 0 (disable reset)

```c
// Reset GPIOA
RCC->AHB1RSTR |= (1 << 0);   // Enable reset
RCC->AHB1RSTR &= ~(1 << 0);  // Disable reset

// GPIOA is now in default state
```

### Reset Macro Implementation

```c
// GPIO Reset Macros
#define RCC_GPIOA_FORCE_RESET()    (RCC->AHB1RSTR |= (1 << 0))
#define RCC_GPIOA_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(1 << 0))

// Combined reset macro using do-while(0) pattern
#define RCC_GPIOA_RESET()  do { \
    RCC->AHB1RSTR |= (1 << 0);  \
    RCC->AHB1RSTR &= ~(1 << 0); \
} while(0)

// Usage
RCC_GPIOA_RESET();  // Resets GPIOA to default state
```

**Why `do { ... } while(0)` pattern?**
- Ensures the macro behaves like a single statement
- Allows use with if/else without braces
- Common embedded C idiom

---

# Chapter 4: GPIO Register Reference

## 4.1 GPIO Register Overview

**Reference:** RM0390 Section 8.4

Each GPIO port has 10 control registers. For LED blinking, we primarily use:
1. **MODER** - Configure pin as input/output
2. **OTYPER** - Configure output type
3. **OSPEEDR** - Configure output speed
4. **PUPDR** - Configure pull-up/pull-down
5. **ODR** - Write output data
6. **BSRR** - Atomic bit set/reset (alternative to ODR)

## 4.2 GPIO Port Mode Register (GPIOx_MODER)

**Reference:** RM0390 Section 8.4.1

### Register Details

- **Name:** GPIO port mode register (GPIOx_MODER)
- **Offset:** 0x00
- **Reset Value:** 
  - GPIOA: 0xA8000000 (PA15,14,13 configured by debug)
  - GPIOB: 0x00000280 (PB4,3 configured by debug)
  - Other ports: 0x00000000
- **Access:** Read/Write

### Bit Field Diagram

```
GPIOx_MODER (32-bit register)

Each pin uses 2 bits:
Pin 15        Pin 14        Pin 13        ...        Pin 1         Pin 0
[31:30]       [29:28]       [27:26]       ...        [3:2]         [1:0]
MODER15       MODER14       MODER13       ...        MODER1        MODER0

Bit values:
00 = Input mode (reset state for most pins)
01 = General purpose output mode
10 = Alternate function mode
11 = Analog mode
```

### Pin to Bit Mapping

| Pin | Bits | Calculation |
|-----|------|-------------|
| Pin 0 | [1:0] | 2 × 0 = 0 |
| Pin 1 | [3:2] | 2 × 1 = 2 |
| Pin 2 | [5:4] | 2 × 2 = 4 |
| Pin 3 | [7:6] | 2 × 3 = 6 |
| Pin 4 | [9:8] | 2 × 4 = 8 |
| Pin 5 | [11:10] | 2 × 5 = 10 |
| ... | ... | ... |
| Pin 15 | [31:30] | 2 × 15 = 30 |

### Mode Configuration Examples

#### Example 1: Configure PA5 as Output

```c
// PA5 uses bits [11:10]
// We want: 01 (output mode)

// Step 1: Clear bits [11:10]
GPIOA->MODER &= ~(0x3 << 10);  // 0x3 = 0b11

// Step 2: Set bits [11:10] to 01
GPIOA->MODER |= (0x1 << 10);   // 0x1 = 0b01

// Result: Bits [11:10] = 01 = Output mode
```

**Detailed Bit Manipulation:**

```
Initial MODER value (example): 0xA8000000
Binary: 10101000 00000000 00000000 00000000

Step 1: Clear bits [11:10]
Mask:   00000000 00000000 00001100 00000000 = (0x3 << 10)
~Mask:  11111111 11111111 11110011 11111111 = ~(0x3 << 10)
Result: 10101000 00000000 00000000 00000000 (bits 11:10 now 00)

Step 2: Set to 01
Value:  00000000 00000000 00000100 00000000 = (0x1 << 10)
Result: 10101000 00000000 00000100 00000000
                           ^^
                      Bits [11:10] = 01
```

#### Example 2: Configure Multiple Pins

```c
// Configure PA5 and PA6 as outputs

// Clear bits for both pins
GPIOA->MODER &= ~((0x3 << 10) | (0x3 << 12));

// Set both as outputs (01)
GPIOA->MODER |= ((0x1 << 10) | (0x1 << 12));
```

### Function Implementation

```c
/**
 * @brief  Configures GPIO pin mode
 * @param  GPIOx: GPIO port pointer
 * @param  PinNumber: Pin number (0-15)
 * @param  Mode: Pin mode (INPUT, OUTPUT, ALTERNATE, ANALOG)
 * @retval None
 */
void GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint8_t PinNumber, uint8_t Mode)
{
    // Clear the 2 bits for this pin
    GPIOx->MODER &= ~(0x3 << (PinNumber * 2));
    
    // Set the new mode
    GPIOx->MODER |= (Mode << (PinNumber * 2));
}

// Usage
GPIO_SetPinMode(GPIOA, 5, GPIO_MODE_OUTPUT);  // PA5 as output
```

## 4.3 GPIO Port Output Type Register (GPIOx_OTYPER)

**Reference:** RM0390 Section 8.4.2

### Register Details

- **Name:** GPIO port output type register (GPIOx_OTYPER)
- **Offset:** 0x04
- **Reset Value:** 0x00000000
- **Access:** Read/Write

### Bit Field Diagram

```
GPIOx_OTYPER (32-bit register)

Only lower 16 bits are used (1 bit per pin):
Bit:  31-16  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      [Res]  [OT15][OT14][OT13]...[OT8][OT7][OT6][OT5][OT4][OT3][OT2][OT1][OT0]

Bit values:
0 = Push-pull (reset state)
1 = Open-drain
```

### Output Type Comparison

#### Push-Pull Mode (0)

**Reference:** RM0390 Section 8.3.11

```
Push-Pull Configuration:

             VDD (3.3V)
                │
            ┌───┴───┐
            │ P-MOS │  ◄── Controlled by output bit
            └───┬───┘
                │
                ├──────► GPIO Pin
                │
            ┌───┴───┐
            │ N-MOS │  ◄── Controlled by output bit
            └───┬───┘
                │
               GND

When Output = 1:
  - P-MOS ON, N-MOS OFF
  - Pin driven to VDD (3.3V)

When Output = 0:
  - P-MOS OFF, N-MOS ON
  - Pin driven to GND (0V)

Characteristics:
✓ Can source and sink current
✓ Output voltage levels: 0V or 3.3V
✓ Used for LEDs, standard digital outputs
```

#### Open-Drain Mode (1)

```
Open-Drain Configuration:

             VDD (3.3V)
                │
         External Pull-up
          Resistor (required)
                │
                ├──────► GPIO Pin
                │
            ┌───┴───┐
            │ N-MOS │  ◄── Controlled by output bit
            └───┬───┘
                │
               GND

When Output = 0:
  - N-MOS ON
  - Pin pulled to GND (0V)

When Output = 1:
  - N-MOS OFF
  - Pin floating (pulled high by external resistor)

Characteristics:
✓ Can only sink current (not source)
✓ Requires external pull-up resistor
✓ Used for: I2C, wired-AND configurations
✗ Not suitable for driving LEDs without external circuit
```

### Configuration Example

```c
// Configure PA5 as push-pull output (for LED)
GPIOA->OTYPER &= ~(1 << 5);  // Clear bit 5 = push-pull

// Configure PA9 as open-drain (for I2C)
GPIOA->OTYPER |= (1 << 9);   // Set bit 9 = open-drain
```

### When to Use Each Type

| Application | Type | Reason |
|-------------|------|--------|
| LED driving | Push-Pull | Can source current directly |
| I2C communication | Open-Drain | Multiple devices on same line |
| SPI communication | Push-Pull | Higher speed, clean signals |
| Button input | N/A | Input mode doesn't use OTYPER |
| General digital I/O | Push-Pull | Most common, easiest to use |

## 4.4 GPIO Port Output Speed Register (GPIOx_OSPEEDR)

**Reference:** RM0390 Section 8.4.3

### Register Details

- **Name:** GPIO port output speed register (GPIOx_OSPEEDR)
- **Offset:** 0x08
- **Reset Value:** 0x00000000 (Low speed for all pins)
- **Access:** Read/Write

### Bit Field Diagram

```
GPIOx_OSPEEDR (32-bit register)

Each pin uses 2 bits:
Pin 15        Pin 14        Pin 13        ...        Pin 1         Pin 0
[31:30]       [29:28]       [27:26]       ...        [3:2]         [1:0]
OSPEEDR15     OSPEEDR14     OSPEEDR13     ...        OSPEEDR1      OSPEEDR0

Bit values:
00 = Low speed
01 = Medium speed
10 = Fast speed (previously High speed)
11 = High speed (previously Very high speed)
```

### Speed Specifications

**Reference:** RM0390 Table 52 (I/O port characteristics)

| Setting | Max Frequency | Rise/Fall Time | Use Case |
|---------|---------------|----------------|----------|
| 00 (Low) | 8 MHz | ~25 ns | LEDs, slow peripherals |
| 01 (Medium) | 50 MHz | ~10 ns | Standard communication |
| 10 (Fast) | 100 MHz | ~5 ns | Fast communication (SPI, etc.) |
| 11 (High) | 180 MHz | ~2.5 ns | Very high-speed interfaces |

### Speed vs. Power Consumption

```
Power Consumption Relationship:

High Speed (11) ████████████  (Highest current)
                ↑
Fast Speed (10) ██████████
                ↑
Medium (01)     ██████
                ↑
Low Speed (00)  ███          (Lowest current)

Trade-off:
- Higher speed = Faster transitions = More current
- Lower speed = Slower transitions = Less current
```

**Best Practice:** Use the lowest speed that meets your requirements to minimize:
- Power consumption
- Electromagnetic interference (EMI)
- Signal ringing/overshoot

### Configuration Example

```c
// Configure PA5 for LED (Low/Medium speed is sufficient)
// Clear bits [11:10]
GPIOA->OSPEEDR &= ~(0x3 << 10);
// Set to Medium speed (01)
GPIOA->OSPEEDR |= (0x1 << 10);

// For high-speed SPI on PA7 (Fast speed required)
// Clear bits [15:14]
GPIOA->OSPEEDR &= ~(0x3 << 14);
// Set to Fast speed (10)
GPIOA->OSPEEDR |= (0x2 << 14);
```

### For LED Blinking Application

```c
// For LED blinking, Low or Medium speed is recommended
// LED switching frequency << 1 MHz

// Option 1: Low speed (saves most power)
GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));  // Clear bits
GPIOA->OSPEEDR |= (0x0 << (5 * 2));   // Set to 00 (Low)

// Option 2: Medium speed (good balance)
GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));  // Clear bits
GPIOA->OSPEEDR |= (0x1 << (5 * 2));   // Set to 01 (Medium)
```

## 4.5 GPIO Port Pull-up/Pull-down Register (GPIOx_PUPDR)

**Reference:** RM0390 Section 8.4.4

### Register Details

- **Name:** GPIO port pull-up/pull-down register (GPIOx_PUPDR)
- **Offset:** 0x0C
- **Reset Value:** 0x64000000 (Pull-up on debug pins)
- **Access:** Read/Write

### Bit Field Diagram

```
GPIOx_PUPDR (32-bit register)

Each pin uses 2 bits:
Pin 15        Pin 14        Pin 13        ...        Pin 1         Pin 0
[31:30]       [29:28]       [27:26]       ...        [3:2]         [1:0]
PUPDR15       PUPDR14       PUPDR13       ...        PUPDR1        PUPDR0

Bit values:
00 = No pull-up, no pull-down
01 = Pull-up
10 = Pull-down
11 = Reserved
```

### Pull-up/Pull-down Concept

#### No Pull-up/Pull-down (00)

```
No Internal Resistor:

    GPIO Pin ───────┤  Floating when input
                    │
                  Nothing

Use when:
- Pin configured as output
- External pull-up/down present
- Analog mode
```

#### Pull-up Mode (01)

```
Internal Pull-up (~40kΩ):

         VDD (3.3V)
            │
         [40kΩ]  ◄── Internal pull-up resistor
            │
    GPIO Pin ───────┤
            │
         (Input)

Effect:
- Pin reads HIGH (1) when not driven
- When button connects pin to GND, reads LOW (0)

Use for:
- Active-low buttons
- Open-drain inputs
- I2C (supplementing external pull-ups)
```

#### Pull-down Mode (10)

```
Internal Pull-down (~40kΩ):

    GPIO Pin ───────┤
            │
         [40kΩ]  ◄── Internal pull-down resistor
            │
           GND

Effect:
- Pin reads LOW (0) when not driven
- When button connects pin to VDD, reads HIGH (1)

Use for:
- Active-high buttons
- Preventing floating inputs
```

### Typical Resistor Values

**Reference:** RM0390 Table 52

- Internal pull-up resistor: ~40 kΩ (typical)
- Internal pull-down resistor: ~40 kΩ (typical)
- Tolerance: ±30%
- Current when pulled: ~82 μA (3.3V / 40kΩ)

### Configuration Examples

#### For Output (LED on PA5)

```c
// For output pins, pull-up/pull-down not needed
// Set to no pull-up, no pull-down (00)
GPIOA->PUPDR &= ~(0x3 << 10);  // Clear bits [11:10]
// No need to set anything (00 is default)
```

#### For Input with Button (Active-Low)

```c
// Button connected between pin and GND
// Need pull-up to read HIGH when button not pressed

// Configure PC13 (user button on Nucleo)
// Clear bits [27:26]
GPIOC->PUPDR &= ~(0x3 << 26);
// Set to pull-up (01)
GPIOC->PUPDR |= (0x1 << 26);

// Now:
// Button not pressed → Pin reads 1 (pulled high)
// Button pressed → Pin reads 0 (connected to GND)
```

#### For Input with Button (Active-High)

```c
// Button connected between pin and VDD
// Need pull-down to read LOW when button not pressed

// Configure PA0
// Clear bits [1:0]
GPIOA->PUPDR &= ~(0x3 << 0);
// Set to pull-down (10)
GPIOA->PUPDR |= (0x2 << 0);

// Now:
// Button not pressed → Pin reads 0 (pulled low)
// Button pressed → Pin reads 1 (connected to VDD)
```

## 4.6 GPIO Port Input Data Register (GPIOx_IDR)

**Reference:** RM0390 Section 8.4.5

### Register Details

- **Name:** GPIO port input data register (GPIOx_IDR)
- **Offset:** 0x10
- **Reset Value:** 0x00000000
- **Access:** Read-only

### Bit Field Diagram

```
GPIOx_IDR (32-bit register)

Bits 31-16: Reserved (always read as 0)
Bits 15-0: Input data for pins 15-0

Bit:  31-16  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      [Res]  [IDR15][IDR14]...[IDR8][IDR7][IDR6][IDR5][IDR4][IDR3][IDR2][IDR1][IDR0]

Each bit:
0 = Input is LOW (≈0V)
1 = Input is HIGH (≈3.3V)
```

### Reading Input

```c
// Read single pin (PA0)
uint8_t pin_state = (GPIOA->IDR >> 0) & 0x1;

// Read pin 5
uint8_t pin5_state = (GPIOA->IDR >> 5) & 0x1;

// Read entire port (all 16 pins at once)
uint16_t port_state = (uint16_t)(GPIOA->IDR & 0xFFFF);
```

### Example: Button Reading

```c
// User button on PC13 (Nucleo board)
// Active LOW (pressed = 0, not pressed = 1)

// Read button state
if ((GPIOC->IDR & (1 << 13)) == 0) {
    // Button is pressed
} else {
    // Button is not pressed
}

// Alternative using bit shift
if (((GPIOC->IDR >> 13) & 0x1) == 0) {
    // Button is pressed
}
```

## 4.7 GPIO Port Output Data Register (GPIOx_ODR)

**Reference:** RM0390 Section 8.4.6

### Register Details

- **Name:** GPIO port output data register (GPIOx_ODR)
- **Offset:** 0x14
- **Reset Value:** 0x00000000
- **Access:** Read/Write

### Bit Field Diagram

```
GPIOx_ODR (32-bit register)

Bits 31-16: Reserved
Bits 15-0: Output data for pins 15-0

Bit:  31-16  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      [Res]  [ODR15][ODR14]...[ODR8][ODR7][ODR6][ODR5][ODR4][ODR3][ODR2][ODR1][ODR0]

Each bit:
0 = Output LOW (0V)
1 = Output HIGH (3.3V)
```

### Output Control Methods

#### Method 1: Direct Bit Set

```c
// Turn LED ON (PA5 = 1)
GPIOA->ODR |= (1 << 5);

// Turn LED OFF (PA5 = 0)
GPIOA->ODR &= ~(1 << 5);
```

#### Method 2: Toggle Using XOR

```c
// Toggle LED state
GPIOA->ODR ^= (1 << 5);

// If it was 0 → becomes 1 (turns ON)
// If it was 1 → becomes 0 (turns OFF)
```

#### Method 3: Write Multiple Pins

```c
// Set PA5=1, PA6=0, PA7=1 simultaneously
GPIOA->ODR &= ~((1 << 5) | (1 << 6) | (1 << 7));  // Clear all three
GPIOA->ODR |= ((1 << 5) | (1 << 7));               // Set PA5 and PA7
```

### Read-Modify-Write Issue

**Important Concept:**

```c
// This is Read-Modify-Write operation:
GPIOA->ODR |= (1 << 5);

// Step 1: Read entire ODR register
// Step 2: Modify bit 5
// Step 3: Write back entire register

Problem in multi-threaded/interrupt context:
- If interrupt occurs between read and write
- And interrupt also modifies ODR
- Original modification might be lost
```

### Solution: Use BSRR (Next section)

## 4.8 GPIO Port Bit Set/Reset Register (GPIOx_BSRR)

**Reference:** RM0390 Section 8.4.7

### Register Details

- **Name:** GPIO port bit set/reset register (GPIOx_BSRR)
- **Offset:** 0x18
- **Reset Value:** 0x00000000
- **Access:** Write-only (reads return 0x00000000)

### Bit Field Diagram

```
GPIOx_BSRR (32-bit register)

Upper 16 bits: BR (Bit Reset)
Lower 16 bits: BS (Bit Set)

Bit:  31  30  29  28  27  26  25  24  23  22  21  20  19  18  17  16
      [BR15][BR14][BR13][BR12][BR11][BR10][BR9][BR8][BR7][BR6][BR5][BR4][BR3][BR2][BR1][BR0]
      Reset bits 15-0

Bit:  15  14  13  12  11  10   9   8   7   6   5   4   3   2   1   0
      [BS15][BS14][BS13][BS12][BS11][BS10][BS9][BS8][BS7][BS6][BS5][BS4][BS3][BS2][BS1][BS0]
      Set bits 15-0

Operation:
- Writing 1 to BSx (bit 0-15): Sets corresponding ODR bit to 1
- Writing 1 to BRx (bit 16-31): Resets corresponding ODR bit to 0
- Writing 0: No effect
- If both BSx and BRx set for same pin: SET takes priority
```

### Advantages Over ODR

1. **Atomic Operation**
   - Single write, no read-modify-write
   - Cannot be interrupted mid-operation
   - Safe in interrupt/multi-threaded context

2. **Selective Update**
   - Can set some bits, reset others, leave rest unchanged
   - All in one write operation

3. **Performance**
   - Single instruction
   - Faster than read-modify-write

### Usage Examples

#### Set Pin (Turn LED ON)

```c
// Set PA5 (turn LED ON)
// Write 1 to bit 5 (lower 16 bits)
GPIOA->BSRR = (1 << 5);

// Binary: 00000000 00000000 | 00000000 00100000
//         BR (reset bits)   | BS (set bits)
```

#### Reset Pin (Turn LED OFF)

```c
// Reset PA5 (turn LED OFF)
// Write 1 to bit 21 (16 + 5)
GPIOA->BSRR = (1 << 21);

// Or more clearly:
GPIOA->BSRR = (1 << (5 + 16));

// Binary: 00000000 00100000 | 00000000 00000000
//         BR (reset bits)   | BS (set bits)
```

#### Set and Reset Multiple Pins Simultaneously

```c
// Turn PA5 ON and PA6 OFF in single operation
GPIOA->BSRR = (1 << 5) | (1 << (6 + 16));

// PA5 bit 5 = 1 (SET)
// PA6 bit 22 = 1 (RESET)
```

### Comparison: ODR vs BSRR

```c
// Using ODR (Read-Modify-Write, 3 operations)
uint32_t temp = GPIOA->ODR;     // 1. Read
temp |= (1 << 5);                // 2. Modify
GPIOA->ODR = temp;               // 3. Write

// Using BSRR (Write-Only, 1 operation)
GPIOA->BSRR = (1 << 5);          // 1. Write (atomic)
```

### Toggle Implementation Comparison

```c
// Method 1: Using ODR with XOR
void LED_Toggle_ODR(void) {
    GPIOA->ODR ^= (1 << 5);
    // Still uses read-modify-write internally
}

// Method 2: Using BSRR (recommended)
void LED_Toggle_BSRR(void) {
    // Check current state and toggle
    if (GPIOA->ODR & (1 << 5)) {
        // Currently HIGH, set it LOW
        GPIOA->BSRR = (1 << (5 + 16));
    } else {
        // Currently LOW, set it HIGH
        GPIOA->BSRR = (1 << 5);
    }
}

// Method 3: Using ODR for toggle (simpler, acceptable for LED)
void LED_Toggle_Simple(void) {
    GPIOA->ODR ^= (1 << 5);
    // For LED blinking (no interrupts), this is fine
}
```

### When to Use BSRR vs ODR

| Scenario | Recommendation | Reason |
|----------|----------------|--------|
| Simple LED control | Either works | No critical timing |
| Interrupt context | BSRR | Atomic operation |
| Multiple threads | BSRR | Race condition protection |
| Bit-banging protocols | BSRR | Precise timing |
| Bulk pin updates | BSRR | Single operation |
| Simple toggle | ODR with XOR | Cleaner code |

## 4.9 Register Configuration Summary for LED Blinking

### Complete Register Setup for PA5 (LED)

```c
/**
 * Configure PA5 as output for LED control
 * Step-by-step register configuration
 */

// Step 1: Enable GPIOA Clock
// Register: RCC_AHB1ENR (0x40023830)
// Bit: 0 (GPIOAEN)
RCC->AHB1ENR |= (1 << 0);

// Step 2: Configure as Output
// Register: GPIOA_MODER (0x40020000)
// Bits: [11:10] for pin 5
// Value: 01 (output)
GPIOA->MODER &= ~(0x3 << 10);   // Clear
GPIOA->MODER |= (0x1 << 10);    // Set to output

// Step 3: Configure Output Type
// Register: GPIOA_OTYPER (0x40020004)
// Bit: 5
// Value: 0 (push-pull)
GPIOA->OTYPER &= ~(1 << 5);     // Push-pull

// Step 4: Configure Speed
// Register: GPIOA_OSPEEDR (0x40020008)
// Bits: [11:10] for pin 5
// Value: 01 (medium speed) or 10 (fast speed)
GPIOA->OSPEEDR &= ~(0x3 << 10);  // Clear
GPIOA->OSPEEDR |= (0x1 << 10);   // Medium speed

// Step 5: Configure Pull-up/Pull-down
// Register: GPIOA_PUPDR (0x4002000C)
// Bits: [11:10] for pin 5
// Value: 00 (no pull-up, no pull-down)
GPIOA->PUPDR &= ~(0x3 << 10);    // No pull-up/down

// Step 6: Control Output
// Method A: Using ODR
GPIOA->ODR |= (1 << 5);          // LED ON
GPIOA->ODR &= ~(1 << 5);         // LED OFF
GPIOA->ODR ^= (1 << 5);          // LED Toggle

// Method B: Using BSRR (preferred)
GPIOA->BSRR = (1 << 5);          // LED ON
GPIOA->BSRR = (1 << (5 + 16));   // LED OFF
```

### Register Values After Configuration

```
After configuring PA5 for LED:

RCC_AHB1ENR:    0x00000001  (bit 0 set: GPIOA clock enabled)
GPIOA_MODER:    0xA8000400  (bits [11:10] = 01: output)
GPIOA_OTYPER:   0x00000000  (bit 5 = 0: push-pull)
GPIOA_OSPEEDR:  0x00000400  (bits [11:10] = 01: medium speed)
GPIOA_PUPDR:    0x64000000  (bits [11:10] = 00: no pull)
GPIOA_ODR:      0x00000020  (bit 5 = 1: LED ON)
                0x00000000  (bit 5 = 0: LED OFF)
```

---

# Chapter 5: GPIO Driver Implementation - Output Mode

## 5.1 Driver Architecture

### Layered Design Approach

```
┌─────────────────────────────────────────┐
│  Application Layer (main.c)             │
│  - LED blinking logic                   │
│  - User application code                │
└──────────────┬──────────────────────────┘
               │ Uses
┌──────────────▼──────────────────────────┐
│  Driver API Layer                       │
│  - GPIO_Init()                          │
│  - GPIO_WritePin()                      │
│  - GPIO_TogglePin()                     │
│  (stm32f446xx_gpio_driver.c/.h)        │
└──────────────┬──────────────────────────┘
               │ Uses
┌──────────────▼──────────────────────────┐
│  Hardware Abstraction Layer             │
│  - Register definitions                 │
│  - Memory-mapped structures             │
│  - Helper macros                        │
│  (stm32f446xx.h)                        │
└──────────────┬──────────────────────────┘
               │ Accesses
┌──────────────▼──────────────────────────┐
│  Hardware (Silicon)                     │
│  - Actual GPIO registers                │
│  - Physical pins                        │
└─────────────────────────────────────────┘
```

## 5.2 Header File - stm32f446xx.h

**Purpose:** Define hardware registers and memory map

### Step 5.2.1: Include Standard Headers

```c
/**
 ******************************************************************************
 * @file           : stm32f446xx.h
 * @brief          : STM32F446xx Device Header
 * @reference      : RM0390
 ******************************************************************************
 */

#ifndef STM32F446XX_H
#define STM32F446XX_H

#include <stdint.h>
#include <stddef.h>

/**
 * Standard integer types used:
 * uint8_t  - 8-bit unsigned  (0 to 255)
 * uint16_t - 16-bit unsigned (0 to 65,535)
 * uint32_t - 32-bit unsigned (0 to 4,294,967,295)
 * 
 * 'volatile' keyword:
 * - Prevents compiler optimization
 * - Forces actual memory read/write
 * - Required for hardware registers
 */
```

### Step 5.2.2: Define Memory Base Addresses

**Reference:** RM0390 Section 2.3, Table 1

```c
/*
 * Base addresses of Flash and SRAM memories
 * Reference: RM0390 Section 2.3
 */
#define FLASH_BASE           0x08000000UL  /* Flash base address */
#define SRAM1_BASE           0x20000000UL  /* SRAM1 base address (112 KB) */
#define SRAM2_BASE           0x2001C000UL  /* SRAM2 base address (16 KB) */
#define ROM_BASE             0x1FFF0000UL  /* System memory (ROM) base */

/*
 * Peripheral base addresses
 * Reference: RM0390 Section 2.3, Table 1
 */
#define PERIPH_BASE          0x40000000UL  /* Peripheral base address */
#define APB1PERIPH_BASE      PERIPH_BASE   /* APB1 bus base */
#define APB2PERIPH_BASE      0x40010000UL  /* APB2 bus base */
#define AHB1PERIPH_BASE      0x40020000UL  /* AHB1 bus base */
#define AHB2PERIPH_BASE      0x50000000UL  /* AHB2 bus base */

/*
 * AHB1 peripheral base addresses
 * Reference: RM0390 Section 2.3, Table 1
 */
#define GPIOA_BASE           (AHB1PERIPH_BASE + 0x0000UL)  /* 0x40020000 */
#define GPIOB_BASE           (AHB1PERIPH_BASE + 0x0400UL)  /* 0x40020400 */
#define GPIOC_BASE           (AHB1PERIPH_BASE + 0x0800UL)  /* 0x40020800 */
#define GPIOD_BASE           (AHB1PERIPH_BASE + 0x0C00UL)  /* 0x40020C00 */
#define GPIOE_BASE           (AHB1PERIPH_BASE + 0x1000UL)  /* 0x40021000 */
#define GPIOF_BASE           (AHB1PERIPH_BASE + 0x1400UL)  /* 0x40021400 */
#define GPIOG_BASE           (AHB1PERIPH_BASE + 0x1800UL)  /* 0x40021800 */
#define GPIOH_BASE           (AHB1PERIPH_BASE + 0x1C00UL)  /* 0x40021C00 */

#define RCC_BASE             (AHB1PERIPH_BASE + 0x3800UL)  /* 0x40023800 */

/**
 * Note: UL suffix means "Unsigned Long"
 * - Ensures 32-bit unsigned type
 * - Prevents compiler warnings
 * - Standard practice in embedded C
 */
```

### Step 5.2.3: Define GPIO Register Structure

**Reference:** RM0390 Section 8.4

```c
/**
 * @brief GPIO Register Structure
 * Reference: RM0390 Section 8.4
 * 
 * This structure maps to the physical registers in hardware.
 * Each member corresponds to a specific register at a specific offset.
 */
typedef struct
{
    volatile uint32_t MODER;      /*!< GPIO port mode register,                offset: 0x00, RM0390 Section 8.4.1 */
    volatile uint32_t OTYPER;     /*!< GPIO port output type register,         offset: 0x04, RM0390 Section 8.4.2 */
    volatile uint32_t OSPEEDR;    /*!< GPIO port output speed register,        offset: 0x08, RM0390 Section 8.4.3 */
    volatile uint32_t PUPDR;      /*!< GPIO port pull-up/pull-down register,   offset: 0x0C, RM0390 Section 8.4.4 */
    volatile uint32_t IDR;        /*!< GPIO port input data register,          offset: 0x10, RM0390 Section 8.4.5 */
    volatile uint32_t ODR;        /*!< GPIO port output data register,         offset: 0x14, RM0390 Section 8.4.6 */
    volatile uint32_t BSRR;       /*!< GPIO port bit set/reset register,       offset: 0x18, RM0390 Section 8.4.7 */
    volatile uint32_t LCKR;       /*!< GPIO port configuration lock register,  offset: 0x1C, RM0390 Section 8.4.8 */
    volatile uint32_t AFR[2];     /*!< GPIO alternate function registers,      offset: 0x20-0x24, RM0390 Section 8.4.9-10 */
                                  /*!< AFR[0] = AFRL (low), AFR[1] = AFRH (high) */
} GPIO_TypeDef;

/**
 * Structure explanation:
 * - Each register is volatile uint32_t (32-bit, hardware can change)
 * - Order matches hardware layout exactly
 * - Compiler doesn't add padding (naturally aligned)
 * - sizeof(GPIO_TypeDef) = 40 bytes (0x28)
 */
```

### Step 5.2.4: Define RCC Register Structure

**Reference:** RM0390 Section 6.3

```c
/**
 * @brief RCC Register Structure
 * Reference: RM0390 Section 6.3
 * 
 * Reset and Clock Control peripheral
 * Controls clocks for all peripherals
 */
typedef struct
{
    volatile uint32_t CR;            /*!< RCC clock control register,                   offset: 0x00, RM0390 Section 6.3.1 */
    volatile uint32_t PLLCFGR;       /*!< RCC PLL configuration register,               offset: 0x04, RM0390 Section 6.3.2 */
    volatile uint32_t CFGR;          /*!< RCC clock configuration register,             offset: 0x08, RM0390 Section 6.3.3 */
    volatile uint32_t CIR;           /*!< RCC clock interrupt register,                 offset: 0x0C, RM0390 Section 6.3.4 */
    volatile uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,           offset: 0x10, RM0390 Section 6.3.7 */
    volatile uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,           offset: 0x14, RM0390 Section 6.3.8 */
    volatile uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,           offset: 0x18, RM0390 Section 6.3.9 */
    uint32_t          RESERVED0;     /*!< Reserved,                                     offset: 0x1C */
    volatile uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,           offset: 0x20, RM0390 Section 6.3.10 */
    volatile uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,           offset: 0x24, RM0390 Section 6.3.11 */
    uint32_t          RESERVED1[2];  /*!< Reserved,                                     offset: 0x28-0x2C */
    volatile uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register,    offset: 0x30, RM0390 Section 6.3.12 */
    volatile uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock enable register,    offset: 0x34, RM0390 Section 6.3.13 */
    volatile uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock enable register,    offset: 0x38, RM0390 Section 6.3.14 */
    uint32_t          RESERVED2;     /*!< Reserved,                                     offset: 0x3C */
    volatile uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,    offset: 0x40, RM0390 Section 6.3.15 */
    volatile uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,    offset: 0x44, RM0390 Section 6.3.16 */
    /* ... more registers omitted for brevity ... */
} RCC_TypeDef;

/**
 * Note: Reserved fields maintain correct offset alignment
 * - uint32_t RESERVED0 skips 4 bytes at offset 0x1C
 * - uint32_t RESERVED1[2] skips 8 bytes at 0x28-0x2C
 * This ensures AHB1ENR is at correct offset 0x30
 */
```

### Step 5.2.5: Create Peripheral Pointers

```c
/**
 * Peripheral pointer definitions
 * These macros create typed pointers to hardware addresses
 */
#define GPIOA              ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB              ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC              ((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD              ((GPIO_TypeDef*)GPIOD_BASE)
#define GPIOE              ((GPIO_TypeDef*)GPIOE_BASE)
#define GPIOF              ((GPIO_TypeDef*)GPIOF_BASE)
#define GPIOG              ((GPIO_TypeDef*)GPIOG_BASE)
#define GPIOH              ((GPIO_TypeDef*)GPIOH_BASE)

#define RCC                ((RCC_TypeDef*)RCC_BASE)

/**
 * Usage example:
 * GPIOA->MODER |= (1 << 10);
 * 
 * Expands to:
 * ((GPIO_TypeDef*)0x40020000)->MODER |= (1 << 10);
 * 
 * Which accesses memory at:
 * 0x40020000 + 0x00 = 0x40020000 (MODER register)
 */
```

### Step 5.2.6: Clock Control Macros

**Reference:** RM0390 Section 6.3.12

```c
/**
 * @brief Clock Enable Macros for GPIO peripherals
 * Reference: RM0390 Section 6.3.12 (RCC_AHB1ENR)
 * 
 * Each GPIO port has a dedicated bit in RCC_AHB1ENR register
 */
#define RCC_GPIOA_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 0))  /* Bit 0: GPIOAEN */
#define RCC_GPIOB_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 1))  /* Bit 1: GPIOBEN */
#define RCC_GPIOC_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 2))  /* Bit 2: GPIOCEN */
#define RCC_GPIOD_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 3))  /* Bit 3: GPIODEN */
#define RCC_GPIOE_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 4))  /* Bit 4: GPIOEEN */
#define RCC_GPIOF_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 5))  /* Bit 5: GPIOFEN */
#define RCC_GPIOG_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 6))  /* Bit 6: GPIOGEN */
#define RCC_GPIOH_CLK_ENABLE()     (RCC->AHB1ENR |= (1U << 7))  /* Bit 7: GPIOHEN */

/**
 * @brief Clock Disable Macros for GPIO peripherals
 */
#define RCC_GPIOA_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 0))
#define RCC_GPIOB_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 1))
#define RCC_GPIOC_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 2))
#define RCC_GPIOD_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 3))
#define RCC_GPIOE_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 4))
#define RCC_GPIOF_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 5))
#define RCC_GPIOG_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 6))
#define RCC_GPIOH_CLK_DISABLE()    (RCC->AHB1ENR &= ~(1U << 7))

/**
 * Note: 'U' suffix means unsigned
 * - (1U << 0) ensures unsigned arithmetic
 * - Prevents potential signed integer overflow warnings
 */
```

### Step 5.2.7: GPIO Reset Macros

**Reference:** RM0390 Section 6.3.7

```c
/**
 * @brief GPIO Reset Macros
 * Reference: RM0390 Section 6.3.7 (RCC_AHB1RSTR)
 * 
 * Reset procedure (per RM0390):
 * 1. Set bit to 1 (activate reset)
 * 2. Clear bit to 0 (release reset)
 * 
 * do { ... } while(0) pattern:
 * - Makes macro behave like a single statement
 * - Safe to use in if/else without braces
 * - Common embedded C practice
 */
#define RCC_GPIOA_FORCE_RESET()    (RCC->AHB1RSTR |= (1U << 0))
#define RCC_GPIOA_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(1U << 0))
#define RCC_GPIOA_RESET()          do { RCC_GPIOA_FORCE_RESET(); RCC_GPIOA_RELEASE_RESET(); } while(0)

#define RCC_GPIOB_FORCE_RESET()    (RCC->AHB1RSTR |= (1U << 1))
#define RCC_GPIOB_RELEASE_RESET()  (RCC->AHB1RSTR &= ~(1U << 1))
#define RCC_GPIOB_RESET()          do { RCC_GPIOB_FORCE_RESET(); RCC_GPIOB_RELEASE_RESET(); } while(0)

/* ... similar macros for GPIOC through GPIOH ... */

/**
 * Usage example:
 * RCC_GPIOA_RESET();  // Resets GPIOA to default state
 * 
 * This can be used with if/else safely:
 * if (condition)
 *     RCC_GPIOA_RESET();  // Works correctly due to do-while
 * else
 *     something_else();
 */
```

### Step 5.2.8: Generic Macros

```c
/**
 * @brief Generic Macros
 * Common definitions used throughout the driver
 */
#define ENABLE               1
#define DISABLE              0
#define SET                  ENABLE
#define RESET                DISABLE
#define GPIO_PIN_SET         SET
#define GPIO_PIN_RESET       RESET
#define FLAG_SET             SET
#define FLAG_RESET           RESET

/**
 * @brief NULL pointer definition
 */
#ifndef NULL
#define NULL                 ((void*)0)
#endif

#endif /* STM32F446XX_H */
```

This document continues in the next part due to length. Would you like me to continue with Chapter 5's driver implementation sections?

