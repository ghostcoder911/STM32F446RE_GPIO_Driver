# STM32 GPIO Driver Development - Complete Beginner's Guide
## LED Blinking with Custom GPIO Driver

---

## Slide 1: Introduction
### What We Built Today
- ✅ Custom GPIO driver from scratch (no HAL/Standard Library)
- ✅ LED blinking application for STM32F446RE Nucleo board
- ✅ Direct register-level programming

### Why This Matters
- Understanding how microcontrollers work at the lowest level
- Foundation for embedded systems programming
- Complete control over hardware

---

## Slide 2: What is GPIO?
### GPIO = General Purpose Input/Output

**Simple Analogy:** Think of GPIO pins like light switches in your house
- **Output Mode:** You control the switch (turn LED on/off)
- **Input Mode:** You read the switch state (is button pressed?)

### On STM32F446RE Nucleo Board:
- The board has multiple GPIO pins (PA0-PA15, PB0-PB15, PC0-PC15, etc.)
- **LD2 (User LED)** is connected to **PA5** (Port A, Pin 5)

---

## Slide 3: Understanding Microcontroller Memory
### Everything is Memory!

In microcontrollers, **hardware is controlled through memory addresses**

**Think of it like a huge apartment building:**
```
Address 0x40020000 → GPIOA's apartment
Address 0x40020400 → GPIOB's apartment
Address 0x40020800 → GPIOC's apartment
```

Each "apartment" (peripheral) has multiple "rooms" (registers):
- MODER room → Controls pin mode (input/output)
- ODR room → Controls output data (on/off)
- IDR room → Reads input data

**This is called Memory-Mapped I/O**

---

## Slide 4: STM32F446RE Architecture Overview

```
┌─────────────────────────────────────┐
│     ARM Cortex-M4 Processor         │
│         (Your Brain)                │
└──────────────┬──────────────────────┘
               │
        ┌──────┴──────┐
        │   AHB Bus   │  (High-speed highway)
        └──────┬──────┘
               │
    ┌──────────┴──────────┐
    │                     │
┌───┴────┐          ┌────┴────┐
│  RCC   │          │  GPIO   │
│(Clock) │          │ Ports   │
└────────┘          └─────────┘
```

### Key Components:
1. **RCC (Reset and Clock Control):** Power supply manager
2. **GPIO Ports:** The actual pins you can control
3. **AHB Bus:** Highway connecting everything

---

## Slide 5: The Files We Created

### 1. `stm32f446xx.h` - Device Header File
**What:** Defines the microcontroller's memory map
**Analogy:** This is like a phone book with everyone's address

### 2. `stm32f446xx_gpio_driver.h` - Driver Header
**What:** Function declarations and configuration structures
**Analogy:** Menu at a restaurant (what you can order)

### 3. `stm32f446xx_gpio_driver.c` - Driver Implementation
**What:** Actual implementation of GPIO functions
**Analogy:** Kitchen where food is prepared

### 4. `main.c` - Application Code
**What:** Your program that uses the driver
**Analogy:** You ordering food from the menu

---

## Slide 6: Understanding Registers
### What is a Register?

**A register is a 32-bit storage location in the microcontroller**

```
Register (32 bits) = 32 light switches in a row
Bit:  31 30 29 ... 3  2  1  0
      [0][1][0] ... [1][0][1][0]
```

### Example: MODER Register (Mode Register)
Controls whether each pin is input/output/alternate function

```
For Pin 5, we need bits 10 and 11:
Bits 11:10 = 00 → Input mode
Bits 11:10 = 01 → Output mode
Bits 11:10 = 10 → Alternate Function
Bits 11:10 = 11 → Analog mode
```

**Each pin uses 2 bits in MODER!**

---

## Slide 7: GPIO Registers Explained

### Main Registers We Use:

| Register | Full Name | Purpose | Example |
|----------|-----------|---------|---------|
| **MODER** | Mode Register | Set pin as input/output | Pin as output |
| **OTYPER** | Output Type | Push-pull or Open-drain | Push-pull (normal) |
| **OSPEEDR** | Output Speed | How fast pin switches | Fast speed |
| **PUPDR** | Pull-up/Pull-down | Internal resistor config | No pull-up/down |
| **ODR** | Output Data | Write 1 or 0 to pin | Turn LED on/off |
| **IDR** | Input Data | Read value from pin | Read button state |

---

## Slide 8: Step-by-Step - What We Did

### Step 1: Create Device Header (`stm32f446xx.h`)

**Purpose:** Define the memory map and register structures

#### 1.1 Define Base Addresses
```c
#define GPIOA_BASEADDR   0x40020000U
#define RCC_BASEADDR     0x40023800U
```
**Translation:** "GPIOA lives at address 0x40020000"

#### 1.2 Create Register Structure
```c
typedef struct
{
    volatile uint32_t MODER;    // Offset: 0x00
    volatile uint32_t OTYPER;   // Offset: 0x04
    volatile uint32_t OSPEEDR;  // Offset: 0x08
    // ... more registers
} GPIO_RegDef_t;
```

**What is `volatile`?**
- Tells compiler: "This value can change unexpectedly (by hardware)"
- Prevents compiler optimization that might break hardware access

#### 1.3 Create Peripheral Pointers
```c
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
```
**Translation:** "GPIOA is a pointer to a GPIO structure at address 0x40020000"

---

## Slide 9: Understanding Pointers (For Beginners)

### What is a Pointer?

**Analogy:** A pointer is like a house address

```c
uint32_t* ptr = 0x40020000;
```

This means:
- `ptr` contains an address (0x40020000)
- `*ptr` means "go to that address and get the value"

### Example with GPIO:
```c
GPIOA->MODER = 0x12345678;
```

Breaks down to:
1. Go to address 0x40020000 (GPIOA base)
2. Go to offset 0x00 (MODER register)
3. Write value 0x12345678 there

**It's like:** "Go to GPIOA's house, enter the MODER room, write on the board"

---

## Slide 10: Clock Enable - Why?

### Rule: Before using ANY peripheral, ENABLE ITS CLOCK!

**Analogy:** Before using a room in a building, turn on the lights!

```c
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
```

### Breaking it Down:
```
RCC->AHB1ENR |= (1 << 0)

RCC->AHB1ENR    → Go to RCC's clock enable register
|=              → OR operation (set bit without affecting others)
(1 << 0)        → Shift 1 left by 0 = 00000001 (bit 0)
```

**Result:** Bit 0 of AHB1ENR is set to 1 → GPIOA clock is ON ✅

---

## Slide 11: Bitwise Operations Crash Course

### Why We Use Them?
To control individual bits without affecting others!

### Common Operations:

```c
// 1. SET a bit (turn it ON)
register |= (1 << bit_position);
Example: x |= (1 << 5);  // Set bit 5 to 1

// 2. CLEAR a bit (turn it OFF)
register &= ~(1 << bit_position);
Example: x &= ~(1 << 5);  // Clear bit 5 to 0

// 3. TOGGLE a bit (flip it)
register ^= (1 << bit_position);
Example: x ^= (1 << 5);  // Flip bit 5

// 4. READ a bit
value = (register >> bit_position) & 0x01;
```

### Visual Example:
```
Original: 10110010
Set bit 2:    10110110  (using |=)
Clear bit 1:  10110100  (using &= ~)
Toggle bit 7: 00110100  (using ^=)
```

---

## Slide 12: GPIO Initialization Explained

### Our Goal: Configure PA5 as Output

```c
GPIO_Handle_t GpioLed;
GpioLed.pGPIOx = GPIOA;
GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
```

### What Happens Inside `GPIO_Init()`?

#### Step 1: Enable Clock
```c
GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
```
**Translation:** Turn on GPIOA's power supply

---

## Slide 13: GPIO Initialization - MODER Register

#### Step 2: Configure Mode Register
```c
// For pin 5, we need bits 10 and 11
temp = (GPIO_MODE_OUT << (2 * 5));  // GPIO_MODE_OUT = 1
// temp = 1 << 10 = 0x00000400

// Clear bits 10-11 first
pGPIOx->MODER &= ~(0x3 << 10);

// Set bits 10-11
pGPIOx->MODER |= temp;
```

### Visual Representation:
```
MODER Register (32 bits):
Bit:  31...12 11 10 9  8  7  6  5  4  3  2  1  0
      [....] [0][1][..][..][..][..][..][..][..][..]
              ↑  ↑
         Pin 5 mode = 01 (Output)
```

**Why multiply by 2?** Each pin uses 2 bits!
- Pin 0 → bits 0-1
- Pin 5 → bits 10-11
- Pin 15 → bits 30-31

---

## Slide 14: GPIO Initialization - Other Registers

#### Step 3: Configure Speed Register
```c
temp = (GPIO_SPEED_FAST << (2 * 5));
pGPIOx->OSPEEDR |= temp;
```
**Purpose:** Controls how fast the pin can change state

#### Step 4: Configure Pull-up/Pull-down
```c
temp = (GPIO_NO_PUPD << (2 * 5));
pGPIOx->PUPDR |= temp;
```
**Purpose:** Internal resistors for input pins (we don't need for output)

#### Step 5: Configure Output Type
```c
temp = (GPIO_OP_TYPE_PP << 5);
pGPIOx->OTYPER |= temp;
```
**Purpose:** Push-pull vs Open-drain output type

---

## Slide 15: Push-Pull vs Open-Drain

### Push-Pull (What We Use)
```
MCU can drive pin HIGH (3.3V) or LOW (0V)

HIGH: ──┐        Internal switch connects to VCC
        │        LED gets 3.3V
        LED
        │
LOW:    └── GND  Internal switch connects to GND
```
**Use case:** Most common, LEDs, normal I/O

### Open-Drain
```
MCU can drive pin LOW or make it FLOATING

Only pulls to GND, needs external pull-up resistor
```
**Use case:** I2C communication, connecting multiple devices

---

## Slide 16: LED Control Functions

### Toggle Function
```c
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}
```

**What's happening?**
```c
^= is XOR operation
```

### XOR Truth Table:
```
Current | XOR 1 | Result
--------|-------|-------
   0    |   1   |   1    (LED turns ON)
   1    |   1   |   0    (LED turns OFF)
```

**Magic:** XOR with 1 always flips the bit! Perfect for toggling!

---

## Slide 17: Alternative LED Control Methods

### Method 1: Using ODR (What Toggle Uses Internally)
```c
// Turn ON
GPIOA->ODR |= (1 << 5);

// Turn OFF
GPIOA->ODR &= ~(1 << 5);
```

### Method 2: Using BSRR (Atomic Set/Reset) - More Efficient!
```c
// Turn ON (Set bit)
GPIOA->BSRR = (1 << 5);

// Turn OFF (Reset bit)
GPIOA->BSRR = (1 << (5 + 16));
```

**Why BSRR is better?**
- Atomic operation (cannot be interrupted)
- Single instruction
- Lower half (bits 0-15) = SET
- Upper half (bits 16-31) = RESET

---

## Slide 18: The Main Program Flow

```c
int main(void)
{
    // 1. Print startup message
    printf("Hello World - LED Blinking Program Started\n");
    
    // 2. Create GPIO configuration structure
    GPIO_Handle_t GpioLed;
    
    // 3. Configure settings
    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    // ... more settings
    
    // 4. Initialize GPIO (applies all settings to registers)
    GPIO_Init(&GpioLed);
    
    // 5. Infinite loop - blink LED
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
        delay();  // Wait ~500ms
    }
}
```

---

## Slide 19: Understanding the Delay Function

```c
void delay(void)
{
    for(uint32_t i = 0; i < 500000; i++);
}
```

### How It Works:
- CPU executes empty loop 500,000 times
- At 16 MHz clock, each iteration takes ~4-5 cycles
- Total time ≈ 500ms

### Why This is NOT Ideal:
- ❌ Blocks CPU (wastes power)
- ❌ Not accurate (depends on optimization)
- ❌ CPU can't do other tasks

### Better Alternatives (Future Improvements):
- ✅ Use Timer peripheral
- ✅ Use SysTick timer
- ✅ Use interrupts (non-blocking)

---

## Slide 20: Memory Map Summary

### Complete Address Map:
```
0x40023800  →  RCC (Clock Control)
    ├── Offset 0x30 → AHB1ENR (Enable GPIO clocks)
    └── Offset 0x10 → AHB1RSTR (Reset GPIO)

0x40020000  →  GPIOA
    ├── Offset 0x00 → MODER (Mode)
    ├── Offset 0x04 → OTYPER (Output type)
    ├── Offset 0x08 → OSPEEDR (Speed)
    ├── Offset 0x0C → PUPDR (Pull-up/down)
    ├── Offset 0x10 → IDR (Input data)
    ├── Offset 0x14 → ODR (Output data)
    └── Offset 0x18 → BSRR (Bit set/reset)

0x40020400  →  GPIOB
0x40020800  →  GPIOC
... and so on
```

---

## Slide 21: Execution Flow Diagram

```
Power ON
   │
   ▼
main() starts
   │
   ▼
printf("Hello...")  ────→ ITM Console
   │
   ▼
Create GPIO_Handle_t structure
Configure pin settings
   │
   ▼
GPIO_Init()
   ├──→ Enable GPIOA clock (RCC->AHB1ENR)
   ├──→ Configure MODER (set pin 5 as output)
   ├──→ Configure OSPEEDR (set speed)
   ├──→ Configure PUPDR (no pull-up/down)
   └──→ Configure OTYPER (push-pull)
   │
   ▼
while(1) loop
   ├──→ GPIO_ToggleOutputPin()
   │       └──→ XOR bit 5 in ODR register
   │             └──→ LED toggles
   ├──→ delay()
   │       └──→ Wait ~500ms
   └──→ Repeat forever
```

---

## Slide 22: Driver Architecture - Layered Approach

```
┌─────────────────────────────────┐
│      Application Layer          │
│        (main.c)                 │  ← Your code: LED blinking logic
├─────────────────────────────────┤
│      Driver API Layer           │
│  (stm32f446xx_gpio_driver.h/c) │  ← GPIO functions: Init, Toggle, etc.
├─────────────────────────────────┤
│    Hardware Abstraction Layer   │
│     (stm32f446xx.h)             │  ← Register definitions, macros
├─────────────────────────────────┤
│       Hardware Registers        │  ← Actual silicon in the chip
└─────────────────────────────────┘
```

### Benefits of This Approach:
- ✅ **Portability:** Easy to port to different STM32 chips
- ✅ **Maintainability:** Changes in one layer don't affect others
- ✅ **Reusability:** Driver can be used in multiple projects
- ✅ **Readability:** Clean separation of concerns

---

## Slide 23: Data Types Explained

### Why `volatile`?
```c
volatile uint32_t MODER;
```
**Purpose:** Prevents compiler optimization
- Hardware can change register values independently
- Without `volatile`, compiler might cache the value
- With `volatile`, forces actual memory read every time

### Why `uint32_t` instead of `int`?
```c
uint32_t = unsigned 32-bit integer
```
**Benefits:**
- ✅ Exact size (32 bits = 4 bytes)
- ✅ Portable across platforms
- ✅ Matches hardware register size
- ✅ No negative values (unsigned)

### Sized Integer Types:
```c
uint8_t   → 8 bits  (0 to 255)
uint16_t  → 16 bits (0 to 65,535)
uint32_t  → 32 bits (0 to 4,294,967,295)
```

---

## Slide 24: Common Mistakes & Debugging

### Mistake 1: Forgot to Enable Clock
```c
❌ GPIO_Init(&GpioLed);  // Without clock enabled
✅ GPIO_PeriClockControl(GPIOA, ENABLE);
   GPIO_Init(&GpioLed);
```
**Symptom:** GPIO doesn't work at all

### Mistake 2: Wrong Pin Number
```c
❌ GPIO_PIN_NO_15  // Wrong pin
✅ GPIO_PIN_NO_5   // Correct for LD2
```
**Symptom:** Different LED blinks or nothing happens

### Mistake 3: Infinite Loop Before Code
```c
❌ for(;;);          // Stuck here forever!
   printf("Hello");  // Never executes
   
✅ printf("Hello");  // Executes first
   for(;;);          // Then loop
```

### Mistake 4: Wrong Register Bit Manipulation
```c
❌ MODER = (1 << 5);          // Overwrites entire register
✅ MODER |= (1 << 5);         // Sets only bit 5
✅ MODER &= ~(1 << 5);        // Clears only bit 5
```

---

## Slide 25: How to Verify It's Working

### Hardware Verification:
1. ✅ LED blinks on the board (LD2)
2. ✅ Console shows "Hello World" messages via SWV

### Using Debugger:
```
1. Set breakpoint in while(1) loop
2. Run in debug mode
3. Watch these registers:
   - RCC->AHB1ENR (bit 0 should be 1)
   - GPIOA->MODER (bits 10-11 should be 01)
   - GPIOA->ODR (bit 5 should toggle)
```

### Register Values to Expect:
```
RCC->AHB1ENR   = 0x00000001 (GPIOA clock enabled)
GPIOA->MODER   = 0x00000400 (Pin 5 as output, others default)
GPIOA->ODR     = 0x00000020 (LED ON) or 0x00000000 (LED OFF)
```

---

## Slide 26: Next Steps - Extending the Driver

### What You Can Add:

#### 1. **Button Input Support**
```c
// Read user button (PC13 on Nucleo board)
if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
{
    // Button pressed (active low)
}
```

#### 2. **Interrupt Support**
- Configure EXTI (External Interrupt)
- Implement ISR (Interrupt Service Routine)
- Non-blocking button detection

#### 3. **Other Peripherals**
- UART driver (serial communication)
- I2C driver (sensor communication)
- SPI driver (high-speed communication)
- Timer driver (accurate delays)

#### 4. **Advanced GPIO Features**
- Alternate function configuration
- Open-drain outputs
- GPIO locking

---

## Slide 27: Comparison with HAL Library

### Our Driver vs HAL:

| Aspect | Our Driver | STM32 HAL |
|--------|-----------|-----------|
| Code Size | Small (~200 lines) | Large (thousands of lines) |
| Performance | Fast | Slower (more abstraction) |
| Learning | Learn hardware deeply | Abstract, easier to use |
| Flexibility | Full control | Limited by API |
| Portability | Chip-specific | Cross-STM32 family |

### Our Approach (Register Level):
```c
GPIOA->ODR ^= (1 << 5);  // Direct register access
```

### HAL Approach:
```c
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  // Function call
```

**Both are valid!** Register-level gives you deeper understanding.

---

## Slide 28: Important Concepts Summary

### 1. Memory-Mapped I/O
Hardware is controlled through memory addresses

### 2. Peripheral Clocks
Always enable clock before using peripheral

### 3. Register Configuration
Use bitwise operations to control specific bits

### 4. Structure-Based Access
Organize registers in structs for clean code

### 5. Volatile Keyword
Prevents optimization for hardware registers

### 6. Layered Architecture
Separate hardware definitions from driver logic from application

---

## Slide 29: Quick Reference - GPIO Register Bits

### MODER (2 bits per pin):
```
00 = Input
01 = Output
10 = Alternate Function
11 = Analog
```

### OTYPER (1 bit per pin):
```
0 = Push-Pull
1 = Open-Drain
```

### OSPEEDR (2 bits per pin):
```
00 = Low speed
01 = Medium speed
10 = Fast speed
11 = High speed
```

### PUPDR (2 bits per pin):
```
00 = No pull-up/pull-down
01 = Pull-up
10 = Pull-down
11 = Reserved
```

---

## Slide 30: Resources for Further Learning

### Official Documentation:
1. **STM32F446RE Reference Manual (RM0390)**
   - Complete register descriptions
   - 1100+ pages of details

2. **STM32F446RE Datasheet**
   - Pin descriptions
   - Electrical characteristics

3. **ARM Cortex-M4 Technical Reference Manual**
   - Processor architecture
   - Instruction set

### Online Resources:
- STM32 Community Forum
- GitHub STM32 examples
- Embedded C programming tutorials

### Practice Projects:
1. Multiple LED patterns
2. Button-controlled LED
3. PWM LED dimming
4. UART communication
5. Sensor interfacing

---

## Slide 31: What You've Accomplished! 🎉

### Knowledge Gained:
✅ Understanding of microcontroller architecture
✅ Register-level programming
✅ GPIO peripheral operations
✅ Bitwise operations mastery
✅ Driver development concepts
✅ Embedded C programming

### Skills Developed:
✅ Reading datasheets and reference manuals
✅ Debugging embedded systems
✅ Writing reusable driver code
✅ Hardware-software interfacing

### You Can Now:
✅ Configure any GPIO pin
✅ Create custom peripheral drivers
✅ Read and understand STM32 documentation
✅ Debug register-level issues
✅ Build embedded systems from scratch

---

## Slide 32: Final Thoughts

### The Journey:
```
Beginner → Understanding Hardware → Writing Drivers → Expert
```

### Remember:
- **Start Simple:** We began with just blinking an LED
- **Understand Deeply:** We learned WHY, not just HOW
- **Build Gradually:** Each concept builds on the previous
- **Keep Practicing:** Try different pins, patterns, speeds

### Quote:
> "The best way to learn embedded systems is to build something, 
> break it, understand why it broke, and fix it."

### Your Next Challenge:
Try modifying the code to:
1. Blink at different speeds
2. Create SOS pattern in Morse code
3. Control multiple LEDs
4. Add button control

**You've got this! Keep coding! 🚀**

---

## Appendix A: Complete Code Walkthrough

### File 1: stm32f446xx.h
```c
// This file defines WHERE everything is in memory

// Base addresses (the "apartments")
#define GPIOA_BASEADDR   0x40020000U

// Register structure (the "rooms" in each apartment)
typedef struct
{
    volatile uint32_t MODER;   // Room 0x00
    volatile uint32_t OTYPER;  // Room 0x04
    // ... more rooms
} GPIO_RegDef_t;

// Create easy access pointers
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)

// Clock control macros
#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
```

---

## Appendix B: Calculation Examples

### Example 1: MODER Register for Pin 5
```
Pin 5 uses bits 10 and 11
Position = 2 × 5 = 10

To set as output (01):
Shift left: 0b01 << 10 = 0b010000000000 = 0x400

Clear operation:
Mask: 0b11 << 10 = 0x0C00
Clear: MODER &= ~0x0C00

Set operation:
MODER |= 0x0400
```

### Example 2: Toggle Bit 5 in ODR
```
ODR current value: 0x00000020 (bit 5 is 1, LED ON)
XOR with:          0x00000020 (1 << 5)
Result:            0x00000000 (bit 5 is 0, LED OFF)

Next toggle:
ODR current value: 0x00000000 (bit 5 is 0, LED OFF)
XOR with:          0x00000020
Result:            0x00000020 (bit 5 is 1, LED ON)
```

---

## Appendix C: Debugging Checklist

### LED Not Blinking? Check:
- [ ] Is GPIOA clock enabled? (RCC->AHB1ENR bit 0)
- [ ] Is pin configured as output? (GPIOA->MODER bits 10-11 = 01)
- [ ] Is the correct pin number used? (Pin 5 for LD2)
- [ ] Is delay function being called?
- [ ] Is program stuck in infinite loop before main code?
- [ ] Is power connected to board?
- [ ] Is ST-Link connected?

### Console Not Working? Check:
- [ ] Is SWV enabled in debug configuration?
- [ ] Is ITM Port 0 enabled?
- [ ] Is trace clock configured (16MHz)?
- [ ] Is printf redirected to ITM_SendChar?
- [ ] Is program actually running (not stuck at breakpoint)?

---

## Appendix D: Glossary

**AHB:** Advanced High-performance Bus - Fast data highway in MCU
**Bitwise Operation:** Manipulating individual bits in a number
**Clock Enable:** Turning on power to a peripheral
**GPIO:** General Purpose Input/Output - Configurable pins
**HAL:** Hardware Abstraction Layer - High-level driver library
**IDR:** Input Data Register - Reads pin states
**ISR:** Interrupt Service Routine - Function called on interrupt
**ITM:** Instrumentation Trace Macrocell - Debug output channel
**Memory-Mapped I/O:** Accessing hardware through memory addresses
**MODER:** Mode Register - Configures pin modes
**NVIC:** Nested Vectored Interrupt Controller
**ODR:** Output Data Register - Controls pin outputs
**Peripheral:** Hardware module (GPIO, UART, Timer, etc.)
**Pull-up/Pull-down:** Internal resistors for stable input readings
**RCC:** Reset and Clock Control - Power management
**Register:** 32-bit storage location controlling hardware
**SWV:** Serial Wire Viewer - Debug trace interface
**Volatile:** Keyword preventing compiler optimization for hardware access

---

## End of Presentation

### Questions to Think About:
1. Why do we need to enable clocks?
2. What's the difference between ODR and BSRR?
3. Why use structures for register access?
4. How would you configure a pin as input?
5. What happens if you forget the volatile keyword?

**Good luck with your embedded systems journey!** 💡🔧


