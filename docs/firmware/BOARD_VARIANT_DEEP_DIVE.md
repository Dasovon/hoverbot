# BOARD_VARIANT Deep Dive - Hoverboard Firmware

## Executive Summary

The `BOARD_VARIANT` setting controls GPIO pin mappings for different hoverboard mainboard PCB revisions. Using the wrong variant causes **complete power management failure** because the firmware writes to wrong GPIO pins, preventing the power latch from engaging.

**Your Hardware:** BOARD_VARIANT = 1 ✅

---

## Table of Contents

1. [Pin Mapping Differences](#pin-mapping-differences)
2. [Power-On Sequence](#power-on-sequence)
3. [GPIO Initialization](#gpio-initialization)
4. [Why Wrong Variant Fails](#why-wrong-variant-fails)
5. [Code Walkthrough](#code-walkthrough)
6. [Safe Modification Points](#safe-modification-points)

---

## Pin Mapping Differences

### Critical Pins - BOARD_VARIANT Comparison

**From `Inc/defines.h` lines 130-150:**

```c
// Power Latch Pin (holds board ON after button release)
#if BOARD_VARIANT == 0
  #define OFF_PIN       GPIO_PIN_5
  #define OFF_PORT      GPIOA
#elif BOARD_VARIANT == 1
  #define OFF_PIN       GPIO_PIN_15   // ← YOUR HARDWARE
  #define OFF_PORT      GPIOC         // ← YOUR HARDWARE
#endif

// Power Button Pin (detects button presses)
#if BOARD_VARIANT == 0
  #define BUTTON_PIN    GPIO_PIN_1
  #define BUTTON_PORT   GPIOA
#elif BOARD_VARIANT == 1
  #define BUTTON_PIN    GPIO_PIN_9    // ← YOUR HARDWARE
  #define BUTTON_PORT   GPIOB         // ← YOUR HARDWARE
#endif

// Charger Detection Pin
#if BOARD_VARIANT == 0
  #define CHARGER_PIN   GPIO_PIN_12
  #define CHARGER_PORT  GPIOA
#elif BOARD_VARIANT == 1
  #define CHARGER_PIN   GPIO_PIN_11   // ← YOUR HARDWARE
  #define CHARGER_PORT  GPIOA         // ← YOUR HARDWARE
#endif
```

### Complete Pin Table

| Signal | Purpose | Variant 0 | Variant 1 (YOU) | Critical? |
|--------|---------|-----------|-----------------|-----------|
| **OFF_PIN** | Power latch control | PA5 | **PC15** | ⚠️ YES |
| **BUTTON_PIN** | Power button input | PA1 | **PB9** | ⚠️ YES |
| **CHARGER_PIN** | Charger detect | PA12 | **PA11** | Medium |
| **BUZZER_PIN** | Beeper output | PA4 | **PC13** | Low |
| **DCLINK_PIN** | Battery voltage | PC2 | **PA1** | Medium |
| **LED_PIN** | Status LED | PB2 | **PB2** | Same |

**Why Critical?**
- ❌ Wrong OFF_PIN → Power latch never engages → Board shuts down immediately
- ❌ Wrong BUTTON_PIN → Button presses never detected → Can't boot or control board

---

## Power-On Sequence

### Physical Hardware Behavior

```
User Action             Hardware Response              Firmware Action
────────────────────────────────────────────────────────────────────────
[1] Press button    →   Power button connects      →  (No firmware yet,
    (physical)          3.3V to BUTTON_PIN            board unpowered)
                        
[2] Hold button     →   Battery power flows to     →  Firmware starts
                        voltage regulators             STM32 boots
                        STM32 gets power               
                        
[3] Firmware init   →   (Button still held)        →  GPIO_Init()
                                                       Read BUTTON_PIN
                                                       Initialize system
                        
[4] Latch engage    →   Firmware sets OFF_PIN=HIGH →  HAL_GPIO_WritePin()
    **CRITICAL!**       Power latch circuit locks      OFF_PORT, OFF_PIN
                        Power stays ON                 GPIO_PIN_SET
                        
[5] Release button  →   Button no longer pressed   →  System stays ON
                        But latch holds power!         (if latch worked)
                        
[6] Normal operation →  Board runs continuously    →  Main loop active
```

### The Critical Moment - Line 203 in `Src/main.c`

```c
// From main() function after initialization:

MX_GPIO_Init();         // Initialize all GPIO pins
MX_TIM_Init();          // Initialize timers
MX_ADC1_Init();         // Initialize ADC1
MX_ADC2_Init();         // Initialize ADC2
BLDC_Init();            // Initialize motor controller

// ⚠️ CRITICAL LINE - This must write to correct pin!
HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch

Input_Lim_Init();       // Input limits
Input_Init();           // Input handling
HAL_ADC_Start(&hadc1);  // Start ADC conversions
HAL_ADC_Start(&hadc2);
poweronMelody();        // Play startup beep
```

**What happens:**
1. **BOARD_VARIANT=1 (CORRECT):**
   - OFF_PORT = GPIOC, OFF_PIN = GPIO_PIN_15
   - Sets PC15 HIGH
   - Power latch circuit sees HIGH on PC15
   - ✅ Latch engages, power stays ON

2. **BOARD_VARIANT=0 (WRONG):**
   - OFF_PORT = GPIOA, OFF_PIN = GPIO_PIN_5  
   - Sets PA5 HIGH (wrong pin!)
   - Power latch circuit looking at PC15 sees nothing
   - ❌ Latch never engages
   - ❌ When button released → POWER CUTS → Board dies

---

## GPIO Initialization

### From `Src/setup.c` lines 400-420

```c
// Configure Button Pin as INPUT
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Pin = BUTTON_PIN;          // PA1 or PB9 depending on variant
HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

// Configure OFF Pin (Power Latch) as OUTPUT
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
GPIO_InitStruct.Pin = OFF_PIN;               // PA5 or PC15 depending on variant
HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);

// Configure LED Pin as OUTPUT
GPIO_InitStruct.Pin = LED_PIN;
HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

// Configure Buzzer Pin as OUTPUT  
GPIO_InitStruct.Pin = BUZZER_PIN;
HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);
```

**Key Points:**
- `GPIO_MODE_INPUT` - Pin reads external signal (button state)
- `GPIO_MODE_OUTPUT_PP` - Pin drives signal (controls power latch)
- `GPIO_NOPULL` - No internal pull-up/pull-down resistors
- Each pin initialized according to BOARD_VARIANT definitions

---

## Why Wrong Variant Fails

### Failure Scenario with BOARD_VARIANT=0 on Variant 1 Hardware

```
Timeline of Failure
═══════════════════════════════════════════════════════════

T=0s    User presses power button
        ✓ Button physically connects power
        ✓ Battery → Regulators → STM32 gets power
        
T=0.1s  STM32 boots, firmware starts
        ✓ GPIO initialization begins
        ⚠️ BUTTON_PIN set to PA1 (wrong - should be PB9)
        ⚠️ OFF_PIN set to PA5 (wrong - should be PC15)
        
T=0.2s  Firmware reads button state
        ❌ Reads PA1 (wrong pin, floating/undefined)
        ⚠️ Button actually pressed on PB9 (not being read!)
        ❓ Firmware might see random state on PA1
        
T=0.5s  Firmware reaches power latch line
        ❌ HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, HIGH)
        ✓ PA5 goes HIGH (but nothing cares about PA5!)
        ❌ PC15 stays LOW (power latch circuit needs this!)
        
T=1.0s  User releases button (thinking board is on)
        ❌ Power latch circuit sees PC15=LOW
        ❌ No latch engagement
        ❌ Battery power disconnects
        💀 BOARD DIES IMMEDIATELY
```

### Why Variant 1 Works

```
Timeline of Success  
═══════════════════════════════════════════════════════════

T=0s    User presses power button
        ✓ Button connects power on PB9
        ✓ Battery → Regulators → STM32 gets power
        
T=0.1s  STM32 boots, firmware starts
        ✓ GPIO initialization
        ✓ BUTTON_PIN = PB9 (correct!)
        ✓ OFF_PIN = PC15 (correct!)
        
T=0.2s  Firmware reads button state
        ✓ Reads PB9 (correct pin)
        ✓ Detects button pressed
        ✓ Continues boot sequence
        
T=0.5s  Firmware reaches power latch line
        ✓ HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, HIGH)
        ✓ PC15 goes HIGH
        ✓ Power latch circuit sees HIGH on PC15
        ✓ Latch engages!
        
T=1.0s  User releases button
        ✓ Power latch holds power ON
        ✓ Board continues running
        ✅ SUCCESS - Normal operation
```

---

## Code Walkthrough

### Step 1: Conditional Compilation (`Inc/defines.h`)

```c
// Preprocessor checks BOARD_VARIANT value at compile time

#if BOARD_VARIANT == 0
  #define OFF_PIN GPIO_PIN_5       // Define for variant 0
  #define OFF_PORT GPIOA
#elif BOARD_VARIANT == 1
  #define OFF_PIN GPIO_PIN_15      // Define for variant 1
  #define OFF_PORT GPIOC
#endif

// After preprocessing, ONLY ONE definition exists in compiled code:
// If BOARD_VARIANT=1, the compiler sees:
//   #define OFF_PIN GPIO_PIN_15
//   #define OFF_PORT GPIOC
```

**How it works:**
1. Compiler reads `config.h`: `#define BOARD_VARIANT 1`
2. Preprocessor evaluates `#if BOARD_VARIANT == 0` → FALSE, skips
3. Preprocessor evaluates `#elif BOARD_VARIANT == 1` → TRUE, includes
4. All subsequent code uses OFF_PIN=GPIO_PIN_15, OFF_PORT=GPIOC

### Step 2: GPIO Initialization (`Src/setup.c`)

```c
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  // Configure button pin (input)
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = BUTTON_PIN;        // Becomes PB9 for variant 1
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);  // GPIOB for variant 1
  
  // Configure power latch pin (output)
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = OFF_PIN;           // Becomes PC15 for variant 1
  HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);     // GPIOC for variant 1
}
```

**STM32 HAL Details:**
- `HAL_GPIO_Init()` - STM32 Hardware Abstraction Layer function
- Configures GPIO peripheral registers
- Sets pin mode (input/output), pull resistors, speed, etc.

### Step 3: Power Latch Activation (`Src/main.c`)

```c
int main(void) {
  // ... initialization code ...
  
  MX_GPIO_Init();   // Calls function above, sets up all pins
  
  // THE CRITICAL LINE:
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
  
  // What this expands to for BOARD_VARIANT=1:
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  
  // ... rest of code ...
}
```

**Under the hood (pseudo-code):**
```c
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET) {
  // Sets bit 15 of GPIOC output register
  GPIOC->ODR |= (1 << 15);  // Bitwise OR to set bit 15 HIGH
  
  // Hardware effect:
  // - PC15 pin voltage goes from 0V → 3.3V
  // - Power latch circuit detects HIGH signal
  // - Transistor/FET in latch circuit activates
  // - Battery power locked ON
}
```

### Step 4: Button Reading (`Src/util.c`)

```c
// From util.c line 533 - Input calibration routine
while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && 
       input_cal_timeout++ < 4000) {
  // Wait while button is NOT pressed
  // Timeout after 20 seconds (4000 * 5ms loops)
}

// What HAL_GPIO_ReadPin does:
HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) {
  // Reads bit 9 of GPIOB input register
  return (GPIOB->IDR & (1 << 9)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  
  // Returns:
  // - GPIO_PIN_SET (1) if button pressed (pin HIGH)
  // - GPIO_PIN_RESET (0) if button not pressed (pin LOW)
}
```

---

## Safe Modification Points

### Where You CAN Safely Modify

#### 1. Add Custom Button Actions (`Src/util.c`)

**Current code (line 1560):**
```c
if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
  enable ^= 1;  // Toggle enable on/off
  beepShort(enable ? 24 : 16);
  
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    HAL_Delay(10);  // Wait for button release
  }
}
```

**Safe modification - Add double-press detection:**
```c
if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
  uint32_t press_start = HAL_GetTick();
  
  while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
    HAL_Delay(10);
  }
  
  uint32_t press_duration = HAL_GetTick() - press_start;
  
  if (press_duration > 2000) {
    // Long press (>2 seconds) - Custom action
    beepLong(2);
    // Your code here - enter special mode?
  } else {
    // Short press - Normal toggle
    enable ^= 1;
    beepShort(enable ? 24 : 16);
  }
}
```

#### 2. Add Status Feedback via LED/Buzzer

**Safe location:** After power latch activation in `main.c`

```c
HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch

// ✅ SAFE TO ADD HERE - Custom startup sequence
for (int i = 0; i < 3; i++) {
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
}
```

#### 3. Add Diagnostic Output via UART

**Safe location:** In main loop, after initialization

```c
// In main.c, inside while(1) loop
if (diagnostics_enabled) {
  char debug_msg[64];
  sprintf(debug_msg, "Speed: %d, Battery: %d\r\n", speedL, batVoltage);
  HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);
  HAL_Delay(1000);  // Once per second
}
```

### Where You MUST NOT Modify (Without Deep Understanding)

#### ❌ DANGER ZONE 1: Power Latch Control

```c
// ⚠️ DO NOT MODIFY THIS LINE ⚠️
HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);   // Activate Latch

// Removing or changing this = board won't stay powered
// Moving it earlier = board might not be ready
// Moving it later = might timeout before latch
```

#### ❌ DANGER ZONE 2: BOARD_VARIANT Pin Definitions

```c
// ⚠️ DO NOT CHANGE UNLESS YOU KNOW YOUR EXACT PCB REVISION ⚠️
#if BOARD_VARIANT == 1
  #define OFF_PIN GPIO_PIN_15
  #define OFF_PORT GPIOC
#endif

// Wrong pins = board won't boot or stay powered
```

#### ❌ DANGER ZONE 3: Motor Control Timing

```c
// In BLDC_controller.c - ⚠️ EXPERT ONLY ⚠️
// Modifying PWM frequencies, commutation timing, or FOC parameters
// can damage motor windings or MOSFETs
```

#### ❌ DANGER ZONE 4: ADC/DMA Configuration

```c
// In setup.c - ⚠️ EXPERT ONLY ⚠️
// DMA handles high-speed data transfer for motor control
// Incorrect configuration = unstable motor control
```

---

## Debugging Techniques

### 1. Verify BOARD_VARIANT at Compile Time

Add to `main.c` before `main()`:

```c
// Compile-time assertion
#if BOARD_VARIANT != 1
  #error "Wrong BOARD_VARIANT! This firmware is for variant 1 hardware."
#endif

// Optional: Print variant info
#ifdef DEBUG
  #pragma message "Compiling for BOARD_VARIANT = 1"
  #pragma message "OFF_PIN = PC15, BUTTON_PIN = PB9"
#endif
```

### 2. Add LED Blink During Boot

```c
// In main(), before power latch
for (int i = 0; i < 5; i++) {
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, i % 2);
  HAL_Delay(50);
}
// If you see LED blinking, firmware is running before latch
```

### 3. Check Pin State with Multimeter

**During boot (button held):**
- Measure PC15: Should be LOW before latch line, HIGH after
- Measure PB9: Should be HIGH (3.3V) when button pressed
- Measure PA5: Should stay LOW (variant 1 doesn't use it)

**If PC15 never goes HIGH:**
- Wrong BOARD_VARIANT compiled in
- Power latch line not reached (code crash before it)
- GPIO initialization failed

---

## Hardware PCB Differences

### Why Two Variants Exist

**Variant 0 (Earlier PCB Revision):**
- Original mainboard design
- Power button on PA1
- Power latch on PA5
- Probably earlier manufacturing batch

**Variant 1 (Later PCB Revision):**
- Revised PCB layout
- Moved button to PB9 (better routing?)
- Moved latch to PC15 (conflict resolution?)
- Your hardware!

**How to identify:**
1. Trace PCB copper from button → MCU pin
2. Trace PCB copper from power circuit → MCU pin  
3. Match to pinout diagrams
4. Or: Try variant 0, if board dies immediately → you have variant 1!

---

## Appendix: STM32 GPIO Basics

### GPIO Ports and Pins

```
STM32F103 has 5 GPIO ports (on 64-pin package):
├── GPIOA: PA0-PA15 (16 pins)
├── GPIOB: PB0-PB15 (16 pins)
├── GPIOC: PC13-PC15 (3 pins on this package)
├── GPIOD: PD0-PD1 (2 pins)
└── GPIOE: Not available on 64-pin

Pin naming: GPIO{Port}{Number}
- PA5 = Port A, Pin 5
- PC15 = Port C, Pin 15
- PB9 = Port B, Pin 9
```

### GPIO Registers (Simplified)

```c
// Each GPIO port has these registers:
typedef struct {
  volatile uint32_t CRL;      // Config Low (pins 0-7)
  volatile uint32_t CRH;      // Config High (pins 8-15)
  volatile uint32_t IDR;      // Input Data Register (read pins)
  volatile uint32_t ODR;      // Output Data Register (write pins)
  volatile uint32_t BSRR;     // Bit Set/Reset Register (atomic set)
  volatile uint32_t BRR;      // Bit Reset Register (atomic clear)
  volatile uint32_t LCKR;     // Lock Register
} GPIO_TypeDef;

// Base addresses:
#define GPIOA ((GPIO_TypeDef*) 0x40010800)
#define GPIOB ((GPIO_TypeDef*) 0x40010C00)
#define GPIOC ((GPIO_TypeDef*) 0x40011000)
```

### Reading a Pin

```c
// High-level (HAL):
uint8_t state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

// Low-level (register):
uint8_t state = (GPIOB->IDR & (1 << 9)) ? 1 : 0;

// What happens:
// 1. Read GPIOB Input Data Register (32-bit value)
// 2. Mask bit 9: (value & 0x00000200)
// 3. If bit is 1 → pin is HIGH (3.3V)
// 4. If bit is 0 → pin is LOW (0V)
```

### Writing a Pin

```c
// High-level (HAL):
HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

// Low-level (register - atomic):
GPIOC->BSRR = (1 << 15);  // Set bit 15

// Or to clear:
GPIOC->BSRR = (1 << (15 + 16));  // Reset bit 15

// Why BSRR? Atomic operation - no read-modify-write
// Safe for interrupts and multi-threaded access
```

---

## Summary

**BOARD_VARIANT is not just a configuration option** - it's a **critical hardware mapping** that must match your physical PCB layout.

**Key Takeaways:**
1. ✅ Variant 1 maps OFF_PIN to PC15, BUTTON_PIN to PB9
2. ✅ Power latch MUST be activated on correct pin or board dies
3. ✅ Button MUST be read from correct pin or input doesn't work
4. ✅ Wrong variant = immediate boot failure (no power latch)
5. ✅ Safe to add features AFTER latch activation
6. ⚠️ Never modify power-critical GPIO operations without testing

**Your Configuration (Working):**
```c
#define BOARD_VARIANT 1
// Results in:
// OFF_PIN = GPIO_PIN_15 (PC15)
// BUTTON_PIN = GPIO_PIN_9 (PB9)
// ✅ Power latch works
// ✅ Button detection works
// ✅ Robot operational
```

---

**End of BOARD_VARIANT Deep Dive**
