# Hoverboard Firmware Configuration

Complete documentation for STM32F103 hoverboard controller firmware setup.

---

## Firmware Repository

**Source:** [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC)

**Version:** Latest (as of December 2024)

**Build System:** PlatformIO + VS Code

---

## Hardware Specifications

### Microcontroller
- **MCU:** STM32F103RCT6 / RET6
- **Architecture:** ARM Cortex-M3
- **Flash:** 256KB / 512KB
- **RAM:** 48KB / 64KB
- **Clock:** 72 MHz

### Board Type
- **Configuration:** Single mainboard (not split board)
- **Motors:** Dual BLDC with 5-wire Hall sensors
- **Power:** 36V nominal (10S Li-ion)
- **Interface:** USART3 on right sideboard connector

---

## Critical Configuration Settings

### config.h - Essential Defines

```c
/* ================= POWER MANAGEMENT ================= */
// CRITICAL: Must be 1 for proper power button behavior
#define BOARD_VARIANT 1

/* ================= CONTROL VARIANT ================= */
// Enable USART control mode
#define VARIANT_USART

/* ================= SERIAL CONFIGURATION ================= */
// Use USART3 on RIGHT sideboard (5V tolerant)
#define CONTROL_SERIAL_USART3 0
#define FEEDBACK_SERIAL_USART3

// Communication parameters
#define USART3_BAUD 115200
#define USART3_WORDLENGTH UART_WORDLENGTH_8B

// Protocol settings
#define SERIAL_START_FRAME 0xABCD
#define SERIAL_TIMEOUT 160  // ~800ms (160 * 5ms main loop)

/* ================= DRIVE MODE ================= */
// Enable tank steering for differential drive
#define TANK_STEERING

/* ================= INPUT CONFIGURATION ================= */
// Input ranges for TANK_STEERING mode
// Format: TYPE, MIN, MID, MAX, DEADBAND
#define PRI_INPUT1  3, -1000, 0, 1000, 0  // Right wheel
#define PRI_INPUT2  3, -1000, 0, 1000, 0  // Left wheel
```

---

## BOARD_VARIANT Explanation

### Why BOARD_VARIANT 1 is Required

Different hoverboard mainboards use different GPIO pin assignments for critical functions:

**BOARD_VARIANT 0 (Default):**
- Power button: Different GPIO
- Buzzer: Different GPIO
- Charger detect: Different GPIO

**BOARD_VARIANT 1 (Required for our board):**
- Correct pin mapping for power latch circuit
- Proper power button functionality
- Working buzzer control

**Symptom if wrong:** Power button doesn't latch, board immediately powers off when button released.

**Fix:** Change to `BOARD_VARIANT 1` in config.h and reflash.

---

## Serial Protocol Details

### Packet Structure

**Command Packet (Pi → Hoverboard):**
```c
typedef struct {
    uint16_t start;      // 0xABCD (little-endian)
    int16_t  steer;      // -1000 to +1000
    int16_t  speed;      // -1000 to +1000
    uint16_t checksum;   // start ^ steer ^ speed
} SerialCommand;
```

**Size:** 8 bytes total

**Byte Layout:**
```
Offset  | Field      | Type    | Bytes
--------|------------|---------|-------
0x00    | start      | uint16  | CD AB
0x02    | steer      | int16   | XX XX
0x04    | speed      | int16   | XX XX
0x06    | checksum   | uint16  | XX XX
```

### Checksum Calculation

**Algorithm:** Simple XOR of all 16-bit fields

```c
checksum = start ^ steer ^ speed
```

**Example (zero command):**
```c
start    = 0xABCD
steer    = 0x0000
speed    = 0x0000
checksum = 0xABCD ^ 0x0000 ^ 0x0000 = 0xABCD
```

**Packet bytes:** `CD AB 00 00 00 00 CD AB`

### Feedback Packet (Hoverboard → Pi)

**Structure:**
```c
typedef struct {
    uint16_t start;        // 0xABCD
    int16_t  cmd1;         // Command echo
    int16_t  cmd2;         // Command echo
    int16_t  speedR_meas;  // Right wheel speed
    int16_t  speedL_meas;  // Left wheel speed
    int16_t  batVoltage;   // Battery voltage (V * 100)
    int16_t  boardTemp;    // Board temp (°C * 10)
    uint16_t cmdLed;       // LED command
    uint16_t checksum;     // XOR of all fields
} SerialFeedback;
```

**Size:** 18 bytes total

**Update rate:** ~50 Hz

---

## Control Modes

### TANK_STEERING Mode

When `TANK_STEERING` is defined:

**Input Mapping:**
- `steer` → Right wheel speed
- `speed` → Left wheel speed

**Direct wheel control:**
```c
rightMotor = steer;   // -1000 to +1000
leftMotor  = speed;   // -1000 to +1000
```

**Example Commands:**
```c
// Forward (both wheels same speed)
steer =  500, speed =  500  → Both forward

// Backward
steer = -500, speed = -500  → Both backward

// Turn right (left faster)
steer =  200, speed =  500  → Gentle right turn

// Spin in place right
steer =  500, speed = -500  → Rotate right

// Spin in place left
steer = -500, speed =  500  → Rotate left
```

### Standard Mode (without TANK_STEERING)

**Input Mapping:**
- `steer` → Steering input
- `speed` → Forward/backward speed

**Mixed output:**
```c
rightMotor = speed - steer;
leftMotor  = speed + steer;
```

---

## Timeout and Safety

### Serial Timeout

**Timeout value:** `SERIAL_TIMEOUT = 160`  
**Main loop:** 5ms  
**Actual timeout:** 160 × 5ms = 800ms

**Behavior when timeout occurs:**
1. Motors commanded to zero
2. Beeping starts (3 short beeps repeating)
3. Continues until valid packet received

**This is intentional safety behavior!**

### Maintaining Connection

**Required:** Send commands continuously at ≥1.25 Hz (recommended 50 Hz)

**Python example:**
```python
while running:
    send_command(steer, speed)
    time.sleep(0.02)  # 50 Hz
```

---

## Build and Flash Process

### Build with PlatformIO

```bash
# Clone firmware repository
git clone https://github.com/EFeru/hoverboard-firmware-hack-FOC.git
cd hoverboard-firmware-hack-FOC

# Edit Inc/config.h with your settings

# Build all variants
pio run

# Build specific variant
pio run -e VARIANT_USART
```

**Output location:** `.pio/build/VARIANT_USART/firmware.bin`

### Flashing with ST-LINK

**Hardware setup:**
1. Connect ST-Link to hoverboard SWIM/SWD pins
2. Connect hoverboard battery
3. **Hold power button** (keep holding throughout flash)

**Software steps (Windows - STM32 ST-LINK Utility):**
1. Launch STM32 ST-LINK Utility
2. Target → Connect
3. Set connection speed: 100 kHz (more reliable than 4 MHz)
4. Target → Program & Verify
5. File path: `.pio\build\VARIANT_USART\firmware.bin`
6. Start address: `0x08000000`
7. Check "Verify after programming"
8. Click "Start"
9. Wait for completion
10. **Keep holding power button until programming complete**
11. Disconnect ST-Link
12. Release power button
13. Press power button normally - should latch

**Software steps (Linux - st-flash):**
```bash
# Install stlink tools
sudo apt install stlink-tools

# Flash firmware
st-flash --reset write firmware.bin 0x08000000

# Alternative with verification
st-flash --format binary write firmware.bin 0x08000000
```

### Troubleshooting Flash Issues

**Connection failed:**
- Check SWD wiring
- Try lower speed (100 kHz)
- Ensure battery connected
- Hold power button

**Verification failed:**
- Clean build: `pio run -t clean`
- Rebuild firmware
- Try different USB port
- Check ST-Link firmware version

**Board won't power on after flash:**
- Check `BOARD_VARIANT` setting
- Reflash with correct variant
- Verify .bin file is for correct environment

---

## Pin Mapping

### Right Sideboard Connector (USART3)

**Connector pinout (looking at board):**
```
Pin 1: VCC (not used)
Pin 2: GND
Pin 3: TX (from hoverboard)
Pin 4: RX (to hoverboard)
```

**Connection to Raspberry Pi:**
```
Hoverboard       Raspberry Pi 5
----------       --------------
Pin 2 (GND)  →   Pin 6  (GND)
Pin 3 (TX)   →   Pin 10 (GPIO15 RX)
Pin 4 (RX)   →   Pin 8  (GPIO14 TX)
```

**Why right sideboard?**
- USART3 is 5V tolerant (safer for Pi's 3.3V GPIO)
- USART2 (left sideboard) may have voltage level issues

---

## Verification and Testing

### After Flashing Checklist

1. **Power button test:**
   - Press power button
   - Should hear startup beep
   - Board should stay on after releasing button
   - ✅ Pass if board latches on

2. **Serial communication test:**
   - Connect Pi to right sideboard
   - Send zero commands at 50Hz
   - Listen for response packets
   - ✅ Pass if beeping stops and feedback received

3. **Motor response test:**
   - Wheels OFF ground
   - Send low speed commands (±100)
   - Observe wheel movement
   - ✅ Pass if wheels rotate smoothly

### Expected Behavior

**Normal operation:**
- Single beep on power-up
- Silent when receiving valid commands
- Smooth motor response
- Continuous feedback telemetry

**Fault conditions:**
- Triple beep repeating = serial timeout (no commands)
- Motors disabled = safety feature, send commands to re-enable

---

## Advanced Configuration

### Speed and Current Limits

```c
/* Maximum motor current (Amps * 100) */
#define MOTOR_AMP_LIM  15

/* Maximum motor speed (RPM) */  
#define MAX_SPEED      1000

/* Field weakening (for higher speeds) */
#define FIELD_WEAK_HI  1000
#define FIELD_WEAK_LO  500
#define FIELD_WEAK_MAX 5
```

### Input Filtering

```c
/* Input timeout (ms) */
#define INPUT_TIMEOUT  100

/* Speed deadband */
#define SPEED_DEADBAND 5

/* Filter coefficient (0-1000, higher = more filtering) */
#define FILTER_COEF    200
```

---

## Common Issues and Solutions

### Issue: Board powers off immediately

**Cause:** Wrong BOARD_VARIANT  
**Solution:** Set `BOARD_VARIANT 1` and reflash

### Issue: Continuous beeping, no motor response

**Cause:** Serial timeout - not receiving valid commands  
**Solutions:**
- Check UART wiring
- Verify baud rate (115200)
- Confirm checksum calculation
- Send commands continuously (50Hz)

### Issue: Motors jittery or unresponsive

**Cause:** Hall sensor connections or motor phase wiring  
**Solutions:**
- Check 5-wire Hall sensor connections
- Verify 3-phase motor wiring
- Test with lower speeds first

### Issue: One wheel doesn't move

**Cause:** Motor or Hall sensor fault  
**Solutions:**
- Swap motor connections to isolate issue
- Check Hall sensor 5V supply
- Test with CONTROL_ADC variant for diagnostics

---

## Firmware Variants Reference

The EFeru firmware includes multiple pre-configured variants:

- `VARIANT_ADC` - Dual analog input (potentiometers)
- `VARIANT_USART` - **Our configuration** - Serial UART control
- `VARIANT_NUNCHUK` - Wii Nunchuk control
- `VARIANT_PPM` - RC PPM receiver
- `VARIANT_PWM` - Dual PWM input
- `VARIANT_IBUS` - FlySky iBUS receiver
- `VARIANT_HOVERBOARD` - Stock hoverboard sensor boards
- `VARIANT_HOVERCAR` - Hovercar steering wheel
- `VARIANT_TRANSPOTTER` - Transpotter platform

Each variant has different config.h defaults. Always start from VARIANT_USART for serial control.

---

## Source Code References

**Key files:**
- `Inc/config.h` - Main configuration
- `Inc/util.h` - SerialCommand structure definition
- `Src/util.c` - Serial protocol implementation (line 1249)
- `Src/comms.c` - Debug protocol (different from control protocol)
- `Src/main.c` - Main control loop

**SerialCommand structure:** `Inc/util.h` lines 28-35  
**Checksum calculation:** `Src/util.c` line 1276  
**Timeout handling:** `Src/util.c` lines 1275-1291

---

## Additional Resources

- [Official Firmware Repo](https://github.com/EFeru/hoverboard-firmware-hack-FOC)
- [Firmware Wiki](https://github.com/EFeru/hoverboard-firmware-hack-FOC/wiki)
- [STM32F103 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)
- [PlatformIO Documentation](https://docs.platformio.org/)

---

**Last Updated:** December 2024  
**Firmware Version:** EFeru FOC (latest)  
**Status:** Validated and working configuration
