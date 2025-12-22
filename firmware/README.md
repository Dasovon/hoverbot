# Hoverboard Firmware - Quick Start

**Everything you need to flash working firmware to your hoverboard.**

---

## 📁 What's in This Folder

```
firmware/
├── config/
│   └── config.h          # Complete working configuration
├── bin/
│   └── firmware.bin      # Pre-compiled binary (ready to flash)
├── docs/
│   ├── FIRMWARE.md       # Detailed configuration guide
│   ├── FLASHING.md       # Step-by-step flashing guide
│   └── PROTOCOL.md       # Serial protocol specification
└── README.md             # This file
```

---

## 🚀 Quick Flash (5 minutes)

### What You Need
- ST-Link V2 programmer
- Windows PC with STM32 ST-LINK Utility
- Hoverboard with battery connected

### Steps

1. **Connect ST-Link to hoverboard:**
   - SWDIO → SWDIO
   - SWCLK → SWCLK
   - GND → GND
   - 3.3V → 3.3V

2. **Power on hoverboard** (hold power button)

3. **Flash in STM32 ST-LINK Utility:**
   - Target → Connect (100 kHz speed)
   - Target → Program & Verify
   - Browse to: `firmware/bin/firmware.bin`
   - Start address: `0x08000000`
   - ✅ Verify after programming
   - Click "Start"
   - **Keep holding power button!**

4. **Done!** Release button, press normally. Should beep and stay on.

**Detailed guide:** See `docs/FLASHING.md`

---

## 🔧 Custom Build (Optional)

If you want to modify settings:

1. **Clone EFeru firmware:**
   ```bash
   git clone https://github.com/EFeru/hoverboard-firmware-hack-FOC.git
   cd hoverboard-firmware-hack-FOC
   ```

2. **Replace config.h:**
   ```bash
   cp /path/to/this/repo/firmware/config/config.h Inc/config.h
   ```

3. **Build with PlatformIO:**
   ```bash
   pio run -e VARIANT_USART
   ```

4. **Flash the new binary:**
   `.pio/build/VARIANT_USART/firmware.bin`

---

## ⚙️ Configuration Highlights

Our `config.h` includes:

### Critical Settings
- ✅ `BOARD_VARIANT 1` - Power button works
- ✅ `VARIANT_USART` - Serial control enabled
- ✅ `TANK_STEERING` - Differential drive mode
- ✅ `USART3` - Right sideboard (5V tolerant)

### Serial Protocol
- **Baud:** 115200
- **Packet:** 8 bytes (start, steer, speed, checksum)
- **Checksum:** `start XOR steer XOR speed`
- **Timeout:** 800ms

### Motor Control
- **Range:** -1000 to +1000
- **Current limit:** 15A
- **Max speed:** 1000 RPM

**Full details:** See `docs/FIRMWARE.md`

---

## 🔌 Wiring to Raspberry Pi

Connect to **right sideboard** (USART3):

```
Hoverboard           Raspberry Pi
-----------          ------------
Pin 2 (GND)    →     Pin 6 (GND)
Pin 3 (TX)     →     Pin 10 (GPIO15 RX)
Pin 4 (RX)     →     Pin 8 (GPIO14 TX)
```

**Why right sideboard?** USART3 is 5V tolerant (safe for Pi's 3.3V GPIO)

---

## ✅ Verification

After flashing, you should observe:

1. **Single beep on power-up** ✅
2. **Power button latches** (board stays on) ✅
3. **Triple beep pattern** (waiting for serial commands) ✅

If power button doesn't latch, check `BOARD_VARIANT` in config.h

---

## 📡 Serial Protocol

### Command Packet (Pi → Hoverboard)
```c
uint16_t start    = 0xABCD       // Start marker
int16_t  steer    = -1000..1000  // Right wheel (TANK mode)
int16_t  speed    = -1000..1000  // Left wheel (TANK mode)
uint16_t checksum = start^steer^speed
```

### Feedback Packet (Hoverboard → Pi)
```c
uint16_t start        = 0xABCD
int16_t  cmd1         // Command echo
int16_t  cmd2         // Command echo
int16_t  speedR_meas  // Right wheel speed
int16_t  speedL_meas  // Left wheel speed
int16_t  batVoltage   // Battery (V * 100)
int16_t  boardTemp    // Temp (°C * 10)
uint16_t cmdLed       // LED command
uint16_t checksum     // XOR of all fields
```

**Full protocol:** See `docs/PROTOCOL.md`

---

## 🛠️ Troubleshooting

### Power button won't latch
- **Cause:** Wrong BOARD_VARIANT
- **Fix:** Ensure `BOARD_VARIANT 1` in config.h, rebuild, reflash

### Continuous beeping (3 beeps repeating)
- **This is normal!** Firmware waiting for serial commands
- **Fix:** Send commands from Raspberry Pi (see `raspberry-pi/` folder)

### ST-Link can't connect
- **Check:** Wiring (especially SWCLK and SWDIO)
- **Try:** Lower speed (100 kHz instead of 4 MHz)
- **Ensure:** Battery connected and power button held

### Verification failed
- **Clean:** `pio run -t clean` then rebuild
- **Check:** Correct .bin file selected
- **Try:** Different USB port

---

## 📚 Additional Documentation

- **`docs/FIRMWARE.md`** - Complete configuration reference
- **`docs/FLASHING.md`** - Detailed flashing procedure
- **`docs/PROTOCOL.md`** - Serial protocol deep dive

---

## 🔗 External Resources

- [EFeru FOC Firmware](https://github.com/EFeru/hoverboard-firmware-hack-FOC)
- [STM32F103 Datasheet](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)
- [ST-Link Download](https://www.st.com/en/development-tools/stsw-link004.html)

---

## ⚡ Quick Command Reference

```bash
# Build custom firmware
pio run -e VARIANT_USART

# Clean build
pio run -t clean

# Upload via ST-Link (if configured)
pio run -t upload
```

---

**Ready to flash! The `bin/firmware.bin` file is tested and working.**

Next step: Configure Raspberry Pi (see `../raspberry-pi/README.md`)
