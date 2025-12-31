# Hardware Documentation

**Complete wiring diagrams and assembly instructions.**

---

## 📁 What's in This Folder

```
hardware/
├── wiring/
│   ├── uart_wiring.md        # UART connections (Pi ↔ Hoverboard)
│   ├── power_wiring.md       # Power distribution
│   └── sensor_wiring.md      # Sensor connections
├── schematics/
│   └── pinouts.md            # All connector pinouts
├── docs/
│   ├── ASSEMBLY.md           # Step-by-step assembly
│   └── BOM.md                # Bill of materials
└── README.md                 # This file
```

---

## 🔌 Quick Wiring Reference

### UART Connection (Pi ↔ Hoverboard)

**Use RIGHT sideboard connector (USART3)**

```
Raspberry Pi 4          Hoverboard Right Sideboard
--------------          -------------------------
Pin 6  (GND)      →     Pin 2 (GND)
Pin 8  (GPIO14)   →     Pin 4 (RX)
Pin 10 (GPIO15)   →     Pin 3 (TX)
```

**Critical Notes:**
- ✅ RIGHT sideboard (USART3 is 5V tolerant)
- ✅ TX→RX crossover (Pin 8 → Pin 4)
- ✅ RX→TX crossover (Pin 10 → Pin 3)
- ✅ Common ground connection

**Why right sideboard?**
USART3 pins are 5V tolerant, safe for Pi's 3.3V GPIO

---

## ⚡ Power Connections

### Hoverboard Battery
- **Voltage:** 36V nominal (10S Li-ion)
- **Charged:** ~42V
- **Low:** ~34V
- **Dead:** ~30V

### Raspberry Pi Power
**Options:**
1. **Separate 5V supply** (recommended)
   - USB-C power adapter
   - Isolated from hoverboard

2. **Buck converter from 36V**
   - DC-DC converter (36V → 5V, 3A minimum)
   - Add fuse protection

**DO NOT** connect Pi directly to 36V battery!

---

## 🔍 RPLidar A1 Connection

**Power:** 5V, 500mA (from Pi USB or separate supply)  
**Data:** USB to Pi

```
RPLidar A1              Raspberry Pi 4
----------              --------------
USB connector     →     USB 3.0 port
```

**Mounting:**
- Height: ~25cm above ground
- Position: Center of robot
- Clearance: 360° clear view

---

## 🛠️ Bill of Materials

| Component | Part Number | Qty | Notes |
|-----------|-------------|-----|-------|
| Hoverboard | Generic 6.5" | 1 | STM32F103 mainboard |
| Raspberry Pi 4 | 8GB | 1 | Ubuntu 24.04 |
| RPLidar A1 | RPLIDAR A1M8 | 1 | 360° laser scanner |
| ST-Link V2 | Generic | 1 | For firmware flashing |
| Dupont Wires | F-F 20cm | 10 | UART connections |
| Battery | 36V 10S | 1 | 4.4Ah+ recommended |
| Power Supply | 5V 3A USB-C | 1 | For Raspberry Pi |

**Optional:**
- Buck converter (36V→5V)
- Voltage display
- Emergency stop button
- IMU (MPU6050)

**Full BOM:** See `docs/BOM.md`

---

## 📐 Assembly Steps

1. **Mount Raspberry Pi** on hoverboard platform
2. **Connect UART** (right sideboard, see above)
3. **Mount RPLidar** (centered, elevated)
4. **Connect power** (separate 5V for Pi)
5. **Flash firmware** (see `../firmware/README.md`)
6. **Configure Pi** (see `../raspberry-pi/README.md`)
7. **Test serial** communication
8. **Build ROS workspace** (see `../ros2_ws/README.md`)

**Detailed guide:** See `docs/ASSEMBLY.md`

---

## 🔧 Tools Needed

- Screwdrivers (Phillips, flathead)
- Wire strippers
- Multimeter
- Zip ties / cable management
- Double-sided tape / mounting hardware
- ST-Link programmer

---

## ⚠️ Safety

### Before Power On
- [ ] Check all connections
- [ ] Verify voltage levels
- [ ] Ensure UART crossover correct
- [ ] Wheels off ground for testing
- [ ] Battery voltage >36V

### During Operation
- [ ] Clear area around robot
- [ ] Emergency stop ready (battery disconnect)
- [ ] Monitor battery voltage
- [ ] Watch for overheating

---

## 📸 Photos and Diagrams

*Photos and detailed diagrams to be added*

- Overhead view showing component placement
- UART connection close-up
- Power distribution diagram
- Complete assembled robot

---

## 📚 Documentation

- **`wiring/uart_wiring.md`** - Detailed UART connections
- **`wiring/power_wiring.md`** - Power system design
- **`wiring/sensor_wiring.md`** - All sensor connections
- **`schematics/pinouts.md`** - Connector pinouts
- **`docs/ASSEMBLY.md`** - Step-by-step assembly
- **`docs/BOM.md`** - Complete parts list with links

---

## ✅ Verification

After assembly:

- [ ] Continuity test on all connections
- [ ] Voltage measurements correct
- [ ] No shorts to ground
- [ ] UART loopback test
- [ ] Power on without load
- [ ] Firmware responds to serial
- [ ] Motors move correctly

---

**Hardware complete! Verify all connections before powering on.**

Next: Flash firmware (see `../firmware/README.md`)
