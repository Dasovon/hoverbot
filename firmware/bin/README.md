# Firmware Binary

The pre-compiled `firmware.bin` file would go in this directory.

## How to Get the Binary

### Option 1: Use Our Config (Recommended)

1. Clone EFeru firmware:
   ```bash
   git clone https://github.com/EFeru/hoverboard-firmware-hack-FOC.git
   cd hoverboard-firmware-hack-FOC
   ```

2. Replace their config with ours:
   ```bash
   cp /path/to/hoverbot/firmware/config/config.h Inc/config.h
   ```

3. Build with PlatformIO:
   ```bash
   pio run -e VARIANT_USART
   ```

4. Binary location:
   ```.pio/build/VARIANT_USART/firmware.bin
   ```

### Option 2: Download Pre-built (If Available)

Check releases on the HoverBot GitHub repository for pre-compiled binaries.

## Flashing

Flash using STM32 ST-LINK Utility:
- File: `firmware.bin`
- Address: `0x08000000`
- Verify: ✅ Enabled

See `../docs/FLASHING.md` for detailed instructions.

---

**Note:** Binary not included in git repository due to size and licensing.  
Build fresh with our config for best results.
