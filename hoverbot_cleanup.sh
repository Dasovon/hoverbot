#!/bin/bash
# HoverBot Repository Cleanup Script
# Fixes all simple issues identified in analysis

set -e  # Exit on error

echo "╔════════════════════════════════════════════════════════════╗"
echo "║        HoverBot Repository Cleanup & Fixes                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

REPO_DIR="$HOME/hoverbot"

if [ ! -d "$REPO_DIR" ]; then
    echo "Error: Repository not found at $REPO_DIR"
    exit 1
fi

cd "$REPO_DIR"

# ============================================================================
# FIX 1: Date Corrections (2025 → 2024 for past dates)
# ============================================================================
echo "🔧 Fix 1: Correcting documentation dates..."

# Fix specific known 2025 dates that should be 2024
find . -type f -name "*.md" -exec sed -i 's/Dec 27, 2025/Dec 27, 2024/g' {} \;
find . -type f -name "*.md" -exec sed -i 's/Dec 29, 2025/Dec 29, 2024/g' {} \;
find . -type f -name "*.md" -exec sed -i 's/December 27, 2025/December 27, 2024/g' {} \;
find . -type f -name "*.md" -exec sed -i 's/December 29, 2025/December 29, 2024/g' {} \;

# Fix README.md specifically
sed -i 's/*Last updated: December 27, 2025*/*Last updated: December 31, 2024*/g' README.md

echo "   ✓ Dates corrected"

# ============================================================================
# FIX 2: IP Address Updates
# ============================================================================
echo "🔧 Fix 2: Updating IP addresses to hostname..."

# Update PROJECT_STATUS.md
sed -i 's/192.168.86.20/192.168.86.33 (DHCP - use hostname: hoverbot)/g' docs/pi4/PROJECT_STATUS.md

# Update QUICK_REFERENCE if it has IP
if [ -f "docs/v3/QUICK_REFERENCE.txt" ]; then
    sed -i 's/192.168.86.20/hoverbot/g' docs/v3/QUICK_REFERENCE.txt
fi

echo "   ✓ IP addresses updated"

# ============================================================================
# FIX 3: Pi Model Corrections (Pi 5 → Pi 4)
# ============================================================================
echo "🔧 Fix 3: Fixing Raspberry Pi model references..."

# Fix hardware README
sed -i 's/Raspberry Pi 5/Raspberry Pi 4/g' hardware/README.md
sed -i 's/Pi 5/Pi 4/g' hardware/README.md

# Fix raspberry-pi README
sed -i 's/Raspberry Pi 5/Raspberry Pi 4/g' raspberry-pi/README.md
sed -i 's/Pi 5/Pi 4/g' raspberry-pi/README.md

# Fix docs
find docs/ -type f -name "*.md" -exec sed -i 's/Raspberry Pi 5/Raspberry Pi 4/g' {} \;

# Fix main README to be clear
sed -i 's/| Raspberry Pi 5 | 8GB |/| Raspberry Pi 4 | 4GB |/g' README.md

echo "   ✓ Pi model references corrected"

# ============================================================================
# FIX 4: Update Startup Script to V3
# ============================================================================
echo "🔧 Fix 4: Updating startup script to use V3 launch..."

if [ -f "scripts/hoverbot_startup.sh" ]; then
    sed -i 's/hoverbot_full_v2.launch.py/hoverbot_full_v3.launch.py/g' scripts/hoverbot_startup.sh
    echo "   ✓ Startup script updated"
else
    echo "   ⚠ Startup script not found (may already be updated)"
fi

# ============================================================================
# FIX 5: Archive Deprecated Launch Files
# ============================================================================
echo "🔧 Fix 5: Archiving deprecated launch files..."

LAUNCH_DIR="ros2_ws/src/hoverbot_bringup/launch"

if [ -d "$LAUNCH_DIR" ]; then
    # Create deprecated directory
    mkdir -p "$LAUNCH_DIR/deprecated"
    
    # Move old versions if they exist
    if [ -f "$LAUNCH_DIR/hoverbot_full.launch.py" ]; then
        mv "$LAUNCH_DIR/hoverbot_full.launch.py" "$LAUNCH_DIR/deprecated/"
        echo "   ✓ Archived hoverbot_full.launch.py (V1)"
    fi
    
    if [ -f "$LAUNCH_DIR/hoverbot_full_v2.launch.py" ]; then
        mv "$LAUNCH_DIR/hoverbot_full_v2.launch.py" "$LAUNCH_DIR/deprecated/"
        echo "   ✓ Archived hoverbot_full_v2.launch.py (V2)"
    fi
    
    if [ -f "$LAUNCH_DIR/robot_with_lidar.launch.py" ]; then
        mv "$LAUNCH_DIR/robot_with_lidar.launch.py" "$LAUNCH_DIR/deprecated/"
        echo "   ✓ Archived robot_with_lidar.launch.py (old)"
    fi
    
    # Create README in deprecated folder
    cat > "$LAUNCH_DIR/deprecated/README.md" << 'EOF'
# Deprecated Launch Files

These launch files are kept for historical reference but should not be used.

## V1 (hoverbot_full.launch.py)
- Used tmux for coordination
- Manual startup required
- Deprecated: Nov 2024

## V2 (hoverbot_full_v2.launch.py)
- Serial port scoping bug
- RPLidar buffer overflow issues
- Deprecated: Dec 2024

## Current Version
Use: `hoverbot_full_v3.launch.py` in parent directory

---

Files preserved in git history. Can be deleted if needed.
EOF
    
    echo "   ✓ Deprecated launch files archived"
else
    echo "   ⚠ Launch directory not found"
fi

# ============================================================================
# FIX 6: Create Missing Hardware Directory Structure
# ============================================================================
echo "🔧 Fix 6: Creating missing hardware documentation structure..."

if [ -d "hardware" ]; then
    mkdir -p hardware/{wiring,schematics,docs,photos}
    
    # Create placeholder README files
    cat > hardware/wiring/README.md << 'EOF'
# Wiring Documentation

Detailed connection diagrams and procedures.

## Files to Create:
- [ ] uart_wiring.md - Pi ↔ Hoverboard UART connections
- [ ] power_wiring.md - 36V battery → Buck converter → 5V distribution
- [ ] sensor_wiring.md - All sensor connections (RPLidar, cameras, IMU)

---

*To be added during physical assembly*
EOF

    cat > hardware/schematics/README.md << 'EOF'
# Schematics and Diagrams

Technical drawings and pinout references.

## Files to Create:
- [ ] pinouts.md - All connector pinouts
- [ ] wiring_diagram.png - Visual wiring overview
- [ ] power_distribution.png - Power system diagram

---

*To be added during physical assembly*
EOF

    cat > hardware/docs/README.md << 'EOF'
# Hardware Documentation

Assembly guides and bill of materials.

## Files to Create:
- [ ] ASSEMBLY.md - Step-by-step assembly with photos
- [ ] BOM.md - Complete bill of materials with links
- [ ] CALIBRATION.md - Physical measurement procedures

---

*To be added during physical assembly*
EOF

    cat > hardware/photos/README.md << 'EOF'
# Assembly Photos

Visual documentation of robot construction.

## Photos to Add:
- [ ] Component layout overview
- [ ] UART connection closeup
- [ ] Power wiring
- [ ] Sensor mounting
- [ ] Cable routing
- [ ] Completed robot (multiple angles)

---

*To be added during physical assembly*
EOF

    echo "   ✓ Hardware directory structure created"
else
    echo "   ⚠ Hardware directory not found"
fi

# ============================================================================
# FIX 7: Update README with Current Status
# ============================================================================
echo "🔧 Fix 7: Updating README with current sensor count..."

# Update README to reflect 7 sensors
sed -i 's/## ✅ Current Status (Dec 27, 2024)/## ✅ Current Status (Dec 31, 2024)/g' README.md

# Add sensors to hardware table if not present
if ! grep -q "BNO055 IMU" README.md; then
    echo "   ℹ Note: Update README.md hardware table manually to add:"
    echo "     - BNO055 IMU (I2C)"
    echo "     - RealSense D435 (USB)"
    echo "     - ELP USB Camera (USB)"
fi

echo "   ✓ README updated"

# ============================================================================
# FIX 8: Add New Documentation to Git
# ============================================================================
echo "🔧 Fix 8: Staging new documentation..."

# Stage all changes
git add -A

echo "   ✓ Changes staged"

# ============================================================================
# SUMMARY
# ============================================================================
echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                  FIXES COMPLETE! ✓                         ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Summary of changes:"
echo "  ✓ Documentation dates corrected (2025 → 2024)"
echo "  ✓ IP addresses updated to hostname"
echo "  ✓ Pi model references fixed (Pi 5 → Pi 4)"
echo "  ✓ Startup script updated to V3"
echo "  ✓ Deprecated launch files archived"
echo "  ✓ Missing hardware directories created"
echo "  ✓ README updated with current date"
echo "  ✓ All changes staged for commit"
echo ""
echo "Next steps:"
echo "  1. Review changes: git status"
echo "  2. Commit: git commit -m 'fix: Repository cleanup - dates, references, structure'"
echo "  3. Push: git push origin pi4"
echo ""
echo "Manual tasks remaining:"
echo "  - Update README.md hardware table with new sensors"
echo "  - Create wiring diagrams in hardware/ folders"
echo "  - Take assembly photos"
echo ""
