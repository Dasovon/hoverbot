/**
  * HOVERBOT WORKING CONFIGURATION
  * 
  * This is the complete, tested config.h for the hoverboard controller.
  * Flash this configuration for immediate operation.
  * 
  * Hardware: STM32F103RCT6 single mainboard
  * Control: UART serial via USART3 (right sideboard)
  * Mode: Tank steering (differential drive)
  * 
  * CRITICAL SETTINGS:
  * - BOARD_VARIANT must be 1 for power button to work
  * - TANK_STEERING must be defined for differential drive
  * - USART3 on right sideboard is 5V tolerant (safe for Pi 3.3V GPIO)
  */

#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

/* ================= BOARD VARIANT ================= */
/**
 * CRITICAL: Must be 1 for proper power button behavior!
 * 
 * BOARD_VARIANT controls pin mapping for:
 * - Power button latch circuit
 * - Buzzer control  
 * - Charger detection
 * 
 * Symptom if wrong: Power button won't latch, board shuts off immediately
 */
#define BOARD_VARIANT           1


/* ================= CONTROL VARIANT ================= */
/**
 * Enable USART serial control mode
 * This enables external control via UART protocol
 */
#define VARIANT_USART


/* ================= SERIAL COMMUNICATION ================= */
/**
 * USART3 Configuration (Right Sideboard Connector)
 * 
 * Why USART3:
 * - 5V tolerant (safe for Raspberry Pi 3.3V GPIO)
 * - More accessible wiring on right side
 * 
 * Pin connections:
 * - USART3 TX → Pi GPIO15 (RX)
 * - USART3 RX → Pi GPIO14 (TX)
 * - GND → Pi GND
 */
#define CONTROL_SERIAL_USART3   0      // Enable control via USART3
#define FEEDBACK_SERIAL_USART3         // Enable telemetry feedback


/* ================= SERIAL PROTOCOL ================= */
/**
 * Protocol Settings
 * 
 * Packet format (8 bytes, little-endian):
 *   uint16_t start    = 0xABCD
 *   int16_t  steer    = -1000 to +1000 (right wheel in TANK mode)
 *   int16_t  speed    = -1000 to +1000 (left wheel in TANK mode)
 *   uint16_t checksum = start XOR steer XOR speed
 * 
 * Baud rate: 115200
 * Update rate: ~50 Hz recommended
 * Timeout: 800ms (triggers safety beep and motor stop)
 */
#define SERIAL_START_FRAME      0xABCD
#define SERIAL_TIMEOUT          160     // x5ms = 800ms timeout


/* ================= UART SETTINGS ================= */
#define USART3_BAUD             115200
#define USART3_WORDLENGTH       UART_WORDLENGTH_8B


/* ================= DRIVE MODE ================= */
/**
 * Tank Steering (Differential Drive)
 * 
 * With TANK_STEERING defined:
 * - steer input controls right wheel directly
 * - speed input controls left wheel directly
 * - No mixing, direct wheel speed control
 * 
 * This is required for ROS 2 differential drive robots
 * 
 * Command examples:
 *   Forward:  steer=500, speed=500
 *   Backward: steer=-500, speed=-500
 *   Turn right: steer=500, speed=200
 *   Spin right: steer=500, speed=-500
 */
#define TANK_STEERING


/* ================= INPUT CONFIGURATION ================= */
/**
 * Input range for TANK_STEERING mode
 * 
 * Format: TYPE, MIN, MID, MAX, DEADBAND
 * 
 * Type 3 = auto-detect input type
 * Range: -1000 to +1000 (full motor speed range)
 * Deadband: 0 (no deadband for precise control)
 */
#define PRI_INPUT1              3, -1000, 0, 1000, 0  // Right wheel (steer)
#define PRI_INPUT2              3, -1000, 0, 1000, 0  // Left wheel (speed)


/* ================= MOTOR LIMITS ================= */
/**
 * Safety limits for motor operation
 * Adjust these based on your application
 */
#define MOTOR_AMP_LIM           15      // Motor current limit [A] (default: 15A)
#define MAX_SPEED               1000    // Max motor speed in RPM


/* ================= FIELD WEAKENING ================= */
/**
 * Field weakening for higher top speeds
 * Leave disabled for indoor navigation (not needed)
 */
// #define FIELD_WEAK_ENA       0       // Uncomment to enable
// #define FIELD_WEAK_HI        1000
// #define FIELD_WEAK_LO        500  
// #define FIELD_WEAK_MAX       5


/* ================= ADDITIONAL SETTINGS ================= */
/**
 * Battery and system settings
 */
#define BAT_CALIB_REAL_VOLTAGE  36.0    // Battery voltage when fully charged [V]
#define BAT_CALIB_ADC           1704    // ADC value at full charge
#define BAT_LOW_LVL1            34.0    // Low battery warning level 1 [V]
#define BAT_LOW_LVL2            32.0    // Low battery warning level 2 [V]
#define BAT_LOW_DEAD            30.0    // Battery dead, disable motors [V]


/* ================= DEBUG OPTIONS ================= */
/**
 * Uncomment for debugging (not needed for normal operation)
 */
// #define DEBUG_SERIAL_USART3           // Enable ASCII debug protocol
// #define DEBUG_SERIAL_PROTOCOL         // Enable parameter commands


/* ================= DISABLED FEATURES ================= */
/**
 * These control modes are disabled (VARIANT_USART is active)
 * DO NOT uncomment multiple variants simultaneously!
 */
// #define VARIANT_ADC              // Analog potentiometer control
// #define VARIANT_NUNCHUK          // Wii Nunchuk control  
// #define VARIANT_PPM              // RC PPM receiver
// #define VARIANT_PWM              // Dual PWM input
// #define VARIANT_IBUS             // FlySky iBUS
// #define VARIANT_HOVERBOARD       // Stock sideboard sensors
// #define VARIANT_HOVERCAR         // Hovercar steering
// #define VARIANT_TRANSPOTTER      // Transpotter platform


#endif // CONFIG_H

/**
 * ================= CONFIGURATION SUMMARY =================
 * 
 * Board:         Single mainboard, BOARD_VARIANT 1
 * Control:       USART3 serial (right sideboard)
 * Drive mode:    Tank steering (differential drive)
 * Protocol:      8-byte packets, 115200 baud, 50Hz
 * Timeout:       800ms (safety feature)
 * 
 * This configuration is TESTED and WORKING.
 * Flash and go - no modifications needed.
 * 
 * For protocol details, see: firmware/docs/PROTOCOL.md
 * For flashing guide, see: firmware/docs/FLASHING.md
 * ==========================================================
 */
