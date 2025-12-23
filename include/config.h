#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN CONFIGURATION
// ============================================================================
// Pump Control (MOSFET Gate)
const int PUMP_PIN = 4; 

// User Inputs
const int BUTTON_PIN = 5;       // External Button (Toggle Continuous)
const int BOOT_BUTTON_PIN = 0;  // Boot Button (Start/Stop Calibration)

// Sensors & Feedback
const int DROP_SENSOR_PIN = 6;  // IR Obstacle Sensor (Active LOW)
const int RGB_LED_PIN = 48;     // Onboard WS2812 LED
const int NUM_LEDS = 1;

// ============================================================================
// OPERATIONAL SETTINGS
// ============================================================================
// Default "Bleeding Mode" Settings
// Used on startup or if calibration fails.
// Purpose: Quickly fill the hose with oil and bleed air bubbles.
const unsigned long DEFAULT_BLEED_PULSE_MS = 60;
const unsigned long DEFAULT_BLEED_PAUSE_MS = 250;

// Debounce Settings (ms)
const int BUTTON_DEBOUNCE_MS = 25;
const int DROP_SENSOR_DEBOUNCE_MS = 50; // Increased to filter multiple triggers
const int DROP_SENSOR_MIN_WIDTH_MS = 4; // Minimum signal width to be considered a valid drop (filters noise spikes)

// ============================================================================
// AUTO-CALIBRATION SETTINGS
// ============================================================================
// Pulse Width Search Range (ms)
const unsigned long CAL_PULSE_MIN = 50;
const unsigned long CAL_PULSE_MAX = 150;
const unsigned long CAL_PULSE_STEP = 10;

// Pause Duration Search Range (ms)
// We search downwards from START to MIN to find the fastest stable speed.
const unsigned long CAL_PAUSE_START = 1200;
const unsigned long CAL_PAUSE_MIN = 200;
const unsigned long CAL_PAUSE_STEP = 100;

// Calibration Logic
const int CAL_PRIMING_PULSES = 10;      // Pulses to pressurize hose before measuring
const int CAL_TEST_PULSES = 50;         // Number of pulses to test per step
const int CAL_TARGET_DROPS_MIN = 48;    // Minimum acceptable drops for 50 pulses
const int CAL_TARGET_DROPS_MAX = 52;    // Maximum acceptable drops for 50 pulses

// Stability Criteria
// Maximum allowed jitter (Standard Deviation / Average Interval).
// 0.15 = The drop intervals must not deviate by more than 15% from the average.
// Lower is stricter.
const float CAL_MAX_JITTER_PERCENT = 0.15;

// Safety Factor
// Added to the experimentally found minimum pause to ensure reliability.
// 1.15 = +15% safety margin
const float CAL_SAFETY_MARGIN_FACTOR = 1.15;

#endif
