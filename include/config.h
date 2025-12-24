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
const unsigned long DEFAULT_BLEED_PULSE_MS = 45;
const unsigned long DEFAULT_BLEED_PAUSE_MS = 400;

// Debounce Settings (ms)
const int BUTTON_DEBOUNCE_MS = 25;
const int DROP_SENSOR_DEBOUNCE_MS = 50; // Increased to filter multiple triggers
const int DROP_SENSOR_MIN_WIDTH_MS = 4; // Minimum signal width to be considered a valid drop (filters noise spikes)

// Break-in Period
// Number of strokes required before the pump is considered mechanically stable.
const unsigned long CAL_BREAK_IN_STROKES = 7500;

// ============================================================================
// AUTO-CALIBRATION SETTINGS
// ============================================================================
// Pulse Width Search Range (ms)
const unsigned long CAL_PULSE_MIN = 40;
const unsigned long CAL_PULSE_MAX = 100;
const unsigned long CAL_PULSE_STEP = 5;

// Pause Duration Search Range (ms)
// Binary Search Range: [MIN, MAX]
const unsigned long CAL_PAUSE_MIN = 280;     // Lower bound for binary search
const unsigned long CAL_PAUSE_START = 1000;   // Upper bound for binary search (Max Pause - Optimization Limit)
const unsigned long CAL_PAUSE_STEP = 5;     // Resolution (not strictly used in binary search but good for reference)

// Calibration Logic
const int CAL_PRIMING_PULSES = 20;      // Pulses to pressurize hose before measuring (increased to flush warm oil)
const int CAL_TEST_PULSES = 100;        // Number of pulses to test per step
const int CAL_TARGET_DROPS_MIN = 80;    // Minimum acceptable drops for 100 pulses
const int CAL_TARGET_DROPS_MAX = 120;   // Maximum acceptable drops for 100 pulses

// Stability Criteria
// Maximum allowed jitter (Standard Deviation / Average Interval).
// 0.05 = The drop intervals must not deviate by more than 5% from the average.
// Lower is stricter.
const float CAL_MAX_JITTER_PERCENT = 0.05;

// Safety Factor
// Added to the experimentally found minimum pause to ensure reliability.
// 1.15 = +15% safety margin
const float CAL_SAFETY_MARGIN_FACTOR = 1.15;

// Recommendation Settings
// Rounding for Pulse Width Recommendation (e.g. 5ms).
// The calculated average pulse is rounded UP to the nearest multiple of this value.
// Ensures the solenoid has enough force even if slightly warmer than average.
const int CAL_RECOMMENDATION_PULSE_ROUNDING_MS = 5;

// Physics Optimization
// Minimum Ratio of Pause to Pulse duration.
// The hose needs time to relax (Windkessel effect). 
// 3.0 means: If Pulse is 50ms, Pause must be at least 150ms.
const float CAL_ELASTICITY_RATIO = 5.0;

// Smart Exit Optimization
// If the best found jitter is below this threshold (e.g., 0.8%),
// and subsequent tests with longer pulses are worse, stop early.
const float CAL_SMART_EXIT_JITTER_THRESHOLD = 0.008; 

// Diminishing Returns (Trend Stop)
// Stop calibration if the result does not improve (or gets worse) for X consecutive steps.
// Example: If set to 2, and 60ms is worse than 55ms, and 65ms is worse than 55ms -> STOP.
const int CAL_MAX_CONSECUTIVE_WORSE_STEPS = 2;

// Calibration Cycles (max 10)
// How many times to repeat the full calibration process automatically.
// Useful to analyze thermal drift (Cold vs. Warm performance).
const int CAL_REPEAT_CYCLES = 10;

// Optimization Cap
// Limits the adaptive search lower bound. Even if a previous step required 600ms pause,
// we don't force the next step to start at 600ms if this cap is set to 300ms.
// Values > 300ms are often valid across different pulse widths.
const unsigned long CAL_OPTIMIZATION_LOWER_BOUND_CAP = 300;

// Solenoid Thermal Protection
// Cool-down time between Pulse Width steps to prevent overheating.
const unsigned long CAL_COOLDOWN_MS = 120000; // 2 Minutes

#endif
