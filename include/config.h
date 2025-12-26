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
const unsigned long CAL_BREAK_IN_STROKES = 10000;

// ============================================================================
// AUTO-CALIBRATION SETTINGS
// ============================================================================
// Pulse Width Search Range (ms)
const unsigned long CAL_PULSE_MIN = 80;
const unsigned long CAL_PULSE_MAX = 110;
const unsigned long CAL_PULSE_STEP = 10;

// Pause Duration Search Range (ms)
// Binary Search Range: [MIN, MAX]
const unsigned long CAL_PAUSE_MIN = 240;     // Lower bound for binary search
const unsigned long CAL_PAUSE_START = 750;   // Upper bound for binary search (Max Pause - Optimization Limit)
const unsigned long CAL_PAUSE_STEP = 5;     // Resolution (not strictly used in binary search but good for reference)

// Calibration Logic
const int CAL_PRIMING_PULSES = 20;      // Pulses to pressurize hose before measuring (increased to flush warm oil)
const int CAL_TEST_PULSES = 60;         // Number of pulses to test per step
const int CAL_TARGET_DROPS_MIN = 51;    // Minimum acceptable drops for 60 pulses (Strict 1:1)
const int CAL_TARGET_DROPS_MAX = 63;    // Maximum acceptable drops for 60 pulses

// Stability Criteria
// Maximum allowed jitter (Standard Deviation / Average Interval).
// 0.05 = The drop intervals must not deviate by more than 5% from the average.
// Lower is stricter.
const float CAL_MAX_JITTER_PERCENT = 0.05;

// Safety Factor
// Added to the experimentally found minimum pause to ensure reliability.
// 1.15 = +15% safety margin
const float CAL_SAFETY_MARGIN_FACTOR = 1.00;

// Recommendation Settings
// Rounding for Pulse Width Recommendation (e.g. 5ms).
// The calculated average pulse is rounded UP to the nearest multiple of this value.
// Ensures the solenoid has enough force even if slightly warmer than average.
const int CAL_RECOMMENDATION_PULSE_ROUNDING_MS = 5;

// Physics Optimization
// Minimum Ratio of Pause to Pulse duration.
// The hose needs time to relax (Windkessel effect). 
// 3.0 means: If Pulse is 50ms, Pause must be at least 150ms.
const float CAL_ELASTICITY_RATIO = 3.5;

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
const int CAL_REPEAT_CYCLES = 5;

// Optimization Cap
// Limits the adaptive search lower bound. Even if a previous step required 600ms pause,
// we don't force the next step to start at 600ms if this cap is set to 300ms.
// Values > 300ms are often valid across different pulse widths.
const unsigned long CAL_OPTIMIZATION_LOWER_BOUND_CAP = 250;

// Solenoid Thermal Protection
// Cool-down time between Pulse Width steps to prevent overheating.
const unsigned long CAL_COOLDOWN_MS = 30000; // 30 Seconds

// Validation Run Settings
// Instead of continuous running, we simulate real-world "Bursts".
const int CAL_VALIDATION_BURST_PAUSE_SEC = 45; // Time between bursts (simulating riding)
const int CAL_VALIDATION_REPEATS = 10;          // How many bursts to test per scenario

// Priming Search Settings (Auto-Discovery)
// The system will try to find the optimal "Priming Pulse" (extra time for the first stroke)
// by testing 2-stroke bursts with increasing added time.
// We use 2 strokes because a single stroke might produce a drop but fail to pressurize 
// the system enough for the second stroke.
const int CAL_PRIMING_SEARCH_REPEATS = 3;       // How many successes needed to accept a priming value
const unsigned long CAL_PRIMING_START_MS = 0;   // Start searching at +0ms
const unsigned long CAL_PRIMING_STEP_MS = 10;   // Increase by 10ms if failed
const unsigned long CAL_PRIMING_MAX_MS = 60;    // Stop searching if we reach +60ms

// Pre-Calibration Bleed
// Runs before the first calibration cycle to flush air bubbles and warm up the oil.
const bool CAL_ENABLE_PRE_BLEED = true;
const unsigned long CAL_PRE_BLEED_DURATION_MS = 30000; // 30 Seconds
const unsigned long CAL_PRE_BLEED_PULSE_MS = 80;
const unsigned long CAL_PRE_BLEED_PAUSE_MS = 250;      // Fast pumping to force bubbles out

// PWM Soft-Start / Soft-Stop (Silent Mode)
// Instead of hard 12V pulses, we ramp the voltage up and down.
// This reduces mechanical noise ("clack") and wear.
const bool PUMP_USE_PWM = true;
const int PUMP_PWM_FREQ = 5000;      // 5 kHz is safe for most solenoids
const int PUMP_PWM_CHANNEL = 0;      // ESP32 LEDC Channel
const int PUMP_PWM_RESOLUTION = 8;   // 8-bit resolution (0-255)
const int PUMP_RAMP_UP_MS = 10;      // Soft-Start duration (Generic: 12ms)
const int PUMP_RAMP_DOWN_MS = 10;    // Soft-Stop duration (Generic: 12ms)

#endif
