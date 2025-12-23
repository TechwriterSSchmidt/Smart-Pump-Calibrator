#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include <Preferences.h>
#include "config.h"

Adafruit_NeoPixel rgbLed(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Bounce debouncer = Bounce();
Bounce bootDebouncer = Bounce();
Preferences preferences;

// States
enum SystemState {
  STATE_READY,
  STATE_PUMPING,
  STATE_CALIBRATION
};

SystemState currentState = STATE_READY;
unsigned long pumpStartTime = 0;

unsigned long lastPulseSwitchTime = 0;
bool isPulseHigh = false;

// Operational Variables (Default: Bleeding Mode)
unsigned long currentPulseDuration = DEFAULT_BLEED_PULSE_MS;
unsigned long currentPauseDuration = DEFAULT_BLEED_PAUSE_MS;

// Statistics
unsigned long strokeCounter = 0;

// Drop Detection Variables
volatile unsigned long dropCount = 0;
volatile unsigned long lastDropTime = 0;
volatile unsigned long dropStartTime = 0; // For measuring signal width
unsigned long lastProcessedDropCount = 0;

// Stability Analysis
const int MAX_STORED_DROPS = 100;
volatile unsigned long dropTimestamps[MAX_STORED_DROPS];
volatile int dropTimestampIndex = 0;

// Calibration Variables
unsigned long calPulseDuration = 50;
unsigned long calPauseDuration = 2000;
int calStep = 0;
int calPulseCount = 0;
int calDropCount = 0;

// Interrupt Service Routine for Drop Sensor
// Uses CHANGE to measure signal width and filter noise
void IRAM_ATTR onDropDetected() {
  int pinState = digitalRead(DROP_SENSOR_PIN);
  unsigned long now = millis();

  if (pinState == LOW) {
    // Falling Edge: Object entered sensor (Active LOW)
    dropStartTime = now;
  } else {
    // Rising Edge: Object left sensor
    unsigned long duration = now - dropStartTime;
    
    // Filter 1: Minimum Width (filters electrical noise spikes < 4ms)
    // Filter 2: Debounce/Refractory Period (filters double-triggering on same drop)
    if (duration >= DROP_SENSOR_MIN_WIDTH_MS && (now - lastDropTime > DROP_SENSOR_DEBOUNCE_MS)) {
      dropCount++;
      lastDropTime = now;
      
      // Store timestamp for stability analysis
      if (dropTimestampIndex < MAX_STORED_DROPS) {
        dropTimestamps[dropTimestampIndex] = now;
        dropTimestampIndex++;
      }
    }
  }
}

// Helper function to set color
void setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b)); 
  rgbLed.show();
}

// Helper to check for abort (Boot Button)
bool checkAbort() {
  bootDebouncer.update();
  if (bootDebouncer.fell()) {
    Serial.println("\n!!! ABORTED BY USER !!!");
    return true;
  }
  return false;
}

// Helper: Check if sensor is clear
bool isSensorClear() {
  if (digitalRead(DROP_SENSOR_PIN) == LOW) { // LOW = Blocked
    Serial.println("\nERROR: Sensor is blocked! Clean sensor or check alignment.");
    setStatusColor(255, 0, 0); // Red
    return false;
  }
  return true;
}

// Helper to perform priming pulses (not counted)
// Returns true if aborted
bool performPriming(int pulseMs, int pauseMs) {
  Serial.printf("  [Priming %d pulses]... ", CAL_PRIMING_PULSES);
  unsigned long startDrops = dropCount;

  for (int i = 0; i < CAL_PRIMING_PULSES; i++) {
    if (checkAbort()) return true;
    digitalWrite(PUMP_PIN, HIGH);
    delay(pulseMs);
    digitalWrite(PUMP_PIN, LOW);
    
    // Break down pause into small checks
    unsigned long startPause = millis();
    while (millis() - startPause < (unsigned long)pauseMs) {
      if (checkAbort()) return true;
      delay(10);
    }
  }
  
  unsigned long primingDrops = dropCount - startDrops;
  Serial.printf("Done. (Detected %lu drops)\n", primingDrops);
  return false;
}

// Helper: Calculate Jitter (Coefficient of Variation)
// Returns 0.0 if not enough data, otherwise StdDev / Mean
float calculateJitter() {
  int count = dropTimestampIndex;
  if (count < 2) return 0.0;

  // 1. Calculate Intervals
  unsigned long intervals[MAX_STORED_DROPS];
  unsigned long sumIntervals = 0;
  int intervalCount = count - 1;

  for (int i = 0; i < intervalCount; i++) {
    intervals[i] = dropTimestamps[i+1] - dropTimestamps[i];
    sumIntervals += intervals[i];
  }

  // 2. Calculate Mean
  float mean = (float)sumIntervals / intervalCount;

  // 3. Calculate Variance
  float sumSquaredDiff = 0;
  for (int i = 0; i < intervalCount; i++) {
    float diff = intervals[i] - mean;
    sumSquaredDiff += (diff * diff);
  }
  
  // 4. Calculate StdDev and CV (Jitter)
  float variance = sumSquaredDiff / intervalCount;
  float stdDev = sqrt(variance);
  
  return stdDev / mean;
}

// Helper struct for test results
struct TestResult {
  bool success;
  unsigned long drops;
  float jitter;
  bool aborted;
};

// Helper function to test a specific configuration
TestResult testConfiguration(unsigned long pulse, unsigned long pause) {
  TestResult result = {false, 0, 0.0f, false};
  
  // 0. Pre-Test Check: Ensure Sensor is Clear
  // If the previous test caused a backup, wait for it to clear.
  if (digitalRead(DROP_SENSOR_PIN) == LOW) {
      Serial.print(" [Waiting for sensor to clear]... ");
      unsigned long waitStart = millis();
      while (digitalRead(DROP_SENSOR_PIN) == LOW) {
          if (millis() - waitStart > 3000) {
              Serial.println("TIMEOUT. Sensor clogged.");
              result.aborted = true;
              return result;
          }
          delay(10);
      }
      Serial.println("OK.");
  }

  // 1. Priming (Fast - 5 pulses)
  // We do a mini-prime here to ensure pressure is consistent for this specific timing
  for (int i = 0; i < 5; i++) {
      if (checkAbort()) { result.aborted = true; return result; }
      digitalWrite(PUMP_PIN, HIGH);
      delay(pulse);
      digitalWrite(PUMP_PIN, LOW);
      delay(pause);
  }

  // 2. Measurement Loop
  int testPulses = CAL_TEST_PULSES;
  unsigned long startTotalDrops = dropCount;
  
  // Reset Stability Data
  dropTimestampIndex = 0;
  
  for (int i = 0; i < testPulses; i++) {
    if (checkAbort()) { result.aborted = true; return result; }

    digitalWrite(PUMP_PIN, HIGH);
    delay(pulse);
    digitalWrite(PUMP_PIN, LOW);
    
    // Wait for pause duration with abort check
    unsigned long startPause = millis();
    while (millis() - startPause < pause) {
       if (checkAbort()) { result.aborted = true; return result; }
       // Safety Check: Sensor Blocked?
       // If blocked for > 1000ms, it's likely continuous flow (too fast) or a clog.
       // We treat it as "Test Failed" (need slower speed), not "System Abort".
       if (digitalRead(DROP_SENSOR_PIN) == LOW && (millis() - dropStartTime > 1000)) {
         Serial.print(" [Flow Continuous/Blocked] ");
         result.success = false; 
         result.aborted = false; // Allow search to try slower speed
         return result;
       }
       delay(10);
    }
    
    // QUICK FAIL CHECK
    // Check early (after 10 pulses) to save time on bad configs
    if (i == 9) {
        unsigned long currentDrops = dropCount - startTotalDrops;
        // Threshold: Less than 5 drops (expected ~10). 
        // If we have < 50% drops after 10 pulses, it's likely a stream or no flow.
        if (currentDrops < 5) {
            result.drops = currentDrops; // Update for log
            if (digitalRead(DROP_SENSOR_PIN) == LOW) Serial.print("(Quick Fail: Stream detected) ");
            else Serial.printf("(Quick Fail: Only %lu drops) ", currentDrops);
            return result; // success is false
        }
    }
  }
  
  // Wait a bit for last drop
  delay(500);

  // Check for blocked sensor at the end (Stream)
  if (digitalRead(DROP_SENSOR_PIN) == LOW) {
      Serial.print(" [End: Sensor Blocked/Stream] ");
  }
  
  result.drops = dropCount - startTotalDrops;
  result.jitter = calculateJitter();
  
  bool countOk = (result.drops >= CAL_TARGET_DROPS_MIN && result.drops <= CAL_TARGET_DROPS_MAX);
  bool stabilityOk = (result.jitter <= CAL_MAX_JITTER_PERCENT);
  
  result.success = countOk && stabilityOk;
  return result;
}

void runCalibrationStep() {
  if (!isSensorClear()) return;

  Serial.println("\n=== STARTING FAST AUTO-CALIBRATION (Binary Search) ===");
  Serial.printf("Features: Binary Search, Quick Fail, Stability Check (Max Jitter %.0f%%)\n", CAL_MAX_JITTER_PERCENT * 100);
  Serial.println("Press BOOT BUTTON to STOP.");
  
  unsigned long bestPulse = 0;
  unsigned long bestPause = 0;
  unsigned long bestDrops = 0;
  unsigned long minCycleTime = 99999;
  float bestJitter = 100.0; // Initialize with worst possible jitter
  
  // Optimization: Track the lowest valid pause found so far.
  // A stronger pulse (longer duration) will generally require at least as much pause as a weaker one.
  unsigned long searchLowerBound = CAL_PAUSE_MIN;

  // Time Estimation Variables
  unsigned long calibrationStartTime = millis();
  int totalSteps = (CAL_PULSE_MAX - CAL_PULSE_MIN) / CAL_PULSE_STEP + 1;
  int completedSteps = 0;

  // Iterate through reasonable pulse widths
  for (unsigned long p = CAL_PULSE_MIN; p <= CAL_PULSE_MAX; p += CAL_PULSE_STEP) {
    if (checkAbort()) goto abort_calibration;

    // Calculate Estimated Time Remaining
    String timeMsg = "";
    if (completedSteps > 0) {
        unsigned long elapsed = millis() - calibrationStartTime;
        unsigned long avgTimePerStep = elapsed / completedSteps;
        unsigned long remainingSteps = totalSteps - completedSteps;
        unsigned long remainingTimeMs = avgTimePerStep * remainingSteps;
        
        unsigned long remMin = remainingTimeMs / 60000;
        unsigned long remSec = (remainingTimeMs % 60000) / 1000;
        timeMsg = String(" [ETA: ") + remMin + "m " + remSec + "s]";
    } else {
        timeMsg = " [ETA: Calculating...]";
    }

    Serial.printf("\nTesting Pulse Width: %lu ms%s\n", p, timeMsg.c_str());
    
    unsigned long minPauseForThisPulse = 0;
    unsigned long dropsForMinPause = 0;
    float jitterForMinPause = 100.0;
    
    // BINARY SEARCH for Minimum Valid Pause
    unsigned long low = searchLowerBound;
    unsigned long high = CAL_PAUSE_START;
    unsigned long candidatePause = 0;
    
    while (low <= high) {
        if (checkAbort()) goto abort_calibration;
        
        unsigned long mid = low + (high - low) / 2;
        // Round mid to nearest 5ms step to avoid weird numbers
        mid = (mid / 5) * 5;
        if (mid < CAL_PAUSE_MIN) mid = CAL_PAUSE_MIN;

        Serial.printf("  [Range %lu-%lu] Testing Pause %lu ms... ", low, high, mid);
        
        TestResult res = testConfiguration(p, mid);
        
        if (res.aborted) goto abort_calibration;
        
        if (res.success) {
            Serial.printf("OK (Drops: %lu, Jitter: %.1f%%)\n", res.drops, res.jitter * 100);
            // This pause works! But can we go lower?
            candidatePause = mid;
            dropsForMinPause = res.drops;
            jitterForMinPause = res.jitter;
            // Try smaller pause
            if (mid == 0) break; // prevent underflow
            if (mid < 5) high = 0; else high = mid - 5; 
        } else {
            Serial.printf("FAIL (Drops: %lu, Jitter: %.1f%%)\n", res.drops, res.jitter * 100);
            // This pause is too short (unstable or bad count). Need longer pause.
            low = mid + 5;
        }
    }
    
    // Analyze result for this Pulse Width
    if (candidatePause != 0) {
      minPauseForThisPulse = candidatePause;
      unsigned long currentCycle = p + minPauseForThisPulse;
      Serial.printf("  => Valid Config Found: %lu ms / %lu ms (Cycle: %lu ms, Jitter: %.1f%%)\n", p, minPauseForThisPulse, currentCycle, jitterForMinPause * 100);
      
      // SELECTION CRITERIA: Lowest Jitter wins.
      // If Jitter is very similar (diff < 0.5%), prefer the faster one (shorter cycle).
      bool isBetter = false;
      
      if (jitterForMinPause < bestJitter - 0.005) {
          // Significantly better stability
          isBetter = true;
      } else if (abs(jitterForMinPause - bestJitter) <= 0.005) {
          // Similar stability -> prefer speed
          if (currentCycle < minCycleTime) {
              isBetter = true;
          }
      }

      if (isBetter) {
        bestJitter = jitterForMinPause;
        minCycleTime = currentCycle;
        bestPulse = p;
        bestPause = minPauseForThisPulse;
        bestDrops = dropsForMinPause;
        Serial.println("     (NEW BEST CONFIGURATION!)");
      }

      // OPTIMIZATION: Update lower bound for next pulse width
      // If 40ms pulse needs 200ms pause, 50ms pulse will likely need >= 200ms.
      if (minPauseForThisPulse > searchLowerBound) {
          searchLowerBound = minPauseForThisPulse;
          // Cap optimization at 450ms (User Request: values > 450ms are always valid, don't force search higher)
          if (searchLowerBound > 450) searchLowerBound = 450;
          Serial.printf("     (Optimization: Next search starts at %lu ms)\n", searchLowerBound);
      }

    } else {
        Serial.println("  => No valid config found for this pulse width.");
    }
    
    completedSteps++;
  }
  
  Serial.println("\n==========================================");
  Serial.println("       CALIBRATION COMPLETE");
  Serial.println("==========================================");
  
  if (bestPulse != 0) {
    Serial.printf("Raw Optimal Values (Lab Conditions):\n");
    Serial.printf("  Pulse: %lu ms\n", bestPulse);
    Serial.printf("  Pause: %lu ms\n", bestPause);
    Serial.printf("  Cycle: %lu ms\n", minCycleTime);
    
    // Safety Margin Calculation
    unsigned long safePause = (unsigned long)(bestPause * CAL_SAFETY_MARGIN_FACTOR); 
    unsigned long safeCycle = bestPulse + safePause;
    float dropsPerStroke = (float)bestDrops / (float)CAL_TEST_PULSES;
    
    Serial.println("------------------------------------------");
    Serial.printf("RECOMMENDED SETTINGS (Robust Operation):\n");
    Serial.printf("  Pulse: %lu ms\n", bestPulse);
    Serial.printf("  Pause: %lu ms  (includes +%.0f%% safety margin)\n", safePause, (CAL_SAFETY_MARGIN_FACTOR - 1.0) * 100);
    Serial.printf("  Cycle: %lu ms\n", safeCycle);
    Serial.printf("  Ratio: %.2f Drops/Stroke (Measured: %lu drops / %d pulses)\n", dropsPerStroke, bestDrops, CAL_TEST_PULSES);

    // Apply values
    currentPulseDuration = bestPulse;
    currentPauseDuration = safePause;
    Serial.println(">> SETTINGS APPLIED FOR CONTINUOUS MODE.");

  } else {
    Serial.println("FAILURE: No stable configuration found.");
    Serial.printf(">> Reverting to defaults (Bleeding Mode: %lu/%lu).\n", DEFAULT_BLEED_PULSE_MS, DEFAULT_BLEED_PAUSE_MS);
    currentPulseDuration = DEFAULT_BLEED_PULSE_MS;
    currentPauseDuration = DEFAULT_BLEED_PAUSE_MS;
    setStatusColor(255, 0, 0); // Red
  }
  Serial.println("==========================================");
  
  currentState = STATE_READY;
  setStatusColor(0, 255, 0); // Green
  return;

abort_calibration:
  Serial.println("Calibration Aborted.");
  currentState = STATE_READY;
  setStatusColor(0, 255, 0); // Green
}

void setup() {
  Serial.begin(115200);
  
  // Initialize the pump pin as an output
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Initialize Button with Pullup
  debouncer.attach(BUTTON_PIN, INPUT_PULLUP);
  debouncer.interval(BUTTON_DEBOUNCE_MS); 

  // Initialize Boot Button
  bootDebouncer.attach(BOOT_BUTTON_PIN, INPUT_PULLUP);
  bootDebouncer.interval(BUTTON_DEBOUNCE_MS);

  // Initialize Drop Sensor
  // Sensor Output is usually Active LOW (LOW when obstacle detected)
  pinMode(DROP_SENSOR_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(DROP_SENSOR_PIN), onDropDetected, CHANGE);

  // Initialize RGB LED
  rgbLed.begin();
  rgbLed.setBrightness(50); // Set brightness (0-255)
  
  // Initialize Preferences
  preferences.begin("pump-stats", false);
  strokeCounter = preferences.getULong("strokes", 0);

  Serial.println("Smart Pump Calibrator (ESP32-S3)");
  Serial.println("--------------------------------");
  Serial.printf("Total Lifetime Strokes: %lu\n", strokeCounter);
  if (strokeCounter < 5000) {
      Serial.printf("Break-in Progress: %lu / 5000 (%.1f%%)\n", strokeCounter, (float)strokeCounter / 50.0);
  } else {
      Serial.println("Break-in Period: COMPLETE");
  }
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM Size: %d MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("--------------------------------");
  Serial.println("State: READY (Green).");
  Serial.println("  - External Button (GPIO 5): Toggle Continuous Pumping (ON/OFF)");
  Serial.println("  - Boot Button (GPIO 0): Start/Stop AUTO-CALIBRATION");
  Serial.printf("  - Current Config: Pulse %lu ms / Pause %lu ms\n", currentPulseDuration, currentPauseDuration);

  // Initial State
  setStatusColor(0, 255, 0); // Green = Ready
}

void loop() {
  debouncer.update();
  bootDebouncer.update();

  switch (currentState) {
    case STATE_READY:
      // Green LED is already set
      
      // 1. Toggle ON (External Button)
      if (debouncer.fell()) { 
        Serial.println("External Button -> Starting Continuous Pumping...");
        Serial.printf("Using Config: Pulse %lu ms / Pause %lu ms\n", currentPulseDuration, currentPauseDuration);

        // Break-in Estimation
        if (strokeCounter < 5000) {
            unsigned long remaining = 5000 - strokeCounter;
            unsigned long cycleTime = currentPulseDuration + currentPauseDuration;
            unsigned long totalSeconds = (remaining * cycleTime) / 1000;
            unsigned long minutes = totalSeconds / 60;
            Serial.printf("Break-in Progress: %lu/5000. Est. Remaining Time: %lu min\n", strokeCounter, minutes);
        }

        currentState = STATE_PUMPING;
        
        // Start first pulse immediately
        digitalWrite(PUMP_PIN, HIGH);
        isPulseHigh = true;
        lastPulseSwitchTime = millis();
        
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
        if (strokeCounter == 5000) {
            Serial.println("\n*** BREAK-IN PERIOD COMPLETE (5000 Strokes) ***");
            Serial.println("The pump is now mechanically stable and ready for calibration.");
        }
        
        setStatusColor(255, 255, 0); // Yellow = Pump Active
      }
      
      // 2. Start Calibration (Boot Button)
      if (bootDebouncer.fell()) {
        Serial.println("Boot Button -> Starting Calibration...");
        currentState = STATE_CALIBRATION;
        setStatusColor(0, 0, 255); // Blue
        runCalibrationStep();
      }
      break;

    case STATE_CALIBRATION:
      // Logic is handled in runCalibrationStep() which is blocking.
      // If we return here, it means calibration finished or aborted.
      // Ensure we go back to READY if not already set.
      currentState = STATE_READY;
      setStatusColor(0, 255, 0);
      break;

    case STATE_PUMPING:
      // Check for Stop (External Button)
      if (debouncer.fell()) {
        Serial.println("External Button -> Stopping Pump...");
        digitalWrite(PUMP_PIN, LOW);
        preferences.putULong("strokes", strokeCounter); // Save on stop
        Serial.println("Stats saved.");
        currentState = STATE_READY;
        setStatusColor(0, 255, 0); // Green = Ready
        Serial.println("State: READY (Green).");
      } else {
        // Handle Pulsing Logic (Continuous)
        unsigned long currentMillis = millis();
        if (isPulseHigh) {
          if (currentMillis - lastPulseSwitchTime >= currentPulseDuration) {
            digitalWrite(PUMP_PIN, LOW);
            isPulseHigh = false;
            lastPulseSwitchTime = currentMillis;
          }
        } else {
          if (currentMillis - lastPulseSwitchTime >= currentPauseDuration) {
            digitalWrite(PUMP_PIN, HIGH);
            isPulseHigh = true;
            lastPulseSwitchTime = currentMillis;
            
            strokeCounter++;
            
            // Periodic Save & Status
            if (strokeCounter % 100 == 0) {
                preferences.putULong("strokes", strokeCounter);
                if (strokeCounter < 5000) {
                     unsigned long remaining = 5000 - strokeCounter;
                     unsigned long cycleTime = currentPulseDuration + currentPauseDuration;
                     unsigned long minutes = ((remaining * cycleTime) / 1000) / 60;
                     Serial.printf("Break-in: %lu/5000 (%lu min remaining)\n", strokeCounter, minutes);
                }
            }

            Serial.printf("Stroke Count: %lu\n", strokeCounter);
            
            if (strokeCounter == 5000) {
                Serial.println("\n*** BREAK-IN PERIOD COMPLETE (5000 Strokes) ***");
                Serial.println("The pump is now mechanically stable and ready for calibration.");
            }
          }
        }
      }
      break;

    default:
      currentState = STATE_READY;
      break;
  }
  
  // Essential for ESP32 stability: Yield to FreeRTOS background tasks (WiFi, USB, etc.)
  // Without this, timing jitter can occur due to task starvation.
  delay(1);

  // Check for new drops and print info
  if (dropCount > lastProcessedDropCount) {
    unsigned long currentDropCount = dropCount; // Capture volatile
    unsigned long currentDropTime = lastDropTime;
    
    Serial.printf(">> DROP DETECTED! Total: %lu\n", currentDropCount);
    
    lastProcessedDropCount = currentDropCount;
  }
}
