#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>
#include "config.h"

// Define the pin connected to the MOSFET gate
// Based on the provided pinout image:
// GPIO 4 is on the LEFT side, 4th pin from the top (after 3V3, 3V3, RST)
const int PUMP_PIN = 4; 

// External Button Pin
// Connect button between GPIO 5 and GND
const int BUTTON_PIN = 5;

// Boot Button (Onboard)
const int BOOT_BUTTON_PIN = 0;

// Drop Sensor Pin (IR Obstacle Sensor)
// Connect DO to GPIO 6
const int DROP_SENSOR_PIN = 6;

// Onboard WS2812 RGB LED
// Usually GPIO 48 on ESP32-S3 DevKitC-1 (N16R8) boards.
// If it doesn't work, try GPIO 38.
const int RGB_LED_PIN = 48; 
const int NUM_LEDS = 1;

Adafruit_NeoPixel rgbLed(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Bounce debouncer = Bounce();
Bounce bootDebouncer = Bounce();

// States
enum SystemState {
  STATE_READY,
  STATE_PUMPING,
  STATE_SINGLE_PULSE,
  STATE_STOPPED,
  STATE_CALIBRATION
};

SystemState currentState = STATE_READY;
unsigned long pumpStartTime = 0;
// const unsigned long PUMP_DURATION = 5000; // Removed for continuous mode

unsigned long lastPulseSwitchTime = 0;
bool isPulseHigh = false;

// Statistics
unsigned long strokeCounter = 0;

// Drop Detection Variables
volatile unsigned long dropCount = 0;
volatile unsigned long lastDropTime = 0;
unsigned long lastProcessedDropCount = 0;

// Calibration Variables
unsigned long calPulseDuration = 50;
unsigned long calPauseDuration = 2000;
int calStep = 0;
int calPulseCount = 0;
int calDropCount = 0;

// Interrupt Service Routine for Drop Sensor
void IRAM_ATTR onDropDetected() {
  unsigned long now = millis();
  // Simple debounce (20ms) to avoid multiple counts for one drop
  if (now - lastDropTime > 20) {
    dropCount++;
    lastDropTime = now;
  }
}

// Helper function to set color
void setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b)); 
  rgbLed.show();
}

// Helper to perform priming pulses (not counted)
void performPriming(int pulseMs, int pauseMs) {
  Serial.print("  [Priming 5 pulses]... ");
  for (int i = 0; i < 5; i++) {
    digitalWrite(PUMP_PIN, HIGH);
    delay(pulseMs);
    digitalWrite(PUMP_PIN, LOW);
    delay(pauseMs);
  }
  Serial.println("Done.");
}

void runCalibrationStep() {
  Serial.println("\n=== STARTING ROBUST AUTO-CALIBRATION ===");
  Serial.println("Features: Priming (5x), Cycle Optimization, Safety Margin (+15%)");
  
  unsigned long bestPulse = 0;
  unsigned long bestPause = 0;
  unsigned long minCycleTime = 99999;
  
  // Iterate through reasonable pulse widths
  for (unsigned long p = 50; p <= 150; p += 10) {
    Serial.printf("\nTesting Pulse Width: %lu ms\n", p);
    
    unsigned long minPauseForThisPulse = 0;
    
    // Search for minimum pause (downwards from 1200ms)
    for (unsigned long pause = 1200; pause >= 200; pause -= 100) {
      
      // 1. Priming
      performPriming(p, pause);
      
      // 2. Measurement Loop (50 drops)
      int testPulses = 50;
      unsigned long startTotalDrops = dropCount;
      
      for (int i = 0; i < testPulses; i++) {
        digitalWrite(PUMP_PIN, HIGH);
        delay(p);
        digitalWrite(PUMP_PIN, LOW);
        
        // Wait for pause duration
        delay(pause);
      }
      
      // Wait a bit for last drop
      delay(500);
      
      unsigned long dropsDetected = dropCount - startTotalDrops;
      Serial.printf("  -> Pause %lu ms: %lu/50 drops. ", pause, dropsDetected);
      
      if (dropsDetected >= 48 && dropsDetected <= 52) {
        Serial.println("OK.");
        minPauseForThisPulse = pause;
      } else {
        Serial.println("UNSTABLE.");
        // If we hit instability, the PREVIOUS pause was the limit.
        // Since we go downwards, the last successful one is stored in minPauseForThisPulse.
        break; 
      }
    }
    
    // Analyze result for this Pulse Width
    if (minPauseForThisPulse != 0) {
      unsigned long currentCycle = p + minPauseForThisPulse;
      Serial.printf("  => Valid Config: %lu ms / %lu ms (Cycle: %lu ms)\n", p, minPauseForThisPulse, currentCycle);
      
      if (currentCycle < minCycleTime) {
        minCycleTime = currentCycle;
        bestPulse = p;
        bestPause = minPauseForThisPulse;
      }
    }
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
    unsigned long safePause = (unsigned long)(bestPause * 1.15); // +15%
    unsigned long safeCycle = bestPulse + safePause;
    
    Serial.println("------------------------------------------");
    Serial.printf("RECOMMENDED SETTINGS (Robust Operation):\n");
    Serial.printf("  Pulse: %lu ms\n", bestPulse);
    Serial.printf("  Pause: %lu ms  (includes +15%% safety margin)\n", safePause);
    Serial.printf("  Cycle: %lu ms\n", safeCycle);
  } else {
    Serial.println("FAILURE: No stable configuration found.");
    setStatusColor(255, 0, 0); // Red
  }
  Serial.println("==========================================");
  
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
  debouncer.interval(25); // 25ms debounce time

  // Initialize Boot Button
  bootDebouncer.attach(BOOT_BUTTON_PIN, INPUT_PULLUP);
  bootDebouncer.interval(25);

  // Initialize Drop Sensor
  // Sensor Output is usually Active LOW (LOW when obstacle detected)
  pinMode(DROP_SENSOR_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(DROP_SENSOR_PIN), onDropDetected, FALLING);

  // Initialize RGB LED
  rgbLed.begin();
  rgbLed.setBrightness(50); // Set brightness (0-255)
  
  Serial.println("Pump Volume Test Program Started");
  Serial.println("--------------------------------");
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM Size: %d MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("--------------------------------");
  Serial.println("State: READY (Green).");
  Serial.println("  - External Button (GPIO 5): Toggle Continuous Pumping (ON/OFF)");
  Serial.println("  - Boot Button (GPIO 0): Start AUTO-CALIBRATION");

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
        currentState = STATE_PUMPING;
        
        // Start first pulse immediately
        digitalWrite(PUMP_PIN, HIGH);
        isPulseHigh = true;
        lastPulseSwitchTime = millis();
        
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
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
      // Logic is handled in runCalibrationStep() which is blocking for now
      break;

    case STATE_SINGLE_PULSE:
      if (millis() - pumpStartTime >= PULSE_DURATION_MS) {
        digitalWrite(PUMP_PIN, LOW);
        currentState = STATE_READY;
        setStatusColor(0, 255, 0); // Back to Green
        Serial.println("Single Stroke Complete.");
      }
      break;

    case STATE_PUMPING:
      // Check for Stop (External Button)
      if (debouncer.fell()) {
        Serial.println("External Button -> Stopping Pump...");
        digitalWrite(PUMP_PIN, LOW);
        currentState = STATE_STOPPED;
        setStatusColor(255, 0, 0); // Red = Pump Stopped
        Serial.println("State: STOPPED (Red). Press external button to restart.");
      } else {
        // Handle Pulsing Logic (Continuous)
        unsigned long currentMillis = millis();
        if (isPulseHigh) {
          if (currentMillis - lastPulseSwitchTime >= PULSE_DURATION_MS) {
            digitalWrite(PUMP_PIN, LOW);
            isPulseHigh = false;
            lastPulseSwitchTime = currentMillis;
          }
        } else {
          if (currentMillis - lastPulseSwitchTime >= PAUSE_DURATION_MS) {
            digitalWrite(PUMP_PIN, HIGH);
            isPulseHigh = true;
            lastPulseSwitchTime = currentMillis;
            
            strokeCounter++;
            Serial.printf("Stroke Count: %lu\n", strokeCounter);
          }
        }
      }
      break;

    case STATE_STOPPED:
      // Toggle ON (External Button)
      if (debouncer.fell()) {
        Serial.println("External Button -> Restarting Continuous Pumping...");
        currentState = STATE_PUMPING;
        
        // Start first pulse immediately
        digitalWrite(PUMP_PIN, HIGH);
        isPulseHigh = true;
        lastPulseSwitchTime = millis();
        
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
        setStatusColor(255, 255, 0); // Yellow = Pump Active
      }
      
      // Allow Single Stroke from Stopped state (Returns to Ready/Green)
      if (bootDebouncer.fell()) {
        Serial.println("Boot Button -> Single Stroke...");
        currentState = STATE_SINGLE_PULSE;
        pumpStartTime = millis();
        
        digitalWrite(PUMP_PIN, HIGH);
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
        setStatusColor(0, 0, 255); // Blue
      }
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
