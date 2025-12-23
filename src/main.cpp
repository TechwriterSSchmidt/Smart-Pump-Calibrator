#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Bounce2.h>

// Define the pin connected to the MOSFET gate
// Based on the provided pinout image:
// GPIO 4 is on the LEFT side, 4th pin from the top (after 3V3, 3V3, RST)
const int PUMP_PIN = 4; 

// External Button Pin
// Connect button between GPIO 5 and GND
const int BUTTON_PIN = 5;

// Boot Button (Onboard)
const int BOOT_BUTTON_PIN = 0;

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
  STATE_STOPPED
};

SystemState currentState = STATE_READY;
unsigned long pumpStartTime = 0;
const unsigned long PUMP_DURATION = 5000; // 5 seconds

// Pulse Configuration
const unsigned long PULSE_DURATION_MS = 150;
const unsigned long PAUSE_DURATION_MS = 850;
unsigned long lastPulseSwitchTime = 0;
bool isPulseHigh = false;

// Statistics
unsigned long strokeCounter = 0;

// Helper function to set color
void setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
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

  // Initialize RGB LED
  rgbLed.begin();
  rgbLed.setBrightness(50); // Set brightness (0-255)
  
  Serial.println("Pump Volume Test Program Started");
  Serial.println("--------------------------------");
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM Size: %d MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("--------------------------------");
  Serial.println("State: READY (Green).");
  Serial.println("  - External Button (GPIO 5): Start 5s Test Cycle");
  Serial.println("  - Boot Button (GPIO 0): Single Pump Stroke");

  // Initial State
  setStatusColor(0, 255, 0); // Green = Ready
}

void loop() {
  debouncer.update();
  bootDebouncer.update();

  switch (currentState) {
    case STATE_READY:
      // Green LED is already set
      
      // 1. Test Cycle (External Button)
      if (debouncer.fell()) { 
        Serial.println("External Button -> Starting 5s Test Cycle...");
        currentState = STATE_PUMPING;
        pumpStartTime = millis();
        
        // Start first pulse immediately
        digitalWrite(PUMP_PIN, HIGH);
        isPulseHigh = true;
        lastPulseSwitchTime = millis();
        
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
        setStatusColor(255, 255, 0); // Yellow = Pump Active
      }
      
      // 2. Single Stroke (Boot Button)
      if (bootDebouncer.fell()) {
        Serial.println("Boot Button -> Single Stroke...");
        currentState = STATE_SINGLE_PULSE;
        pumpStartTime = millis(); // Reuse variable for start time
        
        digitalWrite(PUMP_PIN, HIGH);
        strokeCounter++;
        Serial.printf("Stroke Count: %lu\n", strokeCounter);
        
        setStatusColor(0, 0, 255); // Blue = Single Pulse
      }
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
      // Check total duration
      if (millis() - pumpStartTime >= PUMP_DURATION) {
        Serial.println("Time limit reached -> Stopping Pump...");
        digitalWrite(PUMP_PIN, LOW);
        currentState = STATE_STOPPED;
        setStatusColor(255, 0, 0); // Red = Pump Stopped
        Serial.println("State: STOPPED (Red). Press external button to reset.");
      } else {
        // Handle Pulsing Logic
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
      // Wait for button to reset to READY
      if (debouncer.fell()) {
        Serial.println("Button Pressed -> Resetting to READY...");
        currentState = STATE_READY;
        setStatusColor(0, 255, 0); // Green = Ready
      }
      break;
  }
}
