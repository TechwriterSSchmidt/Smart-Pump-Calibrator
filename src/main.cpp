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

// Onboard WS2812 RGB LED
// Usually GPIO 48 on ESP32-S3 DevKitC-1 (N16R8) boards.
// If it doesn't work, try GPIO 38.
const int RGB_LED_PIN = 48; 
const int NUM_LEDS = 1;

Adafruit_NeoPixel rgbLed(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Bounce debouncer = Bounce();

// States
enum SystemState {
  STATE_READY,
  STATE_PUMPING,
  STATE_STOPPED
};

SystemState currentState = STATE_READY;
unsigned long pumpStartTime = 0;
const unsigned long PUMP_DURATION = 5000; // 5 seconds

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

  // Initialize RGB LED
  rgbLed.begin();
  rgbLed.setBrightness(50); // Set brightness (0-255)
  
  Serial.println("Pump Volume Test Program Started");
  Serial.println("--------------------------------");
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("PSRAM Size: %d MB\n", ESP.getPsramSize() / (1024 * 1024));
  Serial.println("--------------------------------");
  Serial.println("State: READY (Green). Press button to start.");

  // Initial State
  setStatusColor(0, 255, 0); // Green = Ready
}

void loop() {
  debouncer.update();

  switch (currentState) {
    case STATE_READY:
      // Green LED is already set
      if (debouncer.fell()) { // Button pressed (transition from HIGH to LOW)
        Serial.println("Button Pressed -> Starting Pump...");
        currentState = STATE_PUMPING;
        pumpStartTime = millis();
        digitalWrite(PUMP_PIN, HIGH);
        setStatusColor(255, 255, 0); // Yellow = Pump Active
      }
      break;

    case STATE_PUMPING:
      // Check timer
      if (millis() - pumpStartTime >= PUMP_DURATION) {
        Serial.println("Time limit reached -> Stopping Pump...");
        digitalWrite(PUMP_PIN, LOW);
        currentState = STATE_STOPPED;
        setStatusColor(255, 0, 0); // Red = Pump Stopped
        Serial.println("State: STOPPED (Red). Press button to reset.");
      }
      // Optional: Allow manual stop with button?
      /*
      if (debouncer.fell()) {
        Serial.println("Manual Stop...");
        digitalWrite(PUMP_PIN, LOW);
        currentState = STATE_STOPPED;
        setStatusColor(255, 0, 0);
      }
      */
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
