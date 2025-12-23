# Pump Volume Test (ESP32-S3)

A small test program to control a pump via a MOSFET using an ESP32-S3 DevKit.

## Hardware
*   **Board**: ESP32-S3 DevKitC-1 (N16R8)
*   **Pump**: [12V Pulse Metering Pump (Webasto/Eberspacher compatible)](https://de.aliexpress.com/item/1005010375479436.html)
    *   **Type**: Pulse Metering Pump for Diesel Heaters (2KW-8KW)
    *   **Voltage**: 12V DC
    *   **Flow Rate (Nominal)**: approx. 0.022 ml - 0.028 ml per stroke (22ml - 28ml / 1000 pulses)
    *   **Frequency Range**: 1 Hz - 5 Hz (Typical application range)
    *   **Application**: Chain Oiler (pumping chain oil through 1.2m tube)
*   **MOSFET Module**: HW-197 (4-Channel MOSFET)
    *   **Supply Voltage**: 5V
    *   **Load Voltage**: 12V (Pump)
    *   **Control Pin**: GPIO 4
*   **External Button**: GPIO 5 (to GND)
*   **Drop Sensor**: IR Obstacle Sensor (LM393) on GPIO 6
*   **Status LED**: Onboard WS2812 (GPIO 48)

## Functionality
1.  **Ready (Green)**: Waiting for input.
2.  **Active (Yellow)**: Pump pulses continuously based on configured timing.
3.  **Stopped (Red)**: Pump stopped. Waiting for restart.
4.  **Calibration (Blue)**: Runs an automated test to find optimal pulse/pause settings.

## Controls
*   **External Button (GPIO 5)**: Toggles continuous pumping ON/OFF.
*   **Boot Button (GPIO 0)**: 
    *   **Short Press**: Triggers a single pump stroke.
    *   **Long Press / Mode**: Starts **Auto-Calibration Mode** (see below).

## Experiment: Resonance & Timing Calibration
We are investigating the fluid dynamics of pumping oil through a 1.2m tube. The goal is to find the "Sweet Spot" where every pump stroke results in exactly one drop, avoiding resonance where drops become asynchronous.

### Setup
*   **Sensor**: An IR Obstacle Sensor (LM393) is mounted at the tube outlet to detect falling drops.
*   **Logic**: The ESP32 counts pulses sent to the pump and drops detected by the sensor.

### Auto-Calibration Process
The firmware includes a routine to automatically determine the best parameters:
1.  **Phase 1 (Pulse Duration)**: 
    *   Iterates pulse lengths from 50ms to 150ms.
    *   Sends 50 pulses per setting with long pauses (1.5s) to let fluid settle.
    *   **Goal**: Find the minimum pulse length that reliably produces 50 drops for 50 pulses (Sync Ratio 1:1).
2.  **Phase 2 (Max Frequency)**:
    *   Uses the optimal pulse length found in Phase 1.
    *   Reduces the pause duration (increasing frequency) from 1200ms down to 500ms.
    *   **Goal**: Find the fastest frequency where the system remains stable (no resonance/missing drops).
3.  **Phase 3 (Periodicity Optimization)**:
    *   Starts with the best settings from Phase 2.
    *   Increases pulse duration slightly to see if a stronger pulse allows for even shorter pauses (higher frequency).
    *   **Goal**: Find the absolute maximum stable pumping speed by balancing pulse strength and pause duration.

### Robustness Features
To ensure the values work in real-world conditions (temperature changes, oil viscosity), the algorithm applies:
*   **Priming:** Before every measurement loop, 5 "blind" pulses are fired to pressurize the hose and ensure a drop is ready at the nozzle.
*   **Safety Margin:** The final recommended "Pause" value includes a **15% safety buffer** added to the experimentally determined limit.
*   **Cycle Optimization:** The algorithm searches for the lowest total cycle time (Pulse + Pause) to maximize flow rate.

### Results (documented in `include/config.h`)
See `include/config.h` for the history of manual and automated test results.

## Pinout Reference
*   [OceanLabz ESP32-S3 Pinout](https://www.oceanlabz.in/esp32-s3-devkit-pinout-reference/)
