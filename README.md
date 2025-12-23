# Pump Volume Test (ESP32-S3)

This firmware controls a 12V metering pump to find the optimal settings for a chain oiler system. It features an **Auto-Calibration** mode that uses an IR drop sensor to detect flow and avoid hydraulic resonance.

## Hardware Setup

*   **Controller**: ESP32-S3 DevKitC-1
*   **Pump**: 12V Pulse Metering Pump (Webasto/Eberspacher type)
*   **Driver**: HW-197 MOSFET Module (GPIO 4)
*   **Sensor**: IR Obstacle Sensor (LM393) at the nozzle (GPIO 6)
*   **Controls**:
    *   **External Button**: GPIO 5 (to GND)
    *   **Boot Button**: GPIO 0 (Onboard)

## How to Use

### 1. Bleeding the System (Manual Mode)
Use this mode to fill the hose with oil or test the pump.
*   **Action**: Press the **External Button** (GPIO 5).
*   **Result**: The pump runs continuously.
    *   *Default Speed*: 60ms Pulse / 250ms Pause (Fast, for bleeding).
    *   *Note*: If you have successfully run calibration, it will use those optimized values instead.
*   **Stop**: Press the button again to stop.

### 2. Auto-Calibration (Fast Mode)
Use this mode to find the optimal settings where pump cycles and drop formation are **perfectly synchronized**. The goal is a steady flow with a stable Pulse-to-Drop ratio, avoiding resonance or chaotic dripping.
*   **Action**: Press the **Boot Button** (GPIO 0).
*   **Result**: The system runs a comprehensive sweep test.
    *   **Pulse Sweep**: Iterates through different pulse durations (e.g., 50ms - 150ms).
    *   **Binary Search**: Uses a smart search algorithm to quickly find the minimum stable pause duration, drastically reducing test time compared to linear searching.
    *   **Quick Fail**: Aborts a test step early if no flow is detected, saving time on invalid configurations.
    *   **Optimization**: It automatically selects the configuration that offers the **highest flow rate** (shortest cycle time) while maintaining perfect stability (Jitter < 10%).
*   **Completion**:
    *   **Success**: LED turns Green. The new values are saved for "Continuous Mode".
    *   **Failure**: LED turns Red. Defaults are restored.
*   **Abort**: Press the Boot Button again to cancel.

## LED Status Codes

| Color  | State | Description |
| :--- | :--- | :--- |
| **Green** | **Ready** | System is idle and ready for input. |
| **Yellow** | **Pumping** | Pump is running (Continuous Mode). |
| **Blue** | **Calibrating** | Auto-Calibration is in progress. |
| **Red** | **Error** | Calibration failed (no stable settings found). |

## Configuration

All adjustable parameters are located in `include/config.h`. You can change:
*   **Pins**: Pump, Buttons, Sensor, LED.
*   **Bleeding Defaults**: Speed for manual mode.
*   **Calibration Ranges**: Min/Max pulse widths and pause times.
*   **Safety Margins**: Default is +15% safety buffer on pause time.

## Technical Details

The Auto-Calibration logic ensures reliability by:
1.  **Priming**: Firing **10 pulses** before every test to pressurize the hose. The system also reports the number of drops detected during priming to verify flow.
2.  **Smart Filtering**:
    *   **Noise Rejection**: Signals shorter than 4ms are ignored (filters electrical noise).
    *   **Debouncing**: A 50ms refractory period prevents double-counting wobbly drops.
3.  **Safety Checks & Flow Management**:
    *   **Pre-Flight**: Checks if the sensor is blocked before starting.
    *   **Auto-Clear**: Waits up to 3 seconds between tests for the sensor to clear if the previous setting caused a backup.
    *   **Continuous Flow Detection**: If the sensor remains blocked for >1 second (continuous stream), the specific test step is marked as "Failed" (too fast), allowing the algorithm to recover and try a slower speed instead of aborting completely.
4.  **Safety Margin**: Adds a **15% buffer** to the calculated pause time to account for temperature or viscosity changes.
5.  **Stability Analysis**: Calculates the **Jitter** (variation in drop intervals). A setting is only accepted if the flow is steady (Jitter < 15%), ensuring true synchronization.
6.  **Efficiency**: Calculates and reports the actual **Drops per Stroke** ratio.
7.  **Search Algorithm (Binary Search)**:
    *   **Speed**: Uses a "Divide and Conquer" approach to find the limit in ~5 steps instead of ~20.
    *   **Accuracy**: Assumes physical continuity (if speed X is unstable, X+1 is also unstable).
    *   **Resolution**: Rounds results to the nearest 5ms. The **Safety Margin** compensates for any minor granularity loss.
