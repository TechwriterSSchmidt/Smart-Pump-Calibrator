# Smart Pump Calibrator (ESP32-S3)

This firmware is an automatic analysis tool for 12V metering pumps (chain oilers). It uses **Auto-Calibration** and **Stability Analysis** to find the perfect flow settings where the pump and hydraulic system work in resonance.

## Hardware Setup
*   **Microcontroller**: ESP32-S3
*   **Pump Driver**: MOSFET Module (e.g., HW-197), 5V Logic, 12V Load.
*   **Sensor**: IR Obstacle Sensor (e.g., LM393 based), Active LOW.
*   **Pinout**:
    *   **Pump**: GPIO 4
    *   **Button**: GPIO 5 (External) / GPIO 0 (Boot)
    *   **Sensor**: GPIO 6

## Controls & Status

| Component | Action / Color | Function |
| :--- | :--- | :--- |
| **Boot Button** | Short Press | **Start / Stop Auto-Calibration** |
| **Ext. Button** | Short Press | **Start / Stop Continuous Pumping** |
| **Ext. Button** | **Long Press (5s)** | **RESET Statistics** (Flash White 3x) |
| **LED** | 🟢 **Green** | **Ready** (Idle or Pumping with valid settings) |
| **LED** | 🔵 **Blue** | **Calibrating** (Do not disturb sensor) |
| **LED** | 🟡 **Yellow** | **Continuous Mode** (Running) |
| **LED** | 🔴 **Red** | **Error / Stopped** (Sensor blocked or Calibration failed) |

## Configuration Defaults (`include/config.h`)

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **Break-In Strokes** | 7500 | Required strokes for mechanical stability |
| **Bleeding Pulse** | 70 ms | Pulse width for manual bleeding |
| **Bleeding Pause** | 300 ms | Pause duration for manual bleeding |
| **Cal. Pulse Range** | 40 - 100 ms | Range of pulse widths to test |
| **Cal. Pause Range** | 200 - 900 ms | Range of pause durations to test |
| **Test Pulses** | 50 | Number of pulses per test step |
| **Max Jitter** | 5% (0.05) | Maximum allowed instability |
| **Smart Exit** | 0.8% (0.008) | Jitter threshold for early optimization stop |
| **Safety Margin** | +15% (1.15) | Added to pause time for reliability |
| **Cool-Down** | 120 s | Rest time between test steps |

---

## How to Use

### 1. Break-In Period
New pumps need a mechanical break-in period to deliver consistent results.
*   **Requirement**: 7500 Strokes.
*   **Status**: The system tracks total strokes persistently (saved to memory).
*   **Warning**: If you try to calibrate before 7500 strokes, the system will warn you that results may drift.
*   **Action**: Run Continuous Mode until the Serial Monitor confirms `*** BREAK-IN PERIOD COMPLETE ***`.

### 2. Bleeding (Initial Setup)
When the system starts, it uses default "Bleeding" settings.
1.  Press the **External Button** to start pumping.
2.  Let it run until oil flows steadily from the nozzle without air bubbles.
3.  Press the button again to stop.

### 3. Tank Calibration (Session Stats)
To calibrate your tank level sensor or measure flow rate:
1.  Start Continuous Mode (Ext. Button).
2.  Let the pump run for a specific amount (e.g., fill a measuring cylinder).
3.  Stop Continuous Mode (Ext. Button).
4.  **Read the Log**: The Serial Monitor will display a **Session Summary** (Strokes, Drops, Ratio). Use these values to calibrate your tank capacity.

### 4. Auto-Calibration
This process finds the physical limits of your hydraulic system.
1.  Ensure the sensor is clean and the hose is full (bled).
2.  Press the **Boot Button**. The LED turns **Blue**.
3.  **Wait.** The system will perform a series of stress tests.
4.  **Watch the Serial Monitor** (115200 baud) for details.

#### Understanding the Log Output
*   `OK (Drops: 50, Jitter: 1.3%)`: **Perfect Result.** 50 drops detected, and the time gap between them varied by only 1.3%.
*   `FAIL (Drops: 49, Jitter: 17.6%)`: **Unstable.** The count was close, but the drops were irregular (oscillating). Rejected.
*   `(Quick Fail: ...)`: **Optimization.** The system detected chaos (e.g., continuous stream or no drops) early and aborted this step to save time.
*   `[Cool-Down]`: The system is actively cooling the coil. You can abort this wait with the Boot Button.

### 5. Result
When finished, the LED turns **Green**.
The Serial Monitor will display:
*   **Raw Values**: The absolute physical limit found.
*   **Recommended Settings**: The limit + **15% Safety Margin**.
The system automatically applies these recommended settings for Continuous Mode.

---

## Technical Details

### Stability Analysis (Jitter)
We measure the **Standard Deviation** of the time intervals between drops.
*   **Goal**: A steady heartbeat-like dripping.
*   **Criteria**: A configuration is only accepted if the Jitter is **< 5%**.
*   **Inertia Fix**: The algorithm ignores the very first drop interval to account for mechanical "stiction" (static friction) and startup inertia.

### Fast Binary Search & Optimization
Instead of testing every millisecond, the algorithm:
*   **Divides & Conquers**: It tests the middle of the range to quickly find limits.
*   **Learns**: It adapts the search window based on previous results.
*   **Smart Exit**: If the system finds a highly stable configuration (Jitter < 0.8%) and subsequent tests with longer pulses yield worse results, it stops early.

### Safety & Protection
*   **Auto-Clear**: If a test runs too fast and causes a continuous stream, the system pauses and waits for the sensor to clear.
*   **Thermal Protection**: A 2-minute cool-down between major steps prevents solenoid overheating and ensures consistent magnetic force.
