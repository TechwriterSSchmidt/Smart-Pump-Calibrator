# Smart Pump Calibrator (ESP32-S3)

This firmware is a automatic analysis tool for 12V metering pumps (chain oilers). It uses **Auto-Calibration** and **Stability Analysis** to find the perfect flow settings where the pump and hydraulic system work in resonance.

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

---

## How to Use

### 1. Break-In Period (Critical!)
New pumps need a mechanical break-in period to deliver consistent results.
*   **Requirement**: 5000 Strokes.
*   **Status**: The system tracks total strokes persistently (saved to memory).
*   **Warning**: If you try to calibrate before 5000 strokes, the system will warn you that results may drift.
*   **Action**: Run Continuous Mode until the Serial Monitor confirms `*** BREAK-IN PERIOD COMPLETE ***`.

### 2. Tank Calibration (Session Stats)
To calibrate your tank level sensor or measure flow rate:
1.  Start Continuous Mode (Ext. Button).
2.  Let the pump run for a specific amount (e.g., fill a measuring cylinder).
3.  Stop Continuous Mode (Ext. Button).
4.  **Read the Log**: The Serial Monitor will display a **Session Summary**:
    ```
    === SESSION STATISTICS ===
      Strokes: 150
      Drops:   1200
      Ratio:   8.00 Drops/Stroke
    ==========================
    ```
    Use these values to calibrate your tank capacity.

### 3. Bleeding (Initial Setup)
When the system starts, it uses default "Bleeding" settings (70ms Pulse / 300ms Pause).
1.  Press the **External Button** to start pumping.
2.  Let it run until oil flows steadily from the nozzle without air bubbles.
3.  Press the button again to stop.

### 3. Auto-Calibration (The "Magic" Button)
This process finds the physical limits of your hydraulic system.
1.  Ensure the sensor is clean and the hose is full (bled).
2.  Press the **Boot Button**. The LED turns **Blue**.
3.  **Wait.** The system will perform a series of stress tests.
4.  **Watch the Serial Monitor** (115200 baud) for details (including ETA).

#### What happens during Calibration?
The system performs a **Binary Search** to find the most stable flow rate. Even if the Serial Log looks fast or abbreviated, the following happens **before every single test step**:
1.  **Sensor Clear**: The system waits until the sensor is free of old drops.
2.  **Priming**: It fires **10 pulses** (blind) to pressurize the hose and ensure the next drop is ready.
3.  **Measurement**: It fires 50 test pulses and records the exact timing of every drop.
4.  **Optimization**: It automatically selects the configuration that offers the **lowest Jitter** (highest stability). If two settings are equally stable, it chooses the faster one.
5.  **Smart Limits**: The search optimization is capped at 450ms pause, as values above this are always considered valid.
6.  **Cool-Down**: Between major test steps (Pulse Width changes), the system pauses for **2 minutes** to let the solenoid coil cool down. This ensures consistent magnetic force and prevents overheating.

#### Understanding the Log Output
*   `OK (Drops: 50, Jitter: 1.3%)`: **Perfect Result.** 50 drops detected, and the time gap between them varied by only 1.3%.
*   `FAIL (Drops: 49, Jitter: 17.6%)`: **Unstable.** The count was close, but the drops were irregular (oscillating). Rejected.
*   `(Quick Fail: ...)`: **Optimization.** The system detected chaos (e.g., continuous stream or no drops) early and aborted this step to save time.
*   `[Cool-Down]`: The system is actively cooling the coil. You can abort this wait with the Boot Button.

### 3. Result
When finished, the LED turns **Green**.
The Serial Monitor will display:
*   **Raw Values**: The absolute physical limit found.
*   **Recommended Settings**: The limit + **15% Safety Margin**.
The system automatically applies these recommended settings for Continuous Mode.

---

## Technical Details

### 1. Stability Analysis (Jitter)
We don't just count drops. We measure the **Standard Deviation** of the time intervals between drops.
*   **Goal**: A steady heartbeat-like dripping.
*   **Criteria**: A configuration is only accepted if the Jitter is **< 5%**. This filters out resonance effects where the drop count might be correct by chance, but the flow is actually chaotic.
*   **Inertia Fix**: The algorithm ignores the very first drop interval to account for mechanical "stiction" (static friction) and startup inertia.

### 2. Fast Binary Search & Adaptive Optimization
Instead of testing every millisecond (which would take hours), the algorithm:
*   **Divides & Conquers**: It tests the middle of the range. If that fails, it knows the limit is slower. If it passes, it tries faster.
*   **Learns**: If 40ms Pulse required 300ms Pause, it won't try 50ms Pulse with 100ms Pause. It adapts the search window based on previous results.
*   **Smart Exit**: If the system has already found a highly stable configuration (Jitter < 1%) and subsequent tests with longer pulses yield worse results, it stops early to save time.

### 3. Safety Checks & Flow Management
*   **Auto-Clear**: If a test runs too fast and causes a continuous stream, the system pauses and waits for the sensor to clear before starting the next test.
*   **Thermal Protection**: The 2-minute cool-down ensures the solenoid resistance doesn't drift due to heat, keeping the calibration valid for "cold start" conditions.

## Configuration (`include/config.h`)
You can adjust the system behavior by editing `include/config.h`:
*   `CAL_BREAK_IN_STROKES`: Number of strokes for break-in (Default: 5000).
*   `CAL_COOLDOWN_MS`: Cool-down time between steps (Default: 120000ms = 2 min).
*   `CAL_SMART_EXIT_JITTER_THRESHOLD`: Jitter target for early exit (Default: 0.01 = 1%).
*   `CAL_MAX_JITTER_PERCENT`: Maximum allowed instability (Default: 0.05 = 5%).
*   **Blockage Protection**: During normal operation, if the sensor is blocked for >2 seconds, the pump performs an emergency stop.
*   **Noise Filter**: Signals shorter than 4ms are ignored to filter out electrical noise.
