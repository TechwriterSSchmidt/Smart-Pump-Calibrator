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
| **Break-In Strokes** | 10000 | Required strokes for mechanical stability |
| **Bleeding Pulse** | 45 ms | Pulse width for manual bleeding |
| **Bleeding Pause** | 400 ms | Pause duration for manual bleeding |
| **Cal. Pulse Range** | 40 - 80 ms | Range of pulse widths to test |
| **Cal. Pause Range** | 280 - 1000 ms | Range of pause durations to test |
| **Test Pulses** | 60 | Number of pulses per test step |
| **Max Jitter** | 5% (0.05) | Maximum allowed instability |
| **Smart Exit** | 0.8% (0.008) | Jitter threshold for early optimization stop |
| **Trend Stop** | 2 Steps | Stop if results get worse consecutively |
| **Repeat Cycles** | 5 | Number of times to run calibration automatically |
| **Optimization Cap** | 300 ms | Max lower bound for adaptive search |
| **Safety Margin** | +15% (1.15) | Added to pause time for reliability |
| **Rec. Pulse Rounding** | 5 ms | Rounding step for pulse recommendation (always rounds up) |
| **Validation Duration** | 15 min | Duration of the automatic post-calibration test run |
| **Pre-Bleed Duration** | 30 s | Duration of the automatic bleeding before calibration starts |
| **Cool-Down** | 30 s | Rest time between test steps |

---

## How to Use

### 1. Break-In Period
New pumps need a mechanical break-in period to deliver consistent results.
*   **Requirement**: 10000 Strokes.
*   **Status**: The system tracks total strokes persistently (saved to memory).
*   **Warning**: If you try to calibrate before 10000 strokes, the system will warn you that results may drift.
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

### 6. Multi-Cycle Calibration (Thermal Drift)
To analyze how the pump behaves as it warms up (Thermal Drift), you can configure `CAL_REPEAT_CYCLES` in `config.h` (e.g., set to 5).
*   The system will run the full calibration process multiple times automatically.
*   It enforces a cool-down period between cycles.
*   **Final Report**: After the last cycle, a summary table is printed to the Serial Monitor, showing the drift in Pulse, Pause, and Flow Ratio over time.

### 7. Validation Run
Immediately after the final recommendation is calculated, the system automatically starts a **15-Minute Validation Run**.
*   **Purpose**: To verify that the calculated settings (Pulse + Safety Margin) remain stable over a longer period of continuous operation.
*   **Operation**: The pump runs at the recommended settings.
*   **Logging**: Every minute, a status line is printed:
    *   `Time`: Elapsed minutes.
    *   `Strokes`: Total strokes in this run.
    *   `Drops`: Total drops detected.
    *   `Ratio`: Drops per Stroke (should be close to 1.00).
    *   `Jitter`: Stability of the drop intervals (should be < 5%).
*   **Abort**: You can stop this run at any time by pressing the **Boot Button**.

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
*   **Trend Stop**: If increasing the pulse width does not improve stability for 2 consecutive steps (Diminishing Returns), the calibration stops to save time.

### Safety & Protection
*   **Auto-Clear**: If a test runs too fast and causes a continuous stream, the system pauses and waits for the sensor to clear.
*   **Thermal Protection**: A 2-minute cool-down between major steps prevents solenoid overheating and ensures consistent magnetic force.

### The Physics: Shockwaves & Resonance
This tool does more than just measure volume. It analyzes the **Hydraulic Resonance** of your specific setup.
*   **Shockwaves**: Every pump stroke sends a pressure wave through the hose.
*   **Reflections**: This wave reflects off the narrow cannula (nozzle) and travels back.
*   **Elasticity**: The rubber hose expands and contracts ("breathing"), acting as a hydraulic capacitor.

If the pump frequency clashes with these shockwaves, the flow becomes chaotic (high Jitter). This tool finds the "Sweet Spot" where the shockwaves dissipate or harmonize, ensuring a clean, reliable drop release.

### Thermal Drift Analysis (Multi-Cycle)
Solenoid pumps and oil viscosity change significantly with temperature.
*   **Cold Oil**: Thicker, higher resistance, requires longer pulses.
*   **Warm Oil**: Thinner, flows faster, might require shorter pulses or longer pauses.
*   **Coil Heat**: As the solenoid warms up, its magnetic force decreases slightly.

By running **Multiple Cycles** (configured via `CAL_REPEAT_CYCLES`), the system captures this "Thermal Drift". It allows you to see if the optimal settings shift as the system reaches operating temperature. The final report highlights these changes, ensuring you choose settings that work for both cold starts and long rides.

### Final Recommendation Strategy (Trimmed Mean)
To ensure the recommended settings are robust against outliers (e.g., a single "lucky shot" or a measurement error), the system uses a **Trimmed Mean** calculation:
1.  **Sort**: All cycle results are sorted.
2.  **Filter**: The lowest and highest values are ignored (outliers).
3.  **Average**: The remaining values are averaged.
4.  **Safety**: The Pulse is rounded UP (to ensure force), and the Pause gets a safety margin.

**Example Calculation:**

| Cycle | Pulse | Pause | Status |
| :--- | :--- | :--- | :--- |
| 1 | 40 ms | 300 ms | *Ignored (Lowest)* |
| 2 | 42 ms | 350 ms | Used |
| 3 | 45 ms | 360 ms | Used |
| 4 | 50 ms | 450 ms | *Ignored (Highest)* |

*   **Avg Pulse**: (42+45)/2 = 43.5 ms -> Rounded Up (5ms step) -> **45 ms** (Force Reserve)
*   **Avg Pause**: (350+360)/2 = 355 ms -> +15% Margin -> **408 ms** (Refill Reserve)

This ensures the pump works reliably in both cold and warm conditions without being skewed by extreme values.

### Elasticity Ratio
The system calculates an **Elasticity Ratio** to determine how much time the hose needs to relax after a pulse. This prevents pressure buildup where the hose never fully contracts.

`Minimum Pause = Pulse Width * Elasticity Ratio`

| Ratio | Hose Type | Behavior |
| :--- | :--- | :--- |
| **2.0 - 3.0** | **Hard / Short** | Stiff lines (e.g., PTFE). Pressure drops instantly. Fast cycles. |
| **4.0 - 6.0** | **Standard** | Typical PVC/Silicon hoses. Balanced damping. |
| **> 8.0** | **Soft / Long** | Soft hoses. The hose expands significantly and needs time to stop dripping. |

> **⚠️ Warning:** If the "Ratio" (Pause/Pulse) exceeds **10.0**, it strongly indicates **air bubbles** in the system (acting as a spring) or a very soft/hot hose. The **Pre-Calibration Bleed** helps to mitigate this.

*The system learns the specific ratio for your setup during calibration.*
