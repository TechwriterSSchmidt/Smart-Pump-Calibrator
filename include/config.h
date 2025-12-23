#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pulse Configuration (pressure buildup in 1.2m tube)
// History of Observations:
// | Pulse (ms) | Pause (ms) | Cycle (ms) | Observation                                      |
// |------------|------------|------------|--------------------------------------------------|
// | --         | --         | --         | Data cleared for auto-calibration                |

const unsigned long PULSE_DURATION_MS = 75;
const unsigned long PAUSE_DURATION_MS = 1125;

#endif
