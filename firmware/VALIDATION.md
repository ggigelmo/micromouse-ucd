# Validation — 26 September 2026

Source branch: `hardware/slow-maze-control`, based on upstream `d1e0b85`.

## Passed

- `SANITIZE=1 python3 scripts/test.py`: C++17 with strict compiler warnings and
  address/undefined-behaviour sanitizers. Three-phase solver, reset in each phase,
  movement failures in each phase, failed turns/partial U-turn, invalid sensing,
  unreachable goal, calibration/geometry validation and control math pass.
- End-to-end mms protocol harness, with no wall collisions:

  | Maze | Cell moves | Quarter-turns | Completed phases |
  |---|---:|---:|---:|
  | empty16.num | 42 | 5 | 3 |
  | obstacle16.num | 42 | 7 | 3 |
  | c00d3p.txt | 177 | 61 | 3 |
  | alljapan-015-1994-exp-fin.txt | 234 | 154 | 3 |
  | apec2019.txt | 377 | 175 | 3 |

- Arduino ESP32 3.3.12 / Pololu VL53L0X 1.3.1 compilation and verified USB upload
  to the connected ESP32-C6 using `scripts/firmware.py --upload /dev/cu.usbmodem1101`.
  Firmware uses approximately 350 KB (26% of application flash), globals 17,576 bytes (5%).
- USB diagnostics: ToF L/F/R found at 0x30/0x31/0x29; MPU found at 0x68.
  Range status 11 and `allFresh=1` observed. Stationary gyro calibration and serial
  stop acknowledged. One boot calibration failed while the robot was being handled;
  stationary recalibration restored valid readings. Diagnostic commands now report
  stopped, I2C-failed or motion/noise causes separately.
- Reported configuration includes track 80, overall span 100, front 60, rear 35,
  wall 12 (assumed), and sensor offsets L/F/R 50/45/50 mm.
- Motors stayed commanded off during this implementation's USB checks. Encoder scale
  is missing, saved calibration is false, and floor/maze movement remains disabled.
- `git diff --check`; Python syntax checks; API.h/API.cpp and MazeMap/FloodFill
  unchanged from the imported repository.

## Still required on the physical robot

| Stage | Required observation | Result |
|---|---|---|
| Encoders | Both directions/signs, measured mm/tick | Pending |
| Raised wheels | Correct inverted forward motion, min starting PWM, stop, sensors stable under motor load | Pending |
| Sensors | Team confirms left/front/right mapping, gyro +90° for a physical CCW quarter-turn | Pending |
| Geometry | Measure actual wall thickness and check pivot/centring clearance | Pending |
| Floor | Five 180 mm moves within ±10 mm; tighter centring for this chassis's pivot | Pending |
| Floor | Five left and five right turns, each within ±5° | Pending |
| Small maze | Wall classification, clearance stops, straight travel and consecutive turns | Pending |
| Battery only | BOOT hold/release, countdown cancellation, stop/restart, no resets under load | Pending |
| Full maze | Centre → start → centre, all slowly | Pending |

There were no powered motor tests during this coding task. Earlier finite wheel tests
from the conversation established motor inversion, but did not calibrate distance or
verify this new closed-loop controller. No physical accuracy result is inferred from
software simulation. CMake/CTest is provided but was not run because CMake is absent;
the direct compiler test script exercised the same shared sources instead.
