# Repository guidance

This repository now includes the user-requested ESP32-C6 hardware port and the original
mms simulator. Read README.md and firmware/README.md for commands and calibration.

- Keep MazeMap/FloodFill shared and free of Arduino or simulator I/O.
- MouseAgent uses RobotPlatform. Commit position only after successful translation,
  heading only after each successful quarter-turn. A fault stops the phase/run.
- API.cpp/API.h are vendored mms files: leave them unchanged. Simulator stdout is
  exclusively protocol traffic; diagnostics use stderr.
- Hardware defaults: 180 mm cells, 16×16, all three phases at 60 mm/s and 35 degrees/s,
  170 PWM cap, both motor directions inverted. Do not invent encoder calibration.
- Hardware motion requires explicit commands, confirmed calibration saved in NVS and
  manual placement acknowledgement. Boot and uploads must remain idle. No timed blind
  navigation, no auto-start after a fault. Keep BOOT and serial stop responsive.
- Run python3 scripts/test.py for regression; python3 scripts/firmware.py for ESP32
  compilation. Physical test results must be recorded separately and never inferred
  from a successful build. See firmware/README.md for the staged validation sequence.
