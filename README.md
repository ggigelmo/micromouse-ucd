# micromouse-ucd — simulator and ESP32-C6 robot

A shared 16×16 flood-fill solver with two platforms: the mms desktop simulator and
an ESP32-C6 robot using encoders, an MPU gyro and three VL53L0X distance sensors.
The robot uses 180 mm squares and drives all three phases slowly: centre → start → centre.

**Robot setup, calibration, wiring and test sequence:** [firmware/README.md](firmware/README.md).
Upload boots idle. Physical movement remains gated until measured calibration is saved.

```sh
python3 scripts/test.py
python3 scripts/firmware.py --upload /dev/cu.usbmodem1101
```

## Layout

- `src/MazeMap`, `src/FloodFill`: unchanged shared wall storage and BFS logic.
- `src/MouseAgent`: shared three-phase solver, commits pose only after completed motion.
- `src/RobotPlatform.h`: wall, movement, stop and logging contract with explicit faults.
- `SimulatorPlatform.h`, `Main.cpp`: mms adapter; `API.h` and `API.cpp` remain unchanged.
- `firmware/`: ESP32 driver, control/calibration math and USB/BOOT interface.
- `scripts/firmware.py`: assemble shared sources and compile/upload the Arduino sketch.
- `scripts/test.py`, `tests/`: solver failure tests and five-maze mms protocol regression.
- `mazes/`: two simple fixtures and three competition layouts.
- `build.bat`: original Windows MSVC simulator build wrapper.

## Build

Requires a C++17 compiler. Two options, verified working:

**MinGW `g++`:**

```bash
g++ -std=c++17 -Wall -O2 -o mouse.exe Main.cpp API.cpp src/MazeMap.cpp src/FloodFill.cpp src/MouseAgent.cpp
```

**MSVC `cl` (from a Developer Command Prompt / after `vcvars64.bat`):**

```bat
cl /std:c++17 /EHsc /O2 /Fe:mouse.exe Main.cpp API.cpp src\MazeMap.cpp src\FloodFill.cpp src\MouseAgent.cpp
```

> Note: on a machine with Windows Smart App Control turned on, a freshly-downloaded
> MinGW build's `cc1plus.exe` may be silently blocked from running (compiles fail with
> no error output). If that happens, either use MSVC's `cl` instead, or install MinGW via
> a more widely-used distribution (e.g. MSYS2) which is more likely to already have
> reputation with Smart App Control.

## Run in mms

1. [Download mms](https://github.com/mackorone/mms#download) and launch it.
2. Click the "+" button to add a new algorithm.
3. Fill in:
   - **Name:** `flood-fill` (anything)
   - **Directory:** this repo's folder
   - **Build Command:** `cmd /c "<path to this repo>\build.bat"` (mms runs the Build/Run
     commands directly, not through a shell, so use this wrapper rather than a bare `cl`
     or `g++` command — see `build.bat`)
   - **Run Command:** the **absolute path** to `micromouse.exe`, e.g.
     `C:\path\to\micromouse-ucd\micromouse.exe` (mms's Run step doesn't use the same
     working directory as Build, so a bare `micromouse.exe` fails with "process failed to
     start")
4. Load a maze: File → Import Maze → pick `mazes/empty16.num` first, then
   `mazes/obstacle16.num`, then a real competition maze (e.g. from
   [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)).
5. Click "Run". It runs three phases in sequence — search (start → goal, discovering
   walls), return (goal → start), and a final run (start → goal again, using the now-known
   map) — then stops. Debug logs for each phase are printed to stderr, visible in mms's
   console/log panel; the Stats tab's best/current run should reflect the final run, not
   the initial search.

## Verification and limits

`python3 scripts/test.py` compiles with C++17 and checks all three phases through the
unchanged mms text protocol on all five bundled mazes. It also checks resets in every
phase, invalid sensing, unreachable goals, motion faults in every phase, and partial
U-turn failure. Motion failure must not advance the logical square; only completed
quarter-turns change heading. Calibration validity, chassis clearance, range validity,
ramps and PWM saturation have portable tests. `SANITIZE=1` adds address/undefined
behaviour sanitizers. CMake/CTest is available as an alternative for the C++ tests.

The upstream repository recorded previous GUI mms runs. The new automated regression
uses a protocol harness, not the GUI. Arduino compilation and desktop success do not
validate motor polarity, encoder scaling, sensor alignment, stopping distance, floor
accuracy, battery resets or actual maze performance. Follow the staged physical tests
in the firmware guide before running autonomously.

The mms API still terminates on a simulator collision response; it is intentionally
unchanged. Hardware reports faults through `MotionResult`, stops and requires manual
repositioning. No diagonal navigation, extra exploration passes or turn-weighted route
costs are introduced.
