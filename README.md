# micromouse-ucd — flood-fill solver (algorithm only)

This is the flood-fill maze-solving algorithm for the UCD Hack Club 16x16 Micromouse
competition, tested against [mackorone/mms](https://github.com/mackorone/mms) — a
software-only step, no hardware involved.

## Layout

- `Main.cpp` — entry point, wires `MouseAgent` up and runs it.
- `API.h` / `API.cpp` — official `mackorone/mms-cpp` stdin/stdout adapter for talking to
  the mms simulator process. Unmodified.
- `src/Direction.h` — compass heading + turn helpers.
- `src/MazeMap.{h,cpp}` — 16x16 wall storage (known vs. sensed) and goal-cell logic.
- `src/FloodFill.{h,cpp}` — BFS distance-to-goal computation and next-move selection.
- `src/MouseAgent.{h,cpp}` — the three-phase run (search → return → speed run), calls
  `API::` directly.
- `mazes/empty16.num` — boundary-only maze, for sanity-checking movement/turning.
- `mazes/obstacle16.num` — a few interior walls near the start and the goal entrance, for
  sanity-checking that the solver actually routes around obstacles.
- `mazes/c00d3p.txt`, `mazes/alljapan-015-1994-exp-fin.txt`, `mazes/apec2019.txt` — real
  competition mazes (map format, from [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles))
  for testing against actual known-solvable layouts.
- `build.bat` — Windows wrapper that loads the MSVC environment (`vcvars64.bat`) and runs
  `cl`, for use as mms's Build Command (see below): mms launches Build/Run commands
  directly rather than through a shell, so a bare `cl ...` command won't have the compiler
  on its PATH unless something first runs `vcvars64.bat` in the same process.

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
   walls), return (goal → start), and a speed run (start → goal again, using the now-known
   map) — then stops. Debug logs for each phase are printed to stderr, visible in mms's
   console/log panel; the Stats tab's best/current run should reflect the speed run, not
   the initial search.

## Local verification (already done for you)

The solver has been compiled with MSVC and smoke-tested end-to-end against a small
stand-in for the mms protocol (not mms itself): all three phases complete and reach the
goal without crashing on both `mazes/empty16.num` and `mazes/obstacle16.num`, a simulated
mid-run Reset button press (at each of the three phases, in separate runs) is correctly
detected, acknowledged, and recovered from without losing any already-learned walls, and
`FloodFill`'s BFS distances and wall-avoidance were checked against hand-built fixtures.

It has also been run for real in mms against real competition mazes:
`AAMC24Maze.txt` with the earlier single-phase (search-only) version (distance 92, 28
turns, score 132), and `alljapan-015-1994-exp-fin.txt` with the three-phase version
(best/current run: distance 144, 44 turns; total: distance 468, 154 turns; score 250.2 —
confirming the speed-run phase does produce a distinct, faster best run). The Reset-button
recovery has only been checked against the harness stand-in so far, not real mms.

Deliberately not implemented (see `CLAUDE.md` for why): diagonal movement, a
turn-weighted path cost, and extra exploration passes to map frontier cells before the
speed run. Also out of scope: recovering from a genuine wall-collision "crash" response
to `moveForward()` — see `CLAUDE.md`'s Architecture section for why that one isn't
recoverable given the vendored `API.cpp`.

## Status / handoff

This repo currently targets the mms simulator only, on purpose — no ESP32, motor, ToF,
or IMU code exists here yet. The layering is deliberately built so a hardware port later
(e.g. onto Arduino/ESP32-C6) only touches one file:

- `src/MazeMap` and `src/FloodFill` are pure logic with no I/O — nothing to change to run
  on real hardware.
- `src/MouseAgent` is the only file that calls `API::` (the mms stdin/stdout protocol).
  Porting to hardware means replacing those `API::wallFront/wallLeft/wallRight/
  moveForward/turnLeft/turnRight/setWall` calls with real sensor reads and motor/turn
  commands — the explore loop, wall bookkeeping, and flood-fill logic don't need to
  change. `API::wasReset`/`ackReset` (mms's Reset-button hook) are mms-specific and have
  no direct hardware equivalent; the hardware team will need their own recovery
  mechanism if they want the same "resume without losing known walls" behavior.
- `Main.cpp` and `API.h`/`API.cpp` are mms-specific and would be dropped/replaced
  entirely for an Arduino build (no `Arduino.h`, `delay()`, or hardware headers are used
  anywhere in `src/`, so there's nothing hardware-specific to unwind first).
