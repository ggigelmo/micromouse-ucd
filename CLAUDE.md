# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The flood-fill maze-solving algorithm for a 16x16 UK-standard Micromouse competition
entry. This is a **software-only** step: the code talks only to the
[mackorone/mms](https://github.com/mackorone/mms) simulator over stdin/stdout, never to
real hardware (no ESP32, motors, ToF sensors, or IMU anywhere in this repo). A separate
team is building the physical robot and will port this onto Arduino/ESP32-C6; don't add
abstractions here in anticipation of that port (see "Hardware handoff" below for why the
current layering already makes it low-effort when it happens).

Current status: validated against a real competition maze in mms (reaches the goal). It
only does a single explore-to-goal run and stops — no return-to-start or speed-run pass
yet. That's the next piece of work.

## Build

Requires a C++17 compiler. Both are confirmed working on Windows:

```bash
# MinGW g++
g++ -std=c++17 -Wall -O2 -o mouse.exe Main.cpp API.cpp src/MazeMap.cpp src/FloodFill.cpp src/MouseAgent.cpp
```

```bat
:: MSVC cl (from a Developer Command Prompt, after vcvars64.bat)
cl /std:c++17 /EHsc /O2 /Fe:mouse.exe Main.cpp API.cpp src\MazeMap.cpp src\FloodFill.cpp src\MouseAgent.cpp
```

> If MinGW's `cc1plus.exe` fails to compile with zero error output on Windows, Smart App
> Control is likely silently blocking that unsigned binary — use the MSVC command instead
> rather than weakening that security setting.

There is no test suite or linter configured; validation is done by running inside mms
(see below) or by writing a throwaway fixture program against `MazeMap`/`FloodFill`
(they have no I/O dependencies, so they link into a standalone test binary trivially).

## Run in mms

Configure this directory as a custom algorithm in the mms GUI (`+` button): Directory =
this repo root. For Build Command, mms launches the Build/Run commands directly rather
than through a shell, so `cl` won't have its environment set up (no `vcvars64.bat` has
run) and a bare `g++`/`cl` command will fail with a generic "process failed to
start"/"file not found" error even if the compiler works fine from a terminal. Use
`build.bat` (loads MSVC's env then runs `cl`) via: `cmd /c "<repo path>\build.bat"`. For
Run Command, mms's Run step doesn't share Build's working directory either, so a bare
`micromouse.exe` also fails with "process failed to start" — use the **absolute path** to
`micromouse.exe` instead. Load `mazes/empty16.num` (boundary-only) first to sanity-check
movement, then `mazes/obstacle16.num` (a few hand-placed interior walls), then a real
maze file from [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles).

## Architecture

Three layers, in dependency order:

1. **`src/MazeMap`** — pure data: a 16x16 grid of wall bitfields (bit index per
   `Direction`, see `src/Direction.h`) plus a parallel "known" bitfield distinguishing
   *sensed* walls from *assumed-open* unknowns. The four boundary walls are pre-seeded in
   the constructor since a competition maze is always fully enclosed. `isGoal(x, y)` is
   the four center cells `(7,7) (7,8) (8,7) (8,8)`. No I/O.

2. **`src/FloodFill`** — pure BFS logic: `recompute()` does a multi-source BFS from the
   goal cells outward over `MazeMap`, treating unknown walls as open (optimistic
   exploration). `bestDirection()` picks the accessible neighbor with the lowest distance,
   tie-breaking in the order straight → left → right → behind to minimize turns. No I/O.

3. **`src/MouseAgent`** — the only layer that talks to the outside world, and it does so
   by calling `API::` (from `API.h`/`API.cpp`, the official unmodified
   `mackorone/mms-cpp` adapter) directly — there is intentionally no abstraction
   interface between the algorithm and `API::`. `exploreToGoal()` repeats: sense the 3
   walls around the current cell (front/left/right, translated from robot-relative to
   absolute `Direction` using the current heading) → record into `MazeMap` → recompute
   `FloodFill` → turn/advance toward the best neighbor → repeat until `isGoal()`.

`Main.cpp` at the repo root just wires `MouseAgent` up and calls `exploreToGoal()`. All
logging goes to stderr (`std::cerr`) — stdout is the mms protocol channel and must never
carry anything but protocol commands.

`API.h`/`API.cpp` are copied verbatim from `mackorone/mms-cpp`; don't hand-modify them —
re-fetch from upstream if the protocol ever needs more commands (e.g. `wallBack`,
`getStat`).

## Hardware handoff

The three-layer split above is exactly what makes a future Arduino/ESP32-C6 port
low-effort: `MazeMap` and `FloodFill` have no I/O at all, so they carry over unchanged.
Only `MouseAgent`'s six `API::` calls
(`wallFront`/`wallLeft`/`wallRight`/`moveForward`/`turnLeft`/`turnRight`, plus the
no-op-on-real-hardware `setWall`) need to become real ToF sensor reads and motor/turn
commands; `Main.cpp` and `API.h`/`API.cpp` are mms-specific and get dropped entirely.
Nothing under `src/` includes `Arduino.h` or calls `delay()` — keep it that way so the
hardware team can lift `src/` as-is.
