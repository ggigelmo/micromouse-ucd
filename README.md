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
- `src/MouseAgent.{h,cpp}` — explore-to-goal loop, calls `API::` directly.
- `mazes/empty16.num` — boundary-only maze, for sanity-checking movement/turning.
- `mazes/obstacle16.num` — a few interior walls near the start and the goal entrance, for
  sanity-checking that the solver actually routes around obstacles.
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
   - **Run Command:** `micromouse.exe`
4. Load a maze: File → Import Maze → pick `mazes/empty16.num` first, then
   `mazes/obstacle16.num`, then a real competition maze (e.g. from
   [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)).
5. Click "Run". The mouse should navigate to one of the four center cells without
   crashing. Debug logs are printed to stderr, visible in mms's console/log panel.

## Local verification (already done for you)

The solver has been compiled with MSVC and smoke-tested end-to-end against a small
stand-in for the mms protocol (not mms itself): it reaches a goal cell without crashing
on both `mazes/empty16.num` and `mazes/obstacle16.num`, and `FloodFill`'s BFS distances
and wall-avoidance were checked against hand-built fixtures. Still worth running for
real in mms to see it visually and try a real competition maze.
