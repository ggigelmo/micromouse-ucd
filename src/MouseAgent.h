#pragma once

#include <utility>
#include <vector>

#include "Direction.h"
#include "FloodFill.h"
#include "MazeMap.h"

// Ties MazeMap + FloodFill together into the classic three-phase micromouse
// run, talking to the mms simulator directly through API:: (see API.h).
// Software-only: no hardware of any kind is involved.
//
// Diagonal movement, turn-weighted path costs, and extra exploration passes
// to map frontier cells before the speed run are deliberately not
// implemented yet - those only pay off once there's a real, calibrated
// motion profile from hardware to weigh them against.
class MouseAgent {
public:
    MouseAgent();

    // Runs all three phases in sequence: search (start -> goal, discovering
    // walls), return (goal -> start, using what's now known), and a speed
    // run (start -> goal again). The speed run is what mms scores as the
    // new best/current run, since it no longer needs to feel out any walls
    // that were already discovered on the way there and back.
    void run();

private:
    // Drives from the current cell to any cell in `targets`, sensing walls
    // and recomputing flood fill at every step. Used for all three phases;
    // they differ only in which cells count as the destination. Checks for
    // a simulator reset (see checkForReset()) before every step.
    void driveTo(const std::vector<std::pair<int, int>>& targets);

    static bool isAtAnyOf(int x, int y, const std::vector<std::pair<int, int>>& cells);

    // If the mms Reset button was pressed (API::wasReset()), acknowledges it
    // (which sends the mouse back to the start cell) and resyncs x_/y_/
    // heading_ to match, without discarding anything already learned about
    // the maze. Returns true if a reset was handled.
    bool checkForReset();

    // Senses the walls around the current cell and records them into mazeMap_.
    void senseWalls();

    // Turns from the current heading to face `target`, then moves forward
    // one cell and updates position_/heading_.
    void turnToFaceAndAdvance(Direction target);

    MazeMap mazeMap_;
    FloodFill floodFill_;
    int x_;
    int y_;
    Direction heading_;
};
