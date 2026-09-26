#pragma once

#include "Direction.h"
#include "FloodFill.h"
#include "MazeMap.h"

// Ties MazeMap + FloodFill together into an explore-to-goal run, talking to
// the mms simulator directly through API:: (see API.h). Software-only: no
// hardware of any kind is involved.
class MouseAgent {
public:
    MouseAgent();

    // Explores the maze cell by cell, using flood fill to always advance
    // toward the goal, until a center cell is reached.
    void exploreToGoal();

private:
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
