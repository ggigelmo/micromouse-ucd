#pragma once

#include <array>
#include <cstdint>

#include "Direction.h"

constexpr int MAZE_SIZE = 16;

// Pure data structure: which walls exist between cells, and which of those
// walls have actually been sensed (vs. assumed open). No I/O of any kind.
class MazeMap {
public:
    MazeMap();

    // True if there is a wall on side `d` of cell (x, y). Cells outside the
    // maze bounds are always considered walled off.
    bool hasWall(int x, int y, Direction d) const;

    // True if side `d` of cell (x, y) has actually been sensed.
    bool isKnown(int x, int y, Direction d) const;

    // Records a wall on side `d` of cell (x, y) and mirrors it onto the
    // neighboring cell's shared side, marking both as known.
    void setWall(int x, int y, Direction d);

    // Records the *absence* of a wall on side `d`, marking it known.
    void setNoWall(int x, int y, Direction d);

    static bool inBounds(int x, int y);

    // True for the four center cells: (7,7), (7,8), (8,7), (8,8).
    static bool isGoal(int x, int y);

    // Neighboring cell coordinate in direction `d`; caller must check
    // inBounds() on the result before using it.
    static void neighbor(int x, int y, Direction d, int& nx, int& ny);

private:
    std::array<std::array<uint8_t, MAZE_SIZE>, MAZE_SIZE> walls_;
    std::array<std::array<uint8_t, MAZE_SIZE>, MAZE_SIZE> known_;
};
