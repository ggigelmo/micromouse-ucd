#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "Direction.h"
#include "MazeMap.h"

// Multi-source BFS distance-to-goal computation over a MazeMap, plus
// next-move selection. Pure logic, no I/O of any kind.
class FloodFill {
public:
    static constexpr uint16_t UNREACHABLE = 0xFFFF;

    FloodFill();

    // Recomputes distance-to-goal for every cell via BFS seeded at
    // `goalCells`, treating unknown walls as open (optimistic).
    void recompute(const MazeMap& map, const std::vector<std::pair<int, int>>& goalCells);

    uint16_t distanceAt(int x, int y) const;

    // Among the (up to 4) neighbors of (x, y) not blocked by a known wall,
    // returns the direction of the one with the lowest distance value.
    // Ties are broken in order: straight ahead, left, right, behind -
    // minimizing turns. Precondition: at least one neighbor is reachable
    // (i.e. distanceAt(x, y) != UNREACHABLE, or the cell itself is a goal).
    Direction bestDirection(const MazeMap& map, int x, int y, Direction heading) const;

private:
    std::array<std::array<uint16_t, MAZE_SIZE>, MAZE_SIZE> distance_;
};
