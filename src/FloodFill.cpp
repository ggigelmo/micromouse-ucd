#include "FloodFill.h"

#include <queue>

FloodFill::FloodFill() {
    for (auto& row : distance_) row.fill(UNREACHABLE);
}

void FloodFill::recompute(const MazeMap& map, const std::vector<std::pair<int, int>>& goalCells) {
    for (auto& row : distance_) row.fill(UNREACHABLE);

    std::queue<std::pair<int, int>> frontier;
    for (const auto& cell : goalCells) {
        distance_[cell.first][cell.second] = 0;
        frontier.push(cell);
    }

    while (!frontier.empty()) {
        auto [x, y] = frontier.front();
        frontier.pop();
        uint16_t nextDist = distance_[x][y] + 1;

        for (int i = 0; i < 4; ++i) {
            Direction d = static_cast<Direction>(i);
            if (map.hasWall(x, y, d)) continue;

            int nx, ny;
            MazeMap::neighbor(x, y, d, nx, ny);
            if (!MazeMap::inBounds(nx, ny)) continue;
            if (distance_[nx][ny] != UNREACHABLE) continue;

            distance_[nx][ny] = nextDist;
            frontier.push({nx, ny});
        }
    }
}

uint16_t FloodFill::distanceAt(int x, int y) const {
    if (!MazeMap::inBounds(x, y)) return UNREACHABLE;
    return distance_[x][y];
}

Direction FloodFill::bestDirection(const MazeMap& map, int x, int y, Direction heading) const {
    const Direction candidates[4] = {
        heading,
        turnLeftFrom(heading),
        turnRightFrom(heading),
        behind(heading),
    };

    Direction best = heading;
    uint16_t bestDist = UNREACHABLE;

    for (Direction d : candidates) {
        if (map.hasWall(x, y, d)) continue;

        int nx, ny;
        MazeMap::neighbor(x, y, d, nx, ny);
        if (!MazeMap::inBounds(nx, ny)) continue;

        uint16_t dist = distance_[nx][ny];
        if (dist < bestDist) {
            bestDist = dist;
            best = d;
        }
    }

    return best;
}
