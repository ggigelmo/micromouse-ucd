#include "MazeMap.h"

MazeMap::MazeMap() {
    for (auto& row : walls_) row.fill(0);
    for (auto& row : known_) row.fill(0);

    // Mazes are required to be fully enclosed, so the outer boundary walls
    // are known from the start even before anything is sensed.
    for (int x = 0; x < MAZE_SIZE; ++x) {
        setWall(x, 0, Direction::SOUTH);
        setWall(x, MAZE_SIZE - 1, Direction::NORTH);
    }
    for (int y = 0; y < MAZE_SIZE; ++y) {
        setWall(0, y, Direction::WEST);
        setWall(MAZE_SIZE - 1, y, Direction::EAST);
    }
}

bool MazeMap::inBounds(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

bool MazeMap::isGoal(int x, int y) {
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

void MazeMap::neighbor(int x, int y, Direction d, int& nx, int& ny) {
    nx = x;
    ny = y;
    switch (d) {
        case Direction::NORTH: ny += 1; break;
        case Direction::EAST: nx += 1; break;
        case Direction::SOUTH: ny -= 1; break;
        case Direction::WEST: nx -= 1; break;
    }
}

bool MazeMap::hasWall(int x, int y, Direction d) const {
    if (!inBounds(x, y)) return true;
    return (walls_[x][y] >> wallBit(d)) & 1;
}

bool MazeMap::isKnown(int x, int y, Direction d) const {
    if (!inBounds(x, y)) return true;
    return (known_[x][y] >> wallBit(d)) & 1;
}

void MazeMap::setWall(int x, int y, Direction d) {
    if (!inBounds(x, y)) return;
    walls_[x][y] |= (1 << wallBit(d));
    known_[x][y] |= (1 << wallBit(d));

    int nx, ny;
    neighbor(x, y, d, nx, ny);
    if (inBounds(nx, ny)) {
        Direction opposite = behind(d);
        walls_[nx][ny] |= (1 << wallBit(opposite));
        known_[nx][ny] |= (1 << wallBit(opposite));
    }
}

void MazeMap::setNoWall(int x, int y, Direction d) {
    if (!inBounds(x, y)) return;
    known_[x][y] |= (1 << wallBit(d));

    int nx, ny;
    neighbor(x, y, d, nx, ny);
    if (inBounds(nx, ny)) {
        Direction opposite = behind(d);
        known_[nx][ny] |= (1 << wallBit(opposite));
    }
}
