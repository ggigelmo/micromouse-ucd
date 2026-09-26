#pragma once

// Absolute compass heading. Values are chosen so that "turn right" is
// always (dir + 1) % 4 and "turn left" is always (dir + 3) % 4.
enum class Direction {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};

inline Direction turnRightFrom(Direction d) {
    return static_cast<Direction>((static_cast<int>(d) + 1) % 4);
}

inline Direction turnLeftFrom(Direction d) {
    return static_cast<Direction>((static_cast<int>(d) + 3) % 4);
}

inline Direction behind(Direction d) {
    return static_cast<Direction>((static_cast<int>(d) + 2) % 4);
}

// Wall-bit index for a given direction, used by MazeMap's bitfields.
inline int wallBit(Direction d) {
    return static_cast<int>(d);
}

inline char directionChar(Direction d) {
    switch (d) {
        case Direction::NORTH: return 'n';
        case Direction::EAST: return 'e';
        case Direction::SOUTH: return 's';
        case Direction::WEST: return 'w';
    }
    return '?';
}
