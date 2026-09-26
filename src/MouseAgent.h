#pragma once
#include <utility>
#include <vector>
#include "Direction.h"
#include "FloodFill.h"
#include "MazeMap.h"
#include "RobotPlatform.h"
class MouseAgent {
public:
    explicit MouseAgent(RobotPlatform& platform) : platform_(platform) {}
    MotionResult run();
    int x() const { return x_; }
    int y() const { return y_; }
    Direction heading() const { return heading_; }
    const MazeMap& map() const { return mazeMap_; }
private:
    MotionResult driveTo(const std::vector<std::pair<int, int>>& targets);
    MotionResult senseWalls();
    MotionResult turnToFaceAndAdvance(Direction target);
    RobotPlatform& platform_;
    MazeMap mazeMap_;
    FloodFill floodFill_;
    int x_ = 0, y_ = 0;
    Direction heading_ = Direction::NORTH;
};
