#include "MouseAgent.h"

#include <iostream>
#include <vector>

#include "../API.h"

namespace {
const std::vector<std::pair<int, int>> kGoalCells = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
const std::vector<std::pair<int, int>> kStartCell = {{0, 0}};

void log(const std::string& text) {
    std::cerr << text << std::endl;
}
}  // namespace

MouseAgent::MouseAgent() : x_(0), y_(0), heading_(Direction::NORTH) {}

void MouseAgent::senseWalls() {
    Direction frontDir = heading_;
    Direction leftDir = turnLeftFrom(heading_);
    Direction rightDir = turnRightFrom(heading_);

    if (API::wallFront()) {
        mazeMap_.setWall(x_, y_, frontDir);
    } else {
        mazeMap_.setNoWall(x_, y_, frontDir);
    }
    if (API::wallLeft()) {
        mazeMap_.setWall(x_, y_, leftDir);
    } else {
        mazeMap_.setNoWall(x_, y_, leftDir);
    }
    if (API::wallRight()) {
        mazeMap_.setWall(x_, y_, rightDir);
    } else {
        mazeMap_.setNoWall(x_, y_, rightDir);
    }
}

void MouseAgent::turnToFaceAndAdvance(Direction target) {
    if (target == turnRightFrom(heading_)) {
        API::turnRight();
    } else if (target == turnLeftFrom(heading_)) {
        API::turnLeft();
    } else if (target == behind(heading_)) {
        API::turnRight();
        API::turnRight();
    }
    // else target == heading_: no turn needed.

    heading_ = target;
    API::moveForward();

    int nx, ny;
    MazeMap::neighbor(x_, y_, heading_, nx, ny);
    x_ = nx;
    y_ = ny;
}

bool MouseAgent::isAtAnyOf(int x, int y, const std::vector<std::pair<int, int>>& cells) {
    for (const auto& cell : cells) {
        if (cell.first == x && cell.second == y) return true;
    }
    return false;
}

bool MouseAgent::checkForReset() {
    if (!API::wasReset()) return false;

    log("reset detected: returning to start, keeping known walls");
    API::ackReset();
    x_ = 0;
    y_ = 0;
    heading_ = Direction::NORTH;
    return true;
}

void MouseAgent::driveTo(const std::vector<std::pair<int, int>>& targets) {
    while (!isAtAnyOf(x_, y_, targets)) {
        if (checkForReset()) continue;

        senseWalls();
        floodFill_.recompute(mazeMap_, targets);

        Direction next = floodFill_.bestDirection(mazeMap_, x_, y_, heading_);
        turnToFaceAndAdvance(next);
    }
}

void MouseAgent::run() {
    log("search phase: starting");
    driveTo(kGoalCells);
    log("search phase: reached goal cell");

    log("return phase: starting");
    driveTo(kStartCell);
    log("return phase: back at start");

    log("speed run: starting");
    driveTo(kGoalCells);
    log("speed run: reached goal cell");
}
