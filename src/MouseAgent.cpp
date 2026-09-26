#include "MouseAgent.h"
#include <algorithm>

MotionResult MouseAgent::senseWalls() {
    WallReadings w;
    const auto result = platform_.readWalls(w);
    if (result != MotionResult::Ok) return result;
    const Direction directions[] = {heading_, turnLeftFrom(heading_), turnRightFrom(heading_)};
    const bool walls[] = {w.front, w.left, w.right};
    for (int i = 0; i < 3; ++i) {
        if (walls[i]) mazeMap_.setWall(x_, y_, directions[i]);
        else mazeMap_.setNoWall(x_, y_, directions[i]);
    }
    return MotionResult::Ok;
}
MotionResult MouseAgent::turnToFaceAndAdvance(Direction target) {
    while (heading_ != target) {
        const bool right = target != turnLeftFrom(heading_);
        const auto result = platform_.turnQuarter(right);
        if (result != MotionResult::Ok) return result;
        // Commit each completed quarter turn, even halfway through a U-turn.
        heading_ = right ? turnRightFrom(heading_) : turnLeftFrom(heading_);
    }
    const auto result = platform_.moveOneCell();
    if (result != MotionResult::Ok) return result;
    int nx, ny;
    MazeMap::neighbor(x_, y_, heading_, nx, ny);
    x_ = nx; y_ = ny;
    return MotionResult::Ok;
}
MotionResult MouseAgent::driveTo(const std::vector<std::pair<int, int>>& targets) {
    while (std::find(targets.begin(), targets.end(), std::make_pair(x_, y_)) == targets.end()) {
        if (platform_.consumeSimulatorReset()) {
            x_ = y_ = 0; heading_ = Direction::NORTH;
            platform_.log("reset: start pose restored; map retained");
            continue;
        }
        auto result = senseWalls();
        if (result != MotionResult::Ok) return result;
        floodFill_.recompute(mazeMap_, targets);
        if (floodFill_.distanceAt(x_, y_) == FloodFill::UNREACHABLE)
            return MotionResult::Unreachable;
        const auto next = floodFill_.bestDirection(mazeMap_, x_, y_, heading_);
        int nx, ny;
        MazeMap::neighbor(x_, y_, next, nx, ny);
        if (mazeMap_.hasWall(x_, y_, next) || !MazeMap::inBounds(nx, ny))
            return MotionResult::Unreachable;
        result = turnToFaceAndAdvance(next);
        if (result != MotionResult::Ok) return result;
    }
    return MotionResult::Ok;
}
MotionResult MouseAgent::run() {
    const std::vector<std::pair<int, int>> goals = {{7,7}, {7,8}, {8,7}, {8,8}};
    const std::vector<std::pair<int, int>> start = {{0,0}};
    const char* phases[] = {"search phase", "return phase", "final run phase"};
    for (int phase = 0; phase < 3; ++phase) {
        platform_.log(phases[phase]);
        const auto result = driveTo(phase == 1 ? start : goals);
        platform_.stop();
        if (result != MotionResult::Ok) {
            platform_.log(resultName(result)); return result;
        }
        platform_.log("phase complete");
    }
    return MotionResult::Ok;
}
