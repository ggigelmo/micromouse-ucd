#pragma once
#include "API.h"
#include "src/RobotPlatform.h"
#include <iostream>
class SimulatorPlatform final : public RobotPlatform {
public:
    MotionResult readWalls(WallReadings& w) override {
        w = {API::wallFront(), API::wallLeft(), API::wallRight()};
        return MotionResult::Ok;
    }
    MotionResult moveOneCell() override { API::moveForward(); return MotionResult::Ok; }
    MotionResult turnQuarter(bool right) override {
        if (right) API::turnRight(); else API::turnLeft();
        return MotionResult::Ok;
    }
    void stop() override {}
    void log(const char* text) override { std::cerr << text << std::endl; }
    bool consumeSimulatorReset() override {
        if (!API::wasReset()) return false;
        API::ackReset(); return true;
    }
};
