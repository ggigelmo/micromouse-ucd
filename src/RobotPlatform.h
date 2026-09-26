#pragma once

enum class MotionResult { Ok, Stopped, SensorFault, Stall, Timeout, Obstacle,
                          NotCalibrated, Unreachable, InvalidGeometry, EncoderFault };
inline const char* resultName(MotionResult r) {
    switch (r) {
    case MotionResult::Ok: return "OK";
    case MotionResult::Stopped: return "STOPPED";
    case MotionResult::SensorFault: return "SENSOR_FAULT";
    case MotionResult::Stall: return "STALL";
    case MotionResult::Timeout: return "TIMEOUT";
    case MotionResult::Obstacle: return "OBSTACLE";
    case MotionResult::NotCalibrated: return "NOT_CALIBRATED";
    case MotionResult::Unreachable: return "UNREACHABLE";
    case MotionResult::InvalidGeometry: return "INVALID_GEOMETRY";
    case MotionResult::EncoderFault: return "ENCODER_FAULT";
    }
    return "UNKNOWN";
}
struct WallReadings { bool front = false, left = false, right = false; };
class RobotPlatform {
public:
    virtual ~RobotPlatform() = default;
    virtual MotionResult readWalls(WallReadings&) = 0;
    virtual MotionResult moveOneCell() = 0;
    virtual MotionResult turnQuarter(bool right) = 0;
    virtual void stop() = 0;
    virtual void log(const char*) = 0;
    // Only a simulator can teleport home. Never used to reset hardware pose.
    virtual bool consumeSimulatorReset() { return false; }
};
