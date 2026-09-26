#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Preferences.h>
#include "Control.h"
#include "src/RobotPlatform.h"

class Hardware final : public RobotPlatform {
public:
    hw::Calibration cal;
    void begin();
    bool poll();
    bool calibrateGyro();
    MotionResult readWalls(WallReadings&) override;
    MotionResult moveOneCell() override { return drive(hw::CELL_MM); }
    MotionResult turnQuarter(bool right) override { return turn(right?-90:90); }
    MotionResult drive(float millimeters);
    MotionResult turn(float degrees);
    void stop() override;
    void log(const char* message) override { Serial.println(message); }
    void pwm(int left, int right);
    void counts(int64_t (&out)[2]);
    void report();
    bool allFresh() const;
    bool idleService();
    bool waitStopped(uint32_t ms);
    bool save();
    void load();
    void invalidate();
    void acknowledgeHome() { stop(); aborted=false; benchArmed=false; homed=true; desiredHeading_=heading; lastFault=MotionResult::Ok; }
    bool calibrated() const { return hw::calibrationValid(cal) && saved_; }
    bool homed = false;
    bool active = false;
    bool aborted = false;
    bool buttonStopLatched = false;
    bool benchArmed = false;
    float heading = 0, gyroRate = 0;
    uint16_t ranges[3] = {};
    uint8_t statuses[3] = {};
    bool rangeValid[3] = {};
    bool imuOk = false;
    MotionResult lastFault = MotionResult::Ok;
    MotionResult benchPulse(int wheel, int duty, uint32_t duration);
private:
    VL53L0X tof_[3];
    bool tofOk_[3] = {};
    uint32_t rangeAt_[3] = {}, imuAt_ = 0, pollAt_ = 0, gyroMicros_ = 0;
    uint8_t imuAddr_ = 0;
    float gyroBias_ = 0, desiredHeading_ = 0;
    bool saved_ = false, pwmOk_ = false;
    bool readBytes(uint8_t address, uint8_t reg, uint8_t* data, size_t count);
    bool writeByte(uint8_t address, uint8_t reg, uint8_t value);
    bool gyroSample(float& rate);
    MotionResult finish(MotionResult result);
    MotionResult move(float distance, float angle, bool rotating);
};
