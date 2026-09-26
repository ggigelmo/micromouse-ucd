#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace hw {
constexpr int DIR[2] = {0, 3}, PWM[2] = {2, 10};
constexpr int ENC_A[2] = {21, 23}, ENC_B[2] = {22, 11};
constexpr int XSHUT[3] = {18, 19, 20}, SDA = 6, SCL = 7, BUTTON = 9;
constexpr uint8_t TOF_ADDR[3] = {0x30, 0x31, 0x29};
constexpr float CELL_MM = 180, MAX_SPEED = 60, ACCEL = 120;
constexpr float MAX_TURN = 35, TURN_ACCEL = 90, PI_F = 3.14159265359f;
constexpr int MAX_PWM = 170;
constexpr uint32_t CAL_VERSION = 1;
inline float clip(float value, float low, float high) { return std::max(low, std::min(value, high)); }
inline float ramp(float now, float target, float step) { return now + clip(target-now, -step, step); }
inline bool finiteBetween(float value, float low, float high) {
    return std::isfinite(value) && value >= low && value <= high;
}
// Geometry is relative to the midpoint of the wheel axle; includes wheels/sensors.
// Chassis dimensions supplied by the team; wall thickness is provisional.
// Encoder scale and explicit sensor confirmation still prohibit movement.
struct Calibration {
    uint32_t version = CAL_VERSION;
    float mmPerTick[2] = {0, 0};
    int32_t encoderSign[2] = {1, 1};
    float minPwm[2] = {0, 0};
    float feedforward[2] = {0.5f, 0.5f}; // PWM per mm/s, adjustable after speed tests.
    // Body measured 95 mm; later sensor offsets imply a 100 mm overall span.
    float track = 80, width = 100, front = 60, rear = 35, wallThickness = 12;
    float sensorLeft = 50, sensorFront = 45, sensorRight = 50;
    float gyroBias = 0;
    int32_t gyroSign = 1;
    uint32_t sensorsConfirmed = 0;
};
inline bool scaleValid(const Calibration& c) {
    for (int i=0;i<2;++i)
        if (!finiteBetween(c.mmPerTick[i],0.001f,5) || (c.encoderSign[i]!=1 && c.encoderSign[i]!=-1)) return false;
    return true;
}
inline bool geometryValid(const Calibration& c) {
    if (!finiteBetween(c.track,30,170) || !finiteBetween(c.width,c.track,175) ||
        !finiteBetween(c.front,1,170) || !finiteBetween(c.rear,1,170) ||
        !finiteBetween(c.wallThickness,1,30) ||
        !finiteBetween(c.sensorLeft,0,c.width/2) ||
        !finiteBetween(c.sensorRight,0,c.width/2) ||
        !finiteBetween(c.sensorFront,0,c.front)) return false;
    const float radius=std::hypot(c.width/2, std::max(c.front,c.rear));
    return radius + 3 < (CELL_MM-c.wallThickness)/2;
}
inline bool calibrationValid(const Calibration& c) {
    if (c.version!=CAL_VERSION || !scaleValid(c) || !geometryValid(c) ||
        (c.gyroSign!=1 && c.gyroSign!=-1) || !finiteBetween(c.gyroBias,-10,10) || c.sensorsConfirmed!=1) return false;
    for(int i=0;i<2;++i)
        if(!finiteBetween(c.minPwm[i],1,150) || !finiteBetween(c.feedforward[i],0,3)) return false;
    return true;
}
inline float wallThreshold(const Calibration& c, float sensorOffset) {
    return (CELL_MM-c.wallThickness)/2 - sensorOffset + CELL_MM/4;
}
inline float brakingClearance(float speed) { return 8 + std::abs(speed)*0.15f + speed*speed/(2*ACCEL); }
inline bool validRange(uint16_t mm, uint8_t rawStatus) {
    // ST's internal device status 11 is RANGE_COMPLETE; 0/8190 are not clear walls.
    return rawStatus == 11 && mm > 0 && mm < 2000;
}
struct SpeedController {
    float integral = 0;
    void reset() { integral=0; }
    float step(float target, float measured, float dt, float deadband, float ff) {
        if (std::abs(target)<0.5f) { reset(); return 0; }
        const float sign=target>0?1:-1;
        const float error=std::abs(target)-measured*sign;
        const float candidate=clip(integral+error*dt*1.2f,-50,50);
        float output=deadband+ff*std::abs(target)+0.8f*error+candidate;
        if ((output<MAX_PWM && output>0) || (output>=MAX_PWM && error<0) || (output<=0 && error>0))
            integral=candidate;
        return sign*clip(deadband+ff*std::abs(target)+0.8f*error+integral,0,MAX_PWM);
    }
};
}
