#include "MouseAgent.h"
#include "../firmware/Control.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

class World : public RobotPlatform {
public:
    MazeMap truth;
    int x=0,y=0,moves=0,turns=0,stops=0,phase=0,completed=0;
    Direction facing=Direction::NORTH;
    int failMove=-1, failTurn=-1, failRead=-1, reads=0, resetPhase=0;
    bool resetDone=false;
    MotionResult failure=MotionResult::Stall;
    MotionResult readWalls(WallReadings& out) override {
        if(++reads==failRead) return MotionResult::SensorFault;
        assert(reads<10000);
        out={truth.hasWall(x,y,facing),truth.hasWall(x,y,turnLeftFrom(facing)),truth.hasWall(x,y,turnRightFrom(facing))};
        return MotionResult::Ok;
    }
    MotionResult moveOneCell() override {
        if(++moves==failMove) return failure;
        assert(!truth.hasWall(x,y,facing));
        int nx,ny; MazeMap::neighbor(x,y,facing,nx,ny);
        assert(MazeMap::inBounds(nx,ny)); x=nx;y=ny;
        return MotionResult::Ok;
    }
    MotionResult turnQuarter(bool right) override {
        if(++turns==failTurn) return failure;
        facing=right?turnRightFrom(facing):turnLeftFrom(facing); return MotionResult::Ok;
    }
    void stop() override { ++stops; }
    void log(const char* s) override {
        if(!strcmp(s,"search phase")||!strcmp(s,"return phase")||!strcmp(s,"final run phase")) ++phase;
        if(!strcmp(s,"phase complete")) {
            assert(phase==2?(x==0&&y==0):MazeMap::isGoal(x,y)); ++completed;
        }
    }
    bool consumeSimulatorReset() override {
        if(resetPhase==phase && moves>3 && !resetDone) {
            resetDone=true; x=y=0; facing=Direction::NORTH;return true;
        }
        return false;
    }
};
void assertPose(const MouseAgent& a,const World& w) {
    assert(a.x()==w.x && a.y()==w.y && a.heading()==w.facing);
}
int main() {
    for(int reset=0;reset<=3;++reset) {
        World w; w.resetPhase=reset;
        // Dead end / detour near the start; the solver must discover and avoid it.
        w.truth.setWall(0,2,Direction::NORTH); w.truth.setWall(0,1,Direction::EAST);
        MouseAgent a(w);assert(a.run()==MotionResult::Ok);assert(w.completed==3);assertPose(a,w);
        if(reset)assert(w.resetDone);
    }
    for(int failAt:{3,20,35}) for(auto fault:{MotionResult::Stopped,MotionResult::Stall,MotionResult::Timeout,MotionResult::Obstacle,MotionResult::EncoderFault,MotionResult::SensorFault}) {
        World w;w.failMove=failAt;w.failure=fault;MouseAgent a(w);
        assert(a.run()==fault);assertPose(a,w);assert(w.stops==w.completed+1);
        assert(w.moves==failAt);assert(w.completed==(failAt<14?0:failAt<28?1:2));
    }
    {World w;w.failRead=1;MouseAgent a(w);assert(a.run()==MotionResult::SensorFault);
     assert(w.moves==0);assert(!a.map().isKnown(0,0,Direction::NORTH));}
    {World w;w.truth.setWall(0,0,Direction::NORTH);w.truth.setWall(0,0,Direction::EAST);
     MouseAgent a(w);assert(a.run()==MotionResult::Unreachable);assert(w.moves==0 && w.turns==0);}
    {World w;w.truth.setWall(0,0,Direction::NORTH);w.failTurn=1;MouseAgent a(w);
     assert(a.run()==MotionResult::Stall);assertPose(a,w);assert(w.moves==0);}
    {World w;w.truth.setWall(0,0,Direction::NORTH);w.failMove=1;MouseAgent a(w);
     assert(a.run()==MotionResult::Stall);assert(a.heading()==Direction::EAST);assertPose(a,w);}
    // Enter a cul-de-sac. Fail on second quarter of the U-turn, preserving first.
    {World w;w.truth.setWall(0,1,Direction::NORTH);w.truth.setWall(0,1,Direction::EAST);
     w.failTurn=2;MouseAgent a(w);assert(a.run()==MotionResult::Stall);
     assert(a.x()==0 && a.y()==1 && a.heading()==Direction::EAST);assertPose(a,w);}
    hw::Calibration c;
    assert(!hw::calibrationValid(c));
    assert(hw::geometryValid(c)); // Team measurements; encoder calibration still missing.
    auto unknown=c;unknown.sensorLeft=-1;assert(!hw::geometryValid(unknown));
    c.mmPerTick[0]=c.mmPerTick[1]=0.08f;c.minPwm[0]=c.minPwm[1]=70;
    c.track=80;c.width=100;c.front=50;c.rear=30;c.wallThickness=12;
    c.sensorLeft=c.sensorRight=45;c.sensorFront=45;c.sensorsConfirmed=1;
    assert(hw::calibrationValid(c));
    auto chassis=c;chassis.width=95;chassis.front=60;chassis.rear=35;
    assert(hw::geometryValid(chassis)); // Team measurements, provisional 12 mm walls.
    assert(hw::wallThreshold(c,c.sensorFront)>39 && hw::wallThreshold(c,c.sensorFront)<219);
    auto bad=c;bad.front=90;assert(!hw::geometryValid(bad));
    bad=c;bad.sensorLeft=bad.width/2+1;assert(!hw::geometryValid(bad));
    bad=c;bad.sensorFront=bad.front+1;assert(!hw::geometryValid(bad));
    bad=c;bad.mmPerTick[0]=std::numeric_limits<float>::quiet_NaN();assert(!hw::calibrationValid(bad));
    bad=c;bad.encoderSign[1]=0;assert(!hw::calibrationValid(bad));
    bad=c;bad.sensorsConfirmed=0;assert(!hw::calibrationValid(bad));
    assert(hw::validRange(100,11));assert(!hw::validRange(8190,11));
    assert(!hw::validRange(100,4));assert(!hw::validRange(0,11));
    assert(hw::brakingClearance(60)>hw::brakingClearance(10));
    assert(hw::ramp(0,60,1.2f)==1.2f);
    hw::SpeedController ctrl;
    for(int i=0;i<1000;++i) assert(std::abs(ctrl.step(60,0,0.01f,150,3))<=170);
    assert(ctrl.step(0,0,0.01f,150,3)==0);
    assert(ctrl.step(-30,0,0.01f,70,0.5f)<0);
    std::cout<<"Solver: three phases, reset recovery, sensor/motion failures, U-turn commit and unreachable goal PASS\n";
    std::cout<<"Control: calibration gates, geometry, invalid ranges, ramps and PWM limits PASS\n";
}
