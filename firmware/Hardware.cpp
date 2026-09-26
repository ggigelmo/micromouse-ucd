#include "Hardware.h"
#include <cstring>

namespace {
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t ticks[2] = {};
volatile uint32_t badTransitions[2] = {};
volatile uint8_t previous[2] = {};
// Quadrature x4: each legal edge changes the count by one.
const int8_t transitions[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
void ARDUINO_ISR_ATTR encoder(int wheel) {
    const uint8_t next=(digitalRead(hw::ENC_A[wheel])<<1)|digitalRead(hw::ENC_B[wheel]);
    portENTER_CRITICAL_ISR(&encoderMux);
    const uint8_t old=previous[wheel];
    if((old^next)==3) ++badTransitions[wheel];
    ticks[wheel]+=transitions[(old<<2)|next];
    previous[wheel]=next;
    portEXIT_CRITICAL_ISR(&encoderMux);
}
void ARDUINO_ISR_ATTR leftEncoder() { encoder(0); }
void ARDUINO_ISR_ATTR rightEncoder() { encoder(1); }
}

void Hardware::counts(int64_t (&out)[2]) {
    portENTER_CRITICAL(&encoderMux);
    out[0]=ticks[0]; out[1]=ticks[1];
    portEXIT_CRITICAL(&encoderMux);
}
void Hardware::pwm(int left,int right) {
    const int commands[2]={left,right};
    for(int i=0;i<2;++i) {
        // Both physical wheels ran backward for the original positive command.
        const int value=constrain(-commands[i],-hw::MAX_PWM,hw::MAX_PWM);
        digitalWrite(hw::DIR[i],value>=0?HIGH:LOW);
        if(pwmOk_) ledcWrite(hw::PWM[i],abs(value));
    }
}
void Hardware::stop() { pwm(0,0); active=false; }
void Hardware::invalidate() { saved_=false; cal.sensorsConfirmed=0; homed=false; }
void Hardware::load() {
    Preferences prefs;
    if(!prefs.begin("ucd-mouse",true)) return;
    hw::Calibration candidate;
    if(prefs.getBytesLength("cal")==sizeof(candidate) &&
       prefs.getBytes("cal",&candidate,sizeof(candidate))==sizeof(candidate) &&
       hw::calibrationValid(candidate)) { cal=candidate; saved_=true; }
    prefs.end();
}
bool Hardware::save() {
    if(!hw::calibrationValid(cal)) return false;
    Preferences prefs;
    if(!prefs.begin("ucd-mouse",false)) return false;
    saved_=prefs.putBytes("cal",&cal,sizeof(cal))==sizeof(cal);
    prefs.end(); return saved_;
}
bool Hardware::readBytes(uint8_t address,uint8_t reg,uint8_t* data,size_t size) {
    Wire.beginTransmission(address); Wire.write(reg);
    if(Wire.endTransmission(false)!=0) return false;
    if(Wire.requestFrom(address,size)!=size) { while(Wire.available()) Wire.read(); return false; }
    for(size_t i=0;i<size;++i) data[i]=Wire.read();
    return true;
}
bool Hardware::writeByte(uint8_t address,uint8_t reg,uint8_t value) {
    Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
    return Wire.endTransmission()==0;
}
bool Hardware::gyroSample(float& rate) {
    uint8_t data[2];
    if(!imuAddr_ || !readBytes(imuAddr_,0x47,data,2)) return false;
    rate=static_cast<int16_t>((data[0]<<8)|data[1])/65.5f;
    return true;
}
bool Hardware::calibrateGyro() {
    stop();
    imuOk=false;
    float sum=0,sumSquares=0;
    for(int i=0;i<250;++i) {
        if(!idleService()) { Serial.println("GYRO calibration stopped"); return false; }
        float value;
        if(!gyroSample(value)) { Serial.println("GYRO calibration: I2C read failed"); imuOk=false; return false; }
        sum+=value; sumSquares+=value*value;
        delay(4);
    }
    const float mean=sum/250;
    if(sumSquares/250-mean*mean>1.0f || std::abs(mean)>10) {
        Serial.printf("GYRO calibration: motion/noise mean=%.3f variance=%.3f\n",mean,sumSquares/250-mean*mean);
        imuOk=false; return false;
    }
    cal.gyroBias=gyroBias_=mean; gyroRate=heading=desiredHeading_=0;
    gyroMicros_=micros(); imuAt_=millis(); imuOk=true;
    Serial.printf("GYRO calibrated bias=%.3f dps; keep board fixed to chassis\n",gyroBias_);
    return true;
}
void Hardware::begin() {
    for(int i=0;i<2;++i) {
        digitalWrite(hw::PWM[i],LOW); pinMode(hw::PWM[i],OUTPUT);
        digitalWrite(hw::DIR[i],LOW); pinMode(hw::DIR[i],OUTPUT);
    }
    const bool l=ledcAttach(hw::PWM[0],20000,8), r=ledcAttach(hw::PWM[1],20000,8);
    pwmOk_=l&&r;
    // Independently clear either channel even if only the other attach failed.
    if(l) ledcWrite(hw::PWM[0],0);
    if(r) ledcWrite(hw::PWM[1],0);
    pinMode(hw::BUTTON,INPUT_PULLUP);
    for(int i=0;i<2;++i) {
        pinMode(hw::ENC_A[i],INPUT_PULLUP); pinMode(hw::ENC_B[i],INPUT_PULLUP);
        previous[i]=(digitalRead(hw::ENC_A[i])<<1)|digitalRead(hw::ENC_B[i]);
    }
    attachInterrupt(digitalPinToInterrupt(hw::ENC_A[0]),leftEncoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(hw::ENC_B[0]),leftEncoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(hw::ENC_A[1]),rightEncoder,CHANGE);
    attachInterrupt(digitalPinToInterrupt(hw::ENC_B[1]),rightEncoder,CHANGE);
    load();
    for(int pin:hw::XSHUT) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
    Wire.begin(hw::SDA,hw::SCL); Wire.setClock(100000); Wire.setTimeOut(10);
    delay(20);
    for(int i=0;i<3;++i) {
        digitalWrite(hw::XSHUT[i],HIGH); delay(10);
        tof_[i].setTimeout(100);
        tofOk_[i]=tof_[i].init();
        if(tofOk_[i]) {
            tof_[i].setAddress(hw::TOF_ADDR[i]);
            tof_[i].setMeasurementTimingBudget(33000);
            tof_[i].startContinuous(50);
            tof_[i].setTimeout(10);
        } else digitalWrite(hw::XSHUT[i],LOW);
        Serial.printf("TOF %c addr=0x%02X %s\n","LFR"[i],hw::TOF_ADDR[i],tofOk_[i]?"FOUND":"MISSING");
    }
    for(uint8_t addr:{0x68,0x69}) {
        uint8_t id;
        if(readBytes(addr,0x75,&id,1) && (id==0x68 || id==0x69)) { imuAddr_=addr; break; }
    }
    imuOk=imuAddr_ && writeByte(imuAddr_,0x6B,1) && writeByte(imuAddr_,0x1B,8) && writeByte(imuAddr_,0x1A,4);
    delay(100);
    if(imuOk) calibrateGyro();
    if(!imuOk || !pwmOk_) lastFault=MotionResult::SensorFault;
    Serial.printf("BOOT motors stopped; calibration=%s; MPU=%s\n",calibrated()?"SAVED":"REQUIRED",imuOk?"FOUND":"MISSING");
}
bool Hardware::poll() {
    const uint32_t now=millis();
    if(now-pollAt_<10) return allFresh();
    pollAt_=now;
    float raw;
    if(gyroSample(raw)) {
        const uint32_t us=micros();
        const float dt=(us-gyroMicros_)*1e-6f;
        gyroMicros_=us;
        gyroRate=(raw-gyroBias_)*cal.gyroSign;
        if(imuOk && dt<0.1f) heading+=gyroRate*dt;
        else if(active) { imuOk=false; return false; }
        imuAt_=now;
    } else imuOk=false;
    for(int i=0;i<3;++i) {
        if(!tofOk_[i]) continue;
        uint8_t ready;
        if(!readBytes(hw::TOF_ADDR[i],0x13,&ready,1)) { rangeValid[i]=false; continue; }
        if(!(ready&7)) continue;
        uint8_t block[12];
        if(!readBytes(hw::TOF_ADDR[i],0x14,block,12)) { rangeValid[i]=false; continue; }
        statuses[i]=(block[0]&0x78)>>3;
        ranges[i]=(block[10]<<8)|block[11];
        rangeValid[i]=hw::validRange(ranges[i],statuses[i]);
        if(!writeByte(hw::TOF_ADDR[i],0x0B,1)) rangeValid[i]=false;
        rangeAt_[i]=millis();
    }
    return allFresh();
}
bool Hardware::allFresh() const {
    const uint32_t now=millis();
    if(!imuOk || now-imuAt_>100) return false;
    for(int i=0;i<3;++i) if(!rangeValid[i] || now-rangeAt_[i]>250) return false;
    return true;
}
bool Hardware::idleService() {
    // During a blocking maneuver consume only stop commands; never queue a start.
    while(Serial.available()) {
        const char c=Serial.read();
        if(c=='s' || c=='!') aborted=true;
    }
    if(digitalRead(hw::BUTTON)==LOW) { aborted=true; buttonStopLatched=true; }
    poll();
    if(aborted) { stop(); return false; }
    return true;
}
bool Hardware::waitStopped(uint32_t ms) {
    stop(); const uint32_t start=millis();
    while(millis()-start<ms) { if(!idleService()) return false; delay(2); }
    return true;
}
MotionResult Hardware::finish(MotionResult result) {
    stop(); lastFault=result;
    if(result!=MotionResult::Ok) { homed=false; benchArmed=false; }
    Serial.printf("MOTION %s heading=%.2f\n",resultName(result),heading);
    return result;
}
void Hardware::report() {
    int64_t c[2]; counts(c);
    uint32_t invalid[2];
    portENTER_CRITICAL(&encoderMux); invalid[0]=badTransitions[0]; invalid[1]=badTransitions[1]; portEXIT_CRITICAL(&encoderMux);
    Serial.printf("L=%u[%u,%s] F=%u[%u,%s] R=%u[%u,%s] mm heading=%.2f rate=%.2f ticks=%lld,%lld bad=%lu,%lu calibrated=%d home=%d fault=%s\n",
       ranges[0],statuses[0],rangeValid[0]?"ok":"invalid",ranges[1],statuses[1],rangeValid[1]?"ok":"invalid",
       ranges[2],statuses[2],rangeValid[2]?"ok":"invalid",heading,gyroRate,c[0],c[1],
       (unsigned long)invalid[0],(unsigned long)invalid[1],calibrated(),homed,resultName(lastFault));
    Serial.printf("MPU=0x%02X %s allFresh=%d motorsActive=%d\n",imuAddr_,imuOk?"ok":"invalid",allFresh(),active);
}
MotionResult Hardware::readWalls(WallReadings& out) {
    if(!calibrated()) return finish(MotionResult::NotCalibrated);
    if(!waitStopped(150)) return finish(MotionResult::Stopped);
    uint16_t samples[3][5];
    for(int sample=0;sample<5;++sample) {
        if(!waitStopped(55)) return finish(MotionResult::Stopped);
        if(!allFresh()) return finish(MotionResult::SensorFault);
        for(int i=0;i<3;++i) samples[i][sample]=ranges[i];
    }
    bool walls[3]; const float offsets[]={cal.sensorLeft,cal.sensorFront,cal.sensorRight};
    for(int i=0;i<3;++i) {
        std::sort(samples[i],samples[i]+5);
        walls[i]=samples[i][2]<hw::wallThreshold(cal,offsets[i]);
    }
    out={walls[1],walls[0],walls[2]}; return MotionResult::Ok;
}
MotionResult Hardware::drive(float mm) {
    if(!hw::finiteBetween(mm,5,hw::CELL_MM)) return finish(MotionResult::InvalidGeometry);
    return move(mm,0,false);
}
MotionResult Hardware::turn(float degrees) {
    if(!std::isfinite(degrees) || std::abs(degrees)>90 || std::abs(degrees)<1)
        return finish(MotionResult::InvalidGeometry);
    return move(0,degrees,true);
}
MotionResult Hardware::move(float distance,float angle,bool rotating) {
    if(!calibrated()) return finish(MotionResult::NotCalibrated);
    if(aborted) return finish(MotionResult::Stopped);
    if(!pwmOk_) return finish(MotionResult::SensorFault);
    if(!waitStopped(150)) return finish(MotionResult::Stopped);
    if(!allFresh()) return finish(MotionResult::SensorFault);
    int64_t start[2],last[2],c[2]; counts(start); last[0]=start[0]; last[1]=start[1];
    hw::SpeedController controllers[2];
    float measured[2]={}, demand=0;
    const float startHeading=heading;
    // Keep a fixed cardinal reference: do not accumulate each turn's residual error.
    const float targetHeading=desiredHeading_+angle;
    uint32_t startTime=millis(),lastTime=startTime, progressed[2]={startTime,startTime}, settled=0;
    active=true;
    while(true) {
        if(!idleService()) return finish(MotionResult::Stopped);
        if(!allFresh()) return finish(MotionResult::SensorFault);
        const uint32_t now=millis();
        if(now-startTime>10000) return finish(MotionResult::Timeout);
        if(now-lastTime<10) { delay(1); continue; }
        const float dt=(now-lastTime)*0.001f;
        if(dt>0.1f) return finish(MotionResult::SensorFault);
        lastTime=now; counts(c);
        float travel[2];
        for(int i=0;i<2;++i) {
            travel[i]=(c[i]-start[i])*cal.encoderSign[i]*cal.mmPerTick[i];
            float speed=(c[i]-last[i])*cal.encoderSign[i]*cal.mmPerTick[i]/dt;
            measured[i]=0.4f*speed+0.6f*measured[i];
            if(c[i]!=last[i]) progressed[i]=now;
            last[i]=c[i];
        }
        const float error=targetHeading-heading;
        const float remaining=distance-(travel[0]+travel[1])/2;
        const bool arrived=rotating?std::abs(error)<1.5f:(remaining<=2 && std::abs(error)<3);
        if(!rotating && (remaining<-10 || std::abs(travel[0]-travel[1])>20 || travel[0]<-5 || travel[1]<-5))
            return finish(MotionResult::EncoderFault);
        if(arrived) {
            pwm(0,0);
            if(std::abs(measured[0])<5 && std::abs(measured[1])<5 && std::abs(gyroRate)<3) {
                if(!settled) settled=now;
                if(now-settled>=200) { desiredHeading_=targetHeading; return finish(MotionResult::Ok); }
            } else settled=0;
            continue;
        }
        settled=0;
        float left,right;
        if(rotating) {
            const float radius=std::hypot(cal.width/2,std::max(cal.front,cal.rear));
            const float offsets[]={cal.sensorLeft,cal.sensorFront,cal.sensorRight};
            for(int i=0;i<3;++i)
                if(ranges[i]+offsets[i]<radius+3) return finish(MotionResult::Obstacle);
            const float requested=(error>0?1:-1)*std::min(hw::MAX_TURN,std::sqrt(2*hw::TURN_ACCEL*std::abs(error)));
            demand=hw::ramp(demand,requested,hw::TURN_ACCEL*dt);
            left=-demand*hw::PI_F/180*cal.track/2; right=-left;
            // A quarter turn must agree with wheel odometry; abort wrong signs/slippage.
            const float odometry=(travel[1]-travel[0])/cal.track*180/hw::PI_F;
            if(std::abs(odometry-(heading-startHeading))>25) return finish(MotionResult::EncoderFault);
        } else {
            demand=hw::ramp(demand,std::min(hw::MAX_SPEED,std::sqrt(2*hw::ACCEL*std::max(0.0f,remaining))),hw::ACCEL*dt);
            const float correction=hw::clip(error*3,-20,20)*hw::PI_F/180*cal.track/2;
            left=std::max(0.0f,demand-correction); right=std::max(0.0f,demand+correction);
            const float peak=std::max(left,right);
            if(peak>hw::MAX_SPEED) { left*=hw::MAX_SPEED/peak; right*=hw::MAX_SPEED/peak; }
            const float clearance=ranges[1]-(cal.front-cal.sensorFront);
            if(clearance<hw::brakingClearance(std::max(std::abs(measured[0]),std::abs(measured[1]))))
                return finish(MotionResult::Obstacle);
        }
        const float targets[]={left,right}; int output[2];
        for(int i=0;i<2;++i) {
            if(std::abs(targets[i])>8 && now-progressed[i]>700) return finish(MotionResult::Stall);
            output[i]=lround(controllers[i].step(targets[i],measured[i],dt,cal.minPwm[i],cal.feedforward[i]));
        }
        pwm(output[0],output[1]);
    }
}
MotionResult Hardware::benchPulse(int wheel,int duty,uint32_t duration) {
    if(!benchArmed || wheel<0 || wheel>1 || abs(duty)>hw::MAX_PWM || duration>500)
        return finish(MotionResult::NotCalibrated);
    if(aborted) return finish(MotionResult::Stopped);
    if(!waitStopped(100)) return finish(MotionResult::Stopped);
    if(!allFresh()) return finish(MotionResult::SensorFault);
    active=true; pwm(wheel==0?duty:0,wheel==1?duty:0);
    uint32_t start=millis();
    while(millis()-start<duration) {
        if(!idleService()) return finish(MotionResult::Stopped);
        if(!allFresh()) return finish(MotionResult::SensorFault);
        delay(2);
    }
    return finish(MotionResult::Ok);
}
