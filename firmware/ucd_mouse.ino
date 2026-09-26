#include "Hardware.h"
#include "src/MouseAgent.h"
#include <cstdio>
#include <cstring>

Hardware robot;
char lineBuffer[160];
size_t lineLength=0;
bool lineOverflow=false, streaming=false;
uint32_t streamAt=0, buttonDownAt=0;
bool buttonDown=false;
int64_t markTicks[2]={};
bool marked=false;

void help() {
    Serial.println("UCD slow mouse: motors stay stopped until commanded.");
    Serial.println("status | sensors (toggle stream) | encoders | config | gyro | stop / s / !");
    Serial.println("mark ; roll FORWARD a measured distance ; distance <mm>");
    Serial.println("Alternative: mark ; rotate one wheel forward N turns ; wheel <left|right> <diameter_mm> <N>");
    Serial.println("set <track|width|front|rear|wall|offset-left|offset-front|offset-right> <mm>");
    Serial.println("set <min-left|min-right|ff-left|ff-right|gyro-sign> <value>");
    Serial.println("bench (wheels raised) ; pulse <left|right> <signed_pwm> [milliseconds<=500]");
    Serial.println("bench ; sweep <left|right> (find minimum starting PWM, wheels raised)");
    Serial.println("confirm-sensors ; save (only after geometry, distance, deadband and sensor checks)");
    Serial.println("home (confirm axle at start-cell centre facing NORTH) ; move <5..180 mm> | cell | left | right");
    Serial.println("home ; go (three slow phases, 3-second countdown). Every manual move requires home again.");
    Serial.println("Battery-only: place at start, hold BOOT 1.5 s and release. Press BOOT anytime to stop.");
    Serial.println("9V motors <=170 PWM. ESP32 USB OR UBEC 5V, never both. Rail + is 3.3V.");
}
void printConfig() {
    const auto& c=robot.cal;
    Serial.printf("cell=180 speed=60 turn=35 pwm_cap=170 motor_sign=-1,-1\n");
    Serial.printf("mm/tick=%.6f,%.6f signs=%ld,%ld minPWM=%.1f,%.1f ff=%.3f,%.3f gyroSign=%ld\n",
      c.mmPerTick[0],c.mmPerTick[1],(long)c.encoderSign[0],(long)c.encoderSign[1],c.minPwm[0],c.minPwm[1],c.feedforward[0],c.feedforward[1],(long)c.gyroSign);
    Serial.printf("track=%.1f width=%.1f front=%.1f rear=%.1f wall=%.1f offsets LFR=%.1f,%.1f,%.1f mm\n",
      c.track,c.width,c.front,c.rear,c.wallThickness,c.sensorLeft,c.sensorFront,c.sensorRight);
    Serial.printf("geometry=%s scale=%s sensorsConfirmed=%lu saved=%d\n",hw::geometryValid(c)?"OK":"MISSING/NO TURN CLEARANCE",
      hw::scaleValid(c)?"OK":"MISSING",(unsigned long)c.sensorsConfirmed,robot.calibrated());
    Serial.printf("gyro bias=%.3f dps (rechecked at boot/start); wall=12 mm is provisional until measured\n",c.gyroBias);
}
void runMaze() {
    robot.benchArmed=false;
    if(!robot.calibrated() || !robot.homed) {
        Serial.println("REFUSED: calibrate/save, place axle at start centre facing north, then home."); return;
    }
    robot.aborted=false;
    Serial.println("START in 3 seconds. Button or ! stops.");
    if(!robot.waitStopped(3000)) { robot.homed=false; return; }
    // Bias belongs to this stationary start, not an earlier laptop session.
    if(!robot.calibrateGyro()) { robot.homed=false; Serial.println("REFUSED: gyro moved or missing"); return; }
    if(!robot.waitStopped(200) || !robot.allFresh()) { robot.homed=false; Serial.println("REFUSED: missing/invalid sensor"); return; }
    MouseAgent agent(robot);
    const auto result=agent.run();
    robot.stop(); robot.homed=false;
    Serial.printf("RUN %s cell=%d,%d heading=%d. Reposition at start before home/restart.\n",
        resultName(result),agent.x(),agent.y(),int(agent.heading()));
}
int wheelNumber(const char* name) {
    if(!strcmp(name,"left")) return 0;
    if(!strcmp(name,"right")) return 1;
    return -1;
}
void changed() { robot.invalidate(); robot.benchArmed=false; Serial.println("Changed in RAM; confirm-sensors and save again before floor movement."); }
void command(const char* text) {
    if(!strcmp(text,"help")) { help(); return; }
    if(!strcmp(text,"stop") || !strcmp(text,"s") || !strcmp(text,"!")) {
        robot.aborted=true; robot.stop(); robot.homed=false; robot.benchArmed=false; Serial.println("STOPPED; home or bench required"); return;
    }
    if(!strcmp(text,"status") || !strcmp(text,"encoders")) { robot.report(); return; }
    if(!strcmp(text,"config")) { printConfig(); return; }
    if(!strcmp(text,"sensors")) { streaming=!streaming; return; }
    if(!strcmp(text,"gyro")) {
        robot.stop(); robot.aborted=false; robot.homed=false; robot.benchArmed=false;
        Serial.println(robot.calibrateGyro()?"GYRO OK":"GYRO FAILED: hold still/check wiring"); return;
    }
    if(!strcmp(text,"mark")) { robot.counts(markTicks); marked=true; Serial.println("Encoder baseline marked. Roll FORWARD, with motor battery off."); return; }
    float value;
    if(sscanf(text,"distance %f",&value)==1) {
        if(!marked || !hw::finiteBetween(value,50,2000)) { Serial.println("Need mark and distance 50..2000 mm"); return; }
        int64_t ticks[2]; robot.counts(ticks);
        hw::Calibration candidate=robot.cal;
        for(int i=0;i<2;++i) {
            const int64_t delta=ticks[i]-markTicks[i];
            if(llabs(delta)<20) { Serial.println("TOO FEW TICKS: check encoder wiring; no calibration applied"); return; }
            candidate.mmPerTick[i]=value/llabs(delta); candidate.encoderSign[i]=delta>0?1:-1;
        }
        if(!hw::scaleValid(candidate)) { Serial.println("Invalid scale; no calibration applied"); return; }
        robot.cal=candidate; marked=false; changed(); printConfig(); return;
    }
    char name[32]; float diameter,turns;
    if(sscanf(text,"wheel %31s %f %f",name,&diameter,&turns)==3) {
        int i=wheelNumber(name); int64_t c[2]; robot.counts(c);
        if(!marked || i<0 || !hw::finiteBetween(diameter,10,100) || !hw::finiteBetween(turns,1,20)) { Serial.println("Need mark, left/right, diameter 10..100 mm, 1..20 forward turns"); return; }
        int64_t delta=c[i]-markTicks[i];
        if(llabs(delta)<20) { Serial.println("TOO FEW TICKS"); return; }
        float scale=hw::PI_F*diameter*turns/llabs(delta);
        if(!hw::finiteBetween(scale,0.001f,5)) { Serial.println("Invalid scale"); return; }
        robot.cal.mmPerTick[i]=scale; robot.cal.encoderSign[i]=delta>0?1:-1; marked=false; changed(); return;
    }
    if(sscanf(text,"set %31s %f",name,&value)==2) {
        if(!std::isfinite(value)) { Serial.println("Finite number required"); return; }
        float* dest=nullptr;
        if(!strcmp(name,"track")) dest=&robot.cal.track;
        if(!strcmp(name,"width")) dest=&robot.cal.width;
        if(!strcmp(name,"front")) dest=&robot.cal.front;
        if(!strcmp(name,"rear")) dest=&robot.cal.rear;
        if(!strcmp(name,"wall")) dest=&robot.cal.wallThickness;
        if(!strcmp(name,"offset-left")) dest=&robot.cal.sensorLeft;
        if(!strcmp(name,"offset-front")) dest=&robot.cal.sensorFront;
        if(!strcmp(name,"offset-right")) dest=&robot.cal.sensorRight;
        if(!strcmp(name,"min-left")) dest=&robot.cal.minPwm[0];
        if(!strcmp(name,"min-right")) dest=&robot.cal.minPwm[1];
        if(!strcmp(name,"ff-left")) dest=&robot.cal.feedforward[0];
        if(!strcmp(name,"ff-right")) dest=&robot.cal.feedforward[1];
        if(!strcmp(name,"gyro-sign") && (value==1 || value==-1)) { robot.cal.gyroSign=value; changed(); return; }
        if(!dest) { Serial.println("Unknown setting (help)"); return; }
        *dest=value; changed(); return;
    }
    if(!strcmp(text,"bench")) {
        robot.stop(); robot.aborted=false; robot.homed=false; robot.benchArmed=true;
        Serial.println("BENCH armed: wheels must be raised. Only explicit pulse/sweep moves them."); return;
    }
    int duty,ms=300;
    const int pulseArgs=sscanf(text,"pulse %31s %d %d",name,&duty,&ms);
    if(pulseArgs>=2) {
        if(ms<1 || ms>500 || duty < -hw::MAX_PWM || duty > hw::MAX_PWM) { Serial.println("Limit: PWM +/-170, duration 1..500 ms"); return; }
        robot.benchPulse(wheelNumber(name),duty,ms); robot.report(); return;
    }
    if(sscanf(text,"sweep %31s",name)==1) {
        int i=wheelNumber(name);
        if(i<0 || !robot.benchArmed) { Serial.println("Use bench first; select left or right"); return; }
        for(int d=40;d<=150;d+=5) {
            int64_t before[2],after[2]; robot.counts(before);
            if(robot.benchPulse(i,d,250)!=MotionResult::Ok) return;
            if(!robot.waitStopped(200)) { robot.homed=false; return; }
            robot.counts(after);
            if(llabs(after[i]-before[i])>=4) {
                robot.cal.minPwm[i]=d; changed();
                Serial.printf("Starting PWM %s=%d (raised-wheel estimate; verify under load)\n",name,d); return;
            }
        }
        robot.stop(); robot.benchArmed=false;
        Serial.println("No encoder motion at PWM <=150; inspect battery, motor and encoder wiring."); return;
    }
    if(!strcmp(text,"confirm-sensors")) {
        if(!robot.allFresh()) { Serial.println("REFUSED: not all readings valid/fresh"); return; }
        robot.cal.sensorsConfirmed=1;
        Serial.println("Recorded your confirmation: L/F/R match hands, gyro increases CCW and quarter-turn reads ~90 deg."); return;
    }
    if(!strcmp(text,"save")) { Serial.println(robot.save()?"CALIBRATION SAVED":"REFUSED: config lists missing calibration/geometry"); return; }
    if(!strcmp(text,"home")) {
        robot.acknowledgeHome();
        Serial.println("Start placement acknowledged: axle centre at (0,0), facing NORTH."); return;
    }
    if(!strcmp(text,"go")) { runMaze(); return; }
    const bool move=sscanf(text,"move %f",&value)==1;
    if(move || !strcmp(text,"cell") || !strcmp(text,"left") || !strcmp(text,"right")) {
        if(!robot.homed || robot.aborted) { Serial.println("REFUSED: home acknowledgement required"); return; }
        robot.benchArmed=false;
        if(move) robot.drive(value);
        else if(!strcmp(text,"cell")) robot.moveOneCell();
        else robot.turnQuarter(!strcmp(text,"right"));
        robot.homed=false; return;
    }
    Serial.println("Unknown command. Type help.");
}
void setup() {
    // Set outputs low before waiting for USB; standalone boot never waits for Serial.
    for(int pin:hw::PWM) { digitalWrite(pin,LOW); pinMode(pin,OUTPUT); }
    Serial.begin(115200);
    uint32_t start=millis(); while(!Serial && millis()-start<1200) delay(1);
    robot.begin(); help();
}
void loop() {
    robot.poll();
    while(Serial.available()) {
        char c=Serial.read();
        if(c=='!') { command("!"); lineLength=0; lineOverflow=false; continue; }
        if(c=='\n' || c=='\r') {
            if(lineOverflow) Serial.println("Command too long; discarded");
            else if(lineLength) { lineBuffer[lineLength]=0; command(lineBuffer); }
            lineLength=0; lineOverflow=false;
        } else if(lineLength<sizeof(lineBuffer)-1) lineBuffer[lineLength++]=c;
        else lineOverflow=true;
    }
    bool down=digitalRead(hw::BUTTON)==LOW;
    // A press that stopped a maneuver cannot also become a restart hold.
    if(robot.buttonStopLatched) {
        buttonDown=false;
        if(!down) robot.buttonStopLatched=false;
        delay(2); return;
    }
    if(down && !buttonDown) { buttonDown=true; buttonDownAt=millis(); }
    if(!down && buttonDown) {
        buttonDown=false;
        if(millis()-buttonDownAt>=1500) {
            // Deliberate hold/release is the physical equivalent of 'home; go'.
            robot.acknowledgeHome(); runMaze();
        }
    }
    if(streaming && millis()-streamAt>=200) { streamAt=millis(); robot.report(); }
    delay(2);
}
