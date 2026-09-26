# ESP32-C6 slow maze firmware

The shared flood-fill solver now drives the physical robot through `RobotPlatform`.
The sequence is centre → start → centre, with the same conservative profile in all
three phases. This is a calibration-first implementation: compiling or uploading it
does not prove that the physical robot drives accurately.

## Build and upload

From the repository directory:

```sh
python3 scripts/firmware.py                         # compile only
python3 scripts/firmware.py --upload /dev/cu.usbmodem1101
python3 scripts/test.py                            # desktop regression
SANITIZE=1 python3 scripts/test.py                 # optional address/UB checks
```

Requires Python 3, Arduino CLI, Arduino ESP32 core **3.3.12**, and Pololu
**VL53L0X 1.3.1**. These are already installed on the team's Mac. The script finds
Arduino IDE's bundled CLI and its configuration, or accepts `--cli` and `--config`.
Target: `esp32:esp32:esp32c6:CDCOnBoot=cdc`. It copies `src/` into a generated sketch;
there is one maintained copy of the maze/solver code. Generated files live in `build/`.
Serial Monitor: **115200 baud**, newline enabled, USB data port. Close other monitors
before upload. Boot leaves motor PWM at zero, even with previously saved calibration.

## Wiring

| Function | GPIO |
|---|---|
| Left motor DIR / PWM | 0 / 2 |
| Right motor DIR / PWM | 3 / 10 |
| Left encoder A / B | 21 / 22 |
| Right encoder A / B | 23 / 11 |
| I2C SDA / SCL | 6 / 7 |
| Left / front / right ToF XSHUT | 18 / 19 / 20 |
| Onboard BOOT start/stop | 9 |

Both motor polarities are inverted from the earlier wheel test. Encoder signs are
measured independently: motor polarity does not establish encoder polarity.

The I2C bus runs at **100 kHz**. XSHUT holds all ToFs off, then wakes them individually:
left `0x30`, front `0x31`, right `0x29`. MPU is discovered at `0x68` or `0x69` and
configured for ±500 degrees/s. Mount it rigidly, with its Z axis vertical.

For USB tests: USB powers ESP32; the raw 9 V battery branch powers motor-driver VM;
UBEC 5 V output is disconnected and insulated. For battery-only operation: unplug USB,
then use UBEC 5 V for ESP32. Share ground. Sensor/encoder logic uses 3.3 V. Do not put
9 V on ESP32 or sensor power, and do not connect USB and UBEC 5 V to ESP32 together.

## Dimensions and what is still unknown

| Measurement | Initial value |
|---|---:|
| Square centre-to-centre pitch | 180 mm, team confirmed |
| Wheel-centre spacing | 80 mm, team measured |
| Body/wheel width | 95 mm, team measured |
| Overall width used for clearance | 100 mm, from the side-sensor span |
| Axle to frontmost edge, including sensors | 60 mm, team measured |
| Axle to rearmost edge | 35 mm, team measured |
| Wall thickness | 12 mm, **provisional assumption** |
| Left/front/right sensor face offsets | 50 / 45 / 50 mm, team measured |
| Millimetres per encoder edge and signs | **must measure** |
| Minimum starting PWM | **must measure** |

These dimensions describe a 78.1 mm maximum corner radius. With assumed 12 mm walls,
84 mm is available from square centre to the nearest wall face: about 5.9 mm geometric
clearance, or 2.9 mm after the firmware's 3 mm margin. This assumes accurate centring
and no protruding wires. Verify the wall thickness and pivot physically; a ±10 mm
straight-distance result alone is not enough to establish turning clearance.

`offset-front` is the front sensor's optical face distance **forward from the axle**.
`offset-left` / `offset-right` are the optical faces' distances **sideways from the
axle midpoint**, positive on both sides. Sensors must face straight front/left/right.
The team's measured offsets are prefilled; negative/unknown offsets prevent saving
calibration. To correct a value use `set offset-front NUMBER` (replace NUMBER; units are mm).
Other dimension commands are `set track`, `set width`, `set front`, `set rear`, `set wall`.

## Calibration: do this before powered floor moves

1. **USB connected, motors stopped.** Disconnect motor power if possible; otherwise
   support the chassis securely with both wheels clear of the surface and send `stop`.
   Keep BOOT released and leave `bench` disarmed during hand measurements. Enter `status` and `config`. Check all three
   range entries read `[11,ok]` with wall-like targets in view. `sensors` toggles a
   200 ms stream. Move a hand in front of each sensor separately and confirm L/F/R.
   An invalid/no-return reading is a fault, never an open passage; a target beyond
   useful sensor range can therefore prevent movement, including in a long corridor.
2. **Robot still:** enter `gyro`. Rotate the entire robot counter-clockwise by a
   measured 90 degrees. `status` should show about +90 degrees. If it shows -90,
   enter `set gyro-sign -1`, recalibrate with `gyro` while still, and repeat. Do not
   confirm sensors until both this check and the L/F/R check pass.
3. **Encoder distance, motor battery still off:** enter `mark`, then roll both wheels
   forward along a measured straight distance, e.g. 360 mm without sliding. Enter
   `distance 360`. This derives each encoder's sign and mm per quadrature edge from
   the actual counts. Alternatively use `mark`, turn one wheel forward exactly N
   full rotations, then `wheel left DIAMETER_MM N`; repeat for `right`. More turns
   reduce measurement error. `encoders` prints raw counts and illegal transitions.

   **If the motor battery cannot be disconnected:** keep the chassis securely raised
   and issue `stop`. Use the single-wheel method above with the actual wheel diameter:
   `mark`, gently rotate one wheel forward a known whole number of turns, then
   `wheel left DIAMETER_MM N` (or `right`). Do not force a gearbox that resists turning.
   These commands only read encoder counts; they never energize a motor. Stay out of
   `bench` and do not hold BOOT during this manual measurement. The generic `mark`
   message recommends disconnecting motor power for the floor-roll method; use this
   raised-wheel alternative when that is unavailable. Battery connection alone does
   not supply the missing encoder scale, and does not bypass the calibration gate.
4. **Wheels raised, raw motor power connected:** enter `bench`, then
   `pulse left 80 250`. Verify forward direction, nonzero counts with the calibrated
   forward sign, and a complete stop. Repeat for right, then negative duty for
   backward. `!` stops immediately; BOOT also stops. No motion command is sent by
   the build script. Each diagnostic pulse is limited to 500 ms / 170 PWM.
5. **Find starting power:** `bench`, `sweep left`, then `bench`, `sweep right`.
   The sweep tries 40 through 150 PWM, in short pulses, and records the first encoder
   movement. These are raised-wheel estimates; check under load later. You can set
   measured values with `set min-left VALUE` / `set min-right VALUE`.
6. Enter measured sensor offsets and any corrected dimensions. Enter
   `confirm-sensors` only after the orientation and range tests pass, then `save`.
   `config` must report saved=1. Confirmed geometry, encoder scale/sign, starting PWM,
   speed feedforward, gyro sign and the current gyro bias are stored in ESP32 NVS.
   Gyro bias is remeasured while stationary at every boot and autonomous start.
   Parameter edits invalidate the current authorization until confirmed/saved again.

Place targets in view of all three sensors during raised-wheel tests. A sensor fault
prevents even a diagnostic pulse. If a sensor disconnects, stop, repair it and reboot;
initialization is deliberately not retried while driving.

## Slow floor tests

`home` is a physical-placement acknowledgement, not a motor command. Place the axle
midpoint at the test mark, facing north, then enter it. Use `move 20` first. After
each manual move/turn, reposition and enter `home` again before the next test.

| USB command | Action |
|---|---|
| `status`, `encoders` | One sensor/heading/encoder report |
| `sensors` | Toggle streaming while idle |
| `config` | Calibration and geometry |
| `gyro` | Stationary gyro-bias measurement; zero relative heading |
| `home` | Acknowledge manual placement facing north |
| `move 20` | Short forward move; allowed range 5–180 mm |
| `cell` | 180 mm forward, stop |
| `left` / `right` | 90° in place, stop |
| `stop`, `s`, `!` | Stop; fresh home/bench acknowledgement required |
| `go` | Countdown, then all three phases |

During motion/countdown, any `s` or `!` character stops; other input is discarded
(including queued starts). Thus typing `status` while moving also stops the robot.
`!` needs no newline. Do not send a batch containing future starts. Losing USB does
not stop a battery-only run; BOOT remains the physical stop control.

Run five independent `cell` tests and measure actual axle travel against 180 mm;
each must be within ±10 mm, and tighter centring is needed for this chassis's pivot.
Refine mm/tick using another measured roll if scale is wrong, and `min-left/right`
and `ff-left/right` if speed tracking is poor (feedforward units PWM per mm/s).
Run five `left` and five `right` tests, each within ±5°. If gyro turns disagree with
physical angles, fix mounting/configuration before running a maze. Do not compensate
for a wrong gyro sign by swapping algorithm directions.

Then test a small maze: walls/openings correctly sensed at the centre, stops before
walls, straight moves, a corner and consecutive turns. Finally test battery-only
starting and stopping, watch for resets under motor load, and run all three phases.
**These physical checks require observations; software tests do not satisfy them.**

## Start, stop and faults

Battery-only: place the axle at the start-square centre, facing north. Hold BOOT for
at least 1.5 seconds and release. That gesture acknowledges manual placement and
starts a 3-second countdown. Any press during countdown, gyro preparation or driving
cancels/stops. Release the stop press fully; it cannot double as a restart hold.
A new deliberate hold/release is needed for another start. Holding BOOT while powering
on may enter ESP download mode; wait until the board has booted first.

Movement uses wheel-speed PI feedback and MPU heading correction, acceleration/braking
ramps, a fixed cardinal heading reference across consecutive turns, at most 60 mm/s straight requests, 35°/s turn requests, and absolute 170/255 PWM.
Each cell move stops before sensing. U-turns are two independently completed 90° turns.
A completed quarter-turn updates heading; a completed cell move updates coordinates.
A partial move or failed turn stops the run and requires manual placement back at the
start. Maze memory starts fresh on a new physical run; simulator Reset retains its map.

Faults include stale/missing/invalid range or gyro data, a wheel stalled for 700 ms,
10-second motion timeout, encoder travel disagreement/wrong sign, turn/odometry
inconsistency, an obstacle inside braking clearance, impossible geometry, and an
unreachable solver goal. There is no timed fallback for navigation. Timed `pulse`/
`sweep` commands are explicitly armed, raised-wheel calibration diagnostics only.

Wall threshold = `(180 − wall thickness)/2 − sensor face offset + 45 mm`, evaluated
on five stopped samples per sensor using their median. Raw VL53L0X device status 11
and a nonzero range below 2000 mm are required; missing and invalid values never
enter the map as open passages. This uses the sensor's internal validity checks,
not the ST full API's optional host-side sigma estimator. Validate on the real wall
material/light conditions. Gyro data older than 100 ms and range data older than
250 ms are rejected. Front braking clearance includes 8 mm margin, 150 ms latency
allowance and `speed²/(2 × 120 mm/s²)`. Turning additionally checks sensed clearance
against the chassis radius. These are conservative starting parameters requiring
physical verification, not a guarantee against collision.

References: [event resources](https://hackclub.ucdelecsoc.com/micromouse-resources),
[Pololu library](https://github.com/pololu/vl53l0x-arduino),
[ST range-status mapping in the installed Adafruit library](https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/core/src/vl53l0x_api_core.cpp),
[Arduino ESP32 LEDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html).
