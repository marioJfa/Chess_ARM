# Stepper Motor Joint Test — Arduino CNC Shield

Build Arduino test code for a stepper-driven robot arm base joint (ring gear / yaw axis).

## Hardware
- Arduino Uno/Mega + CNC Shield v3
- Driver: A4988 on X axis slot
- Motor: stepper (spec unknown — make tunable)
- Drive: ring gear (ratio unknown — make tunable)
- No limit switches yet — open loop

## CNC Shield Pin Mapping (do not change these)
```
X axis STEP → Arduino pin 2
X axis DIR  → Arduino pin 5
Enable      → Arduino pin 8  (LOW = enabled, HIGH = disabled)

Microstepping jumpers (MS1/MS2/MS3 under driver):
  No jumpers    = full step   (1/1)
  MS1 only      = half step   (1/2)
  MS1+MS2       = quarter     (1/4)
  MS1+MS2+MS3   = sixteenth   (1/16)
Default in code: 1/16 microstepping — user must match jumper config
```

## Tunable Parameters (at top of .ino file — clearly labelled)
```cpp
// === MOTOR ===
const int   MOTOR_STEPS_PER_REV = 200;     // full steps/rev (200 = 1.8deg, 400 = 0.9deg)
const int   MICROSTEP            = 16;      // must match jumper: 1, 2, 4, 8, 16
const float GEAR_RATIO           = 1.0;     // ring gear / pinion tooth count ratio — set after calibration
const int   MAX_SPEED_RPM        = 60;      // maximum motor RPM
const int   DEFAULT_SPEED_RPM    = 20;      // startup speed
const int   ACCEL_STEPS          = 200;     // steps to ramp up/down over (simple linear ramp)
const bool  INVERT_DIR           = false;   // flip if rotation direction is wrong

// === CALIBRATION ===
const float DEG_PER_OUTPUT_REV   = 360.0;  // leave at 360 unless you have a partial rotation joint
```

## Three Control Modes — all active simultaneously

### 1. Serial Monitor Commands
Baud rate: 115200
Commands (case insensitive, send with newline):
```
MOVE <degrees>        — rotate output shaft by N degrees (signed: + = CW, - = CCW)
                        example: MOVE 90   MOVE -45   MOVE 360
SPEED <rpm>           — set motor speed in RPM (1–MAX_SPEED_RPM)
HOME                  — rotate back to 0 (tracked position, open loop)
ENABLE                — enable driver (default on startup)
DISABLE               — disable driver (motor free-wheels, use to manually position)
STATUS                — print current position, speed, microstep, gear ratio
STEPS <n>             — raw step command, bypasses degree conversion (for calibration)
CALIBRATE             — print steps-per-degree calculation with current settings
STOP                  — immediate stop
HELP                  — list all commands
```

### 2. Potentiometer Control
- Analog pin A0
- When pot is connected: maps pot range (0–1023) to output angle (0 to +360 degrees)
- Motor follows pot position in real time (positional control, not speed)
- If pot is not connected or reads mid-range on startup: pot mode inactive, serial takes priority
- Print to serial when pot mode activates/deactivates

### 3. Demo Sequence (runs if no serial command and pot inactive)
Loops through:
```
1. Rotate +90°  at default speed
2. Pause 1 second
3. Rotate +90°  (now at 180°)
4. Pause 1 second
5. Rotate -180° (back to 0°)
6. Pause 2 seconds
7. Rotate +360° full rotation
8. Pause 1 second
9. Rotate -360° full rotation back
10. Pause 3 seconds — repeat
```
Demo stops immediately when a serial command is received or pot is moved.

## Position Tracking
- Track current position in steps and degrees (float)
- Track current position as output shaft degrees (accounts for gear ratio)
- Position resets to 0 on ENABLE or on startup
- Serial STATUS prints:
  ```
  === ARM STATUS ===
  Output position : 123.45 deg
  Motor steps     : 98765
  Speed           : 20 RPM
  Microstep       : 1/16
  Gear ratio      : 5.18
  Steps/output deg: 46.2
  Mode            : DEMO / SERIAL / POT
  Driver          : ENABLED
  ==================
  ```

## Motion Implementation
- Use AccelStepper library (handles acceleration cleanly)
- Linear ramp: accelerate over ACCEL_STEPS steps, decelerate same
- Non-blocking: use AccelStepper run() in main loop — never use delay() during motion
- Speed in serial commands is RPM at motor shaft (output shaft speed = RPM / GEAR_RATIO)

## Calibration Helper
CALIBRATE command prints this to serial:
```
=== CALIBRATION HELPER ===
Current settings:
  Motor steps/rev : 200 (full steps)
  Microstep       : 1/16
  Gear ratio      : 1.00
  Steps/motor rev : 3200
  Steps/output rev: 3200
  Steps/degree    : 8.89

To find your gear ratio:
  1. Mark the output shaft
  2. Send: STEPS 3200  (one full motor revolution)
  3. Measure how many degrees the output moved
  4. Gear ratio = 360 / degrees_moved
  5. Update GEAR_RATIO in code and re-upload
==========================
```

## File Structure
```
stepper_test/
├── stepper_test.ino       (main sketch)
├── motion.h / motion.cpp  (AccelStepper wrapper, step calculations)
├── commands.h / commands.cpp (serial parser)
├── config.h               (all tunable params — include this everywhere)
└── README.md              (wiring diagram + calibration procedure)
```

## README.md Must Include
- Wiring table (Arduino pin → CNC shield → A4988 → motor)
- Jumper config table for all microstepping options
- Step-by-step calibration procedure for finding gear ratio
- Common problems: motor buzzes but doesn't move (current too low), motor skips (speed too high), direction wrong (INVERT_DIR)
- Formula reference: steps_per_degree = (MOTOR_STEPS_PER_REV × MICROSTEP × GEAR_RATIO) / 360

## Libraries Required
- AccelStepper (Mike McCauley) — install via Arduino Library Manager
- No other external libraries

## Quality Rules
- All magic numbers in config.h — no hardcoded values in logic files
- Every function has a comment explaining what it does
- Serial output is clean and readable — not spammy (pot position only prints on change > 1 deg)
- Code must compile on both Uno and Mega without changes
