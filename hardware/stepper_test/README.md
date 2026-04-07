# stepper_test — Arduino CNC Shield Stepper Motor Test

Single-axis stepper test for a ring-gear yaw joint.
Hardware: Arduino Uno/Mega + CNC Shield v3 + A4988 driver (X-axis slot).

---

## Wiring

### Arduino → CNC Shield (no wiring needed — shield stacks directly)

| CNC Shield label | Arduino pin | Signal      |
|------------------|-------------|-------------|
| X-STEP           | 2           | Step pulse  |
| X-DIR            | 5           | Direction   |
| EN               | 8           | Enable (active LOW) |
| A0 header        | A0          | Potentiometer wiper (optional) |

### CNC Shield → A4988

The A4988 plugs into the X-axis socket on the shield (pins are labelled).
Orientation: ENABLE pin toward the USB end of the Arduino.

### A4988 → Motor

| A4988 pin | Motor wire |
|-----------|------------|
| 1A        | Coil A+    |
| 1B        | Coil A-    |
| 2A        | Coil B+    |
| 2B        | Coil B-    |

If the motor hums but does not turn, swap either A+/A- OR B+/B- (not both).

### Power

- Motor power (VMOT): 8–35 V DC to the green terminal on the CNC shield (shield labelled V+ / GND)
- Logic power: USB from Arduino
- **Always connect motor power before enabling the driver**

### Potentiometer (optional)

Connect a 10 kΩ potentiometer:
- One end → 5 V
- Other end → GND
- Wiper → A0

---

## Microstepping Jumpers

Jumpers are located under the A4988 chip on the CNC shield (MS1, MS2, MS3).
**Set `MICROSTEP` in `config.h` to match your jumper config.**

| MS1 | MS2 | MS3 | Mode           | MICROSTEP value |
|-----|-----|-----|----------------|-----------------|
| –   | –   | –   | Full step      | 1               |
| ▪   | –   | –   | Half step      | 2               |
| ▪   | ▪   | –   | Quarter step   | 4               |
| –   | –   | ▪   | Eighth step    | 8               |
| ▪   | ▪   | ▪   | Sixteenth step | 16 ← default    |

---

## Current Tuning (A4988)

The small trimmer pot on the A4988 sets motor current.
Turn it **clockwise** to increase current.

Formula: `Vref = I_max × 8 × R_sense`
Typical R_sense on A4988 boards = 0.1 Ω
Example: 0.5 A motor → Vref = 0.5 × 8 × 0.1 = **0.4 V**

Start low, increase until the motor has enough torque without overheating.

---

## Libraries Required

Install via **Sketch → Include Library → Manage Libraries**:

- **AccelStepper** by Mike McCauley

No other external libraries needed.

---

## Calibration Procedure (Finding Gear Ratio)

1. Open Serial Monitor at **115200 baud**.
2. Send: `CALIBRATE` — note the *Steps/motor rev* value shown.
3. Mark the output shaft position with tape.
4. Send: `STEPS <steps_per_motor_rev>` (e.g. `STEPS 3200` for 200-step motor at 1/16).
5. Measure how many degrees the output shaft actually moved.
6. Calculate: `GEAR_RATIO = 360 / degrees_measured`
7. Update `GEAR_RATIO` in `config.h` and re-upload.
8. Send `CALIBRATE` again to confirm *Steps/output deg* looks correct.

---

## Serial Commands

Baud rate: **115200**. Send commands with newline (Arduino Serial Monitor default).

| Command | Description |
|---------|-------------|
| `MOVE <deg>` | Move output shaft N degrees. Signed: `+` = CW, `-` = CCW |
| `SPEED <rpm>` | Set motor shaft speed in RPM |
| `HOME` | Return to 0° (open loop, tracks from last ENABLE) |
| `ENABLE` | Enable driver — resets position counter to 0 |
| `DISABLE` | Disable driver — motor free-wheels (use to manually position) |
| `STATUS` | Print position, speed, config |
| `STEPS <n>` | Raw step move, bypasses gear ratio (calibration use) |
| `CALIBRATE` | Print gear ratio calibration guide |
| `STOP` | Immediate stop |
| `HELP` | List all commands |

---

## Formula Reference

```
steps_per_degree = (MOTOR_STEPS_PER_REV × MICROSTEP × GEAR_RATIO) / 360
```

Example — 200-step motor, 1/16 microstepping, 5.18:1 gear:
`(200 × 16 × 5.18) / 360 = 46.2 steps/degree`

---

## Common Problems

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Motor buzzes, does not turn | Current too low | Increase Vref on A4988 trimmer |
| Motor skips steps, loses position | Speed too high or current too low | Lower `MAX_SPEED_RPM` or increase current |
| Direction is backwards | Motor wires or INVERT_DIR | Set `INVERT_DIR true` in config.h, or swap A+/A- wires |
| Motor is very hot | Current too high | Lower Vref |
| No serial output | Wrong baud rate | Set Serial Monitor to 115200 |
| Motor always enabled / can't free-wheel | ENABLE pin floating | Ensure PIN 8 is driven (motionInit handles this) |
| Pot does not activate | Pot reads mid-range at startup | Move pot away from centre before powering up |
