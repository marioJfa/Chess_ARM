---
name: Hardware knowledge
description: Physical arm control architecture — Pico role, A4988, stepper firmware, arm calculator, ADC notes, ROS hardware topics
type: project
---

## Control Architecture

```
ROS 2 (PC)
    │  joint angle targets (~50Hz)
    ▼
Pico 1 (micro-ROS) — base_yaw + shoulder
Pico 2 (micro-ROS) — elbow + wrist
    │
    ├── Reads potentiometers (onboard ADC)
    ├── Reads encoders (interrupt pins) — if fitted
    ├── Compares commanded vs actual angle
    ├── Flags drift > threshold to ROS
    └── Reports actual joint positions back to ROS
    │
    ▼
Arduino CNC Shield
    └── A4988 drivers → stepper motors (open-loop)
```

## Pico Assignment

| Pico | Joints | ADC pins | Spare ADC |
|---|---|---|---|
| Pico 1 | base_yaw, shoulder | GP26, GP27 | GP28 |
| Pico 2 | elbow, wrist | GP26, GP27 | GP28 |

## Pico Role — Watchdog/Feedback Only

- Steppers are open-loop (A4988, no step feedback)
- Picos do NOT control motors — Arduino CNC Shield does
- Picos monitor actual joint angle via pots and report to ROS
- Drift beyond threshold → publish warning to `/arm/joint_drift`
- ROS decides what to do (stop, recalibrate, continue)

## A4988 Notes

- Current limit set via trim pot — set carefully, main cause of missed steps and heat
- `ENABLE` pin active low — CNC shield handles this
- Logic 5V, motor voltage separate (up to 35V)
- Microstepping up to 1/16 via MS1/MS2/MS3 pins on shield
- Vref formula: `Vref = I_max × 8 × R_sense` (R_sense = 0.1 Ω typical)
  - Example: 0.5 A motor → Vref = 0.4 V
- Start low, increase until torque is sufficient without heat

## Pico ADC Notes

- Known noisy ADC — add 100nF cap between wiper pin and GND at the pot
- Use `AGND` and `ADC_VREF` pins, not regular GND
- Average 8–16 readings per sample in firmware
- 12-bit ADC → ~0.066° resolution over 270° pot range

## Hardware ROS Topics

| Topic | Type | Direction |
|---|---|---|
| `/arm/joint_targets` | JointState | ROS → Picos (commanded angles) |
| `/arm/joint_actual` | JointState | Picos → ROS (measured angles) |
| `/arm/joint_drift` | String (JSON) | Picos → ROS (drift warnings) |

---

## stepper_test Firmware (hardware/stepper_test/)

Arduino Uno/Mega + CNC Shield v3 + A4988 (X-axis slot). Single-axis test for ring-gear yaw joint.

### Files
- `stepper_test.ino` — main sketch, mode state machine
- `config.h` — all tunable params (motor steps, microstep, gear ratio, speed, pins)
- `motion.h / motion.cpp` — AccelStepper wrapper, degree-to-step conversion
- `commands.h / commands.cpp` — serial command parser

### Three control modes (priority: serial > pot > demo)
1. **Serial** — highest priority, overrides demo when any command received
2. **Pot** — A0 potentiometer, activates when moved >5° from centre, deactivates at centre
3. **Demo** — default loop: 90° → 90° → -180° → 360° → -360° with pauses

### Key config (config.h defaults)
| Param | Value | Notes |
|---|---|---|
| MOTOR_STEPS_PER_REV | 200 | 1.8 deg/step |
| MICROSTEP | 16 | jumpers must match |
| GEAR_RATIO | 1.0 | calibrate with CALIBRATE command |
| MAX_SPEED_RPM | 60 | |
| DEFAULT_SPEED_RPM | 20 | |
| INVERT_DIR | false | flip if CW/CCW reversed |
| PIN_STEP/DIR/ENABLE | 2/5/8 | CNC Shield X-axis, do not change |
| PIN_POT | A0 | |

### Serial commands (115200 baud)
`MOVE <deg>`, `SPEED <rpm>`, `HOME`, `ENABLE`, `DISABLE`, `STATUS`, `STEPS <n>`, `CALIBRATE`, `STOP`, `HELP`

### Gear ratio calibration procedure
1. Send `CALIBRATE` — note steps/motor rev
2. Mark output shaft, send `STEPS <steps_per_motor_rev>`
3. Measure actual output degrees moved
4. `GEAR_RATIO = 360 / degrees_measured`
5. Update config.h, re-upload, confirm with `CALIBRATE`

### Steps/degree formula
`steps_per_degree = (MOTOR_STEPS_PER_REV × MICROSTEP × GEAR_RATIO) / 360`

### Wiring
- CNC Shield stacks on Arduino (no extra wiring)
- A4988 → X-axis socket, ENABLE pin toward USB end
- Motor power: 8–35V to green terminal (V+/GND on shield)
- Pot (optional): 10kΩ, ends to 5V/GND, wiper to A0

### Common problems
| Symptom | Fix |
|---|---|
| Hums, doesn't turn | Swap A+/A- OR B+/B- (not both) |
| Skips steps | Lower speed or increase Vref |
| Wrong direction | Set INVERT_DIR true, or swap coil wires |
| Overheating | Lower Vref |
| Pot won't activate | Move away from centre before power-up |

---

## Arm Calculator (hardware/arm_calculator/)

FastAPI web app — torque, weight, motor requirements, Fusion 360 CSV export.

### Stack
- `calculator.py` — pure physics functions
- `main.py` — FastAPI, `/calculate` + `/export` endpoints
- `static/index.html + style.css + app.js` — single-page UI

### Key calculations
- Tube mass: `π/4 × (OD² - ID²) × length × 2700 kg/m³` (aluminium)
- Static torque per joint at given pose angles
- Motor requirement: `joint_torque / gear_ratio / (pulley_r / spool_r)`
- Cable tension: `joint_torque / pulley_radius` vs `break_strength / safety_factor`

### Default geometry
- Link lengths: 200 / 180 / 80 mm
- Tube: 22mm OD, 2mm wall, 18mm ID
- Spool: 11mm radius, pulleys: 15mm radius
- Line: 133 N (30 lb braid), safety factor 3.0

### Fusion 360 export params
Exports: link lengths, tube dims, shaft/bearing/hub dims, spool/pulley, connector dims (clearance 0.3mm, socket depth 25mm, bolt circle 30mm), joint limits

---

## hardware/ Directory

- `hardware/arm_calculator/` — FastAPI torque/geometry tool
- `hardware/stepper_test/` — Arduino CNC Shield + A4988 test sketches
- `hardware/drawings/` — SVG arm geometry (side view, top view)
- `hardware/params/` — Fusion 360 exported params (CSV)
- `hardware/robot_arm_plan.md` — full design plan (phases 1–9)

## Design Status (from robot_arm_plan.md)

- Phase 1 ✅ — parameters, architecture, bore layout, Fusion 360 CSV
- Phase 2 🔄 — tube socket connector (start here in Fusion 360)
- Phases 3–9 ⏳ — pending motor selection + gripper motor location decision

### Pending decisions blocking design
- Motor model — updates all NEMA17 placeholders
- Gripper motor location (base vs wrist) — affects base OD and bay count

## Proactive Checks When Touching Hardware Code

- Does commanded angle match what the pot is actually reading in sim/test?
- Is drift threshold appropriate for the joint's expected load?
- Is the Pico publishing at sufficient rate (min 50Hz for control)?
- Are stepper step counts staying in sync with commanded positions across a full move sequence?
