---
name: Hardware knowledge
description: Physical arm control architecture — Pico role, A4988 stepper driver notes, ADC wiring, ROS hardware topics
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

## Proactive Checks When Touching Hardware Code

- Does commanded angle match what the pot is actually reading in sim/test?
- Is drift threshold appropriate for the joint's expected load?
- Is the Pico publishing at sufficient rate (min 50Hz for control)?
- Are stepper step counts staying in sync with commanded positions across a full move sequence?

## hardware/ Directory (in repo)

- `hardware/arm_calculator/` — FastAPI web app: models arm geometry, calculates torque capacity, exports to Fusion 360 CSV
- `hardware/stepper_test/` — Arduino CNC Shield + A4988 test sketches (AccelStepper, serial commands, potentiometer feedback)
- `hardware/drawings/` — SVG arm geometry reference (side view, top view)
- `hardware/params/` — Fusion 360 exported parameters (CSV)
- `hardware/robot_arm_plan.md` — original design plan
