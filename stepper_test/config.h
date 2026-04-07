#pragma once

// ============================================================
// config.h — All tunable parameters for stepper_test
// Edit these values to match your hardware, then re-upload.
// ============================================================

// === MOTOR ===
const int   MOTOR_STEPS_PER_REV = 200;   // full steps/rev (200 = 1.8 deg/step, 400 = 0.9 deg/step)
const int   MICROSTEP            = 16;   // must match jumper config: 1, 2, 4, 8, or 16
const float GEAR_RATIO           = 1.0f; // ring gear / pinion — set to 1.0 until calibrated
const int   MAX_SPEED_RPM        = 60;   // hard ceiling on motor RPM
const int   DEFAULT_SPEED_RPM    = 20;   // speed used on startup and demo
const int   ACCEL_STEPS          = 200;  // steps over which speed linearly ramps up/down
const bool  INVERT_DIR           = false;// set true if CW/CCW are backwards

// === CALIBRATION ===
const float DEG_PER_OUTPUT_REV   = 360.0f; // leave at 360 unless joint has limited rotation

// === PINS (CNC Shield v3 — do not change) ===
const int PIN_STEP   = 2;
const int PIN_DIR    = 5;
const int PIN_ENABLE = 8;  // LOW = enabled, HIGH = disabled
const int PIN_POT    = A0; // potentiometer input

// === SERIAL ===
const long SERIAL_BAUD = 115200;

// === POT ===
// If the pot reads within this band of 512 (mid-range) on startup, pot mode is disabled.
const int POT_DEADBAND_STARTUP = 50;
// Minimum change in output degrees before reprinting pot position
const float POT_PRINT_THRESHOLD_DEG = 1.0f;
// Output angle range mapped from pot (0-1023 → 0 to POT_MAX_DEG)
const float POT_MAX_DEG = 360.0f;
