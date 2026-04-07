#pragma once

#include <AccelStepper.h>
#include "config.h"

// ============================================================
// motion.h — AccelStepper wrapper declarations
// Handles step-rate calculation, degree-to-step conversion,
// and non-blocking move execution.
// ============================================================

// Exposed stepper instance (needed by main loop to call run())
extern AccelStepper stepper;

// Computed at init — steps required per degree of OUTPUT shaft rotation
extern float stepsPerDegree;

// Current logical position of the OUTPUT shaft in degrees (open loop)
extern float currentOutputDeg;

// Whether the driver is currently enabled
extern bool driverEnabled;

// Current speed setting in RPM (motor shaft)
extern int currentSpeedRPM;

// --- Functions ---

// Initialise pins, compute step rates, enable driver
void motionInit();

// Move output shaft by deltaDeg degrees (signed). Non-blocking — call motionRun() each loop.
void moveDegrees(float deltaDeg);

// Move raw motor steps (bypasses gear ratio — for calibration)
void moveRawSteps(long steps);

// Called every loop() — advances AccelStepper state machine
void motionRun();

// Block until the current move is complete (only use outside of time-critical loops)
void waitForMove();

// Enable or disable the A4988 driver
void setDriverEnabled(bool enable);

// Update speed; clamps to 1–MAX_SPEED_RPM
void setSpeedRPM(int rpm);

// Convert RPM (motor shaft) to AccelStepper speed in steps/sec
float rpmToStepsPerSec(int rpm);

// Stop motion immediately
void motionStop();

// Returns true if a move is in progress
bool isMoving();
