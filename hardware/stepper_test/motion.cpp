#include "motion.h"
#include "config.h"
#include <AccelStepper.h>

// ============================================================
// motion.cpp — AccelStepper wrapper implementation
// ============================================================

// Driver interface: DRIVER mode uses external STEP/DIR pins
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

float stepsPerDegree  = 0.0f;
float currentOutputDeg = 0.0f;
bool  driverEnabled   = false;
int   currentSpeedRPM = DEFAULT_SPEED_RPM;

// --- motionInit ---
// Sets up pins, calculates steps/degree, configures AccelStepper.
void motionInit() {
    pinMode(PIN_ENABLE, OUTPUT);

    // steps/motor rev × microstepping × gear ratio / 360 degrees
    stepsPerDegree = ((float)MOTOR_STEPS_PER_REV * (float)MICROSTEP * GEAR_RATIO) / DEG_PER_OUTPUT_REV;

    setSpeedRPM(DEFAULT_SPEED_RPM);

    // Acceleration: ramp over ACCEL_STEPS steps
    // accel (steps/s²) = speed² / (2 × ACCEL_STEPS)  — kinematic ramp
    float spd = rpmToStepsPerSec(DEFAULT_SPEED_RPM);
    float acc  = (spd * spd) / (2.0f * (float)ACCEL_STEPS);
    stepper.setAcceleration(acc);

    if (INVERT_DIR) stepper.setPinsInverted(true, false, false);

    setDriverEnabled(true);
}

// --- moveDegrees ---
// Queues a relative move of deltaDeg output-shaft degrees.
void moveDegrees(float deltaDeg) {
    long steps = (long)(deltaDeg * stepsPerDegree);
    stepper.move(steps);
    currentOutputDeg += deltaDeg;
}

// --- moveRawSteps ---
// Queues a raw step move (bypasses degree/gear conversion).
void moveRawSteps(long steps) {
    stepper.move(steps);
    // Update position estimate using inverse conversion
    currentOutputDeg += (float)steps / stepsPerDegree;
}

// --- motionRun ---
// Must be called every loop() iteration — advances AccelStepper state.
void motionRun() {
    stepper.run();
}

// --- waitForMove ---
// Blocks until AccelStepper finishes its current move.
void waitForMove() {
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}

// --- setDriverEnabled ---
// Controls the ENABLE pin on the A4988 (LOW = on, HIGH = free-wheel).
void setDriverEnabled(bool enable) {
    driverEnabled = enable;
    digitalWrite(PIN_ENABLE, enable ? LOW : HIGH);
    if (enable) {
        currentOutputDeg = 0.0f;
        stepper.setCurrentPosition(0);
    }
}

// --- setSpeedRPM ---
// Sets motor speed, clamping to [1, MAX_SPEED_RPM].
void setSpeedRPM(int rpm) {
    if (rpm < 1)             rpm = 1;
    if (rpm > MAX_SPEED_RPM) rpm = MAX_SPEED_RPM;
    currentSpeedRPM = rpm;

    float spd = rpmToStepsPerSec(rpm);
    stepper.setMaxSpeed(spd);

    // Recompute acceleration to maintain same ACCEL_STEPS ramp distance
    float acc = (spd * spd) / (2.0f * (float)ACCEL_STEPS);
    stepper.setAcceleration(acc);
}

// --- rpmToStepsPerSec ---
// Converts motor RPM to steps/second for AccelStepper.
float rpmToStepsPerSec(int rpm) {
    return ((float)rpm * (float)MOTOR_STEPS_PER_REV * (float)MICROSTEP) / 60.0f;
}

// --- motionStop ---
// Stops motion immediately and clears queued move.
void motionStop() {
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
}

// --- isMoving ---
// Returns true while a move is in progress.
bool isMoving() {
    return stepper.distanceToGo() != 0;
}
