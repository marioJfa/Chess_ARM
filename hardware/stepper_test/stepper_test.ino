// ============================================================
// stepper_test.ino — Main sketch
//
// Controls a stepper motor via CNC Shield v3 + A4988 (X-axis slot).
// Three simultaneous control modes:
//   1. Serial Monitor commands  (always active)
//   2. Potentiometer on A0      (positional control, activates when moved)
//   3. Demo sequence            (runs when serial idle and pot inactive)
//
// Requires:  AccelStepper library (install via Arduino Library Manager)
// Compiles on: Arduino Uno and Mega without modification
// ============================================================

#include "config.h"
#include "motion.h"
#include "commands.h"
#include <Arduino.h>

// ============================================================
// Mode tracking
// ============================================================
enum ControlMode { MODE_DEMO, MODE_SERIAL, MODE_POT };
static ControlMode currentMode = MODE_DEMO;

// ============================================================
// Demo sequence state machine
// ============================================================
static int   demoStep         = 0;
static unsigned long demoWait = 0; // millis() timestamp for pause end

// Run one tick of the demo sequence.
// Returns immediately if a move is still in progress or we're in a pause.
static void demotick() {
    if (isMoving()) return;                     // wait for current move to finish
    if (millis() < demoWait) return;            // wait for pause to finish

    switch (demoStep) {
        case 0:  moveDegrees(90.0f);  demoStep++; break;
        case 1:  demoWait = millis() + 1000; demoStep++; break;
        case 2:  moveDegrees(90.0f);  demoStep++; break;
        case 3:  demoWait = millis() + 1000; demoStep++; break;
        case 4:  moveDegrees(-180.0f); demoStep++; break;
        case 5:  demoWait = millis() + 2000; demoStep++; break;
        case 6:  moveDegrees(360.0f);  demoStep++; break;
        case 7:  demoWait = millis() + 1000; demoStep++; break;
        case 8:  moveDegrees(-360.0f); demoStep++; break;
        case 9:  demoWait = millis() + 3000; demoStep++; break;
        default: demoStep = 0; break;           // loop back to start
    }
}

// ============================================================
// Potentiometer state
// ============================================================
static bool  potModeActive    = false;
static float potLastDeg       = -9999.0f;  // last printed pot position
static float potTargetDeg     = 0.0f;      // last commanded pot position

// Reads A0, maps 0-1023 to 0-POT_MAX_DEG.
static float readPotDeg() {
    int raw = analogRead(PIN_POT);
    return ((float)raw / 1023.0f) * POT_MAX_DEG;
}

// Checks pot on startup — if mid-range, pot mode starts inactive.
static void potInit() {
    int raw = analogRead(PIN_POT);
    if (abs(raw - 512) < POT_DEADBAND_STARTUP) {
        potModeActive = false;
        Serial.println(F("Pot: mid-range on startup — pot mode inactive. Serial / demo active."));
    } else {
        potModeActive = true;
        potTargetDeg  = readPotDeg();
        currentOutputDeg = potTargetDeg; // sync position to pot
        Serial.println(F("Pot: non-mid-range on startup — pot mode ACTIVE."));
    }
}

// Called every loop() when in pot mode, or to detect pot activation.
// Returns true if pot mode is active and commanding motion.
static bool potTick() {
    float deg = readPotDeg();

    if (!potModeActive) {
        // Activate pot mode if the pot has moved more than 5 degrees from mid-range
        if (abs(deg - (POT_MAX_DEG / 2.0f)) > 5.0f) {
            potModeActive = true;
            potTargetDeg  = deg;
            Serial.println(F("Pot mode ACTIVATED."));
        }
        return false;
    }

    // Deactivate if pot returns to within 2 degrees of centre
    if (abs(deg - (POT_MAX_DEG / 2.0f)) < 2.0f && currentMode == MODE_POT) {
        potModeActive = false;
        Serial.println(F("Pot mode DEACTIVATED — serial / demo active."));
        currentMode = MODE_DEMO;
        return false;
    }

    // Move to pot position if it has changed by more than 1 degree
    float delta = deg - potTargetDeg;
    if (abs(delta) > 1.0f && !isMoving()) {
        potTargetDeg = deg;
        moveDegrees(delta);
    }

    // Print only when position changes by more than threshold
    if (abs(deg - potLastDeg) >= POT_PRINT_THRESHOLD_DEG) {
        potLastDeg = deg;
        Serial.print(F("Pot → "));
        Serial.print(deg, 1);
        Serial.println(F(" deg"));
    }

    return true;
}

// ============================================================
// setup
// ============================================================
void setup() {
    motionInit();
    commandsInit();
    potInit();

    if (potModeActive) {
        currentMode = MODE_POT;
    } else {
        currentMode = MODE_DEMO;
        Serial.println(F("Starting demo sequence. Send any command or move pot to override."));
    }
}

// ============================================================
// loop
// ============================================================
void loop() {
    // 1. Always advance the stepper state machine first
    motionRun();

    // 2. Check for serial commands — takes highest priority
    bool gotCommand = commandsTick();
    if (gotCommand) {
        if (currentMode == MODE_DEMO) {
            // Stop demo, hand control to serial
            motionStop();
            demoStep = 0;
        }
        if (currentMode != MODE_POT) {
            currentMode = MODE_SERIAL;
        }
    }

    // 3. Check / handle potentiometer
    if (currentMode != MODE_SERIAL) {
        bool potActive = potTick();
        if (potActive && currentMode != MODE_POT) {
            motionStop();
            demoStep = 0;
            currentMode = MODE_POT;
        }
    }

    // 4. Run demo if no other mode is active
    if (currentMode == MODE_DEMO) {
        demotick();
    }

    // 5. Update STATUS mode label for serial output (only changes on transition)
    // (status is printed on demand via STATUS command; mode label is passed in)
}
