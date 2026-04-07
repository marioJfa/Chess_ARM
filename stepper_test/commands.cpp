#include "commands.h"
#include "motion.h"
#include "config.h"
#include <Arduino.h>

// ============================================================
// commands.cpp — Serial command parser implementation
// ============================================================

static String serialBuffer = "";        // accumulates incoming characters
static bool   commandReceived = false;  // set true when a full line arrives

// --- commandsInit ---
// Opens the serial port at the configured baud rate.
void commandsInit() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {}  // wait for USB serial on Leonardo/Micro (no-op on Uno/Mega)
    Serial.println(F("=== Stepper Test Ready ==="));
    Serial.println(F("Type HELP for command list."));
}

// --- commandsTick ---
// Non-blocking serial reader. Parses a complete line when '\n' arrives.
// Returns true if any command was processed this tick.
bool commandsTick() {
    commandReceived = false;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBuffer.length() > 0) {
                parseCommand(serialBuffer);
                serialBuffer = "";
                commandReceived = true;
            }
        } else {
            serialBuffer += c;
        }
    }
    return commandReceived;
}

// --- parseCommand ---
// Converts the incoming line to uppercase, splits verb and argument,
// and dispatches to the appropriate handler.
static void parseCommand(String line) {
    line.trim();
    line.toUpperCase();

    // Split into verb (first token) and optional argument
    int spaceIdx = line.indexOf(' ');
    String verb = (spaceIdx == -1) ? line : line.substring(0, spaceIdx);
    String arg  = (spaceIdx == -1) ? "" : line.substring(spaceIdx + 1);
    arg.trim();

    if (verb == "MOVE") {
        if (arg.length() == 0) {
            Serial.println(F("Usage: MOVE <degrees>  e.g. MOVE 90  MOVE -45"));
            return;
        }
        float deg = arg.toFloat();
        Serial.print(F("Moving "));
        Serial.print(deg, 2);
        Serial.println(F(" degrees"));
        moveDegrees(deg);

    } else if (verb == "SPEED") {
        if (arg.length() == 0) {
            Serial.println(F("Usage: SPEED <rpm>"));
            return;
        }
        int rpm = arg.toInt();
        setSpeedRPM(rpm);
        Serial.print(F("Speed set to "));
        Serial.print(currentSpeedRPM);
        Serial.println(F(" RPM"));

    } else if (verb == "HOME") {
        float delta = -currentOutputDeg;
        Serial.print(F("Homing: moving "));
        Serial.print(delta, 2);
        Serial.println(F(" degrees to reach 0"));
        moveDegrees(delta);

    } else if (verb == "ENABLE") {
        setDriverEnabled(true);
        Serial.println(F("Driver ENABLED. Position reset to 0."));

    } else if (verb == "DISABLE") {
        setDriverEnabled(false);
        Serial.println(F("Driver DISABLED. Motor is free-wheeling."));

    } else if (verb == "STATUS") {
        printStatus("SERIAL");

    } else if (verb == "STEPS") {
        if (arg.length() == 0) {
            Serial.println(F("Usage: STEPS <n>  (raw motor steps, signed)"));
            return;
        }
        long steps = arg.toInt();
        Serial.print(F("Raw move: "));
        Serial.print(steps);
        Serial.println(F(" steps"));
        moveRawSteps(steps);

    } else if (verb == "CALIBRATE") {
        printCalibration();

    } else if (verb == "STOP") {
        motionStop();
        Serial.println(F("STOP — motion halted."));

    } else if (verb == "HELP") {
        printHelp();

    } else {
        Serial.print(F("Unknown command: "));
        Serial.println(verb);
        Serial.println(F("Type HELP for command list."));
    }
}

// --- printStatus ---
// Prints a formatted status block. mode = "DEMO", "SERIAL", or "POT".
void printStatus(const char* mode) {
    Serial.println(F("=== ARM STATUS ==="));
    Serial.print(F("Output position : ")); Serial.print(currentOutputDeg, 2); Serial.println(F(" deg"));
    Serial.print(F("Motor steps     : ")); Serial.println(stepper.currentPosition());
    Serial.print(F("Speed           : ")); Serial.print(currentSpeedRPM); Serial.println(F(" RPM"));
    Serial.print(F("Microstep       : 1/")); Serial.println(MICROSTEP);
    Serial.print(F("Gear ratio      : ")); Serial.println(GEAR_RATIO, 2);
    Serial.print(F("Steps/output deg: ")); Serial.println(stepsPerDegree, 2);
    Serial.print(F("Mode            : ")); Serial.println(mode);
    Serial.print(F("Driver          : ")); Serial.println(driverEnabled ? F("ENABLED") : F("DISABLED"));
    Serial.println(F("=================="));
}

// --- printCalibration ---
// Prints instructions and current step-rate maths for calibration.
void printCalibration() {
    float stepsPerRev = (float)MOTOR_STEPS_PER_REV * (float)MICROSTEP;
    float outputStepsPerRev = stepsPerRev * GEAR_RATIO;
    Serial.println(F("=== CALIBRATION HELPER ==="));
    Serial.println(F("Current settings:"));
    Serial.print(F("  Motor steps/rev : ")); Serial.print(MOTOR_STEPS_PER_REV); Serial.println(F(" (full steps)"));
    Serial.print(F("  Microstep       : 1/")); Serial.println(MICROSTEP);
    Serial.print(F("  Gear ratio      : ")); Serial.println(GEAR_RATIO, 2);
    Serial.print(F("  Steps/motor rev : ")); Serial.println((long)stepsPerRev);
    Serial.print(F("  Steps/output rev: ")); Serial.println((long)outputStepsPerRev);
    Serial.print(F("  Steps/degree    : ")); Serial.println(stepsPerDegree, 2);
    Serial.println();
    Serial.println(F("To find your gear ratio:"));
    Serial.println(F("  1. Mark the output shaft"));
    Serial.print(F("  2. Send: STEPS ")); Serial.println((long)stepsPerRev);
    Serial.println(F("     (one full motor revolution)"));
    Serial.println(F("  3. Measure how many degrees the output moved"));
    Serial.println(F("  4. Gear ratio = 360 / degrees_moved"));
    Serial.println(F("  5. Update GEAR_RATIO in config.h and re-upload"));
    Serial.println(F("=========================="));
}

// --- printHelp ---
// Lists all available serial commands.
void printHelp() {
    Serial.println(F("=== COMMANDS ==="));
    Serial.println(F("MOVE <deg>   — rotate output shaft N degrees (+CW / -CCW)"));
    Serial.println(F("SPEED <rpm>  — set motor speed in RPM"));
    Serial.println(F("HOME         — return to 0 degrees (open loop)"));
    Serial.println(F("ENABLE       — enable driver (resets position to 0)"));
    Serial.println(F("DISABLE      — disable driver (motor free-wheels)"));
    Serial.println(F("STATUS       — print position, speed, config"));
    Serial.println(F("STEPS <n>    — raw step move (bypasses degree conversion)"));
    Serial.println(F("CALIBRATE    — print gear ratio calibration guide"));
    Serial.println(F("STOP         — immediate stop"));
    Serial.println(F("HELP         — this list"));
    Serial.println(F("================"));
}
