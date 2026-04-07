#pragma once

// ============================================================
// commands.h — Serial command parser declarations
// Reads lines from Serial and dispatches to motion / status.
// ============================================================

// Call once in setup()
void commandsInit();

// Call every loop() — reads and parses any available serial input.
// Returns true if a command was received (used to exit demo mode).
bool commandsTick();

// Print the STATUS block
void printStatus(const char* mode);

// Print the CALIBRATION helper block
void printCalibration();

// Print the HELP list
void printHelp();
