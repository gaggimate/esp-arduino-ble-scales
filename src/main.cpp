// Minimal entry point for compile-check build environment.
// This project is a library; this file exists only to satisfy the Arduino
// framework's requirement for setup() and loop() when building with pio run.
#include <Arduino.h>

void setup() { /* No-op: library has no standalone initialization */ }
void loop() { /* No-op: library has no standalone run loop */ }
