#define MKII
#include "uArm.h"

void setup() {
  Serial.begin(115200);  // start serial port at 115200 bps
  uArm.setup();

  uArm.moveTo(0, 150, 150);
  Serial.println("[READY]");
}

void loop() {
  uArm.run();
}
