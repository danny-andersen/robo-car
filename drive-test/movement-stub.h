//Test stub for Functions to control drive motors to perform movement of a wheeled device.


bool rotateTo(int16_t directionRequired) {
    if (Serial) {
    Serial.print("Rotating to: ");
    Serial.println(directionRequired);
    // Serial.print(" Rotate Angle: ");
    // Serial.print(rotate);
    // Serial.print(", straightahead is ");
    // Serial.println(currentDirn);
  }

  return true;
}

void drive(Motor_Direction direction, float straightAheadRad, uint8_t speed) {
    // Serial.print("Direction to Drive: ");
    // Serial.print(straightAheadRad);
    // Serial.print(" Motor: ");
    // Serial.print(direction);
    // Serial.print(" Speed: ");
    // Serial.println(speed);
}

void backOut() {
  Serial.println("Backing out");
}

void aboutTurn() {
  Serial.println("About turn");
}

 
