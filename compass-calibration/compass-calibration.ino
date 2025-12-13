/**
 *  QMC5883P – Calibration Sketch
 *  -----------------------------
 *  1. Upload sketch & open Serial Monitor @115200
 *  2. Slowly rotate sensor in large, horizontal figure-eight movements for ~30s
 *  3. Read the output offset and scaling values
 *  4. Copy the finished code lines into your own project
 */

#include <Wire.h>
#include "qmc5883p.h"

/* ---------- Global Variables ---------- */
QMC5883P mag;
float minX =  1e6, maxX = -1e6;
float minY =  1e6, maxY = -1e6;
unsigned long t0;

void setup() {
  Serial.begin(9600);
  
  // Start I²C bus (SDA, SCL - adapt for your MCU)
  Wire.begin();

  if (!mag.begin()) {
    Serial.println("Init FAILED – check wiring!");
    while (1) delay(1000);
  }
  Serial.println("\n>>> START CALIBRATION <<<");
  Serial.println("Rotate module in large figure-eight movements for 30s.");
  t0 = millis();
}

void loop() {
  float v[3];
  if (mag.readXYZ(v)) {               // Values are already in µT
    minX = min(minX, v[0]);  maxX = max(maxX, v[0]);
    minY = min(minY, v[1]);  maxY = max(maxY, v[1]);
  }

  /* Stop after 30 seconds */
  if (millis() - t0 > 30000) {
    float offX   = (maxX + minX) / 2.0f;
    float offY   = (maxY + minY) / 2.0f;
    float scaleX = (maxX - minX) / 2.0f;
    float scaleY = (maxY - minY) / 2.0f;
    float avg    = (scaleX + scaleY) / 2.0f;

    Serial.println("\n=== CALIBRATION RESULTS ===");
    Serial.print("Offset X = ");
    Serial.println(offX);
    Serial.print("Offset Y = ");
    Serial.println(offY);
    Serial.print("Scale  X = ");
    Serial.println(scaleX);
    Serial.print("Scale  Y = ");
    Serial.println(scaleY);
    Serial.print("Average  = ");
    Serial.println(avg);

    Serial.println("\nCopy these lines into your main sketch:");
    Serial.println("mag.setHardIronOffsets(" + String(offX, 3) +
                   "f, " + String(offY, 3) + "f);");
    Serial.println("// Soft-Iron:");
    Serial.println("const float SCALE_AVG = " + String(avg, 3) + "f;");
    Serial.println("const float SCALE_X   = " + String(scaleX, 3) + "f;");
    Serial.println("const float SCALE_Y   = " + String(scaleY, 3) + "f;");

    while (1);                        // Done
  }
}