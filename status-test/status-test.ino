#include "status.h"

float temp = 0.0;
float humid = 0.0;
int16_t tempC = 0;
int16_t humidity = 0;
float battVolt = 0.0;
  uint16_t volts = 0;

void setup() {
  Serial.begin(9600);
  statusInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  battVolt = getBatteryVoltage();
  Serial.print("Current voltage value : ");
  Serial.println(battVolt);
  setStatusLed(battVolt);
  volts = getBatteryVoltageInt();
  Serial.print("Current voltage value : ");
  Serial.println(volts/100.0);
  if (getTempHumidity(&temp, &humid)) {
    Serial.print("RH = ");
    Serial.print(humid);
    Serial.print("%, T = ");
    Serial.print(temp);
    Serial.println(" C");
  }
  if (getTempHumidityInt(&tempC, &humidity)) {
    Serial.print("RH = ");
    Serial.print(humidity / 10.0);
    Serial.print("%, T = ");p
    Serial.print(tempC / 10.0);
    Serial.println(" C");
  }

  // showLedRange();
  delay(1000);
}

void showLedRange() {
  leds[0] = 0x00cc00;
  FastLED.show();
  delay(500);
  leds[0] = 0x66cc00;
  FastLED.show();
  delay(500);
  leds[0] = 0xcccc00;
  FastLED.show();
  delay(500);
  leds[0] = 0xcc9900;
  FastLED.show();
  delay(500);
  leds[0] = 0xcc3300;
  FastLED.show();
  delay(500);
  leds[0] = 0xcc0000;
  FastLED.show();
  delay(500);
}