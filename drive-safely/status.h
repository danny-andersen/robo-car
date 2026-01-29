
#include <FastLED.h>
#include <SparkFun_SHTC3.h>

#define VOL_MEASURE_PIN A3
#define STATUS_TIME 1000  //Update status every second

#define PIN_RBGLED 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

SHTC3 mySHTC3;


void statusInit() {
  pinMode(VOL_MEASURE_PIN, INPUT);
  FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
  FastLED.setBrightness(10);
  mySHTC3.begin();
}

float getBatteryVoltage() {
  float voltage = (analogRead(VOL_MEASURE_PIN) * 5) * ((10 + 1.5) / 1.5) / 1024;  //Read voltage value
  voltage = voltage + (voltage * 0.08);
  return voltage;
}

void setStatusLed(float voltage) {
  if (voltage > 8.2) {
    leds[0] = 0x00cc00;
  } else if (voltage > 7.8) {
    leds[0] = 0x66cc00;
  } else if (voltage > 7.4) {
    leds[0] = 0xcccc00;
  } else if (voltage > 7.0) {
    leds[0] = 0xcc9900;
  } else if (voltage > 6.3) {
    leds[0] = 0xcc3300;
  } else {
    leds[0] = 0xcc0000;
  }
  FastLED.show();
}

void showBatteryStatus() {
  setStatusLed(getBatteryVoltage());
}

bool getTempHumidity(float *tempC, float *humidity) {
  SHTC3_Status_TypeDef result = mySHTC3.update();
  if (mySHTC3.lastStatus == SHTC3_Status_Nominal) {
    *tempC = mySHTC3.toDegC();
    *humidity = mySHTC3.toPercent();
    return true;
  } else {
    return false;
  }
}

bool getTempHumidityInt(int16_t *tempC, int16_t *humidity) {
  float temp, humid;
  if (getTempHumidity(&temp, &humid)) {
    *tempC = int((temp * 10.0) + 0.5);
    *humidity = int((humid * 10.0) + 0.5);
    return true;
  }
  return false;
}
