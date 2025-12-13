
#include "rgb-led.h";
#include "motor-driver.h";

void setup()
{
  motor_Init();
  status_rgb_init();
  set_status_rgb(RED);
  delay(2000);
  drive_motor(FORWARD, 200 /*speed*/);
  set_status_rgb(GREEN);
  delay(2000);
  drive_motor(BACK, 200 /*speed*/);
  set_status_rgb(BLUE);
  delay(2000);
  drive_motor(STOP, 0 /*speed*/);
  set_status_rgb(RED);
  delay(2000);
}

void loop()
{
}
