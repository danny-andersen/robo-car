#ifndef MOTOR_DRIVER_H
#define MOTO_DRIVER_H

#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

enum Motor_Direction {
  FORWARD,         //(1)
  BACK,            //(2)
  ROTATE_LEFT,     //(3)
  ROTATE_RIGHT,    //(4)
  LEFT_FORWARD,    //(5)
  LEFT_BACKWARD,   //(6)
  RIGHT_FORWARD,   //(7)
  RIGHT_BACKWARD,  //(8)
  STOP,
  INITIAL
};

void motor_Init() {
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

void driveMotor(Motor_Direction direction, uint8_t rightSpeed, uint8_t leftSpeed) {
  switch (direction) {
    case FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case BACK:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case ROTATE_LEFT:
      digitalWrite(PIN_Motor_AIN_1, HIGH);  // forward
      digitalWrite(PIN_Motor_BIN_1, LOW);   // reverse
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case ROTATE_RIGHT:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case LEFT_FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed / 2);
      digitalWrite(PIN_Motor_STBY, HIGH);
    case RIGHT_FORWARD:
      digitalWrite(PIN_Motor_AIN_1, HIGH);
      digitalWrite(PIN_Motor_BIN_1, HIGH);
      analogWrite(PIN_Motor_PWMA, rightSpeed / 2);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case LEFT_BACKWARD:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed);
      analogWrite(PIN_Motor_PWMB, leftSpeed / 2);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case RIGHT_BACKWARD:
      digitalWrite(PIN_Motor_AIN_1, LOW);
      digitalWrite(PIN_Motor_BIN_1, LOW);
      analogWrite(PIN_Motor_PWMA, rightSpeed / 2);
      analogWrite(PIN_Motor_PWMB, leftSpeed);
      digitalWrite(PIN_Motor_STBY, HIGH);
      break;
    case STOP:
      analogWrite(PIN_Motor_PWMA, 0);
      analogWrite(PIN_Motor_PWMB, 0);
      digitalWrite(PIN_Motor_STBY, LOW);
      break;
    default:
      analogWrite(PIN_Motor_PWMA, 0);
      analogWrite(PIN_Motor_PWMB, 0);
      digitalWrite(PIN_Motor_STBY, LOW);
      break;
  }
}

#endif