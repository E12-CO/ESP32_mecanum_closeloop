#define PWM_freq  12000
#define PWM_res   12

struct MOTOR_PINS {
  uint8_t IN;
  uint8_t PWMSPEEDCHANNEL;
};

MOTOR_PINS motorPins[] = {
  { 27, 0 },  // FRONT_LEFT_MOTOR IN1
  { 13, 1 },  // FRONT_LEFT_MOTOR IN2
  { 25, 2 },  // FRONT_RIGHT_MOTOR IN1
  { 26, 3 },  // FRONT_RIGHT_MOTOR IN2
  { 23, 4 },  // BACK_LEFT_MOTOR IN1
  { 22, 5 },  // BACK_LEFT_MOTOR IN2
  { 32, 6 },  // BACK_RIGHT_MOTOR IN1
  { 33, 7 },  // BACK_RIGHT_MOTOR IN2
};

void Motor_Init() {
  for (uint8_t i = 0; i < 8; ++i) {
    pinMode(motorPins[i].IN, OUTPUT);
    ledcSetup(motorPins[i].PWMSPEEDCHANNEL, PWM_freq, PWM_res);
    ledcAttachPin(motorPins[i].IN, motorPins[i].PWMSPEEDCHANNEL);
  }
  for (uint8_t i = 0; i < 8; i += 2) {
    rotateMotor(i, STOP);
  }
}

void rotateMotor(uint8_t motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, 0);
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, abs(motorSpeed));
  } else if (motorSpeed > 0) {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, abs(motorSpeed));
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, 0);
  } else {
    ledcWrite(motorPins[motorNumber].PWMSPEEDCHANNEL, 0);
    ledcWrite(motorPins[motorNumber + 1].PWMSPEEDCHANNEL, 0);
  }
}
