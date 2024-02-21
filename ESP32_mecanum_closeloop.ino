// PID Closed-loop controller for mecanum robot in ABU Robocon 2024
// Coded by TinLethax

//#define DEBUG

TaskHandle_t CodeOnCore0;

#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 2
#define BACK_LEFT_MOTOR 4
#define BACK_RIGHT_MOTOR 6
#define STOP 0

#define MAX_VRPM 7040

#define limMaxInt 1384886.25
#define limMinInt -1403292.50

long prevtime = 0;
long prevtime2 = 0;
int speedPWM[4] = {0, 0, 0, 0}; //get value for naming pc
bool state = 0;


struct PIDControl {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousError;
  bool state;
  long prev_sp;
};

PIDControl PIDControllers[] = {
  {1.00, 0.005,0.0, 0, 0, 0}, // PID for Front Left Motor
  {1.00, 0.0045, 0.0, 0, 0, 0}, // PID for Front Right Motor
  {1.00, 0.005, 0.0, 0, 0, 0}, // PID for Back Left Motor
  {1.00, 0.0045, 0.0, 0, 0, 0}, // PID for Back Right Motor
};
struct Velocity {
  float Pulse;
  float PrevPulse;
  float Vrpm;
  int16_t Irpm;// RPM in Int
};

struct Velocity Count[] {
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 },
  { 0, 0, 0 }
};

float ChangetoRPM(long Pulse, uint8_t i) {
  float Vrpm = (((float)Pulse) - Count[i].PrevPulse) * 93.75;
  Count[i].PrevPulse = (float)Pulse;
  return Vrpm;
}

// Params
// setpoint -> RPM speed setpoint
// rpm -> feedback from encoder
// deltaTime -> delta time from millis
// &pid -> pointer to PID gain parameters

long PIDMOTOR(int setpoint, float rpm, float deltaTime, PIDControl &pid) {

  if (setpoint > 0 && pid.state == 1) {
    pid.integral = 0;
    pid.previousError = 0;
    pid.state = 0;
  } else if (setpoint < 0 && pid.state == 0) {
    pid.integral = 0;
    pid.previousError = 0;
    pid.state = 1;
  }

  float error = (setpoint * 1.0) - rpm;
  pid.integral += (error * deltaTime);

  // Anti-windup
  if (pid.integral > limMaxInt) {
    pid.integral = limMaxInt;
  } else if (pid.integral < limMinInt) {
    pid.integral = limMinInt;
  }

  float derivative = (error - pid.previousError) / deltaTime;
  float u = (pid.kp * error + pid.ki * pid.integral + pid.kd * derivative);
  pid.previousError = error;

  long pwm = (static_cast<long>(u) * 4095) / MAX_VRPM;

  if (pwm > 4095) {
    pwm = 4095;
  } else if (pwm < -4095) {
    pwm = -4095;
  }
  return pwm;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
#ifdef DEBUG
  Serial.println("Starting...");
#endif
  Encoder_Init();
  Motor_Init();
  pinMode(2, OUTPUT);

  // Run another task on Core 0
  //  xTaskCreatePinnedToCore(
  //    loop2, /* Function to implement the task */
  //    "loop2", /* Name of the task */
  //    10000,  /* Stack size in words */
  //    NULL,  /* Task input parameter */
  //    0,  /* Priority of the task */
  //    &CodeOnCore0,  /* Task handle. */
  //    0); /* Core where the task should run */
}
uint8_t main_fsm = 0;

// Serial packet from ROS host to ESP32
// 'R' 'B' [(0) 6 digit number - left front motor] ... [(3) 6 digit number - right back motor] '\n'
char serial_receive;
char arg_lf[7];
char arg_lb[7];
char arg_rf[7];
char arg_rb[7];
uint8_t arg_offset = 0;
unsigned long serial_timeout = 0;

unsigned long serial_send_millis = 0;

void loop() {
  // put your main code here, to run repeatedly:

    if ((millis() - serial_timeout) > 500) {
      serial_timeout = millis();
      arg_offset = 0;
      main_fsm = 0;
      speedPWM[0] = 0;
      speedPWM[1] = 0;
      speedPWM[2] = 0;
      speedPWM[3] = 0;
      digitalWrite(2, LOW);
    }

  switch (main_fsm) {
    case 0:// Wait for serial command
      if (Serial.available() > 0) {
        serial_receive = Serial.read();
        if (serial_receive != 'R') {
          break;
        }
        serial_receive = Serial.read();
        if (serial_receive != 'b') {
          break;
        }
#ifdef DEBUG
        Serial.println("Got Robot!");
#endif
        main_fsm = 1;
        serial_timeout = millis();// reset countdown timer
      }
      break;

    case 1:// Receive Left Front motor speed
      if (Serial.available() > 0) {
        serial_receive = Serial.read();
        if (serial_receive == ' ') {
          arg_offset = 0;
#ifdef DEBUG
          Serial.println("Got LF!");
#endif
          main_fsm = 2;
          serial_timeout = millis();// reset countdown timer
          break;
        }
        arg_lf[arg_offset++] = serial_receive;
      }
      break;

    case 2:// Receive Left Back motor speed
      if (Serial.available() > 0) {
        serial_receive = Serial.read();
        if (serial_receive == ' ') {
          arg_offset = 0;
#ifdef DEBUG
          Serial.println("Got LB!");
#endif
          main_fsm = 3;
          serial_timeout = millis();// reset countdown timer
          break;
        }
        arg_lb[arg_offset++] = serial_receive;
      }
      break;

    case 3:// Receive Right Front motor speed
      if (Serial.available() > 0) {
        serial_receive = Serial.read();
        if (serial_receive == ' ') {
          arg_offset = 0;
#ifdef DEBUG
          Serial.println("Got RF!");
#endif
          main_fsm = 4;
          serial_timeout = millis();// reset countdown timer
          break;
        }
        arg_rf[arg_offset++] = serial_receive;
      }
      break;

    case 4:// Receive Right Back motor speed
      if (Serial.available() > 0) {
        serial_receive = Serial.read();
        if (serial_receive == '\n') {
          arg_offset = 0;
#ifdef DEBUG
          Serial.println("Got RB!");
#endif
          main_fsm = 5;
          serial_timeout = millis();// reset countdown timer
          break;
        }
        arg_rb[arg_offset++] = serial_receive;
      }
      break;

    case 5:// Parse serial Data
      digitalWrite(2, HIGH);
      speedPWM[0] = atoi(arg_lf);
      speedPWM[2] = atoi(arg_lb);
      speedPWM[1] = atoi(arg_rf);
      speedPWM[3] = atoi(arg_rb);
      memset(arg_lf, 0, 7);
      memset(arg_lb, 0, 7);
      memset(arg_rf, 0, 7);
      memset(arg_rb, 0, 7);
#ifdef DEBUG
      Serial.print("Got Speed! ");
      Serial.print(speedPWM[0]);
      Serial.print(" ");
      Serial.print(speedPWM[2]);
      Serial.print(" ");
      Serial.print(speedPWM[1]);
      Serial.print(" ");
      Serial.println(speedPWM[3]);
#endif
      main_fsm = 0;
      break;
  }

  if ((millis() - serial_send_millis) > 20) {
    serial_send_millis = millis();
    Serial.write("Rb", 2);
    Serial.print(Count[0].Irpm);
    Serial.write(" ", 1);
    Serial.print(Count[2].Irpm);
    Serial.write(" ", 1);
    Serial.print(Count[1].Irpm);
    Serial.write(" ", 1);
    Serial.print(Count[3].Irpm);
    Serial.write("\n", 1);
  }

  if ((millis() - prevtime) > 10) {
    for (uint8_t i = 0; i < 4; i++) {
      Count[i].Pulse = GetEncoder(i + 1);
      Count[i].Vrpm = ChangetoRPM(Count[i].Pulse, i);
      Count[i].Irpm = round(Count[i].Vrpm);
    }
    int speedFL = PIDMOTOR(speedPWM[0], Count[0].Vrpm, 10, PIDControllers[0]);
    int speedFR = PIDMOTOR(speedPWM[1], Count[1].Vrpm, 10, PIDControllers[1]);
    int speedBL = PIDMOTOR(speedPWM[2], Count[2].Vrpm, 10, PIDControllers[2]);
    int speedBR = PIDMOTOR(speedPWM[3], Count[3].Vrpm, 10, PIDControllers[3]);

    rotateMotor(FRONT_LEFT_MOTOR, speedFL);
    rotateMotor(FRONT_RIGHT_MOTOR, speedFR);
    rotateMotor(BACK_LEFT_MOTOR, speedBL);
    rotateMotor(BACK_RIGHT_MOTOR, speedBR);
    prevtime = millis();
  }

}

void loop2(void *parameter) {

  while (1) {

  }
}
