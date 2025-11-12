/*
  2WD Self-Balancing Robot (ESP32 + MPU6050 + L298N)
  Bluetooth control: send single characters: 
    'F' = forward, 'B' = backward, 'L' = left, 'R' = right, 'S' = stop
  Balance control uses complementary filter + PID.
  
  Wiring & notes below the code.
*/

#include <Wire.h>
#include "BluetoothSerial.h"

// ---------------- MPU6050 (I2C) registers (minimal) ----------------
const int MPU_ADDR = 0x68;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// ---------------- Bluetooth ----------------
BluetoothSerial SerialBT;

// ---------------- Motor driver (L298N) pins ----------------
// Change these pins to match your wiring
const int IN1 = 26; // Motor A direction
const int IN2 = 27;
const int IN3 = 14; // Motor B direction
const int IN4 = 12;
const int ENA = 25; // PWM channel for Motor A
const int ENB = 33; // PWM channel for Motor B

// PWM channels on ESP32
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;
const int PWM_FREQ = 20000; // 20kHz
const int PWM_RES = 8;      // 8-bit resolution -> 0-255

// ---------------- Control variables ----------------
float angle = 0.0;          // estimated tilt angle (degrees)
float gyroRate = 0.0;       // deg/s from gyro
unsigned long lastTime = 0;
float dt = 0.0;

// Complementary filter coefficient (alpha ~ 0.98)
const float alpha = 0.98;

// PID controller for balancing
// Start with small P, I and D = 0; tune gradually
float Kp = 20.0;    // proportional gain
float Ki = 0.8;     // integral gain
float Kd = 0.6;     // derivative gain

float pid_integral = 0.0;
float pid_last_error = 0.0;

// Target angle: 0 means upright. For motion, we offset this
float targetAngle = 0.0;      // degrees
float motionAngleOffset = 0.0; // dynamic offset from user commands

// Max motor pwm
const int MAX_PWM = 230;

// Bluetooth command state
char btCmd = 'S'; // default stop

// MPU calibration offsets (fill after calibration)
float gyroXoffset = 0.0;
float accelAngleOffset = 0.0; // if you want to shift zero

// Safety / tuning
const float MAX_ANGLE_TOLERANCE = 40.0; // if robot tilts beyond this, stop motors

// ---------------- Helper functions ----------------
void setMotorA(int pwm, bool forward) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  ledcWrite(PWM_CH_A, pwm);
}
void setMotorB(int pwm, bool forward) {
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  if (forward) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  ledcWrite(PWM_CH_B, pwm);
}
void stopMotors() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // init MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // wake up
  Wire.endTransmission(true);

  // Setup motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Setup PWM channels
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, PWM_CH_B);

  stopMotors();

  // Start Bluetooth
  SerialBT.begin("ESP32_BalanceBot"); // device name
  Serial.println("Bluetooth started: ESP32_BalanceBot");

  // small delay and calibrate gyro
  delay(100);
  calibrateSensors();

  lastTime = micros();
}

// ---------------- Main loop ----------------
void loop() {
  // timing
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0; // seconds
  if (dt <= 0) dt = 0.001;
  lastTime = now;

  // read sensor
  readMPU6050();

  // compute angle from accelerometer
  float accelAngle = atan2((float)accelY, (float)accelZ) * 180.0 / PI + accelAngleOffset;

  // gyro rate (x-axis, deg/s). divide by 131 for raw to deg/s if using typical MPU6050 default FS= +/-250deg/s
  gyroRate = ((float)gyroX) / 131.0 - gyroXoffset;

  // complementary filter
  angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accelAngle;

  // check bluetooth commands
  handleBluetooth();

  // compute error between target and current
  float error = (targetAngle + motionAngleOffset) - angle;

  // PID
  pid_integral += error * dt;
  float derivative = (error - pid_last_error) / dt;
  float pid_output = Kp * error + Ki * pid_integral + Kd * derivative;
  pid_last_error = error;

  // safety: if angle too large, stop motors
  if (abs(angle) > MAX_ANGLE_TOLERANCE) {
    stopMotors();
    Serial.println("Too tilted - stopping motors!");
    delay(200);
    return;
  }

  // base motor command from PID
  // pid_output positive -> need to move motors forward to bring robot upright (depends on your build orientation)
  int basePWM = (int)constrain(abs(pid_output), 0, MAX_PWM);

  // Choose direction based on pid_output sign and your motor wiring direction
  bool forwardDir = (pid_output > 0); // adjust if your motors are inverted

  // apply motion adjustments for user commands:
  // For forward/back we slightly change target angle (motionAngleOffset already used)
  // For left/right we adjust differential power
  int leftPWM = basePWM;
  int rightPWM = basePWM;
  bool leftForward = forwardDir;
  bool rightForward = forwardDir;

  // steering: small differential added for left/right
  if (btCmd == 'L') {
    // turn left: slow right motor slightly (or reverse one motor a bit)
    leftPWM = max(0, basePWM - 40);
    rightPWM = min(MAX_PWM, basePWM + 40);
  } else if (btCmd == 'R') {
    leftPWM = min(MAX_PWM, basePWM + 40);
    rightPWM = max(0, basePWM - 40);
  } else {
    // straight: same for both
    leftPWM = basePWM;
    rightPWM = basePWM;
  }

  // apply to motors
  setMotorA(leftPWM, leftForward);
  setMotorB(rightPWM, rightForward);

  // debug prints occasionally
  static unsigned long dbgT = 0;
  if (millis() - dbgT > 250) {
    dbgT = millis();
    Serial.print("Angle: "); Serial.print(angle, 2);
    Serial.print(" | err: "); Serial.print(error, 2);
    Serial.print(" | pid: "); Serial.print(pid_output, 2);
    Serial.print(" | Cmd: "); Serial.print(btCmd);
    Serial.print(" | mOff: "); Serial.println(motionAngleOffset, 3);
  }

  // small delay - not too long (we want tight control loop)
  delay(2);
}

// ---------------- Sensor reading ----------------
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register for accel
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // read 14 bytes

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// ---------------- Calibration ----------------
void calibrateSensors() {
  Serial.println("Calibrating gyro... Keep robot still");
  const int samples = 200;
  long gx_sum = 0;
  for (int i = 0; i < samples; i++) {
    readMPU6050();
    gx_sum += gyroX;
    delay(5);
  }
  // raw gyro offset in deg/s units
  float gx_avg = (float)gx_sum / samples;
  gyroXoffset = gx_avg / 131.0; // convert to deg/s
  Serial.print("gyroXoffset (deg/s): "); Serial.println(gyroXoffset, 4);

  // initial accel angle offset - measure the accel angle now and set as zero
  readMPU6050();
  float accelAngle = atan2((float)accelY, (float)accelZ) * 180.0 / PI;
  accelAngleOffset = -accelAngle; // so initial accelAngle becomes ~0
  Serial.print("accelAngleOffset: "); Serial.println(accelAngleOffset, 4);

  // set initial angle to accel measurement
  angle = accelAngle + accelAngleOffset;
  Serial.print("Initial angle: "); Serial.println(angle, 4);
}

// ---------------- Bluetooth handler ----------------
void handleBluetooth() {
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == 'F' || c == 'B' || c == 'L' || c == 'R' || c == 'S') {
      btCmd = c;
      Serial.print("BT cmd: "); Serial.println(btCmd);
      applyCommand(btCmd);
    }
    // ignore other characters
  }
}

void applyCommand(char c) {
  // For forward/backward we change the motionAngleOffset (small tilt) for driving while balancing.
  const float DRIVE_ANGLE = 4.0; // degrees - small tilt for forward/back. Tune as needed.
  const float STOP_TILT = 0.0;

  if (c == 'F') {
    motionAngleOffset = -DRIVE_ANGLE; // tilt forward (sign depends on build)
  } else if (c == 'B') {
    motionAngleOffset = DRIVE_ANGLE; // tilt backward
  } else if (c == 'L') {
    // For turning, keep forward/back offset zero; turning implemented as differential PWM
    motionAngleOffset = STOP_TILT;
  } else if (c == 'R') {
    motionAngleOffset = STOP_TILT;
  } else if (c == 'S') {
    // stop both movement and reset offsets
    motionAngleOffset = STOP_TILT;
  }
}
