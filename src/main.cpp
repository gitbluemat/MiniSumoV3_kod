#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <pololu713.cpp>
#include <PID_v1.h>
#include "../QTRSensors/QTRSensors.h"

// POLOLU-713
#define AIN1  8
#define AIN2  9

#define BIN1 PD6
#define BIN2 PD7

#define PWMA PD3
#define PWMB PD5

// status LED
#define led_red PB2
#define led_green A2
#define led_blue A3

// distance sensors (analog pins — also used as digital)
#define left_sensor A4
#define right_sensor A5

// line sensors
#define left_line A0
#define right_line A1

// start module
#define start_pin 4
#define kill_pin 2

// start jumper pins (PD0 = D0, PD1 = D1)
#define START_LEFT_PIN  0  // PD0
#define START_RIGHT_PIN 1  // PD1

// global start direction: -1 = left, 1 = right, 0 = none/both
int startDirection = 0;
bool initialManeuverDone = false;

// run state machine for start/kill
enum RobotState { POWER_ON, STARTED, STOPPED };
RobotState robotState = POWER_ON;

Motor motor;

QTRSensors qtr;
uint16_t sensorValues[2];

QTRSensors qtr2;
uint16_t sensorValues2[2];

const int sensorPin1 = A5; // right
const int sensorPin2 = A4; // left

// helpers
static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static void spinClockwise(int speed) {
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); // left forward
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); // right backward
  analogWrite(PWMA, pwm); analogWrite(PWMB, pwm);
}
static void spinCounterClockwise(int speed) {
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); // left backward
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); // right forward
  analogWrite(PWMA, pwm); analogWrite(PWMB, pwm);
}

static void pivotRight(int speed) { // Turn right -> left wheel fwd, right wheel stop
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); // left forward
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);  // right stop
  analogWrite(PWMA, pwm); analogWrite(PWMB, 0);
}

static void pivotLeft(int speed) { // Turn left -> right wheel fwd, left wheel stop
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);  // left stop
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); // right forward
  analogWrite(PWMA, 0); analogWrite(PWMB, pwm);
}

// PID_v1 integration
float Kp = 0.6;
float Ki = 0.0;
float Kd = 0.3;
double pidInput = 0.0;
double pidOutput = 0.0;
double pidSetpoint = 0.0;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// tuning
const int EDGE_THRESHOLD = 900;
const int DIST_THRESHOLD = 200;
const int BASE_SPEED = 200;
const int SEARCH_SPEED = 100;
const unsigned long LOOP_MS = 30;

// filtering / smoothing
const float DIST_ALPHA = 0.06;
int smoothedLeft = 0;
int smoothedRight = 0;

// ramping
const int MAX_STEP = 5;
int prevLeftSpeed = 0;
int prevRightSpeed = 0;

const int TURN_AMOUNT = 35;

// digital debouncing for A4/A5 when used as digital inputs
const int DEBOUNCE_COUNT = 2;
int leftDetectCounter = 0;
int rightDetectCounter = 0;

unsigned long lastTime = 0;

void runRobotLogic(); // Function prototype

void recoverEdge() {
  motor.backward(200);
  delay(200);
  motor.right(180);
  delay(300);
  motor.forward(0);
}

void setup() {
  // Serial.begin(9600); // commented per request

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1}, 2);
  qtr.setEmitterPin(255);

  qtr2.setTypeAnalog();
  qtr2.setSensorPins((const uint8_t[]){A4, A5}, 2);
  qtr2.setEmitterPin(255);

  // calibrate
  for (uint8_t i = 0; i < 100; i++) { qtr.calibrate(); delay(20); }
  for (int i = 0; i < 400; i++) { qtr2.calibrate(); delay(1); }

  // start jumper pins
  pinMode(START_LEFT_PIN, INPUT);
  pinMode(START_RIGHT_PIN, INPUT);
  bool left_jumper  = digitalRead(START_LEFT_PIN)  == HIGH;
  bool right_jumper = digitalRead(START_RIGHT_PIN) == HIGH;
  if (left_jumper && !right_jumper) startDirection = -1;
  else if (right_jumper && !left_jumper) startDirection = 1;
  else startDirection = 0;

  // configure LED pins before any digitalWrite
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  // initial visual: POWER_ON -> RED
  robotState = POWER_ON;
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);
  digitalWrite(led_blue, LOW);

  pinMode(sensorPin1, INPUT);
  pinMode(sensorPin2, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(left_sensor, INPUT);
  pinMode(right_sensor, INPUT);
  pinMode(left_line, INPUT);
  pinMode(right_line, INPUT);

  pinMode(start_pin, INPUT);
  pinMode(kill_pin, INPUT);

  // ensure red LED stays on at boot
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);
  digitalWrite(led_blue, LOW);

  // configure PID
  pidSetpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-TURN_AMOUNT, TURN_AMOUNT);
  myPID.SetSampleTime((int)LOOP_MS);
}

void loop() {
  unsigned long now = millis();
  if (lastTime == 0) lastTime = now;
  unsigned long elapsed = now - lastTime;
  if (elapsed < LOOP_MS) { delay(LOOP_MS - elapsed); now = millis(); elapsed = now - lastTime; }
  lastTime = now;

  // --- State Machine Logic based on Start Module ---
  bool startSignal = digitalRead(start_pin);
  bool killSignal = digitalRead(kill_pin);

  // State transitions
  switch (robotState) {
    case POWER_ON:
      digitalWrite(led_red, HIGH);
      digitalWrite(led_green, LOW);
      motor.forward(0);
      if (startSignal == HIGH && killSignal == HIGH) {
        robotState = STARTED;
      }
      break;

    case STARTED:
      digitalWrite(led_red, LOW);
      digitalWrite(led_green, HIGH);
      runRobotLogic();
      if (startSignal == LOW && killSignal == LOW) {
        robotState = STOPPED;
      }
      break;

    case STOPPED:
      motor.forward(0);
      bool ledState = (millis() / 500) % 2;
      digitalWrite(led_red, ledState ? HIGH : LOW);
      digitalWrite(led_green, LOW);
      // Terminal state, requires reset
      break;
  }
}

void runRobotLogic() {
  // This function contains the core operational logic from the original loop()

  // Perform initial start maneuver based on jumper setting
  if (!initialManeuverDone) {
    if (startDirection == -1) { // left
      spinCounterClockwise(SEARCH_SPEED);
      delay(350);
    } else if (startDirection == 1) { // right
      spinClockwise(SEARCH_SPEED);
      delay(350);
    }
    // if startDirection is 0, do nothing and proceed to search
    initialManeuverDone = true;
    motor.forward(0); // Stop briefly before continuing
    delay(100);
  }

  // read QTR sensors
  qtr.read(sensorValues);   // A0/A1
  qtr2.read(sensorValues2); // A4/A5 analogs used for PID

  int qtrL = sensorValues[0];
  int qtrR = sensorValues[1];

  // line detection (QTR-1A): 1023 = black, smaller = white (line)
  bool left_line_detect  = (qtrL < EDGE_THRESHOLD);
  bool right_line_detect = (qtrR < EDGE_THRESHOLD);

  if (left_line_detect || right_line_detect) {
    if (left_line_detect && right_line_detect) {
      // Both sensors see the line, emergency reverse and turn
      motor.backward(200);
      delay(250);
      spinClockwise(180); // Spin right
      delay(400);
    } else if (left_line_detect) {
      // Left sensor sees the line, pivot right
      motor.backward(100); // short backup
      delay(150);
      pivotRight(180);
      delay(300);
    } else if (right_line_detect) {
      // Right sensor sees the line, pivot left
      motor.backward(100); // short backup
      delay(150);
      pivotLeft(180);
      delay(300);
    }
    motor.forward(0); // Stop after maneuver
    delay(50);
    leftDetectCounter = 0;
    rightDetectCounter = 0;
    return;
  }

  // digital distance sensors A4/A5 used as digital inputs (LOW = detected)
  int rawRightDigital = digitalRead(sensorPin1);
  int rawLeftDigital  = digitalRead(sensorPin2);

  if (rawLeftDigital == LOW) {
    if (leftDetectCounter < DEBOUNCE_COUNT) leftDetectCounter++;
  } else {
    if (leftDetectCounter > 0) leftDetectCounter--;
  }
  if (rawRightDigital == LOW) {
    if (rightDetectCounter < DEBOUNCE_COUNT) rightDetectCounter++;
  } else {
    if (rightDetectCounter > 0) rightDetectCounter--;
  }

  // PID correction from analog A4/A5 pair
  pidInput = (double)((int)sensorValues2[0] - (int)sensorValues2[1]);
  myPID.Compute();
  // int corr = (int)round(pidOutput); // Unused variable

  bool left_detected  = (leftDetectCounter  >= DEBOUNCE_COUNT);
  bool right_detected = (rightDetectCounter >= DEBOUNCE_COUNT);

  // if (left_detected || right_detected) {
  //   // Opponent detected, ATTACK!
  //   if (left_detected && !right_detected) {
  //     // Opponent on the left. Turn left to face it.
  //     motor.left(BASE_SPEED);
  //   } else if (right_detected && !left_detected) {
  //     // Opponent on the right. Turn right to face it.
  //     motor.right(BASE_SPEED);
  //   } else {
  //     // Opponent is (roughly) straight ahead. Full speed forward.
  //     motor.forward(BASE_SPEED);
  //   }
  // } else {
  //   // No opponent, search.
  //   spinClockwise(SEARCH_SPEED);
  // }
}