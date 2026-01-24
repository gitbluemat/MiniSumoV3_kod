#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <pololu713.cpp>
#include <PID_v1.h>
#include <IRremote.hpp>
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

//starting
#define IRDA A3
#define IR_START 0xBF3F
#define IR_STOP 0xBE3E
#define IR_LEFT 0xA121
#define IR_RIGHT 0xA020


// distance sensors (analog pins — also used as digital)
#define left_sensor A4
#define right_sensor A5

// line sensors
#define left_line A0
#define right_line A1

// start module
#define start_pin 4
#define kill_pin 2

// global start direction: -1 = left, 1 = right, 0 = none/both
int startDirection = 0;
bool initialManeuverDone = false;

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

bool startSignal = false;
bool killSignal = false;

// State machine for maneuvers
enum RobotState {
  SEARCHING,
  INITIAL_MANEUVER,
  INITIAL_MANEUVER_STOP,
  RECOVER_BOTH_BACKUP,
  RECOVER_BOTH_SPIN,
  RECOVER_LEFT_BACKUP,
  RECOVER_LEFT_PIVOT,
  RECOVER_RIGHT_BACKUP,
  RECOVER_RIGHT_PIVOT,
  MANEUVER_STOP
};
RobotState robotState = SEARCHING;
unsigned long maneuverStartTime = 0;

void runRobotLogic(); // Function prototype

void setup() {
   Serial.begin(115200);
   Serial.println("Robot gotowy do wkurwiania. Zegar 8MHz OK."); // commented per request

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1}, 2);
  qtr.setEmitterPin(255);

  qtr2.setTypeAnalog();
  qtr2.setSensorPins((const uint8_t[]){A4, A5}, 2);
  qtr2.setEmitterPin(255);

  // calibrate
  for (uint8_t i = 0; i < 100; i++) { qtr.calibrate(); delay(20); }
  for (int i = 0; i < 400; i++) { qtr2.calibrate(); delay(1); }

  // configure LED pins before any digitalWrite
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  
  pinMode(IRDA, INPUT_PULLUP);
  IrReceiver.begin(IRDA, ENABLE_LED_FEEDBACK);

  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);

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

  // configure PID
  pidSetpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-TURN_AMOUNT, TURN_AMOUNT);
  myPID.SetSampleTime((int)LOOP_MS);
}

void loop() {
  // Check for IR commands as often as possible, outside the main logic loop
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
    // IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    // IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
    
    if (IrReceiver.decodedIRData.decodedRawData == IR_START) {
      //Serial.println("START");
      startSignal = true;
      killSignal = false;
      // Reset state on start
      robotState = SEARCHING;
      initialManeuverDone = false; 
    } else if (IrReceiver.decodedIRData.decodedRawData == IR_STOP) {
      //Serial.println("STOP");
      killSignal = true;
      startSignal = false;
    } else if (IrReceiver.decodedIRData.decodedRawData == IR_LEFT) {
      //Serial.println("LEFT");
      if (!startSignal) startDirection = -1; // Allow setting direction only before start
    } else if (IrReceiver.decodedIRData.decodedRawData == IR_RIGHT) {
      //Serial.println("RIGHT");
      if (!startSignal) startDirection = 1; // Allow setting direction only before start
    }
    IrReceiver.resume();
  }

  // Run the main robot logic at a fixed interval
  unsigned long now = millis();
  if (now - lastTime < LOOP_MS) {
    return; // Not time yet
  }
  lastTime = now;

  if(startSignal && !killSignal) {
    digitalWrite(led_red, LOW);
    digitalWrite(led_green, HIGH);
    runRobotLogic();
  } else {
    motor.forward(0);
    bool ledState = (millis() / 500) % 2;
    digitalWrite(led_red, ledState ? HIGH : LOW);
    digitalWrite(led_green, LOW);
  }
}

void runRobotLogic() {
  unsigned long currentTime = millis();

  // State machine for maneuvers
  switch (robotState) {
    case INITIAL_MANEUVER:
      if (startDirection == -1) spinCounterClockwise(SEARCH_SPEED);
      else if (startDirection == 1) spinClockwise(SEARCH_SPEED);
      maneuverStartTime = currentTime;
      robotState = INITIAL_MANEUVER_STOP;
      break;

    case INITIAL_MANEUVER_STOP:
      if (currentTime - maneuverStartTime >= 350) {
        motor.forward(0);
        maneuverStartTime = currentTime;
        robotState = SEARCHING; // Or a brief pause state
      }
      break;

    case RECOVER_BOTH_BACKUP:
      motor.backward(200);
      maneuverStartTime = currentTime;
      robotState = RECOVER_BOTH_SPIN;
      break;

    case RECOVER_BOTH_SPIN:
      if (currentTime - maneuverStartTime >= 250) {
        spinClockwise(180);
        maneuverStartTime = currentTime;
        robotState = MANEUVER_STOP;
      }
      break;

    case RECOVER_LEFT_BACKUP:
      motor.backward(100);
      maneuverStartTime = currentTime;
      robotState = RECOVER_LEFT_PIVOT;
      break;

    case RECOVER_LEFT_PIVOT:
      if (currentTime - maneuverStartTime >= 150) {
        pivotRight(180);
        maneuverStartTime = currentTime;
        robotState = MANEUVER_STOP;
      }
      break;

    case RECOVER_RIGHT_BACKUP:
      motor.backward(100);
      maneuverStartTime = currentTime;
      robotState = RECOVER_RIGHT_PIVOT;
      break;

    case RECOVER_RIGHT_PIVOT:
      if (currentTime - maneuverStartTime >= 150) {
        pivotLeft(180);
        maneuverStartTime = currentTime;
        robotState = MANEUVER_STOP;
      }
      break;

    case MANEUVER_STOP:
      if (currentTime - maneuverStartTime >= 300) { // Duration for pivot/spin
        motor.forward(0);
        maneuverStartTime = currentTime;
        robotState = SEARCHING;
      }
      break;

    case SEARCHING:
      // This is the default state where we check sensors and decide what to do
      if (!initialManeuverDone) {
        initialManeuverDone = true;
        robotState = INITIAL_MANEUVER;
        break;
      }
      
      qtr.read(sensorValues);
      qtr2.read(sensorValues2);
      int qtrL = sensorValues[0];
      int qtrR = sensorValues[1];
      bool left_line_detect = (qtrL < EDGE_THRESHOLD);
      bool right_line_detect = (qtrR < EDGE_THRESHOLD);

      if (left_line_detect || right_line_detect) {
        if (left_line_detect && right_line_detect) {
          robotState = RECOVER_BOTH_BACKUP;
        } else if (left_line_detect) {
          robotState = RECOVER_LEFT_BACKUP;
        } else {
          robotState = RECOVER_RIGHT_BACKUP;
        }
        break;
      }

      // Opponent detection logic remains here
      int rawRightDigital = digitalRead(sensorPin1);
      int rawLeftDigital = digitalRead(sensorPin2);

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

      bool left_detected = (leftDetectCounter >= DEBOUNCE_COUNT);
      bool right_detected = (rightDetectCounter >= DEBOUNCE_COUNT);

      if (left_detected || right_detected) {
        if (left_detected && !right_detected) {
          motor.left(BASE_SPEED);
        } else if (right_detected && !left_detected) {
          motor.right(BASE_SPEED);
        } else {
          motor.forward(BASE_SPEED);
        }
      } else {
        spinClockwise(SEARCH_SPEED);
      }
      break;
  }
}