#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <pololu713.cpp>
#include <PID_v1.h>
#define IR_USE_AVR_TIMER1
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

// IR remote
#define IRDA A3
#define IR_START_RIGHT_RAW 0x85050080UL
#define IR_START_RIGHT_RAW_ALT 0xA020UL
#define IR_START_RIGHT_RAW_ALT2 0xA0200080UL
#define IR_START_LEFT_RAW 0xB2320080UL
#define IR_START_LEFT_RAW_ALT 0xB232UL
#define IR_STOP_RAW 0xBF3F0080UL
#define IR_STOP_RAW_ALT_SHORT 0xBF3FUL
#define IR_STOP_RAW_ALT 0xBE3E0080UL
#define IR_STOP_RAW_ALT2 0xBE3EUL
#define IR_REMOTE_ADDRESS_SHORT 0x80U
#define IR_STOP_COMMAND_SHORT 0x3FU
#define IR_STOP_COMMAND_SHORT_ALT 0x3EU
#define IR_STOP_COMMAND_LONG 0xBF3FU
#define IR_STOP_COMMAND_LONG_ALT 0xBE3EU


// distance sensors (analog pins — also used as digital)
#define left_sensor A4
#define right_sensor A5

// line sensors
#define left_line A0
#define right_line A1

// start module
#define start_pin 4
#define kill_pin 2

// -1 = left, 1 = right, 0 = none/both
int startDirection = 0;
bool initialManeuverDone = false;

Motor motor;

QTRSensors qtr;
uint16_t sensorValues[2];

QTRSensors qtr2;
uint16_t sensorValues2[2];

const int sensorPin1 = A5; // right
const int sensorPin2 = A4; // left

static int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
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

static void pivotRight(int speed) {
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  analogWrite(PWMA, pwm); analogWrite(PWMB, 0);
}

static void pivotLeft(int speed) {
  int pwm = clampInt(abs(speed), 0, 255);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
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
bool lastIrWasRepeat = false;

enum IrAction {
  IR_NONE,
  IR_START_RIGHT_ACTION,
  IR_START_LEFT_ACTION,
  IR_STOP_ACTION
};

IrAction lastIrAction = IR_NONE;

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

void runRobotLogic();

static bool isRawStartRight(uint32_t rawData) {
  return rawData == IR_START_RIGHT_RAW ||
         rawData == IR_START_RIGHT_RAW_ALT ||
         rawData == IR_START_RIGHT_RAW_ALT2;
}

static bool isRawStartLeft(uint32_t rawData) {
  return rawData == IR_START_LEFT_RAW ||
         rawData == IR_START_LEFT_RAW_ALT;
}
//
static bool isRawStop(uint32_t rawData) {
  return rawData == IR_STOP_RAW ||
         rawData == IR_STOP_RAW_ALT_SHORT ||
         rawData == IR_STOP_RAW_ALT ||
         rawData == IR_STOP_RAW_ALT2;
}

static bool rawContains16(uint32_t rawData, uint16_t value) {
  return (uint16_t)(rawData & 0xFFFFUL) == value ||
         (uint16_t)((rawData >> 16) & 0xFFFFUL) == value;
}

static bool rawContains8(uint32_t rawData, uint8_t value) {
  return (uint8_t)(rawData & 0xFFUL) == value ||
         (uint8_t)((rawData >> 8) & 0xFFUL) == value ||
         (uint8_t)((rawData >> 16) & 0xFFUL) == value ||
         (uint8_t)((rawData >> 24) & 0xFFUL) == value;
}

static void printIrDebug() {
  Serial.print("IR p=");
  Serial.print((int)IrReceiver.decodedIRData.protocol);
  Serial.print(" a=0x");
  Serial.print(IrReceiver.decodedIRData.address, HEX);
  Serial.print(" c=0x");
  Serial.print(IrReceiver.decodedIRData.command, HEX);
  Serial.print(" raw=0x");
  Serial.print(IrReceiver.decodedIRData.decodedRawData, HEX);
  Serial.print(" f=0x");
  Serial.println(IrReceiver.decodedIRData.flags, HEX);
}

static IrAction decodeIrAction() {
  uint32_t rawData = IrReceiver.decodedIRData.decodedRawData;
  uint16_t address = IrReceiver.decodedIRData.address;
  uint16_t command = IrReceiver.decodedIRData.command;
  uint8_t flags = IrReceiver.decodedIRData.flags;

  if ((flags & IRDATA_FLAGS_IS_REPEAT) != 0 && lastIrAction != IR_NONE) {
    lastIrWasRepeat = true;
    return lastIrAction;
  }

  lastIrWasRepeat = false;

  if (isRawStop(rawData) ||
      rawContains16(rawData, IR_STOP_COMMAND_LONG) ||
      rawContains16(rawData, IR_STOP_COMMAND_LONG_ALT) ||
      rawContains8(rawData, IR_STOP_COMMAND_SHORT) ||
      rawContains8(rawData, IR_STOP_COMMAND_SHORT_ALT) ||
      ((address == IR_REMOTE_ADDRESS_SHORT || address == 0x0U) &&
       (command == IR_STOP_COMMAND_SHORT ||
        command == IR_STOP_COMMAND_SHORT_ALT ||
        command == IR_STOP_COMMAND_LONG ||
        command == IR_STOP_COMMAND_LONG_ALT))) {
    return IR_STOP_ACTION;
  }

  if (isRawStartRight(rawData)) {
    return IR_START_RIGHT_ACTION;
  }

  if (isRawStartLeft(rawData)) {
    return IR_START_LEFT_ACTION;
  }

  return IR_NONE;
}

static void applyStart(int direction) {
  startDirection = direction;
  startSignal = true;
  killSignal = false;
  robotState = SEARCHING;
  initialManeuverDone = false;
}

static void applyStop() {
  killSignal = true;
  startSignal = false;
  initialManeuverDone = false;
  robotState = SEARCHING;
  stopMotors();
}

void setup() {
  Serial.begin(115200);
  Serial.println("IR debug on");

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1}, 2);
  qtr.setEmitterPin(255);

  qtr2.setTypeAnalog();
  qtr2.setSensorPins((const uint8_t[]){A4, A5}, 2);
  qtr2.setEmitterPin(255);

  for (uint8_t i = 0; i < 100; i++) { qtr.calibrate(); delay(20); }
  for (int i = 0; i < 400; i++) { qtr2.calibrate(); delay(1); }

  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  
  pinMode(IRDA, INPUT_PULLUP);
  IrReceiver.begin(IRDA, false);

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

  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, LOW);

  pidSetpoint = 0.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-TURN_AMOUNT, TURN_AMOUNT);
  myPID.SetSampleTime((int)LOOP_MS);
}

void loop() {
  if (IrReceiver.decode()) {
    IrAction action = decodeIrAction();
    printIrDebug();

    if (action == IR_STOP_ACTION) {
      lastIrAction = IR_STOP_ACTION;
      applyStop();
      if (lastIrWasRepeat) {
        Serial.println("IR action: STOP repeat");
      } else {
        Serial.println("IR action: STOP");
      }
    } else if (action == IR_START_RIGHT_ACTION) {
      lastIrAction = IR_START_RIGHT_ACTION;
      applyStart(1);
      Serial.println("IR action: START RIGHT");
    } else if (action == IR_START_LEFT_ACTION) {
      lastIrAction = IR_START_LEFT_ACTION;
      applyStart(-1);
      Serial.println("IR action: START LEFT");
    } else {
      lastIrAction = IR_NONE;
    }
    IrReceiver.resume();
  }

  unsigned long now = millis();
  if (now - lastTime < LOOP_MS) {
    return;
  }
  lastTime = now;

  if(startSignal && !killSignal) {
    digitalWrite(led_red, LOW);
    digitalWrite(led_green, HIGH);
    runRobotLogic();
  } else {
    stopMotors();
    bool ledState = (millis() / 500) % 2;
    digitalWrite(led_red, ledState ? HIGH : LOW);
    digitalWrite(led_green, LOW);
  }
}

void runRobotLogic() {
  unsigned long currentTime = millis();

  switch (robotState) {
    case INITIAL_MANEUVER:
      if (startDirection == -1) spinCounterClockwise(SEARCH_SPEED);
      else if (startDirection == 1) spinClockwise(SEARCH_SPEED);
      maneuverStartTime = currentTime;
      robotState = INITIAL_MANEUVER_STOP;
      break;

    case INITIAL_MANEUVER_STOP:
      if (currentTime - maneuverStartTime >= 350) {
        stopMotors();
        maneuverStartTime = currentTime;
        robotState = SEARCHING;
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
        stopMotors();
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
