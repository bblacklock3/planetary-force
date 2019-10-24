#include "TeensyStep.h"
#include <Servo.h>

#define CCW 48
#define CW 49
#define SPR 1600
#define W_MAX 400

#define STEPPER_1_STEP_PIN 2
#define STEPPER_1_DIRECTION_PIN 3
#define STEPPER_2_STEP_PIN 4
#define STEPPER_2_DIRECTION_PIN 5
#define STEPPER_3_STEP_PIN 6
#define STEPPER_3_DIRECTION_PIN 7

#define WRIST_SERVO_PIN 8
#define GRIPPER_SERVO_PIN 9

#define WRIST_MIN_PULSE 450
#define WRIST_MAX_PULSE 2450
#define GRIPPER_MIN_PULSE 750
#define GRIPPER_MAX_PULSE 1650

#define OUTER_LIM_PIN 31
#define INNER_LIM_PIN 32

Stepper stepper_1(2, 3);
Stepper stepper_2(4, 5);
Stepper stepper_3(6, 7);
Servo wristServo, gripperServo;

RotateControl teleopController_1, teleopController_2, teleopController_3;
StepControl jointController;

//Genearl Vars
int count = 0;
int state;

//Serial vars
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

float w_1 = 0.0;
float w_2 = 0.0;
float w_3 = 0.0;
float w_pos = 0.5;
float g_pos = 0;

float p_1_joint_1 = 0.0;
float p_2_joint_1 = 0.0;
float p_3_joint_1 = 0.0;
float p_1_joint_2 = 0.0;
float p_2_joint_2 = 0.0;
float p_3_joint_2 = 0.0;

int dir_1 = 0;
int dir_2 = 0;
int dir_3 = 0;

float p_1, p_2, p_3, prev_p_1, prev_p_2, prev_p_3;

bool eStop = false;
bool outerStop = false;
bool innerStop = false;

bool running_1 = false;
bool stopping_1 = false;
bool prev_Running_1 = false;

bool running_2 = false;
bool stopping_2 = false;
bool prev_Running_2 = false;

bool running_3 = false;
bool stopping_3 = false;
bool prev_Running_3 = false;

int rec_joints = 0;
int joints = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(OUTER_LIM_PIN, INPUT_PULLUP);
  pinMode(INNER_LIM_PIN, INPUT_PULLUP);
  wristServo.attach(WRIST_SERVO_PIN);
  gripperServo.attach(GRIPPER_SERVO_PIN);
}

void loop()
{
  readLimitSwitches();
  eStop = outerStop || innerStop;

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseSpeedInputs();
    newData = false;
  }

  //////////////// RIGHT STEPPER TELEOP CONTROL ////////////////

  running_1 = w_1 > 0 && joints == 0;

  if (!stopping_1 && !prev_Running_1 && running_1) {
    stepper_1
    .setMaxSpeed(RPMTS(W_MAX))       // steps/s
    .setAcceleration(50000);   // steps/s^2
    teleopController_1.rotateAsync(stepper_1);
  }

  if (stopping_1) {
    teleopController_1.stopAsync();
    if (!teleopController_1.isRunning()) {
      stopping_1 = false;
    }
  }
  else if (running_1) {
    teleopController_1.overrideSpeed(dir_1 * w_1 / 100);
  }
  else {
    stopping_1 = true;
  }

  prev_Running_1 = running_1;

  //////////////// LEFT STEPPER TELEOP CONTROL ////////////////

  running_2 = w_2 > 0 && joints == 0;

  if (!stopping_2 && !prev_Running_2 && running_2) {
    stepper_2
    .setMaxSpeed(RPMTS(W_MAX))       // steps/s
    .setAcceleration(50000);   // steps/s^2
    teleopController_2.rotateAsync(stepper_2);
  }

  if (stopping_2) {
    teleopController_2.stopAsync();
    if (!teleopController_2.isRunning()) {
      stopping_2 = false;
    }
  }
  else if (running_2) {
    teleopController_2.overrideSpeed(dir_2 * w_2 / 100);
  }
  else {
    stopping_2 = true;
  }

  prev_Running_2 = running_2;

  //////////////// YAW STEPPER TELEOP CONTROL ////////////////

  running_3 = w_3 > 0 && joints == 0;

  if (!stopping_3 && !prev_Running_3 && running_3) {
    stepper_3
    .setMaxSpeed(RPMTS(150))       // steps/s
    .setAcceleration(30000);   // steps/s^2
    teleopController_3.rotateAsync(stepper_3);
  }

  if (stopping_3) {
    teleopController_3.stopAsync();
    if (!teleopController_3.isRunning()) {
      stopping_3 = false;
    }
  }
  else if (running_3) {
    teleopController_3.overrideSpeed(dir_3 * w_3 / 100);
  }
  else {
    stopping_3 = true;
  }

  prev_Running_3 = running_3;

  //////////////// SERVO TELEOP CONTROL ////////////////

  wristServo.writeMicroseconds(map(w_pos, 0, 100, WRIST_MIN_PULSE, WRIST_MAX_PULSE));
  gripperServo.writeMicroseconds(map(g_pos, 0, 100, GRIPPER_MIN_PULSE, GRIPPER_MAX_PULSE));

  //////////////// JOINT RECORDING TELEOP CONTROL ////////////////

  if (rec_joints != 0 && !teleopController_1.isRunning() && !teleopController_2.isRunning() && !teleopController_3.isRunning()) {
    if (rec_joints == 1) {
      p_1_joint_1 = stepper_1.getPosition();
      p_2_joint_1 = stepper_2.getPosition();
      p_3_joint_1 = stepper_3.getPosition();
    }
    else if (rec_joints == 2) {
      p_1_joint_2 = stepper_1.getPosition();
      p_2_joint_2 = stepper_2.getPosition();
      p_3_joint_2 = stepper_3.getPosition();
    }
  }

  //////////////// JOINT RECALL TELEOP CONTROL ////////////////

  if (joints != 0 && !teleopController_1.isRunning() && !teleopController_2.isRunning() && !teleopController_3.isRunning()) {
    stepper_1
    .setMaxSpeed(RPMTS(150))
    .setAcceleration(15000);
    stepper_2
    .setMaxSpeed(RPMTS(150))
    .setAcceleration(15000);
    stepper_3
    .setMaxSpeed(RPMTS(150))
    .setAcceleration(15000);
    if (joints == 1) {
      stepper_1.setTargetAbs(p_1_joint_1);
      stepper_2.setTargetAbs(p_2_joint_1);
      stepper_3.setTargetAbs(p_3_joint_1);
    }
    else if (joints == 2) {
      stepper_1.setTargetAbs(p_1_joint_2);
      stepper_2.setTargetAbs(p_2_joint_2);
      stepper_3.setTargetAbs(p_3_joint_2);
    }
    jointController.move(stepper_1, stepper_2, stepper_3);
  }

}

float RPMTS(float w) {
  return round(w * SPR / 60);
}

void readLimitSwitches() {
  outerStop = digitalRead(OUTER_LIM_PIN) == HIGH;
  innerStop = digitalRead(INNER_LIM_PIN) == HIGH;
}

void parseSpeedInputs() {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");
  w_1 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  dir_1 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  w_2 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  dir_2 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  w_3 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  dir_3 = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  w_pos = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  g_pos = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  rec_joints = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  joints = atoi(strtokIndx);
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
