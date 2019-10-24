#include "TeensyStep.h"

#define SPR 1600
#define W_MAX 400
#define RAD2STEP 14260   // Steps per rad (56*1600/2pi)
#define OFFSET_2 3850    // Coordinate system offset in steps (0.27 rad, 15.47 deg)
#define OFFSET_1 8674    // Limit switch offset used for homing in steps (0.60825 rad , 34.85 deg)
#define ZERO_1 32275     // Zero position with respect to L position pi*RAD2STEP - OFFSET_2 - OFFSET_1

#define STEPPER_1_STEP_PIN 2
#define STEPPER_1_DIRECTION_PIN 3
#define STEPPER_2_STEP_PIN 4
#define STEPPER_2_DIRECTION_PIN 5
#define OUTER_LIM_PIN 31
#define INNER_LIM_PIN 32

#define HOMING_STATE 0
#define IK_STATE 1

Stepper stepper_1(2, 3);
Stepper stepper_2(4, 5);

RotateControl teleopController_1, teleopController_2;
StepControl ikController;

//Genearl Vars
int count = 0;
int state;

//Serial vars
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];
boolean newData = false;

//Stepper vars
float p_1 = 0.0;
float p_2 = 0.0;
float target_1 = 0.0;
float target_2 = 0.0;
float w_1 = 0.0;
float w_2 = 0.0;
int dir_1 = 0;
int dir_2 = 0;

bool eStop = false;
bool outerStop = false;
bool innerStop = false;

bool running_2 = false;
bool stopping_2 = false;
bool prev_Running_2 = false;

bool running_1 = false;
bool stopping_1 = false;
bool prev_Running_1 = false;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(OUTER_LIM_PIN, INPUT_PULLUP);
  pinMode(INNER_LIM_PIN, INPUT_PULLUP);

  state = HOMING_STATE;

  delay(5000);
}

void loop()
{
  switch (state) {

    case HOMING_STATE:
      readLimitSwitches();

      //////////////// LEFT STEPPER HOMING ////////////////

      stepper_2
      .setMaxSpeed(-RPMTS(100))
      .setAcceleration(50000);
      teleopController_2.rotateAsync(stepper_2);

      while (!outerStop) {
        readLimitSwitches();
      }
      teleopController_2.emergencyStop();

      stepper_2.setMaxSpeed(RPMTS(100));
      teleopController_2.rotateAsync(stepper_2);

      delay(1000);
      teleopController_2.emergencyStop();

      stepper_2.setMaxSpeed(-RPMTS(30));
      teleopController_2.rotateAsync(stepper_2);

      readLimitSwitches();
      while (!outerStop) {
        readLimitSwitches();
      }
      teleopController_2.emergencyStop();

      stepper_2.setPosition(0);

      //////////////// RIGHT STEPPER HOMING ////////////////

      stepper_2
      .setMaxSpeed(RPMTS(100))
      .setAcceleration(50000);
      teleopController_2.rotateAsync(stepper_2);

      while (!innerStop) {
        readLimitSwitches();
      }
      teleopController_2.emergencyStop();

      stepper_2.setMaxSpeed(-RPMTS(100));
      teleopController_2.rotateAsync(stepper_2);

      delay(1000);
      teleopController_2.emergencyStop();

      stepper_2.setMaxSpeed(RPMTS(30));
      teleopController_2.rotateAsync(stepper_2);

      readLimitSwitches();
      while (!innerStop) {
        readLimitSwitches();
      }
      teleopController_2.emergencyStop();

      stepper_1.setPosition(stepper_2.getPosition() - ZERO_1);

      stepper_2.setMaxSpeed(-RPMTS(100));
      teleopController_2.rotateAsync(stepper_2);

      delay(1000);
      teleopController_2.stop();

      state = IK_STATE;
      break;

    case IK_STATE:
      readLimitSwitches();
      eStop = outerStop || innerStop;

      recvWithStartEndMarkers();
      if (newData == true) {
        strcpy(tempChars, receivedChars);
        parsePositionInputs();
        if (target_1 != 0 && target_2 != 0) {
          stepper_1
          .setMaxSpeed(RPMTS(400))
          .setAcceleration(60000)
          .setTargetAbs(target_1);
          stepper_2
          .setMaxSpeed(RPMTS(400))
          .setAcceleration(60000)
          .setTargetAbs(target_2);
          ikController.moveAsync(stepper_1, stepper_2);
        }
        newData = false;
      }

      if (eStop) {
        ikController.emergencyStop();
        while (true);
      }

      break;
  }
}

float RPMTS(float w) {
  return round(w * SPR / 60);
}

void readLimitSwitches() {
  outerStop = digitalRead(OUTER_LIM_PIN) == HIGH;
  innerStop = digitalRead(INNER_LIM_PIN) == HIGH;
}

void parsePositionInputs() {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");
  target_1 = atof(strtokIndx);
  strtokIndx = strtok(NULL, ",");
  target_2 = atof(strtokIndx);
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
