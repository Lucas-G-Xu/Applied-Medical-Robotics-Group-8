// Communication between Arduino and MATLAB
//   @author         Alejandro Granados
//   @organisation   King's College London
//   @module         Medical Robotics Hardware Development
//   @year           2023

#include <Arduino.h>

// Global variables for MATLAB communication
int i = 0;
String matlabStr = "";
bool readyToSend = false;

char c;
float val1 = 0.0;
float val2 = 0.0;

// PID control constants and variables
const float PPR = 3575.0855;
const float GR = 297.924;
const float CPR = 3;
const float WGR = 30;
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1, aLastState_m2;
const int encoderPinA_m1 = 2, encoderPinB_m1 = 10, encoderPinA_m2 = 3, encoderPinB_m2 = 11;
const int motorPin1_m1 = 4, motorPin2_m1 = 5, enablePin_m1 = 6;
const int motorPin1_m2 = 7, motorPin2_m2 = 8, enablePin_m2 = 9;
long currentPosition_m1 = 0, currentPosition_m2 = 0;
float demandPositionInDegrees_m1 = 0 * WGR;
float demandPositionInDegrees_m2 = -40 * WGR;
float currentPositionInDegrees_m1, currentPositionInDegrees_m2;
unsigned long currentTime, previousTime = 0, deltaT;
float Kp_m1 = 20, Kd_m1 = 0, Ki_m1 = 0;
float Kp_m2 = 20, Kd_m2 = 0, Ki_m2 = 0;
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(encoderPinA_m1, INPUT_PULLUP);
  pinMode(encoderPinA_m2, INPUT_PULLUP);
  pinMode(encoderPinB_m1, INPUT_PULLUP);
  pinMode(encoderPinB_m2, INPUT_PULLUP);
  pinMode(motorPin1_m1, OUTPUT);
  pinMode(motorPin2_m1, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(motorPin1_m2, OUTPUT);
  pinMode(motorPin2_m2, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m1), updateEncoder_m1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinA_m2), updateEncoder_m2, CHANGE);
  aLastState_m1 = digitalRead(encoderPinA_m1);
  aLastState_m2 = digitalRead(encoderPinA_m2);
  delay(3000);
  previousTime = micros();
}

void loop() {
  if (readyToSend == false) {
    if (Serial.available() > 0) {
      c = Serial.read();
      matlabStr += c;
      if (matlabStr.indexOf(";") != -1) {
        readyToSend = true;
        int posComma1 = matlabStr.indexOf(",");
        val1 = matlabStr.substring(1, posComma1).toFloat();
        int posEnd = matlabStr.indexOf(";");
        val2 = matlabStr.substring(posComma1 + 1, posEnd).toFloat();
        matlabStr = "";  // Clear string after processing
      }
    }
  }

  if (readyToSend) {
    currentTime = micros();
    deltaT = currentTime - previousTime;
    previousTime = currentTime;

    // Calculate current positions and PID control
    currentPositionInDegrees_m1 = ((counter_m1 * 360) / (CPR * GR * 2));
    currentPositionInDegrees_m2 = ((counter_m2 * 360) / (CPR * GR * 2));

    // Calculate errors and PID for motor 1
    float errorPositionInDegrees_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;
    float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;
    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);

    // Calculate errors and PID for motor 2
    float errorPositionInDegrees_m2 = demandPositionInDegrees_m2 - currentPositionInDegrees_m2;
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;
    float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;
    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);

    // Motor control logic
    setMotorOutput(motorPin1_m1, motorPin2_m1, enablePin_m1, controllerOutput_m1);
    setMotorOutput(motorPin1_m2, motorPin2_m2, enablePin_m2, controllerOutput_m2);

    // Send PID data to MATLAB
    Serial.print("c");
    Serial.print(currentPositionInDegrees_m1);
    Serial.print(",");
    Serial.print(currentPositionInDegrees_m2);
    Serial.print(";\n");  // Adding newline terminator for consistent reading
    delay(10);  // Brief delay to manage data flow timing

    readyToSend = false;
  }
}

void setMotorOutput(int pin1, int pin2, int enablePin, float output) {
  if (output > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(enablePin, output);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(enablePin, -output);
  }
}

void updateEncoder_m1() {
  int aState_m1 = digitalRead(encoderPinA_m1);
  if (aState_m1 != aLastState_m1) {
    counter_m1 += (digitalRead(encoderPinB_m1) != aState_m1) ? 1 : -1;
    aLastState_m1 = aState_m1;
  }
}

void updateEncoder_m2() {
  int aState_m2 = digitalRead(encoderPinA_m2);
  if (aState_m2 != aLastState_m2) {
    counter_m2 += (digitalRead(encoderPinB_m2) != aState_m2) ? 1 : -1;
    aLastState_m2 = aState_m2;
  }
}
