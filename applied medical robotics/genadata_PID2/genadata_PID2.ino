// Communicaiton between Arduino and MATLAB
//   @author         Alejandro Granados
//   @organisation   King's College London
//   @module         Medical Robotics Hardware Development
//   @year           2023

// Global variables
int T = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab

// Define constants for pulses per revolution (PPR) and gear ratio (GR)
const float PPR = 3575.0855;
const float GR = 297.924;
const float CPR = 3; //cycles per revolution
const float WGR = 30; //worm gear ratio

// Variables for tracking encoder positions
volatile long counter_m1 = 0;
volatile long counter_m2 = 0;
int aLastState_m1;
int aLastState_m2;

// Pins for reading encoders of motor 1 and 2
const int encoderPinA_m1 = 2;
const int encoderPinB_m1 = 10;
const int encoderPinA_m2 = 3;
const int encoderPinB_m2 = 11;

// Pins for setting the direction of motor 1 and 2
const int motorPin1_m1 = 4;
const int motorPin2_m1 = 5;
const int motorPin1_m2 = 7;
const int motorPin2_m2 = 8;

// Pins for setting the speed of rotation (Enable pin) of motors 1 and 2
const int enablePin_m1 = 6;
const int enablePin_m2 = 9;

// Variables for encoder positions and desired positions
long currentPosition_m1 = 0;
long currentPosition_m2 = 0;
float demandPositionInDegrees_m1; // multiply by 30 to account for gear
float demandPositionInDegrees_m2; //always negative to be in the same direction
float currentPositionInDegrees_m1;
float currentPositionInDegrees_m2;
float errorPositionInDegrees_m1=100;
float errorPositionInDegrees_m2=100;

// Time parameters
unsigned long currentTime;
unsigned long previousTime = 0;
unsigned long deltaT;

// PID gains
float Kp_m1 = 18, Kd_m1 = 0.25, Ki_m1 = 0.0; //Start with P, then D, then I. Low P will be slow
float Kp_m2 = 15, Kd_m2 = 0.25, Ki_m2 = 0.0;

// Error values
float errorPositionInDegrees_prev_m1 = 0, errorPositionInDegrees_sum_m1 = 0;
float errorPositionInDegrees_prev_m2 = 0, errorPositionInDegrees_sum_m2 = 0;


/* Initialisation function of Arduino */
void setup() {
  // configure serial communication speed
  Serial.begin(9600);
  // Task 1: Initialize the pins using pinMode and attachInterrupt functions
  digitalWrite(13, LOW);
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

  // First wait to receive data from matlab to arduino
  if (readyToSend == false) {
    if (Serial.available()>0)       // is there anything received?
    {
      c = Serial.read();            // read characters
      matlabStr = matlabStr + c;    // append characters to string as these are received
      
      if (matlabStr.indexOf(";") != -1) // have we received a semi-colon (indicates end of command from matlab)?
      {
        readyToSend = true;         // then set flag to true since we have received the full command
        digitalWrite(13, HIGH);

        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
        val1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
        demandPositionInDegrees_m1=val1*WGR;
        int posEnd = matlabStr.indexOf(";");                        // position of last character
        val2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // float from substring from comma+1 to end-1
        demandPositionInDegrees_m2=val2*WGR;
      }
    }
    T = 0;
  }
  // Task 2: Compute the current position in degrees and bound it to [-360,360]
  currentPositionInDegrees_m1 = ((counter_m1*360)/(CPR*GR*2));
  if (currentPositionInDegrees_m1 >= 10800.0 || currentPositionInDegrees_m1 <= -10800.0) {
    counter_m1 -= ((GR*CPR*2)*((int)(currentPositionInDegrees_m1/360)));
  }
  currentPositionInDegrees_m2 = ((counter_m2*360)/(CPR*GR*2));
  if (currentPositionInDegrees_m2 >= 10800.0 || currentPositionInDegrees_m2 <= -10800.0) {
    counter_m2 -= ((GR*CPR*2)*((int)(currentPositionInDegrees_m2/360)));
  }
  // Task 3: Compute elapsed time (deltaT) and control the frequency of printing​
  currentTime = micros();
  deltaT = currentTime - previousTime;
  previousTime = currentTime;
  //Serial.println(deltaT);

  if (deltaT > 400) {
    // Task 4: Compute error (P,I,D), and ensure that the previous error is updated
    float errorPositionInDegrees_m1 = demandPositionInDegrees_m1 - currentPositionInDegrees_m1;
    float errorPositionInDegrees_diff_m1 = (errorPositionInDegrees_m1 - errorPositionInDegrees_prev_m1) / deltaT;
    errorPositionInDegrees_sum_m1 += errorPositionInDegrees_m1;
    errorPositionInDegrees_prev_m1 = errorPositionInDegrees_m1;

    float errorPositionInDegrees_m2 = demandPositionInDegrees_m2 - currentPositionInDegrees_m2;
    float errorPositionInDegrees_diff_m2 = (errorPositionInDegrees_m2 - errorPositionInDegrees_prev_m2) / deltaT;
    errorPositionInDegrees_sum_m2 += errorPositionInDegrees_m2;
    errorPositionInDegrees_prev_m2 = errorPositionInDegrees_m2;

    // Task 5: Compute the PID output
    float controllerOutput_m1 = errorPositionInDegrees_m1 * Kp_m1 + errorPositionInDegrees_diff_m1 * Kd_m1 + errorPositionInDegrees_sum_m1 * Ki_m1 * deltaT;

    controllerOutput_m1 = constrain(controllerOutput_m1, -255, 255);

    float controllerOutput_m2 = errorPositionInDegrees_m2 * Kp_m2 + errorPositionInDegrees_diff_m2 * Kd_m2 + errorPositionInDegrees_sum_m2 * Ki_m2 * deltaT;

    controllerOutput_m2 = constrain(controllerOutput_m2, -255, 255);

    // Task 6: Send voltage to motors
    if (controllerOutput_m1 > 0) {
      digitalWrite(motorPin1_m1, HIGH);
      digitalWrite(motorPin2_m1, LOW);
      analogWrite(enablePin_m1, controllerOutput_m1);
    } else {
      digitalWrite(motorPin1_m1, LOW);
      digitalWrite(motorPin2_m1, HIGH);
      analogWrite(enablePin_m1, -controllerOutput_m1);
    }

      if (controllerOutput_m2 > 0) {
      digitalWrite(motorPin1_m2, HIGH);
      digitalWrite(motorPin2_m2, LOW);
      analogWrite(enablePin_m2, controllerOutput_m2);
    } else {
      digitalWrite(motorPin1_m2, LOW);
      digitalWrite(motorPin2_m2, HIGH);
      analogWrite(enablePin_m2, -controllerOutput_m2);
    }
  }

  
  // Then, start sending data from Ardiono to matlab
  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {
    //e.g. c1,100
    Serial.print("c");                    // command
    Serial.print(","); 
    Serial.print(currentPositionInDegrees_m1/30);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(currentPositionInDegrees_m2/30); // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
  }

  if (T>1000)
  {
    Serial.print("d");                    // command
    Serial.print(","); 
    Serial.print(currentPositionInDegrees_m1/30);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(currentPositionInDegrees_m2/30); // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
    digitalWrite(13, LOW);
    readyToSend = false;
    matlabStr = "";
  }
  T = T + 1;
}

// Interrupt function for tracking the encoder positions for motor 1
void updateEncoder_m1() {
  int aState_m1 = digitalRead(encoderPinA_m1);  // Read the current state of encoder pin A
  if (aState_m1 != aLastState_m1) {  // If state changes
      counter_m1 += (digitalRead(encoderPinB_m1) != aState_m1) ? 1 : -1;  // Clockwise rotation
      aLastState_m1 = aState_m1;
  }
}

// Interrupt function for tracking the encoder positions for motor 2
void updateEncoder_m2() {
  int aState_m2 = digitalRead(encoderPinA_m2);  // Read the current state of encoder pin A
  if (aState_m2 != aLastState_m2) {  // If state changes
    if (digitalRead(encoderPinB_m2) != aState_m2) {
      counter_m2++;  // Clockwise rotation
    } else {
      counter_m2--;  // Counter-clockwise rotation
    }
    aLastState_m2 = aState_m2;  // Update last state
  }
}
