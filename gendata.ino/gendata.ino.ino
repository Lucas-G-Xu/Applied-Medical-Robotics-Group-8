// Communicaiton between Arduino and MATLAB
//   @author         Alejandro Granados
//   @organisation   King's College London
//   @module         Medical Robotics Hardware Development
//   @year           2023

// Global variables
int i = 0;                // counter
String matlabStr = "";    // receives the string from matlab, it is empty at first
bool readyToSend = false; // flag to indicate a command was received and now ready to send back to matlab

char c;                   // characters received from matlab
float val1 = 0.0;         // input1 from matlab
float val2 = 0.0;         // input2 from matlab


/* Initialisation function of Arduino */
void setup() {
  // configure serial communication speed
  Serial.begin(9600);
}

/* Continuous loop function in Arduino */
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

        // parse incomming data, e.g. C40.0,3.5;
        int posComma1 = matlabStr.indexOf(",");                     // position of comma in string
        val1 = matlabStr.substring(1, posComma1).toFloat();         // float from substring from character 1 to comma position
        int posEnd = matlabStr.indexOf(";");                        // position of last character
        val2 = matlabStr.substring(posComma1+1, posEnd).toFloat();  // float from substring from comma+1 to end-1
      }
    }
  }

  // Then, start sending data from Ardiono to matlab
  if (readyToSend)  // arduino has received command form matlab and now is ready to send
  {
    // e.g. c1,100
    Serial.print("c");                    // command
    Serial.print(i);                      // series 1
    Serial.print(",");                    // delimiter
    Serial.print(val2*sin(i*val1/360.0)); // series 2
    Serial.write(13);                     // carriage return (CR)
    Serial.write(10);                     // new line (NL)
    i += 1;
  }
}
