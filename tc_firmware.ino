#include <AccelStepper.h> //[cite: 3]

// --- Pin Definitions ---[cite: 3]
const int STEP_PIN = 40;  //[cite: 3]
const int DIR_PIN = 44;   //[cite: 3]

// Initialize AccelStepper in DRIVER mode[cite: 3]
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //[cite: 3]

// --- Target Velocity ---[cite: 3]
float targetVelocity = 0.0; // steps per second[cite: 3]

// --- Serial Communication Variables ---[cite: 3]
const byte numChars = 32; //[cite: 3]
char receivedChars[numChars];  //[cite: 3]
boolean newData = false; //[cite: 3]
unsigned long lastTelemetryTime = 0; //[cite: 3]

void setup() {
  Serial.begin(115200); //[cite: 3]
  
  stepper.setMaxSpeed(8000.0);  // Raise the ceiling so Python can command higher speeds
  stepper.setSpeed(0.0); //[cite: 3]
}

void loop() { //[cite: 3]
  receiveSerialData(); //[cite: 3]

  if (newData == true) { //[cite: 3]
    parseCommand(); //[cite: 3]
    newData = false; //[cite: 3]
  }

  // Must remain runSpeed() to execute the streaming velocity commands[cite: 3]
  stepper.runSpeed(); //[cite: 3]

  // Send telemetry back at 20Hz (50ms interval)[cite: 3]
  if (millis() - lastTelemetryTime >= 50) { //[cite: 3]
    lastTelemetryTime = millis(); //[cite: 3]
    sendTelemetry(); //[cite: 3]
  } //[cite: 3]
} //[cite: 3]

void receiveSerialData() { //[cite: 3]
  static byte ndx = 0; //[cite: 3]
  char endMarker = '\n'; //[cite: 3]
  char rc; //[cite: 3]
  
  while (Serial.available() > 0 && newData == false) { //[cite: 3]
    rc = Serial.read(); //[cite: 3]
    if (rc != endMarker) { //[cite: 3]
      receivedChars[ndx] = rc; //[cite: 3]
      ndx++; //[cite: 3]
      if (ndx >= numChars) ndx = numChars - 1;  //[cite: 3]
    } else { //[cite: 3]
      receivedChars[ndx] = '\0';  //[cite: 3]
      ndx = 0; //[cite: 3]
      newData = true; //[cite: 3]
    } //[cite: 3]
  } //[cite: 3]
} //[cite: 3]

void parseCommand() { //[cite: 3]
  char commandType = receivedChars[0]; //[cite: 3]
  char *strtokIndx = strtok(receivedChars, " ");  //[cite: 3]
  strtokIndx = strtok(NULL, " ");  //[cite: 3]
  
  if (strtokIndx != NULL) { //[cite: 3]
    float value = atof(strtokIndx);  //[cite: 3]
    if (commandType == 'V') { //[cite: 3]
      targetVelocity = value; //[cite: 3]
      stepper.setSpeed(targetVelocity); //[cite: 3]
    } //[cite: 3]
  } //[cite: 3]
} //[cite: 3]

void sendTelemetry() { //[cite: 3]
  Serial.print(stepper.currentPosition()); //[cite: 3]
  Serial.print(","); //[cite: 3]
  Serial.print(stepper.speed()); //[cite: 3]
  Serial.print(","); //[cite: 3]
  Serial.println(targetVelocity);  //[cite: 3]
} //[cite: 3]