#include <AccelStepper.h>

// --- Pin Definitions ---
const int STEP_PIN = 5; 
const int DIR_PIN = 4;  

// Initialize AccelStepper in DRIVER mode
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Target Velocity ---
float targetVelocity = 0.0; // steps per second

// --- Serial Communication Variables ---
const byte numChars = 32;
char receivedChars[numChars]; 
boolean newData = false;
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200); //[cite: 1]
  
  stepper.setMaxSpeed(2000.0);
  stepper.setSpeed(0.0);
}

void loop() {
  receiveSerialData();

  if (newData == true) {
    parseCommand();
    newData = false;
  }

  stepper.runSpeed();

  // Send telemetry back at 20Hz (50ms interval)
  if (millis() - lastTelemetryTime >= 50) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}

void receiveSerialData() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) ndx = numChars - 1; 
    } else {
      receivedChars[ndx] = '\0'; 
      ndx = 0;
      newData = true;
    }
  }
}

void parseCommand() {
  char commandType = receivedChars[0];
  char *strtokIndx = strtok(receivedChars, " "); 
  strtokIndx = strtok(NULL, " "); 
  
  if (strtokIndx != NULL) {
    float value = atof(strtokIndx); 
    if (commandType == 'V') {
      targetVelocity = value;
      stepper.setSpeed(targetVelocity);
    }
  }
}

void sendTelemetry() {
  Serial.print(stepper.currentPosition());
  Serial.print(",");
  Serial.print(stepper.speed());
  Serial.print(",");
  Serial.println(targetVelocity); 
}