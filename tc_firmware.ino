// --- Pin Definitions ---
const int ENCODER_A_PIN = 2; // Hardware interrupt pin
const int ENCODER_B_PIN = 3;
const int MOTOR_PWM_PIN = 9; // PWM pin
const int MOTOR_IN1_PIN = 4;
const int MOTOR_IN2_PIN = 5;

// --- PID Parameters ---
float Kp = 1.5;
float Ki = 5.0;
float Kd = 0.05;

// --- Target Velocity ---
float targetVelocity = 0.0; // Default to 0 (stopped) on boot

// --- Shared Volatile Variables (ISR to Main Loop) ---
volatile long encoderTicks = 0;
volatile float currentVelocity = 0.0; 

// --- PID State Variables ---
long previousTicks = 0;
float integralError = 0;
float previousError = 0;

// --- Serial Communication Variables ---
const byte numChars = 32;
char receivedChars[numChars]; // Buffer for incoming data
boolean newData = false;
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200); // Ensure your laptop code uses this baud rate

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);

  // Setup Encoder Interrupt
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), countEncoder, RISING);

  // Setup Timer1 Interrupt (100Hz / 10ms interval)
  noInterrupts();           
  TCCR1A = 0;               
  TCCR1B = 0;
  TCNT1  = 0;               
  OCR1A = 2499;            
  TCCR1B |= (1 << WGM12);   
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);  
  interrupts();             
}

void loop() {
  // 1. Check for incoming commands from the laptop
  receiveSerialData();

  // 2. If a full command package arrived, parse and apply it
  if (newData == true) {
    parseCommand();
    newData = false;
  }

  // 3. Send telemetry back to the laptop every 50ms (20Hz)
  // We use millis() instead of delay() to keep the loop non-blocking
  if (millis() - lastTelemetryTime >= 50) {
    lastTelemetryTime = millis();
    sendTelemetry();
  }
}

// --- Serial Receive (Non-Blocking) ---
void receiveSerialData() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1; // Prevent buffer overflow
      }
    } else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

// --- Parse Laptop Commands ---
void parseCommand() {
  // We expect a format like: "V 500.0" or "P 1.5"
  char commandType = receivedChars[0];
  
  // Convert the rest of the string to a float
  // strtok gets the string after the space
  char *strtokIndx = strtok(receivedChars, " "); 
  strtokIndx = strtok(NULL, " "); // Get the second part (the number)
  
  if (strtokIndx != NULL) {
    float value = atof(strtokIndx); 
    
    // Disable interrupts briefly while updating critical shared variables
    noInterrupts();
    switch (commandType) {
      case 'V': targetVelocity = value; break;
      case 'P': Kp = value; break;
      case 'I': Ki = value; break;
      case 'D': Kd = value; break;
    }
    interrupts();
  }
}

// --- Send Telemetry to Laptop ---
void sendTelemetry() {
  // Safely grab the volatile variables
  noInterrupts();
  float safeVelocity = currentVelocity;
  long safeTicks = encoderTicks;
  interrupts();

  // Send a simple comma-separated string back to the laptop
  // Format: Ticks,CurrentVelocity,TargetVelocity
  Serial.print(safeTicks);
  Serial.print(",");
  Serial.print(safeVelocity);
  Serial.print(",");
  Serial.println(targetVelocity); 
}

// --- External ISR: Encoder Counting ---
void countEncoder() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

// --- Timer ISR: PID Control Loop (Runs 100 times per sec) ---
ISR(TIMER1_COMPA_vect) {
  float dt = 0.01; 

  long currentTicks = encoderTicks; 
  currentVelocity = ((float)(currentTicks - previousTicks)) / dt;
  previousTicks = currentTicks;

  float error = targetVelocity - currentVelocity;
  integralError += error * dt;
  float derivativeError = (error - previousError) / dt;
  
  float controlSignal = (Kp * error) + (Ki * integralError) + (Kd * derivativeError);
  previousError = error;

  driveMotor(controlSignal);
}

// --- Motor Control Function ---
void driveMotor(float controlSignal) {
  int direction = 1;
  if (controlSignal < 0) {
    direction = -1;
    controlSignal = -controlSignal; 
  }

  int pwmValue = (int)controlSignal;
  if (pwmValue > 255) pwmValue = 255;
  if (pwmValue < 0) pwmValue = 0;

  if (direction == 1) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }

  analogWrite(MOTOR_PWM_PIN, pwmValue);
}
