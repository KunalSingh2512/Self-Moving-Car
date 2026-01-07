const int LPWM_L = 5;
const int RPWM_L = 6;
const int LPWM_R = 9;
const int RPWM_R = 10;

const unsigned long travelTime = 2000;
const int motorSpeed = 220;           
const int pulseCount = 6;             

void setup() {
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(RPWM_R, OUTPUT);
  
  stopMotors();
}

void loop() {
  driveForward(motorSpeed);
  delay(travelTime);

  executePulsingUTurn();

  driveForward(motorSpeed); 
  delay(travelTime);

  executePulsingUTurn();

  stopMotors();
  while (true) {
    stopMotors();
  }
}


void executePulsingUTurn() {
  for (int i = 0; i < pulseCount; i++) {
    analogWrite(LPWM_L, 0);
    analogWrite(RPWM_L, motorSpeed); 
    analogWrite(LPWM_R, motorSpeed); 
    analogWrite(RPWM_R, 0);
    delay(450); 

    driveForward(motorSpeed); 
    delay(250); 
  }
}

void driveForward(int speed) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
}

void stopMotors() {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, 0);
}