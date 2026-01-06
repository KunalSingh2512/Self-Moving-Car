const int LPWM_L = 5;
const int RPWM_L = 6;   
const int LPWM_R = 9;
const int RPWM_R = 10;

const unsigned long forwardTime  = 2300;
const unsigned long backwardTime = 2300;
const int motorSpeed = 200;

void setup() {
  pinMode(LPWM_L, OUTPUT);
  pinMode(RPWM_L, OUTPUT);
  pinMode(LPWM_R, OUTPUT);
  pinMode(RPWM_R, OUTPUT);

  stopMotors();
}

void loop() {
  driveForward(motorSpeed);
  delay(forwardTime);

  stopMotors();
  delay(300);

  driveBackward(motorSpeed);
  delay(backwardTime);

  stopMotors();
  
  while (true) {
    stopMotors();
  }
}

void driveForward(int speed) {
  analogWrite(LPWM_L, speed);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, speed);
  analogWrite(RPWM_R, 0);
}

void driveBackward(int speed) {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, speed);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, speed);
}

void stopMotors() {
  analogWrite(LPWM_L, 0);
  analogWrite(RPWM_L, 0);
  analogWrite(LPWM_R, 0);
  analogWrite(RPWM_R, 0);
}