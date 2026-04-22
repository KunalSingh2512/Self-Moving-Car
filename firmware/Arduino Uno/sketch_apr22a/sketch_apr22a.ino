// --- HARDWARE PINOUTS ---
// Encoders (REES52)
const int LEFT_ENC_A = 2;  // Interrupt 0
const int RIGHT_ENC_A = 3; // Interrupt 1
const int LEFT_ENC_B = 4;
const int RIGHT_ENC_B = 5;

// Motors (BTS7960)
const int L_RPWM = 9;  // Left Forward
const int L_LPWM = 10; // Left Reverse
const int R_RPWM = 11; // Right Forward
const int R_LPWM = 6;  // Right Reverse

// Ultrasonic
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// --- VEHICLE PHYSICAL LIMITS ---
const float WHEEL_RADIUS = 0.06; 
const float WHEEL_BASE = 0.36;   
const float MAX_SPEED_MS = 2.5; 

// --- VOLATILE VARIABLES (For high-speed interrupts) ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Timing
unsigned long last_transmit_time = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // Prevent Serial from hanging
  
  // Setup Pins
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Attach Hardware Interrupts (1X Decoding)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  
  // Ensure motors are stopped on boot
  stopMotors();
}

void loop() {
  // 1. LISTEN TO RASPBERRY PI
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    if (incoming.startsWith("SPEED:")) {
      parseAndExecuteSpeed(incoming);
    }
  }

  // 2. SEND DATA TO RASPBERRY PI (Every 50ms / 20Hz)
  if (millis() - last_transmit_time >= 50) {
    // Safely grab ticks (disable interrupts for a microsecond so values don't change while reading)
    noInterrupts();
    long current_left_ticks = left_ticks;
    long current_right_ticks = right_ticks;
    interrupts();
    
    // Read Sonar (with a timeout so it doesn't freeze the Uno if there's no echo)
    float sonar_dist = readUltrasonic();
    
    // Format: "DATA:left,right,sonar"
    Serial.print("DATA:");
    Serial.print(current_left_ticks);
    Serial.print(",");
    Serial.print(current_right_ticks);
    Serial.print(",");
    Serial.println(sonar_dist);
    
    last_transmit_time = millis();
  }
}

// ==========================================
// INTERRUPT SERVICE ROUTINES (High Speed)
// ==========================================
void leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    left_ticks++;  // Forward
  } else {
    left_ticks--;  // Reverse
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    right_ticks++; // Forward
  } else {
    right_ticks--; // Reverse
  }
}

// ==========================================
// KINEMATICS & MOTOR CONTROL
// ==========================================
void parseAndExecuteSpeed(String data) {
  // Extract "v,w" from "SPEED:v,w"
  data.remove(0, 6); 
  int commaIndex = data.indexOf(',');
  if (commaIndex == -1) return;
  
  float v_linear = data.substring(0, commaIndex).toFloat();
  float w_angular = data.substring(commaIndex + 1).toFloat();
  
  // Differential Drive Kinematics Equations
  float v_left = v_linear - (w_angular * WHEEL_BASE / 2.0);
  float v_right = v_linear + (w_angular * WHEEL_BASE / 2.0);
  
  // Convert target speed (m/s) to PWM (0-255)
  int pwm_left = mapSpeedToPWM(v_left);
  int pwm_right = mapSpeedToPWM(v_right);
  
  // Apply to motors
  setLeftMotor(pwm_left);
  setRightMotor(pwm_right);
}

int mapSpeedToPWM(float target_speed) {
  // Constrain speed to absolute max
  if (target_speed > MAX_SPEED_MS) target_speed = MAX_SPEED_MS;
  if (target_speed < -MAX_SPEED_MS) target_speed = -MAX_SPEED_MS;
  
  // Map m/s to 0-255 PWM (Simple linear mapping, can be tuned later)
  int pwm = (int)((abs(target_speed) / MAX_SPEED_MS) * 255.0);
  
  // Preserve direction (Negative PWM means reverse)
  if (target_speed < 0) return -pwm;
  return pwm;
}

void setLeftMotor(int pwm) {
  if (pwm > 0) {
    analogWrite(L_RPWM, pwm);
    analogWrite(L_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, abs(pwm));
  } else {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, 0);
  }
}

void setRightMotor(int pwm) {
  if (pwm > 0) {
    analogWrite(R_RPWM, pwm);
    analogWrite(R_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, abs(pwm));
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, 0);
  }
}

void stopMotors() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);
}

// ==========================================
// ULTRASONIC SENSOR
// ==========================================
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Timeout of 24000 microseconds is roughly 4 meters max distance
  // Prevents the Uno from freezing if no object is detected
  long duration = pulseIn(ECHO_PIN, HIGH, 24000); 
  
  if (duration == 0) return 4.0; // Return 4 meters if no echo (clear path)
  
  // Speed of sound is 343 m/s -> Distance = (time * 0.0343) / 2
  float distance_cm = (duration * 0.0343) / 2.0;
  return distance_cm / 100.0; // Return in meters for ROS2
}
