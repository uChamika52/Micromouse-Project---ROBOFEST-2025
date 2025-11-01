// =================================================================
// --- LIBRARIES ---
// =================================================================
#include <Wire.h>

// =================================================================
// USER CALIBRATION & TUNING
// =================================================================
#define FORWARD_SPEED 150  
#define TURN_SPEED 120     

// --- Encoder Calibration ---
#define TICKS_PER_CELL 347
#define TICKS_PER_90_DEGREE_TURN 145
#define TICKS_PER_180_DEGREE_TURN 380

// --- PID Tuning Constants for Driving Straight ---
double Kp = 1.0;   
double Ki = 0.00;  
double Kd = 0.00;  

// =================================================================
// PIN CONFIGURATION
// =================================================================
#define ENCODER_LEFT_A 32
#define ENCODER_LEFT_B 35
#define ENCODER_RIGHT_A 33
#define ENCODER_RIGHT_B 34
#define MOTOR_LEFT_IN1 18
#define MOTOR_LEFT_IN2 19
#define MOTOR_RIGHT_IN1 23
#define MOTOR_RIGHT_IN2 5

// =================================================================
// PWM CONFIGURATION
// =================================================================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define MOTOR_LEFT_IN1_CHANNEL 0
#define MOTOR_LEFT_IN2_CHANNEL 1
#define MOTOR_RIGHT_IN1_CHANNEL 2
#define MOTOR_RIGHT_IN2_CHANNEL 3

// =================================================================
// GLOBAL OBJECTS & VARIABLES
// =================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// PID control variables
long error = 0, lastError = 0, integral = 0, derivative = 0;

// =================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
// =================================================================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_B) == LOW) { leftEncoderCount++; } else { leftEncoderCount--; }
}
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_B) == LOW) { rightEncoderCount++; } else { rightEncoderCount--; }
}

// =================================================================
// LOW-LEVEL MOTOR CONTROL
// =================================================================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed >= 0) { ledcWrite(MOTOR_LEFT_IN1_CHANNEL, leftSpeed); ledcWrite(MOTOR_LEFT_IN2_CHANNEL, 0); }
  else { ledcWrite(MOTOR_LEFT_IN1_CHANNEL, 0); ledcWrite(MOTOR_LEFT_IN2_CHANNEL, -leftSpeed); }
  if (rightSpeed >= 0) { ledcWrite(MOTOR_RIGHT_IN1_CHANNEL, rightSpeed); ledcWrite(MOTOR_RIGHT_IN2_CHANNEL, 0); }
  else { ledcWrite(MOTOR_RIGHT_IN1_CHANNEL, 0); ledcWrite(MOTOR_RIGHT_IN2_CHANNEL, -rightSpeed); }
}

// =================================================================
// HIGH-LEVEL MOVEMENT FUNCTIONS
// =================================================================

/**
 * @param baseSpeed 
 * @param targetTicks 
 */
void moveForwardPID(int baseSpeed, long targetTicks) {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  error = 0; lastError = 0; integral = 0; derivative = 0;

  while ((leftEncoderCount + rightEncoderCount) / 2 < targetTicks) {
    error = leftEncoderCount - rightEncoderCount;
    integral += error;
    derivative = error - lastError;

    long correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    setMotorSpeed(leftSpeed, rightSpeed);

    lastError = error;
    
    delay(10); 
  }
  
  setMotorSpeed(0, 0); 
}

void moveForwardOneCell() {
  moveForwardPID(FORWARD_SPEED, TICKS_PER_CELL);
}

void turnRight90() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  while (abs(leftEncoderCount) < TICKS_PER_90_DEGREE_TURN || abs(rightEncoderCount) < TICKS_PER_90_DEGREE_TURN) {
    delay(1);
  }
  setMotorSpeed(0, 0);
}

void turnLeft90() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  while (abs(leftEncoderCount) < TICKS_PER_90_DEGREE_TURN || abs(rightEncoderCount) < TICKS_PER_90_DEGREE_TURN) {
    delay(1);
  }
  setMotorSpeed(0, 0);
}

void turnAround180() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  while (abs(leftEncoderCount) < TICKS_PER_180_DEGREE_TURN || abs(rightEncoderCount) < TICKS_PER_180_DEGREE_TURN) {
    delay(1);
  }
  setMotorSpeed(0, 0);
}

// =================================================================
// SETUP AND MAIN LOOP
// =================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Micromouse Motor, Encoder, and PID Straight-Line Test");

  ledcSetup(MOTOR_LEFT_IN1_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_LEFT_IN2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_RIGHT_IN1_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_RIGHT_IN2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_IN1, MOTOR_LEFT_IN1_CHANNEL);
  ledcAttachPin(MOTOR_LEFT_IN2, MOTOR_LEFT_IN2_CHANNEL);
  ledcAttachPin(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN1_CHANNEL);
  ledcAttachPin(MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN2_CHANNEL);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

  Serial.println("\n--- All systems initialized. Starting test sequence in 3 seconds. ---");
  delay(3000);
}

void loop() {
  Serial.println("Action: Moving forward one cell using PID.");
  moveForwardOneCell();
  delay(2000);

  Serial.println("Action: Turning right 90 degrees.");
  turnRight90();
  delay(2000);

  Serial.println("Action: Turning left 90 degrees.");
  turnLeft90();
  delay(2000);

  Serial.println("\n--- Sequence Complete. Restarting. ---\n");
}
