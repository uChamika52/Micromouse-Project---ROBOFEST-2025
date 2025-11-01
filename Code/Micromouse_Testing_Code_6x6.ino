// =================================================================
// --- LIBRARIES ---
// =================================================================
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <queue>

// =================================================================
// USER CALIBRATION & TUNING
// =================================================================
#define FORWARD_SPEED 220
#define TURN_SPEED 200
#define RETURN_SPEED 255 

// --- MOVEMENT CONSTANTS ---
#define TICKS_PER_CELL 290
#define TICKS_FOR_90_TURN_RIGHT 139
#define TICKS_FOR_90_TURN_LEFT 143
#define TICKS_FOR_180_TURN 384

// --- SENSOR THRESHOLDS ---
#define WALL_THRESHOLD_FRONT 80
#define WALL_THRESHOLD_SIDE 95

// --- PID CONSTANTS ---
double Kp_Encoder = 1.15;
double Kp_Wall = 0.15;
double Ki = 0.00;
double Kd = 0.01;

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
#define XSHUT_PIN_FRONT 16
#define SENSOR_FRONT_ADDRESS 0x30
#define XSHUT_PIN_RIGHT 17
#define SENSOR_RIGHT_ADDRESS 0x31
#define XSHUT_PIN_LEFT 4
#define SENSOR_LEFT_ADDRESS 0x32
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define MOTOR_LEFT_IN1_CHANNEL 0
#define MOTOR_LEFT_IN2_CHANNEL 1
#define MOTOR_RIGHT_IN1_CHANNEL 2
#define MOTOR_RIGHT_IN2_CHANNEL 3

// =================================================================
// GLOBAL OBJECTS & ALGORITHM VARIABLES
// =================================================================
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long error = 0, lastError = 0, integral = 0, derivative = 0;
Adafruit_VL53L0X sensors[3];

enum RobotState {
  EXPLORING,
  RETURN_TO_START,
  FINISHED
};
RobotState currentState = EXPLORING;

#define FORWARD   0
#define RIGHT     1
#define BACKWARD  2
#define LEFT      3

int current_x = 0, current_y = 0, previous_x = 0, previous_y = 0;
int orient = FORWARD;

int cellsArray[6][6] = {0};
bool visited[6][6] = {false};

int floodArray[6][6] = {
    {4, 3, 2, 2, 3, 4},
    {3, 2, 1, 1, 2, 3},
    {2, 1, 0, 0, 1, 2},
    {2, 1, 0, 0, 1, 2},
    {3, 2, 1, 1, 2, 3},
    {4, 3, 2, 2, 3, 4}
};

int returnFloodArray[6][6];


// =================================================================
// --- ALGORITHM CORE ---
// =================================================================
void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
  if (left && right && forward) { if (orient == FORWARD) cellsArray[x][y] = 13; else if (orient == RIGHT) cellsArray[x][y] = 12; else if (orient == BACKWARD) cellsArray[x][y] = 11; else if (orient == LEFT) cellsArray[x][y] = 14; }
  else if (left && right && !forward) { if (orient == FORWARD || orient == BACKWARD) cellsArray[x][y] = 9; else if (orient == RIGHT || orient == LEFT) cellsArray[x][y] = 10; }
  else if (left && !right && forward) { if (orient == FORWARD) cellsArray[x][y] = 8; else if (orient == RIGHT) cellsArray[x][y] = 7; else if (orient == BACKWARD) cellsArray[x][y] = 6; else if (orient == LEFT) cellsArray[x][y] = 5; }
  else if (!left && right && forward) { if (orient == FORWARD) cellsArray[x][y] = 7; else if (orient == RIGHT) cellsArray[x][y] = 6; else if (orient == BACKWARD) cellsArray[x][y] = 5; else if (orient == LEFT) cellsArray[x][y] = 8; }
  else if (forward) { if (orient == FORWARD) cellsArray[x][y] = 2; else if (orient == RIGHT) cellsArray[x][y] = 3; else if (orient == BACKWARD) cellsArray[x][y] = 4; else if (orient == LEFT) cellsArray[x][y] = 1; }
  else if (left) { if (orient == FORWARD) cellsArray[x][y] = 1; else if (orient == RIGHT) cellsArray[x][y] = 2; else if (orient == BACKWARD) cellsArray[x][y] = 3; else if (orient == LEFT) cellsArray[x][y] = 4; }
  else if (right) { if (orient == FORWARD) cellsArray[x][y] = 3; else if (orient == RIGHT) cellsArray[x][y] = 4; else if (orient == BACKWARD) cellsArray[x][y] = 1; else if (orient == LEFT) cellsArray[x][y] = 2; }
}

bool isAccessible(int current_x, int current_y, int target_x, int target_y) {
    // REVISED FOR 6x6: Updated boundary check
    if (target_x < 0 || target_x >= 6 || target_y < 0 || target_y >= 6) return false;
    if (current_x == target_x) { if (current_y > target_y) { if ((cellsArray[current_x][current_y] == 4) || (cellsArray[current_x][current_y] == 5) || (cellsArray[current_x][current_y] == 6) || (cellsArray[current_x][current_y] == 10) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } else { if ((cellsArray[current_x][current_y] == 2) || (cellsArray[current_x][current_y] == 7) || (cellsArray[current_x][current_y] == 8) || (cellsArray[current_x][current_y] == 10) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 13) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } }
    else if (current_y == target_y) { if (current_x > target_x) { if ((cellsArray[current_x][current_y] == 1) || (cellsArray[current_x][current_y] == 5) || (cellsArray[current_x][current_y] == 8) || (cellsArray[current_x][current_y] == 9) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 13) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } else { if ((cellsArray[current_x][current_y] == 3) || (cellsArray[current_x][current_y] == 6) || (cellsArray[current_x][current_y] == 7) || (cellsArray[current_x][current_y] == 9) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 13)) return false; else return true; } }
    return false;
}

void getSurroungings(int current_x, int current_y, int *north_x, int *north_y, int *east_x, int *east_y, int *south_x, int *south_y, int *west_x, int *west_y) {
  // REVISED FOR 6x6: Updated boundary check
  *north_x = current_x; *north_y = (current_y + 1) >= 6 ? -1 : current_y + 1;
  *east_x = (current_x + 1) >= 6 ? -1 : current_x + 1; *east_y = current_y;
  *south_x = current_x; *south_y = (current_y - 1) < 0 ? -1 : current_y - 1;
  *west_x = (current_x - 1) < 0 ? -1 : current_x - 1; *west_y = current_y;
}

bool isIncrementConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int cVal = floodArray[current_x][current_y]; int minCnt = 0;
    if ((nX != -1) && isAccessible(current_x, current_y, nX, nY) && (floodArray[nX][nY] == cVal - 1)) minCnt++;
    if ((eX != -1) && isAccessible(current_x, current_y, eX, eY) && (floodArray[eX][eY] == cVal - 1)) minCnt++;
    if ((sX != -1) && isAccessible(current_x, current_y, sX, sY) && (floodArray[sX][sY] == cVal - 1)) minCnt++;
    if ((wX != -1) && isAccessible(current_x, current_y, wX, wY) && (floodArray[wX][wY] == cVal - 1)) minCnt++;
    return minCnt > 0;
}

void makeCellConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int minVal = 1000;
    if ((nX != -1) && isAccessible(current_x, current_y, nX, nY)) minVal = min(minVal, floodArray[nX][nY]);
    if ((eX != -1) && isAccessible(current_x, current_y, eX, eY)) minVal = min(minVal, floodArray[eX][eY]);
    if ((sX != -1) && isAccessible(current_x, current_y, sX, sY)) minVal = min(minVal, floodArray[sX][sY]);
    if ((wX != -1) && isAccessible(current_x, current_y, wX, wY)) minVal = min(minVal, floodArray[wX][wY]);
    if (minVal != 1000) floodArray[current_x][current_y] = minVal + 1;
}

void floodFillUsingQueue(int start_x, int start_y, int previous_x, int previous_y) {
    memset(visited, false, sizeof(visited)); std::queue<int> cellQueue;
    if(!isIncrementConsistent(start_x, start_y)) makeCellConsistent(start_x, start_y);
    cellQueue.push(start_x); cellQueue.push(start_y); visited[start_y][start_x] = true;
    while (!cellQueue.empty()) {
        int cX = cellQueue.front(); cellQueue.pop(); int cY = cellQueue.front(); cellQueue.pop();
        if (!isIncrementConsistent(cX, cY)) {
            makeCellConsistent(cX, cY);
            int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(cX, cY, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
            int neighborsX[] = {nX, eX, sX, wX}; int neighborsY[] = {nY, eY, sY, wY};
            for (int i = 0; i < 4; i++) {
                int nbrX = neighborsX[i], nbrY = neighborsY[i];
                if (nbrX != -1 && isAccessible(cX, cY, nbrX, nbrY) && !visited[nbrY][nbrX]) {
                    cellQueue.push(nbrX); cellQueue.push(nbrY); visited[nbrY][nbrX] = true;
                }
            }
        }
    }
}

char whereToMove(int current_x, int current_y, int orient, int floodMap[6][6]) {
    int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int values[4] = {1000, 1000, 1000, 1000};
    if (isAccessible(current_x, current_y, nX, nY)) values[FORWARD] = floodMap[nX][nY];
    if (isAccessible(current_x, current_y, eX, eY)) values[RIGHT] = floodMap[eX][eY];
    if (isAccessible(current_x, current_y, sX, sY)) values[BACKWARD] = floodMap[sX][sY];
    if (isAccessible(current_x, current_y, wX, wY)) values[LEFT] = floodMap[wX][wY];
    int minValue = 1000;
    for (int i=0; i<4; i++) { if (values[i] < minValue) { minValue = values[i]; } }
    int f_dir = orient; int l_dir = (orient+3)%4; int r_dir = (orient+1)%4; int b_dir = (orient+2)%4;
    if (values[f_dir] == minValue) return 'F'; if (values[l_dir] == minValue) return 'L';
    if (values[r_dir] == minValue) return 'R'; if (values[b_dir] == minValue) return 'B';
    return 'B';
}

int orientation(int orient, char turning) {
    if (turning == 'L') orient = (orient + 3) % 4; else if (turning == 'R') orient = (orient + 1) % 4;
    else if (turning == 'B') orient = (orient + 2) % 4; return orient;
}

void updateCoordinates(int orient, int *new_x, int *new_y) {
    if(orient == FORWARD) (*new_y)++; else if(orient == RIGHT) (*new_x)++;
    else if(orient == BACKWARD) (*new_y)--; else if(orient == LEFT) (*new_x)--;
}

// =================================================================
// --- HARDWARE ABSTRACTION & MOVEMENT ---
// =================================================================
void IRAM_ATTR leftEncoderISR() { leftEncoderCount++; }
void IRAM_ATTR rightEncoderISR() { rightEncoderCount++; }

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);
  if (leftSpeed >= 0) { ledcWrite(MOTOR_LEFT_IN1_CHANNEL, leftSpeed); ledcWrite(MOTOR_LEFT_IN2_CHANNEL, 0); }
  else { ledcWrite(MOTOR_LEFT_IN1_CHANNEL, 0); ledcWrite(MOTOR_LEFT_IN2_CHANNEL, -leftSpeed); }
  if (rightSpeed >= 0) { ledcWrite(MOTOR_RIGHT_IN1_CHANNEL, rightSpeed); ledcWrite(MOTOR_RIGHT_IN2_CHANNEL, 0); }
  else { ledcWrite(MOTOR_RIGHT_IN1_CHANNEL, 0); ledcWrite(MOTOR_RIGHT_IN2_CHANNEL, -rightSpeed); }
}

void initSensor(int sensorIndex, int shutPin, uint8_t newAddress) {
  digitalWrite(shutPin, HIGH); delay(50);
  if (!sensors[sensorIndex].begin(0x29)) { Serial.print(F("Failed to boot sensor index ")); Serial.println(sensorIndex); while(1); }
  sensors[sensorIndex].setAddress(newAddress);
}

bool wallFront() { VL53L0X_RangingMeasurementData_t measure; sensors[0].rangingTest(&measure, false); return (measure.RangeStatus != 4 && measure.RangeMilliMeter < WALL_THRESHOLD_FRONT); }
bool wallRight() { VL53L0X_RangingMeasurementData_t measure; sensors[1].rangingTest(&measure, false); return (measure.RangeStatus != 4 && measure.RangeMilliMeter < WALL_THRESHOLD_SIDE); }
bool wallLeft() { VL53L0X_RangingMeasurementData_t measure; sensors[2].rangingTest(&measure, false); return (measure.RangeStatus != 4 && measure.RangeMilliMeter < WALL_THRESHOLD_SIDE); }

void moveForwardPID(int baseSpeed, long targetTicks) {
  leftEncoderCount = 0; rightEncoderCount = 0; error = 0; lastError = 0; integral = 0;
  setMotorSpeed(baseSpeed, baseSpeed);
  while ((abs(leftEncoderCount) + abs(rightEncoderCount)) / 2 < targetTicks) {
    error = leftEncoderCount - rightEncoderCount;
    integral += error; derivative = error - lastError;
    long encoder_correction = (Kp_Encoder * error) + (Ki * integral) + (Kd * derivative);
    long steering_correction = 0;
    VL53L0X_RangingMeasurementData_t measure;
    sensors[1].rangingTest(&measure, false);
    uint16_t rightDist = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;
    sensors[2].rangingTest(&measure, false);
    uint16_t leftDist = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : 999;
    if (leftDist < 100 && rightDist < 100) {
      int steering_error = rightDist - leftDist;
      steering_correction = Kp_Wall * steering_error;
    }
    int leftSpeed = baseSpeed - encoder_correction - steering_correction;
    int rightSpeed = baseSpeed + encoder_correction + steering_correction;
    setMotorSpeed(leftSpeed, rightSpeed);
    lastError = error; delay(10);
  }
  setMotorSpeed(0, 0);
}

void moveForwardOneCell(int speed) { moveForwardPID(speed, TICKS_PER_CELL); }
void turnRight90() { leftEncoderCount = 0; rightEncoderCount = 0; setMotorSpeed(TURN_SPEED, -TURN_SPEED); while (leftEncoderCount < TICKS_FOR_90_TURN_RIGHT) {} setMotorSpeed(0, 0); }
void turnLeft90() { leftEncoderCount = 0; rightEncoderCount = 0; setMotorSpeed(-TURN_SPEED, TURN_SPEED); while (rightEncoderCount < TICKS_FOR_90_TURN_LEFT) {} setMotorSpeed(0, 0); }
void turnAround180() { leftEncoderCount = 0; rightEncoderCount = 0; setMotorSpeed(TURN_SPEED, -TURN_SPEED); while (leftEncoderCount < TICKS_FOR_180_TURN) {} setMotorSpeed(0, 0); }

// =================================================================
// --- SETUP ---
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Micromouse 6x6 - Flood Fill w/ Return Run");
  Wire.begin();

  for (int x = 0; x < 6; x++) {
    for (int y = 0; y < 6; y++) {
      returnFloodArray[x][y] = x + y;
    }
  }

  ledcSetup(MOTOR_LEFT_IN1_CHANNEL, PWM_FREQ, PWM_RESOLUTION); ledcSetup(MOTOR_LEFT_IN2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MOTOR_RIGHT_IN1_CHANNEL, PWM_FREQ, PWM_RESOLUTION); ledcSetup(MOTOR_RIGHT_IN2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_IN1, MOTOR_LEFT_IN1_CHANNEL); ledcAttachPin(MOTOR_LEFT_IN2, MOTOR_LEFT_IN2_CHANNEL);
  ledcAttachPin(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN1_CHANNEL); ledcAttachPin(MOTOR_RIGHT_IN2, MOTOR_RIGHT_IN2_CHANNEL);
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP); pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING); attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);
  delay(100);
  Serial.println("Initializing VL53L0X sensors...");
  pinMode(XSHUT_PIN_FRONT, OUTPUT); pinMode(XSHUT_PIN_RIGHT, OUTPUT); pinMode(XSHUT_PIN_LEFT, OUTPUT);
  digitalWrite(XSHUT_PIN_FRONT, LOW); digitalWrite(XSHUT_PIN_RIGHT, LOW); digitalWrite(XSHUT_PIN_LEFT, LOW);
  delay(50);
  initSensor(0, XSHUT_PIN_FRONT, SENSOR_FRONT_ADDRESS); initSensor(1, XSHUT_PIN_RIGHT, SENSOR_RIGHT_ADDRESS); initSensor(2, XSHUT_PIN_LEFT, SENSOR_LEFT_ADDRESS);
  Serial.println("\n--- All systems initialized. Starting logic in 3 seconds. ---");
  delay(5000);
}

// =================================================================
// --- MAIN LOOP ---
// =================================================================
void loop() {
  
  switch (currentState) {
    
    case EXPLORING: {
      bool hasWallLeft = wallLeft(); bool hasWallRight = wallRight(); bool hasWallFront = wallFront();
      Serial.print("[EXPLORING] Walls: L:"); Serial.print(hasWallLeft); Serial.print(" F:"); Serial.print(hasWallFront); Serial.print(" R:"); Serial.println(hasWallRight);
      Serial.print("Position: ("); Serial.print(current_x); Serial.print(","); Serial.print(current_y); Serial.print(") | Orient: "); Serial.println(orient);
      updateCells(current_x, current_y, orient, hasWallLeft, hasWallRight, hasWallFront);
      if (floodArray[current_x][current_y] != 0) {
        floodFillUsingQueue(current_x, current_y, previous_x, previous_y);
      } else {
        Serial.println(">>> Goal Reached! Mapping complete. Starting return journey. <<<");
        turnAround180();
        delay(200);
        moveForwardOneCell(RETURN_SPEED);
        orient = orientation(orient, 'B');
        currentState = RETURN_TO_START;
        delay(200);
        return;
      }
      char direction = whereToMove(current_x, current_y, orient, floodArray);
      Serial.print("Decision: "); Serial.println(direction);
      if (direction == 'L') { turnLeft90(); orient = orientation(orient, 'L'); delay(300); }
      else if (direction == 'R') { turnRight90(); orient = orientation(orient, 'R'); delay(300); }
      else if (direction == 'B') { turnAround180(); orient = orientation(orient, 'B'); delay(600); }
      moveForwardOneCell(FORWARD_SPEED);
      previous_x = current_x; previous_y = current_y;
      updateCoordinates(orient, &current_x, &current_y);
      Serial.println("---------------------------------");
      delay(300);
      break;
    }

    case RETURN_TO_START: {
      Serial.print("[RETURN] Position: ("); Serial.print(current_x); Serial.print(","); Serial.print(current_y); Serial.print(") | Orient: "); Serial.println(orient);
      if (current_x == 0 && current_y == 0) {
        Serial.println(">>> Returned to Start! Mission Complete. <<<");
        currentState = FINISHED;
        setMotorSpeed(0, 0);
        return;
      }
      char direction = whereToMove(current_x, current_y, orient, returnFloodArray);
      Serial.print("Decision: "); Serial.println(direction);
      if (direction == 'B') { turnAround180(); orient = orientation(orient, 'B'); delay(600); }
      else if (direction == 'L') { turnLeft90(); orient = orientation(orient, 'L'); delay(300); }
      else if (direction == 'R') { turnRight90(); orient = orientation(orient, 'R'); delay(300); }
      moveForwardOneCell(RETURN_SPEED);
      previous_x = current_x; previous_y = current_y;
      updateCoordinates(orient, &current_x, &current_y);
      Serial.println("---------------------------------");
      delay(100);
      break;
    }

    case FINISHED: {
      setMotorSpeed(0, 0);
      while(true);
      break;
    }
  }
}
