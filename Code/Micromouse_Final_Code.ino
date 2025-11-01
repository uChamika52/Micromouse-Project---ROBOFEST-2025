// =================================================================
// --- LIBRARIES ---
// =================================================================
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <queue>

// =================================================================
// USER CALIBRATION & TUNING
// =================================================================
#define FORWARD_SPEED 200
#define TURN_SPEED 200
#define RETURN_SPEED 220

// --- MOVEMENT CONSTANTS ---
#define TICKS_PER_CELL 315
#define TICKS_FOR_90_TURN_RIGHT 157  
#define TICKS_FOR_90_TURN_LEFT 145
#define TICKS_FOR_180_TURN 385

// --- SENSOR THRESHOLDS ---
#define WALL_THRESHOLD_FRONT 80
#define WALL_THRESHOLD_SIDE 95

// --- PID CONSTANTS ---
double Kp_Encoder = 1.9;
double Kp_Wall = 0.2;
double Ki = 0.00;
double Kd = 0.01;

bool hasStartedReturnRun = false; 

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

int cellsArray[16][16] = {0};
bool visited[16][16] = {false};

int floodArray[16][16] = {
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10,  9, 8, 7, 6, 6, 7, 8,  9, 10, 11, 12, 13},
    {12, 11, 10,  9,  8, 7, 6, 5, 5, 6, 7,  8,  9, 10, 11, 12},
    {11, 10,  9,  8,  7, 6, 5, 4, 4, 5, 6,  7,  8,  9, 10, 11},
    {10,  9,  8,  7,  6, 5, 4, 3, 3, 4, 5,  6,  7,  8,  9, 10},
    { 9,  8,  7,  6,  5, 4, 3, 2, 2, 3, 4,  5,  6,  7,  8,  9},
    { 8,  7,  6,  5,  4, 3, 2, 1, 1, 2, 3,  4,  5,  6,  7,  8},
    { 7,  6,  5,  4,  3, 2, 1, 0, 0, 1, 2,  3,  4,  5,  6,  7},
    { 7,  6,  5,  4,  3, 2, 1, 0, 0, 1, 2,  3,  4,  5,  6,  7},
    { 8,  7,  6,  5,  4, 3, 2, 1, 1, 2, 3,  4,  5,  6,  7,  8},
    { 9,  8,  7,  6,  5, 4, 3, 2, 2, 3, 4,  5,  6,  7,  8,  9},
    {10,  9,  8,  7,  6, 5, 4, 3, 3, 4, 5,  6,  7,  8,  9, 10},
    {11, 10,  9,  8,  7, 6, 5, 4, 4, 5, 6,  7,  8,  9, 10, 11},
    {12, 11, 10,  9,  8, 7, 6, 5, 5, 6, 7,  8,  9, 10, 11, 12},
    {13, 12, 11, 10,  9, 8, 7, 6, 6, 7, 8,  9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
};

int returnFloodArray[16][16];

struct PathStep {
  int x;
  int y;
  int orientation;
};
PathStep explorationPath[300]; 
int pathLength = 0;
int returnStepIndex = 0;


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
    if (target_x < 0 || target_x >= 16 || target_y < 0 || target_y >= 16) return false;
    if (current_x == target_x) { if (current_y > target_y) { if ((cellsArray[current_x][current_y] == 4) || (cellsArray[current_x][current_y] == 5) || (cellsArray[current_x][current_y] == 6) || (cellsArray[current_x][current_y] == 10) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } else { if ((cellsArray[current_x][current_y] == 2) || (cellsArray[current_x][current_y] == 7) || (cellsArray[current_x][current_y] == 8) || (cellsArray[current_x][current_y] == 10) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 13) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } }
    else if (current_y == target_y) { if (current_x > target_x) { if ((cellsArray[current_x][current_y] == 1) || (cellsArray[current_x][current_y] == 5) || (cellsArray[current_x][current_y] == 8) || (cellsArray[current_x][current_y] == 9) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 13) || (cellsArray[current_x][current_y] == 14)) return false; else return true; } else { if ((cellsArray[current_x][current_y] == 3) || (cellsArray[current_x][current_y] == 6) || (cellsArray[current_x][current_y] == 7) || (cellsArray[current_x][current_y] == 9) || (cellsArray[current_x][current_y] == 11) || (cellsArray[current_x][current_y] == 12) || (cellsArray[current_x][current_y] == 13)) return false; else return true; } }
    return false;
}

void getSurroungings(int current_x, int current_y, int *north_x, int *north_y, int *east_x, int *east_y, int *south_x, int *south_y, int *west_x, int *west_y) {
  *north_x = current_x; *north_y = (current_y + 1) >= 16 ? -1 : current_y + 1;
  *east_x = (current_x + 1) >= 16 ? -1 : current_x + 1; *east_y = current_y;
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

bool isReturnIncrementConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int cVal = returnFloodArray[current_x][current_y]; int minCnt = 0;
    if ((nX != -1) && isAccessible(current_x, current_y, nX, nY) && (returnFloodArray[nX][nY] == cVal - 1)) minCnt++;
    if ((eX != -1) && isAccessible(current_x, current_y, eX, eY) && (returnFloodArray[eX][eY] == cVal - 1)) minCnt++;
    if ((sX != -1) && isAccessible(current_x, current_y, sX, sY) && (returnFloodArray[sX][sY] == cVal - 1)) minCnt++;
    if ((wX != -1) && isAccessible(current_x, current_y, wX, wY) && (returnFloodArray[wX][wY] == cVal - 1)) minCnt++;
    return minCnt > 0;
}

void makeReturnCellConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY; getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int minVal = 1000;
    if ((nX != -1) && isAccessible(current_x, current_y, nX, nY)) minVal = min(minVal, returnFloodArray[nX][nY]);
    if ((eX != -1) && isAccessible(current_x, current_y, eX, eY)) minVal = min(minVal, returnFloodArray[eX][eY]);
    if ((sX != -1) && isAccessible(current_x, current_y, sX, sY)) minVal = min(minVal, returnFloodArray[sX][sY]);
    if ((wX != -1) && isAccessible(current_x, current_y, wX, wY)) minVal = min(minVal, returnFloodArray[wX][wY]);
    if (minVal != 1000) returnFloodArray[current_x][current_y] = minVal + 1;
}

void floodFillReturnUsingQueue(int start_x, int start_y) {
    memset(visited, false, sizeof(visited)); 
    std::queue<int> cellQueue;
    
    cellQueue.push(start_x); cellQueue.push(start_y); 
    visited[start_y][start_x] = true;
    
    while (!cellQueue.empty()) {
        int cX = cellQueue.front(); cellQueue.pop(); 
        int cY = cellQueue.front(); cellQueue.pop();
        
        int nX, nY, eX, eY, sX, sY, wX, wY; 
        getSurroungings(cX, cY, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
        int neighborsX[] = {nX, eX, sX, wX}; 
        int neighborsY[] = {nY, eY, sY, wY};
        
        for (int i = 0; i < 4; i++) {
            int nbrX = neighborsX[i], nbrY = neighborsY[i];
            if (nbrX != -1 && nbrY != -1 && !visited[nbrY][nbrX] && isAccessible(cX, cY, nbrX, nbrY)) {
                returnFloodArray[nbrX][nbrY] = returnFloodArray[cX][cY] + 1;
                cellQueue.push(nbrX); cellQueue.push(nbrY); 
                visited[nbrY][nbrX] = true;
            }
        }
    }
}

char whereToMove(int current_x, int current_y, int orient, int floodMap[16][16]) {
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
    if (leftDist < 120 && rightDist < 120) {
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

void turnRight90PID() {
  leftEncoderCount = 0; rightEncoderCount = 0; error = 0; lastError = 0; integral = 0;
  long targetTicks = TICKS_FOR_90_TURN_RIGHT;
  
  while (leftEncoderCount < targetTicks) {
    error = leftEncoderCount - rightEncoderCount; 
    integral += error; derivative = error - lastError;
    long correction = (Kp_Encoder * error) + (Ki * integral) + (Kd * derivative);
    
    int leftSpeed = TURN_SPEED - correction;
    int rightSpeed = -TURN_SPEED - correction;
    
    setMotorSpeed(leftSpeed, rightSpeed);
    lastError = error; delay(5);
  }
  setMotorSpeed(0, 0);
}

void turnLeft90PID() {
  leftEncoderCount = 0; rightEncoderCount = 0; error = 0; lastError = 0; integral = 0;
  long targetTicks = TICKS_FOR_90_TURN_LEFT;
  
  while (rightEncoderCount < targetTicks) {
    error = rightEncoderCount - leftEncoderCount; 
    integral += error; derivative = error - lastError;
    long correction = (Kp_Encoder * error) + (Ki * integral) + (Kd * derivative);
    
    int leftSpeed = -TURN_SPEED - correction;
    int rightSpeed = TURN_SPEED - correction;
    
    setMotorSpeed(leftSpeed, rightSpeed);
    lastError = error; delay(5);
  }
  setMotorSpeed(0, 0);
}

void turnAround180PID() {
  leftEncoderCount = 0; rightEncoderCount = 0; error = 0; lastError = 0; integral = 0;
  long targetTicks = TICKS_FOR_180_TURN;
  
  while (leftEncoderCount < targetTicks) {
    error = leftEncoderCount - rightEncoderCount; 
    integral += error; derivative = error - lastError;
    long correction = (Kp_Encoder * error) + (Ki * integral) + (Kd * derivative);
    
    int leftSpeed = TURN_SPEED - correction;
    int rightSpeed = -TURN_SPEED - correction;
    
    setMotorSpeed(leftSpeed, rightSpeed);
    lastError = error; delay(5);
  }
  setMotorSpeed(0, 0);
}

void turnRight90() { turnRight90PID(); }
void turnLeft90() { turnLeft90PID(); }
void turnAround180() { turnAround180PID(); }

// =================================================================
// --- SETUP ---
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Micromouse 16x16 - Flood Fill w/ Return Run");
  Wire.begin();

  for (int x = 0; x < 16; x++) {
    for (int y = 0; y < 16; y++) {
      returnFloodArray[x][y] = abs(x - 0) + abs(y - 0);      }
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
      bool hasWallLeft = wallLeft(); 
      bool hasWallRight = wallRight(); 
      bool hasWallFront = wallFront();
      
      Serial.print("[EXPLORING] Walls: L:"); Serial.print(hasWallLeft); Serial.print(" F:"); Serial.print(hasWallFront); Serial.print(" R:"); Serial.println(hasWallRight);
      Serial.print("Position: ("); Serial.print(current_x); Serial.print(","); Serial.print(current_y); Serial.print(") | Orient: "); Serial.println(orient);
      
      updateCells(current_x, current_y, orient, hasWallLeft, hasWallRight, hasWallFront);
      
      if (floodArray[current_x][current_y] == 0) {
        Serial.println(">>> Goal Reached! Switching to RETURN_TO_START. <<<");
        
        if (pathLength < 300) { 
          explorationPath[pathLength].x = current_x;
          explorationPath[pathLength].y = current_y;
          explorationPath[pathLength].orientation = orient;
          pathLength++;
        }
        
        Serial.print("Recorded exploration path with "); Serial.print(pathLength); Serial.println(" steps.");
        
        currentState = RETURN_TO_START;
        hasStartedReturnRun = false;
        returnStepIndex = pathLength - 2; 
        
        delay(1000);
        return;
      }

      floodFillUsingQueue(current_x, current_y, previous_x, previous_y);
      char direction = whereToMove(current_x, current_y, orient, floodArray);
      
      Serial.print("Decision: "); Serial.println(direction);
      
      if (direction == 'L') { turnLeft90(); orient = orientation(orient, 'L'); delay(500);}
      else if (direction == 'R') { turnRight90(); orient = orientation(orient, 'R');delay(500); }
      else if (direction == 'B') { turnAround180(); orient = orientation(orient, 'B');delay(500); }
      
      moveForwardOneCell(FORWARD_SPEED);
      
      if (pathLength < 300) { 
        explorationPath[pathLength].x = current_x;
        explorationPath[pathLength].y = current_y;
        explorationPath[pathLength].orientation = orient;
        pathLength++;
      }
      
      previous_x = current_x; 
      previous_y = current_y;
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
      
      if (returnStepIndex < 0) {
        Serial.println(">>> Path completed but not at start - stopping <<<");
        currentState = FINISHED;
        setMotorSpeed(0, 0);
        return;
      }
      
      int target_x = explorationPath[returnStepIndex].x;
      int target_y = explorationPath[returnStepIndex].y;
      
      Serial.print("Following exploration path - Target: ("); 
      Serial.print(target_x); Serial.print(","); Serial.print(target_y); 
      Serial.print(") Step: "); Serial.println(returnStepIndex);
      
      char direction = 'F'; 
      
      if (target_x > current_x) { 
        int target_orient = RIGHT;
        if (orient == target_orient) direction = 'F';
        else if (orient == (target_orient + 1) % 4) direction = 'L';
        else if (orient == (target_orient + 3) % 4) direction = 'R';
        else direction = 'B';
      }
      else if (target_x < current_x) { 
        int target_orient = LEFT;
        if (orient == target_orient) direction = 'F';
        else if (orient == (target_orient + 1) % 4) direction = 'L';
        else if (orient == (target_orient + 3) % 4) direction = 'R';
        else direction = 'B';
      }
      else if (target_y > current_y) { 
        int target_orient = FORWARD;
        if (orient == target_orient) direction = 'F';
        else if (orient == (target_orient + 1) % 4) direction = 'L';
        else if (orient == (target_orient + 3) % 4) direction = 'R';
        else direction = 'B';
      }
      else if (target_y < current_y) { 
        int target_orient = BACKWARD;
        if (orient == target_orient) direction = 'F';
        else if (orient == (target_orient + 1) % 4) direction = 'L';
        else if (orient == (target_orient + 3) % 4) direction = 'R';
        else direction = 'B';
      }
      
      Serial.print("Direction to target: "); Serial.println(direction);
      
      if (direction == 'B') { turnAround180(); orient = orientation(orient, 'B'); delay(300); }
      else if (direction == 'L') { turnLeft90(); orient = orientation(orient, 'L'); delay(300);}
      else if (direction == 'R') { turnRight90(); orient = orientation(orient, 'R'); delay(300); }
      
      moveForwardOneCell(RETURN_SPEED);
      
      updateCoordinates(orient, &current_x, &current_y);
      
      returnStepIndex--;
      
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