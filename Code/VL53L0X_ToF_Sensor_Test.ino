#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SENSOR1_SHUT_PIN 16
#define SENSOR2_SHUT_PIN 17
#define SENSOR3_SHUT_PIN 4

Adafruit_VL53L0X sensors[3];

#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31
#define SENSOR3_ADDRESS 0x32

void setup() {
  Serial.begin(115200);

  Wire.begin();

  pinMode(SENSOR1_SHUT_PIN, OUTPUT);
  pinMode(SENSOR2_SHUT_PIN, OUTPUT);
  pinMode(SENSOR3_SHUT_PIN, OUTPUT);

  digitalWrite(SENSOR1_SHUT_PIN, LOW);
  digitalWrite(SENSOR2_SHUT_PIN, LOW);
  digitalWrite(SENSOR3_SHUT_PIN, LOW);

  delay(50); 

  initSensor(0, SENSOR1_SHUT_PIN, SENSOR1_ADDRESS);
  initSensor(1, SENSOR2_SHUT_PIN, SENSOR2_ADDRESS);
  initSensor(2, SENSOR3_SHUT_PIN, SENSOR3_ADDRESS);
}

void loop() {
  for (int i = 0; i < 3; i++) {
    readDistance(i);
  }
  Serial.println(); 

  delay(100); 
}

void initSensor(int sensorIndex, int shutPin, uint8_t newAddress) {
  digitalWrite(shutPin, HIGH);
  delay(50); 

  if (!sensors[sensorIndex].begin(0x29)) {
    Serial.print(F("Failed to boot sensor "));
    Serial.println(sensorIndex + 1);
    while (1);
  }

  sensors[sensorIndex].setAddress(newAddress);
  Serial.print(F("Sensor "));
  Serial.print(sensorIndex + 1);
  Serial.print(F(" initialized with address 0x"));
  Serial.println(newAddress, HEX);
}

void readDistance(int sensorIndex) {
  VL53L0X_RangingMeasurementData_t measure;

  sensors[sensorIndex].rangingTest(&measure, false);

  Serial.print("S");
  Serial.print(sensorIndex + 1);
  Serial.print(": ");

  if (measure.RangeStatus != 4) {
    Serial.print(measure.RangeMilliMeter);
    Serial.print("mm\t"); // Use a tab for neat columns
  } else {
    Serial.print("Out of range\t");
  }
}
