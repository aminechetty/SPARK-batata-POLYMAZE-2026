#include <Arduino.h>
#include <QTRSensors.h>

uint8_t sensorPins[] = {
  4,   // S1  (odd)
  18,  // S2  (even)
  5,   // S3  (odd)
  8,   // S4  (even)
  6,   // S5  (odd)
  9,   // S6  (even)
  7,   // S7  (odd)
  10,  // S8  (even)
  15,  // S9  (odd)
  11,  // S10 (even)
  16,  // S11 (odd)
  12,  // S12 (even)
  17   // S13 (odd)
};

const uint8_t NUM_SENSORS = 13;
const uint16_t PRINT_DELAY_MS = 500;

QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  qtr.setEmitterPins(13, 14); // CTRL_ODD, CTRL_EVEN

  delay(500);

  Serial.println("Calibrating QTR sensors...");
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
  }

  Serial.println("Calibration minimum values:");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("%4u", qtr.calibrationOn.minimum[i]);
    if (i < NUM_SENSORS - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();

  Serial.println("Calibration maximum values:");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("%4u", qtr.calibrationOn.maximum[i]);
    if (i < NUM_SENSORS - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println();
  Serial.println("Calibrated sensor values and position:");
}

void loop() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  Serial.print("QTR | ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("%4u", sensorValues[i]);
    if (i < NUM_SENSORS - 1) {
      Serial.print(" | ");
    }
  }
  Serial.printf(" || Pos: %u", position);
  Serial.println();

  delay(PRINT_DELAY_MS);
}