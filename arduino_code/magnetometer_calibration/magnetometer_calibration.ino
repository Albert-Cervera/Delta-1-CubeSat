#include "MPU9250.h"
#include "EEPROM.h"

/* MPU 9250 object */
MPU9250 imu(Wire, 0x68);

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float value;

void setup() {
  /* Serial for displaying instructions */
  // Serial.begin(115200);
  Serial.begin(9600);
  while(!Serial) {}
  /* Start communication with IMU */
  imu.begin();
  /* Calibrate magnetometer */
  Serial.print("Calibrating magnetometer, please slowly move in a figure 8 until complete...");
  imu.calibrateMag();
  Serial.println("Done!");
  Serial.print("Saving results to EEPROM...");
  /* Save to EEPROM */
  value = imu.getMagBiasX_uT();
  memcpy(eeprom_buffer, &value, sizeof(value));
  value = imu.getMagBiasY_uT();
  memcpy(eeprom_buffer + 4, &value, sizeof(value));
  value = imu.getMagBiasZ_uT();
  memcpy(eeprom_buffer + 8, &value, sizeof(value));
  value = imu.getMagScaleFactorX();
  memcpy(eeprom_buffer + 12, &value, sizeof(value));
  value = imu.getMagScaleFactorY();
  memcpy(eeprom_buffer + 16, &value, sizeof(value));
  value = imu.getMagScaleFactorZ();
  memcpy(eeprom_buffer + 20, &value, sizeof(value));
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++) {
    EEPROM.write(i, eeprom_buffer[i]);
  }
  Serial.println("Done! You may power off your board.");
}

void loop() {}