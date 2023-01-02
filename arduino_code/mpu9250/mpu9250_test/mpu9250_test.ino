 /*
Author: Albert C.
[ !!! ] WARNING: ONLY USE 3v!

NOTE:
1) Connection:
VCC -> 3v  [ !!! ] IMPORTANT
GND -> GND
SCL -> A5
SDA -> A4
*/

#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on
// I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data  
  Serial.begin(9600);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  } else {
    Serial.print("IMU communication succesful!");    
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();

  // // display the data
  Serial.print("\n");
  Serial.print("AccelX: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print("AccelY: ");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print("AccelZ: ");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");

  
  // Serial.print("\n");
  // Serial.print("GyroX: ");
  // Serial.print(IMU.getGyroX_rads(),6);
  // Serial.print("\t");
  // Serial.print("GyroY: ");
  // Serial.print(IMU.getGyroY_rads(),6);
  // Serial.print("\t");
  // Serial.print("GyroZ: ");
  // Serial.print(IMU.getGyroZ_rads(),6);
  // Serial.print("\t");
  
  // Serial.print("\n");
  // Serial.print("MagX: ");
  // Serial.print(IMU.getMagX_uT(),6);
  // Serial.print("\t");
  // Serial.print("MagY: ");
  // Serial.print(IMU.getMagY_uT(),6);
  // Serial.print("\t");
  // Serial.print("MagZ: ");
  // Serial.print(IMU.getMagZ_uT(),6);
  // Serial.print("\t");
  
  // // Serial.print("\n");
  // Serial.print("Temp: ");
  // Serial.println(IMU.getTemperature_C(),6);


  delay(250);
}
