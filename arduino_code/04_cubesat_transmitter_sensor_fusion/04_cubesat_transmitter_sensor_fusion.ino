// Government Agency Original Software Designation: LAR-18832-1
//
//------------------------------------------------------------------------------
// NASA/GSFC, Software Integration & Visualization Office, Code 610.3
//------------------------------------------------------------------------------
//
// MODULES: DHT22, BMP280, MPU9250
//
//> @author
//> Albert Aarón Cervera Uribe
//
// DESCRIPTION:
//> This script contains a testing version of Albert's NanoSat1.
// This software has not been optimized for the low energy consumption
// requirements of the Delta1 mission. A future version will be developed
// considering low power and sleep modes to further improve energy efficiency :)
//
//
// NOTES & WARNINGS
//
//> BMP280 Barometer:
// It works with both 3.3v and 5v, hence is important to check which
// voltage to use when assesing energy requirements for NanoSat.
// PS: its thermometer is not accurate.
//
//> MPU-9250 Gyroscope/Accelerometer/Magnetometer:
// [ !!! ] WARNING: ONLY USE 3v!
// Connection:
// VCC -> 3v  [ !!! ] IMPORTANT
// GND -> GND
// SCL -> A5 PIN
// SDA -> A4 PIN
//
// ISSUE:
// Program storage space should not pass 80% or it won't execute properly.
//
// REVISION HISTORY:
// 25 November 2022 - Initial Version
// -- -- 2022 - Final first Version
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

//Libraries
#include <DHT.h>
#include <RH_ASK.h>  // Include RadioHead Amplitude Shift Keying Library
// #include <SPI.h>     // Include dependant SPI Library
#include <Adafruit_BMP280.h>
#include "MPU9250.h"

// Create Amplitude Shift Keying Object
RH_ASK rf_driver(2000, A3, 9, 0);  // speed in bps, rxPin, txPin, pttPin <-- Send data in PIN 9 and receive in PIN A3


// Constants
#define DHTPIN 7       // DHT22 Data Pin
#define DHTTYPE DHT22  // DHT 22  (AM2302)

// BMP280 Barometer sensor order: VCC, GND, SCL, SDA, CSB, SDO
#define BME_SCK 13   // SCL
#define BME_MISO 12  // SDO // data pin: it was 12, but the RH transmitter needs it (seems needs to be 12)
#define BME_MOSI 11  // SDA
#define BME_CS 10    // CSB

//Variables
int spacecraftMode = 1;     // Modes -> 0: Safe Mode, 1: weather data, 2: IMU data, 3: weather + IMU
float hum;                  //Stores humidity value
float temp;                 //Stores temperature value
float baro;                 // Stores pressure value
float pressureGroundLevel;  // current pressure when CubeSat started
float localAltitude;        // altitude in meters from ground

// Inertial Measurement Unit variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float pitch, roll, yaw;

// Status variables
bool bmp280_status;
int mpu9250_status;

/*
 TODO: 
 Add the following data
 1) If possible add last current generated by satellite's solar arrays.
 2) Onboard timestamp
 3) GPS coordinates
 4) Other sensors/cameras data -> Perhaps managed by Arduino_B
*/

struct telemetryStruct {
  int mode;
  float voltage;
  float humidity;
  float temperature;
  float pressure;
  float localAltitude;  // altitude in meters from ground
  float pitch;
  float roll;
  float yaw;
} telemetryData;

struct commandStruct {
  float op;
} commandData;

byte tel_buf[sizeof(telemetryData)] = { 0 };  // buffer for sending telemetry data

// INITIALIZATION OF SENSORS
DHT dht(DHTPIN, DHTTYPE);     // Initialize DHT sensor for normal 16mhz Arduino
Adafruit_BMP280 bme(BME_CS);  // hardware SPI for BMP280
MPU9250 IMU(Wire, 0x68);      //MPU-9250 sensor on I2C bus 0 with address 0x68

void setup() {
  Serial.begin(9600);
  dht.begin();

  bmp280_status = bme.begin();
  if (!bmp280_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1) {};
  }

  mpu9250_status = IMU.begin();
  if (mpu9250_status < 0) {
    Serial.println("IMU initialization unsuccessful");
    while (1) {}
  }

  resetPressureGroundLevel();  // Read ground level pressure when initiating board
  rf_driver.init();
  // Serial.print("\nWaiting data ...");
}

void loop() {
  /*
  TODO: add its own 'delay()' functions to each mode so power saving can be added.
  */
  
  switch (spacecraftMode) {
    case 0:
      // Safe Mode: Emergency Low Power Mode (all non-essential systems are shut down )
      /* 
      ---------------------------------------------------------------------------------------------------
      Delta1 Safe Mode:

      Description: 
      In this mode, Delta1 will become a beacon that just transmits data every X minutes.
      It should have a flag to determine which trigger reason triggered the safe mode.
      
      This flag could be a boolean array:
      '{0,0,0}' <- low voltage trigger: false, IMU trigger: false, temperature trigger: false
      '{1,0,0}' <- low voltage trigger: true, IMU trigger: false, temperature trigger: false
      '{0,1,1}' <- low voltage trigger: false, IMU trigger: true, temperature trigger: true
      ... and so on.

      Triggers:
      1) A low battery voltage/level (critical energy)
      2) If the pitch or roll are greater than 90 degrees for more than 8h (good in gravity, so solar panels point up)
      3) Overheating of the spacecraft (temp > 55 celsius)

      Recovery:
      Each trigger should have its recovery (safe mode turning off) process, however
      while in safe mode, the spacecraft should check for the conditions. 
      Only if the safe flag is '{0,0,0}' then, safe mode is turned off.

      Requirements: 

      1) Transmit spacecraft health and telemetry data
      2) Reduce processor clock speed in both Arduino_A and Arduino_B to minimal and/or:
      3) Have sleep modes
    
      */

      temp = dht.readTemperature();

      telemetryData.mode = spacecraftMode;          // <-- CRITICAL
      telemetryData.voltage = getBatteryVoltage();  // <-- CRITICAL
      telemetryData.humidity = 0.0;
      telemetryData.temperature = 0.0;  // <-- CRITICAL (for battery health)
      telemetryData.pressure = 0.0;
      telemetryData.localAltitude = 0.0;
      telemetryData.pitch = 0.0;
      telemetryData.roll = 0.0;
      telemetryData.yaw = 0.0;

      transmitData();

      // Change this delay by a lowPower idle sleep mode of 10 mins
      delay(60000);  // 1,000 ms = 1 second, 60,000 ms = 1 min => 600,000 ms = 10 mins

      break;
    case 1:
      // Read BMP280 and DHT22 sensors

      hum = dht.readHumidity();
      temp = dht.readTemperature();
      baro = (bme.readPressure() / 100.0F);
      localAltitude = getLocalAltitude();

      telemetryData.mode = spacecraftMode;
      telemetryData.voltage = getBatteryVoltage();
      telemetryData.humidity = hum;
      telemetryData.temperature = temp;
      telemetryData.pressure = baro;
      telemetryData.localAltitude = localAltitude;
      telemetryData.pitch = 0.0;
      telemetryData.roll = 0.0;
      telemetryData.yaw = 0.0;

      break;
    case 2:
      // Read MPU9250 sensor
      computePitchRollYaw();

      telemetryData.mode = spacecraftMode;
      telemetryData.voltage = getBatteryVoltage();
      telemetryData.humidity = 0.0;
      telemetryData.temperature = 0.0;
      telemetryData.pressure = 0.0;
      telemetryData.localAltitude = 0.0;
      telemetryData.pitch = pitch;
      telemetryData.roll = roll;
      telemetryData.yaw = yaw;
      break;
    case 3:
      // Read both BMP280, DHT22 & MPU9250 sensors (very power huungry)

      hum = dht.readHumidity();
      temp = dht.readTemperature();
      baro = (bme.readPressure() / 100.0F);
      localAltitude = getLocalAltitude();
      computePitchRollYaw();

      telemetryData.mode = spacecraftMode;
      telemetryData.voltage = getBatteryVoltage();
      telemetryData.humidity = hum;
      telemetryData.temperature = temp;
      telemetryData.pressure = baro;
      telemetryData.localAltitude = localAltitude;
      telemetryData.pitch = pitch;
      telemetryData.roll = roll;
      telemetryData.yaw = yaw;
      break;
  }

  int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  // Receiver code ------------------------------
  if (rf_driver.recv(buf, &buflen) == 1) {
    memcpy(&commandData, buf, sizeof(commandData));
    // Serial.print("\nDATA RECEIVED: \n");
    // Serial.print(commandData.op);
    executeCommand(commandData.op);
  } else {
    delay(100);  // Delay, so it can then send data without signal interference
    transmitData();
  }

  // (delay was considered inside spacecraftMode routines)
  // delay(250);
}  // end void loop

void transmitData() {
  memcpy(tel_buf, &telemetryData, sizeof(telemetryData));
  byte zize = sizeof(telemetryData);
  rf_driver.send((uint8_t *)tel_buf, zize);
  rf_driver.waitPacketSent();
}

void executeCommand(float command) {
  if (command == 0.0) {
    spacecraftMode = 0;
  } else if (command == 1.0) {
    spacecraftMode = 1;
  } else if (command == 2.0) {
    resetPressureGroundLevel();
    spacecraftMode = 1;
  } else if (command == 3.0) {
    spacecraftMode = 2;
  } else if (command == 4.0) {
    spacecraftMode = 3;
  }
}

float getLocalAltitude() {
  // Ground level height measurement
  return localAltitude = bme.readAltitude(pressureGroundLevel);
}

void resetPressureGroundLevel() {
  // Read ground level pressure when initiating board
  // i.e. 77994.4/100  -> 779.94 hPa
  pressureGroundLevel = (bme.readPressure() / 100.0F);
}

void getAccelIMU() {
  IMU.readSensor();
  accelX = IMU.getAccelX_mss();  // 0.09 (without 6 decimal precision :c)
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
}

void getGyroIMU() {
  /*
  Measured in degrees per second, angular velocity is the change 
  in the rotational angle of the object per unit of time.
  */
  IMU.readSensor();
  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
}

void getMagneIMU() {
  IMU.readSensor();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();
}

void computePitchRollYaw() {
  getAccelIMU();
  getMagneIMU();

  // Euler angle from accelerometer
  roll = atan2(accelY, (sqrt((accelX * accelX) + (accelZ * accelZ))));
  pitch = atan2(-accelX, (sqrt((accelY * accelY) + (accelZ * accelZ))));

  // Yaw angle from magnetometer
  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch)) + (magY * sin(roll) * sin(pitch)) + (magZ * cos(roll) * sin(pitch));
  yaw = atan2(Yh, Xh);

  // Convert radians into degrees
  pitch = pitch * (180.0 / PI);
  roll = roll * (180.0 / PI);
  yaw = yaw * (180.0 / PI);
}

float getBatteryVoltage() {
  /*
  TODO: implement a battery voltage SoC algorithm.
  It may use two resistors
  */
  return 3.2;  // Return volts
}
