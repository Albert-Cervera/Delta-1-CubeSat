
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
//
// REVISION HISTORY:
// 25 September 2022 - Initial Version
// -- -- 2022 - Final first Version
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"

// PIN & SENSOR CONFIGURATION

// DHT22
#define DHTPIN 7
#define DHTTYPE DHT22

// BMP280 sensor order: VCC, GND, SCL, SDA, CSB, SDO
#define BME_SCK 13    // SCL
#define BME_MISO 12   // SDO Data pin: 12
#define BME_MOSI 11   // SDA
#define BME_CS 10     // CSB

/*
1013.25 from example. Need to adjust to local sealevel pressure
Altitude at my apartment: 2,246 m above sea level
With sea lavel presusre in 1025.00 and real pressure: 780.3 hPa = 2240 m approx
1025.60 and pressure 780.3 = altitude 2246 m approx
*/
#define SEALEVELPRESSURE_HPA (1025.60)

// Sensor variables
float hum; // humidity in percentage
float temp; // temperature in celsius
float pressure; // air pressure in hPa
float pressureGroundLevel; // current pressure when CubeSat started
float externalPressureAtGround;
float altitude; // altitude in meters from sea level (estimated from air pressure)
float localAltitude; // altitude in meters from ground

// Inertial Measurement Unit variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
float headingCompass;
float timeAccelerationLast = 0.0;
float pitch, roll, yaw;
float maxVal = 0.00;
float minVal = 0.00;

// Status variables
bool bmp280_status;
int mpu9250_status;

// INITIALIZATION OF SENSORS
DHT dht(DHTPIN, DHTTYPE); // DHT22 sensor
Adafruit_BMP280 bme(BME_CS); // hardware SPI for BMP280
MPU9250 IMU(Wire,0x68); //MPU-9250 sensor on I2C bus 0 with address 0x68


void setup() {
  Serial.begin(9600);

  dht.begin(); // DHT22

  bmp280_status = bme.begin();
  if (!bmp280_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  mpu9250_status = IMU.begin();
  if (mpu9250_status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(mpu9250_status);
    while(1) {}
  }

  // Read ground level pressure when initiating board
  resetPressureGroundLevel();
  // setPressureGroundLevel(785.00); // hPa

}

void loop() {  

  // getTemperature();
  // getHumidity();
  // getPressure();
  // getAltitude();  
  // getLocalAltitude();

  /* Raw IMU measurements */  
  // getAccelIMU();
  // getGyroIMU();
  getMagneIMU();

  // getHeading();
  // computeVelocity(); // Time dependant (ms delay)
  // computePitchRollYaw();

  // Delay in ms
  delay(1000);
}

//------------------------------------------------------------------------------

void getMinMax(float currentVal) {
  maxVal = max(maxVal, currentVal);
  minVal = min(minVal, currentVal);

  Serial.print("maxVal: ");
  Serial.print(maxVal);
  Serial.print(" ");

  Serial.print("minVal: ");
  Serial.print(minVal);
  Serial.print(" ");
}

void getTemperature() {  
  temp= dht.readTemperature();

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println(" *C"); // °C
  Serial.println();
}

void getHumidity() {
  hum = dht.readHumidity();
  
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" %");
  Serial.println();
}

void getPressure() {
  pressure = bme.readPressure() / 100.0F;

  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");   
  Serial.println();
}

void getAltitude() {
  // TODO: 
  // Set a given altitude at a given pressure and estimate altitud 
  //from that point
  
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  Serial.print("Approx. Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");
  Serial.println();

  // Serial.print("Temperature from barometer = ");
  // Serial.print(bme.readTemperature());
  // Serial.println(" *C");
  // Serial.println();
}

void getLocalAltitude() {
  // Ground level height measurement
  localAltitude = bme.readAltitude(pressureGroundLevel);

  Serial.print("Approx. Local Altitude = ");
  Serial.print(localAltitude);
  Serial.println(" m");
  Serial.println();
}

void setPressureGroundLevel(float externalPressureAtGround) {
  //Example of pressureAtGround: 782.00 hPa
  pressureGroundLevel = externalPressureAtGround;
}

void resetPressureGroundLevel() {
  // Read ground level pressure when initiating board
  // i.e. 77994.4/100  -> 779.94 hPa
  pressureGroundLevel = (bme.readPressure()/100.0F);
}

void getAccelIMU() {
  IMU.readSensor();
  
  accelX = IMU.getAccelX_mss(); // 0.09 (without 6 decimal precision :c) 
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();  
  
  // Display the data
  // Serial.print("AccelX: ");  
  // Serial.print(accelX, 6); // printing 6 decimal places after point.
  // Serial.print("\t");  
  // Serial.print("AccelY: ");
  // Serial.print(accelY, 6);
  // Serial.print("\t");
  // Serial.print("AccelZ: ");
  // Serial.print(accelZ, 6);
  // Serial.print("\t");
  // Serial.println();      
  
  // // Worst thermometer ever
  // Serial.print("\n");
  // Serial.print("Temp: ");
  // Serial.println(IMU.getTemperature_C(),6);
  // Serial.print("\t");
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

  // Serial.print("\n");
  // Serial.print("GyroX: ");
  // Serial.print(gyroX, 6);
  // Serial.print("\t");
  // Serial.print("GyroY: ");
  // Serial.print(gyroY, 6);
  // Serial.print("\t");
  // Serial.print("GyroZ: ");
  // Serial.print(gyroZ, 6);
  // Serial.print("\t");
  // Serial.println();
}

void getMagneIMU(){  
  IMU.readSensor();

  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();

  // Serial.print("\n");
  // Serial.print("MagX: ");
  // Serial.print(magX, 6);
  // Serial.print("\t");
  // Serial.print("MagY: ");
  // Serial.print(magY, 6);
  // Serial.print("\t");
  // Serial.print("MagZ: ");
  // Serial.print(magZ, 6);
  // Serial.print("\t");
  // Serial.println();
  
}

void getHeading(){ 
  getMagneIMU(); 

  // headingCompass = ((atan2(magY, magX) - 0.1) * 180.0) / PI; // original
  headingCompass = (atan2(magY, magX) * 180.0) / PI;    
  if (headingCompass < 0) headingCompass += 360.0;
  // if (headingCompass > 360) headingCompass -= 360.0;  

  Serial.print("Compass Heading: ");
  Serial.print(headingCompass);
  Serial.print("\t");
  Serial.println();
}

void computeVelocity() {
 /*
  It will be computed for one of the X-Y-Z axis from IMU.
  
  It is heavily dependant on the delay(ms) value from loop.  
  Example: at Z-axis and delay 1500 ms, it should have a 
  velocity of 15 m/s (54 km/h). 

  Therefore for greater accuracy, delay should be as minimum
  as possible in order to have the instant velocity.
  
  The algorithm seems to work just 'fine', however, further 
  testing and finer adjustment must be done in a controlled
  environment with truth value data for comparisson.

  Note: measuring speed walk was difficult. The ms delay must be
  500 or 600 ms to measure 3 kph or so.
 */
  getAccelIMU();

  float acceleration = accelY; // accelX, accelZ or 3.0. <-- m/s2
  float timeAcceleration = millis(); // 5.0 for example
  
  // dv = (a)(dt) <-- m/s
  float dv = acceleration * ((timeAcceleration - timeAccelerationLast)/1000);  
  float velocityKph = dv * 3.6;
  timeAccelerationLast = timeAcceleration;

  // Serial.println("\n--------------\n");
  // Serial.print("Time: ");
  // Serial.print(timeAcceleration - timeAccelerationLast);
  // Serial.print("\n");

  // Serial.print("acceleration: ");
  // Serial.print(acceleration);
  // Serial.print("\n");
  
  Serial.print("Velocity: ");
  Serial.print(velocityKph);
  Serial.print(" km/h\t");
  Serial.println();

  getMinMax(velocityKph);

}

void computePitchRollYaw() {
  getAccelIMU();
  getMagneIMU();
  
  // Euler angle from accelerometer
  roll = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  pitch = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));
  
  // Yaw angle from magnetometer
  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));
  yaw =  atan2(Yh, Xh);
   
  // Convert radians into degrees
  pitch = pitch * (180.0 / PI);
  roll = roll * (180.0 / PI);
  yaw = yaw * (180.0 / PI);
  
  Serial.print("Pitch (X): ");
  Serial.print(pitch);
  Serial.print(" degrees\t");
  Serial.print("Roll (Y): ");
  Serial.print(roll);
  Serial.print(" degrees\t");
  Serial.print("Yaw (z): ");
  Serial.print(yaw);
  Serial.print(" degrees\t");
  Serial.println();
}


