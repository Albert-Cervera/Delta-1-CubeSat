/*
Author: Albert C.
NOTE:
 1) It works with both 3.3v and 5v, hence is important to check which
 voltage to use when assesing energy requirements for NanoSat.
 Best to use 3v!

 2) It's not the best themometer out there, but  at least 
 barometer is accurate!

 TODO:
 Set a given altitude at a given pressure and estimate altitud 
 from that point
 
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Sensor order: VCC, GND, SCL, SDA, CSB, SDO
#define BME_SCK 13    // SCL
#define BME_MISO 9   // SDO Pin data: 12
#define BME_MOSI 11   // SDA
#define BME_CS 10     // CSB

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP280 bme(BME_CS); // hardware SPI <---- This is the one!
//Adafruit_BME280 bme; // I2C
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  bool status;
    
  status = bme.begin();  // OK
  // status = bme.begin(0x76);  // <-- It also works with this address, but not required
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("-- Default Test --");
  delayTime = 2000;

  Serial.println();
}


void loop() { 
  printValues();
  delay(delayTime);
}


void printValues() {
  // Serial.print("Temperature = ");
  // Serial.print(bme.readTemperature());
  // Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  
  // Serial.print("Approx. Altitude = ");
  // Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println(" m");
    
  Serial.println();
  //delay(5000);
}