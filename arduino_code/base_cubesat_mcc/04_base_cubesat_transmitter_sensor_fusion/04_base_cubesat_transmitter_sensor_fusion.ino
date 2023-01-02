
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
// 25 November 2022 - Initial Version
// -- -- 2022 - Final first Version
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

//Libraries
#include <DHT.h>;
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
#include <Adafruit_BMP280.h>

// Create Amplitude Shift Keying Object 
// IMPORTANT: Data pin of receiver should be at PIN 12 (it magically works with library)
RH_ASK rf_driver(2000, A3, 9, 0); // speed in bps, rxPin, txPin, pttPin <-- Send data in PIN 9 and receive in PIN A3
// RH_ASK rf_driver; // PIN 12 by default

// Constants
#define DHTPIN 7     // DHT22 Data Pin
#define DHTTYPE DHT22   // DHT 22  (AM2302)

// BMP280 Barometer sensor order: VCC, GND, SCL, SDA, CSB, SDO
#define BME_SCK 13    // SCL
#define BME_MISO 12   // SDO // data pin: it was 12, but the RH transmitter needs it (seems needs to be 12)
#define BME_MOSI 11   // SDA
#define BME_CS 10     // CSB

//Variables
float hum;  //Stores humidity value
float temp; //Stores temperature value
float baro; // Stores pressure value
float pressureGroundLevel; // current pressure when CubeSat started
float localAltitude; // altitude in meters from ground

// Status variables
bool bmp280_status;

// struct weatherStruct{
//   float humidity ; 
//   float temperature;
//   float pressure;  
// }weatherData;

struct telemetryStruct{
  float humidity; 
  float temperature;
  float pressure; 
  float localAltitude; // altitude in meters from ground  
}telemetryData;

struct commandStruct{
  float op;   
}commandData;

// byte tx_buf[sizeof(weatherData)] = {0}; // buffer for sending  weather data
byte tel_buf[sizeof(telemetryData)] = {0}; // buffer for sending telemetry data

// INITIALIZATION OF SENSORS
DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor for normal 16mhz Arduino
Adafruit_BMP280 bme(BME_CS); // hardware SPI for BMP280

void setup()
{
  Serial.begin(9600);
  dht.begin();

  bmp280_status = bme.begin();
  if (!bmp280_status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Read ground level pressure when initiating board
  resetPressureGroundLevel();

  rf_driver.init();
  Serial.print("\nWaiting data ...");
}

void loop()
{
    //Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp = dht.readTemperature();
    baro = (bme.readPressure() / 100.0F);
    localAltitude = getLocalAltitude();

    // weatherData.humidity = hum;
    // weatherData.temperature = temp;
    // weatherData.pressure = baro;

    telemetryData.humidity = hum;
    telemetryData.temperature = temp;
    telemetryData.pressure = baro;
    telemetryData.localAltitude = localAltitude;
    
    // Serial.print("\nlocalAltitude: ");
    // Serial.print(localAltitude);

    // Serial.print("Waiting ");
    int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    // Receiver code ------------------------------    
    if (rf_driver.recv(buf, &buflen) == 1) {
      memcpy(&commandData, buf, sizeof(commandData));  
      Serial.print("\nDATA RECEIVED: \n");
      Serial.print(commandData.op);  
      executeCommand(commandData.op);    
    } else {
      delay(100); // Delay, so it can then send data without signal interference
      memcpy(tel_buf, &telemetryData, sizeof(telemetryData));
      byte zize2=sizeof(telemetryData);    
      rf_driver.send((uint8_t *)tel_buf, zize2);
      rf_driver.waitPacketSent();
    }

    // Transmitter code ------------------------------    
    // delay(100); // Delay, so it can then send data without signal interference
    // memcpy(tx_buf, &weatherData, sizeof(weatherData));
    // byte zize=sizeof(weatherData);    
    // rf_driver.send((uint8_t *)tx_buf, zize);
    // rf_driver.waitPacketSent();

    

    delay(250); //1000: 1 second
}

void executeCommand(float command) {
  if (command == 1.0) {
    Serial.print("\nResetting pressure level");
    resetPressureGroundLevel();
  }
}

float getLocalAltitude() {
  // Ground level height measurement
  return localAltitude = bme.readAltitude(pressureGroundLevel);  
}

void resetPressureGroundLevel() {
  // Read ground level pressure when initiating board
  // i.e. 77994.4/100  -> 779.94 hPa
  pressureGroundLevel = (bme.readPressure()/100.0F);
}

   