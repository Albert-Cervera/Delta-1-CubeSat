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
// e-mail: astra.delta.v@gmail.com
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
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include <RH_ASK.h>  // Include RadioHead Amplitude Shift Keying Library
#include <Wire.h>
#include <DallasTemperature.h>
#include "LowPower.h"

// Create Amplitude Shift Keying Object
RH_ASK rf_driver(2000, A3, 9, 0);  // speed in bps, rxPin, txPin, pttPin <-- Send data in PIN 9 and receive in PIN A3

// Constants
#define DHTPIN 7         // DHT22 Data Pin
#define DHTTYPE DHT22    // DHT 22  (AM2302)
#define GPS_PIN 3        // Pin in which the MOSFET would turn on/off the GPS sensor
#define FIVE_VOLT_PIN 5  // Pin to turn on/off the 5V sensors (DSB18B20 and DHT22)


// BMP280 Barometer sensor order: VCC, GND, SCL, SDA, CSB, SDO
// #define BME_SCK 13   // SCL
// #define BME_MISO 12  // SDO // data pin: it was 12, but the RH transmitter needs it (seems needs to be 12)
// #define BME_MOSI 11  // SDA
#define BME_CS 10  // CSB

//Variables
// Modes -> 0: Safe Mode, 1: ESM, 2: HIB, 3: RTI, 4: RTWI
int spacecraftMode = 4;
int lastMode = 4;           // should be same as spacecraftMode when compiling
float pressureGroundLevel;  // Current pressure when CubeSat started
float pitch, roll, yaw;     // Inertial Measurement Unit variables
bool isCommandReceived;
bool rtcSynced;

/*
 TODO: 
 Add the following data
 1) If possible add last current generated by satellite's solar arrays.
 2) Onboard timestamp
 3) GPS coordinates
 4) Other sensors/cameras data -> Perhaps managed by Arduino_B
*/

// 28 bytes size and 7 elements (float: 4 bytes, int: 2 bytes)
// Update: 30 bytes with int header
struct telemetryStruct {
  float humidity;
  float temperature;
  float pressure;
  float localAltitude;  // altitude in meters from ground
  float pitch;
  float roll;
  float yaw;
} telemetryData;

// 10 bytes size and 3 elements (float: 4 bytes, int: 2 bytes)
struct dataStruct {
  int mode;
  float voltage;
  float internalTemp;  // dedicated temperature DSB18B20 sensor
  // float solarCurrent; // Future val: current generated by solar arrays
} systemData;

// Get rid of this and only use a byte
struct commandStruct {
  float op;
} commandData;

// Size 8 bytes
struct missionClockStruct {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  uint8_t day;
  uint8_t month;
  int year;
  bool isSynced;
} rtcData;

int initDay, initMonth, initYear, initHour, initMinute, initSecond;

// INITIALIZATION OF SENSORS
DHT dht(DHTPIN, DHTTYPE);     // Initialize DHT sensor for normal 16mhz Arduino
Adafruit_BMP280 bme(BME_CS);  // hardware SPI for BMP280
MPU9250 IMU(Wire, 0x68);      //MPU-9250 sensor on I2C bus 0 with address 0x68
OneWire ourWire(2);           // DSB18B20 on Pin 2
DallasTemperature sensors(&ourWire);

void setup() {
  Serial.begin(9600);

  pinMode(GPS_PIN, OUTPUT);  // Defining this pin as output pin
  pinMode(FIVE_VOLT_PIN, OUTPUT);

  /*
    NOTE: All sensors must be connected at Boot
  */
  digitalWrite(FIVE_VOLT_PIN, HIGH);  // Setting pin to HIGH (turn ON)

  // Serial.print("\nM1");

  dht.begin();
  sensors.begin();  // DSB18B20
  sensors.setResolution(12);

  bool bmp280_status = bme.begin();
  if (!bmp280_status) {
    Serial.println("E1");  //E1 = BARO_ERR. Maybe rise a flag and activate safe mode in case of failure
    while (1) {};
  }

  int mpu9250_status = IMU.begin();
  if (mpu9250_status < 0) {
    Serial.println("E2");  //E2 = IMU_ERR. Maybe rise a flag and activate safe mode in case of failure
    while (1) {}
  }

  resetPressureGroundLevel();  // Read ground level pressure when initiating board

  // Ask SAT_B for mission clock time via I2C
  Wire.requestFrom(8, 8);  // request 8 bytes from peripheral device #8 (device#, bytes) TODO: check the correct amount of bytes to receive
  Wire.readBytes((byte *)&rtcData, sizeof rtcData);
  // // Ensures reading of all bytes from stream
  while (Wire.available()) {
    Wire.read();
  }

  // Serial.print("\nsizeof rtcData): " + String(sizeof rtcData)); // 7 bytes
  initHour = rtcData.hours;
  initMinute = rtcData.minutes;
  initSecond = rtcData.seconds;
  initDay = rtcData.day;
  initMonth = rtcData.month;
  initYear = rtcData.year;
  rtcSynced = rtcData.isSynced;

  Wire.begin();  // join I2C bus (address optional for master)
  rf_driver.init();
  delay(1500);  // Wait x seconds so SAT_B gets GPS time //3200
}

void loop() {
  float hum, temp, baro, localAltitude;
  // Safe mode trigger condition checking
  checkTriggerSM(lastMode);  // Turns ON 5V line
  systemData.mode = spacecraftMode;

  // Modes -> 0: Safe Mode, 1: ESM, 2: RTW, 3: RTI, 4: RTWI
  switch (spacecraftMode) {
    case 0:  // Safe Mode
      {
        // Safe Mode: Emergency Low Power Mode (all non-essential systems are shut down )
        /* 
        ---------------------------------------------------------------------------------------------------
        Delta1 Safe Mode:

        Description: 
        In this mode, Delta1 will become a beacon that just transmits data every X minutes.
   

        Triggers:
        1) A low battery voltage/level (critical energy)
        2) Overheating of the spacecraft (temp > 55 celsius)
        3) If the pitch or roll are greater than 90 degrees for more than 8h (good in gravity, so solar panels point up)
        

        Recovery:
        Each trigger should have its recovery (safe mode turning off) process, however
        while in safe mode, the spacecraft should check for the conditions. 
        Only if the safe flag is '{0,0,0}' then, safe mode is turned off.

        Requirements: 

        1) Transmit spacecraft health data
        2) Sleep modes on SAT_A and SAT_B
        3) Query health status, store and transmit data every 10 natural minutes
        4) Transmit data just 5 seconds and don't listen for commands
      
        */

        digitalWrite(GPS_PIN, LOW);        // Setting pin to LOW (turn OFF)
        digitalWrite(FIVE_VOLT_PIN, LOW);  // Setting pin to LOW (turn OFF)

        telemetryData.humidity = -1.0;
        telemetryData.temperature = -1.0;
        telemetryData.pressure = -1.0;
        telemetryData.localAltitude = -1.0;

        computePitchRollYaw();  // Data that might be of importance

        telemetryData.pitch = pitch;
        telemetryData.roll = roll;
        telemetryData.yaw = yaw;

        // Send telemetryData via I2C to peripheral slave for data saving
        sendDataI2C(3);  // Save system data as Safe Mode

        // Transmitting 4 seconds
        unsigned long startTime = millis();
        unsigned long endTime = startTime;
        while ((endTime - startTime) <= 4000) {
          transmitData();
          endTime = millis();
        }

        // Listen 4 seconds for incomming MCC commands
        // int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
        int8_t buf[sizeof(commandData)];
        uint8_t buflen = sizeof(buf);

        startTime = millis();
        endTime = startTime;

        while ((endTime - startTime) <= 4000) {  // Listen radio for 4,000 ms while transmitting ack
          if (rf_driver.recv(buf, &buflen) == 1) {

            memcpy(&commandData, buf, sizeof(commandData));
            if (commandData.op == 6.0) {
              executeCommand(6.0);  // Reboot OBCs command
            }

            // Transmit an ack message for X seconds (could be the same telemetry data with the SC mode)
            unsigned long startTime2 = millis();
            unsigned long endTime2 = startTime2;
            while ((endTime2 - startTime2) <= 5000) {  // Transmit ack for 5,000 ms // 5k
              systemData.mode = spacecraftMode;        // Update with new mode
              transmitData();
              endTime2 = millis();
            }
            break;  // end 'if' and break while loop
          }
          endTime = millis();
        }

        // Send sleep command to SAT_B and then sleep spacecraft
        // sendDataI2C(6);  // Send sleep command  to peripheral (9 mins, 4 seconds calculated) and/or sync RTC with GPS

        sendDataI2C(8);  // Send SUDO sleep command to peripheral (9 mins, 4 seconds calculated), no GPS-RTC sync
        deepSleep(2);    // sleep SAT_A 9 mins, 50 sec

        break;
      }
    case 1:  // ESM mode
      {
        // Extended Science Mission (ESM) mode (default)
        // Only mode with RTC-GPS synchronization

        /* NOTE: take into account the  total MS time (routine + sleep)
        routine: 20 ms + 10 seconds
        Sleep: 10 mins, 1 sec
        -> Total loop time: 10 mins, 11 secs, 20 ms + range
        
        Time before sleep:
        Computed: 10 seconds 20 ms
        Tested: 10 secs 34 ms || 10 secs 32 ms || 10 secs, 32 ms

        17:19:46.159 -> void setup()
        17:19:50.352 -> ESM activated
        17:19:50.353 -> sendDataI2C
        17:19:50.383 -> transmiting ...
        17:19:50.648 -> 
        17:19:50.648 -> listening ...
        17:20:00.674 -> 
        17:20:00.674 -> Going to sleep ...�

        UPDATE: 
        a) It seems that SAT_A wakes up 3 seconds before the '0 seconds' time :D
        b) SAT_A' sleep time is 9 min, 47 sec on avg --> test it without console prints
        c) SAT_B wakes up 'too early' (12 seconds), I should put it to sleep more

        */
        lastMode = spacecraftMode;
        isCommandReceived = false;  // Reset var

        // Ask SAT_B for mission clock time via I2C
        Wire.requestFrom(8, 8);  // request 8 bytes from peripheral device #8 (device#, bytes) (6 bytes before initDay etc)
        Wire.readBytes((byte *)&rtcData, sizeof rtcData);
        // Ensures reading of all bytes from stream
        while (Wire.available()) {
          Wire.read();
        }

        rtcSynced = rtcData.isSynced;

        // NOTE: real sync should be at rtcData.hours == 06h00, 12h00, 18h00, 00h00
        /* 
          NOTES: 
          1) it may take up to 10+ mins
          2) SAT_A won't wait SAT_B to sync RTC and will execute next line, though SAT_B might be in a loop
          3) Sol. SAT_B could update himseld without command from SAT_A:
            After writting data from ESM mode, if current time = time to update, 
            SAT_B would ignore the current sleep command and start syncing its RTC with GPS.
            After syncing it would RESET and then stay awake, until next ESM command from SAT_A.
          */


        // CHANGE to rtcData.minutes >= 50 in production
                
        // Manage GPS MOSFET: 10 minutes before RTC sync, turn it ON.
        if ((rtcData.hours == 05 || rtcData.hours == 11 || rtcData.hours == 17 || rtcData.hours == 23) && rtcData.minutes >= 40) {
          digitalWrite(GPS_PIN, HIGH);  // Setting pin to HIGH (turn ON)
        }

        // Leave GPS ON 20 mins, since synchronization may take up to five minutes.
        if ((rtcData.hours == 06 || rtcData.hours == 12 || rtcData.hours == 18 || rtcData.hours == 00) && rtcData.minutes >= 10) {
          digitalWrite(GPS_PIN, LOW);  // Setting pin to LOW (turn OFF)
        }


        if (rtcData.minutes == 0 || rtcData.minutes == 10 || rtcData.minutes == 20 || rtcData.minutes == 30 || rtcData.minutes == 40 || rtcData.minutes == 50) {
          // Make sure that only when time is on '0 seconds', execute and save data (gives consistency)
          // Serial.print("\nM2: " + String(rtcData.seconds));

          if (rtcData.seconds >= 0 && rtcData.seconds <= 10) {  // rtcData.seconds == 0 // it is more like a safety check

            hum = dht.readHumidity();
            temp = dht.readTemperature();
            baro = (bme.readPressure() / 100.0F);
            localAltitude = getLocalAltitude();
            computePitchRollYaw();

            digitalWrite(FIVE_VOLT_PIN, LOW);  // Setting pin to LOW (turn OFF)

            // Could be a function 'updateVals()' but then hum, temp, etc, should be global vars
            telemetryData.humidity = hum;
            telemetryData.temperature = temp;
            telemetryData.pressure = baro;
            telemetryData.localAltitude = localAltitude;
            telemetryData.pitch = pitch;
            telemetryData.roll = roll;
            telemetryData.yaw = yaw;

            // Send telemetryData via I2C to peripheral slave for data saving
            sendDataI2C(4);  // SUDO save telemetry data
            sendDataI2C(5);  // SUDO save system data

            // Transmitting 5 seconds
            unsigned long startTime = millis();
            unsigned long endTime = startTime;
            while ((endTime - startTime) <= 5000) {  // Increase it so MCC listens
              transmitData();
              endTime = millis();
            }

            // Listen 5 seconds for incomming MCC commands
            // int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
            int8_t buf[sizeof(commandData)];
            uint8_t buflen = sizeof(buf);

            startTime = millis();
            endTime = startTime;

            while ((endTime - startTime) <= 5000) {  // Listen radio for 5,000 ms while transmitting ack

              if (rf_driver.recv(buf, &buflen)) {
                memcpy(&commandData, buf, sizeof(commandData));

                // Change the SC mode
                executeCommand(commandData.op);  // Execute any command received
                if (commandData.op == 1.0) {
                  isCommandReceived = false;
                } else {
                  isCommandReceived = true;
                }


                // Looks like is receiving even without MCC sending commands !!!
                /*
                  The reason seems to be because MCC sends many times the command 1.0 to Delta-1. Then Delta-1 executes the first time it receives the command, but it kind of receives another one and keeps it in its internal buffer, hence, next time it wakes, is 'receiving' a command, but in reality is the one stucked at buffer.

                  Next time, if MCC isn't sending something, Delta-1 does not execute anything
                */
                // Serial.print("\nOP ");
                // Serial.print(commandData.op);  // 1.00 although MCC sent nothing

                // Transmit an ack message for X seconds (could be the same telemetry data with the SC mode)
                unsigned long startTime2 = millis();
                unsigned long endTime2 = startTime2;
                while ((endTime2 - startTime2) <= 5000) {  // Transmit ack for 5,000 ms
                  systemData.mode = spacecraftMode;        // Update with new mode
                  transmitData();
                  endTime2 = millis();
                }
                break;  // end 'if' and break while loop
              }
              endTime = millis();
            }

            // If no radio signal received, sleep SAT_B 9.7 mins and SAT_A 10 mins, awake and then repeat

            if (isCommandReceived == false) {
              // Send sleep command to SAT_B and then sleep spacecraft
              sendDataI2C(6);  // Send sleep command  to peripheral (9 mins, 4 seconds calculated) and/or sync RTC with GPS
              deepSleep(2);    // sleep SAT_A 9 mins, 50 sec
            }                  // Else, isCommand received is true, and break this case


          }       // end (if 0 >= seconds <= 10)
        } else {  // end (if minutes == 0,10,20,30,40 or 50)
          int sleepSeconds;
          int loops = 0;

          // Test if doesn't affect when waking up
          digitalWrite(FIVE_VOLT_PIN, LOW);  // Setting pin to LOW (turn OFF)

          if (rtcData.minutes > 0 && rtcData.minutes < 9) {
            sleepSeconds = (10 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          } else if (rtcData.minutes > 10 && rtcData.minutes < 19) {
            sleepSeconds = (20 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          } else if (rtcData.minutes > 20 && rtcData.minutes < 29) {
            sleepSeconds = (30 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          } else if (rtcData.minutes > 30 && rtcData.minutes < 39) {
            sleepSeconds = (40 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          } else if (rtcData.minutes > 40 && rtcData.minutes < 49) {
            sleepSeconds = (50 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          } else if (rtcData.minutes > 50 && rtcData.minutes < 59) {
            sleepSeconds = (60 - rtcData.minutes) * 60;
            loops = (sleepSeconds / 2) - 30;  // Wakes up 60 secs before given time
          }

          for (int i = 0; i < loops; i++) {
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
          }
        }  // end else & sleeping

        // call again ESM to check time
        break;
      }
    case 2:  // HIB mode
      {
        // Hibernation (HIB) mode
        lastMode = spacecraftMode;
        isCommandReceived = false;  // Reset var

        digitalWrite(GPS_PIN, LOW);        // Setting pin to LOW (turn OFF)
        digitalWrite(FIVE_VOLT_PIN, LOW);  // Setting pin to LOW (turn OFF)

        telemetryData.humidity = -1.0;
        telemetryData.temperature = -1.0;
        telemetryData.pressure = -1.0;
        telemetryData.localAltitude = -1.0;

        computePitchRollYaw();  // Data that might be of importance

        telemetryData.pitch = pitch;
        telemetryData.roll = roll;
        telemetryData.yaw = yaw;

        // Send telemetryData via I2C to peripheral slave for data saving
        sendDataI2C(9);  // Send system data as SUDO (HIB)

        // Transmitting 5 seconds
        unsigned long startTime = millis();
        unsigned long endTime = startTime;
        while ((endTime - startTime) <= 5000) {
          transmitData();
          endTime = millis();
        }

        // Listen 5 seconds for incomming MCC commands
        // int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
        int8_t buf[sizeof(commandData)];
        uint8_t buflen = sizeof(buf);

        startTime = millis();
        endTime = startTime;

        while ((endTime - startTime) <= 5000) {  // Listen radio for 5,000 ms while transmitting ack

          if (rf_driver.recv(buf, &buflen)) {
            memcpy(&commandData, buf, sizeof(commandData));

            // Change the SC mode
            executeCommand(commandData.op);  // Execute any command received
            if (commandData.op == 3.0) {
              isCommandReceived = false;
            } else {
              isCommandReceived = true;
            }

            // Serial.print("\nOP ");
            // Serial.print(commandData.op);  // 3.00 although MCC sent nothing

            // Transmit an ack message for X seconds (could be the same telemetry data with the SC mode)
            unsigned long startTime2 = millis();
            unsigned long endTime2 = startTime2;
            while ((endTime2 - startTime2) <= 5000) {  // Transmit ack for 5,000 ms
              systemData.mode = spacecraftMode;        // Update with new mode
              transmitData();
              endTime2 = millis();
            }
            break;  // end 'if' and break while loop
          }

          endTime = millis();
        }

        if (isCommandReceived == false) {
          // Send sleep command to SAT_B and then sleep spacecraft
          sendDataI2C(6);  // Send sleep command  to peripheral (9 mins, 4 seconds calculated) and/or sync RTC with GPS
          deepSleep(2);    // sleep SAT_A 9 mins, 50 sec
        }                  // Else, isCommand received is true, and break this case

        break;
      }
    case 3:  // RTI mode
      {
        // Real Time IMU (RTI) mode
        lastMode = spacecraftMode;
        digitalWrite(GPS_PIN, LOW);  // Setting pin to LOW (turn OFF)

        // Read MPU9250 sensor
        computePitchRollYaw();

        telemetryData.humidity = 0.0;
        telemetryData.temperature = 0.0;
        telemetryData.pressure = 0.0;
        telemetryData.localAltitude = 0.0;
        telemetryData.pitch = pitch;
        telemetryData.roll = roll;
        telemetryData.yaw = yaw;

        break;
      }
    case 4:  // RTWI mode
      {
        // Real Time Weather + IMU (RTWI) mode
        lastMode = spacecraftMode;
        digitalWrite(GPS_PIN, LOW);  // Setting pin to LOW (turn OFF)

        // Read both BMP280, DHT22 & MPU9250 sensors (very power hungry)
        hum = dht.readHumidity();
        temp = dht.readTemperature();
        baro = (bme.readPressure() / 100.0F);
        localAltitude = getLocalAltitude();
        computePitchRollYaw();

        telemetryData.humidity = hum;
        telemetryData.temperature = temp;
        telemetryData.pressure = baro;
        telemetryData.localAltitude = localAltitude;
        telemetryData.pitch = pitch;
        telemetryData.roll = roll;
        telemetryData.yaw = yaw;

        break;
      }
  }

  if (spacecraftMode != 0 && spacecraftMode != 1 && spacecraftMode != 2) {
    // Send telemetryData via I2C to peripheral slave. 1: telemetry data, 2 system data (not SUDO)
    sendDataI2C(1);
    sendDataI2C(2);
    transmitData();
    listenReceiver();
  }


  // delay(250);  // <- delay void loop // 250
}  // end void loop

void transmitData() {
  // Auxiliar struct to send all data (max size: 60 bytes)
  struct telemetryStruct {
    // System data:
    int mode = systemData.mode;
    float voltage = systemData.voltage;
    float internalTemp = systemData.internalTemp;
    bool clockSynced = rtcSynced;
    // Boot time data
    int bootDay = initDay;
    int bootMonth = initMonth;
    int bootYear = initYear;
    int bootHour = initHour;
    int bootMinute = initMinute;
    int bootSecond = initSecond;
    // Science payload data:
    float humidity = telemetryData.humidity;
    float temperature = telemetryData.temperature;
    float pressure = telemetryData.pressure;
    float localAltitude = telemetryData.localAltitude;
    float pitch = telemetryData.pitch;
    float roll = telemetryData.roll;
    float yaw = telemetryData.yaw;
  } transferData;

  // byte tran_buf[sizeof(transferData)] = { 0 };  // Buffer for sending telemetry data
  // memcpy(tran_buf, &transferData, sizeof(transferData));
  byte zize = sizeof(transferData);
  // rf_driver.send((uint8_t *)tran_buf, zize);
  rf_driver.send((uint8_t *)&transferData, zize);
  rf_driver.waitPacketSent();
  // Serial.print("\nTX");
}

void listenReceiver() {
  // int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  int8_t buf[sizeof(commandData)];
  uint8_t buflen = sizeof(buf);
  // Turns the receiver on if it not already on, returns true if there's a valid message
  if (rf_driver.recv(buf, &buflen) == 1) {
    memcpy(&commandData, buf, sizeof(commandData));
    executeCommand(commandData.op);
  }
}

void sendDataI2C(int type) {
  /*
  Send data via I2C to peripheral: 
  1 -> telemetryData, 2 -> systemData, 3 -> sleep command, 4 -> sudo telemetryData 5 -> sudo systemData

  WARNING: by default you can only send 32 bytes of data with the Wire.h library.
  I changed it via 'nano' command to 64 bytes
  UPDATE: Arduino UNO buffer is only 32 bytes
  */

  // Auxiliar struct to send all data (max size: 32 bytes)
  // current size: 30 bytes (float: 4 bytes, int: 2 bytes)
  struct auxiliarStruct {
    int header = 1;
    float val1;
    float val2;
    float val3;
    float val4;
    float val5;
    float val6;
    float val7;
  } auxiliarData;

  switch (type) {
    case 1:  // Send RTWI telemetry data
      auxiliarData.header = 1;
      auxiliarData.val1 = telemetryData.humidity;
      auxiliarData.val2 = telemetryData.temperature;
      auxiliarData.val3 = telemetryData.pressure;
      auxiliarData.val4 = telemetryData.localAltitude;
      auxiliarData.val5 = telemetryData.pitch;
      auxiliarData.val6 = telemetryData.roll;
      auxiliarData.val7 = telemetryData.yaw;
      break;
    case 2:  // Send RTWI system data
      auxiliarData.header = 2;
      auxiliarData.val1 = systemData.mode;  // mode is an integer, see if no conflict arises with float of struct
      auxiliarData.val2 = systemData.voltage;
      auxiliarData.val3 = systemData.internalTemp;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 3:  // Safe mode: Send system data as SUDO
      auxiliarData.header = 3;
      auxiliarData.val1 = systemData.mode;  // SUDO save system data: Don't check validTime on SAT_B
      auxiliarData.val2 = systemData.voltage;
      auxiliarData.val3 = systemData.internalTemp;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 4:                     // Send telemetry data as SUDO (ESM)
      auxiliarData.header = 4;  // SUDO save telemetry data: Don't check validTime on SAT_B
      auxiliarData.val1 = telemetryData.humidity;
      auxiliarData.val2 = telemetryData.temperature;
      auxiliarData.val3 = telemetryData.pressure;
      auxiliarData.val4 = telemetryData.localAltitude;
      auxiliarData.val5 = telemetryData.pitch;
      auxiliarData.val6 = telemetryData.roll;
      auxiliarData.val7 = telemetryData.yaw;
      break;
    case 5:  // Send system data as SUDO (ESM)
      auxiliarData.header = 5;
      auxiliarData.val1 = systemData.mode;  // SUDO save system data: Don't check validTime on SAT_B
      auxiliarData.val2 = systemData.voltage;
      auxiliarData.val3 = systemData.internalTemp;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 6:                     // SAT_B to activate sleep mode or sync RTC
      auxiliarData.header = 6;  // activate sleep mode on SAT_B
      auxiliarData.val1 = 0.0;
      auxiliarData.val2 = 0.0;
      auxiliarData.val3 = 0.0;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 7:  // Reboot OBC SAT_B
      auxiliarData.header = 7;
      auxiliarData.val1 = 0.0;
      auxiliarData.val2 = 0.0;
      auxiliarData.val3 = 0.0;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 8:                     // Force SAT_B to deep sleep
      auxiliarData.header = 8;  //Force sleep mode on SAT_B
      auxiliarData.val1 = 0.0;
      auxiliarData.val2 = 0.0;
      auxiliarData.val3 = 0.0;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
    case 9:  // Send system data as SUDO (HIB)
      auxiliarData.header = 10;
      auxiliarData.val1 = systemData.mode;
      auxiliarData.val2 = systemData.voltage;
      auxiliarData.val3 = systemData.internalTemp;
      auxiliarData.val4 = 0.0;
      auxiliarData.val5 = 0.0;
      auxiliarData.val6 = 0.0;
      auxiliarData.val7 = 0.0;
      break;
  }

  // Send data to SAT_B (30 bytes)
  Wire.beginTransmission(8);  // transmit to device #8
  Wire.write((byte *)&auxiliarData, sizeof auxiliarData);
  Wire.endTransmission();  // stop transmitting
}

void deepSleep(int flag) {
  /* Deep Sleep Mode
  Enters power down state for 10 minutes, 1 sec with ADC and BOD modules disabled.
  
  For loop approach -----------------------------------------------
  Current: 0.0703A
  MS vary: 173, 148, 156, 248, 253, 287, 262, 244. Avg: 221.375 ms

  While MS time approach ------------------------------------------
  Current 0.707A

  */

  // 10 mins -> 9 mins, 3 secs with 75 loop
  // adding 8 loops to count for missing 57 secs for a total of 83 loops
  // results: 10 mins, 1 sec, 173 ms || 10 mins, 1 sec, 148, ms || 10 mins, 1 sec, 156 ms || 10 mins, 1 sec, 248  ms
  // || 10 mins, 1 sec, 250  ms || 10 mins, 1 sec, 152  ms

  /* More tests:
  10 mins, 1 sec,
  10 mins, 1 sec, 705 ms,
  10 mins, 2 secs
  10 mins, 2 secs,
  10 mins, 1 secs
  10 mins, 1 secs 

  UPDATE: Sleep loop must be 9 min, 50 seconds to account for the 10 antenna listening and
  have a 'perfect' 10 min loop.

  Trying loop = 81 = 648 sec
  Tests:
  1) x mins, x sec, x ms
  2)

  12:02:18.321 -> Going to sleep ...
  12:11:36.191 -> 9 mins, 18 seconds
  12:11:46.887 -> Going to sleep ...
  12:21:04.727 -> 9 mins, 18 secs
  12:21:15.418 -> Going to sleep ...
  12:30:33.232 -> 

  */

  // delay(20);  // 25 good || 15 = Going to slee�, 10 = Going to, then sleeps and then �����...
  int loop;
  if (flag == 1) {  // Seems like is not being used
    loop = 83;      // for 10 mins, 1 sec
  } else if (flag == 2) {
    loop = 81;  // GOAL: for 9 mins, 50 secs // 77 = 9 mins, 18
    // 81 = 9 mins, 47 sec, 9 mins 47 sec, 9 min, 47 sec,
  }

  for (int i = 0; i < loop; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void (*resetFunc)(void) = 0;  // declare reset fuction at address 0

void executeCommand(float command) {
  /*
  WARNING: When MCC sends a command, SAT_A receives it 3 times on avg,
  hence, if you send a turn ON and then OFF instruction for ESM mode, SAT_A
  would have in its buffer the 'command 1.0' and execute it again
    
  16:33:48.876 -> Command received: 1.00 // <-- To activate ESM
  16:40:02.920 -> Command received: 1.00 // <-- When running ESM loop without MCC sending command
  */

  // Serial.print("\nCommand received: ");
  // Serial.print(command);

  if (command == 0.0) {  // Safe Mode (MCC shouldn't be able to trigger this mode)
    // spacecraftMode = 0; // perhaps I shouldn't have this line (since anyone could send command 0 and trigger it)
  } else if (command == 1.0) {  // Activate ESM mode
    spacecraftMode = 1;
    systemData.mode = spacecraftMode;
    // delay(2000);
    // transmitData();             // To ack
  } else if (command == 2.0) {  // Reset local altitude
    resetPressureGroundLevel();
    // spacecraftMode = 1; // return to default (?)
  } else if (command == 3.0) {  // Activate HIB mode
    spacecraftMode = 2;
  } else if (command == 4.0) {  // Activate RTI mode
    spacecraftMode = 3;
  } else if (command == 5.0) {  // Activate RTWI mode
    spacecraftMode = 4;
  } else if (command == 6.0) {  // Reset OBC and peripheral
    sendDataI2C(7);             // Reboot SAT_B's OBC
    resetFunc();                //call reset
  }
  lastMode = spacecraftMode;
}

float getLocalAltitude() {
  // Ground level height measurement
  return bme.readAltitude(pressureGroundLevel);
}

void resetPressureGroundLevel() {
  // Read ground level pressure when initiating board
  // i.e. 77994.4/100  -> 779.94 hPa
  pressureGroundLevel = (bme.readPressure() / 100.0F);
}

void computePitchRollYaw() {
  IMU.readSensor();
  float accelX = IMU.getAccelX_mss();
  float accelY = IMU.getAccelY_mss();
  float accelZ = IMU.getAccelZ_mss();
  float magX = IMU.getMagX_uT();
  float magY = IMU.getMagY_uT();
  float magZ = IMU.getMagZ_uT();

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
  return 3.7;  // Return volts
}

float getInternalTemperature() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);  //  index 0 refers to the first IC on the wire
  return temp;
}

void checkTriggerSM(int lastMode) {
  /*
  Checks for a trigger condition to activate SafeMode  
  */

  digitalWrite(FIVE_VOLT_PIN, HIGH);  // Always setting pin to HIGH (turn ON)
  delay(200);
  systemData.voltage = getBatteryVoltage();
  systemData.internalTemp = getInternalTemperature();

  /*
  Battery management idea:
  0% - 10% -> Safe Mode
  11% - 35% -> ESM Mode
  35% - 100% -> Any mode (ESM, RTI, RTWI, etc).

  Note: check for the actual battery voltage that
  may enable the safe mode to run at least 12 hours.

  Also check the actual percentages in terms of volts, or an 
  SoC algorithm for batteries.
  */

  if (systemData.voltage <= 2.1) {  // <=10%
    spacecraftMode = 0;
  } else if (systemData.voltage > 2.1 && systemData.voltage <= 2.9) {  // 11% - 35%
    spacecraftMode = 1;                                                // Go to battery efficient ESM mode
  } else if (systemData.voltage > 2.9) {                               // >= 35%
    spacecraftMode = lastMode;
  }

  /* 
  Temperature operations:
    Arduinos: -40 °C to 85 °C
    Adata SD card: -25 °C to 85 °C

  Li-ion Batteries:
    Charge temperature: 0°C to 45°C
    Discharge temperature: –20°C to 60°C
    No charge permitted below freezing.

  */
  if (systemData.internalTemp <= 0.0 || systemData.internalTemp >= 45.0) {  // >= 45.0 degrees
    spacecraftMode = 0;
  } else {
    spacecraftMode = lastMode;
  }
}
