// Government Agency Original Software Designation: LAR-18832-1
//
//------------------------------------------------------------------------------
// NASA/GSFC, Software Integration & Visualization Office, Code 610.3
//------------------------------------------------------------------------------
//
// MODULES: RTC, SD card adapter & controller (master) arduino via I2C.
//
//> @author
//> Albert Aarón Cervera Uribe
//
// DESCRIPTION:
//> This script contains a testing version of the peripheral (slave)
// device that receive data from a master device and stores data into
// persistant storage with timestamps from RTC.
//
// Notation:
//
// MCC_A: MCC controller (master) Arduino.
// MCC_B: MCC peripheral (slave) Arduino.
//
// Objective:
//
// MCC_A received weather & IMU data from Delta1, then sends this data
// to MCC_B (with RTC and SD card connected) to write data into persistant
// storage.
//
// NOTES & WARNINGS
// MCC_A and MCC_B are connected the following way:
// 1) SDA --> SDA
// 2) SCL --> SCL
// 3) MCC_A's 5v -> MCC_B's Vin
// 4) MCC_A's GND -> MCC_B's GND
//
// ISSUE:
// 1) Program storage space should not pass 80% or it won't execute properly.
// 2) Energy received into MCC_B's Vin from MCC_A's 5v may not be enough to
//    power external loads (instruments).
//
// REVISION HISTORY:
// 01 December 2022 - Initial Version
// -- -- 2022 - Modification
// -- -- 2022 - Final first Version
//
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

#include <SPI.h>
#include <SD.h>
#include <virtuabotixRTC.h>
#include <Wire.h>

String date;
String time;
int lastSecondSaved;
int lastMinuteSaved;
File myFile;  // File for meteorological data from Delta1

struct dataStruct {
  // int mode;
  // float voltage;
  float humidity;
  float temperature;
  float pressure;
  float localAltitude;  // altitude in meters from ground
  float pitch; 
  float roll;
  float yaw; 
} telemetryDataLog;

struct missionClockStruct{
  String date;
  String time;
}rtcData;

// Inititialization of instruments
virtuabotixRTC myRTC(6, 7, 8);  // Real Time Clock

void setup() {

  Serial.begin(9600);

  Wire.begin(8);                 // join i2c bus with address #8,you can define whatever address you want like '4'
  Wire.onRequest(requestEvent);  // register event
  Wire.onReceive(receiveEvent);  // register event (To receive commands)

  // Set the current date, and time (monday is day 1, sunday is 7)
  // myRTC.setDS1302Time(00, 25, 13, 1, 5, 12, 2022); // SS, MM, HH, DW, DD, MM, YYYY

  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1)
      ;
  }
  Serial.println(" initialization done.");
}

void loop() {

  String timestamp = getTimestampTime();
  Serial.print("\nTimestamp: \n");
  Serial.print(timestamp);

  // readSD(1);

  delay(1000);
}

String setTwoDigits(int val) {
  if (val < 10) {
    return "0" + String(val);
  } else {
    return String(val);
  }
}

String getTimestampDate() {
  myRTC.updateTime();
  date = setTwoDigits(myRTC.dayofmonth) + '/' + setTwoDigits(myRTC.month) + '/' + setTwoDigits(myRTC.year);
  return date;
}

String getTimestampTime() {
  myRTC.updateTime();
  time = setTwoDigits(myRTC.hours) + ':' + setTwoDigits(myRTC.minutes) + ':' + setTwoDigits(myRTC.seconds);
  return time;
}

// This assumes constant querying to RTC to know exact time so we don't miss second 0 of each hour.
bool validTimeToSave() {
  myRTC.updateTime();
  date = getTimestampDate();
  time = getTimestampTime();

  // Serial.print("\n");
  // Serial.print(time);
 
  // Concurrency problem: same second called twice
  // 17:10:11.396 -> Call writeToSD(1) here :)
  // 17:10:11.915 -> Call writeToSD(1) here :)

  if (myRTC.seconds == lastSecondSaved && myRTC.minutes == lastMinuteSaved) {
    // Serial.print("\nThis was called more than once at the same second");
    return false;
  } 
  

  // if (myRTC.minutes == 0 && myRTC.seconds == 0) { // Saves every hour
  if (myRTC.seconds == 0) {  // saves every minute
  // if (myRTC.seconds == 0 || myRTC.seconds == 10 || myRTC.seconds == 20 || myRTC.seconds == 30 || myRTC.seconds == 40 || myRTC.seconds == 50) {  // saves every 10 seconds
  // if (myRTC.seconds == 0 || myRTC.seconds == 5 || myRTC.seconds == 10 || myRTC.seconds == 15 || myRTC.seconds == 20 || myRTC.seconds == 25 || myRTC.seconds == 30 || myRTC.seconds == 35 || myRTC.seconds == 40 || myRTC.seconds == 45 || myRTC.seconds == 50 || myRTC.seconds == 55) {
    return true;
  }

  return false;
}

// TODO: Add function that creates a new txt file when date.month changes.
void writeToSD(int flag) {
  
  lastSecondSaved = myRTC.seconds;
  lastMinuteSaved = myRTC.minutes;

  // Serial.print("\nCall writeToSD(1) here :)");    

  // Just one file can be open at a time, make sure to close it after writing.
  switch (flag) {
    case 1:
      Serial.print("\nSave telemetry data\n");
      myFile = SD.open("deltaone.txt", FILE_WRITE);  // Looks like can't have 'log' added to its name

      if (myFile) {
        Serial.print("Writing file ...");

        // Similar to a CSV format
        myFile.print(time + ",");
        myFile.print(date + ",");
        // myFile.print(String(telemetryDataLog.mode) + ",");
        // myFile.print(String(telemetryDataLog.voltage) + ",");
        myFile.print(String(telemetryDataLog.humidity) + ",");
        myFile.print(String(telemetryDataLog.temperature) + ",");
        myFile.print(String(telemetryDataLog.pressure) + ",");
        myFile.print(String(telemetryDataLog.localAltitude) + ",");
        myFile.print(String(telemetryDataLog.pitch) + ",");
        myFile.print(String(telemetryDataLog.roll) + ",");
        myFile.print(String(telemetryDataLog.yaw));
        myFile.print("\n");

        myFile.close();
        Serial.println("done.");

      } else {
        Serial.println("[ !!! ] Error opening txt file");
      }
      break;
    case 2:
      Serial.println("Save IMU data (?)");
      break;
  }

  delay(750); // Complete for a full second in sending data to slave.
}

void readSD(int flag) {
  switch (flag) {
    case 1:
      // Read txt file
      myFile = SD.open("deltaone.txt");

      if (myFile) {
        Serial.println("weather2.txt:");
        // read from the file until there's nothing else in it:
        while (myFile.available()) {
          Serial.write(myFile.read());
        }
        myFile.close();
      } else {
        Serial.println("[ !!! ] Error opening weather txt file");
      }

      break;
    case 2:
      // Read other file
      break;
  }
}

void executeCommand(int command) {
  switch (command) {
    case 1:
      // Save weather data to SD card

      // TODO: weather data should come from master via I2C
      // telemetryDataLog.humidity = 43.40,
      // telemetryDataLog.temperature = 21.10;
      // telemetryDataLog.pressure = 781.03;
      // telemetryDataLog.localAltitude = 14.18;
      // telemetryDataLog.timestamp = getTimestamp();

      writeToSD(1);

      break;
    case 2:
      // Save other data (IMU)
      break;
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  // Wire.write("23:55:42");  // respond with message of 6 bytes, as expected by master

  // // Update RTC time, send it as struct
  // rtcData.date = getTimestampDate();
  // rtcData.time = getTimestampTime();

  // Wire.write((byte *)&rtcData, sizeof rtcData);
  // Wire.endTransmission(); // stop transmitting
  
}

/*
  receiveEvent()
  function that executes whenever data is received from master
  this function is registered as an event, see setup().

  TODO: receive command along with weather data
*/
void receiveEvent() {
  // Serial.print("receiveEvent triggered! \n");


  // if (!telemetryDataLog.humidity) {
  //   telemetryDataLog.humidity = 0.0 / 0.0;
  //   telemetryDataLog.temperature = 0.0 / 0.0;
  // }
  // if (!telemetryDataLog.pressure) {
  //   telemetryDataLog.pressure = 0.0 / 0.0;
  //   telemetryDataLog.localAltitude = 0.0 / 0.0;
  // }

  // Serial.print("\ntelemetryDataLog struct: \n");
  // Serial.print(telemetryDataLog.humidity);
  // Serial.print("\n");
  // Serial.print(telemetryDataLog.temperature);
  // Serial.print("\n");
  // Serial.print(telemetryDataLog.pressure);
  // Serial.print("\n");
  // Serial.print(telemetryDataLog.localAltitude);
  // Serial.print("\n");

  // Serial.print("Calling canWrite\n");
  bool canWrite = validTimeToSave();
  if (canWrite) {
    Wire.readBytes((byte*)&telemetryDataLog, sizeof telemetryDataLog);
    writeToSD(1);  // Write to txt file    
  }
  
  // Ensures reading of all bytes from stream
  while (Wire.available()) {
    Wire.read();
  } 
}
