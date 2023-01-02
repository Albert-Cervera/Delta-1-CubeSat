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
// In particular this is the CubeSat peripheral device.
//
// Notation:
//
// SAT_A: CubeSat controller (master) Arduino.
// SAT_B: CubeSat peripheral (slave) Arduino.
//
// Objective:
//
// SAT_A sends intruments data via I2C to SAT_B whose work is
// to write this data into persistant storage.
//
//
// NOTES & WARNINGS
// SAT_A and SAT_B are connected the following way:
// 1) SDA --> SDA
// 2) SCL --> SCL
// 3) SAT_A's 5v -> SAT_B's Vin
// 4) SAT_A's GND -> SAT_B's GND
//
// ISSUE:
// 1) Program storage space should not pass 80% or it won't execute properly.
// 2) Energy received into SAT_B's Vin from SAT_A's 5v may not be enough to
//    power external loads (instruments).
//
// REVISION HISTORY:
// 08 December 2022 - Initial Version
// -- -- 2022 - Modification
// -- -- 2022 - Final first Version
//
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

// #include <SPI.h>
#include <SD.h>
#include <virtuabotixRTC.h>
#include <Wire.h>
#include "LowPower.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

String date;
String time;
int lastSecondSavedTel, lastMinuteSavedTel;
int lastSecondSavedSys, lastMinuteSavedSys;

/*
auxiliarStruct: the idea is to copy every received struct
from controller via I2C and depending on its header, save
the data into its corresponding txt file.

Note: the controller would use an auxiliary struct to copy 
the desired data, in case that data is less than 30 bytes
it will zero pad it.
*/

// 30 bytes (4 bytes per float + 1 header)
// Auxiliar struct to send all data (max size: 32 bytes)
struct auxiliarStruct {
  int header;
  float val1;
  float val2;
  float val3;
  float val4;
  float val5;
  float val6;
  float val7;  // <-- as the last one on telemetryStruct (max from controller)
} auxiliarDataLog;

struct missionClockStruct {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} rtcData;

// Inititialization of instruments
virtuabotixRTC myRTC(6, 7, 8);  // Real Time Clock
SoftwareSerial ss(4, 3);        // The serial connection to the GPS device: RXPin, TXPin

void setup() {
  Serial.begin(9600);

  // Set the current date, and time (monday is day 1, sunday is 7)
  // NOTE: to sync, upload code when real time is about to change to 52 seconds
  // myRTC.setDS1302Time(50, 59, 11, 4, 29, 12, 2022);  // SS, MM, HH, DW, DD, MM, YYYY
  // myRTC.setDS1302Time(00, 10, 19, 6, 31, 12, 2022);  // SS, MM, HH, DW, DD, MM, YYYY

  Wire.begin(8);                 // join i2c bus with address #8,you can define whatever address you want like '4'
  Wire.onRequest(requestEvent);  // register event
  Wire.onReceive(receiveEvent);  // register event (To receive commands)
  ss.begin(9600);                // initialize GPS module at GPSBaud 9600

  if (!SD.begin(10)) {
    Serial.print("ERR SD");  // reduce this message to bare minimum
    while (1)
      ;
  }
  Serial.print("SD OK");
}

void loop() {

  // String timestamp = getTimestampTime();
  // Serial.print("\nTimestamp: ");
  // Serial.print(timestamp);
  // String timestampDate = getTimestampDate();
  // Serial.print(", " + timestampDate);

  // delay(1000);
}

String setTwoDigits(uint8_t val) {
  if (val < 10) {
    return "0" + String(val);
  } else {
    return String(val);
  }
}

String getTimestampDate() {
  myRTC.updateTime();
  // date = setTwoDigits(myRTC.dayofmonth) + "/" + setTwoDigits(myRTC.month) + "/" + String(myRTC.year);
  return setTwoDigits(myRTC.dayofmonth) + "/" + setTwoDigits(myRTC.month) + "/" + String(myRTC.year);
}

String getTimestampTime() {
  myRTC.updateTime();
  // time = setTwoDigits(myRTC.hours) + ":" + setTwoDigits(myRTC.minutes) + ":" + setTwoDigits(myRTC.seconds);
  return setTwoDigits(myRTC.hours) + ":" + setTwoDigits(myRTC.minutes) + ":" + setTwoDigits(myRTC.seconds);
}

// This assumes constant querying to RTC to know exact time so we don't miss second 0 of each hour.
bool validTimeToSave(int flag) {
  myRTC.updateTime();
  switch (flag) {
    case 1:
      if (myRTC.seconds == lastSecondSavedTel && myRTC.minutes == lastMinuteSavedTel) {
        return false;
      }
      break;
    case 2:
      if (myRTC.seconds == lastSecondSavedSys && myRTC.minutes == lastMinuteSavedSys) {
        return false;
      }
      break;
  }
  // if (myRTC.minutes == 0 && myRTC.seconds == 0) { // Saves every hour
  // if (myRTC.seconds == 0) {  // saves every minute
  // if (myRTC.seconds == 0 || myRTC.seconds == 10 || myRTC.seconds == 20 || myRTC.seconds == 30 || myRTC.seconds == 40 || myRTC.seconds == 50) {  // saves every 10 seconds
  if (myRTC.seconds == 0 || myRTC.seconds == 5 || myRTC.seconds == 10 || myRTC.seconds == 15 || myRTC.seconds == 20 || myRTC.seconds == 25 || myRTC.seconds == 30 || myRTC.seconds == 35 || myRTC.seconds == 40 || myRTC.seconds == 45 || myRTC.seconds == 50 || myRTC.seconds == 55) {
    return true;
  }
  return false;
}

// Checks if is the correct time to initiate RTC-GPS synchronization
bool validTimeToSync() {
  if ((myRTC.hours == 06 || myRTC.hours == 12 || myRTC.hours == 18 || myRTC.hours == 00) && myRTC.minutes == 00) {
    return true;
  } else {
    return false;
  }
}

// TODO: Add function that creates a new txt file when date.month changes.
void writeToSD(int flag) {
  // Just one file can be open at a time, make sure to close it after writing.
  // bool canWrite;

  switch (flag) {
    case 1:  // Save RTWI telemetry data: Checking validTime
      //canWrite = validTimeToSave(1);
      if (validTimeToSave(1)) {
        writeFile(1, "2_RTWI/RTWITEL.txt");
      }
      break;
    case 2:  // Save RT system data: Checking validTime
      // canWrite = validTimeToSave(2);
      if (validTimeToSave(2)) {
        // Serial.print("\nSave system data\n");
        writeFile(2, "2_RTWI/RTSYS.txt");
      }
      break;
    case 3:  // Safe mode: SUDO save system data
      writeFile(3, "0_SM/SMSYS.txt");
      break;
    case 4:  // ESM mode: SUDO save telemetry data: Don't check validTime
      writeFile(4, "1_ESM/ESMTEL.txt");
      break;
    case 5:  // ESM mode: SUDO save system data: Don't check validTime
      writeFile(5, "1_ESM/ESMSYS.txt");
      break;
  }

  // delay(750);  // Complete for a full second in sending data to slave.
}

void writeFile(int flag, String fileName) {
  // 3-4-5 are SUDO! 1-2 need to check time
  File myFile;  // File for data handling
  time = getTimestampTime();
  date = getTimestampDate();

  // Serial.print("\n");
  // Serial.print(time);
  // Serial.print(" , " + date);

  if (flag == 1 || flag == 4) {  // save telemetry
    myFile = SD.open(fileName, FILE_WRITE);
    if (myFile) {
      // Serial.print("\nWriting file ...");

      // Similar to a CSV format
      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(String(auxiliarDataLog.val1) + ",");
      myFile.print(String(auxiliarDataLog.val2) + ",");
      myFile.print(String(auxiliarDataLog.val3) + ",");
      myFile.print(String(auxiliarDataLog.val4) + ",");
      myFile.print(String(auxiliarDataLog.val5) + ",");
      myFile.print(String(auxiliarDataLog.val6) + ",");
      myFile.print(String(auxiliarDataLog.val7));
      myFile.print("\n");

      myFile.close();
      // Serial.println(" done.");

      if (flag == 1) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("ERR txt 1-4");
    }
  } else if (flag == 2 || flag == 3 || flag == 5) {  //save system data
    myFile = SD.open(fileName, FILE_WRITE);
    if (myFile) {
      //Serial.print("Writing file ...");
      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(String(int(auxiliarDataLog.val1)) + ",");
      myFile.print(String(auxiliarDataLog.val2) + ",");
      myFile.print(String(auxiliarDataLog.val3));
      myFile.print("\n");

      myFile.close();
      // Serial.println("done.");

      if (flag == 2) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("ERR txt 2-3-5");
    }
  }
}

/*
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
*/

/*
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
*/

/*
function that executes whenever data is requested by master.
Registered as event, this function returns RTC data to SAT_A.
*/
void requestEvent() {
  myRTC.updateTime();
  rtcData.hours = myRTC.hours;
  rtcData.minutes = myRTC.minutes;
  rtcData.seconds = myRTC.seconds;
  Wire.write((byte *)&rtcData, sizeof rtcData);
}

/*
  receiveEvent()
  function that executes whenever data is received from master
  this function is registered as an event, see setup().

  TODO: receive command along with weather data
*/
void receiveEvent() {
  // Serial.print("\nreceiveEvent triggered!\n");
  Wire.readBytes((byte *)&auxiliarDataLog, sizeof auxiliarDataLog);  // 30 bytes

  if (auxiliarDataLog.header == 6) {  // Activate deep sleep OR sync RTC (ESM mode)
    /*
    IDEA: SAT_B could update himseld without command from SAT_A:
          After writting data from ESM mode, if current time = time to update, 
          SAT_B would ignore the current sleep command and start syncing its RTC with GPS.
          After syncing it would RESET and then stay awake, until next ESM command from SAT_A.
    NOTE: Maximum 9 mins of GPS signal acquisition
    */
    if (validTimeToSync()) {
      // syncGPSDateTime();
      deepSleep(); // delete me
    } else {
      deepSleep();
    }
  } else if (auxiliarDataLog.header == 7) {  //Activate deep sleep (SAFE mode)
    deepSleep();
  } else if (auxiliarDataLog.header == 8) {  // Force an RTC update (from MCC command)
    // sJJyncGPSDateTime();
  } else {
    writeToSD(auxiliarDataLog.header);  // Write to txt file
  }

  // Ensures reading of all bytes from stream
  while (Wire.available()) {
    Wire.read();
  }
}

void deepSleep() {
  /* Deep Sleep Mode for SAT_B
  Enters power down state with ADC and BOD modeules disabled.
  Period time should be less than the 10 mins, 1 sec SAT_A is sleep
  Current in sleep mode: 0.0683A = 68.3 mA

  Actual time in for loop is greater for the actual loop ms time.
  

  73 loops of 8 secs = 9 mins 44 secs (584) // SAT_B wakes up 16 seconds before SAT_A sleep ends
  Tests:
  1) 9 mins, 52 secs, 117 ms
  2) 9 mins, 55 secs -> leaving roughly 5 seconds before SAT_B wakes up

  UPDATE: I need to reduce its time, given the fact that sleep time for Arduino_A is gonna
  be 9 min 50 seconds and not the 10 minutes straight.

  GOAL: 9 mins 45 sec

  68 loops of 8 secs = 9 mins, 4 sec (544 secs): real x sec
  With 71 loops, it wakes 12 seconds before the 81 of SAT_B (9 mins 47 sec)
  In theory with 72, it should wake up 4 secs before SAT_A.
  If I add a SLEEP_2S should wake up 2 secs before SAT_A

  */

  // delay(25); // 50 ms
  for (int i = 0; i < 72; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);  // So it wakes up 2 secs before SAT_A
}

// Reset fuction at address 0
// void (*resetFunc)(void) = 0;

// void syncGPSDateTime() {
//   Serial.print("\nGPS");
// }


/*
void syncGPSDateTime() {
  // I need to leave 512 bytes of RAM for SD card

  // Serial.print("\nGPS");
  TinyGPSPlus gps;  // The TinyGPS++ object

  uint8_t day, month, hour, minute, second;
  uint16_t year;  // cast data: (uint8_t *)data16
  bool gpsDate, gpsTime = false;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;

  // 9 mins = 540 secs
  while ((endTime - startTime) <= 540000) {
    if ((ss.available() > 0) && gps.encode(ss.read())) {

      // if (gps.time.isValid()) {
      //   hour = gps.time.hour();
      //   minute = gps.time.minute();
      //   second = gps.time.second();
      //   gpsTime = true;
      //   Serial.print("\n");
      //   Serial.print(second);
      // }

      // if (gps.date.isValid()) {
      //   day = gps.date.day();
      //   month = gps.date.month();
      //   year = gps.date.year();
      //   gpsDate = true;
      // }

      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();
      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();

      // Serial.print("\n:");
      // Serial.print(second);

      if (year != 2000) {
        Serial.print("\n:");
        Serial.print(second);
        break;  // leave the 9 mins loop
      }

      // if (gpsDate && gpsTime) {

      // Serial.print("\nTime: ");
      // Serial.print(hour);
      // Serial.print(":");
      // Serial.print(minute);
      // Serial.print("\n:");
      // Serial.print(second);

      // Serial.print(" , Date: ");
      // Serial.print(day);
      // Serial.print("/");
      // Serial.print(month);
      // Serial.print("/");
      // Serial.print(year);


      // if (year != 2000) {  // day != 0 && month != 0 && year != 2000
      //   // Update RTC with GPS time
      //   // uint8_t YYYY = (uint8_t *)year; // casting to uint8
      //   // myRTC.setDS1302Time(second, minute, hour, 1, day, month, YYYY);  // SS, MM, HH, DW, DD, MM, YYYY
      //   Serial.print("\nBk");
      //   break;  // leave the 9 mins loop
      // } // end if month != 0


      //}  // end if gpsTime && gpsDate
    }
    endTime = millis();
  }  // end while 9 mins
}
*/

