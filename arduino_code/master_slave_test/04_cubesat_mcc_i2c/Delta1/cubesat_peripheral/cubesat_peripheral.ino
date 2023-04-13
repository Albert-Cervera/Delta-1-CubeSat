// Government Agency Original Software Designation: LAR-18832-1
//
//------------------------------------------------------------------------------
// NASA/GSFC, Software Integration & Visualization Office, Code 610.3
//------------------------------------------------------------------------------
//
// MODULES: RTC, SD card adapter, GPS & controller (master) arduino via I2C.
//
//> @author
//> Albert Aarón Cervera Uribe
// e-mail: astra.delta.v@gmail.com
//
// DESCRIPTION:
//> This script contains a testing version of the peripheral (slave)
// device that receive data from a master device and stores data into
// persistant storage with timestamps from RTC.
// The RTC is synced via GPS 4 times a day at 6h00, 12h00, 18h00 and 00h00 UTC.
// In UTC-6: 00h00, 6h00, 12h00, 18h00
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
// 3) Program needs at least 67% of RAM to write all txt
//
// REVISION HISTORY:
// 08 December 2022 - Initial Version
// -- -- 2022 - Modification
// -- -- 2022 - Final first Version
//
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

#include <SD.h>
#include <SPI.h>  // for SD card communication
#include <virtuabotixRTC.h>
#include <Wire.h>
#include "LowPower.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

int lastSecondSavedTel, lastMinuteSavedTel;
int lastSecondSavedSys, lastMinuteSavedSys;
// String latitude, longitude, altitude;

bool startSync = false;  // For RTC-GPS sync
bool synced = false;

// Var to send to SAT_A indicating if a succesful RTC sync was done
bool lastSync = false; 

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
  // 6 bytes until 'seconds'
  uint8_t day;
  uint8_t month;
  int year;
  bool isSynced;
} rtcData;

// Inititialization of instruments
SoftwareSerial ss(4, 3);        // The serial connection to the GPS device: RXPin, TXPin
virtuabotixRTC myRTC(6, 7, 8);  // Real Time Clock

void setup() {
  Serial.begin(9600);

  // Set the current date, and time (monday is day 1, sunday is 7)
  // NOTE: to sync, upload code when real time is about to change to 52 seconds
  // myRTC.setDS1302Time(50, 59, 11, 4, 29, 12, 2022);  // SS, MM, HH, DW, DD, MM, YYYY
  // myRTC.setDS1302Time(00, 10, 19, 6, 31, 12, 2022);  // SS, MM, HH, DW, DD, MM, YYYY

  /* FOR GPS AND RTC TEST:
    1) Set time 10h38
    2) ESM mode should save data at 10h40, 10h50, 11h00
    3) SAT_B will trigger GPS sync at 11h00
    4) Time should be: around 1 am UTC and correct date
    5) Then SAT_B will trigger GPS sync at 2h00, 3h00 UTC

    -------------------------------------------------------------- First test

    // Logs
    19:18:18.713 -> SD_OK
    19:40:29.611 -> 01:18:19 <-- Bad sync (?)
    20:22:22.178 -> 02:22:22 <-- too long
    21:00:12.186 -> 03:00:12 <-- ok
    22:00:12.176 -> 04:00:12 <-- ok

    // ESMSYS
    time,date,mode,voltage,internalTemp
    12:00:00,29/12/2022,1,3.70,21.00
    12:00:00,29/12/2022,1,3.70,20.94
    10:40:00,29/12/2022,1,3.70,21.06 <-- test begins
    10:50:00,29/12/2022,1,3.70,20.94
    11:00:00,29/12/2022,1,3.70,20.88 <-- last local RTC and then GPS sync
    01:30:00,06/01/2023,1,3.70,20.69 <-- new GPS time
    01:40:00,06/01/2023,1,3.70,20.38
    01:50:00,06/01/2023,1,3.70,20.69
    02:00:00,06/01/2023,1,3.70,20.31 <-- 40 mins til next record: it started syncing, took 22 mins, 2h30 should happened
    02:40:00,06/01/2023,1,3.70,20.31
    02:50:00,06/01/2023,1,3.70,20.31
    03:00:00,06/01/2023,1,3.70,20.38 <-- perfect sync
    03:10:00,06/01/2023,1,3.70,20.25
    03:20:00,06/01/2023,1,3.70,20.13
    03:30:00,06/01/2023,1,3.70,20.19
    03:40:00,06/01/2023,1,3.70,20.06
    03:50:00,06/01/2023,1,3.70,19.69
    04:00:00,06/01/2023,1,3.70,19.75 <-- perfect sync
    04:10:00,06/01/2023,1,3.70,20.00
    04:20:00,06/01/2023,1,3.70,20.06

    -------------------------------------------------------------- Second test

    // Logs
    22:43:15.932 -> SD_OK
    23:05:26.029 -> 04:43:16 <-- Bad sync (?)
    23:22:21.189 -> 05:22:21 <-- too long --- it happened again (why?)
    00:00:11.168 -> 06:00:11 <-- ok
    01:00:12.203 -> 07:00:12 <-- ok
    02:00:12.212 -> 08:00:12 <-- ok
    03:00:11.191 -> 09:00:11 <-- ok
    04:00:12.168 -> 10:00:12 <-- ok
    05:00:11.181 -> 11:00:11 <-- ok
    06:00:11.162 -> 12:00:11 <-- ok
    07:00:11.183 -> 13:00:11 <-- ok
    08:00:11.201 -> 14:00:11 <-- ok

    // ESMSYS
    time,date,mode,voltage,internalTemp
    10:40:00,29/12/2022,1,3.70,19.38 <-- test begins
    10:50:00,29/12/2022,1,3.70,20.00
    11:00:00,29/12/2022,1,3.70,20.25 <-- last local RTC and then GPS sync
    05:00:00,06/01/2023,1,3.70,20.44 <-- new GPS time (though first GPS time was 4:43:16 UTC)
    05:40:00,06/01/2023,1,3.70,20.44 <-- 40 mins from last record: it started syncing, took 22 mins (5h00 - 5h22), 5h30 should happened
    05:50:00,06/01/2023,1,3.70,20.25
    06:00:00,06/01/2023,1,3.70,20.25 <-- perfect sync (06:00:11)
    06:10:00,06/01/2023,1,3.70,20.19
    06:20:00,06/01/2023,1,3.70,20.19
    06:30:00,06/01/2023,1,3.70,20.06
    06:40:00,06/01/2023,1,3.70,20.13
    06:50:00,06/01/2023,1,3.70,20.00
    07:00:00,06/01/2023,1,3.70,20.00 <-- perfect sync (07:00:12)
    07:10:00,06/01/2023,1,3.70,20.00
    07:20:00,06/01/2023,1,3.70,20.00
    07:30:00,06/01/2023,1,3.70,19.94
    07:40:00,06/01/2023,1,3.70,19.94
    07:50:00,06/01/2023,1,3.70,19.88
    08:00:00,06/01/2023,1,3.70,19.88 <-- perfect sync (08:00:12)
    08:10:00,06/01/2023,1,3.70,19.88
    08:20:00,06/01/2023,1,3.70,19.88
    08:30:00,06/01/2023,1,3.70,19.81
    08:40:00,06/01/2023,1,3.70,19.81
    08:50:00,06/01/2023,1,3.70,19.81
    09:00:00,06/01/2023,1,3.70,19.81 <-- perfect sync (09:00:11)
    09:10:00,06/01/2023,1,3.70,19.75
    09:20:00,06/01/2023,1,3.70,19.69
    09:30:00,06/01/2023,1,3.70,19.75
    09:40:00,06/01/2023,1,3.70,19.75
    09:50:00,06/01/2023,1,3.70,19.69
    10:00:00,06/01/2023,1,3.70,19.69 <-- perfect sync (10:00:12)
    10:10:00,06/01/2023,1,3.70,19.63
    10:20:00,06/01/2023,1,3.70,19.69
    10:30:00,06/01/2023,1,3.70,19.63
    10:40:00,06/01/2023,1,3.70,19.63
    10:50:00,06/01/2023,1,3.70,19.56
    11:00:00,06/01/2023,1,3.70,19.56 <-- perfect sync (11:00:11)
    11:10:00,06/01/2023,1,3.70,19.56
    11:20:00,06/01/2023,1,3.70,19.50
    11:30:00,06/01/2023,1,3.70,19.50
    11:40:00,06/01/2023,1,3.70,19.50
    11:50:00,06/01/2023,1,3.70,19.44
    12:00:00,06/01/2023,1,3.70,19.44 <-- perfect sync (12:00:11)
    12:10:00,06/01/2023,1,3.70,19.44
    12:20:00,06/01/2023,1,3.70,19.38
    12:30:00,06/01/2023,1,3.70,19.38
    12:40:00,06/01/2023,1,3.70,19.38
    12:50:00,06/01/2023,1,3.70,19.38
    13:00:00,06/01/2023,1,3.70,19.31 <-- perfect sync (13:00:11)
    13:10:00,06/01/2023,1,3.70,19.31
    13:20:01,06/01/2023,1,3.70,19.31 <-- 1 second difference
    13:30:00,06/01/2023,1,3.70,19.31
    13:40:00,06/01/2023,1,3.70,19.31
    13:50:00,06/01/2023,1,3.70,19.31
    14:00:00,06/01/2023,1,3.70,19.25 <-- perfect sync (14:00:11)
    14:10:00,06/01/2023,1,3.70,19.25
    14:20:00,06/01/2023,1,3.70,19.31

    -------------------------------------------------------------- Third test
    Set the RTC with UTC time and wrong date and let GPS sync at the designated times.
    1) Experiment starting time: 12h10 Jan 6, 2023 UTC-6 or 18h10 UTC 
    2) It'll pass 5 hours 50 mins of ESM data recording before GPS sync that should be at 00h00 UTC (18h00 UTC-6)
    3) I'll leave it running for 4 days (my poor Macbook's battery :c)

    12:09:59.120 -> SD_OK    <-- Friday Jan 6 (local)
    18:00:10.264 -> 00:00:10
    00:00:11.206 -> 06:00:11 <-- Saturday Jan 7 (local)
    06:00:11.178 -> 12:00:11
    12:00:11.333 -> 18:00:11
    18:00:11.234 -> 00:00:11
    00:00:11.251 -> 06:00:11 <-- Sunday Jan 8 (local)
    06:00:11.252 -> 12:00:11
    12:00:11.337 -> 18:00:11
    18:00:11.191 -> 00:00:11
    00:00:11.202 -> 06:00:11 <-- Monday Jan 9 (local)
    06:00:11.198 -> 12:00:11
    12:00:11.408 -> 18:00:11
    18:00:12.265 -> 00:00:12
    00:00:11.243 -> 06:00:11 <-- Tuesday Jan 10 (local)
    06:00:11.309 -> 12:00:11
    12:00:11.238 -> 18:00:11
    18:00:11.237 -> 00:00:11 
    -- After this point, Satellite is disconnected from Mac and connected to power outlet

  */
  // myRTC.setDS1302Time(00, 38, 10, 4, 29, 12, 2022); // SS, MM, HH, DW, DD, MM, YYYY
  // myRTC.setDS1302Time(00, 10, 18, 4, 29, 12, 2022);  // SS, MM, HH, DW, DD, MM, YYYY

  Wire.begin(8);                 // join i2c bus with address #8,you can define whatever address you want like '4'
  Wire.onRequest(requestEvent);  // register event
  Wire.onReceive(receiveEvent);  // register event (To receive commands)

  if (!SD.begin(10)) {
    Serial.print("ERR_SD");
    while (1)
      ;
  }
  Serial.print("\nSD_OK");
}

void loop() {

  // String timestamp = getTimestampTime();
  // Serial.print("\nTimestamp: ");
  // Serial.print(timestamp);
  // String timestampDate = getTimestampDate();
  // Serial.print(", " + timestampDate);

  /*
  Idea: Leave here the synchronization of RTC with GPS
  'always running' (test if SAT_B still sleeps)
  and when is valid time to sync, then sync
  */
  if (startSync) {
    syncGPSDateTime(true);  // check if SAT_B still falls asleep
    startSync = false;

    // String timestamp = getTimestampTime();
    // Serial.print("\n" + timestamp);

    // String timestampDate = getTimestampDate();
    // Serial.print(", " + timestampDate);

    // getGPSData();
  }
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
  return setTwoDigits(myRTC.dayofmonth) + "/" + setTwoDigits(myRTC.month) + "/" + String(myRTC.year);
}

String getTimestampTime() {
  myRTC.updateTime();
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

// Checks if is the correct UTC time to initiate RTC-GPS synchronization
bool validTimeToSync() {  
  if ((myRTC.hours == 06 || myRTC.hours == 12 || myRTC.hours == 18 || myRTC.hours == 00) && myRTC.minutes == 00 && synced == false) {
    return true;
  } else {
    synced = false;
    return false;
  }
}

// TODO: Add function that creates a new txt file when date.month changes.
void writeToSD(int flag) {
  // Just one file can be open at a time, make sure to close it after writing.
  switch (flag) {
    case 1:  // Save RTWI telemetry data: Checking validTime
      if (validTimeToSave(1)) {
        writeFile(1, "2_RTWI/RTWITEL.txt");
      }
      break;
    case 2:  // Save RT system data: Checking validTime
      if (validTimeToSave(2)) {
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
    case 10:  // HIB mode: SUDO save system data: Don't check validTime
      // Serial.print("\nM1");
      writeFile(6, "3_HIB/HIBSYS.txt");
      break;
  }
}

void writeFile(int flag, String fileName) {
  // 3-4-5 are SUDO! 1-2 need to check time
  File myFile;
  String time = getTimestampTime();
  String date = getTimestampDate();
  myFile = SD.open(fileName, FILE_WRITE);

  if (flag == 1 || flag == 4) {  // save telemetry
    if (myFile) {
      
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

      if (flag == 1) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("\nERR txt 1-4");
    }
  } else if (flag == 2 || flag == 3 || flag == 5 || flag == 6) {
    //save system data
    // Serial.print("\nM2");
    if (myFile) {
      // Serial.print("\nM3");

      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(String(int(auxiliarDataLog.val1)) + ",");
      myFile.print(String(auxiliarDataLog.val2) + ",");
      myFile.print(String(auxiliarDataLog.val3) + ",");
      myFile.print(String(auxiliarDataLog.val4));
      myFile.print("\n");

      myFile.close();

      if (flag == 2) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("\nERR txt 2-3-5-6");
    }
  }
  /*
  else if (flag == 6) {  // Save GPS data
    if (myFile) {
      // Serial.print(" m2");
      // Serial.print(time);

      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(latitude + ",");
      myFile.print(longitude + ",");
      myFile.print(altitude);
      myFile.print("\n");

      myFile.close();

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("\nERR txt gps");
    }
  }
  */
}

/*
function that executes whenever data is requested by master.
Registered as event, this function returns RTC data to SAT_A.
*/
void requestEvent() {

  // Serial.print("\nmyRTC.year: " + String(myRTC.year));

  myRTC.updateTime();
  rtcData.hours = myRTC.hours;
  rtcData.minutes = myRTC.minutes;
  rtcData.seconds = myRTC.seconds;
  rtcData.day = myRTC.dayofmonth;
  rtcData.month = myRTC.month;
  rtcData.year = myRTC.year;
  rtcData.isSynced = lastSync;
  // rtcData.isSynced = true;

  
  // Serial.print("\nRE: " + String(rtcData.isSynced) );
  Wire.write((byte *)&rtcData, sizeof rtcData);
}

void (*resetFunc)(void) = 0;  // declare reset fuction at address 0

/* receiveEvent()
  function that executes whenever data is received from master
  this function is registered as an event, see setup().
*/
void receiveEvent() {
  Wire.readBytes((byte *)&auxiliarDataLog, sizeof auxiliarDataLog);  // 30 bytes

  if (auxiliarDataLog.header == 6) {  // Activate deep sleep OR sync RTC (ESM mode)
    /*
    IDEA: SAT_B could update himself without command from SAT_A:
          After writting data from ESM mode, if current time = time to update, 
          SAT_B would ignore the current sleep command and start syncing its RTC with GPS.
          After syncing it would RESET and then stay awake, until next ESM command from SAT_A.
    NOTE: Maximum 9 mins of GPS signal acquisition
    */
    if (validTimeToSync()) {  // Sync with GPS and skip sleep
      startSync = true;
    } else {
      deepSleep();
    }
  } else if (auxiliarDataLog.header == 7) {  // reboot this OBC
    resetFunc();                             //call reset
  } else if (auxiliarDataLog.header == 8) {  // Activate deep sleep (SAFE mode)
    deepSleep();
  } else if (auxiliarDataLog.header == 9) {  // TODO: Force an RTC update (from MCC command)
    startSync = true;
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

// Gets UTC time via GPS and syncs RTC
void syncGPSDateTime(bool strictMode) {
  TinyGPSPlus gps;  // The TinyGPS++ object
  ss.begin(9600);
  lastSync = false; // re-start original val

  int day, month, year, hour, minute, second;
  // String latitude, longitude, altitude;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  bool gpsDate, gpsTime = false;

  // 9 mins = 540 secs = 540000 ms
  // 3 mins = 180 secs = 180000 ms
  // 5 mins = 300 secs = 300000 ms
  while ((endTime - startTime) <= 300000) {  // Try to sync 5 mins
    if (ss.available() > 0) {
      if (gps.encode(ss.read())) {

        if (strictMode == true) {

          // Strict Mode: Requires a fixed location to sync time
          if (gps.location.isValid()) {

            if (gps.date.isValid()) {
              day = gps.date.day();
              month = gps.date.month();
              year = gps.date.year();
              gpsDate = true;
            }

            if (gps.time.isValid()) {
              hour = gps.time.hour();
              minute = gps.time.minute();
              second = gps.time.second();
              gpsTime = true;
            }

            if (gpsDate && gpsTime) {
              if (day > 0 && day <= 31 && month <= 12 && year >= 2023) {
                // Update RTC with GPS UTC time
                myRTC.setDS1302Time(second, minute, hour, 1, day, month, year);  // SS, MM, HH, DW, DD, MM, YYYY
                synced = true;
                lastSync = true;
                break;
              }
            }

          }  // End if location

        } else {

          // Loose Mode: Syncs with whatever value Satellites provides
          if (gps.date.isValid()) {
            day = gps.date.day();
            month = gps.date.month();
            year = gps.date.year();
            gpsDate = true;
          }

          if (gps.time.isValid()) {
            hour = gps.time.hour();
            minute = gps.time.minute();
            second = gps.time.second();
            gpsTime = true;
          }


          if (gpsDate && gpsTime) {
            if (day > 0 && day <= 31 && month <= 12 && year >= 2023) {
              // Update RTC with GPS UTC time
              myRTC.setDS1302Time(second, minute, hour, 1, day, month, year);  // SS, MM, HH, DW, DD, MM, YYYY
              synced = true;
              lastSync = true;
              break;
            }
          }
        }  // End else

      }  // end if gps.encode(ss.read())
    }    // end if ss.available()

    endTime = millis();
  }  // end while 3 or 9 mins
  ss.end();
  // Serial.print("\nLS: " + String(lastSync));
}


/*
// Gets latitude, longitude and altitude data from GPS
void getGPSData() {
  TinyGPSPlus gps;  // The TinyGPS++ object
  // ss.begin(9600); // add its end
  
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  // 5 mins = 300 sec
  while ((endTime - startTime) <= 300000) {
    if (ss.available() > 0) {
      if (gps.encode(ss.read())) {

        if (gps.location.isValid()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
          altitude = gps.altitude.meters();

          if (gps.altitude.meters() != 0.0) {
            writeFile(6, "GPSSYNC.txt");
            break;
          }
        }

      }
    }

    endTime = millis();
  }
}
*/
