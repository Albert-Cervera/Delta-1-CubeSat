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

#include <SPI.h>
#include <SD.h>
#include <ThreeWire.h>  // RTC
#include <RtcDS1302.h>  // RTC
#include <Wire.h>
#include "LowPower.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

String date;
String time;
int lastSecondSavedTel, lastMinuteSavedTel;
int lastSecondSavedSys, lastMinuteSavedSys;
File myFile;      // File for data handling
TinyGPSPlus gps;  // The TinyGPS++ object

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

// // 28 bytes size
// struct telemetryStruct {
//   float humidity;
//   float temperature;
//   float pressure;
//   float localAltitude;  // altitude in meters from ground
//   float pitch;
//   float roll;
//   float yaw;
// } telemetryDataLog;

// // 6 bytes size
// struct dataStruct {
//   int mode;
//   float voltage;
//   // float internalTemp; // dedicated temperature DSB18B20 sensor
//   // float solarCurrent; // Future val: current generated by solar arrays
// } systemDataLog;

struct missionClockStruct {
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} rtcData;

// Inititialization of instruments
ThreeWire myWire(7, 6, 8);  // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);
SoftwareSerial ss(4, 3);  // The serial connection to the GPS device: RXPin, TXPin

void setup() {
  Serial.begin(9600);
  Rtc.Begin();

  // NOTE: to sync, upload code when real time is about to change to 53 seconds
  // RtcDateTime("Oct 31 2024", "17:25:05") example
  // RtcDateTime definedTime = RtcDateTime("Dec 16 2022", "18:25:00");
  // Rtc.SetDateTime(definedTime);

  Wire.begin(8);                 // join i2c bus with address #8,you can define whatever address you want like '4'
  Wire.onRequest(requestEvent);  // register event
  Wire.onReceive(receiveEvent);  // register event (To receive commands)
  ss.begin(9600);                // initialize GPS module at GPSBaud 9600

  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");  // reduce this message to bare minimum
    while (1)
      ;
  }
  Serial.println(" initialization done.");
  // syncGPSDateTime();  //Get GPS time 3 seconds and update RTC // 02/4 ,29/12/2150,43.40,17.50,781.89,0.29,-0.30,1.15,111.96
}

void loop() {

  // RtcDateTime now = Rtc.GetDateTime();

  // String timestampDate = getTimestampDate(now);
  // String timestamp = getTimestampTime(now);

  // Serial.print("\nTimestamp: ");
  // Serial.print("\n");
  // Serial.print(timestamp);
  // Serial.print(", " + timestampDate);

  // readSD(1);
  // syncGPSDateTime();

  delay(1000);
}

#define countof(a) (sizeof(a) / sizeof(a[0]))  // For time and date

String getTimestampDate(const RtcDateTime& dt) {
  char date[11];
  snprintf_P(date,
             countof(date),
             PSTR("%02u/%02u/%04u"),
             dt.Day(),
             dt.Month(),
             dt.Year());
  return date;
}

String getTimestampTime(const RtcDateTime& dt) {
  char time[9];
  snprintf_P(time,
             countof(time),
             PSTR("%02u:%02u:%02u"),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  return time;
}

// This assumes constant querying to RTC to know exact time so we don't miss second 0 of each hour.
bool validTimeToSave(int flag, const RtcDateTime& dt) {
  byte seconds = dt.Second();
  byte minutes = dt.Minute();
  switch (flag) {
    case 1:
      if (seconds == lastSecondSavedTel && minutes == lastMinuteSavedTel) {
        return false;
      }
      break;
    case 2:
      if (seconds == lastSecondSavedSys && minutes == lastMinuteSavedSys) {
        return false;
      }
      break;
  }
  // if (minutes == 0 && seconds == 0) { // Saves every hour
  // if (seconds == 0) {  // saves every minute
  // if (seconds == 0 || seconds == 10 || seconds == 20 || seconds == 30 || seconds == 40 || seconds == 50) {  // saves every 10 seconds
  if (seconds == 0 || seconds == 5 || seconds == 10 || seconds == 15 || seconds == 20 || seconds == 25 || seconds == 30 || seconds == 35 || seconds == 40 || seconds == 45 || seconds == 50 || seconds == 55) {
    return true;
  }
  return false;
}

// TODO: Add function that creates a new txt file when date.month changes.
void writeToSD(int flag) {
  // Just one file can be open at a time, make sure to close it after writing.
  bool canWrite;

  // RtcDateTime now = Rtc.GetDateTime();
  // date = getTimestampDate(now);
  // time = getTimestampTime(now);
  // byte seconds = now.Second();
  // byte minutes = now.Minute();

  RtcDateTime now = Rtc.GetDateTime();
  byte seconds = now.Second();
  byte minutes = now.Minute();

  // Serial.print("\n");
  // Serial.print(time);
  // Serial.print(" , " + date);

  switch (flag) {

    case 1:  // Save RTWI telemetry data: Checking validTime
      canWrite = validTimeToSave(1, now);
      if (canWrite) {
        Serial.print("\nSave telemetry data\n");
        myFile = SD.open("2_RTWI/RTWITEL.txt", FILE_WRITE);

        Serial.print("\nlook file");
        Serial.print(myFile);

        if (myFile) {
          Serial.print("\nWriting file ...");

          // Similar to a CSV format

          date = getTimestampDate(now);
          time = getTimestampTime(now);


          myFile.print(time + ",");  // 01:3/4  instead of 01:34
          myFile.print(date + ",");
          // Serial.print("\nok");
          myFile.print(String(auxiliarDataLog.val1) + ",");
          myFile.print(String(auxiliarDataLog.val2) + ",");
          myFile.print(String(auxiliarDataLog.val3) + ",");
          myFile.print(String(auxiliarDataLog.val4) + ",");
          myFile.print(String(auxiliarDataLog.val5) + ",");
          myFile.print(String(auxiliarDataLog.val6) + ",");
          myFile.print(String(auxiliarDataLog.val7));
          myFile.print("\n");

          myFile.close();
          // Serial.println("done.");

          lastSecondSavedTel = seconds;
          lastMinuteSavedTel = minutes;

        } else {
          // TODO: consider trying to initialize the sd card again
          Serial.println("\n[ !!! ] Error opening txt file 1");
        }
      }
      break;
    case 2:  // Save RT system data: Checking validTime
      canWrite = validTimeToSave(2, now);
      if (canWrite) {
        // Serial.print("\nSave system data\n");
        myFile = SD.open("2_RTWI/RTSYS.txt", FILE_WRITE);
        if (myFile) {
          // Serial.print("Writing file ...");

          myFile.print(time + ",");
          myFile.print(date + ",");
          myFile.print(String(int(auxiliarDataLog.val1)) + ",");
          myFile.print(String(auxiliarDataLog.val2) + ",");
          myFile.print(String(auxiliarDataLog.val3));
          myFile.print("\n");

          myFile.close();
          // Serial.println("done.");

          lastSecondSavedSys = seconds;
          lastMinuteSavedSys = minutes;
        } else {
          Serial.println("[ !!! ] Error opening txt file 2");
        }
      }
      break;
    case 3:  // Safe mode: SUDO save system data
      myFile = SD.open("0_SM/SMSYS.txt", FILE_WRITE);
      if (myFile) {
        // Serial.print("Writing file ...");

        myFile.print(time + ",");
        myFile.print(date + ",");
        myFile.print(String(int(auxiliarDataLog.val1)) + ",");
        myFile.print(String(auxiliarDataLog.val2) + ",");
        myFile.print(String(auxiliarDataLog.val3));
        myFile.print("\n");

        myFile.close();
        // Serial.println("done.");

        // lastSecondSavedSys = seconds;
        // lastMinuteSavedSys = minutes;
      } else {
        Serial.println("[ !!! ] Error opening txt file");
      }

      break;
    case 4:  // ESM mode: SUDO save telemetry data: Don't check validTime
      myFile = SD.open("1_ESM/ESMTEL.txt", FILE_WRITE);

      if (myFile) {
        // Serial.print("Writing file ...");

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
        // Serial.println("done.");

        lastSecondSavedTel = seconds;
        lastMinuteSavedTel = minutes;

      } else {
        // TODO: consider trying to initialize the sd card again
        Serial.println("[ !!! ] Error opening txt file");
      }
      break;
    case 5:  // ESM mode: SUDO save system data: Don't check validTime
      myFile = SD.open("1_ESM/ESMSYS.txt", FILE_WRITE);
      if (myFile) {
        // Serial.print("Writing file ...");

        myFile.print(time + ",");
        myFile.print(date + ",");
        myFile.print(String(int(auxiliarDataLog.val1)) + ",");
        myFile.print(String(auxiliarDataLog.val2) + ",");
        myFile.print(String(auxiliarDataLog.val3));
        myFile.print("\n");

        myFile.close();
        // Serial.println("done.");

        lastSecondSavedSys = seconds;
        lastMinuteSavedSys = minutes;
      } else {
        Serial.println("[ !!! ] Error opening txt file");
      }

      break;
  }

  // delay(750);  // Complete for a full second in sending data to slave.
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

      riteToSDxs(1);

      break;
    case 2:
      // Save other data (IMU)
      break;
  }
}
*/

/*
function that executes whenever data is requested by master
this function is registered as an event, see setup()
TODO: Send hours and minutes (at least minutes)
*/
void requestEvent() {
  RtcDateTime now = Rtc.GetDateTime();
  byte hours = now.Hour();
  byte seconds = now.Second();
  byte minutes = now.Minute();

  rtcData.hours = hours;
  rtcData.minutes = minutes;
  rtcData.seconds = seconds;

  Wire.write((byte*)&rtcData, sizeof rtcData);
}

/*
  receiveEvent()
  function that executes whenever data is received from master
  this function is registered as an event, see setup().

  TODO: receive command along with weather data
*/
void receiveEvent() {
  // Serial.print("\nreceiveEvent triggered!\n");

  Wire.readBytes((byte*)&auxiliarDataLog, sizeof auxiliarDataLog);  // 30 bytes
  if (auxiliarDataLog.header == 6) {
    // Activate deep sleep
    // Serial.println("\nGoing to sleep ...");
    deepSleep();
    // Serial.println("\nHey I just woke up");
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

void syncGPSDateTime() {
  uint8_t day, month, hour, minute, second;
  uint16_t year;
  bool gpsDate, gpsTime;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;

  // Get GPS time 3 seconds and update RTC
  // while ((endTime - startTime) <= 3000) {
  while ((ss.available() > 0) && ((endTime - startTime) <= 3000)) {
    if (gps.encode(ss.read())) {

      if (gps.date.isValid()) {
        day = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();
        gpsDate = true;
      } else {
        Serial.print(F("No valid date"));
        gpsDate = false;
      }

      if (gps.time.isValid()) {
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        // centisecond = gps.time.centisecond();
        gpsTime = true;
      } else {
        Serial.print(F("No valid time"));
        gpsTime = false;
      }
    }
    endTime = millis();
  }

  //endTime = millis();
  //}  // end while 3 seec

  if (gpsDate && gpsTime) {
    // Serial.print("\nsec: ");
    // Serial.print(second);
    // Serial.print("\nmin: ");
    // Serial.print(minute);
    // Serial.print("\nhour: ");
    // Serial.print(hour);

    // Sync onboard RTC
    // myRTC.setDS1302Time(second, minute, hour, 1, day, month, 2024);  //day of the week: 1 since I have no reference
    // myRTC.setDS1302Time(00, 20, 15, 1, 25, 12, 2024); // SS, MM, HH, DW, DD, MM, YYYY
    //Serial.print("\n[OK] RTC synced: ");
    // Serial.print(getTimestampTime());
    // delay(500);
  }
}
