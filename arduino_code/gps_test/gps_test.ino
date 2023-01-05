#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
// static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
// TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  // ss.begin(GPSBaud);
  ss.begin(9600);
  Serial.print(F("Testing TinyGPS++ library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());  
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.

  // if (millis() > 5000 && gps.charsProcessed() < 10) {
  //   Serial.println(F("No GPS detected: check wiring."));
  //   while (true)
  //     ;
  // }

  // while (ss.available() > 0) {
  //   if (gps.encode(ss.read())) {
  //     displayInfo();
  //   }
  // }  // end while

  getGPSTime();

  delay(600000);
}

// void displayInfo() {
//   Serial.print(F("Location: "));
//   if (gps.location.isValid()) {
//     Serial.print(gps.location.lat(), 6);
//     Serial.print(F(","));
//     Serial.print(gps.location.lng(), 6);
//   } else {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F("  Date/Time: "));
//   if (gps.date.isValid()) {
//     Serial.print(gps.date.month());
//     Serial.print(F("/"));
//     Serial.print(gps.date.day());
//     Serial.print(F("/"));
//     Serial.print(gps.date.year());
//   } else {
//     Serial.print(F("INVALID"));
//   }

//   Serial.print(F(" "));
//   if (gps.time.isValid()) {
//     if (gps.time.hour() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.hour());
//     Serial.print(F(":"));
//     if (gps.time.minute() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.minute());
//     Serial.print(F(":"));
//     if (gps.time.second() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.second());
//     Serial.print(F("."));
//     if (gps.time.centisecond() < 10) Serial.print(F("0"));
//     Serial.print(gps.time.centisecond());
//   } else {
//     Serial.print(F("INVALID"));
//   }

//   Serial.println();
// }

void getGPSTime() {
  TinyGPSPlus gps;
  uint8_t day, month, hour, minute, second;
  uint16_t year;  // cast data: (uint8_t *)data16
  bool gpsDate, gpsTime;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;

  // 9 mins = 540 secs
  while ((endTime - startTime) <= 540000) {
    if ((ss.available() > 0) && gps.encode(ss.read())) {
    //if ((ss.available() > 0)) {
      // gps.encode(ss.read());

      if (gps.date.isValid()) {
        day = gps.date.day();
        month = gps.date.month();
        year = gps.date.year();    
        gpsDate = true;
      } else {
        Serial.print(F("\nNo valid date"));
        gpsDate = false;
      }

      if (gps.time.isValid()) {
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();
        // centisecond = gps.time.centisecond();
        gpsTime = true;
      } else {
        Serial.print(F("\nNo valid time"));
        gpsTime = false;
      }

      if(gpsDate && gpsTime){

        Serial.print("\nTime: ");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.print(second);

        Serial.print(" , Date: ");
        Serial.print(day);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(year);

        if(day != 0 && month <= 12 && year >= 2023) {
          Serial.print("\nBreak");
          // break; // leave the 9 mins loop
        }         
      }
    } // end if
    endTime = millis();
  }
}