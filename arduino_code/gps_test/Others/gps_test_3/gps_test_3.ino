#include "SD.h"
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

File myFile;
int RXPin = 4;
int TXPin = 3;

TinyGPSPlus gps;
SoftwareSerial SerialGPS(RXPin, TXPin);

String Latitude, Longitude, Altitude, day, month, year, hour, minute, second, Date, Time, Data;

void setup() {
  Serial.begin(9600);
    while (!Serial) {
    ; 
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  Serial.println("Creating GPS_data.txt...");
  myFile = SD.open("GPS_data.txt", FILE_WRITE);
  if (myFile) {
    myFile.println( "Latitude, Longitude, Altitude, Date and Time \r\n");
    myFile.close();

  } else {
    Serial.println("error opening GPS_data.txt");
  }
 SerialGPS.begin(9600);
}

void loop() {
  while (SerialGPS.available() > 0)
    if (gps.encode(SerialGPS.read()))
      obtain_data();
      
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("GPS NOT DETECTED!");
    while(true);
  }

}

void obtain_data()
{
  if (gps.location.isValid())
  {
    Latitude = gps.location.lat();
    Longitude = gps.location.lng();
    Altitude = gps.altitude.meters();
  }
  else
  {
    Serial.println("Location is not available");
  }
  
  if (gps.date.isValid())
  {
    month = gps.date.month();
    day = gps.date.day();
    year = gps.date.year();
    Date = day + "-" + month + "-" + year;
     Serial.println(Date);
  }
  else
  {
    Serial.print("Date Not Available");
  }
 
  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) ;
    hour = gps.time.hour();
    if (gps.time.minute() < 10);
    minute = gps.time.minute();
    if (gps.time.second() < 10) ;
    second = gps.time.second();
    Time = hour + ":" + minute + ":" + second;
  Serial.println(Time);
  }
  else
  {
    Serial.print("Time Not Available");
  }
  
  Data = Latitude + "," + Longitude + "," + Altitude + "," + Date + "," + Time;
  Serial.print("Save data: ");
  Serial.println(Data);
  myFile = SD.open("GPS_data.txt", FILE_WRITE);

  if (myFile) {
    Serial.print("GPS logging to GPS_data.txt...");
    myFile.println(Data);
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening GPS_data.txt");
  } 
 Serial.println();
 delay(10000);
}

