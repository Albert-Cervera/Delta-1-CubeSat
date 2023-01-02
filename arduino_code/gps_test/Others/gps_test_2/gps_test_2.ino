#include "TinyGPS++.h"
#include "SoftwareSerial.h"

SoftwareSerial ss(4, 3); //Rx=11, Tx=10
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);

  // Loop until there is a serial connection between GPS module and Arduino
  while (!ss.available()) {
    delay(1000);
    Serial.println("Loading GPS Connection...");
    ss.begin(9600);
  }
  Serial.println("GPS Connection has been Established");
}

void loop() {
  // Print the number of satellites module is locked into every second
  Serial.println(gps.satellites.value());
  delay(1000);
}