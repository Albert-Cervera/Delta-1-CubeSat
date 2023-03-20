#include <SoftwareSerial.h>
 
const byte rxPin = 7; // Wire this to Tx Pin of ESP8266 // 2
const byte txPin = 6; // Wire this to Rx Pin of ESP8266 // 3
 
// We'll use a software serial interface to connect to ESP8266
SoftwareSerial ESP8266 (rxPin, txPin);
 
void setup() {
  Serial.begin(9600);
 
// You may need to uncomment this block for the first run:
 ESP8266.begin(115200); // Change this to the factory baudrate used by ESP8266
 delay(1000);

 Serial.println("Setting BAUDRATE to 9600");
 ESP8266.println("AT+IPR=9600");
//

ESP8266.begin(9600); 

// Serial.begin(115200);
// ESP8266.begin(115200); 

// Command: 
// AT+CWLAP
// AT+RST
  
}
bool okReceived = false;
 
void loop() {
 
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    Serial.println("Command Sent: " + command);
    Serial.println();
    ESP8266.println(command);
  }
 
  int responseCounter = 0;
  if (ESP8266.available() > 0)
  {
    while (ESP8266.available() > 0)
    {
      if (responseCounter == 0)
      {
        Serial.println("Response Received:");
      }
      
      String response = ESP8266.readStringUntil('\n');
      Serial.println(response);
      responseCounter++;
    }
    Serial.println();
    Serial.println("============");
    Serial.println();
  }
 
  
}