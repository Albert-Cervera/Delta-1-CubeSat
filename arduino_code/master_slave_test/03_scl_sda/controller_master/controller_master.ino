// Wire Controller Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI peripheral device
// Refer to the "Wire Peripheral Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.

#include <Wire.h>

// byte x = 0;
byte command = 0;  // For sending commands to peripheral device (slave Arduino)

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {

  /*
  Idea: the requestFrom() method can be used to request telemetry data,
  or just somew basic status information. For routine execution (i.e. commands)
  is better to use Wire.write(data) with begin and end transmissions.
  */

  // Wire.requestFrom(8, 6);  // request 6 bytes from peripheral device #8
  // while (Wire.available()) {  // peripheral may send less than requested
  //   char c = Wire.read();     // receive a byte as character
  //   Serial.print(c);          // print the character
  // }

  // ------------------------------------------------------------------------------

  // From example:
  // Wire.beginTransmission(4);  // transmit to device #4
  // Wire.write("x is ");        // sends five bytes
  // Wire.write(x);              // sends one byte
  // Wire.endTransmission();     // stop transmitting
  // x++;

  // Mine:

  // command = 0;

  Serial.print("\nSending command 0 to peripheral ...\n");
  Wire.beginTransmission(8);  // transmit to device #8
  Wire.write(command);        // sends one byte
  Wire.endTransmission();     // stop transmitting

  // Now we need a way to receive the asked data with the command from peripheral 

  // while (Wire.available()) {  // peripheral may send less than requested
  //   char c = Wire.read();     // receive a byte as character
  //   Serial.print(c);          // print the character
  // }

  /*
  Seems like waiting for data retrieval that way does not work (further investigation required).
  An approach would be to use the requestFrom() method and expect to receive an array as:
  
    Binary bool data 
    {0,0,1,1,0} = {sdCardWritten, runningClock,imuPowerState}
  
  Or something similar, further research needed so slave can transfer complex data to master,
  such as SD card contents, or RTC timestamps.
  */


  // delay(2000); // Wait 2 seconds to change and send second command
  // command = 1;
  // Wire.beginTransmission(8); // transmit to device #8
  // Wire.write(command); // sends one byte
  // Wire.endTransmission(); // stop transmitting

  delay(1500);
}