// Wire Peripheral Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI peripheral device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  Wire.begin(8);                 // join i2c bus with address #8 // You can define wharever address you want like '4'
  Wire.onRequest(requestEvent);  // register event
  Wire.onReceive(receiveEvent);  // register event (To receive commands)
  Serial.begin(9600);            // start serial for output
}

void loop() {
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write("hello ");  // respond with message of 6 bytes
  // as expected by master
}

/*
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) 30 {
  while (1 < Wire.available())  // loop through all but the last
  {
    char c = Wire.read();  // receive byte as a character
    Serial.print(c);       // print the character
  }
  int x = Wire.read();  // receive byte as an integer
  Serial.println(x);    // print the integer
}
*/

void receiveEvent() {
  // Serial.print("recveiveEvent() triggered! \n");

  // while (1 <= Wire.available())  // loop through all but the last
  // {
  //   char command = Wire.read();  // receive byte as a character
  //   Serial.print(command);       // print the character
  // }

  // Good one!
  // if( 1<= Wire.available()) {
  //   byte command = Wire.read();
  //   Serial.print("\nCommand inside: ");
  //   Serial.print(command);
  // }

  byte command = Wire.read();
  Serial.print("\nCommand as a whole: ");
  Serial.print(command);

  if (command == 0) {
    Wire.write("command 0");  // Send 9 bytes
  } else if (command == 1) {
    Wire.write("command 1");  // Send 9 bytes
  }
}