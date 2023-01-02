// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
RH_ASK rf_driver; // PIN 11 by default
// RH_ASK rf_driver(2000, 0, 8, 11); // speed in bps, rxPin, txPin, pttPin
 
void setup()
{        
    // Initialize ASK Object
    rf_driver.init();

    // Setup Serial Monitor
    Serial.begin(9600);
    
    Serial.print("\nSetup done");

}
 
void loop()
{
    // Set buffer to size of expected message
    uint8_t buf[11];
    uint8_t buflen = sizeof(buf);
    // Serial.print("\nWaiting data ...");
    // Check if received packet is correct size
    if (rf_driver.recv(buf, &buflen))
    {
      
      // Message received with valid checksum
      Serial.print("Message Received: ");
      Serial.println((char*)buf);         
    }
    delay(1000);
}