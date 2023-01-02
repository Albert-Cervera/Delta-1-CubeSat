#include <LiquidCrystal.h>
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library
#include <SPI.h>

// Create Amplitude Shift Keying Object
RH_ASK rf_driver; // PIN 11 by default

// IMPORTANT: Data pin of receiver should be at PIN 11 (it magically works with library)

/*
use constructor arguments to configure different pins, eg:
\code
RH_ASK driver(2000, 2, 4, 5);
\endcode
Which will initialise the driver at 2000 bps, recieve on GPIO2, transmit on GPIO4, PTT on GPIO5
*/

//Change Pins
// RH_ASK rf_driver(2000, 7, 8, 0); // speed in bps, rxPin, txPin, pttPin

#define RS 12 // 12
#define EN 13 // 13 
#define D4 5
#define D5 4
#define D6 3
#define D7 2

unsigned long lastDataReceived; // miliseconds with no signal from transmitter
unsigned long currentMillis; // current miliseconds var to compare data
int losTolerance = 3000; // Loss of Signal tolerance in ms

struct dataStruct{
  float humidity ; 
  float temperature;  
}h22Data;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {

  // Initialize ASK Object
  rf_driver.init();
  // Setup Serial Monitor
  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("NASA - NanoSat 1");
  delay(2000);
  lcd.clear();
  lastDataReceived = 0;
}

void loop() {
  
  // Receiver antenna code -----------------------------
  // Set buffer to size of expected message
  
  int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);
  

  // Serial.print("buflen: ");
  // Serial.print(buflen); // 60
  // Serial.print('\n');
  

  // Serial.print("rf_driver.recv(buf, &buflen): ");
  // Serial.print(rf_driver.recv(buf, &buflen)); // '0' when data is not received, '1 with data'
  // Serial.print('\n');  

  // Non-blocking
  if (rf_driver.recv(buf, &buflen) == 1) {   
    lastDataReceived = millis();     
    memcpy(&h22Data, buf, sizeof(h22Data));
    
    Serial.print("RECEIVED DATA: ");
    // Serial.print(sizeof(h22Data));
    // Serial.print('\n'); 

    Serial.print("humidity: ");
    Serial.print(h22Data.humidity);    
    Serial.print("  temperature: ");
    Serial.print(h22Data.temperature);
    Serial.print('\n'); 

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Hum: ");
    lcd.setCursor(5, 0);
    lcd.print(h22Data.humidity);
    lcd.setCursor(10, 0);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.setCursor(6, 1);
    lcd.print(h22Data.temperature);
    lcd.setCursor(11, 1);
    lcd.print((char)223);
    lcd.setCursor(12, 1);
    lcd.print("C");

  }   
  else if ((rf_driver.recv(buf, &buflen) == 0)){
    currentMillis = millis();

    Serial.print("currentMillis: ");
    Serial.print(currentMillis);
    Serial.print('\n');
    Serial.print("lastDataReceived: ");
    Serial.print(lastDataReceived);
    Serial.print('\n');
    Serial.print("Difference: ");
    Serial.print(currentMillis - lastDataReceived);
    Serial.print('\n');

    // 3 seconds tolerance
    if(currentMillis - lastDataReceived >= losTolerance) {
      Serial.print("\nNo data");
    
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("[!!!] No data");
      lcd.setCursor(0, 1);
      lcd.print("from Cubesat");
    }
       
  }
  
  //delay(250);
  delay(500);
}  // end void loop