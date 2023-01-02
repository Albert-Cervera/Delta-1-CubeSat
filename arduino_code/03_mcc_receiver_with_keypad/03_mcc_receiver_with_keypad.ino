#include <LiquidCrystal.h>
#include <RH_ASK.h> // Include RadioHead Amplitude Shift Keying Library
#include <SPI.h> // Include dependant SPI Library
#include <Keypad.h>

// Create Amplitude Shift Keying Object
RH_ASK rf_driver(2000, 11, 6, 0); // Params: speed in BPS, rxPin, txPin, pttPin <-- Receive on PIN 11 and send on PIN 6 (Before A5,A4)

#define RS 12 // 12
#define EN 13 // 13 
#define D4 5
#define D5 4
#define D6 3
#define D7 2

unsigned long lastDataReceived; // miliseconds with no signal from transmitter
unsigned long currentMillis; // current miliseconds var to compare data
int losTolerance = 2000; // Loss of Signal tolerance in ms
char key;
char lastKey;
char option;
float heatIndex;
int dataMode = 1;

// Keypad definition
const byte ROWS = 4; 
const byte COLS = 4; 
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {10, 9, 8, 7};  // 9 to 6 in example, 10 to 7 digital in real
byte colPins[COLS] = {A0, A1, A2, A3};  // 5 to 2 in example, A0 to A3 in real

struct telemetryStruct{
  float humidity; 
  float temperature; 
  float pressure;
  float localAltitude; // altitude in meters from ground
  float pitch; 
  float roll;
  float yaw;  
}telemetryData;

struct commandStruct{
  float op;   
}commandData;

byte tx_buf[sizeof(commandData)] = {0}; // buffer for sending command data

// INITIALIZATION OF COMPONENTS
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setup() {
  // Setup Serial Monitor
  Serial.begin(9600);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("NASA - NanoSat 1");
  delay(2000);
  lcd.clear();
  lastDataReceived = 0;
  key = 'A';

  // Initialize ASK Object
  rf_driver.init();
}

void loop() {
  
  // Receiver antenna code -----------------------------
  // Set buffer to size of expected message
  int8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  // Serial.print("rf_driver.recv(buf, &buflen): ");
  // Serial.print(rf_driver.recv(buf, &buflen)); // '0' when data is not received, '1 with data'
  // Serial.print('\n');  

  char customKey = customKeypad.getKey();
  
  if (customKey){
    // Serial.println(customKey);
    key = customKey;
  }

  // Non-blocking
  if (rf_driver.recv(buf, &buflen) == 1) {
    // If data received ...
    lastDataReceived = millis();
    memcpy(&telemetryData, buf, sizeof(telemetryData));     
    
    // switch(dataMode) {
    //   case '1':
    //     memcpy(&telemetryData, buf, sizeof(telemetryData));
    //     break;
    //   case '2':
    //     memcpy(&imuData, buf, sizeof(imuData));
    //     break;
    // }

    switch(key) {
      case 'A':
        // Show temp and hum info
        lastKey = 'A';
        // sendCommand(1.0);
        Serial.print("\nShow temperature");
        // Serial.print("RECEIVED DATA: ");
        // Serial.print("humidity: ");
        // Serial.print(telemetryData.humidity);    
        // Serial.print("  temperature: ");
        // Serial.print(telemetryData.temperature);
        // Serial.print('\n'); 
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Hum: ");
        lcd.setCursor(5, 0);
        lcd.print(telemetryData.humidity);
        lcd.setCursor(11, 0);
        lcd.print("%");

        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.setCursor(6, 1);
        lcd.print(telemetryData.temperature);
        lcd.setCursor(12, 1);
        lcd.print((char)223);
        lcd.setCursor(13, 1);
        lcd.print("C");

        break;
      case 'B':
        // Show barometer info
        lastKey = 'B';
        Serial.print("\nShow pressure");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Barometer: ");
        lcd.setCursor(0, 1);        
        lcd.print(telemetryData.pressure);
        lcd.setCursor(7, 1);
        lcd.print("hPa");

        break;
      case 'C':
        lastKey = 'C';
        Serial.print("\nC was pressed");
        Serial.print("\nShow IMU telemetry");

        // lcd.clear();
        // lcd.setCursor(0, 0);
        // lcd.print("Pitch, Roll, Yaw");
        // delay(1000);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("X:");
        lcd.setCursor(2, 0);            
        lcd.print(telemetryData.pitch);
        lcd.setCursor(8, 0);
        lcd.print((char)223);

        lcd.setCursor(10, 0);
        lcd.print("Y:");
        lcd.setCursor(0, 1);            
        lcd.print(telemetryData.roll);
        lcd.setCursor(6, 1);
        lcd.print((char)223);

        lcd.setCursor(7, 1);
        lcd.print("Z:");
        lcd.setCursor(9, 1);            
        lcd.print(telemetryData.yaw);
        lcd.setCursor(15, 1);
        lcd.print((char)223);
        
      
        break;
      case 'D':
        lastKey = 'D';
        Serial.print("\nD was pressed");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("D) Invalid Op.");                
        break;
      case '*':        
        switch(lastKey) {
          case 'A': 
            // A*: Compute and show heat index
            Serial.print("\nA *");
            Serial.print("\nShow heat index");
            
            heatIndex = computeHeatIndex(telemetryData.temperature, telemetryData.humidity);
            // heatIndex = computeHeatIndex(35.00, 86.00); //61 c            
            // test 80 and 112 F = 26.6 and 44.4 C and hum <13% => condition 1
            // 26 and 30 c and R>85 => condition 2
            
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Heat Index: ");
            lcd.setCursor(0, 1);                    
            lcd.print(heatIndex);
            lcd.setCursor(7, 1);
            lcd.print((char)223);
            lcd.setCursor(8, 1);
            lcd.print("C");

            break;
          case 'B': 
          // B*: Show local altitude
            Serial.print("\nB *");
            Serial.print("\nShow local altitude");

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("NanoSat Height: ");
            lcd.setCursor(0, 1);            
            lcd.print(telemetryData.localAltitude);
            lcd.setCursor(7, 1);
            lcd.print("m");

            break;
          case 'C': 
            Serial.print("\nC *");
            break;
          case 'D': 
            Serial.print("\nD *");
            break;
          // case '*': 
          //   lastKey = 'A'; // Return to previous mode
          //   break;
        }
        break;
      case '#':
        if (lastKey == 'B') {
          Serial.print("\nB #");
          Serial.print("\nReset barometer");

          // Send command
          sendCommand(2.0); // ask for reset pressureGroundLevel

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("[OK] Sat Ground");
          lcd.setCursor(0, 1);
          lcd.print("Press. Reseted");
          delay(1500);          
          lastKey = 'B';
          key = '*';          
        } else if(lastKey == 'C') {
          Serial.print("\nC #");
          Serial.print("\nAsk for IMU data");

          // Send command
          sendCommand(3.0);

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("[OK] IMU Data");
          lcd.setCursor(0, 1);
          lcd.print("Requested");
          delay(1500);  
          
          lastKey = 'C';
          key = 'C';

        } else if (lastKey == 'A') {
          Serial.print("\nA #");
          Serial.print("\nAsk for weather data");

          // Send command
          sendCommand(1.0);

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("[OK] TEL Data");
          lcd.setCursor(0, 1);
          lcd.print("Requested");
          delay(1500);  
          
          lastKey = 'A';
          key = 'A';

        } else {
          key = lastKey; // Return to previous mode
        }
        break;
    }
  }   
  else if ((rf_driver.recv(buf, &buflen) == 0)){
    currentMillis = millis();    
    if(currentMillis - lastDataReceived >= losTolerance) {
      Serial.print("\nNo data\n"); 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("[!!!] No data");
      lcd.setCursor(0, 1);
      lcd.print("from CubeSat");
    }       
  }

  delay(250);
  //delay(500);

}  // end void loop

// Transmitter code ------------------------------
void sendCommand(float command) {
  for (int i= 0; i<= 25; i++) {
    delay(100);
    commandData.op = command;
    memcpy(tx_buf, &commandData, sizeof(commandData));
    byte zize=sizeof(commandData);
    rf_driver.send((uint8_t *)tx_buf, zize);
    rf_driver.waitPacketSent();
  }
  Serial.print("\nData command was sent");
}

float computeHeatIndex(float temp, float hum){
  float T = (temp * 1.8) + 32; // temp in Farenheit  
  float R = hum;   
  float HI = 0.5 * (T+61.0 + ((T-68.0)*1.2) +  (R*0.094));
  // Average simple HI with T, if greater than 80F use complex Rothfusz equation
  if ((HI+T)/2.0 > 80) {    
    HI = heatIndexFullEquation(T,R);
  }
  heatIndex = (HI - 32) * (5.0/9.0);
  return heatIndex;
}

float heatIndexFullEquation(float T, float R){
  float adjus;
  float c1 = -42.379;
  float c2 = 2.04901523;
  float c3 = 10.14333127;
  float c4 = -0.22475541;
  float c5 = -6.83783 * (pow(10,-3)); //x10-3
  float c6 = -5.481717 * (pow(10,-2));
  float c7 = 1.22874 * (pow(10,-3));
  float c8 = 8.5282 * (pow(10,-4));
  float c9 = -1.99 * (pow(10,-6));

  float HI = (c1 + (c2*T) + (c3*R) + (c4*T*R) + (c5 *(T*T)) + (c6*(R*R)) + (c7*(T*T)*R) + (c8*T*(R*R)) + (c9*(T*T)*(R*R)));

  if (T>= 80.0 && T<=112.0 && R < 13.0) {    
    adjus = ((13.0-R)/4.0) * sqrt((17.0-abs(T-95.0))/17.0);
    HI = HI - adjus;
  } else if(T>=80.0 && T<=87.0 && R>85.0) {    
    adjus = ((R-85.0)/10.0) * ((87.0-T)/5.0);
    HI = HI + adjus;
  }

  return HI;
}