//Libraries
#include <DHT.h>;
// Include RadioHead Amplitude Shift Keying Library
#include <RH_ASK.h>
// Include dependant SPI Library 
#include <SPI.h> 
 
// Create Amplitude Shift Keying Object
RH_ASK rf_driver; // PIN 12 by default

//Constants
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// IMPORTANT: Data pin of receiver should be at PIN 12 (it magically works with library)
// RH_ASK rf_driver(2000, 13, 8, 0); // speed in bps, rxPin, txPin, pttPin


struct dataStruct{
  float humidity ; 
  float temperature;  
}h22Data;

byte tx_buf[sizeof(h22Data)] = {0};

//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value

void setup()
{
  Serial.begin(9600);
  dht.begin();
  // Initialize ASK Object
  rf_driver.init();
}

void loop()
{
    delay(1000);
    //Read data and store it to variables hum and temp
    hum = dht.readHumidity();
    temp= dht.readTemperature();
    //Print temp and humidity values to serial monitor
    Serial.print("Humidity: ");
    Serial.print(hum);
    Serial.print(" %, Temp: ");
    Serial.print(temp);
    Serial.println(" Celsius");
    

    //Transmitter code ------------------------------    

    h22Data.humidity=hum;
    h22Data.temperature=temp;

    memcpy(tx_buf, &h22Data, sizeof(h22Data) );
    byte zize=sizeof(h22Data);
    rf_driver.send((uint8_t *)tx_buf, zize);
    rf_driver.waitPacketSent();
    
    delay(250); //1000: 1 second
}

   