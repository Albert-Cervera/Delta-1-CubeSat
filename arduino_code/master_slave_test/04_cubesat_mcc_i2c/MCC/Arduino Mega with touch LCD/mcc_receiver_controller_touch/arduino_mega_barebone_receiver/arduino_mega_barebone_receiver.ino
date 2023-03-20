#include <RH_ASK.h>  // RadioHead Amplitude Shift Keying Library

// Params: speed in BPS, rxPin, txPin, pttPin
RH_ASK rf_driver(2000, 21, 20, 0);  // <-- Receive on PIN 21 and transmit on PIN 20

// 28 bytes size and 7 elements (4 bytes each(?))
struct telemetryStruct {
  float humidity;
  float temperature;
  float pressure;
  float localAltitude;  // altitude in meters from ground
  float pitch;
  float roll;
  float yaw;
} telemetryData;

// 10 bytes size and 3 elements
struct dataStruct {
  int mode;
  float voltage;
  float internalTemp;  // dedicated temperature DSB18B20 sensor
  // float solarCurrent; // Future val: current generated by solar arrays
} systemData;

int initDay, initMonth, initYear, initHour, initMinute, initSecond;

void setup() {
  Serial.begin(9600);
  Serial.print("\nReceiver minimal version");

  // Initialize ASK Object
  rf_driver.init();
}

// Barebones version
void loop() {

  struct transferStruct {
    // System data:
    int mode;
    float voltage;
    float internalTemp;
    int bootDay;
    int bootMonth;
    int bootYear;
    int bootHour;
    int bootMinute;
    int bootSecond;
    // Science payload data:
    float humidity;
    float temperature;
    float pressure;
    float localAltitude;  // altitude in meters from ground (not MSL)
    float pitch;
    float roll;
    float yaw;
  } transferData;

  int8_t buf[sizeof(transferData)];
  uint8_t buflen = sizeof(buf);

  if (rf_driver.recv(buf, &buflen)) {

    Serial.print("\n[OK] Data rcvd from CubeSat");

    memcpy(&transferData, buf, sizeof(transferData));

    initDay = transferData.bootDay;
    initMonth = transferData.bootMonth;
    initYear = transferData.bootYear;
    initHour = transferData.bootHour;
    initMinute = transferData.bootMinute;
    initSecond = transferData.bootSecond;

    systemData.mode = transferData.mode;
    systemData.voltage = transferData.voltage;
    systemData.internalTemp = transferData.internalTemp;
    telemetryData.humidity = transferData.humidity;
    telemetryData.temperature = transferData.temperature;
    telemetryData.pressure = transferData.pressure;
    telemetryData.localAltitude = transferData.localAltitude;
    telemetryData.pitch = transferData.pitch;
    telemetryData.roll = transferData.roll;
    telemetryData.yaw = transferData.yaw;
  }

  // delay(250);
}
