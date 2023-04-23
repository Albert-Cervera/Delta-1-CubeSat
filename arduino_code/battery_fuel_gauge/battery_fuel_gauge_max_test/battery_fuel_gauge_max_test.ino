// #include "Arduino.h"
#include "MAX17043.h"

void setup() {
  Serial.begin(9600);
  FuelGauge.begin();
}

void loop() {
  float percentage = FuelGauge.percent();
  float voltage = FuelGauge.voltage();

  Serial.print("\n----------------------");
  Serial.print("\nBattery percentage: ");
  Serial.print(percentage);
  Serial.println("%");

  Serial.print("Battery voltage: ");
  Serial.print(voltage);
  Serial.println(" mV");

  delay(2000);
}