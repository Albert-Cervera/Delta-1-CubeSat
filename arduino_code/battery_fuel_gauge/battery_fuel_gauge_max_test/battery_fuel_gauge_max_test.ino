#include "Arduino.h"
#include "MAX17043.h"

void setup() {
  Serial.begin(9600);
  Wire.begin(4,0); // What are these numbers?
  FuelGauge.begin();
}

void loop(){
  float percentage = FuelGauge.percent();
  float voltage = FuelGauge.voltage(); // mV '/1000' to have V

  Serial.print("Battery percentage: " + String(percentage) + " %");
  Serial.print("Battery voltage: " + String(voltage) + " mV");

  delay (1000);
}