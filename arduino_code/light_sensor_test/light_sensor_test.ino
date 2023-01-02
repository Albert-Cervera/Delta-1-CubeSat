/*
Author: Albert C.
LDR light sensor code.

NOTE:

1) Since it's a solded module with a digital output,
it can only measure light in 0 (light) or 1 (dark).
While analogue gives from 0-1023.

So it's more of a "LIGHT or NOT LIGHT" sensor.

VCC -> 5v

*/

int pinLDR= 9; // A digital pin like 8, 9 works
	int val=0;

	void setup()
	{
		//initialize communication at 9600 bits per second
		Serial.begin(9600);
	}
	
	void loop() {
		val= digitalRead(pinLDR);
		
		//print the value on the serial monitor
		//Go to Tools->Serial Monitor to see the values
		Serial.println(val);
		delay(100);
	}