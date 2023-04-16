April 14, 2023
Albert Cervera-Uribe

18650 batteries range: 2.5V (discharged) - 4.2V (fully charged)
Nominal voltage: 3.7V



I connected at 8:55 UTC-6 the SPM to my Samsung brick while leaving just theCubesat connected to it (no USB-Meter). After 37 mins, the Arduinos lights were on, but very low. So I decided to connect the Arduino to a power brick (instead of leaving it on the ESPM), while the battery gauge is connected to the CS and charging. In this way, CubeSat has power from a brick, but is reading the battery percentage.

Safe mode was of course triggered since battery reading was 0% at 15:32 UTC

Update:
After 1h 35 mins of charge, percentage was still 0% though voltage is 2950.00 mV.
I did a reset of CS at 16h30 UTC (10h31 UTC-6) to see if it changes, but it did not. So percentage still at 0% with 2950.00 mV.
PS: I reset it the MCC server by mistake, so I cannot longer see the minimum minus voltage CS had (but was something around 2.3V).

Update: Finally I have a reading on percentage: 0.01 % with 3273.75 mV
0.02 % with 3285.00 mV
Update: The charging is very very slow, I'll change the USB cable and use the same brick to see if that solves it (it had almost 4 hrs 29 mins charging and still was at 0.09 %, 3378.75 mV)
Change done at 19:23 UTC. First cable was a very large one, now I'm using a ticker one and also a little bit shorter (I believe is from one of my security cameras).
At 19:40 UTC, the Spacecraft recovered from Safe Mode and put itself into RTWI. Command was sent to put it on ESM.
Update: 
At 23:20 UTC it had 101.30% of battery and 4198.75 mV.