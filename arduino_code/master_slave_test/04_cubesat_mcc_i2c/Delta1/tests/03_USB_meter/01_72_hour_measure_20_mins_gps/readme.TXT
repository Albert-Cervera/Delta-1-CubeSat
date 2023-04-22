#24h ESM test with power brick and USB-Meter 

CubeSat is connected to a brick charger as main power source, but is reading battery voltage from battery gauge. MOSFET GPS sync time is only 20 mins per sync (X hour, 50 mins - Y hour, 10 mins).

On April 9th, and with 30 mins GPS time per sync:

* 24h 00m: 1,731 mAh, energy: 8,853 mWh battery
* With an avg of 72.125 mAh per hour

April 18th
**18:10 UTC:** CubeSat connected to power brick with USB-Meter. Battery is only connected via battery gauge, but is not providing power.

15:10 UTC: Battery is being discharged a little, maybe due to SPM: Min: 77.16 % || Max: 78.46 %

**RESULT:**

* 24h 00m: 1,697 mAh, energy: 8,667 mWh battery
* With an avg of 70.708 mAh per hour

However, last transmission received was at 18:00:05 UTC, and it is 18:27 and still nothing.

Update: Last transmission received: 18:30:04 UTC
Min: 77.16 % || Max: 78.46 %
Min: 4003.75 mV || Max: 4013.75 mV

* 48h 00m: 3,389 mAh, energy: 17,314 mWh battery
* With an avg of 70.604 mAh per hour

Min: 76.18 % || Max: 78.46 %
Min: 3996.25 mV || Max: 4013.75 mV

* 72h 00m: 5,074 mAh, energy: 25,922 mWh battery
* With an avg of 70.47 mAh per hour

Min: 75.20 % || Max: 78.46 %
Min: 3988.75 mV || Max: 4013.75 mV

---
April 21
18:14 UTC: I connected the charger to the battery. I'd like it to charge to 100%. CubeSat is connected to a power brick.

1:16 UTC: It's been 7 hours, and battery is at 101%
Min: 100.82 % || Max: 101.61 %
4200.00 mV

I'll disconnect the charger, dump the SD card data, and begin a new discharging test.