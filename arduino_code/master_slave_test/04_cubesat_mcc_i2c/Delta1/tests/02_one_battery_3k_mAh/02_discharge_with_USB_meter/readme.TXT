#New battery setup

Reminder: 18650 batteries range: **2.5V (discharged) - 4.2V (fully charged)**
Nominal voltage: 3.7V

I discovered that my battery gauge uses the MAX17043 chip, which is meant for a single cell (and not for three in parallel), also, after the failed attempts to charge and use the 3 batteries, I decided to get a new EFEST 3,000 mAh 18650 battery and use it with a single battery holder.

After some soldering and new connections, CubeSat is now (April 16th 5:27 UTC) connected to an external outlet, while battery is connected to a samsung brick charging. 



Initial readings: 3593.75 mV and 2.26%

5:28 UTC: I connected SPM to charge the new single 3,000 mAh EFEST battery.

5:38 UTC: Spacecraft was recovered from SM and battery reading was: 15.17%  and 3955.97 mV. Since CubeSat was recovered, it entered RTWI, so I change it to ESM to see how it handles every 10 mins.

5:50 UTC: 29.54% at 4022.50 mV, also, the SPM is hot as well as the brick charger (very nice indeed)

16:50 UTC:  battery 101.61 % (Min: 2.26 % || Max: 101.61 %)
Battery Voltage: 4198.75 mV (Min: 3593.75 mV || Max: 4200.00 mV)

17:00 UTC: 101.61 %, 4198.75 mV and charging time has been 11 hours 32 mins.

I think so far is kind of success, I'm gonna disconnect the charger of batteries, pass the SD data to the Mac, and then power the CubeSat entirely by its battery.

**For some reason, at some point it started to not save voltage data :worried: **

"09:40:00,16/04/2023,1,4183.75,21.38,21.75,99.08
09:50:01,16/04/2023,1,4183.75,21.38,21.69,99.24
10:00:00,16/04/2023,1,21.25,21.56,99.40
10:10:01,16/04/2023,1,21.25,21.56,99.71
10:20:00,16/04/2023,1,21.13,21.44,99.87"

#Discharge test:

17:25 UTC: CubeSat is connected to its main power source (the single 3000 mAh battery) and it also has the USB-Meter connected. Initial battery reading was 95.92% and 4135.94 mV

17:30 UTC: 4133.75 mV, 95.92%

17:40 UTC: 4116.25 mV , 95.29%

17:50 UTC: 4068.75 mV 93.07%

18:00 UTC: 4056.25 mV, 90.86% (note that GPS has been on since 17:40 probably)

18:10 UTC: 4037.50 mV, 88.64% (GPS now is off)

18:20 UTC: 4066.25 mV, 87.93%


18:30 UTC: 4060.00 mV, 87.35%


18:40 UTC: 4056.25 mV, 86.78%


18:50 UTC: 4053.75 mV, 86.34%

15:02 UTC: 2908.75 mV 0.0% and USB-Meter still on and arduino Lightning still on. 

15:06 UTC: 2878.75 mV (Min: 2878.75 mV || Max: 4148.75 mV) and voltage at 0.0%

I think this was a successful test given the Fakt that the satellite has the USB-Meter connected to it drawing energy. 
I'll disconnect the battery, download the SD data and do a recharge experiment.

Time: 21 hours 50 mins