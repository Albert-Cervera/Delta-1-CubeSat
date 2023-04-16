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