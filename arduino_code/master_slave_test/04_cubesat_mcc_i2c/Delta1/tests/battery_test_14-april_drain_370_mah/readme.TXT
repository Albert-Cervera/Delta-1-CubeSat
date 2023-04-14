April 14, 2023
Albert Cervera-Uribe

Context:

1) Batteries: 3x 18650 batteries in parallel of 2800 mAh each, hence giving a total
8,400 mAh battery.

2) I know from previous tests that consumption is ESM: 24h 00m: 1,731 mAh, energy: 8,853 mWh

3) My battery first had a little over 101% of charge according to gauge. 
I connected to Satellite for 3/4 mins, and battery dropped to 88%

Experiment: 

On April 14th at 2:20 UTC the CubeSat was put inESM with 88.07% of battery, with the intention of seeing how far it'd run. It had connected my USB-Meter

ESM ran only from 2:20 to 5:50 UTC.

At 4:00 UTC it had 57.58% of battery.
At 5:55 UTC it had 11:29% of battery.
At 6:19 UTC it had 01:85% of battery.

Then, at one moment it had 1.61% of battery with a voltage of 3,531.25 mV.

*** Finally, last transmission was received on April 14th at 11:28:19 UTC:

Battery: 0.00%
Voltage: 2,521.25 mV

*** Readings from USB-Meter:

Capacity record: 370 mAh
Energy record: 1,864 mWh
Time: 9 hrs, 17 mins, 43 seconds

Remarks:

The battery percentage dropped really fast! Even when having a relatively high voltage
the percentage shown was very little.

Also, when doing the charging testing, it went from 40% to 100% fast.


11:33:49.263 -> voltage: 3808.75 mV
11:33:49.296 -> precentage: 40.82 %

12:55:05.356 -> voltage: 4190.00 mV
12:55:05.388 -> precentage: 95.13 %

13:42:51.450 -> voltage: 4196.25 mV
13:42:51.483 -> precentage: 100.03 %

19:01:45.040 -> voltage: 4200.00 mV
19:01:45.072 -> precentage: 101.77 %

Charge test note: Using the SPM connected to my old Samsung brick charger (probably 5V and 1A, nothing fancy or fast charging). I left it connected even after the 100% charge to see how it handled the charge.

------------------------------------------------------------------------------------------

Preliminary conclusion:

If we split the 370 mAh that batteries gave into the three of them, that is 123.33 mAh each.
Given the fact that each battery is meant to have 2,800 mAh, this seems very off.
Without any investigation, and just speculating, I think the problem might be that:

1) Battery capacity of my 18650 is faked, and in reality they have around 130 mAh each. 
   Note: However, this shouldn't be! My batteries are from a very well know vendor (Steren).
	 Further tests are needed in order to confirm this.

2) The SPM and battery gauge are defective and/or have an issue with my current setup.

3) My current configuration for the SPM and battery gauge has something off.


More investigation is required. In the '1_ESM' and '0_SM' folders are the txts with logs of this experiment.

Also some external data including a USB-Meter report, and photographs are under the '4_external_data' folder.


------------------------------------------------------------------------------------------

Opportunity:

Abstracting  the Safe Mode current consumption from the USB-Meter report will help better understand
its timing and energy consumption patterns to better fine tune it.

