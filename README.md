# Delta-1 CubeSat

(As of January 2023...)

As the first nano-satellite of the Delta-V Project, Delta-1 is a 2U CubeSat with COTS intruments and controllers.
The CubeSat is based on the Arduino architecture and its main payload instruments measure atmosferic conditions and magnetic & inertial measurements, logging its data into an SD card. Delta-1 has an onboard RTC as mission clock which serves to many critical functions and that is synced via GPS with on-orbit satellites.

The onboard computers are composed of two Arduino Uno with the controller-peripheral architecture based on I2C which are identified as SAT_A and SAT_B respectively. 

### SAT_A (controller device)
This computer acts as the 'master' device within Delta-1, its main functions include (but are not limited to):
* Command reception and execution
* Data transmission
* Payload instruments reading
* Manage Safe Mode and mission operational modes
* Send data and instructions to SAT_B
* Order sleep mode on SAT_B

### SAT_B (peripheral device)
This computer acts as the 'slave' device within Delta-1, its main functions include (but are not limited to):
* Data and instruction reception from SAT_A
* Store data into SD card
* Manage the RTC mission clock
* Operate the GPS module and sync RTC


TODO:

Future (but very soon) plans are to include an onboard camera and IR camera in Delta-1 for thermal and regular image acquisition possibly using a third Arduino Uno as second peripheral device.

As of January of 2023, Delta-1 is an on-going projec with many stages in the design phase yet, however, major advancements has been made considering the Electrical Power Supply (EPS), the 3D printing and structure, solar arrays, batteries and thermal protection. As these components get more mature, I'll describe them here.


# CubeSat Spacecraft

## Sensors, antennas & instruments

* Barometer: GY-BMP280
* Dedicated thermometer: DSB18B20
* Payload thermometer and hygrometer: DHT22
* Gyroscope/Accelerometer: MPU-9250
* Magnetometer: AK8963 (from same MPU-9250)
* Antennas: 433MHz RF Wireless transmisor and receptor TLP/RLP 434
* DFRobot Solar Power Manager 5V (version 1.1)
* MicroSD card adapter
* RTC (Real Time Clock): DS1302
* GPS module: NEO-6M (Goouuu Tech GT-U7)



# Mission Control Center (MCC)
(Fully describe it)
* TFT LCD panel for MCC: Elegoo UNO R3 2.8 Inches TFT
* Numeric 4x4 keypad
