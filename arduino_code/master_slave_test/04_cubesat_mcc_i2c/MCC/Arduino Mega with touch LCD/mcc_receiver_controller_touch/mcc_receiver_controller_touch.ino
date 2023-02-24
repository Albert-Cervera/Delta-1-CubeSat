// Government Agency Original Software Designation: LAR-18832-1
//
//------------------------------------------------------------------------------
// NASA/GSFC, Software Integration & Visualization Office, Code 610.3
//------------------------------------------------------------------------------
//
// MODULES: TFT Touch LCD 2.8" Elegoo & 433 mHz antennas.
//
//> @author
//> Albert Aarón Cervera Uribe
//
// DESCRIPTION:
//> This script contains a testing version of the MCC (Mission Control Center)
// for the Delta-1 CubeSat.
// The purpose of MCC is to control and perform the various operations needed
// to manage the Cubesat. This script allows data reception and command
// transmission via a touch interface.
//
// NOTES:
// 1) The touch UI is a modified version of 'JOS' (2011), created by
// Jeremy Saglimbeni - thecustomgeek.com; however the rest of the UI
// was developed 'in-house' accounting for the graphics, and CubeSat
// operations.
//
// 2) Code here is very similar to the Arduino Uno implementation with
// peripheral devices, LCD and Numpad, but adapted to the GUI interface
// without those elements.
//
// ISSUE:
// 1) Program storage space should not pass 80% or it won't execute properly.
//
// REVISION HISTORY:
// 07 February 2023 - Initial Version
// 11 February 2023 - Modification
// -- -- 2023 - Final first Version
//
// TODO_dd_mmm_yyyy - TODO_describe_appropriate_changes - TODO_name
//------------------------------------------------------------------------------

#include <EEPROM.h>
#include <Elegoo_GFX.h>     // Core graphics library
#include <Elegoo_TFTLCD.h>  // Hardware-specific library
#include <TouchScreen.h>
#include <RH_ASK.h>  // RadioHead Amplitude Shift Keying Library
#include <Wire.h>
#include <SD.h>
#include <limits.h>
#include <virtuabotixRTC.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>  // Also for GPS ()

// *****************************************************************************
// Touchscreen configuration ***************************************************

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// My own good hand-calibrated values
#define TS_MINX 96   // 96 gives p.x: 240 at (240,y) [good]
#define TS_MAXX 935  // 928 gives p.x: -1 at (0,y), 935 seems more accurate and consistant [good]

#define TS_MINY 275  //275 gives p.y: 320 at (0,320) [good]
#define TS_MAXY 920  // gives p.y: 0 at (x,0) [good]

// Pressure sensitivity for screen
#define MINPRESSURE 10
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4  // optional

// Color definitions - in 5:6:5
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define TEST 0x1BF5
#define JJCOLOR 0x1CB6
#define JJORNG 0xFD03
#define MARS 0xFD00

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// Plotting parameters
int scale1 = 1;
int scale2 = 2;
int scale4 = 4;
int scale8 = 8;
int incrementation = 24;  // At first: how many pixels to move the starting point of plot

int i = 0;
int page = 0;
int lastPage = 0;
int blv;
int sleep = 0;
int pulsev = 0;
int redflag = 0;
int greenflag = 0;
int redled = 12;
int greenled = 13;
int backlight = 10;
int battfill;
unsigned long sleeptime;
unsigned long battcheck = 10000;  // the amount of time between voltage check and battery icon refresh
unsigned long prevbatt;
int battv;
int battold;
int battpercent;
int barv;
int prevpage;
int sleepnever;
int esleep;
int backlightbox;
int antpos = 278;
unsigned long awakeend;
unsigned long currenttime;
unsigned long ssitime;
char voltage[10];
char battpercenttxt[10];
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA, ADSC))
    ;
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result;  // Back-calculate AVcc in mV
  return result;
}


// Delta-1 MCC parameters
bool enableArea = true;

// *****************************************************************************
// MCC internal configuration **************************************************

#define CS_PIN 53  // Arduino Mega SD pin (10 on Arduino Uno)
File myFile;
char delim = ';';
String buffer;  // For reading file line by line
String latitude, longitude, altitude;

// Create Amplitude Shift Keying Object
// Params: speed in BPS, rxPin, txPin, pttPin
RH_ASK rf_driver(2000, 21, 20, 0);  // <-- Receive on PIN 21 and transmit on PIN 20
virtuabotixRTC myRTC(25, 27, 29);   // Real Time Clock
SoftwareSerial ss(A12, A11);        // The serial connection to the GPS device: RXPin, TXPin

unsigned long lastDataReceived;  // miliseconds with no signal from transmitter
unsigned long currentMillis;     // current miliseconds var to compare data
String referenceTime;            // Time reference for updating mission clock on screen
String rcvdTime;                 // Time reference for last time a signal was received from CubeSat
int lastSecondSavedTel, lastMinuteSavedTel;
int lastSecondSavedSys, lastMinuteSavedSys;
int lastHourSavedHTel;
int lastMinuteSavedESMTel;

bool startSync = false;  // For RTC-GPS sync
bool rcvdTx;
uint32_t losTolerance = 630000;  // 630000, Loss of Signal tolerance in ms //630 sec = 10 min, 30 sec
float heatIndex;
int dataMode = 1;

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

// 6 bytes size and 2 elements (3 bytes each(?))
struct dataStruct {
  int mode;
  float voltage;
  float internalTemp;  // dedicated temperature DSB18B20 sensor
  // float solarCurrent; // Future val: current generated by solar arrays
} systemData;

struct commandStruct {
  float op;
} commandData;

byte tx_buf[sizeof(commandData)] = { 0 };  // buffer for sending command data


void setup() {
  Serial.begin(9600);

  // UI configuration ------------------------------------------------------------
  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  esleep = EEPROM.read(1);
  blv = EEPROM.read(2);
  if (esleep == 1) {
    sleeptime = 10000;
  }
  if (esleep == 2) {
    sleeptime = 20000;
  }
  if (esleep == 3) {
    sleeptime = 30000;
  }
  if (esleep == 4) {
    sleeptime = 60000;
  }
  if (esleep == 5) {
    sleeptime = 120000;
  }
  if (esleep == 6) {
    sleeptime = 300000;
  }
  if (esleep == 7) {
    sleeptime = 600000;
  }
  if (esleep == 8) {
    sleeptime = 1200000;
  }
  if (esleep == 9) {
    sleeptime = 1800000;
  }
  if (esleep == 10) {
    sleeptime = 3600000;
  }
  if (esleep == 11) {
    sleeptime = 14400000;
  }
  if (esleep == 12) {
    sleepnever = 1;
  }
  awakeend = sleeptime + 1000;  // set the current sleep time based on what the saved settings in EEPROM were
  pinMode(backlight, OUTPUT);
  Serial.println("AOS");
  Serial.println("Albert Cervera  -  2023");
  Serial.print("TFT size is ");
  Serial.print(tft.width());
  Serial.print("x");
  Serial.println(tft.height());
  Serial.println("sleeptime: ");
  Serial.println(sleeptime);

  tft.reset();

  uint16_t identifier = tft.readID();

  if (identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if (identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if (identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  } else if (identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if (identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  } else if (identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if (identifier == 0x0101) {
    identifier = 0x9341;
    Serial.println(F("Found 0x9341 LCD driver"));
  } else if (identifier == 0x1111) {
    identifier = 0x9328;
    Serial.println(F("Found 0x9328 LCD driver"));
  } else {
    Serial.print("Unknown driver chip ");
    Serial.println(identifier, HEX);
    while (1)
      ;
  }


  // tft.initDisplay();
  tft.begin(identifier);
  tft.setRotation(3);  // 1
  tft.fillScreen(BLACK);
  tft.fillRect(71, 70, 50, 100, MARS);
  tft.fillRect(134, 70, 50, 100, MARS);  // JJCOLOR
  tft.fillRect(197, 70, 50, 100, MARS);  // JJORNG
  tft.drawRect(46, 45, 228, 150, WHITE);
  for (i = 0; i <= blv; i += 1) {
    analogWrite(backlight, i);
    delay(2);
  }
  delay(250);

  tft.setCursor(85, 100);
  tft.setTextColor(WHITE);
  tft.setTextSize(5);
  tft.println("A");
  delay(250);

  tft.setCursor(147, 100);
  tft.setTextColor(WHITE);
  tft.setTextSize(5);
  tft.println("O");
  delay(250);

  tft.setCursor(210, 100);
  tft.setTextColor(WHITE);
  tft.setTextSize(5);
  tft.println("S");
  delay(500);
  tft.setCursor(100, 210);  // 84, 210
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Albert Cervera - 2023");
  tft.setCursor(84, 230);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Jet Propulsion Laboratory");
  delay(700);  // 500
  tft.fillScreen(BLACK);
  tft.fillRect(0, 0, 320, 10, JJCOLOR);  // status bar
  drawHomeIcon();                        // draw the home icon
  // tft.drawRect(297, 1, 20, 8, WHITE);  //battery body
  // tft.fillRect(317, 3, 2, 4, WHITE);   // battery tip
  // tft.fillRect(298, 2, 18, 6, BLACK);  // clear the center of the battery
  // drawBatt(); // at void setup()
  ant();                                 // draw the base "antenna" line without the "signal waves"
  signal(false);                         // draw the "signal waves" around the "antenna" - MODIFIED: Receives a 'bool green' value.
  homescr();                             // draw the homescreen
  tft.drawRect(0, 200, 245, 40, WHITE);  // message box

  // MCC configuration -----------------------------------------------------------
  systemData.mode = -1;  // To differentiate actual '0 mode' vs no data received.
  lastDataReceived = 0;
  rf_driver.init();  // Initialize ASK Object
  referenceTime = getTimestampTime();

  // Set the current date, and time (monday is day 1, sunday is 7)
  // NOTE: to sync, upload code when real time is about to change to 40 seconds
  // myRTC.setDS1302Time(00, 56, 22, 7, 12, 02, 2023);  // SS, MM, HH, DW, DD, MM, YYYY

  Serial.print("Initializing SD card... ");
  if (!SD.begin(CS_PIN)) {
    Serial.println("initialization failed!");
    while (1)
      ;  // Validate this part: If the SD card fails, wouldn't I want to continue using MCC?
  }
  Serial.println("initialization done.");
  // Remove existing file.
  //  SD.remove("READTEST.TXT"); // Code that I'll never use here, but is useful to know
  ss.begin(9600);  // initialize GPS module at GPSBaud 9600

}  // end void setup()

void loop() {

  // MCC configuration -----------------------------------------------------------

  // String timestamp = getTimestampTime();
  // Serial.print("\nTimestamp: ");
  // Serial.print(timestamp);
  // String timestampDate = getTimestampDate();
  // Serial.print(", " + timestampDate);

  // Receiver antenna code -----------------------------
  // Set buffer to size of expected message
  int8_t buf[RH_ASK_MAX_MESSAGE_LEN];  // Set it to maximum size of 60 bytes, but not the actual expected size
  uint8_t buflen = sizeof(buf);

  // Auxiliar struct to copy ALL data received from CubeSat into a single struct.
  // Then assign it to local systemDSata and telemetryData structs.
  struct transferStruct {
    int mode;
    float voltage;
    float internalTemp;
    float humidity;
    float temperature;
    float pressure;
    float localAltitude;  // altitude in meters from ground
    float pitch;
    float roll;
    float yaw;
  } transferData;

  // Non-blocking
  if (rf_driver.recv(buf, &buflen) == 1) {
    // If data received ...

    // Serial.print("\n[OK] Data rcvd from CubeSat\n");
    rcvdTx = true;
    rcvdTime = getTimestampTime();

    lastDataReceived = millis();
    memcpy(&transferData, buf, sizeof(transferData));

    // Workaround for sending these data through I2C for SD storage
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

    // if (systemData.mode == 1) {
    //   Serial.print("\nESM mode detected");
    //   losTolerance = 630000;  // 10 mins, 30 secs. Should be greater than the 10 mins loop of the ESM mode
    // } else if (systemData.mode == 0) {
    //   // Spacecraft is in SAFE MODE, and MCC should display it
    // } else {
    //   // Serial.print("\nOther mode detected");
    //   losTolerance = 5000;
    // }

    myRTC.updateTime();
    bool validToSave;
    bool validToSaveESM;

    // Save data every hour at minute 30 giving 40 secs tolerance
    if (myRTC.hours != lastHourSavedHTel && myRTC.minutes == 30 && (myRTC.seconds >= 0 && myRTC.seconds <= 40)) {  // Giving 40 seconds tolerance
      lastHourSavedHTel = myRTC.hours;
      validToSave = true;
    } else {
      validToSave = false;
    }

    if (validToSave) {
      Serial.print("\n[OK] Saving hourly data into SD ...");
      writeToSD(6);  // Save telemetry data every hour (regardless of CubeSat mode)
      writeToSD(7);  // Save system data every hour (regardless of CubeSat mode)
      validToSave = false;
    }

    // Save ESM data every 10 minutes giving 40 secs tolerance
    if (myRTC.minutes != lastMinuteSavedESMTel && (myRTC.seconds >= 0 && myRTC.seconds <= 40)) {  // Giving 40 seconds tolerance
      lastMinuteSavedESMTel = myRTC.minutes;
      validToSaveESM = true;
    } else {
      validToSaveESM = false;
    }

    // Modes -> 0: Safe Mode, 1: ESM, 2: RTW, 3: RTI, 4: RTWI
    switch (systemData.mode) {
      case 0:
        writeToSD(3);  // Safe mode: SUDO save system data
        break;
      case 1:
        // Serial.print("\nSaving ESM data into SD ...");
        if (validToSaveESM) {
          writeToSD(4);  // ESM mode: save telemetry data
          writeToSD(5);  // ESM mode: save system data
        }
        break;
      case 2:
        writeToSD(1);  // Save RTWI telemetry data: Checking validTime
        writeToSD(2);  // Save RT system data: Checking validTime
        break;
      case 3:
        writeToSD(1);  // Save RTWI telemetry data: Checking validTime
        writeToSD(2);  // Save RT system data: Checking validTime
        break;
      case 4:
        // Serial.print("\nSaving RTWI data into SD ...");
        writeToSD(1);  // Save RTWI telemetry data: Checking validTime
        writeToSD(2);  // Save RT system data: Checking validTime
        break;
    }

    // sendDataI2C(); // Send telemetryData via I2C to peripheral slave

  } else if ((rf_driver.recv(buf, &buflen) == 0)) {
    // If NO data is received ...
    /*
    TODO: When no signal is received from Delta1, display in some screen
    how many seconds have happened since LOS (Loss of Signal).
    */
    rcvdTx = false;

    currentMillis = millis();
    // If in 10 mins, 30 secs no data is received, display message
    if (currentMillis - lastDataReceived >= losTolerance) {
      signalAct();
      // clearMessageStatusBar();
      // tft.setCursor(1, 1);
      // tft.setTextColor(WHITE);
      // tft.setTextSize(1);
      // tft.println("[!!!] No data from CubeSat    Delta-1 Mission");
    }
  }

  if (validTimeToSync()) {  // Sync with GPS
    startSync = true;
  }


  if (startSync) {
    clearMessage();
    tft.setCursor(12, 213);
    tft.setTextColor(YELLOW);
    tft.setTextSize(2);
    tft.println("Syncing RTC-GPS ...");
    syncGPSDateTime(true);  // Strict Mode
    startSync = false;

    // String timestamp = getTimestampTime();
    // Serial.print("\n" + timestamp);

    // String timestampDate = getTimestampDate();
    // Serial.print(", " + timestampDate);
  }

  // UI configuration ------------------------------------------------------------

  currenttime = millis();
  unsigned long currentawake = millis();
  if ((currentawake > awakeend) && (sleepnever == 0)) {
    if (sleep == 0) {
      for (i = blv; i >= 0; i -= 1) {
        analogWrite(backlight, i);
        delay(4);
      }
      sleep = 1;
    }
  }

  digitalWrite(13, HIGH);  // added
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);  // added


  // if you're sharing pins, you'll need to fix the directions of the touchscreen pins!
  //pinMode(XP, OUTPUT);
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  //pinMode(YM, OUTPUT);

  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!
  if ((p.z > MINPRESSURE && p.z < MAXPRESSURE)) {

    // awakeend = currenttime + sleeptime;  //set the sleep time when screen is pressed
    // if (sleep == 1) {                    // if asleep, then fade the backlight up
    //   for (i = 0; i <= blv; i += 1) {
    //     analogWrite(backlight, i);
    //     delay(1);
    //   }
    //   sleep = 0;  // change the sleep mode to "awake"
    //   return;
    // }



    // Serial.print("X = ");
    // Serial.print(p.x);
    // Serial.print("\tY = ");
    // Serial.print(p.y);
    // Serial.print("\tPressure = ");
    // Serial.println(p.z);



    // turn from 0->1023 to tft.width
    p.x = map(p.x, TS_MINX, TS_MAXX, 240, 0);
    p.y = map(p.y, TS_MINY, TS_MAXY, tft.height(), 0);


    // Serial.print("p.y:");  // this code will help you get the y and x numbers for the touchscreen
    // Serial.print(p.y);
    // Serial.print("   p.x:");
    // Serial.println(p.x);

    // area 1 [calibrated]
    if (p.y > 170 && p.y < 320 && p.x > 173 && p.x < 216 && enableArea) {  // if this area is pressed
      if (page == 5) {                                                     // and if page 5 is drawn on the screen
        m5b1action();                                                      // do whatever this button is
        // tft.drawString(12, 213, "Menu 5 B1", RED, 2); // display the command in the "message box"
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 5 B1");
        yled(550);       // flash the LED yellow for a bit - change the 550 value to change LED time on
        clearMessage();  // after the LED goes out, clear the message
      }
      if (page == 4) {
        m4b1action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Temp & Hum");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b1action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B1");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b1action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 2 B1");
        yled(550);
        clearMessage();
      }
      if (page == 13) {
        m13b1action();
      }
      if (page == 12) {
        m12b1action();
      }
      if (page == 1) {
        m1b1action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Telemetry");
        yled(550);
        clearMessage();
      }
      if (page == 611) {
        settingsb11action();
      }
      if (page == 0) {  // if you are on the "home" page (0)
        page = 1;       // then you just went to the first page
        redraw();       // redraw the screen with the page value 1, giving you the page 1 menu
      }
    }
    // area 2 [calibrated]
    if (p.y > 0 && p.y < 150 && p.x > 170 && p.x < 217 && enableArea) {
      if (page == 5) {
        m5b2action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 5 B2");
        yled(550);
        clearMessage();
      }
      if (page == 4) {
        m4b2action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Heat Index");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b2action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B2");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b2action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 2 B2");
        yled(550);
        clearMessage();
      }
      if (page == 13) {
        m13b2action();
      }
      if (page == 12) {
        m12b2action();
      }
      if (page == 1) {
        m1b2action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("OBC");
        yled(550);
        clearMessage();
      }
      if (page == 611) {
        settingsb12action();
      }
      if (page == 0) {
        page = 2;
        redraw();
      }
    }
    // area 3 [calibrated]
    if (p.y > 170 && p.y < 320 && p.x > 110 && p.x < 160 && enableArea) {
      // if (page == 5) {
      //   m5b3action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B3");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b3action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Air Press");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b3action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B3");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b3action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Local Alt");
        yled(550);
        clearMessage();
      }
      if (page == 13) {
        m13b3action();
      }
      if (page == 12) {
        m12b3action();
      }
      if (page == 1) {
        m1b3action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("SC Modes");
        yled(550);
        clearMessage();
      }
      if (page == 0) {
        page = 3;
        redraw();
      }
    }
    // area 4 [calibrated]
    if (p.y > 0 && p.y < 150 && p.x > 111 && p.x < 158 && enableArea) {
      // if (page == 5) {
      //   m5b4action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B4");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b4action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Local Alt");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b4action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B4");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b4action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 2 B4");
        yled(550);
        clearMessage();
      }
      if (page == 13) {
        m13b4action();
      }
      if (page == 1) {
        m1b4action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 1 B4");
        yled(550);
        clearMessage();
      }
      if (page == 0) {
        page = 4;
        redraw();
      }
    }
    // area 5 [calibrated]
    if (p.y > 170 && p.y < 320 && p.x > 51 && p.x < 95 && enableArea) {
      // if (page == 5) {
      //   m5b5action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B5");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 43) {
        m43b5action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Plotting ...");
        yled(550);
        clearMessage();
      }
      if (page == 4) {
        m4b5action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("IMU");
        yled(550);
        clearMessage();
      }

      // No button or action there
      // if (page == 3) {
      //   m3b5action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 3 B5");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 2) {
        m2b5action();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 2 B5");
        yled(550);
        clearMessage();
      }
      if (page == 13) {
        m13b5action();
      }
      if (page == 61) {
        settingsb1action();
      }
      // No button or action there
      // if (page == 1) {
      //   m1b5action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 1 B5");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 0) {
        page = 5;
        redraw();
      }
    }
    // area 6  [calibrated]
    if (p.y > 0 && p.y < 150 && p.x > 50 && p.x < 97 && enableArea) {
      // if (page == 5) {
      //   m5b6action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B6");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b6action();
        // clearCenter();
        clearMessage();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Summary");
        yled(550);
        clearMessage();
      }
      // No button or action there
      // if (page == 3) {
      //   m3b6action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 3 B6");
      //   yled(550);
      //   clearMessage();
      // }
      // No buttons or action there
      // if (page == 2) {
      //   m2b6action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 2 B6");
      //   yled(550);
      //   clearMessage();
      // }
      // No buttons or action there
      // if (page == 1) {
      //   m1b6action();
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 1 B6");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 0) {
        page = 6;
        redraw();
      }
    }
    // home [calibrated]
    if (p.y > 0 && p.y < 46 && p.x > 0 && p.x < 39) {  // if the home icon is pressed
      if (page == 6) {                                 // if you are leaving the settings page
        clearMessage();                                // clear the battery voltage out of the message box
        // tft.drawString(12, 213, "Settings Saved", RED, 2); // display settings saved in message box
        EEPROM.write(1, esleep);  // write the sleep value to EEPROM, so it will not lose settings without power
        EEPROM.write(2, blv);     // write the backlight value to EEPROM, so it will not lose settings without power
        clearSettings();          // erase all the drawings on the settings page
      }
      if (page == 0) {      // if you are already on the home page
        drawHomeIconRed();  // draw the home icon red
        delay(250);         // wait a bit
        drawHomeIcon();     // draw the home icon back to white
        return;             // if you were on the home page, stop.
      } else {              // if you are not on the settings, home, or keyboard page
        page = prevpage;    // a value to keep track of what WAS on the screen to redraw/erase only what needs to be
        page = 0;           // make the current page home
        redraw();           // redraw the page
      }
    }
    // message area [calibrated]
    if (p.y > 78 && p.y < 320 && p.x > 0 && p.x < 37) {
      clearMessage();  // erase the message
    }
    // backlight buttons [calibrated]
    if (p.y > 259 && p.y < 320 && p.x > 170 && p.x < 218) {
      if (page == 6) {
        blightDown();
      }
    }
    if (p.y > 0 && p.y < 60 && p.x > 170 && p.x < 218) {
      if (page == 6) {
        blightUp();
      }
    }
    // sleep buttons [calibrated]
    if (p.y > 259 && p.y < 320 && p.x > 110 && p.x < 160) {
      if (page == 6) {
        sleepDec();
      }
    }
    if (p.y > 0 && p.y < 60 && p.x > 110 && p.x < 160) {
      if (page == 6) {
        sleepInc();
      }
    }
    /*
    // optional buttons
     if (p.y > 3 && p.y < 66 && p.x > 72 && p.x < 126) {
     if (page == 6) {
     option3down();
     }
     }
     if (p.y > 269 && p.y < 324 && p.x > 72 && p.x < 126) {
     if (page == 6) {
     option3up();
     }
     }
     */
  }

  // If 10 seconds have passed since last signal, put antenna icon in white
  if (currentMillis - lastDataReceived >= 10000 && currentMillis - lastDataReceived < losTolerance) {
    signal(false);
  }

  // Immediately Update screens when receiving data
  if (rcvdTx == true) {
    signal(true);

    switch (lastPage) {
      case 23:
        m2b3action();
        break;
      case 41:
        m4b1action();
        break;
      case 42:
        m4b2action();
        break;
      case 43:
        m4b3action();
        break;
      case 44:
        m4b4action();
        break;
      case 45:
        m4b5action();
        break;
      case 46:
        m4b6action();
        break;
      case 11:
        m1b1action();
        break;
    }
  }

  // Battery check
  if (currenttime - prevbatt > battcheck) {
    // drawBatt(); // at void loop()
    prevbatt = currenttime;
  }

  // Mission Clock update
  String currentTime = getTimestampTime();

  if (referenceTime != currentTime) {
    drawClock(currentTime);
    referenceTime = currentTime;
  }


  // delay(250);  // from MCC code (Menu test didn't had this)
  //delay(1000);  // Just for seeing the clock
}  // end void loop()

// *****************************************************************************
// MCC internal functions ******************************************************

void sendCommandAndListen(float command, int seconds, bool altitudReset) {
  /*
  Description: it loops a defined amount of time (seconds) while 
  transmitting and receiving in equal periods. For example, for 11 minutes
  transmit 2 seconds, then listen 2 seconds, transmit again, listen again, etc...
  */

  uint32_t msTransmit = (seconds * 1000UL);  // To avoid integer overflow with 16 bit math
  commandData.op = command;
  memcpy(tx_buf, &commandData, sizeof(commandData));
  byte zize = sizeof(commandData);

  // consider declaring transferData as global variable
  struct transferStruct {
    int mode;
    float voltage;
    float internalTemp;
    float humidity;
    float temperature;
    float pressure;
    float localAltitude;  // altitude in meters from ground
    float pitch;
    float roll;
    float yaw;
  } transferData;

  bool rcvd = false;
  int8_t buf[RH_ASK_MAX_MESSAGE_LEN];  // Set it to maximum size of 60 bytes
  uint8_t buflen = sizeof(buf);
  unsigned long startTime2;
  unsigned long endTime2;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;

  Serial.print("\nTransmitting 11 mins, with msTransmit: ");
  Serial.print(msTransmit);

  // Transmit command X seconds
  while ((endTime - startTime) <= msTransmit) {  // msTransmit should be equal to 11 mins in ms
    // Do transmit and listen events in periods of 2 seconds:

    Serial.print("\nTransmitting 2 seconds ...");
    startTime2 = millis();
    endTime2 = startTime2;
    while ((endTime2 - startTime2) <= 2000) {  // transmit 2 seconds
      rf_driver.send((uint8_t*)tx_buf, zize);
      rf_driver.waitPacketSent();
      endTime2 = millis();
    }

    Serial.print("\nListening 2 seconds ...");
    startTime2 = millis();
    endTime2 = startTime2;
    while ((endTime2 - startTime2) <= 2000) {  // listen 2 seconds for ack from Spacecraft
      if (rf_driver.recv(buf, &buflen) == 1) {

        Serial.print("\nAck received from spacecraft");

        lastDataReceived = millis();
        memcpy(&transferData, buf, sizeof(transferData));

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

        if (altitudReset) {
          if (telemetryData.localAltitude >= -0.6 && telemetryData.localAltitude <= 0.6) {
            rcvd = true;
            break;  // leave this while loop
          }
        } else {
          // Only if another spacecraft mode different from ESM is received
          if (systemData.mode != 1) {
            rcvd = true;
            break;  // leave this while loop
          }
        }

      }  // end if received
      endTime2 = millis();
    }

    if (rcvd) {
      Serial.print("\nrcvd is true, leaving transmission ...");
      break;  // leave the whole 11 mins while iteration
    }

    endTime = millis();
  }  // end while 11 mins
  Serial.print("\nDone with transmission");
}

void sendCommand(float command, int seconds) {
  int msTransmit = seconds * 1000;
  commandData.op = command;
  memcpy(tx_buf, &commandData, sizeof(commandData));
  byte zize = sizeof(commandData);

  // Transmit command X seconds
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  while ((endTime - startTime) <= msTransmit) {
    rf_driver.send((uint8_t*)tx_buf, zize);
    rf_driver.waitPacketSent();
    endTime = millis();
  }
  Serial.print("\nData command was sent");
}

float computeHeatIndex(float temp, float hum) {
  float T = (temp * 1.8) + 32;  // temp in Farenheit
  float R = hum;
  float HI = 0.5 * (T + 61.0 + ((T - 68.0) * 1.2) + (R * 0.094));
  // Average simple HI with T, if greater than 80F use complex Rothfusz equation
  if ((HI + T) / 2.0 > 80) {
    HI = heatIndexFullEquation(T, R);
  }
  heatIndex = (HI - 32) * (5.0 / 9.0);
  return heatIndex;
}

float heatIndexFullEquation(float T, float R) {
  float adjus;
  float c1 = -42.379;
  float c2 = 2.04901523;
  float c3 = 10.14333127;
  float c4 = -0.22475541;
  float c5 = -6.83783 * (pow(10, -3));  //x10-3
  float c6 = -5.481717 * (pow(10, -2));
  float c7 = 1.22874 * (pow(10, -3));
  float c8 = 8.5282 * (pow(10, -4));
  float c9 = -1.99 * (pow(10, -6));

  float HI = (c1 + (c2 * T) + (c3 * R) + (c4 * T * R) + (c5 * (T * T)) + (c6 * (R * R)) + (c7 * (T * T) * R) + (c8 * T * (R * R)) + (c9 * (T * T) * (R * R)));

  if (T >= 80.0 && T <= 112.0 && R < 13.0) {
    adjus = ((13.0 - R) / 4.0) * sqrt((17.0 - abs(T - 95.0)) / 17.0);
    HI = HI - adjus;
  } else if (T >= 80.0 && T <= 87.0 && R > 85.0) {
    adjus = ((R - 85.0) / 10.0) * ((87.0 - T) / 5.0);
    HI = HI + adjus;
  }

  return HI;
}

String setTwoDigits(uint8_t val) {
  if (val < 10) {
    return "0" + String(val);
  } else {
    return String(val);
  }
}

String getTimestampDate() {
  myRTC.updateTime();
  return setTwoDigits(myRTC.dayofmonth) + "/" + setTwoDigits(myRTC.month) + "/" + String(myRTC.year);
}

String getTimestampTime() {
  myRTC.updateTime();
  return setTwoDigits(myRTC.hours) + ":" + setTwoDigits(myRTC.minutes) + ":" + setTwoDigits(myRTC.seconds);
}

// Checks if is the correct UTC time to initiate RTC-GPS synchronization
bool validTimeToSync() {
  if ((myRTC.hours == 06 || myRTC.hours == 12 || myRTC.hours == 18 || myRTC.hours == 00) && myRTC.minutes == 00) {  // Original (good) proposal
                                                                                                                    // if ((myRTC.minutes == 37 || myRTC.minutes == 10 || myRTC.minutes == 20 || myRTC.minutes == 30 || myRTC.minutes == 40 || myRTC.minutes == 50) && myRTC.seconds == 00) {  // Every 10 minutes
    // if ((myRTC.hours == 11 || myRTC.hours == 05 || myRTC.hours == 06 || myRTC.hours == 07 || myRTC.hours == 8 || myRTC.hours == 9 || myRTC.hours == 10 || myRTC.hours == 11 || myRTC.hours == 12 || myRTC.hours == 13 || myRTC.hours == 14) && myRTC.minutes == 00) {
    return true;
  } else {
    return false;
  }
}

// Gets UTC time via GPS and syncs RTC
void syncGPSDateTime(bool strictMode) {
  TinyGPSPlus gps;  // The TinyGPS++ object

  int day, month, year, hour, minute, second;
  // String latitude, longitude, altitude;
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  bool gpsDate, gpsTime = false;

  // 9 mins = 540 secs = 540000 ms
  // 3 mins = 180 secs = 180000 ms
  while ((endTime - startTime) <= 180000) {
    if (ss.available() > 0) {
      if (gps.encode(ss.read())) {

        if (strictMode) {

          // Strict Mode: Requires a fixed location to sync time
          if (gps.location.isValid()) {

            if (gps.date.isValid()) {
              day = gps.date.day();
              month = gps.date.month();
              year = gps.date.year();
              gpsDate = true;
            }

            if (gps.time.isValid()) {
              hour = gps.time.hour();
              minute = gps.time.minute();
              second = gps.time.second();
              gpsTime = true;
            }

            if (gpsDate && gpsTime) {
              if (day > 0 && day <= 31 && month <= 12 && year >= 2023) {
                // Update RTC with GPS UTC time
                myRTC.setDS1302Time(second, minute, hour, 1, day, month, year);  // SS, MM, HH, DW, DD, MM, YYYY
                clearMessage();
                tft.setCursor(12, 213);
                tft.setTextColor(GREEN);
                tft.setTextSize(2);
                tft.println("[OK] Success (SM)");
                delay(2000);
                break;
              }
            }

          }  // End if location

        } else {

          // Loose Mode: Syncs with whatever value Satellites provides
          if (gps.date.isValid()) {
            day = gps.date.day();
            month = gps.date.month();
            year = gps.date.year();
            gpsDate = true;
          }

          if (gps.time.isValid()) {
            hour = gps.time.hour();
            minute = gps.time.minute();
            second = gps.time.second();
            gpsTime = true;
          }


          if (gpsDate && gpsTime) {
            if (day > 0 && day <= 31 && month <= 12 && year >= 2023) {
              // Update RTC with GPS UTC time
              myRTC.setDS1302Time(second, minute, hour, 1, day, month, year);  // SS, MM, HH, DW, DD, MM, YYYY
              clearMessage();
              tft.setCursor(12, 213);
              tft.setTextColor(GREEN);
              tft.setTextSize(2);
              tft.println("[OK] Success (LM)");
              delay(2000);
              break;
            }
          }
        }  // End else



      }  // end if gps.encode(ss.read())
    }    // end if ss.available()

    endTime = millis();
  }  // end while 3 or 9 mins
}

// Gets latitude, longitude and altitude data from GPS
void getGPSData() {
  TinyGPSPlus gps;  // The TinyGPS++ object

  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  // 5 mins = 300 sec
  // 9 mins = 540 secs = 540000 ms
  // 3 mins = 180 secs = 180000 ms
  while ((endTime - startTime) <= 30000) {  // 30 secs
    if (ss.available() > 0) {
      if (gps.encode(ss.read())) {

        if (gps.location.isValid()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();

          if (gps.altitude.meters() != 0.0) {
            // writeFile(6, "GPSSYNC.txt");
            altitude = gps.altitude.meters();
            clearMessage();
            tft.setCursor(12, 213);
            tft.setTextColor(GREEN);
            tft.setTextSize(2);
            tft.println("[OK] Success");
            delay(2000);
            break;
          }
        }
      }
    }

    endTime = millis();
  }
}

/*

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

// 6 bytes size and 2 elements (3 bytes each(?))
struct dataStruct {
  int mode;
  float voltage;
  float internalTemp;  // dedicated temperature DSB18B20 sensor
  // float solarCurrent; // Future val: current generated by solar arrays
} systemData

*/

// This assumes constant querying to RTC to know exact time so we don't miss second 0 of each hour.
// Flag 1 for telemetry and 2 for system data.
bool validTimeToSave(int flag) {
  myRTC.updateTime();

  switch (flag) {
    case 1:
      if (myRTC.seconds == lastSecondSavedTel && myRTC.minutes == lastMinuteSavedTel) {
        return false;
      }
      break;
    case 2:
      if (myRTC.seconds == lastSecondSavedSys && myRTC.minutes == lastMinuteSavedSys) {
        return false;
      }
      break;
  }

  // if (myRTC.seconds == 0) {  // saves every minute
  // Saves every 5 seconds
  if (myRTC.seconds == 0 || myRTC.seconds == 5 || myRTC.seconds == 10 || myRTC.seconds == 15 || myRTC.seconds == 20 || myRTC.seconds == 25 || myRTC.seconds == 30 || myRTC.seconds == 35 || myRTC.seconds == 40 || myRTC.seconds == 45 || myRTC.seconds == 50 || myRTC.seconds == 55) {
    return true;
  }

  return false;
}

// TODO: Add function that creates a new txt file when date.month changes.
void writeToSD(int flag) {
  // Just one file can be open at a time, make sure to close it after writing.
  switch (flag) {
    case 1:  // Save RTWI telemetry data: Checking validTime
      if (validTimeToSave(1)) {
        writeFile(1, "2_RTWI/RTWITEL.txt");
      }
      break;
    case 2:  // Save RT system data: Checking validTime
      if (validTimeToSave(2)) {
        writeFile(2, "2_RTWI/RTSYS.txt");
      }
      break;
    case 3:  // Safe mode: SUDO save system data
      writeFile(3, "0_SM/SMSYS.txt");
      break;
    case 4:  // ESM mode: SUDO save telemetry data: Don't check validTime
      writeFile(4, "1_ESM/ESMTEL.txt");
      break;
    case 5:  // ESM mode: SUDO save system data: Don't check validTime
      writeFile(5, "1_ESM/ESMSYS.txt");
      break;
    case 6:  // SUDO Save telemetry data every hour (regardless of CubeSat mode)
      writeFile(6, "3_HOUR/HTEL.txt");
      break;
    case 7:  // SUDO Save system data every hour (regardless of CubeSat mode)
      writeFile(7, "3_HOUR/HSYS.txt");
      break;
  }
}

void writeFile(int flag, String fileName) {
  File myFile;
  String time = getTimestampTime();
  String date = getTimestampDate();
  myFile = SD.open(fileName, FILE_WRITE);

  // Add modes 1 and 3 for saving telemetry data. 1: data every x time, 3: always save data
  // Add modes 2 and 4 for saving system data. 2: data every x time, 4: always save data
  if (flag == 1 || flag == 4 || flag == 6) {  // Save telemetry data
    if (myFile) {
      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(String(telemetryData.humidity) + ",");
      myFile.print(String(telemetryData.temperature) + ",");
      myFile.print(String(telemetryData.pressure) + ",");
      myFile.print(String(telemetryData.localAltitude) + ",");
      myFile.print(String(telemetryData.pitch) + ",");
      myFile.print(String(telemetryData.roll) + ",");
      myFile.print(String(telemetryData.yaw));
      myFile.print("\n");

      myFile.close();

      if (flag == 1) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("\nERR txt 1-4-6");
    }
  } else if (flag == 2 || flag == 3 || flag == 5 || flag == 7) {  //save system data
    if (myFile) {

      myFile.print(time + ",");
      myFile.print(date + ",");
      myFile.print(String(int(systemData.mode)) + ",");
      myFile.print(String(systemData.voltage) + ",");
      myFile.print(String(systemData.internalTemp));
      myFile.print("\n");

      myFile.close();

      if (flag == 2) {
        lastSecondSavedTel = myRTC.seconds;
        lastMinuteSavedTel = myRTC.minutes;
      }

    } else {
      // TODO: consider trying to initialize the sd card again
      Serial.print("\nERR txt 2-3-5-7");
    }
  }
}


// *****************************************************************************
// GUI internal functions ******************************************************


void yled(int xled) {  // "flashes" the "yellow" LED
  for (i = xled; i >= 0; i -= 1) {
    digitalWrite(greenled, LOW);
    digitalWrite(redled, HIGH);
    delay(1);
    digitalWrite(greenled, HIGH);
    digitalWrite(redled, LOW);
    delay(1);
  }
  digitalWrite(greenled, LOW);
  if (greenflag == 1) {
    digitalWrite(redled, LOW);
    digitalWrite(greenled, HIGH);
  }
  if (redflag == 1) {
    digitalWrite(greenled, LOW);
    digitalWrite(redled, HIGH);
  }
}

void redraw() {  // redraw the page
  if ((prevpage != 6) || (page != 7)) {
    clearCenter();
  }
  if (page == 0) {
    homescr();
  }
  if (page == 1) {
    menu1();
  }
  if (page == 2) {
    menu2();
  }
  if (page == 3) {
    menu3();
  }
  if (page == 4) {
    menu4();
  }
  if (page == 5) {
    menu5();
  }
  if (page == 6) {
    settingsScr();
  }
}

void clearCenter() {  // the reason for so many small "boxes" is that it's faster than filling the whole thing
  // modified:
  // tft.drawRect(0, 20, 150, 50, BLACK);
  // tft.drawRect(170, 20, 150, 50, BLACK);
  // tft.drawRect(0, 80, 150, 50, BLACK);
  // tft.drawRect(170, 80, 150, 50, BLACK);
  // tft.drawRect(0, 140, 150, 50, BLACK);
  // tft.drawRect(170, 140, 150, 50, BLACK);
  // tft.fillRect(22, 37, 148, 16, BLACK);
  // tft.fillRect(192, 37, 148, 16, BLACK);
  // tft.fillRect(22, 97, 148, 16, BLACK);
  // tft.fillRect(192, 97, 148, 16, BLACK);
  // tft.fillRect(22, 157, 148, 16, BLACK);
  // tft.fillRect(192, 157, 148, 16, BLACK);

  // Update: Better try to fill the whole space so graphs are erased.
  tft.fillRect(0, 10, 320, 190, BLACK);
}

void clearSettings() {  // this is used to erase the extra drawings when exiting the settings page
  tft.fillRect(0, 20, 320, 110, BLACK);
  delay(500);
  clearMessage();
}

void homescr() {
  lastPage = 0;
  boxes(6);
  enableArea = true;
  clearMessage();

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);

  if (currentMillis - lastDataReceived >= losTolerance) {
    tft.println("[!!!] No data from CubeSat    Delta-1 Mission");
  } else {
    tft.println("Mission Control Center        Delta-1 Mission");
  }


  tft.setCursor(41, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("C&DH");

  tft.setCursor(210, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("GNC");

  tft.setCursor(41, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("SPARTAN");

  tft.setCursor(210, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("SCI PLD");

  tft.setCursor(41, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("DCAM");

  tft.setCursor(200, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Settings");
}

void menu1() {
  boxes(4);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Command & Data Handling");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Telemetry");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("OBC");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("SC Modes");

  tft.setCursor(192, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Payloads");

  // tft.setCursor(22, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 1 B5");

  // tft.setCursor(192, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 1 B6");
}

void menu2() {
  boxes(5);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Guidance, Navigation & Control");


  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  // tft.println("Menu 2 B1");
  tft.println("GPS Alt");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("GPS Coord");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Alt");

  tft.setCursor(192, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Magn Decl");

  tft.setCursor(22, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Graphs");

  // tft.setCursor(192, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 2 B6");
}

void menu3() {
  boxes(4);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Spacecraft Power, Art. & Thermal Control");


  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  // tft.println("Menu 3 B1");
  tft.println("Batt Stat");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("SC Temp");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Batt Graph");

  tft.setCursor(192, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temp Graph");

  // tft.setCursor(22, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 3 B5");

  // tft.setCursor(192, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 3 B6");
}

void menu4() {
  boxes(6);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Science Payload Instruments");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temp & Hum");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Heat Index");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Air Press");

  tft.setCursor(192, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Alt");

  tft.setCursor(22, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("IMU");

  tft.setCursor(192, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Summary");
}

void menu5() {
  boxes(2);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Delta Cameras File System");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("IRCAM");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("NavCam");

  // tft.setCursor(22, 97);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 5 B3");

  // tft.setCursor(192, 97);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 5 B4");

  // tft.setCursor(22, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 5 B5");

  // tft.setCursor(192, 157);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(2);
  // tft.println("Menu 5 B6");
}

void settingsScr() {

  page = 61;

  // backlight level
  tft.fillRect(0, 20, 60, 50, RED);
  tft.drawRect(0, 20, 60, 50, WHITE);
  tft.drawRect(80, 20, 160, 50, JJCOLOR);
  tft.fillRect(260, 20, 60, 50, GREEN);
  tft.drawRect(260, 20, 60, 50, WHITE);

  tft.setCursor(22, 33);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("-");

  tft.setCursor(120, 31);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);  // Note: It seems like has no size defined
  tft.println("Backlight Level");

  tft.setCursor(282, 33);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("+");
  tft.drawRect(110, 48, 100, 10, WHITE);
  blBar();
  // sleep time
  tft.fillRect(0, 80, 60, 50, RED);
  tft.drawRect(0, 80, 60, 50, WHITE);
  tft.drawRect(80, 80, 160, 50, JJCOLOR);
  tft.fillRect(260, 80, 60, 50, GREEN);
  tft.drawRect(260, 80, 60, 50, WHITE);

  tft.setCursor(22, 93);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("-");

  tft.setCursor(130, 91);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Sleep Time");

  tft.setCursor(282, 93);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.println("+");
  showSleep();

  tft.drawRect(0, 140, 150, 50, JJCOLOR);
  tft.setCursor(41, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("RTC-GPS");

  //?? uncomment this if you want a third adjustable option
  /*
  tft.fillRect(0, 140, 60, 50, RED);
   tft.drawRect(0, 140, 60, 50, WHITE);
   tft.drawRect(80, 140, 160, 50, JJCOLOR);
   tft.fillRect(260, 140, 60, 50, GREEN);
   tft.drawRect(260, 140, 60, 50, WHITE);
   tft.drawString(22, 153, "-", WHITE, 3);
   tft.drawString(130, 151, "Thing #3", WHITE);
   tft.drawString(282, 153, "+", WHITE, 3);
   tft.drawRect(110, 168, 100, 10, WHITE);
   */
  battv = readVcc();  // read the voltage
  itoa(battv, voltage, 10);
  // tft.drawString(12, 213, voltage, YELLOW, 2);
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.println(voltage);
  // tft.drawString(60, 213, "mV", YELLOW, 2);
  tft.setCursor(60, 213);
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.println("mV");
  /*
  battpercent = (battv / 5000) * 100, 2;
  itoa (battpercent, battpercenttxt, 10);
  tft.drawString(102, 213, battpercenttxt, YELLOW, 2);
  */
}

void sleepInc() {  // sleep increese adjustment
  if (sleeptime == 14400000) {
    sleepnever = 1;
    esleep = 12;
    sleeptime = 11111111;
    showSleep();
  }
  if (sleeptime == 3600000) {
    sleeptime = 14400000;
    esleep = 11;
    showSleep();
  }
  if (sleeptime == 1800000) {
    sleeptime = 3600000;
    esleep = 10;
    showSleep();
  }
  if (sleeptime == 1200000) {
    sleeptime = 1800000;
    esleep = 9;
    showSleep();
  }
  if (sleeptime == 600000) {
    sleeptime = 1200000;
    esleep = 8;
    showSleep();
  }
  if (sleeptime == 300000) {
    sleeptime = 600000;
    esleep = 7;
    showSleep();
  }
  if (sleeptime == 120000) {
    sleeptime = 300000;
    esleep = 6;
    showSleep();
  }
  if (sleeptime == 60000) {
    sleeptime = 120000;
    esleep = 5;
    showSleep();
  }
  if (sleeptime == 30000) {
    sleeptime = 60000;
    esleep = 4;
    showSleep();
  }
  if (sleeptime == 20000) {
    sleeptime = 30000;
    esleep = 3;
    showSleep();
  }
  if (sleeptime == 10000) {
    sleeptime = 20000;
    esleep = 2;
    showSleep();
  }
  delay(350);
}

void sleepDec() {  // sleep decreese adjustment
  if (sleeptime == 20000) {
    sleeptime = 10000;
    esleep = 1;
    showSleep();
  }
  if (sleeptime == 30000) {
    sleeptime = 20000;
    esleep = 2;
    showSleep();
  }
  if (sleeptime == 60000) {
    sleeptime = 30000;
    esleep = 3;
    showSleep();
  }
  if (sleeptime == 120000) {
    sleeptime = 60000;
    esleep = 4;
    showSleep();
  }
  if (sleeptime == 300000) {
    sleeptime = 120000;
    esleep = 5;
    showSleep();
  }
  if (sleeptime == 600000) {
    sleeptime = 300000;
    esleep = 6;
    showSleep();
  }
  if (sleeptime == 1200000) {
    sleeptime = 600000;
    esleep = 7;
    showSleep();
  }
  if (sleeptime == 1800000) {
    sleeptime = 1200000;
    esleep = 8;
    showSleep();
  }
  if (sleeptime == 3600000) {
    sleeptime = 1800000;
    esleep = 9;
    showSleep();
  }
  if (sleeptime == 14400000) {
    sleeptime = 3600000;
    esleep = 10;
    showSleep();
  }
  if (sleepnever == 1) {
    sleeptime = 14400000;
    sleepnever = 0;
    esleep = 11;
    showSleep();
  }
  delay(350);
}

void showSleep() {  // shows the sleep time on the settings page
  tft.fillRect(110, 108, 80, 10, BLACK);
  if (sleeptime == 10000) {
    // tft.drawString(130, 108, "10 Seconds", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("10 Seconds");
  }
  if (sleeptime == 20000) {
    // tft.drawString(130, 108, "20 Seconds", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("20 Seconds");
  }
  if (sleeptime == 30000) {
    // tft.drawString(130, 108, "30 Seconds", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("30 Seconds");
  }
  if (sleeptime == 60000) {
    // tft.drawString(136, 108, "1 Minute", WHITE);
    tft.setCursor(136, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("1 Minute");
  }
  if (sleeptime == 120000) {
    // tft.drawString(133, 108, "2 Minutes", WHITE);
    tft.setCursor(133, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("2 Minutes");
  }
  if (sleeptime == 300000) {
    // tft.drawString(133, 108, "5 Minutes", WHITE);
    tft.setCursor(133, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("5 Minutes");
  }
  if (sleeptime == 600000) {
    // tft.drawString(130, 108, "10 Minutes", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("10 Minute");
  }
  if (sleeptime == 1200000) {
    // tft.drawString(130, 108, "20 Minutes", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("20 Minute");
  }
  if (sleeptime == 1800000) {
    // tft.drawString(130, 108, "30 Minutes", WHITE);
    tft.setCursor(130, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("30 Minute");
  }
  if (sleeptime == 3600000) {
    // tft.drawString(142, 108, "1 Hour", WHITE);
    tft.setCursor(142, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("1 Hour");
  }
  if (sleeptime == 14400000) {
    // tft.drawString(139, 108, "4 Hours", WHITE);
    tft.setCursor(139, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("4 Hour");
  }
  if (sleepnever == 1) {
    // tft.drawString(133, 108, "Always On", WHITE);
    tft.setCursor(133, 108);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.println("Always On");
  }
}

void option3down() {  // adjust option 3 down in the settings screen
}

void option3up() {  // adjust option 3 up in the settings screen
}

//custom defined actions - this is where you put your button functions

void m1b1action() {
  lastPage = 11;
  clearCenter();
  enableArea = false;

  // Modes -> 0: Safe Mode, 1: ESM, 2: RTW, 3: RTI, 4: RTWI
  String stringMode;
  switch (systemData.mode) {
    case -1:
      stringMode = " (UNKN)";
      break;
    case 0:
      stringMode = " (SAFE)";
      break;
    case 1:
      stringMode = " (ESM)";
      break;
    case 2:
      stringMode = " (RTW)";
      break;
    case 3:
      stringMode = " (RTI)";
      break;
    case 4:
      stringMode = " (RTWI)";
      break;
  }

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("CubeSat Mode: " + String(systemData.mode) + stringMode);

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Internal Temp: " + String(systemData.internalTemp) + "\367" + "C");

  tft.setCursor(22, 157);  // 22, 157
  tft.setTextColor(YELLOW);
  tft.setTextSize(1);
  tft.println("Last transmission rcvd:");

  tft.setCursor(22, 177);
  tft.setTextColor(MARS);
  tft.setTextSize(1);
  tft.println(rcvdTime + " UTC");
}

void m1b2action() {
  clearCenter();
  boxes(3);
  page = 12;

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("On-board Computers");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Reboot OBC");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Rtrv Data");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Del Data");
}

void settingsb1action() {
  page = 611;
  clearCenter();
  boxes(2);

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("RTC-GPS");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Sync RTC");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("GPS data");
}

void settingsb11action() {
  clearCenter();
  enableArea = false;
  clearMessage();

  tft.setCursor(12, 213);
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.println("Syncing RTC-GPS ...");

  syncGPSDateTime(false);  // Lose Mode

  String time = getTimestampTime();
  String date = getTimestampDate();

  tft.setCursor(22, 37);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("Mission Clock synced at:");

  tft.setCursor(22, 77);
  tft.setTextColor(MARS);
  tft.setTextSize(2);
  tft.println(time + " UTC");

  tft.setCursor(22, 117);
  tft.setTextColor(MARS);
  tft.setTextSize(2);
  tft.println(date);
}

void settingsb12action() {
  clearCenter();
  enableArea = false;
  clearMessage();

  tft.setCursor(12, 213);
  tft.setTextColor(YELLOW);
  tft.setTextSize(2);
  tft.println("RCV GPS data ...");

  getGPSData();

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Latitude: " + latitude + "\367");

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Longitude: " + longitude + "\367");

  tft.setCursor(22, 117);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Altitude: " + altitude + " m");
}

void m1b3action() {
  clearCenter();
  boxes(5);
  page = 13;

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Spacecraft Modes");

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Act ESM");

  tft.setCursor(192, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("RTWI 11");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Act RTI");

  tft.setCursor(192, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Act RTWI");

  tft.setCursor(22, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Hibernate");
}

void m1b4action() {
}

void m1b5action() {
}

void m1b6action() {
}

void m2b1action() {
}

void m2b2action() {
}

void m2b3action() {
  lastPage = 23;
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(telemetryData.localAltitude) + " m");
}

void m2b4action() {
}

void m2b5action() {
}

void m2b6action() {
}

void m3b1action() {
}

void m3b2action() {
}

void m3b3action() {
}

void m3b4action() {
}

void m3b5action() {
}

void m3b6action() {
}

void m4b1action() {
  lastPage = 41;
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temperature: " + String(telemetryData.temperature) + "\367" + "C");

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Humidity: " + String(telemetryData.humidity) + "%");
}

void m4b2action() {
  lastPage = 42;
  clearCenter();
  enableArea = false;
  heatIndex = computeHeatIndex(telemetryData.temperature, telemetryData.humidity);

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Heat Index: " + String(heatIndex) + "\367" + "C");
}

void m4b3action() {
  lastPage = 43;
  page = 43;  // set current page so area 5 is listened
  clearCenter();
  enableArea = true;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Barometer: " + String(telemetryData.pressure) + " hPa");

  buttonGraph();  // 431 Page 4, button 3, sub-button 1
}

void m43b5action() {
  lastPage = 1;  // set it to a page that does not refresh (maybe 1?)
  clearCenter();
  enableArea = false;

  plotGraph(1, true);
  // 1:barometer, 2: temperature
  // true: ESM, false:RTWI
}

void m4b4action() {
  lastPage = 44;
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(telemetryData.localAltitude) + " m");
}

void m4b5action() {
  lastPage = 45;
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 27);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Inertial Measurement U: ");

  tft.setCursor(22, 57);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("X: " + String(telemetryData.pitch) + +"\367");

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Y: " + String(telemetryData.roll) + +"\367");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Z: " + String(telemetryData.yaw) + +"\367");
}

void m4b6action() {
  lastPage = 46;
  clearCenter();
  enableArea = false;
  heatIndex = computeHeatIndex(telemetryData.temperature, telemetryData.humidity);

  tft.setCursor(22, 27);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temperature: " + String(telemetryData.temperature) + "\367" + "C");

  tft.setCursor(22, 47);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Humidity: " + String(telemetryData.humidity) + "%");

  tft.setCursor(22, 67);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Heat Index: " + String(heatIndex) + "\367" + "C");

  tft.setCursor(22, 87);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Barometer: " + String(telemetryData.pressure) + " hPa");

  tft.setCursor(22, 107);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(telemetryData.localAltitude) + " m");

  tft.setCursor(22, 127);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("X: " + String(telemetryData.pitch) + +"\367");

  Serial.println("m4b6 Pitch: " + String(telemetryData.pitch));

  tft.setCursor(22, 147);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Y: " + String(telemetryData.roll) + +"\367");

  tft.setCursor(22, 167);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Z: " + String(telemetryData.yaw) + +"\367");

  // m4b6action();

  // unsigned long startTime = millis();
  // unsigned long endTime = startTime;
  // while ((endTime - startTime) <= 10000) {
  //   // m4b6action();

  //   tft.setCursor(22, 27);
  //   tft.setTextColor(WHITE);
  //   tft.setTextSize(2);
  //   tft.println("Temperature: " + String(telemetryData.temperature) + "\367" + "C");

  //   telemetryData.temperature += 1;
  //   delay(1000);
  //   endTime = millis();
  // }

  // Serial.println("rcvdTx: " + String(rcvdTx));

  // while (rcvdTx == false) {
  //   tft.setCursor(22, 27);
  //   tft.setTextColor(WHITE);
  //   tft.setTextSize(2);
  //   tft.println("Temperature: " + String(telemetryData.temperature) + "\367" + "C");

  //   telemetryData.temperature += 1;

  //   delay(1000);
  // }

  // while (1) {
  //   Serial.println("rcvdTx: " + String(rcvdTx));
  //   delay(1000);
  // }
}

void m5b1action() {
}

void m5b2action() {
}

void m5b3action() {
}

void m5b4action() {
}

void m5b5action() {
}

void m5b6action() {
}

// Sub-menu definitions

void m12b1action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Rebooting OBC ...");
  yled(550);
  clearMessage();
}

void m12b2action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Retrieving data ...");
  yled(550);
  clearMessage();
}

void m12b3action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Deleting data ...");
  yled(550);
  clearMessage();
}

void m13b1action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  // tft.println("Sending TX for ESM OFF ...");
  tft.println("TX ESM ON ...");

  // Transmit command 3 seconds
  sendCommand(1.0, 3);  // activate ESM

  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("[OK] ESM requested");
  delay(2000);
}

void m13b2action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  // tft.println("Sending TX for ESM OFF ...");
  tft.println("TX ESM OFF ...");

  // Transmit command  for 11 minutes, but listen in between for incomming transmission. 660 secs
  sendCommandAndListen(5.0, 660, false);  // Transmit 11 mins to activate RTWI mode on Delta1, false for altitude reset
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("[OK] RTWI requested");
  delay(2000);
}

void m13b3action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  // tft.println("Sending TX for ESM OFF ...");
  tft.println("TX RTI ON ...");

  // Transmit command 3 seconds
  sendCommand(4.0, 3);  // Activate RTI mode

  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("[OK] RTI requested");
  delay(2000);
}

void m13b4action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  // tft.println("Sending TX for ESM OFF ...");
  tft.println("TX RTWI ON ...");

  // Transmit command 3 seconds
  sendCommand(5.0, 3);  // Activate RTWI mode

  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(GREEN);
  tft.setTextSize(2);
  tft.println("[OK] RTWI requested");
  delay(2000);
}

void m13b5action() {
  clearMessage();
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Hibernation ...");
  yled(550);
  clearMessage();
}

void blightUp() {  // increase the backlight brightness
  blv = blv + 5;
  if (blv >= 255) {
    blv = 255;
  }
  analogWrite(10, blv);
  blBar();
}

void blightDown() {  // decrease the backlight brightness
  blv = blv - 5;
  if (blv <= 5) {
    blv = 5;
  }
  analogWrite(10, blv);
  blBar();
}

void blBar() {  // this function fills the yellow bar in the backlight brightness adjustment
  if (blv < barv) {
    tft.fillRect(111, 49, 98, 8, BLACK);
  }
  backlightbox = map(blv, 1, 255, 0, 98);
  tft.fillRect(111, 49, backlightbox, 8, YELLOW);
  barv = blv;
  delay(25);
}

void buttonGraph() {
  tft.drawRect(0, 140, 150, 50, JJCOLOR);  // 5th position
  tft.setCursor(41, 157);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Graphs");

  // set action with the lastPage pulsed
}

void boxes(int howMany) {  // redraw the button outline boxes
  switch (howMany) {
    case 1:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      break;
    case 2:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      tft.drawRect(170, 20, 150, 50, JJCOLOR);
      break;
    case 3:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      tft.drawRect(170, 20, 150, 50, JJCOLOR);
      tft.drawRect(0, 80, 150, 50, JJCOLOR);
      break;
    case 4:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      tft.drawRect(170, 20, 150, 50, JJCOLOR);
      tft.drawRect(0, 80, 150, 50, JJCOLOR);
      tft.drawRect(170, 80, 150, 50, JJCOLOR);
      break;
    case 5:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      tft.drawRect(170, 20, 150, 50, JJCOLOR);
      tft.drawRect(0, 80, 150, 50, JJCOLOR);
      tft.drawRect(170, 80, 150, 50, JJCOLOR);
      tft.drawRect(0, 140, 150, 50, JJCOLOR);
      break;
    case 6:
      tft.drawRect(0, 20, 150, 50, JJCOLOR);
      tft.drawRect(170, 20, 150, 50, JJCOLOR);
      tft.drawRect(0, 80, 150, 50, JJCOLOR);
      tft.drawRect(170, 80, 150, 50, JJCOLOR);
      tft.drawRect(0, 140, 150, 50, JJCOLOR);
      tft.drawRect(170, 140, 150, 50, JJCOLOR);
      break;
  }
}

void ant() {
  tft.fillRect((antpos + 5), 4, 1, 6, WHITE);  // draws the "antenna" for the signal indicator
}

void signal(bool green) {  // draws a white 'signal indicator'
  if (green) {
    tft.drawLine((antpos + 4), 4, (antpos + 4), 5, GREEN);
    tft.drawPixel((antpos + 3), 2, GREEN);
    tft.drawPixel((antpos + 3), 7, GREEN);
    tft.drawPixel((antpos + 2), 0, GREEN);
    tft.drawLine((antpos + 2), 3, (antpos + 2), 6, GREEN);
    tft.drawPixel((antpos + 2), 9, GREEN);
    tft.drawPixel((antpos + 1), 1, GREEN);
    tft.drawPixel((antpos + 1), 8, GREEN);
    tft.drawLine(antpos, 2, antpos, 7, GREEN);
    tft.drawLine((antpos + 6), 4, (antpos + 6), 5, GREEN);
    tft.drawPixel((antpos + 7), 2, GREEN);
    tft.drawPixel((antpos + 7), 7, GREEN);
    tft.drawPixel((antpos + 8), 0, GREEN);
    tft.drawLine((antpos + 8), 3, (antpos + 8), 6, GREEN);
    tft.drawPixel((antpos + 8), 9, GREEN);
    tft.drawPixel((antpos + 9), 1, GREEN);
    tft.drawPixel((antpos + 9), 8, GREEN);
    tft.drawLine((antpos + 10), 2, (antpos + 10), 7, GREEN);
  } else {
    tft.drawLine((antpos + 4), 4, (antpos + 4), 5, WHITE);
    tft.drawPixel((antpos + 3), 2, WHITE);
    tft.drawPixel((antpos + 3), 7, WHITE);
    tft.drawPixel((antpos + 2), 0, WHITE);
    tft.drawLine((antpos + 2), 3, (antpos + 2), 6, WHITE);
    tft.drawPixel((antpos + 2), 9, WHITE);
    tft.drawPixel((antpos + 1), 1, WHITE);
    tft.drawPixel((antpos + 1), 8, WHITE);
    tft.drawLine(antpos, 2, antpos, 7, WHITE);
    tft.drawLine((antpos + 6), 4, (antpos + 6), 5, WHITE);
    tft.drawPixel((antpos + 7), 2, WHITE);
    tft.drawPixel((antpos + 7), 7, WHITE);
    tft.drawPixel((antpos + 8), 0, WHITE);
    tft.drawLine((antpos + 8), 3, (antpos + 8), 6, WHITE);
    tft.drawPixel((antpos + 8), 9, WHITE);
    tft.drawPixel((antpos + 9), 1, WHITE);
    tft.drawPixel((antpos + 9), 8, WHITE);
    tft.drawLine((antpos + 10), 2, (antpos + 10), 7, WHITE);
  }
}

void signalAct() {  // draws a red'signal indicator'
  tft.drawLine((antpos + 4), 4, (antpos + 4), 5, RED);
  tft.drawPixel((antpos + 3), 2, RED);
  tft.drawPixel((antpos + 3), 7, RED);
  tft.drawPixel((antpos + 2), 0, RED);
  tft.drawLine((antpos + 2), 3, (antpos + 2), 6, RED);
  tft.drawPixel((antpos + 2), 9, RED);
  tft.drawPixel((antpos + 1), 1, RED);
  tft.drawPixel((antpos + 1), 8, RED);
  tft.drawLine(antpos, 2, antpos, 7, RED);
  tft.drawLine((antpos + 6), 4, (antpos + 6), 5, RED);
  tft.drawPixel((antpos + 7), 2, RED);
  tft.drawPixel((antpos + 7), 7, RED);
  tft.drawPixel((antpos + 8), 0, RED);
  tft.drawLine((antpos + 8), 3, (antpos + 8), 6, RED);
  tft.drawPixel((antpos + 8), 9, RED);
  tft.drawPixel((antpos + 9), 1, RED);
  tft.drawPixel((antpos + 9), 8, RED);
  tft.drawLine((antpos + 10), 2, (antpos + 10), 7, RED);
}

void drawHomeIcon() {  // draws a white home icon
  tft.drawLine(280, 219, 299, 200, WHITE);
  tft.drawLine(300, 200, 304, 204, WHITE);
  tft.drawLine(304, 203, 304, 200, WHITE);
  tft.drawLine(305, 200, 307, 200, WHITE);
  tft.drawLine(308, 200, 308, 208, WHITE);
  tft.drawLine(309, 209, 319, 219, WHITE);
  tft.drawLine(281, 219, 283, 219, WHITE);
  tft.drawLine(316, 219, 318, 219, WHITE);
  tft.drawRect(284, 219, 32, 21, WHITE);
  tft.drawRect(295, 225, 10, 15, WHITE);
}

void drawHomeIconRed() {  // draws a red home icon
  tft.drawLine(280, 219, 299, 200, RED);
  tft.drawLine(300, 200, 304, 204, RED);
  tft.drawLine(304, 203, 304, 200, RED);
  tft.drawLine(305, 200, 307, 200, RED);
  tft.drawLine(308, 200, 308, 208, RED);
  tft.drawLine(309, 209, 319, 219, RED);
  tft.drawLine(281, 219, 283, 219, RED);
  tft.drawLine(316, 219, 318, 219, RED);
  tft.drawRect(284, 219, 32, 21, RED);
  tft.drawRect(295, 225, 10, 15, RED);
}

void clearMessage() {
  tft.fillRect(12, 213, 226, 16, BLACK);  // black out the inside of the message box
}

void clearMessageStatusBar() {
  tft.fillRect(0, 0, 278, 10, JJCOLOR);  // status bar
}

void drawBatt() {
  battv = readVcc();      // read the voltage
  if (battv < battold) {  // if the voltage goes down, erase the inside of the battery
    tft.fillRect(298, 2, 18, 6, BLACK);
  }
  battfill = map(battv, 3000, 4150, 2, 18);  // map the battery voltage 3000 nis the low, 4150 is the high
  if (battfill > 7) {                        // if the battfill value is between 8 and 18, fill with green
    tft.fillRect(298, 2, battfill, 6, GREEN);
  } else {  // if the battfill value is below 8, fill with red
    tft.fillRect(298, 2, battfill, 6, RED);
  }
  battold = battv;  // this helps determine if redrawing the battfill area is necessary
}

void drawClock(String time) {

  /*
    Updates the mission clock and displays it on screen.
    It should be updated every second

  */

  clearMessage();

  tft.setCursor(12, 213);
  tft.setTextColor(MARS);
  tft.setTextSize(2);
  tft.println(time);

  tft.setCursor(120, 213);
  tft.setTextColor(MARS);
  tft.setTextSize(2);
  tft.println("UTC");
}

void plotGraph(int type, bool esm) {
  // 1:barometer, 2: temperature
  // true: ESM, false:RTWI

  /*
    Steps: 
    1) read data from SD card
    2) ...
  */

  /*
    x：x coordinate of the starting point
    y：y coordinate of the starting point
    w：the length of the square
    h：the width of the square
    color：the color of the square
  */


  // tft.setCursor(190, 27);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(1);
  // tft.println("Air Pressure");

  // Plot graph bars
  tft.fillRect(20, 15, 3, 145, WHITE);   // Y axis
  tft.fillRect(20, 160, 272, 3, WHITE);  // X axis

  // Y axis cross lines
  tft.drawLine(17, 87, 20, 87, WHITE);
  tft.drawLine(17, 15, 20, 15, WHITE);

  // X axis cross lines (24 lines)
  tft.drawLine(33, 160, 33, 166, WHITE);
  tft.drawLine(43, 160, 43, 166, WHITE);
  tft.drawLine(53, 160, 53, 166, WHITE);
  tft.drawLine(63, 160, 63, 166, WHITE);
  tft.drawLine(73, 160, 73, 166, WHITE);
  tft.drawLine(83, 160, 83, 166, WHITE);
  tft.drawLine(93, 160, 93, 166, WHITE);
  tft.drawLine(103, 160, 103, 166, WHITE);
  tft.drawLine(113, 160, 113, 166, WHITE);
  tft.drawLine(123, 160, 123, 166, WHITE);
  tft.drawLine(133, 160, 133, 166, WHITE);
  tft.drawLine(143, 160, 143, 166, WHITE);
  tft.drawLine(153, 160, 153, 166, WHITE);
  tft.drawLine(163, 160, 163, 166, WHITE);
  tft.drawLine(173, 160, 173, 166, WHITE);
  tft.drawLine(183, 160, 183, 166, WHITE);
  tft.drawLine(193, 160, 193, 166, WHITE);
  tft.drawLine(203, 160, 203, 166, WHITE);
  tft.drawLine(213, 160, 213, 166, WHITE);
  tft.drawLine(223, 160, 223, 166, WHITE);
  tft.drawLine(233, 160, 233, 166, WHITE);
  tft.drawLine(243, 160, 243, 166, WHITE);
  tft.drawLine(253, 160, 253, 166, WHITE);
  tft.drawLine(263, 160, 263, 166, WHITE);  // 24th
  // tft.drawLine(273, 160, 273, 166, WHITE);
  // tft.drawLine(283, 160, 283, 166, WHITE); // 26th

  tft.setCursor(30, 190);
  tft.setTextColor(CYAN);
  tft.setTextSize(1);
  tft.println("Air Pressure");

  // tft.setCursor(125, 190);
  // tft.setTextColor(GREEN);
  // tft.setTextSize(1);
  // tft.println("Heat values");

  // tft.setCursor(212, 190);
  // tft.setTextColor(RED);
  // tft.setTextSize(1);
  // tft.println("Smoke values");

  tft.setCursor(188, 171);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Time (24 hours)");

  tft.setCursor(31, 170);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(scale1);

  tft.setCursor(41, 170);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(scale2);

  tft.setCursor(61, 170);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(scale4);

  tft.setCursor(101, 170);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println(scale8);

  readData();
}

void plotData(int dataPoints[], int dataLength, int minVal, int maxVal) {
  /*
    The 'real' function should read data from SD card
    and extract the needed data points to plot.
    This function should receive a parameter
    indicating which data to retrieve.
  */

  tft.setCursor(10, 164);
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.println(minVal);  // Scale start: Should be the lowest air pressure value (i.e. 776.00, or 1013)

  tft.setCursor(10, 12);
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.println(maxVal);  // Max val on chart

  int medianVal = round((maxVal + minVal) / 2);

  tft.setCursor(10, 87);
  tft.setTextColor(GREEN);
  tft.setTextSize(1);
  tft.println(medianVal);  // Max val on chart

  // tft.drawLine(17, 87, 20, 87, WHITE); // AT MIDDLE
  // tft.drawLine(17, 15, 20, 15, WHITE);

  int mappedPixels[dataLength];

  for (int i = 0; i <= dataLength; i++) {  // sizeof(dataPoints)

    int val = dataPoints[i];

    int fromLow = minVal;   // Min of present pressure // orignally: 0, now 776
    int fromHigh = maxVal;  // Max of present pressure // orignally: 1024 now 781
    int toLow = 159;        // The minimum number of the desired range to which the value is to be mapped
    int toHigh = 15;        // The maximum number of the desired range to which the value is to be mapped

    int valab = map(val, fromLow, fromHigh, toLow, toHigh);  // Map pressure value to a pixel number (between 15 ( = 1024 ) and 159 ( = 0 )) that represent the progress on the Y axis.
    // Idea for some other time: map the int part of val, map the decimal part, and add them together

    // Serial.print("\nMapped pixel: " + String(valab));
    mappedPixels[i] = valab;

    tft.fillCircle(incrementation, valab, 1, CYAN);  // x0：x coordinate of the center point , y0：y coordinate of the center point, r：radius of the circle, color：teh color of the circle
    incrementation += 10;                            // incrementation ++; // 10 or 9 seems fine

    /*
    if (scale1 == 27) {
      tft.fillRect(3, 120, 16, 60, BLACK);
    }
    if (incrementation > 282) {
      tft.fillRect(10, 166, 100, 12, BLACK);
      scale1 = scale1 + 26;
      scale2 = scale2 + 26;
      scale4 = scale4 + 26;
      scale8 = scale8 + 26;
      tft.setCursor(26, 170);
      tft.setTextColor(WHITE);
      tft.setTextSize(1);
      tft.println(scale1);
      tft.setCursor(41, 170);
      if (scale2 > 10) {
        tft.setCursor(39, 170);
      }
      if (scale2 < 100) {
        tft.setTextColor(WHITE);
        tft.setTextSize(1);
        tft.println(scale2);
      }
      tft.setCursor(61, 170);
      tft.setTextColor(WHITE);
      tft.setTextSize(1);
      tft.println(scale4);
      tft.setCursor(101, 170);
      tft.setTextColor(WHITE);
      tft.setTextSize(1);
      tft.println(scale8);
      tft.fillRect(23, 14, 269, 146, BLACK);

      incrementation = 24;
    }
    */


  }  // end for loop
  incrementation = 24;

  for (int i = 0; i < dataLength; i++) {
    // Draw a line between points
    if (i != 0) {
      tft.drawLine(incrementation, mappedPixels[i], incrementation + 10, mappedPixels[i + 1], CYAN);  // (x0,y0, x1,y1, color)
    }
    incrementation += 10;  // Account for next jump
    if (i == int(dataLength) - 1) {
      break;
    }
  }
  incrementation = 24;  // reset it again
}

void readData() {

  /* 
    Function that reads a txt file with CSV 
    style and get time and pressure data.

    From Web example:
    DataType: int, float, float, float, char[8], char[8]
    Variable: tcalc, t1, t2, h1, h2, timeS, dateS
    Delimitor; ','

    ESM real data sample:
    time,date,humidity,temperature,pressure,localAltitude,pitch,roll,yaw
    15:30:00,15/12/2022,47.50,20.10,776.52,0.04,-0.27,-1.37,16.92

    9 vals: 2 time/date and 7 floats (though maybe they're all strings in the txt)
  */


  // open the file for reading:
  String filePath = "3_HOUR/HTEL.txt";
  // String filePath = "24_HOUR/ESM24.txt";
  // String filePath = "72_HOUR/1_ESM/ESMTEL.txt";


  myFile = SD.open(filePath);
  // If we were to write something, then: myFile = SD.open("test.txt", FILE_WRITE);
  int lineCount = 0;
  int lineCount2 = 0;

  if (myFile) {
    Serial.println("Reading " + filePath + ": ");

    // Rewind the file for read.
    // Seeks to a new position in the file, between 0 and size of file (inclusive)
    myFile.seek(68);  // Consider that the frist row is for column names, hence I need to move to first real data index. POS: 68
    // Serial.print("\nPosition after seek(): " + String(myFile.position()));  // 68

    int dataPoints[12];  // '12' to have 24 slots in array Rounded data points from sensor (like pressure)
    int minVal = 3000;   // an amount of hPa that could never be true
    int maxVal = 0;
    int counter = 0;

    // get the line count and then read the last lineCount - 25 to plot the 24h graph

    while (myFile.available()) {
      myFile.readStringUntil('\n');
      lineCount += 1;
    }
    // Serial.println("\nlineCount: " + String(lineCount));

    myFile.seek(68);  // Reset initial position
    unsigned long desiredPosition;
    while (myFile.available()) {
      myFile.readStringUntil('\n');
      lineCount2 += 1;

      if (lineCount2 == lineCount - 25) {  // Warning: could fail if there are 0 registers
        // Get index, position, etc
        // Serial.print("\nlineCount2: " + String(lineCount2));
        desiredPosition = myFile.position();
        break;
      }
    }

    // Serial.println("\ndesiredPosition: " + String(desiredPosition));

    myFile.seek(desiredPosition);
    while (myFile.available()) {

      buffer = myFile.readStringUntil('\n');
      // Serial.println(buffer);

      String pressure = getValue(buffer, ',', 4);  // 4th position for pressure // time,date,humidity,temperature,pressure,localAltitude,pitch,roll,yaw
      Serial.print("\n-----------------\n");
      Serial.println(pressure);
      int val = round(pressure.toFloat());
      Serial.println(val);


      dataPoints[counter] = val;
      counter++;

      // Now I need to get the min and Max vals from an array
      if (val != 0 && val < minVal) {
        // a new minimum is found
        minVal = val;
      }

      if (val > maxVal) {
        // a new maximum is found
        maxVal = val;
      }


    }                // end of while
    myFile.close();  // close the file

    // Serial.print("\nminVal: " + String(minVal));
    // Serial.print("\nmaxVal: " + String(maxVal));

    plotData(dataPoints, sizeof(dataPoints), minVal, maxVal);  // <-- Plotting actual data points

  } else {
    // if the file didn't open, print an error:
    Serial.println("[!!!] Error opening " + filePath);
  }
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}