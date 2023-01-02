/* 
Modified on Nov 25, 2020
Modified by MehranMaleki from Arduino Examples
Home
*/


// CONNECTIONS:
// DS1302 CLK/SCLK --> 5
// DS1302 DAT/IO --> 4
// DS1302 RST/CE --> 2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND

#include <ThreeWire.h>
#include <RtcDS1302.h>

ThreeWire myWire(7, 6, 8);  // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

void setup() {
  Serial.begin(9600);

  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  Rtc.Begin();

  // RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  RtcDateTime compiled = RtcDateTime("Oct 31 2024", "17:25:05");
  Rtc.SetDateTime(compiled);
  
  // RtcDateTime manual = RtcDateTime("Jan 25 2017", "16:30:00");
  // Rtc.SetDateTime(manual);

  
  printDateTime(compiled);
  Serial.println();

  if (!Rtc.IsDateTimeValid()) {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected()) {
    Serial.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  } else if (now > compiled) {
    Serial.println("RTC is newer than compile time. (this is expected)");
  } else if (now == compiled) {
    Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }
}

void loop() {
  RtcDateTime now = Rtc.GetDateTime();

  // printDateTime(now);
  // Serial.println();


  // String date = getTimestampDate(now);
  // Serial.print("\ndate: ");
  // Serial.print(date);

  // String time = getTimestampTime(now);
  // Serial.print("\ntime: ");
  // Serial.print(time);

  byte seconds = now.Second();
  Serial.print("\nTest: ");
  Serial.print(seconds);


  if (!now.IsValid()) {
    // Common Causes:
    //    1) the battery on the device is low or even missing and the power line was disconnected
    Serial.println("RTC lost confidence in the DateTime!");
  }

  delay(5000);  // five seconds
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt) {
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
}

String getTimestampDate(const RtcDateTime& dt) {
  
  // byte minutes = dt.Second();
  // Serial.print("\nTest: ");
  // Serial.print(minutes);

  char date[11];
  snprintf_P(date,
             countof(date),
             PSTR("%02u/%02u/%04u"),
             dt.Day(),
             dt.Month(),             
             dt.Year());
  // Serial.print(date);  
  return date;
}

String getTimestampTime(const RtcDateTime& dt) {
  char time[9];
  snprintf_P(time,
             countof(time),
             PSTR("%02u:%02u:%02u"),
             dt.Hour(),
             dt.Minute(),
             dt.Second());    
  return time;
}