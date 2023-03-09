#include <TimeLib.h>
 
void setup() {
  tmElements_t time1 = {16, 25, 0, 0, 5,  8, CalendarYrToTm(2011)}, // 0:00:00 1st Jan 2015
               time2 = {16, 24, 0, 0, 5, 8, CalendarYrToTm(2023)}; // 0:00:00 8th Oct 2015 (today) // august 3rd 23
  
  uint32_t difference = (uint32_t)(makeTime(time2) - makeTime(time1));
  
  struct elapsedTime_t {
    uint8_t Seconds, Minutes, Hours;
    uint16_t Days;
  } elapsedTime;
  
  elapsedTime.Seconds = difference % 60;
  difference /= 60; // now it is minutes
  
  elapsedTime.Minutes = difference % 60;
  difference /= 60; // now it is hours
  
  elapsedTime.Hours = difference % 24;
  difference /= 24; // now it is days
  
  elapsedTime.Days = difference;

  
  
  char buf[64];
  sprintf(buf, "Elapsed time = %d days, %d hours, %d minutes, %d seconds.\r\n",
    elapsedTime.Days, elapsedTime.Hours, elapsedTime.Minutes, elapsedTime.Seconds);
 
  Serial.begin(9600);
  Serial.print(buf);
}
 
void loop() {}