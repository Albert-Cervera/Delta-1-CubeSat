#include <virtuabotixRTC.h>
// Creation of the Real Time Clock Object
// virtuabotixRTC myRTC(6, 7, 8); // UNO
virtuabotixRTC myRTC(25, 27, 29);  // MEGA

void setup() {
  Serial.begin(9600);
  // Set the current date, and time in the following format:
  // seconds, minutes, hours, day of the week, day of the month, month, year (monday is day 1, sunday is 7)  
  // myRTC.setDS1302Time(00, 26, 17, 5, 30, 11, 2022);
  // myRTC.setDS1302Time(00, 30, 16, 1, 20, 7, 2023); 
  //                  SS, MM, HH, DW, DD, MM, YYYY

  // Upload the code when reference clock's second 56 is about to hit
}

void loop() {
  // This allows for the update of variables for time or accessing the individual elements.
  myRTC.updateTime();

  // Start printing elements as individuals
  Serial.print("Current Date / Time: ");
  Serial.print(myRTC.dayofmonth);
  Serial.print("/");
  Serial.print(myRTC.month);
  Serial.print("/");
  Serial.print(myRTC.year);
  Serial.print("  ");
  Serial.print(myRTC.hours);
  Serial.print(":");
  Serial.print(myRTC.minutes);
  Serial.print(":");
  Serial.println(myRTC.seconds);

  // Delay so the program doesn't print non-stop
  delay(1000);
}