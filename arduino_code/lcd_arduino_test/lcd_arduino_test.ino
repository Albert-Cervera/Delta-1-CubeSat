#include <LiquidCrystal.h>

#define RS 12
#define EN 11
#define D4 5
#define D5 4
#define D6 3
#define D7 2

// LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("NASA - NanoSat 1");
  delay(2000);
  lcd.clear();

}

void loop() {
// set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
lcd.setCursor(0, 1);
// print the number of seconds since reset:
lcd.print(millis()/1000);
}