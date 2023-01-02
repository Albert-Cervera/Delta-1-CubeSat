//.........................Jehnakandy..................
//.....................Arduino Keypad 4x4..............
//.................Download Keypad.h library...........

#include <Keypad.h>

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROWS] = {10, 9, 8, 7};  // 9 to 6 in example, 10 to 7 digital in real
byte colPins[COLS] = {A0, A1, A2, A3};  // 5 to 2 in example, A0 to A3 in real

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

void setup(){
  Serial.begin(9600);
}
  
void loop(){
  char customKey = customKeypad.getKey();
  
  if (customKey){
    Serial.println(customKey);
  }
}

//...................Thankyou....................
//..............coded by : Jehankandy............