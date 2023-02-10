// TFTLCD.h and TouchScreen.h are from adafruit.com where you can also purchase a really nice 2.8" TFT with touchscreen :)
// 2011 Jeremy Saglimbeni - thecustomgeek.com
// 2023: Modified by Albert Cervera

#include <EEPROM.h>
#include <Elegoo_GFX.h>     // Core graphics library
#include <Elegoo_TFTLCD.h>  // Hardware-specific library
#include <TouchScreen.h>

// #define MEGA // Added by Albert C.
/* For the 8 data pins:

 Duemilanove/Diecimila/UNO/etc ('168 and '328 chips) microcontoller:
 D0 connects to digital 8
 D1 connects to digital 9
 D2 connects to digital 2
 D3 connects to digital 3
 D4 connects to digital 4
 D5 connects to digital 5
 D6 connects to digital 6
 D7 connects to digital 7
 For Mega's use pins 22 thru 29 (on the double header at the end)
 */

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin


// My own good hand-calibrated values
#define TS_MINX 96   // 96 gives p.x: 240 at (240,y) [good]
#define TS_MAXX 935  // 928 gives p.x: -1 at (0,y), 935 gives seems more accurate and consistant [good]

#define TS_MINY 275  // 212 gives p.y: 288 at (x,320) // 275 gives p.y: 320 at (0,320) [good]
#define TS_MAXY 920  // gives p.y: 0 at (x,0) [good]


// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4


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

int i = 0;
int page = 0;
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


void setup(void) {
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
  Serial.begin(9600);
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
  // tft.drawString(84, 210, "Albert Cervera  -  2023", WHITE);
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
  // tft.drawString(1, 1, "Your status bar message here.    AOS 1.3 Beta", WHITE);
  // tft.setCursor(1, 1);
  // tft.setTextColor(WHITE);
  // tft.setTextSize(1);
  // tft.println("Mission Control Center        Delta-1 Mission");
  tft.drawRect(297, 1, 20, 8, WHITE);  //battery body
  tft.fillRect(317, 3, 2, 4, WHITE);   // battery tip
  tft.fillRect(298, 2, 18, 6, BLACK);  // clear the center of the battery
  drawBatt();
  ant();                                 // draw the bas "antenna" line without the "signal waves"
  signal();                              // draw the "signal waves" around the "antenna"
  homescr();                             // draw the homescreen
  tft.drawRect(0, 200, 245, 40, WHITE);  // message box
} // end void setup

#define MINPRESSURE 10
#define MAXPRESSURE 1000
void loop() {

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
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {

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


    Serial.print("p.y:");  // this code will help you get the y and x numbers for the touchscreen
    Serial.print(p.y);
    Serial.print("   p.x:");
    Serial.println(p.x);

    // area 1 [calibrated]
    if (p.y > 170 && p.y < 320 && p.x > 173 && p.x < 216 && enableArea) {  // if this area is pressed
      if (page == 5) {                                                     // and if page 5 is drawn on the screen
        m5b1action();                                                      // do whatever this button is
        // tft.drawString(12, 213, "Menu 5 B1", RED, 2); // display the command in the "message box"
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 5 B1");
        yled(550);       // flash the LED yellow for a bit - change the 550 value to change LED time on
        clearMessage();  // after the LED goes out, clear the message
      }
      if (page == 4) {
        m4b1action();
        // tft.drawString(12, 213, "Menu 4 B1", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Temp & Hum");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b1action();
        // tft.drawString(12, 213, "Menu 3 B1", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B1");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b1action();
        // tft.drawString(12, 213, "Menu 2 B1", RED, 2);
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
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Telemetry");
        yled(550);
        clearMessage();
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
        // tft.drawString(12, 213, "Menu 5 B2", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 5 B2");
        yled(550);
        clearMessage();
      }
      if (page == 4) {
        m4b2action();

        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Heat Index");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b2action();
        // tft.drawString(12, 213, "Menu 3 B2", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B2");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b2action();
        // tft.drawString(12, 213, "Menu 2 B2", RED, 2);
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
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("OBC");
        yled(550);
        clearMessage();
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
      //   // tft.drawString(12, 213, "Menu 5 B3", RED, 2);
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B3");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b3action();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Air Press");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b3action();
        // tft.drawString(12, 213, "Menu 3 B3", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B3");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b3action();
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
        // tft.drawString(12, 213, "Menu 1 B3", RED, 2);
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
      //   // tft.drawString(12, 213, "Menu 5 B4", RED, 2);
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B4");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b4action();
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Local Alt");
        yled(550);
        clearMessage();
      }
      if (page == 3) {
        m3b4action();
        // tft.drawString(12, 213, "Menu 3 B4", RED, 2);
        tft.setCursor(12, 213);
        tft.setTextColor(RED);
        tft.setTextSize(2);
        tft.println("Menu 3 B4");
        yled(550);
        clearMessage();
      }
      if (page == 2) {
        m2b4action();
        // tft.drawString(12, 213, "Menu 2 B4", RED, 2);
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
        // tft.drawString(12, 213, "Menu 1 B4", RED, 2);
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
      //   // tft.drawString(12, 213, "Menu 5 B5", RED, 2);
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B5");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b5action();
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
      //   // tft.drawString(12, 213, "Menu 3 B5", RED, 2);
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 3 B5");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 2) {
        m2b5action();
        // tft.drawString(12, 213, "Menu 2 B5", RED, 2);
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
      // No button or action there
      // if (page == 1) {
      //   m1b5action();
      //   // tft.drawString(12, 213, "Menu 1 B5", RED, 2);
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
      //   // tft.drawString(12, 213, "Menu 5 B6", RED, 2);
      //   tft.setCursor(12, 213);
      //   tft.setTextColor(RED);
      //   tft.setTextSize(2);
      //   tft.println("Menu 5 B6");
      //   yled(550);
      //   clearMessage();
      // }
      if (page == 4) {
        m4b6action();

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
      //   // tft.drawString(12, 213, "Menu 3 B6", RED, 2);
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
      //   // tft.drawString(12, 213, "Menu 2 B6", RED, 2);
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
      //   // tft.drawString(12, 213, "Menu 1 B6", RED, 2);
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
  if (currenttime - prevbatt > battcheck) {
    drawBatt();
    prevbatt = currenttime;
  }
} // end void loop ()

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
  boxes(6);
  enableArea = true;

  clearMessageStatusBar();
  tft.setCursor(1, 1);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Mission Control Center        Delta-1 Mission");

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
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("CubeSat Mode: " + String(1));

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Internal Temp: " + String(21.44) + "\367" + "C");
}
void m1b2action() {
  // signalAct();
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
  tft.println("Hibernation");
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
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(17.05) + " m");
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
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temperature: " + String(19.30) + "\367" + "C");

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Humidity: " + String(31.40) + "%");
}
void m4b2action() {
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Heat Index: " + String(18.11) + "\367" + "C");
}
void m4b3action() {
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Barometer: " + String(778.78) + " hPa");
}
void m4b4action() {
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 37);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(17.05) + " m");
}
void m4b5action() {
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 27);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Inertial Measurement U: ");

  tft.setCursor(22, 57);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("X: " + String(-0.33) + +"\367");

  tft.setCursor(22, 77);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Y: " + String(-0.14) + +"\367");

  tft.setCursor(22, 97);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Z: " + String(123.74) + +"\367");
}
void m4b6action() {
  clearCenter();
  enableArea = false;

  tft.setCursor(22, 27);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Temperature: " + String(19.30) + "\367" + "C");

  tft.setCursor(22, 47);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Humidity: " + String(31.40) + "%");

  tft.setCursor(22, 67);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Heat Index: " + String(18.11) + "\367" + "C");

  tft.setCursor(22, 87);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Barometer: " + String(778.78) + " hPa");

  tft.setCursor(22, 107);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Local Altitude: " + String(17.05) + " m");

  tft.setCursor(22, 127);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("X: " + String(-0.33) + +"\367");

  tft.setCursor(22, 147);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Y: " + String(-0.14) + +"\367");

  tft.setCursor(22, 167);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.println("Z: " + String(123.74) + +"\367");
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
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Rebooting OBC ...");
  yled(550);
  clearMessage();
}

void m12b2action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Retrieving data ...");
  yled(550);
  clearMessage();
}

void m12b3action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Deleting data ...");
  yled(550);
  clearMessage();
}

// m13b2action();

void m13b1action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Activating ESM ...");
  yled(550);
  clearMessage();
}

void m13b2action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Loop RTWI ...");
  yled(550);
  clearMessage();
}

void m13b3action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Activating RTI ...");
  yled(550);
  clearMessage();
}

void m13b4action() {
  tft.setCursor(12, 213);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.println("Activating RTWI ...");
  yled(550);
  clearMessage();
}

void m13b5action() {
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
void ant() {
  tft.fillRect((antpos + 5), 4, 1, 6, WHITE);  // draws the "antenna" for the signal indicator
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
void signal() {  // draws a whit 'signal indicator'
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