
#include <Elegoo_GFX.h>     // Core graphics library
#include <TouchScreen.h>

#include <MCUFRIEND_kbv.h>// Touch screen pressure threshold
#define MINPRESSURE 40
#define MAXPRESSURE 1000
// Touch screen calibration
const int16_t XP = 8, XM = A2, YP = A3, YM = 9; //240x320 ID=0x9341
const int16_t TS_LEFT = 122, TS_RT = 929, TS_TOP = 77, TS_BOT = 884;

const TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);