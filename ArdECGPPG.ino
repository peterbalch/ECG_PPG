//-----------------------------------------------------------------------------
// Copyright 2023 Peter Balch
//   displays an ECG and PPG
//   computes QT interval
//-----------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include "SimpleILI9341.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

//#define bFakeECGoutput
#define bHasPPG
#define bHasPAT
#define bHasQT
//#define bDebug
//#define bWindows
//#define bFakeInput

#ifdef bHasPAT
#define bHasPPG
#endif

// get register bit - faster: doesn't turn it into 0/1
#ifndef getBit
#define getBit(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#endif
#define sqr(x) ((x)*(x))

//-----------------------------------------------------------------------------
// Global Constants and Typedefs
//-----------------------------------------------------------------------------

const int TFT_WIDTH = 320;
const int TFT_HEIGHT = 240;

// pins
const int ECG_IN = A2;
#ifdef bHasPPG
const int PPG_IN = A4;
int16_t ppgmin = 1024;
static uint32_t ppgEnergy = 0;
#endif

#ifdef bFakeECGoutput
const int FAKE_OUT = 3;
#endif

const int BUTTON_IN = A5;
const int LO_P_IN = A0;
const int LO_N_IN = A1;

// Display pins
const int TFT_CS    = 10;
const int TFT_CD    = 8;
const int TFT_RST   = 9;

const int sps = 200; // Samples per Sec
const int SamplePeriod = 1000 / sps; // mSec
const int PoincareScale = 10;
const int PoincareLeft = 40;
const int PoincareBottom = TFT_HEIGHT - 12;
const int nDiff = sps / 65; /* differentiate = aa[i]-aa[i-nDiff]*/

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

uint16_t DisplayRepeat = TFT_WIDTH; // helps keep the R peak centred
uint8_t ignoreBeats = 0; // blank the screen after an error
int16_t lastNegDiff1, lastNegDiff2; // how long ago was the last time the differential was negative
uint16_t CurBPM = 60;
#ifdef bHasPPG
  uint8_t PPGreceived = 0; // count how many PPG signals (value < 1000) have been received to check that the PPG is plugged in
  const uint8_t PPGreceivedMin = 100; // must have received >= 100 valid values
#endif
#ifdef bHasQT
  uint16_t CurQT = 360;
#endif
#ifdef bHasPAT
  uint16_t CurPAT = 0;
#endif

enum TMode { mdLargeECG, mdSmallECG, mdPoincare, mdBattery } mode = mdLargeECG;

#ifdef bFakeInput
//-----------------------------------------------------------------------------
const uint8_t data[] PROGMEM = {
  215, 232, 219, 187, 126, 94, 78, 85, 100, 105, 103, 101, 100, 101, 100, 100, 99, 99, 100, 102, 103,
  103, 103, 102, 102, 103, 105, 106, 106, 106, 107, 109, 112, 113, 115, 117, 121, 122, 123, 123, 126, 129,
  134, 136, 137, 137, 138, 140, 143, 142, 136, 132, 129, 127, 120, 114, 107, 107, 107, 105, 100, 98, 99,
  98, 94, 93, 93, 95, 97, 97, 97, 97, 96, 95, 96, 97, 98, 97, 96, 96, 95, 95, 94, 95,
  97, 98, 97, 96, 97, 98, 100, 101, 101, 101, 102, 102, 101, 99, 98, 98, 98, 98, 96, 95, 96,
  96, 96, 96, 96, 97, 99, 100, 101, 102, 103, 104, 104, 104, 102, 101, 102, 103, 107, 108, 107, 105,
  102, 100, 96, 93, 92, 93, 93, 93, 93, 93, 92, 90, 89, 90, 91, 90, 87, 87, 87, 85, 82,
  89, 120, 153, 202, 223, 219, 190, 125, 89, 71, 80, 96, 97, 91, 88, 90, 91, 92, 92, 94, 95,
  94, 94, 96, 98, 100, 101, 103, 104, 105, 104, 104, 105, 109, 111, 112, 112, 110, 111, 115, 120, 125,
  127, 130, 133, 137, 140, 141, 142, 143, 143, 139, 137, 136, 136, 132, 126, 117, 114, 113, 112, 106, 102,
  99, 99, 98, 96, 95, 94, 96, 96, 96, 96, 96, 95, 95, 94, 93, 91, 89, 89, 92, 94, 95,
  95, 94, 95, 97, 96, 95, 94, 93, 93, 93, 92, 92, 92, 92, 92, 91, 91, 89, 88, 88, 88,
  88, 87, 86, 85, 85, 85, 85, 86, 87, 86, 84, 83, 86, 88, 89, 87, 84, 84, 85, 85, 87,
  91, 100, 106, 110, 111, 112, 113, 117, 120, 123, 124, 125, 125, 124, 123, 121, 119, 117, 116, 114, 114,
  115, 115, 112, 110, 107, 107, 108, 109, 110, 109, 108, 108, 107, 104, 100, 100, 116, 139, 188, 223, 255,
  249, 196, 149, 99, 91, 101, 109, 111, 111, 114, 115, 113, 111, 111, 113, 115, 114, 113, 112, 112, 112,
  112, 113, 116, 117, 117, 116, 117, 117, 118, 119, 122, 124, 126, 127, 129, 132, 138, 142, 145, 146, 146,
  148, 150, 151, 149, 147, 145, 143, 138, 132, 123, 119, 117, 114, 109, 105, 102, 101, 98, 96, 95, 96,
  95, 93, 90, 90, 93, 93, 90, 88, 88, 91, 95, 95, 93, 91, 90, 91, 92, 93, 94, 93, 91,
  91, 92, 93, 93, 92, 92, 92, 91, 90, 88, 88, 89, 90, 90, 89, 90, 92, 94, 95, 93, 90,
  88, 89, 92, 93, 91, 90, 89, 89, 89, 88, 88, 88, 89, 89, 91, 93, 94, 93, 90, 90, 91,
  92, 92, 92, 91, 92, 93, 94, 95, 95, 93, 92, 91, 93, 94, 94, 94, 95, 97, 97, 97, 97,
  98, 99, 101, 103, 105, 107, 107, 107, 106, 106, 106, 105, 102, 99, 94, 93, 93, 94, 96, 97, 95,
  94, 92, 91, 91, 91, 91, 91, 93, 94, 93, 91, 89, 92, 111, 135, 187, 220, 242, 226, 164, 119,
  81, 78, 90, 96, 99, 101, 104, 105, 104, 103, 103, 104, 103, 104, 105, 106, 107, 106, 107, 106, 106,
  106, 108, 110, 112, 113, 115, 118, 120, 121, 121, 123, 127, 130, 133, 135, 140, 143, 145, 147, 149, 150,
  153, 153, 149, 144, 137, 135, 135, 133, 124, 117, 111, 111, 111, 108, 103, 102, 102, 102, 101, 99, 98,
  98, 99, 98, 97, 97, 98, 100, 102, 100, 97, 96, 95, 96, 98, 99, 98, 97, 97, 100, 104, 105,
  102, 99, 99, 101, 103, 103, 102, 101, 99, 98, 96, 96, 98, 100, 101, 99, 94, 93, 94, 95, 94,
  93, 94, 95, 97, 98, 98, 97, 95, 93, 92, 94, 97, 97, 95, 93, 91, 90, 90, 91, 92, 94,
  95, 95, 94, 93, 94, 96, 100, 101, 104, 106, 110, 112, 113, 114, 117, 118, 120, 120, 120, 121, 124,
  126, 128, 130, 131, 131, 130, 128, 127, 127, 127, 125, 123, 123, 121, 119, 116, 115, 116, 115, 112, 110,
  111, 113, 116, 116, 113, 111, 111, 111, 109, 106, 107, 117, 150, 181, 225, 239, 222, 186, 121, 93, 88,
  98, 112, 114, 114, 115, 116, 115, 112, 111, 112, 113, 114, 113, 113, 113, 114, 114, 114, 114, 116, 118,
  121, 122, 120, 118, 118, 119, 122, 124, 128, 132, 135, 136, 137, 139, 144, 146, 146, 145, 145, 147, 147,
  146, 141, 137, 130, 126, 118, 114, 109, 106, 103, 100, 97, 96, 95, 92, 88, 86, 86, 87, 88, 87,
  87, 87, 86, 85, 84, 85, 86, 87, 87, 86, 86, 87, 89, 90, 88, 87, 87, 90, 93, 94, 91,
  89, 90, 91, 91, 90, 90, 91, 92, 91, 88, 87, 87, 87, 88, 89, 89, 88, 87, 86, 86, 85,
  84, 85, 88, 90, 90, 88, 86, 87, 89, 90, 89, 88, 86, 86, 88, 90, 90, 87, 84, 84, 87,
  90, 91, 90, 89, 89, 90, 90, 90, 89, 88, 88, 90, 92, 96, 97, 97, 97, 98, 100, 102, 103,
  103, 102, 100, 99, 97, 96, 94, 93, 92, 92, 90, 89, 87, 87, 87, 88, 88, 89, 91, 91, 91,
  89, 88, 87, 85, 85, 95, 114, 160, 195, 229, 226, 181, 138, 87, 75, 81, 89, 93, 93, 95, 97,
  98, 98, 99, 99, 99, 99, 101, 102, 102, 102, 102, 104, 107, 108, 108, 108, 109, 110, 112, 112, 113,
  114, 117, 119, 122, 124, 127, 129, 134, 137, 141, 143, 145, 145, 145, 145, 145, 144, 142, 139, 133, 129,
  124, 120, 113, 109, 107, 107, 106, 105, 102, 101, 102, 103, 103, 101, 98, 96, 96, 97, 99, 100, 102,
  102, 101, 100, 100, 101, 104, 105, 106, 106, 106, 106, 104, 103, 103, 103, 103, 102, 101, 101, 103, 104,
  105, 105, 104, 103, 102, 101, 100, 100, 101, 102, 102, 101, 99, 99, 101, 101, 97, 95, 95, 97, 99,
  99, 98, 98, 98, 98, 98, 97, 97, 97, 98, 99, 99, 98, 96, 96, 95, 95, 94, 94, 96, 97,
  96, 93, 90, 91, 94, 96, 95, 94, 94, 96, 97, 97, 98, 100, 103, 104, 104, 105, 111, 115, 121,
  123, 126, 127, 128, 127, 124, 123, 124, 125, 126, 126, 125, 125, 124, 123, 122, 122, 122, 122, 120, 119,
  116, 114, 116, 125, 158, 191, 243, 255, 253, 217, 147, 114, 102, 110, 123, 125, 124, 125, 126, 125, 123,
  121, 121, 120, 120, 121, 122, 123, 123, 123, 124, 126, 129, 129, 129, 129, 130, 132, 132, 132, 132, 133,
  137, 140, 143, 146, 151, 154, 157, 159, 160, 162, 163, 164, 165, 164, 160, 155, 147, 143, 136, 130, 122,
  119, 116, 113, 106, 102, 99, 99, 99, 98, 96, 96, 96, 95, 93, 92, 92, 93, 94, 93, 93, 93,
  92, 91, 89, 89, 90, 91, 91, 92, 93, 94, 93, 91, 89, 89, 90, 91, 90, 88, 86, 85, 86,
  88, 89, 88, 88, 88, 88, 87, 84, 82, 81, 81, 82, 83, 84, 83, 81, 79, 78, 80, 81, 82,
  82, 82, 82, 82, 82, 83, 84, 84, 82, 82, 83, 85, 87, 87, 86, 86, 85, 85, 88, 90, 90,
  89, 89, 90, 91, 91, 92, 95, 100, 102, 103, 101, 96, 93, 89, 88, 86, 85, 83, 83, 85, 87,
  87, 86, 84, 82, 81, 82, 84, 85, 83, 82, 81, 81, 83, 92, 125, 159, 206, 218, 196, 160, 102,
  77, 68, 76, 91, 95, 95, 96, 96, 94, 91, 91, 95, 97, 98, 97, 97, 98, 99, 99, 99, 99,
  100, 102, 106, 108, 109, 110, 112, 114, 115, 114, 115, 117, 123, 127, 131, 133, 136, 138, 140, 140, 140,
  141, 143, 144, 142, 139, 134, 130, 124, 120, 117, 115, 112, 108, 103, 101, 100, 101, 100, 100, 98, 97,
  97, 97, 98, 99, 100, 100, 100, 100, 101, 100, 99, 99, 100, 101, 103, 104, 103, 101, 100, 100, 103,
  105, 105, 104, 102, 101, 102, 103, 103, 103, 102, 101, 100, 99, 99, 99, 98, 98, 99, 100, 100, 99,
  99, 101, 103, 103, 101, 100, 100, 102, 102, 100, 96, 94, 95, 97, 99, 100, 101, 101, 100, 99, 98,
  98, 98, 98, 98, 97, 100, 103, 107, 106, 104, 102, 105, 109, 112, 113, 111, 111, 109, 106, 100, 97,
  96, 97, 97, 97, 96, 96, 96, 95, 93, 92, 94, 95, 95, 95, 94, 92, 88, 87, 97, 114, 152,
  179, 206, 205, 169, 132, 86, 75, 84, 94, 98, 96, 96, 99, 100, 97, 92, 92, 96, 99, 99, 99,
  99, 101, 102, 102, 107, 112, 122, 126, 131, 134, 137, 138, 140, 143, 149, 151, 151, 151, 155, 160, 166,
  169, 171, 172, 173, 173, 172, 172, 173, 173, 170, 166, 159, 155, 150, 146, 140, 138, 135, 134, 131, 130,
  129, 129, 126, 123, 121, 122, 125, 125, 122, 120, 120, 121, 121, 121, 120, 120, 120, 119, 118, 118, 118,
  117, 116, 116, 116, 115, 114, 112, 111, 112, 113, 113, 110, 108, 106, 106, 107, 107, 106, 105, 104, 104,
  105, 105, 104, 104, 102, 100, 98, 99, 102, 103, 100, 96, 93, 95, 99, 99, 97, 95, 92, 91, 91,
  91, 93, 94, 94, 94, 94, 96, 99, 100, 97, 95, 95, 97, 104, 107, 106, 104, 100, 99, 98, 97,
  93, 90, 87, 86, 86, 87, 88, 88, 86, 83, 82, 82, 85, 86, 85, 83, 81, 80, 79, 82, 99,
  122, 169, 196, 203, 180, 124, 90, 67, 69, 80, 85, 88, 88, 89, 89, 87, 87, 90, 93, 96, 95,
  93, 92, 95, 98, 100, 99, 96, 95, 96, 98, 99, 99, 101, 104, 107, 108, 110, 112, 115, 116, 119,
  122, 130, 134, 135, 134, 135, 137, 137, 135, 133, 132, 131, 127, 118, 113, 108, 106, 102, 100, 96, 93,
  90, 90, 91, 91, 89, 87, 87, 88, 92, 92, 90, 88, 90, 92, 95, 94, 89, 87, 88, 91, 95,
  95, 94, 93, 92, 92, 92, 93, 95, 97, 97, 97, 97, 98, 97, 96, 95, 93, 91, 90, 90, 91,
  92, 93, 94, 95, 95, 94, 92, 92, 92, 93, 94, 94, 93, 93, 94, 94, 94, 93, 92, 93, 93,
  93, 92, 92, 93, 95, 97, 98, 99, 100, 100, 102, 106, 109, 111, 111, 107, 103, 99, 99, 99, 97,
  93, 92, 93, 94, 93, 92, 92, 93, 93, 93, 92, 92, 91, 89, 87, 87, 88, 91, 113, 143, 199,
  226, 223, 191, 125, 90, 71, 77, 92, 98, 98, 96, 96, 98, 98, 96, 96, 97, 100, 100, 100, 101,
  104, 106, 107, 106, 106, 107, 108, 108, 109, 109, 111, 114, 118, 120, 122, 123, 126, 128, 131, 133, 138,
  140, 144, 146, 149, 150, 149, 147, 144, 143, 142, 139, 130, 124, 117, 115, 112, 111, 109, 107, 102, 99,
  96, 96, 100, 101, 99, 97, 97, 98, 100, 99, 98, 97, 98, 98, 99, 100, 103, 103, 102, 101, 101,
  102, 102, 102, 102, 102, 102, 102, 103, 104, 103, 102, 103, 107, 114, 119, 124, 125, 126, 125, 123, 124,
  124, 124, 123, 123, 124, 126, 129, 130, 128, 127, 127, 127, 127, 127, 126, 126, 125, 124, 122, 122, 124,
  126, 126, 126, 126, 127, 129, 130, 132, 134, 136, 135, 133, 132, 129, 126, 120, 117, 115, 114, 112, 110,
  109, 108, 107, 106, 105, 105, 105, 106, 107, 106, 101, 97, 94, 98, 120, 148, 204
};

uint8_t getdata(int i) {
  return pgm_read_byte_near(data + i);
}

uint16_t XanalogRead(int pin) {
#ifdef bHasPPG
  if (pin == PPG_IN) {
    //    return (millis()/5) % 100;

    static int i = 0;
    int j, k;
    static float a = 128;
    static float b = 128;
    static float c = 128;
    i = (i + 1) % sizeof(data);
    //return 128+100*sin(i);
    k = (i + sizeof(data) - 35) % sizeof(data);
    j = (k + sizeof(data) - 35) % sizeof(data);

    if (getdata(k) > 200)
      a = 240; else if (getdata(j) > 200)
      a = 200; else
      a = a + (128 - a) / 20;
    b = b + (a - b) / 8;
    c = c + (b - c) / 8;
    return c;
  } else
#endif
  {
    static int i = 0;
    i = (i + 1) % sizeof(data);
    return min(getdata(i) * 4, 1023);
  }
}
#endif

//---------------------------------------------------------------------------
// DrawStringF
//
//---------------------------------------------------------------------------
void DrawStringF(word s, byte *Font, uint16_t color) {
  while (true) {
    char c = pgm_read_byte_near(s);
    s++;
    if (c == 0)
      return;
    DrawChar(c, Font, color);
  }
}

//---------------------------------------------------------------------------
// TimeConst
//   calc Alpha for a time constant of a decay
//---------------------------------------------------------------------------
float TimeConst(float n) {
  return exp(-1 / n);
}

//---------------------------------------------------------------------------
// MedianOfThree
//---------------------------------------------------------------------------
int16_t MedianOfThree(int16_t c, int16_t d, int16_t e) {
  int16_t tmp;
  if (d < c) {
    tmp = c;
    c = d;
    d = tmp;
  };
  if (e < c) {
    e = c;
  };
  if (d < e) return d; else return e;
}

//---------------------------------------------------------------------------
// MedianOfFive
//---------------------------------------------------------------------------
int16_t MedianOfFive(int16_t a, int16_t b, int16_t c, int16_t d, int16_t e) {
  int16_t tmp;
  if (b < a) {
    tmp = a;
    a = b;
    b = tmp;
  }
  if (d < c) {
    tmp = c;
    c = d;
    d = tmp;
  }
  if (c < a) {
    tmp = b;
    b = d;
    d = tmp;
    c = a;
  }
  if (b < e) {
    tmp = e;
    e = b;
    b = tmp;
  }
  if (e < c) {
    tmp = b;
    d = tmp;
    e = c;
  }
  if (d < e) return d; else return e;
}

//-----------------------------------------------------------------------------
// DrawGridSmall
//-----------------------------------------------------------------------------
void DrawGridSmall(void)
{
  int16_t  i;
  ClearDisplay(TFT_BLACK);
  for (i = 0; i < TFT_WIDTH; i++)
    if (i * 4 * 5 % (1000 / SamplePeriod) == 0)
      DrawVLine(i, 0, TFT_HEIGHT, TFT_RED);
}

//-----------------------------------------------------------------------------
// DrawPoincareLine
//-----------------------------------------------------------------------------
void DrawPoincareLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  DrawLine(
    constrain(PoincareLeft + x1 / PoincareScale, 0, TFT_WIDTH - 1),
    constrain(PoincareBottom - y1 / PoincareScale, 0, TFT_HEIGHT - 1),
    constrain(PoincareLeft + x2 / PoincareScale, 0, TFT_WIDTH - 1),
    constrain(PoincareBottom - y2 / PoincareScale, 0, TFT_HEIGHT - 1),
    color);
}

//-----------------------------------------------------------------------------
// DrawPoincareFrame
//-----------------------------------------------------------------------------
void DrawPoincareFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
  DrawPoincareLine(x1, y1, x2, y1, color);
  DrawPoincareLine(x2, y1, x2, y2, color);
  DrawPoincareLine(x2, y2, x1, y2, color);
  DrawPoincareLine(x1, y2, x1, y1, color);
}

//-----------------------------------------------------------------------------
// DrawGridPoincare
//-----------------------------------------------------------------------------
void DrawGridPoincare(void)
{
#define TFT_DARKDARKGRAY    RGB(64,64,64)
  const uint8_t f[4] = { 30, 60, 100, 160 };
  int16_t i, b;

  ClearDisplay(TFT_BLACK);

  for (i = 0; i < sizeof(f); i++) {
    b = 60000 / f[i] / PoincareScale;
    DrawVLine(PoincareLeft + b, 0, PoincareBottom, TFT_DARKDARKGRAY);
    DrawHLine(PoincareLeft, PoincareBottom - b, TFT_WIDTH, TFT_DARKDARKGRAY);
    ILI9341SetCursor(PoincareLeft + b - 8, TFT_HEIGHT);
    DrawInt(f[i], MediumFont, TFT_WHITE);
    ILI9341SetCursor(PoincareLeft - 20, PoincareBottom - b + 4);
    DrawInt(f[i], MediumFont, TFT_WHITE);
  }
  DrawLine(PoincareLeft, PoincareBottom, PoincareLeft + PoincareBottom, 0, TFT_DARKDARKGRAY);

  DrawPoincareLine(500, 500, 1115, 916, TFT_DARKGREEN); // normal
  DrawPoincareLine(1115, 916, 1115, 1115, TFT_DARKGREEN);
  DrawPoincareLine(1115, 1115, 916, 1115, TFT_DARKGREEN);
  DrawPoincareLine(916, 1115, 500, 500, TFT_DARKGREEN);
  DrawPoincareFrame(500, 1200, 700, 2500, TFT_MAROON); // N-V-N
  DrawPoincareFrame(1200, 500, 2500, 700, TFT_MAROON);
  DrawPoincareFrame(500, 500, 700, 1200, RGB(48, 48, 0)); // premature ventricular
  DrawPoincareFrame(500, 500, 1200, 700, RGB(48, 48, 0));
  DrawPoincareLine(320, 320, 320, 650, RGB(0, 0, 64)); // atrial fibrilation
  DrawPoincareLine(320, 650, 800, 1700, RGB(0, 0, 64));
  DrawPoincareLine(800, 1700, 1700, 1700, RGB(0, 0, 64));
  DrawPoincareLine(1700, 1700, 1700, 800, RGB(0, 0, 64));
  DrawPoincareLine(650, 320, 1700, 800, RGB(0, 0, 64));
  DrawPoincareLine(650, 320, 320, 320, RGB(0, 0, 64));
  DrawPoincareFrame(1700, 900, 2500, 1400, RGB(0, 32, 32)); // missed beat
  DrawPoincareFrame(900, 1700, 1400, 2500, RGB(0, 32, 32));

  DrawVLine(PoincareLeft, 0, PoincareBottom, TFT_WHITE);
  DrawHLine(PoincareLeft, PoincareBottom, TFT_WIDTH, TFT_WHITE);
}

//-----------------------------------------------------------------------------
// DrawGridVLine
//   for Large display, draw a part of the grid
//-----------------------------------------------------------------------------
void DrawGridVLine(int16_t x, int16_t y1, int16_t y2)
{
  int16_t y;

  if (y1 > y2)
    DrawGridVLine(x, y2, y1); else
  {
    if (x * 5 % (1000 / SamplePeriod) == 0)
      DrawVLine(x, y1, y2 - y1 + 1, TFT_RED); else if (x * 25 % (1000 / SamplePeriod) == 0)
      DrawVLine(x, y1, y2 - y1 + 1, TFT_MAROON); else
    {
      DrawVLine(x, y1, y2 - y1 + 1, TFT_BLACK);
      for (y = y1; y <= y2; y++)
        if (y % 40 == 0)
            DrawPixel(x, y, TFT_RED); else if (y % 8 == 0)
          DrawPixel(x, y, TFT_MAROON);
    }
  }
}

//-----------------------------------------------------------------------------
// DrawPoincare
//    Poincare plot
//    t is in range 400..2000 mSec
//-----------------------------------------------------------------------------
void DrawPoincare(uint16_t t)
{
  const int nPeriods = 500;
  static uint8_t Periods[nPeriods] = {0};
  static int16_t i = 0;
  int16_t prev;

  if (t <= 0) {
    memset(Periods, 0, sizeof(Periods));
    return;
  }

  t = t / PoincareScale;

  prev = Periods[i];
  i = (i + 1) % nPeriods;
  if (PoincareLeft + Periods[i] > 0)
    DrawPixel(PoincareLeft + Periods[i], PoincareBottom - (Periods[(i + 1) % nPeriods]), TFT_BLACK);
  Periods[i] = t;
  if (prev > 0)
    DrawPixel(PoincareLeft + prev, PoincareBottom - t, TFT_WHITE);
}

//-----------------------------------------------------------------------------
// DrawGridLarge
//-----------------------------------------------------------------------------
void DrawGridLarge(void)
{
  int16_t i;
  ClearDisplay(TFT_BLACK);
  for (i = 0; i < TFT_HEIGHT; i++)
    if (i % 8 == 0)
      DrawHLine(0, i, TFT_WIDTH, TFT_MAROON);
  for (i = 0; i < TFT_WIDTH; i++)
    if (i * 25 % (1000 / SamplePeriod) == 0)
      DrawVLine(i, 0, TFT_HEIGHT, TFT_MAROON);

  for (i = 0; i < TFT_HEIGHT; i++)
    if (i % 40 == 0)
      DrawHLine(0, i, TFT_WIDTH, TFT_RED);
  for (i = 0; i < TFT_WIDTH; i++)
    if (i * 5 % (1000 / SamplePeriod) == 0)
      DrawVLine(i, 0, TFT_HEIGHT, TFT_RED);
}

//-----------------------------------------------------------------------------
// DrawGrid
//-----------------------------------------------------------------------------
void DrawGrid(void)
{
  switch (mode) {
    case mdLargeECG: DrawGridLarge(); break;
    case mdSmallECG: DrawGridSmall(); break;
    case mdPoincare: DrawGridPoincare(); break;
    case mdBattery: ClearDisplay(TFT_BLACK); break;
  }
  DisplayRepeat = TFT_WIDTH;
  DrawPoincare(0);
  ignoreBeats = 2;
}

//-----------------------------------------------------------------------------
// centrePeak
//   attempt to keep a peak in the first third of the screen
//-----------------------------------------------------------------------------
void centrePeak(uint16_t period, uint16_t x)
{
  int16_t b;
  static int16_t dr = 480;
  static int16_t dt = 0;

  b = period;
  while (b < TFT_WIDTH)
    b += period;
  if (b > dr + 10)
    dr += 10; else if (b < dr - 10)
    dr -= 10;
  if (b > dr)
    dr++; else
    dr--;
  dr = constrain(dr, TFT_WIDTH - 1, 2 * TFT_WIDTH);
  DisplayRepeat = dr;
  if (x < 80)
    dt--; else if (x < 160)
    dt++;
  DisplayRepeat += dt;
  DisplayRepeat = constrain(DisplayRepeat, TFT_WIDTH - 1, 2 * TFT_WIDTH);
}

//-----------------------------------------------------------------------------
// DrawBattVolts
//-----------------------------------------------------------------------------
void DrawBattVolts(void)
{
  const int xBatt = 50;
  const int yBatt = 100;
  static int maxBatt = 3300;
  static unsigned long nextTime = 0;
  int v = read3V3();
  const int w = 40;

  maxBatt = max(maxBatt, v);

  if (millis() > nextTime) {
    nextTime = millis() + 1000;
    ILI9341SetCursor(xBatt, yBatt + 15);
    DrawStringF(F("Battery "), LargeFont, TFT_WHITE);
    DrawBox(Cursorx, yBatt, 54, 20, TFT_BLACK);
    DrawInt(maxBatt, LargeFont, TFT_WHITE);
    maxBatt = 0;

    DrawBox(0, 0, 50, 20, TFT_NAVY);
    ILI9341SetCursor(4, 16);
    DrawStringF(F("BPM"), LargeFont, TFT_WHITE);
    #ifdef bHasQT
      //      DrawBox(TFT_WIDTH-105, 0, 105, 20, TFT_PURPLE);
      //      ILI9341SetCursor(TFT_WIDTH-105+2, 16);
      //      DrawStringF(F(QT interval"), LargeFont, TFT_WHITE);
      DrawBox(TFT_WIDTH - w, 0, w, 20, TFT_PURPLE);
      ILI9341SetCursor(TFT_WIDTH - w + 2, 16);
      DrawStringF(F("QT"), LargeFont, TFT_WHITE);
    #endif
    #ifdef bHasPAT
      if (PPGreceived >= PPGreceivedMin) {
        #ifdef bHasQT
          const int y = 20;
        #else
          const int y = 0;
        #endif
        DrawBox(TFT_WIDTH - w, y, w, 20, TFT_CYAN);
        ILI9341SetCursor(TFT_WIDTH - w + 1, y + 16);
        DrawStringF(F("PAT"), LargeFont, TFT_BLACK);
      }  
    #endif
  }
}

//---------------------------------------------------------------------------
// showBPM
//---------------------------------------------------------------------------
void showBPM() {
  DrawBox(0, 0, 30, 20, TFT_NAVY);
  ILI9341SetCursor(4, 16);
  DrawInt(CurBPM, LargeFont, TFT_WHITE);
}

//---------------------------------------------------------------------------
// showQT
//---------------------------------------------------------------------------
#ifdef bHasQT
void showQT() {
  DrawBox(TFT_WIDTH - 30, 0, 30, 20, TFT_PURPLE);
  ILI9341SetCursor(TFT_WIDTH - 30 + 2, 16);
  DrawInt(CurQT, LargeFont, TFT_WHITE);
}
#endif

//---------------------------------------------------------------------------
// showPAT
//---------------------------------------------------------------------------
#ifdef bHasPAT
void showPAT() {
  if (PPGreceived >= PPGreceivedMin) {
    #ifdef bHasQT
      const int y = 20;
    #else
      const int y = 0;
    #endif
    DrawBox(TFT_WIDTH - 30, y, 30, 20, TFT_CYAN);
    ILI9341SetCursor(TFT_WIDTH - 30 + 1, y + 16);
    DrawInt(CurPAT, LargeFont, TFT_BLACK);
    }
}
#endif

//---------------------------------------------------------------------------
// drawBPM
//   filters and draws BPM
//---------------------------------------------------------------------------
void drawBPM(int16_t b)
{
  int16_t c;
  static int16_t prev[4] = {60, 60, 60, 60};
  static float bpm = 60;

  if (ignoreBeats > 0) {
    ignoreBeats--;
  } else {
    c = MedianOfFive(prev[0], prev[1], prev[2], prev[3], b);
    prev[3] = prev[2];
    prev[2] = prev[1];
    prev[1] = prev[0];
    prev[0] = b;
    bpm = (bpm * 5 + c) / 6; // smoothing
    CurBPM = bpm;

    if (mode != mdLargeECG)
      showBPM();
  }
}

//---------------------------------------------------------------------------
// RpeakFound
//---------------------------------------------------------------------------
int16_t RpeakFound(int16_t x, int16_t sinceRpeak) {
  int16_t bpm;

  if (sinceRpeak < sps / 5) /*R_peaks can't be less than 200mS apart*/
    return 0;

  lastNegDiff2 = lastNegDiff1;
  bpm = 60000 / (sinceRpeak * 1000 / sps);

#if defined bDebug && defined bWindows
  DrawX(Form1->PaintBox1->Canvas, x - sinceRpeak, 10, 5);
  CanvasTextOut(Form1->PaintBox1->Canvas, x + 3, 10, "%d", bpm);
#endif

  drawBPM(60000 / (sinceRpeak * 1000 / sps));

  centrePeak(sinceRpeak, x);
  return bpm;
}

//-----------------------------------------------------------------------------
// findbaseline
//   a is the latest sample*/
//   g is baseline*/
//   exponentially "decays" to current sample but tc is longer the further away aa is from the current sample*/
//-----------------------------------------------------------------------------
float findbaseline(int16_t a, int16_t diff) {
  static float g = TFT_HEIGHT / 2; // estimate of baseline
  float f;

  f  =  1 / (sqr(diff) + sqr(a - g) + 1);
  g = g * (1 - f) + a * f;
  return g;
}

//---------------------------------------------------------------------------
// have got a value of the QT interval
//   QTinterval in samples
//   x: where to draw the yellow bar (debugging)
//   bpm: in order to calculate QTc
//---------------------------------------------------------------------------
#ifdef bHasQT
void HaveQT(int16_t QTinterval,
#ifdef bDebug
            int16_t x,
#endif
            int16_t bpm) {
  static int16_t prev[4] = {50, 350, 350, 350};
  int16_t QT;
  static float QTs = 350;

#ifdef bDebug
  if (mode == mdLargeECG) {
    DrawVLine(x, TFT_HEIGHT - 10, 10, TFT_YELLOW);
    DrawVLine(x + QTinterval, TFT_HEIGHT - 10, 10, TFT_YELLOW);
    DrawHLine(x, TFT_HEIGHT - 5, QTinterval, TFT_YELLOW);
  }
#endif

  QTinterval = QTinterval * SamplePeriod;
  QT = MedianOfFive(prev[0], prev[1], prev[2], prev[3], QTinterval);

  // calculate QTc
  if (bpm > 0)
    QT = QT * ((0.00728 - 0.0000164 * bpm) * bpm + 0.622);

  QTs = (QTs * 5 + QT) / 6; // smoothing
  CurQT = QTs;
  if (mode != mdLargeECG)
    showQT();
  prev[3] = prev[2];
  prev[2] = prev[1];
  prev[1] = prev[0];
  prev[0] = QTinterval;
}
#endif

//---------------------------------------------------------------------------
bool findRpeak(int16_t diff, int16_t x) {
  /*it's an R_peak if it exceeds dmin..dmax*/
  static float dmax = 0; /*historical maximum of diferential; decays towards 0*/
  static float dmin = 0; /*historical minimum of diferential; decays towards 0*/
  static float alpha = 0;
  bool result;

  if (alpha == 0)
    alpha = TimeConst(sps * 2); /*time const of decay*/

  result = false;
  dmax = dmax * alpha;
  if (diff > dmax)
  {
    dmax = diff;
    result = true;
  }

  dmin = dmin * alpha;
  if (diff < dmin)
  {
    dmin = diff;
    result = true;
  }

#if defined bDebug && defined bWindows
  Form1->PaintBox1->Canvas->Pixels[x][Form1->PaintBox1->Height / 2 - dmin] = clBlue;
  Form1->PaintBox1->Canvas->Pixels[x][Form1->PaintBox1->Height / 2 - dmax] = clBlue;
#endif

  return result;
}

//---------------------------------------------------------------------------
// differentiate
//---------------------------------------------------------------------------
int16_t differentiate(int16_t a) {
  static int16_t buf[nDiff + 1]  = {0, 0, 0, 0};
  int16_t d;
  int16_t i;

  for (i = nDiff - 1; i >= 0; i--)
    buf[i + 1] = buf[i];
  buf[0] = a;
  d = buf[0] - buf[nDiff];
  return d;
}

//-----------------------------------------------------------------------------
// AnalysePPG
//-----------------------------------------------------------------------------
#ifdef bHasPAT
void AnalysePPG(uint16_t x, int16_t ppg, int16_t sinceRpeak)
{
#define TC 20
  static int16_t prevppg = 0;
  static float diff = 0;
  static float maxdiff = 0;
  static int16_t tmaxdiff = 0;

  diff = diff * (1.0 - 1.0 / TC) + (ppg - prevppg) * (1.0 / TC);
  prevppg = ppg;

  #ifdef bDebug
    if (mode == mdLargeECG) {
      DrawPixel(x - 1, TFT_HEIGHT - 50 - diff * TC, TFT_GREEN);
    }
    static int16_t xmaxdiff = 0;
    if (diff > maxdiff)
      xmaxdiff = x - TC / 5;
  #endif

  if (diff > maxdiff) {
    tmaxdiff = sinceRpeak;
    maxdiff = diff;
  }

  if (sinceRpeak == 0) {
#ifdef bDebug
    DrawVLine(xmaxdiff, 150, 40, TFT_GREEN);
#endif
    CurPAT = (tmaxdiff - TC / 2) * SamplePeriod;
    tmaxdiff = 0;
    maxdiff = 0;
  }
}
#endif

//---------------------------------------------------------------------------
void AnalyseEPG(uint8_t ecg, uint16_t x, int16_t ppg)
{
  int16_t i, diff;
  float qi, w, g;

  static int16_t bpm = 0;
  static int16_t sinceRpeak = 1000;
  static float b = 0;
  static float c = 0;
  static int16_t ymax = 0;
  static int16_t n = sps / 10;
  static int16_t sinceymax = 1000;
  static float Stt = 0;
  static float Sa = 0;
  static float St = 0;
  static float Sta = 0;
  static float Sw = 0;

  /*Draw dy/dx---------------*/
  diff = differentiate(ecg);
#if defined bDebug && defined bWindows
  static int16_t prevdiff = 0;
  Form1->PaintBox1->Canvas->MoveTo(x - 1, Form1->PaintBox1->Height / 2 - prevdiff);
  Form1->PaintBox1->Canvas->LineTo(x, Form1->PaintBox1->Height / 2 - diff);
  prevdiff = diff;
#endif

  sinceRpeak++;
  lastNegDiff1++;
  lastNegDiff2++;
  if (sinceymax >= 0)
    sinceymax++;
  if ((diff < 0))
    lastNegDiff1 = 0;

  if (diff < 0)
    lastNegDiff1 = 0;

  /*find R peak---------------*/
  if (findRpeak(diff, x)) {
    i = RpeakFound(x, sinceRpeak);
    if (i > 0)
    {
      bpm = i;
      if (mode == mdPoincare)
        DrawPoincare(sinceRpeak * SamplePeriod);
      ymax = 0;
      sinceRpeak = 0;
    }
  }

#ifdef bHasQT
  /*find baseline---------------*/
  g = findbaseline(ecg, diff);
#ifdef bDebug
  if (mode == mdLargeECG)
    DrawPixel(x - nDiff, TFT_HEIGHT - g, TFT_MAGENTA);
#endif

  /*find T peak---------------*/
  if ((sinceRpeak > sps / 5)) /*ignore the hump before 200mS*/
    if ((sinceRpeak < sps / 2)) /*ignore the hump after 500mS*/
    {
      if (ecg > ymax)
      {
        ymax = ecg;
        sinceymax = 0;
        Stt = 0;
        Sta = 0;
        Sa = 0;
        St = 0;
      }
    }

  if ((sinceymax > 0))
  {
    w = sinceymax * (n - 1 - sinceymax);
    Stt = Stt + sinceymax * sinceymax * w;
    Sta = Sta + sinceymax * ecg * w;
    Sa = Sa + ecg * w;
    St = St + sinceymax * w;
  }

  if ((sinceymax == n - 1)) /*100mS after peak of t*/
  {
    Sw = n * (n - 1) * (n - 2) / 6;

    if (Stt > 0)
    {
      b = (Sta * Sw - St * Sa) / (Stt * Sw - St * St);
      c = (Sa - b * St) / Sw;

      if (b < 0)
      {
#ifdef bDebug
        if (mode == mdLargeECG)
          DrawLine(x - sps / 10 + (-sps / 10), TFT_HEIGHT - (b * (-sps / 10) + c), x - sps / 10 + (2 * sps / 10), TFT_HEIGHT - (b * (2 * sps / 10) + c), TFT_OLIVE);
#endif

        qi = (g - c) / b;

        if (qi > 0)
        {
          qi = qi - sinceymax + lastNegDiff2 + nDiff;
          HaveQT(qi,
#ifdef bDebug
                 x - lastNegDiff2 - nDiff,
#endif
                 bpm);
        }
      }
    }
  }
#endif

#ifdef bHasPAT
  AnalysePPG(x, ppg, sinceRpeak);
#endif
}

//-----------------------------------------------------------------------------
// DrawTraceLargePPG
//-----------------------------------------------------------------------------
#ifdef bHasPPG
void DrawTraceLargePPG(uint16_t x, int16_t ppg)
{
  static uint8_t TracePPG[TFT_WIDTH];
  static uint8_t pt = 0;
  static uint8_t py = 0;
  static int16_t ofs = 0;
  static bool bShow = false;

  if (x == 0) {
    ofs = ppgmin - 20;
    ppgmin = 1024;
    bShow = (ppg >= 0) && (ppgEnergy > 10);
    ppgEnergy = 0;
  }

  if (PPGreceived >= PPGreceivedMin) {
    if (bShow)
      ppg = constrain(TFT_HEIGHT - ppg + ofs, 0, TFT_HEIGHT); else
      ppg = 0;
  
    if (x < TFT_WIDTH) {
      DrawGridVLine(x, pt, TracePPG[x]);
  
      if ((x > 0) && ((ppg != 0) || (py != 0)))
        if (ppg >= py)
            DrawVLine(x, py, ppg - py + 1, TFT_CYAN); else
          DrawVLine(x, ppg, py - ppg + 1, TFT_CYAN);
      py = ppg;
  
      pt = TracePPG[x];
      TracePPG[x] = ppg;
    }
  }
}
#endif

//-----------------------------------------------------------------------------
// DrawTraceLargeECG
//-----------------------------------------------------------------------------
void DrawTraceLargeECG(uint16_t x, int16_t ecg, int16_t ppg)
{
  static uint8_t pt = 0;
  static uint8_t py = 0;
  static uint8_t TraceECG[TFT_WIDTH];

  AnalyseEPG(ecg, x, ppg);

  ecg = constrain(TFT_HEIGHT - ecg, 0, TFT_HEIGHT - 1);

#ifdef bDebug
#ifdef bWindows
  if (x == 0)
    Form1->PaintBox1->Canvas->FillRect(Form1->PaintBox1->ClientRect);
  RemoveColor(x, clAqua);
  RemoveColor(x, clYellow);
  RemoveColor(x, clFuchsia);
#endif
  DrawVLine(x, TFT_HEIGHT - 10, 10, 0);
#endif

  if (x < TFT_WIDTH) {
    DrawGridVLine(x, pt, TraceECG[x]);
    if (ecg >= py)
      DrawVLine(x, py, ecg - py + 1, TFT_WHITE); else
      DrawVLine(x, ecg, py - ecg + 1, TFT_WHITE);
    py = ecg;

    pt = TraceECG[x];
    TraceECG[x] = ecg;
  }

  if (x == TFT_WIDTH - 1) {
    showBPM();
#ifdef bHasQT
    showQT();
#endif

#ifdef bHasPAT
    showPAT();
#endif
  }
}

//-----------------------------------------------------------------------------
// DrawTraceLarge
//-----------------------------------------------------------------------------
void DrawTraceLarge(int16_t ecg, int16_t ppg)
{
  static uint16_t x = 0;

  if (ecg < 0) {
    x = 0;
  } else {
    x++;
    x = x % DisplayRepeat;
    DrawTraceLargeECG(x, ecg, ppg);
#ifdef bHasPPG
    DrawTraceLargePPG(x, ppg);
#endif
  }
}

//-----------------------------------------------------------------------------
// DrawTraceSmallPPG
//-----------------------------------------------------------------------------
#ifdef bHasPPG
void DrawTraceSmallPPG(uint16_t x, uint16_t yofs, int16_t ppg)
{
  //  static uint8_t TracePPG[TFT_WIDTH];
  //  static uint8_t pt = 0;
  static uint8_t py = 0;
  //  int16_t i;
  static int16_t ofs = 0;
  static bool bShow = false;

  if (x == 1) {
    if (ppgmin < 1024)
      ofs = ppgmin / 4 - 4;
    ppgmin = 1024;
    bShow = ppg >= 0;
    ppgEnergy = 0;
  }

   if (PPGreceived >= PPGreceivedMin) {
     if (bShow) {
      ppg = TFT_HEIGHT / 4 - ppg / 4 + ofs;
  
      if (x / 4 > 0)
        if (ppg >= py)
            DrawVLine(x / 4, py + yofs, ppg - py + 1, TFT_CYAN); else
          DrawVLine(x / 4, ppg + yofs, py - ppg + 1, TFT_CYAN);
  
      py = ppg;
    }
   }
 }
#endif


//-----------------------------------------------------------------------------
// DrawTraceSmall
//-----------------------------------------------------------------------------
void DrawTraceSmall(int16_t ecg, uint16_t ppg)
{
  static uint16_t x = 0;
  static uint8_t py = 0;
  static uint8_t yofs = 0;
  int16_t x4, y4;

  if (ecg < 0) {
    x = 0;
    return;
  }

  x++;
  x = x % (TFT_WIDTH * 4);
  x4 = x / 4;
  y4 = ecg / 4;

  if (x == 0) {
    yofs = (yofs + TFT_HEIGHT / 4);
    if (yofs >= TFT_HEIGHT)
      yofs = 0;
  }

  if (x % 4 == 0) {
    if (x4 * 4 * 5 % (1000 / SamplePeriod) == 0)
      DrawVLine(x4, yofs, TFT_HEIGHT / 4, TFT_RED); else
      DrawVLine(x4, yofs, TFT_HEIGHT / 4, TFT_BLACK);
  }

  y4 = constrain(TFT_HEIGHT / 4 - y4, 0, TFT_HEIGHT / 4 - 1);
  if (y4 >= py)
    DrawVLine(x4, py + yofs, y4 - py + 1, TFT_WHITE); else
    DrawVLine(x4, y4 + yofs, py - y4 + 1, TFT_WHITE);

  py = y4;

  AnalyseEPG(ecg, x, ppg);
  #ifdef bHasPPG
    DrawTraceSmallPPG(x, yofs, ppg);
  #endif
}

//-------------------------------------------------------------------------
// MakeFakePulse
//-------------------------------------------------------------------------
#ifdef bFakeECGoutput
void MakeFakePulse(void)
{
  static uint16_t fakePeriod = 1000;
  static uint16_t i = 0;
  i++;
  i = i % (fakePeriod / SamplePeriod);
  digitalWrite(FAKE_OUT, i < 10);
  if (i == 0)
    fakePeriod = (fakePeriod * 3 + random(600, 1300)) / 4;
}
#endif

//-----------------------------------------------------------------------------
// CheckButton
//   check pushbutton and change mode
//-----------------------------------------------------------------------------
void CheckButton(void)
{
  static int btnCnt = 0;
  if (digitalRead(BUTTON_IN)) {
    btnCnt = 0;
  } else {
    btnCnt++;
    if (btnCnt == 255) {
      mode = mdBattery;
      DrawGrid();
    }
    if (btnCnt == 20) {
      mode = mode + 1;
      if (mode > mdPoincare)
        mode = 0;
      DrawGrid();
      DrawTraceSmall(-1, 0);
      DrawTraceLarge(-1, 0);
    }
  }
}

//-----------------------------------------------------------------------------
// CheckLeadsOff
//-----------------------------------------------------------------------------
void CheckLead(byte pin, int y, char c, bool *LO)
{
  static uint8_t i = 0;
  i++;
  return;

  if (digitalRead(pin)) {
    if ((!*LO) || (i < 2)) {
      DrawBox(0, y, 54, 20, TFT_MAROON);
      ILI9341SetCursor(4, y + 15);
      DrawChar(c, LargeFont, TFT_WHITE);
      DrawStringF(F(" off"), LargeFont, TFT_WHITE);
      *LO = true;
    }
    ignoreBeats = 10;
  } else {
    if (*LO) {
      DrawGrid();
      *LO = false;
    }
  }
}

void CheckLeadsOff(void)
{
  static bool LO_P = false;
  static bool LO_N = false;
  CheckLead(LO_P_IN, 20, 'L', &LO_P);
  CheckLead(LO_N_IN, 40, 'R', &LO_N);
}

//-------------------------------------------------------------------------
// Filter
//   Low Pass Filter
//-------------------------------------------------------------------------
int LowPassFilter(int ecg)
{
  static int py = 0;
  static int ppy = 0;
  static int ppecg = 0;
  static int pecg = 0;
  int y;
  static int mid = 0;

  const long filt_a0 = 8775;
  const long filt_a1 = 17550;
  const long filt_a2 = 8775;
  const long filt_b1 = -50049;
  const long filt_b2 = 19612;

  if (ecg > mid)
    mid++; else
    mid--;

  ecg -= mid; // to remove DC offset
  y = (filt_a0 * ecg + filt_a1 * pecg + filt_a2 * ppecg - filt_b1 * py - filt_b2 * ppy) >> 16;
  ppy = py;
  py = y;
  ppecg = pecg;
  pecg = ecg;
  return y + mid;
}

//-------------------------------------------------------------------------
// FilterLowPass
//   Low Pass Filter
//-------------------------------------------------------------------------
int FilterLowPass(int ecg)
{
  static int py = 0;
  static int ppy = 0;
  static int ppecg = 0;
  static int pecg = 0;
  int y;
  static int mid = 0;

  const long filt_a0 = 8775;
  const long filt_a1 = 17550;
  const long filt_a2 = 8775;
  const long filt_b1 = -50049;
  const long filt_b2 = 19612;

  if (ecg > mid)
    mid++; else
    mid--;

  ecg -= mid; // to remove DC offset
  y = (filt_a0 * ecg + filt_a1 * pecg + filt_a2 * ppecg - filt_b1 * py - filt_b2 * ppy) >> 16;
  ppy = py;
  py = y;
  ppecg = pecg;
  pecg = ecg;
  return constrain(y + mid, 0, 1023);
}

//-------------------------------------------------------------------------
// FilterNotch50HzQ1
//   Notch Filter 50Hz
//   Q = 1 or 2
//-------------------------------------------------------------------------
int FilterNotch50HzQ1(int ecg)
{
  static int py = 0;
  static int ppy = 0;
  static int ppecg = 0;
  static int pecg = 0;
  int y;
  static int mid = 0;

  const long filt_a0 = 43691; // Q=1
  const long filt_b2 = 21845; // Q=1

  if (ecg > mid)
    mid++; else
    mid--;

  ecg -= mid; // to remove DC offset
  y = (filt_a0 * (ecg + ppecg) - filt_b2 * ppy) >> 16;
  ppy = py;
  py = y;
  ppecg = pecg;
  pecg = ecg;
  return constrain(y + mid, 0, 1023);
}

//-------------------------------------------------------------------------
// FilterNotch50HzQ2
//   Notch Filter 50Hz
//   Q = 1 or 2
//-------------------------------------------------------------------------
int FilterNotch50HzQ2(int ecg)
{
  static int py = 0;
  static int ppy = 0;
  static int ppecg = 0;
  static int pecg = 0;
  int y;
  static int mid = 0;

  const long filt_a0 = 52429; // Q=2
  const long filt_b2 = 39322; // Q=2

  if (ecg > mid)
    mid++; else
    mid--;

  ecg -= mid; // to remove DC offset
  y = (filt_a0 * (ecg + ppecg) - filt_b2 * ppy) >> 16;
  ppy = py;
  py = y;
  ppecg = pecg;
  pecg = ecg;
  return constrain(y + mid, 0, 1023);
}

//-------------------------------------------------------------------------
// FilterNotch60Hz
//   Notch Filter 60Hz
//   Q = 1
//-------------------------------------------------------------------------
int FilterNotch60Hz(int ecg)
{
  static int py = 0;
  static int ppy = 0;
  static int ppecg = 0;
  static int pecg = 0;
  int y;
  static int mid = 0;

  const long filt_a0 = 44415;
  const long filt_a1 = 27450;
  const long filt_b2 = 23294;

  if (ecg > mid)
    mid++; else
    mid--;

  ecg -= mid; // to remove DC offset
  y = (filt_a0 * (ecg + ppecg) + filt_a1 * (pecg - py) - filt_b2 * ppy) >> 16;
  ppy = py;
  py = y;
  ppecg = pecg;
  pecg = ecg;
  return constrain(y + mid, 0, 1023);
}

//-----------------------------------------------------------------------------
// read3V3
//    voltage at 3V3 pin in mV
//    useful if running on Li-ion
//    (mustn't try to measure Vcc when VRef connected to 3V3 cos it crashes the processor)
//-----------------------------------------------------------------------------
long read3V3(void) {
  long result;

  ACSR = 0x10;
  ADCSRA = 0x97;
  ADCSRB = 0x0;

  // Read 1.1V reference
  ADMUX = _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delayMicroseconds(100);
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1125300L / result; // Back-calculate "3V3" pin in mV
  return result;
}

//-------------------------------------------------------------------------
// CheckBattery
//-------------------------------------------------------------------------
void CheckBattery(void)
{
  const int wBatt = 48;
  const int xBatt = TFT_WIDTH - wBatt;
  const int yBatt = 0;
  static int maxBatt = 3300;
  static uint8_t n = 0;
  int v = read3V3();

  maxBatt = max(maxBatt, v);

  n++;
  if ((maxBatt < 3000) && ((n & 127) == 0)) {
    DrawBox(xBatt, yBatt, wBatt, 20, TFT_MAROON);
    if (n == 0) {
      ILI9341SetCursor(xBatt + 4, yBatt + 15);
      DrawStringF(F("Batt"), LargeFont, TFT_WHITE);
    }
    //      ILI9341SetCursor(xBatt+2, yBatt+15);
    //      DrawInt(maxBatt, LargeFont, TFT_WHITE);
    maxBatt = 0;
  }
}

//-------------------------------------------------------------------------
// setup
//-------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(57600);
  Serial.println("ECG");

  pinMode(ECG_IN, INPUT);
  analogReference(EXTERNAL);
  //  XanalogRead(ECG_IN); // initialise ADC to read input

  pinMode(BUTTON_IN, INPUT_PULLUP);
  pinMode(LO_P_IN, INPUT);
  pinMode(LO_N_IN, INPUT);

  #ifdef bHasPPG
    pinMode(PPG_IN, INPUT_PULLUP);
  #endif

#ifdef bFakeECGoutput
  pinMode(FAKE_OUT, OUTPUT);
#endif

  ILI9341Begin(TFT_CS, TFT_CD, TFT_RST, TFT_WIDTH, TFT_HEIGHT, ILI9341_Rotation3);

  DrawGrid();
}

//-----------------------------------------------------------------------------
// Main routines
// loop
//-----------------------------------------------------------------------------
void loop(void)
{
  int ecg;
  int ppg = 0;

  static unsigned long nextTime = 0;
  unsigned long t;
  static int16_t prev_ppg = 0;

  t = millis();

  if (t > nextTime) {
    if (t > nextTime + 10)
      nextTime = t; else
      nextTime = nextTime + 5;

    #ifdef bFakeInput
      ecg = XanalogRead(ECG_IN);
    #else
      ecg = analogRead(ECG_IN);
    #endif

    ecg = FilterNotch50HzQ1(ecg);
    //    ecg = FilterNotch50HzQ2(ecg);
    //    ecg = FilterLowPass(ecg);
    //    ecg = FilterNotch60Hz(ecg);

    #ifdef bHasPPG
      if ((mode == mdLargeECG) || (mode == mdSmallECG)) {
        #ifdef bFakeInput
          ppg = XanalogRead(PPG_IN);
        #else 
          ppg = analogRead(PPG_IN);
        #endif
        ppgmin = min(ppg, ppgmin);
      }
      ppgEnergy += abs(ppg - prev_ppg);
      prev_ppg = ppg;
      if (ppgEnergy > 10000)
        ppg = -1;
      Serial.print(ppg);
      Serial.print(' ');
      Serial.println(ppgEnergy);
      if ((ppg < 1000) && (PPGreceived < PPGreceivedMin))
        PPGreceived++;
    #endif
    
    #ifdef bFakeECGoutput
      MakeFakePulse();
    #endif

    switch (mode) {
      case mdLargeECG: DrawTraceLarge(ecg / 4, ppg); break;
      case mdSmallECG: DrawTraceSmall(ecg / 4, ppg); break;
      case mdPoincare: AnalyseEPG(ecg / 4, 0, ppg); break;
      case mdBattery: DrawBattVolts(); break;
    }

    CheckButton();
    CheckLeadsOff();
    CheckBattery();
  }
}
