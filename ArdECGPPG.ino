//-----------------------------------------------------------------------------
// Copyright 2023 Peter Balch
//   displays an ECG and PPG
//   computes QT interval
//   computes PAT
//-----------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include "SimpleILI9341.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define bHasPPG
#define bHasPAT
#define bHasQT
#define bHasChartMode

#ifdef bHasPAT
#define bHasPPG
#endif

//#define bDebug
//#define bWindows
//#define bFakeECGoutput
//#define bFakeInput

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
  float CurPAT = 0;
#endif

enum TMode { mdLargeECG, mdSmallECG,  mdPoincare, mdBattery } mode = mdLargeECG;

bool SendingSerial = false;

//---------------------------------------------------------------------------

#ifdef bFakeInput
#include "FakeInput.h"
#endif

//---------------------------------------------------------------------------
// TimeConst
//   calc Alpha for a time constant of a decay
//    n: number of samples for exponential smoothing
//    returns exp(-1 / n);
//---------------------------------------------------------------------------
float TimeConst(float n) {
  return exp(-1 / n);
}

//---------------------------------------------------------------------------
// MedianOfThree
//    c,d,e: three values
//    returns median value
//---------------------------------------------------------------------------
int16_t MedianOfThree(int16_t c, int16_t d, int16_t e) {
  int16_t tmp;
  if (d < c) {    tmp = c;    c = d;    d = tmp;  };
  if (e < c) {    e = c;  };
  if (d < e) return d; else return e;
}

//---------------------------------------------------------------------------
// MedianOfFive
//    a,b,c,d,e: five values
//    returns median value
//---------------------------------------------------------------------------
int16_t MedianOfFive(int16_t a, int16_t b, int16_t c, int16_t d, int16_t e) {
  int16_t tmp;
  if (b < a) {    tmp = a;    a = b;    b = tmp;  }
  if (d < c) {    tmp = c;    c = d;    d = tmp;  }
  if (c < a) {    tmp = b;    b = d;    d = tmp;    c = a;  }
  if (b < e) {    tmp = e;    e = b;    b = tmp;  }
  if (e < c) {    tmp = b;    d = tmp;  e = c;    }
  if (d < e) return d; else return e;
}

//-----------------------------------------------------------------------------
// DrawGridSmall
//   draw empty grid for small display
//-----------------------------------------------------------------------------
void DrawGridSmall(void)
{
  int16_t  i;
  ClearDisplay(TFT_BLACK);

  #ifdef bHasChartMode
    for (i=0; i < TFT_WIDTH; i++)
      if (i*4*5 % sps == 0)
        DrawVLine(i, 0, TFT_HEIGHT/4, TFT_RED);
    for (i=TFT_HEIGHT/4; i < TFT_HEIGHT; i+=TFT_HEIGHT/16)
      if (i % (TFT_HEIGHT/4) == 0)
        DrawHLine(0, i, TFT_WIDTH, TFT_DARKGRAY); else
        DrawHLine(0, i, TFT_WIDTH, TFT_MAROON);
  #else
    for (i = 0; i < TFT_WIDTH; i++)
      if (i*4*5 % sps == 0)
        DrawVLine(i, 0, TFT_HEIGHT, TFT_RED);
  #endif
}

//-----------------------------------------------------------------------------
// DrawPoincareLine
//   draw a Line on the Poincare display
//    x1,y1: start point
//    x2,y2: end point
//    color: the color it is to be drawn
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
//   draw a rectangle Poincare display 
//    x1,y1,x2,y2: the corners of the frame
//    color: the color it is to be drawn
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
//   draw Poincare display background
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
//    x: column
//    y1,y2: ends of line
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
//   draw the current BPM in the Poincare display 
//    Poincare plot
//    init: initialise the points array?
//-----------------------------------------------------------------------------
void DrawPoincare(bool init)
{
  const int nPeriods = 400;
  static uint8_t Periods[nPeriods] = {0};
  static int16_t i = 0;
  int16_t prev,t;

  if (init) {
    memset(Periods, 0, sizeof(Periods));
    return;
  }

  t = 60000 / (CurBPM*PoincareScale);

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
//   draw the background of the Large display 
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
//   draw the background of the current display 
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
  DrawPoincare(true);
  ignoreBeats = 2;
}

//-----------------------------------------------------------------------------
// centrePeak
//   attempt to keep a peak in the first third of the screen
//    x: position of the peak being drawn
//-----------------------------------------------------------------------------
void centrePeak(uint16_t x)
{
  uint16_t period = (60000L/SamplePeriod) /CurBPM;
  int16_t b;
  static int16_t dr = 480;
  static int16_t dt = 0;

  b = period;
  while (b < TFT_WIDTH)
    b += period;
  if (b > dr+10)
    dr += 10; else
  if (b < dr-10)
    dr -= 10;
  if (b > dr)
    dr++; else
    dr--;
  dr = constrain(dr,TFT_WIDTH-1,2*TFT_WIDTH);
  DisplayRepeat = dr;
  if (x < 80)
    dt--; else
  if (x < 160)
    dt++;
  DisplayRepeat += dt;
  DisplayRepeat = constrain(DisplayRepeat,TFT_WIDTH-1,2*TFT_WIDTH);
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

//-----------------------------------------------------------------------------
// DrawChart
//   draw the current BPM and PAY in the vhart display 
//    init: initialise the display?
//-----------------------------------------------------------------------------
#ifdef bHasChartMode
void DrawChart(bool init)
{
  static int16_t x = -1;
  static uint8_t pyBPM = 0;
  static uint8_t pyPAT = 0;
  static uint8_t yofs = TFT_HEIGHT/4;
  int16_t y4, y;
  const int16_t  minBPM = 40;
  const int16_t  maxBPM = 150;
  const int16_t  minPAT = 150;
  const int16_t  maxPAT = 400;
  const int16_t  BPMmaxRed = 120; // BPM above this shown as red
  const int16_t  BPMmaxYellow = 100; // BPM above this shown as Yellow
  const int16_t  BPMminYellow = 50; // BPM below this shown as Yellow
  const int16_t  BPMminRed = 40; // BPM below this shown as red

  if (init) {
    x = -1;
    yofs = 0;
    return;
  }

  x++;
  x = x % TFT_WIDTH;

  if (x == 0) {
    yofs = (yofs+TFT_HEIGHT/4);
    if (yofs >= TFT_HEIGHT)
      yofs = TFT_HEIGHT/4;
  }

  DrawVLine(x, yofs+1, TFT_HEIGHT/4-1, TFT_BLACK);
  DrawVLine(x+1, yofs+1, TFT_HEIGHT/4-1, TFT_WHITE);
  DrawPixel(x, yofs, TFT_DARKGRAY);
  DrawPixel(x, yofs+TFT_HEIGHT/16, TFT_MAROON);
  DrawPixel(x, yofs+2*TFT_HEIGHT/16, TFT_MAROON);
  DrawPixel(x, yofs+3*TFT_HEIGHT/16, TFT_MAROON);

  // display BPM
  y4 = (CurBPM-minBPM)*TFT_HEIGHT/(4*(maxBPM-minBPM));
  y = constrain(TFT_HEIGHT/4-y4, 0,TFT_HEIGHT/4-1);
  if (x == 0)
    pyBPM = y;

  if ((CurBPM > BPMmaxRed) || (CurBPM < BPMminRed))
    DrawVLine(x, y+yofs, y4, TFT_MAROON); else
  if ((CurBPM > BPMmaxYellow) || (CurBPM < BPMminYellow))
    DrawVLine(x, y+yofs, y4, TFT_OLIVE); else
    DrawVLine(x, y+yofs, y4, TFT_DARKGREEN);

  if (y >= pyBPM)
    DrawVLine(x, pyBPM+yofs, y-pyBPM+1, TFT_WHITE); else
    DrawVLine(x, y+yofs, pyBPM-y+1, TFT_WHITE);

  pyBPM = y;

  #ifdef bHasPAT
    // display PAT
    y4 = (CurPAT-minPAT)*TFT_HEIGHT/(4*(maxPAT-minPAT));
    y = constrain(TFT_HEIGHT/4-y4, 0,TFT_HEIGHT/4-1);
    if (x == 0)
      pyPAT = y;

    if (y >= pyPAT)
      DrawVLine(x, pyPAT+yofs, y-pyPAT+1, TFT_CYAN); else
      DrawVLine(x, y+yofs, pyPAT-y+1, TFT_CYAN);

    pyPAT = y;
  #endif
}
#endif

//-----------------------------------------------------------------------------
// DrawCharCol
//   draws a single column of a character
//   used by DrawBoxIntCol
//     c: char to be drawn
//     Cursorx: position of char
//     Cursory: position of char
//     x: column to be drawn
//     Font: Font
//     color: foreground color
//-----------------------------------------------------------------------------
bool DrawCharCol(uint8_t c, int x, word Font, uint16_t color) {
  word n, j;
  byte ymax, desc;
  unsigned long b,d;

  ymax = pgm_read_byte_near(Font); // height of char
  Font++;
  desc = pgm_read_byte_near(Font); // size of descender
  Font++;

  j = pgm_read_byte_near(Font); // first char
  Font++;
  if (c < j) return false;

  while (c > j) {
    b = pgm_read_byte_near(Font); // number of columns in char
    if (b == 0)
      return false;
    if (ymax > 15)
      Font += b * 3 + 1;
    else if (ymax > 7)
      Font += b * 2 + 1;
    else
      Font += b + 1;
    c--;
  }

  // c now points at the start of the required character
  n = pgm_read_byte_near(Font); // number of columns in char
  Font++;

  if (Cursorx+n < x) {
    Cursorx += n;
  } else {
    while (n > 0) {
      if (Cursorx == x) {
        b = pgm_read_byte_near(Font);
        Font++;
        if (ymax > 7) {
          d = pgm_read_byte_near(Font);
          b |= d << 8;
          Font++;
        }
        if (ymax > 15) {
          d = pgm_read_byte_near(Font);
          b |= d << 16;
          Font++;
        }
        for (j = 0; j <= ymax; j++) {
          if (b & 1) 
            DrawPixel(Cursorx, Cursory + desc - j, color);
          b = b >> 1;
        }
        return true;
      } else {
        Font += ymax / 8 + 1;
      }

      Cursorx++;
      n--;
    }
  }
  Cursorx += letter_gap;
  return false;
}

//-----------------------------------------------------------------------------
// DrawBoxIntCol
//   draws a single column of an int in a box
//   needed because it takes 40mS to draw a whole int in a box
//     left: left edge of the box on the screen
//     top: top edge of the box
//     width: width of the box
//     height: height of the box
//     xOffset: left-hand offset to chars
//     yOffset: top offset to chars
//     x: position on the screen of the column to be drawn
//     i: integer value to be drawn
//     Font: Font
//     FG: foreground color
//     BG: background color
//-----------------------------------------------------------------------------
void DrawBoxIntCol(int left, int top, int width, int height, int xOffset, int yOffset,
    int x, int i, word Font, uint16_t FG, uint16_t BG) {

  if ((x >= left) && (x < left+width)) {
    DrawVLine(x, top, height, BG);

    if (FG == BG)
      return;

    ILI9341SetCursor(left+xOffset,top+yOffset);

    bool HasDigit = false;
    int n =  10000;
    while (n > 0) {
      if ((i >= n) || (n == 1) || HasDigit) {
        if (DrawCharCol('0' + (i / n), x, Font, FG))
          return;
        HasDigit = true;
      }
      i %= n;
      n /= 10;
    }
  }
}

//---------------------------------------------------------------------------
// showBPM
//---------------------------------------------------------------------------
void showBPM(int x) {
  const int left = 0;
  static int i;
  if (x == left)
    i = CurBPM;
  DrawBoxIntCol(left, 0, 30, 20, 4,16, x, i, LargeFont, TFT_WHITE, TFT_NAVY);
}

//---------------------------------------------------------------------------
// showQT
//---------------------------------------------------------------------------
#ifdef bHasQT
void showQT(int x) {
  const int left = TFT_WIDTH-30;
  static int i;
  if (x == left)
    i = CurQT;

  DrawBoxIntCol(left, 0, 30, 20, 2,16,
    x, i, LargeFont, TFT_WHITE, TFT_PURPLE);
}
#endif

//---------------------------------------------------------------------------
// showPAT
//---------------------------------------------------------------------------
#ifdef bHasPAT
void showPAT(int x) {
  #ifdef bHasQT
    const int y = 20;
  #else
    const int y = 0;
  #endif
  const int left = TFT_WIDTH-30;

  if ((CurPAT > 150) && (CurPAT < 500)) {
    static int i;
    if (x == left)
      i = CurPAT;

    DrawBoxIntCol(left, y, 30, 20, 2,16,
      x, i, LargeFont, TFT_BLACK, TFT_CYAN);
  } else {
    DrawBoxIntCol(left, y, 30, 20,
      2,17,
      x, 0, LargeFont, TFT_BLACK, TFT_BLACK);
  }
}
#endif

//---------------------------------------------------------------------------
// drawBPM
//   filters and draws BPM in the chart
//    b: rough estimate of bpm
//---------------------------------------------------------------------------
void drawBPM(int16_t b)
{
  static int16_t prev[4] = {60, 60, 60, 60};
  static float bpm = 60;

  if (ignoreBeats > 0) {
    ignoreBeats--;
  } else {    
    bpm = bpm + (MedianOfFive(prev[0], prev[1], prev[2], prev[3], b) - bpm)/4; // smoothing
    CurBPM = bpm;

    prev[3] = prev[2];
    prev[2] = prev[1];
    prev[1] = prev[0];
    prev[0] = b;

    #ifdef bHasChartMode
      if (mode == mdSmallECG)
        DrawChart(false);
    #endif
  }
}

//---------------------------------------------------------------------------
// RpeakFound
//   an R-peak has been found, deall with it
//    x: x-coord of peak
//    sinceRpeak: time since previous R-peak (in mS)
//    return: whether this is a good peak
//---------------------------------------------------------------------------
bool RpeakFound(int16_t x, int16_t sinceRpeak) {
  int16_t bpm;

  if (sinceRpeak < 200) /*R_peaks can't be less than 200mS apart*/
    return false;

  lastNegDiff2 = lastNegDiff1;
  bpm = 60000/sinceRpeak;

  #if defined bDebug && defined bWindows
    DrawX(Form1->PaintBox1->Canvas,x-sinceRpeak/SamplePeriod,10,5);
    CanvasTextOut(Form1->PaintBox1->Canvas, x+3,10,"%d",bpm);
  #endif

  drawBPM(bpm);
  centrePeak(x);
  return true;
}

//-----------------------------------------------------------------------------
// findbaseline
//   calculate the baseline value
//   a is the latest sample*/
//   g is baseline*/
//   exponentially "decays" to current sample but tc is longer the further away aa is from the current sample*/
//    diff: differential of ecg sample values
//    return: estimate of the "baseline"
//-----------------------------------------------------------------------------
float findbaseline(int16_t a, int16_t diff) {
  static float g = TFT_HEIGHT / 2; // estimate of baseline
  float f;

  f  =  1 / (sqr(diff) + sqr(a - g) + 1);
  g = g * (1 - f) + a * f;
  return g;
}

//---------------------------------------------------------------------------
//  HaveQT
//   have calculated a value of the QT interval
//    QTinterval in samples
//    x: where to draw the yellow bar (debugging)
//    bpm: in order to calculate QTc
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
  prev[3] = prev[2];
  prev[2] = prev[1];
  prev[1] = prev[0];
  prev[0] = QTinterval;
}
#endif

//---------------------------------------------------------------------------
// findRpeak
//   is the current ecg making an R-peak?
//    diff: differential of ecg sample values
//    x: x-coord of peak (for debugging)
//    return: whether this is a peak
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
//   differentiate the current ecg 
//    a: ecg sample value
//    return: differential of ecg sample values
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
//   analyse the current PPG and xalxulate the PAT
//    x: x-coord being displayed
//    ppg: ppg sample value
//    timeSinceR: time since previous R-peak (in mS)
//-----------------------------------------------------------------------------
#ifdef bHasPAT
void AnalysePPG(uint16_t x, int16_t ppg, int16_t timeSinceR)
{
  #define TC 20
  static int16_t prevppg = 0;
  static float diff = 0;
  static float maxdiff = 0; // in samples
  static int16_t tmaxdiff = 0;
  static int16_t prev[5] = {360, 360, 360, 360, 360};

  diff = diff*(1.0-1.0/TC) + (ppg-prevppg)*(1.0/TC);
  prevppg = ppg;

  #ifdef bDebug
    if (mode == mdLargeECG)
      DrawPixel(x-1, TFT_HEIGHT-50-diff*TC, TFT_GREEN);
    static int16_t xmaxdiff = 0;
    if (diff > maxdiff)
      xmaxdiff = x-TC/5;
  #endif

  if (diff > maxdiff) {
    tmaxdiff = timeSinceR/SamplePeriod;
    maxdiff = diff;
  }

  if (timeSinceR == 0) {
    #ifdef bDebug
      if (mode == mdLargeECG)
        DrawVLine(xmaxdiff, 150, 40, TFT_GREEN);
    #endif

    prev[4] = prev[3];
    prev[3] = prev[2];
    prev[2] = prev[1];
    prev[1] = prev[0];
    prev[0] = (tmaxdiff-TC/2)*SamplePeriod;
    CurPAT = CurPAT + (MedianOfFive(prev[0], prev[1], prev[2], prev[3], prev[4]) - CurPAT)/4; // smoothing

    tmaxdiff = 0;
    maxdiff = 0;
  }
}
#endif

//---------------------------------------------------------------------------
//  AnalyseEPG
//   analyse the current ECG and xalxulate the QT
//    ecg: ecg sample value
//    x: x-coord being displayed
//    ppg: ppg sample value
//---------------------------------------------------------------------------
void AnalyseEPG(uint8_t ecg, uint16_t x, int16_t ppg)
{
  int16_t i, diff;
  float qi, w, g;

  static long RpeakTime = 0;
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
  static int16_t sinceRpeak = 0;

  /*Draw dy/dx---------------*/
  diff = differentiate(ecg);
  #if defined bDebug && defined bWindows
    static int16_t prevdiff = 0;
    Form1->PaintBox1->Canvas->MoveTo(x - 1, Form1->PaintBox1->Height / 2 - prevdiff);
    Form1->PaintBox1->Canvas->LineTo(x, Form1->PaintBox1->Height / 2 - diff);
    prevdiff = diff;
  #endif

  lastNegDiff1++;
  lastNegDiff2++;
  if (sinceymax >= 0)
    sinceymax++;
  if ((diff < 0))
    lastNegDiff1 = 0;

  if (diff < 0)
    lastNegDiff1 = 0;

  /*find R peak---------------*/
  sinceRpeak++;
  if (findRpeak(diff, x) && RpeakFound(x,millis()-RpeakTime))
  {
    if (mode == mdPoincare)
      DrawPoincare(false);
    ymax = 0;
    RpeakTime = millis();
    sinceRpeak = 0;
  }

  int16_t timeSinceR = millis()-RpeakTime;
  
  #ifdef bHasQT
    /*find baseline---------------*/
    g = findbaseline(ecg, diff);
    #ifdef bDebug
      if (mode == mdLargeECG)
        DrawPixel(x - nDiff, TFT_HEIGHT - g, TFT_MAGENTA);
    #endif
  
    /*find T peak---------------*/
    if ((timeSinceR > 200) && (timeSinceR < 500)) /*ignore the hump before 200mS and after 500mS*/
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
              CurBPM);
          }
        }
      }
    }
  #endif

  #ifdef bHasPAT
    AnalysePPG(x, ppg, timeSinceR);
  #endif
}

//-----------------------------------------------------------------------------
// DrawTraceLargePPG
//   draw the PPG in the Large display
//    x: x-coord being displayed
//    ppg: ppg sample value
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
  
      if ((x > 0) && ((ppg != 0) || (py != 0))) {
        if (ppg >= py)
            DrawVLine(x, py, ppg - py + 1, TFT_CYAN); else
          DrawVLine(x, ppg, py - ppg + 1, TFT_CYAN);
      }
      py = ppg;
  
      pt = TracePPG[x];
      TracePPG[x] = ppg;
    }
  }
}
#endif

//-----------------------------------------------------------------------------
// DrawTraceLargeECG
//   draw the ECG in the Large display
//    x: x-coord being displayed
//    ecg: ecg sample value
//    ppg: ppg sample value
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
}

//-----------------------------------------------------------------------------
// DrawTraceLarge
//   draw the ECG and PPG in the Large display
//    ecg: ecg sample value
//    ppg: ppg sample value
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

    showBPM(x);
    #ifdef bHasQT
      showQT(x);
    #endif
    #ifdef bHasPAT
      showPAT(x);
    #endif
  }
}

//-----------------------------------------------------------------------------
// DrawPPGTraceSmall
//   draw the PPG in the Small display
//    x: x-coord being displayed
//    yofs: which quarter of the screen we're in
//    ppg: ppg sample value
//-----------------------------------------------------------------------------
#ifdef bHasPPG
void DrawPPGTraceSmall(uint16_t x, uint16_t yofs, int16_t ppg)
{
  static uint8_t py = 0;
  static float ofsq = 0;
  static bool bShow = false;

  if (x == 1) {
    bShow = ppg >= 0;
    ppgEnergy = 0;
  }

  if (PPGreceived >= PPGreceivedMin) {
    if (bShow) {
      ppg = TFT_HEIGHT/4-ppg/4;
      ofsq = ofsq+(TFT_HEIGHT*3/(4*4) - ppg - ofsq)/100;
      if (ppg + ofsq > TFT_HEIGHT/4)
        ofsq = TFT_HEIGHT/4 - ppg;
      if (ppg + ofsq < 0)
        ofsq = -ppg;
      ppg = ppg + ofsq;
  
      #ifdef bHasChartMode
        ppg = constrain(ppg,0,TFT_HEIGHT/4-1);
      #endif

      if (x / 4 > 0) {
        if (ppg >= py)
          DrawVLine(x / 4, py + yofs, ppg - py + 1, TFT_CYAN); else
          DrawVLine(x / 4, ppg + yofs, py - ppg + 1, TFT_CYAN);
      }
  
      py = ppg;
    }
  }
}
#endif

//-----------------------------------------------------------------------------
// DrawTraceSmall
//   draw the ECG and PPG in the Small display
//    ecg: ecg sample value
//    ppg: ppg sample value
//-----------------------------------------------------------------------------
void DrawTraceSmall(int16_t ecg, uint16_t ppg)
{
  static uint16_t x = 0;
  static uint8_t ymin = 0;
  static uint8_t ymax = 0;
  static uint8_t yofs = 0;
  int16_t x4, y4;

  if (ecg < 0) {
    x = 0;
    return;
  }

  x++;
  x = x % (TFT_WIDTH*4);
  x4 = x/4;
  y4 = ecg/4;

  #ifdef bHasChartMode
    yofs = 0;
  #else
    if (x == 0) {
      yofs = (yofs+TFT_HEIGHT/4);
      if (yofs >= TFT_HEIGHT)
        yofs = 0;
    }
  #endif
  y4 = constrain(TFT_HEIGHT/4-y4, 0,TFT_HEIGHT/4-1);

  ymin = min(ymin, y4);
  ymax = max(ymax, y4);

  if (x % 4 == 0) {
    if (x4*4*5 % sps == 0)
      DrawVLine(x4, yofs, TFT_HEIGHT/4, TFT_RED); else
      DrawVLine(x4, yofs, TFT_HEIGHT/4, TFT_BLACK);
    DrawVLine(x4, ymin+yofs, ymax-ymin+1, TFT_WHITE);
    ymin = y4;
    ymax = y4;
  }

  AnalyseEPG(ecg, x, ppg);
  #ifdef bHasPPG
    DrawPPGTraceSmall(x, yofs, ppg);
  #endif
  
  if (x % 4 == 3) {
    showBPM(x4);
    showBPM(x4 - TFT_WIDTH/2-15);
    #ifdef bHasQT
      showQT(x4);
      showQT(x4 + TFT_WIDTH/2+15);
    #endif
    #ifdef bHasPAT
      showPAT(x4);
      showPAT(x4 + TFT_WIDTH/2-15);
    #endif
  }
}

//-------------------------------------------------------------------------
// MakeFakePulse
//   output a fake "pulse" to the FAKE_OUT pin 
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
      #ifdef bHasChartMode
        DrawChart(true);
      #else
        DrawTraceSmall(-1, 0);
      #endif
      DrawTraceLarge(-1, 0);
    }
  }
}

//-----------------------------------------------------------------------------
// CheckLeadsOff
//   check for Leads-Off and display error message
//    pin: Arduino pin number
//    y: where to display error message
//    c: 'L' or 'R'
//    LO: whether is error message currently drawn
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
// FilterLowPass
//   filter the ECG 
//     Low Pass Filter
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
//   filter the ECG 
//     Notch Filter 50Hz
//     Q = 1 
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
//   filter the ECG 
//     Notch Filter 50Hz
//     Q = 2
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
//   filter the ECG 
//     Notch Filter 60Hz
//     Q = 1
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
//   warn if the Battery is low
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
  ILI9341fast = true;

  DrawGrid();
}

//-----------------------------------------------------------------------------
// Main loop
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

    if (SendingSerial)
      Serial.print(ecg);

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
      if ((ppg < 1000) && (PPGreceived < PPGreceivedMin))
        PPGreceived++;
      if (SendingSerial) {
        Serial.print(' ');
        Serial.print(ppg);
      }
    #endif
    if (SendingSerial)
      Serial.println("");
    
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

    if ((Serial.available()) && (Serial.read() == 'S'))
      SendingSerial = true;
  }
}
