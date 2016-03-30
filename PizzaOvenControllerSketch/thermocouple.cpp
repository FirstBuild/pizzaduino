/*
  thermocouple.cpp

  Handle reading the thermocouples

  Copyright (c) 2015 FirstBuild

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/

#include <Arduino.h>
#include "thermocouple.h"
#include <avr/pgmspace.h>
#include <wiring_private.h>
#include "adcRead.h"

static float AnalogTCVolts(uint16_t rawA2D);
static float AD8495KTCInterpolate(float A2D);
static float FmultiMap(float val, const float * _in, const float * _out, uint8_t size);

// Hard Coded to check correct number of values are defined

#define A2D8495_K_TABLE_SIZE  (47)

// Analog Devices AD8495 K Thermocouple Lookup Tables
//  Table of A2D Values to be interpolated
const PROGMEM float analogAD8495KLookup[A2D8495_K_TABLE_SIZE] =
{
  0.003, 0.100, 0.125, 0.200, 0.301, 0.402, 0.504, 0.605, 0.705, 0.803,
  0.901, 0.999, 1.097, 1.196, 1.295, 1.396, 1.497, 1.599, 1.701, 1.803,
  1.906, 2.010, 2.113, 2.217, 2.321, 2.425, 2.529, 2.634, 2.738, 2.843,
  2.947, 3.051, 3.155, 3.259, 3.362, 3.465, 3.568, 3.670, 3.772, 3.874,
  3.975, 4.076, 4.176, 4.275, 4.374, 4.473, 4.571
};

// Table of temperatures to be interpolated
//const uint16_t analogAD8495KTemp[A2D8495_K_TABLE_SIZE] =
const PROGMEM float analogAD8495KTemp[A2D8495_K_TABLE_SIZE] =
{
    0,  20,  25,  40,  60,  80, 100, 120, 140, 160,
  180, 200, 220, 240, 260, 280, 300, 320, 340, 360,
  380, 400, 420, 440, 460, 480, 500, 520, 540, 560,
  580, 600, 620, 640, 660, 680, 700, 720, 740, 760,
  780, 800, 820, 840, 860, 880, 900
};

// Modified for _out to be uint8_t, so cast as (float) before using
static float FmultiMap(float val, const float * _in, const float * _out, uint8_t size)
{
  // take care the value is within range
  if (val <= pgm_read_float(&_in[0])) return pgm_read_float(&_out[0]);
  if (val >= pgm_read_float(&_in[size - 1])) return pgm_read_float(&_out[size - 1]);

  // search right interval (added bound check even though shouldn't hit)
  // TODO: replace with a binary search for speed, if needed
  uint8_t pos = 0;  // _in[0] already tested
  while ((val > pgm_read_float(&_in[pos])) && (pos < size)) pos++;

  // this will handle all exact "points" in the _in array
  if (val == pgm_read_float(&_in[pos])) return pgm_read_float(&_out[pos]);

  // interpolate in the right segment for the rest
  return (val - pgm_read_float(&_in[pos - 1])) * (pgm_read_float(&_out[pos]) - pgm_read_float(&_out[pos - 1])) / 
    (pgm_read_float(&_in[pos]) - pgm_read_float(&_in[pos - 1])) + pgm_read_float(&_out[pos - 1]);
}

static float AD8495KTCInterpolate(float A2D)
{
  float tempC;

  tempC = FmultiMap(A2D, analogAD8495KLookup, analogAD8495KTemp, (uint8_t) A2D8495_K_TABLE_SIZE);
  return tempC;
}

static float AnalogTCVolts(uint16_t rawA2D)
{
  float analogVoltage;

  analogVoltage = (float)rawA2D * ANALOG_REFERENCE_VOLTAGE / 1023.0;

  return analogVoltage;
}

static float c2f(float tempC)
{
  float tempF;

  tempF = tempC * 9 / 5 + 32;

  return tempF;
}

float readAD8495KTC(uint8_t pin)
{
  float tempF;

  tempF = c2f(AD8495KTCInterpolate(AnalogTCVolts(getA2DReadingForPin(pin))));

  return tempF;
  
}



