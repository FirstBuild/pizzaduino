/*
  Copyright (c) 2016 FirstBuild

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

#ifndef UTILITY_H
#define UTILITY_H

#include <stdint.h>
#include "PID_v1.h"

uint32_t timeDiff(uint32_t newTime, uint32_t oldTime);
bool CharIsADigit(unsigned char digit);
void printHeaterTemperatureParameters(const char *pName, uint16_t *pParams);
void printMessageWithTwoUints(uint8_t * text, uint16_t uint1, uint16_t uint2);
void printPidGains(uint8_t * pText, PID *pPid);
uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf);
float GetFloatInputValue(float *pValue, uint8_t *pBuf);
void floatToAscii(double *pVal, uint8_t *pBuf, uint8_t decimals);

// These functions are defined in places other than utility.cpp
void AllHeatersOffStateClear();
void UpdateHeaterHardware();

#endif
