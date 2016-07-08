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

#include "utility.h"
#include "Arduino.h"

// Accounts for timer roll-over
uint32_t timeDiff(uint32_t newTime, uint32_t oldTime)
{
   uint32_t result = 0;

   if (newTime >= oldTime)
   {
      result = newTime - oldTime;
   }
   else
   {
      result = newTime + (~oldTime + 1);
   }
      
   return result;
}

bool CharIsADigit(unsigned char digit)
{
  bool retVal = false;
  if ((digit >= '0') && (digit <= '9'))
  {
    retVal = true;
  }

  return retVal;
}

void printHeaterTemperatureParameters(const char *pName, uint16_t *pParams)
{
  uint8_t i;

  Serial.print(pName);
  for (i = 0; i < 4; i++)
  {
    Serial.print(pParams[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf)
{
  uint16_t inputValue = 0;
  uint8_t i = 0;

  while (pBuf[i] != 10 && pBuf[i] != 13)
  {
    if (CharIsADigit(pBuf[i]))
    {
      inputValue *= 10;
      inputValue += (uint16_t)(pBuf[i] - '0');
    }
    else
    {
      Serial.print(F("DEBUG Invalid input, only digits expected: "));
      Serial.print((char*)pBuf);
      inputValue = *pValue;
      break;
    }
    i++;
  }

  *pValue = inputValue;

  return inputValue;
}

float GetFloatInputValue(float *pValue, uint8_t *pBuf)
{
  float inputValue = 0.0;
  float fractionDivider = 10.0;
  float fraction;
  bool numberHasFraction = false;
  uint8_t i = 0;

  while (pBuf[i] != 10 && pBuf[i] != 13)
  {
    if (CharIsADigit(pBuf[i]))
    {
      if (numberHasFraction)
      {
        fraction = (float)(pBuf[i] - '0');
        fraction = fraction / fractionDivider;
        inputValue = inputValue + fraction;
        fractionDivider = fractionDivider * 10;
      }
      else
      {
        inputValue *= 10;
        inputValue += (float)(pBuf[i] - '0');
      }
    }
    else
    {
      if ('.' == pBuf[i])
      {
        numberHasFraction = true;
      }
      else
      {
        Serial.print(F("DEBUG Invalid input, only digits expected: "));
        Serial.print((char*)pBuf);
        inputValue = *pValue;
        break;
      }
    }
    i++;
  }

  *pValue = inputValue;

  return inputValue;
}

