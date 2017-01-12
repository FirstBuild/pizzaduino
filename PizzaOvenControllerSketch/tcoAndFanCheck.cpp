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

#include "tcoAndFanCheck.h"
#include "acInput.h"
#include <Arduino.h>

static uint32_t oldMillis;
static bool m_TcoHasFailed = false;
static bool m_CoolingFanHasFailed = false;

extern bool doorHasDeployed;

TcoAndFan::TcoAndFan(void) 
{
  oldMillis = millis();
}

void TcoAndFan::reset(void) 
{
  oldMillis = millis();
}

bool TcoAndFan::areOk(void)
{
  if (powerButtonIsOn() && acPowerIsPresent())
  {
    if(millis() - oldMillis > 5000)
    {
      if (!tcoInputIsOn())
      {
        m_TcoHasFailed = true;
      }
      else if (!sailSwitchIsOn())
      {
        m_CoolingFanHasFailed = true;
      }
    }
  }
  else
  {
    oldMillis = millis();
  }
  return !m_CoolingFanHasFailed && !m_TcoHasFailed && !doorHasDeployed;
}

bool TcoAndFan::coolingFanHasFailed(void)
{
   return m_CoolingFanHasFailed;
}

bool TcoAndFan::tcoHasFailed(void)
{
   return m_TcoHasFailed;
}

