/*
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

#include "relayBoost.h"
#include "pinDefinitions.h"
#include <stdint.h>

// Macros related to relay operation
#define BOOST_MODE_ON_TIME_MS 211
#define BOOST_MODE_BUMP_TIME_MS 103

void boostEnable(RelayBoostMode relayBoostMode)
{
  static uint32_t oldTime = millis();
  static uint32_t onTime = 0;
  uint32_t newTime = millis();

  if (relayBoostMode == relayBoostOn)
  {
    oldTime = newTime;
    digitalWrite(BOOST_ENABLE, HIGH);
    onTime = BOOST_MODE_ON_TIME_MS;
  }

  if (relayBoostMode == relayBoostRun)
  {
    if (digitalRead(BOOST_ENABLE))
    {
      if (newTime >= oldTime) {
        if ((newTime - oldTime) > onTime)
        {
          digitalWrite(BOOST_ENABLE, LOW);
          oldTime = newTime;
        }
      }
      else
      {
        // account for wrap around
        // it is ok if the boost voltage is applied for a longer time.
        oldTime = newTime;
      }
    }
    else
    {
      // handle bumping the boost once per second
      if (newTime >= oldTime)
      {
        if ((newTime - oldTime) >= (1000 - BOOST_MODE_BUMP_TIME_MS))
        {
          oldTime = newTime;
          digitalWrite(BOOST_ENABLE, HIGH);
          onTime = BOOST_MODE_BUMP_TIME_MS;
        }
      }
      else
      {
        // account for wrap around
        // it is ok if the boost voltage is applied for a longer time.
        oldTime = newTime;
      }
    }
  }
}

