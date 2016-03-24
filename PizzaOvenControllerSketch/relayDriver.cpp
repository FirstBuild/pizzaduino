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

#include <Arduino.h>
#include "relayDriver.h"
#include "relayBoost.h"
#include <stdint.h>
#include "CircularBuffer.h"

static CircularBuffer<uint8_t, 10> relaysToTurnOn; 
static CircularBuffer<uint8_t, 10> relaysToTurnOff; 

#define RELAY_OPERATION_DELAY_MS 211

void relayDriverInit(void)
{
}

void relayDriverRun(void)
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  uint8_t pin;

  if (newTime >= oldTime)
  {
    if ((newTime - oldTime) >= RELAY_OPERATION_DELAY_MS)
    {
      if (relaysToTurnOff.remain() > 0)
      {
        pin = relaysToTurnOff.pop();
//        Serial.print("Turning relay ");
//        Serial.print(pin);
//        Serial.println(" off.");
        digitalWrite(pin, LOW);
        oldTime = newTime;
      }
      else if (relaysToTurnOn.remain() > 0)
      {
        boostEnable(relayBoostOn);
        pin = relaysToTurnOn.pop();
//        Serial.print("Turning relay ");
//        Serial.print(pin);
//        Serial.println(" on.");
        digitalWrite(pin, HIGH);
        oldTime = newTime;
      }
    }
  }
  else
  {
    oldTime = newTime;
  }
}

void changeRelayState(uint8_t pin, RelayState relayState)
{
  switch(relayState)
  {
    case relayStateOn:
//      Serial.print("Staging relay ");
//      Serial.print(pin);
//      Serial.println(" on.");
      relaysToTurnOn.push(pin);
      break;
      
    case relayStateOff:
//      Serial.print("Staging relay ");
//      Serial.print(pin);
//      Serial.println(" off.");
      relaysToTurnOff.push(pin);
      break;
  }
}

