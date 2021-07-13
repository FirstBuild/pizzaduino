/*
  coolingFan.cpp

  Cooling fan pin control

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

#include "Arduino.h"
#include "coolingFan.h"
#include "pinDefinitions.h"
#include "relayDriver.h"
#include "serialCommWrapper.h"

static CoolingFanSpeed resolveCurrentCoolingFanSpeed(void)
{
  static CoolingFanSpeed currentSpeed = coolingFanInvalid;

  if(getRelayState(COOLING_FAN_RELAY) == relayStateOff)
  {
    currentSpeed = coolingFanOff;
    Serial.println(F("DEBUG Current Fan Speed is Off"));
  }
  else if(getRelayState(COOLING_FAN_RELAY) == relayStateOn)
  {
    if(getRelayState(COOLING_FAN_LOW_SPEED) == relayStateOff)
    {
      currentSpeed = coolingFanHigh;
      Serial.println(F("DEBUG Current Fan Speed is High"));
    }
    else if(getRelayState(COOLING_FAN_LOW_SPEED) == relayStateOn)
    {
      currentSpeed = coolingFanLow;
      Serial.println(F("DEBUG Current Fan Speed is Low"));
    }
  }

  return currentSpeed;
}

void CoolingFanControl(CoolingFanSpeed speed)
{
  Serial.println(F("DEBUG Fan speed update requested"));
  static CoolingFanSpeed lastSpeed = resolveCurrentCoolingFanSpeed();

  if (lastSpeed != speed)
  {
    switch(speed) {
      default:
      case coolingFanOff:
        changeRelayState(COOLING_FAN_RELAY, relayStateOff);
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOff);
        Serial.println(F("DEBUG Fan speed updated to Off"));
        break;
      case coolingFanLow:
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOn);
        changeRelayState(COOLING_FAN_RELAY, relayStateOn);
        Serial.println(F("DEBUG Fan speed updated to Low"));
        break;
      case coolingFanHigh:
        changeRelayState(COOLING_FAN_RELAY, relayStateOn);
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOff);
        Serial.println(F("DEBUG Fan speed updated to High"));
        break;
    }
  }
}


