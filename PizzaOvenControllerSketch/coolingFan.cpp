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

void CoolingFanControl(CoolingFanSpeed speed)
{
  static CoolingFanSpeed lastSpeed = coolingFanInvalid;

  if (lastSpeed != speed)
  {
    switch(speed) {
    Serial.println(F("DEBUG Changing the state of the cooling fan."));
      case coolingFanOff:
        changeRelayState(COOLING_FAN_RELAY, relayStateOff);
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOff);
        break;
      case coolingFanLow:
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOn);
        changeRelayState(COOLING_FAN_RELAY, relayStateOn);
        break;
      case coolingFanHigh:
        changeRelayState(COOLING_FAN_RELAY, relayStateOn);
        changeRelayState(COOLING_FAN_LOW_SPEED, relayStateOff);
      break;
    }
  }
}


