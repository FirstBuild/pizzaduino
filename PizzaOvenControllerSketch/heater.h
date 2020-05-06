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

#ifndef HEATER_H_
#define HEATER_H_

#include "config.h"
#include "projectTypeDefs.h"
#include "filter.h"

struct HeaterParameters
{
  bool enabled;
  union {
    struct {
      uint16_t tempSetPointLowOn;   // In integer degrees C
      uint16_t tempSetPointHighOff; // "      "
      uint16_t onPercent;       // Time when a heater turns on in percent
      uint16_t offPercent;      // Time when a heater turns off in percent
    };
    uint16_t parameterArray[4];
  };
};

#define RELAY_STATE_INVALID 127

typedef struct RelayTimeStamp {
  uint32_t time;
  uint8_t state;
}

struct Heater
{
  HeaterParameters parameter;
  uint32_t heaterCountsOn;
  uint32_t heaterCountsOff;
  RelayState relayState;
  bool heaterCoolDownState;
  double thermocouple;
  FilterBeLp2 tcFilter;
  RelayTimeStamp event[3];
  float calculatedDutyCycle;
};


void UpdateHeatControl(Heater *pHeater, uint16_t currentCounterTimer);
#ifdef USE_PID
void UpdateHeatControlWithPID(Heater *pHeater, uint16_t currentCounterTimer);
#endif

#endif /* HEATER_H_ */
