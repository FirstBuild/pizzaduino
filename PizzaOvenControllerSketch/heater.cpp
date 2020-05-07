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

#include <Arduino.h>
#include "heater.h"
#include "utility.h"

static void UpdateDutyCycle(Heater *pHeater)
{
  uint32_t currentTime = millis();
  uint8_t currentState = pHeater->relayState;
  uint8_t i;
  uint32_t Ton;
  uint32_t Toff;

  for(i=0; i<EVENT_MEMORY_COUNT; i++)
  {
    if (pHeater->event[i].state == RELAY_STATE_INVALID)
    {
      break;
    }
  }

  if (i<EVENT_MEMORY_COUNT)
  {
    pHeater->event[i].state = currentState;
    pHeater->event[i].time = currentTime;
  }
  else
  {
    // we have three points, calculate the duty cycle
    if (currentState == relayStateOn)
    {
      Toff = timeDiff(currentTime, pHeater->event[1].time);
      Ton = timeDiff(pHeater->event[1].time, pHeater->event[0].time);
    }
    else
    {
      Ton = timeDiff(currentTime, pHeater->event[1].time);
      Toff = timeDiff(pHeater->event[1].time, pHeater->event[0].time);      
    }
    pHeater->calculatedDutyCycle = 100.0 * Ton / (Ton + Toff);
    // push the stack
    pHeater->event[0] = pHeater->event[1];
    pHeater->event[1].time = currentTime;
    pHeater->event[1].state = currentState;
  }
}

void UpdateHeatControl(Heater *pHeater, uint16_t currentCounterTimer)
{
  double temperature;

  pHeater->relayState = relayStateOff;
  if ((pHeater->parameter.enabled == true) &&
      (currentCounterTimer >= pHeater->heaterCountsOn) &&
      (currentCounterTimer <= pHeater->heaterCountsOff))
  {
    temperature = pHeater->thermocouple;

    // If not in cool down and less than High Set Point Turn on Heater
    if (pHeater->heaterCoolDownState == false)
    {
      pHeater->relayState = relayStateOn;
      if (temperature >= (double)pHeater->parameter.tempSetPointHighOff)
      {
        pHeater->heaterCoolDownState = true;
        pHeater->relayState = relayStateOff;
        UpdateDutyCycle(pHeater);
      }
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else
    {
      pHeater->relayState = relayStateOff;
      if (temperature <= (double)pHeater->parameter.tempSetPointLowOn)
      {
        pHeater->heaterCoolDownState = false;
        pHeater->relayState = relayStateOn;
        UpdateDutyCycle(pHeater);
      }
    }
  }
  else  // Heater Disabled or Outside the percentage limits of the cycle
  {
    pHeater->relayState = relayStateOff;
  }
}

#ifdef USE_PID
void UpdateHeatControlWithPID(Heater *pHeater, uint16_t currentCounterTimer)
{
  pHeater->relayState = relayStateOff;
  if ((pHeater->parameter.enabled == true) &&
      (currentCounterTimer >= pHeater->heaterCountsOn) &&
      (currentCounterTimer <= pHeater->heaterCountsOff) &&
      (pHeater->parameter.onPercent < pHeater->parameter.offPercent))
  {
    pHeater->relayState = relayStateOn;
  }
  else  // Heater Disabled or Outside the percentage limits of the cycle
  {
    pHeater->relayState = relayStateOff;
  }
}
#endif

