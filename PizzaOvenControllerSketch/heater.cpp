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

#include "heater.h"

static void UpdateDutyCycle(Heater *pHeater)
{
  

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

