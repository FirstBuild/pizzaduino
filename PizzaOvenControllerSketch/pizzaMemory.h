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

#ifndef PIZZA_MEMORY_H
#define PIZZA_MEMORY_H

#include <Arduino.h>
#include "projectTypeDefs.h"
#include "heater.h"

typedef struct MemoryStore
{
  HeaterParameters upperFrontHeaterParameters;
  HeaterParameters upperRearHeaterParameters;
  HeaterParameters lowerFrontHeaterParameters;
  HeaterParameters lowerRearHeaterParameters;
  uint16_t triacPeriodSeconds;
  uint16_t relayPeriodSeconds;
  PidParameters upperFrontPidParameters;
  PidParameters upperRearPidParameters;
  uint16_t doorDeployCount;
  bool doorHasDeployed;
  uint16_t ufTcLimitExceededCount;
  uint16_t urTcLimitExceededCount;
  uint16_t lfTcLimitExceededCount;
  uint16_t lrTcLimitExceededCount;
  uint8_t preheatFanSetting;
  uint8_t cookingFanSetting;
} MemoryStore;

enum pizzaMemoryReturnTypes
{
  pizzamemorySuccess,
  pizzamemoryFail,
  pizzaMemoryWasEmpty,
  pizzamemoryWasInitialized
};

pizzaMemoryReturnTypes pizzaMemoryInit(void);
pizzaMemoryReturnTypes pizzaMemoryRead(uint8_t *pBuf, uint16_t addr, uint16_t size);
pizzaMemoryReturnTypes pizzaMemoryWrite(uint8_t *pBuf, uint16_t addr, uint16_t size);

#endif
