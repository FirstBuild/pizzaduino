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

#include <EEPROM.h>
#include "pizzaMemory.h"
#include "crc.h"

static uint16_t calculateCrcOfEeprom(void)
{
  uint16_t crc = crc_init();
  uint16_t i;
  uint8_t val;

  for (i=0; i<EEPROM.length()-2; i++)
  {
    val = EEPROM.read(i);
    crc = crc_update(crc, &val, 1);
  }

  return crc_finalize(crc);
}

static void updateCrcOfEeprom(void)
{
  uint16_t crc = calculateCrcOfEeprom();
  EEPROM.put(EEPROM.length()-2, crc);

}

pizzaMemoryReturnTypes pizzaMemoryInit(void)
{
  uint16_t crcCalculated;
  uint16_t crcStored;
  pizzaMemoryReturnTypes retval = pizzamemoryWasInitialized;

  crcCalculated = calculateCrcOfEeprom();
  EEPROM.get(EEPROM.length()-2, crcStored);

  if (crcCalculated != crcStored)
  {
    EEPROM.put(EEPROM.length()-2, crcCalculated);
    retval = pizzaMemoryWasEmpty;
  }

  return retval;
}

pizzaMemoryReturnTypes pizzaMemoryRead(uint8_t *pBuf, uint16_t addr, uint16_t size)
{
  uint16_t i;

  for(i=0; i<size; i++)
  {
    pBuf[i] = EEPROM.read(addr + i);
  }

  return pizzamemorySuccess;
}

pizzaMemoryReturnTypes pizzaMemoryWrite(uint8_t *pBuf, uint16_t addr, uint16_t size)
{
  uint16_t i;

  for(i=0; i<size; i++)
  {
    EEPROM.update(addr + i, pBuf[i]);
  }
  updateCrcOfEeprom();

  return pizzamemorySuccess;
}


