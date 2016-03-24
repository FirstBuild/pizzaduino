/*
   adc_read.h

   Manage reading ADC channels without blocking.

   A2D_READ_PERIOD_MS is the reading period.  Every A2D pin that is initialized
   will be read once every A2D_READ_PERIOD_MS.  The pins are read in port-order,
   meaing that A0 is read first, followed by A1.  If a pin is not initialized via
   a call to adc_read_init, that pin will not be read.

   Call adc_read_init with the analog port pin that you want to periodically read from.
   Call adc_read_run in the loop function to cause the readings to occur.
   Call getA2DReadingForPin with the analog port pin that you want to get the value of.

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

#ifndef ADC_READ_H
#define ADC_READ_H

#include <stdint.h>

#define A2D_READ_PERIOD_MS (20)

void adcReadInit(uint8_t pin);
void adcReadRun(void);
uint16_t getA2DReadingForPin(uint8_t pin);

#endif
