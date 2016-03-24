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
*/

#ifndef ADC_READ_H
#define ADC_READ_H

#include <stdint.h>

#define A2D_READ_PERIOD_MS (20)

void adc_read_init(uint8_t pin);
void adc_read_run(void);
uint16_t getA2DReadingForPin(uint8_t pin);

#endif
