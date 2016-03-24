/*
   adc_read.c

   Manage reading ADC channels without blocking.
*/

#include <Arduino.h>
#include <wiring_private.h>
#include "adc_read.h"

#define MAX_CHANNELS 8

static uint16_t adcValue[MAX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
static bool pinToRead[MAX_CHANNELS] = {0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t a2dPin = 0;

enum {
  a2d_idle,
  a2d_StartConversion,
  a2d_AwaitConversionComplete
};

static uint8_t a2dState = a2d_idle;

void adc_read_init(uint8_t pin)
{
  pin = pin - 14;
  Serial.print("Setting pin ");
  Serial.print(pin);
  Serial.println(" for A2D scanning.");
  pinToRead[pin] = true;
}

bool itIsTimeToStartScanning(void)
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  bool retval = false;

  if ((newTime - oldTime) > A2D_READ_PERIOD_MS)
  {
    oldTime = newTime;
    retval = true;
  }

  return retval;
}

void adc_read_run(void)
{
  uint8_t low, high;
  
  switch (a2dState)
  {
    case a2d_idle:
      if (itIsTimeToStartScanning())
      {
        a2dPin = 0;
        a2dState = a2d_StartConversion;
      }
      break;

    case a2d_StartConversion:
      if (pinToRead[a2dPin])
      {
        ADMUX = (DEFAULT << 6) | (a2dPin & 0x07);
        // start the conversion
        sbi(ADCSRA, ADSC);
        a2dState = a2d_AwaitConversionComplete;
      }
      else
      {
        a2dPin++;
      }
      break;

    case a2d_AwaitConversionComplete:
      if (bit_is_clear(ADCSRA, ADSC))
      {
        low  = ADCL;
        high = ADCH;
        adcValue[a2dPin] = (high << 8) | low;
        a2dPin++;
        a2dState = a2d_StartConversion;
      }
      break;
  }

  if (a2dPin >= MAX_CHANNELS)
  {
    a2dState = a2d_idle;
  }

}

uint16_t getA2DReadingForPin(uint8_t pin)
{
  return adcValue[pin-14];
}

