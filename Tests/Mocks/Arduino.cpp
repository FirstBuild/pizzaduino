#include "Arduino.h"

static uint32_t millisTimer = 0;

void setMillis(uint32_t val)
{
   millisTimer = val;
}

uint32_t millis(void)
{
   return millisTimer;
}
