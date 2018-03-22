#include "Arduino.h"
#include <stdio.h>

static uint32_t millisTimer = 0;

void setMillis(uint32_t val)
{
   millisTimer = val;
}

uint32_t millis(void)
{
   return millisTimer;
}

char *  itoa ( int value, char * str, int base )
{
   if (base != 10)
   {
      printf("TEST ERROR: Test mock does not exist for this number base: %d\r\n", base);
   }
   else
   {
      sprintf(str,"%d", value);
   }
      
   return str;
}

