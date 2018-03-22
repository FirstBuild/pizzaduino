#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ftoa.h"
#include <Arduino.h>

/*
char *ultoa(unsigned long val, char *buf, int radix) __attribute__((weak));

char *ultoa(unsigned long val, char *buf, int radix)
{
   (void)radix;
   sprintf(buf, "%lu", val);
   return buf;
}
*/

void ftoa(double val, uint8_t *pBuf, uint8_t precision)
{
   uint32_t intPart;
   uint8_t i;
   uint8_t pos=0;
   double rounding = 0.5;

   pBuf[0] = 0;

   if (val < 0)
   {
      pBuf[0] = '-';
      pBuf[1] = 0;
      val = -val;
   }
   for(i=0; i<precision; i++)
   {
      rounding /= 10.0;
   }
   val += rounding;

   intPart = (uint32_t)val;
   val = val - intPart;

   pos = (uint8_t)strlen((const char *)pBuf);
   ultoa(intPart, (char *)&pBuf[pos], 10);
   if (precision > 0)
   {
      strcat((char *)pBuf, ".");
      pos = (uint8_t)strlen((char *)pBuf);
      for(i=0; i<precision; i++)
      {
         val = val * 10;
         intPart = (uint32_t)val;
         val = val - intPart;
         pBuf[pos++] = (uint8_t)('0' + intPart);
      }
      pBuf[pos] = 0;
   }
}
