#ifndef FTOA_H
#define FTOA_H

#include <stdint.h>

void ftoa(double val, uint8_t *pBuf, uint8_t precision);

char *ultoa(unsigned long val, char *buf, int radix) __attribute__((weak));

#endif // FTOA_H
