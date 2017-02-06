// Arduino.h mock
//

#ifndef ARDUINO_H_MOCK
#define ARDUINO_H_MOCK

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

uint32_t millis(void);
void setMillis(uint32_t val);

#define strcpy_P(d, s) strcpy((char *)d, (const char*)s)
#define strlen_P(s) strlen((const char*)s)

char *  itoa ( int value, char * str, int base );

#endif // ARDUINO_H_MOCK
