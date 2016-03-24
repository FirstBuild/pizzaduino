/*
   thermocouple.h

   Read the thermocouple
*/
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include <stdint.h>
#include "Arduino.h"

// Thermocouple Definitions
#define ANALOG_REFERENCE_VOLTAGE    ((double)5.0)

#define ANALOG_THERMO_UPPER_FRONT    (A2)
#define ANALOG_THERMO_UPPER_REAR    (A3)
#define ANALOG_THERMO_LOWER_FRONT   (A6)
#define ANALOG_THERMO_LOWER_REAR    (A7)
#define ANALOG_THERMO_FAN       (A4)

float readAD8495KTC(uint8_t pin);

#endif
