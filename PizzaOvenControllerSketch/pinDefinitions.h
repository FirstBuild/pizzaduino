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

#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H
#include <Arduino.h>


// Pin Definitions
#define COOLING_FAN_RELAY         (12) // cooling fan relay
#define COOLING_FAN_HIGH_SPEED    ( 2) // cooling fan high speed relay PD2

#define HEATER_UPPER_FRONT_DLB     ( 8) //Relay that provides L1 to the triac, must be ON for heat
#define HEATER_UPPER_REAR_DLB      ( 9) //Relay that provides L1 to the triac, must be ON for heat
#define HEATER_TRIAC_UPPER_FRONT  (A0) // triac output
#define HEATER_TRIAC_UPPER_REAR   (A1) // triac output

// Lower heater relays - These are cascaded.  Only turn on one at a time.
#define HEATER_RELAY_LOWER_FRONT  (10) // relay output
#define HEATER_RELAY_LOWER_REAR   (11) // relay output

#define BOOST_ENABLE               ( 7) //Enable 15V pull in voltage for relays
#define RELAY_WATCHDOG             ( 6) //Signal must toggle at a rate of X Hz in order to enable relays
#define VOLTAGE_DETECT             ( 5) //Used to determine supply voltage as 208VAC or 240VAC
#define POWER_SWITCH_AC_INPUT      ( 3) //AC Input that indicates the state of the power switch
#define DLB_STATUS_AC_INPUT        ( 4) //AC Input that indicates the status of the L2 panel mount DLB relays
#define DOOR_STATUS_INPUT          (13)
#define TCO_AC_INPUT               (A5) //AC Input for the TCO, also port PC5

// Temperature inputs
#define ANALOG_THERMO_UPPER_FRONT  (A7)
#define ANALOG_THERMO_UPPER_REAR   (A6)
#define ANALOG_THERMO_LOWER_FRONT  (A2)
#define ANALOG_THERMO_LOWER_REAR   (A3)
#define ANALOG_THERMO_FAN          (A4)

#endif

