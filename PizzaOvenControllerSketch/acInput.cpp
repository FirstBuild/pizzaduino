/*
  ac_input.cpp

  Read the AC inputs

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

/*
 * Port pin interrupt stuff
 * PD5 - PCINT21 - Pin change interrupt 21 - Using PCINT2
 * PD4 - PCINT20 - Pin change interrupt 20 - Using PCINT2
 * PD3 - PCINT19 - Pin change interrupt 19 - Using INT1
 * PC5 - PCINT13 - Pin change interrupt 13 - Using PCINT1
*/

/*
 * Pin definitions
 * AC Input 1 - PD4 - Arduino pin D4
 * AC Input 2 - PD3 - Arduino pin D3
 * AC Input 3 - PC5 - Arduino pin A5
 * AC voltage detect input - PD5/OC0B/T1 - Arduino pin D5
 */

#include "acInput.h"
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define AC_POWER_PIN (PIND & (1<<PD5))
#define AC_INPUT_1_PIN (PIND & (1<<PD4))

static volatile uint8_t acPowerPinLastValue = AC_POWER_PIN;
static volatile uint8_t acInput1PinLastValue = AC_INPUT_1_PIN;

static uint32_t oldTime = 0;
static bool powerButtonState = false;
static bool sailSwitchState = false;
static bool tcoState = false;
static bool acPowerPinState = false;
static volatile uint32_t powerButtonCount = 0;
static volatile uint32_t sailSwitchCount = 0;
static volatile uint32_t tcoCount = 0;
static volatile uint32_t acPowerPinCount = 0;

static void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT2_vect)
{
  if(acInput1PinLastValue != AC_INPUT_1_PIN)
  {
    powerButtonCount++;
    acInput1PinLastValue = AC_INPUT_1_PIN;
  }
  if(acPowerPinLastValue != AC_POWER_PIN)
  {
    acPowerPinCount++;
    acPowerPinLastValue = AC_POWER_PIN;
  }
}

ISR (PCINT1_vect)
{
  tcoCount++;
}

void myINT1_vect(void)
{
  sailSwitchCount++;
}

void acInputsInit(void)
{
  pinMode(A5, INPUT);
  // Arduino pin 4 is tied to 328P port pin PD4, use the pin change interrupt for input
  pciSetup(4);
  pinMode(5, INPUT);
  pciSetup(5);
  // Arduino pin 3 is tied to 328P port pin PD3, use the INT1 interrupt for input
  attachInterrupt(digitalPinToInterrupt(3), myINT1_vect, CHANGE);
  // Arduino pin A5 is tied to 328P port pin PC5, use the pin change interrupt for input
  pciSetup(A5);
}

void acInputsRun(void)
{
  uint32_t newTime = millis();

  if ((newTime - oldTime) >= 100)
  {
    oldTime = newTime;
    noInterrupts();
    powerButtonState = powerButtonCount > 5;
    sailSwitchState = sailSwitchCount > 5;
    tcoState = tcoCount > 5;
    acPowerPinState = acPowerPinCount > 5;
    powerButtonCount = 0;
    sailSwitchCount = 0;
    tcoCount = 0;
    acPowerPinCount = 0;
    interrupts();
  }
}

bool powerButtonIsOn(void)
{
  return powerButtonState;
}

bool sailSwitchIsOn(void)
{
  return sailSwitchState;
}

bool tcoInputIsOn(void)
{
  return tcoState;
}

bool acPowerIsPresent(void)
{
  return acPowerPinState;
}

