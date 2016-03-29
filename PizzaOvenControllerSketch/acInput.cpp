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

#include "acInput.h"
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

static uint32_t oldTime = 0;
static bool acInputOneState = false;
static bool acInputTwoState = false;
static volatile uint32_t acInputOneCount = 0;
static volatile uint32_t acInputTwoCount = 0;

static void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT2_vect)
{
  acInputOneCount++;
}

void myINT1_vect(void)
{
  acInputTwoCount++;
}

void acInputsInit(void)
{
  // Arduino pin 4 is tied to 328P port pin PD4, use the pin change interrupt for input
  pciSetup(4);
  // Arduino pin 3 is tied to 328P port pin PD3, use the INT1 interrupt for input
  attachInterrupt(digitalPinToInterrupt(3), myINT1_vect, CHANGE);
}

void acInputsRun(void)
{
  uint32_t newTime = millis();

  if ((newTime - oldTime) >= 100)
  {
    oldTime = newTime;
    noInterrupts();
    acInputOneState = acInputOneCount > 5;
    acInputTwoState = acInputTwoCount > 5;
    acInputOneCount = 0;
    acInputTwoCount = 0;
    interrupts();
  }
}

bool powerButtonIsOn(void)
{
  return acInputOneState;
}

bool l2DlbIsOn(void)
{
  return acInputTwoState;
}

