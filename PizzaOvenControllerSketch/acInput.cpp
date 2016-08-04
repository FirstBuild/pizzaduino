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
 * PD4 - PCINT20 - Pin change interrupt 20 - Using PCINT2
 * PD3 - PCINT19 - Pin change interrupt 19 - Using INT1
 * PC5 - PCINT13 - Pin change interrupt 13 - Using PCINT1
*/

/*
 * Pin definitions
 * AC Input 1 - PD4 - Arduino pin D4
 * AC Input 2 - PD3 - Arduino pin D3
 * AC Input 3 - PC5 - Arduino pin A5
 */

#include "acInput.h"
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

static uint32_t oldTime = 0;
static bool acInputOneState = false;
static bool acInputTwoState = false;
static bool acInputThreeState = false;
static volatile uint32_t acInputOneCount = 0;
static volatile uint32_t acInputTwoCount = 0;
static volatile uint32_t acInputThreeCount = 0;

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

ISR (PCINT1_vect)
{
  acInputThreeCount++;
}

void myINT1_vect(void)
{
  acInputTwoCount++;
}

void acInputsInit(void)
{
  pinMode(A5, INPUT);
  // Arduino pin 4 is tied to 328P port pin PD4, use the pin change interrupt for input
  pciSetup(4);
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
    acInputOneState = acInputOneCount > 5;
    acInputTwoState = acInputTwoCount > 5;
    acInputThreeState = acInputThreeCount > 5;
    acInputOneCount = 0;
    acInputTwoCount = 0;
    acInputThreeCount = 0;
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

bool TcoInputIsOn(void)
{
  return acInputThreeState;
}

