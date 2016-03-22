/*
   ac_input.cpp

   Read the AC inputs
*/

#include "ac_input.h"
#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>

static uint32_t oldTime=0;
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

void setupAcInputs(void)
{
  // Arduino pin 4 is tied to 328P port pin PD4, use the pin change interrupt for input
  pciSetup(4);
  // Arduino pin 3 is tied to 328P port pin PD3, use the INT1 interrupt for input
  attachInterrupt(digitalPinToInterrupt(3), myINT1_vect, CHANGE);
}

void runAcInputs(void)
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

bool getAcInputOne(void)
{
  return acInputOneState;
}

bool getAcInputTwo(void)
{
  return acInputTwoState;
}

