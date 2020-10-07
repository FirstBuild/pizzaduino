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

#ifdef CONFIGURATION_ORIGINAL
/*
 * Port pin interrupt stuff
 * PD5 - PCINT21 - Pin change interrupt 21 - Using PCINT2
 * PD4 - PCINT20 - Pin change interrupt 20 - Using PCINT2
 * PD3 - PCINT19 - Pin change interrupt 19 - Using INT1
 * PC5 - PCINT13 - Pin change interrupt 13 - Using PCINT1
*/

/*
 * Pin definitions
 * AC Input 1 - PD4 - Arduino pin D4 - Power button input
 * AC Input 2 - PD3 - Arduino pin D3 - Sail switch input
 * AC Input 3 - PC5 - Arduino pin A5 - TCO Input
 * AC voltage detect input - PD5/OC0B/T1 - Arduino pin D5
 */
#endif

#ifdef CONFIGURATION_LOW_COST
/*
 * Port pin interrupt stuff
 * PB3 - PCINT11 - Pin change interrupt 11 - Using PCI1
 * PB2 - PCINT10 - Pin change interrupt 10 - Using PCI1
 * PB0 - PCINT8  - Pin change interrupt  8 - Using PCI1
 * PD2 - PCINT26 - Pin change interrupt 26 - Using PCI3
*/

/*
 * Pin definitions
 * AC Input 1 - PB3 - Power button input
 * AC Input 2 - PB2 - Sail switch input
 * AC Input 3 - PB0 - TCO Input
 * AC voltage detect input - PD2
 */
#endif


#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "acInput.h"
#include "config.h"
#include "pinDefinitions.h"

#ifdef CONFIGURATION_ORIGINAL
#define AC_POWER_PIN (PIND & (1<<PD5))
#define POWER_BUTTON_PIN (PIND & (1<<PD4))
#endif

#ifdef CONFIGURATION_LOW_COST
typedef struct AcInputDefinition
{
  volatile uint32_t count;
  volatile uint8_t lastValue;
  volatile bool state;
  volatile byte *pInput;
  volatile byte mask;
} AcInputDefinition;

static AcInputDefinition voltageDetect;
static AcInputDefinition sailSwitch;
static AcInputDefinition tco;
static AcInputDefinition powerButton;
#endif

#ifdef CONFIGURATION_ORIGINAL
static volatile uint8_t acPowerPinLastValue = AC_POWER_PIN;
static volatile uint8_t powerButtonPinLastValue = POWER_BUTTON_PIN;

static bool powerButtonState = false;
static bool sailSwitchState = false;
static bool tcoState = false;
static bool acPowerPinState = false;
static volatile uint32_t powerButtonCount = 0;
static volatile uint32_t sailSwitchCount = 0;
static volatile uint32_t tcoCount = 0;
static volatile uint32_t acPowerPinCount = 0;
#endif

static void configureInputPin(byte pin, AcInputDefinition *pDef)
{
  pDef->pInput = portInputRegister(digitalPinToPort(pin));
  pDef->mask = digitalPinToBitMask(pin);
  pinMode(pin, INPUT);
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

#ifdef CONFIGURATION_ORIGINAL
static void pciSetup(byte pin)
{
  pinMode(pin, INPUT);
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT2_vect)
{
  if(powerButtonPinLastValue != POWER_BUTTON_PIN)
  {
    powerButtonCount++;
    powerButtonPinLastValue = POWER_BUTTON_PIN;
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
#endif

#ifdef CONFIGURATION_LOW_COST
static void UpdateInput(AcInputDefinition *pInput)
{
  byte current = *pInput->pInput & pInput->mask;

  if(pInput->lastValue != current)
  {
    pInput->count++;
    pInput->lastValue = current;
  }
}

ISR (PCINT1_vect)
{
  UpdateInput(&sailSwitch);
  UpdateInput(&powerButton);
  UpdateInput(&tco);
}

ISR (PCINT3_vect)
{
  UpdateInput(&voltageDetect);
}
#endif

void acInputsInit(void)
{
  #ifdef CONFIGURATION_ORIGINAL
  pinMode(A5, INPUT);
  // Arduino pin 4 is tied to 328P port pin PD4, use the pin change interrupt for input
  pciSetup(4);
  pinMode(5, INPUT);
  pciSetup(5);
  // Arduino pin 3 is tied to 328P port pin PD3, use the INT1 interrupt for input
  attachInterrupt(digitalPinToInterrupt(3), myINT1_vect, CHANGE);
  // Arduino pin A5 is tied to 328P port pin PC5, use the pin change interrupt for input
  pciSetup(A5);
  #endif
  #ifdef CONFIGURATION_LOW_COST
  configureInputPin(VOLTAGE_DETECT_INPUT, &voltageDetect);
  configureInputPin(DLB_STATUS_AC_INPUT, &sailSwitch);
  configureInputPin(POWER_SWITCH_AC_INPUT, &powerButton);
  configureInputPin(TCO_AC_INPUT, &tco);
  #endif
}

static void UpdateState(AcInputDefinition *pInput)
{
  pInput->state = (pInput->count > 5) && (voltageDetect.state);
  pInput->count = 0;
}

void acInputsRun(void)
{
  uint32_t newTime = millis();
  static uint32_t oldAcTime = 0;
  static uint32_t oldTime = 0;

  if ((newTime - oldAcTime) >= 60)
  {
    oldAcTime = newTime;
    noInterrupts();
    #ifdef CONFIGURATION_ORIGINAL
    acPowerPinState = acPowerPinCount > 4;
    acPowerPinCount = 0;
    #endif
    #ifdef CONFIGURATION_LOW_COST
    voltageDetect.state = voltageDetect.count > 4;
    voltageDetect.count = 0;
    #endif
    interrupts();
  }

  if ((newTime - oldTime) >= 120)
  {
    oldTime = newTime;
    noInterrupts();
    #ifdef CONFIGURATION_ORIGINAL
    powerButtonState = (powerButtonCount > 5) && acPowerPinState;
    sailSwitchState = (sailSwitchCount > 5) && acPowerPinState;
    tcoState = (tcoCount > 5) && acPowerPinState;
    powerButtonCount = 0;
    sailSwitchCount = 0;
    tcoCount = 0;
    #endif
    #ifdef CONFIGURATION_LOW_COST
    UpdateState(&tco);
    UpdateState(&sailSwitch);
    UpdateState(&powerButton);
    #endif
    interrupts();
  }
}

bool powerButtonIsOn(void)
{
  #ifdef CONFIGURATION_LOW_COST
  return powerButton.state;
  #else
  return powerButtonState;
  #endif  
}

bool sailSwitchIsOn(void)
{
  #ifdef CONFIGURATION_LOW_COST
  return sailSwitch.state;
  #else
  return sailSwitchState;
  #endif  
}

bool tcoInputIsOn(void)
{
  #ifdef CONFIGURATION_LOW_COST
  return tco.state;
  #else
  return tcoState;
  #endif  
}

bool acPowerIsPresent(void)
{
  #ifdef CONFIGURATION_LOW_COST
  return voltageDetect.state;
  #else
  return acPowerPinState;
  #endif  
}
