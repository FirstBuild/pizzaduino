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

#include <Arduino.h>
#include "relayDriver.h"
#include "relayBoost.h"
#include <stdint.h>
#include "CircularBuffer.h"

#define NUMBER_OF_RELAYS 8

static CircularBuffer<uint8_t, NUMBER_OF_RELAYS> relaysToTurnOn;
static CircularBuffer<uint8_t, NUMBER_OF_RELAYS> relaysToTurnOff;

typedef struct Relay
{
  uint8_t pin;
  RelayState desiredState;
  bool actionCompleted;
} Relay;

static Relay relay[NUMBER_OF_RELAYS];
static uint8_t lastRelay = 0;

#define RELAY_OPERATION_DELAY_MS 211

void initializeRelayPin(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  relayDriverInit(pin, relayStateOff);
}

static uint8_t getIndexOfRelay(uint8_t pin)
{
  uint8_t index = 0;
  uint8_t i;

  for (i = 0; i < lastRelay; i++)
  {
    if (relay[i].pin == pin)
    {
      index = i;
      break;
    }
  }

  return index;
}

static void setRelayActionComplete(uint8_t pin)
{
  uint8_t index = getIndexOfRelay(pin);
  relay[index].actionCompleted = true;
}

void relayDriverInit(uint8_t pin, RelayState relayState)
{
  relay[lastRelay].pin = pin;
  relay[lastRelay].desiredState = relayState;
  relay[lastRelay].actionCompleted = true;

  lastRelay++;
}

void relayDriverRun(void)
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  uint8_t pin;

  if (newTime >= oldTime)
  {
    if ((newTime - oldTime) >= RELAY_OPERATION_DELAY_MS)
    {
      if (relaysToTurnOff.remain() > 0)
      {
        pin = relaysToTurnOff.pop();
        //        Serial.print("Turning relay ");
        //        Serial.print(pin);
        //        Serial.println(" off.");
        digitalWrite(pin, LOW);
        setRelayActionComplete(pin);
        oldTime = newTime;
      }
      else if (relaysToTurnOn.remain() > 0)
      {
        boostEnable(relayBoostOn);
        pin = relaysToTurnOn.pop();
        //        Serial.print("Turning relay ");
        //        Serial.print(pin);
        //        Serial.println(" on.");
        digitalWrite(pin, HIGH);
        setRelayActionComplete(pin);
        oldTime = newTime;
      }
    }
  }
  else
  {
    oldTime = newTime;
  }
}

static void addRelayToChangeList(uint8_t pin, RelayState desiredState)
{
  switch (desiredState)
  {
    case relayStateOn:
      //      Serial.print("Staging relay ");
      //      Serial.print(pin);
      //      Serial.println(" on.");
      relaysToTurnOn.push(pin);
      break;

    case relayStateOff:
      //      Serial.print("Staging relay ");
      //      Serial.print(pin);
      //      Serial.println(" off.");
      relaysToTurnOff.push(pin);
      break;
  }
}

#ifdef KILL
extern uint16_t saveTimer1Counter;
static void printRelayDesiredStates(void)
{
  uint32_t t = millis();
  uint8_t i;
  static bool headerNotPrinted = true;

  if (headerNotPrinted)
  {
    headerNotPrinted = false;
    Serial.println("Relay Pins");
    Serial.print("Millis,TimerCounter,");
    Serial.print(relay[0].pin);
    for (i = 1; i < lastRelay; i++)
    {
      Serial.print(",");
      Serial.print(relay[i].pin);
    }
    Serial.println("");
  }

  Serial.print(t);
  Serial.print(",");
  Serial.print(saveTimer1Counter);
  Serial.print(",");
  Serial.print(relay[0].desiredState == relayStateOn ? 1 : 0);
  for (i = 1; i < lastRelay; i++)
  {
    Serial.print(",");
    Serial.print(relay[i].desiredState == relayStateOn ? 1 : 0);
  }
  Serial.println("");
}
#endif

void changeRelayState(uint8_t pin, RelayState desiredState)
{
  uint8_t index = getIndexOfRelay(pin);
  RelayState currentState = relayStateOff;

  relay[index].desiredState = desiredState;
  if (digitalRead(relay[index].pin) == HIGH)
  {
    currentState = relayStateOn;
  }
  if ((currentState != desiredState) && (relay[index].actionCompleted == true))
  {
//    printRelayDesiredStates();
    relay[index].actionCompleted = false;
    addRelayToChangeList(pin, desiredState);
  }
}


