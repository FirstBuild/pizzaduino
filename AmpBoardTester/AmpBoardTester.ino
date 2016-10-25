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

// Pizza Oven Project

/*
#include "TimerOne.h"
#include <avr/pgmspace.h>
#include "thermocouple.h"
#include "acInput.h"
#include "pinDefinitions.h"
#include "relayBoost.h"
#include "relayDriver.h"
#include "pizzaMemory.h"
#include "PID_v1.h"
#include <limits.h>
#include <stdint.h>
#include "DigitalInputDebounced.h"
#include "adcRead.h"
#include "projectTypeDefs.h"
#include "watchdogRelay.h"
#include "coolingFan.h"
#include "utility.h"
#include "config.h"
#include "heater.h"
#include "cookingStateMachine.h"
#include "tcoAndFanCheck.h"
*/

#include <Wire.h>
#include <SeeedGrayOLED.h>
#include "DigitalInputDebounced.h"

#ifndef UINT32_MAX
#define UINT32_MAX (0xffffffff)
#endif

//------------------------------------------
// Macros
//------------------------------------------
#define FIRMWARE_MAJOR_VERSION   1
#define FIRMWARE_MINOR_VERSION   0
#define FIRMWARE_BUILD_VERSION   1
#define RESET_LINE 0
#define RESET_TEXT "Reset : "

//------------------------------------------
// Macros for Constants and Pin Definitions
//------------------------------------------
#define RESET_INPUT 4

//------------------------------------------
// Global Definitions
//------------------------------------------
DigitalInputDebounced resetLineInput(RESET_INPUT, false, false);

//------------------------------------------
// Prototypes
//------------------------------------------

void outputResetState()
{
  Serial.print(F("Reset "));
  Serial.println(resetLineInput.IsActive() ? 1 : 0);
}


//------------------------------------------
// Setup Routine
//------------------------------------------
void setup()
{
  Serial.begin(19200);
  Serial.println(F("DEBUG Starting Amp Board Tester..."));
  
  Serial.println(F("DEBUG Initializing LCD."));

  pinMode(RESET_INPUT, INPUT_PULLUP);
  Wire.begin();
  SeeedGrayOled.init();             //initialize SEEED OLED display
  SeeedGrayOled.clearDisplay();     //Clear Display.
  SeeedGrayOled.setNormalDisplay(); //Set Normal Display Mode
  SeeedGrayOled.setVerticalMode();  // Set to vertical mode for displaying text
  SeeedGrayOled.setGrayLevel(15); //Set Grayscale level. Any number between 0 - 15.
  SeeedGrayOled.setTextXY(RESET_LINE, 0);
  SeeedGrayOled.putString(RESET_TEXT);

  Serial.println(F("DEBUG Initialization complete."));
  
}


#define RECEVIED_COMMAND_BUFFER_LENGTH (16)

/*
   Serial commands prefixes:

   f - set off percent
   g - set pid gains
   G - get the pid gains
   l - set lower temp limit
   n - set on percent
   p - print heater parameters
   q - stop oven
   s - start oven
   t - set time base
   v - query firmware version
   u - set upper temp limit
   r - get state of reset line

*/
void handleIncomingCommands(void)
{
  static uint8_t receivedCommandBuffer[RECEVIED_COMMAND_BUFFER_LENGTH];
  static uint8_t receivedCommandBufferIndex = 0;
  uint8_t lastByteReceived;

  if (Serial.available() > 0)
  {
    lastByteReceived = Serial.read();
    receivedCommandBuffer[receivedCommandBufferIndex++] = lastByteReceived;
    //    Serial.print("DEBUG Char rcvd: [");
    //    Serial.print(lastByteReceived);
    //    Serial.println("]");
    if (receivedCommandBufferIndex >= RECEVIED_COMMAND_BUFFER_LENGTH)
    {
      Serial.println(F("DEBUG command input buffer exceeded."));
      receivedCommandBufferIndex = 0;
    }
    else if (receivedCommandBufferIndex > 0)
    {
      // attempt to parse the command
      switch (receivedCommandBuffer[0])
      {
        case 'r':  // Start Pizza Oven Cycle
          outputResetState();
          receivedCommandBufferIndex = 0;
          break;

        case 'v': // query protocol version
          Serial.print(F("V "));
          Serial.print(FIRMWARE_MAJOR_VERSION);
          Serial.print(F("."));
          Serial.print(FIRMWARE_MINOR_VERSION);
          Serial.print(F(" bugfix "));
          Serial.println(FIRMWARE_BUILD_VERSION);
          receivedCommandBufferIndex = 0;
          break;

        case 10:
        case 13:
          // ignore cr/lf as a command
          receivedCommandBufferIndex = 0;
          break;

        default:
          Serial.print(F("DEBUG unknown command received: "));
          Serial.println(receivedCommandBuffer[0]);
          receivedCommandBufferIndex = 0;
          break;
      }
    }
  }
}

void updateDcInputs(void)
{
  static int oldMillis = 0;
  int newMillis = millis();

  if ((newMillis - oldMillis) >= 10)
  {
    oldMillis = newMillis;

    resetLineInput.UpdateInput();
  }
}

//------------------------------------------
// Main Loop
//------------------------------------------
void loop()
{
  bool resetLineOld = resetLineInput.IsActive();
  bool resetLineNew;

  handleIncomingCommands();
  updateDcInputs();

  resetLineNew = resetLineInput.IsActive();

  if (resetLineOld != resetLineNew)
  {
    SeeedGrayOled.setTextXY(RESET_LINE, 8);
    SeeedGrayOled.putChar(resetLineNew ? '1' : '0');
  }
}

