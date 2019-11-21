/*
  Copyright (c) 2015 FirstBuild

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.f

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

// Pizza Oven Project
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
#include <avr/wdt.h>
#include "ftoa.h"
#include "tcLimitCheck.h"
#include "globals.h"
#include "serialCommWrapper.h"

static TcoAndFan tcoAndFan;
static TcLimitCheck ufTcLimit(1400, 30000);
static TcLimitCheck urTcLimit(1400, 30000);
static TcLimitCheck lfTcLimit(1000, 5000);
static TcLimitCheck lrTcLimit(1000, 5000);

#ifndef UINT32_MAX
#define UINT32_MAX (0xffffffff)
#endif

//------------------------------------------
// Macros
//------------------------------------------
#define FIRMWARE_MAJOR_VERSION   1
#define FIRMWARE_MINOR_VERSION   2
#define FIRMWARE_BUILD_VERSION   2

const char versionString[] = {'V', ' ', '0' + FIRMWARE_MAJOR_VERSION, '.', '0' + FIRMWARE_MINOR_VERSION, ' ', 'b', 'u', 'g', 'f', 'i', 'x', ' ', '0' + FIRMWARE_BUILD_VERSION, 0};

//------------------------------------------
// Macros for Constants and Pin Definitions
//------------------------------------------


// Timer1 Used to keep track of heat control cycles
#define TIMER1_PERIOD_MICRO_SEC			(1000) 	// timer1 1 mSec interval 
#define TIMER1_PERIOD_CLOCK_FACTOR	(1) 		// Clock multiplier for Timer1
#define MILLISECONDS_PER_SECOND		  (1000)  // Count down for a period of 1 second
#define TIMER1_OUTPUT_TEMP_PERIODIC (1000)  // Multiple of TIMER1_PERIOD_MICRO_SEC to output periodic temp

//------------------------------------------
// Global Definitions
//------------------------------------------

// Timer one is running to control heat percentage and start
//  Re-synchronized on the start of heating
volatile uint32_t triacTimeBase = 0;
volatile uint32_t relayTimeBase = 0;
uint16_t countOutputTempPeriodic = 0;

// Heater cycle time in multiples of 1 second
uint16_t triacPeriodSeconds = 4;
uint16_t relayPeriodSeconds = 60;
uint16_t doorDeployCount = 0;
bool doorHasDeployed = false;
DigitalInputDebounced doorInput(DOOR_STATUS_INPUT, false, true);

bool ufTcTempLimitFailed = false;
uint16_t ufTcLimitExceededCount = 0;
bool urTcTempLimitFailed = false;
uint16_t urTcLimitExceededCount = 0;
bool lfTcTempLimitFailed = false;
uint16_t lfTcLimitExceededCount = 0;
bool lrTcTempLimitFailed = false;
uint16_t lrTcLimitExceededCount = 0;

bool upperTempDiffExceeded = false;
uint8_t upperTempDiffExceededCount = 0;
bool lowerTempDiffExceeded = false;
uint8_t lowerTempDiffExceededCount = 0;

Heater upperFrontHeater = {{true, 1200, 1300,  0, 100}, 0, 0, relayStateOff, false, 0.0};
Heater upperRearHeater  = {{true, 1100, 1200,  0, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerFrontHeater = {{true,  600,  650, 50, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerRearHeater  = {{true,  575,  625,  0,  49}, 0, 0, relayStateOff, false, 0};

// convenience array, could go into flash
Heater *aHeaters[4] =
{
  &upperFrontHeater,
  &upperRearHeater,
  &lowerFrontHeater,
  &lowerRearHeater
};

const uint16_t maxTempSetting[] = {MAX_UPPER_TEMP, MAX_UPPER_TEMP, MAX_LOWER_TEMP, MAX_LOWER_TEMP};

#ifdef USE_PID
// PID stuff
#define MAX_PID_OUTPUT 100
//PidIo upperFrontPidIo = {1000, 47, {0.175, 0.001110517, 0.4}};
//PidIo upperRearPidIo  = {1000, 47, {0.175, 0.0010935,   0.4}};
//PidIo upperFrontPidIo = {1000, 47, {0.175, 0.0005, 0.4}};
//PidIo upperRearPidIo  = {1000, 47, {0.175, 0.0005, 0.4}};
//PidIo upperFrontPidIo = {1000, 47, {0.25, 0.0005, 0.4}};
//PidIo upperRearPidIo  = {1000, 47, {0.25, 0.0005, 0.4}};
PidIo upperFrontPidIo = {1000, 47, {0.9, 0.00113, 0.4}};
PidIo upperRearPidIo  = {1000, 47, {0.6, 0.00107, 0.4}};
PID upperFrontPID(&upperFrontHeater.thermocouple, &upperFrontPidIo.Output, &upperFrontPidIo.Setpoint,
                  upperFrontPidIo.pidParameters.kp, upperFrontPidIo.pidParameters.ki, upperFrontPidIo.pidParameters.kd, DIRECT);
PID upperRearPID(&upperRearHeater.thermocouple, &upperRearPidIo.Output, &upperRearPidIo.Setpoint,
                 upperRearPidIo.pidParameters.kp, upperRearPidIo.pidParameters.ki, upperRearPidIo.pidParameters.kd, DIRECT);
#endif

volatile bool outputTempPeriodic = false;

static uint8_t watchdogResetOccurred = 0;


//------------------------------------------
// Prototypes
//------------------------------------------
void ConvertHeaterPercentCounts(void);
void readThermocouples(void);
void PeriodicOutputInfo(void);
uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf);
float GetFloatInputValue(float *pValue, uint8_t *pBuf);
void HeaterTimerInterrupt(void);
void saveParametersToMemory(void);
void readParametersFromMemory(void);
static void handleIncomingMessage(uint8_t *pData, uint8_t length);
bool needSave = false;

static float slewRateLimit(float newValue, float oldValue, float limit)
{
  float difference;
  
  if (newValue > oldValue)
  {
    difference = newValue - oldValue;
    if (difference <= limit)
    {
      return newValue;  
    }
    else
    {
      return oldValue + limit;
    }
  }
  else
  {
    difference = oldValue - newValue;
    if (difference <= limit)
    {
      return newValue;  
    }
    else
    {
      return oldValue - limit;
    }    
  }
}

void readThermocouples(void)
{
  static uint32_t oldTime = 1000;
  uint32_t newTime = millis();
  static bool filtersInitialized = false;

  if (newTime >= oldTime)
  {
    if ((newTime - oldTime) < A2D_READ_PERIOD_MS)
    {
      return;
    }
  }
  else
  {
    oldTime = newTime + (UINT32_MAX - oldTime);
    if ((newTime) < A2D_READ_PERIOD_MS)
    {
      return;
    }
  }

  oldTime = newTime;

  if (!filtersInitialized)
  {
    if (newTime < 1000) return;
    upperFrontHeater.thermocouple = readAD8495KTC(ANALOG_THERMO_UPPER_FRONT);
    upperRearHeater.thermocouple = readAD8495KTC(ANALOG_THERMO_UPPER_REAR);
    lowerFrontHeater.thermocouple = readAD8495KTC(ANALOG_THERMO_LOWER_FRONT);
    lowerRearHeater.thermocouple = readAD8495KTC(ANALOG_THERMO_LOWER_REAR);
    upperFrontHeater.tcFilter.initialize(upperFrontHeater.thermocouple);
    upperRearHeater.tcFilter.initialize(upperRearHeater.thermocouple);
    lowerFrontHeater.tcFilter.initialize(lowerFrontHeater.thermocouple);
    lowerRearHeater.tcFilter.initialize(lowerRearHeater.thermocouple);
    filtersInitialized = true;
  }

  upperFrontHeater.thermocouple = upperFrontHeater.tcFilter.step(
    slewRateLimit(readAD8495KTC(ANALOG_THERMO_UPPER_FRONT), upperFrontHeater.thermocouple, 250.0)); // 57.0
  upperRearHeater.thermocouple = upperRearHeater.tcFilter.step(
    slewRateLimit(readAD8495KTC(ANALOG_THERMO_UPPER_REAR), upperRearHeater.thermocouple, 250.0));
  lowerFrontHeater.thermocouple = lowerFrontHeater.tcFilter.step(
    slewRateLimit(readAD8495KTC(ANALOG_THERMO_LOWER_FRONT), lowerFrontHeater.thermocouple, 250.0)); // 4.4
  lowerRearHeater.thermocouple = lowerRearHeater.tcFilter.step(
    slewRateLimit(readAD8495KTC(ANALOG_THERMO_LOWER_REAR), lowerRearHeater.thermocouple, 250.0));

  ufTcLimit.checkLimit(upperFrontHeater.thermocouple);
  urTcLimit.checkLimit(upperRearHeater.thermocouple);
  lfTcLimit.checkLimit(lowerFrontHeater.thermocouple);
  lrTcLimit.checkLimit(lowerRearHeater.thermocouple);

  if (ufTcLimit.limitExceeded())
  {
    if (ufTcTempLimitFailed == false)
    {
      ufTcTempLimitFailed = true;
      ufTcLimitExceededCount++;
      needSave = true;
    }
  }
  if (urTcLimit.limitExceeded())
  {
    if (urTcTempLimitFailed == false)
    {
      urTcTempLimitFailed = true;
      urTcLimitExceededCount++;
      needSave = true;
    }
  }
  if (lfTcLimit.limitExceeded())
  {
    if (lfTcTempLimitFailed == false)
    {
      lfTcTempLimitFailed = true;
      lfTcLimitExceededCount++;
      needSave = true;
    }
  }
  if (lrTcLimit.limitExceeded())
  {
    if (lrTcTempLimitFailed == false)
    {
      lrTcTempLimitFailed = true;
      lrTcLimitExceededCount++;
      needSave = true;
    }
  }
  if (needSave)
  {
    saveParametersToMemory(); 
  }

  // Check differentials
  if (upperTempDiffExceeded == false)
  {
    if(fabs(fabs(upperFrontHeater.thermocouple - upperRearHeater.thermocouple) - fabs(upperFrontPidIo.Setpoint - upperRearPidIo.Setpoint)) > 500)
    {
      upperTempDiffExceededCount++; 
      if (upperTempDiffExceededCount >= 250) 
      {
        upperTempDiffExceeded = true;
      }
    }
    else 
    {
      upperTempDiffExceededCount = 0;
    }
  }

  double lowerFrontSetpoint = (lowerFrontHeater.parameter.tempSetPointHighOff + lowerFrontHeater.parameter.tempSetPointLowOn) / 2;
  double lowerRearSetpoint = (lowerRearHeater.parameter.tempSetPointHighOff + lowerRearHeater.parameter.tempSetPointLowOn) / 2;
  if (lowerTempDiffExceeded == false)
  {
    if(fabs(fabs(lowerFrontHeater.thermocouple - lowerRearHeater.thermocouple) - fabs(lowerFrontSetpoint - lowerRearSetpoint)) > 250)
    {
      lowerTempDiffExceededCount++; 
      if (lowerTempDiffExceededCount >= 250) 
      {
        lowerTempDiffExceeded = true;
      }
    }
    else
    {
      lowerTempDiffExceededCount = 0;
    }
  }
}

//------------------------------------------
// Code
//------------------------------------------

void ConvertHeaterPercentCounts(void)
{
  upperFrontHeater.heaterCountsOn  = (((uint32_t)upperFrontHeater.parameter.onPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;
  upperFrontHeater.heaterCountsOff = (((uint32_t)upperFrontHeater.parameter.offPercent * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;

  upperRearHeater.heaterCountsOn   = (((uint32_t)upperRearHeater.parameter.onPercent   * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;
  upperRearHeater.heaterCountsOff  = (((uint32_t)upperRearHeater.parameter.offPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;

  lowerFrontHeater.heaterCountsOn  = (((uint32_t)lowerFrontHeater.parameter.onPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
  lowerFrontHeater.heaterCountsOff = (((uint32_t)lowerFrontHeater.parameter.offPercent * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;

  lowerRearHeater.heaterCountsOn   = (((uint32_t)lowerRearHeater.parameter.onPercent   * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
  lowerRearHeater.heaterCountsOff  = (((uint32_t)lowerRearHeater.parameter.offPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
}

void UpdateHeaterHardware(void)
{
  changeRelayState(HEATER_TRIAC_UPPER_FRONT, upperFrontHeater.relayState);
  changeRelayState(HEATER_TRIAC_UPPER_REAR, upperRearHeater.relayState);
  changeRelayState(HEATER_RELAY_LOWER_FRONT, lowerFrontHeater.relayState);
  changeRelayState(HEATER_RELAY_LOWER_REAR, lowerRearHeater.relayState);
}

void AllHeatersOffStateClear(void)
{
	uint8_t i;

	for (i=0; i<4; i++)
	{
		aHeaters[i]->relayState = relayStateOff;
		aHeaters[i]->heaterCoolDownState = false;
	}

  UpdateHeaterHardware();

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOff);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOff);
}

void outputAcInputStates(void)
{
  //                                            0000000000111111111122222222223
  //                                            0123456789012345678901234567890
  static const uint8_t msgTemplate[] PROGMEM = "Power 7 L2DLB 8 TCO 9 AC 3";
  uint8_t msg[28];
  strcpy_P((char *)&msg[0], (const char *)&msgTemplate[0]);
  msg[6]  = powerButtonIsOn() ? '1' : '0';
  msg[14] = sailSwitchIsOn() ? '1' : '0';
  msg[20] = tcoInputIsOn() ? '1' : '0';
  msg[25] = acPowerIsPresent() ? '1' : '0';
  serialCommWrapperSendMessage(&msg[0], strlen((const char *)&msg[0]));
}

void outputDoorStatus(void)
{
  //                                            0000000000111111111122222222223
  //                                            0123456789012345678901234567890
  static const uint8_t msgTemplate[] PROGMEM = "Door 7 Count 65535";
  uint8_t msg[22];
  strcpy_P((char *)&msg[0], (char *)&msgTemplate[0]);
  msg[5] = '0' + (doorInput.IsActive() ? 1 : 0);
  itoa(doorDeployCount, (char *)&msg[13], 10);
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputDomeState(void)
{
  //                                           0000000000111111111122222222223
  //                                           0123456789012345678901234567890
  static const uint8_t msgDomeOn[]  PROGMEM = "Dome On";
  static const uint8_t msgDomeOff[] PROGMEM = "Dome Off";
  uint8_t msg[20];
  if(getDomeState())
  {
    strcpy_P((char *)&msg[0], (char *)&msgDomeOn[0]);
  }
  else
  {
    strcpy_P((char *)&msg[0], (char *)&msgDomeOff[0]);
  }
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputTemps(void)
{
  uint16_t intTempCUF, intTempCUR, intTempCLF, intTempCLR;
  // 0000000000111111111122222222223
  // 0123456789012345678901234567890
  // Temps 1111 2222 3333 4444
  uint8_t msg[30];
  uint8_t buf[7];

  intTempCUF =  (uint16_t) (upperFrontHeater.thermocouple + 0.5);
  intTempCUR =  (uint16_t) (upperRearHeater.thermocouple  + 0.5);
  intTempCLF =  (uint16_t) (lowerFrontHeater.thermocouple + 0.5);
  intTempCLR =  (uint16_t) (lowerRearHeater.thermocouple  + 0.5);
  
  msg[0] = 0;
  strcat((char *)&msg[0], "Temps ");
  itoa(intTempCUF, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (char *)&buf[0]);
  strcat((char *)&msg[0], " ");
  itoa(intTempCUR, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (char *)&buf[0]);
  strcat((char *)&msg[0], " ");
  itoa(intTempCLF, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (char *)&buf[0]);
  strcat((char *)&msg[0], " ");
  itoa(intTempCLR, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (char *)&buf[0]);
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputCookingState(void)
{
  //                                            0000000000111111111122222222223
  //                                            0123456789012345678901234567890
  static const uint8_t msgStandby[]  PROGMEM = "State Standby";
  static const uint8_t msgDlb[]      PROGMEM = "State DLB";
  static const uint8_t msgPreheat[] PROGMEM = "State Preheat";
  static const uint8_t msgPreheatStoneOnly[] PROGMEM = "State PreheatStoneOnly";
  static const uint8_t msgCooking[]  PROGMEM = "State Cooking";
  static const uint8_t msgStandbyBottom[]  PROGMEM = "State Idle";
  static const uint8_t msgCooldown[] PROGMEM = "State Cooldown";
  static const uint8_t msgInvalid[] PROGMEM = "State INVALID";
  uint8_t msg[20];
  switch (getCookingState())
  {
    case cookingStandby:
      strcpy_P((char *)&msg[0], (char *)&msgStandby[0]);
      break;
    case cookingWaitForDlb:
      strcpy_P((char *)&msg[0], (char *)&msgDlb[0]);
      break;
    case cookingPreheat:
      strcpy_P((char *)&msg[0], (char *)&msgPreheat[0]);
      break;
    case cookingPreheatStoneOnly:
      strcpy_P((char *)&msg[0], (char *)&msgPreheatStoneOnly[0]);
      break;
    case cookingCooking:
      strcpy_P((char *)&msg[0], (char *)&msgCooking[0]);
      break;
    case cookingIdle:
      strcpy_P((char *)&msg[0], (char *)&msgStandbyBottom[0]);
      break;
    case cookingCooldown:
      strcpy_P((char *)&msg[0], (char *)&msgCooldown[0]);
      break;
    default:
      strcpy_P((char *)&msg[0], (char *)&msgInvalid[0]);
      break;
  }
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputFailures(void)
{
  //                                                       0000000000111111111122222222223
  //                                                       0123456789012345678901234567890
  static const uint8_t msgUpperDiffExceeded[]   PROGMEM = "FAIL: upper_diff_exceeded";
  static const uint8_t msgLowerDiffExceeded[]   PROGMEM = "FAIL: lower_diff_exceeded";
  static const uint8_t msgWatchdogReset[]       PROGMEM = "WARN: watchdog_reset";
  static const uint8_t msgTcoFailure[]          PROGMEM = "FAIL: tco_failure";
  static const uint8_t msgCoolingFanFailure[]   PROGMEM = "FAIL: cooling_fan";
  static const uint8_t msgUfTcOvertempFailure[] PROGMEM = "FAIL: uf_overtemp";
  static const uint8_t msgUrTcOvertempFailure[] PROGMEM = "FAIL: ur_overtemp";
  static const uint8_t msgLfTcOvertempFailure[] PROGMEM = "FAIL: lf_overtemp";
  static const uint8_t msgLrTcOvertempFailure[] PROGMEM = "FAIL: lr_overtemp";
  static const uint8_t msgDoorDropped[]         PROGMEM = "FAIL: door_dropped";
  uint8_t msg[31];
  
  if(tcoAndFan.tcoHasFailed())
  {
    strcpy_P((char *)&msg[0], (char *)&msgTcoFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }
  if(tcoAndFan.coolingFanHasFailed())
  {
    strcpy_P((char *)&msg[0], (char *)&msgCoolingFanFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }
  if (watchdogResetOccurred != 0)
  {
    strcpy_P((char *)&msg[0], (char *)&msgWatchdogReset[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  if (ufTcTempLimitFailed != 0)
  {
    strcpy_P((char *)&msg[0], (char *)&msgUfTcOvertempFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  
  if (urTcTempLimitFailed != 0)
  {
    strcpy_P((char *)&msg[0], (char *)&msgUrTcOvertempFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  
  if (lfTcTempLimitFailed != 0)
  {
    strcpy_P((char *)&msg[0], (char *)&msgLfTcOvertempFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  
  if (lrTcTempLimitFailed != 0)
  {
    strcpy_P((char *)&msg[0], (char *)&msgLrTcOvertempFailure[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  

  if (upperTempDiffExceeded)
  {
    strcpy_P((char *)&msg[0], (char *)&msgUpperDiffExceeded[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  if (lowerTempDiffExceeded)
  {
    strcpy_P((char *)&msg[0], (char *)&msgLowerDiffExceeded[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }  
  if (doorInput.IsActive())
  {
    strcpy_P((char *)&msg[0], (const char *)&msgDoorDropped[0]);
    serialCommWrapperSendMessage(msg, strlen((char *)&msg[0]));  
  }
}

void outputRelayStates(void)
{
  //                                            0000000000111111111122222222223
  //                                            0123456789012345678901234567890
  static const uint8_t msgTemplate[] PROGMEM = "Relays 1 2 3 4";
  uint8_t msg[18];
  strcpy_P((char *)&msg[0], (const char *)&msgTemplate[0]);
  msg[7] = '0' + digitalRead(HEATER_TRIAC_UPPER_FRONT);
  msg[9] = '0' + digitalRead(HEATER_TRIAC_UPPER_REAR);
  msg[11] = '0' + digitalRead(HEATER_RELAY_LOWER_FRONT);
  msg[13] = '0' + digitalRead(HEATER_RELAY_LOWER_REAR);
  
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputPidDutyCycles(void)
{
  // 0000000000111111111122222222223
  // 0123456789012345678901234567890
  // PidDC 100.0000000 100.0000000
  uint8_t msg[35];
  strcpy((char *)&msg[0], "PidDC ");
  ftoa(upperFrontPidIo.Output, &msg[strlen((char *)&msg[0])], 7);
  strcat((char *)&msg[0], " ");
  ftoa(upperRearPidIo.Output, &msg[strlen((char *)&msg[0])], 7);
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void outputTimeInfo(void)
{
  // 00000000001111111111222222222233333333334
  // 01234567890123456789012345678901234567890
  // Time: 4294967295, thresh: 4294967295
  uint8_t msg[45];
  uint8_t buf[15];
  
  msg[0] = 0;
  strcat((char *)&msg[0], "Time: ");
  ultoa(relayTimeBase, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (char *)&buf[0]);
  strcat((char *)&msg[0], ", thresh: ");
  ultoa(lowerRearHeater.heaterCountsOn, (char *)&buf[0], 10);
  strcat((char *)&msg[0], (const char *)&buf[0]);
  serialCommWrapperSendMessage(&msg[0], strlen((char *)&msg[0]));
}

void PeriodicOutputInfo()
{
  static uint8_t printPhase = 0;
#ifdef USE_PID
#ifdef ENABLE_PID_TUNING
    double pTerm;
    double iTerm;
    double dTerm;
#endif
#endif

  switch (printPhase)
  {
    case 0:
      if (true == outputTempPeriodic)
      {
        printPhase++;
        outputTempPeriodic = false;
      }
      break;
    case 1:
      outputTemps();
      outputAcInputStates();
      outputDoorStatus();
      outputCookingState();
      outputFailures();
      handleRelayWatchdog();
      outputRelayStates();
      outputPidDutyCycles();
      outputTimeInfo();
      printPhase++;
    break;
    case 2:
      outputDomeState();
      printPhase++;
      break;
    
#ifdef USE_PID
#ifdef ENABLE_PID_TUNING
    case 2:
      Serial.print(F("DEBUG, Time, UR KP, UR KI, UR KD, UR Raw, UR Temp, UR DC, UR Setpoint, UR pTerm, "));
      printPhase++;
      break;
    case 3:
      Serial.println(F("UR iTerm, UR dTerm, UF KP, UF KI, UF KD, UF Raw, UF Temp, UF DC, UF Setpoint, UF pTerm, UF iTerm, UF dTerm"));
      printPhase++;
      break;
    case 4:
      printPhase++;
      break;
    case 5:
      Serial.print(F("DEBUG, "));
      Serial.print(millis());
      Serial.print(F(", "));
      Serial.print(upperRearPID.GetKp(), 7);
      Serial.print(F(", "));
      Serial.print(upperRearPID.GetKi(), 7);
      Serial.print(F(", "));
      Serial.print(upperRearPID.GetKd(), 7);
      Serial.print(F(", "));
      printPhase++;
      break;
    case 6:
      Serial.print(readAD8495KTC(ANALOG_THERMO_UPPER_REAR));
      Serial.print(F(", "));
      Serial.print(upperRearHeater.thermocouple);
      Serial.print(F(", "));
      Serial.print(upperRearPidIo.Output);
      Serial.print(F(", "));
      Serial.print(upperRearPidIo.Setpoint);
      Serial.print(F(", "));
      printPhase++;
      break;
    case 7:
      upperRearPID.GetTerms(&pTerm, &iTerm, &dTerm);
      Serial.print(pTerm, 6);
      Serial.print(F(", "));
      Serial.print(iTerm, 6);
      Serial.print(F(", "));
      Serial.print(dTerm, 6);
      Serial.print(F(", "));
      printPhase++;
      break;
    case 8:
      Serial.print(upperFrontPID.GetKp(), 7);
      Serial.print(F(", "));
      Serial.print(upperFrontPID.GetKi(), 7);
      Serial.print(F(", "));
      Serial.print(upperFrontPID.GetKd(), 7);
      Serial.print(F(", "));
      printPhase++;
      break;
    case 9:
      Serial.print(readAD8495KTC(ANALOG_THERMO_UPPER_FRONT));
      Serial.print(F(", "));
      Serial.print(upperFrontHeater.thermocouple);
      Serial.print(F(", "));
      Serial.print(upperFrontPidIo.Output);
      Serial.print(F(", "));
      Serial.print(upperFrontPidIo.Setpoint);
      Serial.print(F(", "));
      printPhase++;
      break;
    case 10:
      upperFrontPID.GetTerms(&pTerm, &iTerm, &dTerm);
      Serial.print(pTerm, 6);
      Serial.print(F(", "));
      Serial.print(iTerm, 6);
      Serial.print(F(", "));
      Serial.print(dTerm, 6);
      Serial.println("");
      printPhase++;
      break;
#endif
#endif
    case 11:
      printPhase++;
      break;
    default:
      printPhase = 0;
  }
}

//------------------------------------------
// Update Heater State Interrupt
//------------------------------------------
void HeaterTimerInterrupt()
{
  if (triacTimeBase < (MILLISECONDS_PER_SECOND * triacPeriodSeconds))
  {
    triacTimeBase++;
  }
  else
  {
    triacTimeBase = 0;
  }

  if (relayTimeBase < (MILLISECONDS_PER_SECOND * relayPeriodSeconds))
  {
    relayTimeBase++;
  }
  else
  {
    relayTimeBase = 0;
  }

  countOutputTempPeriodic++;

  if (countOutputTempPeriodic >= TIMER1_OUTPUT_TEMP_PERIODIC)
  {
    countOutputTempPeriodic = 0;
    outputTempPeriodic = true;
  }
}

void saveParametersToMemory(void)
{
  pizzaMemoryWrite((uint8_t*)&upperFrontHeater.parameter, offsetof(MemoryStore, upperFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&upperRearHeater.parameter, offsetof(MemoryStore, upperRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&lowerFrontHeater.parameter, offsetof(MemoryStore, lowerFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&lowerRearHeater.parameter, offsetof(MemoryStore, lowerRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&triacPeriodSeconds, offsetof(MemoryStore, triacPeriodSeconds), sizeof(triacPeriodSeconds));
  pizzaMemoryWrite((uint8_t*)&relayPeriodSeconds, offsetof(MemoryStore, relayPeriodSeconds), sizeof(relayPeriodSeconds));
#ifdef USE_PID
  pizzaMemoryWrite((uint8_t*)&upperFrontPidIo.pidParameters, offsetof(MemoryStore, upperFrontPidParameters), sizeof(PidParameters));
  pizzaMemoryWrite((uint8_t*)&upperRearPidIo.pidParameters, offsetof(MemoryStore, upperRearPidParameters), sizeof(PidParameters));
#endif
  pizzaMemoryWrite((uint8_t*)&doorDeployCount, offsetof(MemoryStore, doorDeployCount), sizeof(doorDeployCount));
  pizzaMemoryWrite((uint8_t*)&doorHasDeployed, offsetof(MemoryStore, doorHasDeployed), sizeof(doorHasDeployed));
  pizzaMemoryWrite((uint8_t*)&ufTcLimitExceededCount, offsetof(MemoryStore, ufTcLimitExceededCount), sizeof(ufTcLimitExceededCount));
  pizzaMemoryWrite((uint8_t*)&urTcLimitExceededCount, offsetof(MemoryStore, urTcLimitExceededCount), sizeof(urTcLimitExceededCount));
  pizzaMemoryWrite((uint8_t*)&lfTcLimitExceededCount, offsetof(MemoryStore, lfTcLimitExceededCount), sizeof(lfTcLimitExceededCount));
  pizzaMemoryWrite((uint8_t*)&lrTcLimitExceededCount, offsetof(MemoryStore, lrTcLimitExceededCount), sizeof(lrTcLimitExceededCount));
}

void readParametersFromMemory(void)
{
  pizzaMemoryRead((uint8_t*)&upperFrontHeater.parameter, offsetof(MemoryStore, upperFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&upperRearHeater.parameter, offsetof(MemoryStore, upperRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&lowerFrontHeater.parameter, offsetof(MemoryStore, lowerFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&lowerRearHeater.parameter, offsetof(MemoryStore, lowerRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&triacPeriodSeconds, offsetof(MemoryStore, triacPeriodSeconds), sizeof(triacPeriodSeconds));
  pizzaMemoryRead((uint8_t*)&relayPeriodSeconds, offsetof(MemoryStore, relayPeriodSeconds), sizeof(relayPeriodSeconds));
#ifdef USE_PID
  pizzaMemoryRead((uint8_t*)&upperFrontPidIo.pidParameters, offsetof(MemoryStore, upperFrontPidParameters), sizeof(PidParameters));
  pizzaMemoryRead((uint8_t*)&upperRearPidIo.pidParameters, offsetof(MemoryStore, upperRearPidParameters), sizeof(PidParameters));
#endif
  pizzaMemoryRead((uint8_t*)&doorDeployCount, offsetof(MemoryStore, doorDeployCount), sizeof(doorDeployCount));
  pizzaMemoryRead((uint8_t*)&ufTcLimitExceededCount, offsetof(MemoryStore, ufTcLimitExceededCount), sizeof(ufTcLimitExceededCount));
  pizzaMemoryRead((uint8_t*)&urTcLimitExceededCount, offsetof(MemoryStore, urTcLimitExceededCount), sizeof(urTcLimitExceededCount));
  pizzaMemoryRead((uint8_t*)&lfTcLimitExceededCount, offsetof(MemoryStore, lfTcLimitExceededCount), sizeof(lfTcLimitExceededCount));
  pizzaMemoryRead((uint8_t*)&lrTcLimitExceededCount, offsetof(MemoryStore, lrTcLimitExceededCount), sizeof(lrTcLimitExceededCount));
}

static void sendSerialByte(uint8_t b)
{
  Serial.write(b);
}

//------------------------------------------
// Setup Routine
//------------------------------------------
void setup()
{
  uint8_t mcusrAtStart = MCUSR;
  cli(); 
  wdt_reset();

  upperTempDiffExceededCount = 0;
  lowerTempDiffExceededCount = 0;

  /* Clear all flags in MCUSR */
  MCUSR &= ~((1<<WDRF) | (1<<BORF) | (1<<EXTRF) | (1<<PORF) );
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;

  pizzaMemoryReturnTypes pizzaMemoryInitResponse;
  uint8_t i;
  uint8_t relaysToInitialize[] = {
    COOLING_FAN_RELAY,
    COOLING_FAN_LOW_SPEED,
    HEATER_TRIAC_UPPER_FRONT,
    HEATER_TRIAC_UPPER_REAR,
    HEATER_RELAY_LOWER_FRONT,
    HEATER_RELAY_LOWER_REAR,
    HEATER_UPPER_FRONT_DLB,
    HEATER_UPPER_REAR_DLB
  };
  uint8_t adcsToInitialize[] = {
    ANALOG_THERMO_UPPER_FRONT,
    ANALOG_THERMO_UPPER_REAR,
    ANALOG_THERMO_LOWER_FRONT,
    ANALOG_THERMO_LOWER_REAR
  };

  // disable the watchdog timer

  Serial.begin(19200);
  Serial.println(F("DEBUG Starting pizza oven..."));

  if (mcusrAtStart & (1<<WDRF))
  {
    Serial.println(F("DEBUG The system restarted due to watchdog timer reset."));
    watchdogResetOccurred = 1;
  }
  if (mcusrAtStart & (1<<BORF))
  {
    Serial.println(F("DEBUG The system restarted due to brown-out reset."));
  }
  if (mcusrAtStart & (1<<EXTRF))
  {
    Serial.println(F("DEBUG The system restarted due to external reset."));
  }
  if (mcusrAtStart & (1<<PORF))
  {
    Serial.println(F("DEBUG The system restarted due to power on reset."));
  }

  serialCommWrapperInit(sendSerialByte, handleIncomingMessage);
  handleIncomingMessage((uint8_t *)"v", 1);
  
  pizzaMemoryInitResponse = pizzaMemoryInit();

  if (pizzaMemoryWasEmpty == pizzaMemoryInitResponse)
  {
    Serial.println(F("DEBUG Initializing empty EEPROM memory."));
    saveParametersToMemory();
  }
  else
  {
    Serial.println(F("DEBUG Reading parameters from EEPROM memory."));
    readParametersFromMemory();
  }

  acInputsInit();

  // Initialize Timer1
  Timer1.initialize(TIMER1_PERIOD_MICRO_SEC * TIMER1_PERIOD_CLOCK_FACTOR);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10);
  Timer1.attachInterrupt(HeaterTimerInterrupt);

  // init relays
  for (i=0; i<sizeof(relaysToInitialize)/sizeof(uint8_t); i++)
  {
    initializeRelayPin(relaysToInitialize[i]);
  }

  pinMode(BOOST_ENABLE, OUTPUT);
  digitalWrite(BOOST_ENABLE, LOW);
  pinMode(RELAY_WATCHDOG, OUTPUT);

  // Initialize the A2D engine
  for (i=0; i<sizeof(adcsToInitialize)/sizeof(uint8_t); i++)
  {
    adcReadInit(adcsToInitialize[i]);
  }

  ConvertHeaterPercentCounts();

#ifdef USE_PID
  upperFrontPID.SetMode(MANUAL);
  upperFrontPID.SetOutputLimits(0, MAX_PID_OUTPUT);
  upperFrontPID.SetSampleTime(4000);
  upperFrontPID.SetTunings(upperFrontPidIo.pidParameters.kp, upperFrontPidIo.pidParameters.ki, upperFrontPidIo.pidParameters.kd);

  upperRearPID.SetMode(MANUAL);
  upperRearPID.SetOutputLimits(0, MAX_PID_OUTPUT);
  upperRearPID.SetSampleTime(4000);
  upperRearPID.SetTunings(upperRearPidIo.pidParameters.kp, upperRearPidIo.pidParameters.ki, upperRearPidIo.pidParameters.kd);
#endif

  wdt_enable(WDTO_2S);

  Serial.println(F("DEBUG Initialization complete."));
  sei();
}

#define RECEVIED_COMMAND_BUFFER_LENGTH (16)

/*
   Serial commands prefixes:

   d - set dome on or off
   D - get dome state
   f - set off percent
   g - set pid gains
   G - get the pid gains
   l - set lower temp limit
   n - set on percent
   p - get heater parameters
   q - stop oven
   s - start oven
   t - set time base
   v - get firmware version
   u - set upper temp limit

*/
static void handleIncomingMessage(uint8_t *pData, uint8_t length)
{
  uint8_t receivedCommandBuffer[RECEVIED_COMMAND_BUFFER_LENGTH];
  uint8_t receivedCommandBufferIndex = 0;
  uint8_t lastByteReceived;
  PID *pPid = NULL;
  uint8_t heaterIndex;
  const uint8_t * pText;
  uint16_t newSetpoint;
  // Messages
  // 0000000001111111111222222222233333333334444444444555555555566666666667
  // 1234567890123456789012345678901234567890123456789012345678901234567890
  // @nTimes 12345 12345[0xaa,0xbb]rn

  while (length > 0)
  {
    lastByteReceived = *pData++;
    length--;
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
        case 's':  // Start Pizza Oven Cycle
          requestPizzaOvenStart();
          receivedCommandBufferIndex = 0;
          Serial.println(F("DEBUG Pizza oven start requested."));
          break;

        case 'd':  // Set dome state
          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '0' :
                Serial.println(F("DEBUG Setting dome off."));
                setDomeState(0);
                break;
              case '1' :
                Serial.println(F("DEBUG Setting dome on."));
                setDomeState(1);
                break;
              default:
                Serial.print(F("DEBUG unknown command received for the 'd' command: "));
                Serial.println(receivedCommandBuffer[1]);
                break;
            }

            receivedCommandBufferIndex = 0;
          }
          break;

        case 'D':  // get dome state
          outputDomeState();
          receivedCommandBufferIndex = 0;
          break;

        case 'q': // Quit Pizza Oven Cycle
          requestPizzaOvenStop();
          receivedCommandBufferIndex = 0;
          upperFrontPID.SetMode(MANUAL);
          Serial.println(F("DEBUG Pizza oven stop requested."));
          break;

        case 'v': // query protocol version
          serialCommWrapperSendMessage((uint8_t *)&versionString[0], strlen(versionString));
          receivedCommandBufferIndex = 0;
          break;

        case 'p': // query heat control parameters
          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '0' :
                static const uint8_t nTimesText[] PROGMEM = "nTimes ";
                printMessageWithTwoUints(nTimesText, triacPeriodSeconds, relayPeriodSeconds); 
                break;
              case '1' :
                static const char ufText[] PROGMEM = "UF ";
                printHeaterTemperatureParameters(ufText, upperFrontHeater.parameter.parameterArray);
                break;
              case '2' :
                static const char urText[] PROGMEM = "UR ";
                printHeaterTemperatureParameters(urText, upperRearHeater.parameter.parameterArray);
                break;
              case '3' :
                static const char lfText[] PROGMEM = "LF ";
                printHeaterTemperatureParameters(lfText, lowerFrontHeater.parameter.parameterArray);
                break;
              case '4' :
                static const char lrText[] PROGMEM = "LR ";
                printHeaterTemperatureParameters(lrText, lowerRearHeater.parameter.parameterArray);
                break;
              default:
                Serial.print(F("DEBUG unknown command received for the 'p' command: "));
                Serial.println(receivedCommandBuffer[1]);
                break;
            }

            receivedCommandBufferIndex = 0;
          }
          break;

        case 'u':  // Set Upper Set Point Parameter
        case 'l':  // Set Lower Set Point Parameter
          // wait for CR or LF
          if ((lastByteReceived == 10) || (lastByteReceived == 13))
          {
            Heater *pHeater;
            if (CharIsADigit(receivedCommandBuffer[1]) && (receivedCommandBuffer[1] > '0') && (receivedCommandBuffer[1] < '5'))
            {
              heaterIndex = (receivedCommandBuffer[1] - '0')-1;
              pHeater = aHeaters[heaterIndex];
              
              GetInputValue(&newSetpoint, &receivedCommandBuffer[2]);
              if (newSetpoint > maxTempSetting[heaterIndex])
              {
                newSetpoint = maxTempSetting[heaterIndex];
              }
              
              if (receivedCommandBuffer[0] == 'l')
              {
                Serial.println(F("DEBUG Setting lower setpoint."));
                if (newSetpoint > pHeater->parameter.tempSetPointLowOn)
                {
                  theSetpointWasIncreased(stoneSetpointIncreased);
                }
                pHeater->parameter.tempSetPointLowOn = newSetpoint;
              }
              else
              {
                Serial.println(F("DEBUG Setting upper setpoint."));
                if (newSetpoint > pHeater->parameter.tempSetPointHighOff)
                {
                  theSetpointWasIncreased(domeSetpointIncreased);
                }
                pHeater->parameter.tempSetPointHighOff = newSetpoint;
              }
              saveParametersToMemory();
            }
            else
            {
              Serial.print(F("DEBUG Invalid heater selected: "));
              Serial.println((char)receivedCommandBuffer[1]);
            }
            receivedCommandBufferIndex = 0;
          }
          break;

        case 'f':  // Set Off Percent Parameter
        case 'n':  // Set On Percent Parameter
          // wait for CR or LF
          if ((lastByteReceived == 10) || (lastByteReceived == 13))
          {
            Heater *pHeater;
            if (CharIsADigit(receivedCommandBuffer[1]) && (receivedCommandBuffer[1] > '0') && (receivedCommandBuffer[1] < '5'))
            {
              pHeater = aHeaters[(receivedCommandBuffer[1] - '0')-1];
              if (receivedCommandBuffer[0] == 'n')
              {
                GetInputValue(&pHeater->parameter.onPercent, &receivedCommandBuffer[2]);
                Serial.println(F("DEBUG Setting on percent."));
              }
              else
              {
                GetInputValue(&pHeater->parameter.offPercent, &receivedCommandBuffer[2]);
                Serial.println(F("DEBUG Setting off percent."));
              }
              ConvertHeaterPercentCounts();
              saveParametersToMemory();
              Serial.println(F("Updated duty cycle parameter."));
            }
            else
            {
              Serial.print(F("DEBUG Invalid heater selected: "));
              Serial.println((char)receivedCommandBuffer[1]);
            }
            receivedCommandBufferIndex = 0;
          }
          break;

        case 't':  // Set time base in seconds
          // wait for CR or LF
          if ((lastByteReceived == 10) || (lastByteReceived == 13))
          {
            if ((cookingStandby == getCookingState()) ||
                cookingCooldown == getCookingState())
            {
              if (receivedCommandBufferIndex > 3)
              {
                uint16_t tempMultiply;

                GetInputValue(&tempMultiply, &receivedCommandBuffer[2]);

                switch (receivedCommandBuffer[1])
                {
                  case '1':
                    if ((tempMultiply > 0) && (tempMultiply <= 30))
                    {
                      triacPeriodSeconds = tempMultiply;
                      ConvertHeaterPercentCounts();
                      saveParametersToMemory();
                    }
                    else
                    {
                      Serial.print(F("DEBUG Invalid time multiplier: "));
                      Serial.println(tempMultiply);
                    }
                    break;
                  case '2':
                    if ((tempMultiply > 0) && (tempMultiply <= 600))
                    {
                      relayPeriodSeconds = tempMultiply;
                      ConvertHeaterPercentCounts();
                      saveParametersToMemory();
                    }
                    else
                    {
                      Serial.print(F("DEBUG Invalid time multiplier: "));
                      Serial.println(tempMultiply);
                    }
                    break;
                  default:
                    Serial.println(F("DEBUG Invalid time base selected."));
                    break;
                }


              }
              else
              {
                Serial.println(F("DEBUG Cannot set time period, not enough characters"));
              }
            }
            else
            {
              Serial.println(F("DEBUG Cannot set time period at this time."));
            }
            receivedCommandBufferIndex = 0;
          }
          break;

#ifdef USE_PID
        case 'g':
          if ((lastByteReceived == 10) || (lastByteReceived == 13))
          {
            if ((cookingStandby == getCookingState()) ||
                cookingCooldown == getCookingState())
            {
              if (receivedCommandBufferIndex > 4)
              {
                PidParameters *pPidParams = NULL;
                if (receivedCommandBuffer[1] == '1')
                {
                  pPidParams = &upperFrontPidIo.pidParameters;
                }
                else if (receivedCommandBuffer[1] == '2')
                {
                  pPidParams = &upperRearPidIo.pidParameters;
                }
                else
                {
                  Serial.println(F("DEBUG unknown PID location"));
                }

                if (pPidParams != NULL)
                {

                  switch (receivedCommandBuffer[2])
                  {
                    case 'p':
                      GetFloatInputValue(&pPidParams->kp, &receivedCommandBuffer[3]);
                      break;
                    case 'i':
                      GetFloatInputValue(&pPidParams->ki, &receivedCommandBuffer[3]);
                      break;
                    case 'd':
                      GetFloatInputValue(&pPidParams->kd, &receivedCommandBuffer[3]);
                      break;
                    default:
                      Serial.println(F("Unknown gain parameter."));
                      break;
                  }
                  saveParametersToMemory();
                  upperFrontPID.SetTunings(upperFrontPidIo.pidParameters.kp, upperFrontPidIo.pidParameters.ki, upperFrontPidIo.pidParameters.kd);
                  upperRearPID.SetTunings(upperRearPidIo.pidParameters.kp, upperRearPidIo.pidParameters.ki, upperRearPidIo.pidParameters.kd);
                }
                else
                {
                  Serial.println(F("DEBUG Error setting PID value."));
                }
              }
              else
              {
                Serial.println(F("DEBUG Cannot set PID gain, not enough characters"));
              }
            }
            else
            {
              Serial.println(F("DEBUG Cannot set PID gain at this time."));
            }
            receivedCommandBufferIndex = 0;
          }
          break;

        case 'G': // query PID parameters
          static const uint8_t ufgText[] PROGMEM = "UFG ";
          static const uint8_t urgText[] PROGMEM = "URG ";
          
          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '1' :
                pText = ufgText;
                pPid = &upperFrontPID;
                break;
              case '2' :
                pText = urgText;
                pPid = &upperRearPID;
                break;
              default:
                Serial.print(F("DEBUG unknown command received for the 'G' command: "));
                Serial.println(receivedCommandBuffer[1]);
                break;
            }

            if (pPid != NULL)
            {
              printPidGains(pText, pPid);
            }

            receivedCommandBufferIndex = 0;
          }
          break;
#endif

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

byte queryDone = false;
uint32_t liveCount = 0;
uint16_t inputValue;
uint16_t tempMultiply, tempPercent, tempTemp;

void updateDcInputs(void)
{
  static int oldMillis = millis();
  int newMillis = millis();

  if ((newMillis - oldMillis) >= 10)
  {
    oldMillis = newMillis;

    doorInput.UpdateInput();
  }
}

//------------------------------------------
// Main Loop
//------------------------------------------
void loop()
{
  bool oldPowerButtonState = powerButtonIsOn();
  bool oldDlbState = sailSwitchIsOn();

  // pet the watchdog
  wdt_reset();

  // Gather inputs and process
  adcReadRun();
  acInputsRun();
  readThermocouples();
  if (Serial.available() > 0)
  {
    serialCommWrapperHandleByte(Serial.read());
  }
  updateDcInputs();

  // handle door status
  if (doorInput.IsActive())
  {
    if (doorHasDeployed == false)
    {
      doorHasDeployed = true;
      doorDeployCount++; 
      saveParametersToMemory();
    }
  }

  if ((oldPowerButtonState != powerButtonIsOn()) || (oldDlbState != sailSwitchIsOn()))
  {
    outputAcInputStates();
  }

//poStateMachine.update();
  updateCookingStateMachine();

  boostEnable(relayBoostRun);
  relayDriverRun();

  handleRelayWatchdog();

  PeriodicOutputInfo();

  // PID
#ifdef USE_PID
  upperFrontPidIo.Setpoint = (upperFrontHeater.parameter.tempSetPointHighOff + upperFrontHeater.parameter.tempSetPointLowOn) / 2;
  upperRearPidIo.Setpoint = (upperRearHeater.parameter.tempSetPointHighOff + upperRearHeater.parameter.tempSetPointLowOn) / 2;
  upperFrontPID.Compute();
  upperRearPID.Compute();
  ConvertHeaterPercentCounts();
  upperFrontHeater.heaterCountsOff = (uint16_t)(((uint32_t)((upperFrontPidIo.Output * MILLISECONDS_PER_SECOND + 50)) / 100)) * triacPeriodSeconds;
  upperRearHeater.heaterCountsOn  = (uint16_t)(((uint32_t)(((100.0 - upperRearPidIo.Output) * MILLISECONDS_PER_SECOND + 50)) / 100)) * triacPeriodSeconds;

#endif
}

