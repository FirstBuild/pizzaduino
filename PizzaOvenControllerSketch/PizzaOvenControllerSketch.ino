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

//#include "FiniteStateMachine.h"
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

static TcoAndFan tcoAndFan;

#ifndef UINT32_MAX
#define UINT32_MAX (0xffffffff)
#endif

//------------------------------------------
// Macros
//------------------------------------------
#define FIRMWARE_MAJOR_VERSION   1
#define FIRMWARE_MINOR_VERSION   0
#define FIRMWARE_BUILD_VERSION   4

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

static bool TCsHaveBeenInitialized = false;

Heater upperFrontHeater = {{true, 1200, 1300,   0,  100}, 0, 0, relayStateOff, false, 0.0};
Heater upperRearHeater  = {{true, 1100, 1200,  0, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerFrontHeater = {{true,  600,  650,  50, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerRearHeater  = {{true,  575,  625,   0,  49}, 0, 0, relayStateOff, false, 0};

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
void ConvertHeaterPercentCounts();
void readThermocouples(void);
void PeriodicOutputTemps();
uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf);
float GetFloatInputValue(float *pValue, uint8_t *pBuf);
void HeaterTimerInterrupt();
void saveParametersToMemory(void);
void readParametersFromMemory(void);

void readThermocouples(void)
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

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

  upperFrontHeater.thermocouple = upperFrontHeater.tcFilter.step(readAD8495KTC(ANALOG_THERMO_UPPER_FRONT));
  upperRearHeater.thermocouple = upperRearHeater.tcFilter.step(readAD8495KTC(ANALOG_THERMO_UPPER_REAR));
  lowerFrontHeater.thermocouple = lowerFrontHeater.tcFilter.step(readAD8495KTC(ANALOG_THERMO_LOWER_FRONT));
  lowerRearHeater.thermocouple = lowerRearHeater.tcFilter.step(readAD8495KTC(ANALOG_THERMO_LOWER_REAR));
  TCsHaveBeenInitialized = true;
}

//------------------------------------------
// Code
//------------------------------------------

void ConvertHeaterPercentCounts()
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

void UpdateHeaterHardware()
{
  changeRelayState(HEATER_TRIAC_UPPER_FRONT, upperFrontHeater.relayState);
  changeRelayState(HEATER_TRIAC_UPPER_REAR, upperRearHeater.relayState);
  changeRelayState(HEATER_RELAY_LOWER_FRONT, lowerFrontHeater.relayState);
  changeRelayState(HEATER_RELAY_LOWER_REAR, lowerRearHeater.relayState);
}

void AllHeatersOffStateClear()
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

void outputAcInputStates()
{
  Serial.print(F("Power "));
  Serial.print(powerButtonIsOn());
  Serial.print(F(" L2DLB "));
  Serial.print(sailSwitchIsOn());
  Serial.print(F(" TCO "));
  Serial.println(tcoInputIsOn());
}

void outputDoorStatus()
{
  Serial.print(F("Door "));
  Serial.print(doorInput.IsActive() ? 1 : 0);
  Serial.print(F(" Count "));
  Serial.println(doorDeployCount);
}

void PeriodicOutputTemps()
{
  uint16_t intTempCUF, intTempCUR, intTempCLF, intTempCLR;
  static uint8_t printPhase = 0;
#ifdef USE_PID
#ifdef ENABLE_PID_TUNING
    double pTerm;
    double iTerm;
    double dTerm;
#endif
#endif

  intTempCUF =  (uint16_t) (upperFrontHeater.thermocouple + 0.5);
  intTempCUR =  (uint16_t) (upperRearHeater.thermocouple  + 0.5);
  intTempCLF =  (uint16_t) (lowerFrontHeater.thermocouple + 0.5);
  intTempCLR =  (uint16_t) (lowerRearHeater.thermocouple  + 0.5);

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
      Serial.print(F("Temps "));
      Serial.print(intTempCUF);
      Serial.print(F(" "));
      Serial.print(intTempCUR);
      Serial.print(F(" "));
      Serial.print(intTempCLF);
      Serial.print(F(" "));
      Serial.println(intTempCLR);
  
      outputAcInputStates();
      outputDoorStatus();
    
      switch (getCookingState())
      {
        case cookingStandby:
          Serial.println(F("State Standby"));
          break;
        case cookingWaitForDlb:
          Serial.println(F("State DLB"));
          break;
        case cookingCooking:
          Serial.println(F("State Cooking"));
          break;
        case cookingCooldown:
          Serial.println(F("State Cooldown"));
          break;
      }
    
      if(tcoAndFan.tcoHasFailed())
      {
          Serial.println(F("TCO failure"));      
      }
      if(tcoAndFan.coolingFanHasFailed())
      {
          Serial.println(F("Cooling fan failure"));      
      }
      if (watchdogResetOccurred != 0)
      {
          Serial.println(F("Watchdog reset occurred"));      
      }
    
      Serial.print(F("Relays "));
      Serial.print(digitalRead(HEATER_TRIAC_UPPER_FRONT));
      Serial.print(F(" "));
      Serial.print(digitalRead(HEATER_TRIAC_UPPER_REAR));
      Serial.print(F(" "));
      Serial.print(digitalRead(HEATER_RELAY_LOWER_FRONT));
      Serial.print(F(" "));
      Serial.println(digitalRead(HEATER_RELAY_LOWER_REAR));
      Serial.print(F("PidDC "));
      Serial.print(upperFrontPidIo.Output, 7);
      Serial.print(F(" "));
      Serial.println(upperRearPidIo.Output, 7);
  
      Serial.print("Time: ");
      Serial.print(relayTimeBase);
      Serial.print(", thresh: ");
      Serial.println(lowerRearHeater.heaterCountsOn);
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
}

//------------------------------------------
// Setup Routine
//------------------------------------------
void setup()
{
  uint8_t mcusrAtStart = MCUSR;
  cli(); 
  wdt_reset();

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
    Serial.println(F("The system restarted due to watchdog timer reset."));
    watchdogResetOccurred = 1;
  }
  if (mcusrAtStart & (1<<BORF))
  {
    Serial.println(F("The system restarted due to brown-out reset."));
  }
  if (mcusrAtStart & (1<<EXTRF))
  {
    Serial.println(F("The system restarted due to external reset."));
  }
  if (mcusrAtStart & (1<<PORF))
  {
    Serial.println(F("The system restarted due to power on reset."));
  }

  
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

*/
void handleIncomingCommands(void)
{
  static uint8_t receivedCommandBuffer[RECEVIED_COMMAND_BUFFER_LENGTH];
  static uint8_t receivedCommandBufferIndex = 0;
  uint8_t lastByteReceived;
  PID *pPid = NULL;
  uint8_t heaterIndex;

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
        case 's':  // Start Pizza Oven Cycle
          //pizzaOvenStartRequested = true;
          requestPizzaOvenStart();
          receivedCommandBufferIndex = 0;
          Serial.println(F("DEBUG Pizza oven start requested."));
          break;

        case 'q': // Quit Pizza Oven Cycle
          //pizzaOvenStopRequested = true;
          requestPizzaOvenStop();
          receivedCommandBufferIndex = 0;
          upperFrontPID.SetMode(MANUAL);
          Serial.println(F("DEBUG Pizza oven stop requested."));
          break;

        case 'v': // query protocol version
        #ifdef KILL
          Serial.print(F("V "));
          Serial.print(FIRMWARE_MAJOR_VERSION);
          Serial.print(F("."));
          Serial.print(FIRMWARE_MINOR_VERSION);
          Serial.print(F(" bugfix "));
          Serial.println(FIRMWARE_BUILD_VERSION);
          #endif
          Serial.println(versionString);
          receivedCommandBufferIndex = 0;
          break;

        case 'p': // query heat control parameters
          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '0' :
                Serial.print(F("nTimes "));
                Serial.print(triacPeriodSeconds);
                Serial.print(" ");
                Serial.println(relayPeriodSeconds);
                break;
              case '1' :
                printHeaterTemperatureParameters("UF ", upperFrontHeater.parameter.parameterArray);
                break;
              case '2' :
                printHeaterTemperatureParameters("UR ", upperRearHeater.parameter.parameterArray);
                break;
              case '3' :
                printHeaterTemperatureParameters("LF ", lowerFrontHeater.parameter.parameterArray);
                break;
              case '4' :
                printHeaterTemperatureParameters("LR ", lowerRearHeater.parameter.parameterArray);
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
              if (receivedCommandBuffer[0] == 'l')
              {
                Serial.println(F("DEBUG Setting lower setpoint."));
                GetInputValue(&pHeater->parameter.tempSetPointLowOn, &receivedCommandBuffer[2]);
                if (&pHeater->parameter.tempSetPointLowOn > maxTempSetting[heaterIndex])
                {
                  pHeater->parameter.tempSetPointLowOn = maxTempSetting[heaterIndex];
                }
              }
              else
              {
                Serial.println(F("DEBUG Setting upper setpoint."));
                GetInputValue(&pHeater->parameter.tempSetPointHighOff, &receivedCommandBuffer[2]);
                if (&pHeater->parameter.tempSetPointHighOff > maxTempSetting[heaterIndex])
                {
                  pHeater->parameter.tempSetPointHighOff = maxTempSetting[heaterIndex];
                }
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

          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '1' :
                Serial.print(F("UFG "));
                pPid = &upperFrontPID;
                break;
              case '2' :
                Serial.print(F("URG "));
                pPid = &upperRearPID;
                break;
              default:
                Serial.print(F("DEBUG unknown command received for the 'G' command: "));
                Serial.println(receivedCommandBuffer[1]);
                break;
            }

            if (pPid != NULL)
            {

              Serial.print(pPid->GetKp(), 7);
              Serial.print(F(" "));
              Serial.print(pPid->GetKi(), 7);
              Serial.print(F(" "));
              Serial.print(pPid->GetKd(), 7);
              Serial.println(F(" "));
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
  handleIncomingCommands();
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

  PeriodicOutputTemps();
  handleRelayWatchdog();

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

