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

#include "FiniteStateMachine.h"
#include "TimerOne.h"
#include "EEPROM.h"
#include <avr/pgmspace.h>
#include "thermocouple.h"
#include "acInput.h"
#include "adcRead.h"
#include "pinDefinitions.h"
#include "relayBoost.h"
#include "relayDriver.h"

//------------------------------------------
// Macros
//------------------------------------------
#define PROTOCOL_MAJOR_VERSION   0
#define PROTOCOL_MINOR_VERSION   2003
#define PROTOCOL_BUGFIX_VERSION  0

//------------------------------------------
// Macros for Constants and Pin Definitions
//------------------------------------------

// For now just check cool down on fan
#define COOL_DOWN_EXIT_FAN_TEMP				((float)38.0)  // 100 degrees F
#define COOL_DOWN_EXIT_HEATER_TEMP		((float)65.0)  // 150 degrees F

// Timer1 Used to keep track of heat control cycles
#define TIMER1_PERIOD_MICRO_SEC			(1000) 	// timer1 1 mSec interval 
#define TIMER1_PERIOD_CLOCK_FACTOR	(1) 		// Clock multiplier for Timer1
#define TIMER1_COUNTER_WRAP				  (1000)  // Count down for a period of 1 second
#define TIMER1_OUTPUT_TEMP_PERIODIC (1000)  // Multiple of TIMER1_PERIOD_MICRO_SEC to output periodic temp

//------------------------------------------
//state machine setup
//------------------------------------------
void stateStandbyEnter();
void stateStandbyUpdate();
void stateStandbyExit();
State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

void stateTurnOnDlbEnter();
void stateTurnOnDlbUpdate();
void stateTurnOnDlbExit();
State stateTurnOnDlb = State(stateTurnOnDlbEnter, stateTurnOnDlbUpdate, stateTurnOnDlbExit);

void stateHeatCycleEnter();
void stateHeatCycleUpdate();
void stateHeatCycleExit();
State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);

void stateCoolDownEnter();
void stateCoolDownUpdate();
void stateCoolDownExit();
State stateCoolDown = State(stateCoolDownEnter, stateCoolDownUpdate, stateCoolDownExit);

FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby

//------------------------------------------
// Global Definitions
//------------------------------------------

// Timer one is running to control heat percentage and start
//  Re-synchronized on the start of heating
volatile uint16_t timer1Counter = 0;
uint16_t countOutputTempPeriodic = 0;

// Heater cycle time in multiples of 1 second
uint16_t nTimesCycleTime = 4;

struct HeaterParameters
{
  boolean enabled;
  union {
    struct {
      uint16_t tempSetPointLowOn;   // In integer degrees C
      uint16_t tempSetPointHighOff; // "      "
      uint16_t onPercent;       // Time when a heater turns on in percent
      uint16_t offPercent;      // Time when a heater turns off in percent
    };
    uint16_t parameterArray[4];
  };
};

HeaterParameters heaterParmsUpperFront = {true,   1800, 1900,  0, 80};
HeaterParameters heaterParmsUpperRear  = {true,   1800, 1900,  22, 100 };
HeaterParameters heaterParmsLowerFront = {true,   387,  399,   60, 100};
HeaterParameters heaterParmsLowerRear  = {true,   377,  388,   0,  33 };

uint16_t heaterCountsOnUpperFront;
uint16_t heaterCountsOffUpperFront;
uint16_t heaterCountsOnUpperRear;
uint16_t heaterCountsOffUpperRear;
uint16_t heaterCountsOnLowerFront;
uint16_t heaterCountsOffLowerFront;
uint16_t heaterCountsOnLowerRear;
uint16_t heaterCountsOffLowerRear;

// flags to keep the state of the heater hardware
RelayState heaterHardwareStateUpperFront = relayStateOff;
RelayState heaterHardwareStateUpperRear  = relayStateOff;
RelayState heaterHardwareStateLowerFront = relayStateOff;
RelayState heaterHardwareStateLowerRear  = relayStateOff;

// flags to keep the state of the heater cool down once reach High Off Set Point
bool heaterCoolDownStateUpperFront = false;
bool heaterCoolDownStateUpperRear  = false;
bool heaterCoolDownStateLowerFront = false;
bool heaterCoolDownStateLowerRear  = false;

volatile bool outputTempPeriodic = false;

float thermocoupleUpperFront = 0.0;
float thermocoupleUpperRear = 0.0;
float thermocoupleLowerFront = 0.0;
float thermocoupleLowerRear = 0.0;
float thermocoupleFan = 0.0;

//------------------------------------------
// Prototypes
//------------------------------------------
void ConvertHeaterPercentCounts();
void UpdateHeaterHardware();
void AllHeatersOffStateClear();
void CoolingFanControl(boolean control);
float AnalogThermocoupleTemp(uint16_t rawA2D);
float InputThermocoupleFan();
void readThermocouples(void);
void UpdateHeatControlUpperFront(uint16_t currentCounterTimer);
void UpdateHeatControlUpperRear(uint16_t currentCounterTimer);
void UpdateHeatControlLowerFront(uint16_t currentCounterTimer);
void UpdateHeatControlLowerRear(uint16_t currentCounterTimer);
void PeriodicOutputTemps();
bool CharValidDigit(unsigned char digit);
uint16_t GetInputValue();
void HeaterTimerInterrupt();
float readAD8495KTC(uint8_t pin);
void handleRelayWatchdog(void);

void readThermocouples(void)
{
  static uint8_t nextReading = 0;
  // TODO: Don't make this blocking.  Interact with the A2D
  //       registers directly.  Interleaving the TC readings
  //       with the relay watchdogs is a total kludge.
  handleRelayWatchdog();
  switch (nextReading++) {
    case 0:
      thermocoupleUpperFront = readAD8495KTC(ANALOG_THERMO_UPPER_FRONT);
      break;
    case 1:
      thermocoupleUpperRear = readAD8495KTC(ANALOG_THERMO_UPPER_REAR);
      break;
    case 2:
      thermocoupleLowerFront = readAD8495KTC(ANALOG_THERMO_LOWER_FRONT);
      break;
    case 3:
      thermocoupleLowerRear = readAD8495KTC(ANALOG_THERMO_LOWER_REAR);
      break;
    case 4:
      thermocoupleFan = InputThermocoupleFan();
      nextReading = 0;
      break;
  }
}

//------------------------------------------
// Code
//------------------------------------------

void ConvertHeaterPercentCounts()
{
  heaterCountsOnUpperFront  = (uint16_t)(((uint32_t)heaterParmsUpperFront.onPercent  * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;
  heaterCountsOffUpperFront = (uint16_t)(((uint32_t)heaterParmsUpperFront.offPercent * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;

  heaterCountsOnUpperRear   = (uint16_t)(((uint32_t)heaterParmsUpperRear.onPercent   * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;
  heaterCountsOffUpperRear  = (uint16_t)(((uint32_t)heaterParmsUpperRear.offPercent  * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;

  heaterCountsOnLowerFront  = (uint16_t)(((uint32_t)heaterParmsLowerFront.onPercent  * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;
  heaterCountsOffLowerFront = (uint16_t)(((uint32_t)heaterParmsLowerFront.offPercent * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;

  heaterCountsOnLowerRear   = (uint16_t)(((uint32_t)heaterParmsLowerRear.onPercent   * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;
  heaterCountsOffLowerRear  = (uint16_t)(((uint32_t)heaterParmsLowerRear.offPercent  * TIMER1_COUNTER_WRAP + 50) / 100) * nTimesCycleTime;
}

void UpdateHeaterHardware()
{
  changeRelayState(HEATER_ENABLE_UPPER_FRONT, heaterHardwareStateUpperFront);
  changeRelayState(HEATER_ENABLE_UPPER_REAR, heaterHardwareStateUpperRear);
  changeRelayState(HEATER_ENABLE_LOWER_FRONT, heaterHardwareStateLowerFront);
  changeRelayState(HEATER_ENABLE_LOWER_REAR, heaterHardwareStateLowerRear);
}
void AllHeatersOffStateClear()
{
  heaterHardwareStateUpperFront = relayStateOff;
  heaterHardwareStateUpperRear  = relayStateOff;
  heaterHardwareStateLowerFront = relayStateOff;
  heaterHardwareStateLowerRear  = relayStateOff;

  heaterCoolDownStateUpperFront = false;
  heaterCoolDownStateUpperRear  = false;
  heaterCoolDownStateLowerFront = false;
  heaterCoolDownStateLowerRear  = false;

  UpdateHeaterHardware();

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOff);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOff);
}

void CoolingFanControl(boolean control)
{
  static bool lastControl = !control;

  if (lastControl != control)
  {
    if (control == true)
    {
      changeRelayState(COOLING_FAN_SIGNAL, relayStateOn);
    }
    else
    {
      changeRelayState(COOLING_FAN_SIGNAL, relayStateOff);
    }
    lastControl = control;
  }
}

float AnalogThermocoupleTemp(uint16_t rawA2D)
{
  float analogVoltage;
  float tempC;

  analogVoltage = (float)rawA2D * ANALOG_REFERENCE_VOLTAGE / 1023.0;
  tempC = (analogVoltage - 1.25)  / 0.005;
  return tempC;
}

float InputThermocoupleFan()
{
  float tempC;

  //	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_FAN));

  tempC = 0;//AD8495KTCInterpolate(AnalogTCVolts(analogRead(ANALOG_THERMO_FAN)));
  return tempC;
}

void UpdateHeatControlUpperFront(uint16_t currentCounterTimer)
{
  float tempC;
  uint16_t icase = 0;

  heaterHardwareStateUpperFront = relayStateOff;
  if ((heaterParmsUpperFront.enabled == true) &&
      (currentCounterTimer >= heaterCountsOnUpperFront) &&
      (currentCounterTimer <= heaterCountsOffUpperFront))
  {
    tempC = thermocoupleUpperFront;

    // If not in cool down and less than High Set Point Turn on Heater
    if ((heaterCoolDownStateUpperFront == false) && (tempC < (float)heaterParmsUpperFront.tempSetPointHighOff))
    {
      icase = 1;
      heaterHardwareStateUpperFront = relayStateOn;
    }
    // If not in cool down and greater than High Set Point turn off heater and set cool down
    else if ((heaterCoolDownStateUpperFront == false) && (tempC >= (float)heaterParmsUpperFront.tempSetPointHighOff))
    {
      icase = 2;
      heaterCoolDownStateUpperFront = true;
      heaterHardwareStateUpperFront = relayStateOff;
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else if ((heaterCoolDownStateUpperFront == true) && (tempC <= (float)heaterParmsUpperFront.tempSetPointLowOn))
    {
      icase = 3;
      heaterCoolDownStateUpperFront = false;
      heaterHardwareStateUpperFront = relayStateOn;
    }
    else // In cool down but have not reached the Low Set Point
    {
      icase = 4;
      heaterHardwareStateUpperFront = relayStateOff;
    }
  }
  else	// Heater Disabled or Outside the percentage limits of the cycle
  {
    icase = 9;
    heaterHardwareStateUpperFront = relayStateOff;
  }
}

void UpdateHeatControlUpperRear(uint16_t currentCounterTimer)
{
  float tempC;
  uint16_t icase = 0;  // For case debug

  heaterHardwareStateUpperRear = relayStateOff;
  if ((heaterParmsUpperRear.enabled == true) &&
      (currentCounterTimer >= heaterCountsOnUpperRear) &&
      (currentCounterTimer <= heaterCountsOffUpperRear))
  {
    tempC = thermocoupleUpperRear;

    // If not in cool down and less than High Set Point Turn on Heater
    if ((heaterCoolDownStateUpperRear == false) && (tempC < (float)heaterParmsUpperRear.tempSetPointHighOff))
    {
      icase = 11;
      heaterHardwareStateUpperRear = relayStateOn;
    }
    // If not in cool down and greater than High Set Point turn off heater and set cool down
    else if ((heaterCoolDownStateUpperRear == false) && (tempC >= (float)heaterParmsUpperRear.tempSetPointHighOff))
    {
      icase = 12;
      heaterCoolDownStateUpperRear = true;
      heaterHardwareStateUpperRear = relayStateOff;
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else if ((heaterCoolDownStateUpperRear == true) && (tempC <= (float)heaterParmsUpperRear.tempSetPointLowOn))
    {
      icase = 13;
      heaterCoolDownStateUpperRear = false;
      heaterHardwareStateUpperRear = relayStateOn;
    }
    else // In cool down but have not reached the Low Set Point
    {
      icase = 14;
      heaterHardwareStateUpperRear = relayStateOff;
    }
  }
  else	// Heater Disabled or Outside the percentage limits of the cycle
  {
    icase = 19;
    heaterHardwareStateUpperRear = relayStateOff;
  }
}

void UpdateHeatControlLowerFront(uint16_t currentCounterTimer)
{
  float tempC;
  uint16_t icase = 0;  // For case debug

  heaterHardwareStateLowerFront = relayStateOff;
  if ((heaterParmsLowerFront.enabled == true) &&
      (currentCounterTimer >= heaterCountsOnLowerFront) &&
      (currentCounterTimer <= heaterCountsOffLowerFront))
  {
    tempC = thermocoupleLowerFront;

    // If not in cool down and less than High Set Point Turn on Heater
    if ((heaterCoolDownStateLowerFront == false) && (tempC < (float)heaterParmsLowerFront.tempSetPointHighOff))
    {
      icase = 21;
      heaterHardwareStateLowerFront = relayStateOn;
    }
    // If not in cool down and greater than High Set Point turn off heater and set cool down
    else if ((heaterCoolDownStateLowerFront == false) && (tempC >= (float)heaterParmsLowerFront.tempSetPointHighOff))
    {
      icase = 22;
      heaterCoolDownStateLowerFront = true;
      heaterHardwareStateLowerFront = relayStateOff;
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else if ((heaterCoolDownStateLowerFront == true) && (tempC <= (float)heaterParmsLowerFront.tempSetPointLowOn))
    {
      icase = 23;
      heaterCoolDownStateLowerFront = false;
      heaterHardwareStateLowerFront = relayStateOn;
    }
    else // In cool down but have not reached the Low Set Point
    {
      icase = 24;
      heaterHardwareStateLowerFront = relayStateOff;
    }
  }
  else	// Heater Disabled or Outside the percentage limits of the cycle
  {
    icase = 29;
    heaterHardwareStateLowerFront = relayStateOff;
  }
}

void UpdateHeatControlLowerRear(uint16_t currentCounterTimer)
{
  float tempC;
  uint16_t icase = 0;  // For case debug

  heaterHardwareStateLowerRear = relayStateOff;
  if ((heaterParmsLowerRear.enabled == true) &&
      (currentCounterTimer >= heaterCountsOnLowerRear) &&
      (currentCounterTimer <= heaterCountsOffLowerRear))
  {
    tempC = thermocoupleLowerRear;

    // If not in cool down and less than High Set Point Turn on Heater
    if ((heaterCoolDownStateLowerRear == false) && (tempC < (float)heaterParmsLowerRear.tempSetPointHighOff))
    {
      icase = 31;
      heaterHardwareStateLowerRear = relayStateOn;
    }
    // If not in cool down and greater than High Set Point turn off heater and set cool down
    else if ((heaterCoolDownStateLowerRear == false) && (tempC >= (float)heaterParmsLowerRear.tempSetPointHighOff))
    {
      icase = 32;
      heaterCoolDownStateLowerRear = true;
      heaterHardwareStateLowerRear = relayStateOff;
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else if ((heaterCoolDownStateLowerRear == true) && (tempC <= (float)heaterParmsLowerRear.tempSetPointLowOn))
    {
      icase = 33;
      heaterCoolDownStateLowerRear = false;
      heaterHardwareStateLowerRear = relayStateOn;
    }
    else // In cool down but have not reached the Low Set Point
    {
      icase = 34;
      heaterHardwareStateLowerRear = relayStateOff;
    }
  }
  else	// Heater Disabled or Outside the percentage limits of the cycle
  {
    icase = 39;
    heaterHardwareStateLowerRear = relayStateOff;
  }
}

void PeriodicOutputTemps()
{
  uint8_t strLen;
  char formatStr[25];
  uint16_t intTempCUF, intTempCUR, intTempCLF, intTempCLR, intTempCFan;

  // Output Periodic Temperatures
  if (true == outputTempPeriodic)
  {
    outputTempPeriodic = false;

    intTempCUF =  (uint16_t) (thermocoupleUpperFront + 0.5);
    intTempCUR =  (uint16_t) (thermocoupleUpperRear  + 0.5);
    intTempCLF =  (uint16_t) (thermocoupleLowerFront + 0.5);
    intTempCLR =  (uint16_t) (thermocoupleLowerRear  + 0.5);
    //    intTempCFan = (uint16_t) (thermocoupleFan		+ 0.5);
    intTempCFan = getA2DReadingForPin(ANALOG_THERMO_FAN);

    Serial.print("Temps ");
    Serial.print(intTempCUF);
    Serial.print(" ");
    Serial.print(intTempCUR);
    Serial.print(" ");
    Serial.print(intTempCLF);
    Serial.print(" ");
    Serial.print(intTempCLR);
    Serial.print(" ");
    Serial.println(intTempCFan);

    Serial.print("Power ");
    Serial.print(powerButtonIsOn());
    Serial.print(" L2DLB ");
    Serial.println(l2DlbIsOn());
  }
}

//------------------------------------------
// Update Heater State Interrupt
//------------------------------------------
void HeaterTimerInterrupt()
{
  timer1Counter++;

  if (timer1Counter > (TIMER1_COUNTER_WRAP * nTimesCycleTime))
  {
    timer1Counter = 0;
  }

  countOutputTempPeriodic++;

  if (countOutputTempPeriodic >= TIMER1_OUTPUT_TEMP_PERIODIC)
  {
    countOutputTempPeriodic = 0;
    outputTempPeriodic = true;
  }
}

void initializeRelayPin(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  relayDriverInit(pin, relayStateOff);
}

//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  setupAcInputs();

  Serial.begin(9600);
  Serial.println("BLE Arduino Slave");

  // Initialize Timer1
  Timer1.initialize(TIMER1_PERIOD_MICRO_SEC * TIMER1_PERIOD_CLOCK_FACTOR);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10);
  Timer1.attachInterrupt(HeaterTimerInterrupt);

  // Setup Cooling Fan as Output and Turn Off
  initializeRelayPin(COOLING_FAN_SIGNAL);

  // Setup Heater Enables as Outputs and Turn Off
  initializeRelayPin(HEATER_ENABLE_UPPER_FRONT);
  initializeRelayPin(HEATER_ENABLE_UPPER_REAR);
  initializeRelayPin(HEATER_ENABLE_LOWER_FRONT);
  initializeRelayPin(HEATER_ENABLE_LOWER_REAR);

  // Setup triac DLB relays
  initializeRelayPin(HEATER_UPPER_FRONT_DLB);
  initializeRelayPin(HEATER_UPPER_REAR_DLB);

  pinMode(TEN_V_ENABLE, OUTPUT);
  digitalWrite(TEN_V_ENABLE, LOW);
  pinMode(BOOST_ENABLE, OUTPUT);
  digitalWrite(BOOST_ENABLE, LOW);
  pinMode(RELAY_WATCHDOG, OUTPUT);

  // Initialize the A2D engine
  adcReadInit(ANALOG_THERMO_UPPER_FRONT);
  adcReadInit(ANALOG_THERMO_UPPER_REAR);
  adcReadInit(ANALOG_THERMO_LOWER_FRONT);
  adcReadInit(ANALOG_THERMO_LOWER_REAR);
  adcReadInit(ANALOG_THERMO_FAN);

  ConvertHeaterPercentCounts();
  Serial.println("Start");
  //  if (poStateMachine.isInState(stateStandby) ||
  //      poStateMachine.isInState(stateCoolDown))
  //  {
  //    poStateMachine.transitionTo(stateTurnOnDlb);
  //  }
  //  else
  //  {
  //    Serial.println("Invalid");
  //  }
}

void handleRelayWatchdog(void)
{
  static unsigned long oldTime = micros();
  unsigned long newTime = micros();

  if ((newTime < oldTime) || ((newTime - oldTime) >= 500))
  {
    digitalWrite(RELAY_WATCHDOG, !digitalRead(RELAY_WATCHDOG));
    oldTime = newTime;
  }
}

void printHeaterTemperatureParameters(char *pName, uint16_t *pParams)
{
  uint8_t i;

  Serial.print(pName);
  for (i = 0; i < 4; i++)
  {
    Serial.print(pParams[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

#define RECEVIED_COMMAND_BUFFER_LENGTH (16)

void handleIncomingCommands(void)
{
  static char receivedCommandBuffer[RECEVIED_COMMAND_BUFFER_LENGTH];
  static uint8_t receivedCommandBufferIndex = 0;

  if (Serial.available() > 0)
  {
    receivedCommandBuffer[receivedCommandBufferIndex++] = Serial.read();
    if (receivedCommandBufferIndex >= RECEVIED_COMMAND_BUFFER_LENGTH)
    {
      Serial.println("DEBUG command input buffer exceeded.");
      receivedCommandBufferIndex = 0;
    }
    else
    {
      // attempt to parse the command
      switch (receivedCommandBuffer[0])
      {
        case 'v': // query protocol version
          Serial.print("V ");
          Serial.print(PROTOCOL_MAJOR_VERSION);
          Serial.print(".");
          Serial.print(PROTOCOL_MINOR_VERSION);
          Serial.print(" bugfix ");
          Serial.println(PROTOCOL_BUGFIX_VERSION);
          receivedCommandBufferIndex = 0;
          break;

        case 'p': // query heat control parameters
          if (receivedCommandBufferIndex >= 2)
          {
            switch (receivedCommandBuffer[1])
            {
              case '0' :
                //              strLen = sprintf(formatStr, "nTimes %u\n",
                //                               nTimesCycleTime);
                Serial.print("nTimes ");
                Serial.println(nTimesCycleTime);
                break;
              case '1' :
                printHeaterTemperatureParameters("UF ", heaterParmsUpperFront.parameterArray);
                break;
              case '2' :
                printHeaterTemperatureParameters("UR ", heaterParmsUpperRear.parameterArray);
                break;
              case '3' :
                printHeaterTemperatureParameters("LF ", heaterParmsLowerFront.parameterArray);
                break;
              case '4' :
                printHeaterTemperatureParameters("LR ", heaterParmsLowerRear.parameterArray);
                break;
              default:
                Serial.print("DEBUG unknown command received for the 'p' command: ");
                Serial.println(receivedCommandBuffer[1]);
                break;
            }

            receivedCommandBufferIndex = 0;
          }
          break;

        case 10:
        case 13:
          // ignore cr/lf as a command
          receivedCommandBufferIndex = 0;
          break;

        default:
          Serial.print("DEBUG unknown command received: ");
          Serial.println(receivedCommandBuffer[0]);
          receivedCommandBufferIndex = 0;
          break;
      }
    }
  }
}

//------------------------------------------
// Main Loop
//------------------------------------------
byte queryDone = false;
uint32_t liveCount = 0;
uint16_t inputValue;
uint16_t tempMultiply, tempPercent, tempTemp;

void loop()
{
  char formatStr[25];
  uint16_t strLen;

#ifdef KILL

  // Process Blue Tooth Command if available
  // TBD ble_available returns -1 if nothing available
  if (Serial.available() > 0)
  {
    byte cmd;
    cmd = Serial.read();
    //Serial.write(cmd); <-- no need to echo command without BLE

    // Parse data here
    switch (cmd)
    {

      //case 's':	// Start Pizza Oven Cycle


      /*
          break;

        case 'q':	// Quit Pizza Oven Cycle
        Serial.println("Exit");
        if(poStateMachine.isInState(stateHeatCycle))
        {
      	poStateMachine.transitionTo(stateCoolDown);
        }
        else
        {
      	Serial.println("Invalid");
        }
          break;

        case 'l':	// Set Lower Set Point Parameter
        Serial.println("Set Lower Set Point Parameter");
        if(ble_available() > 2)
        {
          cmd = ble_read();
      	Serial.write(cmd);

      	// TBD Validate Range
      	switch(cmd)
      	{
      	case '1' :
      		heaterParmsUpperFront.tempSetPointLowOn = GetInputValue();
      		break;
      	case '2' :
      		heaterParmsUpperRear.tempSetPointLowOn = GetInputValue();
      		break;
      	case '3' :
      		heaterParmsLowerFront.tempSetPointLowOn = GetInputValue();
      		break;
      	case '4' :
      		heaterParmsLowerRear.tempSetPointLowOn = GetInputValue();
      		break;
      	default :
      		;
      	}
        }
          break;

        case 'u':	// Set Upper Set Point Parameter
        Serial.println("Set Upper Set Point Parameter");
        if(ble_available() > 2)
        {
          cmd = ble_read();
      	Serial.write(cmd);

      	// TBD Validate Range
      	switch(cmd)
      	{
      	case '1' :
      		heaterParmsUpperFront.tempSetPointHighOff = GetInputValue();
      		break;
      	case '2' :
      		heaterParmsUpperRear.tempSetPointHighOff = GetInputValue();
      		break;
      	case '3' :
      		heaterParmsLowerFront.tempSetPointHighOff = GetInputValue();
      		break;
      	case '4' :
      		heaterParmsLowerRear.tempSetPointHighOff = GetInputValue();
      		break;
      	default :
      		;
      	}
        }
          break;

        case 'n':	// Set On Percent Parameter
        Serial.println("Set On Percent Parameter");
        if(ble_available() > 2)
        {
          cmd = ble_read();
      	Serial.write(cmd);

      	// TBD Validate Range
      	switch(cmd)
      	{
      	case '1' :
      		heaterParmsUpperFront.onPercent = GetInputValue();
      		break;
      	case '2' :
      		heaterParmsUpperRear.onPercent = GetInputValue();
      		break;
      	case '3' :
      		heaterParmsLowerFront.onPercent = GetInputValue();
      		break;
      	case '4' :
      		heaterParmsLowerRear.onPercent = GetInputValue();
      		break;
      	default :
      		;
      	}
        }
          break;

        case 'f':	// Set Off Percent Parameter
        Serial.println("Set Off Percent Parameter");
        if(ble_available() > 2)
        {
          cmd = ble_read();
      	Serial.write(cmd);

      	// TBD Validate Range
      	switch(cmd)
      	{
      	case '1' :
      		heaterParmsUpperFront.offPercent = GetInputValue();
      		break;
      	case '2' :
      		heaterParmsUpperRear.offPercent = GetInputValue();
      		break;
      	case '3' :
      		heaterParmsLowerFront.offPercent = GetInputValue();
      		break;
      	case '4' :
      		heaterParmsLowerRear.offPercent = GetInputValue();
      		break;
      	default :
      		;
      	}
        }
          break;

        case 't':	// Set Time Multiplier Minutes
        	if(poStateMachine.isInState(stateStandby) ||
      	poStateMachine.isInState(stateCoolDown))
        {
      	Serial.println("Set Time Period Cycle Seconds");
      	if(ble_available() >= 1)
      	{
      		tempMultiply = GetInputValue();
      		if((tempMultiply > 0) && (tempMultiply <= 30))
      		{
      			nTimesCycleTime = tempMultiply;
      			ConvertHeaterPercentCounts();
      		}
      		else
      		{
        //		    		ble_write_bytes((byte *)"Invalid n", 9);
          		Serial.println("Invalid n");
          	}
        	}
        }
        else
        {
        //    		ble_write_bytes((byte *)"Invalid Heat", 12);
      		Serial.println("Invalid Heat");
      	}
        break;

        default:
        ;
      	}
        }
      */
      default:
        Serial.print("DEBUG unknown command: ");
        Serial.println(cmd);
    }
  }

#endif

  handleIncomingCommands();
  poStateMachine.update();

  adcReadRun();
  readThermocouples();
  runAcInputs();
  PeriodicOutputTemps();
  boostEnable(relayBoostRun);
  relayDriverRun();
  handleRelayWatchdog();

}

bool CharValidDigit(unsigned char digit)
{
  if ((digit >= '0') && (digit <= '9'))
    return true;
  else
    return false;
}

uint16_t GetInputValue()
{
  uint16_t inputValue = 0;
  unsigned char cBytesAvailable;
  byte nBytesAvailable;
  unsigned char digit;
  uint16_t loop;

  //	cBytesAvailable = ble_available();
  //	Serial.println((uint16_t)cBytesAvailable);
  if (cBytesAvailable > 0)
  {
    nBytesAvailable = (byte)cBytesAvailable;
    // limit to 4 digits at most
    if (nBytesAvailable > 4)
    {
      nBytesAvailable = 4;
    }

    for (loop = 0; loop < nBytesAvailable; loop++)
    {
      //			digit = ble_read();
      if (CharValidDigit(digit))
      {
        inputValue *= 10;
        inputValue += (uint16_t)(digit - '0');
      }
      else
      {
        inputValue = 0;
        break;
      }
    }
  }
  return inputValue;
}

//------------------------------------------
//state machine stateStandby
//------------------------------------------
//State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

void stateStandbyEnter()
{
  Serial.println("stateStandbyEnter");
  digitalWrite(TEN_V_ENABLE, LOW);
}

void stateStandbyUpdate()
{
  if (powerButtonIsOn())
  {
    poStateMachine.transitionTo(stateTurnOnDlb);
  }
}

void stateStandbyExit()
{
  Serial.println("SX");
}

/*
   State Machine stateTurnOnDlb
*/
//State stateTurnOnDlb = State(stateTurnOnDlbEnter, stateTurnOnDlbUpdate, stateTurnOnDlbExit);
void stateTurnOnDlbEnter(void)
{
  Serial.println("Entering stateTurnOnDlb.");
  digitalWrite(TEN_V_ENABLE, HIGH);  //Turn on 10V supply
  CoolingFanControl(true);
}

void stateTurnOnDlbUpdate(void)
{
  if (!powerButtonIsOn())
  {
    poStateMachine.transitionTo(stateStandby);
  }
  else if (l2DlbIsOn())
  {
    poStateMachine.transitionTo(stateHeatCycle);
  }
}

void stateTurnOnDlbExit(void)
{
  Serial.println("Exiting stateTurnOnDlb.");
}

//------------------------------------------
//state machine stateHeatCycle
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
uint16_t saveTimer1Counter, lastTimer1Counter;

void stateHeatCycleEnter()
{
  Serial.println("stateHeatCycleEnter");

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  timer1Counter = 0;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);
}

void stateHeatCycleUpdate()
{
  //    if((liveCount % 100) == 0)
  //      Serial.println("HU");

  // Save working value timer1Counter since can be updated by interrupt
  saveTimer1Counter = timer1Counter;

  // Only update heat control if Timer1 Counter changed

  if (saveTimer1Counter != lastTimer1Counter)
  {
    CoolingFanControl(true);
    UpdateHeatControlUpperFront(saveTimer1Counter);
    UpdateHeatControlUpperRear(saveTimer1Counter);
    UpdateHeatControlLowerFront(saveTimer1Counter);
    UpdateHeatControlLowerRear(saveTimer1Counter);

    UpdateHeaterHardware();

    lastTimer1Counter = saveTimer1Counter;
  }

  if (!powerButtonIsOn() || !l2DlbIsOn())
  {
    poStateMachine.transitionTo(stateCoolDown);
  }
}

void stateHeatCycleExit()
{
  Serial.println("HX");
  AllHeatersOffStateClear();
}

//------------------------------------------
//state machine stateCoolDown
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter,
//	stateCoolDownUpdate, stateCoolDownExit);

void stateCoolDownEnter()
{
  Serial.println("stateCoolDown");
}

void stateCoolDownUpdate()
{
  //    if((liveCount % 100000) == 0)
  //      Serial.println("CU");

  // For now just check cool down on fan
  if ((thermocoupleFan <= COOL_DOWN_EXIT_FAN_TEMP) &&
      (thermocoupleUpperFront <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (thermocoupleUpperRear  <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (thermocoupleLowerFront <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (thermocoupleLowerRear  <= COOL_DOWN_EXIT_HEATER_TEMP))
  {
    CoolingFanControl(false);
    poStateMachine.transitionTo(stateStandby);
  }
}

void stateCoolDownExit()
{
  Serial.println("CX");
}
