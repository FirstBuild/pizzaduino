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
#include <avr/pgmspace.h>
#include "thermocouple.h"
#include "acInput.h"
#include "adcRead.h"
#include "pinDefinitions.h"
#include "relayBoost.h"
#include "relayDriver.h"
#include "pizzaMemory.h"
#include "PID_v1.h"
#include <limits.h>

#define USE_PID
#define ENABLE_PID_TUNING

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
#define COOL_DOWN_EXIT_FAN_TEMP				((double)100.0)  // 100 degrees F
#define COOL_DOWN_EXIT_HEATER_TEMP		((double)150.0)  // 150 degrees F

// Timer1 Used to keep track of heat control cycles
#define TIMER1_PERIOD_MICRO_SEC			(1000) 	// timer1 1 mSec interval 
#define TIMER1_PERIOD_CLOCK_FACTOR	(1) 		// Clock multiplier for Timer1
#define MILLISECONDS_PER_SECOND		  (1000)  // Count down for a period of 1 second
//#define TIMER1_OUTPUT_TEMP_PERIODIC (1000)  // Multiple of TIMER1_PERIOD_MICRO_SEC to output periodic temp
#define TIMER1_OUTPUT_TEMP_PERIODIC (500)  // Multiple of TIMER1_PERIOD_MICRO_SEC to output periodic temp

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

bool pizzaOvenStartRequested = false;
bool pizzaOvenStopRequested = false;

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

static bool TCsHaveBeenInitialized = false;

// From: http://www.schwietering.com/jayduino/filtuino/
// Low pass bessel filter order=2 alpha1=0.005, 0.25 hz corner freq
//class  FilterBeLp2
//{
//  public:
//    FilterBeLp2()
//    {
//      v[0] = 0.0;
//      v[1] = 0.0;
//    }
//  private:
//    float v[3];
//  public:
//    float step(float x) //class II
//    {
//      v[0] = v[1];
//      v[1] = v[2];
//      v[2] = (3.857929638054202206e-4 * x)
//             + (-0.93312011662265637035 * v[0])
//             + (1.93157694476743468925 * v[1]);
//      return
//        (v[0] + v[2])
//        + 2 * v[1];
//    }
//};

//Low pass bessel filter order=2 alpha1=0.0025, 0.125  hz corner freq
class  FilterBeLp2
{
  public:
    FilterBeLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (9.810514574132291022e-5 * x)
         + (-0.96598348806398948163 * v[0])
         + (1.96559106748102419004 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }
};

typedef struct Heater
{
  HeaterParameters parameter;
  uint32_t heaterCountsOn;
  uint32_t heaterCountsOff;
  RelayState relayState;
  bool heaterCoolDownState;
  double thermocouple;
  FilterBeLp2 tcFilter;
};

//Heater upperFrontHeater = {{true, 1200, 1300,   0,  70}, 0, 0, relayStateOff, false, 0.0};
Heater upperFrontHeater = {{true, 900, 1100,   0,  50}, 0, 0, relayStateOff, false, 0};
Heater upperRearHeater  = {{false, 1100, 1200,  45, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerFrontHeater = {{false,  600,  650,  71, 100}, 0, 0, relayStateOff, false, 0};
Heater lowerRearHeater  = {{false,  575,  625,   0,  40}, 0, 0, relayStateOff, false, 0};
Heater dummyHeater;

// convenience array, could go into flash
Heater *aHeaters[5] =
{
  &dummyHeater,
  &upperFrontHeater,
  &upperRearHeater,
  &lowerFrontHeater,
  &lowerRearHeater
};

// PID stuff
double Setpoint = 1000;
double Output = 47;
//  run 2 settings PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.0140072, 0.0005294, 0, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.05, 0.0008925, 0.1479627, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.942, 0.17084, 0.04271, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.4, 0.015, 0.02, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.6875, 0, 0, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 9.78256766574047, 17.1603333333337, 4.29008333333343, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 5.96380722985162, 17.1603333333337, 4.29008333333343, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.1927678, 0.0031192, 2.9782624, DIRECT);
//Time KP KI KD Raw Temp % Set Tuning, 797040, 0.1407757, 0.0017002, 2.9140563, 1025.46, 1024, 53.89, 1000.00, 0
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.1375, 16.75, 3.9, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.1375, 0.07, 4.43625, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 0.8, 0.008, 1.5, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.1375, 0.01, 4.4, DIRECT);
//PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.1375, 0.009, 2.2, DIRECT);
PID upperFrontPID(&upperFrontHeater.thermocouple, &Output, &Setpoint, 1.0, 0.0, 0.0, DIRECT);

volatile bool outputTempPeriodic = false;

double thermocoupleFan = 0.0;

//------------------------------------------
// Prototypes
//------------------------------------------
void ConvertHeaterPercentCounts();
void UpdateHeaterHardware();
void AllHeatersOffStateClear();
void CoolingFanControl(boolean control);
double AnalogThermocoupleTemp(uint16_t rawA2D);
double InputThermocoupleFan();
void readThermocouples(void);
void PeriodicOutputTemps();
bool CharValidDigit(unsigned char digit);
uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf);
void HeaterTimerInterrupt();
void handleRelayWatchdog(void);
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
  thermocoupleFan = InputThermocoupleFan();
  TCsHaveBeenInitialized = true;
}

//------------------------------------------
// Code
//------------------------------------------

void ConvertHeaterPercentCounts()
{
  upperFrontHeater.heaterCountsOn  = (uint16_t)(((uint32_t)upperFrontHeater.parameter.onPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;
  upperFrontHeater.heaterCountsOff = (uint16_t)(((uint32_t)upperFrontHeater.parameter.offPercent * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;

  upperRearHeater.heaterCountsOn   = (uint16_t)(((uint32_t)upperRearHeater.parameter.onPercent   * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;
  upperRearHeater.heaterCountsOff  = (uint16_t)(((uint32_t)upperRearHeater.parameter.offPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * triacPeriodSeconds;

  lowerFrontHeater.heaterCountsOn  = (uint16_t)(((uint32_t)lowerFrontHeater.parameter.onPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
  lowerFrontHeater.heaterCountsOff = (uint16_t)(((uint32_t)lowerFrontHeater.parameter.offPercent * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;

  lowerRearHeater.heaterCountsOn   = (uint16_t)(((uint32_t)lowerRearHeater.parameter.onPercent   * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
  lowerRearHeater.heaterCountsOff  = (uint16_t)(((uint32_t)lowerRearHeater.parameter.offPercent  * MILLISECONDS_PER_SECOND + 50) / 100) * relayPeriodSeconds;
}

void UpdateHeaterHardware()
{
  changeRelayState(HEATER_ENABLE_UPPER_FRONT, upperFrontHeater.relayState);
  changeRelayState(HEATER_ENABLE_UPPER_REAR, upperRearHeater.relayState);
  changeRelayState(HEATER_ENABLE_LOWER_FRONT, lowerFrontHeater.relayState);
  changeRelayState(HEATER_ENABLE_LOWER_REAR, lowerRearHeater.relayState);
}
void AllHeatersOffStateClear()
{
  upperFrontHeater.relayState = relayStateOff;
  upperRearHeater.relayState  = relayStateOff;
  lowerFrontHeater.relayState = relayStateOff;
  lowerRearHeater.relayState  = relayStateOff;

  upperFrontHeater.heaterCoolDownState = false;
  upperRearHeater.heaterCoolDownState  = false;
  lowerFrontHeater.heaterCoolDownState = false;
  lowerRearHeater.heaterCoolDownState  = false;

  UpdateHeaterHardware();

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOff);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOff);
}

void CoolingFanControl(boolean control)
{
  static bool lastControl = !control;

  if (lastControl != control)
  {
    Serial.println("DEBUG Changing the state of the cooling fan.");
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

double AnalogThermocoupleTemp(uint16_t rawA2D)
{
  double analogVoltage;
  double tempC;

  analogVoltage = (double)rawA2D * ANALOG_REFERENCE_VOLTAGE / 1023.0;
  tempC = (analogVoltage - 1.25)  / 0.005;
  return tempC;
}

double InputThermocoupleFan()
{
  double tempC;

  //	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_FAN));

  tempC = 0;//AD8495KTCInterpolate(AnalogTCVolts(analogRead(ANALOG_THERMO_FAN)));
  return tempC;
}

void UpdateHeatControl(Heater *pHeater, uint16_t currentCounterTimer)
{
  double temperature;

  pHeater->relayState = relayStateOff;
  if ((pHeater->parameter.enabled == true) &&
      (currentCounterTimer >= pHeater->heaterCountsOn) &&
      (currentCounterTimer <= pHeater->heaterCountsOff))
  {
    temperature = pHeater->thermocouple;

    // If not in cool down and less than High Set Point Turn on Heater
    if (pHeater->heaterCoolDownState == false)
    {
      pHeater->relayState = relayStateOn;
      if (temperature >= (double)pHeater->parameter.tempSetPointHighOff)
      {
        pHeater->heaterCoolDownState = true;
        pHeater->relayState = relayStateOff;
      }
    }
    // If in cool down and less than equal than low set point, exit cool down and turn heater on
    else
    {
      pHeater->relayState = relayStateOff;
      if (temperature <= (double)pHeater->parameter.tempSetPointLowOn)
      {
        pHeater->heaterCoolDownState = false;
        pHeater->relayState = relayStateOn;
      }
    }
  }
  else  // Heater Disabled or Outside the percentage limits of the cycle
  {
    pHeater->relayState = relayStateOff;
  }
}

void UpdateHeatControlWithPID(Heater *pHeater, uint16_t currentCounterTimer)
{
  double temperature;

  pHeater->relayState = relayStateOff;
  if ((pHeater->parameter.enabled == true) &&
      (currentCounterTimer >= pHeater->heaterCountsOn) &&
      (currentCounterTimer <= pHeater->heaterCountsOff))
  {
    pHeater->relayState = relayStateOn;
  }
  else  // Heater Disabled or Outside the percentage limits of the cycle
  {
    pHeater->relayState = relayStateOff;
  }
}

void outputAcInputStates()
{
  Serial.print(F("Power "));
  Serial.print(powerButtonIsOn());
  Serial.print(F(" L2DLB "));
  Serial.println(l2DlbIsOn());
}

void PeriodicOutputTemps()
{
  uint8_t strLen;
  char formatStr[25];
  uint16_t intTempCUF, intTempCUR, intTempCLF, intTempCLR, intTempCFan;

#define TEMP_BUF_SIZE 10
  static float tempBuf[TEMP_BUF_SIZE];
  static uint8_t bufIndex = 0;
  uint8_t compareIndex;
  float diff;

  // Output Periodic Temperatures
  if (true == outputTempPeriodic)
  {
    outputTempPeriodic = false;

    intTempCUF =  (uint16_t) (upperFrontHeater.thermocouple + 0.5);
    intTempCUR =  (uint16_t) (upperRearHeater.thermocouple  + 0.5);
    intTempCLF =  (uint16_t) (lowerFrontHeater.thermocouple + 0.5);
    intTempCLR =  (uint16_t) (lowerRearHeater.thermocouple  + 0.5);
    //    intTempCFan = (uint16_t) (thermocoupleFan		+ 0.5);
    intTempCFan = getA2DReadingForPin(ANALOG_THERMO_FAN);

#ifndef ENABLE_PID_TUNING
    Serial.print(F("Temps "));
    Serial.print(intTempCUF);
    Serial.print(F(" "));
    Serial.print(intTempCUR);
    Serial.print(F(" "));
    Serial.print(intTempCLF);
    Serial.print(F(" "));
    Serial.print(intTempCLR);
    Serial.print(F(" "));
    Serial.println(intTempCFan);
    Serial.print(F(" "));
    Serial.println(upperFrontHeater.heaterCoolDownState ? 0 : 1);

    outputAcInputStates();

    if(poStateMachine.isInState(stateStandby))
    {
        Serial.println(F("State Standby"));
    }
    if(poStateMachine.isInState(stateTurnOnDlb))
    {
        Serial.println(F("State DLB"));
    }
    if(poStateMachine.isInState(stateHeatCycle))
    {
        Serial.println(F("State Cooking"));
    }
    if(poStateMachine.isInState(stateCoolDown))
    {
        Serial.println(F("State Cooldown"));
    }

    Serial.print(F("Relays "));
    Serial.print(digitalRead(HEATER_ENABLE_UPPER_FRONT));
    Serial.print(F(" "));
    Serial.print(digitalRead(HEATER_ENABLE_UPPER_REAR));
    Serial.print(F(" "));
    Serial.print(digitalRead(HEATER_ENABLE_LOWER_FRONT));
    Serial.print(F(" "));
    Serial.println(digitalRead(HEATER_ENABLE_LOWER_REAR));
    
#endif

#ifdef USE_PID
#ifdef ENABLE_PID_TUNING
    double pTerm;
    double iTerm;
    double dTerm;

    upperFrontPID.GetTerms(&pTerm, &iTerm, &dTerm);

    Serial.print(F("Time KP KI KD Raw Temp % Set pTerm iTerm dTerm, "));
    Serial.print(millis());
    Serial.print(F(", "));
    Serial.print(upperFrontPID.GetKp(), 7);
    Serial.print(F(", "));
    Serial.print(upperFrontPID.GetKi(), 7);
    Serial.print(F(", "));
    Serial.print(upperFrontPID.GetKd(), 7);
    Serial.print(F(", "));
    Serial.print(readAD8495KTC(ANALOG_THERMO_UPPER_FRONT));
    Serial.print(F(", "));
    Serial.print(upperFrontHeater.thermocouple);
    Serial.print(F(", "));
    Serial.print(Output);
    Serial.print(F(", "));
    Serial.print(Setpoint);
    Serial.print(F(", "));
    Serial.print(pTerm, 6);
    Serial.print(F(", "));
    Serial.print(iTerm, 6);
    Serial.print(F(", "));
    Serial.print(dTerm, 6);
    Serial.println("");
#endif
#endif

    // stuff for impulse response testing
    //    tempBuf[bufIndex] = intTempCUF;
    //    compareIndex = bufIndex + 1;
    //    if (compareIndex >= TEMP_BUF_SIZE)
    //    {
    //      compareIndex = 0;
    //    }
    //    diff = tempBuf[bufIndex] - tempBuf[compareIndex];
    //    bufIndex++;
    //    if (bufIndex >= TEMP_BUF_SIZE)
    //    {
    //      bufIndex = 0;
    //    }
    //    Serial.print(F("Time Setpoint Temp Dutycycle Diff, "));
    //    Serial.print(millis());
    //    Serial.print(F(", "));
    //    Serial.print(Setpoint);
    //    Serial.print(F(", "));
    //    Serial.print(intTempCUF);
    //    Serial.print(F(", "));
    //    Serial.print(Output);
    //    Serial.print(F(", "));
    //    Serial.println(diff);
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

void initializeRelayPin(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  relayDriverInit(pin, relayStateOff);
}

void saveParametersToMemory(void)
{
  pizzaMemoryWrite((uint8_t*)&upperFrontHeater.parameter, offsetof(MemoryStore, upperFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&upperRearHeater.parameter, offsetof(MemoryStore, upperRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&lowerFrontHeater.parameter, offsetof(MemoryStore, lowerFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&lowerRearHeater.parameter, offsetof(MemoryStore, lowerRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryWrite((uint8_t*)&triacPeriodSeconds, offsetof(MemoryStore, triacPeriodSeconds), sizeof(triacPeriodSeconds));
  pizzaMemoryWrite((uint8_t*)&relayPeriodSeconds, offsetof(MemoryStore, relayPeriodSeconds), sizeof(relayPeriodSeconds));
}

void readParametersFromMemory(void)
{
  pizzaMemoryRead((uint8_t*)&upperFrontHeater.parameter, offsetof(MemoryStore, upperFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&upperRearHeater.parameter, offsetof(MemoryStore, upperRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&lowerFrontHeater.parameter, offsetof(MemoryStore, lowerFrontHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&lowerRearHeater.parameter, offsetof(MemoryStore, lowerRearHeaterParameters), sizeof(HeaterParameters));
  pizzaMemoryRead((uint8_t*)&triacPeriodSeconds, offsetof(MemoryStore, triacPeriodSeconds), sizeof(triacPeriodSeconds));
  pizzaMemoryRead((uint8_t*)&relayPeriodSeconds, offsetof(MemoryStore, relayPeriodSeconds), sizeof(relayPeriodSeconds));
}

//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  pizzaMemoryReturnTypes pizzaMemoryInitResponse;

  Serial.begin(19200);
  Serial.println(F("DEBUG Starting pizza oven..."));

  pizzaMemoryInitResponse = pizzaMemoryInit();

  printHeaterTemperatureParameters("Upper front before: ", upperFrontHeater.parameter.parameterArray);
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
  printHeaterTemperatureParameters("Upper front after: ", upperFrontHeater.parameter.parameterArray);

  acInputsInit();

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

  // PID
  //  upperFrontPID.SetMode(AUTOMATIC);
  upperFrontPID.SetMode(MANUAL);
  upperFrontPID.SetOutputLimits(0, 90);
  upperFrontPID.SetSampleTime(4000);

  Serial.println(F("DEBUG Initialization comlete."));
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
        case 's':  // Start Pizza Oven Cycle
          pizzaOvenStartRequested = true;
          receivedCommandBufferIndex = 0;
          Serial.println(F("DEBUG Pizza oven start requested."));
          break;

        case 'q': // Quit Pizza Oven Cycle
          pizzaOvenStopRequested = true;
          receivedCommandBufferIndex = 0;
          upperFrontPID.SetMode(MANUAL);
          Serial.println(F("DEBUG Pizza oven stop requested."));
          break;

        case 'v': // query protocol version
          Serial.print(F("V "));
          Serial.print(PROTOCOL_MAJOR_VERSION);
          Serial.print(F("."));
          Serial.print(PROTOCOL_MINOR_VERSION);
          Serial.print(F(" bugfix "));
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
                //                               triacPeriodSeconds);
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
            if (CharValidDigit(receivedCommandBuffer[1]) && ((receivedCommandBuffer[1] - '0') < 5))
            {
              pHeater = aHeaters[(receivedCommandBuffer[1] - '0')];
              if (receivedCommandBuffer[0] == 'l')
              {
                GetInputValue(&pHeater->parameter.tempSetPointLowOn, &receivedCommandBuffer[2]);
              }
              else
              {
                GetInputValue(&pHeater->parameter.tempSetPointHighOff, &receivedCommandBuffer[2]);
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
            if (CharValidDigit(receivedCommandBuffer[1]) && ((receivedCommandBuffer[1] - '0') < 5))
            {
              pHeater = aHeaters[(receivedCommandBuffer[1] - '0')];
              if (receivedCommandBuffer[0] == 'n')
              {
                GetInputValue(&pHeater->parameter.onPercent, &receivedCommandBuffer[2]);
              }
              else
              {
                GetInputValue(&pHeater->parameter.offPercent, &receivedCommandBuffer[2]);
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
            if (poStateMachine.isInState(stateStandby) ||
                poStateMachine.isInState(stateCoolDown))
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

//------------------------------------------
// Main Loop
//------------------------------------------
byte queryDone = false;
uint32_t liveCount = 0;
uint16_t inputValue;
uint16_t tempMultiply, tempPercent, tempTemp;

void loop()
{
  bool oldPowerButtonState = powerButtonIsOn();
  bool oldDlbState = l2DlbIsOn();
  double oldOutput = Output;

  // Gather inputs and process
  adcReadRun();
  acInputsRun();
  readThermocouples();
  handleIncomingCommands();

  if ((oldPowerButtonState != powerButtonIsOn()) || (oldDlbState != l2DlbIsOn()))
  {
    outputAcInputStates();
  }

  poStateMachine.update();

  boostEnable(relayBoostRun);
  relayDriverRun();

  PeriodicOutputTemps();
  handleRelayWatchdog();

  // PID
#ifdef USE_PID
  Setpoint = (upperFrontHeater.parameter.tempSetPointHighOff + upperFrontHeater.parameter.tempSetPointLowOn) / 2;
  upperFrontPID.Compute();
  ConvertHeaterPercentCounts();
  //  Output = (Output + 3 * oldOutput) / 4;
  upperFrontHeater.heaterCountsOff = (uint16_t)(((uint32_t)((Output * MILLISECONDS_PER_SECOND + 50)) / 100)) * triacPeriodSeconds;
#endif
}

bool CharValidDigit(unsigned char digit)
{
  bool retVal = false;
  if ((digit >= '0') && (digit <= '9'))
  {
    retVal = true;
  }

  return retVal;
}

uint16_t GetInputValue(uint16_t *pValue, uint8_t *pBuf)
{
  uint16_t inputValue = 0;
  uint8_t i = 0;

  while (pBuf[i] != 10 && pBuf[i] != 13)
  {
    if (CharValidDigit(pBuf[i]))
    {
      inputValue *= 10;
      inputValue += (uint16_t)(pBuf[i] - '0');
    }
    else
    {
      Serial.print(F("DEBUG Invalid input, only digits expected: "));
      Serial.print((char*)pBuf);
      inputValue = *pValue;
      break;
    }
    i++;
  }

  *pValue = inputValue;

  return inputValue;
}

//------------------------------------------
//state machine stateStandby
//------------------------------------------
//State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

void stateStandbyEnter()
{
  Serial.println(F("DEBUG stateStandbyEnter"));
  digitalWrite(TEN_V_ENABLE, LOW);
}

void stateStandbyUpdate()
{
  if (powerButtonIsOn() && pizzaOvenStartRequested)
  {
    poStateMachine.transitionTo(stateTurnOnDlb);
  }
  else if ((thermocoupleFan > COOL_DOWN_EXIT_FAN_TEMP + 15) ||
           (upperFrontHeater.thermocouple > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
           (upperRearHeater.thermocouple  > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
           (lowerFrontHeater.thermocouple > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
           (lowerRearHeater.thermocouple  > COOL_DOWN_EXIT_HEATER_TEMP + 15))
  {
    poStateMachine.transitionTo(stateCoolDown);
  }

  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
}

void stateStandbyExit()
{
  Serial.println(F("DEBUG SX"));
}

/*
   State Machine stateTurnOnDlb
*/
//State stateTurnOnDlb = State(stateTurnOnDlbEnter, stateTurnOnDlbUpdate, stateTurnOnDlbExit);
void stateTurnOnDlbEnter(void)
{
  Serial.println(F("DEBUG Entering stateTurnOnDlb."));
  digitalWrite(TEN_V_ENABLE, HIGH);  //Turn on 10V supply
  CoolingFanControl(true);
}

void stateTurnOnDlbUpdate(void)
{
  if (!powerButtonIsOn() || pizzaOvenStopRequested)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
  else if (l2DlbIsOn())
  {
    poStateMachine.transitionTo(stateHeatCycle);
  }
}

void stateTurnOnDlbExit(void)
{
  Serial.println(F("DEBUG Exiting stateTurnOnDlb."));
}

//------------------------------------------
//state machine stateHeatCycle
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
uint32_t currentTriacTimerCounter, oldTriacTimerCounter;
uint32_t currentRelayTimerCounter, oldRelayTimerCounter;

void stateHeatCycleEnter()
{
  Serial.println(F("DEBUG stateHeatCycleEnter"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  relayTimeBase = 0;
  // Fake the old time so that we exercise the relays the first time through
  oldTriacTimerCounter = 100000;
  oldRelayTimerCounter = 100000;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);

  Output = 0.0;
  upperFrontPID.SetMode(AUTOMATIC);
}

void stateHeatCycleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  // only update the relays periodically
  if (oldTime < newTime)
  {
    if ((newTime - oldTime) < 7)
    {
      return;
    }
  }
  oldTime = newTime;

  // Save working value triacTimeBase since can be updated by interrupt
  currentTriacTimerCounter = triacTimeBase;
  currentRelayTimerCounter = relayTimeBase;

  // Handle triac control
  if (currentTriacTimerCounter != oldTriacTimerCounter)
  {
    //    CoolingFanControl(true);
#ifdef USE_PID
    UpdateHeatControlWithPID(&upperFrontHeater, currentTriacTimerCounter);
#else
    UpdateHeatControl(&upperFrontHeater, currentTriacTimerCounter);
#endif

    UpdateHeatControl(&upperRearHeater, currentTriacTimerCounter);

    UpdateHeaterHardware();

    oldTriacTimerCounter = currentTriacTimerCounter;
  }

  // Handle relay control
  if (currentRelayTimerCounter != oldRelayTimerCounter)
  {
    //    CoolingFanControl(true);
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !l2DlbIsOn() || pizzaOvenStopRequested)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
}

void stateHeatCycleExit()
{
  Serial.println("DEBUG HX");
  AllHeatersOffStateClear();
  upperFrontPID.SetMode(MANUAL);
  Output = 0.0;
}

//------------------------------------------
//state machine stateCoolDown
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter,
//	stateCoolDownUpdate, stateCoolDownExit);

void stateCoolDownEnter()
{
  Serial.println(F("DEBUG stateCoolDown"));
  digitalWrite(TEN_V_ENABLE, HIGH);  //Turn on 10V supply
  CoolingFanControl(true);
}

void stateCoolDownUpdate()
{
  //    if((liveCount % 100000) == 0)
  //      Serial.println("CU");

  // For now just check cool down on fan
  if ((thermocoupleFan <= COOL_DOWN_EXIT_FAN_TEMP) &&
      (upperFrontHeater.thermocouple <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (upperRearHeater.thermocouple  <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (lowerFrontHeater.thermocouple <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (lowerRearHeater.thermocouple  <= COOL_DOWN_EXIT_HEATER_TEMP))
  {
    CoolingFanControl(false);
    poStateMachine.transitionTo(stateStandby);
  }
  else if (powerButtonIsOn() && pizzaOvenStartRequested)
  {
    pizzaOvenStartRequested = false;
    poStateMachine.transitionTo(stateTurnOnDlb);
  }
}

void stateCoolDownExit()
{
  Serial.println(F("DEBUG CX"));
}

