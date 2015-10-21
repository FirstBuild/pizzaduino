/*
* Copyright (c) 2015 FirstBuild
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

// Pizza Oven - Arduino/Read Bear Labs Blend Micro Project

//#include <String.h>
#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include "Boards.h"
#include "FiniteStateMachine.h"
#include "TimerOne.h"

//------------------------------------------
// Macros
//------------------------------------------
#define PROTOCOL_MAJOR_VERSION   0
#define PROTOCOL_MINOR_VERSION   115
#define PROTOCOL_BUGFIX_VERSION  0

// If defined inputs temperatures from Blue Tooth test command
//#define DEBUG_INPUT_TEMP

//------------------------------------------
// Macros for Constants and Pin Definitions
//------------------------------------------
// Thermocouple Definitions
#define ANALOG_REFERENCE_VOLTAGE		((double)3.3)

// For now just check cool down on fan
#define COOL_DOWN_EXIT_FAN_TEMP				((double)38.0)  // 100 degrees F
#define COOL_DOWN_EXIT_HEATER_TEMP			((double)65.0)  // 150 degrees F

// Pin Definitions
#define COOLING_FAN_SIGNAL 				(13)

#define ANALOG_THERMO_UPPER_FRONT		(A0)
#define ANALOG_THERMO_UPPER_REAR		(A1)
#define ANALOG_THERMO_LOWER_FRONT		(A2)
#define ANALOG_THERMO_LOWER_REAR		(A3)
#define ANALOG_THERMO_FAN				(A4)

#define HEATER_ENABLE_UPPER_FRONT		(9)
#define HEATER_ENABLE_UPPER_REAR		(10)
#define HEATER_ENABLE_LOWER_FRONT		(11)
#define HEATER_ENABLE_LOWER_REAR		(12)

// Timer1 Used to keep track of heat control cycles
#define TIMER1_PERIOD_MICRO_SEC			(10000) 	// timer1 10 mSec interval 
#define TIMER1_PERIOD_CLOCK_FACTOR		(1) 		// Clock multiplier for Timer1
#define TIMER1_COUNTER_WRAP				(100)      // Count down for a period of 1 second
#define TIMER1_OUTPUT_TEMP_PERIODIC     (200)       // Multiple of TIMER1_PERIOD_MICRO_SEC to output periodic temp

//------------------------------------------
//state machine setup 
//------------------------------------------
void stateStandbyEnter();
void stateStandbyUpdate();
void stateStandbyExit();

void stateHeatCycleEnter();
void stateHeatCycleUpdate();
void stateHeatCycleExit();

void stateCoolDownEnter();
void stateCoolDownUpdate();
void stateCoolDownExit();

State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);
State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
State stateCoolDown = State(stateCoolDownEnter, 
	stateCoolDownUpdate, stateCoolDownExit);
FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby

//------------------------------------------
// Global Definitions
//------------------------------------------

// Timer one is running to control heat percentage and start
//  Re-synchronized on the start of heating
volatile uint16_t timer1Counter = 0;
uint16_t countOutputTempPeriodic = 0;

// Heater cycle time in multiples of 1 minute
uint16_t nTimesCycleTime = 4;

double testTemp;

struct HeaterParameters 
{
	boolean enabled;
	uint16_t tempSetPointLowOn;   // In integer degrees C
	uint16_t tempSetPointHighOff; // "      "
	uint16_t onPercent;       // Time when a heater turns on in percent
	uint16_t offPercent;      // Time when a heater turns off in percent
};

HeaterParameters heaterParmsUpperFront = {true,   100, 150,   0, 24};
HeaterParameters heaterParmsUpperRear  = {true,   100, 150,  25, 49};
HeaterParameters heaterParmsLowerFront = {true,    40,  60,  50, 74};
HeaterParameters heaterParmsLowerRear  = {true,    40,  60,  75, 100};

uint16_t heaterCountsOnUpperFront;
uint16_t heaterCountsOffUpperFront;
uint16_t heaterCountsOnUpperRear;
uint16_t heaterCountsOffUpperRear;
uint16_t heaterCountsOnLowerFront;
uint16_t heaterCountsOffLowerFront;
uint16_t heaterCountsOnLowerRear;
uint16_t heaterCountsOffLowerRear;

// flags to keep the state of the heater hardware
bool heaterHardwareStateUpperFront = false;
bool heaterHardwareStateUpperRear  = false;
bool heaterHardwareStateLowerFront = false;
bool heaterHardwareStateLowerRear  = false;

// flags to keep the state of the heater cool down once reach High Off Set Point
bool heaterCoolDownStateUpperFront = false;
bool heaterCoolDownStateUpperRear  = false;
bool heaterCoolDownStateLowerFront = false;
bool heaterCoolDownStateLowerRear  = false;

volatile bool outputTempPeriodic = false;

//------------------------------------------
// Prototypes
//------------------------------------------
void ConvertHeaterPercentCounts();
void UpdateHeaterHardware();
void AllHeatersOffStateClear();
void CoolingFanControl(boolean control);
double AnalogThermocoupleTemp(uint16_t rawA2D);
double InputThermocoupleUpperFront();
double InputThermocoupleUpperRear();
double InputThermocoupleLowerFront();
double InputThermocoupleLowerRear();
double InputThermocoupleFan();
void UpdateHeatControlUpperFront(uint16_t currentCounterTimer);
void UpdateHeatControlUpperRear(uint16_t currentCounterTimer);
void UpdateHeatControlLowerFront(uint16_t currentCounterTimer);
void UpdateHeatControlLowerRear(uint16_t currentCounterTimer);
void PeriodicOutputTemps();
bool CharValidDigit(unsigned char digit);
uint16_t GetInputValue();
void HeaterTimerInterrupt();
void ble_write_string(byte *bytes, uint8_t len);

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
	if(heaterHardwareStateUpperFront == false)
		digitalWrite(HEATER_ENABLE_UPPER_FRONT, LOW);
	else
		digitalWrite(HEATER_ENABLE_UPPER_FRONT, HIGH);

	if(heaterHardwareStateUpperRear == false)
		digitalWrite(HEATER_ENABLE_UPPER_REAR, LOW);
	else
		digitalWrite(HEATER_ENABLE_UPPER_REAR, HIGH);

	if(heaterHardwareStateLowerFront == false)
		digitalWrite(HEATER_ENABLE_LOWER_FRONT, LOW);
	else
		digitalWrite(HEATER_ENABLE_LOWER_FRONT, HIGH);

	if(heaterHardwareStateLowerRear == false)
		digitalWrite(HEATER_ENABLE_LOWER_REAR, LOW);
	else
		digitalWrite(HEATER_ENABLE_LOWER_REAR, HIGH);
}

void AllHeatersOffStateClear()
{
	heaterHardwareStateUpperFront = false;
	heaterHardwareStateUpperRear  = false;
	heaterHardwareStateLowerFront = false;
	heaterHardwareStateLowerRear  = false;
	
	heaterCoolDownStateUpperFront = false;
	heaterCoolDownStateUpperRear  = false;
	heaterCoolDownStateLowerFront = false;
	heaterCoolDownStateLowerRear  = false;
	
	UpdateHeaterHardware();
}

void CoolingFanControl(boolean control)
{
	if(control == true)
  		digitalWrite(COOLING_FAN_SIGNAL, HIGH);
	else
  		digitalWrite(COOLING_FAN_SIGNAL, LOW);
}		

double AnalogThermocoupleTemp(uint16_t rawA2D)
{
	double analogVoltage;
	double tempC;
	
	analogVoltage = (float)rawA2D * ANALOG_REFERENCE_VOLTAGE / 1023.0;
	tempC = (analogVoltage - 1.25)  / 0.005;  
	return tempC;
}

double InputThermocoupleUpperFront()
{
	double tempC;
	
	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_UPPER_FRONT));
	return tempC;
}

double InputThermocoupleUpperRear()
{
	double tempC;
	
	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_UPPER_REAR));
	return tempC;
}

double InputThermocoupleLowerFront()
{
	double tempC;
	
	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_LOWER_FRONT));
	return tempC;
}

double InputThermocoupleLowerRear()
{
	double tempC;
	
	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_LOWER_REAR));
	return tempC;
}

double InputThermocoupleFan()
{
	double tempC;
	
	tempC = AnalogThermocoupleTemp(analogRead(ANALOG_THERMO_FAN));
	return tempC;
}

void UpdateHeatControlUpperFront(uint16_t currentCounterTimer)
{	
	double tempC;
	uint16_t icase = 0;
	
	heaterHardwareStateUpperFront = false;
	if((heaterParmsUpperFront.enabled == true) &&
	   (currentCounterTimer >= heaterCountsOnUpperFront) &&
	   (currentCounterTimer <= heaterCountsOffUpperFront))
	{   
			tempC = InputThermocoupleUpperFront();
    							
			// If not in cool down and less than High Set Point Turn on Heater
			if((heaterCoolDownStateUpperFront == false) && (tempC < (double)heaterParmsUpperFront.tempSetPointHighOff))
			{
				icase = 1;
				heaterHardwareStateUpperFront = true;
			}
			// If not in cool down and greater than High Set Point turn off heater and set cool down
			else if((heaterCoolDownStateUpperFront == false) && (tempC >= (double)heaterParmsUpperFront.tempSetPointHighOff))
			{
				icase = 2;
				heaterCoolDownStateUpperFront = true;
				heaterHardwareStateUpperFront = false;
			}
			// If in cool down and less than equal than low set point, exit cool down and turn heater on
			else if((heaterCoolDownStateUpperFront == true) && (tempC <= (double)heaterParmsUpperFront.tempSetPointLowOn))	
			{
				icase = 3;
				heaterCoolDownStateUpperFront = false;
				heaterHardwareStateUpperFront = true;
			}
			else // In cool down but have not reached the Low Set Point
			{
				icase = 4;
				heaterHardwareStateUpperFront = false;
			}
	}
	else	// Heater Disabled or Outside the percentage limits of the cycle
	{
	        icase = 9;
			heaterHardwareStateUpperFront = false;
	}
}

void UpdateHeatControlUpperRear(uint16_t currentCounterTimer)
{	
	double tempC;
	uint16_t icase = 0;  // For case debug
	
	heaterHardwareStateUpperRear = false;
	if((heaterParmsUpperRear.enabled == true) &&
	   (currentCounterTimer >= heaterCountsOnUpperRear) &&
	   (currentCounterTimer <= heaterCountsOffUpperRear))
	{ 			  
			tempC = InputThermocoupleUpperRear();

			// If not in cool down and less than High Set Point Turn on Heater
			if((heaterCoolDownStateUpperRear == false) && (tempC < (double)heaterParmsUpperRear.tempSetPointHighOff))
			{
				icase = 11;
				heaterHardwareStateUpperRear = true;
			}
			// If not in cool down and greater than High Set Point turn off heater and set cool down
			else if((heaterCoolDownStateUpperRear == false) && (tempC >= (double)heaterParmsUpperRear.tempSetPointHighOff))	
			{
				icase = 12;				
				heaterCoolDownStateUpperRear = true;
				heaterHardwareStateUpperRear = false;
			}
			// If in cool down and less than equal than low set point, exit cool down and turn heater on
			else if((heaterCoolDownStateUpperRear == true) && (tempC <= (double)heaterParmsUpperRear.tempSetPointLowOn))	
			{
				icase = 13;
				heaterCoolDownStateUpperRear = false;
				heaterHardwareStateUpperRear = true;
			}
			else // In cool down but have not reached the Low Set Point
			{
				icase = 14;
				heaterHardwareStateUpperRear = false;
			}
	}
	else	// Heater Disabled or Outside the percentage limits of the cycle
	{
	        icase = 19;
			heaterHardwareStateUpperRear = false;
	}
}

void UpdateHeatControlLowerFront(uint16_t currentCounterTimer)
{	
	double tempC;
	uint16_t icase = 0;  // For case debug
	
	heaterHardwareStateLowerFront = false;
	if((heaterParmsLowerFront.enabled == true) &&
	   (currentCounterTimer >= heaterCountsOnLowerFront) &&
	   (currentCounterTimer <= heaterCountsOffLowerFront))
	{   
			tempC = InputThermocoupleLowerFront();

			// If not in cool down and less than High Set Point Turn on Heater
			if((heaterCoolDownStateLowerFront == false) && (tempC < (double)heaterParmsLowerFront.tempSetPointHighOff))
			{
				icase = 21;
				heaterHardwareStateLowerFront = true;
			}
			// If not in cool down and greater than High Set Point turn off heater and set cool down
			else if((heaterCoolDownStateLowerFront == false) && (tempC >= (double)heaterParmsLowerFront.tempSetPointHighOff))	
			{
				icase = 22;				
				heaterCoolDownStateLowerFront = true;
				heaterHardwareStateLowerFront = false;
			}
			// If in cool down and less than equal than low set point, exit cool down and turn heater on
			else if((heaterCoolDownStateLowerFront == true) && (tempC <= (double)heaterParmsLowerFront.tempSetPointLowOn))	
			{
				icase = 23;
				heaterCoolDownStateLowerFront = false;
				heaterHardwareStateLowerFront = true;
			}
			else // In cool down but have not reached the Low Set Point
			{
				icase = 24;
				heaterHardwareStateLowerFront = false;
			}
	}
	else	// Heater Disabled or Outside the percentage limits of the cycle
	{
	        icase = 29;
			heaterHardwareStateLowerFront = false;
	}
}

void UpdateHeatControlLowerRear(uint16_t currentCounterTimer)
{	
	double tempC;
	uint16_t icase = 0;  // For case debug
	
	heaterHardwareStateLowerRear = false;
	if((heaterParmsLowerRear.enabled == true) &&
	   (currentCounterTimer >= heaterCountsOnLowerRear) &&
	   (currentCounterTimer <= heaterCountsOffLowerRear))
 	{
			tempC = InputThermocoupleLowerRear();
    										
			// If not in cool down and less than High Set Point Turn on Heater
			if((heaterCoolDownStateLowerRear == false) && (tempC < (double)heaterParmsLowerRear.tempSetPointHighOff))
			{
				icase = 31;
				heaterHardwareStateLowerRear = true;
			}
			// If not in cool down and greater than High Set Point turn off heater and set cool down
			else if((heaterCoolDownStateLowerRear == false) && (tempC >= (double)heaterParmsLowerRear.tempSetPointHighOff))	
			{
				icase = 32;				
				heaterCoolDownStateLowerRear = true;
				heaterHardwareStateLowerRear = false;
			}
			// If in cool down and less than equal than low set point, exit cool down and turn heater on
			else if((heaterCoolDownStateLowerRear == true) && (tempC <= (double)heaterParmsLowerRear.tempSetPointLowOn))	
			{
				icase = 33;
				heaterCoolDownStateLowerRear = false;
				heaterHardwareStateLowerRear = true;
			}
			else // In cool down but have not reached the Low Set Point
			{
				icase = 34;
				heaterHardwareStateLowerRear = false;
			}
	}
	else	// Heater Disabled or Outside the percentage limits of the cycle
	{
	        icase = 39;
			heaterHardwareStateLowerRear = false;
	}
}

void PeriodicOutputTemps()
{
	uint8_t strLen;
	char formatStr[25];
    uint16_t intTempCUF, intTempCUR, intTempCLF, intTempCLR, intTempCFan;

 	intTempCUF =  (uint16_t) (InputThermocoupleUpperFront() + 0.5);
	intTempCUR =  (uint16_t) (InputThermocoupleUpperRear()  + 0.5);
	intTempCLF =  (uint16_t) (InputThermocoupleLowerFront() + 0.5);
	intTempCLR =  (uint16_t) (InputThermocoupleLowerRear()  + 0.5);
	intTempCFan = (uint16_t) (InputThermocoupleFan()		+ 0.5);	
 
    strLen = sprintf(formatStr,"Temps %d %d %d %d %d\n", 
		intTempCUF, intTempCUR, intTempCLF, intTempCLR, intTempCFan);
    if(strLen>0)
    	ble_write_string((byte *)&formatStr, strLen);
}

//------------------------------------------
// Update Heater State Interrupt
//------------------------------------------
void HeaterTimerInterrupt()
{  
	timer1Counter++;

	if(timer1Counter > (TIMER1_COUNTER_WRAP * nTimesCycleTime))
  	{
  		timer1Counter = 0;
  	}

	countOutputTempPeriodic++;
	
	if(countOutputTempPeriodic >= TIMER1_OUTPUT_TEMP_PERIODIC)
	{
  		countOutputTempPeriodic = 0;
		outputTempPeriodic = true;
	}
}

//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  Serial1.begin(9600); 
  Serial1.println("BLE Arduino Slave");

  // Initialize Timer1
  Timer1.initialize(TIMER1_PERIOD_MICRO_SEC * TIMER1_PERIOD_CLOCK_FACTOR);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10); 
  Timer1.attachInterrupt(HeaterTimerInterrupt);


  // Setup Cooling Fan as Output and Turn Off
  pinMode(COOLING_FAN_SIGNAL, OUTPUT);
  digitalWrite(COOLING_FAN_SIGNAL, LOW);	
  
  // Setup Heater Enables as Outputs and Turn Off
  pinMode(HEATER_ENABLE_UPPER_FRONT, OUTPUT);
  digitalWrite(HEATER_ENABLE_UPPER_FRONT, LOW);
  pinMode(HEATER_ENABLE_UPPER_REAR, OUTPUT);
  digitalWrite(HEATER_ENABLE_UPPER_REAR, LOW);
  pinMode(HEATER_ENABLE_LOWER_FRONT, OUTPUT);
  digitalWrite(HEATER_ENABLE_LOWER_FRONT, LOW);
  pinMode(HEATER_ENABLE_LOWER_REAR, OUTPUT);
  digitalWrite(HEATER_ENABLE_LOWER_REAR, LOW); 

  ConvertHeaterPercentCounts();
   
  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  ble_set_name("Pizza Oven");
  
  // Init. and start BLE library.
  ble_begin();
  
  //useInterrupt(true);
}

//------------------------------------------
// Main Loop
//------------------------------------------
static byte buf_len = 0;
byte queryDone = false;
uint32_t liveCount = 0;
uint16_t inputValue;
uint16_t tempMultiply, tempPercent, tempTemp;

void loop()
{
	char formatStr[25];
	uint16_t strLen;

  // Process Blue Tooth Command if available
  // TBD ble_available returns -1 if nothing available
  if(ble_available() > 0)
  {
    byte cmd;
    cmd = ble_read();
    Serial1.write(cmd);
    
    // Parse data here
    switch (cmd)
    {
      case 'v': // query protocol version
          strLen = sprintf(formatStr,"V %u.%u bugfix %u\n", 
				(uint16_t)PROTOCOL_MAJOR_VERSION, (uint16_t)PROTOCOL_MINOR_VERSION, (uint16_t)PROTOCOL_BUGFIX_VERSION);
          if(strLen>0)
            ble_write_string((byte *)&formatStr, strLen);	
	      break;

      case 'p': // query heat control parameters
		if(ble_available() >= 1)
		{
		    cmd = ble_read();
			Serial1.write(cmd);
			
			switch(cmd)
			{
			case '0' :			
          		strLen = sprintf(formatStr,"nTimes %u\n", 
					nTimesCycleTime);
				break;    
			case '1' :			
          		strLen = sprintf(formatStr,"UF %u %u %u %u\n", 
					heaterParmsUpperFront.tempSetPointLowOn, heaterParmsUpperFront.tempSetPointHighOff,
					heaterParmsUpperFront.onPercent, heaterParmsUpperFront.offPercent);
				break;          	
			case '2' :			
          		strLen = sprintf(formatStr,"UR %u %u %u %u\n", 
					heaterParmsUpperRear.tempSetPointLowOn, heaterParmsUpperRear.tempSetPointHighOff,
					heaterParmsUpperRear.onPercent, heaterParmsUpperRear.offPercent);
				break;          	
			case '3' :			
          		strLen = sprintf(formatStr,"LF %u %u %u %u\n", 
					heaterParmsLowerFront.tempSetPointLowOn, heaterParmsLowerFront.tempSetPointHighOff,
					heaterParmsLowerFront.onPercent, heaterParmsLowerFront.offPercent);
				break;
			case '4' :			
          		strLen = sprintf(formatStr,"LR %u %u %u %u\n", 
					heaterParmsLowerRear.tempSetPointLowOn, heaterParmsLowerRear.tempSetPointHighOff,
					heaterParmsLowerRear.onPercent, heaterParmsLowerRear.offPercent);
				break;
			default:
				;
			}			 				    
   
           if(strLen>0)
             ble_write_string((byte *)&formatStr, strLen);
        }    	
	    break;
            
      case 's':	// Start Pizza Oven Cycle			
		Serial1.println("Start");
		if(poStateMachine.isInState(stateStandby) || 
			poStateMachine.isInState(stateCoolDown))
		{					
			poStateMachine.transitionTo(stateHeatCycle);
		}
		else
		{
			Serial1.println("Invalid");
		}	
        break;

      case 'q':	// Quit Pizza Oven Cycle
		Serial1.println("Exit");			
		if(poStateMachine.isInState(stateHeatCycle)) 
		{
			poStateMachine.transitionTo(stateCoolDown);
		}
		else
		{
			Serial1.println("Invalid");			
		}		
        break;

      case 'o':	// Test input integer parameter and set test temperature
		Serial1.println("input n test");
		inputValue = GetInputValue();
		Serial1.println(inputValue);
		testTemp = (double) inputValue;
        break;
        
      case 'l':	// Set Lower Set Point Parameter 
		Serial1.println("Set Lower Set Point Parameter");
		if(ble_available() > 2)
		{
		    cmd = ble_read();
			Serial1.write(cmd);
			
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
		Serial1.println("Set Upper Set Point Parameter");
		if(ble_available() > 2)
		{
		    cmd = ble_read();
			Serial1.write(cmd);
			
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
		Serial1.println("Set On Percent Parameter");
		if(ble_available() > 2)
		{
		    cmd = ble_read();
			Serial1.write(cmd);
			
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
		Serial1.println("Set Off Percent Parameter");
		if(ble_available() > 2)
		{
		    cmd = ble_read();
			Serial1.write(cmd);
			
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
			Serial1.println("Set Time Period Cycle Seconds");
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
		    		ble_write_string((byte *)"Invalid n", 9);
		    		Serial1.println("Invalid n");
		    	}
	    	}			
	    }
	    else
		{	 
    		ble_write_string((byte *)"Invalid Heat", 12);
    		Serial1.println("Invalid Heat");
    	}			
		break;		   
        
	  default:	
		;	
    	}	  
 	}
 	
 	poStateMachine.update();
	
 	// send out any outstanding data
    ble_do_events();
    buf_len = 0;
}

bool CharValidDigit(unsigned char digit)
{
	if((digit >= '0') && (digit <= '9'))
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
	
	cBytesAvailable = ble_available();
//	Serial1.println((uint16_t)cBytesAvailable);
	if(cBytesAvailable > 0)
	{
		nBytesAvailable = (byte)cBytesAvailable;
		// limit to 4 digits at most
		if(nBytesAvailable > 4)
		{
			nBytesAvailable = 4;
		}
		
		for(loop=0; loop < nBytesAvailable; loop++)
		{ 
			digit = ble_read();
			if(CharValidDigit(digit))
			{
				inputValue *= 10;
				inputValue += (uint16_t)(digit -'0');
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
	Serial1.println("stateStandbyEnter");
}

void stateStandbyUpdate()
{
//  if((liveCount % 100000) == 0) 
//     Serial1.println("SU");
}

void stateStandbyExit()
{
	     Serial1.println("SX");
}

//------------------------------------------
//state machine stateHeatCycle 
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
uint16_t saveTimer1Counter, lastTimer1Counter;

void stateHeatCycleEnter()
{
	Serial1.println("stateHeatCycleEnter");
	
	// Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
	timer1Counter = 0;
	
	CoolingFanControl(true);
}

void stateHeatCycleUpdate()
{	  
//    if((liveCount % 100) == 0) 
//      Serial1.println("HU");
    
    // Save working value timer1Counter since can be updated by interrupt  
    saveTimer1Counter = timer1Counter;
    
    // Only update heat control if Timer1 Counter changed
    
    if(saveTimer1Counter != lastTimer1Counter)
    {
		CoolingFanControl(true);
    	UpdateHeatControlUpperFront(saveTimer1Counter);
    	UpdateHeatControlUpperRear(saveTimer1Counter);
    	UpdateHeatControlLowerFront(saveTimer1Counter);
    	UpdateHeatControlLowerRear(saveTimer1Counter);

    	UpdateHeaterHardware();   	

    	lastTimer1Counter = saveTimer1Counter;	
    }  

#if 1   
	// Output Periodic Temperatures
    if(true == outputTempPeriodic)
	{
		outputTempPeriodic = false;
		PeriodicOutputTemps();	
	}
#endif	
}

void stateHeatCycleExit()
{
	     Serial1.println("HX");
	     AllHeatersOffStateClear();
}

//------------------------------------------
//state machine stateCoolDown 
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter, 
//	stateCoolDownUpdate, stateCoolDownExit);

void stateCoolDownEnter()
{
	Serial1.println("stateCoolDown");
}

void stateCoolDownUpdate()
{
//    if((liveCount % 100000) == 0) 
//      Serial1.println("CU");

// For now just check cool down on fan
    if((InputThermocoupleFan() <= COOL_DOWN_EXIT_FAN_TEMP) &&
		(InputThermocoupleUpperFront() <= COOL_DOWN_EXIT_HEATER_TEMP) &&
		(InputThermocoupleUpperRear()  <= COOL_DOWN_EXIT_HEATER_TEMP) &&
		(InputThermocoupleLowerFront() <= COOL_DOWN_EXIT_HEATER_TEMP) &&
		(InputThermocoupleLowerRear()  <= COOL_DOWN_EXIT_HEATER_TEMP))
    {
		CoolingFanControl(false);
		poStateMachine.transitionTo(stateStandby);    
    }
    
	// Output Periodic Temperatures
    if(true == outputTempPeriodic)
	{
		outputTempPeriodic = false;
		PeriodicOutputTemps();	
	}
}

void stateCoolDownExit()
{
	     Serial1.println("CX");
}

//------------------------------------------
// Bluetooth Write String Routines
//------------------------------------------
void ble_write_string(byte *bytes, uint8_t len)
{
  if (buf_len + len > 20)
  {
	// Event counts reduced from 15000 from reference code
    for (int j = 0; j < 100; j++)
      ble_do_events();
    
    buf_len = 0;
  }
  
  for (int j = 0; j < len; j++)
  {
    ble_write(bytes[j]);
    buf_len++;
  }
    
  if (buf_len == 20)
  {
    for (int j = 0; j < 100; j++)
      ble_do_events();
    
    buf_len = 0;
  }  
}
