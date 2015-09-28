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

// Pizza Oven Arduino Project

//#include <String.h>
#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include "Boards.h"
#include "Adafruit_MAX31855.h"
#include "FiniteStateMachine.h"
#include "TimerOne.h"

//------------------------------------------
// Constants
//------------------------------------------

static const uint8_t zoneUpperFront = 1;
static const uint8_t zoneUpperRear  = 2;
static const uint8_t zoneLowerFront = 3;
static const uint8_t zoneLowerRear  = 4;

//------------------------------------------
// Pin Definitions
//------------------------------------------

#define COOLING_FAN_SIGNAL 				(A0)

// TBD HW Pin Defs Out of Order and need to be checked.
// Thermocouple Pin Definitions
#define SW_SPI_THERMO_DO                (A4)
#define SW_SPI_THERMO_CLK               (A5)
#define SW_SPI_THERMO_CS_UPPER_FRONT	(8)
#define SW_SPI_THERMO_CS_UPPER_REAR		(5)
#define SW_SPI_THERMO_CS_LOWER_FRONT	(3)
#define SW_SPI_THERMO_CS_LOWER_REAR		(2)

// TBD Fan Temp TC5 on Reset?

// Heater Enable Pin Definitions
#define HEATER_ENABLE_UPPER_FRONT		(9)
#define HEATER_ENABLE_UPPER_REAR		(10)
#define HEATER_ENABLE_LOWER_FRONT		(11)
#define HEATER_ENABLE_LOWER_REAR		(12)

// Timer1 Used to keep track of 4 second heat control cycles
#define TIMER1_PERIOD_MICRO_SEC			(100000)		// 100 ms interval to start
#define TIMER1_PERIOD_CLOCK_FACTOR		(1) 		// Clock multiplier for Timer1
#define TIMER1_COUNTER_WRAP				(40)        // Count down to a period of 4 seconds

//------------------------------------------
// Software SPI Thermocouple Definitions
//------------------------------------------
#if 1
Adafruit_MAX31855 thermocoupleUpperFront(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_UPPER_FRONT, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleUpperRear(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_UPPER_REAR, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleLowerFront(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_LOWER_FRONT, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleLowerRear(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_LOWER_REAR, SW_SPI_THERMO_DO);
#endif

#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  0 // bugfix

//------------------------------------------
// Global Definitions
//------------------------------------------

// Timer one is running to control heat percentage and start
//  Re-synchronized on the start of heating
volatile uint16_t timer1Counter = 0;

double testTemp;

struct HeaterParameters 
{
	boolean enabled;
	uint16_t tempSetPointLowOn;   // In integer degrees C
	uint16_t tempSetPointHighOff; // "      "
	uint16_t onPercent;       // Time when a heater turns on in a 4 second cycle in percent
	uint16_t offPercent;      // Time when a heater turns off in a 4 second cycle in percent
};

HeaterParameters heaterParmsUpperFront = {true,  200, 275,   0, 71};
HeaterParameters heaterParmsUpperRear  = {false, 200, 275,   0, 64};
HeaterParameters heaterParmsLowerFront = {false, 200, 275,  50, 90};
HeaterParameters heaterParmsLowerRear  = {false, 200, 275,   0, 33};

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

// flags to keep the state of the heater cool down one reach High Set Point
bool heaterCoolDownStateUpperFront = false;
bool heaterCoolDownStateUpperRear  = false;
bool heaterCoolDownStateLowerFront = false;
bool heaterCoolDownStateLowerRear  = false;

void ConvertHeaterPercentCounts()
{
	heaterCountsOnUpperFront  = (uint16_t)(((uint32_t) heaterParmsUpperFront.onPercent  * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOffUpperFront = (uint16_t)(((uint32_t) heaterParmsUpperFront.offPercent * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOnUpperRear   = (uint16_t)(((uint32_t) heaterParmsUpperRear.onPercent   * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOffUpperRear  = (uint16_t)(((uint32_t) heaterParmsUpperRear.offPercent  * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOnLowerFront  = (uint16_t)(((uint32_t) heaterParmsLowerFront.onPercent  * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOffLowerFront = (uint16_t)(((uint32_t) heaterParmsLowerFront.offPercent * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOnLowerRear   = (uint16_t)(((uint32_t) heaterParmsLowerRear.onPercent   * TIMER1_COUNTER_WRAP) / 100);
	heaterCountsOffLowerRear  = (uint16_t)(((uint32_t) heaterParmsLowerRear.offPercent  * TIMER1_COUNTER_WRAP) / 100);
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

void UpdateHeatControl(uint16_t currentCounterTimer)
{	
	double tempC;
	
	heaterHardwareStateUpperFront = false;
	if((heaterParmsUpperFront.enabled = true) &&
	   (currentCounterTimer >= heaterCountsOnUpperFront) &&
	   (currentCounterTimer < heaterCountsOffUpperFront))
	{   
			tempC = getTempThermocouple(zoneUpperFront); 
			
			// If not in cool down and less than High Set Point Turn on Heater
			if((heaterCoolDownStateUpperFront == false) && (tempC < (double)heaterParmsUpperFront.tempSetPointHighOff))
			{
				heaterHardwareStateUpperFront = true;
			}
			// If not in cool down and greater than High Set Point turn off heater and set cool down
			else if((heaterCoolDownStateUpperFront == false) && (tempC >= (double)heaterParmsUpperFront.tempSetPointHighOff))	
			{
				heaterCoolDownStateUpperFront = true;
				heaterHardwareStateUpperFront = false;
			}
			// If in cool down and less than equal than low set point, exit cool down and turn heater on
			else if((heaterCoolDownStateUpperFront == true) && (tempC <= (double)heaterParmsUpperFront.tempSetPointLowOn))	
			{
				heaterCoolDownStateUpperFront = false;
				heaterHardwareStateUpperFront = true;
			}
			else // In cool down but have not reached the Low Set Point
			{
				heaterHardwareStateUpperFront = false;
			}
	}
	else	// Outside the percentage limits of the cycle
	{
			heaterCoolDownStateUpperFront = false;
			heaterHardwareStateUpperFront = false;
	}
	
#if 0			
	heaterParmsStateUpperRear = false;	
	if((heaterParmsUpperRear.enabled = true) &&
	   (saveTimer1Counter >= heaterCountsOnUpperRear) &&
	   (saveTimer1Counter < heaterCountsOffUpperRear))
			heaterParmsStateUpperRear = true;
				
	heaterParmsStateLowerFront = false;
	if((heaterParmsLowerFront.enabled = true) &&
	   (saveTimer1Counter >= heaterCountsOnLowerFront) &&
	   (saveTimer1Counter < heaterCountsOffLowerFront))
			heaterParmsStateLowerFront = true;
			
	heaterParmsStateLowerRear = false;	
	if((heaterParmsLowerRear.enabled = true) &&
	   (saveTimer1Counter >= heaterCountsOnLowerRear) &&
	   (saveTimer1Counter < heaterCountsOffLowerRear))
			heaterParmsStateLowerRear = true;
#endif

	UpdateHeaterHardware();
}

//------------------------------------------
//state machine setup 
//------------------------------------------
State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);
State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
State stateCoolDown = State(stateCoolDownEnter, 
	stateCoolDownUpdate, stateCoolDownExit);
FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby
 
//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  Serial1.begin(9600); 
  Serial1.println("BLE Arduino Slave");

#if 1
  // Initialize Timer1
  Timer1.initialize(TIMER1_PERIOD_MICRO_SEC * TIMER1_PERIOD_CLOCK_FACTOR);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10); 
  Timer1.attachInterrupt(HeaterTimerInterrupt);
#endif

#if 1
  // Setup Cooling Fan as Output and Turn Off
  pinMode(COOLING_FAN_SIGNAL, OUTPUT);
  digitalWrite(COOLING_FAN_SIGNAL, HIGH);	

  
  // Setup Heater Enables as Outputs and Turn Off
  pinMode(HEATER_ENABLE_UPPER_FRONT, OUTPUT);
  digitalWrite(HEATER_ENABLE_UPPER_FRONT, HIGH);
  pinMode(HEATER_ENABLE_UPPER_REAR, OUTPUT);
  digitalWrite(HEATER_ENABLE_UPPER_REAR, HIGH);
  pinMode(HEATER_ENABLE_LOWER_FRONT, OUTPUT);
  digitalWrite(HEATER_ENABLE_LOWER_FRONT, HIGH);
  pinMode(HEATER_ENABLE_LOWER_REAR, OUTPUT);
  digitalWrite(HEATER_ENABLE_LOWER_REAR, HIGH);   
#endif

  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  //ble_set_name("My Name");
  
  // Init. and start BLE library.
  ble_begin();
  
  //useInterrupt(true);
}

//------------------------------------------
// Update Heater State Interrupt
//------------------------------------------
void HeaterTimerInterrupt(void)
{
//	ble_write_string((byte *)"I", 1);  
	timer1Counter++;

  if(timer1Counter > TIMER1_COUNTER_WRAP)
  {
  	timer1Counter = 0;
  }

//	ble_write_string(timer1Counter); 
}

static byte buf_len = 0;
//------------------------------------------
// Main Loop
//------------------------------------------
byte queryDone = false;
uint32_t liveCount = 0;
uint16_t inputValue;

void loop()
{
  liveCount++;
  if((liveCount % 100000) == 0) {
     Serial1.println("[");
    ble_write_string((byte *)"[", 1);
  }
     
  // Process Blue Tooth Command if available
  // TBD ble_available returns -1 if nothing available switch ignores?
  if(ble_available())
  {
    byte cmd;
    cmd = ble_read();
    Serial1.write(cmd);
    
    // Parse data here
    switch (cmd)
    {
      case 'v': // query protocol version
        {
          byte buf[] = {'V', 'a', 'b', 'c'};
          ble_write_string(buf, 4);
        }
        break;
            
      case 't': // Test Thermistors
        {
            Serial1.println("Tc");
        	double tempC;
			tempC = getTempThermocouple(zoneUpperFront);
			tempC = getTempThermocouple(zoneUpperRear);
			tempC = getTempThermocouple(zoneLowerFront);
			tempC = getTempThermocouple(zoneLowerRear);
        }
        break;
        
      case 's':	// Start Pizza Oven Cycle
        {			
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
        }
        break;

      case 'q':	// Exit Pizza Oven Cycle
        {
			Serial1.println("Exit");			
			if(poStateMachine.isInState(stateHeatCycle)) 
			{
				poStateMachine.transitionTo(stateCoolDown);
			}
			else
			{
				Serial1.println("Invalid");			
			}		
        }
        break;

      case 'n':	// Test input integer parameter and set test temperature
        {
			Serial1.println("input n test");
			inputValue = GetInputValue();
			Serial1.println(inputValue);
			testTemp = (double) inputValue;			
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
#if 1
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
#endif
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
  if((liveCount % 100000) == 0) 
     Serial1.println("SU");
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
	
	// Start the timer1 counter over at the start of heat cycle
	timer1Counter = 0;
}

void stateHeatCycleUpdate()
{
    if((liveCount % 100000) == 0) 
      Serial1.println("HU");
    
    // Save working value timer1Counter since can be updated by interrupt  
    saveTimer1Counter = timer1Counter;
    
    // Only update heat control if Timer1 Counter changed
    if(saveTimer1Counter != lastTimer1Counter)
    {
    	UpdateHeatControl(saveTimer1Counter);
    	lastTimer1Counter = saveTimer1Counter;	
    }  
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
    if((liveCount % 100000) == 0) 
      Serial1.println("CU");
    // when reach ? set temperature on ?  
}

void stateCoolDownExit()
{
	     Serial1.println("CX");
}

//------------------------------------------
// Cooling Fan Control
//------------------------------------------
void CoolingFanControl(boolean control)
{
	if(control == true)
  		digitalWrite(COOLING_FAN_SIGNAL, HIGH);	
	else
  		digitalWrite(COOLING_FAN_SIGNAL, LOW);	
}		

//------------------------------------------
// Get Thermistor Temperatures For Passed Sensor
//------------------------------------------
double getTempThermocouple(uint8_t sensor) 
{
	double degreesC = 0.0;
	uint16_t degreesCx10;
	
	switch(sensor)
	{
	case 0:
		degreesC = testTemp;
		break;
	case zoneUpperFront:
		degreesC = thermocoupleUpperFront.readCelsius();
		Serial1.print("tempUF");
//		ble_write_string((byte *)"tcUF", 4);
		break;
  	case zoneUpperRear:
  		degreesC = thermocoupleUpperRear.readCelsius();
  		Serial1.print("tempUR");
//  		ble_write_string((byte *)"tcUR", 4);
  		break;
  	case zoneLowerFront:
  		degreesC = thermocoupleLowerFront.readCelsius();
  		Serial1.print("tempLF");
//  		ble_write_string((byte *)"tcLF", 4);
  		break;
  	case zoneLowerRear:
 		degreesC = thermocoupleLowerRear.readCelsius();
  		Serial1.print("tempLR");
//  		ble_write_string((byte *)"tcLR", 4);  		
  		break;
  	default:
  	    Serial1.print("Invalid tc!");
	}
    
    if (isnan(degreesC)) {
      Serial1.println("Error!");
    } else {
    	Serial1.println(degreesC);	
    	degreesCx10 = (uint16_t) (degreesC + 0.05) * 10.0;
    	ble_write_string((byte *)&degreesCx10, 4);  // send as a binary temp * 10
    }
    
	return degreesC;
};		
	
//------------------------------------------
// Bluetooth Write String Routines
//------------------------------------------
void ble_write_string(byte *bytes, uint8_t len)
{
  if (buf_len + len > 20)
  {
	// TBD the count of 15000 need to be reduce, from reference codes
    for (int j = 0; j < 15000; j++)
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
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }  
}
