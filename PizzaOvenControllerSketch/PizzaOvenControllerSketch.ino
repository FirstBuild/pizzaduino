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

#define COOLING_FAN_SIGNAL 				(0)

// TBD HW Pin Defs Out of Order and need to be checked.
// Thermocouple Pin Definitions
#define SW_SPI_THERMO_DO                (2)
#define SW_SPI_THERMO_CLK               (5)
#define SW_SPI_THERMO_CS_UPPER_FRONT	(3)
#define SW_SPI_THERMO_CS_UPPER_REAR		(8)
#define SW_SPI_THERMO_CS_LOWER_FRONT	(5)
#define SW_SPI_THERMO_CS_LOWER_REAR		(3)

// Heater Enable Pin Definitions
#define HEATER_ENABLE_UPPER_FRONT		(9)
#define HEATER_ENABLE_UPPER_REAR		(10)
#define HEATER_ENABLE_LOWER_FRONT		(11)
#define HEATER_ENABLE_LOWER_REAR		(12)

// Timer1 Period in Microseconds
#define TIMER1_PERIOD_MICRO_SEC			(10000000)	// 1000 ms interval to start
#define TIMER1_PERIOD_CLOCK_FACTOR		(1) 		// Clock multiplier for Timer1
#define TIMER1_COUNTER_WRAP				(2400)      // Count down to a period of 4 minutes (240 sec)

uint32_t timer1Counter = 0;

//------------------------------------------
// Software SPI Thermocouple Definitions
//------------------------------------------
#if 0
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
struct HeaterParameters 
{
	boolean enabled;
	double tempSetPointLowOn;
	double tempSetPointHighOff;
	uint16_t onTime;
	uint16_t offTime;
};

HeaterParameters heaterParmsUpperFront  = {true,  200.0, 275.0,   0, 640};
HeaterParameters heaterParmsUpperRear   = {false, 200.0, 275.0,   0, 850};
HeaterParameters heaterParamsLowerFront = {false, 200.0, 275.0,   0, 500};
HeaterParameters heatersParamsLowerRear = {false, 200.0, 275.0, 500,   0};

#if 1
//------------------------------------------
//state machine setup 
//------------------------------------------
State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);
State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
State stateCoolDown = State(stateCoolDownEnter, 
	stateCoolDownUpdate, stateCoolDownExit);
FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby
#endif

 #define TOTAL_PINS              20 // 14 digital + 6 analog
 
#if 0 
const byte rxPin = 0;
const byte txPin = 1;

// set up a new serial object
SoftwareSerial Serial (rxPin, txPin);
#endif
//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  Serial1.begin(9600); 
  Serial1.println("BLE Arduino Slave");

  /* Default all to digital input */
 
  for (int pin = 0; pin < TOTAL_PINS; pin++)
  {
    // Set pin to input with internal pull up
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
  }

#if 1
  // Initialize Timer1
  Timer1.initialize(TIMER1_PERIOD_MICRO_SEC * TIMER1_PERIOD_CLOCK_FACTOR);
  Timer1.disablePwm(9);
  Timer1.disablePwm(10); 
  Timer1.attachInterrupt(HeaterTimerInterrupt);
#endif

#if 0
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
//    Serial.println("I");
 	ble_write_string((byte *)"I", 1);  
  	timer1Counter++;
#if 1  
  if(timer1Counter >= TIMER1_COUNTER_WRAP)
  {
  	timer1Counter = 0;
  }
#endif
//  Serial.println(timer1Counter); 
}

static byte buf_len = 0;
//------------------------------------------
// Main Loop
//------------------------------------------
byte queryDone = false;
uint32_t liveCount = 0;
void loop()
{
  liveCount++;
  if((liveCount % 100000) == 0) {
     Serial1.println("@");
    ble_write_string((byte *)"@", 1);
  }   
  // Process Blue Tooth Command if available
  while(ble_available())
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
        	double tempC;
			tempC = getTempThermocouple(zoneUpperFront);
			tempC = getTempThermocouple(zoneUpperRear);
			tempC = getTempThermocouple(zoneLowerFront);
			tempC = getTempThermocouple(zoneLowerRear);
        }
        break;
        
      case 's':	// Start Pizza Oven Cycle
        {
			Serial1.println("Start Pizza Oven Cycle");
#if 0			
//			if(poStateMachine.isInState(stateStandby) || 
//				poStateMachine.isInState(stateCoolDown))
				poStateMachine.transitionTo(stateHeatCycle);
#endif
        }
        break;

      case 'q':	// Exit Pizza Oven Cycle
        {
			Serial1.println("Exit Pizza Oven Cycle");
        }
        break;
        
	  default:	
		;	
    }	
// 	poStateMachine.update();  
 }
// 	poStateMachine.update();
 	
 	// send out any outstanding data
    ble_do_events();
    buf_len = 0;
}

//------------------------------------------
//state machine stateStandby 
//------------------------------------------
//State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

void stateStandbyEnter()
{
	Serial.println("stateStandbyEnter");
}

void stateStandbyUpdate()
{
}

void stateStandbyExit()
{
}

//------------------------------------------
//state machine stateHeatCycle 
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);

void stateHeatCycleEnter()
{
	Serial.println("stateHeatCycleEnter");
	ble_write_string((byte *)"HC",2);
	// Check if Upper Front Heater is Enabled
	if(heaterParmsUpperFront.enabled == true) 
	{
	
	}
}

void stateHeatCycleUpdate()
{
	Serial.println("stateHeatCycleUpdate");
		ble_write_string((byte *)"HU",2);
}

void stateHeatCycleExit()
{
}

//------------------------------------------
//state machine stateHeatCycleComplete 
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter, 
//	stateCoolDownUpdate, stateCoolDownExit);

void stateCoolDownEnter()
{
}

void stateCoolDownUpdate()
{
}

void stateCoolDownExit()
{
}

//------------------------------------------
// Cooling Fan Control
//------------------------------------------
void CoolingFanControl(boolean control)
{
	if(control == true)
	{
  		digitalWrite(COOLING_FAN_SIGNAL, HIGH);	
	}
	else
	{
  		digitalWrite(COOLING_FAN_SIGNAL, LOW);	
	}
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
	case zoneUpperFront:
//		degreesC = thermocoupleUpperFront.readCelsius();
		Serial1.print("tempUF");
		ble_write_string((byte *)"tcUF", 4);
		break;
  	case zoneUpperRear:
//  		degreesC = thermocoupleUpperRear.readCelsius();
  		Serial1.print("tempUR");
  		ble_write_string((byte *)"tcUR", 4);
  		break;
  	case zoneLowerFront:
//  		degreesC = thermocoupleLowerFront.readCelsius();
  		Serial1.print("tempLF");
  		ble_write_string((byte *)"tcLF", 4);
  		break;
  	case zoneLowerRear:
 // 		degreesC = thermocoupleLowerRear.readCelsius();
  		Serial1.print("tempLR");
  		ble_write_string((byte *)"tcLR", 4);  		
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
