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
//#include "Boards.h"
#include "Adafruit_MAX31855.h"
#include "FiniteStateMachine.h"

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

//------------------------------------------
// Software SPI Thermocouple Definitions
//------------------------------------------
Adafruit_MAX31855 thermocoupleUpperFront(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_UPPER_FRONT, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleUpperRear(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_UPPER_REAR, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleLowerFront(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_LOWER_FRONT, SW_SPI_THERMO_DO);
Adafruit_MAX31855 thermocoupleLowerRear(SW_SPI_THERMO_CLK, SW_SPI_THERMO_CS_LOWER_REAR, SW_SPI_THERMO_DO);

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

//------------------------------------------
// Setup Routines
//------------------------------------------
void setup()
{
  Serial1.begin(9600); 
  Serial.println("BLE Arduino Slave");

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

  // Default pins set to 9 and 8 for REQN and RDYN
  // Set your REQN and RDYN here before ble_begin() if you need
  //ble_set_pins(3, 2);
  
  // Set your BLE Shield name here, max. length 10
  //ble_set_name("My Name");
  
  // Init. and start BLE library.
  ble_begin();
}

static byte buf_len = 0;
//------------------------------------------
// Main Loop
//------------------------------------------
byte queryDone = false;

void loop()
{
  while(ble_available())
  {
    byte cmd;
    cmd = ble_read();
    Serial.write(cmd);
    
    // Parse data here
    switch (cmd)
    {
      case 'V': // query protocol version
        {
          byte buf[] = {'V', 'a', 'b', 'c'};
          ble_write_string(buf, 4);
        }
        break;
            
      case 'Q': // Output TEmp
        {
          byte buf[4];
       Serial1.print("Internal Temp = ");
     Serial1.println(thermocoupleUpperFront.readInternal());
          double c = thermocoupleUpperFront.readCelsius();
    if (isnan(c)) {
     Serial1.println("Something wrong with thermocouple!");
   } else {
     Serial1.print("C = "); 
     Serial1.println(c);
   }
          byte* p = (byte*)(void*)&c;
         for (int i = 0; i < sizeof(c); i++)
             buf[i]=*p++;

         // ble_write_string(buf, 4);
          ble_write_string((byte *)"hello", 5);
        }
        break;
        
      case 'Z':
        {
          byte len = ble_read();
          byte buf[len];
          for (int i=0;i<len;i++)
            buf[i] = ble_read();
          Serial.println("->");
          Serial.print("Received: ");
          Serial.print(len);
          Serial.println(" byte(s)");
          Serial.print(" Hex: ");
          for (int i=0;i<len;i++)
            Serial.print(buf[i], HEX);
          Serial.println();
        }
    }

    // send out any outstanding data
    ble_do_events();
    buf_len = 0;
    
    return; // only do this task in this loop
  }

  // process text data
  if (Serial.available())
  {
    byte d = 'Z';
    ble_write(d);

    delay(5);
    while(Serial.available())
    {
      d = Serial.read();
      ble_write(d);
    }
    
    ble_do_events();
    buf_len = 0;
    
    return;    
  }

#if 0
  // No input data, no commands, process analog data
  if (!ble_connected())
    queryDone = false; // reset query state
    
  if (queryDone) // only report data after the query state
  { 
    byte input_data_pending = reportDigitalInput();  
    if (input_data_pending)
    {
      ble_do_events();
      buf_len = 0;
      
      return; // only do this task in this loop
    }
    
    reportPinAnalogData();
    
    ble_do_events();
    buf_len = 0;
    
    return;  
  }
 #endif 
    
  ble_do_events();
  buf_len = 0;
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
		degreesC = thermocoupleUpperFront.readCelsius();
		Serial1.print("tempUF");
		ble_write_string((byte *)"tcUF", 4);
		break;
  	case zoneUpperRear:
  		degreesC = thermocoupleUpperRear.readCelsius();
  		Serial1.print("tempUR");
  		ble_write_string((byte *)"tcUR", 4);
  		break;
  	case zoneLowerFront:
  		degreesC = thermocoupleLowerFront.readCelsius();
  		Serial1.print("tempLF");
  		ble_write_string((byte *)"tcLF", 4);
  		break;
  	case zoneLowerRear:
  		degreesC = thermocoupleLowerRear.readCelsius();
  		Serial1.print("tempLR");
  		ble_write_string((byte *)"tcLR", 4);  		
  		break;
  	default:
  	    Serial1.print("Invalid tc!");
	}
    Serial1.println(degreesC);	
    degreesCx10 = (uint16_t) (degreesC + 0.05) * 10.0;
    ble_write_string((byte *)&degreesCx10, 4);  // send as a binary temp * 10
    
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

