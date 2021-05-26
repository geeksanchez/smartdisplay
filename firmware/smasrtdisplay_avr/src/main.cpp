#include <Arduino.h>
#include "Task.h"
#include "TaskScheduler.h"
#include "Wire.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP085_U.h"
#include "UTFT.h"
#include "TouchScreen.h"

// Declare which fonts we will be using
extern uint8_t SmallFont[];
//***********************************************//
// If you use OPEN-SMART TFT breakout board                 //
// You need to add 5V-3.3V level converting circuit.
// Of course you can use OPEN-SMART UNO Black version with 5V/3.3V power switch,
// you just need switch to 3.3V.
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
//----------------------------------------|
// TFT Breakout  -- OPEN-SMART UNO Black /Red
// GND              -- GND
// 3V3               -- 3.3V
// CS                 -- A3
// RS                 -- A2
// WR                -- A1
// RD                 -- 3.3V
// RST                -- A0
// LED                -- GND
// DB0                -- 8
// DB1                -- 9
// DB2                -- 10
// DB3                -- 11
// DB4                -- 4
// DB5                -- 13
// DB6                -- 6
// DB7                -- 7

//
// Remember to change the model parameter to suit your display module!
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_RS A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RST 5 // 

UTFT tft(ILI9225,LCD_RS,LCD_WR,LCD_CS,LCD_RST);

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*****************************************************************************************
*	Class:		Debugger
*	Task Type:	Task (always runs)
*	Purpose:	This expands on Alan Burlison's original example code which demonstrates
*				a task that reads from the serial port and echoes to the Serial Monitor.
*				I've expanded it so that other classes use a pointer to the debugger object
*				to output simple debug messages while this example executes.
*				Classes that use the debugger object are passed a reference to &debugger
*				in their respective constructors.
*
*				For example: Blinker(uint8_t _pin, uint32_t _rate, Debugger *_ptrDebugger);
*
*				To output debug information use: ptrDebugger->debugWrite("debug info");
*
******************************************************************************************/

// ***
// *** Define the Debugger Class as type Task
// ***
class Debugger : public Task
{
public:
	Debugger();
	void debugWrite(String debugMsg);	//Used for simple debugging of other tasks
	virtual void run(uint32_t now);		//Override the run() method
	virtual bool canRun(uint32_t now);	//Override the canRun() method
};

// ***
// *** Debugger Constructor
// ***
Debugger::Debugger()
	: Task()
	{
		Serial.begin(57600);
	}

// ***
// *** Debugger::canRun() <--checked by TaskScheduler
// ***
bool Debugger::canRun(uint32_t now)
{
	return Serial.available() > 0;
}

// ***
// *** Debugger::run() <--executed by TaskScheduler as a result of canRun() returning true.
// ***
void Debugger::run(uint32_t now)
{
	uint16_t byteCount = 0;
	
	Serial.println("-----------------");
	Serial.println("Input Received...");
	Serial.println("-----------------");
	while (Serial.available() > 0) {
		int byte = Serial.read();
		Serial.print("'") ;
		Serial.print(char(byte));
		Serial.print("' = ");
		Serial.print(byte, DEC);
		Serial.println(" ");
		if (byte == '\r') {
			Serial.print('\n', DEC);
		}
		
		byteCount++;
	}
	
	Serial.println("-----------------");
	Serial.print("Bytes Received: "); Serial.println(String(byteCount));
	Serial.println("-----------------");
	
}

// ***
// *** Debugger::debugWrite() <--provides basic debug info from other tasks
// ***
void Debugger::debugWrite(String debugMsg)
{
	Serial.println(debugMsg);
}

/****************************************************************************************
*	Class:		Display
*	Task Type:	TriggeredTask (normally dormant, triggered by sensors via ptrDisplay->setRunnable())
*	Purpose:	Shows sensor data in the display whenever sensors update measurements.
*
*****************************************************************************************/

// ***
// *** Define the Display Class as type TriggeredTask
// ***
class Display : public TriggeredTask
{
  public:
    Display(Debugger *_ptrDebugger);
    virtual void run(uint32_t now);
    void setUV(uint8_t _uv);
    void setPressure(float _pressure);
    void setTemperature(float _temperature);
    void setAltitude(float _altitude);
  private:
    uint8_t uv;
    float pressure;
    float temperature;
    float altitude;
    Debugger *ptrDebugger;
};

// ***
// *** Display Constructor
// ***
Display::Display(Debugger *_ptrDebugger)
	:TriggeredTask(),
  uv(0),
  pressure(0.0),
  temperature(0.0),
  altitude(0.0),
  ptrDebugger(_ptrDebugger)
  {
    // Setup the LCD
    tft.InitLCD();
    tft.setFont(SmallFont);
    tft.setColor(VGA_WHITE);
    tft.clrScr();
	}

// ***
// *** Display::setUV() <--updates uv level via ptrDisplay->setUV(uv)
// ***
void Display::setUV(uint8_t _uv)
{
  uv = _uv;
}

// ***
// *** Display::setPressure() <--updates pressure via ptrDisplay->setPressure(pressure)
// ***
void Display::setPressure(float _pressure)
{
  pressure = _pressure;
}

// ***
// *** Display::setTemperature() <--updates temperature via ptrDisplay->setTemperature(temperature)
// ***
void Display::setTemperature(float _temperature)
{
  temperature = _temperature;
}

// ***
// *** Display::setAltitude() <--updates altitude via ptrDisplay->setAltitude(altitude)
// ***
void Display::setAltitude(float _altitude)
{
  altitude = _altitude;
}

// ***
// *** Display::run() <--executed by TaskScheduler as a result of canRun() returning true,
// ***							 in this case, sensors utilizing ptrDisplay->setRunnable()
// ***
void Display::run(uint32_t now)
{
  String s = "Rayos UV = " + String(uv) + "    ";
	tft.print(s, LEFT, 30);
  ptrDebugger->debugWrite(s);
  s = "Temperatura = " + String(temperature) + " C    ";
	tft.print(s, LEFT, 60);
  ptrDebugger->debugWrite(s);
  s = "Presion = " + String(pressure) + " hPa    ";
	tft.print(s, LEFT, 90);
  ptrDebugger->debugWrite(s);
  s = "Altitud = " + String(altitude) + " m    ";
	tft.print(s, LEFT, 120);
  ptrDebugger->debugWrite(s);
  // ***
	// *** resetRunnable() IMPORTANT! IMPORTANT!
	// *** It's important to resetRunnable() after executing a TriggeredTask.
	// *** If bool runFlag in a TriggeredTask is not reset, the TriggeredTask will
	// *** continue to run indefinitely which defeats its purpose. It will stay dormant
	// *** and be ignored by the TaskScheduler until triggered again.
	// ***
	resetRunnable();
}

/*****************************************************************************************
*	Class:		uvSensor
*	Task Type:	TimedTask (runs when a specific time interval is reached)
*	Purpose:	This class read the uv sensor value.  The output value
*				is the average of the samples defined.
*
******************************************************************************************/

// ***
// *** Define the uvSensor Class as type TimedTask
// ***
class uvSensor : public TimedTask
{
  public:
    uvSensor(uint8_t _pinRef, uint8_t _pinValue, uint32_t _rate, Display *_ptrDisplay, Debugger *_ptrDebugger);
    virtual void run(uint32_t now);
  private:
    uint8_t pinRef;
    uint8_t pinValue;
    uint32_t rate;
    uint8_t count;
    uint32_t value;
    Display *ptrDisplay;
    Debugger *ptrDebugger;		// Pointer to debugger
};

uvSensor::uvSensor(uint8_t _pinRef, uint8_t _pinValue, uint32_t _rate, Display *_ptrDisplay, Debugger *_ptrDebugger)
  : TimedTask(millis()),
    pinRef(_pinRef),
    pinValue(_pinValue),
    rate(_rate),
    count(0),
    value(0),
  	ptrDisplay(_ptrDisplay),
  	ptrDebugger(_ptrDebugger)
{
  pinMode(pinRef, INPUT);
  pinMode(pinValue, INPUT);
}

void uvSensor::run(uint32_t now)
{
  int uvLevel = analogRead(pinValue);
  int refLevel = analogRead(pinRef);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
  value += round(uvIntensity);
  count++;
//  String s = "UV level:" + String(value); 
//  ptrDebugger->debugWrite(s);

  if (count > 10) {
    value = value / count;
    count = 0;
    value = 0;
    ptrDisplay->setUV((uint8_t)value);
  	ptrDisplay->setRunnable();
  }

  incRunTime(rate);
}

/*****************************************************************************************
*	Class:		BMP180Sensor
*	Task Type:	TimedTask (runs when a specific time interval is reached)
*	Purpose:	This class read the BMP180 sensor values.
*
******************************************************************************************/

// ***
// *** Define the uvSensor Class as type TimedTask
// ***
class BMP180Sensor : public TimedTask
{
  public:
    BMP180Sensor(uint32_t _rate, Display *_ptrDisplay, Debugger *_ptrDebugger);
    virtual void run(uint32_t now);
  private:
    uint32_t rate;
    bool enabled;
    Adafruit_BMP085_Unified bmp180;
    float pressure;
    float temperature;
    float altitude;
    uint8_t count;
    Display *ptrDisplay;
    Debugger *ptrDebugger;		// Pointer to debugger
};

BMP180Sensor::BMP180Sensor(uint32_t _rate, Display *_ptrDisplay, Debugger *_ptrDebugger)
  : TimedTask(millis()),
    rate(_rate),
    enabled(true),
    bmp180(Adafruit_BMP085_Unified(10085)),
    pressure(0),
    temperature(0),
    altitude(0),
    count(0),
  	ptrDisplay(_ptrDisplay),
  	ptrDebugger(_ptrDebugger)
{
  if(!bmp180.begin())
  {
    enabled = false;
		ptrDebugger->debugWrite("BMP180: NOT FOUND!!!");
  }
}

void BMP180Sensor::run(uint32_t now)
{
  if(enabled)
  {
    /* Get a new sensor event */ 
    sensors_event_t event;
    bmp180.getEvent(&event);
  
    /* Display the results (barometric pressure is measure in hPa) */
    if (event.pressure)
    {
      /* Display atmospheric pressue in hPa */
    //  String s = "Pressure: " + String(event.pressure) + " hPa";
    //  ptrDebugger->debugWrite(s);
      float p = event.pressure;

      
      /* Calculating altitude with reasonable accuracy requires pressure    *
      * sea level pressure for your position at the moment the data is     *
      * converted, as well as the ambient temperature in degress           *
      * celcius.  If you don't have these values, a 'generic' value of     *
      * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
      * in sensors.h), but this isn't ideal and will give variable         *
      * results from one day to the next.                                  *
      *                                                                    *
      * You can usually find the current SLP value by looking at weather   *
      * websites or from environmental information centers near any major  *
      * airport.                                                           *
      *                                                                    *
      * For example, for Paris, France you can check the current mean      *
      * pressure and sea level at: http://bit.ly/16Au8ol                   */
      
      /* First we get the current temperature from the BMP085 */
      float t;
      bmp180.getTemperature(&t);
    //  s = "Temperature: " + String(temperature) + " C";
    //  ptrDebugger->debugWrite(s);

      /* Then convert the atmospheric pressure, and SLP to altitude         */
      /* Update this next line with the current SLP for better results      */
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    //  s = "Altitude: " + String(bmp180.pressureToAltitude(seaLevelPressure, event.pressure)) + " m";
    //  ptrDebugger->debugWrite(s);
      float a = bmp180.pressureToAltitude(seaLevelPressure, event.pressure);

      pressure += p;
      temperature += t;
      altitude += a;
      count++;

      if (count > 10) {
        pressure /= count;
        temperature /= count;
        altitude /= count;

        ptrDisplay->setPressure(pressure);
        ptrDisplay->setTemperature(temperature);
        ptrDisplay->setAltitude(altitude);
        ptrDisplay->setRunnable();

        pressure = 0;
        temperature = 0;
        altitude = 0;
        count = 0;
      }

    }
  }
  incRunTime(rate);
}


/*****************************************************************************************
*  Start of the application
******************************************************************************************/
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
	Debugger debugger;
  Display display(&debugger);
  uvSensor uvsensor(A7, A6, 1000, &display, &debugger);
  BMP180Sensor bmp180sensor(1000, &display, &debugger);
// ***
// *** Create an array of pointers (eek!) to the task objects we just instantiated.
// ***
// *** The order matters here.  When the TaskScheduler is running it finds the first task it can
// *** run--canRun(), runs the task--run(), then returns to the top of the list and starts the 
// *** process again. I've experimented with different orders, but couldn't find any astonishing
// *** differences; and considering this isn't a "real" application, this'll do.  One other note:
// *** of all the tasks, communicating with the Serial Monitor via the Debugger object is the by 
// *** far the biggest tax on the MCU, but can be removed fairly from your final release. Have fun.
// ***
	Task *tasks[] = {
		
		&debugger,		
		&display,
		&uvsensor,
		&bmp180sensor
				
	};
	
	// ***
	// *** Instantiate the TaskScheduler and fill it with tasks.			
	// ***
	TaskScheduler scheduler(tasks, NUM_TASKS(tasks));
	
	// GO! Run the scheduler - it never returns.
	scheduler.runTasks();

}