/*
    Your code description here
    (c) you, 20XX
    All rights reserved.

    Functionality: 
    
    Version log:
    
    20XX-MM-DD:
        v1.0.0  - Initial release
        
*/
// ==== DEFINES ===================================================================================
#define NTP_TIMEOUT 1500

// ==== Debug and Test options ==================
#define _DEBUG_
//#define _TEST_

//===== Debugging macros ========================
#ifdef _DEBUG_
#define SerialD Serial
#define _PM(a) SerialD.print(millis()); SerialD.print(": "); SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)
#else
#define _PM(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#endif




// ==== INCLUDES ==================================================================================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESPNtpClient.h>

// ==== Uncomment desired compile options =================================
// #define _TASK_SLEEP_ON_IDLE_RUN  // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_TIMECRITICAL       // Enable monitoring scheduling overruns
// #define _TASK_STATUS_REQUEST     // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS            // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER        // Compile with support for local task storage pointer
// #define _TASK_PRIORITY           // Support for layered scheduling priority
// #define _TASK_MICRO_RES          // Support for microsecond resolution
// #define _TASK_STD_FUNCTION       // Support for std::function (ESP8266 and ESP32 ONLY)
// #define _TASK_DEBUG              // Make all methods and variables public for debug purposes
// #define _TASK_INLINE             // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT            // Support for overall task timeout
// #define _TASK_OO_CALLBACKS       // Support for dynamic callback method binding
// #define _TASK_DEFINE_MILLIS      // Force forward declaration of millis() and micros() "C" style
// #define _TASK_EXPOSE_CHAIN       // Methods to access tasks in the task chain
// #define _TASK_SCHEDULING_OPTIONS // Support for multiple scheduling options

#include <TaskScheduler.h>

// ==== GLOBALS ===================================================================================
WiFiManager wm;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

bool firstNTPConnection = false;

const PROGMEM char *ntpServer = "pool.ntp.org";

// ==== Scheduler ==============================
Scheduler ts;

void connectWifi();
void connectNTP();
void checkNTP();

// ==== Scheduling defines (cheat sheet) =====================
/*
TASK_MILLISECOND
TASK_SECOND
TASK_MINUTE
TASK_HOUR
TASK_IMMEDIATE
TASK_FOREVER
TASK_ONCE
TASK_NOTIMEOUT  

TASK_SCHEDULE     - schedule is a priority, with "catch up" (default)
TASK_SCHEDULE_NC  - schedule is a priority, without "catch up"
TASK_INTERVAL     - interval is a priority, without "catch up"
*/

// ==== Task definitions ========================
Task tConnectWifi(TASK_IMMEDIATE, TASK_ONCE, &connectWifi);
Task tConnectNTP(TASK_IMMEDIATE, TASK_ONCE, &connectNTP);
Task tCheckNTP(10 * TASK_SECOND, TASK_FOREVER, &checkNTP);

// ==== CODE ======================================================================================

void wifiEventHandler(WiFiEvent_t event)
{
  Serial.printf("Got Event: %d\n", event);
}

void onWifiConnect(const WiFiEventStationModeConnected &event)
{
  Serial.println("Station connected");
  tConnectNTP.enable();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
  Serial.println("Station disconnected");
  tCheckNTP.disable();
  NTP.stop();
}

void connectNTP()
{
  _PM("connectNTP()");
  NTP.setTimeZone(TZ_America_Bogota);
  NTP.setInterval(600);
  NTP.setNTPTimeout(NTP_TIMEOUT);
  NTP.begin(ntpServer);
  firstNTPConnection = true;
  tCheckNTP.enable();
}

/**************************************************************************/
/*!
    @brief    Standard Arduino SETUP method - initialize sketch
    @param    none
    @returns  none
*/
/**************************************************************************/
void setup()
{
      // put your setup code here, to run once:
#if defined(_DEBUG_) || defined(_TEST_)
  Serial.begin(9600);-
  delay(2000);
  _PL("Scheduler Template: setup()");
#endif
  Serial.setDebugOutput(true);
  WiFi.onEvent(wifiEventHandler);

  wifiConnectHandler = WiFi.onStationModeConnected(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  

  ts.addTask(tConnectWifi);
  ts.addTask(tConnectNTP);
  ts.addTask(tCheckNTP);

  tConnectWifi.enable();
}


/**************************************************************************/
/*!
    @brief    Standard Arduino LOOP method - using with TaskScheduler there 
              should be nothing here but ts.execute()
    @param    none
    @returns  none
*/
/**************************************************************************/
void loop() {
  ts.execute();
  wm.process();
  /*if ((WiFi.status() == WL_CONNECTED) && (firstNTPConnection)) 
  {
    tCheckNTP.enable();
    firstNTPConnection = false;
  }*/
}


/**************************************************************************/
/*!
    @brief    Connect wifi
    @param    none
    @returns  none
*/
/**************************************************************************/
void connectWifi() {
_PM("connectWifi()");
//  task code
  WiFi.mode(WIFI_STA);
  //wm.resetSettings();  // Erase stored configuration for testing
  wm.setConfigPortalBlocking(false);
  wm.autoConnect("SmartDisplayAP","password");
}


/**************************************************************************/
/*!
    @brief    Call ntp info
    @param    none
    @returns  none
*/
/**************************************************************************/
void checkNTP() {
_PM("checkNTP()");
//  task code
  
  _PL(NTP.getTimeDateStringUs());
}




