/*! \mainpage Controller

Simple house air conditioner Controller.

The Controller running on an Arduino will transfer manual commands from the 
USB/COM port to the physical ports or to the I²C port and displays the outcome from a connected plant.

Initially this controller program is just a dummy to get you started.

The controller uses a simple non-preemptive multitasking.

Sections:
- \link Controller.ino Controller \endlink
- \link Commands Manual Commands \endlink
- \link I2C_Master I2C_Master \endlink

Arduinos for Plant (top) and controller (below).

![Plant and controller below](../../UNO2UNO_I2C.png)

When using multiple I2C slaves note that
Master.SendRequest(...)
serves only one slave at a time.

<pre>
(c)2013-2020 Ingenieurbüro Dr. Friedrich Haase
             Consulting - Automatisierungstechnik
</pre>

<pre>
Versions
July 2014 Dr. Haase  created
Sept 2015 Dr. Haase  converted for IT project
Sept 2015 Dr. Haase  some minor corrections
Oct 2016  Dr. Haase  exchanged toFloat and toInt by atof and atoi
Sept 2017 Dr. Haase  minor improvements
Aug  2018 Dr. Haase  minor improvements
Jan 2020  Dr. Haase  redesigned for better C/C++ coding
Mai 2020  Dr. Haase  party mode
Aug 2020  Dr. Haase  complete redesign
</pre>
*/

/*! \page Commands Manual Commands

The Controller running on an Arduino will transfer manual commands from the 
USB/COM port to the I²C port and displays the outcome from a connected house simulation.

<table border="0" width="80%">
<tr><td> H=n </td><td> heating power in % </td></tr>
<tr><td> E=n </td><td> heat exchanger amount setting in % </td></tr>
<tr><td> e=n </td><td> heat exchanger transfer setting in % </td></tr>
<tr><td> P=n </td><td> no of persons </td></tr>
<tr><td> S=n </td><td> temperature setpoint </td></tr>
<tr><td> w=x </td><td> winter or summer </td></tr>
<tr><td> f=x </td><td> fixed outside state </td></tr>
<tr><td> T? </td><td> get simulation time in seconds </td></tr>
<tr><td> I? </td><td> inside temperature </td></tr>
<tr><td> O? </td><td> outside temperature </td></tr>
<tr><td> H? </td><td> get heater setting in % </td></tr>
<tr><td> E? </td><td> get heat exchanger amount </td></tr>
<tr><td> e? </td><td> get heat exchanger transfer in % </td></tr>
<tr><td> i? </td><td> inside humidity </td></tr>
<tr><td> o? </td><td> outside humidity </td></tr>
<tr><td> C? </td><td> amount of CO2 </td></tr>
<tr><td> k? </td><td> heating energy tally </td></tr>
<tr><td> S? </td><td> get temperature setpoint </td></tr>
<tr><td> w? </td><td> get winter or summer setting </td></tr>
<tr><td> W? </td><td> get warning if any </td></tr>
<tr><td> f? </td><td> get fixed outside state </td></tr>
<tr><td> R </td><td> (re)init </td></tr>
<tr><td> V=x </td><td> verbose on/off for the house </td></tr>
<tr><td> v=x </td><td> verbose on/off for the controller </td></tr>
</table>
*/

// include standard Arduino library
#include "Arduino.h"
// include standard I²C library
#include "Wire.h"
// include I²C connection
#include "I2C_Master.h"

                                                // time management
//! for 10 msec detection
unsigned long   msecPreviousMillis = 0;
//! counter for 10 msec up to 100 msec
int             nCount10msec = 0;
//! counter for 100 msec up to 1sec
int             nCount100msec = 0;

                                                // I2C
//! Address for I²C slave
const int       I2C_PLANT_ADDR = 10;
//! standard 100KHz for I²C
const long      I2C_FREQUENCY = 100000L;

                                                // live data from plant over I²C bus
double          dTime = 0.0;                    ///< simulation time in min
double          dIndoorTemperature = 0.0;       ///< indoor temperature in °C
double          dIndoorHumidity = 0.0;          ///< indoor humidity in %
int             nWarnings = 0;                  ///< warning bits
bool            bVerbose = true;                ///< local verbose flag

//! Banner and version number
const char      szBanner[] = "# House Air Conditioner Controller V3.04";
//! usual Arduino pin 13 LED
const int       LEDpin = 13;

//! Toggle digital IO port
/*!
Toggle digital IO port.

\param nPin IO pin number
*/
void ToggleDigitalIOPort(int nPin)
{
  //// read, invert, write
  digitalWrite(nPin, ! digitalRead(nPin));      // read, invert, write
}

//! usual arduino init function
void setup()
{
  pinMode(LEDpin, OUTPUT);                      // init arduino LED pin as an output

  Serial.begin(115200);                         // set up serial port for 9600 Baud
#if defined (__AVR_ATmega32U4__)
  while ( ! Serial )                            // wait for serial port, ATmega32U4 chips only
    ;
#endif
  Serial.println(szBanner);                     // show banner and version

  msecPreviousMillis = millis();                // init global timing

  I2C_Master_Setup(I2C_FREQUENCY);              // start I²C master
}

//! Check if a command has been typed
/*!
Check if a command has been typed over COM-Port.

\param szCommand storage for a typed command
\param nCommandLengthMax size of szCommand
\returns true if a command has been typed
*/
bool CheckIfTypedAvailable(char szCommand[], int nCommandLengthMax)
{
  *szCommand = 0;                               // initially empty result
  if ( Serial.available() > 0 )
  {                                             // work on manual command
    int   ch;
    int   nCommandLen = 0;
    while ( ( ch = Serial.read() ) > 0 )
    {
      if ( nCommandLen < nCommandLengthMax )
      {
        szCommand[nCommandLen++] = ch;          // collect characters
        szCommand[nCommandLen] = 0;             // append a trailing zero byte
      }
    }
    return true;                                // command complete
  }
  else
    return false;
}

//! Filter local commands
/*!
Work on local commands which should not be transfered to the house.
\param szCommand command to check
\returns true if a command was a local command
*/
bool FilterLocalCommands(char szCommand[])
{
  if ( szCommand[0] == 'v' )
  {
    if ( szCommand[1] == '=' )
      bVerbose = ( szCommand[2] == '1' );
    return true;
  }
  return false;
}

//! Create next steady transmitted command
/*!
Create next of steadily transmitted requests or commands to the plant.

Assume the buffer is large enough for all automatically created commands.

The state machine here may be expanded to support more commands.
Commands are created to cyclically transmit and/or request values to or from the plant.

Please note the state value nIndex has to be static to remain in existence after leaving the function.
The index is incremented in every call and reset to zero after all commands have been generated once.

\param szCommand storage for a typed command
\returns true if a command has been created
*/
bool CreateNextSteadyCommand(char szCommand[])
{
  *szCommand = 0;                               // initially empty

  static int   nIndex = 0;                      // index walking through requests
  switch ( ++nIndex )
  {
  case 1:                                       // request time
    strcpy(szCommand, "T?");                    // build command
    break;
  case 2:                                       // request indoor temperature
    strcpy(szCommand, "I?");                    // build command
    break;
  case 3:                                       // request indoor humidity
    strcpy(szCommand, "i?");                    // build command
    break;
  case 4:                                       // request warnings
    strcpy(szCommand, "W?");                    // build command
    break;
  // more cases for other requests or settings
  default:
    nIndex = 0;                                 // start over
    break;
  }
  return ( *szCommand != 0 );                   // return true if command buffer not empty
}

//! Interpret an I²C response from the plant
/*!
Interpret an I²C response from the plant.

If more command types have been issued in function CreateNextSteadyCommand() the responses from the plant will return here.

\param szResponse storage for a typed command
\returns true if response has been used
*/
bool InterpreteResponse(char szResponse[])
{
  if ( szResponse[1] == '=' )
  {                                             // seems to be a valid response
    switch ( szResponse[0] )
    {
    case 'T':                                   // got a fresh dTime value
      dTime = atof(szResponse+2);               // convert response part after '=' to double
      return true;                              // done
    case 'I':                                   // got a fresh indoor temperature value
      dIndoorTemperature = atof(szResponse+2);  // convert response part after '=' to double
      return true;                              // done
    case 'i':                                   // got a fresh indoor humidity value
      dIndoorHumidity = atof(szResponse+2);     // convert response part after '=' to double
      return true;                              // done
    case 'W':                                   // got a fresh warning bits value
      nWarnings = atoi(szResponse+2);           // convert response part after '=' to integer
      return true;                              // done
    // more cases may follow
    }
  }
  return false;                                 // response not handled
}

//! Show some data values
/*!
Show some data values

Remember, each character at 9600 Baud requires about 1 msec.
*/
void ShowData()
{                                               // just to show a result
  if ( bVerbose )
    Serial.print("T=");
  Serial.print(dTime/60);                       // time in hours
  Serial.print(" ");
  if ( bVerbose )
    Serial.print("I=");
  Serial.print(dIndoorTemperature);
  Serial.print(" ");
  if ( bVerbose )
    Serial.print("i=");
  Serial.print(dIndoorHumidity);
  Serial.print(" ");
  if ( bVerbose )
    Serial.print("W=0x");
  Serial.print(nWarnings, HEX);
  Serial.println("");
}

//! Function Task_10ms called every 10 msec
void Task_10ms()
{
  ;                                              // nothing to do so far
}

//! Function Task_100ms called every 100 msec
/*!
I²C communication and keyboard input.
 */
void Task_100ms()
{
  static char  szCommand[I2C_DATA_MAX+1];       // buffer for commands
  static char  szResponse[I2C_DATA_MAX+1];      // buffer for responses
  static bool  bOperatesCommand = false;        // flag tells if request is under way

  if ( ! bOperatesCommand )                     // if not busy at working on a current command
  {
    if ( CheckIfTypedAvailable(szCommand, I2C_DATA_MAX+1) )
    {
      bOperatesCommand = true;                  // we have a new manual command to work on
      if ( FilterLocalCommands(szCommand) )
        bOperatesCommand = false;               // was a local command
    }
    else if ( CreateNextSteadyCommand(szCommand) )
    {
      bOperatesCommand = true;                  // we have a new generated command to work on
    }
  }

  if ( bOperatesCommand )                       // working on a current command
  {
    *szResponse = 0;                            // clean response
    int           nRes;
    nRes = I2C_SendRequest(I2C_PLANT_ADDR, szCommand); // request something from I²C slave
    if ( nRes == -2 )
      Serial.println("request too long");
    if ( *szCommand == 'R' )                    // handle special "reset" request
      bOperatesCommand = false;                 // no response expected after reset
  }

  long          nResponseTime;
  int           nSlaveNo;
  int           nResult = I2C_GetResponse(&nSlaveNo, szResponse, &nResponseTime);
  if ( nResult >= 0 )
  {
    if ( ! InterpreteResponse(szResponse) )     // use response we got
    {
#if 1                                           // possibly disable
      Serial.print(" -> ");                     // show not handled command and response
      Serial.println(szResponse);
#endif
    }
    bOperatesCommand = false;                   // note command complete done
  }
  else if ( nResult == -1 )
    ;//Serial.println("busy");
  else if ( nResult == -3 )
  {
    Serial.print(nSlaveNo);
    Serial.println(":timeout");
    bOperatesCommand = false;                   // note command complete done
  }
  else if ( nResult == -4 )
    ;//Serial.println("no response yet");
}

//! Function Task_1s called every 1 sec
/*!
Use communication verbose flag (-v) to remove all but pure values.
This allows to use the integrated Arduino serial plotter or an external software like gnuplot.
*/
void Task_1s()
{
  ToggleDigitalIOPort(LEDpin);                  // toggle output to LED

  ShowData();                                   // possibly remove later
}

//! Usual arduino steadily called function
/*!
Usual arduino steadily called function.

This function will be called again and again as fast as possible.

It will dispatch the CPU power between tasks which are expected to be executed in some regular intervals.
Such intervals are often called sampling time.
*/
void loop()
{
  long  msecCurrentMillis = millis();
  if ( ( msecCurrentMillis - msecPreviousMillis ) < 10 )
    return;
  msecPreviousMillis = msecCurrentMillis;

  Task_10ms();                                  // call user 10 msec function
  if ( ++nCount10msec >= 10 )
  {
    nCount10msec = 0;
    Task_100ms();                               // call user 100 msec function
    if ( ++nCount100msec >= 10 )
    {
      nCount100msec = 0;
      Task_1s();                                // call user 1 sec function
    }
  }

  I2C_Master_Steady();                          // give background processing a chance
  delay(1);
}
