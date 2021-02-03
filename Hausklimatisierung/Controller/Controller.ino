/*! \mainpage IT Project Air Conditioning by Selim Jaeschke and Raphael Biermann
 *
 * \section intro_sec Welcome!
 *
 *  This is the documentation to the project code file of our project!
 *  We built an air conditioning controller unit that will work flawless in your house.
 *  
 *  Note that you have to connect more Arduino modules according to the Fritzing diagram on the mainpage of Controller
 *  in order to use this program to its full extend.
 *  
 * \section intro_sec What does this unit provide?
 * -PI controlled Heating
 * 
 * -PI controlled Cooling
 * 
 * -Efficient fresh air exchange with heat-exchanger
 * 
 * -Temperature timetable for the perfect temperature at every time of the day
 * 
 * -Display to show current data, active modules and warnings
 * 
 * -buttons for custom settings
 * 
 * -Alarm sounds to corresponding display warnings
 *
 * You can find more notes and diagrams in the project folders.
 *
 *\section Usage
 *You can type a custom setpoint value for the temperature using s=x.
 *
 *Note that we saved setpoint values inside the controller instead of the simulation to reduce lag.
 *
 *Hold down Buttons to change values.
 *
 *Enjoy!
 *
 */


/*! mainpage Controller
@file Controller.ino
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

![Plant and controller with final addons](../../fritzingfinal.png)

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

Nov 2020 Jaeschke, Biermann implementation of full 
</pre>
*/

/*! \page Commands Manual Commands

The Controller running on an Arduino will transfer manual commands from the 
USB/COM port to the I²C port and displays the outcome from a connected house simulation.

<table border="0" width="80%">
<tr><td> H=n </td><td> heating power in % </td></tr>      //commands can be handled by both plant and controller
<tr><td> E=n </td><td> heat exchanger amount setting in % </td></tr>
<tr><td> e=n </td><td> heat exchanger transfer setting in % </td></tr>
<tr><td> P=n </td><td> no of persons </td></tr>
<tr><td> S=n </td><td> temperature setpoint, not used </td></tr>
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
<tr><td> S? </td><td> get temperature setpoint, not used! </td></tr> //similar to soll, defined in plant
<tr><td> w? </td><td> get winter or summer setting </td></tr> //w is boolean variable for winter, 1 is winter, 0 summer
<tr><td> W? </td><td> get warning if any </td></tr>
<tr><td> f? </td><td> get fixed outside state </td></tr>
<tr><td> R </td><td> (re)init </td></tr>
<tr><td> V=x </td><td> verbose on/off for the house </td></tr>
<tr><td> v=x </td><td> verbose on/off for the controller </td></tr>
<tr><td> s=n </td><td> edit setpoint temperature inside controller </td></tr> //New extra command for setting the Setpoint over Command Prompt.

</table>
*/

// include standard Arduino library
#include "Arduino.h"
// include standard I²C library
#include "Wire.h"
// include I²C connection
#include "I2C_Master.h" //prepared i2c protocol for string exchange
#include "string.h"
#include "LiquidCrystal_I2C.h"  //loaded i2c protocol for display
LiquidCrystal_I2C lcd(0x27, 16, 2); ///<initialising display with i2c adress 0x27, 16 characters, 2 rows

using namespace std;
                                                // time management
//! for 10 msec detection
unsigned long   msecPreviousMillis = 0;
//! counter for 10 msec up to 100 msec
int             nCount10msec = 0;
//! counter for 100 msec up to 1sec
int             nCount100msec = 0;

                                                // I2C
//! Address for I²C slave
const int       I2C_PLANT_ADDR = 10;  //more slaves possible with other adresses, master doesnt have adress
//! standard 100KHz for I²C
const long      I2C_FREQUENCY = 100000L;

                                                // live data from plant over I²C bus
double          dTime = 0.0;                    ///< simulation time in min
double          dIndoorTemperature = 0.0;       ///< indoor temperature in °C
double          dIndoorHumidity = 0.0;          ///< indoor humidity in %
int             nWarnings = 0;                  ///< warning bits
bool            bVerbose = true;                ///< local verbose flag

//Own Variables



int             humiditysp = 50;                      ///<setpoint for optimal humidity (should be 40-60%)             
double          soll = 20.0;                    ///<Setpoint temperature, temperature aimed for in all tasks
int             nHeating;                       ///<percentage for radiator heat


int             nWinter=0; ///< needed in order to know if its winter or summer
double          dAC; ///< AC level in percentage
double          dOutdoorTemperature; ///< outdoor temperature for comparing with indoor temperature
double          dOutdoorHumidity; ///< outdoor humidity for exchanging
double             Tcounter = 0; ///< Counter used in thermostat function in order to enable a soft switch between radiator and ac
int             lcdmode; ///< used for deciding what to show on the display

double          dHET; ///<Heat exchanger transfer
double          dHEA; ///<Heat exchanger amount

double          dEnergy; ///< total heating energy sum


double          dCO2; ///< Indoor CO2 value. Should be <1000 for decent air quality.
bool            systemOn=1;                 ///< activation switch for the system. Not used for anything right now. Could be used for vacation mode (normally you wouldn't turn off the system tho, because e.g freezing danger. Rather set low standard temp (winter)).
bool            bDaytime;                   ///< Boolean for night and day


//! Banner and version number
const char      szBanner[] = "# House Air Conditioner Controller V3.04";
//! usual Arduino pin 13 LED
const int       LEDpin = 13;
const int       UpButtonPin = 2; ///< pin for button to increase temperature
const int       DownButtonPin = 3; ///< pin for button to decrease temperature
bool             DownButtonState=0; ///< button pressed?
bool             UpButtonState=0;  ///< button pressed?

bool            userInput=0; ///<has there been a custom setpoint temperature change?
bool             toneOn; ///<is a tone playing?
int             toneCounter; //&<for the duration of the sound
int             lcdWarning; ///<what warning is to be shown on the display?

char            s1[2]; ///< String for saving value of custom setpoint value from command prompt 

double iIntegralH; ///< Integral part of PI Controller for Radiator
double iIntegralAC;///< Integral part of PI Controller for Cooling
double iIntegralHEH; ///< Integral part of PI Controller for Heat exchanger in heating mode
double iIntegralHEC; ///< Integral part of PI Controller for Heat exchanger in cooling mode

byte moonChar[] = { ///<custom chars for lcd display
 B00011,
  B01111,
  B01110,
  B11100,
  B11100,
  B01110,
  B01111,
  B00011
};

byte sunChar[] = {  ///<custom chars for lcd display
  B00100,
  B10101,
  B01110,
  B11111,
  B11111,
  B01110,
  B10101,
  B00100
};

byte thermometerChar[] = {  ///<custom chars for lcd display
  B00100,
  B01010,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110
};


byte dropChar[] = { ///<custom chars for lcd display
  B00100,
  B00100,
  B01010,
  B01010,
  B10001,
  B10001,
  B10001,
  B01110
};



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
  pinMode(DownButtonPin, INPUT);
  pinMode(UpButtonPin, INPUT);

  Serial.begin(115200);                         // set up serial port for 9600 Baud
#if defined (__AVR_ATmega32U4__)
  while ( ! Serial )                            // wait for serial port, ATmega32U4 chips only
    ;
#endif
  //Serial.println(szBanner);                     // show banner and version
  
  msecPreviousMillis = millis();                // init global timing, counts milliseconds +1 

  I2C_Master_Setup(I2C_FREQUENCY);              // start I²C master //100000L frequency

  //Own Additions 
  lcd.init(); //starting lcd
lcd.backlight();

lcd.createChar(0, moonChar);
lcd.createChar(1, sunChar);
lcd.createChar(2, thermometerChar);
lcd.createChar(3, dropChar);
  
}

//! Check if a command has been typed
/*!
Check if a command has been typed over COM-Port.

\param szCommand storage for a typed command
\param nCommandLengthMax size of szCommand
\returns true if a command has been typed
*/
bool CheckIfTypedAvailable(char szCommand[], int nCommandLengthMax) //if command manually typed, command is parameter char szcommand
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
  if ( szCommand[0] == 'v' ) //verbose off and on
  {
    if ( szCommand[1] == '=' )
      bVerbose = ( szCommand[2] == '1' );
    return true;
  }
   if ( szCommand[0] == 's' )
  {
    if ( szCommand[1] == '=' )

      userInput=1;   //manual overwrite of setpoint temperature
      s1[0]=szCommand[2]; //temporary char for saving the temperature value
      s1[1]=szCommand[3];
      
      soll=atof(s1); //convert string to double
      
      
    return true;
  }
  
  
  return false;
}


void ReglerHeizung(){ //double variable for radiator is being defined 
  double cTemp = dIndoorTemperature; //save current temperature
  //PAnteil
  double tFehler = (soll)-cTemp; //offset so nHeating is not directly zero after temperature is higher than setpoint
  int pAnteil = 7; //proportional coefficient
  
  //!IAnteil
  double iAnteil = 0.1; //integral coefficient
   iIntegralH+= (soll-cTemp)*iAnteil; //calculating the integral

 if(iIntegralH>100){ //limiter
  iIntegralH=100;
 }

 if(iIntegralH < 0){
  iIntegralH=0;
 }
  
  nHeating = pAnteil * tFehler + iIntegralH; //summing integral and proportional part of controller
  
  if(nHeating > 100){ //limiter, nHeating is allowed to be max 100%
    nHeating = 100;
  }
  if(nHeating < 0){
    nHeating = 0;
  }

}

/*!
 * This function uses a simple integral and linear regulator to control the AC unit.
 * It's basically a copy of the radiator function with inverted heat analysis in order to get a higher AC level in percentage when the temperature inside is higher than
 * the setpoint.
 * 
 * Since it is a copy, look up into "ReglerHeizung" function to see what this function does exactly
 * 
 *
 * 
 */

void ACController(){
double cTemp = dIndoorTemperature; 
  //PAnteil
  double tFehler = -((soll)-cTemp);
  int pAnteil = 5;
  //Serial.println(tFehler);
  //IAnteil
  double iAnteil = 0.1;
   iIntegralAC+= -(soll-cTemp)*iAnteil;

 if(iIntegralAC>100){ //limiter
  iIntegralAC=100;
 }

 if(iIntegralAC < 0){
  iIntegralAC=0;
 }
   
  dAC = pAnteil * tFehler + iIntegralAC;

  if(dAC > 100){ //limiter
    dAC = 100;
  }
  if(dAC < 0){
    dAC = 0;
  }


}


/*!
 * The Heat Exchanger is useful when the outside temperature is desired inside the house
 * This function uses the heat exchanger as a support for the radiator when the outside temperature is higher.
 * 
 * Its a straight copy of the radiator function since it should be controlled with the same regulator to achieve the setpoint temperature.
 * .
 */

void HEControllerHeat(){

 double cTemp = dIndoorTemperature; 
  
  //PAnteil
  double tFehler = (soll)-cTemp;
  int pAnteil = 5;
  //Serial.println(tFehler);
  //IAnteil
  double iAnteil = 0.1;
   iIntegralHEH+= (soll-cTemp)*iAnteil;

 if(iIntegralHEH>100){ //limiter
  iIntegralHEH=100;
 }

 if(iIntegralHEH < 0){
  iIntegralHEH=0;
 }
  
  dHEA = pAnteil * tFehler + iIntegralHEH;
  
  if(dHEA > 100){ //limiter
    dHEA = 100;
  }
  if(dHEA < 0){
    dHEA = 0;
  }
}





  



/*!
 * The Heat Exchanger is useful when the outside temperature is desired inside the house
 * This function uses the heat exchanger as a substitude to the air conditioner when the outside temperature is lower.
 */


void HEControllerCool(){

double cTemp = dIndoorTemperature; 
  //PAnteil
  double tFehler = -((soll)-cTemp);
  int pAnteil = 5;
  //Serial.println(tFehler);
  //IAnteil
  double iAnteil = 0.1;
   iIntegralHEC+= -(soll-cTemp)*iAnteil;

 if(iIntegralHEC>100){ //limiter
  iIntegralHEC=100;
 }

 if(iIntegralHEC < 0){
  iIntegralHEC=0;
 }
  
  
  dHEA = pAnteil * tFehler + iIntegralHEC;


  
  if(dHEA > 100){ //limiter
    dHEA = 100;
  }
  if(dHEA < 0){
    dHEA = 0;
  }



}


/*!
 * Works similar to a thermostat with a heat sensitive bending metal. Since the AC should not just turn on after the temperature rises above a certain level,
 * we need a counter to give each unit a margin, in which it can operate without being turned off.
 * 
 */


void thermostat(){ //negative Counter means summer


if(dIndoorTemperature > soll+1){
  Tcounter--;
}
if(dIndoorTemperature < soll-1){
  Tcounter++;
}


  if(Tcounter > 25){ //Vibrating only allowed for max 25s
    Tcounter = 25;
  }
  if (Tcounter < -25){
    Tcounter = -25;
  }
  //Serial.println(Tcounter);
}

/*!
 * Since we dont want the house to be hot at night in order to have a good sleep, the Setpoint temperature is set to be lower at night.
 * This also saves energy, since the heating unit does not have to turn on a lot from 10pm to 7am because the outdoor temperature falls 
 * to a similar temperature.
 * 
 * 
 * 
 */


void schedule(){

if(dTime < 20){
userInput = 0;
}

if (userInput == 0){ //if no useroverwrite of temperature, continue schedule.
if(dTime < (60*7)){ //low temperature until 6am
  soll = 17;
  bDaytime=0;
}else if(dTime < (60*22)){ //high temperature from 6am to 10pm
  soll = 21;
  bDaytime=1;
}else{ //low temperature from 10pm to 0am.
  bDaytime=0;
  soll=17;
}

}
}

/*!
 * Basic logic for what to do: 
 * This function decides based on the temperature conditions inside and outside, whether to turn on the radiator or air-conditioner.
 * It relies on Tcounter, which is gives a margin before turning on the AC or radiator in both winter and summer.
 * This is an upgrade considering most thermostat manufacturers actually rely on a winter or summer mode in order to decide which 
 * unit has to be turned on.
 * 
 */


void logic(){

double dTolerance=3;

if(systemOn){ //if on switch pressed
if(Tcounter > 0){  //if heating is suggested
  ReglerHeizung();
  iIntegralAC=0; //Reset of other values
  iIntegralHEC=0;
  lcdmode=1; //shows Radiator Setting on screen
  if(soll < dOutdoorTemperature){ //if its warmer outside then we can let in air
    HEControllerHeat();
  }else{
    dHEA=0; //heat exchanger amount
    dHET=0; //heat transfer
  }
}else{
  nHeating = 0;
}

if(Tcounter < 0){
  ACController();
  iIntegralH=0;
  iIntegralHEH=0;
  lcdmode=0;
  if(soll > dOutdoorTemperature){
    HEControllerCool(); //pure freshair exchange
  }else{
    dHEA=0; //heat exchanger amount 
    dHET=0; //heat transfer 
  }
}else{
  dAC = 0;
}
}

if((dCO2 > 900) or (dIndoorHumidity > 60) or (dIndoorHumidity < 30)){  //! improving air quality without ruining the indoor temperature
  dHEA = 100;
  
  dHET = 100;
}

  
}

 /*! 
  * Experimental warning sounds for events like too high CO2 Concentration / warning bits
  * Also added some corresponding display information.
*/


void sounds(){

toneCounter++;




if((dCO2 > 1000 or nWarnings == 2)){ //noise for one second (100*10ms) //CO2 above max ratings
if(toneCounter < 100){

  tone(4, 500);
}
else{
  noTone(4);
  
}
}

if(nWarnings == 1){ //noise for one second (100*10ms) //freezing inside house
if(toneCounter < 100){

  tone(4, 500);
}
else{
  noTone(4);
  
}
}



if(nWarnings == 4){ //noise for one second (100*10ms) //above 45°C
lcdWarning=3;
if(toneCounter < 100){

  tone(4, 500);
}
else{
  noTone(4);
  
}
}

if(toneCounter > 200){
  toneCounter = 0;
}

if(dCO2 < 1000 and nWarnings == 0){ //avoid continuous tone
  noTone(4);  
}

  
//if(dCO2 > 100){
//  tone(4, 300);
//}else{
//  noTone(4);
//}
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

bool CreateNextSteadyCommand(char szCommand[]) //for automatic output, the values have to be requested and sent first, this function is called every 100ms 
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
  case 5:                                       // request warnings
    
    strcpy(szCommand, "H=");                // build command
    itoa(nHeating, szCommand+2, 10);
    
    break;
  case 6:                                       // request winter setting
    strcpy(szCommand, "w?");                    // build command
    break;
  case 7:                                       // send AC setting
    
    strcpy(szCommand, "F=");                // build command
    itoa(dAC, szCommand+2, 10);
    
    break;
  case 8:                                       // request outside temperature
    strcpy(szCommand, "O?");                    // build command
    break;
  case 9:                                       // request outside humidity
    strcpy(szCommand, "o?");                    // build command
    break;
  case 10:                                       // send Heat exchanger amount setting
    
    strcpy(szCommand, "E=");                // build command
    itoa(dHEA, szCommand+2, 10);
    
    break;
   case 11:                                       // send Heat exchanger amount setting
    
    strcpy(szCommand, "e=");                // build command
    itoa(dHET, szCommand+2, 10);
    
    break;
  case 12:                                       // request CO2 concentration
    strcpy(szCommand, "C?");                    // build command
    break;
    case 13:                                       // request energy consumption
    strcpy(szCommand, "k?");                    // build command
    break;
  
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

Only ask plant for values we really need! Plant crashes occasionally otherwise.


\param szResponse storage for a typed command
\returns true if response has been used
*/
bool InterpreteResponse(char szResponse[]) //saving responses in local variables
{
  if ( szResponse[1] == '=' ) //szResponse is a string sent by the plant. Since the 2nd letter always has to be a "=" to be a proper response, it can be used as a responsecheck by the controller
  {                                             // seems to be a valid response
    switch ( szResponse[0] )
    {
    case 'T':                                   // got a fresh dTime value
      dTime = atof(szResponse+2);               // convert response part after '=' to double //the +2 is for selecting chars after = sign
      return true;                              // done
    case 'I':                                   // got a fresh indoor temperature value
      dIndoorTemperature = atof(szResponse+2);  // convert response part after '=' to double
      return true;                              // done
    case 'i':                                   // got a fresh indoor humidity value
      dIndoorHumidity = atof(szResponse+2);     // convert response part after '=' to double
      return true;                              // done
    case 'W':                                   // got a fresh warning bits value
      nWarnings = atoi(szResponse+2);           // convert response part after '=' to integer //the bits the warning consists of are important for handling errors
      return true;                              // done
    case 'w':                                   // got a fresh winter value
      nWinter = atoi(szResponse+2);          
      return true;  
    case 'O':                                   // got a fresh Outdoor temperature value
      dOutdoorTemperature = atoi(szResponse+2);           
      return true;   
    case 'o':                                   // got a fresh Outdoor humidity value
      dOutdoorHumidity = atoi(szResponse+2);           
      return true;
    case 'C':                                   // got a fresh CO2 value
      dCO2 = atoi(szResponse+2);           
      return true; 
    case 'k':                                   // got a fresh Energy value
      dEnergy = atoi(szResponse+2);           
      return true; 

          
    // more cases may follow //understand responses for manual questions
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
// if ( bVerbose )
//    Serial.print("T=");
//  Serial.print(dTime/60);                       // time in hours
//  Serial.print(" ");
// 
// if ( bVerbose )
//    Serial.print("i=");
//  Serial.print(dIndoorHumidity);
//  Serial.print(" ");
//  if ( bVerbose )
//    Serial.print("W=0x");
//  Serial.print(nWarnings, HEX);
//  Serial.println("");
if ( bVerbose )
    Serial.print("I=");
  Serial.println(dIndoorTemperature);
  Serial.print(" ");

//if ( bVerbose )
//    Serial.print("E=");
//  Serial.println(dEnergy);
//  Serial.print(" ");  
//  if ( bVerbose )
//    Serial.print("o=");
//  Serial.println(dOutdoorHumidity);
//  Serial.print(" ");  
//    if ( bVerbose )
//    Serial.print("C=");
//  Serial.println(dCO2);
//  Serial.print(" ");  
//if ( bVerbose )
//    Serial.print("O=");
//  Serial.println(dOutdoorTemperature);
//  Serial.print(" ");
  
//
//if ( bVerbose )
//    Serial.print("HEA=");
//  Serial.print(dHEA);
//  Serial.print(" ");
//if ( bVerbose )
//    Serial.print("HET=");
//  Serial.print(dHEA);
//  Serial.print(" ");  
//if ( bVerbose )
//    Serial.print("HEA=");
//  Serial.print(dHEA);
//  Serial.print(" ");
//  if ( bVerbose )
//    Serial.print("H=");
//  Serial.print(nHeating);
//  Serial.print(" ");
//  if ( bVerbose )
//    Serial.print("dAC=");
//  Serial.print(dHEA);
//  Serial.print(" ");
}

//! Function Task_10ms called every 10 msec
void Task_10ms()  //no delays allowed in automatisation
{
                                             // nothing to do so far

//SaveHumidity();        //Saving current indoor humidity every 10ms

sounds();
  
}

//! Function Task_100ms called every 100 msec
/*!
I²C communication and keyboard input.
 */
void Task_100ms() //most important, has to be done under 100ms
{
  static char  szCommand[I2C_DATA_MAX+1];       // buffer for commands
  static char  szResponse[I2C_DATA_MAX+1];      // buffer for responses
  static bool  bOperatesCommand = false;        // flag tells if request is under way


//SaveTemps();          //Saving current temp every 100ms


  if ( ! bOperatesCommand )                     // if not busy at working on a current command
  {
    if ( CheckIfTypedAvailable(szCommand, I2C_DATA_MAX+1) ) //check for manual command, command is saved in szcommand
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

  if ( bOperatesCommand )                        // working on a current command //If Command still available, send it to plant
  {
    *szResponse = 0;                            // clean response
    int           nRes;
    nRes = I2C_SendRequest(I2C_PLANT_ADDR, szCommand); // request command from I²C slave //SENDING COMMAND
    if ( nRes == -2 )       //error handling
      Serial.println("request too long");
    if ( *szCommand == 'R' )                    // handle special "reset" request
      bOperatesCommand = false;                 // no response expected after reset
  }

  long          nResponseTime;
  int           nSlaveNo;
  int           nResult = I2C_GetResponse(&nSlaveNo, szResponse, &nResponseTime); //saving the slave response in szResponse (char array), slave adress important (for display?)
  if ( nResult >= 0 )
  {
    if ( ! InterpreteResponse(szResponse) )     // use response we got
    {
#if 0                                           // possibly disable //Response for entered command
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



//Additions
//ReglerHeizung(); //Calling radiator for temperature response
//ACController();
//HEController();

logic();








}




void lcdTask(){
  //LCD Management


lcd.clear();

//1st Row
  lcd.setCursor(0, 0);//Hier wird die Position des ersten Zeichens festgelegt. In diesem Fall bedeutet (0,0) das erste Zeichen in der ersten Zeile. 

switch (nWarnings){
  case 2:
  
  lcd.print("High CO2!");
  lcd.setCursor(0, 1);
  lcd.print(" C=");
  lcd.print(dCO2);
  lcd.print("ppm");

  break;
 case 1:

    lcd.print("It's freezing! ");
    lcd.setCursor(0, 1);
    lcd.write(2);
    lcd.print(dIndoorTemperature);
  break;
case 4:
    lcd.print("It's too hot! ");
    lcd.setCursor(0, 1);
    lcd.write(2);
    lcd.print(dIndoorTemperature);
    break;

default:

if(dCO2>1000){
  lcd.print("High CO2!");
  lcd.setCursor(0, 1);
  lcd.print(" C=");
  lcd.print(dCO2);
  lcd.print("ppm");
}else{

lcd.write(2); //thermometer icon
lcd.print(dIndoorTemperature);
lcd.print("C");

if(!bDaytime){
lcd.write(0); //output moon
}else{
lcd.write(1);  //output sun
}

lcd.print("SP=");
lcd.print(soll); 

//2nd Row
lcd.setCursor(0, 1); // In diesem Fall bedeutet (0,1) das erste Zeichen in der zweiten Zeile.

if(lcdmode == 1){ 
lcd.print("H="); 
lcd.print(nHeating);
lcd.print("%");
}
if(lcdmode == 0){
  lcd.print("AC="); 

lcd.print((int)dAC);
lcd.print("%");
}


//lcd.print(" C=");
//lcd.print((int) dCO2);
//lcd.print("%");

lcd.print(" ");
lcd.write(3); //drop of water icon
lcd.print((int)dIndoorHumidity);
lcd.print("%");

//lcd.print(" k=");
//lcd.print((int)dEnergy);
//
//lcd.print(" ");
//lcd.print((int)(dTime/60));

}

}
}







//! Function Task_1s called every 1 sec
/*!
Use communication verbose flag (-v) to remove all but pure values.
This allows to use the integrated Arduino serial plotter or an external software like gnuplot.
*/
void Task_1s() //everything not so often needed
{


  
 //Serial.println(nWarnings);
 
  
  ToggleDigitalIOPort(LEDpin);                  // toggle output to LED

  ShowData();                                   // possibly remove later
  lcdTask();
  schedule();
  thermostat();


DownButtonState = digitalRead(DownButtonPin);
  UpButtonState = digitalRead(UpButtonPin);

if(DownButtonState ==HIGH){
  soll--;
  userInput = 1;
}

if(UpButtonState == HIGH){
  soll++;
  userInput = 1;
}

}



//! Usual arduino steadily called function
/*!
Usual arduino steadily called function.

This function will be called again and again as fast as possible.

It will dispatch the CPU power between tasks which are expected to be executed in some regular intervals.
Such intervals are often called sampling time.
*/
void loop() //calling primitive functions, mainly responsible for timing
{

 

  
  long  msecCurrentMillis = millis();
  if ( ( msecCurrentMillis - msecPreviousMillis ) < 10 )
    return;
  msecPreviousMillis = msecCurrentMillis;

  Task_10ms();                                  // call user 10 msec function
  if ( ++nCount10msec >= 10 ) //counted how often task 10ms was called, 10* is 100ms
  {
    nCount10msec = 0;
    Task_100ms();                               // call user 100 msec function
    if ( ++nCount100msec >= 10 )
    {
      nCount100msec = 0;
      Task_1s();                                // call user 1 sec function
    }
  }

  I2C_Master_Steady();                          // give background processing a chance //often calling to check for slave responses
  delay(1); //1ms
}
