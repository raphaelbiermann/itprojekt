/* I²C Master
(c)2014..2020 Ingenieurbüro Dr. Friedrich Haase
*/

// include standard Arduino library
#include <Arduino.h>
// include standard I²C library
#include <Wire.h>
// include I²C master
#include "I2C_Master.h"

//! 16MHz CPU as for Arduino UNO and NANO
static const long     CPU_FREQ = 16000000L;

                                                // I²C communication data
//! last used slave
static int            nSlaveNoLast = 0;
//! response storage
static char           szResponse[I2C_DATA_MAX+1];
//! response storage length
static int            nResponseLength = 0;
//! last request time in usec
static unsigned long  nRequestTime = 0;
//! response time in usec
static unsigned long  nResponseTime = 0;
//! response timeout in usec
static unsigned long  nResponseTimeOut = 100000;

//! state machine states
enum I2C_State { I2C_READY, I2C_BUSY, I2C_DONE, I2C_TIMEOUT };
//! current I2C state
static I2C_State      nI2CState = I2C_READY;

// I²C master setup
void I2C_Master_Setup(long nFrequency)
{
  nI2CState = I2C_READY;
  TWBR = ( CPU_FREQ / nFrequency - 16 ) / 2;    // set I²C speed
//??  setClock(nFrequency);
  Wire.begin();                                 // start I²C bus as master
}

// I²C master steady call
void I2C_Master_Steady()
{
  switch ( nI2CState )
  {
  case I2C_READY:
    // wait for a new job
    break;
  case I2C_BUSY:
    if ( Wire.available() )
    {
      int   nReceived = 0;
      char  ch;
      while ( Wire.available() )
      {
        ch = Wire.read();                       // receive byte, typically I2C_DATA_MAX as requested
        if ( nReceived < I2C_DATA_MAX )
          szResponse[nReceived++] = ch;         // store received byte
        // else forget it
      }
      szResponse[nReceived] = 0;                // make sure there is a trailing 0
      nResponseLength = nReceived;
      nResponseTime = micros() - nRequestTime;
      nI2CState = I2C_DONE;                     // received complete, expect user fetches response
    }
    else if ( ( micros() - nRequestTime ) >= nResponseTimeOut )
    {                                           // timeout
      nResponseTime = micros() - nRequestTime;  //?? required
      nI2CState = I2C_TIMEOUT;
    }
    break;
  case I2C_DONE:
    // remain here until response has been delivered
    break;
  case I2C_TIMEOUT:
    // remain here until response has been asked
    break;
  }
  delay(1);
}

// Send request to slave
int I2C_SendRequest(int nSlaveNo, const char * const pszRequest)
{
  if ( nI2CState != I2C_READY )
    return -1;                                  // fail if not ready

  int  nLength = strlen(pszRequest);            // message length
  if ( nLength > I2C_DATA_MAX )
    return -2;                                  // fail, too long
  
  nI2CState = I2C_BUSY;                         // busy now
  nSlaveNoLast = nSlaveNo;
  nRequestTime = micros();
  Wire.beginTransmission(nSlaveNo);             // transmit to slave device
  Wire.write(pszRequest, nLength);              // send data
  Wire.endTransmission();                       // ends transmitting

  delay(1);
  Wire.requestFrom(nSlaveNo, I2C_DATA_MAX);     // request from slave

  return nLength;                               // request done
}

// Get text response for last request
int I2C_GetResponse(int * pnSlaveNo, char * pszResponse, unsigned long * pnResponseTime /*=nullptr*/)
{
  switch ( nI2CState )
  {
  case I2C_BUSY:
    return -1;                                  // fail if busy
  case I2C_READY:
    return -4;                                  // fail if no outstanding response
  case I2C_TIMEOUT:
    *pszResponse = 0;
    *pnSlaveNo = nSlaveNoLast;
    if ( pnResponseTime != nullptr )
      *pnResponseTime = nResponseTimeOut;
    nI2CState = I2C_READY;
    return -3;                                  // fail if timed out
  case I2C_DONE:
    strcpy(pszResponse, szResponse);            // return received answer
    *pnSlaveNo = nSlaveNoLast;
    if ( pnResponseTime != nullptr )
      *pnResponseTime = nResponseTime;
    nI2CState = I2C_READY;
    return nResponseLength;
  }
  return -1;                                    // should not happen, fail
}

// See if ready for a request
bool I2C_IsReady()
{
  return nI2CState == I2C_READY;
}

// See if request has been fulfilled
bool I2C_HasReply()
{
  return ( nI2CState == I2C_DONE ) || ( nI2CState == I2C_TIMEOUT );
}

// not used so far, possibly not required
#if 0
//! See if request in progress
/*!
See if request in progress.
\return true if I2C is busy on a previous command
*/
bool I2C_IsBusy()
{
  return nI2CState == I2C_BUSY;
}

//! See if request timed out
/*!
See if request timed out.
\return true if I2C previous command got no reply in due time
*/
bool I2C_HasTimeout()
{
  return nI2CState == I2C_TIMEOUT;
}
#endif
