/*! \page I2C_Master I2C Master
Arduino I²C Master.

This I²C master uses a request/response scheme.
Response from a slave should always answer the last request.

<pre>
(c)2015-2020 IngenieurbÃ¼ro Dr. Friedrich Haase
             Consulting - Automatisierungstechnik
</pre>

<pre>
<b>Versions</b>
July 2015  Dr. Haase  created
Sept 2015  Dr. Haase  converted for IT project
Aug 2020   Dr. F. Haase  complete redesign
</pre>
*/

// include standard Arduino library
#include <Arduino.h>
// include standard I²C library
#include <Wire.h>

                                                // I²C attributes
//! max request and response length
const int       I2C_DATA_MAX = 132;

                                                // I²C prototypes
//! I²C master setup
/*!
I²C master setup
\param nFrequency I2C frequency
*/
extern void I2C_Master_Setup(long nFrequency);

//! I²C master steady call
/*!
I²C master steady call
*/
extern void I2C_Master_Steady();

//! Send request to slave
/*!
Send request to slave.
Expects a response.
\param nSlaveNo target slave number for the request
\param pszRequest request message, may be nullptr
\return >=0 on success, -1 if busy, -2 if too long
*/
extern int I2C_SendRequest(int nSlaveNo, const char * const pszRequest);

//! Get text response for last request
/*!
Gets a received text response.
\param pnSlaveNo slave number the request was targeted
\param pszResponse storage for received response
\param pnResponseTime duration between request transmit start and response delivery in microseconds
\returns >=0 length on success, -1 if busy, -3 if timed out, -4 while no outstanding response
*/
extern int I2C_GetResponse(int * pnSlaveNo, char * pszResponse, unsigned long * pnResponseTime = nullptr);

//! See if ready for a request
/*!
See if ready for a request.
\return true if I2C is ready
*/
extern bool I2C_IsReady();

//! See if request has been fulfilled
/*!
See if request has been fulfilled.
\return true if I2C previous command got a reply
*/
extern bool I2C_HasReply();
