/*
   serialCommWrapper
  
   Take an array of bytes, wrap them in a header, add a CRC, and send it out of
   the serial port.
   Receive bytes, looking for a valid wrapped packet.  When a valid packet is
   received, call the registerd callback with the payload.
  
Usage:
   To initialize the serial comm wrapper, call serialCommWrapperInit with a 
   pointer to a function that allows the serial comm wrapper to send a byte on 
   the serial port, and a pointer to a message handler when valid packets are 
   received.  
   
   To run, periodically call serialCommWrapperRun to exercise the state machine.
   Call serialCommWrapperHandleByte when a byte is received on the serial port.

  
   Copyright (c) 2015 FirstBuild

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.
   */

#ifndef SERIAL_COMM_WRAPPER_H
#define SERIAL_COMM_WRAPPER_H

#include <stdint.h>
#include "crc.h"

// Callback to handle a message received on the serial port.
typedef void (*SerialCommWrappperMessageReceivedHandler)(uint8_t *pData, uint8_t length);
// send a byte to the serial comm wrapper receiver
void serialCommWrapperHandleByte(uint8_t data);

// Callback for the serial comm wrapper to send bytes
typedef void (*SerialCommWrapperSendByteFunc)(uint8_t data);
// send a message 
void serialCommWrapperSendMessage(uint8_t *pData, uint8_t length);
// initialize the serial comm wrapper
void serialCommWrapperInit(SerialCommWrapperSendByteFunc sendByte, 
      SerialCommWrappperMessageReceivedHandler msgReceivedHandler);

void serialCommWrapperRun(void);

#endif // SERIAL_COMM_WRAPPER_H

