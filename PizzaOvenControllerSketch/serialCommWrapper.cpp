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

#include "serialCommWrapper.h"

#include <stdio.h>

#ifndef MSB_OF_U16
  #define MSB_OF_U16(v) (uint8_t)((v>>8)&0x00ff) 
#endif
#ifndef LSB_OF_U16
  #define LSB_OF_U16(v) (uint8_t)(v&0x00ff) 
#endif

enum serialCommWrapperEnum {
   STX = '@',
   ESC = 0xfe
};

static SerialCommWrappperMessageReceivedHandler m_msgReceivedHandler = NULL;
static SerialCommWrapperSendByteFunc m_sendByte = NULL;

// structures for handling reception
#define SERIAL_RECEIVE_BUF_SIZE 64
static uint16_t m_recvCrcCalced;
static uint8_t m_recvBufIndex;
static uint8_t m_recvBuf[SERIAL_RECEIVE_BUF_SIZE];
static uint8_t m_dataBytesReceived;

static void waitForStx(uint8_t data);
static void waitForData(uint8_t data);
static void waitForLineFeed(uint8_t data);
typedef void (*ReceiveStateMachineHandler)(uint8_t data);
static ReceiveStateMachineHandler receiveState = waitForStx;
static void verifyReceivedPacket(void);

static void waitForStx(uint8_t data)
{
   if (data == STX)
   {
      receiveState = waitForData;
      m_recvBufIndex = 0;
      m_dataBytesReceived = 0;
      m_recvCrcCalced = (uint16_t)crc_init();
   }
}

static void waitForData(uint8_t data)
{
   if (data == STX) {
      waitForStx(data);
      return;
   }
   m_recvBuf[m_recvBufIndex++] = data;
   if (m_recvBufIndex >= SERIAL_RECEIVE_BUF_SIZE)
   {
      receiveState = waitForStx;
   }
   else if (data == '[')
   {
      receiveState = waitForLineFeed;
   }
   else
   {
      m_dataBytesReceived++;
      m_recvCrcCalced = (uint16_t)crc_update(m_recvCrcCalced, &data, 1);
   }
}

static void waitForLineFeed(uint8_t data)
{
   if (data == STX) {
      waitForStx(data);
      return;
   }
   m_recvBuf[m_recvBufIndex++] = data;
   if (m_recvBufIndex >= SERIAL_RECEIVE_BUF_SIZE)
   {
      receiveState = waitForStx;
   }
   else if (data == '\n')
   {
      verifyReceivedPacket();
      receiveState = waitForStx;
   }
}

static bool isValidAsciiHexDigit(uint8_t digit)
{
   bool retVal = false;

   if (digit >= '0' && digit <= '9') retVal = true;
   if (digit >= 'a' && digit <= 'f') retVal = true;
   if (digit >= 'A' && digit <= 'F') retVal = true;

   return retVal;
}

static uint8_t asciiHexByteToNum(uint8_t *pBuf)
{
   uint8_t i;
   uint8_t val=0;
   uint8_t base = '0';


   for(i=0; i<2; i++)
   {
      val = (uint8_t)(val << 4);
      base = '0';
      if (pBuf[i] >= 'a' && pBuf[i] <= 'f')
      {
         base = 'a' - 10;
      }
      else if (pBuf[i] >= 'A' && pBuf[i] <= 'F')
      {
         base = 'A' - 10;
      }
      val = (uint8_t)(val + (pBuf[i] - base));
   }

   return val;
}

static void verifyReceivedPacket(void)
{
   //                           1111111111000000000
   //                           9876543210987654321
   // end of packet should look like: [0x12,0x34]rn
   uint16_t crcSent;
   
   if (m_recvBuf[m_recvBufIndex - 13] != '[') 
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 12] != '0')
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 11] != 'x')
   {
      return;
   }
   if (!isValidAsciiHexDigit(m_recvBuf[m_recvBufIndex - 10]))
   {
      return;
   }
   if (!isValidAsciiHexDigit(m_recvBuf[m_recvBufIndex - 9]))
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 8] != ',')
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 7] != '0')
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 6] != 'x')
   {
      return;
   }
   if (!isValidAsciiHexDigit(m_recvBuf[m_recvBufIndex - 5]))
   {
      return;
   }
   if (!isValidAsciiHexDigit(m_recvBuf[m_recvBufIndex - 4]))
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 3] != ']')
   {
      return;
   }
   if (m_recvBuf[m_recvBufIndex - 2] != '\r')
   {
      return;
   }
   crcSent = asciiHexByteToNum(&m_recvBuf[m_recvBufIndex - 10]);
   crcSent = (uint16_t)((crcSent<<8) + asciiHexByteToNum(&m_recvBuf[m_recvBufIndex - 5]));
   if (crcSent == m_recvCrcCalced)
   {
      if(m_msgReceivedHandler != NULL)
      {
         m_msgReceivedHandler(&m_recvBuf[0], m_dataBytesReceived);
      }
   }
}

// initialize the serial comm wrapper
void serialCommWrapperInit(SerialCommWrapperSendByteFunc sendByte, SerialCommWrappperMessageReceivedHandler msgReceivedHandler)
{
   m_msgReceivedHandler = msgReceivedHandler;
   m_sendByte = sendByte;
}

static uint8_t valToAsciiDigit(uint8_t val)
{
   uint8_t base = 'a' - 10;

   val = val & 0x0f;
   if (val < 10) base = '0';
   return (uint8_t)(base + val);
}

static void byteToStringHex(uint8_t *pBuf, uint8_t val)
{
   pBuf[0] = valToAsciiDigit((uint8_t)((val >> 4)&0xf));
   pBuf[1] = valToAsciiDigit((uint8_t)(val&0xf));
}

void serialCommWrapperSendMessage(uint8_t *pData, uint8_t length)
{
   uint16_t crc = (uint16_t)crc_init();
   uint8_t i;
   //                   000000000011111
   //                   012345678901234
   uint8_t crcText[] = "[0x12,0x34]\r\n";

   if (m_sendByte == NULL)
   {
      return;
   }

   m_sendByte('@');

   // send packet
   for(i=0; i<length; i++) {
      crc = (uint16_t)crc_update(crc, &pData[i], 1);
      m_sendByte(pData[i]);
   }

   // send crc
   byteToStringHex(&crcText[3], (uint8_t)(crc>>8));
   byteToStringHex(&crcText[8], (uint8_t)(crc&0xff));
   for(i=0; crcText[i]!=0; i++) 
   {
      m_sendByte(crcText[i]);
   }
}

void serialCommWrapperHandleByte(uint8_t data)
{
   receiveState(data);
}

void serialCommWrapperRun(void)
{

}
