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

#ifndef MSB_OF_U16
  #define MSB_OF_U16(v) (uint8_t)((v>>8)&0x00ff) 
#endif
#ifndef LSB_OF_U16
  #define LSB_OF_U16(v) (uint8_t)(v&0x00ff) 
#endif

enum serialCommWrapperEnum {
   STX = 0xff,
   ESC = 0xfe
};

static SerialCommWrappperMessageReceivedHandler m_msgReceivedHandler = NULL;
static SerialCommWrapperSendByteFunc m_sendByte = NULL;

// structures for handling reception
#define SERIAL_RECEIVE_BUF_SIZE 64
static uint16_t m_recvCrcCalced;
static uint16_t m_recvCrcRecvd;
static uint8_t m_recvBufIndex;
static uint8_t m_recvBuf[SERIAL_RECEIVE_BUF_SIZE];
static uint8_t m_recvExpectedDataLength;

static void waitForStx(uint8_t data);
static void waitForLength(uint8_t data);
static void waitForLengthAfterEsc(uint8_t data);
static void waitForPacketByte(uint8_t data);
static void waitForPacketByteAfterEsc(uint8_t data);
static void waitForCrcMsb(uint8_t data);
static void waitForCrcMsbAfterEsc(uint8_t data);
static void waitForCrcLsb(uint8_t data);
static void waitForCrcLsbAfterEsc(uint8_t data);
typedef void (*ReceiveStateMachineHandler)(uint8_t data);
ReceiveStateMachineHandler receiveState = waitForStx;
static void verifyReceivedPacket(void);

static void waitForStx(uint8_t data)
{
   if (data == STX)
   {
      receiveState = waitForLength;
      m_recvBufIndex = 0;
      m_recvCrcCalced = (uint16_t)crc_init();
   }
}
static void waitForLength(uint8_t data)
{
   switch(data)
   {
      case ESC:
         receiveState = waitForLengthAfterEsc;
         break;
      case STX:
         waitForStx(data);
         break;
      default:
         m_recvExpectedDataLength = data;
         if (m_recvExpectedDataLength > SERIAL_RECEIVE_BUF_SIZE)
         {
            receiveState = waitForStx;
         }
         else
         {
            receiveState = waitForPacketByte;
         }
   }
}
static void waitForLengthAfterEsc(uint8_t data)
{
   if (data == ESC || data == STX)
   {
      m_recvExpectedDataLength = data;
      if (m_recvExpectedDataLength > SERIAL_RECEIVE_BUF_SIZE)
      {
         receiveState = waitForStx;
      }
      else
      {
         receiveState = waitForPacketByte;
      }
   }
   else
   {
      receiveState = waitForStx;
   }
}
static void acceptReceivedPacketByte(uint8_t data)
{
   m_recvBuf[m_recvBufIndex++] = data;
   m_recvCrcCalced = (uint16_t)crc_update(m_recvCrcCalced, &data, 1);
   if (m_recvBufIndex >= m_recvExpectedDataLength)
   {
      receiveState = waitForCrcMsb;
   }
   else
   {
      receiveState = waitForPacketByte;
   }
}
static void waitForPacketByte(uint8_t data)
{
   switch(data)
   {
      case ESC:
         receiveState = waitForPacketByteAfterEsc;
         break;
      case STX:
         waitForStx(data);
         break;
      default:
         acceptReceivedPacketByte(data);
   }
}
static void waitForPacketByteAfterEsc(uint8_t data)
{
   switch(data)
   {
      case ESC:
      case STX:
         acceptReceivedPacketByte(data);
         break;
      default:
         receiveState = waitForStx;
   }
}
static void waitForCrcMsb(uint8_t data)
{
   switch(data)
   {
      case ESC:
         receiveState = waitForCrcMsbAfterEsc;
         break;
      case STX:
         waitForStx(data);
         break;
      default:
         m_recvCrcRecvd = (uint16_t)(data << 8);
         receiveState = waitForCrcLsb;
   }
}
static void waitForCrcMsbAfterEsc(uint8_t data)
{
   switch(data)
   {
      case ESC:
      case STX:
         m_recvCrcRecvd = (uint16_t)(data << 8);
         receiveState = waitForCrcLsb;
         break;
      default:
         receiveState = waitForStx;
   }
}
static void waitForCrcLsb(uint8_t data)
{
   switch(data)
   {
      case ESC:
         receiveState = waitForCrcLsbAfterEsc;
         break;
      case STX:
         waitForStx(data);
         break;
      default:
         m_recvCrcRecvd = (uint16_t)(m_recvCrcRecvd + data);
         verifyReceivedPacket();
         receiveState = waitForStx;
   }
}
static void waitForCrcLsbAfterEsc(uint8_t data)
{
   switch(data)
   {
      case ESC:
      case STX:
         m_recvCrcRecvd = (uint16_t)(m_recvCrcRecvd + data);
         verifyReceivedPacket();
         receiveState = waitForStx;
      default:
         receiveState = waitForStx;
   }
}
static void verifyReceivedPacket(void)
{
   if (m_recvCrcRecvd == m_recvCrcCalced)
   {
      if(m_msgReceivedHandler != NULL)
      {
         m_msgReceivedHandler(m_recvBuf, m_recvBufIndex);
      }
   }
}

static uint8_t isControlChar(uint8_t c) {
   switch(c) {
      case STX:
         return 1;
      case ESC:
         return 1;
      default:
         return 0;
   }
}

static void outputChar(uint8_t c) 
{
   if (isControlChar(c)) 
   {
      m_sendByte(ESC);
   }
   m_sendByte(c);
}

// initialize the serial comm wrapper
void serialCommWrapperInit(SerialCommWrapperSendByteFunc sendByte, SerialCommWrappperMessageReceivedHandler msgReceivedHandler)
{
   m_msgReceivedHandler = msgReceivedHandler;
   m_sendByte = sendByte;
}

void serialCommWrapperSendMessage(uint8_t *pData, uint8_t length)
{
   uint16_t crc = (uint16_t)crc_init();
   uint8_t i;

   if (m_sendByte == NULL)
   {
      return;
   }

   m_sendByte(STX);
   outputChar(length);

   // send packet
   for(i=0; i<length; i++) {
      crc = (uint16_t)crc_update(crc, &pData[i], 1);
      outputChar(pData[i]);
   }

   // send crc
   outputChar(MSB_OF_U16(crc));
   outputChar(LSB_OF_U16(crc));
}

void serialCommWrapperHandleByte(uint8_t data)
{
   (void)data;
}

void serialCommWrapperRun(void)
{

}
