/*
 */

#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include "string.h"
#include <stdint.h>
#include "serialCommWrapper.h"
#include <stdio.h>

static uint8_t sendBuffer[2048];
static uint16_t sendBufferIndex;
static uint8_t recvBuffer[2048];

static void messageReceivedHandler(uint8_t *pData, uint8_t length)
{
   memcpy(&recvBuffer[0], pData, length);
   mock().actualCall("messageReceivedHandler").withParameter("length", length);
}

static void sendByteFunc(uint8_t data)
{
   sendBuffer[sendBufferIndex++] = data;
}

TEST_GROUP(SerialCommWrapper)
{ 
   void setup()
   {
      serialCommWrapperInit(sendByteFunc, messageReceivedHandler);
      sendBufferIndex = 0;
   }
   void teardown()
   {
   }

   void whenNBytesAreReceived(uint16_t bytesToSend)
   {
      uint16_t i;
      uint8_t msg[300];
      uint8_t frame[300];
      uint16_t crc;
      uint8_t crcMsb;
      uint8_t crcLsb;
      uint8_t crcText[64];

      memset(msg, 0, sizeof(msg));
      memset(msg, 'a', bytesToSend);
      memset(frame, 0, sizeof(frame));

      frame[0] = '@';
      frame[1] = 0;
      strcat((char*)&frame[0], (char *)&msg[0]);

      crc = (uint16_t)crc_init();
      crc = (uint16_t)crc_update(crc, msg, bytesToSend);
      crcMsb = (uint8_t)(crc >> 8);
      crcLsb = (uint8_t)(crc & 0xff);
      sprintf((char *)&crcText[0], "[0x%02x,0x%02x]", crcMsb, crcLsb);
      strcat((char*)&frame[0], (char *)&crcText[0]);
      strcat((char*)&frame[0], "\r\n");

      for (i=0; i<strlen((const char *)&frame[0]); i++)
      {
         serialCommWrapperHandleByte(frame[i]);
      }
   }
};

TEST(SerialCommWrapper, testSendHappyPath)
{
   uint8_t msg[] = "123456789";
   uint8_t asSent[] = "@123456789[0x29,0xb1]\r\n";

   serialCommWrapperSendMessage(&msg[0], (uint8_t)strlen((const char *)&msg[0]));
   CHECK_EQUAL(strlen((const char *)&asSent[0]), sendBufferIndex);
   MEMCMP_EQUAL(asSent, sendBuffer, sendBufferIndex);
}

TEST(SerialCommWrapper, testSendNoBytesSentIfNoSendByteFunction)
{
   uint8_t msg[] = "123456789";

   serialCommWrapperInit(NULL, messageReceivedHandler);
   serialCommWrapperSendMessage(&msg[0], (uint8_t)strlen((const char *)&msg[0]));
   CHECK_EQUAL(0, sendBufferIndex);
}

TEST(SerialCommWrapper, testRecv_HappyPath)
{
   uint16_t i;
   uint8_t msg[] = "123456789";
   uint8_t frame[] = "@123456789[0x29,0xb1]\r\n";

   mock().expectOneCall("messageReceivedHandler").withParameter("length", 9);
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
   MEMCMP_EQUAL(&msg[0], &recvBuffer[0], 9);
}

TEST(SerialCommWrapper, testRecv_HappyPathWithUpperCaseCrc)
{
   uint16_t i;
   uint8_t msg[] = "123456789";
   uint8_t frame[] = "@123456789[0x29,0xB1]\r\n";

   mock().expectOneCall("messageReceivedHandler").withParameter("length", 9);
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
   MEMCMP_EQUAL(&msg[0], &recvBuffer[0], 9);
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfCrcIsBad)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xb0]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_CallHappensIfDataIsShort)
{
   uint16_t bytesToSend = 16;

   mock().expectOneCall("messageReceivedHandler").withParameter("length", bytesToSend);
   whenNBytesAreReceived(bytesToSend);
}

TEST(SerialCommWrapper, testRecv_CallDoesNotHappenIfTooMuchData)
{
   uint16_t bytesToSend = 64;

   mock().expectNCalls(0, "messageReceivedHandler").withParameter("length", bytesToSend);
   whenNBytesAreReceived(bytesToSend);
}

TEST(SerialCommWrapper, testRecv_PartialPacketDiscardedAndGoodPacketReceived)
{
   uint16_t i;
   uint8_t msg[] = "123456789";
   uint8_t frame[] = "@12345@123456789[0x29,0xb1]\r\n";

   mock().expectOneCall("messageReceivedHandler").withParameter("length", 9);
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
   MEMCMP_EQUAL(&msg[0], &recvBuffer[0], 9);
}

TEST(SerialCommWrapper, testRecv_PacketRestartedAfterDataReceived)
{
   uint16_t i;
   uint8_t msg[] = "123456789";
   uint8_t frame[] = "@123456789[0x29,@123456789[0x29,0xb1]\r\n";

   mock().expectOneCall("messageReceivedHandler").withParameter("length", 9);
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
   MEMCMP_EQUAL(&msg[0], &recvBuffer[0], 9);
}

TEST(SerialCommWrapper, testRecv_CallbackNotCalledIfTooMuchDataAfterEndOfDataDetected)
{
   uint16_t i;
   uint8_t frame[] = "@12345@123456789[0x29,ddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfOpeningBracketNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xb1] \r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfMsbZeroNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[ x29,0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfMsbXNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0 29,0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfMsbUpperNibbleNotValidAsciiHex)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0xi9,0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfMsbLowerNibbleNotValidAsciiHex)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x2i,0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfCommaNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29 0xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfLsbZeroNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29, xb1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfLsbXNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0 b1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfLsbUpperNibbleNotValidAsciiHex)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xi1]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfLsbLowerNibbleNotValidAsciiHex)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xbi]\r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfClosingBracketNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xb1 \r\n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_NoCallbackIfCarriageReturnNotFound)
{
   uint16_t i;
   uint8_t frame[] = "@123456789[0x29,0xb1] \n";

   mock().expectNCalls(0, "messageReceivedHandler");
   for (i=0; i<sizeof(frame); i++)
   {
      serialCommWrapperHandleByte(frame[i]);
   }
}

TEST(SerialCommWrapper, testRecv_RunIsANoop)
{
   serialCommWrapperRun();
}

