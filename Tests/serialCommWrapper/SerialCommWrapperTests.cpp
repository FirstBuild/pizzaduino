/*
 */

#include "CppUTest/TestHarness.h"
#include "string.h"
#include <stdint.h>
#include "serialCommWrapper.h"

static uint8_t sendBuffer[2048];
static uint16_t sendBufferIndex;

static void messageReceivedHandler(uint8_t *pData, uint8_t length)
{
   (void)pData;
   (void)length;
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
};

TEST(SerialCommWrapper, testSendHappyPath)
{
   uint8_t msg[] = "123456789";
   uint8_t asSent[] = {255, (uint8_t)strlen((const char *)&msg[0]), 
      '1', '2', '3', '4', '5', '6', '7', '8', '9', 0x29, 0xB1, 0};

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

TEST(SerialCommWrapper, testSendEscapeEmittedIfNeeded)
{
   uint8_t msg[] = {0xff, 0x61, 0xfe, 0};
   uint8_t asSent[] = {255, (uint8_t)strlen((const char *)&msg[0]), 
      0xfe, 0xff, 0x61, 0xfe, 0xfe, 0x35, 0x35, 0};

   serialCommWrapperSendMessage(&msg[0], (uint8_t)strlen((const char *)&msg[0]));
   CHECK_EQUAL(strlen((const char *)&asSent[0]), sendBufferIndex);
   MEMCMP_EQUAL(asSent, sendBuffer, sendBufferIndex);
}

