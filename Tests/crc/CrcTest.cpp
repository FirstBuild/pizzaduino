/*
 */

#include "CppUTest/TestHarness.h"
#include "string.h"
#include <stdint.h>
#include "crc.h"

TEST_GROUP(CRC)
{ 
   void setup()
   {
   }
   void teardown()
   {
   }
};

TEST(CRC, test)
{
   uint16_t crc = (uint16_t)crc_init();
   uint8_t msg[] = "123456789";

   crc = (uint16_t)crc_update(crc, &msg[0], strlen((const char*)msg));
   CHECK_EQUAL(0x29B1,crc);

}

