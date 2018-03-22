#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#include "ftoa.h"

char *ultoa(unsigned long val, char *buf, int radix)
{
   (void)radix;
   sprintf(buf, "%lu", val);
   return buf;
}

TEST_GROUP(ftoaTests)
{
   void setup()
   {
   }

   void teardown()
   {
   }
};

TEST(ftoaTests, positive)
{
   double val = 3.1415927;
   char buf[128];

   ftoa(val, (uint8_t *)&buf[0], 4);
   STRCMP_EQUAL("3.1416", buf);
}

TEST(ftoaTests, negative)
{
   double val = -3.1415927;
   char buf[128];

   ftoa(val, (uint8_t *)&buf[0], 4);
   STRCMP_EQUAL("-3.1416", buf);
}

