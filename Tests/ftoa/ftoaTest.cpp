#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include <stdint.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string>
#include <cstring>
//#include <iostream>

using namespace std;

#include "ftoa.h"

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

