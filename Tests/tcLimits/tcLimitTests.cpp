#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#include "tcLimitCheck.h"
#include "Arduino.h"

TEST_GROUP(tcLimitCheckTests)
{
   void setup()
   {
      setMillis(0);
   }

   void teardown()
   {
   }
};

TEST(tcLimitCheckTests, noFailIfLimitNotExceeded)
{
   uint32_t timeout = 0;
   bool check;

   TcLimitCheck tcLimit(1400, timeout);

   tcLimit.checkLimit(100);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

}

TEST(tcLimitCheckTests, failIfLimitExceededAndNoTimeout)
{
   uint32_t timeout = 0;
   bool check;

   TcLimitCheck tcLimit(1400, timeout);

   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(true, check);
}

TEST(tcLimitCheckTests, noFailIfLimitExceededAndTimeoutNotReached)
{
   uint32_t timeout = 10;
   bool check;

   TcLimitCheck tcLimit(1400, timeout);

   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);
}

TEST(tcLimitCheckTests, failIfLimitExceededAndTimeoutReached)
{
   uint32_t timeout = 10;
   bool check;

   // init
   TcLimitCheck tcLimit(1400, timeout);

   // first time limit exceeded
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

   // advance clock
   setMillis(20);

   // Check limit again
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(true, check);
}

TEST(tcLimitCheckTests, timerResetsIfTempGoesBelowLimit)
{
   uint32_t timeout = 10;
   bool check;

   // init
   TcLimitCheck tcLimit(1400, timeout);

   // first time limit exceeded
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

   // advance clock
   setMillis(5);

   // limit goes below, timer should reset
   tcLimit.checkLimit(1000);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

   // advance clock
   setMillis(20);

   // limit goes above but should not fail
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);
}

TEST(tcLimitCheckTests, noFailIfTimerRollerAndTimerNotExceeded)
{
   uint32_t timeout = 20;
   bool check;

   setMillis(UINT32_MAX - 5);

   // init
   TcLimitCheck tcLimit(1400, timeout);

   // first time limit exceeded
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

   // advance clock
   setMillis(5);

   // limit goes above but should not fail
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);
}

TEST(tcLimitCheckTests, failIfTimerRollerAndTimerExceeded)
{
   uint32_t timeout = 20;
   bool check;

   setMillis(UINT32_MAX - 5);

   // init
   TcLimitCheck tcLimit(1400, timeout);

   // first time limit exceeded
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(false, check);

   // advance clock
   setMillis(18);

   // limit goes above but should not fail
   tcLimit.checkLimit(1500);
   check = tcLimit.limitExceeded();
   CHECK_EQUAL(true, check);
}
