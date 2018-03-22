#include "tcLimitCheck.h"
#include <Arduino.h>

TcLimitCheck::TcLimitCheck(double limit, uint32_t timeout)
{
   m_limit = limit;
   m_timeout = timeout;
   m_startTime = 0;
   m_limitFailure = false;
   m_timerStarted =false;
}

void TcLimitCheck::checkLimit(double value)
{
   uint32_t currentTime;

   if (value > m_limit)
   {
      if (!m_timerStarted)
      {
         m_startTime = millis();
         m_timerStarted = true;
      }
      currentTime = millis();
      if (currentTime >= m_startTime)
      {
         if ((currentTime - m_startTime) >= m_timeout)
         {
            m_limitFailure = true;
         }
      }
      else
      {
         if((currentTime + (UINT32_MAX - m_startTime)) >= m_timeout)
         {
            m_limitFailure = true;
         }
      }
   }
   else
   {
      m_timerStarted = false;
   }
}

bool TcLimitCheck::limitExceeded(void)
{
   return m_limitFailure;
}

