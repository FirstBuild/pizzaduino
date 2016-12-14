#ifndef TC_LIMIT_CHECK_H
#define TC_LIMIT_CHECK_H

#include <stdint.h>

class TcLimitCheck {
   public:
      TcLimitCheck(double limit, uint32_t timeout);
      void checkLimit(double value);
      bool limitExceeded(void);
   private:
      double m_limit;
      uint32_t m_timeout;
      uint32_t m_startTime;
      bool m_timerStarted;
      bool m_limitFailure;
};

#endif // TC_LIMIT_CHECK_H
