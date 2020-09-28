#ifndef GLOBALS_H
#define GLOBALS_H

extern Heater upperFrontHeater;
extern Heater upperRearHeater;
extern Heater  lowerFrontHeater;
extern Heater lowerRearHeater;
extern volatile uint32_t triacTimeBase;
extern volatile uint32_t relayTimeBase;
extern bool doorHasDeployed;

extern bool ufTcTempLimitFailed;
extern bool urTcTempLimitFailed;
extern bool lfTcTempLimitFailed;
extern bool lrTcTempLimitFailed;
#define SOME_TC_HAS_FAILED (ufTcTempLimitFailed || urTcTempLimitFailed || lfTcTempLimitFailed || lrTcTempLimitFailed)
#define ALL_TCS_OK (!SOME_TC_HAS_FAILED)

#endif // GLOBALS_H

