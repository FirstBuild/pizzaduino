#ifndef GLOBALS_H
#define GLOBALS_H
#include "config.h"

extern Heater upperFrontHeater;
extern Heater  lowerFrontHeater;
#ifdef CONFIGURATION_ORIGINAL
extern Heater upperRearHeater;
extern Heater lowerRearHeater;
#endif
extern volatile uint32_t triacTimeBase;
extern volatile uint32_t relayTimeBase;
extern bool doorHasDeployed;

extern bool ufTcTempLimitFailed;
extern bool lfTcTempLimitFailed;
#ifdef CONFIGURATION_ORIGINAL
extern bool urTcTempLimitFailed;
extern bool lrTcTempLimitFailed;
#define SOME_TC_HAS_FAILED (ufTcTempLimitFailed || urTcTempLimitFailed || lfTcTempLimitFailed || lrTcTempLimitFailed)
#endif
#ifdef CONFIGURATION_LOW_COST
#define SOME_TC_HAS_FAILED (ufTcTempLimitFailed  || lfTcTempLimitFailed)
#endif
#define ALL_TCS_OK (!SOME_TC_HAS_FAILED)

extern CoolingFanSpeed preheatFanSetting;
extern CoolingFanSpeed cookingFanSetting;

#ifdef CONFIGURATION_ORIGINAL
extern bool upperTempDiffExceeded;
extern bool lowerTempDiffExceeded;
#define TEMP_DIFF_FAIL (upperTempDiffExceeded || lowerTempDiffExceeded)
#define TEMP_DIFFS_OK (!TEMP_DIFF_FAIL)
#endif
#ifdef CONFIGURATION_LOW_COST
#define TEMP_DIFF_FAIL (1 == 0)
#define TEMP_DIFFS_OK (1 == 1)
#endif

#endif // GLOBALS_H

