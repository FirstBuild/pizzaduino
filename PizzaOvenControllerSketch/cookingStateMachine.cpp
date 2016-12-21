#include "FiniteStateMachine.h"
#include "cookingStateMachine.h"
#include "utility.h"
#include "acInput.h"
#include "heater.h"
#include "config.h"
#include "coolingFan.h"
#include "pinDefinitions.h"
#include "PID_v1.h"
#include "tcoAndFanCheck.h"
#include "globals.h"

static bool pizzaOvenStartRequested = false;
static bool pizzaOvenStopRequested = false;
static TcoAndFan tcoAndFan;

static uint32_t startTime = 0;
#define MAX_RUN_TIME (3 * 3600 * 1000)

#ifdef USE_PID
// PID stuff
#define MAX_PID_OUTPUT 100
extern PidIo upperFrontPidIo;
extern PidIo upperRearPidIo;
extern PID upperFrontPID;
extern PID upperRearPID;
#endif
 
//------------------------------------------
//state machine setup
//------------------------------------------
static void stateStandbyEnter();
static void stateStandbyUpdate();
static void stateStandbyExit();
static State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

static void stateWaitForDlbEnter();
static void stateWaitForDlbUpdate();
static void stateWaitForDlbExit();
static State stateWaitForDlb = State(stateWaitForDlbEnter, stateWaitForDlbUpdate, stateWaitForDlbExit);

static void stateHeatCycleEnter();
static void stateHeatCycleUpdate();
static void stateHeatCycleExit();
static State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);

static void stateCoolDownEnter();
static void stateCoolDownUpdate();
static void stateCoolDownExit();
static State stateCoolDown = State(stateCoolDownEnter, stateCoolDownUpdate, stateCoolDownExit);

static FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby

void requestPizzaOvenStart(void)
{
  pizzaOvenStartRequested = true;
  startTime = millis();
}

void requestPizzaOvenStop(void)
{
  pizzaOvenStopRequested = true;
}

void initCookingStateMachine(void)
{
  // nothing to do
}

void updateCookingStateMachine(void)
{
    poStateMachine.update();
}

cookingState getCookingState(void)
{
  cookingState state = cookingStandby;
  
  if (poStateMachine.isInState(stateStandby))
  {
    state = cookingStandby;
  }
  else if (poStateMachine.isInState(stateWaitForDlb))
  {
    state = cookingWaitForDlb;
  }
  else if (poStateMachine.isInState(stateHeatCycle))
  {
    state = cookingCooking;
  }
  else if (poStateMachine.isInState(stateCoolDown))
  {
    state = cookingCooldown;
  }
  
  return state;
}

//------------------------------------------
//state machine stateStandby
//------------------------------------------
//State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

static void stateStandbyEnter()
{
  Serial.println(F("DEBUG stateStandbyEnter"));
  AllHeatersOffStateClear();
  CoolingFanControl(coolingFanOff);
}

static void stateStandbyUpdate()
{
  AllHeatersOffStateClear();
  if (!tcoAndFan.coolingFanHasFailed() && !tcoAndFan.tcoHasFailed() && !doorHasDeployed)
  {
    if (powerButtonIsOn() && pizzaOvenStartRequested && ALL_TCS_OK && TEMP_DIFFS_OK)
    {
      poStateMachine.transitionTo(stateWaitForDlb);
    }
    else if ((upperFrontHeater.thermocouple > COOL_DOWN_EXIT_TEMP + 15) ||
             (upperRearHeater.thermocouple  > COOL_DOWN_EXIT_TEMP + 15) ||
             (lowerFrontHeater.thermocouple > COOL_DOWN_EXIT_TEMP + 15) ||
             (lowerRearHeater.thermocouple  > COOL_DOWN_EXIT_TEMP + 15))
    {
      poStateMachine.transitionTo(stateCoolDown);
    }
  }

  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
}

static void stateStandbyExit()
{
  Serial.println(F("DEBUG SX"));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
  tcoAndFan.reset();
}

/*
   State Machine stateWaitForDlb
*/
//State stateWaitForDlb = State(stateWaitForDlbEnter, stateWaitForDlbUpdate, stateWaitForDlbExit);
static void stateWaitForDlbEnter(void)
{
  Serial.println(F("DEBUG Entering stateWaitForDlb."));
  CoolingFanControl(coolingFanHigh);
}

static void stateWaitForDlbUpdate(void)
{
  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);    
  }
  else if (!powerButtonIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED || TEMP_DIFF_FAIL)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
  else if (sailSwitchIsOn() && tcoInputIsOn())
  {
    poStateMachine.transitionTo(stateHeatCycle);
  } 
}

static void stateWaitForDlbExit(void)
{
  Serial.println(F("DEBUG Exiting stateWaitForDlb."));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;

}


/*
 * UF = 4.2266777983124E-5 x^2 - 0.0083436994 x + 7.0316553821
 * UR = 5.49012226585957E-5 x^2 - 0.0238419207 x + 11.2096886045
*/

double getUFSeedValue(double t)
{
   return 0.000042266777983124 * t * t - 0.0083436994 * t + 7.0316553821;
}

double getURSeedValue(double t)
{
   return 0.0000549012226585957 * t * t - 0.0238419207 * t + 11.2096886045;

}
//------------------------------------------
//state machine stateHeatCycle
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
static uint32_t currentTriacTimerCounter, oldTriacTimerCounter;
static uint32_t currentRelayTimerCounter, oldRelayTimerCounter;

static void stateHeatCycleEnter()
{
  double seed;
  
  Serial.println(F("DEBUG stateHeatCycleEnter"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  relayTimeBase = 0;
  // Fake the old time so that we exercise the relays the first time through
  oldTriacTimerCounter = 100000;
  oldRelayTimerCounter = 100000;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);
  
  upperFrontPidIo.Output = getUFSeedValue(upperFrontHeater.thermocouple);
  upperFrontPID.SetMode(AUTOMATIC);
  
  upperRearPidIo.Output = getURSeedValue(upperRearHeater.thermocouple);
  upperRearPID.SetMode(AUTOMATIC);

  startTime = millis();
}

static void stateHeatCycleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  uint32_t elapsedTime = (newTime - startTime);

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
  }

  // check max time limit
  if (elapsedTime >= MAX_RUN_TIME)
  {
   requestPizzaOvenStop();
  }

  // only update the relays periodically
  if (oldTime < newTime)
  {
    if ((newTime - oldTime) < 7)
    {
      return;
    }
  }
  oldTime = newTime;

  // Save working value triacTimeBase since can be updated by interrupt
  currentTriacTimerCounter = triacTimeBase;
  currentRelayTimerCounter = relayTimeBase;

  // Handle triac control
  if (currentTriacTimerCounter != oldTriacTimerCounter)
  {
    //    CoolingFanControl(true);
#ifdef USE_PID
    UpdateHeatControlWithPID(&upperFrontHeater, currentTriacTimerCounter);
    UpdateHeatControlWithPID(&upperRearHeater, currentTriacTimerCounter);
#else
    UpdateHeatControl(&upperFrontHeater, currentTriacTimerCounter);
    UpdateHeatControl(&upperRearHeater, currentTriacTimerCounter);
#endif

    UpdateHeaterHardware();

    oldTriacTimerCounter = currentTriacTimerCounter;
  }

  // Handle relay control
  if (currentRelayTimerCounter != oldRelayTimerCounter)
  {
    //    CoolingFanControl(true);
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED || TEMP_DIFF_FAIL)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
}

static void stateHeatCycleExit()
{
  Serial.println("DEBUG HX");
  AllHeatersOffStateClear();
  upperFrontPID.SetMode(MANUAL);
  upperFrontPidIo.Output = 0.0;
  upperRearPID.SetMode(MANUAL);
  upperRearPidIo.Output = 0.0;
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
}

//------------------------------------------
//state machine stateCoolDown
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter,
//  stateCoolDownUpdate, stateCoolDownExit);

static void stateCoolDownEnter()
{
  Serial.println(F("DEBUG stateCoolDown"));
  CoolingFanControl(coolingFanLow);
  AllHeatersOffStateClear();
}

static void stateCoolDownUpdate()
{
  AllHeatersOffStateClear();

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
  }

  if ((upperFrontHeater.thermocouple <= COOL_DOWN_EXIT_TEMP) &&
      (upperRearHeater.thermocouple  <= COOL_DOWN_EXIT_TEMP) &&
      (lowerFrontHeater.thermocouple <= COOL_DOWN_EXIT_TEMP) &&
      (lowerRearHeater.thermocouple  <= COOL_DOWN_EXIT_TEMP))
  {
    CoolingFanControl(coolingFanOff);
    poStateMachine.transitionTo(stateStandby);
  }
  else if (powerButtonIsOn() && pizzaOvenStartRequested && ALL_TCS_OK && TEMP_DIFFS_OK)
  {
    pizzaOvenStartRequested = false;
    poStateMachine.transitionTo(stateWaitForDlb);
  }
}

static void stateCoolDownExit()
{
  Serial.println(F("DEBUG CX"));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;

}

