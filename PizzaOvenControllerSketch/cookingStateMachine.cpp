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
static uint32_t currentTriacTimerCounter, oldTriacTimerCounter;
static uint32_t currentRelayTimerCounter, oldRelayTimerCounter;
static bool domeOn = true;
static bool setpointIncreaseOccurred = false;
static bool stoneIsPreheated = false;
static bool upperFrontPreheated = false;
static bool upperRearPreheated = false;

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

static void statePreheatStoneOnlyEnter();
static void statePreheatStoneOnlyUpdate();
static void statePreheatStoneOnlyExit();
static State statePreheatStoneOnly = State(statePreheatStoneOnlyEnter, statePreheatStoneOnlyUpdate, statePreheatStoneOnlyExit);

static void statePreheatEnter();
static void statePreheatUpdate();
static void statePreheatExit();
static State statePreheat = State(statePreheatEnter, statePreheatUpdate, statePreheatExit);

static void stateHeatCycleEnter();
static void stateHeatCycleUpdate();
static void stateHeatCycleExit();
static State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);

static void stateIdleEnter();
static void stateIdleUpdate();
static void stateIdleExit();
static State stateIdle = State(stateIdleEnter, stateIdleUpdate, stateIdleExit);

static void stateCoolDownEnter();
static void stateCoolDownUpdate();
static void stateCoolDownExit();
static State stateCoolDown = State(stateCoolDownEnter, stateCoolDownUpdate, stateCoolDownExit);

static FSM poStateMachine = FSM(stateStandby);     //initialize state machine, start in state: stateStandby

void requestPizzaOvenStart(void)
{
  pizzaOvenStartRequested = true;
}

void requestPizzaOvenStop(void)
{
  pizzaOvenStopRequested = true;
}

void setDomeState(uint8_t state)
{
  domeOn = (state == 1) ? true : false;
}

uint8_t getDomeState(void)
{
  return domeOn ? 1 : 0;
}

void theSetpointWasIncreased(thisSetpointIncreased which)
{
  setpointIncreaseOccurred = true;

  if (which == stoneSetpointIncreased)
  {
    stoneIsPreheated = false;
  }
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
  else if (poStateMachine.isInState(statePreheatStoneOnly))
  {
    state = cookingPreheatStoneOnly;
  }
  else if (poStateMachine.isInState(statePreheat))
  {
    state = cookingPreheat;
  }
  else if (poStateMachine.isInState(stateHeatCycle))
  {
    state = cookingCooking;
  }
  else if (poStateMachine.isInState(stateCoolDown))
  {
    state = cookingCooldown;
  }
  else if (poStateMachine.isInState(stateIdle))
  {
    state = cookingIdle;
  }
  
  return state;
}

//------------------------------------------
//state machine stateStandby
//------------------------------------------
//State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

static void stateStandbyEnter()
{
  Serial.println(F("DEBUG entering standby"));
  AllHeatersOffStateClear();
  CoolingFanControl(coolingFanOff);
  upperFrontPID.SetMode(MANUAL);
  upperRearPID.SetMode(MANUAL);
}

static void stateStandbyUpdate()
{
  AllHeatersOffStateClear();
  if (!tcoAndFan.coolingFanHasFailed() && !tcoAndFan.tcoHasFailed() && !doorHasDeployed)
  {
    if (powerButtonIsOn() && pizzaOvenStartRequested && ALL_TCS_OK)
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
  stoneIsPreheated = false;
}

static void stateStandbyExit()
{
  Serial.println(F("DEBUG exiting standby"));
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
  Serial.println(F("DEBUG entering wait for DLB."));
  CoolingFanControl(coolingFanHigh);
}

static void stateWaitForDlbUpdate(void)
{
  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);    
  }
  else if (!powerButtonIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
  else if (sailSwitchIsOn() && tcoInputIsOn())
  {
    if (domeOn == true)
    {
      poStateMachine.transitionTo(statePreheat);    
    }
    else
    {
      poStateMachine.transitionTo(statePreheatStoneOnly);    
    }
  } 
}

static void stateWaitForDlbExit(void)
{
  Serial.println(F("DEBUG Exiting wait for DLB."));
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
//state machine statePreheatStoneOnly
//------------------------------------------
// State statePreheatStage1 = State(statePreheatStoneOnlyEnter, statePreheatStoneOnlyUpdate, statePreheatStoneOnlyExit);

static void statePreheatStoneOnlyEnter()
{
  Serial.println(F("DEBUG entering preheat stone only"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  relayTimeBase = 0;
  setpointIncreaseOccurred = false;


  upperFrontPID.SetMode(MANUAL);
  upperRearPID.SetMode(MANUAL);
}

static void statePreheatStoneOnlyUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
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
  currentRelayTimerCounter = relayTimeBase;

  // force triacs off
  upperFrontHeater.relayState = relayStateOff;
  upperRearHeater.relayState = relayStateOff;
  UpdateHeaterHardware();

  // Handle relay control
  if (currentRelayTimerCounter != oldRelayTimerCounter)
  {
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
    return;
  }
  
  if (domeOn)
  {
    poStateMachine.transitionTo(statePreheat);
  }
  else
  {
    if (((lowerFrontHeater.thermocouple >= lowerFrontHeater.parameter.tempSetPointLowOn) &&
        (lowerRearHeater.thermocouple >= lowerRearHeater.parameter.tempSetPointLowOn)) || stoneIsPreheated)
    {
      stoneIsPreheated = true;
      poStateMachine.transitionTo(stateIdle);
    }
  }
}

static void statePreheatStoneOnlyExit()
{
  Serial.println(F("DEBUG Exiting preheat stone only"));
}

//------------------------------------------
//state machine statePreheat
//------------------------------------------
// State statePreheat = State(statePreheatEnter, statePreheatUpdate, statePreheatExit);

static void statePreheatEnter()
{
  Serial.println(F("DEBUG entering preheat"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  // Fake the old time so that we exercise the relays the first time through
  oldTriacTimerCounter = 100000;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);
  
  upperFrontPidIo.Output = getUFSeedValue(upperFrontHeater.thermocouple);
  upperFrontPID.SetMode(AUTOMATIC);
  
  upperRearPidIo.Output = getURSeedValue(upperRearHeater.thermocouple);
  upperRearPID.SetMode(AUTOMATIC);

  upperFrontPreheated = false;
  upperRearPreheated = false;
}

static void statePreheatUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
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
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
    return;
  }

  if (!domeOn)
  {
    poStateMachine.transitionTo(statePreheatStoneOnly);
    return;
  }

  #define PREHEAT_DONE_OFFSET 5.0
  if ((upperFrontHeater.thermocouple + PREHEAT_DONE_OFFSET) >= (upperFrontHeater.parameter.tempSetPointHighOff + upperFrontHeater.parameter.tempSetPointLowOn)/2) 
  {
    upperFrontPreheated = true;
  }

  if ((upperRearHeater.thermocouple + PREHEAT_DONE_OFFSET) >= (upperRearHeater.parameter.tempSetPointHighOff + upperRearHeater.parameter.tempSetPointLowOn)/2)
  {
    upperRearPreheated = true;
  }

  if ((((lowerFrontHeater.thermocouple >= (lowerFrontHeater.parameter.tempSetPointHighOff + lowerFrontHeater.parameter.tempSetPointLowOn)/2) &&
      (  lowerRearHeater.thermocouple >= (lowerRearHeater.parameter.tempSetPointHighOff + lowerRearHeater.parameter.tempSetPointLowOn)/2)) || stoneIsPreheated) &&
      ( upperFrontPreheated ) &&
      ( upperRearPreheated ))
  {
    stoneIsPreheated = true;
    poStateMachine.transitionTo(stateHeatCycle);
  }
}

static void statePreheatExit()
{
  Serial.println(F("DEBUG Exiting preheat"));
}

//------------------------------------------
//state machine stateHeatCycle
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);

static void stateHeatCycleEnter()
{
  Serial.println(F("DEBUG entering heat cycle"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  relayTimeBase = 0;
  // Fake the old time so that we exercise the relays the first time through
  oldTriacTimerCounter = 100000;
  oldRelayTimerCounter = 100000;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);

  if (upperFrontPID.GetMode() == MANUAL) {
    upperFrontPidIo.Output = getUFSeedValue(upperFrontHeater.thermocouple);
    upperFrontPID.SetMode(AUTOMATIC);
  }

  if (upperRearPID.GetMode() == MANUAL) {
    upperRearPidIo.Output = getURSeedValue(upperRearHeater.thermocouple);
    upperRearPID.SetMode(AUTOMATIC);
  }
}

static void stateHeatCycleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
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
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
    return;
  }

  if (!domeOn)
  {
    poStateMachine.transitionTo(stateIdle);
    return;
  }

  if (setpointIncreaseOccurred)
  {
    setpointIncreaseOccurred = false;
    poStateMachine.transitionTo(statePreheat);
    return;
  }
}

static void stateHeatCycleExit()
{
  Serial.println(F("DEBUG exiting heat cycle"));
}

//------------------------------------------
//state machine stateIdle
//------------------------------------------
// State stateIdle = State(stateIdleEnter, stateIdleUpdate, stateIdleExit);

static void stateIdleEnter()
{
  Serial.println(F("DEBUG entering idle"));
  upperFrontPID.SetMode(MANUAL);
  upperRearPID.SetMode(MANUAL);
}

static void stateIdleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  if (!tcoAndFan.areOk())
  {
    Serial.println(F("Fans are not ok, transitioning to standby"));
    poStateMachine.transitionTo(stateStandby);
    return;
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
  currentRelayTimerCounter = relayTimeBase;

  // force triacs off
  upperFrontHeater.relayState = relayStateOff;
  upperRearHeater.relayState = relayStateOff;
  UpdateHeaterHardware();

  // Handle relay control
  if (currentRelayTimerCounter != oldRelayTimerCounter)
  {
    UpdateHeatControl(&lowerFrontHeater, currentRelayTimerCounter);
    UpdateHeatControl(&lowerRearHeater, currentRelayTimerCounter);

    UpdateHeaterHardware();

    oldRelayTimerCounter = currentRelayTimerCounter;
  }

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested || SOME_TC_HAS_FAILED)
  {
    Serial.println(F("Transitioning to cooldown from idle..."));
    poStateMachine.transitionTo(stateCoolDown);
    return;
  }

  // The error conditions are checked above, just check the start requested bit.
  if (domeOn)
  {
    poStateMachine.transitionTo(statePreheat);
    return;
  }

  if (setpointIncreaseOccurred)
  {
    setpointIncreaseOccurred = false;
    if (domeOn == true)
    {
      poStateMachine.transitionTo(statePreheat);
    }
    else
    {
      poStateMachine.transitionTo(statePreheatStoneOnly);
    }
    return;
  }
}

static void stateIdleExit()
{
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
  Serial.println(F("DEBUG Exiting idle"));
}

//------------------------------------------
//state machine stateCoolDown
//------------------------------------------
//State stateCoolDown = State(stateCoolDownEnter,
//  stateCoolDownUpdate, stateCoolDownExit);

static void stateCoolDownEnter()
{
  Serial.println(F("DEBUG entering cooldown"));
  CoolingFanControl(coolingFanLow);
  AllHeatersOffStateClear();
  upperFrontPID.SetMode(MANUAL);
  upperFrontPidIo.Output = 0.0;
  upperRearPID.SetMode(MANUAL);
  upperRearPidIo.Output = 0.0;
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
  stoneIsPreheated = false;
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
    return;
  }
  
  if (powerButtonIsOn() && pizzaOvenStartRequested && ALL_TCS_OK)
  {
    pizzaOvenStartRequested = false;
    poStateMachine.transitionTo(stateWaitForDlb);
    return;
  }
}

static void stateCoolDownExit()
{
  Serial.println(F("DEBUG exiting cooldown"));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;
}
