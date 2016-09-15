#include "FiniteStateMachine.h"
#include "cookingStateMachine.h"
#include "utility.h"
#include "acInput.h"
#include "heater.h"
#include "config.h"
#include "coolingFan.h"
#include "pinDefinitions.h"
#include "PID_v1.h"

static bool pizzaOvenStartRequested = false;
static bool pizzaOvenStopRequested = false;

// Externed globals...sniff
extern Heater upperFrontHeater;
extern Heater upperRearHeater;
extern Heater  lowerFrontHeater;
extern Heater lowerRearHeater;
extern volatile uint32_t triacTimeBase;
extern volatile uint32_t relayTimeBase;

#ifdef USE_PID
// PID stuff
#define MAX_PID_OUTPUT 100
extern PidIo upperFrontPidIo;
extern PidIo upperRearPidIo;
extern PID upperFrontPID;
extern PID upperRearPID;
#endif

static uint32_t oldMillis;

static bool tcoHasFailed = false;
static bool coolingFanHasFailed = false;

bool coollingFanFailed(void) {return coolingFanHasFailed;}
bool tcoFailed(void) {return tcoHasFailed;}
 
//------------------------------------------
//state machine setup
//------------------------------------------
static void stateStandbyEnter();
static void stateStandbyUpdate();
static void stateStandbyExit();
static State stateStandby = State(stateStandbyEnter, stateStandbyUpdate, stateStandbyExit);

static void stateTurnOnTcoEnter();
static void stateTurnOnTcoUpdate();
static void stateTurnOnTcoExit();
static State stateTurnOnTco = State(stateTurnOnTcoEnter, stateTurnOnTcoUpdate, stateTurnOnTcoExit);

static void stateTurnOnSailSwitchEnter();
static void stateTurnOnSailSwitchUpdate();
static void stateTurnOnSailSwitchExit();
static State stateTurnOnSailSwitch = State(stateTurnOnSailSwitchEnter, stateTurnOnSailSwitchUpdate, stateTurnOnSailSwitchExit);

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
  else if (poStateMachine.isInState(stateTurnOnTco))
  {
    state = cookingWaitForTco;
  }
  else if (poStateMachine.isInState(stateTurnOnSailSwitch))
  {
    state = cookingWaitForSailSwitch;
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
}

static void stateStandbyUpdate()
{
  AllHeatersOffStateClear();
  if (!coolingFanHasFailed)
  {
    if (powerButtonIsOn() && pizzaOvenStartRequested && !tcoHasFailed)
    {
      poStateMachine.transitionTo(stateTurnOnTco);
    }
    else if ((upperFrontHeater.thermocouple > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
             (upperRearHeater.thermocouple  > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
             (lowerFrontHeater.thermocouple > COOL_DOWN_EXIT_HEATER_TEMP + 15) ||
             (lowerRearHeater.thermocouple  > COOL_DOWN_EXIT_HEATER_TEMP + 15))
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
}

/*
   State Machine stateTurnOnTco
*/
static void stateTurnOnTcoEnter(void)
{
  Serial.println(F("DEBUG Entering stateTurnOnTco."));
  CoolingFanControl(coolingFanHigh);
  oldMillis = millis();
}

static void stateTurnOnTcoUpdate(void)
{
  if (!powerButtonIsOn() || pizzaOvenStopRequested)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
  else if (tcoInputIsOn())
  {
    poStateMachine.transitionTo(stateTurnOnSailSwitch);
  }
  else
  {
    if ((millis() - oldMillis) > 5000)
    {
      // tco fault state...
      tcoHasFailed = true;
      Serial.println(F("DEBUG TCO input not detected after delay, failure."));
      poStateMachine.transitionTo(stateCoolDown);
    }
  }
}

static void stateTurnOnTcoExit(void)
{
  Serial.println(F("DEBUG Exiting stateTurnOnTco."));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;

}

/*
   State Machine stateTurnOnSailSwitch
*/
//State stateTurnOnSailSwitch = State(stateTurnOnSailSwitchEnter, stateTurnOnSailSwitchUpdate, stateTurnOnSailSwitchExit);
static void stateTurnOnSailSwitchEnter(void)
{
  Serial.println(F("DEBUG Entering stateTurnOnSailSwitch."));
  CoolingFanControl(coolingFanHigh);
  oldMillis = millis();
}

static void stateTurnOnSailSwitchUpdate(void)
{
  if (!powerButtonIsOn() || pizzaOvenStopRequested)
  {
    pizzaOvenStopRequested = false;
    poStateMachine.transitionTo(stateCoolDown);
  }
  else if (!tcoInputIsOn())
  {
      tcoHasFailed = true;
      Serial.println(F("DEBUG TCO input failure while waiting for sail switch."));
      poStateMachine.transitionTo(stateCoolDown);
  }
  else if (sailSwitchIsOn())
  {
    poStateMachine.transitionTo(stateHeatCycle);
  }
  else if(millis() - oldMillis > 5000)
  {
    // fan failure
    coolingFanHasFailed = true;
  }
}

static void stateTurnOnSailSwitchExit(void)
{
  Serial.println(F("DEBUG Exiting stateTurnOnSailSwitch."));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;

}

//------------------------------------------
//state machine stateHeatCycle
//------------------------------------------
//State stateHeatCycle = State(stateHeatCycleEnter, stateHeatCycleUpdate, stateHeatCycleExit);
static uint32_t currentTriacTimerCounter, oldTriacTimerCounter;
static uint32_t currentRelayTimerCounter, oldRelayTimerCounter;

static void stateHeatCycleEnter()
{
  Serial.println(F("DEBUG stateHeatCycleEnter"));

  // Start the timer1 counter over at the start of heat cycle volatile since used in interrupt
  triacTimeBase = 0;
  relayTimeBase = 0;
  // Fake the old time so that we exercise the relays the first time through
  oldTriacTimerCounter = 100000;
  oldRelayTimerCounter = 100000;

  changeRelayState(HEATER_UPPER_FRONT_DLB, relayStateOn);
  changeRelayState(HEATER_UPPER_REAR_DLB, relayStateOn);

  upperFrontPidIo.Output = 0.0;
  upperFrontPID.SetMode(AUTOMATIC);
  upperRearPidIo.Output = 0.0;
  upperRearPID.SetMode(AUTOMATIC);
}

static void stateHeatCycleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();

  // check for cooling fan failure
  if(!sailSwitchIsOn())
  {
    coolingFanHasFailed = true;
    poStateMachine.transitionTo(stateStandby);
  }
  // check for cooling fan failure
  if(!tcoInputIsOn())
  {
    tcoHasFailed = true;
    poStateMachine.transitionTo(stateCoolDown);
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

  if (!powerButtonIsOn() || !sailSwitchIsOn() || pizzaOvenStopRequested)
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

  // For now just check cool down on fan
  if ((upperFrontHeater.thermocouple <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (upperRearHeater.thermocouple  <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (lowerFrontHeater.thermocouple <= COOL_DOWN_EXIT_HEATER_TEMP) &&
      (lowerRearHeater.thermocouple  <= COOL_DOWN_EXIT_HEATER_TEMP))
  {
    CoolingFanControl(coolingFanOff);
    poStateMachine.transitionTo(stateStandby);
  }
  else if (powerButtonIsOn() && pizzaOvenStartRequested)
  {
    pizzaOvenStartRequested = false;
    poStateMachine.transitionTo(stateTurnOnSailSwitch);
  }
}

static void stateCoolDownExit()
{
  Serial.println(F("DEBUG CX"));
  pizzaOvenStartRequested = false;
  pizzaOvenStopRequested = false;

}

