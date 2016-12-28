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

// JAMES - startTime is the number of millis when the start button is pressed
// JAMES - on the front panel.
static uint32_t startTime = 0;
// JAMES - MAX_RUN_TIME is the total number of millis that the unit should run
// JAMES - without start being pressed.
// JAMES - When I test the code, I used a much smaller number here, something
// JAMES - perhaps equivalent to 3 minutes of cook time.  For the release,
// JAMES - I just changed the numbers here to make is 3 hours.  Check my math
// JAMES - here.  3 hours X 3600 seconds per hour X 1000 milliseconds per
// JAMES - second.
// JAMES - Does this need a type cast here?  Should this actually be
// JAMES - #define MAX_RUN_TIME ((uint32_t)((uint32_t)3 * (uint32_t)3600 * (uint32_t)1000))
#define MAX_RUN_TIME (3 * 3600 * 1000)

#ifdef USE_PID
// PID stuff
#define MAX_PID_OUTPUT 100
extern PidIo upperFrontPidIo;
extern PidIo upperRearPidIo;
extern PID upperFrontPID;
extern PID upperRearPID;
#endif
 
// JAMES - These are the state definitions for the cooking state machine.
// JAMES - Each state includes 3 functions, Enter, Update, and Exit.
// JAMES - The Enter function is called once when a state is entered or
// JAMES - become active. The Enter function does any prep required for this
// JAMES - state.
// JAMES - The Update function is called whenever the .update function is 
// JAMES - called.  This function is called from the updateCookingStateMachine
// JAMES - function.  The update function is the 'run' function for a state.
// JAMES - The code here should do whatever is needed to manage this state.
// JAMES - The Exit function is called prior to exiting a state.  It contains
// JAMES - any cleanup necessary for this state.
//
// JAMES - A transition to another state is accompilished by calling .transitionTo.
// JAMES - If the state you are in is A, and you are transitioning to state B, the
// JAMES - the state machine will call the functions in this order:
// JAMES - A.Exit(), B.Enter()
// JAMES - Note that the names given above are for reference.  We have used these
// JAMES - names within our own function names, but these names are not
// JAMES - mandatory.  We use them merely for convention.
//
// JAMES - Each state defintion below follows a similar pattern.  The three
// JAMES - required functions are called
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

// JAMES - This function is called to start cooking.  This is where millis is captured
// JAMES - into the startTime variable.
// JAMES - The pizzaOvenStart variable is a little dumb in the sense that this is really
// JAMES - how the oven gets started.  This is left over from the original starting metho
// JAMES - and really should be refactored.
void requestPizzaOvenStart(void)
{
   // JAMES - It might make sense to put a guard around this.  You should only 
   // JAMES - be able to start the oven is you are not in the HeatCycle state.
  pizzaOvenStartRequested = true;
  startTime = millis();
}

// JAMES - This function is called to stop cooking.
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
    // JAMES - Tim wanted to put a debounce around this for impulse noise performance.
    // JAMES - If you do this before I return, don't do it without a lot of 
    // JAMES - thought.
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

// JAMES - millis is recaptured here.  This is probably not needed, but this
// JAMES - state is entered when after the start button is pressed and the
// JAMES - the DLB closes.  This state should be entered seconds after
// JAMES - the start button is pressed. See Tim for details regarding
// JAMES - the DLB and sail switch.
  startTime = millis();
}

static void stateHeatCycleUpdate()
{
  static uint32_t oldTime = 0;
  uint32_t newTime = millis();
  // JAMES - Calculate the elapsed run time.
  // JAMES - Note:  I tested this for rollover with a uint8_t and it worked.
  // JAMES - There may be a corner case here that I did not consider, but 
  // JAMES - since millis is supposed to rollover after about 50 days, I 
  // JAMES - would not expect rollover to become an issued until then.
  uint32_t elapsedTime = (newTime - startTime);

  if (!tcoAndFan.areOk())
  {
    poStateMachine.transitionTo(stateStandby);
    return;
  }

  // JAMES - This is where the 3-hour check happens.  Pretty simple, right?
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

