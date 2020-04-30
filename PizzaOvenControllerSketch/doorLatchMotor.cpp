/*
  doorLatchMotor.h

  Control the door lock motor

  Copyright (c) 2020 FirstBuild

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.


 */

#include "doorLatchMotor.h"
#include "config.h"
#include "pinDefinitions.h"
#include "relayDriver.h"

#ifdef CONFIGURATION_LOW_COST

static LatchMotorState latchMotorState;
DigitalInputDebounced latchMotorHomeInput(DOOR_LATCH_MOTOR_HOME_PIN, false, false);

void LatchMotorPosition_Init() {
    latchMotorState = LatchMotorIdle;
}

void LatchMotorPosition_Run() {
    switch(latchMotorState) {
        case LatchMotorIdle:
            changeRelayState(DOOR_LATCH_MOTOR_DRIVE_PIN, relayStateOff);
            break;

        case LatchMotorLocking:
            changeRelayState(DOOR_LATCH_MOTOR_DRIVE_PIN, relayStateOn);
            if (!latchMotorHomeInput.IsActive()) {
                latchMotorState = LatchMotorIdle;
            }
            break;

        case LatchMotorOpening:
            changeRelayState(DOOR_LATCH_MOTOR_DRIVE_PIN, relayStateOn);
            if (latchMotorHomeInput.IsActive()) {
                latchMotorState = LatchMotorIdle;
            }
            break;
    }
}

void LatchMotorPosition_Toggle() {
    // only change if we are idle
    if (LatchMotorIdle == latchMotorState) {
        if (latchMotorHomeInput.IsActive()) {
            latchMotorState = LatchMotorLocking;
        } else {
            latchMotorState = LatchMotorOpening;
        }
        LatchMotorPosition_Run();
    }
}

LatchMotorState LatchMotorPosition_GetState() {
    return latchMotorState;
}

#endif // CONFIGURATION_LOW_COST
