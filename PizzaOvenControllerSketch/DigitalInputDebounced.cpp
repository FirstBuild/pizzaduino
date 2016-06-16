/*
  Copyright (c) 2016 FirstBuild

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

#include "DigitalInputDebounced.h"
#include <Arduino.h>

#define DEBOUNCE_COUNT 5

DigitalInputDebounced::DigitalInputDebounced(int pin, bool initialState, bool activeLow)
{
  m_activeLow = activeLow;
  m_count = 0;
  m_inputState = initialState;
  m_pin = pin;
  pinMode(pin, INPUT);
}

void DigitalInputDebounced::UpdateInput(void)
{
  if (digitalRead(m_pin) == 1)
  {
    if (m_count < DEBOUNCE_COUNT)
    {
      m_count++;
    }
    else
    {
      m_inputState = m_activeLow ? false : true;
    }
  } 
  else 
  {
    if (m_count > 0)
    {
      m_count--;
    }
    else
    {
      m_inputState = m_activeLow ? true : false;
    }
  }
}

bool DigitalInputDebounced::IsActive(void)
{
  return m_inputState;  
}

