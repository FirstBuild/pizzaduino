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

#ifndef CONFIG_H_
#define CONFIG_H_

#define USE_PID
//#define ENABLE_PID_TUNING

#define COOL_DOWN_EXIT_TEMP    ((double)425.0)
#define MAX_UPPER_TEMP (1300)
#define MAX_LOWER_TEMP (805)

//#define CONFIGURATION_ORIGINAL
#define CONFIGURATION_LOW_COST

#if defined(CONFIGURATION_ORIGINAL) && defined(CONFIGURATION_LOW_COST)
#error CONGFIGURAITON ERROR: Only define CONFIGURATION_ORIGINAL or CONFIGURATION_LOW_COST
#elif !defined(CONFIGURATION_ORIGINAL) && !defined(CONFIGURATION_LOW_COST)
#error CONGFIGURAITON ERROR: Either CONFIGURATION_ORIGINAL or CONFIGURATION_LOW_COST must be defined
#endif

#endif /* CONFIG_H_ */
