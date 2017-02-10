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

#ifndef FILTER_H_
#define FILTER_H_

// From: http://www.schwietering.com/jayduino/filtuino/
//Low pass bessel filter order=2 alpha1=0.0025, 0.125  hz corner freq
class  FilterBeLp2
{
  public:
    FilterBeLp2()
    {
      v[0] = 0.0;
      v[1] = 0.0;
    }
    initialize(float initVal)
    {
      v[0] = initVal/4;
      v[1] = initVal/4;
      v[2] = initVal/4;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (9.810514574132291022e-5 * x)
             + (-0.96598348806398948163 * v[0])
             + (1.96559106748102419004 * v[1]);
      return
        (v[0] + v[2])
        + 2 * v[1];
    }
};

#endif /* FILTER_H_ */
