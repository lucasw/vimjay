/* sdnoise1234, Simplex noise with true analytic
 * derivative in 1D to 4D.
 *
 * Author: Stefan Gustavson, 2003-2005
 * Contact: stegu@itn.liu.se
 */

/*
This code was GPL licensed until February 2011.
As the original author of this code, I hereby
release it irrevocably into the public domain.
Please feel free to use it for whatever you want.
Credit is appreciated where appropriate, and I also
appreciate being told where this code finds any use,
but you may do as you like. Alternatively, if you want
to have a familiar OSI-approved license, you may use
This code under the terms of the MIT license:

Copyright (C) 2003-2005 by Stefan Gustavson. All rights reserved.
This code is licensed to you under the terms of the MIT license:

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

/** \file
    \brief C header file for Perlin simplex noise with analytic
    derivative over 1, 2, 3 and 4 dimensions.
    \author Stefan Gustavson (stefan.gustavson@gmail.com)
*/

/*
 * This is an implementation of Perlin "simplex noise" over one
 * dimension (x), two dimensions (x,y), three dimensions (x,y,z)
 * and four dimensions (x,y,z,w). The analytic derivative is
 * returned, to make it possible to do lots of fun stuff like
 * flow animations, curl noise, analytic antialiasing and such.
 *
 * Visually, this noise is exactly the same as the plain version of
 * simplex noise provided in the file "snoise1234.c". It just returns
 * all partial derivatives in addition to the scalar noise value.
 *
 */

#include <math.h>

/** 1D simplex noise with derivative.
 * If the last argument is not null, the analytic derivative
 * is also calculated.
 */
float sdnoise1(float x, float *dnoise_dx);

/** 2D simplex noise with derivatives.
 * If the last two arguments are not null, the analytic derivative
 * (the 2D gradient of the scalar noise field) is also calculated.
 */
float sdnoise2(float x, float y, float *dnoise_dx, float *dnoise_dy);

/** 3D simplex noise with derivatives.
 * If the last tthree arguments are not null, the analytic derivative
 * (the 3D gradient of the scalar noise field) is also calculated.
 */
float sdnoise3(float x, float y, float z, float *dnoise_dx, float *dnoise_dy,
               float *dnoise_dz);

/** 4D simplex noise with derivatives.
 * If the last four arguments are not null, the analytic derivative
 * (the 4D gradient of the scalar noise field) is also calculated.
 */
float sdnoise4(float x, float y, float z, float w, float *dnoise_dx,
               float *dnoise_dy, float *dnoise_dz, float *dnoise_dw);
