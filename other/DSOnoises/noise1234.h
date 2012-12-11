// noise1234
//
// Author: Stefan Gustavson, 2003-2005
// Contact: stegu@itn.liu.se
//

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
		\brief Declares the "noise1" through "noise4" functions for Perlin noise.
		\author Stefan Gustavson (stegu@itn.liu.se)
*/

/*
 * This is a backport to C of my improved noise class in C++.
 * It is highly reusable without source code modifications.
 *
 * Note:
 * Replacing the "float" type with "double" can actually make this run faster
 * on some platforms. A templatized version of Noise1234 could be useful.
 */

#ifndef __NOISE_H__
#define __NOISE_H__
/** 1D, 2D, 3D and 4D float Perlin noise, SL "noise()"
 */
//extern "C" {
extern float noise1( float x );
extern float noise2( float x, float y );
extern float noise3( float x, float y, float z );
extern float noise4( float x, float y, float z, float w );

/** 1D, 2D, 3D and 4D float Perlin periodic noise, SL "pnoise()"
 */
extern float pnoise1( float x, int px );
extern float pnoise2( float x, float y, int px, int py );
extern float pnoise3( float x, float y, float z, int px, int py, int pz );
extern float pnoise4( float x, float y, float z, float w,
                              int px, int py, int pz, int pw );
//}
#endif //
