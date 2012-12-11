/* srdnoise23, Simplex noise with rotating gradients
 * and a true analytic derivative in 2D and 3D.
 *
 * This is version 2 of srdnoise23 written in early 2008.
 * A stupid bug was corrected. Do not use earlier versions.
 *
 * Author: Stefan Gustavson, 2003-2008
 *
 * Contact: stefan.gustavson@gmail.com
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
    \brief C header file for Perlin simplex noise with rotating
    gradients and analytic derivative over 2 and 3 dimensions.
    \author Stefan Gustavson (stegu@itn.liu.se)
*/

/*
 * This is an implementation of Perlin "simplex noise" over two dimensions
 * (x,y) and three dimensions (x,y,z). One extra parameter 't' rotates the
 * underlying gradients of the grid, which gives a swirling, flow-like
 * motion. The derivative is returned, to make it possible to do pseudo-
 * advection and implement "flow noise", as presented by Ken Perlin and
 * Fabrice Neyret at Siggraph 2001.
 *
 * When not animated and presented in one octave only, this noise
 * looks exactly the same as the plain version of simplex noise.
 * It's nothing magical by itself, although the extra animation
 * parameter 't' is useful. Fun stuff starts to happen when you
 * do fractal sums of several octaves, with different rotation speeds
 * and an advection of smaller scales by larger scales (or even the
 * other way around it you feel adventurous).
 *
 * The gradient rotations that can be performed by this noise function
 * and the true analytic derivatives are required to do flow noise.
 * You can't do it properly with regular Perlin noise.
 *
 */

/**
 * Simplex, rotating, derivative noise over 2 dimensions
 */
float srdnoise2( float x, float y, float t, float *dnoise_dx, float *dnoise_dy );

/**
 * Simplex, rotating, derivative noise over 3 dimensions
 */
float srdnoise3( float x, float y, float z, float t, float *dnoise_dx, float *dnoise_dy, float *dnoise_dz );

