/*
 * DSOsrdnoise.c
 *
 * Simplex noise for SL with rotating gradients and an analytic
 * derivative, over 2 and 3 dimensions, in a DSO shadeop.
 *
 * The SL functions defined by this DSO are:
 * srdnoise( float x, y, t )
 * srdnoise( float x, y, t; output float dx, dy )
 * srdnoise( point P; float t )
 * srdnoise( point P; float t; output float dx, dy, dz )
 *
 * The actual "srdnoise2" and "srdnoise3" functions in C are
 * implemented in a separate file, "srdnoise23.c", which needs
 * to be compiled and linked with this file.
 *
 * The parameters to the SL functions are:
 * (x,y) or P    Texture coordinates for the point being shaded
 * t             Rotation angle for the swirling motion (should be animated!)
 * dx, dy [, dz] Output: the true analytic derivative of the noise function
 *
 * Note that the return value is in the interval [-1, 1], not in [0, 1],
 * simply because it is most convenient and useful to have it that way.
 *
 * AUTHOR: Stefan Gustavson (stegu@itn.liu.se), Dec 09, 2005
 *
 */

/*
Copyright (C) 2005 by Stefan Gustavson. All rights reserved.
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

#include "shadeop.h"
#include "srdnoise23.h"

SHADEOP_TABLE ( srdnoise ) = {
    { "float f_srdnoise2 (float, float, float)", "", ""},
    { "float fD_srdnoise2 (float, float, float, float, float)", "", ""},
    { "float f_srdnoise3 (point, float)", "", ""},
    { "float fD_srdnoise3 (point, float, float, float, float)", "", ""},
    { "", "", "" }
};

SHADEOP ( f_srdnoise2 ) {
    float *result = (float *) argv[0];
    float *x = (float *) argv[1];
    float *y = (float *) argv[2];
    float *t = (float *) argv[3];

    *result = srdnoise2( *x, *y, *t, (float *) 0, (float *) 0 );

    return 0;
}

SHADEOP ( fD_srdnoise2 ) {
    float *result = (float *) argv[0];
    float *x = (float *) argv[1];
    float *y = (float *) argv[2];
    float *t = (float *) argv[3];
    float *dx = (float *) argv[4];
    float *dy = (float *) argv[5];

    *result = srdnoise2( *x, *y, *t, dx, dy );

    return 0;
}

SHADEOP ( f_srdnoise3 ) {
    float *result = (float *) argv[0];
    float *P = (float *) argv[1];
    float *t = (float *) argv[2];

    *result = srdnoise3( P[0], P[1], P[2], *t, (float *) 0, (float *) 0, (float *) 0 );

    return 0;
}

SHADEOP ( fD_srdnoise3 ) {
    float *result = (float *)argv[0];
    float *P = (float*) argv[1];
    float *t = (float*) argv[2];
    float *dx = (float*) argv[3];
    float *dy = (float*) argv[4];
    float *dz = (float*) argv[5];

    *result = srdnoise3( P[0], P[1], P[2], *t, dx, dy, dz );

    return 0;
}
