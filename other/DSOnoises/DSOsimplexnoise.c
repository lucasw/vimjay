/*
 * simplexnoise() -- a new and better noise function for SL
 *
 * This is a useful, real world example of a DSO shadeop.
 * The actual "snoise" functions in C are implemented in
 * a separate file, "simplexnoise1234.c", which needs to be
 * compiled and linked with this file.
 *
 * AUTHOR: Stefan Gustavson (stegu@itn.liu.se), Nov 26, 2005
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
#include "simplexnoise1234.h"

// Carefully chosen but somewhat arbitrary x, y, z, t offsets
// for repeated evaluation for vector return types.
#define	O1x	19.34
#define	O1y	7.66
#define	O1z	3.23
#define O1t 2.77

#define	O2x	5.47
#define	O2y	17.85
#define	O2z	11.04
#define	O2t	13.19

// These are actually never used, because SL has no 4D return types
#define	O3x	23.54
#define	O3y	29.11
#define	O3z	31.91
#define	O3t	37.48


SHADEOP_TABLE (simplexnoise) = {
    { "float f_snoiseF (float)", "", ""},
    { "float f_snoiseFF (float, float)", "", ""},
    { "float f_snoiseP (point)", "", ""},
    { "float f_snoisePF (point, float)", "", ""},
    { "vector v_snoiseF (float)", "", ""},
    { "vector v_snoiseFF (float, float)", "", ""},
    { "vector v_snoiseP (point)", "", ""},
    { "vector v_snoisePF (point, float)", "", ""},
    { "point v_snoiseF (float)", "", ""},
    { "point v_snoiseFF (float, float)", "", ""},
    { "point v_snoiseP (point)", "", ""},
    { "point v_snoisePF (point, float)", "", ""},
    { "color v_snoiseF (float)", "", ""},
    { "color v_snoiseFF (float, float)", "", ""},
    { "color v_snoiseP (point)", "", ""},
    { "color v_snoisePF (point, float)", "", ""},
    { "", "", "" }
};

SHADEOP (f_snoiseF) {
    float *result = (float *)argv[0];
    float *x = (float*) argv[1];

    *result = (1.0f+snoise1(*x))*0.5f;

    return 0;
}

SHADEOP (f_snoiseFF) {
    float *result = (float *)argv[0];
    float *x = (float*) argv[1];
    float *y = (float*) argv[2];

    *result = (1.0f+snoise2(*x, *y))*0.5f;

    return 0;
}

SHADEOP (f_snoiseP) {
    float *result = (float *)argv[0];
    float *P = (float*) argv[1];

    *result = (1.0f+snoise3(P[0], P[1], P[2]))*0.5f;

    return 0;
}

SHADEOP (f_snoisePF) {
    float *result = (float *)argv[0];
    float *P = (float*) argv[1];
    float *t = (float*) argv[2];

    *result = (1.0f+snoise4(P[0], P[1], P[2], *t))*0.5f;

    return 0;
}

// "vector" and "point" types are treated exactly the same,
// and so is "color", which is strictly not correct. A "color"
// might actually be something else than an RGB triplet, even
// though it almost never is in current RI implementations.

SHADEOP (v_snoiseF) {
    float *result = (float *)argv[0];
    float *x = (float*) argv[1];

    result[0] = (1.0f+snoise1(*x))*0.5f;
    result[1] = (1.0f+snoise1(*x + O1x))*0.5f;
    result[2] = (1.0f+snoise1(*x + O2x))*0.5f;

    return 0;
}

SHADEOP (v_snoiseFF) {
    float *result = (float *)argv[0];
    float *x = (float*) argv[1];
    float *y = (float*) argv[2];

    result[0] = (1.0f+snoise2(*x, *y))*0.5f;
    result[1] = (1.0f+snoise2(*x+O1x, *y+O1y))*0.5f;
    result[2] = (1.0f+snoise2(*x+O2x, *y+O2y))*0.5f;

    return 0;
}

SHADEOP (v_snoiseP) {
    float *result = (float *)argv[0];
    float *P = (float*) argv[1];

    result[0] = (1.0f+snoise3(P[0], P[1], P[2]))*0.5f;
    result[1] = (1.0f+snoise3(P[0]+O1x, P[1]+O1y, P[2]+O1z))*0.5f;
    result[2] = (1.0f+snoise3(P[0]+O2x, P[1]+O2y, P[2]+O2z))*0.5f;

    return 0;
}

SHADEOP (v_snoisePF) {
    float *result = (float *)argv[0];
    float *P = (float*) argv[1];
    float *t = (float*) argv[2];

    result[0] = (1.0f+snoise4(P[0], P[1], P[2], *t))*0.5f;
    result[1] = (1.0f+snoise4(P[0]+O1x, P[1]+O1y, P[2]+O1z, *t+O1t))*0.5f;
    result[2] = (1.0f+snoise4(P[0]+O2x, P[1]+O2y, P[2]+O2z, *t+O2t))*0.5f;

    return 0;
}
