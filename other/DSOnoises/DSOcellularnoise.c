/*
 * cellularnoise() -- a new and fun noise function for SL
 *
 * The actual "Worley" function in C is implemented in
 * a separate file, "cellular.c", which needs to be
 * compiled and linked with this file.
 * Stephen Worley wrote that function, I just wired it
 * into a DSO shadeop by writing this file. Please refer
 * to the file "cellular.c" for his licensing details.
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
#include "cellular.h"  // cellular.c is (C) Stephen Worley. See file for details.

SHADEOP_TABLE ( cellularnoise ) = {
    { "float f_worleyFF (float, float)", "", "" },
    { "float f_worleyP (point)", "", "" },
    { "void worleyFF_F (float, float, float)", "", "" },
    { "void worleyP_F (point, float)", "", "" },
    { "void worleyFF_FF (float, float, float, float)", "", "" },
    { "void worleyP_FF (point, float, float)", "", ""  },
    { "void worleyFF_FFF (float, float, float, float, float)", "", "" },
    { "void worleyP_FFF (point, float, float, float)", "", "" },
    { "", "", "" }
};

SHADEOP ( f_worleyFF ) {
    float *result = ( float* )argv[0];
    float *x = ( float* ) argv[1];
    float *y = ( float* ) argv[2];
    double at[3];
    at[0] = *x;
    at[1] = *y;
    at[2] = 0.0;
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *result = ( float ) F[0];
    return 0;
}

SHADEOP ( f_worleyP ) {
    float *result = ( float* ) argv[0];
    float *P = ( float* ) argv[1];
    double at[3];
    at[0] = ( double ) P[0];
    at[1] = ( double ) P[1];
    at[2] = ( double ) P[2];
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *result = ( float ) F[0];
    return 0;
}

SHADEOP ( worleyFF_F ) {
    float *x = ( float* ) argv[1];
    float *y = ( float* ) argv[2];
    float *f1 = ( float* ) argv[3];
    double at[3];
    at[0] = *x;
    at[1] = *y;
    at[2] = 0.0;
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    return 0;
}

SHADEOP ( worleyP_F ) {
    float *P = ( float* ) argv[1];
    float *f1 = ( float* )argv[2];
    double at[3];
    at[0] = (double)P[0];
    at[1] = (double)P[1];
    at[2] = (double)P[2];
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    return 0;
}

SHADEOP ( worleyFF_FF ) {
    float *x = ( float* ) argv[1];
    float *y = ( float* ) argv[2];
    float *f1 = ( float* ) argv[3];
    float *f2 = ( float* ) argv[4];
    double at[3];
    at[0] = *x;
    at[1] = *y;
    at[2] = 0.0;
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    *f2 = ( float ) F[1];
    return 0;
}

SHADEOP ( worleyP_FF ) {
    float *P = ( float* ) argv[1];
    float *f1 = ( float* ) argv[2];
    float *f2 = ( float* ) argv[3];
    double at[3];
    at[0] = ( double ) P[0];
    at[1] = ( double ) P[1];
    at[2] = ( double ) P[2];
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    *f2 = ( float ) F[1];
    return 0;
}

SHADEOP ( worleyFF_FFF ) {
    float *x = ( float* ) argv[1];
    float *y = ( float* ) argv[2];
    float *f1 = ( float* ) argv[3];
    float *f2 = ( float* ) argv[4];
    float *id1 = ( float* ) argv[5];
    double at[3];
    at[0] = *x;
    at[1] = *y;
    at[2] = 0.0;
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    *f2 = ( float ) F[1];
    *id1 = ( float ) ID[0]; // Scaling this to [0,1] could be more useful
    return 0;
}

SHADEOP ( worleyP_FFF ) {
    float *P = ( float* ) argv[1];
    float *f1 = ( float* ) argv[2];
    float *f2 = ( float* ) argv[3];
    float *id1 = ( float* ) argv[4];
    double at[3];
    at[0] = ( double ) P[0];
    at[1] = ( double ) P[1];
    at[2] = ( double ) P[2];
    double F[2];
    double delta[2][3];
    unsigned long ID[2];
    Worley( at, 2, F, delta, ID );
    *f1 = ( float ) F[0];
    *f2 = ( float ) F[1];
    *id1 = ( float ) ID[0]; // Scaling this to [0,1] could be more useful
    return 0;
}
