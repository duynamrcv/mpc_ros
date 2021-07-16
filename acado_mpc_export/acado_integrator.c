/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
/* Vector of auxiliary variables; number of elements: 4. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = (sin(xd[2]));
a[3] = (cos(xd[2]));

/* Compute outputs: */
out[0] = ((u[0]*a[0])-(u[1]*a[1]));
out[1] = ((u[0]*a[2])+(u[1]*a[3]));
out[2] = u[2];
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 21;
/* Vector of auxiliary variables; number of elements: 32. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (cos(xd[2]));
a[1] = (sin(xd[2]));
a[2] = (sin(xd[2]));
a[3] = (cos(xd[2]));
a[4] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[5] = (xd[9]*a[4]);
a[6] = (cos(xd[2]));
a[7] = (xd[9]*a[6]);
a[8] = (xd[10]*a[4]);
a[9] = (xd[10]*a[6]);
a[10] = (xd[11]*a[4]);
a[11] = (xd[11]*a[6]);
a[12] = (cos(xd[2]));
a[13] = (xd[9]*a[12]);
a[14] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[2])));
a[15] = (xd[9]*a[14]);
a[16] = (xd[10]*a[12]);
a[17] = (xd[10]*a[14]);
a[18] = (xd[11]*a[12]);
a[19] = (xd[11]*a[14]);
a[20] = (xd[18]*a[4]);
a[21] = (xd[18]*a[6]);
a[22] = (xd[19]*a[4]);
a[23] = (xd[19]*a[6]);
a[24] = (xd[20]*a[4]);
a[25] = (xd[20]*a[6]);
a[26] = (xd[18]*a[12]);
a[27] = (xd[18]*a[14]);
a[28] = (xd[19]*a[12]);
a[29] = (xd[19]*a[14]);
a[30] = (xd[20]*a[12]);
a[31] = (xd[20]*a[14]);

/* Compute outputs: */
out[0] = ((u[0]*a[0])-(u[1]*a[1]));
out[1] = ((u[0]*a[2])+(u[1]*a[3]));
out[2] = u[2];
out[3] = ((u[0]*a[5])-(u[1]*a[7]));
out[4] = ((u[0]*a[8])-(u[1]*a[9]));
out[5] = ((u[0]*a[10])-(u[1]*a[11]));
out[6] = ((u[0]*a[13])+(u[1]*a[15]));
out[7] = ((u[0]*a[16])+(u[1]*a[17]));
out[8] = ((u[0]*a[18])+(u[1]*a[19]));
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (((u[0]*a[20])-(u[1]*a[21]))+a[0]);
out[13] = (((u[0]*a[22])-(u[1]*a[23]))+((real_t)(0.0000000000000000e+00)-a[1]));
out[14] = ((u[0]*a[24])-(u[1]*a[25]));
out[15] = (((u[0]*a[26])+(u[1]*a[27]))+a[2]);
out[16] = (((u[0]*a[28])+(u[1]*a[29]))+a[3]);
out[17] = ((u[0]*a[30])+(u[1]*a[31]));
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(1.0000000000000000e+00);
}

/* Fixed step size:0.0111111 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
rk_eta[3] = 1.0000000000000000e+00;
rk_eta[4] = 0.0000000000000000e+00;
rk_eta[5] = 0.0000000000000000e+00;
rk_eta[6] = 0.0000000000000000e+00;
rk_eta[7] = 1.0000000000000000e+00;
rk_eta[8] = 0.0000000000000000e+00;
rk_eta[9] = 0.0000000000000000e+00;
rk_eta[10] = 0.0000000000000000e+00;
rk_eta[11] = 1.0000000000000000e+00;
rk_eta[12] = 0.0000000000000000e+00;
rk_eta[13] = 0.0000000000000000e+00;
rk_eta[14] = 0.0000000000000000e+00;
rk_eta[15] = 0.0000000000000000e+00;
rk_eta[16] = 0.0000000000000000e+00;
rk_eta[17] = 0.0000000000000000e+00;
rk_eta[18] = 0.0000000000000000e+00;
rk_eta[19] = 0.0000000000000000e+00;
rk_eta[20] = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[21] = rk_eta[21];
acadoWorkspace.rk_xxx[22] = rk_eta[22];
acadoWorkspace.rk_xxx[23] = rk_eta[23];

for (run1 = 0; run1 < 9; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + rk_eta[20];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[15] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[16] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[17] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[18] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[19] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)3.7037037037037038e-03*acadoWorkspace.rk_kkk[20] + rk_eta[20];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 21 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[21] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[22] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[23] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[24] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[25] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[26] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[27] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[28] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[29] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[30] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[31] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[32] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[33] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[34] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[35] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[36] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[37] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[38] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[39] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[40] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)7.4074074074074077e-03*acadoWorkspace.rk_kkk[41] + rk_eta[20];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 42 ]) );
rk_eta[0] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[0] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[42];
rk_eta[1] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[1] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[43];
rk_eta[2] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[2] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[44];
rk_eta[3] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[3] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[45];
rk_eta[4] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[4] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[46];
rk_eta[5] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[5] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[47];
rk_eta[6] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[6] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[48];
rk_eta[7] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[7] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[49];
rk_eta[8] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[8] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[50];
rk_eta[9] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[9] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[51];
rk_eta[10] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[10] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[52];
rk_eta[11] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[11] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[53];
rk_eta[12] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[12] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[54];
rk_eta[13] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[13] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[55];
rk_eta[14] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[14] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[56];
rk_eta[15] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[15] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[57];
rk_eta[16] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[16] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[58];
rk_eta[17] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[17] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[59];
rk_eta[18] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[18] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[60];
rk_eta[19] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[19] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[61];
rk_eta[20] += + (real_t)2.7777777777777779e-03*acadoWorkspace.rk_kkk[20] + (real_t)8.3333333333333332e-03*acadoWorkspace.rk_kkk[62];
acadoWorkspace.rk_ttt += 1.1111111111111110e-01;
}
error = 0;
return error;
}

