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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 15 */
real_t state[ 15 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 2];
state[1] = acadoVariables.x[lRun1 * 2 + 1];

state[8] = acadoVariables.u[lRun1];
state[9] = acadoVariables.od[lRun1 * 6];
state[10] = acadoVariables.od[lRun1 * 6 + 1];
state[11] = acadoVariables.od[lRun1 * 6 + 2];
state[12] = acadoVariables.od[lRun1 * 6 + 3];
state[13] = acadoVariables.od[lRun1 * 6 + 4];
state[14] = acadoVariables.od[lRun1 * 6 + 5];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 2] = state[0] - acadoVariables.x[lRun1 * 2 + 2];
acadoWorkspace.d[lRun1 * 2 + 1] = state[1] - acadoVariables.x[lRun1 * 2 + 3];

acadoWorkspace.evGx[lRun1 * 4] = state[2];
acadoWorkspace.evGx[lRun1 * 4 + 1] = state[3];
acadoWorkspace.evGx[lRun1 * 4 + 2] = state[4];
acadoWorkspace.evGx[lRun1 * 4 + 3] = state[5];

acadoWorkspace.evGu[lRun1 * 2] = state[6];
acadoWorkspace.evGu[lRun1 * 2 + 1] = state[7];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj];
acadoWorkspace.objValueIn[3] = acadoVariables.od[runObj * 6];
acadoWorkspace.objValueIn[4] = acadoVariables.od[runObj * 6 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 6 + 2];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 6 + 3];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 6 + 4];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 6 + 5];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.x[41];
acadoWorkspace.objValueIn[2] = acadoVariables.od[120];
acadoWorkspace.objValueIn[3] = acadoVariables.od[121];
acadoWorkspace.objValueIn[4] = acadoVariables.od[122];
acadoWorkspace.objValueIn[5] = acadoVariables.od[123];
acadoWorkspace.objValueIn[6] = acadoVariables.od[124];
acadoWorkspace.objValueIn[7] = acadoVariables.od[125];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1];
Gu2[1] = + Gx1[2]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 21] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + (real_t)2.0000000000000001e-01;
acadoWorkspace.H[iRow * 21] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[2]*Gu1[1];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[3]*Gu1[1];
}

void acado_multQEW2( real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + (real_t)1.0000000000000000e+01*Gu1[0] + Gu2[0];
Gu3[1] = +Gu1[1] + Gu2[1];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[2]*w11[1] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[3]*w11[1] + w12[1];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1];
}

void acado_macQSbarW2( real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + (real_t)1.0000000000000000e+01*w11[0] + w12[0];
w13[1] = +w11[1] + w12[1];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = acadoWorkspace.H[(iCol * 20) + (iRow)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + (real_t)2.0000000000000001e-01*Dy1[2];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + (real_t)1.0000000000000000e+01*Dy1[0];
QDy1[1] = +Dy1[1];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e-02*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e-02*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e-02*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e-02*Gx1[3];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e-02*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e-02*Gu1[1];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 40 */
static const int xBoundIndices[ 40 ] = 
{ 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41 };
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 2 ]), &(acadoWorkspace.E[ lRun3 * 2 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (2)) * (2)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (2)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (2)) * (1)) + (0) ]) );
}

acado_multQN1Gu( &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (2)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 2 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 4 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (2)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.evGu[ lRun2 * 2 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acadoWorkspace.sbar[12] = acadoWorkspace.d[10];
acadoWorkspace.sbar[13] = acadoWorkspace.d[11];
acadoWorkspace.sbar[14] = acadoWorkspace.d[12];
acadoWorkspace.sbar[15] = acadoWorkspace.d[13];
acadoWorkspace.sbar[16] = acadoWorkspace.d[14];
acadoWorkspace.sbar[17] = acadoWorkspace.d[15];
acadoWorkspace.sbar[18] = acadoWorkspace.d[16];
acadoWorkspace.sbar[19] = acadoWorkspace.d[17];
acadoWorkspace.sbar[20] = acadoWorkspace.d[18];
acadoWorkspace.sbar[21] = acadoWorkspace.d[19];
acadoWorkspace.sbar[22] = acadoWorkspace.d[20];
acadoWorkspace.sbar[23] = acadoWorkspace.d[21];
acadoWorkspace.sbar[24] = acadoWorkspace.d[22];
acadoWorkspace.sbar[25] = acadoWorkspace.d[23];
acadoWorkspace.sbar[26] = acadoWorkspace.d[24];
acadoWorkspace.sbar[27] = acadoWorkspace.d[25];
acadoWorkspace.sbar[28] = acadoWorkspace.d[26];
acadoWorkspace.sbar[29] = acadoWorkspace.d[27];
acadoWorkspace.sbar[30] = acadoWorkspace.d[28];
acadoWorkspace.sbar[31] = acadoWorkspace.d[29];
acadoWorkspace.sbar[32] = acadoWorkspace.d[30];
acadoWorkspace.sbar[33] = acadoWorkspace.d[31];
acadoWorkspace.sbar[34] = acadoWorkspace.d[32];
acadoWorkspace.sbar[35] = acadoWorkspace.d[33];
acadoWorkspace.sbar[36] = acadoWorkspace.d[34];
acadoWorkspace.sbar[37] = acadoWorkspace.d[35];
acadoWorkspace.sbar[38] = acadoWorkspace.d[36];
acadoWorkspace.sbar[39] = acadoWorkspace.d[37];
acadoWorkspace.sbar[40] = acadoWorkspace.d[38];
acadoWorkspace.sbar[41] = acadoWorkspace.d[39];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 2;
lRun4 = ((lRun3) / (2)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 39)) / (2)) + (lRun4)) - (1)) * (2)) + ((lRun3) % (2));
acadoWorkspace.A[(lRun1 * 20) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.g[ 19 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 22 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.QDy[ 26 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.QDy[ 34 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.QDy[ 38 ]) );

acadoWorkspace.QDy[40] = + (real_t)1.0000000000000000e-02*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[41] = + (real_t)1.0000000000000000e-02*acadoWorkspace.DyN[1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 40 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 22 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 44 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 52 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 56 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 60 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 34 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 68 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 38 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 76 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.sbar[ 40 ]) );

acadoWorkspace.w1[0] = + (real_t)1.0000000000000000e-02*acadoWorkspace.sbar[40] + acadoWorkspace.QDy[40];
acadoWorkspace.w1[1] = + (real_t)1.0000000000000000e-02*acadoWorkspace.sbar[41] + acadoWorkspace.QDy[41];
acado_macBTw1( &(acadoWorkspace.evGu[ 38 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 76 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 38 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 38 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 34 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 68 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 34 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 34 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 26 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 52 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 26 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 26 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 22 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 44 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 22 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 22 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 14 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];

tmp = acadoWorkspace.sbar[2] + acadoVariables.x[2];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[3] + acadoVariables.x[3];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[4] + acadoVariables.x[4];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[5] + acadoVariables.x[5];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[6] + acadoVariables.x[6];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[7] + acadoVariables.x[7];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[11] + acadoVariables.x[11];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = acadoWorkspace.sbar[23] + acadoVariables.x[23];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = acadoWorkspace.sbar[38] + acadoVariables.x[38];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = acadoWorkspace.sbar[41] + acadoVariables.x[41];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acadoWorkspace.sbar[12] = acadoWorkspace.d[10];
acadoWorkspace.sbar[13] = acadoWorkspace.d[11];
acadoWorkspace.sbar[14] = acadoWorkspace.d[12];
acadoWorkspace.sbar[15] = acadoWorkspace.d[13];
acadoWorkspace.sbar[16] = acadoWorkspace.d[14];
acadoWorkspace.sbar[17] = acadoWorkspace.d[15];
acadoWorkspace.sbar[18] = acadoWorkspace.d[16];
acadoWorkspace.sbar[19] = acadoWorkspace.d[17];
acadoWorkspace.sbar[20] = acadoWorkspace.d[18];
acadoWorkspace.sbar[21] = acadoWorkspace.d[19];
acadoWorkspace.sbar[22] = acadoWorkspace.d[20];
acadoWorkspace.sbar[23] = acadoWorkspace.d[21];
acadoWorkspace.sbar[24] = acadoWorkspace.d[22];
acadoWorkspace.sbar[25] = acadoWorkspace.d[23];
acadoWorkspace.sbar[26] = acadoWorkspace.d[24];
acadoWorkspace.sbar[27] = acadoWorkspace.d[25];
acadoWorkspace.sbar[28] = acadoWorkspace.d[26];
acadoWorkspace.sbar[29] = acadoWorkspace.d[27];
acadoWorkspace.sbar[30] = acadoWorkspace.d[28];
acadoWorkspace.sbar[31] = acadoWorkspace.d[29];
acadoWorkspace.sbar[32] = acadoWorkspace.d[30];
acadoWorkspace.sbar[33] = acadoWorkspace.d[31];
acadoWorkspace.sbar[34] = acadoWorkspace.d[32];
acadoWorkspace.sbar[35] = acadoWorkspace.d[33];
acadoWorkspace.sbar[36] = acadoWorkspace.d[34];
acadoWorkspace.sbar[37] = acadoWorkspace.d[35];
acadoWorkspace.sbar[38] = acadoWorkspace.d[36];
acadoWorkspace.sbar[39] = acadoWorkspace.d[37];
acadoWorkspace.sbar[40] = acadoWorkspace.d[38];
acadoWorkspace.sbar[41] = acadoWorkspace.d[39];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 40 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 22 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 44 ]), &(acadoWorkspace.evGu[ 22 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 52 ]), &(acadoWorkspace.evGu[ 26 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 56 ]), &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 60 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 34 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 68 ]), &(acadoWorkspace.evGu[ 34 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 38 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 76 ]), &(acadoWorkspace.evGu[ 38 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.sbar[ 40 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -2.0000000000000001e-01;
acadoVariables.lbValues[1] = -2.0000000000000001e-01;
acadoVariables.lbValues[2] = -2.0000000000000001e-01;
acadoVariables.lbValues[3] = -2.0000000000000001e-01;
acadoVariables.lbValues[4] = -2.0000000000000001e-01;
acadoVariables.lbValues[5] = -2.0000000000000001e-01;
acadoVariables.lbValues[6] = -2.0000000000000001e-01;
acadoVariables.lbValues[7] = -2.0000000000000001e-01;
acadoVariables.lbValues[8] = -2.0000000000000001e-01;
acadoVariables.lbValues[9] = -2.0000000000000001e-01;
acadoVariables.lbValues[10] = -2.0000000000000001e-01;
acadoVariables.lbValues[11] = -2.0000000000000001e-01;
acadoVariables.lbValues[12] = -2.0000000000000001e-01;
acadoVariables.lbValues[13] = -2.0000000000000001e-01;
acadoVariables.lbValues[14] = -2.0000000000000001e-01;
acadoVariables.lbValues[15] = -2.0000000000000001e-01;
acadoVariables.lbValues[16] = -2.0000000000000001e-01;
acadoVariables.lbValues[17] = -2.0000000000000001e-01;
acadoVariables.lbValues[18] = -2.0000000000000001e-01;
acadoVariables.lbValues[19] = -2.0000000000000001e-01;
acadoVariables.ubValues[0] = 2.0000000000000001e-01;
acadoVariables.ubValues[1] = 2.0000000000000001e-01;
acadoVariables.ubValues[2] = 2.0000000000000001e-01;
acadoVariables.ubValues[3] = 2.0000000000000001e-01;
acadoVariables.ubValues[4] = 2.0000000000000001e-01;
acadoVariables.ubValues[5] = 2.0000000000000001e-01;
acadoVariables.ubValues[6] = 2.0000000000000001e-01;
acadoVariables.ubValues[7] = 2.0000000000000001e-01;
acadoVariables.ubValues[8] = 2.0000000000000001e-01;
acadoVariables.ubValues[9] = 2.0000000000000001e-01;
acadoVariables.ubValues[10] = 2.0000000000000001e-01;
acadoVariables.ubValues[11] = 2.0000000000000001e-01;
acadoVariables.ubValues[12] = 2.0000000000000001e-01;
acadoVariables.ubValues[13] = 2.0000000000000001e-01;
acadoVariables.ubValues[14] = 2.0000000000000001e-01;
acadoVariables.ubValues[15] = 2.0000000000000001e-01;
acadoVariables.ubValues[16] = 2.0000000000000001e-01;
acadoVariables.ubValues[17] = 2.0000000000000001e-01;
acadoVariables.ubValues[18] = 2.0000000000000001e-01;
acadoVariables.ubValues[19] = 2.0000000000000001e-01;
acadoVariables.lbAValues[0] = -5.0000000000000003e-02;
acadoVariables.lbAValues[1] = -2.9999999999999999e-01;
acadoVariables.lbAValues[2] = -5.0000000000000003e-02;
acadoVariables.lbAValues[3] = -2.9999999999999999e-01;
acadoVariables.lbAValues[4] = -5.0000000000000003e-02;
acadoVariables.lbAValues[5] = -2.9999999999999999e-01;
acadoVariables.lbAValues[6] = -5.0000000000000003e-02;
acadoVariables.lbAValues[7] = -2.9999999999999999e-01;
acadoVariables.lbAValues[8] = -5.0000000000000003e-02;
acadoVariables.lbAValues[9] = -2.9999999999999999e-01;
acadoVariables.lbAValues[10] = -5.0000000000000003e-02;
acadoVariables.lbAValues[11] = -2.9999999999999999e-01;
acadoVariables.lbAValues[12] = -5.0000000000000003e-02;
acadoVariables.lbAValues[13] = -2.9999999999999999e-01;
acadoVariables.lbAValues[14] = -5.0000000000000003e-02;
acadoVariables.lbAValues[15] = -2.9999999999999999e-01;
acadoVariables.lbAValues[16] = -5.0000000000000003e-02;
acadoVariables.lbAValues[17] = -2.9999999999999999e-01;
acadoVariables.lbAValues[18] = -5.0000000000000003e-02;
acadoVariables.lbAValues[19] = -2.9999999999999999e-01;
acadoVariables.lbAValues[20] = -5.0000000000000003e-02;
acadoVariables.lbAValues[21] = -2.9999999999999999e-01;
acadoVariables.lbAValues[22] = -5.0000000000000003e-02;
acadoVariables.lbAValues[23] = -2.9999999999999999e-01;
acadoVariables.lbAValues[24] = -5.0000000000000003e-02;
acadoVariables.lbAValues[25] = -2.9999999999999999e-01;
acadoVariables.lbAValues[26] = -5.0000000000000003e-02;
acadoVariables.lbAValues[27] = -2.9999999999999999e-01;
acadoVariables.lbAValues[28] = -5.0000000000000003e-02;
acadoVariables.lbAValues[29] = -2.9999999999999999e-01;
acadoVariables.lbAValues[30] = -5.0000000000000003e-02;
acadoVariables.lbAValues[31] = -2.9999999999999999e-01;
acadoVariables.lbAValues[32] = -5.0000000000000003e-02;
acadoVariables.lbAValues[33] = -2.9999999999999999e-01;
acadoVariables.lbAValues[34] = -5.0000000000000003e-02;
acadoVariables.lbAValues[35] = -2.9999999999999999e-01;
acadoVariables.lbAValues[36] = -5.0000000000000003e-02;
acadoVariables.lbAValues[37] = -2.9999999999999999e-01;
acadoVariables.lbAValues[38] = -5.0000000000000003e-02;
acadoVariables.lbAValues[39] = -2.9999999999999999e-01;
acadoVariables.ubAValues[0] = 5.0000000000000003e-02;
acadoVariables.ubAValues[1] = 2.9999999999999999e-01;
acadoVariables.ubAValues[2] = 5.0000000000000003e-02;
acadoVariables.ubAValues[3] = 2.9999999999999999e-01;
acadoVariables.ubAValues[4] = 5.0000000000000003e-02;
acadoVariables.ubAValues[5] = 2.9999999999999999e-01;
acadoVariables.ubAValues[6] = 5.0000000000000003e-02;
acadoVariables.ubAValues[7] = 2.9999999999999999e-01;
acadoVariables.ubAValues[8] = 5.0000000000000003e-02;
acadoVariables.ubAValues[9] = 2.9999999999999999e-01;
acadoVariables.ubAValues[10] = 5.0000000000000003e-02;
acadoVariables.ubAValues[11] = 2.9999999999999999e-01;
acadoVariables.ubAValues[12] = 5.0000000000000003e-02;
acadoVariables.ubAValues[13] = 2.9999999999999999e-01;
acadoVariables.ubAValues[14] = 5.0000000000000003e-02;
acadoVariables.ubAValues[15] = 2.9999999999999999e-01;
acadoVariables.ubAValues[16] = 5.0000000000000003e-02;
acadoVariables.ubAValues[17] = 2.9999999999999999e-01;
acadoVariables.ubAValues[18] = 5.0000000000000003e-02;
acadoVariables.ubAValues[19] = 2.9999999999999999e-01;
acadoVariables.ubAValues[20] = 5.0000000000000003e-02;
acadoVariables.ubAValues[21] = 2.9999999999999999e-01;
acadoVariables.ubAValues[22] = 5.0000000000000003e-02;
acadoVariables.ubAValues[23] = 2.9999999999999999e-01;
acadoVariables.ubAValues[24] = 5.0000000000000003e-02;
acadoVariables.ubAValues[25] = 2.9999999999999999e-01;
acadoVariables.ubAValues[26] = 5.0000000000000003e-02;
acadoVariables.ubAValues[27] = 2.9999999999999999e-01;
acadoVariables.ubAValues[28] = 5.0000000000000003e-02;
acadoVariables.ubAValues[29] = 2.9999999999999999e-01;
acadoVariables.ubAValues[30] = 5.0000000000000003e-02;
acadoVariables.ubAValues[31] = 2.9999999999999999e-01;
acadoVariables.ubAValues[32] = 5.0000000000000003e-02;
acadoVariables.ubAValues[33] = 2.9999999999999999e-01;
acadoVariables.ubAValues[34] = 5.0000000000000003e-02;
acadoVariables.ubAValues[35] = 2.9999999999999999e-01;
acadoVariables.ubAValues[36] = 5.0000000000000003e-02;
acadoVariables.ubAValues[37] = 2.9999999999999999e-01;
acadoVariables.ubAValues[38] = 5.0000000000000003e-02;
acadoVariables.ubAValues[39] = 2.9999999999999999e-01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 2];
state[1] = acadoVariables.x[index * 2 + 1];
state[8] = acadoVariables.u[index];
state[9] = acadoVariables.od[index * 6];
state[10] = acadoVariables.od[index * 6 + 1];
state[11] = acadoVariables.od[index * 6 + 2];
state[12] = acadoVariables.od[index * 6 + 3];
state[13] = acadoVariables.od[index * 6 + 4];
state[14] = acadoVariables.od[index * 6 + 5];

acado_integrate(state, index == 0);

acadoVariables.x[index * 2 + 2] = state[0];
acadoVariables.x[index * 2 + 3] = state[1];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[40] = xEnd[0];
acadoVariables.x[41] = xEnd[1];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[40];
state[1] = acadoVariables.x[41];
if (uEnd != 0)
{
state[8] = uEnd[0];
}
else
{
state[8] = acadoVariables.u[19];
}
state[9] = acadoVariables.od[120];
state[10] = acadoVariables.od[121];
state[11] = acadoVariables.od[122];
state[12] = acadoVariables.od[123];
state[13] = acadoVariables.od[124];
state[14] = acadoVariables.od[125];

acado_integrate(state, 1);

acadoVariables.x[40] = state[0];
acadoVariables.x[41] = state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[19] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1];
acadoWorkspace.objValueIn[3] = acadoVariables.od[lRun1 * 6];
acadoWorkspace.objValueIn[4] = acadoVariables.od[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 6 + 5];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.x[41];
acadoWorkspace.objValueIn[2] = acadoVariables.od[120];
acadoWorkspace.objValueIn[3] = acadoVariables.od[121];
acadoWorkspace.objValueIn[4] = acadoVariables.od[122];
acadoWorkspace.objValueIn[5] = acadoVariables.od[123];
acadoWorkspace.objValueIn[6] = acadoVariables.od[124];
acadoWorkspace.objValueIn[7] = acadoVariables.od[125];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3]*(real_t)1.0000000000000000e+01;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2]*(real_t)2.0000000000000001e-01;
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e-02;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e-02;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

