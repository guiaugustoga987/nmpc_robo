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


real_t rk_dim2_swap;

/** Column vector of size: 2 */
real_t rk_dim2_bPerm[ 2 ];

/** Column vector of size: 7 */
real_t auxVar[ 7 ];

real_t rk_ttt;

/** Row vector of size: 11 */
real_t rk_xxx[ 11 ];

/** Column vector of size: 2 */
real_t rk_kkk[ 2 ];

/** Matrix of size: 2 x 2 (row major format) */
real_t rk_A[ 4 ];

/** Column vector of size: 2 */
real_t rk_b[ 2 ];

/** Row vector of size: 2 */
int rk_dim2_perm[ 2 ];

/** Column vector of size: 2 */
real_t rk_rhsTemp[ 2 ];

/** Row vector of size: 6 */
real_t rk_diffsTemp2[ 6 ];

/** Column vector of size: 2 */
real_t rk_diffK[ 2 ];

/** Matrix of size: 2 x 3 (row major format) */
real_t rk_diffsPrev2[ 6 ];

/** Matrix of size: 2 x 3 (row major format) */
real_t rk_diffsNew2[ 6 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim2_perm, rk_A, rk_b, rk_diffsPrev2, rk_diffsNew2, rk_diffsTemp2, rk_dim2_swap, rk_dim2_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;
const real_t* od = in + 3;
/* Vector of auxiliary variables; number of elements: 7. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(od[0]));
a[1] = (cos(od[1]));
a[2] = (cos(od[0]));
a[3] = (cos(od[1]));
a[4] = (cos(od[1]));
a[5] = (cos(od[2]));
a[6] = (tan(od[4]));

/* Compute outputs: */
out[0] = ((((a[0]/a[1])*od[5])*od[6])+(((((a[2]/a[3])*od[5])*od[3])+((a[4]*a[5])*od[7]))*a[6]));
out[1] = u[0];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim2_triangular( real_t* const A, real_t* const b )
{

b[1] = b[1]/A[3];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim2_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 2; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
if(fabs(A[2]) > fabs(A[0])) {
rk_dim2_swap = A[0];
A[0] = A[2];
A[2] = rk_dim2_swap;
rk_dim2_swap = A[1];
A[1] = A[3];
A[3] = rk_dim2_swap;
rk_dim2_swap = b[0];
b[0] = b[1];
b[1] = rk_dim2_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[1];
rk_perm[1] = intSwap;
}

A[2] = -A[2]/A[0];
A[3] += + A[2]*A[1];
b[1] += + A[2]*b[0];

det = + det*A[0];

det = + det*A[3];

det = fabs(det);
acado_solve_dim2_triangular( A, b );
return det;
}

void acado_solve_dim2_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim2_bPerm[0] = b[rk_perm[0]];
rk_dim2_bPerm[1] = b[rk_perm[1]];
rk_dim2_bPerm[1] += A[2]*rk_dim2_bPerm[0];


acado_solve_dim2_triangular( A, rk_dim2_bPerm );
b[0] = rk_dim2_bPerm[0];
b[1] = rk_dim2_bPerm[1];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 5.0000000000000003e-02 };


/* Fixed step size:0.1 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[2] = rk_eta[8];
rk_xxx[3] = rk_eta[9];
rk_xxx[4] = rk_eta[10];
rk_xxx[5] = rk_eta[11];
rk_xxx[6] = rk_eta[12];
rk_xxx[7] = rk_eta[13];
rk_xxx[8] = rk_eta[14];
rk_xxx[9] = rk_eta[15];
rk_xxx[10] = rk_eta[16];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 2; ++i)
{
rk_diffsPrev2[i * 3] = rk_eta[i * 2 + 2];
rk_diffsPrev2[i * 3 + 1] = rk_eta[i * 2 + 3];
rk_diffsPrev2[i * 3 + 2] = rk_eta[i + 6];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 6 ]) );
for (j = 0; j < 2; ++j)
{
tmp_index1 = (run1 * 2) + (j);
rk_A[tmp_index1 * 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 6) + (j * 3)];
rk_A[tmp_index1 * 2 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 6) + (j * 3 + 1)];
if( 0 == run1 ) rk_A[(tmp_index1 * 2) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 2] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 2 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
}
det = acado_solve_dim2_system( rk_A, rk_b, rk_dim2_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 2];
rk_kkk[j + 1] += rk_b[j * 2 + 1];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 2] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 2 + 1] = rk_kkk[run1 + 1] - rk_rhsTemp[1];
}
acado_solve_dim2_system_reuse( rk_A, rk_b, rk_dim2_perm );
for (j = 0; j < 1; ++j)
{
rk_kkk[j] += rk_b[j * 2];
rk_kkk[j + 1] += rk_b[j * 2 + 1];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1]*rk_kkk[tmp_index1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 6 ]) );
for (j = 0; j < 2; ++j)
{
tmp_index1 = (run1 * 2) + (j);
rk_A[tmp_index1 * 2] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 6) + (j * 3)];
rk_A[tmp_index1 * 2 + 1] = + acado_Ah_mat[run1]*rk_diffsTemp2[(run1 * 6) + (j * 3 + 1)];
if( 0 == run1 ) rk_A[(tmp_index1 * 2) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 1; ++i)
{
rk_b[i * 2] = - rk_diffsTemp2[(i * 6) + (run1)];
rk_b[i * 2 + 1] = - rk_diffsTemp2[(i * 6) + (run1 + 3)];
}
if( 0 == run1 ) {
det = acado_solve_dim2_system( rk_A, rk_b, rk_dim2_perm );
}
 else {
acado_solve_dim2_system_reuse( rk_A, rk_b, rk_dim2_perm );
}
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 2];
rk_diffK[i + 1] = rk_b[i * 2 + 1];
}
for (i = 0; i < 2; ++i)
{
rk_diffsNew2[(i * 3) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 3) + (run1)] += + rk_diffK[i]*(real_t)1.0000000000000001e-01;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index1 = (i * 2) + (j);
tmp_index2 = (run1) + (j * 3);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 6) + (tmp_index2 + 2)];
}
}
acado_solve_dim2_system_reuse( rk_A, rk_b, rk_dim2_perm );
for (i = 0; i < 1; ++i)
{
rk_diffK[i] = rk_b[i * 2];
rk_diffK[i + 1] = rk_b[i * 2 + 1];
}
for (i = 0; i < 2; ++i)
{
rk_diffsNew2[(i * 3) + (run1 + 2)] = + rk_diffK[i]*(real_t)1.0000000000000001e-01;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)1.0000000000000001e-01;
rk_eta[1] += + rk_kkk[1]*(real_t)1.0000000000000001e-01;
if( run == 0 ) {
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 2] = rk_diffsNew2[(i * 3) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 6] = rk_diffsNew2[(i * 3) + (j + 2)];
}
}
}
else {
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 2] = + rk_diffsNew2[i * 3]*rk_diffsPrev2[j];
rk_eta[tmp_index2 + 2] += + rk_diffsNew2[i * 3 + 1]*rk_diffsPrev2[j + 3];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 6] = rk_diffsNew2[(i * 3) + (j + 2)];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 3]*rk_diffsPrev2[j + 2];
rk_eta[tmp_index2 + 6] += + rk_diffsNew2[i * 3 + 1]*rk_diffsPrev2[j + 5];
}
}
}
resetIntegrator = 0;
rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 2; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



