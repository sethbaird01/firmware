/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.c
 *
 * Code generated for Simulink model 'Electronics'.
 *
 * Model version                  : 1.78
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Tue Oct 18 21:02:27 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Electronics.h"
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "data.h"
#define NumBitsPerChar                 8U

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
static void Electronics_Init(DW_Electronics *localDW);
static void Electronics_j(const real_T rtu_TVS_Information[2], const real_T
  rtu_TVS_Information_h[3], real_T rtu_TVS_Information_o, const real_T
  rtu_TVS_Information_e[4], const real_T rtu_TVS_Information_n[2], const real_T
  rtu_TVS_Information_d[4], const real_T rtu_TVS_Information_e2[4], real_T
  rtu_TVS_Information_ds, real_T rtu_TVS_Information_a, const real_T
  rtu_TVS_Information_e1[4], real_T rty_Tx[4], DW_Electronics *localDW);

/* Forward declaration for local functions */
static real_T maximum(const real_T x[4]);
static real_T minimum_b(const real_T x[4]);
static void pchip(const real_T x[7], const real_T y[7], const real_T xx[4],
                  real_T v[4], DW_Electronics *localDW);
static void minimum(const real_T x[1004], real_T ex[4], int32_T idx[4],
                    DW_Electronics *localDW);
static real_T xnrm2(int32_T n, const real_T x[4], int32_T ix0, DW_Electronics
                    *localDW);
static void xgeqp3(real_T A[4], real_T *tau, int32_T *jpvt, DW_Electronics
                   *localDW);
static void interp1(const real_T varargin_1[68], const real_T varargin_2[68],
                    const real_T varargin_3[4], real_T Vq[4], DW_Electronics
                    *localDW);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T tmp;
  real_T tmp_0;
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T u0_0;
  int32_T u1_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2(u0_0, u1_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S3>/Tire Model' */
static real_T maximum(const real_T x[4])
{
  real_T ex;
  int32_T b_k;
  int32_T idx;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    b_k = 2;
    exitg1 = false;
    while ((!exitg1) && (b_k < 5)) {
      if (!rtIsNaN(x[b_k - 1])) {
        idx = b_k;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 4) {
      if (ex < x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S3>/Tire Model' */
static real_T minimum_b(const real_T x[4])
{
  real_T ex;
  int32_T b_k;
  int32_T idx;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    idx = 0;
    b_k = 2;
    exitg1 = false;
    while ((!exitg1) && (b_k < 5)) {
      if (!rtIsNaN(x[b_k - 1])) {
        idx = b_k;
        exitg1 = true;
      } else {
        b_k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    while (idx + 1 <= 4) {
      if (ex > x[idx]) {
        ex = x[idx];
      }

      idx++;
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S3>/Tire Model' */
static void pchip(const real_T x[7], const real_T y[7], const real_T xx[4],
                  real_T v[4], DW_Electronics *localDW)
{
  int32_T b_k;
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;
  int32_T mid_i;
  for (b_k = 0; b_k < 6; b_k++) {
    localDW->h_c = x[b_k + 1] - x[b_k];
    localDW->del[b_k] = (y[b_k + 1] - y[b_k]) / localDW->h_c;
    localDW->h[b_k] = localDW->h_c;
  }

  for (b_k = 0; b_k < 5; b_k++) {
    localDW->h_c = localDW->h[b_k + 1];
    localDW->hs = localDW->h_c + localDW->h[b_k];
    localDW->hs3 = 3.0 * localDW->hs;
    localDW->dzzdx = (localDW->h[b_k] + localDW->hs) / localDW->hs3;
    localDW->hs = (localDW->h_c + localDW->hs) / localDW->hs3;
    localDW->d_s[b_k + 1] = 0.0;
    if (localDW->del[b_k] < 0.0) {
      localDW->h_c = localDW->del[b_k + 1];
      if (localDW->h_c <= localDW->del[b_k]) {
        localDW->d_s[b_k + 1] = localDW->del[b_k] / (localDW->del[b_k] /
          localDW->h_c * localDW->dzzdx + localDW->hs);
      } else if (localDW->h_c < 0.0) {
        localDW->d_s[b_k + 1] = localDW->h_c / (localDW->h_c / localDW->del[b_k]
          * localDW->hs + localDW->dzzdx);
      }
    } else if (localDW->del[b_k] > 0.0) {
      localDW->h_c = localDW->del[b_k + 1];
      if (localDW->h_c >= localDW->del[b_k]) {
        localDW->d_s[b_k + 1] = localDW->del[b_k] / (localDW->del[b_k] /
          localDW->h_c * localDW->dzzdx + localDW->hs);
      } else if (localDW->h_c > 0.0) {
        localDW->d_s[b_k + 1] = localDW->h_c / (localDW->h_c / localDW->del[b_k]
          * localDW->hs + localDW->dzzdx);
      }
    }
  }

  localDW->dzzdx = ((2.0 * localDW->h[0] + localDW->h[1]) * localDW->del[0] -
                    localDW->h[0] * localDW->del[1]) / (localDW->h[0] +
    localDW->h[1]);
  if (rtIsNaN(localDW->del[0])) {
    localDW->hs = localDW->del[0];
  } else if (localDW->del[0] < 0.0) {
    localDW->hs = -1.0;
  } else {
    localDW->hs = (localDW->del[0] > 0.0);
  }

  if (rtIsNaN(localDW->dzzdx)) {
    localDW->h_c = localDW->dzzdx;
  } else if (localDW->dzzdx < 0.0) {
    localDW->h_c = -1.0;
  } else {
    localDW->h_c = (localDW->dzzdx > 0.0);
  }

  if (localDW->h_c != localDW->hs) {
    localDW->dzzdx = 0.0;
  } else {
    if (rtIsNaN(localDW->del[1])) {
      localDW->h_c = localDW->del[1];
    } else if (localDW->del[1] < 0.0) {
      localDW->h_c = -1.0;
    } else {
      localDW->h_c = (localDW->del[1] > 0.0);
    }

    if ((localDW->hs != localDW->h_c) && (fabs(localDW->dzzdx) > fabs(3.0 *
          localDW->del[0]))) {
      localDW->dzzdx = 3.0 * localDW->del[0];
    }
  }

  localDW->d_s[0] = localDW->dzzdx;
  localDW->dzzdx = ((2.0 * localDW->h[5] + localDW->h[4]) * localDW->del[5] -
                    localDW->del[4] * localDW->h[5]) / (localDW->h[4] +
    localDW->h[5]);
  if (rtIsNaN(localDW->del[5])) {
    localDW->hs = localDW->del[5];
  } else if (localDW->del[5] < 0.0) {
    localDW->hs = -1.0;
  } else {
    localDW->hs = (localDW->del[5] > 0.0);
  }

  if (rtIsNaN(localDW->dzzdx)) {
    localDW->h_c = localDW->dzzdx;
  } else if (localDW->dzzdx < 0.0) {
    localDW->h_c = -1.0;
  } else {
    localDW->h_c = (localDW->dzzdx > 0.0);
  }

  if (localDW->h_c != localDW->hs) {
    localDW->dzzdx = 0.0;
  } else {
    if (rtIsNaN(localDW->del[4])) {
      localDW->h_c = localDW->del[4];
    } else if (localDW->del[4] < 0.0) {
      localDW->h_c = -1.0;
    } else {
      localDW->h_c = (localDW->del[4] > 0.0);
    }

    if ((localDW->hs != localDW->h_c) && (fabs(localDW->dzzdx) > fabs(3.0 *
          localDW->del[5]))) {
      localDW->dzzdx = 3.0 * localDW->del[5];
    }
  }

  localDW->d_s[6] = localDW->dzzdx;
  for (b_k = 0; b_k < 6; b_k++) {
    localDW->h_c = localDW->h[b_k];
    localDW->hs = localDW->del[b_k];
    localDW->dzzdx = (localDW->hs - localDW->d_s[b_k]) / localDW->h_c;
    localDW->hs = (localDW->d_s[b_k + 1] - localDW->hs) / localDW->h_c;
    localDW->pp_coefs[b_k] = (localDW->hs - localDW->dzzdx) / localDW->h_c;
    localDW->pp_coefs[b_k + 6] = 2.0 * localDW->dzzdx - localDW->hs;
    localDW->pp_coefs[b_k + 12] = localDW->d_s[b_k];
    localDW->pp_coefs[b_k + 18] = y[b_k];
  }

  for (b_k = 0; b_k < 4; b_k++) {
    if (rtIsNaN(xx[b_k])) {
      v[b_k] = xx[b_k];
    } else {
      high_i = 7;
      low_i = 0;
      low_ip1 = 2;
      while (high_i > low_ip1) {
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (xx[b_k] >= x[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      localDW->dzzdx = xx[b_k] - x[low_i];
      v[b_k] = ((localDW->dzzdx * localDW->pp_coefs[low_i] + localDW->
                 pp_coefs[low_i + 6]) * localDW->dzzdx + localDW->pp_coefs[low_i
                + 12]) * localDW->dzzdx + localDW->pp_coefs[low_i + 18];
    }
  }
}

/* Function for MATLAB Function: '<S3>/Constraint Generation' */
static void minimum(const real_T x[1004], real_T ex[4], int32_T idx[4],
                    DW_Electronics *localDW)
{
  real_T tmp;
  int32_T i;
  int32_T j;
  boolean_T p;
  for (j = 0; j < 4; j++) {
    idx[j] = 1;
    ex[j] = x[251 * j];
    for (i = 0; i < 250; i++) {
      localDW->ex = ex[j];
      tmp = x[(251 * j + i) + 1];
      if (rtIsNaN(tmp)) {
        p = false;
      } else if (rtIsNaN(localDW->ex)) {
        p = true;
      } else {
        p = (localDW->ex > tmp);
      }

      if (p) {
        localDW->ex = tmp;
        idx[j] = i + 2;
      }

      ex[j] = localDW->ex;
    }
  }
}

/* Function for MATLAB Function: '<S3>/Constraint Generation' */
static real_T xnrm2(int32_T n, const real_T x[4], int32_T ix0, DW_Electronics
                    *localDW)
{
  real_T y;
  int32_T b_k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      localDW->scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (b_k = ix0; b_k <= kend; b_k++) {
        localDW->absxk = fabs(x[b_k - 1]);
        if (localDW->absxk > localDW->scale) {
          localDW->t = localDW->scale / localDW->absxk;
          y = y * localDW->t * localDW->t + 1.0;
          localDW->scale = localDW->absxk;
        } else {
          localDW->t = localDW->absxk / localDW->scale;
          y += localDW->t * localDW->t;
        }
      }

      y = localDW->scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a_0;
  real_T y;
  a_0 = fabs(u0);
  y = fabs(u1);
  if (a_0 < y) {
    a_0 /= y;
    y *= sqrt(a_0 * a_0 + 1.0);
  } else if (a_0 > y) {
    y /= a_0;
    y = sqrt(y * y + 1.0) * a_0;
  } else if (!rtIsNaN(y)) {
    y = a_0 * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S3>/Constraint Generation' */
static void xgeqp3(real_T A[4], real_T *tau, int32_T *jpvt, DW_Electronics
                   *localDW)
{
  int32_T c_k;
  int32_T knt;
  *tau = 0.0;
  localDW->atmp = A[0];
  localDW->xnorm = xnrm2(3, A, 2, localDW);
  if (localDW->xnorm != 0.0) {
    localDW->xnorm = rt_hypotd_snf(A[0], localDW->xnorm);
    if (A[0] >= 0.0) {
      localDW->xnorm = -localDW->xnorm;
    }

    if (fabs(localDW->xnorm) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (c_k = 1; c_k < 4; c_k++) {
          A[c_k] *= 9.9792015476736E+291;
        }

        localDW->xnorm *= 9.9792015476736E+291;
        localDW->atmp *= 9.9792015476736E+291;
      } while ((fabs(localDW->xnorm) < 1.0020841800044864E-292) && (knt < 20));

      localDW->xnorm = rt_hypotd_snf(localDW->atmp, xnrm2(3, A, 2, localDW));
      if (localDW->atmp >= 0.0) {
        localDW->xnorm = -localDW->xnorm;
      }

      *tau = (localDW->xnorm - localDW->atmp) / localDW->xnorm;
      localDW->atmp = 1.0 / (localDW->atmp - localDW->xnorm);
      for (c_k = 1; c_k < 4; c_k++) {
        A[c_k] *= localDW->atmp;
      }

      for (c_k = 0; c_k < knt; c_k++) {
        localDW->xnorm *= 1.0020841800044864E-292;
      }

      localDW->atmp = localDW->xnorm;
    } else {
      *tau = (localDW->xnorm - A[0]) / localDW->xnorm;
      localDW->atmp = 1.0 / (A[0] - localDW->xnorm);
      for (knt = 1; knt < 4; knt++) {
        A[knt] *= localDW->atmp;
      }

      localDW->atmp = localDW->xnorm;
    }
  }

  A[0] = localDW->atmp;
  *jpvt = 1;
}

/* Function for MATLAB Function: '<S3>/Constraint Generation' */
static void interp1(const real_T varargin_1[68], const real_T varargin_2[68],
                    const real_T varargin_3[4], real_T Vq[4], DW_Electronics
                    *localDW)
{
  int32_T exitg1;
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;
  int32_T mid_i;
  memcpy(&localDW->y[0], &varargin_2[0], 68U * sizeof(real_T));
  memcpy(&localDW->x[0], &varargin_1[0], 68U * sizeof(real_T));
  Vq[0] = (rtNaN);
  Vq[1] = (rtNaN);
  Vq[2] = (rtNaN);
  Vq[3] = (rtNaN);
  high_i = 0;
  do {
    exitg1 = 0;
    if (high_i < 68) {
      if (rtIsNaN(varargin_1[high_i])) {
        exitg1 = 1;
      } else {
        high_i++;
      }
    } else {
      if (varargin_1[1] < varargin_1[0]) {
        for (high_i = 0; high_i < 34; high_i++) {
          localDW->xtmp = localDW->x[high_i];
          localDW->x[high_i] = localDW->x[67 - high_i];
          localDW->x[67 - high_i] = localDW->xtmp;
          localDW->xtmp = localDW->y[high_i];
          localDW->y[high_i] = localDW->y[67 - high_i];
          localDW->y[67 - high_i] = localDW->xtmp;
        }
      }

      if (rtIsNaN(varargin_3[0])) {
        Vq[0] = (rtNaN);
      } else if ((!(varargin_3[0] > localDW->x[67])) && (!(varargin_3[0] <
                   localDW->x[0]))) {
        high_i = 68;
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (varargin_3[0] >= localDW->x[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        localDW->xtmp = localDW->x[low_i - 1];
        localDW->xtmp = (varargin_3[0] - localDW->xtmp) / (localDW->x[low_i] -
          localDW->xtmp);
        if (localDW->xtmp == 0.0) {
          Vq[0] = localDW->y[low_i - 1];
        } else if (localDW->xtmp == 1.0) {
          Vq[0] = localDW->y[low_i];
        } else {
          localDW->d1 = localDW->y[low_i - 1];
          if (localDW->d1 == localDW->y[low_i]) {
            Vq[0] = localDW->d1;
          } else {
            Vq[0] = (1.0 - localDW->xtmp) * localDW->d1 + localDW->xtmp *
              localDW->y[low_i];
          }
        }
      }

      if (rtIsNaN(varargin_3[1])) {
        Vq[1] = (rtNaN);
      } else if ((!(varargin_3[1] > localDW->x[67])) && (!(varargin_3[1] <
                   localDW->x[0]))) {
        high_i = 68;
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (varargin_3[1] >= localDW->x[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        localDW->xtmp = localDW->x[low_i - 1];
        localDW->xtmp = (varargin_3[1] - localDW->xtmp) / (localDW->x[low_i] -
          localDW->xtmp);
        if (localDW->xtmp == 0.0) {
          Vq[1] = localDW->y[low_i - 1];
        } else if (localDW->xtmp == 1.0) {
          Vq[1] = localDW->y[low_i];
        } else {
          localDW->d1 = localDW->y[low_i - 1];
          if (localDW->d1 == localDW->y[low_i]) {
            Vq[1] = localDW->d1;
          } else {
            Vq[1] = (1.0 - localDW->xtmp) * localDW->d1 + localDW->xtmp *
              localDW->y[low_i];
          }
        }
      }

      if (rtIsNaN(varargin_3[2])) {
        Vq[2] = (rtNaN);
      } else if ((!(varargin_3[2] > localDW->x[67])) && (!(varargin_3[2] <
                   localDW->x[0]))) {
        high_i = 68;
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (varargin_3[2] >= localDW->x[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        localDW->xtmp = localDW->x[low_i - 1];
        localDW->xtmp = (varargin_3[2] - localDW->xtmp) / (localDW->x[low_i] -
          localDW->xtmp);
        if (localDW->xtmp == 0.0) {
          Vq[2] = localDW->y[low_i - 1];
        } else if (localDW->xtmp == 1.0) {
          Vq[2] = localDW->y[low_i];
        } else {
          localDW->d1 = localDW->y[low_i - 1];
          if (localDW->d1 == localDW->y[low_i]) {
            Vq[2] = localDW->d1;
          } else {
            Vq[2] = (1.0 - localDW->xtmp) * localDW->d1 + localDW->xtmp *
              localDW->y[low_i];
          }
        }
      }

      if (rtIsNaN(varargin_3[3])) {
        Vq[3] = (rtNaN);
      } else if ((!(varargin_3[3] > localDW->x[67])) && (!(varargin_3[3] <
                   localDW->x[0]))) {
        high_i = 68;
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (varargin_3[3] >= localDW->x[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        localDW->xtmp = localDW->x[low_i - 1];
        localDW->xtmp = (varargin_3[3] - localDW->xtmp) / (localDW->x[low_i] -
          localDW->xtmp);
        if (localDW->xtmp == 0.0) {
          Vq[3] = localDW->y[low_i - 1];
        } else if (localDW->xtmp == 1.0) {
          Vq[3] = localDW->y[low_i];
        } else {
          localDW->d1 = localDW->y[low_i - 1];
          if (localDW->d1 == localDW->y[low_i]) {
            Vq[3] = localDW->d1;
          } else {
            Vq[3] = (1.0 - localDW->xtmp) * localDW->d1 + localDW->xtmp *
              localDW->y[low_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

/* System initialize for atomic system: '<Root>/Electronics' */
static void Electronics_Init(DW_Electronics *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' */
  localDW->DiscreteTimeIntegrator2_PrevRes = 2;
}

/* Output and update for atomic system: '<Root>/Electronics' */
static void Electronics_j(const real_T rtu_TVS_Information[2], const real_T
  rtu_TVS_Information_h[3], real_T rtu_TVS_Information_o, const real_T
  rtu_TVS_Information_e[4], const real_T rtu_TVS_Information_n[2], const real_T
  rtu_TVS_Information_d[4], const real_T rtu_TVS_Information_e2[4], real_T
  rtu_TVS_Information_ds, real_T rtu_TVS_Information_a, const real_T
  rtu_TVS_Information_e1[4], real_T rty_Tx[4], DW_Electronics *localDW)
{
  int32_T e_x_tmp;
  real32_T M_max_idx_0;
  real32_T M_max_idx_1;
  real32_T rtb_A_idx_2;
  real32_T rtb_A_idx_3;
  real32_T rtb_Aeq_idx_0;
  real32_T rtb_Aeq_idx_1;
  real32_T rtb_Aeq_idx_2;
  real32_T rtb_Aeq_idx_3;
  real32_T rtb_lb;
  boolean_T exitg1;
  boolean_T p;
  boolean_T rtb_UnitDelay4;

  /* MATLAB Function: '<S3>/Normal Force Model' */
  localDW->V_rr_idx_0 = 200.0;
  if (rtIsNaN(rtu_TVS_Information_e2[0])) {
    p = false;
  } else {
    p = (rtu_TVS_Information_e2[0] > 200.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = rtu_TVS_Information_e2[0];
  }

  localDW->FZ[0] = 1200.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 1200.0);
  }

  if (p) {
    localDW->FZ[0] = localDW->V_rr_idx_0;
  }

  localDW->V_rr_idx_0 = 200.0;
  if (rtIsNaN(rtu_TVS_Information_e2[1])) {
    p = false;
  } else {
    p = (rtu_TVS_Information_e2[1] > 200.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = rtu_TVS_Information_e2[1];
  }

  localDW->FZ[1] = 1200.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 1200.0);
  }

  if (p) {
    localDW->FZ[1] = localDW->V_rr_idx_0;
  }

  localDW->V_rr_idx_0 = 200.0;
  if (rtIsNaN(rtu_TVS_Information_e2[2])) {
    p = false;
  } else {
    p = (rtu_TVS_Information_e2[2] > 200.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = rtu_TVS_Information_e2[2];
  }

  localDW->FZ[2] = 1200.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 1200.0);
  }

  if (p) {
    localDW->FZ[2] = localDW->V_rr_idx_0;
  }

  localDW->V_rr_idx_0 = 200.0;
  if (rtIsNaN(rtu_TVS_Information_e2[3])) {
    p = false;
  } else {
    p = (rtu_TVS_Information_e2[3] > 200.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = rtu_TVS_Information_e2[3];
  }

  localDW->beq = 1200.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 1200.0);
  }

  if (p) {
    localDW->beq = localDW->V_rr_idx_0;
  }

  /* End of MATLAB Function: '<S3>/Normal Force Model' */

  /* MATLAB Function: '<S3>/Steering Model' */
  if ((rtu_TVS_Information_a <= -130.0) || rtIsNaN(rtu_TVS_Information_a)) {
    localDW->left_steering_angle = -130.0;
  } else {
    localDW->left_steering_angle = rtu_TVS_Information_a;
  }

  if ((localDW->left_steering_angle >= 130.0) || rtIsNaN
      (localDW->left_steering_angle)) {
    localDW->left_steering_angle = 130.0;
  }

  if ((localDW->left_steering_angle < rtP.deadband_angle) &&
      (localDW->left_steering_angle > -rtP.deadband_angle)) {
    localDW->left_steering_angle = 0.0;
  }

  localDW->rack_displacement = steer_slope * inch2mm *
    localDW->left_steering_angle;
  localDW->left_steering_angle = -(((-localDW->rack_displacement *
    -localDW->rack_displacement * S2 + S1 * rt_powd_snf
    (-localDW->rack_displacement, 3.0)) + S3 * -localDW->rack_displacement) + S4)
    * deg2rad;
  localDW->rack_displacement = (((localDW->rack_displacement *
    localDW->rack_displacement * S2 + S1 * rt_powd_snf
    (localDW->rack_displacement, 3.0)) + S3 * localDW->rack_displacement) + S4) *
    deg2rad;
  localDW->steering[0] = (localDW->left_steering_angle +
    localDW->rack_displacement) / 2.0;
  localDW->steering[1] = localDW->left_steering_angle;
  localDW->steering[2] = localDW->rack_displacement;

  /* MATLAB Function: '<S3>/Tire Model' incorporates:
   *  MATLAB Function: '<S2>/Optimization '
   *  MATLAB Function: '<S3>/Reference Generation'
   *  MATLAB Function: '<S3>/Steering Model'
   */
  localDW->V_rl_idx_0_tmp_tmp = s[1] * rtu_TVS_Information_h[2];
  localDW->V_rl_idx_0_tmp = localDW->V_rl_idx_0_tmp_tmp + rtu_TVS_Information[0];
  localDW->V_rr_idx_0 = rtu_TVS_Information_h[2] * -s[1] + rtu_TVS_Information[0];
  localDW->SA_fr = l[0] * rtu_TVS_Information_h[2] + rtu_TVS_Information[1];
  localDW->SA_rl = s[0] * rtu_TVS_Information_h[2];
  localDW->V_fl_tmp = localDW->SA_rl + rtu_TVS_Information[0];
  localDW->kx_idx_1 = localDW->SA_fr * -sin(localDW->left_steering_angle) +
    localDW->V_fl_tmp * cos(localDW->left_steering_angle);
  localDW->V_flt_idx_1 = (rtu_TVS_Information_h[2] * -s[0] +
    rtu_TVS_Information[0]) * cos(localDW->rack_displacement) + localDW->SA_fr *
    -sin(localDW->rack_displacement);
  localDW->d_k = fabs(rtu_TVS_Information[0]);
  if (localDW->d_k > rtP.Vth) {
    localDW->rack_displacement = -rt_atan2d_snf(localDW->SA_fr,
      localDW->V_fl_tmp) + localDW->left_steering_angle;
    localDW->SA_fr = -rt_atan2d_snf(localDW->SA_fr, rtu_TVS_Information[0] -
      localDW->SA_rl) + localDW->steering[2];
    localDW->left_steering_angle = rtu_TVS_Information[1] - l[1] *
      rtu_TVS_Information_h[2];
    localDW->SA_rl = -rt_atan2d_snf(localDW->left_steering_angle,
      localDW->V_rl_idx_0_tmp);
    localDW->left_steering_angle = -rt_atan2d_snf(localDW->left_steering_angle,
      rtu_TVS_Information[0] - localDW->V_rl_idx_0_tmp_tmp);
  } else {
    localDW->rack_displacement = 0.0;
    localDW->SA_fr = 0.0;
    localDW->SA_rl = 0.0;
    localDW->left_steering_angle = 0.0;
  }

  localDW->SA[1] = localDW->SA_fr;
  localDW->SA[2] = localDW->SA_rl;
  localDW->SA[3] = localDW->left_steering_angle;
  if (rtu_TVS_Information[0] > rtP.Vth) {
    localDW->SL[0] = RE * rtu_TVS_Information_e1[0] / (localDW->kx_idx_1 * cos
      (localDW->rack_displacement)) - 1.0;
    localDW->SA_fr = RE * rtu_TVS_Information_e1[1] / (localDW->V_flt_idx_1 *
      cos(localDW->SA_fr)) - 1.0;
    localDW->SA_rl = RE * rtu_TVS_Information_e1[2] / (localDW->V_rl_idx_0_tmp *
      cos(localDW->SA_rl)) - 1.0;
    localDW->left_steering_angle = RE * rtu_TVS_Information_e1[3] /
      (localDW->V_rr_idx_0 * cos(localDW->left_steering_angle)) - 1.0;
  } else {
    localDW->SL[0] = (RE * rtu_TVS_Information_e1[0] - localDW->kx_idx_1) /
      (localDW->kx_idx_1 * localDW->kx_idx_1 / rtP.Vth + rtP.Vth);
    localDW->SA_fr = (RE * rtu_TVS_Information_e1[1] - localDW->V_flt_idx_1) /
      (localDW->V_flt_idx_1 * localDW->V_flt_idx_1 / rtP.Vth + rtP.Vth);
    localDW->SA_rl = (RE * rtu_TVS_Information_e1[2] - localDW->V_rl_idx_0_tmp) /
      (localDW->V_rl_idx_0_tmp * localDW->V_rl_idx_0_tmp / rtP.Vth + rtP.Vth);
    localDW->left_steering_angle = (RE * rtu_TVS_Information_e1[3] -
      localDW->V_rr_idx_0) / (localDW->V_rr_idx_0 * localDW->V_rr_idx_0 /
      rtP.Vth + rtP.Vth);
  }

  localDW->kx_idx_1 = fabs(localDW->kx_idx_1) / RE;
  localDW->omega_fake[0] = localDW->kx_idx_1;
  localDW->omega_fake[1] = fabs(localDW->V_flt_idx_1) / RE;
  localDW->V_fl_tmp = fabs(localDW->V_rl_idx_0_tmp) / RE;
  localDW->omega_fake[2] = localDW->V_fl_tmp;
  localDW->omega_fake[3] = fabs(localDW->V_rr_idx_0) / RE;
  if (maximum(localDW->omega_fake) - minimum_b(localDW->omega_fake) <
      rtP.max_delta_omega) {
    localDW->V_rr_idx_0 = (((localDW->kx_idx_1 + localDW->omega_fake[1]) +
      localDW->V_fl_tmp) + localDW->omega_fake[3]) / 4.0;
    localDW->omega_fake[0] = 1.01 * localDW->V_rr_idx_0;
    localDW->omega_fake[1] = 1.01 * localDW->V_rr_idx_0;
    localDW->omega_fake[2] = localDW->V_rr_idx_0;
    localDW->omega_fake[3] = localDW->V_rr_idx_0;
  }

  localDW->kx_idx_0 = A1 * localDW->FZ[0] + A2;
  localDW->RPM_index[0] = fabs(localDW->FZ[0]);
  localDW->V_rl_idx_0_tmp_tmp = localDW->FZ[0] * localDW->FZ[0];
  localDW->relative_PL = (localDW->V_rl_idx_0_tmp_tmp * B1 + B2 * localDW->FZ[0])
    + B3;
  localDW->kx_idx_1 = A1 * localDW->FZ[1] + A2;
  localDW->RPM_index[1] = fabs(localDW->FZ[1]);
  localDW->V_rl_idx_0_tmp = localDW->FZ[1] * localDW->FZ[1];
  localDW->V_fl_tmp = (localDW->V_rl_idx_0_tmp * B1 + B2 * localDW->FZ[1]) + B3;
  localDW->kx_idx_2 = A1 * localDW->FZ[2] + A2;
  localDW->RPM_index[2] = fabs(localDW->FZ[2]);
  localDW->V_flt_idx_1 = localDW->FZ[2] * localDW->FZ[2];
  localDW->mu_x = (localDW->V_flt_idx_1 * B1 + B2 * localDW->FZ[2]) + B3;
  localDW->kx_idx_3 = A1 * localDW->beq + A2;
  localDW->RPM_index[3] = fabs(localDW->beq);
  localDW->V_rr_idx_0 = localDW->beq * localDW->beq;
  localDW->Sum_j = (localDW->V_rr_idx_0 * B1 + B2 * localDW->beq) + B3;
  pchip(&FZ_C[0], &C_param[0], localDW->RPM_index, localDW->C, localDW);
  localDW->V_rl_idx_0_tmp_tmp = ((localDW->V_rl_idx_0_tmp_tmp * C2 + C1 *
    rt_powd_snf(localDW->FZ[0], 3.0)) + C3 * localDW->FZ[0]) + C4;
  localDW->Fx_max[0] = localDW->relative_PL * localDW->FZ[0];
  localDW->V_rl_idx_0_tmp = ((localDW->V_rl_idx_0_tmp * C2 + C1 * rt_powd_snf
    (localDW->FZ[1], 3.0)) + C3 * localDW->FZ[1]) + C4;
  localDW->Fx_max[1] = localDW->V_fl_tmp * localDW->FZ[1];
  localDW->mu_y = ((localDW->V_flt_idx_1 * C2 + C1 * rt_powd_snf(localDW->FZ[2],
    3.0)) + C3 * localDW->FZ[2]) + C4;
  localDW->Fx_max[2] = localDW->mu_x * localDW->FZ[2];
  localDW->mu_y_c = ((localDW->V_rr_idx_0 * C2 + C1 * rt_powd_snf(localDW->beq,
    3.0)) + C3 * localDW->beq) + C4;
  localDW->V_flt_idx_1 = localDW->Sum_j * localDW->beq;
  localDW->maxval_idx_3 = localDW->kx_idx_3 * localDW->left_steering_angle /
    localDW->V_flt_idx_1;
  localDW->rack_displacement = tan(localDW->rack_displacement);
  localDW->a_bar_idx_0 = localDW->C[0] * localDW->rack_displacement /
    (localDW->V_rl_idx_0_tmp_tmp * localDW->FZ[0]);
  localDW->C[0] = localDW->C[0] * localDW->relative_PL / (localDW->kx_idx_0 *
    localDW->V_rl_idx_0_tmp_tmp);
  localDW->V_rr_idx_0 = 1.0;
  localDW->a_bar_idx_1_tmp = tan(localDW->SA[1]);
  localDW->a_bar_idx_1 = localDW->C[1] * localDW->a_bar_idx_1_tmp /
    (localDW->V_rl_idx_0_tmp * localDW->FZ[1]);
  localDW->C[1] = localDW->C[1] * localDW->V_fl_tmp / (localDW->kx_idx_1 *
    localDW->V_rl_idx_0_tmp);
  localDW->V_rl_idx_0_tmp = 1.0;
  localDW->a_bar_idx_2_tmp = tan(localDW->SA[2]);
  localDW->a_bar_idx_2 = localDW->C[2] * localDW->a_bar_idx_2_tmp /
    (localDW->mu_y * localDW->FZ[2]);
  localDW->C[2] = localDW->C[2] * localDW->mu_x / (localDW->kx_idx_2 *
    localDW->mu_y);
  localDW->mu_y = 1.0;
  localDW->a_bar_tmp = tan(localDW->SA[3]);
  localDW->a_bar = localDW->C[3] * localDW->a_bar_tmp / (localDW->mu_y_c *
    localDW->beq);
  localDW->mu_y_tmp = localDW->a_bar * localDW->a_bar;
  localDW->V_rl_idx_0_tmp_tmp = sqrt(localDW->maxval_idx_3 *
    localDW->maxval_idx_3 + localDW->mu_y_tmp);
  localDW->C[3] = localDW->C[3] * localDW->Sum_j / (localDW->kx_idx_3 *
    localDW->mu_y_c);
  localDW->maxval_idx_3 = 1.0;
  if (fabs(localDW->V_rl_idx_0_tmp_tmp) <= 6.2831853071795862) {
    localDW->V_rl_idx_0_tmp_tmp = cos(rtP.k_limit / 2.0) * 0.5;
    localDW->V_rr_idx_0 = (localDW->C[0] + 1.0) * 0.5 - (1.0 - localDW->C[0]) *
      localDW->V_rl_idx_0_tmp_tmp;
    localDW->V_rl_idx_0_tmp = (localDW->C[1] + 1.0) * 0.5 - (1.0 - localDW->C[1])
      * localDW->V_rl_idx_0_tmp_tmp;
    localDW->mu_y = (localDW->C[2] + 1.0) * 0.5 - (1.0 - localDW->C[2]) *
      localDW->V_rl_idx_0_tmp_tmp;
    localDW->maxval_idx_3 = (localDW->C[3] + 1.0) * 0.5 - (1.0 - localDW->C[3]) *
      localDW->V_rl_idx_0_tmp_tmp;
  }

  localDW->V_rl_idx_0_tmp_tmp = 0.0;
  if (rtP.k_limit > fabs(localDW->a_bar_idx_0)) {
    localDW->V_rl_idx_0_tmp_tmp = sqrt(rtP.k_limit * rtP.k_limit -
      localDW->a_bar_idx_0 * localDW->a_bar_idx_0);
  }

  localDW->V_rl_idx_0_tmp_tmp = localDW->V_rl_idx_0_tmp_tmp *
    localDW->relative_PL * localDW->FZ[0] / localDW->kx_idx_0;
  localDW->kx_idx_0 = exp(b * rtP.k_limit) * a + exp(d * rtP.k_limit) * c;
  localDW->Fx_max[0] = localDW->kx_idx_0 * localDW->Fx_max[0] *
    localDW->V_rl_idx_0_tmp_tmp / sqrt(localDW->V_rr_idx_0 * localDW->V_rr_idx_0
    * (localDW->rack_displacement * localDW->rack_displacement) +
    localDW->V_rl_idx_0_tmp_tmp * localDW->V_rl_idx_0_tmp_tmp) * rtP.mu_factor;
  localDW->V_rl_idx_0_tmp_tmp = 0.0;
  if (rtP.k_limit > fabs(localDW->a_bar_idx_1)) {
    localDW->V_rl_idx_0_tmp_tmp = sqrt(rtP.k_limit * rtP.k_limit -
      localDW->a_bar_idx_1 * localDW->a_bar_idx_1);
  }

  localDW->V_rl_idx_0_tmp_tmp = localDW->V_rl_idx_0_tmp_tmp * localDW->V_fl_tmp *
    localDW->FZ[1] / localDW->kx_idx_1;
  localDW->Fx_max[1] = localDW->kx_idx_0 * localDW->Fx_max[1] *
    localDW->V_rl_idx_0_tmp_tmp / sqrt(localDW->V_rl_idx_0_tmp *
    localDW->V_rl_idx_0_tmp * (localDW->a_bar_idx_1_tmp *
    localDW->a_bar_idx_1_tmp) + localDW->V_rl_idx_0_tmp_tmp *
    localDW->V_rl_idx_0_tmp_tmp) * rtP.mu_factor;
  localDW->V_rl_idx_0_tmp_tmp = 0.0;
  if (rtP.k_limit > fabs(localDW->a_bar_idx_2)) {
    localDW->V_rl_idx_0_tmp_tmp = sqrt(rtP.k_limit * rtP.k_limit -
      localDW->a_bar_idx_2 * localDW->a_bar_idx_2);
  }

  localDW->V_rl_idx_0_tmp_tmp = localDW->V_rl_idx_0_tmp_tmp * localDW->mu_x *
    localDW->FZ[2] / localDW->kx_idx_2;
  localDW->Fx_max[2] = localDW->kx_idx_0 * localDW->Fx_max[2] *
    localDW->V_rl_idx_0_tmp_tmp / sqrt(localDW->mu_y * localDW->mu_y *
    (localDW->a_bar_idx_2_tmp * localDW->a_bar_idx_2_tmp) +
    localDW->V_rl_idx_0_tmp_tmp * localDW->V_rl_idx_0_tmp_tmp) * rtP.mu_factor;
  localDW->V_rl_idx_0_tmp_tmp = 0.0;
  if (rtP.k_limit > fabs(localDW->a_bar)) {
    localDW->V_rl_idx_0_tmp_tmp = sqrt(rtP.k_limit * rtP.k_limit -
      localDW->mu_y_tmp);
  }

  localDW->V_rl_idx_0_tmp_tmp = localDW->V_rl_idx_0_tmp_tmp * localDW->Sum_j *
    localDW->beq / localDW->kx_idx_3;
  localDW->Fx_max[3] = localDW->kx_idx_0 * localDW->V_flt_idx_1 *
    localDW->V_rl_idx_0_tmp_tmp / sqrt(localDW->maxval_idx_3 *
    localDW->maxval_idx_3 * (localDW->a_bar_tmp * localDW->a_bar_tmp) +
    localDW->V_rl_idx_0_tmp_tmp * localDW->V_rl_idx_0_tmp_tmp) * rtP.mu_factor;
  if ((fabs(localDW->SA_rl) > rtP.SL_limit) + (fabs(localDW->left_steering_angle)
       > rtP.SL_limit) > 0) {
    localDW->Fx_max[2] = rtP.Fx_limit[0];
    localDW->Fx_max[3] = rtP.Fx_limit[1];
  }

  if ((fabs(localDW->SL[0]) > rtP.SL_limit) + (fabs(localDW->SA_fr) >
       rtP.SL_limit) > 0) {
    localDW->Fx_max[0] = rtP.Fx_limit[0];
    localDW->Fx_max[1] = rtP.Fx_limit[1];
  }

  /* End of MATLAB Function: '<S3>/Tire Model' */

  /* MATLAB Function: '<S3>/Reference Generation' incorporates:
   *  MATLAB Function: '<S2>/Optimization '
   */
  localDW->kx_idx_2 = 3.3121686421112381E-170;
  if (localDW->d_k > 3.3121686421112381E-170) {
    localDW->left_steering_angle = 1.0;
    localDW->kx_idx_2 = localDW->d_k;
  } else {
    localDW->a_bar_idx_1 = localDW->d_k / 3.3121686421112381E-170;
    localDW->left_steering_angle = localDW->a_bar_idx_1 * localDW->a_bar_idx_1;
  }

  localDW->mu_x = fabs(rtu_TVS_Information[1]);
  if (localDW->mu_x > localDW->kx_idx_2) {
    localDW->a_bar_idx_1 = localDW->kx_idx_2 / localDW->mu_x;
    localDW->left_steering_angle = localDW->left_steering_angle *
      localDW->a_bar_idx_1 * localDW->a_bar_idx_1 + 1.0;
    localDW->kx_idx_2 = localDW->mu_x;
  } else {
    localDW->a_bar_idx_1 = localDW->mu_x / localDW->kx_idx_2;
    localDW->left_steering_angle += localDW->a_bar_idx_1 * localDW->a_bar_idx_1;
  }

  localDW->left_steering_angle = localDW->kx_idx_2 * sqrt
    (localDW->left_steering_angle);
  localDW->rack_displacement = localDW->left_steering_angle * localDW->steering
    [0] / (l[0] + l[1]);
  if (localDW->rack_displacement == 0.0) {
    localDW->kx_idx_2 = 0.0;
  } else {
    if (!rtIsNaN(localDW->rack_displacement)) {
      if (localDW->rack_displacement < 0.0) {
        localDW->rack_displacement = -1.0;
      } else {
        localDW->rack_displacement = (localDW->rack_displacement > 0.0);
      }
    }

    localDW->kx_idx_2 = localDW->rack_displacement *
      localDW->left_steering_angle / rtu_TVS_Information_ds;
  }

  /* Sum: '<S9>/Sum1' */
  localDW->left_steering_angle = localDW->kx_idx_2 - rtu_TVS_Information_h[2];

  /* UnitDelay: '<S3>/Unit Delay4' */
  rtb_UnitDelay4 = localDW->UnitDelay4_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  UnitDelay: '<S3>/Unit Delay4'
   */
  if (localDW->UnitDelay4_DSTATE || (localDW->DiscreteTimeIntegrator_PrevRese !=
       0)) {
    localDW->DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  Abs: '<S9>/Abs'
   *  Constant: '<S9>/Constant'
   *  Gain: '<S9>/I'
   *  Product: '<S9>/Product'
   *  Product: '<S9>/Product3'
   *  RelationalOperator: '<S9>/Less Than'
   *  UnitDelay: '<S3>/Unit Delay'
   */
  localDW->rack_displacement = (real_T)(rtP.yaw_deadband > fabs
    (localDW->left_steering_angle)) * (rtP.I * localDW->left_steering_angle) *
    localDW->UnitDelay_DSTATE * 0.015 + localDW->DiscreteTimeIntegrator_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' incorporates:
   *  UnitDelay: '<S3>/Unit Delay4'
   */
  if (localDW->UnitDelay4_DSTATE && (localDW->DiscreteTimeIntegrator2_PrevRes <=
       0)) {
    localDW->DiscreteTimeIntegrator2_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' incorporates:
   *  Gain: '<S9>/T'
   *  Product: '<S9>/Product1'
   *  UnitDelay: '<S3>/Unit Delay'
   *  UnitDelay: '<S3>/Unit Delay1'
   */
  localDW->SA_rl = rtP.T * localDW->UnitDelay1_DSTATE *
    localDW->UnitDelay_DSTATE * 0.015 + localDW->DiscreteTimeIntegrator2_DSTATE;

  /* Sum: '<S9>/Sum7' incorporates:
   *  Gain: '<S9>/P'
   *  Sum: '<S9>/Sum'
   */
  localDW->SA_fr = (rtP.P * localDW->left_steering_angle +
                    localDW->rack_displacement) + localDW->SA_rl;

  /* Sum: '<S13>/Sum' incorporates:
   *  UnitDelay: '<S13>/Unit Delay1'0
   */
  localDW->left_steering_angle = localDW->SA_fr + localDW->UnitDelay1_DSTATE_e;

  /* Product: '<S13>/Product1' */
  localDW->kx_idx_3 = localDW->left_steering_angle * 0.0895161414022789;

  /* Sum: '<S14>/Sum' incorporates:
   *  UnitDelay: '<S14>/Unit Delay1'
   */
  localDW->Sum_j = localDW->kx_idx_3 + localDW->UnitDelay1_DSTATE_h;

  /* Product: '<S14>/Product1' */
  localDW->kx_idx_0 = localDW->Sum_j * 0.070255736826063453;

  /* MATLAB Function: '<S3>/Brake Model' */
  if (rtIsNaN(rtu_TVS_Information[0])) {
    localDW->relative_PL = rtu_TVS_Information[0];
  } else if (rtu_TVS_Information[0] < 0.0) {
    localDW->relative_PL = -1.0;
  } else {
    localDW->relative_PL = (rtu_TVS_Information[0] > 0.0);
  }

  localDW->x_m[1] = -brakecoeff[0] * localDW->relative_PL *
    rtu_TVS_Information_e[0] / gr[0];
  localDW->V_rr_idx_0 = -25.0;
  if (rtIsNaN(localDW->x_m[1])) {
    p = false;
  } else {
    p = (localDW->x_m[1] > -25.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = localDW->x_m[1];
  }

  localDW->FZ[0] = 0.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 0.0);
  }

  if (p) {
    localDW->FZ[0] = localDW->V_rr_idx_0;
  }

  /* MATLAB Function: '<S3>/Constraint Generation' */
  localDW->kx_idx_1 = localDW->omega_fake[0] * gr[0];
  localDW->omega_fake[0] = localDW->kx_idx_1;

  /* MATLAB Function: '<S3>/Brake Model' */
  localDW->x_m[3] = -brakecoeff[1] * localDW->relative_PL *
    rtu_TVS_Information_e[1] / gr[1];
  localDW->V_rr_idx_0 = -25.0;
  if (rtIsNaN(localDW->x_m[3])) {
    p = false;
  } else {
    p = (localDW->x_m[3] > -25.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = localDW->x_m[3];
  }

  localDW->FZ[1] = 0.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 0.0);
  }

  if (p) {
    localDW->FZ[1] = localDW->V_rr_idx_0;
  }

  /* MATLAB Function: '<S3>/Constraint Generation' */
  localDW->V_fl_tmp = localDW->omega_fake[1] * gr[1];
  localDW->omega_fake[1] = localDW->V_fl_tmp;

  /* MATLAB Function: '<S3>/Brake Model' */
  localDW->x_m[5] = -brakecoeff[2] * localDW->relative_PL *
    rtu_TVS_Information_e[2] / gr[2];
  localDW->V_rr_idx_0 = -25.0;
  if (rtIsNaN(localDW->x_m[5])) {
    p = false;
  } else {
    p = (localDW->x_m[5] > -25.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = localDW->x_m[5];
  }

  localDW->FZ[2] = 0.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 0.0);
  }

  if (p) {
    localDW->FZ[2] = localDW->V_rr_idx_0;
  }

  /* MATLAB Function: '<S3>/Constraint Generation' */
  localDW->maxval_idx_3 = localDW->omega_fake[2] * gr[2];
  localDW->omega_fake[2] = localDW->maxval_idx_3;

  /* MATLAB Function: '<S3>/Brake Model' */
  localDW->x_m[7] = -brakecoeff[3] * localDW->relative_PL *
    rtu_TVS_Information_e[3] / gr[3];
  localDW->V_rr_idx_0 = -25.0;
  if (rtIsNaN(localDW->x_m[7])) {
    p = false;
  } else {
    p = (localDW->x_m[7] > -25.0);
  }

  if (p) {
    localDW->V_rr_idx_0 = localDW->x_m[7];
  }

  localDW->FZ[3] = 0.0;
  if (rtIsNaN(localDW->V_rr_idx_0)) {
    p = false;
  } else {
    p = (localDW->V_rr_idx_0 < 0.0);
  }

  if (p) {
    localDW->FZ[3] = localDW->V_rr_idx_0;
  }

  /* MATLAB Function: '<S3>/Constraint Generation' incorporates:
   *  UnitDelay: '<S3>/Unit Delay2'
   *  UnitDelay: '<S3>/Unit Delay3'
   */
  localDW->a_bar_idx_1_tmp = localDW->omega_fake[3] * gr[3];
  localDW->omega_fake[3] = localDW->a_bar_idx_1_tmp;
  localDW->V_rr_idx_0 = s[0] * cos(localDW->steering[1]) + l[0] * sin
    (localDW->steering[1]);
  localDW->V_rl_idx_0_tmp = -s[0] * cos(localDW->steering[2]) + l[0] * sin
    (localDW->steering[2]);
  if (rtu_TVS_Information_o <= 0.0) {
    if ((rtu_TVS_Information_n[1] >= absolute_battery_power_limits[1]) ||
        rtIsNaN(absolute_battery_power_limits[1])) {
      localDW->relative_PL = rtu_TVS_Information_n[1];
    } else {
      localDW->relative_PL = absolute_battery_power_limits[1];
    }

    if ((localDW->relative_PL >= 0.0) || rtIsNaN(localDW->relative_PL)) {
      localDW->relative_PL = 0.0;
    }
  } else {
    localDW->steering[0] = rtu_TVS_Information_n[0];
    localDW->steering[1] = absolute_battery_power_limits[0];
    localDW->steering[2] = ((motor_enable[0] * 28.0 * localDW->kx_idx_1 +
      motor_enable[1] * 28.0 * localDW->V_fl_tmp) + motor_enable[2] * 28.0 *
      localDW->maxval_idx_3) + motor_enable[3] * 28.0 * localDW->a_bar_idx_1_tmp;
    if (!rtIsNaN(rtu_TVS_Information_n[0])) {
      localDW->idx = 1;
    } else {
      localDW->idx = 0;
      localDW->j = 2;
      exitg1 = false;
      while ((!exitg1) && (localDW->j < 4)) {
        if (!rtIsNaN(localDW->steering[localDW->j - 1])) {
          localDW->idx = localDW->j;
          exitg1 = true;
        } else {
          localDW->j++;
        }
      }
    }

    if (localDW->idx == 0) {
      localDW->relative_PL = rtu_TVS_Information_n[0];
    } else {
      localDW->relative_PL = localDW->steering[localDW->idx - 1];
      while (localDW->idx + 1 <= 3) {
        if (localDW->relative_PL > localDW->steering[localDW->idx]) {
          localDW->relative_PL = localDW->steering[localDW->idx];
        }

        localDW->idx++;
      }
    }

    if ((localDW->relative_PL <= 0.0) || rtIsNaN(localDW->relative_PL)) {
      localDW->relative_PL = 0.0;
    }
  }

  for (localDW->j = 0; localDW->j < 4; localDW->j++) {
    localDW->V_rl_idx_0_tmp_tmp = localDW->omega_fake[localDW->j];
    localDW->beq = localDW->FZ[localDW->j];
    localDW->V_flt_idx_1 = localDW->Fx_max[localDW->j] * RE / gr[localDW->j];
    localDW->SL[localDW->j] = localDW->V_flt_idx_1 - localDW->beq;
    localDW->RPM_index[localDW->j] = localDW->V_rl_idx_0_tmp_tmp /
      rpm_index_calibration;
    localDW->RPM_index[localDW->j] = ceil(localDW->RPM_index[localDW->j]);
    localDW->RPM_index[localDW->j] += (real_T)(localDW->V_rl_idx_0_tmp_tmp ==
      0.0);
    for (localDW->idx = 0; localDW->idx < 251; localDW->idx++) {
      localDW->c_x[localDW->idx + 251 * localDW->j] = power_loss_grid[((int32_T)
        localDW->RPM_index[localDW->j] - 1) * 251 + localDW->idx] -
        power_loss_limit[localDW->j];
    }

    localDW->Fx_max[localDW->j] = -localDW->V_flt_idx_1 - localDW->beq;
  }

  for (localDW->j = 0; localDW->j < 1004; localDW->j++) {
    localDW->varargin_1[localDW->j] = fabs(localDW->c_x[localDW->j]);
  }

  minimum(localDW->varargin_1, localDW->SA, localDW->iindx, localDW);
   for (localDW->idx = 0; localDW->idx < 4; localDW->idx++) {
     for (localDW->j = 0; localDW->j < 251; localDW->j++) {
       localDW->c_x[localDW->j + 251 * localDW->idx] = power_loss_grid[((int32_T)
         (localDW->RPM_index[localDW->idx] + 1.0) - 1) * 251 + localDW->j] -
         power_loss_limit[localDW->idx];
     }
   }

  for (localDW->j = 0; localDW->j < 1004; localDW->j++) {
    localDW->varargin_1[localDW->j] = fabs(localDW->c_x[localDW->j]);
  }

  minimum(localDW->varargin_1, localDW->SA, localDW->b_iindx, localDW);
  p = (localDW->relative_PL <= 0.0);
  localDW->V_rl_idx_0_tmp_tmp = torque_sweep[localDW->iindx[0] - 1];
  localDW->V_flt_idx_1 = (localDW->RPM_index[0] - 1.0) * rpm_index_calibration;
  localDW->FZ[0] = (localDW->V_rl_idx_0_tmp_tmp - torque_sweep[localDW->b_iindx
                    [0] - 1]) * (localDW->kx_idx_1 - localDW->V_flt_idx_1);
  localDW->C[0] = localDW->V_rl_idx_0_tmp_tmp;
  localDW->RPM_index[0] = localDW->RPM_index[0] * rpm_index_calibration -
    localDW->V_flt_idx_1;
  localDW->V_rl_idx_0_tmp_tmp = torque_sweep[localDW->iindx[1] - 1];
  localDW->V_flt_idx_1 = (localDW->RPM_index[1] - 1.0) * rpm_index_calibration;
  localDW->FZ[1] = (localDW->V_rl_idx_0_tmp_tmp - torque_sweep[localDW->b_iindx
                    [1] - 1]) * (localDW->V_fl_tmp - localDW->V_flt_idx_1);
  localDW->C[1] = localDW->V_rl_idx_0_tmp_tmp;
  localDW->RPM_index[1] = localDW->RPM_index[1] * rpm_index_calibration -
    localDW->V_flt_idx_1;
  localDW->V_rl_idx_0_tmp_tmp = torque_sweep[localDW->iindx[2] - 1];
  localDW->V_flt_idx_1 = (localDW->RPM_index[2] - 1.0) * rpm_index_calibration;
  localDW->FZ[2] = (localDW->V_rl_idx_0_tmp_tmp - torque_sweep[localDW->b_iindx
                    [2] - 1]) * (localDW->maxval_idx_3 - localDW->V_flt_idx_1);
  localDW->C[2] = localDW->V_rl_idx_0_tmp_tmp;
  localDW->RPM_index[2] = localDW->RPM_index[2] * rpm_index_calibration -
    localDW->V_flt_idx_1;
  localDW->V_rl_idx_0_tmp_tmp = torque_sweep[localDW->iindx[3] - 1];
  localDW->V_flt_idx_1 = (localDW->RPM_index[3] - 1.0) * rpm_index_calibration;
  localDW->RPM_index[3] = localDW->RPM_index[3] * rpm_index_calibration -
    localDW->V_flt_idx_1;
  xgeqp3(localDW->RPM_index, &localDW->beq, &localDW->j, localDW);
  localDW->idx = 0;
  localDW->a_bar_idx_1 = fabs(localDW->RPM_index[0]);
  if (!(localDW->a_bar_idx_1 <= 8.8817841970012523E-15 * localDW->a_bar_idx_1))
  {
    localDW->idx = 1;
  }

  localDW->a_bar_idx_1 = 0.0;
  if (localDW->beq != 0.0) {
    localDW->V_flt_idx_1 = ((localDW->V_rl_idx_0_tmp_tmp - torque_sweep
      [localDW->b_iindx[3] - 1]) * (localDW->a_bar_idx_1_tmp -
      localDW->V_flt_idx_1) * localDW->RPM_index[3] + ((localDW->RPM_index[1] *
      localDW->FZ[1] + localDW->FZ[0]) + localDW->RPM_index[2] * localDW->FZ[2]))
      * localDW->beq;
    if (localDW->V_flt_idx_1 != 0.0) {
      localDW->FZ[0] -= localDW->V_flt_idx_1;
    }
  }

  localDW->j = 0;
  while (localDW->j <= localDW->idx - 1) {
    localDW->a_bar_idx_1 = localDW->FZ[0];
    localDW->j = 1;
  }

  while (localDW->idx > 0) {
    localDW->a_bar_idx_1 /= localDW->RPM_index[0];
    localDW->idx = 0;
  }

  interp1(&max_rpm[0], &max_torque[0], localDW->omega_fake, localDW->FZ, localDW);
  localDW->x_m[0] = localDW->FZ[0];
  localDW->x_m[1] = (real_T)!((real_T)(rtu_TVS_Information_d[0] < max_motor_temp)
    * motor_enable[0] * (real_T)p != 0.0) * (localDW->C[0] -
    localDW->a_bar_idx_1);
  localDW->x_m[2] = localDW->FZ[1];
  localDW->x_m[3] = (real_T)!((real_T)(rtu_TVS_Information_d[1] < max_motor_temp)
    * motor_enable[1] * (real_T)p != 0.0) * (localDW->C[1] -
    localDW->a_bar_idx_1);
  localDW->x_m[4] = localDW->FZ[2];
  localDW->x_m[5] = (real_T)!((real_T)(rtu_TVS_Information_d[2] < max_motor_temp)
    * motor_enable[2] * (real_T)p != 0.0) * (localDW->C[2] -
    localDW->a_bar_idx_1);
  localDW->x_m[6] = localDW->FZ[3];
  localDW->x_m[7] = (real_T)!((real_T)(rtu_TVS_Information_d[3] < max_motor_temp)
    * motor_enable[3] * (real_T)p != 0.0) * (localDW->V_rl_idx_0_tmp_tmp -
    localDW->a_bar_idx_1);
  for (localDW->j = 0; localDW->j < 4; localDW->j++) {
    localDW->idx = localDW->j << 1;
    localDW->beq = localDW->x_m[localDW->idx];
    localDW->V_rl_idx_0_tmp_tmp = localDW->x_m[localDW->idx + 1];
    if (rtIsNaN(localDW->V_rl_idx_0_tmp_tmp)) {
      p = false;
    } else if (rtIsNaN(localDW->beq)) {
      p = true;
    } else {
      p = (localDW->beq > localDW->V_rl_idx_0_tmp_tmp);
    }

    if (p) {
      localDW->beq = localDW->V_rl_idx_0_tmp_tmp;
    }

    if ((localDW->beq <= 0.0) || rtIsNaN(localDW->beq)) {
      localDW->beq = 0.0;
    }

    localDW->e_x[3 * localDW->j] = localDW->SL[localDW->j];
    localDW->idx = 3 * localDW->j + 1;
    localDW->e_x[localDW->idx] = localDW->beq;
    e_x_tmp = 3 * localDW->j + 2;
    localDW->e_x[e_x_tmp] = abs_max_torque[localDW->j];
    localDW->V_flt_idx_1 = localDW->e_x[3 * localDW->j];
    localDW->V_rl_idx_0_tmp_tmp = localDW->e_x[localDW->idx];
    if (rtIsNaN(localDW->V_rl_idx_0_tmp_tmp)) {
      p = false;
    } else if (rtIsNaN(localDW->V_flt_idx_1)) {
      p = true;
    } else {
      p = (localDW->V_flt_idx_1 > localDW->V_rl_idx_0_tmp_tmp);
    }

    if (p) {
      localDW->V_flt_idx_1 = localDW->V_rl_idx_0_tmp_tmp;
    }

    localDW->V_rl_idx_0_tmp_tmp = localDW->e_x[e_x_tmp];
    if (rtIsNaN(localDW->V_rl_idx_0_tmp_tmp)) {
      p = false;
    } else if (rtIsNaN(localDW->V_flt_idx_1)) {
      p = true;
    } else {
      p = (localDW->V_flt_idx_1 > localDW->V_rl_idx_0_tmp_tmp);
    }

    if (p) {
      localDW->V_flt_idx_1 = localDW->V_rl_idx_0_tmp_tmp;
    }

    if ((dTx >= localDW->V_flt_idx_1) || rtIsNaN(localDW->V_flt_idx_1)) {
      localDW->V_flt_idx_1 = dTx;
    }

    localDW->ub[localDW->j] = (real32_T)localDW->V_flt_idx_1;
    localDW->e_x[3 * localDW->j] = localDW->Fx_max[localDW->j];
    localDW->e_x[localDW->idx] = -localDW->beq;
    localDW->e_x[e_x_tmp] = -abs_max_torque[localDW->j];
    localDW->V_flt_idx_1 = localDW->e_x[3 * localDW->j];
    if (rtIsNaN(localDW->e_x[localDW->idx])) {
      p = false;
    } else if (rtIsNaN(localDW->V_flt_idx_1)) {
      p = true;
    } else {
      p = (localDW->V_flt_idx_1 < localDW->e_x[localDW->idx]);
    }

    if (p) {
      localDW->V_flt_idx_1 = localDW->e_x[localDW->idx];
    }

    if (rtIsNaN(localDW->e_x[e_x_tmp])) {
      p = false;
    } else if (rtIsNaN(localDW->V_flt_idx_1)) {
      p = true;
    } else {
      p = (localDW->V_flt_idx_1 < localDW->e_x[e_x_tmp]);
    }

    if (p) {
      localDW->V_flt_idx_1 = localDW->e_x[e_x_tmp];
    }

    if ((-dTx <= localDW->V_flt_idx_1) || rtIsNaN(localDW->V_flt_idx_1)) {
      localDW->beq = -dTx;
    } else {
      localDW->beq = localDW->V_flt_idx_1;
    }

    localDW->lb[localDW->j] = (real32_T)localDW->beq;
  }

  if (rtu_TVS_Information[0] < min_velocity_regen) {
    localDW->lb[0] = (real32_T)-dTx;
    localDW->lb[1] = (real32_T)-dTx;
    localDW->lb[2] = (real32_T)-dTx;
    localDW->lb[3] = (real32_T)-dTx;
  }

  localDW->rtb_A_idx_0 = (real32_T)(localDW->kx_idx_1 / motor_efficiency[0]);
  rtb_Aeq_idx_0 = (real32_T)localDW->V_rr_idx_0 * T2F[0] * (real32_T)gr[0] /
    (real32_T)J_z;
  localDW->rtb_A_idx_1 = (real32_T)(localDW->V_fl_tmp / motor_efficiency[1]);
  rtb_Aeq_idx_1 = (real32_T)localDW->V_rl_idx_0_tmp * T2F[1] * (real32_T)gr[1] /
    (real32_T)J_z;
  rtb_A_idx_2 = (real32_T)(localDW->maxval_idx_3 / motor_efficiency[2]);
  rtb_Aeq_idx_2 = (real32_T)s[1] * T2F[2] * (real32_T)gr[2] / (real32_T)J_z;
  rtb_A_idx_3 = (real32_T)(localDW->a_bar_idx_1_tmp / motor_efficiency[3]);
  rtb_Aeq_idx_3 = (real32_T)-s[1] * T2F[3] * (real32_T)gr[3] / (real32_T)J_z;
  M_max_idx_0 = (localDW->ub[0] * T2F[0] * (real32_T)gr[0] * (real32_T)
                 localDW->V_rr_idx_0 + localDW->ub[2] * T2F[2] * (real32_T)gr[2]
                 * (real32_T)s[1]) / (real32_T)J_z;
  M_max_idx_1 = (localDW->ub[1] * T2F[1] * (real32_T)gr[1] * (real32_T)
                 localDW->V_rl_idx_0_tmp + localDW->ub[3] * T2F[3] * (real32_T)
                 gr[3] * (real32_T)-s[1]) / (real32_T)J_z;
  if (localDW->kx_idx_0 >= M_max_idx_0) {
    localDW->kx_idx_0 = M_max_idx_0;

    /* Update for UnitDelay: '<S3>/Unit Delay' */
    localDW->UnitDelay_DSTATE = 0.0;
  } else if (localDW->kx_idx_0 <= M_max_idx_1) {
    localDW->kx_idx_0 = M_max_idx_1;

    /* Update for UnitDelay: '<S3>/Unit Delay' */
    localDW->UnitDelay_DSTATE = 0.0;
  } else {
    /* Update for UnitDelay: '<S3>/Unit Delay' */
    localDW->UnitDelay_DSTATE = 1.0;
  }

  localDW->beq = (real_T)((rtu_TVS_Information[0] > rtP.deadband_velocity) &&
    ((localDW->kx_idx_2 != 0.0) && (rtu_TVS_Information_o > 0.0))) *
    localDW->kx_idx_0 + 0.01;
  if (rtIsNaN(localDW->UnitDelay3_DSTATE)) {
    localDW->V_rl_idx_0_tmp_tmp = localDW->UnitDelay3_DSTATE;
  } else if (localDW->UnitDelay3_DSTATE < 0.0) {
    localDW->V_rl_idx_0_tmp_tmp = -1.0;
  } else {
    localDW->V_rl_idx_0_tmp_tmp = (localDW->UnitDelay3_DSTATE > 0.0);
  }

  if (rtIsNaN(localDW->relative_PL)) {
    localDW->V_rr_idx_0 = localDW->relative_PL;
  } else if (localDW->relative_PL < 0.0) {
    localDW->V_rr_idx_0 = -1.0;
  } else {
    localDW->V_rr_idx_0 = (localDW->relative_PL > 0.0);
  }

  if (localDW->V_rl_idx_0_tmp_tmp != localDW->V_rr_idx_0) {
    /* Update for UnitDelay: '<S3>/Unit Delay4' */
    localDW->UnitDelay4_DSTATE = true;
  } else if ((localDW->UnitDelay2_DSTATE[0] < rtP.deadband_velocity) &&
             (rtu_TVS_Information[0] >= rtP.deadband_velocity)) {
    /* Update for UnitDelay: '<S3>/Unit Delay4' */
    localDW->UnitDelay4_DSTATE = true;
  } else {
    if (!rtIsNaN(localDW->kx_idx_0)) {
      if (localDW->kx_idx_0 < 0.0) {
        localDW->kx_idx_0 = -1.0;
      } else {
        localDW->kx_idx_0 = (localDW->kx_idx_0 > 0.0);
      }
    }

    if (!rtIsNaN(localDW->kx_idx_2)) {
      if (localDW->kx_idx_2 < 0.0) {
        localDW->kx_idx_2 = -1.0;
      } else {
        localDW->kx_idx_2 = (localDW->kx_idx_2 > 0.0);
      }
    }

    /* Update for UnitDelay: '<S3>/Unit Delay4' */
    localDW->UnitDelay4_DSTATE = (localDW->kx_idx_0 != localDW->kx_idx_2);
  }

  /* MATLAB Function: '<S2>/Optimization ' incorporates:
   *  MATLAB Function: '<S3>/Constraint Generation'
   */
  localDW->kx_idx_2 = 3.3121686421112381E-170;
  if (localDW->d_k > 3.3121686421112381E-170) {
    localDW->V_flt_idx_1 = 1.0;
    localDW->kx_idx_2 = localDW->d_k;
  } else {
    localDW->a_bar_idx_1 = localDW->d_k / 3.3121686421112381E-170;
    localDW->V_flt_idx_1 = localDW->a_bar_idx_1 * localDW->a_bar_idx_1;
  }

  if (localDW->mu_x > localDW->kx_idx_2) {
    localDW->a_bar_idx_1 = localDW->kx_idx_2 / localDW->mu_x;
    localDW->V_flt_idx_1 = localDW->V_flt_idx_1 * localDW->a_bar_idx_1 *
      localDW->a_bar_idx_1 + 1.0;
    localDW->kx_idx_2 = localDW->mu_x;
  } else {
    localDW->a_bar_idx_1 = localDW->mu_x / localDW->kx_idx_2;
    localDW->V_flt_idx_1 += localDW->a_bar_idx_1 * localDW->a_bar_idx_1;
  }

  localDW->V_flt_idx_1 = localDW->kx_idx_2 * sqrt(localDW->V_flt_idx_1);
  M_max_idx_0 = localDW->lb[0] + 25.0F;
  localDW->lb[0] += 25.0F;
  M_max_idx_1 = localDW->lb[1] + 25.0F;
  localDW->lb[1] += 25.0F;
  rtb_lb = localDW->lb[2] + 25.0F;
  localDW->lb[2] += 25.0F;
 
  if (((!(localDW->V_flt_idx_1 < min_velocity_regen)) ||
       (!(rtu_TVS_Information_o < 0.0))) && (rtu_TVS_Information_o > 0.0)) {
    localDW->Tx21 = 0.0F;
    localDW->Tx22 = 0.0F;
    localDW->Tx23 = 0.0F;
    localDW->Tx24 = 0.0F;
    localDW->j = bigM_func(T2F[0] * (real32_T)gr[0], T2F[1] * (real32_T)gr[1],
      T2F[2] * (real32_T)gr[2], T2F[3] * (real32_T)gr[3], (((25.0F *
      localDW->rtb_A_idx_0 + 25.0F * localDW->rtb_A_idx_1) + 25.0F * rtb_A_idx_2)
      + 25.0F * rtb_A_idx_3) + (real32_T)(rtu_TVS_Information_o *
      localDW->relative_PL), localDW->rtb_A_idx_0, localDW->rtb_A_idx_1,
      rtb_A_idx_2, rtb_A_idx_3, (((25.0F * rtb_Aeq_idx_0 + 25.0F * rtb_Aeq_idx_1)
      + 25.0F * rtb_Aeq_idx_2) + 25.0F * rtb_Aeq_idx_3) + (real32_T)localDW->beq,
      rtb_Aeq_idx_0, rtb_Aeq_idx_1, rtb_Aeq_idx_2, rtb_Aeq_idx_3, M_max_idx_0,
      M_max_idx_1, rtb_lb, localDW->lb[3] + 25.0F, localDW->ub[0] + 25.0F,
      localDW->ub[1] + 25.0F, localDW->ub[2] + 25.0F, localDW->ub[3] + 25.0F,
      &localDW->Tx21, &localDW->Tx22, &localDW->Tx23, &localDW->Tx24, 1.0, 1.0,
      1.0, 1.0, yaw_error_limit);
    if ((localDW->j == 0) || (localDW->j == 1)) {
      localDW->kx_idx_1 = rtu_TVS_Information_o * 25.0 + 25.0;
      localDW->omega_fake[0] = localDW->kx_idx_1;
      localDW->omega_fake[1] = localDW->kx_idx_1;
      localDW->omega_fake[2] = localDW->kx_idx_1;
      localDW->omega_fake[3] = localDW->kx_idx_1;
    } else {
      localDW->omega_fake[0] = localDW->Tx21;
      localDW->omega_fake[1] = localDW->Tx22;
      localDW->omega_fake[2] = localDW->Tx23;
      localDW->omega_fake[3] = localDW->Tx24;
    }
  } else {
    localDW->omega_fake[0] = 25.0;
    localDW->omega_fake[1] = 25.0;
    localDW->omega_fake[2] = 25.0;
    localDW->omega_fake[3] = 25.0;
  }

  localDW->d_k = 2.0 * dTx;
  if ((localDW->lb[0] >= (real32_T)localDW->omega_fake[0]) || rtIsNaNF((real32_T)
       localDW->omega_fake[0])) {
    localDW->rtb_A_idx_0 = localDW->lb[0];
  } else {
    localDW->rtb_A_idx_0 = (real32_T)localDW->omega_fake[0];
  }

  rty_Tx[0] = fabs(localDW->omega_fake[0]);
  if ((localDW->ub[0] + 25.0F <= localDW->rtb_A_idx_0) || rtIsNaNF
      (localDW->rtb_A_idx_0)) {
    localDW->rtb_A_idx_0 = localDW->ub[0] + 25.0F;
  }

  rty_Tx[0] = (localDW->rtb_A_idx_0 - 25.0F) * (real_T)(rty_Tx[0] > localDW->d_k);
  if ((localDW->lb[1] >= (real32_T)localDW->omega_fake[1]) || rtIsNaNF((real32_T)
       localDW->omega_fake[1])) {
    localDW->rtb_A_idx_0 = localDW->lb[1];
  } else {
    localDW->rtb_A_idx_0 = (real32_T)localDW->omega_fake[1];
  }

  rty_Tx[1] = fabs(localDW->omega_fake[1]);
  if ((localDW->ub[1] + 25.0F <= localDW->rtb_A_idx_0) || rtIsNaNF
      (localDW->rtb_A_idx_0)) {
    localDW->rtb_A_idx_0 = localDW->ub[1] + 25.0F;
  }

  rty_Tx[1] = (localDW->rtb_A_idx_0 - 25.0F) * (real_T)(rty_Tx[1] > localDW->d_k);
  if ((localDW->lb[2] >= (real32_T)localDW->omega_fake[2]) || rtIsNaNF((real32_T)
       localDW->omega_fake[2])) {
    localDW->rtb_A_idx_0 = localDW->lb[2];
  } else {
    localDW->rtb_A_idx_0 = (real32_T)localDW->omega_fake[2];
  }

  rty_Tx[2] = fabs(localDW->omega_fake[2]);
  if ((localDW->ub[2] + 25.0F <= localDW->rtb_A_idx_0) || rtIsNaNF
      (localDW->rtb_A_idx_0)) {
    localDW->rtb_A_idx_0 = localDW->ub[2] + 25.0F;
  }

  rty_Tx[2] = (localDW->rtb_A_idx_0 - 25.0F) * (real_T)(rty_Tx[2] > localDW->d_k);
  if ((localDW->lb[3] + 25.0F >= (real32_T)localDW->omega_fake[3]) || rtIsNaNF
      ((real32_T)localDW->omega_fake[3])) {
    localDW->rtb_A_idx_0 = localDW->lb[3] + 25.0F;
  } else {
    localDW->rtb_A_idx_0 = (real32_T)localDW->omega_fake[3];
  }

  rty_Tx[3] = fabs(localDW->omega_fake[3]);
  if ((localDW->ub[3] + 25.0F <= localDW->rtb_A_idx_0) || rtIsNaNF
      (localDW->rtb_A_idx_0)) {
    localDW->rtb_A_idx_0 = localDW->ub[3] + 25.0F;
  }

  rty_Tx[3] = (localDW->rtb_A_idx_0 - 25.0F) * (real_T)(rty_Tx[3] > localDW->d_k);
  if (localDW->V_flt_idx_1 > 4.0) {
    rty_Tx[0] = rtu_TVS_Information_o * 0.0;
    rty_Tx[1] = rtu_TVS_Information_o * 0.0;
    rty_Tx[2] = rtu_TVS_Information_o * 25.0;
    rty_Tx[3] = rtu_TVS_Information_o * 25.0;
  }

  /* RateLimiter: '<S2>/Rate Limiter' */
  localDW->V_rl_idx_0_tmp_tmp = rty_Tx[0] - localDW->PrevY[0];
  if (localDW->V_rl_idx_0_tmp_tmp > 1.875) {
    rty_Tx[0] = localDW->PrevY[0] + 1.875;
  } else if (localDW->V_rl_idx_0_tmp_tmp < -4.5) {
    rty_Tx[0] = localDW->PrevY[0] + -4.5;
  }

  localDW->PrevY[0] = rty_Tx[0];
  localDW->V_rl_idx_0_tmp_tmp = rty_Tx[1] - localDW->PrevY[1];
  if (localDW->V_rl_idx_0_tmp_tmp > 1.875) {
    rty_Tx[1] = localDW->PrevY[1] + 1.875;
  } else if (localDW->V_rl_idx_0_tmp_tmp < -4.5) {
    rty_Tx[1] = localDW->PrevY[1] + -4.5;
  }

  localDW->PrevY[1] = rty_Tx[1];
  localDW->V_rl_idx_0_tmp_tmp = rty_Tx[2] - localDW->PrevY[2];
  if (localDW->V_rl_idx_0_tmp_tmp > 1.875) {
    rty_Tx[2] = localDW->PrevY[2] + 1.875;
  } else if (localDW->V_rl_idx_0_tmp_tmp < -4.5) {
    rty_Tx[2] = localDW->PrevY[2] + -4.5;
  }

  localDW->PrevY[2] = rty_Tx[2];
  localDW->V_rl_idx_0_tmp_tmp = rty_Tx[3] - localDW->PrevY[3];
  if (localDW->V_rl_idx_0_tmp_tmp > 1.875) {
    rty_Tx[3] = localDW->PrevY[3] + 1.875;
  } else if (localDW->V_rl_idx_0_tmp_tmp < -4.5) {
    rty_Tx[3] = localDW->PrevY[3] + -4.5;
  }

  localDW->PrevY[3] = rty_Tx[3];

  /* End of RateLimiter: '<S2>/Rate Limiter' */

  /* Update for UnitDelay: '<S3>/Unit Delay2' */
  localDW->UnitDelay2_DSTATE[0] = rtu_TVS_Information[0];
  localDW->UnitDelay2_DSTATE[1] = rtu_TVS_Information[1];

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_DSTATE = localDW->rack_displacement;
  localDW->DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_UnitDelay4;

  /* Update for UnitDelay: '<S3>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE = localDW->beq;

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   */
  localDW->DiscreteTimeIntegrator2_DSTATE = localDW->SA_rl;
  localDW->DiscreteTimeIntegrator2_PrevRes = (int8_T)rtb_UnitDelay4;

  /* Update for UnitDelay: '<S13>/Unit Delay1' incorporates:
   *  Gain: '<S13>/Gain1'
   *  Product: '<S13>/Product5'
   *  Sum: '<S13>/Sum1'
   *  Sum: '<S13>/Sum2'
   *  UnitDelay: '<S13>/Unit Delay2'
   */
  localDW->UnitDelay1_DSTATE_e = (localDW->left_steering_angle *
    0.62711646319811842 + localDW->SA_fr) * 2.0 + localDW->UnitDelay2_DSTATE_j;

  /* Update for UnitDelay: '<S14>/Unit Delay1' incorporates:
   *  Gain: '<S14>/Gain1'
   *  Product: '<S14>/Product5'
   *  Sum: '<S14>/Sum1'
   *  Sum: '<S14>/Sum2'
   *  UnitDelay: '<S14>/Unit Delay2'
   */
  localDW->UnitDelay1_DSTATE_h = (localDW->Sum_j * 0.49218530320406634 +
    localDW->kx_idx_3) * 2.0 + localDW->UnitDelay2_DSTATE_e;

  /* Update for UnitDelay: '<S3>/Unit Delay3' incorporates:
   *  MATLAB Function: '<S3>/Constraint Generation'
   */
  localDW->UnitDelay3_DSTATE = localDW->relative_PL;

  /* Update for UnitDelay: '<S14>/Unit Delay2' incorporates:
   *  Product: '<S14>/Product2'
   *  Sum: '<S14>/Sum3'
   */
  localDW->UnitDelay2_DSTATE_e = localDW->kx_idx_3 - localDW->Sum_j *
    0.26539355371238654;

  /* Update for UnitDelay: '<S13>/Unit Delay2' incorporates:
   *  Product: '<S13>/Product2'
   *  Sum: '<S13>/Sum3'
   */
  localDW->UnitDelay2_DSTATE_j = localDW->SA_fr - localDW->left_steering_angle *
    0.61229749200535255;
}

/* Model step function */
void TV_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY)
{
  DW *rtDW = rtM->dwork;

  /* Outputs for Atomic SubSystem: '<Root>/Electronics' */

  /* Inport: '<Root>/vel' incorporates:
   *  Inport: '<Root>/FZ'
   *  Inport: '<Root>/ang_vel'
   *  Inport: '<Root>/brake_pressure'
   *  Inport: '<Root>/driver_input'
   *  Inport: '<Root>/motor_temp'
   *  Inport: '<Root>/omega'
   *  Inport: '<Root>/power_limits'
   *  Inport: '<Root>/r_ref'
   *  Inport: '<Root>/steering_angle'
   *  Outport: '<Root>/Tx'
   */
  Electronics_j(rtU->vel, rtU->ang_vel, rtU->driver_input, rtU->brake_pressure,
                rtU->power_limits, rtU->motor_temp, rtU->FZ, rtU->r_ref,
                rtU->steering_angle, rtU->omega, rtY->Tx, &rtDW->Electronics_jy);

  /* End of Outputs for SubSystem: '<Root>/Electronics' */
}

/* Model initialize function */
void Electronics_initialize(RT_MODEL *const rtM)
{
  DW *rtDW = rtM->dwork;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* SystemInitialize for Atomic SubSystem: '<Root>/Electronics' */
  Electronics_Init(&rtDW->Electronics_jy);

  /* End of SystemInitialize for SubSystem: '<Root>/Electronics' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
