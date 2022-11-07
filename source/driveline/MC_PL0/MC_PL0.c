/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MC_PL0.c
 *
 * Code generated for Simulink model 'MC_PL0'.
 *
 * Model version                  : 1.223
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sun Nov  6 18:42:13 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "MC_PL0.h"
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include <stddef.h>
#define NumBitsPerChar                 8U

static real_T look1_binlc(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
static uint32_T plook_evenca(real_T u, real_T bp0, real_T bpSpace, uint32_T
  maxIndex, real_T *fraction);
static real_T intrp2d_la(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[]);
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

static real_T look1_binlc(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Clip'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = 0.0;
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = 1.0;
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

static uint32_T plook_evenca(real_T u, real_T bp0, real_T bpSpace, uint32_T
  maxIndex, real_T *fraction)
{
  real_T fbpIndex;
  real_T invSpc;
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else {
    invSpc = 1.0 / bpSpace;
    fbpIndex = (u - bp0) * invSpc;
    if (fbpIndex < maxIndex) {
      bpIndex = (uint32_T)fbpIndex;
      *fraction = (u - ((real_T)(uint32_T)fbpIndex * bpSpace + bp0)) * invSpc;
    } else {
      bpIndex = maxIndex;
      *fraction = 0.0;
    }
  }

  return bpIndex;
}

static real_T intrp2d_la(const uint32_T bpIndex[], const real_T frac[], const
  real_T table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

/* Model step function */
void MC_PL0_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY)
{
  DW *rtDW = rtM->dwork;
  real_T rtb_RateLimiter;
  real_T rtb_Switch;
  real_T rtb_Switch_c;
  real_T tmp;
  real_T u0;
  int32_T i;
  int32_T iU;
  int32_T i_0;
  int32_T rtb_Abs1_tmp;
  uint8_T rtb_Gain4[4];

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Wx'
   */
  u0 = 0.095492965964253843 * rtU->Wx[0];

  /* Saturate: '<S3>/Saturation1' */
  if (u0 > 106.0) {
    u0 = 106.0;
  } else if (u0 < 1.0) {
    u0 = 1.0;
  }

  /* Rounding: '<S3>/Floor1' incorporates:
   *  Switch: '<S6>/Switch'
   */
  rtDW->Switch_o[0] = floor(u0);

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Wx'
   */
  u0 = 0.095492965964253843 * rtU->Wx[1];

  /* Saturate: '<S3>/Saturation1' */
  if (u0 > 106.0) {
    u0 = 106.0;
  } else if (u0 < 1.0) {
    u0 = 1.0;
  }

  /* Rounding: '<S3>/Floor1' incorporates:
   *  Switch: '<S6>/Switch'
   */
  rtDW->Switch_o[1] = floor(u0);

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Wx'
   */
  u0 = 0.095492965964253843 * rtU->Wx[2];

  /* Saturate: '<S3>/Saturation1' */
  if (u0 > 106.0) {
    u0 = 106.0;
  } else if (u0 < 1.0) {
    u0 = 1.0;
  }

  /* Rounding: '<S3>/Floor1' incorporates:
   *  Switch: '<S6>/Switch'
   */
  rtDW->Switch_o[2] = floor(u0);

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Wx'
   */
  u0 = 0.095492965964253843 * rtU->Wx[3];

  /* Saturate: '<S3>/Saturation1' */
  if (u0 > 106.0) {
    u0 = 106.0;
  } else if (u0 < 1.0) {
    u0 = 1.0;
  }

  /* Rounding: '<S3>/Floor1' incorporates:
   *  Switch: '<S6>/Switch'
   */
  rtDW->Switch_o[3] = floor(u0);

  /* Sum: '<S2>/Sum1' incorporates:
   *  Inport: '<Root>/Tx'
   */
  rtDW->Sum1 = ((rtU->Tx[0] + rtU->Tx[1]) + rtU->Tx[2]) + rtU->Tx[3];

  /* Switch: '<S2>/Switch' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Divide'
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  if (rtDW->Sum1 != 0.0) {
    rtDW->RateLimiter[0] = rtU->Tx[0] / rtDW->Sum1;
    rtDW->RateLimiter[1] = rtU->Tx[1] / rtDW->Sum1;
    rtDW->RateLimiter[2] = rtU->Tx[2] / rtDW->Sum1;
    rtDW->RateLimiter[3] = rtU->Tx[3] / rtDW->Sum1;
  } else {
    rtDW->RateLimiter[0] = 0.25;
    rtDW->RateLimiter[1] = 0.25;
    rtDW->RateLimiter[2] = 0.25;
    rtDW->RateLimiter[3] = 0.25;
  }

  /* End of Switch: '<S2>/Switch' */

  /* Product: '<S2>/MatrixMultiply' incorporates:
   *  Inport: '<Root>/power_limits'
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_RateLimiter = rtDW->RateLimiter[i_0];
    iU = i_0 << 1;
    rtDW->dv[iU] = rtU->power_limits[0] * rtb_RateLimiter;
    rtDW->dv[iU + 1] = rtU->power_limits[1] * rtb_RateLimiter;
  }

  memcpy(&rtDW->Abs[0], &rtDW->dv[0], sizeof(real_T) << 3U);

  /* End of Product: '<S2>/MatrixMultiply' */

  /* Selector: '<S2>/Selector' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[0] = rtDW->Abs[0];

  /* Abs: '<S2>/Abs1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[0] = fabs(rtU->Tx[0]);

  /* Selector: '<S2>/Selector' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[1] = rtDW->Abs[2];

  /* Abs: '<S2>/Abs1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[1] = fabs(rtU->Tx[1]);

  /* Selector: '<S2>/Selector' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[2] = rtDW->Abs[4];

  /* Abs: '<S2>/Abs1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[2] = fabs(rtU->Tx[2]);

  /* Selector: '<S2>/Selector' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[3] = rtDW->Abs[6];

  /* Abs: '<S2>/Abs1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[3] = fabs(rtU->Tx[3]);

  /* Lookup_n-D: '<S2>/2-D Lookup Table' incorporates:
   *  Inport: '<Root>/Wx'
   *  Switch: '<S5>/Switch'
   */
  rtDW->bpIndices[0U] = plook_evenca(rtDW->Switch[0], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions[0U] = rtDW->Sum1;
  rtDW->bpIndices[1U] = plook_evenca(rtU->Wx[0], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions[1U] = rtDW->Sum1;
  rtDW->Switch[0] = intrp2d_la(rtDW->bpIndices, rtDW->fractions,
    rtConstP.pooled4, 161U, rtConstP.pooled9);
  rtDW->bpIndices[0U] = plook_evenca(rtDW->Switch[1], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions[0U] = rtDW->Sum1;
  rtDW->bpIndices[1U] = plook_evenca(rtU->Wx[1], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions[1U] = rtDW->Sum1;
  rtDW->Switch[1] = intrp2d_la(rtDW->bpIndices, rtDW->fractions,
    rtConstP.pooled4, 161U, rtConstP.pooled9);
  rtDW->bpIndices[0U] = plook_evenca(rtDW->Switch[2], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions[0U] = rtDW->Sum1;
  rtDW->bpIndices[1U] = plook_evenca(rtU->Wx[2], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions[1U] = rtDW->Sum1;
  rtDW->Switch[2] = intrp2d_la(rtDW->bpIndices, rtDW->fractions,
    rtConstP.pooled4, 161U, rtConstP.pooled9);
  rtDW->bpIndices[0U] = plook_evenca(rtDW->Switch[3], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions[0U] = rtDW->Sum1;
  rtDW->bpIndices[1U] = plook_evenca(rtU->Wx[3], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions[1U] = rtDW->Sum1;
  rtb_RateLimiter = intrp2d_la(rtDW->bpIndices, rtDW->fractions,
    rtConstP.pooled4, 161U, rtConstP.pooled9);

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[0])) {
    rtDW->Sum1 = rtU->Tx[0];
  } else if (rtU->Tx[0] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[0] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtb_Switch = rtDW->Switch[0] * rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[0])) {
    rtDW->Sum1 = rtU->Tx[0];
  } else if (rtU->Tx[0] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[0] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[0] *= rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[1])) {
    rtDW->Sum1 = rtU->Tx[1];
  } else if (rtU->Tx[1] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[1] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  u0 = rtDW->Switch[1] * rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[1])) {
    rtDW->Sum1 = rtU->Tx[1];
  } else if (rtU->Tx[1] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[1] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[1] *= rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[2])) {
    rtDW->Sum1 = rtU->Tx[2];
  } else if (rtU->Tx[2] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[2] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtb_Switch_c = rtDW->Switch[2] * rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[2])) {
    rtDW->Sum1 = rtU->Tx[2];
  } else if (rtU->Tx[2] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtU->Tx[2] > 0.0);
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[2] *= rtDW->Sum1;

  /* Signum: '<S2>/Sign1' incorporates:
   *  Inport: '<Root>/Tx'
   *  Product: '<S2>/Product2'
   *  Switch: '<S5>/Switch'
   */
  if (rtIsNaN(rtU->Tx[3])) {
    rtDW->Sum1 = rtU->Tx[3];
    tmp = rtU->Tx[3];
  } else {
    if (rtU->Tx[3] < 0.0) {
      rtDW->Sum1 = -1.0;
    } else {
      rtDW->Sum1 = (rtU->Tx[3] > 0.0);
    }

    if (rtU->Tx[3] < 0.0) {
      tmp = -1.0;
    } else {
      tmp = (rtU->Tx[3] > 0.0);
    }
  }

  /* Product: '<S2>/Product2' incorporates:
   *  Switch: '<S5>/Switch'
   */
  rtDW->Switch[3] = rtb_RateLimiter * tmp;

  /* Gain: '<S2>/Gain' */
  for (i_0 = 0; i_0 < 8; i_0++) {
    rtDW->Abs[i_0] = -rtDW->Abs[i_0];
  }

  /* End of Gain: '<S2>/Gain' */

  /* Selector: '<S2>/Selector1' incorporates:
   *  Product: '<S4>/Divide'
   */
  rtDW->Divide_l[0] = rtDW->Abs[1];

  /* Switch: '<S5>/Switch2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  RelationalOperator: '<S5>/LowerRelop1'
   *  RelationalOperator: '<S5>/UpperRelop'
   *  Selector: '<S2>/Selector1'
   *  Switch: '<S5>/Switch'
   */
  if (rtDW->Switch[0] > rtDW->RateLimiter[0]) {
    rtb_Switch = rtDW->RateLimiter[0];
  } else if (rtDW->Switch[0] < rtDW->Abs[1]) {
    /* Switch: '<S5>/Switch' incorporates:
     *  Selector: '<S2>/Selector1'
     */
    rtb_Switch = rtDW->Abs[1];
  }

  rtDW->Switch[0] = rtb_Switch;
  rtb_Switch = u0;

  /* Selector: '<S2>/Selector1' incorporates:
   *  Product: '<S4>/Divide'
   */
  rtDW->Divide_l[1] = rtDW->Abs[3];

  /* Switch: '<S5>/Switch2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  RelationalOperator: '<S5>/LowerRelop1'
   *  RelationalOperator: '<S5>/UpperRelop'
   *  Selector: '<S2>/Selector1'
   *  Switch: '<S5>/Switch'
   */
  if (rtDW->Switch[1] > rtDW->RateLimiter[1]) {
    rtb_Switch = rtDW->RateLimiter[1];
  } else if (rtDW->Switch[1] < rtDW->Abs[3]) {
    /* Switch: '<S5>/Switch' incorporates:
     *  Selector: '<S2>/Selector1'
     */
    rtb_Switch = rtDW->Abs[3];
  }

  rtDW->Switch[1] = rtb_Switch;
  rtb_Switch = rtb_Switch_c;

  /* Selector: '<S2>/Selector1' incorporates:
   *  Product: '<S4>/Divide'
   */
  rtDW->Divide_l[2] = rtDW->Abs[5];

  /* Switch: '<S5>/Switch2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  RelationalOperator: '<S5>/LowerRelop1'
   *  RelationalOperator: '<S5>/UpperRelop'
   *  Selector: '<S2>/Selector1'
   *  Switch: '<S5>/Switch'
   */
  if (rtDW->Switch[2] > rtDW->RateLimiter[2]) {
    rtb_Switch = rtDW->RateLimiter[2];
  } else if (rtDW->Switch[2] < rtDW->Abs[5]) {
    /* Switch: '<S5>/Switch' incorporates:
     *  Selector: '<S2>/Selector1'
     */
    rtb_Switch = rtDW->Abs[5];
  }

  rtDW->Switch[2] = rtb_Switch;

  /* Product: '<S2>/Product2' incorporates:
   *  Lookup_n-D: '<S2>/2-D Lookup Table'
   */
  rtb_Switch = rtb_RateLimiter * rtDW->Sum1;

  /* Selector: '<S2>/Selector1' incorporates:
   *  Product: '<S4>/Divide'
   */
  rtDW->Divide_l[3] = rtDW->Abs[7];

  /* Switch: '<S5>/Switch2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  RelationalOperator: '<S5>/LowerRelop1'
   *  RelationalOperator: '<S5>/UpperRelop'
   *  Selector: '<S2>/Selector1'
   *  Switch: '<S5>/Switch'
   */
  if (rtDW->Switch[3] > rtDW->RateLimiter[3]) {
    rtb_Switch = rtDW->RateLimiter[3];
  } else if (rtDW->Switch[3] < rtDW->Abs[7]) {
    /* Switch: '<S5>/Switch' incorporates:
     *  Selector: '<S2>/Selector1'
     */
    rtb_Switch = rtDW->Abs[7];
  }

  rtDW->Switch[3] = rtb_Switch;
  for (i_0 = 0; i_0 < 4; i_0++) {
    /* Abs: '<S3>/Abs1' incorporates:
     *  Bias: '<S3>/Bias'
     *  Selector: '<S3>/Selector1'
     *  Switch: '<S6>/Switch'
     */
    rtb_RateLimiter = rtDW->Switch_o[i_0];

    /* Product: '<S3>/MatrixMultiply1' incorporates:
     *  Abs: '<S3>/Abs'
     *  Constant: '<S3>/Constant1'
     *  Selector: '<S3>/Selector'
     *  Sum: '<S3>/Add'
     *  Switch: '<S5>/Switch2'
     */
    rtDW->Sum1 = rtDW->Switch[i_0];
    for (iU = 0; iU < 161; iU++) {
      /* Abs: '<S3>/Abs1' incorporates:
       *  Abs: '<S3>/Abs'
       *  Bias: '<S3>/Bias'
       *  Constant: '<S3>/Constant3'
       *  Constant: '<S3>/Constant4'
       *  Selector: '<S3>/Selector'
       *  Selector: '<S3>/Selector1'
       *  Sum: '<S3>/Add1'
       */
      i = (int32_T)rtConstP.pooled3[iU];
      rtb_Abs1_tmp = 161 * i_0 + iU;
      rtDW->Abs1[rtb_Abs1_tmp] = fabs(rtConstP.pooled4[(((int32_T)
        (rtb_RateLimiter + 1.0) - 1) * 161 + i) - 1] - rtDW->Sum1);
      rtDW->Abs[rtb_Abs1_tmp] = fabs(rtConstP.pooled4[(((int32_T)rtb_RateLimiter
        - 1) * 161 + i) - 1] - rtDW->Sum1);
    }

    /* End of Product: '<S3>/MatrixMultiply1' */
  }

  /* S-Function (sdspstatminmax): '<S3>/Minimum' incorporates:
   *  Abs: '<S3>/Abs'
   */
  iU = 0;
  i_0 = 0;
  for (i = 0; i < 4; i++) {
    rtDW->Minimum_o1[i_0] = rtDW->Abs[iU];
    rtDW->Minimum_o2[i_0] = 1.0;
    iU++;
    for (rtb_Abs1_tmp = 0; rtb_Abs1_tmp < 160; rtb_Abs1_tmp++) {
      if (rtDW->Abs[iU] < rtDW->Minimum_o1[i_0]) {
        rtDW->Minimum_o1[i_0] = rtDW->Abs[iU];
        rtDW->Minimum_o2[i_0] = (real_T)rtb_Abs1_tmp + 2.0;
      }

      iU++;
    }

    i_0++;
  }

  /* End of S-Function (sdspstatminmax): '<S3>/Minimum' */

  /* S-Function (sdspstatminmax): '<S3>/Minimum1' incorporates:
   *  Abs: '<S3>/Abs1'
   *  RelationalOperator: '<S3>/GreaterThan'
   *  Switch: '<S5>/Switch2'
   */
  iU = 0;
  i_0 = 0;
  for (i = 0; i < 4; i++) {
    u0 = rtDW->Switch[i];
    rtDW->Minimum1_Valdata[i_0] = rtDW->Abs1[iU];
    rtDW->Minimum1[i_0] = 1.0;
    iU++;
    for (rtb_Abs1_tmp = 0; rtb_Abs1_tmp < 160; rtb_Abs1_tmp++) {
      if (rtDW->Abs1[iU] < rtDW->Minimum1_Valdata[i_0]) {
        rtDW->Minimum1_Valdata[i_0] = rtDW->Abs1[iU];
        rtDW->Minimum1[i_0] = (real_T)rtb_Abs1_tmp + 2.0;
      }

      iU++;
    }

    i_0++;

    /* Gain: '<S3>/Gain4' incorporates:
     *  Abs: '<S3>/Abs1'
     *  Logic: '<S3>/OR'
     *  Product: '<S4>/Divide'
     *  RateLimiter: '<S3>/Rate Limiter'
     *  RelationalOperator: '<S3>/GreaterThan'
     *  RelationalOperator: '<S3>/Less Than1'
     *  S-Function (sdspstatminmax): '<S3>/Minimum1'
     *  Switch: '<S5>/Switch2'
     */
    rtb_Gain4[i] = (uint8_T)((u0 <= rtDW->Divide_l[i]) || (u0 >=
      rtDW->RateLimiter[i]) ? (int32_T)rtConstP.Gain4_Gain[i] : 0);
  }

  /* End of S-Function (sdspstatminmax): '<S3>/Minimum1' */

  /* Switch: '<S3>/Switch1' incorporates:
   *  Constant: '<S3>/Constant'
   *  Constant: '<S3>/Constant6'
   *  Constant: '<S3>/Constant8'
   *  Gain: '<S3>/Gain1'
   *  Gain: '<S3>/Gain2'
   *  Gain: '<S3>/Gain4'
   *  Inport: '<Root>/Tx'
   *  Inport: '<Root>/Wx'
   *  Inport: '<Root>/motor_T'
   *  Product: '<S3>/Divide'
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product1'
   *  Product: '<S3>/Product2'
   *  Product: '<S4>/Divide'
   *  RelationalOperator: '<S3>/Less Than'
   *  Rounding: '<S3>/Floor'
   *  S-Function (sdspstatminmax): '<S3>/Minimum1'
   *  Selector: '<S3>/Selector3'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   *  Sum: '<S3>/Sum2'
   *  Sum: '<S3>/Sum3'
   */
  if ((uint8_T)((((uint32_T)rtb_Gain4[0] + rtb_Gain4[1]) + rtb_Gain4[2]) +
                rtb_Gain4[3]) > 0) {
    /* Selector: '<S3>/Selector2' incorporates:
     *  Constant: '<S3>/Constant7'
     *  S-Function (sdspstatminmax): '<S3>/Minimum'
     */
    rtb_RateLimiter = rtConstP.pooled6[(int32_T)rtDW->Minimum_o2[0] - 1];

    /* Signum: '<S3>/Sign' incorporates:
     *  Switch: '<S5>/Switch2'
     */
    if (rtIsNaN(rtDW->Switch[0])) {
      rtDW->Sum1 = rtDW->Switch[0];
    } else if (rtDW->Switch[0] < 0.0) {
      rtDW->Sum1 = -1.0;
    } else {
      rtDW->Sum1 = (rtDW->Switch[0] > 0.0);
    }

    rtDW->Divide_l[0] = ((rtU->Wx[0] - floor(0.095492965964253843 * rtU->Wx[0]) *
                          10.4719755) / 10.4719755 * (rtConstP.pooled6[(int32_T)
      rtDW->Minimum1[0] - 1] - rtb_RateLimiter) + rtb_RateLimiter) * (real_T)
      (rtU->motor_T[0] < 75.0) * rtDW->Sum1;

    /* Selector: '<S3>/Selector2' incorporates:
     *  Constant: '<S3>/Constant'
     *  Constant: '<S3>/Constant6'
     *  Constant: '<S3>/Constant7'
     *  Constant: '<S3>/Constant8'
     *  Gain: '<S3>/Gain1'
     *  Gain: '<S3>/Gain2'
     *  Inport: '<Root>/Wx'
     *  Inport: '<Root>/motor_T'
     *  Product: '<S3>/Divide'
     *  Product: '<S3>/Product'
     *  Product: '<S3>/Product1'
     *  Product: '<S3>/Product2'
     *  Product: '<S4>/Divide'
     *  RelationalOperator: '<S3>/Less Than'
     *  Rounding: '<S3>/Floor'
     *  S-Function (sdspstatminmax): '<S3>/Minimum'
     *  S-Function (sdspstatminmax): '<S3>/Minimum1'
     *  Selector: '<S3>/Selector3'
     *  Sum: '<S3>/Sum'
     *  Sum: '<S3>/Sum2'
     *  Sum: '<S3>/Sum3'
     */
    rtb_RateLimiter = rtConstP.pooled6[(int32_T)rtDW->Minimum_o2[1] - 1];

    /* Signum: '<S3>/Sign' incorporates:
     *  Switch: '<S5>/Switch2'
     */
    if (rtIsNaN(rtDW->Switch[1])) {
      rtDW->Sum1 = rtDW->Switch[1];
    } else if (rtDW->Switch[1] < 0.0) {
      rtDW->Sum1 = -1.0;
    } else {
      rtDW->Sum1 = (rtDW->Switch[1] > 0.0);
    }

    rtDW->Divide_l[1] = ((rtU->Wx[1] - floor(0.095492965964253843 * rtU->Wx[1]) *
                          10.4719755) / 10.4719755 * (rtConstP.pooled6[(int32_T)
      rtDW->Minimum1[1] - 1] - rtb_RateLimiter) + rtb_RateLimiter) * (real_T)
      (rtU->motor_T[1] < 75.0) * rtDW->Sum1;

    /* Selector: '<S3>/Selector2' incorporates:
     *  Constant: '<S3>/Constant'
     *  Constant: '<S3>/Constant6'
     *  Constant: '<S3>/Constant7'
     *  Constant: '<S3>/Constant8'
     *  Gain: '<S3>/Gain1'
     *  Gain: '<S3>/Gain2'
     *  Inport: '<Root>/Wx'
     *  Inport: '<Root>/motor_T'
     *  Product: '<S3>/Divide'
     *  Product: '<S3>/Product'
     *  Product: '<S3>/Product1'
     *  Product: '<S3>/Product2'
     *  Product: '<S4>/Divide'
     *  RelationalOperator: '<S3>/Less Than'
     *  Rounding: '<S3>/Floor'
     *  S-Function (sdspstatminmax): '<S3>/Minimum'
     *  S-Function (sdspstatminmax): '<S3>/Minimum1'
     *  Selector: '<S3>/Selector3'
     *  Sum: '<S3>/Sum'
     *  Sum: '<S3>/Sum2'
     *  Sum: '<S3>/Sum3'
     */
    rtb_RateLimiter = rtConstP.pooled6[(int32_T)rtDW->Minimum_o2[2] - 1];

    /* Signum: '<S3>/Sign' incorporates:
     *  Switch: '<S5>/Switch2'
     */
    if (rtIsNaN(rtDW->Switch[2])) {
      rtDW->Sum1 = rtDW->Switch[2];
    } else if (rtDW->Switch[2] < 0.0) {
      rtDW->Sum1 = -1.0;
    } else {
      rtDW->Sum1 = (rtDW->Switch[2] > 0.0);
    }

    rtDW->Divide_l[2] = ((rtU->Wx[2] - floor(0.095492965964253843 * rtU->Wx[2]) *
                          10.4719755) / 10.4719755 * (rtConstP.pooled6[(int32_T)
      rtDW->Minimum1[2] - 1] - rtb_RateLimiter) + rtb_RateLimiter) * (real_T)
      (rtU->motor_T[2] < 75.0) * rtDW->Sum1;

    /* Selector: '<S3>/Selector2' incorporates:
     *  Constant: '<S3>/Constant'
     *  Constant: '<S3>/Constant6'
     *  Constant: '<S3>/Constant7'
     *  Constant: '<S3>/Constant8'
     *  Gain: '<S3>/Gain1'
     *  Gain: '<S3>/Gain2'
     *  Inport: '<Root>/Wx'
     *  Inport: '<Root>/motor_T'
     *  Product: '<S3>/Divide'
     *  Product: '<S3>/Product'
     *  Product: '<S3>/Product1'
     *  Product: '<S3>/Product2'
     *  Product: '<S4>/Divide'
     *  RelationalOperator: '<S3>/Less Than'
     *  Rounding: '<S3>/Floor'
     *  S-Function (sdspstatminmax): '<S3>/Minimum'
     *  S-Function (sdspstatminmax): '<S3>/Minimum1'
     *  Selector: '<S3>/Selector3'
     *  Sum: '<S3>/Sum'
     *  Sum: '<S3>/Sum2'
     *  Sum: '<S3>/Sum3'
     */
    rtb_RateLimiter = rtConstP.pooled6[(int32_T)rtDW->Minimum_o2[3] - 1];

    /* Signum: '<S3>/Sign' */
    if (rtIsNaN(rtb_Switch)) {
      rtDW->Sum1 = rtb_Switch;
    } else if (rtb_Switch < 0.0) {
      rtDW->Sum1 = -1.0;
    } else {
      rtDW->Sum1 = (rtb_Switch > 0.0);
    }

    rtDW->Divide_l[3] = ((rtU->Wx[3] - floor(0.095492965964253843 * rtU->Wx[3]) *
                          10.4719755) / 10.4719755 * (rtConstP.pooled6[(int32_T)
      rtDW->Minimum1[3] - 1] - rtb_RateLimiter) + rtb_RateLimiter) * (real_T)
      (rtU->motor_T[3] < 75.0) * rtDW->Sum1;
  } else {
    rtDW->Divide_l[0] = rtU->Tx[0];
    rtDW->Divide_l[1] = rtU->Tx[1];
    rtDW->Divide_l[2] = rtU->Tx[2];
    rtDW->Divide_l[3] = rtU->Tx[3];
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Wx'
   */
  rtb_RateLimiter = look1_binlc(rtU->Wx[0], rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 67U);

  /* Switch: '<S6>/Switch2' incorporates:
   *  Product: '<S4>/Divide'
   *  RelationalOperator: '<S6>/LowerRelop1'
   */
  if (!(rtDW->Divide_l[0] > rtb_RateLimiter)) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Gain: '<S3>/Gain3'
     *  RelationalOperator: '<S6>/UpperRelop'
     */
    if (rtDW->Divide_l[0] < -rtb_RateLimiter) {
      rtb_RateLimiter = -rtb_RateLimiter;
    } else {
      rtb_RateLimiter = rtDW->Divide_l[0];
    }
  }

  /* DeadZone: '<S3>/Dead Zone' */
  if (rtb_RateLimiter > 0.02) {
    rtb_RateLimiter -= 0.02;
  } else if (rtb_RateLimiter >= -0.02) {
    rtb_RateLimiter = 0.0;
  } else {
    rtb_RateLimiter -= -0.02;
  }

  /* RateLimiter: '<S3>/Rate Limiter' */
  rtDW->Sum1 = rtb_RateLimiter - rtDW->PrevY[0];
  if (rtDW->Sum1 > 1.875) {
    rtb_RateLimiter = rtDW->PrevY[0] + 1.875;
  } else if (rtDW->Sum1 < -4.5) {
    rtb_RateLimiter = rtDW->PrevY[0] + -4.5;
  }

  rtDW->PrevY[0] = rtb_RateLimiter;

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[0] = rtb_RateLimiter;

  /* Abs: '<S4>/Abs2' */
  rtDW->Divide_l[0] = fabs(rtb_RateLimiter);

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Wx'
   */
  rtb_RateLimiter = look1_binlc(rtU->Wx[1], rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 67U);

  /* Switch: '<S6>/Switch2' incorporates:
   *  Product: '<S4>/Divide'
   *  RelationalOperator: '<S6>/LowerRelop1'
   */
  if (!(rtDW->Divide_l[1] > rtb_RateLimiter)) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Gain: '<S3>/Gain3'
     *  RelationalOperator: '<S6>/UpperRelop'
     */
    if (rtDW->Divide_l[1] < -rtb_RateLimiter) {
      rtb_RateLimiter = -rtb_RateLimiter;
    } else {
      rtb_RateLimiter = rtDW->Divide_l[1];
    }
  }

  /* DeadZone: '<S3>/Dead Zone' */
  if (rtb_RateLimiter > 0.02) {
    rtb_RateLimiter -= 0.02;
  } else if (rtb_RateLimiter >= -0.02) {
    rtb_RateLimiter = 0.0;
  } else {
    rtb_RateLimiter -= -0.02;
  }

  /* RateLimiter: '<S3>/Rate Limiter' */
  rtDW->Sum1 = rtb_RateLimiter - rtDW->PrevY[1];
  if (rtDW->Sum1 > 1.875) {
    rtb_RateLimiter = rtDW->PrevY[1] + 1.875;
  } else if (rtDW->Sum1 < -4.5) {
    rtb_RateLimiter = rtDW->PrevY[1] + -4.5;
  }

  rtDW->PrevY[1] = rtb_RateLimiter;

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[1] = rtb_RateLimiter;

  /* Abs: '<S4>/Abs2' */
  rtDW->Divide_l[1] = fabs(rtb_RateLimiter);

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Wx'
   */
  rtb_RateLimiter = look1_binlc(rtU->Wx[2], rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 67U);

  /* Switch: '<S6>/Switch2' incorporates:
   *  Product: '<S4>/Divide'
   *  RelationalOperator: '<S6>/LowerRelop1'
   */
  if (!(rtDW->Divide_l[2] > rtb_RateLimiter)) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Gain: '<S3>/Gain3'
     *  RelationalOperator: '<S6>/UpperRelop'
     */
    if (rtDW->Divide_l[2] < -rtb_RateLimiter) {
      rtb_RateLimiter = -rtb_RateLimiter;
    } else {
      rtb_RateLimiter = rtDW->Divide_l[2];
    }
  }

  /* DeadZone: '<S3>/Dead Zone' */
  if (rtb_RateLimiter > 0.02) {
    rtb_RateLimiter -= 0.02;
  } else if (rtb_RateLimiter >= -0.02) {
    rtb_RateLimiter = 0.0;
  } else {
    rtb_RateLimiter -= -0.02;
  }

  /* RateLimiter: '<S3>/Rate Limiter' */
  rtDW->Sum1 = rtb_RateLimiter - rtDW->PrevY[2];
  if (rtDW->Sum1 > 1.875) {
    rtb_RateLimiter = rtDW->PrevY[2] + 1.875;
  } else if (rtDW->Sum1 < -4.5) {
    rtb_RateLimiter = rtDW->PrevY[2] + -4.5;
  }

  rtDW->PrevY[2] = rtb_RateLimiter;

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  rtDW->RateLimiter[2] = rtb_RateLimiter;

  /* Abs: '<S4>/Abs2' */
  rtDW->Divide_l[2] = fabs(rtb_RateLimiter);

  /* Lookup_n-D: '<S3>/1-D Lookup Table' incorporates:
   *  Inport: '<Root>/Wx'
   */
  rtb_RateLimiter = look1_binlc(rtU->Wx[3], rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 67U);

  /* Switch: '<S6>/Switch2' incorporates:
   *  Product: '<S4>/Divide'
   *  RelationalOperator: '<S6>/LowerRelop1'
   */
  if (!(rtDW->Divide_l[3] > rtb_RateLimiter)) {
    /* Switch: '<S6>/Switch' incorporates:
     *  Gain: '<S3>/Gain3'
     *  RelationalOperator: '<S6>/UpperRelop'
     */
    if (rtDW->Divide_l[3] < -rtb_RateLimiter) {
      rtb_RateLimiter = -rtb_RateLimiter;
    } else {
      rtb_RateLimiter = rtDW->Divide_l[3];
    }
  }

  /* DeadZone: '<S3>/Dead Zone' */
  if (rtb_RateLimiter > 0.02) {
    rtb_RateLimiter -= 0.02;
  } else if (rtb_RateLimiter >= -0.02) {
    rtb_RateLimiter = 0.0;
  } else {
    rtb_RateLimiter -= -0.02;
  }

  /* RateLimiter: '<S3>/Rate Limiter' */
  rtDW->Sum1 = rtb_RateLimiter - rtDW->PrevY[3];
  if (rtDW->Sum1 > 1.875) {
    rtb_RateLimiter = rtDW->PrevY[3] + 1.875;
  } else if (rtDW->Sum1 < -4.5) {
    rtb_RateLimiter = rtDW->PrevY[3] + -4.5;
  }

  rtDW->PrevY[3] = rtb_RateLimiter;

  /* Lookup_n-D: '<S4>/2-D Lookup Table1' incorporates:
   *  Abs: '<S4>/Abs2'
   *  Inport: '<Root>/Wx'
   *  Product: '<S4>/Divide'
   */
  rtDW->bpIndices_c[0U] = plook_evenca(rtDW->Divide_l[0], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions_m[0U] = rtDW->Sum1;
  rtDW->bpIndices_c[1U] = plook_evenca(rtU->Wx[0], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions_m[1U] = rtDW->Sum1;
  rtDW->Divide_l[0] = intrp2d_la(rtDW->bpIndices_c, rtDW->fractions_m,
    rtConstP.uDLookupTable1_tableData, 161U, rtConstP.pooled9);
  rtDW->bpIndices_c[0U] = plook_evenca(rtDW->Divide_l[1], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions_m[0U] = rtDW->Sum1;
  rtDW->bpIndices_c[1U] = plook_evenca(rtU->Wx[1], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions_m[1U] = rtDW->Sum1;
  rtDW->Divide_l[1] = intrp2d_la(rtDW->bpIndices_c, rtDW->fractions_m,
    rtConstP.uDLookupTable1_tableData, 161U, rtConstP.pooled9);
  rtDW->bpIndices_c[0U] = plook_evenca(rtDW->Divide_l[2], 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions_m[0U] = rtDW->Sum1;
  rtDW->bpIndices_c[1U] = plook_evenca(rtU->Wx[2], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions_m[1U] = rtDW->Sum1;
  rtDW->Divide_l[2] = intrp2d_la(rtDW->bpIndices_c, rtDW->fractions_m,
    rtConstP.uDLookupTable1_tableData, 161U, rtConstP.pooled9);
  rtDW->bpIndices_c[0U] = plook_evenca(fabs(rtb_RateLimiter), 0.0, 0.15625, 160U,
    &rtDW->Sum1);
  rtDW->fractions_m[0U] = rtDW->Sum1;
  rtDW->bpIndices_c[1U] = plook_evenca(rtU->Wx[3], 0.0, 10.4719755, 106U,
    &rtDW->Sum1);
  rtDW->fractions_m[1U] = rtDW->Sum1;

  /* Product: '<S4>/Divide' incorporates:
   *  Inport: '<Root>/Vbatt'
   */
  u0 = rtDW->Divide_l[0] / rtU->Vbatt;

  /* Signum: '<S4>/Sign2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  if (rtIsNaN(rtDW->RateLimiter[0])) {
    rtDW->Sum1 = rtDW->RateLimiter[0];
  } else if (rtDW->RateLimiter[0] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtDW->RateLimiter[0] > 0.0);
  }

  /* MinMax: '<S4>/Min' */
  if (!(u0 <= 1.0)) {
    u0 = 1.0;
  }

  /* Outport: '<Root>/k' incorporates:
   *  Gain: '<S4>/Gain1'
   *  Product: '<S4>/Product1'
   */
  rtY->k[0] = rtDW->Sum1 * u0 * 4095.0;

  /* Outport: '<Root>/T' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  Signum: '<S4>/Sign2'
   */
  rtY->T[0] = rtDW->RateLimiter[0];

  /* Outport: '<Root>/P' incorporates:
   *  Switch: '<S5>/Switch2'
   */
  rtY->P_b[0] = rtDW->Switch[0];

  /* Product: '<S4>/Divide' incorporates:
   *  Inport: '<Root>/Vbatt'
   */
  u0 = rtDW->Divide_l[1] / rtU->Vbatt;

  /* Signum: '<S4>/Sign2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  if (rtIsNaN(rtDW->RateLimiter[1])) {
    rtDW->Sum1 = rtDW->RateLimiter[1];
  } else if (rtDW->RateLimiter[1] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtDW->RateLimiter[1] > 0.0);
  }

  /* MinMax: '<S4>/Min' */
  if (!(u0 <= 1.0)) {
    u0 = 1.0;
  }

  /* Outport: '<Root>/k' incorporates:
   *  Gain: '<S4>/Gain1'
   *  Product: '<S4>/Product1'
   */
  rtY->k[1] = rtDW->Sum1 * u0 * 4095.0;

  /* Outport: '<Root>/T' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  Signum: '<S4>/Sign2'
   */
  rtY->T[1] = rtDW->RateLimiter[1];

  /* Outport: '<Root>/P' incorporates:
   *  Switch: '<S5>/Switch2'
   */
  rtY->P_b[1] = rtDW->Switch[1];

  /* Product: '<S4>/Divide' incorporates:
   *  Inport: '<Root>/Vbatt'
   */
  u0 = rtDW->Divide_l[2] / rtU->Vbatt;

  /* Signum: '<S4>/Sign2' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   */
  if (rtIsNaN(rtDW->RateLimiter[2])) {
    rtDW->Sum1 = rtDW->RateLimiter[2];
  } else if (rtDW->RateLimiter[2] < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtDW->RateLimiter[2] > 0.0);
  }

  /* MinMax: '<S4>/Min' */
  if (!(u0 <= 1.0)) {
    u0 = 1.0;
  }

  /* Outport: '<Root>/k' incorporates:
   *  Gain: '<S4>/Gain1'
   *  Product: '<S4>/Product1'
   */
  rtY->k[2] = rtDW->Sum1 * u0 * 4095.0;

  /* Outport: '<Root>/T' incorporates:
   *  RateLimiter: '<S3>/Rate Limiter'
   *  Signum: '<S4>/Sign2'
   */
  rtY->T[2] = rtDW->RateLimiter[2];

  /* Outport: '<Root>/P' incorporates:
   *  Switch: '<S5>/Switch2'
   */
  rtY->P_b[2] = rtDW->Switch[2];

  /* Product: '<S4>/Divide' incorporates:
   *  Inport: '<Root>/Vbatt'
   *  Lookup_n-D: '<S4>/2-D Lookup Table1'
   */
  u0 = intrp2d_la(rtDW->bpIndices_c, rtDW->fractions_m,
                  rtConstP.uDLookupTable1_tableData, 161U, rtConstP.pooled9) /
    rtU->Vbatt;

  /* Signum: '<S4>/Sign2' */
  if (rtIsNaN(rtb_RateLimiter)) {
    rtDW->Sum1 = rtb_RateLimiter;
  } else if (rtb_RateLimiter < 0.0) {
    rtDW->Sum1 = -1.0;
  } else {
    rtDW->Sum1 = (rtb_RateLimiter > 0.0);
  }

  /* MinMax: '<S4>/Min' */
  if (!(u0 <= 1.0)) {
    u0 = 1.0;
  }

  /* Outport: '<Root>/k' incorporates:
   *  Gain: '<S4>/Gain1'
   *  Product: '<S4>/Product1'
   */
  rtY->k[3] = rtDW->Sum1 * u0 * 4095.0;

  /* Outport: '<Root>/T' */
  rtY->T[3] = rtb_RateLimiter;

  /* Outport: '<Root>/P' */
  rtY->P_b[3] = rtb_Switch;
}

/* Model initialize function */
void MC_PL0_initialize(RT_MODEL *const rtM)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  UNUSED_PARAMETER(rtM);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
