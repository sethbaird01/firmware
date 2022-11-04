/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.c
 *
 * Code generated for Simulink model 'Electronics'.
 *
 * Model version                  : 1.199
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Thu Nov  3 19:42:22 2022
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
#include <stddef.h>
#define NumBitsPerChar                 8U
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */
static real_T look1_binlg(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);
static uint32_T plook_evenc(real_T u, real_T bp0, real_T bpSpace, uint32_T
  maxIndex, real_T *fraction);
static real_T intrp1d_l(uint32_T bpIndex, real_T frac, const real_T table[]);
static uint32_T plook_u32u32u32n31_even20c_gf(uint32_T u, uint32_T bp0, uint32_T
  maxIndex, uint32_T *fraction);
static int32_T intrp1d_s32s64s64u32u32n31l_f(uint32_T bpIndex, uint32_T frac,
  const int32_T table[]);
static uint32_T plook_u32u32u64n32_even20c_gf(uint32_T u, uint32_T bp0, uint32_T
  maxIndex, uint64_T *fraction);
static int32_T intrp1d_s32s64s64u32u64n32l_f(uint32_T bpIndex, uint64_T frac,
  const int32_T table[]);
static void Electronics_Init(DW_Electronics *localDW);
static void Electronics_o(const real_T rtu_TVS_Information[4], const real_T
  rtu_TVS_Information_d[2], real_T rtu_TVS_Information_c, const real_T
  rtu_TVS_Information_b[3], real_T rtu_TVS_Information_i, const real_T
  rtu_TVS_Information_k[2], int32_T *rty_bigM_flag, real_T rty_Tx[4],
  DW_Electronics *localDW);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
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

static real_T look1_binlg(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex)
{
  real_T yL_0d0;
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
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

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]) * (table[iLeft + 1U]
    - yL_0d0) + yL_0d0;
}

static uint32_T plook_evenc(real_T u, real_T bp0, real_T bpSpace, uint32_T
  maxIndex, real_T *fraction)
{
  real_T fbpIndex;
  real_T invSpc;
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
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
      bpIndex = maxIndex - 1U;
      *fraction = 1.0;
    }
  }

  return bpIndex;
}

static real_T intrp1d_l(uint32_T bpIndex, real_T frac, const real_T table[])
{
  real_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (table[bpIndex + 1U] - yL_0d0) * frac + yL_0d0;
}

static uint32_T plook_u32u32u32n31_even20c_gf(uint32_T u, uint32_T bp0, uint32_T
  maxIndex, uint32_T *fraction)
{
  uint32_T bpIndex;
  uint32_T uAdjust;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'floor'
   */
  uAdjust = u - bp0;
  bpIndex = uAdjust >> 20U;
  if (bpIndex < maxIndex) {
    *fraction = (uAdjust & 1048575U) << 11;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 2147483648U;
  }

  return bpIndex;
}

static int32_T intrp1d_s32s64s64u32u32n31l_f(uint32_T bpIndex, uint32_T frac,
  const int32_T table[])
{
  int32_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'floor'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (int32_T)((((int64_T)table[bpIndex + 1U] - yL_0d0) * frac) >> 31) +
    yL_0d0;
}

static uint32_T plook_u32u32u64n32_even20c_gf(uint32_T u, uint32_T bp0, uint32_T
  maxIndex, uint64_T *fraction)
{
  uint32_T bpIndex;
  uint32_T uAdjust;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'floor'
   */
  uAdjust = u - bp0;
  bpIndex = uAdjust >> 20U;
  if (bpIndex < maxIndex) {
    *fraction = (uint64_T)(uAdjust & 1048575U) << 12;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 4294967296ULL;
  }

  return bpIndex;
}

static int32_T intrp1d_s32s64s64u32u64n32l_f(uint32_T bpIndex, uint64_T frac,
  const int32_T table[])
{
  int32_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'floor'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (int32_T)((((int64_T)table[bpIndex + 1U] - yL_0d0) * (int64_T)frac) >>
                   32) + yL_0d0;
}

/* System initialize for atomic system: '<Root>/Electronics' */
static void Electronics_Init(DW_Electronics *localDW)
{
  /* SystemInitialize for Atomic SubSystem: '<S1>/Fixed Point Sub' */
  /* SystemInitialize for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* InitializeConditions for UnitDelay: '<S17>/Unit Delay4' */
  localDW->UnitDelay4_DSTATE = true;

  /* End of SystemInitialize for SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* End of SystemInitialize for SubSystem: '<S1>/Fixed Point Sub' */
}

/* Output and update for atomic system: '<Root>/Electronics' */
static void Electronics_o(const real_T rtu_TVS_Information[4], const real_T
  rtu_TVS_Information_d[2], real_T rtu_TVS_Information_c, const real_T
  rtu_TVS_Information_b[3], real_T rtu_TVS_Information_i, const real_T
  rtu_TVS_Information_k[2], int32_T *rty_bigM_flag, real_T rty_Tx[4],
  DW_Electronics *localDW)
{
  real_T rtb_Abs_m_idx_0;
  real_T rtb_Abs_m_idx_1;
  real_T rtb_UnitDelay_idx_0;
  int32_T i;
  int32_T rtb_Abs1_tmp;
  int32_T rtb_TmpSignalConversionAtDotP_0;
  int32_T rtb_TmpSignalConversionAtDotP_1;
  uint32_T frac;
  uint32_T rtb_QuadHandle1b;
  uint32_T rtb_QuadHandle1b_idx_0;
  uint32_T rtb_QuadHandle1b_tmp;
  boolean_T rtb_LTEp25_p_idx_0;
  boolean_T rtb_LTEp50_m_idx_0;
  boolean_T rtb_LessThan_e;
  boolean_T rtb_LowerRelop1;

  /* Outputs for Atomic SubSystem: '<S1>/Fixed Point Sub' */
  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Sum: '<S12>/Subtract' */
  localDW->rtb_Sum4_idx_1 = rtu_TVS_Information[0] - rtu_TVS_Information[0];
  localDW->CCaller_o5 = rtu_TVS_Information[1] - rtu_TVS_Information[1];
  localDW->rtu_TVS_Information_idx_2 = rtu_TVS_Information[2] -
    rtu_TVS_Information[2];
  localDW->Square1 = rtu_TVS_Information[3] - rtu_TVS_Information[3];
  rty_Tx[0] = localDW->rtb_Sum4_idx_1;
  rty_Tx[1] = localDW->CCaller_o5;
  rty_Tx[2] = localDW->rtu_TVS_Information_idx_2;
  rty_Tx[3] = localDW->Square1;

  /* Abs: '<S12>/Abs' */
  rty_Tx[0] = fabs(rty_Tx[0]);
  rty_Tx[1] = fabs(rty_Tx[1]);
  rty_Tx[2] = fabs(rty_Tx[2]);
  rty_Tx[3] = fabs(rty_Tx[3]);

  /* Switch: '<S12>/Switch' */
  localDW->rtb_Sum4_idx_1 = rtu_TVS_Information[0];
  localDW->CCaller_o5 = rtu_TVS_Information[1];
  localDW->rtu_TVS_Information_idx_2 = rtu_TVS_Information[2];
  localDW->Square1 = rtu_TVS_Information[3];
  rty_Tx[0] = localDW->rtb_Sum4_idx_1;
  rty_Tx[1] = localDW->CCaller_o5;
  rty_Tx[2] = localDW->rtu_TVS_Information_idx_2;
  rty_Tx[3] = localDW->Square1;

  /* Gain: '<S14>/Gain1' */
  rty_Tx[0] *= 6.63043;
  rty_Tx[1] *= 6.63043;
  rty_Tx[2] *= 6.63;
  rty_Tx[3] *= 6.63;

  /* Gain: '<S15>/Gain' incorporates:
   *  Bias: '<S5>/Add Constant'
   */
  localDW->AddConstant[0] = 0.095492965964253843 * rty_Tx[0];
  localDW->AddConstant[1] = 0.095492965964253843 * rty_Tx[1];
  localDW->AddConstant[2] = 0.095492965964253843 * rty_Tx[2];
  localDW->AddConstant[3] = 0.095492965964253843 * rty_Tx[3];
  for (i = 0; i < 4; i++) {
    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S5>/Add Constant'
     */
    localDW->rtb_Sum4_idx_1 = localDW->AddConstant[i];
    if (localDW->rtb_Sum4_idx_1 > 106.0) {
      localDW->rtb_Sum4_idx_1 = 106.0;
    } else if (localDW->rtb_Sum4_idx_1 < 1.0) {
      localDW->rtb_Sum4_idx_1 = 1.0;
    }

    /* Switch: '<S15>/Switch' incorporates:
     *  S-Function (sdspstatminmax): '<S15>/Minimum1'
     */
    localDW->Minimum1[i] = 1800.0;
    for (rtb_TmpSignalConversionAtDotP_0 = 0; rtb_TmpSignalConversionAtDotP_0 <
         51; rtb_TmpSignalConversionAtDotP_0++) {
      /* Abs: '<S15>/Abs1' incorporates:
       *  Abs: '<S15>/Abs'
       *  Bias: '<S15>/Bias'
       *  Constant: '<S15>/Constant4'
       *  Constant: '<S15>/Constant5'
       *  Gain: '<S15>/Gain3'
       *  Selector: '<S15>/Selector'
       *  Selector: '<S15>/Selector1'
       *  Sum: '<S15>/Sum1'
       */
      rtb_TmpSignalConversionAtDotP_1 = (int32_T)
        rtConstP.pooled5[rtb_TmpSignalConversionAtDotP_0];
      rtb_Abs1_tmp = 51 * i + rtb_TmpSignalConversionAtDotP_0;
      localDW->Abs1[rtb_Abs1_tmp] = fabs(rtConstP.pooled9[(((int32_T)
        (localDW->rtb_Sum4_idx_1 + 1.0) - 1) * 51 +
        rtb_TmpSignalConversionAtDotP_1) - 1] - 1800.0);

      /* Gain: '<S15>/Gain3' incorporates:
       *  Abs: '<S15>/Abs'
       *  Constant: '<S15>/Constant1'
       *  Selector: '<S15>/Selector'
       *  Sum: '<S15>/Sum'
       */
      localDW->Abs_i[rtb_Abs1_tmp] = fabs(rtConstP.pooled9[(((int32_T)
        localDW->rtb_Sum4_idx_1 - 1) * 51 + rtb_TmpSignalConversionAtDotP_1) - 1]
        - 1800.0);
    }

    /* Saturate: '<S15>/Saturation1' incorporates:
     *  Bias: '<S5>/Add Constant'
     */
    localDW->AddConstant[i] = localDW->rtb_Sum4_idx_1;
  }

  /* S-Function (sdspstatminmax): '<S15>/Minimum1' incorporates:
   *  Abs: '<S15>/Abs1'
   */
  rtb_TmpSignalConversionAtDotP_1 = 0;
  rtb_Abs1_tmp = 0;
  for (i = 0; i < 4; i++) {
    localDW->Minimum1_Valdata[rtb_Abs1_tmp] = localDW->
      Abs1[rtb_TmpSignalConversionAtDotP_1];
    localDW->Minimum1[rtb_Abs1_tmp] = 1.0;
    rtb_TmpSignalConversionAtDotP_1++;
    for (rtb_TmpSignalConversionAtDotP_0 = 0; rtb_TmpSignalConversionAtDotP_0 <
         50; rtb_TmpSignalConversionAtDotP_0++) {
      if (localDW->Abs1[rtb_TmpSignalConversionAtDotP_1] <
          localDW->Minimum1_Valdata[rtb_Abs1_tmp]) {
        localDW->Minimum1_Valdata[rtb_Abs1_tmp] = localDW->
          Abs1[rtb_TmpSignalConversionAtDotP_1];
        localDW->Minimum1[rtb_Abs1_tmp] = (real_T)
          rtb_TmpSignalConversionAtDotP_0 + 2.0;
      }

      rtb_TmpSignalConversionAtDotP_1++;
    }

    rtb_Abs1_tmp++;
  }

  /* End of S-Function (sdspstatminmax): '<S15>/Minimum1' */

  /* S-Function (sdspstatminmax): '<S15>/Minimum' incorporates:
   *  Abs: '<S15>/Abs'
   *  Gain: '<S16>/Gain8'
   */
  rtb_TmpSignalConversionAtDotP_1 = 0;
  rtb_Abs1_tmp = 0;
  for (i = 0; i < 4; i++) {
    localDW->Minimum_Valdata[rtb_Abs1_tmp] = localDW->
      Abs_i[rtb_TmpSignalConversionAtDotP_1];
    localDW->Gain8[rtb_Abs1_tmp] = 1.0;
    rtb_TmpSignalConversionAtDotP_1++;
    for (rtb_TmpSignalConversionAtDotP_0 = 0; rtb_TmpSignalConversionAtDotP_0 <
         50; rtb_TmpSignalConversionAtDotP_0++) {
      if (localDW->Abs_i[rtb_TmpSignalConversionAtDotP_1] <
          localDW->Minimum_Valdata[rtb_Abs1_tmp]) {
        localDW->Minimum_Valdata[rtb_Abs1_tmp] = localDW->
          Abs_i[rtb_TmpSignalConversionAtDotP_1];
        localDW->Gain8[rtb_Abs1_tmp] = (real_T)rtb_TmpSignalConversionAtDotP_0 +
          2.0;
      }

      rtb_TmpSignalConversionAtDotP_1++;
    }

    rtb_Abs1_tmp++;
  }

  /* End of S-Function (sdspstatminmax): '<S15>/Minimum' */

  /* Selector: '<S15>/Selector2' incorporates:
   *  Constant: '<S15>/Constant7'
   *  Gain: '<S16>/Gain8'
   */
  localDW->rtb_Gain8_tmp = rtConstP.pooled6[(int32_T)localDW->Gain8[0] - 1];
  localDW->rtu_TVS_Information_idx_2 = rtConstP.pooled6[(int32_T)localDW->Gain8
    [1] - 1];
  localDW->CCaller_o2 = rtConstP.pooled6[(int32_T)localDW->Gain8[2] - 1];
  localDW->Square = rtConstP.pooled6[(int32_T)localDW->Gain8[3] - 1];

  /* Sum: '<S15>/Sum4' incorporates:
   *  Bias: '<S15>/Bias3'
   *  Bias: '<S5>/Add Constant'
   *  Gain: '<S15>/Gain1'
   */
  localDW->rtb_Sum4_idx_0 = rty_Tx[0] - (localDW->AddConstant[0] + -1.0) *
    10.4719755;
  localDW->rtb_Sum4_idx_1 = rty_Tx[1] - (localDW->AddConstant[1] + -1.0) *
    10.4719755;
  localDW->rtb_Sum4_idx_2 = rty_Tx[2] - (localDW->AddConstant[2] + -1.0) *
    10.4719755;
  localDW->Square1 = rty_Tx[3] - (localDW->AddConstant[3] + -1.0) * 10.4719755;

  /* Lookup_n-D: '<S15>/1-D Lookup Table' incorporates:
   *  Bias: '<S5>/Add Constant'
   */
  localDW->CCaller_o5 = rty_Tx[0];
  localDW->AddConstant[0] = look1_binlg(localDW->CCaller_o5,
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 67U);
  localDW->CCaller_o5 = rty_Tx[1];
  localDW->AddConstant[1] = look1_binlg(localDW->CCaller_o5,
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 67U);
  localDW->CCaller_o5 = rty_Tx[2];
  localDW->AddConstant[2] = look1_binlg(localDW->CCaller_o5,
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 67U);
  localDW->CCaller_o5 = rty_Tx[3];
  localDW->AddConstant[3] = look1_binlg(localDW->CCaller_o5,
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 67U);

  /* Selector: '<S15>/Selector3' incorporates:
   *  Constant: '<S15>/Constant8'
   *  S-Function (sdspstatminmax): '<S15>/Minimum1'
   *  Sum: '<S15>/Sum2'
   */
  localDW->CCaller_o5 = rtConstP.pooled6[(int32_T)localDW->Minimum1[0] - 1];

  /* Switch: '<S15>/Switch2' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S15>/Constant7'
   *  Constant: '<S15>/Constant8'
   *  Gain: '<S15>/Gain2'
   *  Gain: '<S16>/Gain8'
   *  Product: '<S15>/Product1'
   *  Selector: '<S15>/Selector2'
   *  Selector: '<S15>/Selector3'
   *  Sum: '<S15>/Sum2'
   *  Sum: '<S15>/Sum3'
   *  Sum: '<S15>/Sum4'
   */
  localDW->Gain8[0] = localDW->rtb_Gain8_tmp - (localDW->rtb_Gain8_tmp -
    localDW->CCaller_o5) * localDW->rtb_Sum4_idx_0 * 0.095492965964253843;
  if (!(localDW->rtb_Gain8_tmp - (localDW->rtb_Gain8_tmp - localDW->CCaller_o5) *
        localDW->rtb_Sum4_idx_0 * 0.095492965964253843 > 1.0)) {
    localDW->Gain8[0] = localDW->AddConstant[0];
  }

  /* Sum: '<S15>/Sum2' incorporates:
   *  Constant: '<S15>/Constant8'
   *  Gain: '<S15>/Gain2'
   *  Product: '<S15>/Product1'
   *  S-Function (sdspstatminmax): '<S15>/Minimum1'
   *  Selector: '<S15>/Selector3'
   *  Sum: '<S15>/Sum3'
   *  Sum: '<S15>/Sum4'
   */
  localDW->rtb_Gain8_tmp = localDW->rtu_TVS_Information_idx_2 -
    (localDW->rtu_TVS_Information_idx_2 - rtConstP.pooled6[(int32_T)
     localDW->Minimum1[1] - 1]) * localDW->rtb_Sum4_idx_1 * 0.095492965964253843;

  /* Switch: '<S15>/Switch2' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Gain: '<S16>/Gain8'
   *  Sum: '<S15>/Sum2'
   */
  localDW->Gain8[1] = localDW->rtb_Gain8_tmp;
  if (!(localDW->rtb_Gain8_tmp > 1.0)) {
    localDW->Gain8[1] = localDW->AddConstant[1];
  }

  /* Selector: '<S15>/Selector3' incorporates:
   *  Constant: '<S15>/Constant8'
   *  S-Function (sdspstatminmax): '<S15>/Minimum1'
   *  Sum: '<S15>/Sum2'
   */
  localDW->rtb_Gain8_tmp = rtConstP.pooled6[(int32_T)localDW->Minimum1[2] - 1];

  /* Switch: '<S15>/Switch2' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S15>/Constant7'
   *  Constant: '<S15>/Constant8'
   *  Gain: '<S15>/Gain2'
   *  Gain: '<S16>/Gain8'
   *  Product: '<S15>/Product1'
   *  Selector: '<S15>/Selector2'
   *  Selector: '<S15>/Selector3'
   *  Sum: '<S15>/Sum2'
   *  Sum: '<S15>/Sum3'
   *  Sum: '<S15>/Sum4'
   */
  localDW->Gain8[2] = localDW->CCaller_o2 - (localDW->CCaller_o2 -
    localDW->rtb_Gain8_tmp) * localDW->rtb_Sum4_idx_2 * 0.095492965964253843;
  if (!(localDW->CCaller_o2 - (localDW->CCaller_o2 - localDW->rtb_Gain8_tmp) *
        localDW->rtb_Sum4_idx_2 * 0.095492965964253843 > 1.0)) {
    localDW->Gain8[2] = localDW->AddConstant[2];
  }

  /* Selector: '<S15>/Selector3' incorporates:
   *  Constant: '<S15>/Constant8'
   *  S-Function (sdspstatminmax): '<S15>/Minimum1'
   *  Sum: '<S15>/Sum2'
   */
  localDW->rtb_Gain8_tmp = rtConstP.pooled6[(int32_T)localDW->Minimum1[3] - 1];

  /* Sum: '<S15>/Sum2' incorporates:
   *  Constant: '<S15>/Constant8'
   *  Gain: '<S15>/Gain2'
   *  Product: '<S15>/Product1'
   *  Selector: '<S15>/Selector3'
   *  Sum: '<S15>/Sum3'
   *  Sum: '<S15>/Sum4'
   */
  localDW->rtu_TVS_Information_idx_2 = localDW->Square - (localDW->Square -
    localDW->rtb_Gain8_tmp) * localDW->Square1 * 0.095492965964253843;

  /* Switch: '<S15>/Switch2' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S15>/Constant7'
   *  Gain: '<S15>/Gain2'
   *  Gain: '<S16>/Gain8'
   *  Product: '<S15>/Product1'
   *  Selector: '<S15>/Selector2'
   *  Sum: '<S15>/Sum2'
   *  Sum: '<S15>/Sum3'
   *  Sum: '<S15>/Sum4'
   */
  if (!(localDW->Square - (localDW->Square - localDW->rtb_Gain8_tmp) *
        localDW->Square1 * 0.095492965964253843 > 1.0)) {
    localDW->rtu_TVS_Information_idx_2 = localDW->AddConstant[3];
  }

  /* Switch: '<S16>/Switch1' incorporates:
   *  Constant: '<S16>/Constant'
   */
  if (rtu_TVS_Information_d[0] >= 1.4) {
    /* Saturate: '<S16>/Min Torque' incorporates:
     *  Gain: '<S16>/Gain4'
     *  Switch: '<S15>/Switch2'
     *  Switch: '<S16>/Switch1'
     */
    if (-0.0 * localDW->Gain8[0] > -0.01) {
      localDW->Minimum1[0] = -0.01;
    } else {
      localDW->Minimum1[0] = (rtNaN);
    }

    if (-0.0 * localDW->Gain8[1] > -0.01) {
      localDW->Minimum1[1] = -0.01;
    } else {
      localDW->Minimum1[1] = (rtNaN);
    }

    if (-localDW->Gain8[2] > -0.01) {
      localDW->Minimum1[2] = -0.01;
    } else if (-localDW->Gain8[2] < -0.01) {
      localDW->Minimum1[2] = -0.01;
    } else {
      localDW->Minimum1[2] = -localDW->Gain8[2];
    }

    if (-localDW->rtu_TVS_Information_idx_2 > -0.01) {
      localDW->Minimum1[3] = -0.01;
    } else if (-localDW->rtu_TVS_Information_idx_2 < -0.01) {
      localDW->Minimum1[3] = -0.01;
    } else {
      localDW->Minimum1[3] = -localDW->rtu_TVS_Information_idx_2;
    }

    /* End of Saturate: '<S16>/Min Torque' */
  } else {
    localDW->Minimum1[0] = 0.0;
    localDW->Minimum1[1] = 0.0;
    localDW->Minimum1[2] = 0.0;
    localDW->Minimum1[3] = 0.0;
  }

  /* End of Switch: '<S16>/Switch1' */

  /* DeadZone: '<S11>/Dead Zone' */
  if (rtu_TVS_Information_c > rtP.deadband_angle) {
    localDW->CCaller_o5 = rtu_TVS_Information_c - rtP.deadband_angle;
  } else if (rtu_TVS_Information_c >= -rtP.deadband_angle) {
    localDW->CCaller_o5 = 0.0;
  } else {
    localDW->CCaller_o5 = rtu_TVS_Information_c - (-rtP.deadband_angle);
  }

  /* End of DeadZone: '<S11>/Dead Zone' */

  /* Gain: '<S11>/Gain' */
  localDW->CCaller_o5 *= 0.28194;

  /* Math: '<S27>/Square1' */
  localDW->Square1 = localDW->CCaller_o5 * localDW->CCaller_o5;

  /* Product: '<S27>/Product' */
  localDW->Square1 *= localDW->CCaller_o5;

  /* Gain: '<S27>/Gain' */
  localDW->Square1 *= 7.0E-5;

  /* Math: '<S27>/Square' */
  localDW->Square = localDW->CCaller_o5 * localDW->CCaller_o5;

  /* Gain: '<S27>/Gain1' */
  localDW->Square *= 0.0038;

  /* Gain: '<S27>/Gain2' */
  localDW->CCaller_o2 = 0.6535 * localDW->CCaller_o5;

  /* Sum: '<S27>/Sum of Elements' */
  localDW->AddConstant[0] = localDW->Square1;
  localDW->AddConstant[1] = localDW->Square;
  localDW->AddConstant[2] = localDW->CCaller_o2;
  localDW->Square1 = -0.0;
  localDW->Square1 += localDW->AddConstant[0];
  localDW->Square1 += localDW->AddConstant[1];
  localDW->Square1 += localDW->AddConstant[2];
  localDW->Square1 += -0.1061;

  /* Math: '<S28>/Square1' */
  localDW->Square = localDW->CCaller_o5 * localDW->CCaller_o5;

  /* Product: '<S28>/Product' */
  localDW->Square *= localDW->CCaller_o5;

  /* Gain: '<S28>/Gain' */
  localDW->Square *= 7.0E-5;

  /* Math: '<S28>/Square' */
  localDW->rtb_Sum4_idx_2 = localDW->CCaller_o5 * localDW->CCaller_o5;

  /* Gain: '<S28>/Gain1' */
  localDW->CCaller_o2 = -0.0038 * localDW->rtb_Sum4_idx_2;

  /* Gain: '<S28>/Gain2' */
  localDW->CCaller_o5 *= 0.6535;

  /* Sum: '<S28>/Sum of Elements' */
  localDW->AddConstant[0] = localDW->Square;
  localDW->AddConstant[1] = localDW->CCaller_o2;
  localDW->AddConstant[2] = localDW->CCaller_o5;
  localDW->Square = -0.0;
  localDW->Square += localDW->AddConstant[0];
  localDW->Square += localDW->AddConstant[1];
  localDW->Square += localDW->AddConstant[2];
  localDW->Square += 0.1061;

  /* SignalConversion generated from: '<S11>/Mean' */
  rtb_Abs_m_idx_0 = localDW->Square1;
  rtb_Abs_m_idx_1 = localDW->Square;

  /* S-Function (sdspstatfcns): '<S11>/Mean' */
  localDW->CCaller_o5 = rtb_Abs_m_idx_0;
  localDW->CCaller_o5 += rtb_Abs_m_idx_1;
  localDW->CCaller_o5 /= 2.0;

  /* S-Function (sdspstatfcns): '<S17>/Mean' incorporates:
   *  UnitDelay: '<S2>/Unit Delay'
   */
  localDW->Square1 = localDW->UnitDelay_DSTATE[0];
  localDW->Square1 += localDW->UnitDelay_DSTATE[1];
  localDW->Square1 += localDW->UnitDelay_DSTATE[2];
  localDW->Square1 += localDW->UnitDelay_DSTATE[3];
  localDW->Square1 /= 4.0;

  /* Product: '<S17>/Divide' */
  localDW->Square = localDW->Square1 / 2.0;

  /* Gain: '<S17>/Gain' */
  rtb_UnitDelay_idx_0 = 0.0 * localDW->Square;
  localDW->rtb_UnitDelay_idx_1 = 0.0 * localDW->Square;
  localDW->rtb_UnitDelay_idx_2 = localDW->Square;
  localDW->rtb_UnitDelay_idx_3 = localDW->Square;

  /* Gain: '<S17>/Gain2' */
  localDW->rtb_Sum4_idx_0 = 0.0 * localDW->Square1;
  localDW->rtb_Sum4_idx_1 = 0.0 * localDW->Square1;
  localDW->rtb_Sum4_idx_2 = localDW->Square1;

  /* Sum: '<S17>/Add' incorporates:
   *  Gain: '<S17>/Gain'
   *  Gain: '<S17>/Gain2'
   */
  localDW->rtb_Gain6_idx_1 = localDW->rtb_Sum4_idx_2 +
    localDW->rtb_UnitDelay_idx_2;

  /* Saturate: '<S16>/Max Torque' incorporates:
   *  Gain: '<S16>/Gain8'
   *  Switch: '<S15>/Switch2'
   */
  if (localDW->Gain8[2] > 25.0) {
    localDW->rtb_Gain8_tmp = 25.0;
  } else if (localDW->Gain8[2] < 0.01) {
    localDW->rtb_Gain8_tmp = 0.01;
  } else {
    localDW->rtb_Gain8_tmp = localDW->Gain8[2];
  }

  /* Sum: '<S17>/Add' incorporates:
   *  Gain: '<S17>/Gain'
   *  Gain: '<S17>/Gain2'
   */
  localDW->rtb_Sum4_idx_2 = localDW->Square1 + localDW->rtb_UnitDelay_idx_3;

  /* Saturate: '<S16>/Max Torque' */
  if (localDW->rtu_TVS_Information_idx_2 > 25.0) {
    localDW->rtu_TVS_Information_idx_2 = 25.0;
  } else if (localDW->rtu_TVS_Information_idx_2 < 0.01) {
    localDW->rtu_TVS_Information_idx_2 = 0.01;
  }

  /* Gain: '<S11>/Gain1' */
  localDW->Square1 = 0.01745329 * rtb_Abs_m_idx_0;

  /* Gain: '<S13>/Gain' incorporates:
   *  Abs: '<S13>/Abs'
   */
  localDW->Square = 0.15915494309189535 * fabs(localDW->Square1);

  /* DataTypeConversion: '<S21>/CastU16En16' */
  rtb_QuadHandle1b = (uint32_T)(localDW->Square * 4.294967296E+9);

  /* RelationalOperator: '<S21>/LTEp25' */
  rtb_LessThan_e = (rtb_QuadHandle1b <= 1073741824U);

  /* RelationalOperator: '<S21>/GTEp75' */
  rtb_LowerRelop1 = (rtb_QuadHandle1b >= 3221225472U);

  /* Switch: '<S21>/QuadHandle2' incorporates:
   *  Constant: '<S21>/Point75'
   *  RelationalOperator: '<S21>/LTEp50'
   *  Sum: '<S21>/p75mA'
   *  Switch: '<S21>/QuadHandle1b'
   */
  if (rtb_QuadHandle1b <= 2147483648U) {
    /* Switch: '<S21>/QuadHandle1a' incorporates:
     *  Constant: '<S21>/Point25'
     *  Sum: '<S21>/Amp25'
     *  Sum: '<S21>/p25mA'
     */
    if (rtb_LessThan_e) {
      rtb_QuadHandle1b = 1073741824U - rtb_QuadHandle1b;
    } else {
      rtb_QuadHandle1b -= 1073741824U;
    }
  } else if (rtb_LowerRelop1) {
    /* Switch: '<S21>/QuadHandle1b' incorporates:
     *  Constant: '<S21>/Point75'
     *  Sum: '<S21>/Amp75'
     */
    rtb_QuadHandle1b -= 3221225472U;
  } else {
    rtb_QuadHandle1b = 3221225472U - rtb_QuadHandle1b;
  }

  /* Gain: '<S11>/Gain1' */
  rtb_Abs_m_idx_0 = localDW->Square1;

  /* RelationalOperator: '<S21>/LTEp25' */
  rtb_LTEp25_p_idx_0 = rtb_LessThan_e;

  /* RelationalOperator: '<S21>/GTEp75' */
  rtb_LTEp50_m_idx_0 = rtb_LowerRelop1;

  /* Gain: '<S11>/Gain1' */
  localDW->Square1 = 0.01745329 * rtb_Abs_m_idx_1;

  /* DataTypeConversion: '<S21>/CastU16En16' incorporates:
   *  Abs: '<S13>/Abs'
   *  DataTypeConversion: '<S23>/CastU16En16'
   *  Gain: '<S13>/Gain'
   */
  rtb_QuadHandle1b_tmp = (uint32_T)(0.15915494309189535 * fabs(localDW->Square1)
    * 4.294967296E+9);

  /* RelationalOperator: '<S21>/LTEp25' incorporates:
   *  DataTypeConversion: '<S21>/CastU16En16'
   */
  rtb_LessThan_e = (rtb_QuadHandle1b_tmp <= 1073741824U);

  /* RelationalOperator: '<S21>/GTEp75' incorporates:
   *  DataTypeConversion: '<S21>/CastU16En16'
   */
  rtb_LowerRelop1 = (rtb_QuadHandle1b_tmp >= 3221225472U);

  /* Lookup_n-D: '<S20>/Look-Up Table' incorporates:
   *  SignalConversion generated from: '<S13>/Dot Product2'
   */
  rtb_QuadHandle1b_idx_0 = plook_u32u32u32n31_even20c_gf(rtb_QuadHandle1b, 0U,
    1024U, &frac);
  rtb_TmpSignalConversionAtDotP_0 = intrp1d_s32s64s64u32u32n31l_f
    (rtb_QuadHandle1b_idx_0, frac, rtConstP.pooled18);

  /* Switch: '<S21>/QuadHandle2' incorporates:
   *  Constant: '<S21>/Point75'
   *  DataTypeConversion: '<S21>/CastU16En16'
   *  RelationalOperator: '<S21>/LTEp50'
   *  Sum: '<S21>/p75mA'
   *  Switch: '<S21>/QuadHandle1b'
   */
  if (rtb_QuadHandle1b_tmp <= 2147483648U) {
    /* Switch: '<S21>/QuadHandle1a' incorporates:
     *  Constant: '<S21>/Point25'
     *  Sum: '<S21>/Amp25'
     *  Sum: '<S21>/p25mA'
     */
    if (rtb_LessThan_e) {
      rtb_QuadHandle1b = 1073741824U - rtb_QuadHandle1b_tmp;
    } else {
      rtb_QuadHandle1b = rtb_QuadHandle1b_tmp - 1073741824U;
    }
  } else if (rtb_LowerRelop1) {
    /* Switch: '<S21>/QuadHandle1b' incorporates:
     *  Constant: '<S21>/Point75'
     *  Sum: '<S21>/Amp75'
     */
    rtb_QuadHandle1b = rtb_QuadHandle1b_tmp - 3221225472U;
  } else {
    rtb_QuadHandle1b = 3221225472U - rtb_QuadHandle1b_tmp;
  }

  /* Lookup_n-D: '<S20>/Look-Up Table' incorporates:
   *  SignalConversion generated from: '<S13>/Dot Product2'
   */
  rtb_QuadHandle1b_idx_0 = plook_u32u32u32n31_even20c_gf(rtb_QuadHandle1b, 0U,
    1024U, &frac);
  rtb_TmpSignalConversionAtDotP_1 = intrp1d_s32s64s64u32u32n31l_f
    (rtb_QuadHandle1b_idx_0, frac, rtConstP.pooled18);

  /* Switch: '<S21>/SignCorrected' incorporates:
   *  Logic: '<S21>/1st or 4th Quad'
   *  SignalConversion generated from: '<S13>/Dot Product2'
   *  UnaryMinus: '<S21>/Negate'
   */
  i = rtb_TmpSignalConversionAtDotP_0;
  if ((!rtb_LTEp25_p_idx_0) && (!rtb_LTEp50_m_idx_0)) {
    i = -rtb_TmpSignalConversionAtDotP_0;
  }

  /* DataTypeConversion: '<S23>/CastU16En16' */
  rtb_QuadHandle1b = (uint32_T)(localDW->Square * 4.294967296E+9);

  /* RelationalOperator: '<S23>/LTEp50' */
  rtb_LTEp25_p_idx_0 = (rtb_QuadHandle1b <= 2147483648U);

  /* Switch: '<S23>/QuadHandle1' incorporates:
   *  Constant: '<S23>/Point50'
   *  Sum: '<S23>/Amp50'
   */
  if (!rtb_LTEp25_p_idx_0) {
    rtb_QuadHandle1b -= 2147483648U;
  }

  /* Switch: '<S23>/QuadHandle2' incorporates:
   *  Constant: '<S23>/Point50'
   *  RelationalOperator: '<S23>/LTEp25'
   *  Sum: '<S23>/p50mA'
   */
  if (rtb_QuadHandle1b > 1073741824U) {
    rtb_QuadHandle1b = 2147483648U - rtb_QuadHandle1b;
  }

  /* RelationalOperator: '<S23>/LTEp50' */
  rtb_LTEp50_m_idx_0 = rtb_LTEp25_p_idx_0;

  /* Switch: '<S21>/SignCorrected' incorporates:
   *  SignalConversion generated from: '<S13>/Dot Product2'
   */
  rtb_TmpSignalConversionAtDotP_0 = i;

  /* DataTypeConversion: '<S23>/CastU16En16' incorporates:
   *  Switch: '<S23>/QuadHandle2'
   */
  rtb_QuadHandle1b_idx_0 = rtb_QuadHandle1b;

  /* Switch: '<S21>/SignCorrected' incorporates:
   *  Logic: '<S21>/1st or 4th Quad'
   *  SignalConversion generated from: '<S13>/Dot Product2'
   *  UnaryMinus: '<S21>/Negate'
   */
  i = rtb_TmpSignalConversionAtDotP_1;
  if ((!rtb_LessThan_e) && (!rtb_LowerRelop1)) {
    i = -rtb_TmpSignalConversionAtDotP_1;
  }

  /* DataTypeConversion: '<S23>/CastU16En16' */
  rtb_QuadHandle1b = rtb_QuadHandle1b_tmp;

  /* RelationalOperator: '<S23>/LTEp50' */
  rtb_LTEp25_p_idx_0 = (rtb_QuadHandle1b_tmp <= 2147483648U);

  /* Switch: '<S23>/QuadHandle1' incorporates:
   *  Constant: '<S23>/Point50'
   *  Sum: '<S23>/Amp50'
   */
  if (!rtb_LTEp25_p_idx_0) {
    rtb_QuadHandle1b = rtb_QuadHandle1b_tmp - 2147483648U;
  }

  /* Switch: '<S23>/QuadHandle2' incorporates:
   *  Constant: '<S23>/Point50'
   *  RelationalOperator: '<S23>/LTEp25'
   *  Sum: '<S23>/p50mA'
   */
  if (rtb_QuadHandle1b > 1073741824U) {
    rtb_QuadHandle1b = 2147483648U - rtb_QuadHandle1b;
  }

  /* Lookup_n-D: '<S22>/Look-Up Table' incorporates:
   *  SignalConversion generated from: '<S13>/Dot Product1'
   *  Switch: '<S23>/QuadHandle2'
   */
  rtb_QuadHandle1b_idx_0 = plook_u32u32u64n32_even20c_gf(rtb_QuadHandle1b_idx_0,
    0U, 1024U, &localDW->frac);
  rtb_TmpSignalConversionAtDotP_1 = intrp1d_s32s64s64u32u64n32l_f
    (rtb_QuadHandle1b_idx_0, localDW->frac, rtConstP.pooled18);
  rtb_QuadHandle1b_idx_0 = plook_u32u32u64n32_even20c_gf(rtb_QuadHandle1b, 0U,
    1024U, &localDW->frac);

  /* Switch: '<S23>/SignCorrected' incorporates:
   *  SignalConversion generated from: '<S13>/Dot Product1'
   *  UnaryMinus: '<S23>/Negate'
   */
  if (!rtb_LTEp50_m_idx_0) {
    rtb_TmpSignalConversionAtDotP_1 = -rtb_TmpSignalConversionAtDotP_1;
  }

  /* Signum: '<S13>/Sign' */
  if (rtIsNaN(rtb_Abs_m_idx_0)) {
    localDW->CCaller_o2 = rtb_Abs_m_idx_0;
  } else if (rtb_Abs_m_idx_0 < 0.0) {
    localDW->CCaller_o2 = -1.0;
  } else {
    localDW->CCaller_o2 = (rtb_Abs_m_idx_0 > 0.0);
  }

  /* DotProduct: '<S13>/Dot Product1' incorporates:
   *  Constant: '<S13>/Constant5'
   *  Product: '<S13>/Product1'
   *  SignalConversion generated from: '<S13>/Dot Product1'
   */
  localDW->rtb_TmpSignalConversionAtDotP_m = (real_T)(int32_T)floor((real_T)
    rtb_TmpSignalConversionAtDotP_1 * 9.3132257461547852E-10 *
    localDW->CCaller_o2 * 1.073741824E+9) * 9.3132257461547852E-10 * 0.7922471 +
    (real_T)rtb_TmpSignalConversionAtDotP_0 * 9.3132257461547852E-10 * 0.647895;

  /* Switch: '<S23>/SignCorrected' incorporates:
   *  Lookup_n-D: '<S22>/Look-Up Table'
   *  UnaryMinus: '<S23>/Negate'
   */
  if (rtb_LTEp25_p_idx_0) {
    rtb_TmpSignalConversionAtDotP_0 = intrp1d_s32s64s64u32u64n32l_f
      (rtb_QuadHandle1b_idx_0, localDW->frac, rtConstP.pooled18);
  } else {
    rtb_TmpSignalConversionAtDotP_0 = -intrp1d_s32s64s64u32u64n32l_f
      (rtb_QuadHandle1b_idx_0, localDW->frac, rtConstP.pooled18);
  }

  /* Signum: '<S13>/Sign' */
  if (rtIsNaN(localDW->Square1)) {
  } else if (localDW->Square1 < 0.0) {
    localDW->Square1 = -1.0;
  } else {
    localDW->Square1 = (localDW->Square1 > 0.0);
  }

  /* DotProduct: '<S13>/Dot Product2' incorporates:
   *  Constant: '<S13>/Constant6'
   *  Product: '<S13>/Product1'
   */
  localDW->CCaller_o2 = (real_T)(int32_T)floor((real_T)
    rtb_TmpSignalConversionAtDotP_0 * 9.3132257461547852E-10 * localDW->Square1 *
    1.073741824E+9) * 9.3132257461547852E-10 * 0.7922471 + (real_T)i *
    9.3132257461547852E-10 * -0.647895;

  /* Math: '<S10>/Square' */
  rtb_Abs_m_idx_0 = rtu_TVS_Information_d[0] * rtu_TVS_Information_d[0];
  rtb_Abs_m_idx_1 = rtu_TVS_Information_d[1] * rtu_TVS_Information_d[1];

  /* Sum: '<S10>/Sum' incorporates:
   *  Math: '<S10>/Square'
   */
  localDW->Square1 = -0.0;
  localDW->Square1 += rtb_Abs_m_idx_0;
  localDW->Square1 += rtb_Abs_m_idx_1;

  /* Sqrt: '<S10>/Sqrt' */
  localDW->Square1 = sqrt(localDW->Square1);

  /* Gain: '<S10>/Gain' */
  localDW->Square = rtP.Ku * localDW->Square1;

  /* Lookup_n-D: '<S10>/1-D Lookup Table' incorporates:
   *  CCaller: '<S5>/C Caller'
   */
  rtb_QuadHandle1b_idx_0 = plook_evenc(localDW->Square, 0.0, 3.0, 9U,
    &rtb_Abs_m_idx_0);
  localDW->Square = intrp1d_l(rtb_QuadHandle1b_idx_0, rtb_Abs_m_idx_0,
    rtConstP.uDLookupTable_tableData_h);

  /* Gain: '<S11>/Gain2' */
  localDW->CCaller_o5 *= 0.01745329;

  /* Product: '<S10>/Divide1' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  localDW->Square1 = localDW->Square1 * localDW->CCaller_o5 *
    0.63488032505872649;

  /* Abs: '<S10>/Abs' */
  rtb_Abs_m_idx_0 = localDW->Square;
  rtb_Abs_m_idx_1 = fabs(localDW->Square1);

  /* MinMax: '<S10>/Min of Elements' */
  localDW->CCaller_o5 = rtb_Abs_m_idx_0;
  if ((localDW->CCaller_o5 <= rtb_Abs_m_idx_1) || rtIsNaN(rtb_Abs_m_idx_1)) {
  } else {
    localDW->CCaller_o5 = rtb_Abs_m_idx_1;
  }

  /* End of MinMax: '<S10>/Min of Elements' */

  /* Signum: '<S10>/Sign' */
  if (rtIsNaN(localDW->Square1)) {
  } else if (localDW->Square1 < 0.0) {
    localDW->Square1 = -1.0;
  } else {
    localDW->Square1 = (localDW->Square1 > 0.0);
  }

  /* End of Signum: '<S10>/Sign' */

  /* Product: '<S10>/Product' */
  rtb_Abs_m_idx_0 = localDW->CCaller_o5 * localDW->Square1;

  /* Switch: '<S17>/Switch1' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S13>/Constant3'
   *  Constant: '<S13>/Constant4'
   *  Constant: '<S17>/Constant'
   *  DotProduct: '<S13>/Dot Product1'
   *  Gain: '<S17>/Gain'
   *  Gain: '<S17>/Gain2'
   *  Gain: '<S17>/Gain7'
   *  Product: '<S17>/Product1'
   *  Sum: '<S17>/Add'
   *  Switch: '<S25>/Switch'
   */
  if (rtb_Abs_m_idx_0 != 0.0) {
    localDW->AddConstant[0] = (localDW->rtb_Sum4_idx_0 + rtb_UnitDelay_idx_0) *
      localDW->rtb_TmpSignalConversionAtDotP_m * 0.38765914066666662;
    localDW->AddConstant[1] = localDW->CCaller_o2 * localDW->rtb_UnitDelay_idx_1
      * 0.38765914066666662;

    /* Switch: '<S25>/Switch2' incorporates:
     *  Bias: '<S5>/Add Constant'
     *  DotProduct: '<S13>/Dot Product1'
     *  Gain: '<S17>/Gain'
     *  Gain: '<S17>/Gain2'
     *  Gain: '<S17>/Gain7'
     *  Product: '<S17>/Product1'
     *  RelationalOperator: '<S25>/LowerRelop1'
     *  RelationalOperator: '<S25>/UpperRelop'
     *  Saturate: '<S16>/Max Torque'
     *  Sum: '<S17>/Add'
     *  Switch: '<S16>/Switch1'
     *  Switch: '<S25>/Switch'
     */
    if (localDW->rtb_Gain6_idx_1 > localDW->rtb_Gain8_tmp) {
      localDW->rtb_Gain6_idx_1 = localDW->rtb_Gain8_tmp;
    } else if (localDW->rtb_Gain6_idx_1 < localDW->Minimum1[2]) {
      localDW->rtb_Gain6_idx_1 = localDW->Minimum1[2];
    }

    localDW->AddConstant[2] = 0.62102 * localDW->rtb_Gain6_idx_1 * 0.387634;

    /* Switch: '<S25>/Switch2' incorporates:
     *  Bias: '<S5>/Add Constant'
     *  Constant: '<S13>/Constant3'
     *  Gain: '<S17>/Gain7'
     *  Product: '<S17>/Product1'
     *  RelationalOperator: '<S25>/LowerRelop1'
     *  RelationalOperator: '<S25>/UpperRelop'
     *  Saturate: '<S16>/Max Torque'
     *  Switch: '<S16>/Switch1'
     *  Switch: '<S25>/Switch'
     */
    if (localDW->rtb_UnitDelay_idx_3 > localDW->rtu_TVS_Information_idx_2) {
      localDW->CCaller_o5 = localDW->rtu_TVS_Information_idx_2;
    } else if (localDW->rtb_UnitDelay_idx_3 < localDW->Minimum1[3]) {
      /* Switch: '<S25>/Switch' incorporates:
       *  Switch: '<S16>/Switch1'
       */
      localDW->CCaller_o5 = localDW->Minimum1[3];
    } else {
      localDW->CCaller_o5 = localDW->rtb_UnitDelay_idx_3;
    }

    localDW->AddConstant[3] = -0.62102 * localDW->CCaller_o5 * 0.387634;
  } else {
    localDW->AddConstant[0] = 0.0;
    localDW->AddConstant[1] = 0.0;
    localDW->AddConstant[2] = 0.0;
    localDW->AddConstant[3] = 0.0;
  }

  /* End of Switch: '<S17>/Switch1' */

  /* Gain: '<S17>/Gain6' incorporates:
   *  DotProduct: '<S13>/Dot Product1'
   */
  localDW->rtb_Sum4_idx_0 = 0.38765914066666662 *
    localDW->rtb_TmpSignalConversionAtDotP_m;
  localDW->rtb_Gain6_idx_1 = 0.38765914066666662 * localDW->CCaller_o2;

  /* Gain: '<S14>/Gain5' */
  localDW->rtb_UnitDelay_idx_3 = 0.95 * rty_Tx[0];
  rtb_Abs_m_idx_1 = 0.95 * rty_Tx[1];
  localDW->rtb_Gain5_idx_2 = 0.95 * rty_Tx[2];
  localDW->rtb_Gain5_idx_3 = 0.95 * rty_Tx[3];

  /* Sum: '<S9>/Sum1' */
  localDW->CCaller_o5 = rtb_Abs_m_idx_0 - rtu_TVS_Information_b[2];

  /* Abs: '<S9>/Abs' */
  localDW->Square1 = fabs(localDW->CCaller_o5);

  /* RelationalOperator: '<S9>/Less Than' incorporates:
   *  Constant: '<S9>/Constant'
   */
  rtb_LessThan_e = (localDW->Square1 < 0.5);

  /* Product: '<S9>/Product' incorporates:
   *  UnitDelay: '<S17>/Unit Delay'
   */
  localDW->Square1 = (localDW->UnitDelay_DSTATE_e ? (real_T)rtb_LessThan_e : 0.0)
    * localDW->CCaller_o5;

  /* Gain: '<S9>/Gain' */
  localDW->Square1 *= rtP.I;

  /* UnitDelay: '<S17>/Unit Delay4' */
  rtb_LessThan_e = localDW->UnitDelay4_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  UnitDelay: '<S17>/Unit Delay4'
   */
  if (localDW->UnitDelay4_DSTATE || (localDW->DiscreteTimeIntegrator_PrevRese !=
       0)) {
    localDW->DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  localDW->Square = 0.015 * localDW->Square1 +
    localDW->DiscreteTimeIntegrator_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  if (localDW->Square >= 1.0) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
    localDW->Square = 1.0;
  } else if (localDW->Square <= -1.0) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
    localDW->Square = -1.0;
  }

  /* Gain: '<S9>/P' */
  localDW->Square1 = rtP.P * localDW->CCaller_o5;

  /* Sum: '<S9>/Sum7' */
  localDW->Sum7 = localDW->Square + localDW->Square1;

  /* Sum: '<S17>/Sum1' incorporates:
   *  Bias: '<S5>/Add Constant'
   */
  localDW->CCaller_o5 = -0.0;
  localDW->CCaller_o5 += localDW->AddConstant[0];
  localDW->CCaller_o5 += localDW->AddConstant[1];
  localDW->CCaller_o5 += localDW->AddConstant[2];
  localDW->CCaller_o5 += localDW->AddConstant[3];

  /* RelationalOperator: '<S24>/LowerRelop1' */
  rtb_LowerRelop1 = (localDW->Sum7 > localDW->CCaller_o5);

  /* Switch: '<S24>/Switch2' */
  if (!rtb_LowerRelop1) {
    /* Switch: '<S17>/Switch2' incorporates:
     *  Constant: '<S13>/Constant3'
     *  Constant: '<S13>/Constant4'
     *  Constant: '<S17>/Constant'
     *  DotProduct: '<S13>/Dot Product1'
     *  Gain: '<S17>/Gain'
     *  Gain: '<S17>/Gain1'
     *  Gain: '<S17>/Gain2'
     *  Product: '<S17>/Product2'
     *  Sum: '<S17>/Add'
     *  Switch: '<S26>/Switch'
     */
    if (rtb_Abs_m_idx_0 != 0.0) {
      localDW->AddConstant[0] = localDW->rtb_TmpSignalConversionAtDotP_m *
        rtb_UnitDelay_idx_0 * 0.38765914066666662;
      localDW->AddConstant[1] = (localDW->rtb_Sum4_idx_1 +
        localDW->rtb_UnitDelay_idx_1) * localDW->CCaller_o2 *
        0.38765914066666662;

      /* Switch: '<S26>/Switch2' incorporates:
       *  DotProduct: '<S13>/Dot Product1'
       *  Gain: '<S17>/Gain'
       *  Gain: '<S17>/Gain1'
       *  Gain: '<S17>/Gain2'
       *  Product: '<S17>/Product2'
       *  RelationalOperator: '<S26>/LowerRelop1'
       *  RelationalOperator: '<S26>/UpperRelop'
       *  Saturate: '<S16>/Max Torque'
       *  Sum: '<S17>/Add'
       *  Switch: '<S16>/Switch1'
       *  Switch: '<S17>/Switch2'
       *  Switch: '<S26>/Switch'
       */
      if (localDW->rtb_UnitDelay_idx_2 > localDW->rtb_Gain8_tmp) {
        localDW->CCaller_o5 = localDW->rtb_Gain8_tmp;
      } else if (localDW->rtb_UnitDelay_idx_2 < localDW->Minimum1[2]) {
        localDW->CCaller_o5 = localDW->Minimum1[2];
      } else {
        localDW->CCaller_o5 = localDW->rtb_UnitDelay_idx_2;
      }

      localDW->AddConstant[2] = 0.62102 * localDW->CCaller_o5 * 0.387634;

      /* Switch: '<S26>/Switch2' incorporates:
       *  Constant: '<S13>/Constant3'
       *  Gain: '<S17>/Gain1'
       *  Product: '<S17>/Product2'
       *  RelationalOperator: '<S26>/LowerRelop1'
       *  RelationalOperator: '<S26>/UpperRelop'
       *  Saturate: '<S16>/Max Torque'
       *  Switch: '<S16>/Switch1'
       *  Switch: '<S17>/Switch2'
       *  Switch: '<S26>/Switch'
       */
      if (localDW->rtb_Sum4_idx_2 > localDW->rtu_TVS_Information_idx_2) {
        localDW->rtb_Sum4_idx_2 = localDW->rtu_TVS_Information_idx_2;
      } else if (localDW->rtb_Sum4_idx_2 < localDW->Minimum1[3]) {
        /* Switch: '<S26>/Switch' incorporates:
         *  Switch: '<S16>/Switch1'
         */
        localDW->rtb_Sum4_idx_2 = localDW->Minimum1[3];
      }

      localDW->AddConstant[3] = -0.62102 * localDW->rtb_Sum4_idx_2 * 0.387634;
    } else {
      localDW->AddConstant[0] = 0.0;
      localDW->AddConstant[1] = 0.0;
      localDW->AddConstant[2] = 0.0;
      localDW->AddConstant[3] = 0.0;
    }

    /* End of Switch: '<S17>/Switch2' */

    /* Sum: '<S17>/Sum2' incorporates:
     *  Switch: '<S17>/Switch2'
     */
    localDW->rtb_Sum4_idx_1 = ((localDW->AddConstant[0] + localDW->AddConstant[1])
      + localDW->AddConstant[2]) + localDW->AddConstant[3];

    /* Switch: '<S24>/Switch' incorporates:
     *  RelationalOperator: '<S24>/UpperRelop'
     */
    if (localDW->Sum7 < localDW->rtb_Sum4_idx_1) {
      localDW->CCaller_o5 = localDW->rtb_Sum4_idx_1;
    } else {
      localDW->CCaller_o5 = localDW->Sum7;
    }

    /* End of Switch: '<S24>/Switch' */
  }

  /* End of Switch: '<S24>/Switch2' */

  /* Abs: '<S17>/Abs3' */
  localDW->Square1 = fabs(localDW->CCaller_o5);

  /* RelationalOperator: '<S17>/Equal' incorporates:
   *  Constant: '<S17>/Constant4'
   */
  rtb_LowerRelop1 = (localDW->Square1 < 0.01);

  /* Switch: '<S17>/Switch' incorporates:
   *  Constant: '<S17>/Constant5'
   *  Constant: '<S17>/Constant6'
   */
  if (rtb_LowerRelop1) {
    localDW->Square1 = 0.0001;
  } else {
    localDW->Square1 = 0.0;
  }

  /* End of Switch: '<S17>/Switch' */

  /* Sum: '<S17>/Sum3' */
  localDW->rtb_Sum4_idx_1 = localDW->CCaller_o5 + localDW->Square1;

  /* Abs: '<S17>/Abs1' */
  localDW->CCaller_o5 = fabs(localDW->CCaller_o5);

  /* Abs: '<S17>/Abs2' */
  localDW->Square1 = fabs(localDW->Sum7);

  /* Update for UnitDelay: '<S17>/Unit Delay' incorporates:
   *  RelationalOperator: '<S17>/GreaterThanOrEqual'
   */
  localDW->UnitDelay_DSTATE_e = (localDW->CCaller_o5 < localDW->Square1);

  /* Abs: '<S17>/Abs' */
  localDW->Square1 = fabs(rtb_Abs_m_idx_0);

  /* UnitDelay: '<S17>/Unit Delay1' */
  localDW->CCaller_o5 = localDW->UnitDelay1_DSTATE;

  /* RelationalOperator: '<S17>/GreaterThan1' incorporates:
   *  Constant: '<S17>/Constant1'
   */
  rtb_LowerRelop1 = (localDW->CCaller_o5 > 0.05);

  /* Switch: '<S14>/Switch' incorporates:
   *  Product: '<S14>/Product'
   *  Product: '<S14>/Product1'
   *  SignalConversion generated from: '<S4>/driver_input'
   */
  if (rtu_TVS_Information_i > 0.0) {
    /* Saturate: '<S14>/Discharge Limits' */
    localDW->CCaller_o5 = rtu_TVS_Information_k[0];
    if (localDW->CCaller_o5 > 75000.0) {
      localDW->CCaller_o5 = 75000.0;
    } else if (localDW->CCaller_o5 < 0.0) {
      localDW->CCaller_o5 = 0.0;
    }

    /* End of Saturate: '<S14>/Discharge Limits' */
    localDW->rtb_Sum4_idx_2 = localDW->CCaller_o5 * rtu_TVS_Information_i;
  } else {
    /* Saturate: '<S14>/Charge Limits' */
    localDW->CCaller_o5 = rtu_TVS_Information_k[1];
    if (localDW->CCaller_o5 > 0.0) {
      localDW->CCaller_o5 = 0.0;
    } else if (localDW->CCaller_o5 < 0.0) {
      localDW->CCaller_o5 = 0.0;
    }

    /* End of Saturate: '<S14>/Charge Limits' */
    localDW->rtb_Sum4_idx_2 = rtu_TVS_Information_i * localDW->CCaller_o5;
  }

  /* End of Switch: '<S14>/Switch' */

  /* UnitDelay: '<S17>/Unit Delay5' */
  localDW->CCaller_o5 = localDW->UnitDelay5_DSTATE;

  /* Signum: '<S17>/Sign2' */
  if (rtIsNaN(localDW->CCaller_o5)) {
  } else if (localDW->CCaller_o5 < 0.0) {
    localDW->CCaller_o5 = -1.0;
  } else {
    localDW->CCaller_o5 = (localDW->CCaller_o5 > 0.0);
  }

  /* End of Signum: '<S17>/Sign2' */

  /* Signum: '<S17>/Sign' incorporates:
   *  Switch: '<S14>/Switch'
   */
  if (rtIsNaN(localDW->rtb_Sum4_idx_2)) {
    localDW->CCaller_o2 = localDW->rtb_Sum4_idx_2;
  } else if (localDW->rtb_Sum4_idx_2 < 0.0) {
    localDW->CCaller_o2 = -1.0;
  } else {
    localDW->CCaller_o2 = (localDW->rtb_Sum4_idx_2 > 0.0);
  }

  /* End of Signum: '<S17>/Sign' */

  /* Update for UnitDelay: '<S17>/Unit Delay4' incorporates:
   *  Constant: '<S17>/Constant2'
   *  Logic: '<S17>/NOT1'
   *  Logic: '<S17>/NOT2'
   *  Logic: '<S17>/OR'
   *  RelationalOperator: '<S17>/Equal1'
   *  RelationalOperator: '<S17>/Equal2'
   *  RelationalOperator: '<S17>/GreaterThan'
   */
  localDW->UnitDelay4_DSTATE = (((localDW->Square1 > 0.05) != (int32_T)
    rtb_LowerRelop1) || (!(localDW->CCaller_o2 == localDW->CCaller_o5)));

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_DSTATE = localDW->Square;
  localDW->DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_LessThan_e;

  /* Update for UnitDelay: '<S17>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE = localDW->Square1;

  /* Update for UnitDelay: '<S17>/Unit Delay5' incorporates:
   *  Switch: '<S14>/Switch'
   */
  localDW->UnitDelay5_DSTATE = localDW->rtb_Sum4_idx_2;

  /* Saturate: '<S16>/Max Torque' incorporates:
   *  Gain: '<S16>/Gain8'
   *  Switch: '<S15>/Switch2'
   */
  if (0.0 * localDW->Gain8[0] < 0.01) {
    localDW->CCaller_o2 = 0.01;
  } else {
    localDW->CCaller_o2 = (rtNaN);
  }

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* Bias: '<S5>/Add Constant1' */
  rty_Tx[0] = localDW->CCaller_o2 + 25.0;

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Saturate: '<S16>/Max Torque' incorporates:
   *  Gain: '<S16>/Gain8'
   *  Switch: '<S15>/Switch2'
   */
  if (0.0 * localDW->Gain8[1] < 0.01) {
    localDW->CCaller_o2 = 0.01;
  } else {
    localDW->CCaller_o2 = (rtNaN);
  }

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* Bias: '<S5>/Add Constant1' incorporates:
   *  Saturate: '<S16>/Max Torque'
   */
  rty_Tx[1] = localDW->CCaller_o2 + 25.0;

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  rty_Tx[2] = localDW->rtb_Gain8_tmp + 25.0;
  rty_Tx[3] = localDW->rtu_TVS_Information_idx_2 + 25.0;

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* CCaller: '<S5>/C Caller' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S4>/Constant3'
   *  Constant: '<S5>/Constant'
   *  Constant: '<S5>/Constant1'
   *  Constant: '<S5>/Constant2'
   *  Constant: '<S5>/Constant4'
   *  DotProduct: '<S5>/Dot Product'
   *  DotProduct: '<S5>/Dot Product1'
   *  Gain: '<S14>/Gain5'
   *  Gain: '<S17>/Gain6'
   *  Sum: '<S17>/Sum3'
   *  Sum: '<S5>/Add'
   *  Sum: '<S5>/Add1'
   *  Switch: '<S14>/Switch'
   *  Switch: '<S16>/Switch1'
   */
  localDW->CCaller_o2 = 0.0;
  localDW->Square = 0.0;
  localDW->Square1 = 0.0;
  localDW->CCaller_o5 = 0.0;
  *rty_bigM_flag = bigM_func(29.074435549999997, 29.074435549999997, 29.07255,
    29.07255, (((25.0 * localDW->rtb_UnitDelay_idx_3 + 25.0 * rtb_Abs_m_idx_1) +
                25.0 * localDW->rtb_Gain5_idx_2) + 25.0 *
               localDW->rtb_Gain5_idx_3) + localDW->rtb_Sum4_idx_2,
    localDW->rtb_UnitDelay_idx_3, rtb_Abs_m_idx_1, localDW->rtb_Gain5_idx_2,
    localDW->rtb_Gain5_idx_3, (((25.0 * localDW->rtb_Sum4_idx_0 + 25.0 *
    localDW->rtb_Gain6_idx_1) + 6.018211667) + -6.018211667) +
    localDW->rtb_Sum4_idx_1, localDW->rtb_Sum4_idx_0, localDW->rtb_Gain6_idx_1,
    0.24072846668, -0.24072846668, localDW->Minimum1[0] + 25.0,
    localDW->Minimum1[1] + 25.0, localDW->Minimum1[2] + 25.0, localDW->Minimum1
    [3] + 25.0, rty_Tx[0], rty_Tx[1], rty_Tx[2], rty_Tx[3], &localDW->CCaller_o2,
    &localDW->Square, &localDW->Square1, &localDW->CCaller_o5, 0.2);

  /* Switch: '<S6>/Switch' incorporates:
   *  Bias: '<S6>/Add Constant3'
   */
  if (*rty_bigM_flag > 2) {
    rty_Tx[0] = localDW->CCaller_o2;
    rty_Tx[1] = localDW->Square;
    rty_Tx[2] = localDW->Square1;
    rty_Tx[3] = localDW->CCaller_o5;
  } else {
    /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
    /* Bias: '<S6>/Add Constant3' incorporates:
     *  Gain: '<S6>/Gain'
     *  SignalConversion generated from: '<S4>/driver_input'
     */
    localDW->CCaller_o2 = 0.0 * rtu_TVS_Information_i + 25.0;

    /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */
    rty_Tx[0] = localDW->CCaller_o2;
    rty_Tx[1] = localDW->CCaller_o2;

    /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
    /* Bias: '<S6>/Add Constant3' incorporates:
     *  Gain: '<S6>/Gain'
     *  SignalConversion generated from: '<S4>/driver_input'
     */
    localDW->CCaller_o2 = 25.0 * rtu_TVS_Information_i + 25.0;

    /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */
    rty_Tx[2] = localDW->CCaller_o2;
    rty_Tx[3] = localDW->CCaller_o2;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Bias: '<S6>/Add Constant2' */
  rty_Tx[0] += -25.0;
  rty_Tx[1] += -25.0;
  rty_Tx[2] += -25.0;
  rty_Tx[3] += -25.0;

  /* DeadZone: '<S6>/Dead Zone' */
  if (rty_Tx[0] > 0.02) {
    rty_Tx[0] -= 0.02;
  } else if (rty_Tx[0] >= -0.02) {
    rty_Tx[0] = 0.0;
  } else {
    rty_Tx[0] -= -0.02;
  }

  if (rty_Tx[1] > 0.02) {
    rty_Tx[1] -= 0.02;
  } else if (rty_Tx[1] >= -0.02) {
    rty_Tx[1] = 0.0;
  } else {
    rty_Tx[1] -= -0.02;
  }

  if (rty_Tx[2] > 0.02) {
    rty_Tx[2] -= 0.02;
  } else if (rty_Tx[2] >= -0.02) {
    rty_Tx[2] = 0.0;
  } else {
    rty_Tx[2] -= -0.02;
  }

  if (rty_Tx[3] > 0.02) {
    rty_Tx[3] -= 0.02;
  } else if (rty_Tx[3] >= -0.02) {
    rty_Tx[3] = 0.0;
  } else {
    rty_Tx[3] -= -0.02;
  }

  /* End of DeadZone: '<S6>/Dead Zone' */

  /* RateLimiter: '<S6>/Rate Limiter' */
  localDW->AddConstant[0] = rty_Tx[0] - localDW->PrevY[0];
  localDW->AddConstant[1] = rty_Tx[1] - localDW->PrevY[1];
  localDW->AddConstant[2] = rty_Tx[2] - localDW->PrevY[2];
  localDW->AddConstant[3] = rty_Tx[3] - localDW->PrevY[3];
  if (localDW->AddConstant[0] > 1.875) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[0] + 1.875;
  } else if (localDW->AddConstant[0] < -4.5) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[0] + -4.5;
  } else {
    localDW->rtb_Sum4_idx_1 = rty_Tx[0];
  }

  rty_Tx[0] = localDW->rtb_Sum4_idx_1;
  if (localDW->AddConstant[1] > 1.875) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[1] + 1.875;
  } else if (localDW->AddConstant[1] < -4.5) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[1] + -4.5;
  } else {
    localDW->rtb_Sum4_idx_1 = rty_Tx[1];
  }

  rty_Tx[1] = localDW->rtb_Sum4_idx_1;
  if (localDW->AddConstant[2] > 1.875) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[2] + 1.875;
  } else if (localDW->AddConstant[2] < -4.5) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[2] + -4.5;
  } else {
    localDW->rtb_Sum4_idx_1 = rty_Tx[2];
  }

  rty_Tx[2] = localDW->rtb_Sum4_idx_1;
  if (localDW->AddConstant[3] > 1.875) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[3] + 1.875;
  } else if (localDW->AddConstant[3] < -4.5) {
    localDW->rtb_Sum4_idx_1 = localDW->PrevY[3] + -4.5;
  } else {
    localDW->rtb_Sum4_idx_1 = rty_Tx[3];
  }

  rty_Tx[3] = localDW->rtb_Sum4_idx_1;
  localDW->PrevY[0] = rty_Tx[0];
  localDW->PrevY[1] = rty_Tx[1];
  localDW->PrevY[2] = rty_Tx[2];
  localDW->PrevY[3] = rty_Tx[3];

  /* End of RateLimiter: '<S6>/Rate Limiter' */

  /* Update for UnitDelay: '<S2>/Unit Delay' */
  localDW->UnitDelay_DSTATE[0] = rty_Tx[0];
  localDW->UnitDelay_DSTATE[1] = rty_Tx[1];
  localDW->UnitDelay_DSTATE[2] = rty_Tx[2];
  localDW->UnitDelay_DSTATE[3] = rty_Tx[3];

  /* End of Outputs for SubSystem: '<S1>/Fixed Point Sub' */
}

/* Model step function */
void Electronics_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY)
{
  DW *rtDW = rtM->dwork;

  /* Outputs for Atomic SubSystem: '<Root>/Electronics' */

  /* Inport: '<Root>/omega' incorporates:
   *  Inport: '<Root>/ang_vel'
   *  Inport: '<Root>/driver_input'
   *  Inport: '<Root>/motor_temperature'
   *  Inport: '<Root>/power_limits'
   *  Inport: '<Root>/steering_angle'
   *  Inport: '<Root>/vel'
   *  Outport: '<Root>/Tx'
   *  Outport: '<Root>/bigM_flag'
   */
  Electronics_o(rtU->omega, rtU->vel, rtU->steering_angle, rtU->ang_vel,
                rtU->driver_input, rtU->power_limits, &rtY->bigM_flag, rtY->Tx,
                &rtDW->Electronics_o4);

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
  Electronics_Init(&rtDW->Electronics_o4);

  /* End of SystemInitialize for SubSystem: '<Root>/Electronics' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
