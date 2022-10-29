/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.c
 *
 * Code generated for Simulink model 'Electronics'.
 *
 * Model version                  : 1.131
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Fri Oct 28 19:01:23 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Electronics.h"
#include "bigM_v2_func.h"
#include "rtwtypes.h"
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
extern uint16_T rt_sqrt_Uu16En3_Yu16En_God3C1DL(uint16_T u);
static uint16_T look1_is16lu64n32tu16_binlgse(int16_T u0, const int16_T bp0[],
  const uint16_T table[], uint32_T maxIndex);
static uint32_T plook_u32u16u64n48_evenc_s(uint16_T u, uint16_T bp0, uint16_T
  bpSpace, uint32_T maxIndex, uint64_T *fraction);
static int16_T intrp1d_s16s32s32u32u64n48l_s(uint32_T bpIndex, uint64_T frac,
  const int16_T table[]);
static uint32_T plook_u32u16u64n48_even7c_gf(uint16_T u, uint16_T bp0, uint32_T
  maxIndex, uint64_T *fraction);
static int16_T intrp1d_s16s32s32u32u64n48l_f(uint32_T bpIndex, uint64_T frac,
  const int16_T table[]);
static void Electronics_Init(DW_Electronics *localDW);
static void Electronics_a(real_T rtu_TVS_Information_l, real_T
  rtu_TVS_Information_f, const real_T rtu_TVS_Information_g[3], const real_T
  rtu_TVS_Information_d[2], const real_T rtu_TVS_Information_h[2], const real_T
  rtu_TVS_Information_ge[4], const real_T rtu_TVS_Information_e[4], real_T
  rtu_TVS_Information_b, int32_T *rty_bigM_flag, real_T rty_Tx[4], const
  ConstB_Electronics *localC, DW_Electronics *localDW);
static uint16_T look1_is16lu64n32tu16_binlgse(int16_T u0, const int16_T bp0[],
  const uint16_T table[], uint32_T maxIndex)
{
  uint64_T frac;
  uint32_T bpIdx;
  uint32_T iLeft;
  uint32_T iRght;
  int16_T bpLeftVar;
  uint16_T y;
  uint16_T yL_0d0;
  uint16_T yR_0d0;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'simplest'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'simplest'
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

  bpLeftVar = bp0[iLeft];
  frac = ((uint64_T)(uint16_T)(u0 - bpLeftVar) << 32) / (uint16_T)(bp0[iLeft +
    1U] - bpLeftVar);

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  yR_0d0 = table[iLeft + 1U];
  yL_0d0 = table[iLeft];
  if (yR_0d0 >= yL_0d0) {
    y = (uint16_T)((uint32_T)(uint16_T)(((uint16_T)((uint32_T)yR_0d0 - yL_0d0) *
      frac) >> 32) + yL_0d0);
  } else {
    y = (uint16_T)((uint32_T)yL_0d0 - (uint16_T)(((uint16_T)((uint32_T)yL_0d0 -
      yR_0d0) * frac) >> 32));
  }

  return y;
}

static uint32_T plook_u32u16u64n48_evenc_s(uint16_T u, uint16_T bp0, uint16_T
  bpSpace, uint32_T maxIndex, uint64_T *fraction)
{
  uint32_T bpIndex;
  uint16_T fbpIndex;
  uint16_T uAdjust;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
     Rounding mode: 'simplest'
   */
  if (u <= bp0) {
    bpIndex = 0U;
    *fraction = 0ULL;
  } else {
    uAdjust = (uint16_T)((uint32_T)u - bp0);
    fbpIndex = (uint16_T)((uint32_T)uAdjust / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = fbpIndex;
      *fraction = ((uint64_T)(uint16_T)((uint32_T)uAdjust - (uint16_T)((uint32_T)
        fbpIndex * bpSpace)) << 48) / bpSpace;
    } else {
      bpIndex = maxIndex - 1U;
      *fraction = 281474976710656ULL;
    }
  }

  return bpIndex;
}

static int16_T intrp1d_s16s32s32u32u64n48l_s(uint32_T bpIndex, uint64_T frac,
  const int16_T table[])
{
  int16_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (int16_T)((int16_T)(((table[bpIndex + 1U] - yL_0d0) * (int64_T)frac) >>
    48) + yL_0d0);
}

static uint32_T plook_u32u16u64n48_even7c_gf(uint16_T u, uint16_T bp0, uint32_T
  maxIndex, uint64_T *fraction)
{
  uint32_T bpIndex;
  uint16_T fbpIndex;
  uint16_T uAdjust;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'floor'
   */
  uAdjust = (uint16_T)((uint32_T)u - bp0);
  fbpIndex = (uint16_T)((uint32_T)uAdjust >> 7U);
  if (fbpIndex < maxIndex) {
    bpIndex = fbpIndex;
    *fraction = (uint64_T)(uint16_T)(uAdjust & 127) << 41;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 281474976710656ULL;
  }

  return bpIndex;
}

static int16_T intrp1d_s16s32s32u32u64n48l_f(uint32_T bpIndex, uint64_T frac,
  const int16_T table[])
{
  int16_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Rounding mode: 'floor'
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (int16_T)((int16_T)(((table[bpIndex + 1U] - yL_0d0) * (int64_T)frac) >>
    48) + yL_0d0);
}

uint16_T rt_sqrt_Uu16En3_Yu16En_God3C1DL(uint16_T u)
{
  int32_T iBit;
  uint32_T tmp03_u;
  uint16_T shiftMask;
  uint16_T tmp01_y;
  uint16_T y;

  /* Fixed-Point Sqrt Computation by the bisection method. */
  if (u > 0) {
    y = 0U;
    shiftMask = 32768U;
    tmp03_u = (uint32_T)u << 15;
    for (iBit = 0; iBit < 16; iBit++) {
      tmp01_y = (uint16_T)(y | shiftMask);
      if ((uint32_T)tmp01_y * tmp01_y <= tmp03_u) {
        y = tmp01_y;
      }

      shiftMask = (uint16_T)((uint32_T)shiftMask >> 1U);
    }
  } else {
    y = 0U;
  }

  return y;
}

/* System initialize for atomic system: '<Root>/Electronics' */
static void Electronics_Init(DW_Electronics *localDW)
{
  /* SystemInitialize for Atomic SubSystem: '<S1>/Fixed Point Sub' */
  /* SystemInitialize for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* InitializeConditions for UnitDelay: '<S8>/Unit Delay4' */
  localDW->UnitDelay4_DSTATE = true;

  /* End of SystemInitialize for SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* End of SystemInitialize for SubSystem: '<S1>/Fixed Point Sub' */
}

/* Output and update for atomic system: '<Root>/Electronics' */
static void Electronics_a(real_T rtu_TVS_Information_l, real_T
  rtu_TVS_Information_f, const real_T rtu_TVS_Information_g[3], const real_T
  rtu_TVS_Information_d[2], const real_T rtu_TVS_Information_h[2], const real_T
  rtu_TVS_Information_ge[4], const real_T rtu_TVS_Information_e[4], real_T
  rtu_TVS_Information_b, int32_T *rty_bigM_flag, real_T rty_Tx[4], const
  ConstB_Electronics *localC, DW_Electronics *localDW)
{
  int32_T i;
  int32_T u_tmp;
  int32_T u_tmp_0;
  uint32_T bpIdx;
  int16_T DiscreteTimeIntegrator2;
  int16_T rtb_P;
  int16_T rtb_Sum;
  int16_T rtb_Switch_p_idx_1;
  int16_T rtb_Switch_p_idx_2;
  int16_T rtb_Switch_p_idx_3;
  int16_T rtb_TmpSignalConversionAtDotP_0;
  int16_T rtb_TorqueVectoringMicroCont__0;
  int16_T rtb_TorqueVectoringMicroContr_0;
  int16_T rtb_TorqueVectoringMicroContr_1;
  int16_T rtb_TorqueVectoringMicroContr_2;
  int16_T rtb_TorqueVectoringMicroContr_n;
  int16_T u;
  uint16_T minV_tmp;
  uint16_T rtb_QuadHandle2_idx_0;
  uint16_T rtb_Sqrt;
  boolean_T rtb_GreaterThan;
  boolean_T rtb_LTEp25_h_idx_0;
  boolean_T rtb_LTEp50_h_idx_0;
  boolean_T rtb_UnitDelay4;

  /* Outputs for Atomic SubSystem: '<S1>/Fixed Point Sub' */
  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC1' */
  localDW->rtb_FixedPointSub_boundary_DT_m = rtu_TVS_Information_l * 16384.0;
  if (localDW->rtb_FixedPointSub_boundary_DT_m < 32768.0) {
    rtb_TorqueVectoringMicroCont__0 = (int16_T)
      localDW->rtb_FixedPointSub_boundary_DT_m;
  } else {
    rtb_TorqueVectoringMicroCont__0 = MAX_int16_T;
  }

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC2' */
  localDW->rtb_FixedPointSub_boundary_DT_c = rtu_TVS_Information_f * 128.0;
  if (localDW->rtb_FixedPointSub_boundary_DT_c < 32768.0) {
    rtb_TmpSignalConversionAtDotP_0 = (int16_T)
      localDW->rtb_FixedPointSub_boundary_DT_c;
  } else {
    rtb_TmpSignalConversionAtDotP_0 = MAX_int16_T;
  }

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC5' */
  localDW->rtb_FixedPointSub_boundary_DT_c = rtu_TVS_Information_g[2] * 2048.0;

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC6' */
  localDW->CCaller_o2 = rtu_TVS_Information_d[0] * 512.0;
  if (localDW->CCaller_o2 < 32768.0) {
    rtb_TorqueVectoringMicroContr_1 = (int16_T)localDW->CCaller_o2;
  } else {
    rtb_TorqueVectoringMicroContr_1 = MAX_int16_T;
  }

  localDW->CCaller_o2 = rtu_TVS_Information_d[1] * 512.0;
  if (localDW->CCaller_o2 < 32768.0) {
    rtb_TorqueVectoringMicroContr_2 = (int16_T)localDW->CCaller_o2;
  } else {
    rtb_TorqueVectoringMicroContr_2 = MAX_int16_T;
  }

  /* End of DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC6' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC11' */
  localDW->CCaller_o2 = rtu_TVS_Information_h[0] * 0.25;
  if (localDW->CCaller_o2 < 32768.0) {
    rtb_TorqueVectoringMicroContr_0 = (int16_T)localDW->CCaller_o2;
  } else {
    rtb_TorqueVectoringMicroContr_0 = MAX_int16_T;
  }

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC8' */
  localDW->CCaller_o2 = rtu_TVS_Information_ge[0] * 128.0;
  if (localDW->CCaller_o2 < 32768.0) {
    localDW->TorqueVectoringMicroCont_jz[0] = (int16_T)localDW->CCaller_o2;
  } else {
    localDW->TorqueVectoringMicroCont_jz[0] = MAX_int16_T;
  }

  localDW->CCaller_o2 = rtu_TVS_Information_ge[1] * 128.0;
  if (localDW->CCaller_o2 < 32768.0) {
    localDW->TorqueVectoringMicroCont_jz[1] = (int16_T)localDW->CCaller_o2;
  } else {
    localDW->TorqueVectoringMicroCont_jz[1] = MAX_int16_T;
  }

  localDW->CCaller_o2 = rtu_TVS_Information_ge[2] * 128.0;
  if (localDW->CCaller_o2 < 32768.0) {
    localDW->TorqueVectoringMicroCont_jz[2] = (int16_T)localDW->CCaller_o2;
  } else {
    localDW->TorqueVectoringMicroCont_jz[2] = MAX_int16_T;
  }

  localDW->CCaller_o2 = rtu_TVS_Information_ge[3] * 128.0;
  if (localDW->CCaller_o2 < 32768.0) {
    localDW->TorqueVectoringMicroCont_jz[3] = (int16_T)localDW->CCaller_o2;
  } else {
    localDW->TorqueVectoringMicroCont_jz[3] = MAX_int16_T;
  }

  /* End of DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC8' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC12' */
  localDW->CCaller_o2 = rtu_TVS_Information_e[2] * 512.0;
  localDW->Add1 = rtu_TVS_Information_e[3] * 512.0;

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  for (i = 0; i < 4; i++) {
    /* Gain: '<S8>/Gain1' incorporates:
     *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC8'
     */
    rtb_TorqueVectoringMicroContr_n = (int16_T)((rtConstP.Gain1_Gain_n[i] *
      localDW->TorqueVectoringMicroCont_jz[i]) >> 15);

    /* Gain: '<S18>/Gain' */
    localDW->sigIdx = 25033 * rtb_TorqueVectoringMicroContr_n;

    /* Sum: '<S18>/Sum' incorporates:
     *  Gain: '<S18>/Gain'
     *  RelationalOperator: '<S18>/Equal'
     */
    rtb_Sum = (int16_T)((((((localDW->sigIdx & 8388607U) != 0U) +
      (localDW->sigIdx >> 23)) << 1) + (rtb_TorqueVectoringMicroContr_n == 0)) <<
                        7);

    /* Sum: '<S18>/Sum1' incorporates:
     *  RelationalOperator: '<S18>/Equal1'
     */
    rtb_Sum = (int16_T)((rtb_Sum - ((rtb_Sum == 6528) << 7)) >> 7);
    for (localDW->sigIdx = 0; localDW->sigIdx < 51; localDW->sigIdx++) {
      /* Abs: '<S18>/Abs1' incorporates:
       *  Abs: '<S18>/Abs'
       *  Bias: '<S18>/Bias1'
       *  Bias: '<S18>/Bias2'
       *  Constant: '<S18>/Constant4'
       *  Constant: '<S18>/Constant5'
       *  Selector: '<S18>/Selector1'
       */
      u_tmp = 51 * i + localDW->sigIdx;
      u_tmp_0 = rtConstP.pooled11[u_tmp];
      u = (int16_T)(rtConstP.pooled10[(51 * rtb_Sum + rtConstP.pooled19
        [localDW->sigIdx]) - 1] + u_tmp_0);
      if (u < 0) {
        localDW->Abs1[u_tmp] = (int16_T)-u;
      } else {
        localDW->Abs1[u_tmp] = u;
      }

      /* End of Abs: '<S18>/Abs1' */

      /* Abs: '<S18>/Abs' incorporates:
       *  Bias: '<S18>/Bias1'
       *  Constant: '<S18>/Constant1'
       *  Constant: '<S18>/Constant3'
       *  Selector: '<S18>/Selector'
       */
      u = (int16_T)(rtConstP.pooled10[((rtb_Sum - 1) * 51 +
        rtConstP.pooled19[localDW->sigIdx]) - 1] + u_tmp_0);
      if (u < 0) {
        localDW->Abs[u_tmp] = (int16_T)-u;
      } else {
        localDW->Abs[u_tmp] = u;
      }
    }

    /* Gain: '<S8>/Gain1' */
    localDW->TorqueVectoringMicroCont_jz[i] = rtb_TorqueVectoringMicroContr_n;

    /* Sum: '<S18>/Sum' */
    localDW->Sum[i] = rtb_Sum;
  }

  /* S-Function (sdspstatminmax): '<S18>/Minimum' incorporates:
   *  Abs: '<S18>/Abs'
   */
  u_tmp = 0;
  u_tmp_0 = 0;
  for (localDW->sigIdx = 0; localDW->sigIdx < 4; localDW->sigIdx++) {
    localDW->Minimum_Valdata[u_tmp_0] = localDW->Abs[u_tmp];
    localDW->Minimum[u_tmp_0] = 1U;
    u_tmp++;
    for (i = 0; i < 50; i++) {
      if (localDW->Abs[u_tmp] < localDW->Minimum_Valdata[u_tmp_0]) {
        localDW->Minimum_Valdata[u_tmp_0] = localDW->Abs[u_tmp];
        localDW->Minimum[u_tmp_0] = (uint32_T)(i + 2);
      }

      u_tmp++;
    }

    u_tmp_0++;
  }

  /* End of S-Function (sdspstatminmax): '<S18>/Minimum' */

  /* S-Function (sdspstatminmax): '<S18>/Minimum1' incorporates:
   *  Abs: '<S18>/Abs1'
   */
  u_tmp = 0;
  u_tmp_0 = 0;
  for (localDW->sigIdx = 0; localDW->sigIdx < 4; localDW->sigIdx++) {
    localDW->Minimum1_Valdata[u_tmp_0] = localDW->Abs1[u_tmp];
    localDW->Minimum1[u_tmp_0] = 1U;
    u_tmp++;
    for (i = 0; i < 50; i++) {
      if (localDW->Abs1[u_tmp] < localDW->Minimum1_Valdata[u_tmp_0]) {
        localDW->Minimum1_Valdata[u_tmp_0] = localDW->Abs1[u_tmp];
        localDW->Minimum1[u_tmp_0] = (uint32_T)(i + 2);
      }

      u_tmp++;
    }

    u_tmp_0++;

    /* Selector: '<S18>/Selector2' incorporates:
     *  Abs: '<S18>/Abs1'
     *  Constant: '<S18>/Constant7'
     *  MinMax: '<S8>/Min1'
     *  S-Function (sdspstatminmax): '<S18>/Minimum'
     *  S-Function (sdspstatminmax): '<S18>/Minimum1'
     */
    localDW->Min1[localDW->sigIdx] = rtConstP.pooled15[(int32_T)localDW->
      Minimum[localDW->sigIdx] - 1];
  }

  /* End of S-Function (sdspstatminmax): '<S18>/Minimum1' */
  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC12' */
  if (localDW->CCaller_o2 < 65536.0) {
    rtb_QuadHandle2_idx_0 = (uint16_T)localDW->CCaller_o2;
  } else {
    rtb_QuadHandle2_idx_0 = MAX_uint16_T;
  }

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Product: '<S18>/Product' incorporates:
   *  Bias: '<S18>/Bias3'
   *  Constant: '<S18>/Constant8'
   *  Gain: '<S18>/Gain1'
   *  Gain: '<S18>/Gain2'
   *  Gain: '<S7>/Gain'
   *  Gain: '<S8>/Gain1'
   *  MinMax: '<S8>/Min1'
   *  Product: '<S18>/Product1'
   *  RelationalOperator: '<S18>/Less Than'
   *  S-Function (sdspstatminmax): '<S18>/Minimum1'
   *  Selector: '<S18>/Selector3'
   *  Sum: '<S18>/Sum2'
   *  Sum: '<S18>/Sum3'
   *  Sum: '<S18>/Sum4'
   */
  rtb_Sum = (int16_T)(rtb_QuadHandle2_idx_0 < 38400 ? (localDW->Min1[2] -
    ((((int16_T)(((int16_T)(localDW->TorqueVectoringMicroCont_jz[2] - (int16_T)
    (((localDW->Sum[2] + -1) * 21447) >> 7)) * ((localDW->Min1[2] -
    rtConstP.pooled15[(int32_T)localDW->Minimum1[2] - 1]) >> 1)) >> 15) * 25033)
      >> 15) << 9)) >> 9 : 0);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Gain: '<S8>/Gain1'
   *  Lookup_n-D: '<S8>/1-D Lookup Table'
   *  Sum: '<S18>/Sum4'
   */
  if (rtb_Sum > 4) {
    rtb_Sqrt = (uint16_T)(rtb_Sum << 1);
  } else {
    rtb_Sqrt = (uint16_T)((uint32_T)look1_is16lu64n32tu16_binlgse
                          (localDW->TorqueVectoringMicroCont_jz[2],
      rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData_i, 67U) >>
                          8);
  }

  /* Saturate: '<S8>/Saturation' */
  if (((uint32_T)rtb_Sqrt << 8) <= 51200U) {
    if (rtb_Sqrt > 255) {
      /* Gain: '<S8>/Gain8' */
      rtb_Sqrt = MAX_uint16_T;
    } else {
      /* Gain: '<S8>/Gain8' */
      rtb_Sqrt = (uint16_T)(rtb_Sqrt << 8);
    }
  } else {
    /* Gain: '<S8>/Gain8' */
    rtb_Sqrt = 51200U;
  }

  /* Saturate: '<S8>/Max Torque Limiter' incorporates:
   *  MinMax: '<S8>/Min1'
   */
  localDW->sigIdx = (uint16_T)((uint32_T)rtb_Sqrt >> 2) << 1;
  if (localDW->sigIdx >= 25600) {
    rtb_TorqueVectoringMicroContr_n = 25600;
  } else if (localDW->sigIdx <= 10) {
    rtb_TorqueVectoringMicroContr_n = 10;
  } else {
    rtb_TorqueVectoringMicroContr_n = (int16_T)localDW->sigIdx;
  }

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC12' */
  if (localDW->Add1 < 65536.0) {
    rtb_QuadHandle2_idx_0 = (uint16_T)localDW->Add1;
  } else {
    rtb_QuadHandle2_idx_0 = MAX_uint16_T;
  }

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Product: '<S18>/Product' incorporates:
   *  Bias: '<S18>/Bias3'
   *  Constant: '<S18>/Constant8'
   *  Gain: '<S18>/Gain1'
   *  Gain: '<S18>/Gain2'
   *  Gain: '<S7>/Gain'
   *  Gain: '<S8>/Gain1'
   *  MinMax: '<S8>/Min1'
   *  Product: '<S18>/Product1'
   *  RelationalOperator: '<S18>/Less Than'
   *  S-Function (sdspstatminmax): '<S18>/Minimum1'
   *  Selector: '<S18>/Selector3'
   *  Sum: '<S18>/Sum2'
   *  Sum: '<S18>/Sum3'
   *  Sum: '<S18>/Sum4'
   */
  rtb_Sum = (int16_T)(rtb_QuadHandle2_idx_0 < 38400 ? (localDW->Min1[3] -
    ((((int16_T)(((int16_T)(localDW->TorqueVectoringMicroCont_jz[3] - (int16_T)
    (((localDW->Sum[3] + -1) * 21447) >> 7)) * ((localDW->Min1[3] -
    rtConstP.pooled15[(int32_T)localDW->Minimum1[3] - 1]) >> 1)) >> 15) * 25033)
      >> 15) << 9)) >> 9 : 0);

  /* Switch: '<S8>/Switch2' incorporates:
   *  Gain: '<S8>/Gain1'
   *  Lookup_n-D: '<S8>/1-D Lookup Table'
   *  Sum: '<S18>/Sum4'
   */
  if (rtb_Sum > 4) {
    rtb_Sqrt = (uint16_T)(rtb_Sum << 1);
  } else {
    rtb_Sqrt = (uint16_T)((uint32_T)look1_is16lu64n32tu16_binlgse
                          (localDW->TorqueVectoringMicroCont_jz[3],
      rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData_i, 67U) >>
                          8);
  }

  /* Saturate: '<S8>/Saturation' */
  if (((uint32_T)rtb_Sqrt << 8) <= 51200U) {
    if (rtb_Sqrt > 255) {
      /* Gain: '<S8>/Gain8' */
      rtb_Sqrt = MAX_uint16_T;
    } else {
      /* Gain: '<S8>/Gain8' */
      rtb_Sqrt = (uint16_T)(rtb_Sqrt << 8);
    }
  } else {
    /* Gain: '<S8>/Gain8' */
    rtb_Sqrt = 51200U;
  }

  /* Saturate: '<S8>/Max Torque Limiter' incorporates:
   *  MinMax: '<S8>/Min1'
   */
  localDW->sigIdx = (uint16_T)((uint32_T)rtb_Sqrt >> 2) << 1;
  if (localDW->sigIdx >= 25600) {
    rtb_Sum = 25600;
  } else if (localDW->sigIdx <= 10) {
    rtb_Sum = 10;
  } else {
    rtb_Sum = (int16_T)localDW->sigIdx;
  }

  /* Math: '<S10>/Square' incorporates:
   *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC6'
   */
  localDW->sigIdx = (rtb_TorqueVectoringMicroContr_1 *
                     rtb_TorqueVectoringMicroContr_1) >> 14;
  if (localDW->sigIdx < 0) {
    localDW->sigIdx = 0;
  }

  i = (rtb_TorqueVectoringMicroContr_2 * rtb_TorqueVectoringMicroContr_2) >> 14;
  if (i < 0) {
    i = 0;
  }

  /* Sqrt: '<S10>/Sqrt' incorporates:
   *  Math: '<S10>/Square'
   *  Sum: '<S10>/Sum'
   */
  rtb_Sqrt = rt_sqrt_Uu16En3_Yu16En_God3C1DL((uint16_T)(((uint32_T)(uint16_T)
    localDW->sigIdx + (uint16_T)i) >> 1));

  /* DeadZone: '<S11>/Dead Zone' incorporates:
   *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC2'
   */
  if (rtb_TmpSignalConversionAtDotP_0 > 1024) {
    rtb_TmpSignalConversionAtDotP_0 = (int16_T)(rtb_TmpSignalConversionAtDotP_0
      - 1024);
  } else if (rtb_TmpSignalConversionAtDotP_0 >= -1024) {
    rtb_TmpSignalConversionAtDotP_0 = 0;
  } else {
    rtb_TmpSignalConversionAtDotP_0 = (int16_T)(rtb_TmpSignalConversionAtDotP_0
      - -1024);
  }

  /* End of DeadZone: '<S11>/Dead Zone' */

  /* Gain: '<S11>/Gain' incorporates:
   *  Gain: '<S9>/I'
   */
  rtb_TmpSignalConversionAtDotP_0 = (int16_T)((18477 *
    rtb_TmpSignalConversionAtDotP_0) >> 14);

  /* Math: '<S23>/Square1' incorporates:
   *  Gain: '<S9>/I'
   *  Math: '<S23>/Square'
   *  Math: '<S24>/Square1'
   */
  localDW->sigIdx = (rtb_TmpSignalConversionAtDotP_0 *
                     rtb_TmpSignalConversionAtDotP_0) >> 13;
  i = localDW->sigIdx;
  if (localDW->sigIdx < 0) {
    i = 0;
  }

  /* Math: '<S23>/Square' incorporates:
   *  Math: '<S24>/Square1'
   */
  u_tmp = localDW->sigIdx;
  if (localDW->sigIdx < 0) {
    u_tmp = 0;
    localDW->sigIdx = 0;
  }

  /* Sum: '<S23>/Sum of Elements' incorporates:
   *  Gain: '<S23>/Gain'
   *  Gain: '<S23>/Gain1'
   *  Gain: '<S23>/Gain2'
   *  Gain: '<S9>/I'
   *  Math: '<S23>/Square'
   *  Math: '<S23>/Square1'
   *  Product: '<S23>/Product'
   *  Sum: '<S24>/Sum of Elements'
   */
  u_tmp_0 = (int16_T)((10707 * rtb_TmpSignalConversionAtDotP_0) >> 13);
  i = (((int16_T)((((i * rtb_TmpSignalConversionAtDotP_0) >> 15) * 9395) >> 16)
        + (int16_T)((63753U * u_tmp) >> 19)) + u_tmp_0) + -109;

  /* Sum: '<S24>/Sum of Elements' incorporates:
   *  Gain: '<S24>/Gain'
   *  Gain: '<S9>/I'
   *  Math: '<S24>/Square1'
   *  Product: '<S24>/Product'
   */
  localDW->sigIdx = ((int16_T)((((localDW->sigIdx *
    rtb_TmpSignalConversionAtDotP_0) >> 15) * 9395) >> 16) + u_tmp_0) + 109;

  /* Gain: '<S10>/Gain' incorporates:
   *  Product: '<S10>/Product1'
   *  S-Function (sdspstatfcns): '<S11>/Mean'
   *  Sqrt: '<S10>/Sqrt'
   *  Sum: '<S23>/Sum of Elements'
   *  Sum: '<S24>/Sum of Elements'
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  rtb_TorqueVectoringMicroContr_2 = (int16_T)(((((int16_T)((int16_T)((int16_T)
    localDW->sigIdx + (int16_T)i) >> 1) * rtb_Sqrt) >> 15) * 5201) >> 13);

  /* Signum: '<S10>/Sign' incorporates:
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  if (rtb_TorqueVectoringMicroContr_2 < 0) {
    rtb_TmpSignalConversionAtDotP_0 = -1;
  } else {
    rtb_TmpSignalConversionAtDotP_0 = (int16_T)(rtb_TorqueVectoringMicroContr_2 >
      0);
  }

  /* End of Signum: '<S10>/Sign' */

  /* Lookup_n-D: '<S10>/1-D Lookup Table' incorporates:
   *  Sqrt: '<S10>/Sqrt'
   */
  bpIdx = plook_u32u16u64n48_evenc_s(rtb_Sqrt, 0U, 1536U, 9U, &localDW->frac);

  /* Abs: '<S10>/Abs' incorporates:
   *  Lookup_n-D: '<S10>/1-D Lookup Table'
   */
  if (rtb_TorqueVectoringMicroContr_2 < 0) {
    rtb_QuadHandle2_idx_0 = (uint16_T)(-rtb_TorqueVectoringMicroContr_2 << 1);
  } else {
    rtb_QuadHandle2_idx_0 = (uint16_T)(rtb_TorqueVectoringMicroContr_2 << 1);
  }

  minV_tmp = (uint16_T)(intrp1d_s16s32s32u32u64n48l_s(bpIdx, localDW->frac,
    rtConstP.uDLookupTable_tableData) << 1);

  /* MinMax: '<S10>/Min of Elements' incorporates:
   *  Abs: '<S10>/Abs'
   */
  if (minV_tmp > rtb_QuadHandle2_idx_0) {
    minV_tmp = rtb_QuadHandle2_idx_0;
  }

  /* Product: '<S10>/Product' incorporates:
   *  MinMax: '<S10>/Min of Elements'
   */
  rtb_TorqueVectoringMicroContr_2 = (int16_T)(((minV_tmp << 11) *
    rtb_TmpSignalConversionAtDotP_0) >> 1);

  /* Switch: '<S10>/Switch' incorporates:
   *  Product: '<S10>/Product'
   */
  if (rtb_TorqueVectoringMicroContr_2 != 0) {
    /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC' incorporates:
     *  Switch: '<S10>/Switch1'
     */
    localDW->CCaller_o2 = rtu_TVS_Information_b * 256.0;
    if (localDW->CCaller_o2 < 65536.0) {
      rtb_QuadHandle2_idx_0 = (uint16_T)localDW->CCaller_o2;
    } else {
      rtb_QuadHandle2_idx_0 = MAX_uint16_T;
    }

    /* End of DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC' */

    /* UnitDelay: '<S8>/Unit Delay5' incorporates:
     *  Product: '<S10>/Divide'
     *  Product: '<S10>/Product2'
     *  Sqrt: '<S10>/Sqrt'
     *  Switch: '<S10>/Switch1'
     */
    rtb_TorqueVectoringMicroContr_2 = (int16_T)(((uint16_T)(rtb_Sqrt /
      ((uint32_T)rtb_QuadHandle2_idx_0 << 2)) * rtb_TmpSignalConversionAtDotP_0)
      >> 1);
  }

  /* End of Switch: '<S10>/Switch' */

  /* DataTypeConversion: '<S20>/CastU16En16' incorporates:
   *  DataTypeConversion: '<S22>/CastU16En16'
   *  Sum: '<S23>/Sum of Elements'
   */
  rtb_Sqrt = (uint16_T)((uint16_T)i << 6);

  /* RelationalOperator: '<S20>/LTEp25' incorporates:
   *  DataTypeConversion: '<S20>/CastU16En16'
   */
  rtb_UnitDelay4 = (rtb_Sqrt <= 16384);

  /* RelationalOperator: '<S20>/GTEp75' incorporates:
   *  DataTypeConversion: '<S20>/CastU16En16'
   */
  rtb_GreaterThan = (rtb_Sqrt >= 49152);

  /* Switch: '<S20>/QuadHandle2' incorporates:
   *  Constant: '<S20>/Point75'
   *  DataTypeConversion: '<S20>/CastU16En16'
   *  RelationalOperator: '<S20>/LTEp50'
   *  Sum: '<S20>/p75mA'
   *  Switch: '<S20>/QuadHandle1b'
   */
  if (rtb_Sqrt <= 32768) {
    /* Switch: '<S20>/QuadHandle1a' incorporates:
     *  Constant: '<S20>/Point25'
     *  Sum: '<S20>/Amp25'
     *  Sum: '<S20>/p25mA'
     */
    if (rtb_UnitDelay4) {
      rtb_QuadHandle2_idx_0 = (uint16_T)(16384 - rtb_Sqrt);
    } else {
      rtb_QuadHandle2_idx_0 = (uint16_T)(rtb_Sqrt - 16384);
    }
  } else if (rtb_GreaterThan) {
    /* Switch: '<S20>/QuadHandle1b' incorporates:
     *  Constant: '<S20>/Point75'
     *  Sum: '<S20>/Amp75'
     */
    rtb_QuadHandle2_idx_0 = (uint16_T)(rtb_Sqrt - 49152);
  } else {
    rtb_QuadHandle2_idx_0 = (uint16_T)(49152 - rtb_Sqrt);
  }

  /* RelationalOperator: '<S20>/LTEp25' */
  rtb_LTEp25_h_idx_0 = rtb_UnitDelay4;

  /* RelationalOperator: '<S20>/GTEp75' */
  rtb_LTEp50_h_idx_0 = rtb_GreaterThan;

  /* DataTypeConversion: '<S20>/CastU16En16' incorporates:
   *  DataTypeConversion: '<S22>/CastU16En16'
   *  Sum: '<S24>/Sum of Elements'
   */
  minV_tmp = (uint16_T)((uint16_T)localDW->sigIdx << 6);

  /* RelationalOperator: '<S20>/LTEp25' incorporates:
   *  DataTypeConversion: '<S20>/CastU16En16'
   */
  rtb_UnitDelay4 = (minV_tmp <= 16384);

  /* RelationalOperator: '<S20>/GTEp75' incorporates:
   *  DataTypeConversion: '<S20>/CastU16En16'
   */
  rtb_GreaterThan = (minV_tmp >= 49152);

  /* Lookup_n-D: '<S19>/Look-Up Table' incorporates:
   *  SignalConversion generated from: '<S8>/Dot Product2'
   */
  bpIdx = plook_u32u16u64n48_even7c_gf(rtb_QuadHandle2_idx_0, 0U, 128U,
    &localDW->frac);
  rtb_TmpSignalConversionAtDotP_0 = intrp1d_s16s32s32u32u64n48l_f(bpIdx,
    localDW->frac, rtConstP.pooled6);

  /* Switch: '<S20>/QuadHandle2' incorporates:
   *  Constant: '<S20>/Point75'
   *  DataTypeConversion: '<S20>/CastU16En16'
   *  RelationalOperator: '<S20>/LTEp50'
   *  Sum: '<S20>/p75mA'
   *  Switch: '<S20>/QuadHandle1b'
   */
  if (minV_tmp <= 32768) {
    /* Switch: '<S20>/QuadHandle1a' incorporates:
     *  Constant: '<S20>/Point25'
     *  Sum: '<S20>/Amp25'
     *  Sum: '<S20>/p25mA'
     */
    if (rtb_UnitDelay4) {
      rtb_QuadHandle2_idx_0 = (uint16_T)(16384 - minV_tmp);
    } else {
      rtb_QuadHandle2_idx_0 = (uint16_T)(minV_tmp - 16384);
    }
  } else if (rtb_GreaterThan) {
    /* Switch: '<S20>/QuadHandle1b' incorporates:
     *  Constant: '<S20>/Point75'
     *  Sum: '<S20>/Amp75'
     */
    rtb_QuadHandle2_idx_0 = (uint16_T)(minV_tmp - 49152);
  } else {
    rtb_QuadHandle2_idx_0 = (uint16_T)(49152 - minV_tmp);
  }

  /* Lookup_n-D: '<S19>/Look-Up Table' incorporates:
   *  SignalConversion generated from: '<S8>/Dot Product2'
   */
  bpIdx = plook_u32u16u64n48_even7c_gf(rtb_QuadHandle2_idx_0, 0U, 128U,
    &localDW->frac);
  rtb_TorqueVectoringMicroContr_1 = intrp1d_s16s32s32u32u64n48l_f(bpIdx,
    localDW->frac, rtConstP.pooled6);

  /* Switch: '<S20>/SignCorrected' incorporates:
   *  Logic: '<S20>/1st or 4th Quad'
   *  SignalConversion generated from: '<S8>/Dot Product2'
   *  UnaryMinus: '<S20>/Negate'
   */
  u = rtb_TmpSignalConversionAtDotP_0;
  if ((!rtb_LTEp25_h_idx_0) && (!rtb_LTEp50_h_idx_0)) {
    u = (int16_T)-rtb_TmpSignalConversionAtDotP_0;
  }

  /* RelationalOperator: '<S22>/LTEp50' */
  rtb_LTEp25_h_idx_0 = (rtb_Sqrt <= 32768);

  /* Switch: '<S22>/QuadHandle1' incorporates:
   *  Constant: '<S22>/Point50'
   *  Sum: '<S22>/Amp50'
   */
  if (!rtb_LTEp25_h_idx_0) {
    rtb_Sqrt = (uint16_T)(rtb_Sqrt - 32768);
  }

  /* Switch: '<S22>/QuadHandle2' incorporates:
   *  Constant: '<S22>/Point50'
   *  RelationalOperator: '<S22>/LTEp25'
   *  Sum: '<S22>/p50mA'
   */
  if (rtb_Sqrt > 16384) {
    rtb_Sqrt = (uint16_T)(32768 - rtb_Sqrt);
  }

  /* RelationalOperator: '<S22>/LTEp50' */
  rtb_LTEp50_h_idx_0 = rtb_LTEp25_h_idx_0;

  /* Switch: '<S20>/SignCorrected' incorporates:
   *  SignalConversion generated from: '<S8>/Dot Product2'
   */
  rtb_TmpSignalConversionAtDotP_0 = u;

  /* DataTypeConversion: '<S22>/CastU16En16' incorporates:
   *  Switch: '<S22>/QuadHandle2'
   */
  rtb_QuadHandle2_idx_0 = rtb_Sqrt;

  /* Switch: '<S20>/SignCorrected' incorporates:
   *  Logic: '<S20>/1st or 4th Quad'
   *  SignalConversion generated from: '<S8>/Dot Product2'
   *  UnaryMinus: '<S20>/Negate'
   */
  u = rtb_TorqueVectoringMicroContr_1;
  if ((!rtb_UnitDelay4) && (!rtb_GreaterThan)) {
    u = (int16_T)-rtb_TorqueVectoringMicroContr_1;
  }

  /* DataTypeConversion: '<S22>/CastU16En16' */
  rtb_Sqrt = minV_tmp;

  /* RelationalOperator: '<S22>/LTEp50' */
  rtb_LTEp25_h_idx_0 = (minV_tmp <= 32768);

  /* Switch: '<S22>/QuadHandle1' incorporates:
   *  Constant: '<S22>/Point50'
   *  Sum: '<S22>/Amp50'
   */
  if (!rtb_LTEp25_h_idx_0) {
    rtb_Sqrt = (uint16_T)(minV_tmp - 32768);
  }

  /* Switch: '<S22>/QuadHandle2' incorporates:
   *  Constant: '<S22>/Point50'
   *  RelationalOperator: '<S22>/LTEp25'
   *  Sum: '<S22>/p50mA'
   */
  if (rtb_Sqrt > 16384) {
    rtb_Sqrt = (uint16_T)(32768 - rtb_Sqrt);
  }

  /* Lookup_n-D: '<S21>/Look-Up Table' incorporates:
   *  Switch: '<S22>/QuadHandle2'
   *  Switch: '<S22>/SignCorrected'
   */
  bpIdx = plook_u32u16u64n48_even7c_gf(rtb_QuadHandle2_idx_0, 0U, 128U,
    &localDW->frac);
  rtb_TorqueVectoringMicroContr_1 = intrp1d_s16s32s32u32u64n48l_f(bpIdx,
    localDW->frac, rtConstP.pooled6);
  bpIdx = plook_u32u16u64n48_even7c_gf(rtb_Sqrt, 0U, 128U, &localDW->frac);

  /* Switch: '<S22>/SignCorrected' incorporates:
   *  UnaryMinus: '<S22>/Negate'
   */
  rtb_Switch_p_idx_1 = rtb_TorqueVectoringMicroContr_1;
  if (!rtb_LTEp50_h_idx_0) {
    rtb_Switch_p_idx_1 = (int16_T)-rtb_TorqueVectoringMicroContr_1;
  }

  rtb_TorqueVectoringMicroContr_1 = rtb_Switch_p_idx_1;

  /* Lookup_n-D: '<S21>/Look-Up Table' */
  rtb_Switch_p_idx_1 = intrp1d_s16s32s32u32u64n48l_f(bpIdx, localDW->frac,
    rtConstP.pooled6);

  /* Switch: '<S22>/SignCorrected' incorporates:
   *  UnaryMinus: '<S22>/Negate'
   */
  if (!rtb_LTEp25_h_idx_0) {
    rtb_Switch_p_idx_1 = (int16_T)-rtb_Switch_p_idx_1;
  }

  /* DotProduct: '<S8>/Dot Product1' incorporates:
   *  Constant: '<S8>/Constant5'
   *  SignalConversion generated from: '<S8>/Dot Product1'
   */
  rtb_TorqueVectoringMicroContr_1 = (int16_T)((int16_T)
    ((rtb_TmpSignalConversionAtDotP_0 * 10615) >> 18) +
    ((rtb_TorqueVectoringMicroContr_1 * 3245) >> 16));

  /* DotProduct: '<S8>/Dot Product2' incorporates:
   *  Constant: '<S8>/Constant6'
   */
  u = (int16_T)((int16_T)((u * -10615) >> 18) + ((rtb_Switch_p_idx_1 * 3245) >>
    16));

  /* Switch: '<S15>/Switch' incorporates:
   *  Constant: '<S15>/Constant'
   *  DotProduct: '<S8>/Dot Product'
   *  Gain: '<S8>/Gain7'
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  if (rtb_TorqueVectoringMicroContr_2 != 0) {
    /* DotProduct: '<S8>/Dot Product' incorporates:
     *  DotProduct: '<S8>/Dot Product1'
     *  DotProduct: '<S8>/Dot Product2'
     *  Saturate: '<S8>/Max Torque Limiter'
     */
    rtb_Switch_p_idx_3 = (int16_T)((int16_T)((int16_T)((int16_T)
      ((rtb_TorqueVectoringMicroContr_1 * 10) >> 16) + ((u * 10) >> 16)) + ((159
      * rtb_TorqueVectoringMicroContr_n) >> 14)) + ((-159 * rtb_Sum) >> 14));
    rtb_TmpSignalConversionAtDotP_0 = (int16_T)((12703 * rtb_Switch_p_idx_3) >>
      8);
    rtb_Switch_p_idx_1 = rtb_TmpSignalConversionAtDotP_0;
    rtb_Switch_p_idx_2 = (int16_T)((6351 * rtb_Switch_p_idx_3) >> 7);
    rtb_Switch_p_idx_3 = rtb_Switch_p_idx_2;
  } else {
    rtb_TmpSignalConversionAtDotP_0 = 0;
    rtb_Switch_p_idx_1 = 0;
    rtb_Switch_p_idx_2 = 0;
    rtb_Switch_p_idx_3 = 0;
  }

  /* End of Switch: '<S15>/Switch' */

  /* Sum: '<S15>/Sum3' */
  rtb_Switch_p_idx_2 += rtb_TmpSignalConversionAtDotP_0;

  /* UnitDelay: '<S8>/Unit Delay4' */
  rtb_UnitDelay4 = localDW->UnitDelay4_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  UnitDelay: '<S8>/Unit Delay4'
   */
  if (localDW->UnitDelay4_DSTATE || (localDW->DiscreteTimeIntegrator_PrevRese !=
       0)) {
    localDW->DiscreteTimeIntegrator_DSTATE = 0;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  rtb_TmpSignalConversionAtDotP_0 = localDW->DiscreteTimeIntegrator_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  if (localDW->DiscreteTimeIntegrator_DSTATE >= 20480) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
    rtb_TmpSignalConversionAtDotP_0 = 20480;
  } else if (localDW->DiscreteTimeIntegrator_DSTATE <= -20480) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
    rtb_TmpSignalConversionAtDotP_0 = -20480;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' incorporates:
   *  UnitDelay: '<S8>/Unit Delay4'
   */
  if (localDW->UnitDelay4_DSTATE || (localDW->DiscreteTimeIntegrator2_PrevRes !=
       0)) {
    localDW->DiscreteTimeIntegrator2_DSTATE = 0;
  }

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' */
  DiscreteTimeIntegrator2 = localDW->DiscreteTimeIntegrator2_DSTATE;

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' */
  if (localDW->DiscreteTimeIntegrator2_DSTATE >= 20480) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' */
    DiscreteTimeIntegrator2 = 20480;
  } else if (localDW->DiscreteTimeIntegrator2_DSTATE <= -20480) {
    /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' */
    DiscreteTimeIntegrator2 = -20480;
  }

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC5' */
  if (localDW->rtb_FixedPointSub_boundary_DT_c < 32768.0) {
    rtb_P = (int16_T)localDW->rtb_FixedPointSub_boundary_DT_c;
  } else {
    rtb_P = MAX_int16_T;
  }

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Sum: '<S9>/Sum7' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator2'
   *  Gain: '<S9>/P'
   *  Sum: '<S9>/Sum'
   *  Sum: '<S9>/Sum1'
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  rtb_P = (int16_T)((((int16_T)((((int16_T)(((int16_T)
    (((rtb_TorqueVectoringMicroContr_2 << 13) - rtb_P) >> 13) * 3) >> 1) << 13)
    + rtb_TmpSignalConversionAtDotP_0) >> 13) << 13) + DiscreteTimeIntegrator2) >>
                    13);

  /* RelationalOperator: '<S16>/LowerRelop1' incorporates:
   *  RelationalOperator: '<S16>/UpperRelop'
   *  Sum: '<S9>/Sum7'
   *  Switch: '<S16>/Switch2'
   */
  localDW->sigIdx = rtb_P << 14;

  /* Switch: '<S16>/Switch2' incorporates:
   *  RelationalOperator: '<S16>/LowerRelop1'
   *  Sum: '<S15>/Sum3'
   */
  if (localDW->sigIdx > rtb_Switch_p_idx_2) {
    rtb_Switch_p_idx_2 = (int16_T)(rtb_Switch_p_idx_2 >> 14);
  } else {
    /* Sum: '<S15>/Sum4' */
    rtb_Switch_p_idx_2 = (int16_T)(rtb_Switch_p_idx_1 + rtb_Switch_p_idx_3);

    /* Switch: '<S16>/Switch' incorporates:
     *  RelationalOperator: '<S16>/UpperRelop'
     *  Sum: '<S15>/Sum4'
     *  Sum: '<S9>/Sum7'
     *  Switch: '<S16>/Switch2'
     */
    if (localDW->sigIdx < rtb_Switch_p_idx_2) {
      rtb_Switch_p_idx_2 = (int16_T)(rtb_Switch_p_idx_2 >> 14);
    } else {
      rtb_Switch_p_idx_2 = rtb_P;
    }

    /* End of Switch: '<S16>/Switch' */
  }

  /* RelationalOperator: '<S8>/GreaterThan' incorporates:
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  rtb_GreaterThan = (rtb_TorqueVectoringMicroContr_2 != 0);

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC1' */
  if (localDW->rtb_FixedPointSub_boundary_DT_m < 32768.0) {
    rtb_Switch_p_idx_1 = (int16_T)localDW->rtb_FixedPointSub_boundary_DT_m;
  } else {
    rtb_Switch_p_idx_1 = MAX_int16_T;
  }

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  /* Sum: '<S8>/Sum' incorporates:
   *  Gain: '<S8>/Gain'
   *  Gain: '<S8>/Gain1'
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  localDW->sigIdx = (int16_T)((7 * localDW->TorqueVectoringMicroCont_jz[2]) >> 3)
    + (int16_T)((7 * localDW->TorqueVectoringMicroCont_jz[3]) >> 3);
  rtb_TorqueVectoringMicroContr_2 = (int16_T)localDW->sigIdx;

  /* Abs: '<S8>/Abs' incorporates:
   *  Sum: '<S8>/Sum'
   */
  if ((int16_T)localDW->sigIdx < 0) {
    rtb_TorqueVectoringMicroContr_2 = (int16_T)-(int16_T)localDW->sigIdx;
  }

  /* End of Abs: '<S8>/Abs' */

  /* Switch: '<S8>/Switch' incorporates:
   *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC1'
   */
  if (rtb_TorqueVectoringMicroCont__0 > 0) {
    /* Saturate: '<S8>/Saturation1' incorporates:
     *  Abs: '<S8>/Abs'
     */
    if (rtb_TorqueVectoringMicroContr_2 >= 50) {
      /* MinMax: '<S8>/Min3' */
      localDW->sigIdx = rtb_TorqueVectoringMicroContr_2 >> 1;
    } else {
      /* MinMax: '<S8>/Min3' */
      localDW->sigIdx = 25;
    }

    /* End of Saturate: '<S8>/Saturation1' */

    /* MinMax: '<S8>/Min3' incorporates:
     *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC11'
     */
    if (localDW->sigIdx <= rtb_TorqueVectoringMicroContr_0) {
      rtb_TorqueVectoringMicroContr_2 = (int16_T)localDW->sigIdx;
    } else {
      rtb_TorqueVectoringMicroContr_2 = rtb_TorqueVectoringMicroContr_0;
    }

    /* Saturate: '<S8>/Discharge Limits' incorporates:
     *  MinMax: '<S8>/Min3'
     *  Switch: '<S8>/Switch'
     */
    if ((rtb_TorqueVectoringMicroContr_2 << 1) <= 0) {
      rtb_Sqrt = 0U;
    } else {
      rtb_Sqrt = (uint16_T)((uint16_T)rtb_TorqueVectoringMicroContr_2 << 1);
    }

    /* End of Saturate: '<S8>/Discharge Limits' */
  } else {
    /* Saturate: '<S8>/Charge Limits' incorporates:
     *  Switch: '<S8>/Switch'
     */
    rtb_Sqrt = 0U;
  }

  /* End of Switch: '<S8>/Switch' */

  /* Product: '<S8>/Product' incorporates:
   *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC1'
   *  Switch: '<S8>/Switch'
   */
  rtb_TorqueVectoringMicroCont__0 = (int16_T)((rtb_Sqrt *
    rtb_TorqueVectoringMicroCont__0) >> 15);

  /* Update for UnitDelay: '<S8>/Unit Delay' incorporates:
   *  RelationalOperator: '<S8>/Equal'
   *  Sum: '<S9>/Sum7'
   *  Switch: '<S16>/Switch2'
   */
  localDW->UnitDelay_DSTATE = (rtb_Switch_p_idx_2 == rtb_P);

  /* Signum: '<S8>/Sign' incorporates:
   *  Product: '<S8>/Product'
   */
  if (rtb_TorqueVectoringMicroCont__0 < 0) {
    rtb_TorqueVectoringMicroContr_0 = -1;
  } else {
    rtb_TorqueVectoringMicroContr_0 = (int16_T)(rtb_TorqueVectoringMicroCont__0 >
      0);
  }

  /* End of Signum: '<S8>/Sign' */

  /* Signum: '<S8>/Sign1' incorporates:
   *  UnitDelay: '<S8>/Unit Delay5'
   */
  if (localDW->UnitDelay5_DSTATE < 0) {
    rtb_P = -1;
  } else {
    rtb_P = (int16_T)(localDW->UnitDelay5_DSTATE > 0);
  }

  /* End of Signum: '<S8>/Sign1' */

  /* Update for UnitDelay: '<S8>/Unit Delay4' incorporates:
   *  Logic: '<S8>/NAND'
   *  RelationalOperator: '<S8>/Equal1'
   */
  localDW->UnitDelay4_DSTATE = ((!rtb_GreaterThan) ||
    (rtb_TorqueVectoringMicroContr_0 != rtb_P));

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_DSTATE = rtb_TmpSignalConversionAtDotP_0;
  localDW->DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_UnitDelay4;

  /* Update for UnitDelay: '<S8>/Unit Delay1' incorporates:
   *  Switch: '<S16>/Switch2'
   */
  localDW->UnitDelay1_DSTATE = rtb_Switch_p_idx_2;

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator2' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   */
  localDW->DiscreteTimeIntegrator2_DSTATE = DiscreteTimeIntegrator2;
  localDW->DiscreteTimeIntegrator2_PrevRes = (int8_T)rtb_UnitDelay4;

  /* Update for UnitDelay: '<S8>/Unit Delay5' incorporates:
   *  Product: '<S8>/Product'
   */
  localDW->UnitDelay5_DSTATE = rtb_TorqueVectoringMicroCont__0;

  /* DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC4' incorporates:
   *  DotProduct: '<S8>/Dot Product1'
   *  DotProduct: '<S8>/Dot Product2'
   *  Gain: '<S8>/Gain6'
   */
  localDW->rtb_FixedPointSub_boundary_DT_m = (real_T)((12703 *
    rtb_TorqueVectoringMicroContr_1) >> 11) * 6.103515625E-5;
  localDW->rtb_FixedPointSub_boundary_DT_c = (real_T)((12703 * u) >> 11) *
    6.103515625E-5;

  /* DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC7' incorporates:
   *  Gain: '<S8>/Gain1'
   *  Gain: '<S8>/Gain5'
   */
  rty_Tx[0] = (real_T)((15565 * localDW->TorqueVectoringMicroCont_jz[0]) >> 15) *
    0.125;
  rty_Tx[1] = (real_T)((15565 * localDW->TorqueVectoringMicroCont_jz[1]) >> 15) *
    0.125;
  rty_Tx[2] = (real_T)((15565 * localDW->TorqueVectoringMicroCont_jz[2]) >> 15) *
    0.125;
  rty_Tx[3] = (real_T)((15565 * localDW->TorqueVectoringMicroCont_jz[3]) >> 15) *
    0.125;

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* DotProduct: '<S5>/Dot Product' incorporates:
   *  Constant: '<S5>/Constant1'
   */
  localDW->CCaller_o2 = rty_Tx[0];
  localDW->Add1 = rty_Tx[1];
  localDW->CCaller_o3 = rty_Tx[2];
  localDW->CCaller_o4 = rty_Tx[3];
  localDW->CCaller_o2 = ((25.0 * localDW->CCaller_o2 + 25.0 * localDW->Add1) +
    25.0 * localDW->CCaller_o3) + 25.0 * localDW->CCaller_o4;

  /* Sum: '<S5>/Add1' incorporates:
   *  DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC5'
   *  Product: '<S8>/Product'
   */
  localDW->Add1 = (real_T)rtb_TorqueVectoringMicroCont__0 * 4.0 +
    localDW->CCaller_o2;

  /* CCaller: '<S5>/C Caller' incorporates:
   *  Bias: '<S5>/Add Constant1'
   *  Constant: '<S5>/Constant'
   *  Constant: '<S5>/Constant2'
   *  Constant: '<S5>/Constant4'
   *  DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC1'
   *  DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC3'
   *  DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC4'
   *  DotProduct: '<S5>/Dot Product1'
   *  Saturate: '<S8>/Max Torque Limiter'
   *  Sum: '<S5>/Add'
   *  Switch: '<S16>/Switch2'
   */
  localDW->CCaller_o2 = 0.0;
  localDW->CCaller_o3 = 0.0;
  localDW->CCaller_o4 = 0.0;
  localDW->CCaller_o5 = 0.0;

  /* Outputs for Atomic SubSystem: '<S2>/Torque Vectoring Micro Controller' */
  *rty_bigM_flag = bigM_func(localC->FixedPointSub_boundary_DTC6[0],
    localC->FixedPointSub_boundary_DTC6[1], localC->FixedPointSub_boundary_DTC6
    [2], localC->FixedPointSub_boundary_DTC6[3], localDW->Add1, rty_Tx[0],
    rty_Tx[1], rty_Tx[2], rty_Tx[3], (((25.0 *
    localDW->rtb_FixedPointSub_boundary_DT_m + 25.0 *
    localDW->rtb_FixedPointSub_boundary_DT_c) + 6.01806640625) +
    -6.01959228515625) + (real_T)rtb_Switch_p_idx_2 * 8.0,
    localDW->rtb_FixedPointSub_boundary_DT_m,
    localDW->rtb_FixedPointSub_boundary_DT_c, 0.24072265625, -0.24078369140625,
    24.9892578125, 24.9892578125, 24.9892578125, 24.9892578125, 25.009765625,
    25.009765625, (real_T)rtb_TorqueVectoringMicroContr_n * 0.0009765625 + 25.0,
    (real_T)rtb_Sum * 0.0009765625 + 25.0, &localDW->CCaller_o2,
    &localDW->CCaller_o3, &localDW->CCaller_o4, &localDW->CCaller_o5, 0.2);

  /* End of Outputs for SubSystem: '<S2>/Torque Vectoring Micro Controller' */

  /* Switch: '<S6>/Switch' incorporates:
   *  Bias: '<S6>/Add Constant3'
   */
  if (*rty_bigM_flag > 2) {
    rty_Tx[0] = localDW->CCaller_o2;
    rty_Tx[1] = localDW->CCaller_o3;
    rty_Tx[2] = localDW->CCaller_o4;
    rty_Tx[3] = localDW->CCaller_o5;
  } else {
    rty_Tx[0] = 25.0;
    rty_Tx[1] = 25.0;

    /* Bias: '<S6>/Add Constant3' incorporates:
     *  DataTypeConversion: '<S2>/Fixed Point Sub_boundary_DTC'
     *  DataTypeConversion: '<S2>/Torque Vectoring Micro Controller_boundary_DTC1'
     *  Gain: '<S6>/Gain'
     */
    localDW->rtb_FixedPointSub_boundary_DT_m = (real_T)rtb_Switch_p_idx_1 *
      6.103515625E-5 * 25.0 + 25.0;
    rty_Tx[2] = localDW->rtb_FixedPointSub_boundary_DT_m;
    rty_Tx[3] = localDW->rtb_FixedPointSub_boundary_DT_m;
  }

  /* End of Switch: '<S6>/Switch' */

  /* Bias: '<S6>/Add Constant2' */
  rty_Tx[0] += -24.99;
  rty_Tx[1] += -24.99;
  rty_Tx[2] += -24.99;
  rty_Tx[3] += -24.99;

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
  localDW->Add1 = rty_Tx[0] - localDW->PrevY[0];
  localDW->rtb_FixedPointSub_boundary_DT_m = rty_Tx[1] - localDW->PrevY[1];
  localDW->rtb_FixedPointSub_boundary_DT_c = rty_Tx[2] - localDW->PrevY[2];
  localDW->CCaller_o2 = rty_Tx[3] - localDW->PrevY[3];
  if (localDW->Add1 > 1.875) {
    localDW->Add1 = localDW->PrevY[0] + 1.875;
  } else if (localDW->Add1 < -4.5) {
    localDW->Add1 = localDW->PrevY[0] + -4.5;
  } else {
    localDW->Add1 = rty_Tx[0];
  }

  rty_Tx[0] = localDW->Add1;
  if (localDW->rtb_FixedPointSub_boundary_DT_m > 1.875) {
    localDW->Add1 = localDW->PrevY[1] + 1.875;
  } else if (localDW->rtb_FixedPointSub_boundary_DT_m < -4.5) {
    localDW->Add1 = localDW->PrevY[1] + -4.5;
  } else {
    localDW->Add1 = rty_Tx[1];
  }

  rty_Tx[1] = localDW->Add1;
  if (localDW->rtb_FixedPointSub_boundary_DT_c > 1.875) {
    localDW->Add1 = localDW->PrevY[2] + 1.875;
  } else if (localDW->rtb_FixedPointSub_boundary_DT_c < -4.5) {
    localDW->Add1 = localDW->PrevY[2] + -4.5;
  } else {
    localDW->Add1 = rty_Tx[2];
  }

  rty_Tx[2] = localDW->Add1;
  if (localDW->CCaller_o2 > 1.875) {
    localDW->Add1 = localDW->PrevY[3] + 1.875;
  } else if (localDW->CCaller_o2 < -4.5) {
    localDW->Add1 = localDW->PrevY[3] + -4.5;
  } else {
    localDW->Add1 = rty_Tx[3];
  }

  rty_Tx[3] = localDW->Add1;
  localDW->PrevY[0] = rty_Tx[0];
  localDW->PrevY[1] = rty_Tx[1];
  localDW->PrevY[2] = rty_Tx[2];
  localDW->PrevY[3] = rty_Tx[3];

  /* End of RateLimiter: '<S6>/Rate Limiter' */
  /* End of Outputs for SubSystem: '<S1>/Fixed Point Sub' */
}

/* Model step function */
void Electronics_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY)
{
  DW *rtDW = rtM->dwork;

  /* Outputs for Atomic SubSystem: '<Root>/Electronics' */

  /* Inport: '<Root>/brake_pressure' incorporates:
   *  Inport: '<Root>/ang_vel'
   *  Inport: '<Root>/driver_input'
   *  Inport: '<Root>/motor_temp'
   *  Inport: '<Root>/omega'
   *  Inport: '<Root>/power_limits'
   *  Inport: '<Root>/r_ref'
   *  Inport: '<Root>/steering_angle'
   *  Inport: '<Root>/vel'
   *  Outport: '<Root>/Tx'
   *  Outport: '<Root>/bigM_flag'
   */
  Electronics_a(rtU->driver_input, rtU->steering_angle, rtU->ang_vel, rtU->vel,
                rtU->power_limits, rtU->omega, rtU->motor_temp, rtU->r_ref,
                &rtY->bigM_flag, rtY->Tx, &rtConstB.Electronics_am,
                &rtDW->Electronics_am);

  /* End of Outputs for SubSystem: '<Root>/Electronics' */
}

/* Model initialize function */
void Electronics_initialize(RT_MODEL *const rtM)
{
  DW *rtDW = rtM->dwork;

  /* SystemInitialize for Atomic SubSystem: '<Root>/Electronics' */
  Electronics_Init(&rtDW->Electronics_am);

  /* End of SystemInitialize for SubSystem: '<Root>/Electronics' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
