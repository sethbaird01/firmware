/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MC_PL0.c
 *
 * Code generated for Simulink model 'MC_PL0'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sat Oct 22 12:32:00 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "MC_PL0.h"
#include "rtwtypes.h"
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_roundd_snf(real_T u);
static void MC_PL(real_T rtu_In1, const real_T rtu_In1_i[2], real_T rtu_In1_k,
                  const real_T rtu_In1_im[2], real_T rtu_In1_g, real_T rtu_In1_f,
                  real_T rtu_In1_p, real_T *rty_Outport, real_T *rty_Outport_i,
                  real_T *rty_Outport_k);

/* Forward declaration for local functions */
static real_T interp2(const real_T varargin_1[107], const real_T varargin_2[251],
                      const real_T varargin_3[26857], real_T varargin_4, real_T
                      varargin_5);
static void minimum(const real_T x[251], real_T *ex, int32_T *idx);
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

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static real_T interp2(const real_T varargin_1[107], const real_T varargin_2[251],
                      const real_T varargin_3[26857], real_T varargin_4, real_T
                      varargin_5)
{
  real_T Vq;
  real_T qx1;
  real_T rx;
  real_T tmp;
  int32_T b_low_ip1;
  int32_T b_mid_i;
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;
  if (varargin_4 >= varargin_1[0]) {
    if (varargin_4 <= varargin_1[106]) {
      if (varargin_5 >= varargin_2[0]) {
        if (varargin_5 <= varargin_2[250]) {
          high_i = 107;
          low_i = 0;
          low_ip1 = 2;
          while (high_i > low_ip1) {
            b_low_ip1 = ((low_i + high_i) + 1) >> 1;
            if (varargin_4 >= varargin_1[b_low_ip1 - 1]) {
              low_i = b_low_ip1 - 1;
              low_ip1 = b_low_ip1 + 1;
            } else {
              high_i = b_low_ip1;
            }
          }

          high_i = 251;
          low_ip1 = 1;
          b_low_ip1 = 2;
          while (high_i > b_low_ip1) {
            b_mid_i = (low_ip1 + high_i) >> 1;
            if (varargin_5 >= varargin_2[b_mid_i - 1]) {
              low_ip1 = b_mid_i;
              b_low_ip1 = b_mid_i + 1;
            } else {
              high_i = b_mid_i;
            }
          }

          if (varargin_4 == varargin_1[low_i]) {
            low_i = 251 * low_i + low_ip1;
            qx1 = varargin_3[low_i - 1];
            Vq = varargin_3[low_i];
          } else {
            tmp = varargin_1[low_i + 1];
            if (tmp == varargin_4) {
              low_i = (low_i + 1) * 251 + low_ip1;
              qx1 = varargin_3[low_i - 1];
              Vq = varargin_3[low_i];
            } else {
              rx = (varargin_4 - varargin_1[low_i]) / (tmp - varargin_1[low_i]);
              tmp = varargin_3[((low_i + 1) * 251 + low_ip1) - 1];
              high_i = (251 * low_i + low_ip1) - 1;
              qx1 = varargin_3[high_i];
              if (tmp == qx1) {
                qx1 = varargin_3[high_i];
              } else {
                qx1 = (1.0 - rx) * qx1 + tmp * rx;
              }

              tmp = varargin_3[(low_i + 1) * 251 + low_ip1];
              high_i = 251 * low_i + low_ip1;
              if (tmp == varargin_3[high_i]) {
                Vq = varargin_3[high_i];
              } else {
                Vq = (1.0 - rx) * varargin_3[high_i] + tmp * rx;
              }
            }
          }

          tmp = varargin_2[low_ip1 - 1];
          if (tmp == varargin_5) {
            Vq = qx1;
          } else if (qx1 == Vq) {
            Vq = qx1;
          } else if (!(varargin_5 == varargin_2[low_ip1])) {
            rx = (varargin_5 - tmp) / (varargin_2[low_ip1] - tmp);
            Vq = (1.0 - rx) * qx1 + rx * Vq;
          }
        } else {
          Vq = (rtNaN);
        }
      } else {
        Vq = (rtNaN);
      }
    } else {
      Vq = (rtNaN);
    }
  } else {
    Vq = (rtNaN);
  }

  return Vq;
}

/* Function for MATLAB Function: '<S1>/MATLAB Function' */
static void minimum(const real_T x[251], real_T *ex, int32_T *idx)
{
  int32_T k;
  boolean_T exitg1;
  if (!rtIsNaN(x[0])) {
    *idx = 1;
  } else {
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 252)) {
      if (!rtIsNaN(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[*idx - 1];
    for (k = *idx; k < 251; k++) {
      if (*ex > x[k]) {
        *ex = x[k];
        *idx = k + 1;
      }
    }
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Output and update for atomic system: '<Root>/MC_PL' */
static void MC_PL(real_T rtu_In1, const real_T rtu_In1_i[2], real_T rtu_In1_k,
                  const real_T rtu_In1_im[2], real_T rtu_In1_g, real_T rtu_In1_f,
                  real_T rtu_In1_p, real_T *rty_Outport, real_T *rty_Outport_i,
                  real_T *rty_Outport_k)
{
  real_T d_x[251];
  real_T varargin_1[251];
  const real_T *c_x;
  const real_T *y_0;
  real_T Pmax_MC;
  real_T Pmin_MC;
  real_T T_max;
  real_T T_max_tmp;
  real_T Tr;
  real_T y;
  int32_T exitg1;
  int32_T high_i;
  int32_T low_i;
  int32_T low_ip1;
  int32_T mid_i;

  /* MATLAB Function: '<S1>/MATLAB Function' */
  *rty_Outport = rtu_In1;
  c_x = &rtConstP.MATLABFunction_max_rpm[0];
  y_0 = &rtConstP.MATLABFunction_max_torque[0];
  if ((*rty_Outport <= 0.1) || rtIsNaN(*rty_Outport)) {
    y = 0.1;
  } else {
    y = *rty_Outport;
  }

  if ((y >= 1100.0) || rtIsNaN(y)) {
    y = 1100.0;
  }

  if ((rtu_In1_i[0] <= 0.1) || rtIsNaN(rtu_In1_i[0])) {
    Tr = 0.1;
  } else {
    Tr = rtu_In1_i[0];
  }

  if ((Tr >= 1100.0) || rtIsNaN(Tr)) {
    Pmax_MC = 1100.0;
  } else {
    Pmax_MC = Tr;
  }

  if ((rtu_In1_i[1] <= 0.1) || rtIsNaN(rtu_In1_i[1])) {
    Tr = 0.1;
  } else {
    Tr = rtu_In1_i[1];
  }

  if (rtu_In1_k == 0.0) {
    Tr = 0.25;
  } else {
    if ((Tr >= 1100.0) || rtIsNaN(Tr)) {
      Tr = 1100.0;
    }

    Tr = fabs(rtu_In1_k * y / (fabs(rtu_In1_im[0] * Pmax_MC) + fabs(rtu_In1_im[1]
                * Tr)));
  }

  Pmax_MC = rtu_In1_g * Tr;
  Pmin_MC = rtu_In1_f * Tr;
  if (rtIsNaN(rtu_In1_k)) {
    T_max_tmp = rtu_In1_k;
  } else if (rtu_In1_k < 0.0) {
    T_max_tmp = -1.0;
  } else {
    T_max_tmp = (rtu_In1_k > 0.0);
  }

  Tr = T_max_tmp * interp2(rtConstP.MATLABFunction_rpm_sweep,
    rtConstP.MATLABFunction_torque_sweep,
    rtConstP.MATLABFunction_power_input_grid, y, fabs(rtu_In1_k));
  if ((!(Tr >= Pmin_MC)) && (!rtIsNaN(Pmin_MC))) {
    Tr = Pmin_MC;
  }

  if ((Pmax_MC <= Tr) || rtIsNaN(Tr)) {
    Tr = Pmax_MC;
  }

  T_max = (rtNaN);
  high_i = 0;
  do {
    exitg1 = 0;
    if (high_i < 68) {
      if (rtIsNaN(rtConstP.MATLABFunction_max_rpm[high_i])) {
        exitg1 = 1;
      } else {
        high_i++;
      }
    } else {
      if ((!(y > c_x[67])) && (!(y < c_x[0]))) {
        low_i = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (low_i + high_i) >> 1;
          if (y >= c_x[mid_i - 1]) {
            low_i = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }

        T_max_tmp = c_x[low_i - 1];
        T_max = (y - T_max_tmp) / (c_x[low_i] - T_max_tmp);
        if (T_max == 0.0) {
          T_max = y_0[low_i - 1];
        } else if (T_max == 1.0) {
          T_max = y_0[low_i];
        } else {
          T_max_tmp = y_0[low_i - 1];
          if (T_max_tmp == y_0[low_i]) {
            T_max = T_max_tmp;
          } else {
            T_max = (1.0 - T_max) * T_max_tmp + T_max * y_0[low_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  low_ip1 = (int32_T)ceil(y / 10.4719755);
  T_max_tmp = rtu_In1_k;
  if ((Tr >= Pmax_MC) || (Tr <= Pmin_MC)) {
    Pmax_MC = fabs(Tr);
    for (high_i = 0; high_i < 251; high_i++) {
      varargin_1[high_i] = fabs(rtConstP.MATLABFunction_power_input_grid
        [(low_ip1 - 1) * 251 + high_i] - Pmax_MC);
      d_x[high_i] = rtConstP.MATLABFunction_power_input_grid[low_ip1 * 251 +
        high_i] - Pmax_MC;
    }

    minimum(varargin_1, &Pmin_MC, &low_i);
    for (high_i = 0; high_i < 251; high_i++) {
      varargin_1[high_i] = fabs(d_x[high_i]);
    }

    minimum(varargin_1, &Pmax_MC, &high_i);
    Pmax_MC = ((real_T)low_ip1 - 1.0) * 10.4719755;
    if (Tr < 0.0) {
      Tr = -1.0;
    } else {
      Tr = (Tr > 0.0);
    }

    Pmin_MC = rtConstP.MATLABFunction_torque_sweep[low_i - 1];
    T_max_tmp = (Pmin_MC - (Pmin_MC -
      rtConstP.MATLABFunction_torque_sweep[high_i - 1]) * (y - Pmax_MC) /
                 ((real_T)low_ip1 * 10.4719755 - Pmax_MC)) * Tr;
  }

  if ((-T_max >= T_max_tmp) || rtIsNaN(T_max_tmp)) {
    T_max_tmp = -T_max;
  }

  if ((T_max <= T_max_tmp) || rtIsNaN(T_max_tmp)) {
    Tr = T_max;
  } else {
    Tr = T_max_tmp;
  }

  *rty_Outport = y;
  if ((rtu_In1_p <= 200.0) || rtIsNaN(rtu_In1_p)) {
    Pmax_MC = 200.0;
  } else {
    Pmax_MC = rtu_In1_p;
  }

  if ((Pmax_MC >= 340.0) || rtIsNaN(Pmax_MC)) {
    Pmax_MC = 340.0;
  }

  y = interp2(rtConstP.MATLABFunction_rpm_sweep,
              rtConstP.MATLABFunction_torque_sweep,
              rtConstP.MATLABFunction_voltage_grid, y, fabs(Tr)) / Pmax_MC;
  if (rtIsNaN(Tr)) {
    T_max_tmp = Tr;
  } else if (Tr < 0.0) {
    T_max_tmp = -1.0;
  } else {
    T_max_tmp = (Tr > 0.0);
  }

  if ((y >= 1.0) || rtIsNaN(y)) {
    y = 1.0;
  }

  *rty_Outport_k = rt_roundd_snf(T_max_tmp * y) * 100.0;
  *rty_Outport_i = Tr;

  /* End of MATLAB Function: '<S1>/MATLAB Function' */
}

/* Model step function */
void MC_PL0_step(void)
{
  /* Outputs for Atomic SubSystem: '<Root>/MC_PL' */

  /* Inport: '<Root>/Wxx' incorporates:
   *  Inport: '<Root>/Pmax'
   *  Inport: '<Root>/Pmin'
   *  Inport: '<Root>/Too'
   *  Inport: '<Root>/Txx'
   *  Inport: '<Root>/Vbatt'
   *  Inport: '<Root>/Woo'
   *  Outport: '<Root>/T'
   *  Outport: '<Root>/Wxxb'
   *  Outport: '<Root>/k'
   */
  MC_PL(rtU.Wxx, rtU.Woo, rtU.Txx, rtU.Too, rtU.Pmax, rtU.Pmin, rtU.Vbatt,
        &rtY.Wxxb, &rtY.T, &rtY.k);

  /* End of Outputs for SubSystem: '<Root>/MC_PL' */
}

/* Model initialize function */
void MC_PL0_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* ConstCode for Outport: '<Root>/Out2' */
  rtY.Out2 = 0.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
