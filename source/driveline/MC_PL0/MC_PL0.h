/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MC_PL0.h
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

#ifndef RTW_HEADER_MC_PL0_h_
#define RTW_HEADER_MC_PL0_h_
#ifndef MC_PL0_COMMON_INCLUDES_
#define MC_PL0_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* MC_PL0_COMMON_INCLUDES_ */

#include "MC_PL0_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: max_rpm
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_max_rpm[68];

  /* Expression: max_torque
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_max_torque[68];

  /* Expression: power_input_grid
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_power_input_grid[26857];

  /* Expression: rpm_sweep
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_rpm_sweep[107];

  /* Expression: torque_sweep
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_torque_sweep[251];

  /* Expression: voltage_grid
   * Referenced by: '<S1>/MATLAB Function'
   */
  real_T MATLABFunction_voltage_grid[26857];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Wxx;                          /* '<Root>/Wxx' */
  real_T Woo[2];                       /* '<Root>/Woo' */
  real_T Txx;                          /* '<Root>/Txx' */
  real_T Too[2];                       /* '<Root>/Too' */
  real_T Pmax;                         /* '<Root>/Pmax' */
  real_T Pmin;                         /* '<Root>/Pmin' */
  real_T Vbatt;                        /* '<Root>/Vbatt' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Wxxb;                         /* '<Root>/Wxxb' */
  real_T T;                            /* '<Root>/T' */
  real_T k;                            /* '<Root>/k' */
  real_T Out2;                         /* '<Root>/Out2' */
} ExtY;

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void MC_PL0_initialize(void);
extern void MC_PL0_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('mc_pl/MC_PL')    - opens subsystem mc_pl/MC_PL
 * hilite_system('mc_pl/MC_PL/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'mc_pl'
 * '<S1>'   : 'mc_pl/MC_PL'
 * '<S2>'   : 'mc_pl/MC_PL/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_MC_PL0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
