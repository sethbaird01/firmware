/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MC_PL0.h
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

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Minimum1_Valdata[4];          /* '<S3>/Minimum1' */
  real_T PrevY[4];                     /* '<S3>/Rate Limiter' */
  real_T Abs[644];                     /* '<S3>/Abs' */
  real_T Abs1[644];                    /* '<S3>/Abs1' */
  real_T dv[8];
  real_T Switch_o[4];                  /* '<S6>/Switch' */
  real_T RateLimiter[4];               /* '<S3>/Rate Limiter' */
  real_T Switch[4];                    /* '<S5>/Switch' */
  real_T Divide_l[4];                  /* '<S4>/Divide' */
  real_T Minimum_o1[4];                /* '<S3>/Minimum' */
  real_T Minimum_o2[4];                /* '<S3>/Minimum' */
  real_T Minimum1[4];                  /* '<S3>/Minimum1' */
  real_T fractions[2];
  real_T fractions_m[2];
  real_T Sum1;                         /* '<S2>/Sum1' */
  uint32_T bpIndices[2];
  uint32_T bpIndices_c[2];
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: [1:161])
   * Referenced by:
   *   '<S3>/Constant3'
   *   '<S3>/Constant5'
   */
  real_T pooled3[161];

  /* Pooled Parameter (Expression: power_input_grid)
   * Referenced by:
   *   '<S2>/2-D Lookup Table'
   *   '<S3>/Constant1'
   *   '<S3>/Constant4'
   */
  real_T pooled4[17227];

  /* Pooled Parameter (Expression: torque_sweep)
   * Referenced by:
   *   '<S2>/2-D Lookup Table'
   *   '<S3>/Constant7'
   *   '<S3>/Constant8'
   *   '<S4>/2-D Lookup Table1'
   */
  real_T pooled6[161];

  /* Expression: max_torque
   * Referenced by: '<S3>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[68];

  /* Expression: max_rpm
   * Referenced by: '<S3>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[68];

  /* Expression: voltage_grid
   * Referenced by: '<S4>/2-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[17227];

  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S2>/2-D Lookup Table'
   *   '<S4>/2-D Lookup Table1'
   */
  uint32_T pooled9[2];

  /* Computed Parameter: Gain4_Gain
   * Referenced by: '<S3>/Gain4'
   */
  uint8_T Gain4_Gain[4];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Wx[4];                        /* '<Root>/Wx' */
  real_T Tx[4];                        /* '<Root>/Tx' */
  real_T motor_T[4];                   /* '<Root>/motor_T' */
  real_T power_limits[2];              /* '<Root>/power_limits' */
  real_T Vbatt;                        /* '<Root>/Vbatt' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T k[4];                         /* '<Root>/k' */
  real_T P_b[4];                       /* '<Root>/P' */
  real_T T[4];                         /* '<Root>/T' */
} ExtY;

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
  DW *dwork;
};

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void MC_PL0_initialize(RT_MODEL *const rtM);
extern void MC_PL0_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY);
void rt_OneStep(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Propagation' : Unused code path elimination
 * Block '<S3>/Dot Product' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Propagation' : Unused code path elimination
 */

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
 * hilite_system('complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL')    - opens subsystem complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL
 * hilite_system('complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery'
 * '<S1>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL'
 * '<S2>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Power Limiting'
 * '<S3>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Torque Limiting'
 * '<S4>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Voltage Limiting'
 * '<S5>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Power Limiting/Saturation Dynamic'
 * '<S6>'   : 'complete_plant_v6/Vehicle Model/Powertrain/Motor and Battery/MC_PL/Torque Limiting/Saturation Dynamic1'
 */
#endif                                 /* RTW_HEADER_MC_PL0_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
