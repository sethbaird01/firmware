/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics_data.c
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

/* Block parameters (default storage) */
P rtP = {
  /* Variable: Fx_limit
   * Referenced by: '<S3>/Tire Model'
   */
  { 300.0, 300.0 },

  /* Variable: I
   * Referenced by: '<S9>/I'
   */
  60.0,

  /* Variable: P
   * Referenced by: '<S9>/P'
   */
  3.0,

  /* Variable: SL_limit
   * Referenced by: '<S3>/Tire Model'
   */
  0.3,

  /* Variable: Vth
   * Referenced by: '<S3>/Tire Model'
   */
  2.0,

  /* Variable: deadband_angle
   * Referenced by: '<S3>/Steering Model'
   */
  4.0,

  /* Variable: deadband_velocity
   * Referenced by: '<S3>/Constraint Generation'
   */
  0.5,

  /* Variable: k_limit
   * Referenced by: '<S3>/Tire Model'
   */
  3.461,

  /* Variable: max_delta_omega
   * Referenced by: '<S3>/Tire Model'
   */
  5.0,

  /* Variable: mu_factor
   * Referenced by: '<S3>/Tire Model'
   */
  0.7,

  /* Variable: yaw_deadband
   * Referenced by: '<S9>/Constant'
   */
  1.0,

  /* Variable: T
   * Referenced by: '<S9>/T'
   */
  0.0F
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
