/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: data.h
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

#ifndef RTW_HEADER_data_h_
#define RTW_HEADER_data_h_
#include "rtwtypes.h"
#include "Electronics_types.h"

/* Exported data declaration */

/* Const memory section */
/* Declaration for custom storage class: Const */
extern const real_T A1;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T A2;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T B1;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T B2;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T B3;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T C1;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T C2;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T C3;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T C4;                /* Referenced by: '<S3>/Tire Model' */
extern const real_T C_param[7];        /* Referenced by: '<S3>/Tire Model' */
extern const real_T FZ_C[7];           /* Referenced by: '<S3>/Tire Model' */
extern const real_T J_z;       /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T RE;                /* Referenced by:
                                        * '<S3>/Constraint Generation'
                                        * '<S3>/Tire Model'
                                        */
extern const real_T S1;               /* Referenced by: '<S3>/Steering Model' */
extern const real_T S2;               /* Referenced by: '<S3>/Steering Model' */
extern const real_T S3;               /* Referenced by: '<S3>/Steering Model' */
extern const real_T S4;               /* Referenced by: '<S3>/Steering Model' */
extern const real32_T T2F[4];  /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T a;                 /* Referenced by: '<S3>/Tire Model' */
extern const real_T abs_max_torque[4];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T absolute_battery_power_limits[2];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T b;                 /* Referenced by: '<S3>/Tire Model' */
extern const real_T brakecoeff[4];     /* Referenced by: '<S3>/Brake Model' */
extern const real_T c;                 /* Referenced by: '<S3>/Tire Model' */
extern const real_T d;                 /* Referenced by: '<S3>/Tire Model' */
extern const real_T dTx;               /* Referenced by:
                                        * '<S2>/Optimization '
                                        * '<S3>/Constraint Generation'
                                        */
extern const real_T deg2rad;          /* Referenced by: '<S3>/Steering Model' */
extern const real_T gr[4];             /* Referenced by:
                                        * '<S3>/Brake Model'
                                        * '<S3>/Constraint Generation'
                                        */
extern const real_T inch2mm;          /* Referenced by: '<S3>/Steering Model' */
extern const real_T l[2];              /* Referenced by:
                                        * '<S3>/Constraint Generation'
                                        * '<S3>/Reference Generation'
                                        * '<S3>/Tire Model'
                                        */
extern const real_T max_motor_temp;
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T max_rpm[68];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T max_torque[68];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T min_velocity_regen;/* Referenced by:
                                        * '<S2>/Optimization '
                                        * '<S3>/Constraint Generation'
                                        */
extern const real_T motor_efficiency[4];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T motor_enable[4];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T power_loss_grid[26857];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T power_loss_limit[4];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T rpm_index_calibration;
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T s[2];              /* Referenced by:
                                        * '<S3>/Constraint Generation'
                                        * '<S3>/Tire Model'
                                        */
extern const real_T steer_slope;      /* Referenced by: '<S3>/Steering Model' */
extern const real_T torque_sweep[251];
                               /* Referenced by: '<S3>/Constraint Generation' */
extern const real_T yaw_error_limit;   /* Referenced by: '<S2>/Optimization ' */

#endif                                 /* RTW_HEADER_data_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
