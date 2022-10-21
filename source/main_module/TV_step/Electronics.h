/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.h
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

#ifndef RTW_HEADER_Electronics_h_
#define RTW_HEADER_Electronics_h_
#ifndef Electronics_COMMON_INCLUDES_
#define Electronics_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "bigM_v2_func.h"
#endif                                 /* Electronics_COMMON_INCLUDES_ */

#include "Electronics_types.h"
//#include "main_linker.h" 

/* Includes for objects with custom storage classes */
#include "data.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals and states (default storage) for system '<Root>/Electronics' */
typedef struct {
  real_T UnitDelay2_DSTATE[2];         /* '<S3>/Unit Delay2' */
  real_T PrevY[4];                     /* '<S2>/Rate Limiter' */
  real_T varargin_1[1004];
  real_T c_x[1004];
  real_T y[68];
  real_T x[68];
  real_T e_x[12];
  real_T x_m[8];
  real_T SA[4];
  real_T SL[4];
  real_T C[4];
  real_T RPM_index[4];
  real_T FZ[4];                        /* '<S3>/Normal Force Model' */
  real_T omega_fake[4];                /* '<S3>/Tire Model' */
  real_T Fx_max[4];                    /* '<S3>/Tire Model' */
  real_T steering[3];                  /* '<S3>/Steering Model' */
  real_T pp_coefs[24];
  real_T d_s[7];
  real_T h[6];
  real_T del[6];
  real_T UnitDelay_DSTATE;             /* '<S3>/Unit Delay' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S9>/Discrete-Time Integrator' */
  real_T UnitDelay1_DSTATE;            /* '<S3>/Unit Delay1' */
  real_T DiscreteTimeIntegrator2_DSTATE;/* '<S9>/Discrete-Time Integrator2' */
  real_T UnitDelay1_DSTATE_e;          /* '<S13>/Unit Delay1' */
  real_T UnitDelay1_DSTATE_h;          /* '<S14>/Unit Delay1' */
  real_T UnitDelay3_DSTATE;            /* '<S3>/Unit Delay3' */
  real_T UnitDelay2_DSTATE_e;          /* '<S14>/Unit Delay2' */
  real_T UnitDelay2_DSTATE_j;          /* '<S13>/Unit Delay2' */
  real_T rack_displacement;
  real_T left_steering_angle;
  real_T SA_fr;
  real_T SA_rl;
  real_T relative_PL;
  real_T Sum_j;                        /* '<S14>/Sum' */
  real_T beq;                          /* '<S3>/Constraint Generation' */
  real_T a_bar;
  real_T V_flt_idx_1;
  real_T V_rr_idx_0;
  real_T maxval_idx_3;
  real_T a_bar_idx_0;
  real_T kx_idx_0;
  real_T a_bar_idx_1;
  real_T kx_idx_1;
  real_T a_bar_idx_2;
  real_T kx_idx_2;
  real_T kx_idx_3;
  real_T mu_x;
  real_T mu_y;
  real_T mu_y_c;
  real_T V_fl_tmp;
  real_T V_rl_idx_0_tmp;
  real_T V_rl_idx_0_tmp_tmp;
  real_T a_bar_idx_1_tmp;
  real_T a_bar_idx_2_tmp;
  real_T mu_y_tmp;
  real_T a_bar_tmp;
  real_T d_k;
  real_T xtmp;
  real_T d1;
  real_T hs;
  real_T hs3;
  real_T dzzdx;
  real_T h_c;
  real_T atmp;
  real_T xnorm;
  real_T scale;
  real_T absxk;
  real_T t;
  real_T ex;
  real32_T ub[4];                      /* '<S3>/Constraint Generation' */
  real32_T lb[4];                      /* '<S3>/Constraint Generation' */
  real32_T Tx21;
  real32_T Tx22;
  real32_T Tx23;
  real32_T Tx24;
  real32_T rtb_A_idx_0;
  real32_T rtb_A_idx_1;
  int32_T iindx[4];
  int32_T b_iindx[4];
  int32_T j;
  int32_T idx;
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S9>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator2_PrevRes;/* '<S9>/Discrete-Time Integrator2' */
  boolean_T UnitDelay4_DSTATE;         /* '<S3>/Unit Delay4' */
} DW_Electronics;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_Electronics Electronics_jy;       /* '<Root>/Electronics' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T r_ref;                        /* '<Root>/r_ref' */
  real_T driver_input;                 /* '<Root>/driver_input' */
  real_T steering_angle;               /* '<Root>/steering_angle' */
  real_T brake_pressure[4];            /* '<Root>/brake_pressure' */
  real_T ang[3];                       /* '<Root>/ang' */
  real_T ang_vel[3];                   /* '<Root>/ang_vel' */
  real_T vel[2];                       /* '<Root>/vel' */
  real_T accel[3];                     /* '<Root>/accel' */
  real_T omega[4];                     /* '<Root>/omega' */
  real_T shock_displacement[4];        /* '<Root>/shock_displacement' */
  real_T shock_velocity[4];            /* '<Root>/shock_velocity' */
  real_T power_limits[2];              /* '<Root>/power_limits' */
  real_T motor_temp[4];                /* '<Root>/motor_temp' */
  real_T battery_voltage;              /* '<Root>/battery_voltage' */
  real_T FZ[4];                        /* '<Root>/FZ' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Tx[4];                        /* '<Root>/Tx' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T Fx_limit[2];                  /* Variable: Fx_limit
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T I;                            /* Variable: I
                                        * Referenced by: '<S9>/I'
                                        */
  real_T P;                            /* Variable: P
                                        * Referenced by: '<S9>/P'
                                        */
  real_T SL_limit;                     /* Variable: SL_limit
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T Vth;                          /* Variable: Vth
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T deadband_angle;               /* Variable: deadband_angle
                                        * Referenced by: '<S3>/Steering Model'
                                        */
  real_T deadband_velocity;            /* Variable: deadband_velocity
                                        * Referenced by: '<S3>/Constraint Generation'
                                        */
  real_T k_limit;                      /* Variable: k_limit
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T max_delta_omega;              /* Variable: max_delta_omega
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T mu_factor;                    /* Variable: mu_factor
                                        * Referenced by: '<S3>/Tire Model'
                                        */
  real_T yaw_deadband;                 /* Variable: yaw_deadband
                                        * Referenced by: '<S9>/Constant'
                                        */
  real32_T T;                          /* Variable: T
                                        * Referenced by: '<S9>/T'
                                        */
};

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
  DW *dwork;
};

/* Block parameters (default storage) */
extern P rtP;

static ExtU rtU;                       /* External inputs */
static ExtY rtY;                       /* External outputs */

/* TV Definitions */
static RT_MODEL rtM_;
static RT_MODEL *const rtMPtr = &rtM_; /* Real-time model */
static DW rtDW;                        /* Observable states */
static RT_MODEL *const rtM = rtMPtr;


/* Model entry point functions */
extern void Electronics_initialize(RT_MODEL *const rtM);
extern void TV_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY);
void rt_OneStep(RT_MODEL *const rtM);

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
 * hilite_system('complete_plant_v3/Electronics')    - opens subsystem complete_plant_v3/Electronics
 * hilite_system('complete_plant_v3/Electronics/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'complete_plant_v3'
 * '<S1>'   : 'complete_plant_v3/Electronics'
 * '<S2>'   : 'complete_plant_v3/Electronics/Torque Vectoring FPGA'
 * '<S3>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller'
 * '<S4>'   : 'complete_plant_v3/Electronics/Torque Vectoring FPGA/Optimization '
 * '<S5>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Brake Model'
 * '<S6>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Constraint Generation'
 * '<S7>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Discrete Varying Lowpass'
 * '<S8>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Normal Force Model'
 * '<S9>'   : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/PIT Controller'
 * '<S10>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Reference Generation'
 * '<S11>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Steering Model'
 * '<S12>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Tire Model'
 * '<S13>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Discrete Varying Lowpass/SOS1'
 * '<S14>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Discrete Varying Lowpass/SOS2'
 * '<S15>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Discrete Varying Lowpass/SOS1/Arithmetic'
 * '<S16>'  : 'complete_plant_v3/Electronics/Torque Vectoring Micro Controller/Discrete Varying Lowpass/SOS2/Arithmetic'
 */
#endif                                 /* RTW_HEADER_Electronics_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
