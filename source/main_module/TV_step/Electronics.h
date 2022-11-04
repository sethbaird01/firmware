/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.h
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

#ifndef RTW_HEADER_Electronics_h_
#define RTW_HEADER_Electronics_h_
#ifndef Electronics_COMMON_INCLUDES_
#define Electronics_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Electronics_COMMON_INCLUDES_ */

#include "Electronics_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals and states (default storage) for system '<Root>/Electronics' */
typedef struct {
  real_T UnitDelay_DSTATE[4];          /* '<S2>/Unit Delay' */
  real_T Minimum1_Valdata[4];          /* '<S15>/Minimum1' */
  real_T Minimum_Valdata[4];           /* '<S15>/Minimum' */
  real_T PrevY[4];                     /* '<S6>/Rate Limiter' */
  real_T Abs_i[204];                   /* '<S15>/Abs' */
  real_T Abs1[204];                    /* '<S15>/Abs1' */
  real_T AddConstant[4];               /* '<S5>/Add Constant' */
  real_T Minimum1[4];                  /* '<S15>/Minimum1' */
  real_T Gain8[4];                     /* '<S16>/Gain8' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S9>/Discrete-Time Integrator' */
  real_T UnitDelay1_DSTATE;            /* '<S17>/Unit Delay1' */
  real_T UnitDelay5_DSTATE;            /* '<S17>/Unit Delay5' */
  real_T CCaller_o5;                   /* '<S5>/C Caller' */
  real_T Square1;                      /* '<S27>/Square1' */
  real_T Square;                       /* '<S27>/Square' */
  real_T CCaller_o2;                   /* '<S5>/C Caller' */
  real_T Sum7;                         /* '<S9>/Sum7' */
  real_T rtb_Sum4_idx_0;
  real_T rtb_Sum4_idx_2;
  real_T rtb_Gain6_idx_1;
  real_T rtb_Sum4_idx_1;
  real_T rtb_Gain5_idx_2;
  real_T rtb_Gain5_idx_3;
  real_T rtu_TVS_Information_idx_2;
  real_T rtb_TmpSignalConversionAtDotP_m;
  real_T rtb_Gain8_tmp;
  real_T rtb_UnitDelay_idx_3;
  real_T rtb_UnitDelay_idx_2;
  real_T rtb_UnitDelay_idx_1;
  uint64_T frac;
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S9>/Discrete-Time Integrator' */
  boolean_T UnitDelay_DSTATE_e;        /* '<S17>/Unit Delay' */
  boolean_T UnitDelay4_DSTATE;         /* '<S17>/Unit Delay4' */
} DW_Electronics;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_Electronics Electronics_o4;       /* '<Root>/Electronics' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: [1:51])
   * Referenced by:
   *   '<S15>/Constant3'
   *   '<S15>/Constant5'
   */
  real_T pooled5[51];

  /* Pooled Parameter (Expression: torque_sweep)
   * Referenced by:
   *   '<S15>/Constant7'
   *   '<S15>/Constant8'
   */
  real_T pooled6[51];

  /* Pooled Parameter (Expression: power_loss_grid)
   * Referenced by:
   *   '<S15>/Constant1'
   *   '<S15>/Constant4'
   */
  real_T pooled9[5457];

  /* Expression: max_torque
   * Referenced by: '<S15>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData[68];

  /* Expression: max_rpm
   * Referenced by: '<S15>/1-D Lookup Table'
   */
  real_T uDLookupTable_bp01Data[68];

  /* Expression: max_yaw_field
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  real_T uDLookupTable_tableData_h[10];

  /* Pooled Parameter (Expression: YData)
   * Referenced by:
   *   '<S20>/Look-Up Table'
   *   '<S22>/Look-Up Table'
   */
  int32_T pooled18[1025];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T pos[2];                       /* '<Root>/pos' */
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
  real_T motor_temperature[4];         /* '<Root>/motor_temperature' */
  real_T battery_voltage;              /* '<Root>/battery_voltage' */
  real_T FZ[4];                        /* '<Root>/FZ' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  int32_T bigM_flag;                   /* '<Root>/bigM_flag' */
  real_T Tx[4];                        /* '<Root>/Tx' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T I;                            /* Variable: I
                                        * Referenced by: '<S9>/Gain'
                                        */
  real_T Ku;                           /* Variable: Ku
                                        * Referenced by: '<S10>/Gain'
                                        */
  real_T P;                            /* Variable: P
                                        * Referenced by: '<S9>/P'
                                        */
  real_T deadband_angle;               /* Variable: deadband_angle
                                        * Referenced by: '<S11>/Dead Zone'
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

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Electronics_initialize(RT_MODEL *const rtM);
extern void Electronics_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY);
void rt_OneStep(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S24>/Data Type Duplicate' : Unused code path elimination
 * Block '<S24>/Data Type Propagation' : Unused code path elimination
 * Block '<S25>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Data Type Propagation' : Unused code path elimination
 * Block '<S26>/Data Type Duplicate' : Unused code path elimination
 * Block '<S26>/Data Type Propagation' : Unused code path elimination
 * Block '<S9>/Constant1' : Unused code path elimination
 * Block '<S9>/Constant2' : Unused code path elimination
 * Block '<S9>/Product2' : Unused code path elimination
 * Block '<S9>/Product3' : Unused code path elimination
 * Block '<S9>/Sum' : Unused code path elimination
 * Block '<S9>/Sum2' : Unused code path elimination
 * Block '<S9>/Unit Delay' : Unused code path elimination
 * Block '<S10>/Constant' : Unused code path elimination
 * Block '<S10>/Divide' : Unused code path elimination
 * Block '<S10>/GreaterThan' : Unused code path elimination
 * Block '<S10>/Sim Enable' : Unused code path elimination
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
 * hilite_system('complete_plant_v6/Electronics')    - opens subsystem complete_plant_v6/Electronics
 * hilite_system('complete_plant_v6/Electronics/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'complete_plant_v6'
 * '<S1>'   : 'complete_plant_v6/Electronics'
 * '<S2>'   : 'complete_plant_v6/Electronics/Fixed Point Sub'
 * '<S3>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring FPGA'
 * '<S4>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller'
 * '<S5>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring FPGA/Subsystem'
 * '<S6>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring FPGA/Subsystem/Subsystem'
 * '<S7>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation'
 * '<S8>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Normal Force Model Fixed'
 * '<S9>'   : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/PIT Controller Fixed'
 * '<S10>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Reference Model Fixed'
 * '<S11>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model Fixed'
 * '<S12>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Tire Model Fixed'
 * '<S13>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed'
 * '<S14>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Power Constraint Fixed'
 * '<S15>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Power Limit Fixed'
 * '<S16>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Torque Boundary Fixed'
 * '<S17>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Yaw Constraint Fixed'
 * '<S18>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Cosine Lookup'
 * '<S19>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Sine Lookup'
 * '<S20>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Cosine Lookup/Cosine'
 * '<S21>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Cosine Lookup/Cosine/Handle Quarter Symmetry Cosine'
 * '<S22>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Sine Lookup/Sine'
 * '<S23>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Moment Arm Fixed/Sine Lookup/Sine/Handle Quarter Symmetry Sine'
 * '<S24>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Yaw Constraint Fixed/Saturation Dynamic'
 * '<S25>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Yaw Constraint Fixed/Saturation Dynamic1'
 * '<S26>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Yaw Constraint Fixed/Saturation Dynamic2'
 * '<S27>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model Fixed/left_steering'
 * '<S28>'  : 'complete_plant_v6/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model Fixed/right_steering'
 */
#endif                                 /* RTW_HEADER_Electronics_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
