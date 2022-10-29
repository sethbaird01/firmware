/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Electronics.h
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
  real_T PrevY[4];                     /* '<S6>/Rate Limiter' */
  real_T CCaller_o2;                   /* '<S5>/C Caller' */
  real_T CCaller_o3;                   /* '<S5>/C Caller' */
  real_T CCaller_o4;                   /* '<S5>/C Caller' */
  real_T CCaller_o5;                   /* '<S5>/C Caller' */
  real_T Add1;                         /* '<S5>/Add1' */
  real_T rtb_FixedPointSub_boundary_DT_m;
  real_T rtb_FixedPointSub_boundary_DT_c;
  uint64_T frac;
  int32_T sigIdx;
  uint32_T Minimum[4];                 /* '<S18>/Minimum' */
  uint32_T Minimum1[4];                /* '<S18>/Minimum1' */
  int16_T Minimum_Valdata[4];          /* '<S18>/Minimum' */
  int16_T Minimum1_Valdata[4];         /* '<S18>/Minimum1' */
  int16_T Abs1[204];                   /* '<S18>/Abs1' */
  int16_T Abs[204];                    /* '<S18>/Abs' */
  int16_T TorqueVectoringMicroCont_jz[4];
                    /* '<S2>/Torque Vectoring Micro Controller_boundary_DTC8' */
  int16_T Sum[4];                      /* '<S18>/Sum' */
  int16_T DiscreteTimeIntegrator_DSTATE;/* '<S9>/Discrete-Time Integrator' */
  int16_T DiscreteTimeIntegrator2_DSTATE;/* '<S9>/Discrete-Time Integrator2' */
  int16_T UnitDelay5_DSTATE;           /* '<S8>/Unit Delay5' */
  int16_T UnitDelay1_DSTATE;           /* '<S8>/Unit Delay1' */
  uint16_T Min1[4];                    /* '<S8>/Min1' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S9>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator2_PrevRes;/* '<S9>/Discrete-Time Integrator2' */
  boolean_T UnitDelay_DSTATE;          /* '<S8>/Unit Delay' */
  boolean_T UnitDelay4_DSTATE;         /* '<S8>/Unit Delay4' */
} DW_Electronics;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_Electronics Electronics_am;       /* '<Root>/Electronics' */
} DW;

/* Invariant block signals for system '<Root>/Electronics' */
typedef struct {
  const real_T FixedPointSub_boundary_DTC6[4];
                                      /* '<S2>/Fixed Point Sub_boundary_DTC6' */
} ConstB_Electronics;

/* Invariant block signals (default storage) */
typedef struct {
  ConstB_Electronics Electronics_am;   /* '<Root>/Electronics' */
} ConstB;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Gain1_Gain_n
   * Referenced by: '<S8>/Gain1'
   */
  int16_T Gain1_Gain_n[4];

  /* Pooled Parameter (Expression: YData)
   * Referenced by:
   *   '<S19>/Look-Up Table'
   *   '<S21>/Look-Up Table'
   */
  int16_T pooled6[129];

  /* Pooled Parameter (Expression: power_loss_grid)
   * Referenced by:
   *   '<S18>/Constant1'
   *   '<S18>/Constant4'
   */
  int16_T pooled10[5457];

  /* Pooled Parameter (Expression: -power_loss_limit.*ones(51, 4))
   * Referenced by:
   *   '<S18>/Bias1'
   *   '<S18>/Bias2'
   */
  int16_T pooled11[204];

  /* Computed Parameter: uDLookupTable_bp01Data
   * Referenced by: '<S8>/1-D Lookup Table'
   */
  int16_T uDLookupTable_bp01Data[68];

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S10>/1-D Lookup Table'
   */
  int16_T uDLookupTable_tableData[10];

  /* Pooled Parameter (Expression: torque_sweep)
   * Referenced by:
   *   '<S18>/Constant7'
   *   '<S18>/Constant8'
   */
  uint16_T pooled15[51];

  /* Computed Parameter: uDLookupTable_tableData_i
   * Referenced by: '<S8>/1-D Lookup Table'
   */
  uint16_T uDLookupTable_tableData_i[68];

  /* Pooled Parameter (Expression: [1:51])
   * Referenced by:
   *   '<S18>/Constant3'
   *   '<S18>/Constant5'
   */
  uint16_T pooled19[51];
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
  real_T motor_temp[4];                /* '<Root>/motor_temp' */
  real_T battery_voltage;              /* '<Root>/battery_voltage' */
  real_T FZ[4];                        /* '<Root>/FZ' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  int32_T bigM_flag;                   /* '<Root>/bigM_flag' */
  real_T Tx[4];                        /* '<Root>/Tx' */
} ExtY;

/* Code_Instrumentation_Declarations_Placeholder */

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
  DW *dwork;
};

extern const ConstB rtConstB;          /* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Electronics_initialize(RT_MODEL *const rtM);
extern void Electronics_step(RT_MODEL *const rtM, ExtU *rtU, ExtY *rtY);
void rt_OneStep(RT_MODEL *const rtM);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S16>/Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/Saturation' : Unused code path elimination
 * Block '<S13>/Constant1' : Unused code path elimination
 * Block '<S13>/Constant2' : Unused code path elimination
 * Block '<S13>/Constant3' : Unused code path elimination
 * Block '<S13>/Gain1' : Unused code path elimination
 * Block '<S13>/Gain2' : Unused code path elimination
 * Block '<S13>/Matrix Multiply' : Unused code path elimination
 * Block '<S13>/Sum' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC10' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC13' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC14' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC4' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC7' : Unused code path elimination
 * Block '<S2>/Torque Vectoring Micro Controller_boundary_DTC9' : Unused code path elimination
 * Block '<S8>/Bias' : Eliminated nontunable bias of 0
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
 * hilite_system('complete_plant_v5/Electronics')    - opens subsystem complete_plant_v5/Electronics
 * hilite_system('complete_plant_v5/Electronics/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'complete_plant_v5'
 * '<S1>'   : 'complete_plant_v5/Electronics'
 * '<S2>'   : 'complete_plant_v5/Electronics/Fixed Point Sub'
 * '<S3>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring FPGA'
 * '<S4>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller'
 * '<S5>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring FPGA/Subsystem'
 * '<S6>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring FPGA/Subsystem/Subsystem'
 * '<S7>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Brake Model'
 * '<S8>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation'
 * '<S9>'   : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/PIT Controller'
 * '<S10>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Reference Model'
 * '<S11>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model'
 * '<S12>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Subsystem'
 * '<S13>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Subsystem1'
 * '<S14>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Cosine Lookup'
 * '<S15>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Find M_Limit'
 * '<S16>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Saturation Dynamic'
 * '<S17>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Sine Lookup'
 * '<S18>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Subsystem1'
 * '<S19>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Cosine Lookup/Cosine'
 * '<S20>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Cosine Lookup/Cosine/Handle Quarter Symmetry Cosine'
 * '<S21>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Sine Lookup/Sine'
 * '<S22>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Constraint Generation/Sine Lookup/Sine/Handle Quarter Symmetry Sine'
 * '<S23>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model/left_steering'
 * '<S24>'  : 'complete_plant_v5/Electronics/Fixed Point Sub/Torque Vectoring Micro Controller/Steering Model/right_steering'
 */
#endif                                 /* RTW_HEADER_Electronics_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
