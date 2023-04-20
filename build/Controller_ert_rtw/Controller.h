/*
 * File: Controller.h
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 4.11
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Thu Apr 20 23:18:22 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Controller_h_
#define RTW_HEADER_Controller_h_
#ifndef Controller_COMMON_INCLUDES_
#define Controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Controller_COMMON_INCLUDES_ */

#include "Controller_types.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T v_data[10000000];
  real32_T v_data_m[10000000];
  real32_T tInteg_data[10000000];
  real32_T intervalcumPathLength_data[10000000];
  real32_T v_data_c[10000000];
  real32_T v_data_k[10000000];
  real32_T v_data_cx[10000000];
  real32_T v_data_b[10000000];
  real32_T v_data_p[10000000];
  real32_T tmp_data[10000000];
  real32_T in1_data[10000000];
  real32_T Merge;                      /* '<S11>/Merge' */
  real32_T DataTypeConversion[2];      /* '<S121>/Data Type Conversion' */
} B_Controller_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  HelperPathAnalyzer_Controller_T obj; /* '<S1>/MATLAB System' */
  PathSmootherSpline_Controller_T obj_l;/* '<S119>/MATLAB System' */
  VelocityProfiler_Controller_T obj_n; /* '<S119>/MATLAB System1' */
  real_T goalIndex;                    /* '<S2>/Behavior Planner' */
  SpeedConfig_Bus speedConfig;         /* '<S2>/Behavior Planner' */
  real32_T Integrator_DSTATE;          /* '<S100>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S49>/Integrator' */
  uint32_T DelayInput1_DSTATE;         /* '<S120>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE;/* '<S133>/Discrete-Time Integrator' */
  real32_T endSpeed;                   /* '<S2>/Behavior Planner' */
  real32_T nextGoalPose[6];            /* '<S2>/Behavior Planner' */
  int8_T Integrator_PrevResetState;    /* '<S100>/Integrator' */
  int8_T Integrator_PrevResetState_h;  /* '<S49>/Integrator' */
  boolean_T goalIndex_not_empty;       /* '<S2>/Behavior Planner' */
  boolean_T finalReached;              /* '<S2>/Behavior Planner' */
} DW_Controller_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Sum;                    /* '<S123>/Sum' */
  const real_T ff;                     /* '<S123>/Multiply3' */
  const real_T Sum4;                   /* '<S123>/Sum4' */
  const real_T deg2rad2;               /* '<S122>/deg2rad2' */
  const real_T SinCos_o1;              /* '<S125>/SinCos' */
  const real_T SinCos_o2;              /* '<S125>/SinCos' */
} ConstB_Controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  Controller_In_Bus Controller_In;     /* '<Root>/Controller_In' */
  VehicheInfo_Bus VehicheInfo;         /* '<Root>/VehicheInfo' */
} ExtU_Controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  Controller_Out_Bus Controller_Out;   /* '<Root>/Controller_Out' */
} ExtY_Controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_Controller_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_Controller_T Controller_B;

/* Block states (default storage) */
extern DW_Controller_T Controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Controller_T Controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Controller_T Controller_Y;
extern const ConstB_Controller_T Controller_ConstB;/* constant block i/o */

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern struct_s3Kybo3q7i7BAN1v6EZjhH CONTROL_PARAM;/* Variable: CONTROL_PARAM
                                                    * Referenced by:
                                                    *   '<S46>/Integral Gain'
                                                    *   '<S54>/Proportional Gain'
                                                    *   '<S97>/Integral Gain'
                                                    *   '<S105>/Proportional Gain'
                                                    */

/* Model entry point functions */
extern void Controller_initialize(void);
extern void Controller_step(void);
extern void Controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Controller_T *const Controller_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S121>/Constant' : Unused code path elimination
 * Block '<S122>/Gain' : Unused code path elimination
 * Block '<S122>/Sum1' : Unused code path elimination
 * Block '<S11>/Data Type Conversion' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Controller'
 * '<S1>'   : 'Controller/Stanley Controller'
 * '<S2>'   : 'Controller/Track Pose'
 * '<S3>'   : 'Controller/actuator_output'
 * '<S4>'   : 'Controller/Stanley Controller/Lateral Controller Stanley'
 * '<S5>'   : 'Controller/Stanley Controller/Longitudinal Controller Stanley'
 * '<S6>'   : 'Controller/Stanley Controller/Subsystem'
 * '<S7>'   : 'Controller/Stanley Controller/Lateral Controller Stanley/Dynamic'
 * '<S8>'   : 'Controller/Stanley Controller/Lateral Controller Stanley/Kinematic'
 * '<S9>'   : 'Controller/Stanley Controller/Lateral Controller Stanley/Dynamic/Dynamic Disabled'
 * '<S10>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/Convert Command '
 * '<S11>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller'
 * '<S12>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/Verify Direction'
 * '<S13>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward'
 * '<S14>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse'
 * '<S15>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward '
 * '<S16>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup'
 * '<S17>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /D Gain'
 * '<S18>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter'
 * '<S19>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter ICs'
 * '<S20>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /I Gain'
 * '<S21>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain'
 * '<S22>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain Fdbk'
 * '<S23>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator'
 * '<S24>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator ICs'
 * '<S25>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Copy'
 * '<S26>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Gain'
 * '<S27>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /P Copy'
 * '<S28>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Parallel P Gain'
 * '<S29>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Reset Signal'
 * '<S30>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation'
 * '<S31>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation Fdbk'
 * '<S32>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum'
 * '<S33>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum Fdbk'
 * '<S34>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode'
 * '<S35>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode Sum'
 * '<S36>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Integral'
 * '<S37>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Ngain'
 * '<S38>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /postSat Signal'
 * '<S39>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /preSat Signal'
 * '<S40>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel'
 * '<S41>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S42>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S43>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /D Gain/Disabled'
 * '<S44>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter/Disabled'
 * '<S45>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter ICs/Disabled'
 * '<S46>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /I Gain/Internal Parameters'
 * '<S47>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain/Passthrough'
 * '<S48>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain Fdbk/Disabled'
 * '<S49>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator/Discrete'
 * '<S50>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator ICs/Internal IC'
 * '<S51>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Copy/Disabled wSignal Specification'
 * '<S52>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Gain/Disabled'
 * '<S53>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /P Copy/Disabled'
 * '<S54>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Parallel P Gain/Internal Parameters'
 * '<S55>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Reset Signal/External Reset'
 * '<S56>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation/Enabled'
 * '<S57>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation Fdbk/Disabled'
 * '<S58>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum/Sum_PI'
 * '<S59>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum Fdbk/Disabled'
 * '<S60>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode/Disabled'
 * '<S61>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode Sum/Passthrough'
 * '<S62>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Integral/Passthrough'
 * '<S63>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Ngain/Passthrough'
 * '<S64>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /postSat Signal/Forward_Path'
 * '<S65>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /preSat Signal/Forward_Path'
 * '<S66>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse'
 * '<S67>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup'
 * '<S68>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/D Gain'
 * '<S69>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter'
 * '<S70>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter ICs'
 * '<S71>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/I Gain'
 * '<S72>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain'
 * '<S73>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain Fdbk'
 * '<S74>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator'
 * '<S75>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator ICs'
 * '<S76>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Copy'
 * '<S77>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Gain'
 * '<S78>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/P Copy'
 * '<S79>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Parallel P Gain'
 * '<S80>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Reset Signal'
 * '<S81>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation'
 * '<S82>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation Fdbk'
 * '<S83>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum'
 * '<S84>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum Fdbk'
 * '<S85>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode'
 * '<S86>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode Sum'
 * '<S87>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Integral'
 * '<S88>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Ngain'
 * '<S89>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/postSat Signal'
 * '<S90>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/preSat Signal'
 * '<S91>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel'
 * '<S92>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S93>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S94>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/D Gain/Disabled'
 * '<S95>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter/Disabled'
 * '<S96>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter ICs/Disabled'
 * '<S97>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/I Gain/Internal Parameters'
 * '<S98>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain/Passthrough'
 * '<S99>'  : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain Fdbk/Disabled'
 * '<S100>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator/Discrete'
 * '<S101>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator ICs/Internal IC'
 * '<S102>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Copy/Disabled wSignal Specification'
 * '<S103>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Gain/Disabled'
 * '<S104>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/P Copy/Disabled'
 * '<S105>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Parallel P Gain/Internal Parameters'
 * '<S106>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Reset Signal/External Reset'
 * '<S107>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation/Enabled'
 * '<S108>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation Fdbk/Disabled'
 * '<S109>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum/Sum_PI'
 * '<S110>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum Fdbk/Disabled'
 * '<S111>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode/Disabled'
 * '<S112>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode Sum/Passthrough'
 * '<S113>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Integral/Passthrough'
 * '<S114>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Ngain/Passthrough'
 * '<S115>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/postSat Signal/Forward_Path'
 * '<S116>' : 'Controller/Stanley Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/preSat Signal/Forward_Path'
 * '<S117>' : 'Controller/Track Pose/Behavior Planner'
 * '<S118>' : 'Controller/Track Pose/Path Planner'
 * '<S119>' : 'Controller/Track Pose/Trajectory Generation'
 * '<S120>' : 'Controller/Track Pose/Path Planner/Detect Change'
 * '<S121>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem'
 * '<S122>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT'
 * '<S123>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LAT2FLAT Curve'
 * '<S124>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap'
 * '<S125>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/Rotation'
 * '<S126>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Weap Angle 180'
 * '<S127>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude'
 * '<S128>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Weap Angle 180/Compare To Constant1'
 * '<S129>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Compare To Constant1'
 * '<S130>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Weap Angle 180'
 * '<S131>' : 'Controller/Track Pose/Path Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Weap Angle 180/Compare To Constant1'
 * '<S132>' : 'Controller/Track Pose/Trajectory Generation/MATLAB Function'
 * '<S133>' : 'Controller/actuator_output/Subsystem1'
 */
#endif                                 /* RTW_HEADER_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
