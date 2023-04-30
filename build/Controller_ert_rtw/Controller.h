/*
 * File: Controller.h
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 4.69
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Sat Apr 29 00:32:15 2023
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
  real32_T Merge;                      /* '<S27>/Merge' */
  real32_T velocities[50];             /* '<S138>/MATLAB System1' */
  real32_T poses[150];                 /* '<S138>/MATLAB System' */
  real32_T directions[50];             /* '<S138>/MATLAB System' */
  real32_T curvatures[50];             /* '<S138>/MATLAB System' */
  real32_T ref_poses[150];             /* '<S2>/Vector Concatenate' */
  uint16_T valid_num;                  /* '<S2>/Saturation' */
} B_Controller_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  HelperPathAnalyzer_Controller_T obj; /* '<S15>/MATLAB System' */
  PathSmootherSpline_Controller_T obj_l;/* '<S138>/MATLAB System' */
  VelocityProfiler_Controller_T obj_n; /* '<S138>/MATLAB System1' */
  real_T goalIndex;                    /* '<S18>/Behavior Planner' */
  SpeedConfig_Bus speedConfig;         /* '<S18>/Behavior Planner' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S134>/Discrete-Time Integrator' */
  real32_T UnitDelay1_DSTATE;          /* '<S3>/Unit Delay1' */
  real32_T UnitDelay_DSTATE;           /* '<S24>/Unit Delay' */
  real32_T UnitDelay_DSTATE_m;         /* '<S3>/Unit Delay' */
  real32_T Integrator_DSTATE;          /* '<S116>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S65>/Integrator' */
  uint32_T DelayInput1_DSTATE;         /* '<S1>/Delay Input1' */
  uint32_T DiscreteTimeIntegrator_DSTATE_a;/* '<S133>/Discrete-Time Integrator' */
  real32_T endSpeed;                   /* '<S18>/Behavior Planner' */
  real32_T nextGoalPose[6];            /* '<S18>/Behavior Planner' */
  int8_T Integrator_PrevResetState;    /* '<S116>/Integrator' */
  int8_T Integrator_PrevResetState_h;  /* '<S65>/Integrator' */
  boolean_T goalIndex_not_empty;       /* '<S18>/Behavior Planner' */
  boolean_T finalReached;              /* '<S18>/Behavior Planner' */
} DW_Controller_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Sum;                    /* '<S6>/Sum' */
  const real_T ff;                     /* '<S6>/Multiply3' */
  const real_T Sum4;                   /* '<S6>/Sum4' */
  const real_T deg2rad2;               /* '<S5>/deg2rad2' */
  const real_T SinCos_o1;              /* '<S8>/SinCos' */
  const real_T SinCos_o2;              /* '<S8>/SinCos' */
  const real32_T VectorConcatenate3[3];/* '<S136>/Vector Concatenate3' */
} ConstB_Controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  INS_Out_Bus INS_Out;                 /* '<Root>/INS_Out' */
  Mission_Bus Mission_In;              /* '<Root>/Mission_In' */
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
extern struct_FEfdQPjK3kgMiMOMDtkurF CONTROL_PARAM;/* Variable: CONTROL_PARAM
                                                    * Referenced by:
                                                    *   '<S62>/Integral Gain'
                                                    *   '<S70>/Proportional Gain'
                                                    *   '<S113>/Integral Gain'
                                                    *   '<S121>/Proportional Gain'
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
 * Block '<S4>/Constant' : Unused code path elimination
 * Block '<S5>/Gain' : Unused code path elimination
 * Block '<S5>/Sum1' : Unused code path elimination
 * Block '<S17>/Signal Copy' : Eliminate redundant signal conversion block
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
 * '<S1>'   : 'Controller/Detect Change'
 * '<S2>'   : 'Controller/Mission Planner'
 * '<S3>'   : 'Controller/Track Pose'
 * '<S4>'   : 'Controller/Mission Planner/Enabled Subsystem'
 * '<S5>'   : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT'
 * '<S6>'   : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LAT2FLAT Curve'
 * '<S7>'   : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap'
 * '<S8>'   : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/Rotation'
 * '<S9>'   : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Weap Angle 180'
 * '<S10>'  : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude'
 * '<S11>'  : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Weap Angle 180/Compare To Constant1'
 * '<S12>'  : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Compare To Constant1'
 * '<S13>'  : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Weap Angle 180'
 * '<S14>'  : 'Controller/Mission Planner/Enabled Subsystem/LLA2FLAT/LatLon Wrap/Wrap Latitude/Weap Angle 180/Compare To Constant1'
 * '<S15>'  : 'Controller/Track Pose/     Stanley_Controller'
 * '<S16>'  : 'Controller/Track Pose/Actuator_Output'
 * '<S17>'  : 'Controller/Track Pose/Bus_Constructor'
 * '<S18>'  : 'Controller/Track Pose/Trajectory_Generation'
 * '<S19>'  : 'Controller/Track Pose/     Stanley_Controller/Lateral Controller Stanley'
 * '<S20>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley'
 * '<S21>'  : 'Controller/Track Pose/     Stanley_Controller/Subsystem'
 * '<S22>'  : 'Controller/Track Pose/     Stanley_Controller/Lateral Controller Stanley/Dynamic'
 * '<S23>'  : 'Controller/Track Pose/     Stanley_Controller/Lateral Controller Stanley/Kinematic'
 * '<S24>'  : 'Controller/Track Pose/     Stanley_Controller/Lateral Controller Stanley/Dynamic/Dynamic Enabled'
 * '<S25>'  : 'Controller/Track Pose/     Stanley_Controller/Lateral Controller Stanley/Dynamic/Dynamic Enabled/Radians to Degrees'
 * '<S26>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/Convert Command '
 * '<S27>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller'
 * '<S28>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/Verify Direction'
 * '<S29>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward'
 * '<S30>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse'
 * '<S31>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward '
 * '<S32>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup'
 * '<S33>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /D Gain'
 * '<S34>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter'
 * '<S35>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter ICs'
 * '<S36>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /I Gain'
 * '<S37>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain'
 * '<S38>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain Fdbk'
 * '<S39>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator'
 * '<S40>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator ICs'
 * '<S41>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Copy'
 * '<S42>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Gain'
 * '<S43>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /P Copy'
 * '<S44>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Parallel P Gain'
 * '<S45>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Reset Signal'
 * '<S46>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation'
 * '<S47>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation Fdbk'
 * '<S48>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum'
 * '<S49>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum Fdbk'
 * '<S50>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode'
 * '<S51>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode Sum'
 * '<S52>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Integral'
 * '<S53>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Ngain'
 * '<S54>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /postSat Signal'
 * '<S55>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /preSat Signal'
 * '<S56>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel'
 * '<S57>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S58>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S59>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /D Gain/Disabled'
 * '<S60>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter/Disabled'
 * '<S61>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Filter ICs/Disabled'
 * '<S62>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /I Gain/Internal Parameters'
 * '<S63>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain/Passthrough'
 * '<S64>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Ideal P Gain Fdbk/Disabled'
 * '<S65>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator/Discrete'
 * '<S66>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Integrator ICs/Internal IC'
 * '<S67>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Copy/Disabled wSignal Specification'
 * '<S68>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /N Gain/Disabled'
 * '<S69>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /P Copy/Disabled'
 * '<S70>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Parallel P Gain/Internal Parameters'
 * '<S71>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Reset Signal/External Reset'
 * '<S72>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation/Enabled'
 * '<S73>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Saturation Fdbk/Disabled'
 * '<S74>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum/Sum_PI'
 * '<S75>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Sum Fdbk/Disabled'
 * '<S76>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode/Disabled'
 * '<S77>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tracking Mode Sum/Passthrough'
 * '<S78>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Integral/Passthrough'
 * '<S79>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /Tsamp - Ngain/Passthrough'
 * '<S80>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /postSat Signal/Forward_Path'
 * '<S81>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Forward/PI Forward /preSat Signal/Forward_Path'
 * '<S82>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse'
 * '<S83>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup'
 * '<S84>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/D Gain'
 * '<S85>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter'
 * '<S86>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter ICs'
 * '<S87>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/I Gain'
 * '<S88>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain'
 * '<S89>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain Fdbk'
 * '<S90>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator'
 * '<S91>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator ICs'
 * '<S92>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Copy'
 * '<S93>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Gain'
 * '<S94>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/P Copy'
 * '<S95>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Parallel P Gain'
 * '<S96>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Reset Signal'
 * '<S97>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation'
 * '<S98>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation Fdbk'
 * '<S99>'  : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum'
 * '<S100>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum Fdbk'
 * '<S101>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode'
 * '<S102>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode Sum'
 * '<S103>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Integral'
 * '<S104>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Ngain'
 * '<S105>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/postSat Signal'
 * '<S106>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/preSat Signal'
 * '<S107>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel'
 * '<S108>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S109>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S110>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/D Gain/Disabled'
 * '<S111>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter/Disabled'
 * '<S112>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Filter ICs/Disabled'
 * '<S113>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/I Gain/Internal Parameters'
 * '<S114>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain/Passthrough'
 * '<S115>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Ideal P Gain Fdbk/Disabled'
 * '<S116>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator/Discrete'
 * '<S117>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Integrator ICs/Internal IC'
 * '<S118>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Copy/Disabled wSignal Specification'
 * '<S119>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/N Gain/Disabled'
 * '<S120>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/P Copy/Disabled'
 * '<S121>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Parallel P Gain/Internal Parameters'
 * '<S122>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Reset Signal/External Reset'
 * '<S123>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation/Enabled'
 * '<S124>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Saturation Fdbk/Disabled'
 * '<S125>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum/Sum_PI'
 * '<S126>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Sum Fdbk/Disabled'
 * '<S127>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode/Disabled'
 * '<S128>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tracking Mode Sum/Passthrough'
 * '<S129>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Integral/Passthrough'
 * '<S130>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/Tsamp - Ngain/Passthrough'
 * '<S131>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/postSat Signal/Forward_Path'
 * '<S132>' : 'Controller/Track Pose/     Stanley_Controller/Longitudinal Controller Stanley/PI Controller/Reverse/PI Reverse/preSat Signal/Forward_Path'
 * '<S133>' : 'Controller/Track Pose/Actuator_Output/Counter'
 * '<S134>' : 'Controller/Track Pose/Bus_Constructor/Delayed Steering System'
 * '<S135>' : 'Controller/Track Pose/Bus_Constructor/Psi To DCM'
 * '<S136>' : 'Controller/Track Pose/Bus_Constructor/Psi To DCM/Rotation Matrix Z'
 * '<S137>' : 'Controller/Track Pose/Trajectory_Generation/Behavior Planner'
 * '<S138>' : 'Controller/Track Pose/Trajectory_Generation/Trajectory Generation'
 * '<S139>' : 'Controller/Track Pose/Trajectory_Generation/Trajectory Generation/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_Controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
