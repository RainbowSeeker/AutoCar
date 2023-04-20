/*
 * File: Controller_types.h
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

#ifndef RTW_HEADER_Controller_types_h_
#define RTW_HEADER_Controller_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_Controller_In_Bus_
#define DEFINED_TYPEDEF_FOR_Controller_In_Bus_

typedef struct {
  uint16_T valid_num;
  uint32_T timestamp;
  real32_T savePose[150];
  int32_T lat_0;
  int32_T lon_0;
} Controller_In_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicheInfo_Bus_
#define DEFINED_TYPEDEF_FOR_VehicheInfo_Bus_

typedef struct {
  int32_T lat;
  int32_T lon;

  /* +- 1 */
  real32_T direction;
  real32_T currSteer;

  /* Psi Rate */
  real32_T currR;

  /* vx forward speed */
  real32_T currVelocity;

  /* current pose ---> (x y psi) */
  real32_T currPose[3];
} VehicheInfo_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SpeedConfig_Bus_
#define DEFINED_TYPEDEF_FOR_SpeedConfig_Bus_

typedef struct {
  real32_T StartSpeed;
  real32_T EndSpeed;
} SpeedConfig_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Controller_Out_Bus_
#define DEFINED_TYPEDEF_FOR_Controller_Out_Bus_

typedef struct {
  uint32_T timestamp;
  int16_T actuator_cmd[4];
} Controller_Out_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_s3Kybo3q7i7BAN1v6EZjhH_
#define DEFINED_TYPEDEF_FOR_struct_s3Kybo3q7i7BAN1v6EZjhH_

typedef struct {
  real32_T VEL_XY_P;
  real32_T VEL_XY_I;
  real32_T VEL_XY_D;
  real32_T VEL_Z_P;
  real32_T VEL_Z_I;
  real32_T VEL_Z_D;
  real32_T VEL_XY_I_MIN;
  real32_T VEL_XY_I_MAX;
  real32_T VEL_XY_D_MIN;
  real32_T VEL_XY_D_MAX;
  real32_T VEL_Z_I_MIN;
  real32_T VEL_Z_I_MAX;
  real32_T VEL_Z_D_MIN;
  real32_T VEL_Z_D_MAX;
  real32_T ROLL_P;
  real32_T PITCH_P;
  real32_T ROLL_PITCH_CMD_LIM;
  real32_T ROLL_RATE_P;
  real32_T PITCH_RATE_P;
  real32_T YAW_RATE_P;
  real32_T ROLL_RATE_I;
  real32_T PITCH_RATE_I;
  real32_T YAW_RATE_I;
  real32_T ROLL_RATE_D;
  real32_T PITCH_RATE_D;
  real32_T YAW_RATE_D;
  real32_T RATE_I_MIN;
  real32_T RATE_I_MAX;
  real32_T RATE_D_MIN;
  real32_T RATE_D_MAX;
  real32_T P_Q_CMD_LIM;
  real32_T R_CMD_LIM;
  real32_T KP;
  real32_T KI;
  real32_T dt;
} struct_s3Kybo3q7i7BAN1v6EZjhH;

#endif

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_Controller_T
#define typedef_cell_wrap_Controller_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_Controller_T;

#endif                                 /* typedef_cell_wrap_Controller_T */

#ifndef struct_tag_NHricsWXFJawkPMuGTMmGE
#define struct_tag_NHricsWXFJawkPMuGTMmGE

struct tag_NHricsWXFJawkPMuGTMmGE
{
  boolean_T tunablePropertyChanged[4];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  cell_wrap_Controller_T inputVarSize[6];
  real32_T ClosestPointIndex;
  real32_T NumPathSegments;
  real32_T CurrentSegmentIndex;
  real32_T SegmentStartIndex;
  real32_T SegmentEndIndex;
  real32_T RefPosesInternal[150];
  real32_T DirectionsInternal[50];
  real32_T CurvaturesInternal[50];
  real32_T VelocityProfileInternal[50];
  real32_T LastRefPoseOutput[3];
  real32_T LastRefVelocityOutput;
  real32_T LastCurvatureOutput;
  real32_T LastDirectionOutput;
};

#endif                                 /* struct_tag_NHricsWXFJawkPMuGTMmGE */

#ifndef typedef_HelperPathAnalyzer_Controller_T
#define typedef_HelperPathAnalyzer_Controller_T

typedef struct tag_NHricsWXFJawkPMuGTMmGE HelperPathAnalyzer_Controller_T;

#endif                             /* typedef_HelperPathAnalyzer_Controller_T */

#ifndef struct_tag_Xv1NUE09ukoPqH7YltEs2E
#define struct_tag_Xv1NUE09ukoPqH7YltEs2E

struct tag_Xv1NUE09ukoPqH7YltEs2E
{
  int32_T isInitialized;
  cell_wrap_Controller_T inputVarSize[2];
  real32_T RefPosesInternal[6];
  real32_T RefDirectionsInternal[2];
  real32_T LastPosesOutput[150];
  real32_T LastDirectionsOutput[50];
  real32_T LastCumLengthsOutput[50];
  real32_T LastCurvaturesOutput[50];
};

#endif                                 /* struct_tag_Xv1NUE09ukoPqH7YltEs2E */

#ifndef typedef_PathSmootherSpline_Controller_T
#define typedef_PathSmootherSpline_Controller_T

typedef struct tag_Xv1NUE09ukoPqH7YltEs2E PathSmootherSpline_Controller_T;

#endif                             /* typedef_PathSmootherSpline_Controller_T */

#ifndef struct_tag_JGGYYzag0Yb9VlDyTMJKDC
#define struct_tag_JGGYYzag0Yb9VlDyTMJKDC

struct tag_JGGYYzag0Yb9VlDyTMJKDC
{
  int32_T isInitialized;
  cell_wrap_Controller_T inputVarSize[5];
  real_T MaxSpeed;
  real32_T LastVelocities[50];
  real32_T LastStartVelocity;
  real32_T LastEndVelocity;
  real32_T LastCumLengths[50];
  real32_T LastCurvatures[50];
  real32_T LastDirections[50];
};

#endif                                 /* struct_tag_JGGYYzag0Yb9VlDyTMJKDC */

#ifndef typedef_VelocityProfiler_Controller_T
#define typedef_VelocityProfiler_Controller_T

typedef struct tag_JGGYYzag0Yb9VlDyTMJKDC VelocityProfiler_Controller_T;

#endif                               /* typedef_VelocityProfiler_Controller_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_Controller_T RT_MODEL_Controller_T;

#endif                                 /* RTW_HEADER_Controller_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
