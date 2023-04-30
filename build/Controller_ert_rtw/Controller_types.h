/*
 * File: Controller_types.h
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

#ifndef RTW_HEADER_Controller_types_h_
#define RTW_HEADER_Controller_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_INS_Out_Bus_
#define DEFINED_TYPEDEF_FOR_INS_Out_Bus_

typedef struct {
  uint32_T timestamp;
  real32_T phi;
  real32_T theta;
  real32_T psi;
  real32_T quat[4];
  real32_T p;
  real32_T q;
  real32_T r;
  real32_T ax;
  real32_T ay;
  real32_T az;
  real32_T vn;
  real32_T ve;
  real32_T vd;
  real_T lat;
  real_T lon;
  real_T alt;
  real_T lat_0;
  real_T lon_0;
  real_T alt_0;
  real32_T x_R;
  real32_T y_R;
  real32_T h_R;
  uint32_T flag;
  uint32_T status;
} INS_Out_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Mission_Bus_
#define DEFINED_TYPEDEF_FOR_Mission_Bus_

typedef struct {
  /* x y bias */
  real32_T bias[2];
  uint32_T timestamp;
  uint16_T valid_num;
  real32_T poses[150];
  int32_T lat_0;
  int32_T lon_0;
} Mission_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicheInfo_Bus_
#define DEFINED_TYPEDEF_FOR_VehicheInfo_Bus_

typedef struct {
  /* current pose ---> (x y psi) */
  real32_T currPose[3];

  /* vx forward speed */
  real32_T currVelocity;

  /* Psi Rate */
  real32_T currYawRate;
  real32_T currSteer;
  real32_T direction;
} VehicheInfo_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Controller_Out_Bus_
#define DEFINED_TYPEDEF_FOR_Controller_Out_Bus_

typedef struct {
  uint32_T timestamp;
  int16_T actuator_cmd[4];
} Controller_Out_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SpeedConfig_Bus_
#define DEFINED_TYPEDEF_FOR_SpeedConfig_Bus_

typedef struct {
  real32_T StartSpeed;
  real32_T EndSpeed;
} SpeedConfig_Bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_FEfdQPjK3kgMiMOMDtkurF_
#define DEFINED_TYPEDEF_FOR_struct_FEfdQPjK3kgMiMOMDtkurF_

typedef struct {
  real32_T KP;
  real32_T KI;
  real32_T dt;
  real32_T VIEWSHED;
  real32_T CRUISE_SPEED;
} struct_FEfdQPjK3kgMiMOMDtkurF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_KC49xBu3lthzAH0AV3wmnF_
#define DEFINED_TYPEDEF_FOR_struct_KC49xBu3lthzAH0AV3wmnF_

typedef struct {
  real32_T ref_poses[150];
  uint16_T valid_num;
} struct_KC49xBu3lthzAH0AV3wmnF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_IvhNmiTT4xYgeJRZsBlIFD_
#define DEFINED_TYPEDEF_FOR_struct_IvhNmiTT4xYgeJRZsBlIFD_

typedef struct {
  real32_T poses[150];
  real32_T directions[50];
  real32_T curvatures[50];
  real32_T velocities[50];
} struct_IvhNmiTT4xYgeJRZsBlIFD;

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
