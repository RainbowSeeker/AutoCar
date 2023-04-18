/*
 * File: Controller.c
 *
 * Code generated for Simulink model 'Controller'.
 *
 * Model version                  : 4.15
 * Simulink Coder version         : 9.8 (R2022b) 13-May-2022
 * C/C++ source code generated on : Tue Apr 18 21:27:54 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Controller.h"
#include "rtwtypes.h"
#include "Controller_types.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "Controller_private.h"
#include <string.h>
#include "rt_assert.h"
#include <float.h>
#include "rt_defines.h"
#include <stdlib.h>
#include <stddef.h>

/* Exported block parameters */
struct_V8yFhBVIWEAg7ttd1ermIG CONTROL_PARAM = {
  2.0F,
  0.5F,
  0.02F
} ;                                    /* Variable: CONTROL_PARAM
                                        * Referenced by:
                                        *   '<S46>/Integral Gain'
                                        *   '<S54>/Proportional Gain'
                                        *   '<S97>/Integral Gain'
                                        *   '<S105>/Proportional Gain'
                                        */

/* Block signals (default storage) */
B_Controller_T Controller_B;

/* Block states (default storage) */
DW_Controller_T Controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Controller_T Controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Controller_T Controller_Y;

/* Real-time model */
static RT_MODEL_Controller_T Controller_M_;
RT_MODEL_Controller_T *const Controller_M = &Controller_M_;

/* Forward declaration for local functions */
static void Controller_filterPoses(const real32_T refPoses[6], const real32_T
  refDirections[2], real32_T poses_data[], int32_T poses_size[2], real32_T
  directions_data[], int32_T *directions_size);
static void Controller_binary_expand_op_l5x(boolean_T in1_data[], int32_T
  *in1_size, const real32_T in2_data[], int32_T in3, int32_T in4, int32_T in5);
static void Co_getSegmentBoundaryPointIndex(const real32_T directions_data[],
  const int32_T *directions_size, real_T segStartIdx_data[], int32_T
  *segStartIdx_size, real_T segEndIdx_data[], int32_T *segEndIdx_size);
static void Controll_binary_expand_op_l5xxx(real32_T in1_data[], int32_T
  *in1_size, const real32_T in3_data[], int32_T in4, int32_T in5, int32_T in6);
static void Controlle_binary_expand_op_l5xx(real32_T in1_data[], int32_T
  *in1_size, const real32_T in3_data[], const int32_T in3_size[2], int32_T in4,
  int32_T in5, int32_T in6);
static void Controller_plus(real32_T in1_data[], int32_T *in1_size, const
  real32_T in2_data[], const int32_T *in2_size);
static void Co_computeCumulativeChordLength(const real32_T poses_data[], const
  int32_T poses_size[2], real32_T cumChordLength_data[], int32_T
  *cumChordLength_size);
static void Controller_emxInit_real32_T(emxArray_real32_T_Controller_T
  **pEmxArray, int32_T numDimensions);
static void Controller_spline(const real32_T x_data[], const int32_T *x_size,
  const real32_T y_data[], const int32_T *y_size, real32_T output_breaks_data[],
  int32_T output_breaks_size[2], real32_T output_coefs_data[], int32_T
  output_coefs_size[2]);
static void Controller_bsxfun(const real32_T a_data[], const int32_T a_size[2],
  real32_T c_data[], int32_T c_size[2]);
static void Cont_emxEnsureCapacity_real32_T(emxArray_real32_T_Controller_T
  *emxArray, int32_T oldNumel);
static void Controller_linspace(real32_T d2, real32_T n,
  emxArray_real32_T_Controller_T *y);
static int32_T Controller_bsearch(const real32_T x_data[], const int32_T x_size
  [2], real32_T xi);
static void Controller_ppval(const real32_T pp_breaks_data[], const int32_T
  pp_breaks_size[2], const real32_T pp_coefs_data[], const int32_T
  pp_coefs_size[2], const real32_T x_data[], const int32_T x_size[2], real32_T
  v_data[], int32_T v_size[2]);
static void Controller_expand_atan2(const real32_T a_data[], const int32_T
  a_size[2], const real32_T b_data[], const int32_T b_size[2], real32_T c_data[],
  int32_T c_size[2]);
static void Controller_emxFree_real32_T(emxArray_real32_T_Controller_T
  **pEmxArray);
static void Controller_plus_l(emxArray_real32_T_Controller_T *in1, const
  emxArray_real32_T_Controller_T *in2);
static void Controller_cumtrapz(const emxArray_real32_T_Controller_T *x,
  emxArray_real32_T_Controller_T *z);
static void Con_computeCumulativePathLength(const real32_T splineDx_breaks_data[],
  const int32_T splineDx_breaks_size[2], const real32_T splineDx_coefs_data[],
  const real32_T splineDy_breaks_data[], const int32_T splineDy_breaks_size[2],
  const real32_T splineDy_coefs_data[], const real32_T tQuery_data[], const
  int32_T tQuery_size[2], real32_T cumLengths_data[], int32_T cumLengths_size[2]);
static void Controller_bsxfun_l(const real32_T a_data[], const int32_T a_size[2],
  real32_T c_data[], int32_T c_size[2]);
static void Contro_binary_expand_op_l5xxx52(real32_T in1_data[], int32_T
  in1_size[2], const emxArray_real32_T_Controller_T *in3, const
  emxArray_real32_T_Controller_T *in4);
static void Control_binary_expand_op_l5xxx5(real32_T in1_data[], int32_T
  in1_size[2], const emxArray_real32_T_Controller_T *in2, const
  emxArray_real32_T_Controller_T *in3, const real32_T in4_data[], const int32_T
  in4_size[2], const real32_T in5_data[], const int32_T in5_size[2]);
static void Controlle_computePathCurvatures(const real32_T splineDx_breaks_data[],
  const int32_T splineDx_breaks_size[2], const real32_T splineDx_coefs_data[],
  const int32_T splineDx_coefs_size[3], const real32_T splineDy_breaks_data[],
  const int32_T splineDy_breaks_size[2], const real32_T splineDy_coefs_data[],
  const int32_T splineDy_coefs_size[3], const real32_T tQuery_data[], const
  int32_T tQuery_size[2], real32_T curvatures_data[], int32_T curvatures_size[2]);
static void Controller_smoothPathSpline(const real32_T refPoses[6], const
  real32_T refDirections[2], real32_T poses[150], real32_T directions[50],
  real32_T varargout_1[50], real32_T varargout_2[50]);
static boolean_T Controller_isequal(const real32_T varargin_1[50], const
  real32_T varargin_2[50]);
static void getSegmentBoundaryPointIndex_l(const real32_T directions[50], real_T
  segStartIdx_data[], int32_T *segStartIdx_size, real_T segEndIdx_data[],
  int32_T *segEndIdx_size);
static real32_T Controller_minimum(const real32_T x_data[], const int32_T
  *x_size);
static void getNonConstSpeedIntervalDistanc(real32_T vBound, real32_T vMax,
  real32_T S[3]);
static void getNonConstSpeedIntervalDista_l(real32_T vBound, real32_T vMax,
  real32_T S[3]);
static real32_T Controller_sum(const real32_T x[3]);
static boolean_T Controller_ifWhileCond(const boolean_T x_data[], const int32_T
  x_size[2]);
static void Controller_xzlartg(const creal32_T f, const creal32_T g, real32_T
  *cs, creal32_T *sn, creal32_T *r);
static creal32_T Controller_sqrt(const creal32_T x);
static void Controller_xzlartg_l(const creal32_T f, const creal32_T g, real32_T *
  cs, creal32_T *sn);
static void Controller_xzhgeqz(const creal32_T A_data[], const int32_T A_size[2],
  int32_T ilo, int32_T ihi, int32_T *info, creal32_T alpha1_data[], int32_T
  *alpha1_size, creal32_T beta1_data[], int32_T *beta1_size);
static void Controller_xzgeev(const real32_T A_data[], const int32_T A_size[2],
  int32_T *info, creal32_T alpha1_data[], int32_T *alpha1_size, creal32_T
  beta1_data[], int32_T *beta1_size);
static boolean_T Controller_cplxpairv_l(creal32_T x_data[], const int32_T
  *x_size, real32_T tol);
static void Controller_roots(const real32_T c[4], creal32_T r_data[], int32_T
  *r_size);
static void generateSegmentVelocityProfile_(const real32_T sPoints[8], real32_T
  vMax, const real32_T vPoints[8], const real32_T tPoints[8], real32_T x,
  real32_T *varargout_1, real32_T *varargout_2);
static void Controller_arrayfun(const real32_T fun_workspace_sPoints[8],
  real32_T fun_workspace_vMax, const real32_T fun_workspace_vPoints[8], const
  real32_T fun_workspace_tPoints[8], const real32_T varargin_1_data[], const
  int32_T *varargin_1_size, real32_T varargout_1_data[], int32_T
  *varargout_1_size, real32_T varargout_2_data[], int32_T *varargout_2_size);
static void Contr_VelocityProfiler_stepImpl(VelocityProfiler_Controller_T *obj,
  const real32_T directions[50], const real32_T cumLengths[50], const real32_T
  curvatures[50], real32_T startVelocity, real32_T endVelocity, real32_T
  varargout_1[50]);
static void Controller_eml_find(const boolean_T x_data[], const int32_T *x_size,
  int32_T i_data[], int32_T *i_size, int32_T j_data[], int32_T *j_size);
static real32_T Controller_sind(real32_T x);
static void Con_HelperPathAnalyzer_stepImpl(HelperPathAnalyzer_Controller_T *obj,
  const real32_T currPose[3], real32_T currVel, const real32_T varargin_1[150],
  const real32_T varargin_2[50], const real32_T varargin_3[50], const real32_T
  varargin_4[50], real32_T refPose[3], real32_T *refVel, real32_T *direction,
  real32_T *curvature, real32_T *varargout_1);
static void Contro_angleUtilities_wrapTo2Pi(real32_T *theta);
static void emxFreeStruct_HelperPathAnalyze(HelperPathAnalyzer_Controller_T
  *pStruct);
static void emxInitStruct_HelperPathAnalyze(HelperPathAnalyzer_Controller_T
  *pStruct);
static void Controller_SystemCore_setup(HelperPathAnalyzer_Controller_T *obj);
real32_T rt_remf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u1)) {
    y = u0;
  } else if ((u1 != 0.0F) && (u1 != truncf(u1))) {
    real32_T q;
    q = fabsf(u0 / u1);
    if (!(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q)) {
      y = 0.0F * u0;
    } else {
      y = fmodf(u0, u1);
    }
  } else {
    y = fmodf(u0, u1);
  }

  return y;
}

static void Controller_filterPoses(const real32_T refPoses[6], const real32_T
  refDirections[2], real32_T poses_data[], int32_T poses_size[2], real32_T
  directions_data[], int32_T *directions_size)
{
  int32_T d_size_idx_0;
  int32_T e_size_idx_0;
  int32_T i;
  int32_T trueCount;
  real32_T a;
  real32_T a_0;
  int8_T d_data[2];
  int8_T e_data[2];
  boolean_T b[2];
  a = refPoses[1] - refPoses[0];
  a_0 = refPoses[3] - refPoses[2];
  b[0] = true;
  b[1] = (a * a + a_0 * a_0 >= 1.00000011E-6F);
  trueCount = 0;
  for (i = 0; i < 2; i++) {
    if (b[i]) {
      trueCount++;
    }
  }

  d_size_idx_0 = trueCount;
  trueCount = 0;
  for (i = 0; i < 2; i++) {
    if (b[i]) {
      d_data[trueCount] = (int8_T)(i + 1);
      trueCount++;
    }
  }

  poses_size[0] = d_size_idx_0;
  poses_size[1] = 3;
  for (i = 0; i < 3; i++) {
    for (trueCount = 0; trueCount < d_size_idx_0; trueCount++) {
      poses_data[trueCount + d_size_idx_0 * i] = refPoses[((i << 1) +
        d_data[trueCount]) - 1];
    }
  }

  trueCount = 0;
  for (i = 0; i < 2; i++) {
    if (b[i]) {
      trueCount++;
    }
  }

  e_size_idx_0 = trueCount;
  trueCount = 0;
  for (i = 0; i < 2; i++) {
    if (b[i]) {
      e_data[trueCount] = (int8_T)(i + 1);
      trueCount++;
    }
  }

  *directions_size = e_size_idx_0;
  for (i = 0; i < e_size_idx_0; i++) {
    directions_data[i] = refDirections[e_data[i] - 1];
  }

  poses_data[d_size_idx_0 - 1] = refPoses[1];
  poses_data[(d_size_idx_0 + d_size_idx_0) - 1] = refPoses[3];
  poses_data[(d_size_idx_0 + (d_size_idx_0 << 1)) - 1] = refPoses[5];
  directions_data[e_size_idx_0 - 1] = refDirections[1];
}

static void Controller_binary_expand_op_l5x(boolean_T in1_data[], int32_T
  *in1_size, const real32_T in2_data[], int32_T in3, int32_T in4, int32_T in5)
{
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  *in1_size = (in5 - in4) + 1 == 1 ? in3 : (in5 - in4) + 1;
  stride_0_0 = (in3 != 1);
  stride_1_0 = ((in5 - in4) + 1 != 1);
  loop_ub = (in5 - in4) + 1 == 1 ? in3 : (in5 - in4) + 1;
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = (in2_data[(i * stride_1_0 + in4) - 1] + in2_data[i *
                   stride_0_0] == 0.0F);
  }
}

static void Co_getSegmentBoundaryPointIndex(const real32_T directions_data[],
  const int32_T *directions_size, real_T segStartIdx_data[], int32_T
  *segStartIdx_size, real_T segEndIdx_data[], int32_T *segEndIdx_size)
{
  int32_T ii_data[2];
  int32_T switchIndex_data[2];
  int32_T e;
  int32_T f;
  int32_T idx;
  int32_T x_size;
  boolean_T x_data[2];
  boolean_T exitg1;
  if (*directions_size - 1 < 1) {
    idx = 0;
  } else {
    idx = 1;
  }

  if (*directions_size < 2) {
    f = 0;
    e = 0;
  } else {
    f = 1;
    e = 2;
  }

  if (e - f == idx) {
    x_size = idx;
    if (idx - 1 >= 0) {
      x_data[0] = (directions_data[0] + directions_data[f] == 0.0F);
    }
  } else {
    Controller_binary_expand_op_l5x(x_data, &x_size, directions_data, idx, f + 1,
      e);
  }

  idx = 0;
  e = 1;
  exitg1 = false;
  while ((!exitg1) && (e - 1 <= x_size - 1)) {
    if (x_data[e - 1]) {
      idx++;
      ii_data[idx - 1] = e;
      if (idx >= x_size) {
        exitg1 = true;
      } else {
        e++;
      }
    } else {
      e++;
    }
  }

  if (x_size == 1) {
    if (idx == 0) {
      x_size = 0;
    }
  } else {
    if (idx < 1) {
      idx = 0;
    }

    x_size = idx;
  }

  if (x_size - 1 >= 0) {
    memcpy(&switchIndex_data[0], &ii_data[0], (uint32_T)x_size * sizeof(int32_T));
  }

  *segStartIdx_size = x_size + 1;
  segStartIdx_data[0] = 1.0;
  for (f = 0; f < x_size; f++) {
    segStartIdx_data[f + 1] = switchIndex_data[f];
  }

  *segEndIdx_size = x_size + 1;
  for (f = 0; f < x_size; f++) {
    segEndIdx_data[f] = switchIndex_data[f];
  }

  segEndIdx_data[x_size] = *directions_size;
}

static void Controll_binary_expand_op_l5xxx(real32_T in1_data[], int32_T
  *in1_size, const real32_T in3_data[], int32_T in4, int32_T in5, int32_T in6)
{
  int32_T i;
  int32_T in3_size_idx_0;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  real32_T in3_data_0[2];
  in3_size_idx_0 = in6 == 1 ? (in5 - in4) + 1 : in6;
  stride_0_0 = ((in5 - in4) + 1 != 1);
  stride_1_0 = (in6 != 1);
  loop_ub = in6 == 1 ? (in5 - in4) + 1 : in6;
  for (i = 0; i < loop_ub; i++) {
    in3_data_0[i] = in3_data[(i * stride_0_0 + in4) - 1] - in3_data[i *
      stride_1_0];
  }

  *in1_size = in3_size_idx_0;
  for (i = 0; i < in3_size_idx_0; i++) {
    real32_T varargin_1;
    varargin_1 = in3_data_0[i];
    in1_data[i] = varargin_1 * varargin_1;
  }
}

static void Controlle_binary_expand_op_l5xx(real32_T in1_data[], int32_T
  *in1_size, const real32_T in3_data[], const int32_T in3_size[2], int32_T in4,
  int32_T in5, int32_T in6)
{
  int32_T i;
  int32_T in3_size_idx_0;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  real32_T in3_data_0[2];
  in3_size_idx_0 = in6 == 1 ? (in5 - in4) + 1 : in6;
  stride_0_0 = ((in5 - in4) + 1 != 1);
  stride_1_0 = (in6 != 1);
  loop_ub = in6 == 1 ? (in5 - in4) + 1 : in6;
  for (i = 0; i < loop_ub; i++) {
    in3_data_0[i] = in3_data[((i * stride_0_0 + in4) + in3_size[0]) - 1] -
      in3_data[i * stride_1_0 + in3_size[0]];
  }

  *in1_size = in3_size_idx_0;
  for (i = 0; i < in3_size_idx_0; i++) {
    real32_T varargin_1;
    varargin_1 = in3_data_0[i];
    in1_data[i] = varargin_1 * varargin_1;
  }
}

static void Controller_plus(real32_T in1_data[], int32_T *in1_size, const
  real32_T in2_data[], const int32_T *in2_size)
{
  int32_T i;
  int32_T in1_size_idx_0;
  int32_T loop_ub;
  int32_T stride_0_0;
  int32_T stride_1_0;
  real32_T in1_data_0[2];
  in1_size_idx_0 = *in2_size == 1 ? *in1_size : *in2_size;
  stride_0_0 = (*in1_size != 1);
  stride_1_0 = (*in2_size != 1);
  loop_ub = *in2_size == 1 ? *in1_size : *in2_size;
  for (i = 0; i < loop_ub; i++) {
    in1_data_0[i] = in1_data[i * stride_0_0] + in2_data[i * stride_1_0];
  }

  *in1_size = in1_size_idx_0;
  if (in1_size_idx_0 - 1 >= 0) {
    memcpy(&in1_data[0], &in1_data_0[0], (uint32_T)in1_size_idx_0 * sizeof
           (real32_T));
  }
}

static void Co_computeCumulativeChordLength(const real32_T poses_data[], const
  int32_T poses_size[2], real32_T cumChordLength_data[], int32_T
  *cumChordLength_size)
{
  int32_T b_x_size;
  int32_T c;
  int32_T d;
  int32_T f;
  int32_T j;
  int32_T k;
  int32_T loop_ub;
  int32_T m;
  real32_T b_x_data[2];
  real32_T tmp_data[2];
  real32_T varargin_1;
  if (poses_size[0] < 2) {
    d = 0;
    c = 0;
    f = 0;
    k = 0;
    j = 0;
    m = 0;
  } else {
    d = 1;
    c = 2;
    f = 1;
    k = 1;
    j = 2;
    m = 1;
  }

  loop_ub = c - d;
  if (loop_ub == f) {
    b_x_size = loop_ub;
    for (c = 0; c < loop_ub; c++) {
      varargin_1 = poses_data[d + c] - poses_data[c];
      b_x_data[c] = varargin_1 * varargin_1;
    }
  } else {
    Controll_binary_expand_op_l5xxx(b_x_data, &b_x_size, poses_data, d + 1, c, f);
  }

  loop_ub = j - k;
  if (loop_ub == m) {
    for (c = 0; c < loop_ub; c++) {
      varargin_1 = poses_data[(k + c) + poses_size[0]] - poses_data[c +
        poses_size[0]];
      tmp_data[c] = varargin_1 * varargin_1;
    }
  } else {
    Controlle_binary_expand_op_l5xx(tmp_data, &loop_ub, poses_data, poses_size,
      k + 1, j, m);
  }

  if (b_x_size == loop_ub) {
    for (c = 0; c < b_x_size; c++) {
      b_x_data[c] += tmp_data[c];
    }
  } else {
    Controller_plus(b_x_data, &b_x_size, tmp_data, &loop_ub);
  }

  c = b_x_size - 1;
  for (d = 0; d <= c; d++) {
    b_x_data[d] = sqrtf(b_x_data[d]);
  }

  *cumChordLength_size = b_x_size + 1;
  cumChordLength_data[0] = 0.0F;
  if (b_x_size - 1 >= 0) {
    memcpy(&cumChordLength_data[1], &b_x_data[0], (uint32_T)b_x_size * sizeof
           (real32_T));
  }

  if (b_x_size + 1 != 1) {
    c = b_x_size - 1;
    for (d = 0; d <= c; d++) {
      cumChordLength_data[d + 1] += cumChordLength_data[d];
    }
  }
}

static void Controller_emxInit_real32_T(emxArray_real32_T_Controller_T
  **pEmxArray, int32_T numDimensions)
{
  emxArray_real32_T_Controller_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real32_T_Controller_T *)malloc(sizeof
    (emxArray_real32_T_Controller_T));
  emxArray = *pEmxArray;
  emxArray->data = (real32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * (uint32_T)numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

static void Controller_spline(const real32_T x_data[], const int32_T *x_size,
  const real32_T y_data[], const int32_T *y_size, real32_T output_breaks_data[],
  int32_T output_breaks_size[2], real32_T output_coefs_data[], int32_T
  output_coefs_size[2])
{
  int32_T b_k;
  int32_T cpage;
  int32_T yoffset;
  real32_T g_coefs_data[12];
  real32_T md_data[4];
  real32_T s_data[4];
  real32_T dvdf_data[3];
  real32_T dx_data[3];
  boolean_T has_endslopes;
  has_endslopes = (*x_size + 2 == *y_size);
  if ((*x_size <= 2) || ((*x_size <= 3) && (!has_endslopes))) {
    int32_T s_size_idx_1;
    if (*x_size <= 2) {
      if (*x_size + 2 == *y_size) {
        if (*x_size - 2 >= 0) {
          real32_T d31;
          real32_T dnnm2;
          real32_T r;
          dnnm2 = x_data[1] - x_data[0];
          r = (y_data[2] - y_data[1]) / dnnm2;
          d31 = (r - y_data[0]) / dnnm2;
          r = (y_data[*y_size - 1] - r) / dnnm2;
          md_data[0] = (r - d31) / dnnm2;
          md_data[1] = 2.0F * d31 - r;
          md_data[2] = y_data[0];
          md_data[3] = y_data[1];
        }

        s_size_idx_1 = 4;
        for (yoffset = 0; yoffset < 4; yoffset++) {
          s_data[yoffset] = md_data[yoffset];
        }
      } else {
        s_size_idx_1 = 2;
        s_data[0] = (y_data[1] - y_data[0]) / (x_data[1] - x_data[0]);
        s_data[1] = y_data[0];
      }

      output_breaks_size[0] = 1;
      output_breaks_size[1] = *x_size;
      if (*x_size - 1 >= 0) {
        memcpy(&output_breaks_data[0], &x_data[0], (uint32_T)*x_size * sizeof
               (real32_T));
      }

      memcpy(&md_data[0], &s_data[0], (uint32_T)s_size_idx_1 * sizeof(real32_T));
    } else {
      real32_T d31;
      real32_T dnnm2;
      d31 = x_data[1] - x_data[0];
      dnnm2 = (y_data[1] - y_data[0]) / d31;
      s_data[0] = ((y_data[2] - y_data[1]) / (x_data[2] - x_data[1]) - dnnm2) /
        (x_data[2] - x_data[0]);
      s_data[1] = dnnm2 - s_data[0] * d31;
      s_data[2] = y_data[0];
      output_breaks_size[0] = 1;
      output_breaks_size[1] = 2;
      output_breaks_data[0] = x_data[0];
      output_breaks_data[1] = x_data[2];
      s_size_idx_1 = 3;
      for (yoffset = 0; yoffset < 3; yoffset++) {
        md_data[yoffset] = s_data[yoffset];
      }
    }

    output_coefs_size[0] = 1;
    output_coefs_size[1] = s_size_idx_1;
    memcpy(&output_coefs_data[0], &md_data[0], (uint32_T)s_size_idx_1 * sizeof
           (real32_T));
  } else {
    int32_T nxm1_tmp;
    int32_T s_size_idx_1;
    real32_T d31;
    real32_T dnnm2;
    real32_T md_data_tmp;
    real32_T r;
    nxm1_tmp = *x_size - 1;
    if (has_endslopes) {
      s_size_idx_1 = *y_size - 2;
      yoffset = 0;
    } else {
      s_size_idx_1 = *y_size;
      yoffset = -1;
    }

    cpage = *x_size - 2;
    for (b_k = 0; b_k <= cpage; b_k++) {
      int32_T dvdf_tmp;
      dnnm2 = x_data[b_k + 1] - x_data[b_k];
      dvdf_tmp = yoffset + b_k;
      dvdf_data[b_k] = (y_data[dvdf_tmp + 2] - y_data[dvdf_tmp + 1]) / dnnm2;
      dx_data[b_k] = dnnm2;
    }

    for (cpage = 2; cpage <= nxm1_tmp; cpage++) {
      s_data[cpage - 1] = (dx_data[cpage - 1] * dvdf_data[cpage - 2] +
                           dx_data[cpage - 2] * dvdf_data[cpage - 1]) * 3.0F;
    }

    if (has_endslopes) {
      d31 = 0.0F;
      dnnm2 = 0.0F;
      s_data[0] = y_data[0] * dx_data[1];
      s_data[*x_size - 1] = dx_data[*x_size - 3] * y_data[*x_size + 1];
    } else {
      d31 = x_data[2] - x_data[0];
      dnnm2 = x_data[*x_size - 1] - x_data[*x_size - 3];
      s_data[0] = ((2.0F * d31 + dx_data[0]) * dx_data[1] * dvdf_data[0] +
                   dx_data[0] * dx_data[0] * dvdf_data[1]) / d31;
      r = dx_data[*x_size - 2];
      s_data[*x_size - 1] = ((2.0F * dnnm2 + r) * dx_data[*x_size - 3] *
        dvdf_data[*x_size - 2] + r * r * dvdf_data[*x_size - 3]) / dnnm2;
    }

    md_data[0] = dx_data[1];
    md_data_tmp = dx_data[*x_size - 3];
    md_data[*x_size - 1] = md_data_tmp;
    for (cpage = 2; cpage <= nxm1_tmp; cpage++) {
      md_data[cpage - 1] = (dx_data[cpage - 1] + dx_data[cpage - 2]) * 2.0F;
    }

    r = dx_data[1] / md_data[0];
    md_data[1] -= r * d31;
    s_data[1] -= r * s_data[0];
    for (cpage = 3; cpage <= nxm1_tmp; cpage++) {
      r = dx_data[2] / md_data[1];
      md_data[2] -= r * dx_data[0];
      s_data[2] -= r * s_data[1];
    }

    r = dnnm2 / md_data[*x_size - 2];
    md_data[*x_size - 1] -= md_data_tmp * r;
    s_data[*x_size - 1] -= s_data[*x_size - 2] * r;
    s_data[*x_size - 1] /= md_data[*x_size - 1];
    for (cpage = nxm1_tmp; cpage >= 2; cpage--) {
      s_data[cpage - 1] = (s_data[cpage - 1] - dx_data[cpage - 2] * s_data[cpage])
        / md_data[cpage - 1];
    }

    s_data[0] = (s_data[0] - d31 * s_data[1]) / md_data[0];
    cpage = *x_size - 2;
    for (b_k = 0; b_k <= cpage; b_k++) {
      dnnm2 = dx_data[b_k];
      r = dvdf_data[b_k];
      md_data_tmp = s_data[b_k];
      d31 = (r - md_data_tmp) / dnnm2;
      r = (s_data[b_k + 1] - r) / dnnm2;
      g_coefs_data[b_k] = (r - d31) / dnnm2;
      g_coefs_data[(s_size_idx_1 + b_k) - 1] = 2.0F * d31 - r;
      g_coefs_data[((s_size_idx_1 - 1) << 1) + b_k] = md_data_tmp;
      g_coefs_data[3 * (s_size_idx_1 - 1) + b_k] = y_data[(yoffset + b_k) + 1];
    }

    output_breaks_size[0] = 1;
    output_breaks_size[1] = *x_size;
    memcpy(&output_breaks_data[0], &x_data[0], (uint32_T)*x_size * sizeof
           (real32_T));
    output_coefs_size[0] = s_size_idx_1 - 1;
    output_coefs_size[1] = 4;
    cpage = (s_size_idx_1 - 1) << 2;
    if (cpage - 1 >= 0) {
      memcpy(&output_coefs_data[0], &g_coefs_data[0], (uint32_T)cpage * sizeof
             (real32_T));
    }
  }
}

static void Controller_bsxfun(const real32_T a_data[], const int32_T a_size[2],
  real32_T c_data[], int32_T c_size[2])
{
  int32_T k;
  int32_T k_0;
  c_size[0] = a_size[0];
  c_size[1] = 3;
  if (a_size[0] != 0) {
    int32_T acoef;
    acoef = (a_size[0] != 1);
    for (k = 0; k < 3; k++) {
      int32_T d;
      d = c_size[0];
      for (k_0 = 0; k_0 < d; k_0++) {
        c_data[k_0 + c_size[0] * k] = a_data[acoef * k_0 + a_size[0] * k] *
          (3.0F - (real32_T)k);
      }
    }
  }
}

static void Cont_emxEnsureCapacity_real32_T(emxArray_real32_T_Controller_T
  *emxArray, int32_T oldNumel)
{
  int32_T i;
  int32_T newNumel;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(real32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real32_T) * (uint32_T)oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

static void Controller_linspace(real32_T d2, real32_T n,
  emxArray_real32_T_Controller_T *y)
{
  int32_T d_k;
  if (!(n >= 0.0F)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    real32_T delta1;
    delta1 = floorf(n);
    d_k = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)delta1;
    Cont_emxEnsureCapacity_real32_T(y, d_k);
    if ((int32_T)delta1 >= 1) {
      y->data[(int32_T)delta1 - 1] = d2;
      if (y->size[1] >= 2) {
        y->data[0] = 0.0F;
        if (y->size[1] >= 3) {
          if (-d2 == 0.0F) {
            int32_T g;
            delta1 = d2 / (real32_T)(y->size[1] - 1);
            g = y->size[1];
            for (d_k = 2; d_k < g; d_k++) {
              y->data[d_k - 1] = (real32_T)(((d_k << 1) - y->size[1]) - 1) *
                delta1;
            }

            if ((y->size[1] & 1) == 1) {
              y->data[y->size[1] >> 1] = 0.0F;
            }
          } else if ((d2 < 0.0F) && (fabsf(d2) > 1.70141173E+38F)) {
            int32_T g;
            delta1 = d2 / (real32_T)((real_T)y->size[1] - 1.0);
            g = y->size[1] - 3;
            for (d_k = 0; d_k <= g; d_k++) {
              y->data[d_k + 1] = (real32_T)((real_T)d_k + 1.0) * delta1;
            }
          } else {
            int32_T g;
            delta1 = d2 / (real32_T)((real_T)y->size[1] - 1.0);
            g = y->size[1] - 3;
            for (d_k = 0; d_k <= g; d_k++) {
              y->data[d_k + 1] = (real32_T)((real_T)d_k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
}

static int32_T Controller_bsearch(const real32_T x_data[], const int32_T x_size
  [2], real32_T xi)
{
  int32_T high_i;
  int32_T low_ip1;
  int32_T n;
  n = 1;
  low_ip1 = 1;
  high_i = x_size[1];
  while (high_i > low_ip1 + 1) {
    int32_T mid_i;
    mid_i = (n >> 1) + (high_i >> 1);
    if (((n & 1) == 1) && ((high_i & 1) == 1)) {
      mid_i++;
    }

    if (xi >= x_data[mid_i - 1]) {
      n = mid_i;
      low_ip1 = mid_i;
    } else {
      high_i = mid_i;
    }
  }

  return n;
}

static void Controller_ppval(const real32_T pp_breaks_data[], const int32_T
  pp_breaks_size[2], const real32_T pp_coefs_data[], const int32_T
  pp_coefs_size[2], const real32_T x_data[], const int32_T x_size[2], real32_T
  v_data[], int32_T v_size[2])
{
  int32_T b;
  int32_T b_ix;
  int32_T ic;
  int32_T numTerms;
  numTerms = pp_coefs_size[1];
  v_size[0] = 1;
  v_size[1] = x_size[1];
  b = x_size[1] - 1;
  for (b_ix = 0; b_ix <= b; b_ix++) {
    real32_T x;
    x = x_data[b_ix];
    if (!rtIsNaNF(x)) {
      int32_T ip;
      real32_T xloc;
      ip = Controller_bsearch(pp_breaks_data, pp_breaks_size, x) - 1;
      xloc = x - pp_breaks_data[ip];
      x = pp_coefs_data[ip];
      for (ic = 2; ic <= numTerms; ic++) {
        x = pp_coefs_data[(ic - 1) * (pp_breaks_size[1] - 1) + ip] + xloc * x;
      }
    }

    v_data[b_ix] = x;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2f((real32_T)tmp, (real32_T)tmp_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }

  return y;
}

static void Controller_expand_atan2(const real32_T a_data[], const int32_T
  a_size[2], const real32_T b_data[], const int32_T b_size[2], real32_T c_data[],
  int32_T c_size[2])
{
  int32_T csz_idx_1;
  int32_T k;
  if (b_size[1] == 1) {
    csz_idx_1 = a_size[1];
  } else if (a_size[1] == 1) {
    csz_idx_1 = b_size[1];
  } else if (a_size[1] <= b_size[1]) {
    csz_idx_1 = a_size[1];
  } else {
    csz_idx_1 = b_size[1];
  }

  c_size[0] = 1;
  c_size[1] = csz_idx_1;
  if (csz_idx_1 != 0) {
    boolean_T d;
    boolean_T e;
    d = (a_size[1] != 1);
    e = (b_size[1] != 1);
    for (k = 0; k < csz_idx_1; k++) {
      c_data[k] = rt_atan2f_snf(a_data[d * k], b_data[e * k]);
    }
  }
}

static void Controller_emxFree_real32_T(emxArray_real32_T_Controller_T
  **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T_Controller_T *)NULL) {
    if (((*pEmxArray)->data != (real32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real32_T_Controller_T *)NULL;
  }
}

static void Controller_plus_l(emxArray_real32_T_Controller_T *in1, const
  emxArray_real32_T_Controller_T *in2)
{
  emxArray_real32_T_Controller_T *in1_0;
  int32_T i;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  Controller_emxInit_real32_T(&in1_0, 2);
  i = in1_0->size[0] * in1_0->size[1];
  in1_0->size[0] = 1;
  in1_0->size[1] = in2->size[1] == 1 ? in1->size[1] : in2->size[1];
  Cont_emxEnsureCapacity_real32_T(in1_0, i);
  stride_0_1 = (in1->size[1] != 1);
  stride_1_1 = (in2->size[1] != 1);
  loop_ub = in2->size[1] == 1 ? in1->size[1] : in2->size[1];
  for (i = 0; i < loop_ub; i++) {
    in1_0->data[i] = in1->data[i * stride_0_1] + in2->data[i * stride_1_1];
  }

  i = in1->size[0] * in1->size[1];
  in1->size[0] = 1;
  in1->size[1] = in1_0->size[1];
  Cont_emxEnsureCapacity_real32_T(in1, i);
  loop_ub = in1_0->size[1];
  if (loop_ub - 1 >= 0) {
    memcpy(&in1->data[0], &in1_0->data[0], (uint32_T)loop_ub * sizeof(real32_T));
  }

  Controller_emxFree_real32_T(&in1_0);
}

static void Controller_cumtrapz(const emxArray_real32_T_Controller_T *x,
  emxArray_real32_T_Controller_T *z)
{
  int32_T ylast_tmp;
  ylast_tmp = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = x->size[1];
  Cont_emxEnsureCapacity_real32_T(z, ylast_tmp);
  if (x->size[1] != 0) {
    int32_T c;
    real32_T s;
    real32_T ylast;
    s = 0.0F;
    ylast = x->data[0];
    z->data[0] = 0.0F;
    c = x->size[1] - 1;
    for (ylast_tmp = 0; ylast_tmp < c; ylast_tmp++) {
      real32_T s_tmp;
      s_tmp = x->data[ylast_tmp + 1];
      s += (s_tmp + ylast) / 2.0F;
      ylast = s_tmp;
      z->data[ylast_tmp + 1] = s;
    }
  }
}

static void Con_computeCumulativePathLength(const real32_T splineDx_breaks_data[],
  const int32_T splineDx_breaks_size[2], const real32_T splineDx_coefs_data[],
  const real32_T splineDy_breaks_data[], const int32_T splineDy_breaks_size[2],
  const real32_T splineDy_coefs_data[], const real32_T tQuery_data[], const
  int32_T tQuery_size[2], real32_T cumLengths_data[], int32_T cumLengths_size[2])
{
  emxArray_real32_T_Controller_T *intervalcumPathLength;
  emxArray_real32_T_Controller_T *tInteg;
  emxArray_real32_T_Controller_T *v;
  int32_T tmp_data[50];
  int32_T b_ix;
  int32_T ip;
  int32_T nx;
  real32_T stepSize;
  real32_T tSpacing;
  real32_T xloc;
  int8_T y_data[49];
  tSpacing = tQuery_data[1] - tQuery_data[0];
  stepSize = tSpacing / fmaxf(ceilf(tSpacing / 0.01F), 10.0F);
  tSpacing = roundf(tSpacing / stepSize);
  Controller_emxInit_real32_T(&tInteg, 2);
  Controller_linspace(tQuery_data[tQuery_size[1] - 1], ((real32_T)tQuery_size[1]
    - 1.0F) * tSpacing + 1.0F, tInteg);
  Controller_emxInit_real32_T(&intervalcumPathLength, 2);
  b_ix = intervalcumPathLength->size[0] * intervalcumPathLength->size[1];
  intervalcumPathLength->size[0] = 1;
  intervalcumPathLength->size[1] = tInteg->size[1];
  Cont_emxEnsureCapacity_real32_T(intervalcumPathLength, b_ix);
  nx = tInteg->size[1] - 1;
  for (b_ix = 0; b_ix <= nx; b_ix++) {
    xloc = tInteg->data[b_ix];
    if (rtIsNaNF(xloc)) {
      intervalcumPathLength->data[b_ix] = xloc;
    } else {
      ip = Controller_bsearch(splineDx_breaks_data, splineDx_breaks_size, xloc)
        - 1;
      xloc -= splineDx_breaks_data[ip];
      intervalcumPathLength->data[b_ix] = (splineDx_coefs_data[(ip +
        splineDx_breaks_size[1]) - 1] + splineDx_coefs_data[ip] * xloc) * xloc +
        splineDx_coefs_data[((splineDx_breaks_size[1] - 1) << 1) + ip];
    }
  }

  Controller_emxInit_real32_T(&v, 2);
  b_ix = v->size[0] * v->size[1];
  v->size[0] = 1;
  v->size[1] = tInteg->size[1];
  Cont_emxEnsureCapacity_real32_T(v, b_ix);
  nx = tInteg->size[1] - 1;
  for (b_ix = 0; b_ix <= nx; b_ix++) {
    xloc = tInteg->data[b_ix];
    if (rtIsNaNF(xloc)) {
      v->data[b_ix] = xloc;
    } else {
      ip = Controller_bsearch(splineDy_breaks_data, splineDy_breaks_size, xloc)
        - 1;
      xloc -= splineDy_breaks_data[ip];
      v->data[b_ix] = (splineDy_coefs_data[(ip + splineDy_breaks_size[1]) - 1] +
                       splineDy_coefs_data[ip] * xloc) * xloc +
        splineDy_coefs_data[((splineDy_breaks_size[1] - 1) << 1) + ip];
    }
  }

  b_ix = intervalcumPathLength->size[0] * intervalcumPathLength->size[1];
  intervalcumPathLength->size[0] = 1;
  Cont_emxEnsureCapacity_real32_T(intervalcumPathLength, b_ix);
  nx = intervalcumPathLength->size[1] - 1;
  for (b_ix = 0; b_ix <= nx; b_ix++) {
    xloc = intervalcumPathLength->data[b_ix];
    intervalcumPathLength->data[b_ix] = xloc * xloc;
  }

  b_ix = tInteg->size[0] * tInteg->size[1];
  tInteg->size[0] = 1;
  tInteg->size[1] = v->size[1];
  Cont_emxEnsureCapacity_real32_T(tInteg, b_ix);
  nx = v->size[1];
  for (b_ix = 0; b_ix < nx; b_ix++) {
    xloc = v->data[b_ix];
    tInteg->data[b_ix] = xloc * xloc;
  }

  Controller_emxFree_real32_T(&v);
  if (intervalcumPathLength->size[1] == tInteg->size[1]) {
    nx = intervalcumPathLength->size[1] - 1;
    b_ix = intervalcumPathLength->size[0] * intervalcumPathLength->size[1];
    intervalcumPathLength->size[0] = 1;
    Cont_emxEnsureCapacity_real32_T(intervalcumPathLength, b_ix);
    for (b_ix = 0; b_ix <= nx; b_ix++) {
      intervalcumPathLength->data[b_ix] += tInteg->data[b_ix];
    }
  } else {
    Controller_plus_l(intervalcumPathLength, tInteg);
  }

  b_ix = tInteg->size[0] * tInteg->size[1];
  tInteg->size[0] = 1;
  tInteg->size[1] = intervalcumPathLength->size[1];
  Cont_emxEnsureCapacity_real32_T(tInteg, b_ix);
  nx = intervalcumPathLength->size[1];
  if (nx - 1 >= 0) {
    memcpy(&tInteg->data[0], &intervalcumPathLength->data[0], (uint32_T)nx *
           sizeof(real32_T));
  }

  nx = intervalcumPathLength->size[1] - 1;
  for (b_ix = 0; b_ix <= nx; b_ix++) {
    tInteg->data[b_ix] = sqrtf(tInteg->data[b_ix]);
  }

  Controller_cumtrapz(tInteg, intervalcumPathLength);
  Controller_emxFree_real32_T(&tInteg);
  b_ix = intervalcumPathLength->size[0] * intervalcumPathLength->size[1];
  intervalcumPathLength->size[0] = 1;
  Cont_emxEnsureCapacity_real32_T(intervalcumPathLength, b_ix);
  nx = intervalcumPathLength->size[1] - 1;
  for (b_ix = 0; b_ix <= nx; b_ix++) {
    intervalcumPathLength->data[b_ix] *= stepSize;
  }

  if (tQuery_size[1] - 1 < 1) {
    ip = 0;
  } else {
    ip = tQuery_size[1] - 1;
    nx = tQuery_size[1] - 2;
    for (b_ix = 0; b_ix <= nx; b_ix++) {
      y_data[b_ix] = (int8_T)(b_ix + 1);
    }
  }

  cumLengths_size[0] = 1;
  cumLengths_size[1] = ip + 1;
  nx = ip + 1;
  tmp_data[0] = 0;
  for (b_ix = 0; b_ix < ip; b_ix++) {
    tmp_data[b_ix + 1] = (int32_T)(tSpacing * (real32_T)y_data[b_ix] + 1.0F) - 1;
  }

  for (b_ix = 0; b_ix < nx; b_ix++) {
    cumLengths_data[b_ix] = intervalcumPathLength->data[tmp_data[b_ix]];
  }

  Controller_emxFree_real32_T(&intervalcumPathLength);
}

static void Controller_bsxfun_l(const real32_T a_data[], const int32_T a_size[2],
  real32_T c_data[], int32_T c_size[2])
{
  int32_T acoef;
  int32_T k;
  int32_T k_0;
  c_size[0] = a_size[0];
  c_size[1] = 2;
  acoef = (a_size[0] != 1);
  for (k = 0; k < 2; k++) {
    int32_T d;
    d = c_size[0];
    for (k_0 = 0; k_0 < d; k_0++) {
      c_data[k_0 + c_size[0] * k] = a_data[acoef * k_0 + a_size[0] * k] * (2.0F
        - (real32_T)k);
    }
  }
}

real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else {
    real32_T tmp;
    real32_T tmp_0;
    tmp = fabsf(u0);
    tmp_0 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (tmp == 1.0F) {
        y = 1.0F;
      } else if (tmp > 1.0F) {
        if (u1 > 0.0F) {
          y = (rtInfF);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = (rtInfF);
      }
    } else if (tmp_0 == 0.0F) {
      y = 1.0F;
    } else if (tmp_0 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = (rtNaNF);
    } else {
      y = powf(u0, u1);
    }
  }

  return y;
}

static void Contro_binary_expand_op_l5xxx52(real32_T in1_data[], int32_T
  in1_size[2], const emxArray_real32_T_Controller_T *in3, const
  emxArray_real32_T_Controller_T *in4)
{
  int32_T i;
  int32_T in3_size_idx_1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  real32_T in3_data[50];
  in3_size_idx_1 = in4->size[1] == 1 ? in3->size[1] : in4->size[1];
  stride_0_1 = (in3->size[1] != 1);
  stride_1_1 = (in4->size[1] != 1);
  loop_ub = in4->size[1] == 1 ? in3->size[1] : in4->size[1];
  for (i = 0; i < loop_ub; i++) {
    in3_data[i] = in3->data[i * stride_0_1] + in4->data[i * stride_1_1];
  }

  in1_size[0] = 1;
  in1_size[1] = in3_size_idx_1;
  for (i = 0; i < in3_size_idx_1; i++) {
    in1_data[i] = rt_powf_snf(in3_data[i], 1.5F);
  }
}

static void Control_binary_expand_op_l5xxx5(real32_T in1_data[], int32_T
  in1_size[2], const emxArray_real32_T_Controller_T *in2, const
  emxArray_real32_T_Controller_T *in3, const real32_T in4_data[], const int32_T
  in4_size[2], const real32_T in5_data[], const int32_T in5_size[2])
{
  int32_T i;
  int32_T in2_size_idx_1;
  int32_T loop_ub;
  int32_T stride_0_1;
  int32_T stride_1_1;
  int32_T stride_2_1;
  int32_T stride_3_1;
  int32_T stride_4_1;
  real32_T in2_data[50];
  in2_size_idx_1 = in5_size[1] == 1 ? (in4_size[1] == 1 ? in3->size[1] :
    in4_size[1]) == 1 ? in1_size[1] == 1 ? in2->size[1] : in1_size[1] :
    in4_size[1] == 1 ? in3->size[1] : in4_size[1] : in5_size[1];
  stride_0_1 = (in2->size[1] != 1);
  stride_1_1 = (in1_size[1] != 1);
  stride_2_1 = (in3->size[1] != 1);
  stride_3_1 = (in4_size[1] != 1);
  stride_4_1 = (in5_size[1] != 1);
  loop_ub = in5_size[1] == 1 ? (in4_size[1] == 1 ? in3->size[1] : in4_size[1]) ==
    1 ? in1_size[1] == 1 ? in2->size[1] : in1_size[1] : in4_size[1] == 1 ?
    in3->size[1] : in4_size[1] : in5_size[1];
  for (i = 0; i < loop_ub; i++) {
    in2_data[i] = (in2->data[i * stride_0_1] * in1_data[i * stride_1_1] -
                   in3->data[i * stride_2_1] * in4_data[i * stride_3_1]) /
      in5_data[i * stride_4_1];
  }

  in1_size[0] = 1;
  in1_size[1] = in2_size_idx_1;
  if (in2_size_idx_1 - 1 >= 0) {
    memcpy(&in1_data[0], &in2_data[0], (uint32_T)in2_size_idx_1 * sizeof
           (real32_T));
  }
}

static void Controlle_computePathCurvatures(const real32_T splineDx_breaks_data[],
  const int32_T splineDx_breaks_size[2], const real32_T splineDx_coefs_data[],
  const int32_T splineDx_coefs_size[3], const real32_T splineDy_breaks_data[],
  const int32_T splineDy_breaks_size[2], const real32_T splineDy_coefs_data[],
  const int32_T splineDy_coefs_size[3], const real32_T tQuery_data[], const
  int32_T tQuery_size[2], real32_T curvatures_data[], int32_T curvatures_size[2])
{
  emxArray_real32_T_Controller_T *tmp;
  emxArray_real32_T_Controller_T *v;
  emxArray_real32_T_Controller_T *v_0;
  emxArray_real32_T_Controller_T *v_1;
  emxArray_real32_T_Controller_T *v_2;
  int32_T e_size[2];
  int32_T splineDx_breaks_size_0[2];
  int32_T splineDx_coefs_size_0[2];
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T emptyDimValue;
  int32_T emptyDimValue_tmp;
  int32_T ip;
  int32_T nx;
  real32_T e_data[50];
  real32_T tmp_data_0[50];
  real32_T splineD2x_coefs_data[6];
  real32_T splineDx_coefs_data_0[6];
  real32_T tmp_data[6];
  real32_T splineDx_breaks_data_0[4];
  real32_T xloc;
  emptyDimValue = (int32_T)((uint32_T)(splineDx_coefs_size[1] * 3) / 3U);
  splineDx_coefs_size_0[0] = emptyDimValue;
  splineDx_coefs_size_0[1] = 2;
  for (ip = 0; ip < 2; ip++) {
    for (nx = 0; nx < emptyDimValue; nx++) {
      splineDx_coefs_data_0[nx + emptyDimValue * ip] =
        splineDx_coefs_data[splineDx_coefs_size[1] * ip + nx];
    }
  }

  Controller_bsxfun_l(splineDx_coefs_data_0, splineDx_coefs_size_0, tmp_data,
                      tmp_size);
  emptyDimValue_tmp = (splineDx_breaks_size[1] - 1) << 1;
  if (emptyDimValue_tmp - 1 >= 0) {
    memcpy(&splineD2x_coefs_data[0], &tmp_data[0], (uint32_T)emptyDimValue_tmp *
           sizeof(real32_T));
  }

  emptyDimValue = (int32_T)((uint32_T)(splineDy_coefs_size[1] * 3) / 3U);
  splineDx_coefs_size_0[0] = emptyDimValue;
  splineDx_coefs_size_0[1] = 2;
  for (ip = 0; ip < 2; ip++) {
    for (nx = 0; nx < emptyDimValue; nx++) {
      splineDx_coefs_data_0[nx + emptyDimValue * ip] =
        splineDy_coefs_data[splineDy_coefs_size[1] * ip + nx];
    }
  }

  Controller_bsxfun_l(splineDx_coefs_data_0, splineDx_coefs_size_0, tmp_data,
                      tmp_size);
  if (emptyDimValue_tmp - 1 >= 0) {
    memcpy(&splineDx_coefs_data_0[0], &tmp_data[0], (uint32_T)emptyDimValue_tmp *
           sizeof(real32_T));
  }

  Controller_emxInit_real32_T(&v, 2);
  emptyDimValue = v->size[0] * v->size[1];
  v->size[0] = 1;
  v->size[1] = tQuery_size[1];
  Cont_emxEnsureCapacity_real32_T(v, emptyDimValue);
  nx = tQuery_size[1] - 1;
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      v->data[emptyDimValue] = xloc;
    } else {
      ip = Controller_bsearch(splineDx_breaks_data, splineDx_breaks_size, xloc)
        - 1;
      xloc -= splineDx_breaks_data[ip];
      v->data[emptyDimValue] = (splineDx_coefs_data[(ip + splineDx_breaks_size[1])
        - 1] + splineDx_coefs_data[ip] * xloc) * xloc +
        splineDx_coefs_data[emptyDimValue_tmp + ip];
    }
  }

  curvatures_size[0] = 1;
  curvatures_size[1] = tQuery_size[1];
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      curvatures_data[emptyDimValue] = xloc;
    } else {
      ip = splineDx_breaks_size[1];
      splineDx_breaks_size_0[0] = 1;
      splineDx_breaks_size_0[1] = splineDx_breaks_size[1];
      if (ip - 1 >= 0) {
        memcpy(&splineDx_breaks_data_0[0], &splineDx_breaks_data[0], (uint32_T)
               ip * sizeof(real32_T));
      }

      ip = Controller_bsearch(splineDx_breaks_data_0, splineDx_breaks_size_0,
        xloc) - 1;
      curvatures_data[emptyDimValue] = (xloc - splineDx_breaks_data[ip]) *
        splineDx_coefs_data_0[ip] + splineDx_coefs_data_0[(ip +
        splineDx_breaks_size[1]) - 1];
    }
  }

  Controller_emxInit_real32_T(&v_0, 2);
  emptyDimValue = v_0->size[0] * v_0->size[1];
  v_0->size[0] = 1;
  v_0->size[1] = tQuery_size[1];
  Cont_emxEnsureCapacity_real32_T(v_0, emptyDimValue);
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      v_0->data[emptyDimValue] = xloc;
    } else {
      ip = Controller_bsearch(splineDy_breaks_data, splineDy_breaks_size, xloc)
        - 1;
      xloc -= splineDy_breaks_data[ip];
      v_0->data[emptyDimValue] = (splineDy_coefs_data[(ip +
        splineDy_breaks_size[1]) - 1] + splineDy_coefs_data[ip] * xloc) * xloc +
        splineDy_coefs_data[((splineDy_breaks_size[1] - 1) << 1) + ip];
    }
  }

  e_size[0] = 1;
  e_size[1] = tQuery_size[1];
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      e_data[emptyDimValue] = xloc;
    } else {
      ip = splineDx_breaks_size[1];
      splineDx_breaks_size_0[0] = 1;
      splineDx_breaks_size_0[1] = splineDx_breaks_size[1];
      if (ip - 1 >= 0) {
        memcpy(&splineDx_breaks_data_0[0], &splineDx_breaks_data[0], (uint32_T)
               ip * sizeof(real32_T));
      }

      ip = Controller_bsearch(splineDx_breaks_data_0, splineDx_breaks_size_0,
        xloc) - 1;
      e_data[emptyDimValue] = (xloc - splineDx_breaks_data[ip]) *
        splineD2x_coefs_data[ip] + splineD2x_coefs_data[(ip +
        splineDx_breaks_size[1]) - 1];
    }
  }

  Controller_emxInit_real32_T(&v_1, 2);
  emptyDimValue = v_1->size[0] * v_1->size[1];
  v_1->size[0] = 1;
  v_1->size[1] = tQuery_size[1];
  Cont_emxEnsureCapacity_real32_T(v_1, emptyDimValue);
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      v_1->data[emptyDimValue] = xloc;
    } else {
      ip = Controller_bsearch(splineDx_breaks_data, splineDx_breaks_size, xloc)
        - 1;
      xloc -= splineDx_breaks_data[ip];
      v_1->data[emptyDimValue] = (splineDx_coefs_data[(ip +
        splineDx_breaks_size[1]) - 1] + splineDx_coefs_data[ip] * xloc) * xloc +
        splineDx_coefs_data[emptyDimValue_tmp + ip];
    }
  }

  Controller_emxInit_real32_T(&v_2, 2);
  emptyDimValue = v_2->size[0] * v_2->size[1];
  v_2->size[0] = 1;
  v_2->size[1] = tQuery_size[1];
  Cont_emxEnsureCapacity_real32_T(v_2, emptyDimValue);
  for (emptyDimValue = 0; emptyDimValue <= nx; emptyDimValue++) {
    xloc = tQuery_data[emptyDimValue];
    if (rtIsNaNF(xloc)) {
      v_2->data[emptyDimValue] = xloc;
    } else {
      ip = Controller_bsearch(splineDy_breaks_data, splineDy_breaks_size, xloc)
        - 1;
      xloc -= splineDy_breaks_data[ip];
      v_2->data[emptyDimValue] = (splineDy_coefs_data[(ip +
        splineDy_breaks_size[1]) - 1] + splineDy_coefs_data[ip] * xloc) * xloc +
        splineDy_coefs_data[((splineDy_breaks_size[1] - 1) << 1) + ip];
    }
  }

  Controller_emxInit_real32_T(&tmp, 2);
  emptyDimValue = tmp->size[0] * tmp->size[1];
  tmp->size[0] = 1;
  tmp->size[1] = v_1->size[1];
  Cont_emxEnsureCapacity_real32_T(tmp, emptyDimValue);
  emptyDimValue = v_1->size[1];
  for (ip = 0; ip < emptyDimValue; ip++) {
    xloc = v_1->data[ip];
    tmp->data[ip] = xloc * xloc;
  }

  emptyDimValue = v_1->size[0] * v_1->size[1];
  v_1->size[0] = 1;
  v_1->size[1] = v_2->size[1];
  Cont_emxEnsureCapacity_real32_T(v_1, emptyDimValue);
  emptyDimValue = v_2->size[1];
  for (ip = 0; ip < emptyDimValue; ip++) {
    xloc = v_2->data[ip];
    v_1->data[ip] = xloc * xloc;
  }

  Controller_emxFree_real32_T(&v_2);
  if (tmp->size[1] == v_1->size[1]) {
    tmp_size_0[0] = 1;
    tmp_size_0[1] = tmp->size[1];
    emptyDimValue = tmp->size[1];
    for (ip = 0; ip < emptyDimValue; ip++) {
      tmp_data_0[ip] = rt_powf_snf(tmp->data[ip] + v_1->data[ip], 1.5F);
    }
  } else {
    Contro_binary_expand_op_l5xxx52(tmp_data_0, tmp_size_0, tmp, v_1);
  }

  Controller_emxFree_real32_T(&tmp);
  Controller_emxFree_real32_T(&v_1);
  if ((v->size[1] == tQuery_size[1]) && (v_0->size[1] == tQuery_size[1]) &&
      ((v->size[1] == 1 ? tQuery_size[1] : v->size[1]) == (v_0->size[1] == 1 ?
        tQuery_size[1] : v_0->size[1])) && (((v->size[1] == 1 ? tQuery_size[1] :
         v->size[1]) == 1 ? v_0->size[1] == 1 ? tQuery_size[1] : v_0->size[1] :
        v->size[1] == 1 ? tQuery_size[1] : v->size[1]) == tmp_size_0[1])) {
    emptyDimValue = v->size[1] - 1;
    curvatures_size[0] = 1;
    curvatures_size[1] = v->size[1];
    for (ip = 0; ip <= emptyDimValue; ip++) {
      curvatures_data[ip] = (v->data[ip] * curvatures_data[ip] - v_0->data[ip] *
        e_data[ip]) / tmp_data_0[ip];
    }
  } else {
    Control_binary_expand_op_l5xxx5(curvatures_data, curvatures_size, v, v_0,
      e_data, e_size, tmp_data_0, tmp_size_0);
  }

  Controller_emxFree_real32_T(&v_0);
  Controller_emxFree_real32_T(&v);
}

static void Controller_smoothPathSpline(const real32_T refPoses[6], const
  real32_T refDirections[2], real32_T poses[150], real32_T directions[50],
  real32_T varargout_1[50], real32_T varargout_2[50])
{
  emxArray_real32_T_Controller_T *v;
  emxArray_real32_T_Controller_T *v_0;
  real_T segEndIdx_data[3];
  real_T segStartIdx_data[3];
  real_T k;
  real_T segEndIdx;
  real_T segStartIdx;
  int32_T splineDx_coefs_size[3];
  int32_T splineDy_coefs_size[3];
  int32_T b_refPoses_size[2];
  int32_T splineDx_breaks_size[2];
  int32_T splineDy_breaks_size[2];
  int32_T splineX_breaks_size[2];
  int32_T splineX_coefs_size[2];
  int32_T splineX_coefs_size_0[2];
  int32_T splineY_coefs_size[2];
  int32_T tQuery_size[2];
  int32_T theta_size[2];
  int32_T theta_size_0[2];
  int32_T tmp_size[2];
  int32_T c_refPoses_data_tmp;
  int32_T c_refPoses_size_idx_0;
  int32_T c_refPoses_tmp;
  int32_T h;
  int32_T i;
  int32_T ip;
  int32_T m;
  int32_T segStartIdx_size;
  int32_T tmp_size_0;
  int32_T vlen;
  int32_T x;
  real32_T posesSeg_data[150];
  real32_T tQuery_data[50];
  real32_T theta_data[50];
  real32_T theta_data_0[50];
  real32_T x_data[50];
  real32_T y_data[50];
  real32_T splineX_coefs_data[12];
  real32_T splineY_coefs_data[12];
  real32_T splineDx_coefs_data[9];
  real32_T splineX_coefs_data_0[9];
  real32_T tmp_data[9];
  real32_T b_refPoses_data[6];
  real32_T c_refPoses_data[6];
  real32_T cumChordLength_data[4];
  real32_T splineX_breaks_data[4];
  real32_T splineY_breaks_data[4];
  real32_T tInterp_data[4];
  real32_T tmp_data_0[4];
  real32_T chordLengthOfSegments_data[3];
  real32_T z_data[3];
  real32_T b_refDirections_data[2];
  real32_T y;
  Controller_filterPoses(refPoses, refDirections, b_refPoses_data,
    b_refPoses_size, b_refDirections_data, &vlen);
  c_refPoses_size_idx_0 = b_refPoses_size[0];
  i = b_refPoses_size[0] * 3;
  if (i - 1 >= 0) {
    memcpy(&c_refPoses_data[0], &b_refPoses_data[0], (uint32_T)i * sizeof
           (real32_T));
  }

  Co_getSegmentBoundaryPointIndex(b_refDirections_data, &vlen, segStartIdx_data,
    &segStartIdx_size, segEndIdx_data, &i);
  i = b_refPoses_size[0];
  for (vlen = 0; vlen < i; vlen++) {
    c_refPoses_data_tmp = (b_refPoses_size[0] << 1) + vlen;
    c_refPoses_data[c_refPoses_data_tmp] = 0.0174532924F *
      b_refPoses_data[c_refPoses_data_tmp];
  }

  memset(&poses[0], 0, 150U * sizeof(real32_T));
  memset(&directions[0], 0, 50U * sizeof(real32_T));
  i = (b_refPoses_size[0] + segStartIdx_size) - 1;
  if (i - 1 >= 0) {
    memset(&cumChordLength_data[0], 0, (uint32_T)i * sizeof(real32_T));
  }

  memset(&varargout_1[0], 0, 50U * sizeof(real32_T));
  memset(&varargout_2[0], 0, 50U * sizeof(real32_T));
  c_refPoses_tmp = segStartIdx_size - 1;
  for (c_refPoses_data_tmp = 0; c_refPoses_data_tmp <= c_refPoses_tmp;
       c_refPoses_data_tmp++) {
    segEndIdx = segEndIdx_data[c_refPoses_data_tmp];
    segStartIdx = segStartIdx_data[c_refPoses_data_tmp];
    if (segStartIdx > segEndIdx) {
      h = 0;
      i = 0;
    } else {
      h = (int32_T)segStartIdx - 1;
      i = (int32_T)segEndIdx;
    }

    segStartIdx = (((real_T)c_refPoses_data_tmp + 1.0) + segStartIdx) - 1.0;
    k = (((real_T)c_refPoses_data_tmp + 1.0) + segEndIdx) - 1.0;
    if (segStartIdx > k) {
      m = 0;
      ip = 0;
    } else {
      m = (int32_T)segStartIdx - 1;
      ip = (int32_T)k;
    }

    i -= h;
    b_refPoses_size[0] = i;
    b_refPoses_size[1] = 3;
    for (vlen = 0; vlen < 3; vlen++) {
      for (x = 0; x < i; x++) {
        b_refPoses_data[x + i * vlen] = c_refPoses_data[(h + x) +
          c_refPoses_size_idx_0 * vlen];
      }
    }

    Co_computeCumulativeChordLength(b_refPoses_data, b_refPoses_size, z_data,
      &vlen);
    i = ip - m;
    for (vlen = 0; vlen < i; vlen++) {
      cumChordLength_data[m + vlen] = z_data[vlen];
    }

    chordLengthOfSegments_data[c_refPoses_data_tmp] = cumChordLength_data
      [(int32_T)((((real_T)c_refPoses_data_tmp + 1.0) + segEndIdx) - 1.0) - 1];
  }

  if (segStartIdx_size == 0) {
    y = 0.0F;
  } else {
    y = chordLengthOfSegments_data[0];
    for (i = 2; i <= segStartIdx_size; i++) {
      y += chordLengthOfSegments_data[i - 1];
    }
  }

  for (vlen = 0; vlen < segStartIdx_size; vlen++) {
    z_data[vlen] = chordLengthOfSegments_data[vlen] * 50.0F / y;
  }

  i = segStartIdx_size - 1;
  for (vlen = 0; vlen <= i; vlen++) {
    z_data[vlen] = floorf(z_data[vlen]);
  }

  if (segStartIdx_size - 1 < 1) {
    vlen = 0;
  } else {
    vlen = segStartIdx_size - 1;
  }

  if ((vlen == 0) || (vlen == 0)) {
    y = 0.0F;
  } else {
    y = z_data[0];
    for (i = 2; i <= vlen; i++) {
      y += z_data[1];
    }
  }

  z_data[segStartIdx_size - 1] = 50.0F - y;
  Controller_emxInit_real32_T(&v, 2);
  Controller_emxInit_real32_T(&v_0, 2);
  for (c_refPoses_data_tmp = 0; c_refPoses_data_tmp <= c_refPoses_tmp;
       c_refPoses_data_tmp++) {
    real32_T startIdx;
    real32_T xloc;
    segStartIdx = segStartIdx_data[c_refPoses_data_tmp];
    segEndIdx = (((real_T)c_refPoses_data_tmp + 1.0) + segStartIdx) - 1.0;
    k = (((real_T)c_refPoses_data_tmp + 1.0) +
         segEndIdx_data[c_refPoses_data_tmp]) - 1.0;
    if (segEndIdx > k) {
      m = 0;
      h = 0;
    } else {
      m = (int32_T)segEndIdx - 1;
      h = (int32_T)k;
    }

    i = h - m;
    for (vlen = 0; vlen < i; vlen++) {
      tInterp_data[vlen] = cumChordLength_data[m + vlen];
    }

    if (segStartIdx > segEndIdx_data[c_refPoses_data_tmp]) {
      x = 1;
      segStartIdx_size = 0;
    } else {
      x = (int32_T)segStartIdx;
      segStartIdx_size = (int32_T)segEndIdx_data[c_refPoses_data_tmp];
    }

    ip = segStartIdx_size - x;
    tmp_size_0 = ip + 3;
    y = b_refDirections_data[(int32_T)segEndIdx_data[c_refPoses_data_tmp] - 1];
    xloc = c_refPoses_data[((c_refPoses_size_idx_0 << 1) + x) - 1];
    tmp_data_0[0] = cosf(xloc) * y;
    for (vlen = 0; vlen <= ip; vlen++) {
      tmp_data_0[vlen + 1] = c_refPoses_data[(x + vlen) - 1];
    }

    startIdx = c_refPoses_data[((c_refPoses_size_idx_0 << 1) + segStartIdx_size)
      - 1];
    tmp_data_0[ip + 2] = cosf(startIdx) * y;
    Controller_spline(tInterp_data, &i, tmp_data_0, &tmp_size_0,
                      splineX_breaks_data, splineX_breaks_size,
                      splineX_coefs_data, splineX_coefs_size);
    tmp_size_0 = ip + 3;
    tmp_data_0[0] = sinf(xloc) * y;
    for (vlen = 0; vlen <= ip; vlen++) {
      tmp_data_0[vlen + 1] = c_refPoses_data[((x + vlen) + c_refPoses_size_idx_0)
        - 1];
    }

    tmp_data_0[ip + 2] = sinf(startIdx) * y;
    Controller_spline(tInterp_data, &i, tmp_data_0, &tmp_size_0,
                      splineY_breaks_data, b_refPoses_size, splineY_coefs_data,
                      splineY_coefs_size);
    splineDx_breaks_size[0] = 1;
    splineDx_breaks_size[1] = i;
    for (vlen = 0; vlen < i; vlen++) {
      tInterp_data[vlen] = cumChordLength_data[m + vlen];
    }

    segStartIdx_size = (int32_T)((real_T)i - 1.0);
    splineX_coefs_size_0[0] = (int32_T)((real_T)i - 1.0);
    splineX_coefs_size_0[1] = 3;
    for (vlen = 0; vlen < 3; vlen++) {
      for (x = 0; x < segStartIdx_size; x++) {
        splineX_coefs_data_0[x + (int32_T)((real_T)i - 1.0) * vlen] =
          splineX_coefs_data[(int32_T)((real_T)i - 1.0) * vlen + x];
      }
    }

    Controller_bsxfun(splineX_coefs_data_0, splineX_coefs_size_0, tmp_data,
                      tmp_size);
    splineDx_coefs_size[0] = 1;
    splineDx_coefs_size[1] = i - 1;
    splineDx_coefs_size[2] = 3;
    ip = (i - 1) * 3;
    if (ip - 1 >= 0) {
      memcpy(&splineDx_coefs_data[0], &tmp_data[0], (uint32_T)ip * sizeof
             (real32_T));
    }

    splineDy_breaks_size[0] = 1;
    splineDy_breaks_size[1] = i;
    for (vlen = 0; vlen < i; vlen++) {
      tmp_data_0[vlen] = cumChordLength_data[m + vlen];
    }

    splineX_coefs_size_0[0] = (int32_T)((real_T)i - 1.0);
    splineX_coefs_size_0[1] = 3;
    for (vlen = 0; vlen < 3; vlen++) {
      for (x = 0; x < segStartIdx_size; x++) {
        splineX_coefs_data_0[x + (int32_T)((real_T)i - 1.0) * vlen] =
          splineY_coefs_data[(int32_T)((real_T)i - 1.0) * vlen + x];
      }
    }

    Controller_bsxfun(splineX_coefs_data_0, splineX_coefs_size_0, tmp_data,
                      tmp_size);
    splineDy_coefs_size[0] = 1;
    splineDy_coefs_size[1] = i - 1;
    splineDy_coefs_size[2] = 3;
    if (ip - 1 >= 0) {
      memcpy(&splineX_coefs_data_0[0], &tmp_data[0], (uint32_T)ip * sizeof
             (real32_T));
    }

    Controller_linspace(chordLengthOfSegments_data[c_refPoses_data_tmp],
                        z_data[c_refPoses_data_tmp], v);
    tQuery_size[0] = 1;
    tQuery_size[1] = v->size[1];
    x = v->size[1];
    if (x - 1 >= 0) {
      memcpy(&tQuery_data[0], &v->data[0], (uint32_T)x * sizeof(real32_T));
    }

    Controller_ppval(splineX_breaks_data, splineX_breaks_size,
                     splineX_coefs_data, splineX_coefs_size, tQuery_data,
                     tQuery_size, x_data, tmp_size);
    Controller_ppval(splineY_breaks_data, b_refPoses_size, splineY_coefs_data,
                     splineY_coefs_size, tQuery_data, tQuery_size, y_data,
                     splineX_breaks_size);
    x = v->size[0] * v->size[1];
    v->size[0] = 1;
    v->size[1] = tQuery_size[1];
    Cont_emxEnsureCapacity_real32_T(v, x);
    vlen = tQuery_size[1] - 1;
    for (x = 0; x <= vlen; x++) {
      xloc = tQuery_data[x];
      if (rtIsNaNF(xloc)) {
        v->data[x] = xloc;
      } else {
        ip = Controller_bsearch(tInterp_data, splineDx_breaks_size, xloc) - 1;
        xloc -= cumChordLength_data[m + ip];
        v->data[x] = (splineDx_coefs_data[((ip + h) - m) - 1] +
                      splineDx_coefs_data[ip] * xloc) * xloc +
          splineDx_coefs_data[((i - 1) << 1) + ip];
      }
    }

    x = v_0->size[0] * v_0->size[1];
    v_0->size[0] = 1;
    v_0->size[1] = tQuery_size[1];
    Cont_emxEnsureCapacity_real32_T(v_0, x);
    for (x = 0; x <= vlen; x++) {
      xloc = tQuery_data[x];
      if (rtIsNaNF(xloc)) {
        v_0->data[x] = xloc;
      } else {
        ip = Controller_bsearch(tmp_data_0, splineDy_breaks_size, xloc) - 1;
        xloc -= cumChordLength_data[m + ip];
        v_0->data[x] = (splineX_coefs_data_0[((ip + h) - m) - 1] +
                        splineX_coefs_data_0[ip] * xloc) * xloc +
          splineX_coefs_data_0[(((h - m) - 1) << 1) + ip];
      }
    }

    segEndIdx = (real_T)(y == -1.0F) * 3.1415926535897931;
    if (v_0->size[1] == v->size[1]) {
      theta_size[1] = v_0->size[1];
      i = v_0->size[1];
      for (vlen = 0; vlen < i; vlen++) {
        theta_data[vlen] = rt_atan2f_snf(v_0->data[vlen], v->data[vlen]);
      }
    } else {
      Controller_expand_atan2(v_0->data, v_0->size, v->data, v->size, theta_data,
        theta_size);
    }

    theta_size_0[1] = theta_size[1];
    i = theta_size[1];
    for (vlen = 0; vlen < i; vlen++) {
      theta_data_0[vlen] = theta_data[vlen] - (real32_T)segEndIdx;
    }

    i = theta_size_0[1];
    for (vlen = 0; vlen < i; vlen++) {
      xloc = theta_data_0[vlen];
      if (rtIsNaNF(xloc) || rtIsInfF(xloc)) {
        startIdx = (rtNaNF);
      } else if (xloc == 0.0F) {
        startIdx = 0.0F;
      } else {
        boolean_T rEQ0;
        startIdx = fmodf(xloc, 6.28318548F);
        rEQ0 = (startIdx == 0.0F);
        if (!rEQ0) {
          real32_T q;
          q = fabsf(xloc / 6.28318548F);
          rEQ0 = !(fabsf(q - floorf(q + 0.5F)) > 1.1920929E-7F * q);
        }

        if (rEQ0) {
          startIdx = 0.0F;
        } else if (xloc < 0.0F) {
          startIdx += 6.28318548F;
        }
      }

      theta_data[vlen] = startIdx;
    }

    i = theta_size_0[1] - 1;
    for (vlen = 0; vlen <= i; vlen++) {
      xloc = theta_data[vlen];
      theta_data[vlen] = (real32_T)((xloc == 0.0F) && (theta_data_0[vlen] > 0.0F))
        * 6.28318548F + xloc;
    }

    h = tmp_size[1];
    i = tmp_size[1];
    if (i - 1 >= 0) {
      memcpy(&posesSeg_data[0], &x_data[0], (uint32_T)i * sizeof(real32_T));
    }

    i = splineX_breaks_size[1];
    for (vlen = 0; vlen < i; vlen++) {
      posesSeg_data[vlen + tmp_size[1]] = y_data[vlen];
    }

    i = theta_size[1];
    for (vlen = 0; vlen < i; vlen++) {
      posesSeg_data[(vlen + tmp_size[1]) + splineX_breaks_size[1]] =
        theta_data[vlen];
    }

    xloc = z_data[0];
    for (i = 2; i <= c_refPoses_data_tmp + 1; i++) {
      xloc += z_data[i - 1];
    }

    startIdx = (xloc - z_data[c_refPoses_data_tmp]) + 1.0F;
    if (startIdx > xloc) {
      x = 0;
    } else {
      x = (int32_T)startIdx - 1;
    }

    Con_computeCumulativePathLength(tInterp_data, splineDx_breaks_size,
      splineDx_coefs_data, tmp_data_0, splineDy_breaks_size,
      splineX_coefs_data_0, tQuery_data, tQuery_size, x_data, tmp_size);
    i = tmp_size[1];
    for (vlen = 0; vlen < i; vlen++) {
      varargout_1[x + vlen] = x_data[vlen];
    }

    if (startIdx > xloc) {
      x = 0;
    } else {
      x = (int32_T)startIdx - 1;
    }

    Controlle_computePathCurvatures(tInterp_data, splineDx_breaks_size,
      splineDx_coefs_data, splineDx_coefs_size, tmp_data_0, splineDy_breaks_size,
      splineX_coefs_data_0, splineDy_coefs_size, tQuery_data, tQuery_size,
      x_data, tmp_size);
    i = tmp_size[1];
    for (vlen = 0; vlen < i; vlen++) {
      varargout_2[x + vlen] = x_data[vlen];
    }

    if (startIdx > xloc) {
      m = 0;
    } else {
      m = (int32_T)startIdx - 1;
    }

    for (vlen = 0; vlen < 3; vlen++) {
      for (x = 0; x < h; x++) {
        poses[(m + x) + 50 * vlen] = posesSeg_data[h * vlen + x];
      }
    }

    if (startIdx > xloc) {
      i = 0;
    } else {
      i = (int32_T)startIdx - 1;
    }

    for (vlen = 0; vlen < h; vlen++) {
      directions[i + vlen] = y;
    }
  }

  Controller_emxFree_real32_T(&v_0);
  Controller_emxFree_real32_T(&v);
  for (vlen = 0; vlen < 50; vlen++) {
    poses[vlen + 100] *= 57.2957802F;
  }
}

static boolean_T Controller_isequal(const real32_T varargin_1[50], const
  real32_T varargin_2[50])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 50)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

static void getSegmentBoundaryPointIndex_l(const real32_T directions[50], real_T
  segStartIdx_data[], int32_T *segStartIdx_size, real_T segEndIdx_data[],
  int32_T *segEndIdx_size)
{
  int32_T ii_data[49];
  int32_T switchIndex_data[49];
  int32_T b_ii;
  int32_T idx;
  boolean_T x[49];
  boolean_T exitg1;
  for (b_ii = 0; b_ii < 49; b_ii++) {
    x[b_ii] = (directions[(b_ii + 2) - 1] + directions[b_ii] == 0.0F);
  }

  idx = 0;
  b_ii = 1;
  exitg1 = false;
  while ((!exitg1) && (b_ii - 1 < 49)) {
    if (x[b_ii - 1]) {
      idx++;
      ii_data[idx - 1] = b_ii;
      if (idx >= 49) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }

  if (idx < 1) {
    idx = 0;
  }

  if (idx - 1 >= 0) {
    memcpy(&switchIndex_data[0], &ii_data[0], (uint32_T)idx * sizeof(int32_T));
  }

  *segStartIdx_size = idx + 1;
  segStartIdx_data[0] = 1.0;
  for (b_ii = 0; b_ii < idx; b_ii++) {
    segStartIdx_data[b_ii + 1] = (real_T)switchIndex_data[b_ii] + 1.0;
  }

  *segEndIdx_size = idx + 1;
  for (b_ii = 0; b_ii < idx; b_ii++) {
    segEndIdx_data[b_ii] = switchIndex_data[b_ii];
  }

  segEndIdx_data[idx] = 50.0;
}

static real32_T Controller_minimum(const real32_T x_data[], const int32_T
  *x_size)
{
  int32_T k;
  int32_T last;
  real32_T ex;
  last = *x_size;
  if ((uint8_T)(*x_size - 1) + 1 <= 2) {
    if ((uint8_T)(*x_size - 1) + 1 == 1) {
      ex = x_data[0];
    } else {
      ex = x_data[*x_size - 1];
      if ((x_data[0] > ex) || (rtIsNaNF(x_data[0]) && (!rtIsNaNF(ex)))) {
      } else {
        ex = x_data[0];
      }
    }
  } else {
    int32_T b_idx;
    if (!rtIsNaNF(x_data[0])) {
      b_idx = 1;
    } else {
      boolean_T exitg1;
      b_idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= *x_size)) {
        if (!rtIsNaNF(x_data[k - 1])) {
          b_idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (b_idx == 0) {
      ex = x_data[0];
    } else {
      ex = x_data[b_idx - 1];
      for (k = b_idx + 1; k <= last; k++) {
        real32_T tmp;
        tmp = x_data[k - 1];
        if (ex > tmp) {
          ex = tmp;
        }
      }
    }
  }

  return ex;
}

static void getNonConstSpeedIntervalDistanc(real32_T vBound, real32_T vMax,
  real32_T S[3])
{
  /* ------------------------------------------------------------------ */
  /* getNonConstSpeedIntervalDistance Calculate the travelled distance */
  /*    during the acceleration or the deceleration interval, i.e., intervals */
  /*    1-3 or 5-7, given the starting or the ending velocity, vBound, */
  /*    the maximum speed, vMax and the maximum acceleration or */
  /*    deceleration, aMax. Before calculating the distance, it needs */
  /*    to determine if vMax can be reached without reaching aMax, */
  /*    i.e., if interval 2 or 6 exists. intervalFlag is used as a flag to */
  /*    distinguish the acceleration interval from the deceleration one. */
  /*  A speed threshold below which aMax is not reached */
  if (vBound + 40.0F <= vMax) {
    real32_T S_tmp;

    /*  interval 2 or 6 needed */
    S[0] = 2.0F * vBound + 13.333333F;
    S_tmp = (vMax - vBound) / 20.0F - 2.0F;
    S[1] = ((S_tmp * 20.0F + (vBound + 20.0F)) + (vBound + 20.0F)) * (S_tmp /
      2.0F);
    S[2] = 2.0F * vMax - 13.333333F;
  } else {
    real32_T S_tmp;
    real32_T deltaT1;
    deltaT1 = sqrtf((vMax - vBound) * 10.0F) / 10.0F;

    /*  interval 2 or 6 disappears */
    S_tmp = 1.66666663F * rt_powf_snf(deltaT1, 3.0F);
    S[0] = deltaT1 * vBound + S_tmp;
    S[1] = 0.0F;
    S[2] = deltaT1 * vMax - S_tmp;
  }

  /*  acceleration interval */
  /*  Interval time */
  /*  Velocity at the interval boundary */
  /*  Interval distance */
}

static void getNonConstSpeedIntervalDista_l(real32_T vBound, real32_T vMax,
  real32_T S[3])
{
  /* ------------------------------------------------------------------ */
  /* getNonConstSpeedIntervalDistance Calculate the travelled distance */
  /*    during the acceleration or the deceleration interval, i.e., intervals */
  /*    1-3 or 5-7, given the starting or the ending velocity, vBound, */
  /*    the maximum speed, vMax and the maximum acceleration or */
  /*    deceleration, aMax. Before calculating the distance, it needs */
  /*    to determine if vMax can be reached without reaching aMax, */
  /*    i.e., if interval 2 or 6 exists. intervalFlag is used as a flag to */
  /*    distinguish the acceleration interval from the deceleration one. */
  /*  A speed threshold below which aMax is not reached */
  if (vBound + 40.0F <= vMax) {
    real32_T S_tmp;

    /*  interval 2 or 6 needed */
    S[2] = 2.0F * vBound + 13.333333F;
    S_tmp = (vMax - vBound) / 20.0F - 2.0F;
    S[1] = ((S_tmp * 20.0F + (vBound + 20.0F)) + (vBound + 20.0F)) * (S_tmp /
      2.0F);
    S[0] = 2.0F * vMax - 13.333333F;
  } else {
    real32_T S_tmp;
    real32_T deltaT1;
    deltaT1 = sqrtf((vMax - vBound) * 10.0F) / 10.0F;

    /*  interval 2 or 6 disappears */
    S_tmp = 1.66666663F * rt_powf_snf(deltaT1, 3.0F);
    S[2] = deltaT1 * vBound + S_tmp;
    S[1] = 0.0F;
    S[0] = deltaT1 * vMax - S_tmp;
  }

  /*  deceleration interval */
  /*  Flip the order */
}

static real32_T Controller_sum(const real32_T x[3])
{
  return (x[0] + x[1]) + x[2];
}

static boolean_T Controller_ifWhileCond(const boolean_T x_data[], const int32_T
  x_size[2])
{
  boolean_T y;
  y = (x_size[1] != 0);
  if (y) {
    int32_T c;
    int32_T k;
    boolean_T exitg1;
    c = x_size[1] - 1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= c)) {
      if (!x_data[0]) {
        y = false;
        exitg1 = true;
      } else {
        k = 1;
      }
    }
  }

  return y;
}

real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T a;
  real32_T b;
  real32_T y;
  a = fabsf(u0);
  b = fabsf(u1);
  if (a < b) {
    a /= b;
    y = sqrtf(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = sqrtf(b * b + 1.0F) * a;
  } else if (rtIsNaNF(b)) {
    y = (rtNaNF);
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

static void Controller_xzlartg(const creal32_T f, const creal32_T g, real32_T
  *cs, creal32_T *sn, creal32_T *r)
{
  int32_T b_i;
  int32_T count;
  real32_T di;
  real32_T f2s;
  real32_T fs_im;
  real32_T fs_re;
  real32_T gs_im;
  real32_T gs_re;
  real32_T scale;
  int8_T rescaledir;
  boolean_T guard1 = false;
  f2s = fabsf(f.re);
  di = fabsf(f.im);
  scale = f2s;
  if (di > f2s) {
    scale = di;
  }

  gs_re = fabsf(g.re);
  gs_im = fabsf(g.im);
  if (gs_im > gs_re) {
    gs_re = gs_im;
  }

  if (gs_re > scale) {
    scale = gs_re;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = -1;
  rescaledir = 0;
  guard1 = false;
  if (scale >= 5.49755814E+11F) {
    do {
      count++;
      fs_re *= 1.8189894E-12F;
      fs_im *= 1.8189894E-12F;
      gs_re *= 1.8189894E-12F;
      gs_im *= 1.8189894E-12F;
      scale *= 1.8189894E-12F;
    } while ((scale >= 5.49755814E+11F) && (count + 1 < 20));

    rescaledir = 1;
    guard1 = true;
  } else if (scale <= 1.8189894E-12F) {
    if (((g.re == 0.0F) && (g.im == 0.0F)) || (rtIsNaNF(g.re) || rtIsNaNF(g.im)))
    {
      *cs = 1.0F;
      sn->re = 0.0F;
      sn->im = 0.0F;
      *r = f;
    } else {
      do {
        count++;
        fs_re *= 5.49755814E+11F;
        fs_im *= 5.49755814E+11F;
        gs_re *= 5.49755814E+11F;
        gs_im *= 5.49755814E+11F;
        scale *= 5.49755814E+11F;
      } while (!!(scale <= 1.8189894E-12F));

      rescaledir = -1;
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    real32_T b_x;
    real32_T g2;
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    b_x = g2;
    if (g2 < 1.0F) {
      b_x = 1.0F;
    }

    if (scale <= b_x * 1.97215226E-31F) {
      if ((f.re == 0.0F) && (f.im == 0.0F)) {
        *cs = 0.0F;
        r->re = rt_hypotf_snf(g.re, g.im);
        r->im = 0.0F;
        f2s = rt_hypotf_snf(gs_re, gs_im);
        sn->re = gs_re / f2s;
        sn->im = -gs_im / f2s;
      } else {
        scale = sqrtf(g2);
        *cs = rt_hypotf_snf(fs_re, fs_im) / scale;
        if (di > f2s) {
          f2s = di;
        }

        if (f2s > 1.0F) {
          f2s = rt_hypotf_snf(f.re, f.im);
          fs_re = f.re / f2s;
          fs_im = f.im / f2s;
        } else {
          fs_re = 5.49755814E+11F * f.re;
          di = 5.49755814E+11F * f.im;
          f2s = rt_hypotf_snf(fs_re, di);
          fs_re /= f2s;
          fs_im = di / f2s;
        }

        gs_re /= scale;
        gs_im = -gs_im / scale;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
        r->re = (sn->re * g.re - sn->im * g.im) + *cs * f.re;
        r->im = (sn->re * g.im + sn->im * g.re) + *cs * f.im;
      }
    } else {
      f2s = sqrtf(g2 / scale + 1.0F);
      r->re = f2s * fs_re;
      r->im = f2s * fs_im;
      *cs = 1.0F / f2s;
      f2s = scale + g2;
      fs_re = r->re / f2s;
      f2s = r->im / f2s;
      sn->re = fs_re * gs_re - f2s * -gs_im;
      sn->im = fs_re * -gs_im + f2s * gs_re;
      if (rescaledir > 0) {
        for (b_i = 0; b_i <= count; b_i++) {
          r->re *= 5.49755814E+11F;
          r->im *= 5.49755814E+11F;
        }
      } else if (rescaledir < 0) {
        for (b_i = 0; b_i <= count; b_i++) {
          r->re *= 1.8189894E-12F;
          r->im *= 1.8189894E-12F;
        }
      }
    }
  }
}

static creal32_T Controller_sqrt(const creal32_T x)
{
  creal32_T b_x;
  real32_T absxi;
  real32_T absxr;
  if (x.im == 0.0F) {
    if (x.re < 0.0F) {
      absxr = 0.0F;
      absxi = sqrtf(-x.re);
    } else {
      absxr = sqrtf(x.re);
      absxi = 0.0F;
    }
  } else if (x.re == 0.0F) {
    if (x.im < 0.0F) {
      absxr = sqrtf(-x.im / 2.0F);
      absxi = -absxr;
    } else {
      absxr = sqrtf(x.im / 2.0F);
      absxi = absxr;
    }
  } else if (rtIsNaNF(x.re)) {
    absxr = (rtNaNF);
    absxi = (rtNaNF);
  } else if (rtIsNaNF(x.im)) {
    absxr = (rtNaNF);
    absxi = (rtNaNF);
  } else if (rtIsInfF(x.im)) {
    absxr = fabsf(x.im);
    absxi = x.im;
  } else if (rtIsInfF(x.re)) {
    if (x.re < 0.0F) {
      absxr = 0.0F;
      absxi = x.im * -x.re;
    } else {
      absxr = x.re;
      absxi = 0.0F;
    }
  } else {
    absxr = fabsf(x.re);
    absxi = fabsf(x.im);
    if ((absxr > 8.50705867E+37F) || (absxi > 8.50705867E+37F)) {
      absxr *= 0.5F;
      absxi = rt_hypotf_snf(absxr, absxi * 0.5F);
      if (absxi > absxr) {
        absxr = sqrtf(absxr / absxi + 1.0F) * sqrtf(absxi);
      } else {
        absxr = sqrtf(absxi) * 1.41421354F;
      }
    } else {
      absxr = sqrtf((rt_hypotf_snf(absxr, absxi) + absxr) * 0.5F);
    }

    if (x.re > 0.0F) {
      absxi = x.im / absxr * 0.5F;
    } else {
      if (x.im < 0.0F) {
        absxi = -absxr;
      } else {
        absxi = absxr;
      }

      absxr = x.im / absxi * 0.5F;
    }
  }

  b_x.re = absxr;
  b_x.im = absxi;
  return b_x;
}

static void Controller_xzlartg_l(const creal32_T f, const creal32_T g, real32_T *
  cs, creal32_T *sn)
{
  int32_T count;
  real32_T d;
  real32_T f2s;
  real32_T fs_im;
  real32_T fs_re;
  real32_T gs_im;
  real32_T gs_re;
  real32_T scale;
  boolean_T guard1 = false;
  d = fabsf(f.re);
  f2s = fabsf(f.im);
  scale = d;
  if (f2s > d) {
    scale = f2s;
  }

  gs_re = fabsf(g.re);
  gs_im = fabsf(g.im);
  if (gs_im > gs_re) {
    gs_re = gs_im;
  }

  if (gs_re > scale) {
    scale = gs_re;
  }

  fs_re = f.re;
  fs_im = f.im;
  gs_re = g.re;
  gs_im = g.im;
  count = 0;
  guard1 = false;
  if (scale >= 5.49755814E+11F) {
    do {
      count++;
      fs_re *= 1.8189894E-12F;
      fs_im *= 1.8189894E-12F;
      gs_re *= 1.8189894E-12F;
      gs_im *= 1.8189894E-12F;
      scale *= 1.8189894E-12F;
    } while ((scale >= 5.49755814E+11F) && (count < 20));

    guard1 = true;
  } else if (scale <= 1.8189894E-12F) {
    if (((g.re == 0.0F) && (g.im == 0.0F)) || (rtIsNaNF(g.re) || rtIsNaNF(g.im)))
    {
      *cs = 1.0F;
      sn->re = 0.0F;
      sn->im = 0.0F;
    } else {
      do {
        fs_re *= 5.49755814E+11F;
        fs_im *= 5.49755814E+11F;
        gs_re *= 5.49755814E+11F;
        gs_im *= 5.49755814E+11F;
        scale *= 5.49755814E+11F;
      } while (!!(scale <= 1.8189894E-12F));

      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    real32_T b_x;
    real32_T g2;
    scale = fs_re * fs_re + fs_im * fs_im;
    g2 = gs_re * gs_re + gs_im * gs_im;
    b_x = g2;
    if (g2 < 1.0F) {
      b_x = 1.0F;
    }

    if (scale <= b_x * 1.97215226E-31F) {
      if ((f.re == 0.0F) && (f.im == 0.0F)) {
        *cs = 0.0F;
        d = rt_hypotf_snf(gs_re, gs_im);
        sn->re = gs_re / d;
        sn->im = -gs_im / d;
      } else {
        scale = sqrtf(g2);
        *cs = rt_hypotf_snf(fs_re, fs_im) / scale;
        if (f2s > d) {
          d = f2s;
        }

        if (d > 1.0F) {
          d = rt_hypotf_snf(f.re, f.im);
          fs_re = f.re / d;
          fs_im = f.im / d;
        } else {
          fs_re = 5.49755814E+11F * f.re;
          f2s = 5.49755814E+11F * f.im;
          d = rt_hypotf_snf(fs_re, f2s);
          fs_re /= d;
          fs_im = f2s / d;
        }

        gs_re /= scale;
        gs_im = -gs_im / scale;
        sn->re = fs_re * gs_re - fs_im * gs_im;
        sn->im = fs_re * gs_im + fs_im * gs_re;
      }
    } else {
      f2s = sqrtf(g2 / scale + 1.0F);
      *cs = 1.0F / f2s;
      d = scale + g2;
      fs_re = f2s * fs_re / d;
      fs_im = f2s * fs_im / d;
      sn->re = fs_re * gs_re - fs_im * -gs_im;
      sn->im = fs_re * -gs_im + fs_im * gs_re;
    }
  }
}

static void Controller_xzhgeqz(const creal32_T A_data[], const int32_T A_size[2],
  int32_T ilo, int32_T ihi, int32_T *info, creal32_T alpha1_data[], int32_T
  *alpha1_size, creal32_T beta1_data[], int32_T *beta1_size)
{
  creal32_T b_A_data[9];
  creal32_T ad22;
  creal32_T ascale;
  creal32_T ascale_0;
  creal32_T ascale_1;
  creal32_T ctemp;
  creal32_T y;
  int32_T b_A_size_idx_0;
  int32_T col;
  int32_T ctemp_tmp_tmp;
  int32_T iiter;
  int32_T ilast;
  int32_T ilastm1;
  int32_T jp1;
  int32_T n;
  int32_T nm1;
  real32_T absxk;
  real32_T anorm;
  real32_T colscale;
  real32_T colssq;
  real32_T eshift_im;
  real32_T eshift_re;
  real32_T scale;
  real32_T t;
  boolean_T failed;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  b_A_size_idx_0 = A_size[0];
  nm1 = A_size[0] * A_size[1];
  if (nm1 - 1 >= 0) {
    memcpy(&b_A_data[0], &A_data[0], (uint32_T)nm1 * sizeof(creal32_T));
  }

  *info = 0;
  if ((A_size[0] == 1) && (A_size[1] == 1)) {
    ihi = 1;
  }

  n = A_size[0];
  *alpha1_size = A_size[0];
  nm1 = A_size[0];
  if (nm1 - 1 >= 0) {
    memset(&alpha1_data[0], 0, (uint32_T)nm1 * sizeof(creal32_T));
  }

  *beta1_size = A_size[0];
  nm1 = A_size[0];
  for (ilast = 0; ilast < nm1; ilast++) {
    beta1_data[ilast].re = 1.0F;
    beta1_data[ilast].im = 0.0F;
  }

  eshift_re = 0.0F;
  eshift_im = 0.0F;
  ctemp.re = 0.0F;
  ctemp.im = 0.0F;
  anorm = 0.0F;
  if (ilo <= ihi) {
    scale = 1.29246971E-26F;
    anorm = 0.0F;
    nm1 = ihi - ilo;
    for (ilastm1 = 0; ilastm1 <= nm1; ilastm1++) {
      colscale = 1.29246971E-26F;
      colssq = 0.0F;
      col = (ilo + ilastm1) - 1;
      if (ilastm1 + 1 <= nm1) {
        ilast = ilastm1 + 1;
      } else {
        ilast = nm1;
      }

      ilast += ilo;
      for (iiter = ilo; iiter <= ilast; iiter++) {
        absxk = fabsf(A_data[(A_size[0] * col + iiter) - 1].re);
        if (absxk > colscale) {
          t = colscale / absxk;
          colssq = colssq * t * t + 1.0F;
          colscale = absxk;
        } else {
          t = absxk / colscale;
          colssq += t * t;
        }

        absxk = fabsf(A_data[(A_size[0] * col + iiter) - 1].im);
        if (absxk > colscale) {
          t = colscale / absxk;
          colssq = colssq * t * t + 1.0F;
          colscale = absxk;
        } else {
          t = absxk / colscale;
          colssq += t * t;
        }
      }

      if (scale >= colscale) {
        absxk = colscale / scale;
        anorm += absxk * absxk * colssq;
      } else {
        absxk = scale / colscale;
        anorm = absxk * absxk * anorm + colssq;
        scale = colscale;
      }
    }

    anorm = scale * sqrtf(anorm);
  }

  colscale = 1.1920929E-7F * anorm;
  scale = 1.17549435E-38F;
  if (colscale > 1.17549435E-38F) {
    scale = colscale;
  }

  colscale = 1.17549435E-38F;
  if (anorm > 1.17549435E-38F) {
    colscale = anorm;
  }

  anorm = 1.0F / colscale;
  colscale = 1.0F / sqrtf((real32_T)A_size[0]);
  failed = true;
  for (ilastm1 = ihi + 1; ilastm1 <= n; ilastm1++) {
    alpha1_data[ilastm1 - 1] = A_data[((ilastm1 - 1) * A_size[0] + ilastm1) - 1];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    int32_T jiter;
    boolean_T goto60;
    boolean_T goto70;
    boolean_T goto90;
    n = ilo;
    nm1 = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    col = ihi;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (jiter <= ((ihi - ilo) + 1) * 30 - 1) {
        int32_T absxk_tmp;
        int32_T j;
        boolean_T exitg2;
        boolean_T guard11 = false;
        guard11 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          guard11 = true;
        } else {
          ctemp_tmp_tmp = b_A_size_idx_0 * ilastm1;
          absxk_tmp = ilast + ctemp_tmp_tmp;
          jp1 = b_A_size_idx_0 * ilast + ilast;
          ctemp_tmp_tmp += ilastm1;
          if (fabsf(b_A_data[absxk_tmp].re) + fabsf(b_A_data[absxk_tmp].im) <=
              fmaxf(1.17549435E-38F, ((fabsf(b_A_data[jp1].re) + fabsf
                 (b_A_data[jp1].im)) + (fabsf(b_A_data[ctemp_tmp_tmp].re) +
                 fabsf(b_A_data[ctemp_tmp_tmp].im))) * 1.1920929E-7F)) {
            b_A_data[absxk_tmp].re = 0.0F;
            b_A_data[absxk_tmp].im = 0.0F;
            goto60 = true;
            guard11 = true;
          } else {
            boolean_T guard3 = false;
            j = ilastm1 - 1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (j + 2 >= ilo)) {
              if (j + 2 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                ctemp_tmp_tmp = b_A_size_idx_0 * j;
                absxk_tmp = (j + ctemp_tmp_tmp) + 1;
                jp1 = ((j + 1) * b_A_size_idx_0 + j) + 1;
                ctemp_tmp_tmp += j;
                if (fabsf(b_A_data[absxk_tmp].re) + fabsf(b_A_data[absxk_tmp].im)
                    <= fmaxf(1.17549435E-38F, ((fabsf(b_A_data[jp1].re) + fabsf
                       (b_A_data[jp1].im)) + (fabsf(b_A_data[ctemp_tmp_tmp].re)
                       + fabsf(b_A_data[ctemp_tmp_tmp].im))) * 1.1920929E-7F)) {
                  b_A_data[absxk_tmp].re = 0.0F;
                  b_A_data[absxk_tmp].im = 0.0F;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  j--;
                  guard3 = false;
                }
              }
            }

            if (guard3) {
              n = j + 2;
              goto70 = true;
            }

            if (goto70) {
              guard11 = true;
            } else {
              n = *alpha1_size;
              for (ilast = 0; ilast < n; ilast++) {
                alpha1_data[ilast].re = (rtNaNF);
                alpha1_data[ilast].im = 0.0F;
              }

              n = *beta1_size;
              for (ilast = 0; ilast < n; ilast++) {
                beta1_data[ilast].re = (rtNaNF);
                beta1_data[ilast].im = 0.0F;
              }

              *info = 1;
              exitg1 = 1;
            }
          }
        }

        if (guard11) {
          if (goto60) {
            goto60 = false;
            alpha1_data[ilast] = b_A_data[b_A_size_idx_0 * ilast + ilast];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              failed = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0F;
              eshift_im = 0.0F;
              col = ilast + 1;
              jiter++;
            }
          } else {
            if (goto70) {
              real32_T temp;
              real32_T y_im;
              goto70 = false;
              iiter++;
              if (iiter - iiter / 10 * 10 != 0) {
                creal32_T tmp;
                creal32_T tmp_0;
                absxk_tmp = b_A_size_idx_0 * ilast + ilast;
                absxk = anorm * b_A_data[absxk_tmp].re;
                colssq = anorm * b_A_data[absxk_tmp].im;
                if (colssq == 0.0F) {
                  ad22.re = absxk / colscale;
                  ad22.im = 0.0F;
                } else if (absxk == 0.0F) {
                  ad22.re = 0.0F;
                  ad22.im = colssq / colscale;
                } else {
                  ad22.re = absxk / colscale;
                  ad22.im = colssq / colscale;
                }

                absxk_tmp = b_A_size_idx_0 * ilast + ilastm1;
                absxk = anorm * b_A_data[absxk_tmp].re;
                colssq = anorm * b_A_data[absxk_tmp].im;
                if (colssq == 0.0F) {
                  ascale_0.re = absxk / colscale;
                  ascale_0.im = 0.0F;
                } else if (absxk == 0.0F) {
                  ascale_0.re = 0.0F;
                  ascale_0.im = colssq / colscale;
                } else {
                  ascale_0.re = absxk / colscale;
                  ascale_0.im = colssq / colscale;
                }

                tmp = Controller_sqrt(ascale_0);
                absxk_tmp = b_A_size_idx_0 * ilastm1 + ilast;
                absxk = anorm * b_A_data[absxk_tmp].re;
                colssq = anorm * b_A_data[absxk_tmp].im;
                if (colssq == 0.0F) {
                  ascale_1.re = absxk / colscale;
                  ascale_1.im = 0.0F;
                } else if (absxk == 0.0F) {
                  ascale_1.re = 0.0F;
                  ascale_1.im = colssq / colscale;
                } else {
                  ascale_1.re = absxk / colscale;
                  ascale_1.im = colssq / colscale;
                }

                tmp_0 = Controller_sqrt(ascale_1);
                ctemp.re = tmp.re * tmp_0.re - tmp.im * tmp_0.im;
                ctemp.im = tmp.re * tmp_0.im + tmp.im * tmp_0.re;
                if ((ctemp.re != 0.0F) || (ctemp.im != 0.0F)) {
                  real32_T y_re;
                  real32_T z_im;
                  real32_T z_re;
                  absxk_tmp = b_A_size_idx_0 * ilastm1 + ilastm1;
                  absxk = anorm * b_A_data[absxk_tmp].re;
                  colssq = anorm * b_A_data[absxk_tmp].im;
                  if (colssq == 0.0F) {
                    absxk /= colscale;
                    t = 0.0F;
                  } else if (absxk == 0.0F) {
                    absxk = 0.0F;
                    t = colssq / colscale;
                  } else {
                    absxk /= colscale;
                    t = colssq / colscale;
                  }

                  colssq = (absxk - ad22.re) * 0.5F;
                  absxk = (t - ad22.im) * 0.5F;
                  t = fabsf(colssq) + fabsf(absxk);
                  temp = fmaxf(fabsf(ctemp.re) + fabsf(ctemp.im), t);
                  if (absxk == 0.0F) {
                    y_re = colssq / temp;
                    y_im = 0.0F;
                  } else if (colssq == 0.0F) {
                    y_re = 0.0F;
                    y_im = absxk / temp;
                  } else {
                    y_re = colssq / temp;
                    y_im = absxk / temp;
                  }

                  if (ctemp.im == 0.0F) {
                    z_re = ctemp.re / temp;
                    z_im = 0.0F;
                  } else if (ctemp.re == 0.0F) {
                    z_re = 0.0F;
                    z_im = ctemp.im / temp;
                  } else {
                    z_re = ctemp.re / temp;
                    z_im = ctemp.im / temp;
                  }

                  y.re = (y_re * y_re - y_im * y_im) + (z_re * z_re - z_im *
                    z_im);
                  y_im *= y_re;
                  y_re = z_re * z_im;
                  y.im = (y_im + y_im) + (y_re + y_re);
                  tmp = Controller_sqrt(y);
                  y_re = temp * tmp.re;
                  y_im = temp * tmp.im;
                  if (t > 0.0F) {
                    if (absxk == 0.0F) {
                      temp = colssq / t;
                      t = 0.0F;
                    } else {
                      if (colssq == 0.0F) {
                        temp = 0.0F;
                      } else {
                        temp = colssq / t;
                      }

                      t = absxk / t;
                    }

                    if (temp * y_re + t * y_im < 0.0F) {
                      y_re = -y_re;
                      y_im = -y_im;
                    }
                  }

                  t = colssq + y_re;
                  absxk += y_im;
                  if (absxk == 0.0F) {
                    if (ctemp.im == 0.0F) {
                      colssq = ctemp.re / t;
                      absxk = 0.0F;
                    } else if (ctemp.re == 0.0F) {
                      colssq = 0.0F;
                      absxk = ctemp.im / t;
                    } else {
                      colssq = ctemp.re / t;
                      absxk = ctemp.im / t;
                    }
                  } else if (t == 0.0F) {
                    if (ctemp.re == 0.0F) {
                      colssq = ctemp.im / absxk;
                      absxk = 0.0F;
                    } else if (ctemp.im == 0.0F) {
                      colssq = 0.0F;
                      absxk = -(ctemp.re / absxk);
                    } else {
                      colssq = ctemp.im / absxk;
                      absxk = -(ctemp.re / absxk);
                    }
                  } else {
                    temp = fabsf(t);
                    colssq = fabsf(absxk);
                    if (temp > colssq) {
                      temp = absxk / t;
                      absxk = temp * absxk + t;
                      colssq = (temp * ctemp.im + ctemp.re) / absxk;
                      absxk = (ctemp.im - temp * ctemp.re) / absxk;
                    } else if (colssq == temp) {
                      t = t > 0.0F ? 0.5F : -0.5F;
                      absxk = absxk > 0.0F ? 0.5F : -0.5F;
                      colssq = (ctemp.re * t + ctemp.im * absxk) / temp;
                      absxk = (ctemp.im * t - ctemp.re * absxk) / temp;
                    } else {
                      temp = t / absxk;
                      absxk += temp * t;
                      colssq = (temp * ctemp.re + ctemp.im) / absxk;
                      absxk = (temp * ctemp.im - ctemp.re) / absxk;
                    }
                  }

                  ad22.re -= ctemp.re * colssq - ctemp.im * absxk;
                  ad22.im -= ctemp.re * absxk + ctemp.im * colssq;
                }
              } else {
                if (iiter - iiter / 20 * 20 == 0) {
                  absxk_tmp = b_A_size_idx_0 * ilast + ilast;
                  absxk = anorm * b_A_data[absxk_tmp].re;
                  colssq = anorm * b_A_data[absxk_tmp].im;
                  if (colssq == 0.0F) {
                    absxk /= colscale;
                    t = 0.0F;
                  } else if (absxk == 0.0F) {
                    absxk = 0.0F;
                    t = colssq / colscale;
                  } else {
                    absxk /= colscale;
                    t = colssq / colscale;
                  }

                  eshift_re += absxk;
                  eshift_im += t;
                } else {
                  absxk_tmp = b_A_size_idx_0 * ilastm1 + ilast;
                  absxk = anorm * b_A_data[absxk_tmp].re;
                  colssq = anorm * b_A_data[absxk_tmp].im;
                  if (colssq == 0.0F) {
                    absxk /= colscale;
                    t = 0.0F;
                  } else if (absxk == 0.0F) {
                    absxk = 0.0F;
                    t = colssq / colscale;
                  } else {
                    absxk /= colscale;
                    t = colssq / colscale;
                  }

                  eshift_re += absxk;
                  eshift_im += t;
                }

                ad22.re = eshift_re;
                ad22.im = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > n)) {
                nm1 = j + 1;
                ctemp_tmp_tmp = b_A_size_idx_0 * j;
                absxk_tmp = j + ctemp_tmp_tmp;
                ctemp.re = anorm * b_A_data[absxk_tmp].re - ad22.re * colscale;
                ctemp.im = anorm * b_A_data[absxk_tmp].im - ad22.im * colscale;
                temp = fabsf(ctemp.re) + fabsf(ctemp.im);
                jp1 += ctemp_tmp_tmp;
                t = (fabsf(b_A_data[jp1].re) + fabsf(b_A_data[jp1].im)) * anorm;
                colssq = temp;
                if (t > temp) {
                  colssq = t;
                }

                if ((colssq < 1.0F) && (colssq != 0.0F)) {
                  temp /= colssq;
                  t /= colssq;
                }

                ctemp_tmp_tmp = (j - 1) * b_A_size_idx_0 + j;
                if ((fabsf(b_A_data[ctemp_tmp_tmp].re) + fabsf
                     (b_A_data[ctemp_tmp_tmp].im)) * t <= temp * scale) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                nm1 = n;
                absxk_tmp = ((n - 1) * b_A_size_idx_0 + n) - 1;
                ctemp.re = b_A_data[absxk_tmp].re * anorm - ad22.re * colscale;
                ctemp.im = b_A_data[absxk_tmp].im * anorm - ad22.im * colscale;
              }

              goto90 = false;
              j = (nm1 - 1) * b_A_size_idx_0 + nm1;
              ascale.re = b_A_data[j].re * anorm;
              ascale.im = b_A_data[j].im * anorm;
              Controller_xzlartg_l(ctemp, ascale, &colssq, &ad22);
              j = nm1;
              jp1 = nm1 - 2;
              while (j < ilast + 1) {
                int32_T temp_tmp;
                if (j > nm1) {
                  Controller_xzlartg(b_A_data[1 + b_A_size_idx_0 * jp1],
                                     b_A_data[j + b_A_size_idx_0 * jp1], &colssq,
                                     &ad22, &b_A_data[1 + b_A_size_idx_0 * jp1]);
                  ctemp_tmp_tmp = b_A_size_idx_0 * jp1 + j;
                  b_A_data[ctemp_tmp_tmp].re = 0.0F;
                  b_A_data[ctemp_tmp_tmp].im = 0.0F;
                }

                for (jp1 = j; jp1 <= col; jp1++) {
                  absxk_tmp = (jp1 - 1) * b_A_size_idx_0 + j;
                  absxk = b_A_data[absxk_tmp].im;
                  t = b_A_data[absxk_tmp].re;
                  temp_tmp = ((jp1 - 1) * b_A_size_idx_0 + j) - 1;
                  temp = b_A_data[temp_tmp].re;
                  y_im = b_A_data[temp_tmp].im;
                  ctemp_tmp_tmp = (jp1 - 1) * b_A_size_idx_0;
                  absxk_tmp = j + ctemp_tmp_tmp;
                  b_A_data[absxk_tmp].re = t * colssq - (temp * ad22.re + y_im *
                    ad22.im);
                  b_A_data[absxk_tmp].im = absxk * colssq - (y_im * ad22.re -
                    ad22.im * temp);
                  b_A_data[(j + ctemp_tmp_tmp) - 1].re = (t * ad22.re - absxk *
                    ad22.im) + temp * colssq;
                  b_A_data[(j + b_A_size_idx_0 * (jp1 - 1)) - 1].im = (absxk *
                    ad22.re + t * ad22.im) + y_im * colssq;
                }

                ad22.re = -ad22.re;
                ad22.im = -ad22.im;
                jp1 = j + 2;
                if (ilast + 1 < j + 2) {
                  jp1 = ilast + 1;
                }

                for (ctemp_tmp_tmp = n; ctemp_tmp_tmp <= jp1; ctemp_tmp_tmp++) {
                  absxk_tmp = ((j - 1) * b_A_size_idx_0 + ctemp_tmp_tmp) - 1;
                  absxk = b_A_data[absxk_tmp].im;
                  t = b_A_data[absxk_tmp].re;
                  temp_tmp = (b_A_size_idx_0 * j + ctemp_tmp_tmp) - 1;
                  temp = b_A_data[temp_tmp].re;
                  y_im = b_A_data[temp_tmp].im;
                  b_A_data[absxk_tmp].re = t * colssq - (temp * ad22.re + y_im *
                    ad22.im);
                  b_A_data[absxk_tmp].im = absxk * colssq - (y_im * ad22.re -
                    ad22.im * temp);
                  b_A_data[temp_tmp].re = (t * ad22.re - absxk * ad22.im) + temp
                    * colssq;
                  b_A_data[temp_tmp].im = (absxk * ad22.re + t * ad22.im) + y_im
                    * colssq;
                }

                jp1 = j - 1;
                j++;
              }
            }

            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      for (n = 0; n <= ilast; n++) {
        alpha1_data[n].re = (rtNaNF);
        alpha1_data[n].im = 0.0F;
        beta1_data[n].re = (rtNaNF);
        beta1_data[n].im = 0.0F;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    ilast = (uint8_T)(ilo - 1) - 1;
    for (n = 0; n <= ilast; n++) {
      alpha1_data[n] = b_A_data[b_A_size_idx_0 * n + n];
    }
  }
}

static void Controller_xzgeev(const real32_T A_data[], const int32_T A_size[2],
  int32_T *info, creal32_T alpha1_data[], int32_T *alpha1_size, creal32_T
  beta1_data[], int32_T *beta1_size)
{
  creal32_T At_data[9];
  creal32_T s;
  int32_T At_size[2];
  int32_T i;
  int32_T ihi;
  int32_T ii;
  int32_T jcol;
  int32_T loop_ub;
  real32_T absxk;
  real32_T anrm;
  real32_T cfromc;
  boolean_T exitg1;
  At_size[0] = A_size[0];
  At_size[1] = A_size[1];
  loop_ub = A_size[0] * A_size[1];
  for (ihi = 0; ihi < loop_ub; ihi++) {
    At_data[ihi].re = A_data[ihi];
    At_data[ihi].im = 0.0F;
  }

  *info = 0;
  anrm = 0.0F;
  loop_ub = A_size[0] * A_size[1];
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= loop_ub - 1)) {
    absxk = rt_hypotf_snf(At_data[ii].re, At_data[ii].im);
    if (rtIsNaNF(absxk)) {
      anrm = (rtNaNF);
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }

      ii++;
    }
  }

  if (rtIsInfF(anrm) || rtIsNaNF(anrm)) {
    *alpha1_size = A_size[0];
    loop_ub = A_size[0];
    for (ihi = 0; ihi < loop_ub; ihi++) {
      alpha1_data[ihi].re = (rtNaNF);
      alpha1_data[ihi].im = 0.0F;
    }

    *beta1_size = A_size[0];
    loop_ub = A_size[0];
    for (ihi = 0; ihi < loop_ub; ihi++) {
      beta1_data[ihi].re = (rtNaNF);
      beta1_data[ihi].im = 0.0F;
    }
  } else {
    int32_T b_jj;
    int32_T ctoc_tmp;
    int32_T nzcount;
    real32_T cfrom1;
    real32_T cto1;
    real32_T ctoc;
    real32_T mul;
    boolean_T guard1 = false;
    boolean_T ilascl;
    boolean_T notdone;
    ilascl = false;
    absxk = anrm;
    guard1 = false;
    if ((anrm > 0.0F) && (anrm < 9.09494702E-13F)) {
      absxk = 9.09494702E-13F;
      ilascl = true;
      guard1 = true;
    } else if (anrm > 1.09951163E+12F) {
      absxk = 1.09951163E+12F;
      ilascl = true;
      guard1 = true;
    }

    if (guard1) {
      cfromc = anrm;
      ctoc = absxk;
      notdone = true;
      while (notdone) {
        cfrom1 = cfromc * 1.97215226E-31F;
        cto1 = ctoc / 5.0706024E+30F;
        if ((cfrom1 > ctoc) && (ctoc != 0.0F)) {
          mul = 1.97215226E-31F;
          cfromc = cfrom1;
        } else if (cto1 > cfromc) {
          mul = 5.0706024E+30F;
          ctoc = cto1;
        } else {
          mul = ctoc / cfromc;
          notdone = false;
        }

        for (ihi = 0; ihi < loop_ub; ihi++) {
          At_data[ihi].re *= mul;
          At_data[ihi].im *= mul;
        }
      }
    }

    loop_ub = 1;
    ihi = A_size[0];
    if (At_size[0] <= 1) {
      ihi = 1;
    } else {
      int32_T exitg3;
      boolean_T exitg4;
      do {
        exitg3 = 0;
        i = -1;
        jcol = 0;
        notdone = false;
        ii = ihi;
        exitg1 = false;
        while ((!exitg1) && (ii > 0)) {
          nzcount = 0;
          i = ii - 1;
          jcol = ihi;
          b_jj = 1;
          exitg4 = false;
          while ((!exitg4) && (b_jj - 1 <= (uint8_T)ihi - 1)) {
            ctoc_tmp = ((b_jj - 1) * At_size[0] + ii) - 1;
            if ((At_data[ctoc_tmp].re != 0.0F) || (At_data[ctoc_tmp].im != 0.0F)
                || (ii == b_jj)) {
              if (nzcount == 0) {
                jcol = b_jj;
                nzcount = 1;
                b_jj++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              b_jj++;
            }
          }

          if (nzcount < 2) {
            notdone = true;
            exitg1 = true;
          } else {
            ii--;
          }
        }

        if (!notdone) {
          exitg3 = 2;
        } else {
          nzcount = At_size[0];
          if (i + 1 != ihi) {
            for (ii = 1; ii <= nzcount; ii++) {
              cfromc = At_data[(ii - 1) * At_size[0] + i].re;
              b_jj = (ii - 1) * At_size[0];
              ctoc_tmp = i + b_jj;
              ctoc = At_data[ctoc_tmp].im;
              At_data[ctoc_tmp] = At_data[(ihi + b_jj) - 1];
              ctoc_tmp = ((ii - 1) * At_size[0] + ihi) - 1;
              At_data[ctoc_tmp].re = cfromc;
              At_data[ctoc_tmp].im = ctoc;
            }
          }

          if (jcol != ihi) {
            nzcount = (uint8_T)ihi - 1;
            for (ii = 0; ii <= nzcount; ii++) {
              b_jj = (jcol - 1) * At_size[0] + ii;
              cfromc = At_data[b_jj].re;
              ctoc = At_data[b_jj].im;
              i = (ihi - 1) * At_size[0];
              ctoc_tmp = ii + i;
              At_data[b_jj] = At_data[ctoc_tmp];
              At_data[ctoc_tmp].re = cfromc;
              At_data[ii + i].im = ctoc;
            }
          }

          ihi--;
          if (ihi == 1) {
            exitg3 = 1;
          }
        }
      } while (exitg3 == 0);

      if (exitg3 == 1) {
      } else {
        int32_T exitg2;
        do {
          exitg2 = 0;
          i = 0;
          jcol = -1;
          notdone = false;
          b_jj = loop_ub;
          exitg1 = false;
          while ((!exitg1) && (b_jj <= ihi)) {
            nzcount = 0;
            i = ihi;
            jcol = b_jj - 1;
            ii = loop_ub;
            exitg4 = false;
            while ((!exitg4) && (ii <= ihi)) {
              ctoc_tmp = ((b_jj - 1) * At_size[0] + ii) - 1;
              if ((At_data[ctoc_tmp].re != 0.0F) || (At_data[ctoc_tmp].im !=
                   0.0F) || (ii == b_jj)) {
                if (nzcount == 0) {
                  i = ii;
                  nzcount = 1;
                  ii++;
                } else {
                  nzcount = 2;
                  exitg4 = true;
                }
              } else {
                ii++;
              }
            }

            if (nzcount < 2) {
              notdone = true;
              exitg1 = true;
            } else {
              b_jj++;
            }
          }

          if (!notdone) {
            exitg2 = 1;
          } else {
            nzcount = At_size[0];
            if (i != loop_ub) {
              for (ii = loop_ub; ii <= nzcount; ii++) {
                ctoc_tmp = (ii - 1) * At_size[0];
                b_jj = (i + ctoc_tmp) - 1;
                cfromc = At_data[b_jj].re;
                ctoc = At_data[b_jj].im;
                ctoc_tmp = (loop_ub + ctoc_tmp) - 1;
                At_data[b_jj] = At_data[ctoc_tmp];
                At_data[ctoc_tmp].re = cfromc;
                At_data[ctoc_tmp].im = ctoc;
              }
            }

            if (jcol + 1 != loop_ub) {
              nzcount = (uint8_T)ihi - 1;
              for (ii = 0; ii <= nzcount; ii++) {
                b_jj = At_size[0] * jcol + ii;
                cfromc = At_data[b_jj].re;
                ctoc = At_data[b_jj].im;
                ctoc_tmp = (loop_ub - 1) * At_size[0] + ii;
                At_data[b_jj] = At_data[ctoc_tmp];
                At_data[ctoc_tmp].re = cfromc;
                At_data[ctoc_tmp].im = ctoc;
              }
            }

            loop_ub++;
            if (loop_ub == ihi) {
              exitg2 = 1;
            }
          }
        } while (exitg2 == 0);
      }
    }

    nzcount = A_size[0];
    if ((A_size[0] <= 1) || (ihi < loop_ub + 2)) {
    } else {
      jcol = loop_ub;
      while (jcol < 2) {
        Controller_xzlartg(At_data[1], At_data[2], &cfromc, &s, &At_data[1]);
        At_data[2].re = 0.0F;
        At_data[2].im = 0.0F;
        for (jcol = 2; jcol <= nzcount; jcol++) {
          b_jj = (jcol - 1) * At_size[0];
          ctoc = At_data[2 + b_jj].im;
          cfrom1 = At_data[(jcol - 1) * At_size[0] + 2].re;
          cto1 = At_data[1 + b_jj].re;
          mul = At_data[(jcol - 1) * At_size[0] + 1].im;
          ctoc_tmp = (jcol - 1) * At_size[0] + 2;
          At_data[ctoc_tmp].re = cfrom1 * cfromc - (cto1 * s.re + mul * s.im);
          At_data[ctoc_tmp].im = ctoc * cfromc - (mul * s.re - s.im * cto1);
          ctoc_tmp = (jcol - 1) * At_size[0] + 1;
          At_data[ctoc_tmp].re = (cfrom1 * s.re - ctoc * s.im) + cto1 * cfromc;
          At_data[ctoc_tmp].im = (ctoc * s.re + cfrom1 * s.im) + mul * cfromc;
        }

        s.re = -s.re;
        s.im = -s.im;
        for (i = 1; i < 4; i++) {
          b_jj = (i + At_size[0]) - 1;
          ctoc = At_data[b_jj].im;
          cfrom1 = At_data[b_jj].re;
          ii = ((At_size[0] << 1) + i) - 1;
          cto1 = At_data[ii].re;
          mul = At_data[ii].im;
          At_data[b_jj].re = cfrom1 * cfromc - (cto1 * s.re + mul * s.im);
          At_data[b_jj].im = ctoc * cfromc - (mul * s.re - s.im * cto1);
          ctoc_tmp = ((At_size[0] << 1) + i) - 1;
          At_data[ctoc_tmp].re = (cfrom1 * s.re - ctoc * s.im) + cto1 * cfromc;
          At_data[ctoc_tmp].im = (ctoc * s.re + cfrom1 * s.im) + mul * cfromc;
        }

        jcol = 2;
      }
    }

    Controller_xzhgeqz(At_data, At_size, loop_ub, ihi, info, alpha1_data,
                       alpha1_size, beta1_data, beta1_size);
    if ((*info != 0) || (!ilascl)) {
    } else {
      notdone = true;
      while (notdone) {
        cfrom1 = absxk * 1.97215226E-31F;
        cto1 = anrm / 5.0706024E+30F;
        if ((cfrom1 > anrm) && (anrm != 0.0F)) {
          mul = 1.97215226E-31F;
          absxk = cfrom1;
        } else if (cto1 > absxk) {
          mul = 5.0706024E+30F;
          anrm = cto1;
        } else {
          mul = anrm / absxk;
          notdone = false;
        }

        loop_ub = *alpha1_size;
        for (ihi = 0; ihi < loop_ub; ihi++) {
          s = alpha1_data[ihi];
          s.re *= mul;
          s.im *= mul;
          alpha1_data[ihi] = s;
        }
      }
    }
  }
}

static boolean_T Controller_cplxpairv_l(creal32_T x_data[], const int32_T
  *x_size, real32_T tol)
{
  int32_T idx_data[3];
  int32_T iwork_data[3];
  int32_T i;
  int32_T j;
  int32_T k0;
  int32_T kEnd;
  int32_T n_0;
  int32_T ng;
  int32_T nr;
  int32_T pEnd;
  int32_T q;
  int32_T qEnd;
  real32_T absx_data[3];
  real32_T zi_data[3];
  real32_T zr_data[3];
  real32_T zr0;
  boolean_T exitg1;
  boolean_T pairable;
  n_0 = *x_size;
  for (i = 0; i < n_0; i++) {
    zr_data[i] = x_data[i].re;
  }

  if (*x_size - 1 >= 0) {
    memset(&idx_data[0], 0, (uint32_T)*x_size * sizeof(int32_T));
  }

  if (*x_size - 1 >= 1) {
    if ((zr_data[0] <= zr_data[1]) || rtIsNaNF(zr_data[1])) {
      idx_data[0] = 1;
      idx_data[1] = 2;
    } else {
      idx_data[0] = 2;
      idx_data[1] = 1;
    }
  }

  if (((uint32_T)*x_size & 1U) != 0U) {
    idx_data[*x_size - 1] = *x_size;
  }

  i = 2;
  while (i < *x_size) {
    j = 1;
    pEnd = 3;
    while (pEnd < *x_size + 1) {
      i = j - 1;
      q = pEnd - 1;
      qEnd = j + 4;
      if (j + 4 > *x_size + 1) {
        qEnd = *x_size + 1;
      }

      nr = 0;
      kEnd = qEnd - j;
      while (nr + 1 <= kEnd) {
        zr0 = zr_data[idx_data[q] - 1];
        if ((zr_data[idx_data[i] - 1] <= zr0) || rtIsNaNF(zr0)) {
          iwork_data[nr] = idx_data[i];
          i++;
          if (i + 1 == pEnd) {
            while (q + 1 < qEnd) {
              nr++;
              iwork_data[nr] = idx_data[q];
              q++;
            }
          }
        } else {
          iwork_data[nr] = idx_data[q];
          q++;
          if (q + 1 == qEnd) {
            while (i + 1 < pEnd) {
              nr++;
              iwork_data[nr] = idx_data[i];
              i++;
            }
          }
        }

        nr++;
      }

      for (pEnd = 0; pEnd < kEnd; pEnd++) {
        idx_data[(j + pEnd) - 1] = iwork_data[pEnd];
      }

      j = qEnd;
      pEnd = qEnd + 2;
    }

    i = 4;
  }

  nr = 0;
  n_0 = 0;
  k0 = *x_size - 1;
  for (i = 0; i <= k0; i++) {
    real32_T absxk;
    real32_T absxk_tmp;
    ng = idx_data[i];
    zr0 = x_data[ng - 1].re;
    absxk_tmp = x_data[ng - 1].im;
    absxk = rt_hypotf_snf(zr0, absxk_tmp);
    if (fabsf(x_data[ng - 1].im) <= tol * absxk) {
      nr++;
      zr_data[*x_size - nr] = zr0;
      zi_data[*x_size - nr] = 0.0F;
      absx_data[i] = absxk;
    } else {
      n_0++;
      zr_data[n_0 - 1] = zr0;
      zi_data[n_0 - 1] = absxk_tmp;
      absx_data[n_0 - 1] = absxk;
    }
  }

  for (pEnd = 0; pEnd < nr; pEnd++) {
    i = n_0 + pEnd;
    x_data[i].re = zr_data[(*x_size - pEnd) - 1];
    x_data[i].im = 0.0F;
  }

  i = n_0 - 1;
  for (nr = 0; nr <= i; nr++) {
    x_data[nr].re = zr_data[nr];
    x_data[nr].im = zi_data[nr];
  }

  pairable = ((n_0 & 1) != 1);
  if (pairable) {
    nr = 0;
    exitg1 = false;
    while ((!exitg1) && (nr + 1 <= n_0)) {
      if (zr_data[nr + 1] - zr_data[nr] > tol * absx_data[nr]) {
        pairable = false;
        exitg1 = true;
      } else {
        nr += 2;
      }
    }
  }

  if (pairable) {
    nr = 0;
    exitg1 = false;
    while ((!exitg1) && (nr + 1 < n_0)) {
      int32_T n;
      uint32_T pairable_tmp;
      zr0 = zr_data[nr];
      k0 = nr + 1;
      n = nr - 1;
      while ((nr + 1 < n_0) && (zr_data[nr + 1] - zr0 <= tol * absx_data[nr])) {
        nr++;
      }

      ng = (nr - k0) + 2;
      pairable_tmp = (uint32_T)ng & 1U;
      pairable = (pairable_tmp == 0U);
      if (!pairable) {
        exitg1 = true;
      } else {
        boolean_T exitg2;
        for (i = 1; i <= ng - 1; i += 2) {
          zr0 = zi_data[n + 2];
          if ((zi_data[n + 1] <= zr0) || rtIsNaNF(zr0)) {
            idx_data[n + 1] = 1;
            idx_data[n + 2] = 2;
          } else {
            idx_data[n + 1] = 2;
            idx_data[n + 2] = 1;
          }
        }

        if (pairable_tmp != 0U) {
          idx_data[n + ng] = ng;
        }

        i = 2;
        while (i < ng) {
          j = 1;
          pEnd = 3;
          while (pEnd < ng + 1) {
            int32_T k;
            i = j;
            q = pEnd;
            qEnd = j + 4;
            if (j + 4 > ng + 1) {
              qEnd = ng + 1;
            }

            k = 0;
            kEnd = qEnd - j;
            while (k + 1 <= kEnd) {
              int32_T b_tmp_tmp;
              int32_T i_tmp;
              b_tmp_tmp = idx_data[n + q];
              zr0 = zi_data[b_tmp_tmp + n];
              i_tmp = idx_data[n + i];
              if ((zi_data[i_tmp + n] <= zr0) || rtIsNaNF(zr0)) {
                iwork_data[k] = i_tmp;
                i++;
                if (i == pEnd) {
                  while (q < qEnd) {
                    k++;
                    iwork_data[k] = idx_data[n + q];
                    q++;
                  }
                }
              } else {
                iwork_data[k] = b_tmp_tmp;
                q++;
                if (q == qEnd) {
                  while (i < pEnd) {
                    k++;
                    iwork_data[k] = idx_data[n + i];
                    i++;
                  }
                }
              }

              k++;
            }

            for (pEnd = 0; pEnd < kEnd; pEnd++) {
              idx_data[(n + j) + pEnd] = iwork_data[pEnd];
            }

            j = qEnd;
            pEnd = qEnd + 2;
          }

          i = 4;
        }

        j = k0;
        exitg2 = false;
        while ((!exitg2) && (j <= nr + 1)) {
          i = idx_data[j - 1] + n;
          if (fabsf(zi_data[idx_data[(nr - j) + k0] + n] + zi_data[i]) > tol *
              absx_data[i]) {
            pairable = false;
            exitg2 = true;
          } else {
            j++;
          }
        }

        if (!pairable) {
          exitg1 = true;
        } else {
          ng = (uint8_T)(ng >> 1) - 1;
          for (k0 = 0; k0 <= ng; k0++) {
            i = n + idx_data[nr];
            x_data[n + 1].re = zr_data[i];
            x_data[n + 1].im = -zi_data[i];
            x_data[n + 2].re = zr_data[i];
            x_data[n + 2].im = zi_data[i];
          }

          nr++;
        }
      }
    }
  }

  return pairable;
}

static void Controller_roots(const real32_T c[4], creal32_T r_data[], int32_T
  *r_size)
{
  creal32_T b_vwork_data[3];
  creal32_T beta1_data[3];
  creal32_T eiga_data[3];
  int32_T a_size[2];
  int32_T dim;
  int32_T eiga_size;
  int32_T j;
  int32_T k;
  int32_T k1;
  int32_T k2;
  real32_T a_data[9];
  real32_T ctmp[4];
  r_data[0].re = 0.0F;
  r_data[0].im = 0.0F;
  r_data[1].re = 0.0F;
  r_data[1].im = 0.0F;
  r_data[2].re = 0.0F;
  r_data[2].im = 0.0F;
  k1 = 1;
  while ((k1 <= 4) && (!(c[k1 - 1] != 0.0F))) {
    k1++;
  }

  k2 = 4;
  while ((k2 >= k1) && (!(c[k2 - 1] != 0.0F))) {
    k2--;
  }

  if (k1 < k2) {
    int32_T companDim;
    boolean_T exitg1;
    companDim = k2 - k1;
    exitg1 = false;
    while ((!exitg1) && (companDim > 0)) {
      boolean_T exitg2;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j + 1 <= companDim)) {
        ctmp[j] = c[k1 + j] / c[k1 - 1];
        if (rtIsInfF(fabsf(ctmp[j]))) {
          exitg2 = true;
        } else {
          j++;
        }
      }

      if (j + 1 > companDim) {
        exitg1 = true;
      } else {
        k1++;
        companDim--;
      }
    }

    if (companDim < 1) {
      if (4 - k2 < 1) {
        j = 0;
      } else {
        j = 4 - k2;
      }

      if (j - 1 >= 0) {
        memset(&r_data[0], 0, (uint32_T)j * sizeof(creal32_T));
      }

      *r_size = j;
    } else {
      a_size[0] = companDim;
      a_size[1] = companDim;
      memset(&a_data[0], 0, (uint32_T)(companDim * companDim) * sizeof(real32_T));
      k1 = companDim - 2;
      for (dim = 0; dim <= k1; dim++) {
        j = companDim * dim;
        a_data[j] = -ctmp[dim];
        a_data[(dim + j) + 1] = 1.0F;
      }

      a_data[companDim * (companDim - 1)] = -ctmp[companDim - 1];
      k1 = 3 - k2;
      if (3 - k2 >= 0) {
        memset(&r_data[0], 0, (uint32_T)((3 - k2) + 1) * sizeof(creal32_T));
      }

      if (companDim == 1) {
        for (k1 = 0; k1 < companDim; k1++) {
          eiga_data[k1].re = a_data[k1];
          eiga_data[k1].im = 0.0F;
        }
      } else {
        int32_T d_tmp;
        boolean_T paired;
        boolean_T paired_0;
        Controller_xzgeev(a_data, a_size, &k1, eiga_data, &eiga_size, beta1_data,
                          &j);
        k1 = companDim - 1;
        for (j = 0; j <= k1; j++) {
          creal32_T eiga;
          real32_T ar;
          real32_T bi;
          real32_T br;
          eiga = eiga_data[j];
          ar = eiga.re;
          br = beta1_data[j].re;
          bi = beta1_data[j].im;
          if (bi == 0.0F) {
            if (eiga.im == 0.0F) {
              eiga.re /= br;
              eiga.im = 0.0F;
            } else if (eiga.re == 0.0F) {
              eiga.re = 0.0F;
              eiga.im /= br;
            } else {
              eiga.re /= br;
              eiga.im /= br;
            }
          } else if (br == 0.0F) {
            if (eiga.re == 0.0F) {
              eiga.re = eiga.im / bi;
              eiga.im = 0.0F;
            } else if (eiga.im == 0.0F) {
              eiga.re = 0.0F;
              eiga.im = -(ar / bi);
            } else {
              eiga.re = eiga.im / bi;
              eiga.im = -(ar / bi);
            }
          } else {
            real32_T bim;
            real32_T brm;
            brm = fabsf(br);
            bim = fabsf(bi);
            if (brm > bim) {
              brm = bi / br;
              bi = brm * bi + br;
              eiga.re = (brm * eiga.im + eiga.re) / bi;
              eiga.im = (eiga.im - brm * ar) / bi;
            } else if (bim == brm) {
              br = br > 0.0F ? 0.5F : -0.5F;
              bi = bi > 0.0F ? 0.5F : -0.5F;
              eiga.re = (eiga.re * br + eiga.im * bi) / brm;
              eiga.im = (eiga.im * br - ar * bi) / brm;
            } else {
              brm = br / bi;
              bi += brm * br;
              eiga.re = (brm * eiga.re + eiga.im) / bi;
              eiga.im = (brm * eiga.im - ar) / bi;
            }
          }

          eiga_data[j] = eiga;
        }

        if (eiga_size - 1 >= 0) {
          memcpy(&beta1_data[0], &eiga_data[0], (uint32_T)eiga_size * sizeof
                 (creal32_T));
        }

        dim = 2;
        if (eiga_size != 1) {
          dim = 1;
        }

        paired = true;
        if (eiga_size == 1) {
          paired = Controller_cplxpairv_l(beta1_data, &eiga_size, 3.81469727E-6F);
        } else {
          if (dim <= 1) {
            k1 = eiga_size;
          } else {
            k1 = 1;
          }

          if (k1 != 1) {
            paired = Controller_cplxpairv_l(beta1_data, &eiga_size,
              3.81469727E-6F);
          } else {
            if (dim <= 1) {
              k1 = eiga_size;
            } else {
              k1 = 1;
            }

            j = 1;
            dim -= 2;
            for (k = 0; k <= dim; k++) {
              j *= eiga_size;
            }

            for (k = 0; k < j; k++) {
              d_tmp = k1 - 1;
              for (dim = 0; dim <= d_tmp; dim++) {
                b_vwork_data[dim] = beta1_data[dim * j + k];
              }

              paired_0 = Controller_cplxpairv_l(b_vwork_data, &k1,
                3.81469727E-6F);
              for (dim = 0; dim <= d_tmp; dim++) {
                beta1_data[k + dim * j] = b_vwork_data[dim];
              }

              if (paired && paired_0) {
              } else {
                paired = false;
              }
            }
          }
        }

        if (!paired) {
          dim = 2;
          if (eiga_size != 1) {
            dim = 1;
          }

          paired = true;
          if (eiga_size == 1) {
            paired = Controller_cplxpairv_l(beta1_data, &eiga_size,
              3.05175781E-5F);
          } else {
            if (dim <= 1) {
              k1 = eiga_size;
            } else {
              k1 = 1;
            }

            if (k1 != 1) {
              paired = Controller_cplxpairv_l(beta1_data, &eiga_size,
                3.05175781E-5F);
            } else {
              if (dim <= 1) {
                k1 = eiga_size;
              } else {
                k1 = 1;
              }

              j = 1;
              dim -= 2;
              for (k = 0; k <= dim; k++) {
                j *= eiga_size;
              }

              for (k = 0; k < j; k++) {
                d_tmp = k1 - 1;
                for (dim = 0; dim <= d_tmp; dim++) {
                  b_vwork_data[dim] = beta1_data[dim * j + k];
                }

                paired_0 = Controller_cplxpairv_l(b_vwork_data, &k1,
                  3.05175781E-5F);
                for (dim = 0; dim <= d_tmp; dim++) {
                  beta1_data[k + dim * j] = b_vwork_data[dim];
                }

                if (paired && paired_0) {
                } else {
                  paired = false;
                }
              }
            }
          }
        }

        if (paired) {
          k1 = companDim - 1;
          for (j = 0; j <= k1; j++) {
            eiga_data[j] = beta1_data[(companDim - j) - 1];
          }
        }
      }

      k1 = companDim - 1;
      for (j = 0; j <= k1; j++) {
        r_data[(j - k2) + 4] = eiga_data[j];
      }

      *r_size = (companDim - k2) + 4;
    }
  } else {
    if (4 - k2 < 1) {
      j = 0;
    } else {
      j = 4 - k2;
    }

    if (j - 1 >= 0) {
      memset(&r_data[0], 0, (uint32_T)j * sizeof(creal32_T));
    }

    *r_size = j;
  }
}

static void generateSegmentVelocityProfile_(const real32_T sPoints[8], real32_T
  vMax, const real32_T vPoints[8], const real32_T tPoints[8], real32_T x,
  real32_T *varargout_1, real32_T *varargout_2)
{
  creal32_T r_data[3];
  int32_T intervalIndex_size[2];
  int32_T b_exponent;
  int32_T b_ii;
  int32_T idx;
  int32_T ii_data;
  int32_T ii_size_idx_1;
  real32_T tmp[4];
  boolean_T x_0[7];
  boolean_T intervalIndex_data;

  /* ------------------------------------------------------------------ */
  /* querySpeedByDistance Find the speed given the distance from */
  /*    the starting point */
  /*  Determine which interval the specified distance is located in */
  if (x == sPoints[7]) {
    ii_size_idx_1 = 1;
    ii_data = 7;
  } else {
    boolean_T exitg1;
    for (idx = 0; idx < 7; idx++) {
      x_0[idx] = (sPoints[idx + 1] > x);
    }

    idx = 0;
    ii_size_idx_1 = 1;
    b_ii = 1;
    exitg1 = false;
    while ((!exitg1) && (b_ii - 1 < 7)) {
      if (x_0[b_ii - 1]) {
        idx = 1;
        ii_data = b_ii;
        exitg1 = true;
      } else {
        b_ii++;
      }
    }

    if (idx == 0) {
      ii_size_idx_1 = 0;
    }
  }

  /*  See Algorithm section in help for the definition of each */
  /*  intervals */
  intervalIndex_size[0] = 1;
  intervalIndex_size[1] = ii_size_idx_1;
  if (ii_size_idx_1 - 1 >= 0) {
    intervalIndex_data = (ii_data == 1);
  }

  if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size)) {
    /*  Accelerate from starting velocity to */
    /*  reach maximum longitudinal acceleration */
    /* ------------------------------------------------------------------ */
    /* computeTimeFromDistance Solve a cubic equation to get time */
    /*    p(1)*t^3+p(2)*t^2+p(3)*t+p(4) = 0 */
    tmp[0] = 1.66666663F;
    tmp[1] = 0.0F;
    tmp[2] = vPoints[0];
    tmp[3] = -x;
    Controller_roots(tmp, r_data, &ii_data);
    *varargout_2 = tPoints[1];

    /*  Robust numerical comparison to find the smallest positive */
    /*  real root to avoid limitation in codegen */
    idx = ii_data - 1;
    for (b_ii = 0; b_ii <= idx; b_ii++) {
      real32_T r_re;
      r_re = r_data[b_ii].re;
      if ((r_re >= 0.0F) && (r_re < *varargout_2)) {
        real32_T r;
        if (r_re < 2.3509887E-38F) {
          r = 1.4013E-45F;
        } else {
          frexpf(r_re, &b_exponent);
          r = ldexpf(1.0F, b_exponent - 24);
        }

        if (fabsf(r_data[b_ii].im) <= sqrtf(r)) {
          *varargout_2 = r_re;
        }
      }
    }

    *varargout_1 = *varargout_2 * *varargout_2 * 5.0F + vPoints[0];
  } else {
    intervalIndex_size[0] = 1;
    intervalIndex_size[1] = ii_size_idx_1;
    if (ii_size_idx_1 - 1 >= 0) {
      intervalIndex_data = (ii_data == 2);
    }

    if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size)) {
      /*  Accelerate at maximum longitudinal acceleration */
      *varargout_1 = sqrtf((x - sPoints[1]) * 2.0F * 20.0F + vPoints[1] *
                           vPoints[1]);
      *varargout_2 = (*varargout_1 - vPoints[1]) / 20.0F + tPoints[1];
    } else {
      intervalIndex_size[0] = 1;
      intervalIndex_size[1] = ii_size_idx_1;
      if (ii_size_idx_1 - 1 >= 0) {
        intervalIndex_data = (ii_data == 3);
      }

      if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size)) {
        /*  Decrease acceleration to zero and reach the maximum speed */
        /* ------------------------------------------------------------------ */
        /* computeTimeFromDistance Solve a cubic equation to get time */
        /*    p(1)*t^3+p(2)*t^2+p(3)*t+p(4) = 0 */
        tmp[0] = 1.66666663F;
        tmp[1] = 0.0F;
        tmp[2] = -vMax;
        tmp[3] = sPoints[3] - x;
        Controller_roots(tmp, r_data, &ii_data);
        *varargout_2 = tPoints[3];

        /*  Robust numerical comparison to find the smallest positive */
        /*  real root to avoid limitation in codegen */
        idx = ii_data - 1;
        for (b_ii = 0; b_ii <= idx; b_ii++) {
          real32_T r_re;
          r_re = r_data[b_ii].re;
          if ((r_re >= 0.0F) && (r_re < *varargout_2)) {
            real32_T r;
            if (r_re < 2.3509887E-38F) {
              r = 1.4013E-45F;
            } else {
              frexpf(r_re, &b_exponent);
              r = ldexpf(1.0F, b_exponent - 24);
            }

            if (fabsf(r_data[b_ii].im) <= sqrtf(r)) {
              *varargout_2 = r_re;
            }
          }
        }

        *varargout_1 = vPoints[3] - *varargout_2 * *varargout_2 * 5.0F;
        *varargout_2 = tPoints[3] - *varargout_2;
      } else {
        intervalIndex_size[0] = 1;
        intervalIndex_size[1] = ii_size_idx_1;
        if (ii_size_idx_1 - 1 >= 0) {
          intervalIndex_data = (ii_data == 4);
        }

        if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size)) {
          /*  Keep constant velocity */
          *varargout_1 = vMax;
          *varargout_2 = (x - sPoints[3]) / vMax + tPoints[3];
        } else {
          intervalIndex_size[0] = 1;
          intervalIndex_size[1] = ii_size_idx_1;
          if (ii_size_idx_1 - 1 >= 0) {
            intervalIndex_data = (ii_data == 5);
          }

          if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size)) {
            /*  Decelerate to reach maximum longitudinal deceleration */
            /* ------------------------------------------------------------------ */
            /* computeTimeFromDistance Solve a cubic equation to get time */
            /*    p(1)*t^3+p(2)*t^2+p(3)*t+p(4) = 0 */
            tmp[0] = 1.66666663F;
            tmp[1] = 0.0F;
            tmp[2] = -vMax;
            tmp[3] = x - sPoints[4];
            Controller_roots(tmp, r_data, &ii_data);
            *varargout_2 = tPoints[5];

            /*  Robust numerical comparison to find the smallest positive */
            /*  real root to avoid limitation in codegen */
            idx = ii_data - 1;
            for (b_ii = 0; b_ii <= idx; b_ii++) {
              real32_T r_re;
              r_re = r_data[b_ii].re;
              if ((r_re >= 0.0F) && (r_re < *varargout_2)) {
                real32_T r;
                if (r_re < 2.3509887E-38F) {
                  r = 1.4013E-45F;
                } else {
                  frexpf(r_re, &b_exponent);
                  r = ldexpf(1.0F, b_exponent - 24);
                }

                if (fabsf(r_data[b_ii].im) <= sqrtf(r)) {
                  *varargout_2 = r_re;
                }
              }
            }

            *varargout_1 = vPoints[4] - *varargout_2 * *varargout_2 * 5.0F;
            *varargout_2 += tPoints[4];
          } else {
            intervalIndex_size[0] = 1;
            intervalIndex_size[1] = ii_size_idx_1;
            if (ii_size_idx_1 - 1 >= 0) {
              intervalIndex_data = (ii_data == 6);
            }

            if (Controller_ifWhileCond(&intervalIndex_data, intervalIndex_size))
            {
              /*  Decelerate at maximum longitudinal deceleration */
              *varargout_1 = sqrtf((x - sPoints[5]) * -2.0F * 20.0F + vPoints[5]
                                   * vPoints[5]);
              *varargout_2 = (vPoints[5] - *varargout_1) / 20.0F + tPoints[5];
            } else {
              /* 7 Decrease the deceleration to zero to reach ending velocity */
              /* ------------------------------------------------------------------ */
              /* computeTimeFromDistance Solve a cubic equation to get time */
              /*    p(1)*t^3+p(2)*t^2+p(3)*t+p(4) = 0 */
              tmp[0] = 1.66666663F;
              tmp[1] = 0.0F;
              tmp[2] = vPoints[7];
              tmp[3] = -(sPoints[7] - x);
              Controller_roots(tmp, r_data, &ii_data);
              *varargout_2 = tPoints[7];

              /*  Robust numerical comparison to find the smallest positive */
              /*  real root to avoid limitation in codegen */
              idx = ii_data - 1;
              for (b_ii = 0; b_ii <= idx; b_ii++) {
                real32_T r_re;
                r_re = r_data[b_ii].re;
                if ((r_re >= 0.0F) && (r_re < *varargout_2)) {
                  real32_T r;
                  if (r_re < 2.3509887E-38F) {
                    r = 1.4013E-45F;
                  } else {
                    frexpf(r_re, &b_exponent);
                    r = ldexpf(1.0F, b_exponent - 24);
                  }

                  if (fabsf(r_data[b_ii].im) <= sqrtf(r)) {
                    *varargout_2 = r_re;
                  }
                }
              }

              *varargout_1 = *varargout_2 * *varargout_2 * 5.0F + vPoints[7];
              *varargout_2 = tPoints[7] - *varargout_2;
            }
          }
        }
      }
    }
  }
}

static void Controller_arrayfun(const real32_T fun_workspace_sPoints[8],
  real32_T fun_workspace_vMax, const real32_T fun_workspace_vPoints[8], const
  real32_T fun_workspace_tPoints[8], const real32_T varargin_1_data[], const
  int32_T *varargin_1_size, real32_T varargout_1_data[], int32_T
  *varargout_1_size, real32_T varargout_2_data[], int32_T *varargout_2_size)
{
  int32_T b;
  int32_T b_k;
  *varargout_1_size = *varargin_1_size;
  *varargout_2_size = *varargin_1_size;
  b = *varargin_1_size - 1;
  for (b_k = 0; b_k <= b; b_k++) {
    generateSegmentVelocityProfile_(fun_workspace_sPoints, fun_workspace_vMax,
      fun_workspace_vPoints, fun_workspace_tPoints, varargin_1_data[b_k],
      &varargout_1_data[b_k], &varargout_2_data[b_k]);
  }
}

static void Contr_VelocityProfiler_stepImpl(VelocityProfiler_Controller_T *obj,
  const real32_T directions[50], const real32_T cumLengths[50], const real32_T
  curvatures[50], real32_T startVelocity, real32_T endVelocity, real32_T
  varargout_1[50])
{
  real_T segEndIndex_data[50];
  real_T segStartIndex_data[50];
  int32_T g;
  int32_T i;
  int32_T segStartIndex_size;
  int32_T y_size;
  real32_T b_times_data[50];
  real32_T cumLengths_data[50];
  real32_T segEndSpeeds_data[50];
  real32_T segStartSpeeds_data[50];
  real32_T y_data[50];
  real32_T segStartSpeeds[8];
  real32_T x[8];
  real32_T x_0[8];
  real32_T b_sAccel[3];
  real32_T b_sDecel[3];
  boolean_T guard1 = false;
  boolean_T solFound;

  /* ------------------------------------------------------------------ */
  /* stepImpl Implement the main algorithm */
  /*  Run-time check of input values */
  /* ------------------------------------------------------------------ */
  /*  Check consistency between bound velocities and directions */
  /*  Check if the path is new. If not, use the previous output. */
  /* ------------------------------------------------------------------ */
  guard1 = false;
  if (Controller_isequal(cumLengths, obj->LastCumLengths)) {
    solFound = false;
    if (startVelocity == obj->LastStartVelocity) {
      solFound = true;
    }

    if (solFound) {
      solFound = false;
      if (endVelocity == obj->LastEndVelocity) {
        solFound = true;
      }

      if (solFound && Controller_isequal(directions, obj->LastDirections) &&
          Controller_isequal(curvatures, obj->LastCurvatures)) {
        memcpy(&varargout_1[0], &obj->LastVelocities[0], 50U * sizeof(real32_T));
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    int32_T loop_ub_tmp;

    /*  Call the internal function */
    /* generateVelocityProfile Generate a velocity profile for a reference path */
    /*  */
    /*    This function is for internal use only. It may be removed in the future. */
    /*  */
    /*    [velocities, times]=driving.internal.planning.generateVelocityProfile(directions, ... */
    /*    cumLengths, curvatures, startVelocity, endVelocity) generates a velocity */
    /*    profile, velocities, given the cumulative path length, cumLengths, the  */
    /*    path curvatures, curvatures, the driving directions at each point of  */
    /*    the path, directions, the start velocity, startVelocity, and the end  */
    /*    velocity, endVelocity. */
    /*     */
    /*    [...] = driving.internal.planning.generateVelocityProfile(..., Name, Value)    */
    /*    specifies additional name-value pair arguments as described below: */
    /*  */
    /*    'MaxSpeed'                  - A positive number specifying the maximum */
    /*                                  allowable speed on the reference path.  */
    /*                 */
    /*                                  Default: 1 (m/s) */
    /*   */
    /*    'MaxLateralAccel'           - A positive scalar specifying the maximum */
    /*                                  allowable lateral acceleration. */
    /*     */
    /*                                  Default: 1 (m/s^2) */
    /*  */
    /*    'MaxLongitudinalAccel'      - A positive scalar specifying the maximum */
    /*                                  allowable longitudinal acceleration. */
    /*  */
    /*                                  Default: 3 (m/s^2) */
    /*  */
    /*    'MaxLongitudinalDecel'      - A positive scalar specifying the maximum */
    /*                                  allowable longitudinal deceleration. */
    /*  */
    /*                                  Default: 6 (m/s^2) */
    /*  */
    /*    'MaxLongitudinalJerk'       - A positive scalar specifying the maximum */
    /*                                  allowable longitudinal jerk. The jerk */
    /*                                  of the generated velocity profile is  */
    /*                                  constrained in the range of  */
    /*                                  [-MaxLongitudinalJerk, MaxLongitudinalJerk]. */
    /*  */
    /*                                  Default: 1 (m/s^3) */
    /*  */
    /*    Algorithms */
    /*    ---------- */
    /*    - Generally the generated velocity profile contains seven intervals: */
    /*      1: Accelerate from the starting velocity to reach the maximum */
    /*         longitudinal acceleration. The initial acceleration is assumed zero */
    /*      2: Continue accelerating at the maximum longitudinal acceleration */
    /*      3: Decrease the acceleration to zero and reach the maximum speed */
    /*      4: Keep constant velocity */
    /*      5: Decelerate to reach the maximum longitudinal deceleration */
    /*      6: Continue decelerating at the maximum longitudinal deceleration */
    /*      7: Decrease the deceleration to zero and reach the ending velocity */
    /*    - When the maximum velocity is too close to the starting and ending */
    /*      velocities, the maximum longitudinal acceleration/deceleration may */
    /*      not be reached in intervals 1 and 5, in this case intervals 2 and 6 */
    /*      do not exist. */
    /*    - When the path length is too short to reach the maximum speed, a new */
    /*      maximum speed is calculate to satisfy the path length constraint. */
    /*    - In all the intervals, the longitudinal jerk is constrained in the */
    /*      range of [-MaxLongitudinalJerk, MaxLongitudinalJerk]. */
    /*  */
    /*    Class Support */
    /*    ------------- */
    /*    The inputs can be single or double. The output velocity profile have   */
    /*    the same data type as the first input. */
    /*  */
    /*    Example: Generate a velocity profile for a smooth path  */
    /*    -------------------------------------------------------- */
    /*    % Load a costmap for a parking lot */
    /*    data = load('parkingLotCostmap.mat'); */
    /*    parkMap = data.parkingLotCostmap; */
    /*    */
    /*    % Define a start and goal pose as [x,y,theta] */
    /*    startPose = [4,  4, 90]; % [meters, meters, degrees] */
    /*    goalPose  = [30, 13, 0]; */
    /*    */
    /*    % Create an RRT planner */
    /*    planner = pathPlannerRRT(parkMap); */
    /*    */
    /*    % Plan a route from start to goal pose */
    /*    refPath = plan(planner, startPose, goalPose); */
    /*    */
    /*    % Retrieve transition poses and directions */
    /*    [refPoses, refDirections] = interpolate(refPath); */
    /*  */
    /*    % Return 1000 discretized poses along the smooth path  */
    /*    [~, directions, cumLengths, curvatures] = smoothPathSpline(refPoses, refDirections, 1000); */
    /*  */
    /*    % Generate velocity profile for the smooth path. The vehicle starts and */
    /*    % end with zero velocity */
    /*    velocities = driving.internal.planning.generateVelocityProfiler(directions, cumLengths, curvatures, 0, 0); */
    /*    */
    /*    % Plot velocity profile along the path */
    /*    plot(cumLengths, velocities) */
    /*    Copyright 2019 The MathWorks, Inc. */
    /*  Parse parameters */
    /*  NOTE: input validation is not performed here */
    /* -------------------------------------------------------------------------- */
    /*  Parse parameters */
    /* -------------------------------------------------------------------------- */
    /* parseInputs Parse parameters for both simulation and codegen  */
    /*  Define default values */
    /*  Parse inputs */
    /*  Code generation */
    /* -------------------------------------------------------------------------- */
    /* parseOptionalInputsCodegen Parse parameters for codegen workflow */
    /*  Extract parameters */
    /*  Divide the path to segments based on driving direction */
    getSegmentBoundaryPointIndex_l(directions, segStartIndex_data,
      &segStartIndex_size, segEndIndex_data, &i);

    /*  Set the boundary speeds of all the segments */
    /*  Use absolute values of boundary velocities in computation */
    segStartSpeeds_data[0] = fabsf(startVelocity);
    loop_ub_tmp = segStartIndex_size - 1;
    if ((segStartIndex_size - 1) - 1 >= 0) {
      memset(&segStartSpeeds_data[1], 0, (uint32_T)(segStartIndex_size - 1) *
             sizeof(real32_T));
    }

    if ((segStartIndex_size - 1) - 1 >= 0) {
      memset(&segEndSpeeds_data[0], 0, (uint32_T)(segStartIndex_size - 1) *
             sizeof(real32_T));
    }

    segEndSpeeds_data[segStartIndex_size - 1] = fabsf(endVelocity);

    /*  Output datatype is the same as the first input argument */
    memset(&obj->LastVelocities[0], 0, 50U * sizeof(real32_T));

    /*  Generate velocity profile for each segment of path */
    for (i = 0; i <= loop_ub_tmp; i++) {
      int32_T c;
      int32_T h_tmp;
      int32_T loop_ub;
      int32_T q;
      real32_T deltaS1;
      real32_T deltaS1_0;
      real32_T deltaS2;
      real32_T deltaS2_0;
      real32_T deltaS3;
      real32_T deltaS3_0;
      real32_T deltaT1;
      real32_T deltaT1_0;
      real32_T deltaT2;
      real32_T deltaT2_0;
      real32_T sConst;
      real32_T sol;
      real32_T tmp;
      real32_T tmp_0;
      real32_T v1;
      real32_T v1_0;
      real32_T v2;
      real32_T v2_0;
      if (segStartIndex_data[i] > segEndIndex_data[i]) {
        h_tmp = 0;
        g = -1;
        loop_ub = 0;
        segStartIndex_size = 0;
        q = 0;
      } else {
        h_tmp = (int32_T)segStartIndex_data[i] - 1;
        segStartIndex_size = (int32_T)segEndIndex_data[i];
        g = segStartIndex_size - 1;
        loop_ub = h_tmp;
        q = h_tmp;
      }

      /*  Assign to the corresponding segment */
      /* ------------------------------------------------------------------ */
      /* generateSegmentVelocityProfile Generate a velcoity profile for */
      /*    a path segment */
      /*  Maximum speed needs to satisfy lateral acceleration constraint */
      segStartIndex_size -= loop_ub;
      y_size = segStartIndex_size;
      c = (uint8_T)segStartIndex_size - 1;
      for (segStartIndex_size = 0; segStartIndex_size <= c; segStartIndex_size++)
      {
        y_data[segStartIndex_size] = fabsf(curvatures[loop_ub +
          segStartIndex_size]);
      }

      for (segStartIndex_size = 0; segStartIndex_size < y_size;
           segStartIndex_size++) {
        y_data[segStartIndex_size] = 20.0F / y_data[segStartIndex_size];
      }

      loop_ub = y_size - 1;
      for (segStartIndex_size = 0; segStartIndex_size <= loop_ub;
           segStartIndex_size++) {
        y_data[segStartIndex_size] = sqrtf(y_data[segStartIndex_size]);
      }

      sol = fminf((real32_T)obj->MaxSpeed, Controller_minimum(y_data, &y_size));

      /*  Run-time check of velocity inputs */
      /*  In reverse motion, flip acceleration and deceleration */
      /*  Calculate the minimum distance that is kinematically feasible */
      /*  If above the minimum distance, check if vMax can be reached */
      /*  within the distance. If not, find a new maximum speed. */
      tmp = segStartSpeeds_data[i];
      tmp_0 = segEndSpeeds_data[i];
      getNonConstSpeedIntervalDistanc(tmp, sol, b_sAccel);
      getNonConstSpeedIntervalDista_l(tmp_0, sol, b_sDecel);
      if (Controller_sum(b_sAccel) + Controller_sum(b_sDecel) > cumLengths[g]) {
        /* ------------------------------------------------------------------ */
        /* calculateNewMaximumSpeed For given distance, calculate the */
        /*    maximum possible speed */
        /*  Consider the case where all the seven intervals are valid by */
        /*  solving a quadratic equation: A*vMax^2 + B*vMax + C = 0 */
        sol = (sqrtf(4.0F - (((20.0F * tmp + 20.0F * tmp_0) / 20.0F - (tmp * tmp
                   / 40.0F + tmp_0 * tmp_0 / 40.0F)) - cumLengths[g]) * 0.2F) -
               2.0F) / 0.1F;

        /*  Positive solution */
        /*  Check if the solution is valid */
        if ((sol >= tmp + 40.0F) && (sol >= tmp_0 + 40.0F)) {
        } else {
          /*  Search for a new speed starting from max(vEnd, vStart) as */
          /*  analytic solution is intractable */
          sol = fmaxf(tmp_0, tmp);

          /*  percentage */
          /*  0.1 m/s */
          solFound = false;
          while (!solFound) {
            sol += 0.1F;
            getNonConstSpeedIntervalDistanc(tmp, sol, b_sAccel);
            getNonConstSpeedIntervalDista_l(tmp_0, sol, b_sDecel);
            solFound = (Controller_sum(b_sAccel) + Controller_sum(b_sDecel) >
                        cumLengths[g]);
          }

          sol -= 0.1F;
        }
      }

      /*  Given the updated vMax, continue with the normal computation */
      /*  Distance in intervals 1-3: acceleration intervals */
      /* ------------------------------------------------------------------ */
      /* getNonConstSpeedIntervalDistance Calculate the travelled distance */
      /*    during the acceleration or the deceleration interval, i.e., intervals */
      /*    1-3 or 5-7, given the starting or the ending velocity, vBound, */
      /*    the maximum speed, vMax and the maximum acceleration or */
      /*    deceleration, aMax. Before calculating the distance, it needs */
      /*    to determine if vMax can be reached without reaching aMax, */
      /*    i.e., if interval 2 or 6 exists. intervalFlag is used as a flag to */
      /*    distinguish the acceleration interval from the deceleration one. */
      /*  A speed threshold below which aMax is not reached */
      if (segStartSpeeds_data[i] + 40.0F <= sol) {
        /*  interval 2 or 6 needed */
        deltaT1 = 2.0F;
        deltaT2 = (sol - segStartSpeeds_data[i]) / 20.0F - 2.0F;
        x[3] = 2.0F;
        v1 = tmp + 20.0F;
        v2 = (tmp + 20.0F) + deltaT2 * 20.0F;
        deltaS1 = 2.0F * tmp + 13.333333F;
        deltaS2 = ((tmp + 20.0F) + v2) * (deltaT2 / 2.0F);
        deltaS3 = 2.0F * sol - 13.333333F;
      } else {
        deltaT1 = sqrtf((sol - tmp) * 10.0F) / 10.0F;
        deltaT2 = 0.0F;

        /*  interval 2 or 6 disappears */
        x[3] = deltaT1;
        v1 = deltaT1 * deltaT1 * 5.0F + tmp;
        v2 = v1;
        deltaS3_0 = 1.66666663F * rt_powf_snf(deltaT1, 3.0F);
        deltaS1 = deltaT1 * segStartSpeeds_data[i] + deltaS3_0;
        deltaS2 = 0.0F;
        deltaS3 = deltaT1 * sol - deltaS3_0;
      }

      /*  acceleration interval */
      /*  Interval time */
      /*  Velocity at the interval boundary */
      /*  Interval distance */
      b_sAccel[0] = deltaS1;
      b_sAccel[1] = deltaS2;
      b_sAccel[2] = deltaS3;

      /*  Distance in intervals 5-7: deceleration intervals */
      /* ------------------------------------------------------------------ */
      /* getNonConstSpeedIntervalDistance Calculate the travelled distance */
      /*    during the acceleration or the deceleration interval, i.e., intervals */
      /*    1-3 or 5-7, given the starting or the ending velocity, vBound, */
      /*    the maximum speed, vMax and the maximum acceleration or */
      /*    deceleration, aMax. Before calculating the distance, it needs */
      /*    to determine if vMax can be reached without reaching aMax, */
      /*    i.e., if interval 2 or 6 exists. intervalFlag is used as a flag to */
      /*    distinguish the acceleration interval from the deceleration one. */
      /*  A speed threshold below which aMax is not reached */
      if (segEndSpeeds_data[i] + 40.0F <= sol) {
        /*  interval 2 or 6 needed */
        deltaT1_0 = 2.0F;
        deltaT2_0 = (sol - segEndSpeeds_data[i]) / 20.0F - 2.0F;
        x[5] = 2.0F;
        v1_0 = tmp_0 + 20.0F;
        v2_0 = (tmp_0 + 20.0F) + deltaT2_0 * 20.0F;
        deltaS1_0 = 2.0F * tmp_0 + 13.333333F;
        deltaS2_0 = ((tmp_0 + 20.0F) + v2_0) * (deltaT2_0 / 2.0F);
        deltaS3_0 = 2.0F * sol - 13.333333F;
      } else {
        deltaT1_0 = sqrtf((sol - tmp_0) * 10.0F) / 10.0F;
        deltaT2_0 = 0.0F;

        /*  interval 2 or 6 disappears */
        x[5] = deltaT1_0;
        v1_0 = deltaT1_0 * deltaT1_0 * 5.0F + tmp_0;
        v2_0 = v1_0;
        deltaS3_0 = 1.66666663F * rt_powf_snf(deltaT1_0, 3.0F);
        deltaS1_0 = deltaT1_0 * tmp_0 + deltaS3_0;
        deltaS2_0 = 0.0F;
        deltaS3_0 = deltaT1_0 * sol - deltaS3_0;
      }

      /*  deceleration interval */
      /*  Flip the order */
      b_sDecel[0] = deltaS3_0;
      b_sDecel[1] = deltaS2_0;
      b_sDecel[2] = deltaS1_0;

      /*  Distance in interval 4: constant speed */
      sConst = cumLengths[g] - (Controller_sum(b_sAccel) + Controller_sum
        (b_sDecel));

      /*  Calculate kinematics at interval boundaries */
      x[0] = 0.0F;
      x[1] = deltaT1;
      x[2] = deltaT2;
      x[4] = sConst / sol;
      x[6] = deltaT2_0;
      x[7] = deltaT1_0;

      /*  Acceleration [0 accelMax accelMax 0 0 -decelMax -decelMax 0] */
      x_0[0] = 0.0F;
      x_0[4] = sConst;
      x_0[1] = deltaS1;
      x_0[5] = deltaS3_0;
      x_0[2] = deltaS2;
      x_0[6] = deltaS2_0;
      x_0[3] = deltaS3;
      x_0[7] = deltaS1_0;
      for (segStartIndex_size = 0; segStartIndex_size < 7; segStartIndex_size++)
      {
        x[segStartIndex_size + 1] += x[segStartIndex_size];
        x_0[segStartIndex_size + 1] += x_0[segStartIndex_size];
      }

      x_0[7] = cumLengths[g];

      /*  Avoid numerical issue */
      segStartSpeeds[0] = tmp;
      segStartSpeeds[1] = v1;
      segStartSpeeds[2] = v2;
      segStartSpeeds[3] = sol;
      segStartSpeeds[4] = sol;
      segStartSpeeds[5] = v2_0;
      segStartSpeeds[6] = v1_0;
      segStartSpeeds[7] = tmp_0;
      loop_ub = g - h_tmp;
      g = loop_ub + 1;
      for (segStartIndex_size = 0; segStartIndex_size <= loop_ub;
           segStartIndex_size++) {
        cumLengths_data[segStartIndex_size] = cumLengths[h_tmp +
          segStartIndex_size];
      }

      Controller_arrayfun(x_0, sol, segStartSpeeds, x, cumLengths_data, &g,
                          y_data, &y_size, b_times_data, &segStartIndex_size);
      if (segStartIndex_data[i] > segEndIndex_data[i]) {
        h_tmp = 0;
        g = 0;
      } else {
        h_tmp = (int32_T)segStartIndex_data[i] - 1;
        g = (int32_T)segEndIndex_data[i];
      }

      sol = directions[q];
      q = g - h_tmp;
      for (segStartIndex_size = 0; segStartIndex_size < q; segStartIndex_size++)
      {
        obj->LastVelocities[h_tmp + segStartIndex_size] =
          y_data[segStartIndex_size] * sol;
      }

      /*  Time is monotonic across segments */
      /*  Update previous segment total time */
    }

    /*  Store the inputs and outputs */
    obj->LastStartVelocity = startVelocity;
    obj->LastEndVelocity = endVelocity;
    for (i = 0; i < 50; i++) {
      obj->LastCumLengths[i] = cumLengths[i];
      obj->LastCurvatures[i] = curvatures[i];
      obj->LastDirections[i] = directions[i];
      varargout_1[i] = obj->LastVelocities[i];
    }
  }
}

static void Controller_eml_find(const boolean_T x_data[], const int32_T *x_size,
  int32_T i_data[], int32_T *i_size, int32_T j_data[], int32_T *j_size)
{
  int32_T k;
  k = (*x_size >= 1);
  if (*x_size == 0) {
    *i_size = 0;
    *j_size = 0;
  } else {
    int32_T idx;
    int32_T ii;
    idx = 0;
    *i_size = k;
    *j_size = k;
    ii = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (x_data[ii - 1]) {
        idx = 1;
        i_data[0] = ii;
        j_data[0] = 1;
        exitg1 = 1;
      } else {
        ii++;
        if (ii > *x_size) {
          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);

    if (k == 1) {
      if (idx == 0) {
        *i_size = 0;
        *j_size = 0;
      }
    } else {
      *i_size = (idx >= 1);
      k = (idx >= 1);
      if (k - 1 >= 0) {
        j_data[0] = 1;
      }

      *j_size = k;
    }
  }
}

static real32_T Controller_sind(real32_T x)
{
  real32_T b_x;
  int8_T n;
  if (rtIsInfF(x) || rtIsNaNF(x)) {
    b_x = (rtNaNF);
  } else {
    real32_T absx;
    b_x = rt_remf_snf(x, 360.0F);
    absx = fabsf(b_x);
    if (absx > 180.0F) {
      if (b_x > 0.0F) {
        b_x -= 360.0F;
      } else {
        b_x += 360.0F;
      }

      absx = fabsf(b_x);
    }

    if (absx <= 45.0F) {
      b_x = sinf(0.0174532924F * b_x);
    } else {
      boolean_T guard1 = false;
      guard1 = false;
      if (absx <= 135.0F) {
        if (b_x > 0.0F) {
          b_x = cosf((b_x - 90.0F) * 0.0174532924F);
        } else {
          b_x = (b_x + 90.0F) * 0.0174532924F;
          n = -1;
          guard1 = true;
        }
      } else {
        if (b_x > 0.0F) {
          b_x = (b_x - 180.0F) * 0.0174532924F;
          n = 2;
        } else {
          b_x = (b_x + 180.0F) * 0.0174532924F;
          n = -2;
        }

        guard1 = true;
      }

      if (guard1) {
        if (n == -1) {
          b_x = -cosf(b_x);
        } else {
          b_x = -sinf(b_x);
        }
      }
    }
  }

  return b_x;
}

static void Con_HelperPathAnalyzer_stepImpl(HelperPathAnalyzer_Controller_T *obj,
  const real32_T currPose[3], real32_T currVel, const real32_T varargin_1[150],
  const real32_T varargin_2[50], const real32_T varargin_3[50], const real32_T
  varargin_4[50], real32_T refPose[3], real32_T *refVel, real32_T *direction,
  real32_T *curvature, real32_T *varargout_1)
{
  int32_T ii_data[49];
  int32_T switchIndex_data[49];
  int32_T refPoses_size[2];
  int32_T segRefPoses_size[2];
  int32_T i;
  int32_T idx;
  int32_T ii_data_0;
  int32_T jj_data;
  int32_T refPoses_size_idx_0;
  real32_T refPoses_data[150];
  real32_T segRefPoses_data[150];
  real32_T dis2PointsSquare_data[50];
  real32_T refPoses_data_0[50];
  int8_T n;
  boolean_T dis2PointsSquare_data_0[50];
  boolean_T x[49];
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  /* ------------------------------------------------------------------ */
  /* stepImpl Implement the main algorithm and return the reference */
  /*    pose, velocity and driving direction. varargout is an */
  /*    optional output in Simulink that signifies reaching */
  /*    intermediate goals within a reference path, i.e., reaching */
  /*    the direction-switching positions.  */
  /*  Check if the reference path is new */
  p = false;
  p_0 = true;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 150)) {
    if (!(obj->RefPosesInternal[i] == varargin_1[i])) {
      p_0 = false;
      exitg1 = true;
    } else {
      i++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (!p) {
    memcpy(&obj->RefPosesInternal[0], &varargin_1[0], 150U * sizeof(real32_T));
    memcpy(&obj->DirectionsInternal[0], &varargin_2[0], 50U * sizeof(real32_T));
    memcpy(&obj->CurvaturesInternal[0], &varargin_3[0], 50U * sizeof(real32_T));
    memcpy(&obj->VelocityProfileInternal[0], &varargin_4[0], 50U * sizeof
           (real32_T));
    obj->CurrentSegmentIndex = 1.0F;
    obj->ClosestPointIndex = 1.0F;
  }

  /*  Divide the path to segments based on driving direction */
  /* ------------------------------------------------------------------ */
  /* findSegmentBoundaryPointIndex Divide the path to segments  */
  /* based on driving direction */
  /*  Find the switching points */
  for (i = 0; i < 49; i++) {
    x[i] = (obj->DirectionsInternal[(i + 2) - 1] + obj->DirectionsInternal[i] ==
            0.0F);
  }

  idx = 0;
  i = 1;
  exitg1 = false;
  while ((!exitg1) && (i - 1 < 49)) {
    if (x[i - 1]) {
      idx++;
      ii_data[idx - 1] = i;
      if (idx >= 49) {
        exitg1 = true;
      } else {
        i++;
      }
    } else {
      i++;
    }
  }

  if (idx < 1) {
    idx = 0;
  }

  if (idx - 1 >= 0) {
    memcpy(&switchIndex_data[0], &ii_data[0], (uint32_T)idx * sizeof(int32_T));
  }

  /*  Divide the path into segments */
  refPoses_size_idx_0 = obj->SegmentStartIndex->size[0];
  obj->SegmentStartIndex->size[0] = idx + 1;
  Cont_emxEnsureCapacity_real32_T(obj->SegmentStartIndex, refPoses_size_idx_0);
  obj->SegmentStartIndex->data[0] = 1.0F;
  for (i = 0; i < idx; i++) {
    obj->SegmentStartIndex->data[i + 1] = (real32_T)((real_T)switchIndex_data[i]
      + 1.0);
  }

  refPoses_size_idx_0 = obj->SegmentEndIndex->size[0];
  obj->SegmentEndIndex->size[0] = idx + 1;
  Cont_emxEnsureCapacity_real32_T(obj->SegmentEndIndex, refPoses_size_idx_0);
  for (i = 0; i < idx; i++) {
    obj->SegmentEndIndex->data[i] = (real32_T)switchIndex_data[i];
  }

  obj->SegmentEndIndex->data[idx] = 50.0F;
  obj->NumPathSegments = (real32_T)obj->SegmentStartIndex->size[0];

  /*  Check if reaching the final goal. If yes, use the previous */
  /*  outputs */
  if (obj->CurrentSegmentIndex > obj->NumPathSegments) {
    refPose[0] = obj->LastRefPoseOutput[0];
    refPose[1] = obj->LastRefPoseOutput[1];
    refPose[2] = obj->LastRefPoseOutput[2];
    *refVel = obj->LastRefVelocityOutput;
    *direction = obj->LastDirectionOutput;
    *curvature = obj->LastCurvatureOutput;

    /* ------------------------------------------------------------------ */
    /* isSimulinkBlock Check if the system object in used in Simulink */
    /*  0 for MATLAB, 1 for Simulink */
    *varargout_1 = 1.0F;
  } else {
    real_T segClosestPointIndex;
    int32_T b_idx;
    real32_T scale;
    real32_T t;
    real32_T vec1_idx_0;
    real32_T vec1_idx_1;
    real32_T vec2_idx_0;
    real32_T vec2_idx_1;

    /*  Get the desired pose, desired velocity and driving direction */
    vec2_idx_0 = currPose[0];
    vec2_idx_1 = currPose[1];

    /* ------------------------------------------------------------------ */
    /* findDesiredPoseAndVelocity Determine the desired pose and */
    /*    velocity based on the current pose. The desired pose is */
    /*    determined by searching the closest point on the reference */
    /*    path. The desired velocity is the velocity corresponding to */
    /*    the closest point. */
    /*  Get the current segment indexes */
    t = 0.0174532924F * currPose[2];

    /*  Only search within the current segment of the path */
    scale = obj->SegmentStartIndex->data[(int32_T)obj->CurrentSegmentIndex - 1];
    vec1_idx_0 = obj->SegmentEndIndex->data[(int32_T)obj->CurrentSegmentIndex -
      1];
    if (scale > vec1_idx_0) {
      b_idx = 1;
      idx = 0;
    } else {
      b_idx = (int32_T)scale;
      idx = (int32_T)vec1_idx_0;
    }

    idx -= b_idx;
    segRefPoses_size[0] = idx + 1;
    for (i = 0; i < 3; i++) {
      for (refPoses_size_idx_0 = 0; refPoses_size_idx_0 <= idx;
           refPoses_size_idx_0++) {
        segRefPoses_data[refPoses_size_idx_0 + (idx + 1) * i] =
          obj->RefPosesInternal[((b_idx + refPoses_size_idx_0) + 50 * i) - 1];
      }
    }

    for (i = 0; i <= idx; i++) {
      segRefPoses_data[i + ((idx + 1) << 1)] = obj->RefPosesInternal[(b_idx + i)
        + 99] * 0.0174532924F;
    }

    /*  Current driving direction */
    *direction = obj->DirectionsInternal[(int32_T)obj->SegmentEndIndex->data
      [(int32_T)obj->CurrentSegmentIndex - 1] - 1];

    /*  Compute the index of the closest point on the path segment */
    refPoses_size[0] = idx + 1;
    i = (idx + 1) * 3;
    if (i - 1 >= 0) {
      memcpy(&refPoses_data[0], &segRefPoses_data[0], (uint32_T)i * sizeof
             (real32_T));
    }

    /* ------------------------------------------------------------------ */
    /* findClosestPathPoint Find the index of the closest point */
    if (obj->DirectionsInternal[(int32_T)obj->SegmentStartIndex->data[(int32_T)
        obj->CurrentSegmentIndex - 1] - 1] == 1.0F) {
      /*  forward driving uses front wheel as reference */
      vec2_idx_0 = 2.8F * cosf(t) + currPose[0];
      vec2_idx_1 = 2.8F * sinf(t) + currPose[1];
      refPoses_size[0] = idx + 1;
      if (i - 1 >= 0) {
        memcpy(&refPoses_data[0], &segRefPoses_data[0], (uint32_T)i * sizeof
               (real32_T));
      }

      idx++;
      for (i = 0; i < idx; i++) {
        dis2PointsSquare_data[i] = segRefPoses_data[(segRefPoses_size[0] << 1) +
          i];
      }

      idx = segRefPoses_size[0] - 1;
      for (i = 0; i <= idx; i++) {
        dis2PointsSquare_data[i] = cosf(dis2PointsSquare_data[i]);
      }

      idx = segRefPoses_size[0];
      for (i = 0; i < idx; i++) {
        refPoses_data[i] = 2.8F * dis2PointsSquare_data[i] + segRefPoses_data[i];
      }

      idx = refPoses_size[0];
      for (i = 0; i < idx; i++) {
        dis2PointsSquare_data[i] = refPoses_data[(refPoses_size[0] << 1) + i];
      }

      idx = refPoses_size[0] - 1;
      for (i = 0; i <= idx; i++) {
        dis2PointsSquare_data[i] = sinf(dis2PointsSquare_data[i]);
      }

      refPoses_size_idx_0 = refPoses_size[0];
      idx = refPoses_size[0];
      for (i = 0; i < idx; i++) {
        refPoses_data_0[i] = 2.8F * dis2PointsSquare_data[i] + refPoses_data[i +
          refPoses_size[0]];
      }

      for (i = 0; i < refPoses_size_idx_0; i++) {
        refPoses_data[i + refPoses_size[0]] = refPoses_data_0[i];
      }
    }

    idx = refPoses_size[0];
    for (i = 0; i < idx; i++) {
      vec1_idx_0 = refPoses_data[i] - vec2_idx_0;
      dis2PointsSquare_data[i] = vec1_idx_0 * vec1_idx_0;
    }

    idx = refPoses_size[0];
    for (i = 0; i < idx; i++) {
      vec1_idx_0 = refPoses_data[i + refPoses_size[0]] - vec2_idx_1;
      refPoses_data_0[i] = vec1_idx_0 * vec1_idx_0;
    }

    idx = refPoses_size[0];
    for (i = 0; i < idx; i++) {
      dis2PointsSquare_data[i] += refPoses_data_0[i];
    }

    /*  Find the closest point on the reference path */
    idx = refPoses_size[0];
    if ((uint8_T)(refPoses_size[0] - 1) + 1 <= 2) {
      if ((uint8_T)(refPoses_size[0] - 1) + 1 == 1) {
        vec1_idx_1 = dis2PointsSquare_data[0];
      } else {
        vec1_idx_1 = dis2PointsSquare_data[refPoses_size[0] - 1];
        if ((dis2PointsSquare_data[0] > vec1_idx_1) || (rtIsNaNF
             (dis2PointsSquare_data[0]) && (!rtIsNaNF(vec1_idx_1)))) {
        } else {
          vec1_idx_1 = dis2PointsSquare_data[0];
        }
      }
    } else {
      if (!rtIsNaNF(dis2PointsSquare_data[0])) {
        b_idx = 1;
      } else {
        b_idx = 0;
        i = 2;
        exitg1 = false;
        while ((!exitg1) && (i <= idx)) {
          if (!rtIsNaNF(dis2PointsSquare_data[i - 1])) {
            b_idx = i;
            exitg1 = true;
          } else {
            i++;
          }
        }
      }

      if (b_idx == 0) {
        vec1_idx_1 = dis2PointsSquare_data[0];
      } else {
        vec1_idx_1 = dis2PointsSquare_data[b_idx - 1];
        for (i = b_idx + 1; i <= idx; i++) {
          vec1_idx_0 = dis2PointsSquare_data[i - 1];
          if (vec1_idx_1 > vec1_idx_0) {
            vec1_idx_1 = vec1_idx_0;
          }
        }
      }
    }

    refPoses_size_idx_0 = refPoses_size[0];
    idx = refPoses_size[0];
    for (i = 0; i < idx; i++) {
      dis2PointsSquare_data_0[i] = (dis2PointsSquare_data[i] == vec1_idx_1);
    }

    Controller_eml_find(dis2PointsSquare_data_0, &refPoses_size_idx_0,
                        &ii_data_0, &i, &jj_data, &idx);
    if (i - 1 >= 0) {
      jj_data = ii_data_0;
    }

    /*  Enforce to be a scalar in Simulink */
    segClosestPointIndex = jj_data;

    /*  If the reference pose is lagging behind the current pose, */
    /*  move to the next reference path. */
    /* ------------------------------------------------------------------ */
    /* moveToNext Check if the refPose is lagging behind the current */
    /*    pose. If yes, move to the next refPose. */
    /*    The is necessary when the vehicle is at accelerating stage. */
    /*    When the reference speed is small it takes relatively */
    /*    longer time to reach the desired maximum speed. When the */
    /*    vehicle reaches somewhere between two reference points, */
    /*    use the next one as the reference to set a larger */
    /*    reference speed. */
    if (obj->DirectionsInternal[0] == 1.0F) {
      vec1_idx_1 = refPoses_data[((refPoses_size[0] << 1) + jj_data) - 1];
      vec1_idx_0 = cosf(vec1_idx_1);
      vec1_idx_1 = sinf(vec1_idx_1);
      vec2_idx_0 = (2.8F * cosf(t) + vec2_idx_0) - (vec1_idx_0 * 2.8F +
        refPoses_data[jj_data - 1]);
      vec2_idx_1 = (2.8F * sinf(t) + vec2_idx_1) - (refPoses_data[(jj_data +
        refPoses_size[0]) - 1] + vec1_idx_1 * 2.8F);
    } else {
      t = refPoses_data[((refPoses_size[0] << 1) + jj_data) - 1];
      vec1_idx_0 = cosf(t);
      vec1_idx_1 = sinf(t);
      vec2_idx_0 -= refPoses_data[jj_data - 1];
      vec2_idx_1 -= refPoses_data[(jj_data + refPoses_size[0]) - 1];
    }

    if (((vec1_idx_0 * vec2_idx_0 + vec1_idx_1 * vec2_idx_1) *
         obj->DirectionsInternal[0] > 0.0F) && (jj_data != refPoses_size[0])) {
      segClosestPointIndex = (real_T)jj_data + 1.0;
    }

    /*  Convert the segment index to the whole path index */
    obj->ClosestPointIndex = (scale + (real32_T)segClosestPointIndex) - 1.0F;

    /*  Get the desired velocity. Set a lower threshold to avoid zero  */
    /*  reference velocity at the very beginning. */
    if (segClosestPointIndex == 1.0) {
      *refVel = fmaxf(fabsf(obj->VelocityProfileInternal[(int32_T)
                            obj->ClosestPointIndex - 1]), 0.1F) * *direction;
    } else {
      *refVel = obj->VelocityProfileInternal[(int32_T)obj->ClosestPointIndex - 1];
    }

    /*  Get the desired pose. In forward motion, the refPose is */
    /*  specified for the front wheel. */
    if (*direction == 1.0F) {
      /*  forward */
      scale = segRefPoses_data[((segRefPoses_size[0] << 1) + (int32_T)
        segClosestPointIndex) - 1];
      refPose[2] = scale;
      refPose[0] = cosf(scale) * 2.8F + segRefPoses_data[(int32_T)
        segClosestPointIndex - 1];
      refPose[1] = segRefPoses_data[((int32_T)segClosestPointIndex +
        segRefPoses_size[0]) - 1] + sinf(scale) * 2.8F;
    } else {
      refPose[0] = segRefPoses_data[(int32_T)segClosestPointIndex - 1];
      refPose[1] = segRefPoses_data[((int32_T)segClosestPointIndex +
        segRefPoses_size[0]) - 1];
      refPose[2] = segRefPoses_data[((segRefPoses_size[0] << 1) + (int32_T)
        segClosestPointIndex) - 1];
    }

    /*  Workaround to support lateralControllerStanley in MATLAB */
    /*  that does not require curvature input */
    *curvature = obj->CurvaturesInternal[(int32_T)obj->ClosestPointIndex - 1];
    refPose[2] *= 57.2957802F;

    /*  Check if the vehicle reaches the intermediate goal. If yes, */
    /*  increment the path segment index and reset reference velocity */
    /*  to zero as the vehicle needs to switch direction at the */
    /*  intermediate goal positions */
    vec2_idx_1 = obj->VelocityProfileInternal[(int32_T)obj->
      SegmentEndIndex->data[(int32_T)obj->CurrentSegmentIndex - 1] - 1];
    *varargout_1 = 0.0F;

    /*  The goal checker acts when the distance from the vehicle to the goal */
    /*  point is within a distance tolerance, disTol.  */
    i = (int32_T)obj->SegmentEndIndex->data[(int32_T)obj->CurrentSegmentIndex -
      1];
    scale = 1.29246971E-26F;
    vec2_idx_0 = fabsf(obj->RefPosesInternal[i - 1] - currPose[0]);
    if (vec2_idx_0 > 1.29246971E-26F) {
      vec1_idx_0 = 1.0F;
      scale = vec2_idx_0;
    } else {
      t = vec2_idx_0 / 1.29246971E-26F;
      vec1_idx_0 = t * t;
    }

    vec2_idx_0 = fabsf(obj->RefPosesInternal[i + 49] - currPose[1]);
    if (vec2_idx_0 > scale) {
      t = scale / vec2_idx_0;
      vec1_idx_0 = vec1_idx_0 * t * t + 1.0F;
      scale = vec2_idx_0;
    } else {
      t = vec2_idx_0 / scale;
      vec1_idx_0 += t * t;
    }

    vec1_idx_0 = scale * sqrtf(vec1_idx_0);
    if (vec1_idx_0 > 1.0F) {
      p = false;
    } else {
      /*  Check if the vehicle has passed the goal position by checking the angle */
      /*  between two vectors */
      scale = obj->RefPosesInternal[(int32_T)obj->SegmentEndIndex->data[(int32_T)
        obj->CurrentSegmentIndex - 1] + 99];
      if (rtIsInfF(scale) || rtIsNaNF(scale)) {
        vec2_idx_0 = (rtNaNF);
      } else {
        vec2_idx_0 = rt_remf_snf(scale, 360.0F);
        t = fabsf(vec2_idx_0);
        if (t > 180.0F) {
          if (vec2_idx_0 > 0.0F) {
            vec2_idx_0 -= 360.0F;
          } else {
            vec2_idx_0 += 360.0F;
          }

          t = fabsf(vec2_idx_0);
        }

        if (t <= 45.0F) {
          vec2_idx_0 = cosf(0.0174532924F * vec2_idx_0);
        } else {
          boolean_T guard1 = false;
          guard1 = false;
          if (t <= 135.0F) {
            if (vec2_idx_0 > 0.0F) {
              vec2_idx_0 = -sinf((vec2_idx_0 - 90.0F) * 0.0174532924F);
            } else {
              vec2_idx_0 = (vec2_idx_0 + 90.0F) * 0.0174532924F;
              n = -1;
              guard1 = true;
            }
          } else {
            if (vec2_idx_0 > 0.0F) {
              vec2_idx_0 = (vec2_idx_0 - 180.0F) * 0.0174532924F;
              n = 2;
            } else {
              vec2_idx_0 = (vec2_idx_0 + 180.0F) * 0.0174532924F;
              n = -2;
            }

            guard1 = true;
          }

          if (guard1) {
            if (n == -1) {
              vec2_idx_0 = sinf(vec2_idx_0);
            } else {
              vec2_idx_0 = -cosf(vec2_idx_0);
            }
          }
        }
      }

      /*  Steps of goal checking: */
      /*  1) If the vehicle passes the goal, check its current velocity.  */
      /*  2) If the velocity is less than a threshold, assume the vehicle stops. */
      /*  Otherwise, check the reference velocity at the goal position. */
      /*  3) If the goal reference velocity is non-zero, then assume the vehicle   */
      /*  reaches the goal. If the goal reference velocity is zero, i.e., the  */
      /*  vehicle is supposed to stop at the goal position, then allow the vehicle  */
      /*  to move until its velocity decreases below the threshold. */
      /*  meters/second */
      if (((currPose[0] - obj->RefPosesInternal[(int32_T)obj->
            SegmentEndIndex->data[(int32_T)obj->CurrentSegmentIndex - 1] - 1]) *
           vec2_idx_0 + (currPose[1] - obj->RefPosesInternal[(int32_T)
                         obj->SegmentEndIndex->data[(int32_T)
                         obj->CurrentSegmentIndex - 1] + 49]) * Controller_sind
           (scale)) * *direction > 0.0F) {
        p = ((fabsf(currVel) < 0.05) || (vec2_idx_1 != 0.0F));
      } else {
        p = (fabsf(currVel) < 0.05);
      }
    }

    if (p) {
      obj->CurrentSegmentIndex++;
      *refVel = vec2_idx_1;
      *varargout_1 = 1.0F;
    }

    /* ------------------------------------------------------------------ */
    /* isSimulinkBlock Check if the system object in used in Simulink */
    /*  0 for MATLAB, 1 for Simulink */
    /*  Store the output */
    obj->LastRefPoseOutput[0] = refPose[0];
    obj->LastRefPoseOutput[1] = refPose[1];
    obj->LastRefPoseOutput[2] = refPose[2];
    obj->LastRefVelocityOutput = *refVel;
    obj->LastDirectionOutput = *direction;
    obj->LastCurvatureOutput = *curvature;
  }
}

/* Function for MATLAB Function: '<S4>/Kinematic' */
static void Contro_angleUtilities_wrapTo2Pi(real32_T *theta)
{
  real32_T x;
  boolean_T positiveInput;
  positiveInput = (*theta > 0.0F);
  x = *theta;
  if (rtIsNaNF(*theta)) {
    *theta = (rtNaNF);
  } else if (rtIsInfF(*theta)) {
    *theta = (rtNaNF);
  } else if (*theta == 0.0F) {
    *theta = 0.0F;
  } else {
    boolean_T rEQ0;
    *theta = fmodf(*theta, 6.28318548F);
    rEQ0 = (*theta == 0.0F);
    if (!rEQ0) {
      real32_T q;
      q = fabsf(x / 6.28318548F);
      rEQ0 = !(fabsf(q - floorf(q + 0.5F)) > 1.1920929E-7F * q);
    }

    if (rEQ0) {
      *theta = 0.0F;
    } else if (x < 0.0F) {
      *theta += 6.28318548F;
    }
  }

  *theta += (real32_T)((*theta == 0.0F) && positiveInput) * 6.28318548F;
}

static void emxFreeStruct_HelperPathAnalyze(HelperPathAnalyzer_Controller_T
  *pStruct)
{
  Controller_emxFree_real32_T(&pStruct->SegmentStartIndex);
  Controller_emxFree_real32_T(&pStruct->SegmentEndIndex);
}

static void emxInitStruct_HelperPathAnalyze(HelperPathAnalyzer_Controller_T
  *pStruct)
{
  Controller_emxInit_real32_T(&pStruct->SegmentStartIndex, 1);
  Controller_emxInit_real32_T(&pStruct->SegmentEndIndex, 1);
}

static void Controller_SystemCore_setup(HelperPathAnalyzer_Controller_T *obj)
{
  int32_T i;
  obj->isInitialized = 1;

  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Validate inputs to the step method at initialization */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /*  RefPoses */
  /*  Directions */
  /*  Curvatures */
  /*  VelocityProfile */
  /* ------------------------------------------------------------------ */
  /* setupImpl Perform one-time calculations */
  obj->ClosestPointIndex = 1.0F;
  obj->NumPathSegments = 1.0F;
  obj->CurrentSegmentIndex = 1.0F;
  for (i = 0; i < 150; i++) {
    obj->RefPosesInternal[i] = (rtNaNF);
  }

  for (i = 0; i < 50; i++) {
    obj->DirectionsInternal[i] = (rtNaNF);
    obj->CurvaturesInternal[i] = (rtNaNF);
    obj->VelocityProfileInternal[i] = (rtNaNF);
  }

  obj->TunablePropsChanged = false;
}

/* Model step function */
void Controller_step(void)
{
  real_T rtb_Sum2_e;
  real_T rtb_Sum3;
  real_T rtb_Sum_idx_0;
  int32_T i;
  real32_T b_varargout_3[50];
  real32_T currPose[3];
  real32_T refPose[3];
  real32_T tmp[2];
  real32_T b;
  real32_T d_idx_1;
  real32_T distToGoal;
  real32_T rtb_DataTypeConversion_idx_0;
  real32_T rtb_DataTypeConversion_idx_1;
  real32_T t;
  int8_T n;
  boolean_T rtb_FixPtRelationalOperator;
  boolean_T rtb_planNext;

  /* RelationalOperator: '<S118>/FixPt Relational Operator' incorporates:
   *  Inport: '<Root>/Controller_In'
   *  UnitDelay: '<S118>/Delay Input1'
   *
   * Block description for '<S118>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_FixPtRelationalOperator = (Controller_U.Controller_In.timestamp !=
    Controller_DW.DelayInput1_DSTATE);

  /* Outputs for Enabled SubSystem: '<S2>/Path Planner' incorporates:
   *  EnablePort: '<S119>/Enable'
   */
  if (rtb_FixPtRelationalOperator) {
    real_T rtb_Multiply1_tmp;

    /* Outputs for Atomic SubSystem: '<S119>/Enabled Subsystem' */
    /* Gain: '<S121>/Gain2' incorporates:
     *  Inport: '<Root>/Controller_In'
     */
    rtb_Sum3 = 1.0000000003410059E-9 * (real_T)Controller_U.Controller_In.lat_0;

    /* Gain: '<S122>/deg2rad1' incorporates:
     *  Gain: '<S121>/Gain'
     *  Inport: '<Root>/VehicheInfo'
     *  Sum: '<S122>/Sum'
     */
    rtb_Sum_idx_0 = (1.0000000003410059E-9 * (real_T)
                     Controller_U.VehicheInfo.lat - rtb_Sum3) *
      0.017453292519943295;

    /* Gain: '<S122>/deg2rad' */
    rtb_Sum3 *= 0.017453292519943295;

    /* Trigonometry: '<S123>/Sin' */
    rtb_Sum2_e = sin(rtb_Sum3);

    /* Product: '<S123>/Multiply1' incorporates:
     *  Math: '<S123>/Square1'
     *  Product: '<S123>/Multiply'
     */
    rtb_Multiply1_tmp = rtb_Sum2_e * rtb_Sum2_e * Controller_ConstB.ff;

    /* Product: '<S123>/Divide' incorporates:
     *  Constant: '<S123>/Constant'
     *  Constant: '<S123>/R'
     *  Sqrt: '<S123>/Sqrt'
     *  Sum: '<S123>/Sum1'
     */
    rtb_Sum2_e = 6.378137E+6 / sqrt(1.0 - rtb_Multiply1_tmp);

    /* Product: '<S122>/Multiply' incorporates:
     *  Constant: '<S123>/Constant1'
     *  Gain: '<S121>/Gain1'
     *  Gain: '<S121>/Gain3'
     *  Gain: '<S122>/deg2rad1'
     *  Inport: '<Root>/Controller_In'
     *  Inport: '<Root>/VehicheInfo'
     *  Product: '<S123>/Multiply1'
     *  Product: '<S123>/Multiply2'
     *  Product: '<S123>/Product3'
     *  Sum: '<S122>/Sum'
     *  Sum: '<S123>/Sum2'
     *  Trigonometry: '<S123>/Cos'
     */
    rtb_Sum_idx_0 *= 1.0 / (1.0 - rtb_Multiply1_tmp) * Controller_ConstB.Sum4 *
      rtb_Sum2_e;
    rtb_Sum3 = (1.0000000003410059E-9 * (real_T)Controller_U.VehicheInfo.lon -
                1.0000000003410059E-9 * (real_T)Controller_U.Controller_In.lon_0)
      * 0.017453292519943295 * (rtb_Sum2_e * cos(rtb_Sum3));

    /* DataTypeConversion: '<S121>/Data Type Conversion' incorporates:
     *  Product: '<S125>/Multiply1'
     *  Product: '<S125>/Multiply2'
     *  Product: '<S125>/Multiply3'
     *  Product: '<S125>/Multiply4'
     *  Sum: '<S125>/Sum2'
     *  Sum: '<S125>/Sum3'
     */
    rtb_DataTypeConversion_idx_0 = (real32_T)(rtb_Sum_idx_0 *
      Controller_ConstB.SinCos_o2 + rtb_Sum3 * Controller_ConstB.SinCos_o1);
    rtb_DataTypeConversion_idx_1 = (real32_T)(rtb_Sum3 *
      Controller_ConstB.SinCos_o2 - rtb_Sum_idx_0 * Controller_ConstB.SinCos_o1);

    /* End of Outputs for SubSystem: '<S119>/Enabled Subsystem' */
    for (i = 0; i < 50; i++) {
      /* Sum: '<S119>/Sum' incorporates:
       *  Concatenate: '<S119>/Vector Concatenate'
       *  Inport: '<Root>/Controller_In'
       *  Selector: '<S119>/Selector Col1'
       */
      Controller_B.VectorConcatenate[i] = rtb_DataTypeConversion_idx_0 +
        Controller_U.Controller_In.savePose[i];

      /* Sum: '<S119>/Sum1' incorporates:
       *  Concatenate: '<S119>/Vector Concatenate'
       *  Inport: '<Root>/Controller_In'
       *  Selector: '<S119>/Selector Col2'
       */
      Controller_B.VectorConcatenate[i + 50] =
        Controller_U.Controller_In.savePose[i + 50] +
        rtb_DataTypeConversion_idx_1;

      /* Selector: '<S119>/Selector Col3' incorporates:
       *  Concatenate: '<S119>/Vector Concatenate'
       *  Inport: '<Root>/Controller_In'
       */
      Controller_B.VectorConcatenate[i + 100] =
        Controller_U.Controller_In.savePose[i + 100];
    }
  }

  /* End of Outputs for SubSystem: '<S2>/Path Planner' */

  /* MATLAB Function: '<S2>/Behavior Planner' incorporates:
   *  BusCreator generated from: '<S2>/Behavior Planner'
   *  Concatenate: '<S119>/Vector Concatenate'
   *  Inport: '<Root>/Controller_In'
   *  Inport: '<Root>/VehicheInfo'
   */
  if ((!Controller_DW.goalIndex_not_empty) || rtb_FixPtRelationalOperator) {
    Controller_DW.finalReached = false;
    Controller_DW.endSpeed = 1.0F;
    Controller_DW.goalIndex_not_empty = true;
    Controller_DW.nextGoalPose[0] = Controller_U.VehicheInfo.currPose[0];
    Controller_DW.nextGoalPose[1] = Controller_B.VectorConcatenate[0];
    Controller_DW.nextGoalPose[2] = Controller_U.VehicheInfo.currPose[1];
    Controller_DW.nextGoalPose[3] = Controller_B.VectorConcatenate[50];
    Controller_DW.nextGoalPose[4] = Controller_U.VehicheInfo.currPose[2];
    Controller_DW.nextGoalPose[5] = Controller_B.VectorConcatenate[100];
    Controller_DW.speedConfig.StartSpeed = Controller_U.VehicheInfo.currVelocity;
    Controller_DW.speedConfig.EndSpeed = 1.0F;
    Controller_DW.goalIndex = 2.0;
  }

  rtb_planNext = false;
  if (!Controller_DW.finalReached) {
    boolean_T reachGoal;
    rtb_DataTypeConversion_idx_0 = 1.29246971E-26F;
    rtb_DataTypeConversion_idx_1 = fabsf(Controller_DW.nextGoalPose[1] -
      Controller_U.VehicheInfo.currPose[0]);
    if (rtb_DataTypeConversion_idx_1 > 1.29246971E-26F) {
      distToGoal = 1.0F;
      rtb_DataTypeConversion_idx_0 = rtb_DataTypeConversion_idx_1;
    } else {
      t = rtb_DataTypeConversion_idx_1 / 1.29246971E-26F;
      distToGoal = t * t;
    }

    rtb_DataTypeConversion_idx_1 = fabsf(Controller_DW.nextGoalPose[3] -
      Controller_U.VehicheInfo.currPose[1]);
    if (rtb_DataTypeConversion_idx_1 > rtb_DataTypeConversion_idx_0) {
      t = rtb_DataTypeConversion_idx_0 / rtb_DataTypeConversion_idx_1;
      distToGoal = distToGoal * t * t + 1.0F;
      rtb_DataTypeConversion_idx_0 = rtb_DataTypeConversion_idx_1;
    } else {
      t = rtb_DataTypeConversion_idx_1 / rtb_DataTypeConversion_idx_0;
      distToGoal += t * t;
    }

    distToGoal = rtb_DataTypeConversion_idx_0 * sqrtf(distToGoal);
    if (distToGoal > 1.0F) {
      reachGoal = false;
    } else {
      if (rtIsInfF(Controller_DW.nextGoalPose[5]) || rtIsNaNF
          (Controller_DW.nextGoalPose[5])) {
        rtb_DataTypeConversion_idx_0 = (rtNaNF);
        rtb_DataTypeConversion_idx_1 = (rtNaNF);
      } else {
        rtb_DataTypeConversion_idx_0 = rt_remf_snf(Controller_DW.nextGoalPose[5],
          360.0F);
        rtb_DataTypeConversion_idx_1 = fabsf(rtb_DataTypeConversion_idx_0);
        if (rtb_DataTypeConversion_idx_1 > 180.0F) {
          if (rtb_DataTypeConversion_idx_0 > 0.0F) {
            rtb_DataTypeConversion_idx_0 -= 360.0F;
          } else {
            rtb_DataTypeConversion_idx_0 += 360.0F;
          }

          rtb_DataTypeConversion_idx_1 = fabsf(rtb_DataTypeConversion_idx_0);
        }

        if (rtb_DataTypeConversion_idx_1 <= 45.0F) {
          rtb_DataTypeConversion_idx_0 *= 0.0174532924F;
          n = 0;
        } else if (rtb_DataTypeConversion_idx_1 <= 135.0F) {
          if (rtb_DataTypeConversion_idx_0 > 0.0F) {
            rtb_DataTypeConversion_idx_0 = (rtb_DataTypeConversion_idx_0 - 90.0F)
              * 0.0174532924F;
            n = 1;
          } else {
            rtb_DataTypeConversion_idx_0 = (rtb_DataTypeConversion_idx_0 + 90.0F)
              * 0.0174532924F;
            n = -1;
          }
        } else if (rtb_DataTypeConversion_idx_0 > 0.0F) {
          rtb_DataTypeConversion_idx_0 = (rtb_DataTypeConversion_idx_0 - 180.0F)
            * 0.0174532924F;
          n = 2;
        } else {
          rtb_DataTypeConversion_idx_0 = (rtb_DataTypeConversion_idx_0 + 180.0F)
            * 0.0174532924F;
          n = -2;
        }

        switch (n) {
         case 0:
          rtb_DataTypeConversion_idx_0 = cosf(rtb_DataTypeConversion_idx_0);
          break;

         case 1:
          rtb_DataTypeConversion_idx_0 = -sinf(rtb_DataTypeConversion_idx_0);
          break;

         case -1:
          rtb_DataTypeConversion_idx_0 = sinf(rtb_DataTypeConversion_idx_0);
          break;

         default:
          rtb_DataTypeConversion_idx_0 = -cosf(rtb_DataTypeConversion_idx_0);
          break;
        }

        rtb_DataTypeConversion_idx_1 = rt_remf_snf(Controller_DW.nextGoalPose[5],
          360.0F);
        t = fabsf(rtb_DataTypeConversion_idx_1);
        if (t > 180.0F) {
          if (rtb_DataTypeConversion_idx_1 > 0.0F) {
            rtb_DataTypeConversion_idx_1 -= 360.0F;
          } else {
            rtb_DataTypeConversion_idx_1 += 360.0F;
          }

          t = fabsf(rtb_DataTypeConversion_idx_1);
        }

        if (t <= 45.0F) {
          rtb_DataTypeConversion_idx_1 *= 0.0174532924F;
          n = 0;
        } else if (t <= 135.0F) {
          if (rtb_DataTypeConversion_idx_1 > 0.0F) {
            rtb_DataTypeConversion_idx_1 = (rtb_DataTypeConversion_idx_1 - 90.0F)
              * 0.0174532924F;
            n = 1;
          } else {
            rtb_DataTypeConversion_idx_1 = (rtb_DataTypeConversion_idx_1 + 90.0F)
              * 0.0174532924F;
            n = -1;
          }
        } else if (rtb_DataTypeConversion_idx_1 > 0.0F) {
          rtb_DataTypeConversion_idx_1 = (rtb_DataTypeConversion_idx_1 - 180.0F)
            * 0.0174532924F;
          n = 2;
        } else {
          rtb_DataTypeConversion_idx_1 = (rtb_DataTypeConversion_idx_1 + 180.0F)
            * 0.0174532924F;
          n = -2;
        }

        switch (n) {
         case 0:
          rtb_DataTypeConversion_idx_1 = sinf(rtb_DataTypeConversion_idx_1);
          break;

         case 1:
          rtb_DataTypeConversion_idx_1 = cosf(rtb_DataTypeConversion_idx_1);
          break;

         case -1:
          rtb_DataTypeConversion_idx_1 = -cosf(rtb_DataTypeConversion_idx_1);
          break;

         default:
          rtb_DataTypeConversion_idx_1 = -sinf(rtb_DataTypeConversion_idx_1);
          break;
        }
      }

      if (((Controller_U.VehicheInfo.currPose[0] - Controller_DW.nextGoalPose[1])
           * rtb_DataTypeConversion_idx_0 + (Controller_U.VehicheInfo.currPose[1]
            - Controller_DW.nextGoalPose[3]) * rtb_DataTypeConversion_idx_1) *
          Controller_U.VehicheInfo.direction > 0.0F) {
        reachGoal = ((fabsf(Controller_U.VehicheInfo.currVelocity) < 0.05) ||
                     (Controller_DW.speedConfig.EndSpeed != 0.0F));
      } else {
        reachGoal = (fabsf(Controller_U.VehicheInfo.currVelocity) < 0.05);
      }
    }

    if (reachGoal) {
      if (Controller_DW.goalIndex <= Controller_U.Controller_In.valid_num) {
        Controller_DW.nextGoalPose[0] = Controller_B.VectorConcatenate[(int32_T)
          Controller_DW.goalIndex - 1];
        Controller_DW.nextGoalPose[1] = Controller_B.VectorConcatenate[(int32_T)
          (Controller_DW.goalIndex + 1.0) - 1];
        Controller_DW.nextGoalPose[2] = Controller_B.VectorConcatenate[(int32_T)
          Controller_DW.goalIndex + 49];
        Controller_DW.nextGoalPose[3] = Controller_B.VectorConcatenate[(int32_T)
          (Controller_DW.goalIndex + 1.0) + 49];
        Controller_DW.nextGoalPose[4] = Controller_B.VectorConcatenate[(int32_T)
          Controller_DW.goalIndex + 99];
        Controller_DW.nextGoalPose[5] = Controller_B.VectorConcatenate[(int32_T)
          (Controller_DW.goalIndex + 1.0) + 99];
        Controller_DW.speedConfig.StartSpeed =
          Controller_U.VehicheInfo.currVelocity;
        Controller_DW.speedConfig.EndSpeed = Controller_DW.endSpeed;
        Controller_DW.goalIndex++;
        rtb_planNext = true;
      } else {
        for (i = 0; i < 6; i++) {
          Controller_DW.nextGoalPose[i] = 0.0F;
        }

        Controller_DW.speedConfig.StartSpeed = 0.0F;
        Controller_DW.speedConfig.EndSpeed = 0.0F;
        Controller_DW.finalReached = true;
      }
    }
  }

  /* Outputs for Enabled SubSystem: '<S2>/Subsystem' incorporates:
   *  EnablePort: '<S120>/Enable'
   */
  /* Logic: '<S2>/Logical Operator' */
  if (rtb_FixPtRelationalOperator || rtb_planNext) {
    boolean_T exitg1;
    boolean_T guard1 = false;

    /* MATLABSystem: '<S120>/MATLAB System' incorporates:
     *  MATLAB Function: '<S120>/MATLAB Function'
     *  MATLAB Function: '<S2>/Behavior Planner'
     */
    /*  Define total number of inputs */
    /*  Define total number of outputs */
    /* ------------------------------------------------------------------ */
    /* stepImpl Implement the algorithm of interpolating a spline */
    /*  Check if refDirections contains invalid values */
    /*  If the input poses are not new, use the previous output */
    rtb_FixPtRelationalOperator = false;
    rtb_planNext = true;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 6)) {
      if ((Controller_DW.nextGoalPose[i] ==
           Controller_DW.obj_m.RefPosesInternal[i]) || (rtIsNaNF
           (Controller_DW.nextGoalPose[i]) && rtIsNaNF
           (Controller_DW.obj_m.RefPosesInternal[i]))) {
        i++;
      } else {
        rtb_planNext = false;
        exitg1 = true;
      }
    }

    if (rtb_planNext) {
      rtb_FixPtRelationalOperator = true;
    }

    guard1 = false;
    if (rtb_FixPtRelationalOperator) {
      rtb_FixPtRelationalOperator = false;
      rtb_planNext = true;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 2)) {
        if (Controller_DW.obj_m.RefDirectionsInternal[i] == 1.0F) {
          i++;
        } else {
          rtb_planNext = false;
          exitg1 = true;
        }
      }

      if (rtb_planNext) {
        rtb_FixPtRelationalOperator = true;
      }

      if (rtb_FixPtRelationalOperator) {
        memcpy(&Controller_B.MATLABSystem_o1[0],
               &Controller_DW.obj_m.LastPosesOutput[0], 150U * sizeof(real32_T));

        /* MATLABSystem: '<S120>/MATLAB System' */
        memcpy(&Controller_B.MATLABSystem_o2[0],
               &Controller_DW.obj_m.LastDirectionsOutput[0], 50U * sizeof
               (real32_T));
        memcpy(&b_varargout_3[0], &Controller_DW.obj_m.LastCumLengthsOutput[0],
               50U * sizeof(real32_T));

        /* MATLABSystem: '<S120>/MATLAB System' */
        memcpy(&Controller_B.MATLABSystem_o4[0],
               &Controller_DW.obj_m.LastCurvaturesOutput[0], 50U * sizeof
               (real32_T));
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      for (i = 0; i < 6; i++) {
        Controller_DW.obj_m.RefPosesInternal[i] = Controller_DW.nextGoalPose[i];
      }

      /*  Smooth the path */
      Controller_DW.obj_m.RefDirectionsInternal[0] = 1.0F;

      /* MATLAB Function: '<S120>/MATLAB Function' incorporates:
       *  MATLAB Function: '<S2>/Behavior Planner'
       */
      tmp[0] = 1.0F;
      Controller_DW.obj_m.RefDirectionsInternal[1] = 1.0F;

      /* MATLAB Function: '<S120>/MATLAB Function' */
      tmp[1] = 1.0F;
      Controller_smoothPathSpline(Controller_DW.nextGoalPose, tmp,
        Controller_B.MATLABSystem_o1, Controller_B.MATLABSystem_o2,
        b_varargout_3, Controller_B.MATLABSystem_o4);

      /*  Optional outputs */
      /*  Store the outputs */
      memcpy(&Controller_DW.obj_m.LastPosesOutput[0],
             &Controller_B.MATLABSystem_o1[0], 150U * sizeof(real32_T));
      memcpy(&Controller_DW.obj_m.LastDirectionsOutput[0],
             &Controller_B.MATLABSystem_o2[0], 50U * sizeof(real32_T));
      memcpy(&Controller_DW.obj_m.LastCumLengthsOutput[0], &b_varargout_3[0],
             50U * sizeof(real32_T));
      memcpy(&Controller_DW.obj_m.LastCurvaturesOutput[0],
             &Controller_B.MATLABSystem_o4[0], 50U * sizeof(real32_T));
    }

    /* MATLABSystem: '<S120>/MATLAB System1' incorporates:
     *  MATLAB Function: '<S2>/Behavior Planner'
     *  MATLABSystem: '<S120>/MATLAB System'
     */
    if (Controller_DW.obj_mc.MaxSpeed != 20.0) {
      /* ------------------------------------------------------------------ */
      Controller_DW.obj_mc.MaxSpeed = 20.0;
    }

    /* ------------------------------------------------------------------ */
    Contr_VelocityProfiler_stepImpl(&Controller_DW.obj_mc,
      Controller_B.MATLABSystem_o2, b_varargout_3, Controller_B.MATLABSystem_o4,
      Controller_DW.speedConfig.StartSpeed, Controller_DW.speedConfig.EndSpeed,
      Controller_B.y);

    /* End of MATLABSystem: '<S120>/MATLAB System1' */

    /* MATLAB Function: '<S120>/Verify Velocities ' */
    rtb_FixPtRelationalOperator = true;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 50)) {
      if (rtIsNaNF(Controller_B.y[i]) || Controller_DW.finalReached) {
        rtb_FixPtRelationalOperator = false;
        memset(&Controller_B.y[0], 0, 50U * sizeof(real32_T));
        exitg1 = true;
      } else {
        i++;
      }
    }

    /* Assertion: '<S120>/Assertion' */
    utAssert(rtb_FixPtRelationalOperator);
  }

  /* End of Logic: '<S2>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S2>/Subsystem' */

  /* MATLABSystem: '<S1>/MATLAB System' incorporates:
   *  Inport: '<Root>/VehicheInfo'
   *  MATLABSystem: '<S120>/MATLAB System'
   */
  if (Controller_DW.obj.TunablePropsChanged) {
    Controller_DW.obj.TunablePropsChanged = false;

    /* ------------------------------------------------------------------ */
    /*  processTunedPropertiesImpl Perform actions when tunable  */
    /*  properties change between calls to the System object */
    if (Controller_DW.obj.tunablePropertyChanged[0] ||
        Controller_DW.obj.tunablePropertyChanged[1] ||
        Controller_DW.obj.tunablePropertyChanged[2]) {
      Controller_DW.obj.CurrentSegmentIndex = 1.0F;
      Controller_DW.obj.ClosestPointIndex = 1.0F;
    }

    Controller_DW.obj.tunablePropertyChanged[0] = false;
    Controller_DW.obj.tunablePropertyChanged[1] = false;
    Controller_DW.obj.tunablePropertyChanged[2] = false;
    Controller_DW.obj.tunablePropertyChanged[3] = false;
  }

  /* ------------------------------------------------------------------ */
  /*  Define total number of inputs for system with optional inputs */
  /* ------------------------------------------------------------------ */
  /* isSimulinkBlock Check if the system object in used in Simulink */
  /*  0 for MATLAB, 1 for Simulink */
  /* ------------------------------------------------------------------ */
  /*  Define total number of outputs for system with optional */
  /*  outputs */
  Con_HelperPathAnalyzer_stepImpl(&Controller_DW.obj,
    Controller_U.VehicheInfo.currPose, Controller_U.VehicheInfo.currVelocity,
    Controller_B.MATLABSystem_o1, Controller_B.MATLABSystem_o2,
    Controller_B.MATLABSystem_o4, Controller_B.y, refPose,
    &rtb_DataTypeConversion_idx_0, &rtb_DataTypeConversion_idx_1, &distToGoal,
    &t);

  /* MATLAB Function: '<S4>/Kinematic' incorporates:
   *  Inport: '<Root>/VehicheInfo'
   *  MATLABSystem: '<S1>/MATLAB System'
   */
  currPose[0] = Controller_U.VehicheInfo.currPose[0];
  currPose[1] = Controller_U.VehicheInfo.currPose[1];
  refPose[2] *= 0.0174532924F;
  Contro_angleUtilities_wrapTo2Pi(&refPose[2]);
  currPose[2] = 0.0174532924F * Controller_U.VehicheInfo.currPose[2];
  Contro_angleUtilities_wrapTo2Pi(&currPose[2]);
  if (rtb_DataTypeConversion_idx_1 == 1.0F) {
    currPose[0] = 0.3F * cosf(currPose[2]) + Controller_U.VehicheInfo.currPose[0];
    currPose[1] = 0.3F * sinf(currPose[2]) + Controller_U.VehicheInfo.currPose[1];
    distToGoal = currPose[0] - refPose[0];
    d_idx_1 = currPose[1] - refPose[1];
  } else {
    distToGoal = currPose[0] - refPose[0];
    d_idx_1 = currPose[1] - refPose[1];
  }

  b = (currPose[2] - refPose[2]) + 3.14159274F;
  Contro_angleUtilities_wrapTo2Pi(&b);
  if (rtb_DataTypeConversion_idx_1 == 1.0F) {
    distToGoal = -(atanf(-(distToGoal * sinf(refPose[2]) - cosf(refPose[2]) *
      d_idx_1) * 2.0F / (Controller_U.VehicheInfo.currVelocity + 1.0F)) + (b -
      3.14159274F));
  } else {
    if (rtb_DataTypeConversion_idx_1 == 1.0F) {
      rtb_Sum_idx_0 = 2.0;
    } else {
      rtb_Sum_idx_0 = 2.5;
    }

    distToGoal = atanf(-(distToGoal * sinf(refPose[2]) - cosf(refPose[2]) *
                         d_idx_1) * (real32_T)rtb_Sum_idx_0 /
                       (Controller_U.VehicheInfo.currVelocity - 1.0F)) + (b -
      3.14159274F);
  }

  d_idx_1 = 57.2957802F * distToGoal;
  if (rtIsNaNF(d_idx_1)) {
    distToGoal = (rtNaNF);
  } else if (d_idx_1 < 0.0F) {
    distToGoal = -1.0F;
  } else {
    distToGoal = (real32_T)(d_idx_1 > 0.0F);
  }

  /* Saturate: '<S3>/Saturation2' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Gain: '<S3>/Gain'
   *  MATLAB Function: '<S4>/Kinematic'
   *  Sum: '<S3>/Add'
   */
  rtb_Sum_idx_0 = floor(distToGoal * fminf(fabsf(d_idx_1), 30.0F) * 5.55555534F
                        + 750.0);

  /* Sum: '<S5>/Minus' incorporates:
   *  Inport: '<Root>/VehicheInfo'
   *  MATLABSystem: '<S1>/MATLAB System'
   */
  d_idx_1 = rtb_DataTypeConversion_idx_0 - Controller_U.VehicheInfo.currVelocity;

  /* SwitchCase: '<S11>/Switch Case' incorporates:
   *  MATLABSystem: '<S1>/MATLAB System'
   */
  distToGoal = truncf(rtb_DataTypeConversion_idx_1);
  if (rtIsNaNF(distToGoal) || rtIsInfF(distToGoal)) {
    distToGoal = 0.0F;
  } else {
    distToGoal = fmodf(distToGoal, 4.2949673E+9F);
  }

  switch (distToGoal < 0.0F ? -(int32_T)(uint32_T)-distToGoal : (int32_T)
          (uint32_T)distToGoal) {
   case 1:
    {
      int8_T rtb_Switch1_m;

      /* Outputs for IfAction SubSystem: '<S11>/Forward' incorporates:
       *  ActionPort: '<S13>/Action Port'
       */
      /* DiscreteIntegrator: '<S49>/Integrator' incorporates:
       *  DataTypeConversion: '<S11>/Data Type Conversion'
       */
      if ((t != 0.0F) || (Controller_DW.Integrator_PrevResetState_h != 0)) {
        Controller_DW.Integrator_DSTATE_i = 0.0F;
      }

      /* Sum: '<S58>/Sum' incorporates:
       *  DiscreteIntegrator: '<S49>/Integrator'
       *  Gain: '<S54>/Proportional Gain'
       */
      distToGoal = CONTROL_PARAM.KP * d_idx_1 +
        Controller_DW.Integrator_DSTATE_i;

      /* Saturate: '<S56>/Saturation' incorporates:
       *  DeadZone: '<S42>/DeadZone'
       */
      if (distToGoal > 20.0F) {
        /* Merge: '<S11>/Merge' */
        Controller_B.Merge = 20.0F;
        distToGoal -= 20.0F;
      } else {
        if (distToGoal < -20.0F) {
          /* Merge: '<S11>/Merge' */
          Controller_B.Merge = -20.0F;
        } else {
          /* Merge: '<S11>/Merge' */
          Controller_B.Merge = distToGoal;
        }

        if (distToGoal >= -20.0F) {
          distToGoal = 0.0F;
        } else {
          distToGoal -= -20.0F;
        }
      }

      /* End of Saturate: '<S56>/Saturation' */

      /* RelationalOperator: '<S40>/Relational Operator' incorporates:
       *  Constant: '<S40>/Clamping_zero'
       */
      rtb_FixPtRelationalOperator = (distToGoal != 0.0F);

      /* Switch: '<S40>/Switch1' incorporates:
       *  Constant: '<S40>/Clamping_zero'
       *  Constant: '<S40>/Constant'
       *  Constant: '<S40>/Constant2'
       *  RelationalOperator: '<S40>/fix for DT propagation issue'
       */
      if (distToGoal > 0.0F) {
        rtb_Switch1_m = 1;
      } else {
        rtb_Switch1_m = -1;
      }

      /* End of Switch: '<S40>/Switch1' */

      /* Gain: '<S46>/Integral Gain' */
      distToGoal = CONTROL_PARAM.KI * d_idx_1;

      /* Switch: '<S40>/Switch2' incorporates:
       *  Constant: '<S40>/Clamping_zero'
       *  Constant: '<S40>/Constant3'
       *  Constant: '<S40>/Constant4'
       *  RelationalOperator: '<S40>/fix for DT propagation issue1'
       */
      if (distToGoal > 0.0F) {
        n = 1;
      } else {
        n = -1;
      }

      /* Switch: '<S40>/Switch' incorporates:
       *  Constant: '<S40>/Constant1'
       *  Logic: '<S40>/AND3'
       *  RelationalOperator: '<S40>/Equal1'
       *  Switch: '<S40>/Switch2'
       */
      if (rtb_FixPtRelationalOperator && (rtb_Switch1_m == n)) {
        distToGoal = 0.0F;
      }

      /* Update for DiscreteIntegrator: '<S49>/Integrator' incorporates:
       *  DataTypeConversion: '<S11>/Data Type Conversion'
       *  Switch: '<S40>/Switch'
       */
      Controller_DW.Integrator_DSTATE_i += 0.02F * distToGoal;
      Controller_DW.Integrator_PrevResetState_h = (int8_T)(t != 0.0F);

      /* End of Outputs for SubSystem: '<S11>/Forward' */
    }
    break;

   case -1:
    {
      int8_T rtb_Switch1_m;

      /* Outputs for IfAction SubSystem: '<S11>/Reverse' incorporates:
       *  ActionPort: '<S14>/Action Port'
       */
      /* DiscreteIntegrator: '<S100>/Integrator' incorporates:
       *  DataTypeConversion: '<S11>/Data Type Conversion'
       */
      if ((t != 0.0F) || (Controller_DW.Integrator_PrevResetState != 0)) {
        Controller_DW.Integrator_DSTATE = 0.0F;
      }

      /* Sum: '<S109>/Sum' incorporates:
       *  DiscreteIntegrator: '<S100>/Integrator'
       *  Gain: '<S105>/Proportional Gain'
       */
      distToGoal = CONTROL_PARAM.KP * d_idx_1 + Controller_DW.Integrator_DSTATE;

      /* Saturate: '<S107>/Saturation' incorporates:
       *  DeadZone: '<S93>/DeadZone'
       */
      if (distToGoal > 20.0F) {
        /* Merge: '<S11>/Merge' */
        Controller_B.Merge = 20.0F;
        distToGoal -= 20.0F;
      } else {
        if (distToGoal < -20.0F) {
          /* Merge: '<S11>/Merge' */
          Controller_B.Merge = -20.0F;
        } else {
          /* Merge: '<S11>/Merge' */
          Controller_B.Merge = distToGoal;
        }

        if (distToGoal >= -20.0F) {
          distToGoal = 0.0F;
        } else {
          distToGoal -= -20.0F;
        }
      }

      /* End of Saturate: '<S107>/Saturation' */

      /* RelationalOperator: '<S91>/Relational Operator' incorporates:
       *  Constant: '<S91>/Clamping_zero'
       */
      rtb_FixPtRelationalOperator = (distToGoal != 0.0F);

      /* Switch: '<S91>/Switch1' incorporates:
       *  Constant: '<S91>/Clamping_zero'
       *  Constant: '<S91>/Constant'
       *  Constant: '<S91>/Constant2'
       *  RelationalOperator: '<S91>/fix for DT propagation issue'
       */
      if (distToGoal > 0.0F) {
        rtb_Switch1_m = 1;
      } else {
        rtb_Switch1_m = -1;
      }

      /* End of Switch: '<S91>/Switch1' */

      /* Gain: '<S97>/Integral Gain' */
      distToGoal = CONTROL_PARAM.KI * d_idx_1;

      /* Switch: '<S91>/Switch2' incorporates:
       *  Constant: '<S91>/Clamping_zero'
       *  Constant: '<S91>/Constant3'
       *  Constant: '<S91>/Constant4'
       *  RelationalOperator: '<S91>/fix for DT propagation issue1'
       */
      if (distToGoal > 0.0F) {
        n = 1;
      } else {
        n = -1;
      }

      /* Switch: '<S91>/Switch' incorporates:
       *  Constant: '<S91>/Constant1'
       *  Logic: '<S91>/AND3'
       *  RelationalOperator: '<S91>/Equal1'
       *  Switch: '<S91>/Switch2'
       */
      if (rtb_FixPtRelationalOperator && (rtb_Switch1_m == n)) {
        distToGoal = 0.0F;
      }

      /* Update for DiscreteIntegrator: '<S100>/Integrator' incorporates:
       *  DataTypeConversion: '<S11>/Data Type Conversion'
       *  Switch: '<S91>/Switch'
       */
      Controller_DW.Integrator_DSTATE += 0.02F * distToGoal;
      Controller_DW.Integrator_PrevResetState = (int8_T)(t != 0.0F);

      /* End of Outputs for SubSystem: '<S11>/Reverse' */
    }
    break;
  }

  /* End of SwitchCase: '<S11>/Switch Case' */

  /* Product: '<S10>/Multiply' incorporates:
   *  MATLABSystem: '<S1>/MATLAB System'
   */
  t = rtb_DataTypeConversion_idx_1 * Controller_B.Merge;

  /* Switch: '<S10>/Switch1' incorporates:
   *  Abs: '<S10>/Abs'
   *  Constant: '<S10>/Constant'
   *  Switch: '<S10>/Switch'
   */
  if (t > 0.0F) {
    rtb_Sum3 = 0.0;
    rtb_Sum2_e = t;
  } else {
    rtb_Sum3 = fabsf(t);
    rtb_Sum2_e = 0.0;
  }

  /* End of Switch: '<S10>/Switch1' */

  /* Gain: '<S3>/Gain1' incorporates:
   *  Sum: '<S1>/Add'
   *  Switch: '<S10>/Switch'
   */
  rtb_Sum3 = (rtb_Sum2_e - rtb_Sum3) * 10000.0;

  /* Saturate: '<S3>/Saturation1' */
  if (rtb_Sum3 > 10000.0) {
    rtb_Sum3 = 10000.0;
  } else if (rtb_Sum3 < -10000.0) {
    rtb_Sum3 = -10000.0;
  }

  rtb_Sum2_e = floor(rtb_Sum3);
  if (rtIsNaN(rtb_Sum2_e)) {
    i = 0;
  } else {
    i = (int32_T)fmod(rtb_Sum2_e, 65536.0);
  }

  /* Outport: '<Root>/Controller_Out' incorporates:
   *  DiscreteIntegrator: '<S134>/Discrete-Time Integrator'
   */
  Controller_Y.Controller_Out.timestamp =
    Controller_DW.DiscreteTimeIntegrator_DSTATE;

  /* Saturate: '<S3>/Saturation2' */
  if (rtIsNaN(rtb_Sum_idx_0)) {
    /* Outport: '<Root>/Controller_Out' */
    Controller_Y.Controller_Out.actuator_cmd[0] = 0;
  } else {
    /* Outport: '<Root>/Controller_Out' */
    Controller_Y.Controller_Out.actuator_cmd[0] = (int16_T)(int32_T)
      rtb_Sum_idx_0;
  }

  /* Outport: '<Root>/Controller_Out' incorporates:
   *  BusAssignment: '<S3>/Bus Assignment'
   *  Constant: '<S3>/Constant2'
   *  Saturate: '<S3>/Saturation1'
   */
  Controller_Y.Controller_Out.actuator_cmd[1] = (int16_T)(i < 0 ? (int32_T)
    (int16_T)-(int16_T)(uint16_T)-(real_T)i : i);
  Controller_Y.Controller_Out.actuator_cmd[2] = 0;
  Controller_Y.Controller_Out.actuator_cmd[3] = 0;

  /* Signum: '<S12>/Sign2' incorporates:
   *  MATLABSystem: '<S1>/MATLAB System'
   */
  if (rtIsNaNF(rtb_DataTypeConversion_idx_0)) {
    distToGoal = (rtNaNF);
  } else if (rtb_DataTypeConversion_idx_0 < 0.0F) {
    distToGoal = -1.0F;
  } else {
    distToGoal = (real32_T)(rtb_DataTypeConversion_idx_0 > 0.0F);
  }

  /* Signum: '<S12>/Sign1' incorporates:
   *  Inport: '<Root>/VehicheInfo'
   */
  if (rtIsNaNF(Controller_U.VehicheInfo.currVelocity)) {
    t = (rtNaNF);
  } else if (Controller_U.VehicheInfo.currVelocity < 0.0F) {
    t = -1.0F;
  } else {
    t = (real32_T)(Controller_U.VehicheInfo.currVelocity > 0.0F);
  }

  /* Assertion: '<S12>/Assertion' incorporates:
   *  Constant: '<S12>/Constant1'
   *  Constant: '<S12>/Constant4'
   *  Inport: '<Root>/VehicheInfo'
   *  Logic: '<S12>/AND1'
   *  Logic: '<S12>/AND3'
   *  Logic: '<S12>/NOT1'
   *  Logic: '<S12>/OR'
   *  MATLABSystem: '<S1>/MATLAB System'
   *  RelationalOperator: '<S12>/Equal1'
   *  RelationalOperator: '<S12>/Equal2'
   *  RelationalOperator: '<S12>/Equal3'
   *  RelationalOperator: '<S12>/Equal6'
   *  Signum: '<S12>/Sign1'
   *  Signum: '<S12>/Sign2'
   */
  utAssert(((!(rtb_DataTypeConversion_idx_1 != distToGoal)) ||
            (!(rtb_DataTypeConversion_idx_0 != 0.0F))) &&
           ((!(rtb_DataTypeConversion_idx_1 != t)) ||
            (!(Controller_U.VehicheInfo.currVelocity != 0.0F))));

  /* Assertion: '<S12>/Assertion1' incorporates:
   *  Constant: '<S12>/Constant2'
   *  Constant: '<S12>/Constant3'
   *  Logic: '<S12>/AND2'
   *  Logic: '<S12>/NOT'
   *  MATLABSystem: '<S1>/MATLAB System'
   *  RelationalOperator: '<S12>/Equal4'
   *  RelationalOperator: '<S12>/Equal5'
   */
  utAssert((!(rtb_DataTypeConversion_idx_1 != -1.0F)) ||
           (!(rtb_DataTypeConversion_idx_1 != 1.0F)));

  /* Update for UnitDelay: '<S118>/Delay Input1' incorporates:
   *  Inport: '<Root>/Controller_In'
   *
   * Block description for '<S118>/Delay Input1':
   *
   *  Store in Global RAM
   */
  Controller_DW.DelayInput1_DSTATE = Controller_U.Controller_In.timestamp;

  /* Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S134>/Constant'
   */
  Controller_DW.DiscreteTimeIntegrator_DSTATE += 20U;
}

/* Model initialize function */
void Controller_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    int32_T i;

    /* SystemInitialize for Enabled SubSystem: '<S2>/Subsystem' */
    /* Start for MATLABSystem: '<S120>/MATLAB System' */
    /* ------------------------------------------------------------------ */
    /*  Support name-value pair arguments when constructing object */
    /* ------------------------------------------------------------------ */
    /*  Only inherited and discrete sample time are supported for */
    /*  variable-size signal inputs */
    Controller_DW.obj_m.isInitialized = 1;

    /*  Define total number of inputs */
    /* ------------------------------------------------------------------ */
    /*  Validate inputs to the step method at initialization */
    /*  Validate refPoses */
    /*  Validate refDirections */
    /*  Input datatypes should be the same */
    /* ------------------------------------------------------------------ */
    /* setupImpl Perform one-time calculations */
    /*  Initialize  */
    for (i = 0; i < 6; i++) {
      Controller_DW.obj_m.RefPosesInternal[i] = (rtNaNF);
    }

    Controller_DW.obj_m.RefDirectionsInternal[0] = (rtNaNF);
    Controller_DW.obj_m.RefDirectionsInternal[1] = (rtNaNF);
    for (i = 0; i < 150; i++) {
      Controller_DW.obj_m.LastPosesOutput[i] = (rtNaNF);
    }

    /* Start for MATLABSystem: '<S120>/MATLAB System1' */
    /* ------------------------------------------------------------------ */
    /*  Support name-value pair arguments when constructing object */
    /* ------------------------------------------------------------------ */
    Controller_DW.obj_mc.MaxSpeed = 20.0;

    /* ------------------------------------------------------------------ */
    /*  Only inherited and discrete sample time are supported for */
    /*  variable-size signal inputs */
    /*  % Reuse catalog */
    Controller_DW.obj_mc.isInitialized = 1;

    /* ------------------------------------------------------------------ */
    /*  Validate inputs to the step method at initialization */
    /*  Validate directions */
    /*  Validate cumLengths */
    /*  Validate curvatures */
    /*  Validate startVelocity */
    /*  Validate endVelocity */
    /* ------------------------------------------------------------------ */
    /* setupImpl Initialize internal properties */
    for (i = 0; i < 50; i++) {
      /* Start for MATLABSystem: '<S120>/MATLAB System' */
      Controller_DW.obj_m.LastDirectionsOutput[i] = (rtNaNF);
      Controller_DW.obj_m.LastCumLengthsOutput[i] = (rtNaNF);
      Controller_DW.obj_m.LastCurvaturesOutput[i] = (rtNaNF);

      /* Start for MATLABSystem: '<S120>/MATLAB System1' */
      Controller_DW.obj_mc.LastVelocities[i] = 0.0F;
      Controller_DW.obj_mc.LastCumLengths[i] = 0.0F;
      Controller_DW.obj_mc.LastDirections[i] = 1.0F;
      Controller_DW.obj_mc.LastCurvatures[i] = 0.0F;
    }

    /* Start for MATLABSystem: '<S120>/MATLAB System1' */
    Controller_DW.obj_mc.LastStartVelocity = 0.0F;
    Controller_DW.obj_mc.LastEndVelocity = 0.0F;

    /* End of SystemInitialize for SubSystem: '<S2>/Subsystem' */
    emxInitStruct_HelperPathAnalyze(&Controller_DW.obj);

    /* Start for MATLABSystem: '<S1>/MATLAB System' */
    Controller_DW.obj.LastRefPoseOutput[0] = 0.0F;
    Controller_DW.obj.LastRefPoseOutput[1] = 0.0F;
    Controller_DW.obj.LastRefPoseOutput[2] = 0.0F;
    Controller_DW.obj.LastRefVelocityOutput = 0.0F;
    Controller_DW.obj.LastCurvatureOutput = 0.0F;
    Controller_DW.obj.LastDirectionOutput = 1.0F;

    /* ------------------------------------------------------------------ */
    Controller_DW.obj.isInitialized = 0;
    Controller_DW.obj.tunablePropertyChanged[0] = false;
    Controller_DW.obj.tunablePropertyChanged[1] = false;
    Controller_DW.obj.tunablePropertyChanged[2] = false;
    Controller_DW.obj.tunablePropertyChanged[3] = false;

    /* HelperPathAnalyzer Constructor  */
    /* ------------------------------------------------------------------ */
    Controller_SystemCore_setup(&Controller_DW.obj);
  }
}

/* Model terminate function */
void Controller_terminate(void)
{
  emxFreeStruct_HelperPathAnalyze(&Controller_DW.obj);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
