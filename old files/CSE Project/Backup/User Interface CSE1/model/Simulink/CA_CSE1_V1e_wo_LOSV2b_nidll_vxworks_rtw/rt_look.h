/*
 * rt_look.h
 *
 * Real-Time Workshop code generation for Simulink model "CA_CSE1_V1e_wo_LOSV2b.mdl".
 *
 * Model Version              : 1.193
 * Real-Time Workshop version : 7.3  (R2009a)  15-Jan-2009
 * C source code generated on : Fri Sep 20 14:51:46 2013
 *
 * Target selection: nidll_vxworks.tlc
 *   Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_rt_look_h_
#define RTW_HEADER_rt_look_h_
#include "rtwtypes.h"
#ifdef DOINTERPSEARCH
#include <float.h>
#endif

#ifndef INTERP
# define INTERP(x,x1,x2,y1,y2)         ( (y1)+(((y2) - (y1))/((x2) - (x1)))*((x)-(x1)) )
#endif

#ifndef ZEROTECHNIQUE
#define ZEROTECHNIQUE

typedef enum {
  NORMAL_INTERP,
  AVERAGE_VALUE,
  MIDDLE_VALUE
} ZeroTechnique;

#endif

extern int_T rt_GetLookupIndex(const real_T *x, int_T xlen, real_T u) ;

#endif                                 /* RTW_HEADER_rt_look_h_ */
