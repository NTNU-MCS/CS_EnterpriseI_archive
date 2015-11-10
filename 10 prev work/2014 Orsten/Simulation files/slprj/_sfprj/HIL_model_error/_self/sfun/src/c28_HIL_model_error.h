#ifndef __c28_HIL_model_error_h__
#define __c28_HIL_model_error_h__

/* Include files */
#include "sfc_sf.h"
#include "sfc_mex.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_c28_ResolvedFunctionInfo
#define typedef_c28_ResolvedFunctionInfo

typedef struct {
  const char * context;
  const char * name;
  const char * dominantType;
  const char * resolved;
  uint32_T fileTimeLo;
  uint32_T fileTimeHi;
  uint32_T mFileTimeLo;
  uint32_T mFileTimeHi;
} c28_ResolvedFunctionInfo;

#endif                                 /*typedef_c28_ResolvedFunctionInfo*/

#ifndef typedef_SFc28_HIL_model_errorInstanceStruct
#define typedef_SFc28_HIL_model_errorInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c28_sfEvent;
  boolean_T c28_isStable;
  boolean_T c28_doneDoubleBufferReInit;
  uint8_T c28_is_active_c28_HIL_model_error;
} SFc28_HIL_model_errorInstanceStruct;

#endif                                 /*typedef_SFc28_HIL_model_errorInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c28_HIL_model_error_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c28_HIL_model_error_get_check_sum(mxArray *plhs[]);
extern void c28_HIL_model_error_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
