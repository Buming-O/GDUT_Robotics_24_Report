/*
 * Matrix_dh_initialize.c
 *
 * Code generation for function 'Matrix_dh_initialize'
 *
 */

/* Include files */
#include "Matrix_dh_initialize.h"
#include "Matrix_dh_data.h"
#include "_coder_Matrix_dh_mex.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void Matrix_dh_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mex_InitInfAndNan();
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (Matrix_dh_initialize.c) */
