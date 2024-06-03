/*
 * Matrix_dh_terminate.c
 *
 * Code generation for function 'Matrix_dh_terminate'
 *
 */

/* Include files */
#include "Matrix_dh_terminate.h"
#include "Matrix_dh_data.h"
#include "_coder_Matrix_dh_mex.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void Matrix_dh_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void Matrix_dh_terminate(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (Matrix_dh_terminate.c) */
