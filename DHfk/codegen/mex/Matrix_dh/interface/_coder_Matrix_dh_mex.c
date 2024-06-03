/*
 * _coder_Matrix_dh_mex.c
 *
 * Code generation for function '_coder_Matrix_dh_mex'
 *
 */

/* Include files */
#include "_coder_Matrix_dh_mex.h"
#include "Matrix_dh_data.h"
#include "Matrix_dh_initialize.h"
#include "Matrix_dh_terminate.h"
#include "_coder_Matrix_dh_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void Matrix_dh_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                           const mxArray *prhs[2])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        9, "Matrix_dh");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "Matrix_dh");
  }
  /* Call the function. */
  Matrix_dh_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&Matrix_dh_atexit);
  /* Module initialization. */
  Matrix_dh_initialize();
  /* Dispatch the entry-point. */
  Matrix_dh_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  Matrix_dh_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2021a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_Matrix_dh_mex.c) */
