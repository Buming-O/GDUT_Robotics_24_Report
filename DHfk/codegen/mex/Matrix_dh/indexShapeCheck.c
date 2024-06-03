/*
 * indexShapeCheck.c
 *
 * Code generation for function 'indexShapeCheck'
 *
 */

/* Include files */
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"

/* Variable Definitions */
static emlrtRSInfo c_emlrtRSI =
    {
        42,                /* lineNo */
        "indexShapeCheck", /* fcnName */
        "D:\\MATLAB_R2021b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\indexShapeCheck.m" /* pathName */
};

static emlrtRTEInfo emlrtRTEI =
    {
        122,           /* lineNo */
        5,             /* colNo */
        "errOrWarnIf", /* fName */
        "D:\\MATLAB_R2021b\\toolbox\\eml\\eml\\+coder\\+"
        "internal\\indexShapeCheck.m" /* pName */
};

/* Function Definitions */
void indexShapeCheck(const emlrtStack *sp, int32_T matrixSize)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &c_emlrtRSI;
  if (matrixSize == 1) {
    emlrtErrorWithMessageIdR2018a(&st, &emlrtRTEI,
                                  "Coder:FE:PotentialVectorVector",
                                  "Coder:FE:PotentialVectorVector", 0);
  }
}

/* End of code generation (indexShapeCheck.c) */
