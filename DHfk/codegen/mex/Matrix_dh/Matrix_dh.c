/*
 * Matrix_dh.c
 *
 * Code generation for function 'Matrix_dh'
 *
 */

/* Include files */
#include "Matrix_dh.h"
#include "Matrix_dh_types.h"
#include "indexShapeCheck.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = {
    18,          /* lineNo */
    "Matrix_dh", /* fcnName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m" /* pathName */
};

static emlrtRSInfo b_emlrtRSI = {
    19,          /* lineNo */
    "Matrix_dh", /* fcnName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m" /* pathName */
};

static emlrtRSInfo d_emlrtRSI = {
    26,    /* lineNo */
    "cat", /* fcnName */
    "D:\\MATLAB_R2021b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                      */
};

static emlrtRSInfo e_emlrtRSI = {
    100,        /* lineNo */
    "cat_impl", /* fcnName */
    "D:\\MATLAB_R2021b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pathName
                                                                      */
};

static emlrtDCInfo emlrtDCI = {
    6,           /* lineNo */
    12,          /* colNo */
    "Matrix_dh", /* fName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m", /* pName */
    1                /* checkKind */
};

static emlrtBCInfo emlrtBCI = {
    1,           /* iFirst */
    5,           /* iLast */
    6,           /* lineNo */
    12,          /* colNo */
    "Link",      /* aName */
    "Matrix_dh", /* fName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m", /* pName */
    0                /* checkKind */
};

static emlrtRTEInfo b_emlrtRTEI = {
    275,                   /* lineNo */
    27,                    /* colNo */
    "check_non_axis_size", /* fName */
    "D:\\MATLAB_R2021b\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m" /* pName */
};

static emlrtBCInfo b_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    18,          /* lineNo */
    42,          /* colNo */
    "Link(i).a", /* aName */
    "Matrix_dh", /* fName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    18,          /* lineNo */
    27,          /* colNo */
    "Link(i).o", /* aName */
    "Matrix_dh", /* fName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m", /* pName */
    0                /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = {
    -1,          /* iFirst */
    -1,          /* iLast */
    18,          /* lineNo */
    12,          /* colNo */
    "Link(i).n", /* aName */
    "Matrix_dh", /* fName */
    "C:"
    "\\Users\\owen1\\Desktop\\研一课程\\机器人学\\hw2\\Robotics2024Report\\DHfk"
    "\\Matrix_dh.m", /* pName */
    0                /* checkKind */
};

/* Function Definitions */
void Matrix_dh(const emlrtStack *sp, struct0_T Link[5], real_T i)
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T tmp_data[16];
  real_T e_Link[9];
  real_T C;
  real_T Ca;
  real_T S;
  real_T Sa;
  int32_T b_Link;
  int32_T c_Link;
  int32_T d_Link;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  /*  Caculate the D-H Matrix */
  /*  global Link; */
  if (i != (int32_T)muDoubleScalarFloor(i)) {
    emlrtIntegerCheckR2012b(i, &emlrtDCI, (emlrtCTX)sp);
  }
  if (((int32_T)i < 1) || ((int32_T)i > 5)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)i, 1, 5, &emlrtBCI, (emlrtCTX)sp);
  }
  Sa = Link[(int32_T)i - 1].th;
  C = muDoubleScalarCos(Sa);
  S = muDoubleScalarSin(Sa);
  Sa = Link[(int32_T)i - 1].alf;
  Ca = muDoubleScalarCos(Sa);
  Sa = muDoubleScalarSin(Sa);
  /* distance between zi and zi-1 */
  /* distance between xi and xi-1 */
  Link[(int32_T)i - 1].n.size[0] = 4;
  Link[(int32_T)i - 1].n.data[0] = C;
  Link[(int32_T)i - 1].n.data[1] = S;
  Link[(int32_T)i - 1].n.data[2] = 0.0;
  Link[(int32_T)i - 1].n.data[3] = 0.0;
  Link[(int32_T)i - 1].o.size[0] = 4;
  Link[(int32_T)i - 1].o.data[0] = -S * Ca;
  Link[(int32_T)i - 1].o.data[1] = C * Ca;
  Link[(int32_T)i - 1].o.data[2] = Sa;
  Link[(int32_T)i - 1].o.data[3] = 0.0;
  Link[(int32_T)i - 1].a.size[0] = 4;
  Link[(int32_T)i - 1].a.data[0] = S * Sa;
  Link[(int32_T)i - 1].a.data[1] = -C * Sa;
  Link[(int32_T)i - 1].a.data[2] = Ca;
  Link[(int32_T)i - 1].a.data[3] = 0.0;
  Sa = Link[(int32_T)i - 1].dx;
  Ca = Link[(int32_T)i - 1].dz;
  Link[(int32_T)i - 1].p.size[0] = 4;
  Link[(int32_T)i - 1].p.data[0] = Sa * C;
  Link[(int32_T)i - 1].p.data[1] = Sa * S;
  Link[(int32_T)i - 1].p.data[2] = Ca;
  Link[(int32_T)i - 1].p.data[3] = 1.0;
  st.site = &emlrtRSI;
  indexShapeCheck(&st, Link[(int32_T)i - 1].n.size[0]);
  st.site = &emlrtRSI;
  indexShapeCheck(&st, Link[(int32_T)i - 1].o.size[0]);
  st.site = &emlrtRSI;
  indexShapeCheck(&st, Link[(int32_T)i - 1].a.size[0]);
  b_Link = Link[(int32_T)i - 1].n.size[0];
  c_Link = Link[(int32_T)i - 1].o.size[0];
  d_Link = Link[(int32_T)i - 1].a.size[0];
  if (1 > b_Link) {
    emlrtDynamicBoundsCheckR2012b(1, 1, b_Link, &d_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[0] = Link[(int32_T)i - 1].n.data[0];
  if (2 > b_Link) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &d_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[1] = Link[(int32_T)i - 1].n.data[1];
  if (3 > b_Link) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &d_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[2] = Link[(int32_T)i - 1].n.data[2];
  if (1 > c_Link) {
    emlrtDynamicBoundsCheckR2012b(1, 1, c_Link, &c_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[3] = Link[(int32_T)i - 1].o.data[0];
  if (2 > c_Link) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &c_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[4] = Link[(int32_T)i - 1].o.data[1];
  if (3 > c_Link) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &c_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[5] = Link[(int32_T)i - 1].o.data[2];
  if (1 > d_Link) {
    emlrtDynamicBoundsCheckR2012b(1, 1, d_Link, &b_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[6] = Link[(int32_T)i - 1].a.data[0];
  if (2 > d_Link) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &b_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[7] = Link[(int32_T)i - 1].a.data[1];
  if (3 > d_Link) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &b_emlrtBCI, (emlrtCTX)sp);
  }
  e_Link[8] = Link[(int32_T)i - 1].a.data[2];
  Link[(int32_T)i - 1].R.size[0] = 3;
  Link[(int32_T)i - 1].R.size[1] = 3;
  for (d_Link = 0; d_Link < 9; d_Link++) {
    Link[(int32_T)i - 1].R.data[d_Link] = e_Link[d_Link];
  }
  st.site = &b_emlrtRSI;
  b_st.site = &d_emlrtRSI;
  c_st.site = &e_emlrtRSI;
  if (Link[(int32_T)i - 1].o.size[0] != Link[(int32_T)i - 1].n.size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if (Link[(int32_T)i - 1].a.size[0] != Link[(int32_T)i - 1].n.size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  if (Link[(int32_T)i - 1].p.size[0] != Link[(int32_T)i - 1].n.size[0]) {
    emlrtErrorWithMessageIdR2018a(&c_st, &b_emlrtRTEI,
                                  "MATLAB:catenate:matrixDimensionMismatch",
                                  "MATLAB:catenate:matrixDimensionMismatch", 0);
  }
  b_Link = Link[(int32_T)i - 1].n.size[0];
  c_Link = Link[(int32_T)i - 1].n.size[0];
  for (d_Link = 0; d_Link < c_Link; d_Link++) {
    tmp_data[d_Link] = Link[(int32_T)i - 1].n.data[d_Link];
  }
  c_Link = Link[(int32_T)i - 1].o.size[0];
  for (d_Link = 0; d_Link < c_Link; d_Link++) {
    tmp_data[d_Link + b_Link] = Link[(int32_T)i - 1].o.data[d_Link];
  }
  c_Link = Link[(int32_T)i - 1].a.size[0];
  for (d_Link = 0; d_Link < c_Link; d_Link++) {
    tmp_data[d_Link + b_Link * 2] = Link[(int32_T)i - 1].a.data[d_Link];
  }
  c_Link = Link[(int32_T)i - 1].p.size[0];
  for (d_Link = 0; d_Link < c_Link; d_Link++) {
    tmp_data[d_Link + b_Link * 3] = Link[(int32_T)i - 1].p.data[d_Link];
  }
  Link[(int32_T)i - 1].A.size[0] = b_Link;
  Link[(int32_T)i - 1].A.size[1] = 4;
  c_Link = b_Link * 4;
  for (d_Link = 0; d_Link < c_Link; d_Link++) {
    Link[(int32_T)i - 1].A.data[d_Link] = tmp_data[d_Link];
  }
}

/* End of code generation (Matrix_dh.c) */
