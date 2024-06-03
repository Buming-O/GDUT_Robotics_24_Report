/*
 * _coder_Matrix_dh_api.c
 *
 * Code generation for function '_coder_Matrix_dh_api'
 *
 */

/* Include files */
#include "_coder_Matrix_dh_api.h"
#include "Matrix_dh.h"
#include "Matrix_dh_data.h"
#include "Matrix_dh_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T y[5]);

static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size);

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[2]);

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3]);

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *Link,
                             const char_T *identifier, struct0_T y[5]);

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct0_T u[5]);

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T *y_size);

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2]);

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2]);

static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *i,
                                 const char_T *identifier);

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[2]);

static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T *ret_size);

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2]);

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2]);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               struct0_T y[5])
{
  static const int32_T dims[2] = {1, 5};
  static const char_T *fieldNames[12] = {"name", "th", "dz", "dx", "alf", "az",
                                         "n",    "o",  "a",  "p",  "R",   "A"};
  emlrtMsgIdentifier thisId;
  int32_T i;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtCTX)sp, parentId, u, 12,
                         (const char_T **)&fieldNames[0], 2U, (void *)&dims[0]);
  for (i = 0; i < 5; i++) {
    thisId.fIdentifier = "name";
    c_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 0,
                                                      (const char_T *)"name")),
                       &thisId, y[i].name);
    thisId.fIdentifier = "th";
    y[i].th =
        d_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtCTX)sp, u, i, 1, (const char_T *)"th")),
                           &thisId);
    thisId.fIdentifier = "dz";
    y[i].dz =
        d_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtCTX)sp, u, i, 2, (const char_T *)"dz")),
                           &thisId);
    thisId.fIdentifier = "dx";
    y[i].dx =
        d_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtCTX)sp, u, i, 3, (const char_T *)"dx")),
                           &thisId);
    thisId.fIdentifier = "alf";
    y[i].alf =
        d_emlrt_marshallIn(sp,
                           emlrtAlias(emlrtGetFieldR2017b(
                               (emlrtCTX)sp, u, i, 4, (const char_T *)"alf")),
                           &thisId);
    thisId.fIdentifier = "az";
    e_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 5,
                                                      (const char_T *)"az")),
                       &thisId, y[i].az);
    thisId.fIdentifier = "n";
    f_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 6,
                                                      (const char_T *)"n")),
                       &thisId, y[i].n.data, &y[i].n.size[0]);
    thisId.fIdentifier = "o";
    f_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 7,
                                                      (const char_T *)"o")),
                       &thisId, y[i].o.data, &y[i].o.size[0]);
    thisId.fIdentifier = "a";
    f_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 8,
                                                      (const char_T *)"a")),
                       &thisId, y[i].a.data, &y[i].a.size[0]);
    thisId.fIdentifier = "p";
    f_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 9,
                                                      (const char_T *)"p")),
                       &thisId, y[i].p.data, &y[i].p.size[0]);
    thisId.fIdentifier = "R";
    g_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 10,
                                                      (const char_T *)"R")),
                       &thisId, y[i].R.data, y[i].R.size);
    thisId.fIdentifier = "A";
    h_emlrt_marshallIn(sp,
                       emlrtAlias(emlrtGetFieldR2017b((emlrtCTX)sp, u, i, 11,
                                                      (const char_T *)"A")),
                       &thisId, y[i].A.data, y[i].A.size);
  }
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const real_T u_data[],
                                          const int32_T u_size)
{
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T b_i;
  int32_T i;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&u_size, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (b_i = 0; b_i < u_size; b_i++) {
    pData[i] = u_data[b_i];
    i++;
  }
  emlrtAssign(&y, m);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, char_T y[2])
{
  j_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[3])
{
  l_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *Link,
                             const char_T *identifier, struct0_T y[5])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(Link), &thisId, y);
  emlrtDestroyArray(&Link);
}

static const mxArray *emlrt_marshallOut(const emlrtStack *sp,
                                        const struct0_T u[5])
{
  static const int32_T iv1[2] = {1, 2};
  static const int32_T i1 = 3;
  static const char_T *sv[12] = {"name", "th", "dz", "dx", "alf", "az",
                                 "n",    "o",  "a",  "p",  "R",   "A"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *f_y;
  const mxArray *g_y;
  const mxArray *h_y;
  const mxArray *i_y;
  const mxArray *m;
  const mxArray *y;
  const struct0_T *r;
  real_T b_u_data[16];
  real_T u_data[9];
  real_T c_u;
  real_T u_idx_1;
  real_T u_idx_2;
  real_T *pData;
  int32_T iv[2];
  int32_T b_i;
  int32_T i;
  int32_T i2;
  int32_T loop_ub;
  int32_T u_size_idx_0;
  int32_T u_size_idx_1;
  char_T b_u[2];
  y = NULL;
  iv[0] = 1;
  iv[1] = 5;
  emlrtAssign(&y,
              emlrtCreateStructArray(2, &iv[0], 12, (const char_T **)&sv[0]));
  for (i = 0; i < 5; i++) {
    r = &u[i];
    b_u[0] = r->name[0];
    b_u[1] = r->name[1];
    b_y = NULL;
    m = emlrtCreateCharArray(2, &iv1[0]);
    emlrtInitCharArrayR2013a((emlrtCTX)sp, 2, m, &b_u[0]);
    emlrtAssign(&b_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"name", b_y, 0);
    c_u = r->th;
    c_y = NULL;
    m = emlrtCreateDoubleScalar(c_u);
    emlrtAssign(&c_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"th", c_y, 1);
    c_u = r->dz;
    d_y = NULL;
    m = emlrtCreateDoubleScalar(c_u);
    emlrtAssign(&d_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"dz", d_y, 2);
    c_u = r->dx;
    e_y = NULL;
    m = emlrtCreateDoubleScalar(c_u);
    emlrtAssign(&e_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"dx", e_y, 3);
    c_u = r->alf;
    f_y = NULL;
    m = emlrtCreateDoubleScalar(c_u);
    emlrtAssign(&f_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"alf", f_y, 4);
    c_u = r->az[0];
    u_idx_1 = r->az[1];
    u_idx_2 = r->az[2];
    g_y = NULL;
    m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    pData[0] = c_u;
    pData[1] = u_idx_1;
    pData[2] = u_idx_2;
    emlrtAssign(&g_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"az", g_y, 5);
    emlrtSetFieldR2017b(y, i, (const char_T *)"n",
                        b_emlrt_marshallOut(r->n.data, r->n.size[0]), 6);
    emlrtSetFieldR2017b(y, i, (const char_T *)"o",
                        b_emlrt_marshallOut(r->o.data, r->o.size[0]), 7);
    emlrtSetFieldR2017b(y, i, (const char_T *)"a",
                        b_emlrt_marshallOut(r->a.data, r->a.size[0]), 8);
    emlrtSetFieldR2017b(y, i, (const char_T *)"p",
                        b_emlrt_marshallOut(r->p.data, r->p.size[0]), 9);
    u_size_idx_0 = r->R.size[0];
    u_size_idx_1 = r->R.size[1];
    loop_ub = r->R.size[0] * r->R.size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      u_data[i2] = r->R.data[i2];
    }
    h_y = NULL;
    iv[0] = u_size_idx_0;
    iv[1] = u_size_idx_1;
    m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    i2 = 0;
    for (loop_ub = 0; loop_ub < u_size_idx_1; loop_ub++) {
      for (b_i = 0; b_i < u_size_idx_0; b_i++) {
        pData[i2] = u_data[b_i + u_size_idx_0 * loop_ub];
        i2++;
      }
    }
    emlrtAssign(&h_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"R", h_y, 10);
    u_size_idx_0 = r->A.size[0];
    u_size_idx_1 = r->A.size[1];
    loop_ub = r->A.size[0] * r->A.size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_u_data[i2] = r->A.data[i2];
    }
    i_y = NULL;
    iv[0] = u_size_idx_0;
    iv[1] = u_size_idx_1;
    m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(m);
    i2 = 0;
    for (loop_ub = 0; loop_ub < u_size_idx_1; loop_ub++) {
      for (b_i = 0; b_i < u_size_idx_0; b_i++) {
        pData[i2] = b_u_data[b_i + u_size_idx_0 * loop_ub];
        i2++;
      }
    }
    emlrtAssign(&i_y, m);
    emlrtSetFieldR2017b(y, i, (const char_T *)"A", i_y, 11);
  }
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T *y_size)
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2])
{
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y_data[], int32_T y_size[2])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *i,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(i), &thisId);
  emlrtDestroyArray(&i);
  return y;
}

static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, char_T ret[2])
{
  static const int32_T dims[2] = {1, 2};
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"char",
                          false, 2U, (void *)&dims[0]);
  emlrtImportCharArrayR2015b((emlrtCTX)sp, src, &ret[0], 2);
  emlrtDestroyArray(&src);
}

static real_T k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims = 3;
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 1U, (void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T *ret_size)
{
  static const int32_T dims = 4;
  const boolean_T b = true;
  emlrtCheckVsBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                            false, 1U, (void *)&dims, &b, ret_size);
  emlrtImportArrayR2015b((emlrtCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {3, 3};
  int32_T iv[2];
  const boolean_T bv[2] = {true, true};
  emlrtCheckVsBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                            false, 2U, (void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  emlrtImportArrayR2015b((emlrtCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret_data[], int32_T ret_size[2])
{
  static const int32_T dims[2] = {4, 4};
  int32_T iv[2];
  const boolean_T bv[2] = {true, true};
  emlrtCheckVsBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                            false, 2U, (void *)&dims[0], &bv[0], &iv[0]);
  ret_size[0] = iv[0];
  ret_size[1] = iv[1];
  emlrtImportArrayR2015b((emlrtCTX)sp, src, &ret_data[0], 8, false);
  emlrtDestroyArray(&src);
}

void Matrix_dh_api(const mxArray *const prhs[2], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  struct0_T Link[5];
  real_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "Link", Link);
  i = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "i");
  /* Invoke the target function */
  Matrix_dh(&st, Link, i);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(&st, Link);
}

/* End of code generation (_coder_Matrix_dh_api.c) */
