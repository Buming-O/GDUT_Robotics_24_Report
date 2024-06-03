/*
 * Matrix_dh_types.h
 *
 * Code generation for function 'Matrix_dh'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T_4
#define struct_emxArray_real_T_4
struct emxArray_real_T_4 {
  real_T data[4];
  int32_T size[1];
};
#endif /* struct_emxArray_real_T_4 */
#ifndef typedef_emxArray_real_T_4
#define typedef_emxArray_real_T_4
typedef struct emxArray_real_T_4 emxArray_real_T_4;
#endif /* typedef_emxArray_real_T_4 */

#ifndef struct_emxArray_real_T_3x3
#define struct_emxArray_real_T_3x3
struct emxArray_real_T_3x3 {
  real_T data[9];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_3x3 */
#ifndef typedef_emxArray_real_T_3x3
#define typedef_emxArray_real_T_3x3
typedef struct emxArray_real_T_3x3 emxArray_real_T_3x3;
#endif /* typedef_emxArray_real_T_3x3 */

#ifndef struct_emxArray_real_T_4x4
#define struct_emxArray_real_T_4x4
struct emxArray_real_T_4x4 {
  real_T data[16];
  int32_T size[2];
};
#endif /* struct_emxArray_real_T_4x4 */
#ifndef typedef_emxArray_real_T_4x4
#define typedef_emxArray_real_T_4x4
typedef struct emxArray_real_T_4x4 emxArray_real_T_4x4;
#endif /* typedef_emxArray_real_T_4x4 */

#ifndef typedef_struct0_T
#define typedef_struct0_T
typedef struct {
  char_T name[2];
  real_T th;
  real_T dz;
  real_T dx;
  real_T alf;
  real_T az[3];
  emxArray_real_T_4 n;
  emxArray_real_T_4 o;
  emxArray_real_T_4 a;
  emxArray_real_T_4 p;
  emxArray_real_T_3x3 R;
  emxArray_real_T_4x4 A;
} struct0_T;
#endif /* typedef_struct0_T */

/* End of code generation (Matrix_dh_types.h) */
