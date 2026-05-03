#ifndef _XSO_H_
#define _XSO_H_
#include <stddef.h>
#include <stdint.h>
#define _XSO_ML 250
typedef struct {
const char*_a;uint8_t _b,_c,_d;
const uint32_t*_e,*_f;uint32_t _g,_h,_i;
const uint8_t*_j;} _xso_p_t;
int _xso_b(const _xso_p_t*,char*,size_t);
#endif
