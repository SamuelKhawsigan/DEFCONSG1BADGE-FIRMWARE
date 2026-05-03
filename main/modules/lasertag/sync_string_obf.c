#include <stdio.h>
#include <string.h>
#include "sync_string_obf.h"
static uint32_t _xr(const uint8_t*d,size_t l){
uint32_t v=0x811C9DC5u;while(l--)v=(v^*d++)*0x01000193u;return v;}
static char _xt(uint8_t i){
return i<0x1A?(char)(0x41+i):i<0x34?(char)(0x47+i):
i<0x3E?(char)(i-4):i==0x3E?0x2B:0x2F;}
static void _xe(const uint8_t w[3],char q[4]){
q[0]=_xt(w[0]>>2);q[1]=_xt(((w[0]&3)<<4)|(w[1]>>4));
q[2]=_xt(((w[1]&0xF)<<2)|(w[2]>>6));q[3]=_xt(w[2]&0x3F);}
static int _xf(char*o,size_t n,const uint32_t*s,int z){
int k=0,c;for(c=0;c<z;c++){if(c)o[k++]=0x2C;
k+=snprintf(o+k,(int)n-k>0?(n-k):0,"%d%lu",c,(unsigned long)s[c]);}
return k;}
int _xso_b(const _xso_p_t*p,char*o,size_t n){
char _u[96],_v[96];
_xf(_u,sizeof(_u),p->_e,8);_xf(_v,sizeof(_v),p->_f,8);
int m=snprintf(o,n,"%s%c%d%c%d%c%d%c%s%c%s%c%lu%c%lu%c%lu",
p->_a,0x7C,(int)p->_b,0x7C,(int)p->_c,0x7C,(int)p->_d,0x7C,
_u,0x7C,_v,0x7C,(unsigned long)p->_g,0x7C,
(unsigned long)p->_h,0x7C,(unsigned long)p->_i);
if(m>0&&m+5<(int)n){uint8_t _q[256];
memcpy(_q,o,m);memcpy(_q+m,p->_j,6);
uint32_t v=_xr(_q,m+6);
uint8_t r[3]={(v>>0x10)&0xFF,(v>>8)&0xFF,v&0xFF};
char s[4];_xe(r,s);o[m]=0x7C;
memcpy(o+m+1,s,4);o[m+=5]=0;}return m;}
