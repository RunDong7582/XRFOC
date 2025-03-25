#ifndef _FOCMATH_H
#define _FOCMATH_H
typedef long iq_t;
#define Q24MAX       (iq_t)(0xFFFFFF)
#define Q16MAX       (iq_t)(0xFFFF)
#define Q15MAX       (iq_t)(32768)
#define Q18MAX       (iq_t)(0x3FFFF)
#define QnMpy(A,B,n) ((iq_t)(A * B) >> n)
#define QnDiv(A,B,n) (((iq_t)(A << n) / B))
#define Q24Mpy(A,B)   QnMpy(A,B,24)
#define Q24Div(A,B)   QnDiv(A,B,24) 
#define Q18Mpy(A,B)   QnMpy(A,B,18)
#define Q18Div(A,B)   QnDiv(A,B,18) 
//#define Q16Mpy(A,B)   QnMpy((iq_t)A,(iq_t)B,16)
//#define Q16Div(A,B)   QnDiv(A,B,16)
#define Q15Mpy(A,B)   QnMpy((iq_t)A,(iq_t)B,15)
#define Q15Div(A,B)   QnDiv(A,B,15)
#define Q24ToF(A)     (A / (1.0f * Q24MAX)) 
#define Q18ToF(A)     (A / (1.0f * Q18MAX)) 
#define Q16ToF(A)     (A / (1.0f * Q16MAX))
#define Q15ToF(A)     (A / (1.0f * Q15MAX)) 
#define Q24(A)       (iq_t)(A * Q24MAX)
#define Q18(A)       (iq_t)(A * Q18MAX)
//#define Q16(A)       (iq_t)((float)Q16MAX *(float)A)
#define Q15(A)       (iq_t)((float)Q15MAX *(float)A)
#define Q16Mod(A)    (iq_t)(A % Q16MAX)
void Q24SinCos( iq_t q24Angle,iq_t * q24_sin_cos);
void Q18SinCos( iq_t q18Angle,iq_t * q18_sin_cos);
void Q16SinCos( iq_t q16Angle,iq_t * q16_sin_cos);
void Q15SinCos( iq_t q15Angle,iq_t * q15_sin_cos);
iq_t QSprt( iq_t M);

#define QGlobal(A)       Q16(A)
#define QMpyGlobal(A,B)  Q16Mpy(A,B)
#define QDivGlobal(A,B)  Q16Div(A,B)
#define QMaxGlobal       Q16MAX
#define QToFGlobal(A)    Q16ToF(A)  
#endif