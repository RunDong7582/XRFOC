#ifndef _SVPWM_H
#define _SVPWM_H
#include "FocDataBase.h"
#include "FocMath.h"
#define SVPWM_COMPUTATIONAL_SECTOR    0
#define SVPWM_COMPUTATIONAL_HARMONIC  1
#define SVPWM_COMPUTATIONAL_METHOD    SVPWM_COMPUTATIONAL_HARMONIC
typedef struct
{
		iq_t vd,vq;
		iq_t ta,tb,tc;
		iq_t sector;
		iq_t vAlpha,vBeta;
		iq_t tx,ty;
		iq_t tAlpha,tBeta;
		iq_t vBus;
		iq_t sin_cos[2];
}ComponentsSvpwm_t;
void SvpwmStep(ComponentsSvpwm_t *pHandle);
void SvpwmTest(void);
#endif
