#ifndef _FOCSAMPLE_H
#define _FOCSAMPLE_H
#include "FocMath.h"
typedef struct
{
	int aCurrentOffset;
	int bCurrentOffset;
	int cCurrentOffset;
	iq_t aCurrent,bCurrent,cCurrent;
	iq_t iAlpha,iBeta;
	iq_t iD,iQ;
	
}FocSample_t;
typedef struct
{
	int aCurrentOffset;
	int bCurrentOffset;
	int cCurrentOffset;
	float aCurrent,bCurrent,cCurrent;
	float iAlpha,iBeta;
	float iD,iQ;
	
}FocSample_f;
void CurrentSampleInit(FocSample_t * pHandle);
void CurrentSampleStep(FocSample_t * pHandle,iq_t sin_cos[2]);
void CurrentSampleStepf(FocSample_f * pHandle,float sin_cos[2]);
void AdcSampleTest(void);
#endif
