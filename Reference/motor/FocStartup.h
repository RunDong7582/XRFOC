#ifndef _FocStartup_H
#define _FocStartup_H
#include "FocMath.h"
typedef struct
{
		iq_t theta;
		iq_t omega;
	  iq_t thetaDiff;
		iq_t iq;
	  int state;
	  iq_t time;
	  int diffCnt;
		float err;
}ifStartup_t;
void IfStartupStep(ifStartup_t *pHandle,iq_t obTheta,iq_t sin_cos[]);
#endif
