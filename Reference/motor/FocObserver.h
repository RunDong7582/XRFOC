#ifndef _FocObserver_H
#define _FocObserver_H
#include "FocMath.h"
#include "PIController.h"
typedef struct
{
		iq_t alphaCurreantHat,betaCurreantHat;
		iq_t alphaErr,betaErr;
		iq_t A,B;
		iq_t alphaEMF,betaEMF;
		iq_t alphaZ,betaZ;
		iq_t h,lpfK;
		iq_t pllKp,pllKi;
		iq_t period;
		iq_t sin_cos[2];
		iq_t oemga,theta;
		PIController_t pi;
}ObserverSmc_t;
void ObserverSmcInit(ObserverSmc_t* pHandle);
void ObserverSmcStep(ObserverSmc_t* pHandle,iq_t vAlpha,iq_t vBeta,iq_t realAlphaCurrent,iq_t realBetaCurrent);

//typedef struct
//{
//		float alphaCurreantHat,betaCurreantHat;
//		float alphaErr,betaErr;
//		float A,B;
//		float alphaEMF,betaEMF;
//		float alphaZ,betaZ;
//		float h,lpfK;
//		float pllKp,pllKi;
//		float period;
//		float sin_cos[2];
//		float oemga,theta;
//		PIController_t pi;
//}ObserverSmc_t;
//void ObserverSmcInit(ObserverSmc_t* pHandle);
//void ObserverSmcStep(ObserverSmc_t* pHandle,float vAlpha,float vBeta,float realAlphaCurrent,float realBetaCurrent);
#endif
