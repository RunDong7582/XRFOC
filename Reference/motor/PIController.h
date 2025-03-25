#ifndef _PIController_H
#define _PIController_H
#include "PIController.h"
#include "FocMath.h"
typedef struct
{
		iq_t integral;
		iq_t kp,ki,kc;
		iq_t maxValue,minValue;
		iq_t out,real,target,err;
}PIController_t;
void PIControllerStep(PIController_t *pi);
#endif
