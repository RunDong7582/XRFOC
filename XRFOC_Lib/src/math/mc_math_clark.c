#include "mc_math_clark.h"

void Clarke_Transform(float ia, float ib, float *i_alpha, float *i_beta) {
    *i_alpha = ia;
    *i_beta = (ia + 2*ib) * ONE_BY_SQRT3;
}