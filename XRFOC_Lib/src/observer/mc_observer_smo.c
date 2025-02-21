#include "mc_observer_smo.h"
// observer.c
void SlidingMode_Update(float v_alpha, float v_beta, float i_alpha, float i_beta) {
    // 滑模观测器核心计算
    // ...
    est_angle = atan2(z_beta, z_alpha);
}