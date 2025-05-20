#ifndef __MC_COMMON_H
#define __MC_COMMON_H

#include <stdint.h>

#define _PI                         3.14159265359f
#define _PI_2                       1.57079632679f
#define _PI_3                        1.0471975512f
#define _3PI_2                      4.71238898038f
#define _SQRT3_2                    0.86602540378f

#define _SQRT3                      1.73205080757f
#define _SQRT3_2                    0.86602540378f
#define _1_SQRT3                    0.57735026919f
#define _2_SQRT3                    1.15470053838f


#define rad_s_to_rpm(x)         ((x) * 60.0f / (2.0f * _PI))
#define rpm_to_rad_s(x)         ((x) * (2.0f * _PI) / 60.0f)

#define deg_to_rad(y)           ((y) * _PI / 180.0f)
#define ABS(y)                  (((y) >= 0 ) ? (y) : -(y))

#define min(a,b)            (((a) < (b)) ? (a) : (b))
#define max(a,b)            (((a) > (b)) ? (a) : (b))

#define Tperiod             1

#define CCW                         ( 2 )                 /* 逆时针 */
#define CW                          ( 1 )  

#define err_rad                 deg_to_rad(5)
#endif 
