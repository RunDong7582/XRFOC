#ifndef __MC_MATH_PID_H
#define __MC_MATH_PID_H

typedef struct {
    void (*init)(void);
    void (*set_duty)(uint8_t phase, float duty);
} PWM_Interface;

#endif // MC_MATH_PID_H