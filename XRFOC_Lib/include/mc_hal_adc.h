#ifndef __MC_HAL_ADC_H
#define __MC_HAL_ADC_H

typedef struct {
    void (*init)(void);
    float (*read_current)(uint8_t phase);
} ADC_Interface;

#endif // MC_HAL_ADC_H