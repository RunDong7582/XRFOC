#ifndef __MC_TIME_H
#define __MC_TIME_H

// #include "stm32f4xx.h"

static inline void mc_timebase_init(void)
{
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static inline uint32_t mc_time_us(void)
{
    mc_timebase_init();
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

#endif
