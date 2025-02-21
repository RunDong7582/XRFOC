#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

typedef int (*mc_adaptor_periphal_pwm_fn_t) (void *adaptor);
typedef int (*mc_adaptor_periphal_adc_fn_t) (void *adaptor);
typedef int (*mc_adaptor_periphal_timer_fn_t) (void *adaptor);


#endif // MC_ADAPTOR_H