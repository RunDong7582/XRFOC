#ifndef __MC_STM32_CONF_H
#define __MC_STM32_CONF_H

#include "../../include/mc_adaptor.h"

struct mc_adaptor_stm32_hw {
    struct mc_adaptor_i *adaptor;
};

int mc_adaptor_stm32_hw_init (struct mc_adaptor_stm32_hw *mcu);

#endif 