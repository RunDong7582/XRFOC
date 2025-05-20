#ifndef __MC_STM32_CONF_H
#define __MC_STM32_CONF_H

#include "../XRFOC_Lib/common/mc_common.h"
#include "../XRFOC_Lib/include/mc_adaptor.h"
#include "../XRFOC_Lib/APP/app.h"

struct mc_adaptor_stm32_hw {
    struct mc_adaptor_i *adaptor;
};

int mc_adaptor_stm32_hw_init (struct mc_adaptor_stm32_hw *mcu);

#endif 
