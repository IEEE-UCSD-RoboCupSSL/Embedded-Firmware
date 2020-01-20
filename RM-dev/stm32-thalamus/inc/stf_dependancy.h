#ifndef __STF_DEPENDANCY_H
#define __STF_DEPENDANCY_H


/*========================================================================*/
#define DEVICE_FAMILY_STM32F4
/*========================================================================*/

/*========================================================================*/
    /*C includes*/
extern "C" {

#ifdef DEVICE_FAMILY_STM32F1
    #include "stm32f1xx_hal.h"
#endif

#ifdef DEVICE_FAMILY_STM32F3
    #include "stm32f3xx_hal.h"
#endif

#ifdef DEVICE_FAMILY_STM32F4
    #include "stm32f4xx_hal.h"
#endif

#ifdef DEVICE_FAMILY_STM32F7
    #include "stm32f7xx_hal.h"
#endif

    #include "cmsis_os.h"
}
/*========================================================================*/


/*========================================================================*/
    /*C++ includes*/

#include <iostream>
#include <atomic>
#include <cstring>
#include <vector>
#include <sstream>

/*========================================================================*/

namespace stf {
    enum logic_level : int {High, Low};
    enum periph_status : int
    {	
        NotReady,
        Initialized,
        InProgress,
        Completed,
        TimeOut,
        Error
    };
    enum periph_mode {
        Polling,
        Interrupt,
        DMA // direct memory access
    };
    enum delay_mode {
        HAL,
        RTOS
    };
}

typedef uint8_t byte_t;



__weak void exception(std::string str);


#endif
