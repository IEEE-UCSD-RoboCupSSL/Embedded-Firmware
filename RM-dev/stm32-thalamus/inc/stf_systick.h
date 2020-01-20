#ifndef __STF_SYSTICK_H
#define __STF_SYSTICK_H

#include "stf_dependancy.h"


namespace stf {
    uint32_t millis(void);
    uint32_t micros(void);
    void delay_us(uint32_t microseconds); 
    void delay(uint32_t milliseconds, delay_mode mode = RTOS);
}

#endif // !__stf_SYSTICK_H
