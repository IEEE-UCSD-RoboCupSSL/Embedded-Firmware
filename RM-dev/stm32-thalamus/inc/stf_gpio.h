#ifndef __STF_GPIO_H
#define __STF_GPIO_H 


#include "stf_dependancy.h"

namespace stf {
    class GPIO {
    private:
        GPIO_TypeDef *GPIOx;
        uint16_t GPIO_Pin;
        logic_level curr_level = Low;



    public:
        GPIO(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
            this->GPIOx = GPIOx;
            this->GPIO_Pin = GPIO_Pin;
        }
        ~GPIO() {}

        void write(logic_level level);
        logic_level read(void);
        void toggle(void);

    };
}




#endif 
