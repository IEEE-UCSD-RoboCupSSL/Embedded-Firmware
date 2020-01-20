#include "stf_gpio.h"

using namespace stf;

void GPIO::write(logic_level level) {
    if(level == High)  {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
    }
	else {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
    }
    curr_level = level;
}

logic_level GPIO::read(void) {
    return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET ? High : Low;
}

void GPIO::toggle(void) {
    if(curr_level == High) {
    	this->write(Low);
    }
    else {
        this->write(High);
    }
}
