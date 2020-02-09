#ifndef __STF_H
#define __STF_H

#include "stf_gpio.h"
#include "stf_systick.h"
#include "stf_usart.h"
#include "stf_i2c.h"
#include "stf_spi.h"
#include "stf_util.h"
#include "stf_timer.h"


//====================================================================//
/*For C and C++ mix programming*/
extern "C" {
	#include "main.h"
	
}
//====================================================================//


void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void * ptr);
void operator delete[](void * ptr);

namespace stf {
	extern int new_cnt;
	extern std::string endl;
}

#endif
