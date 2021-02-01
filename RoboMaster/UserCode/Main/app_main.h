#ifndef __stf_MAIN_H
#define __stf_MAIN_H

#include "stf.h"


//====================================================================//
/*Callback functions called by different thread task func in main.c*/ 
//omit "_callback" postfix to make it less intimidating

// equivalent to task0 if multitasking
extern "C" void setup(void);

// user may modify it to accept params if needed, 
extern "C" void defaultLoop(void);

extern "C" void blinkLEDLoop(void);

extern "C" void updatePIDLoop(void);

extern "C" void printInfoLoop(void);

extern "C" void actuatorsLoop(void);

extern "C" void sensorsLoop(void);

extern "C" void usbReadLoop(void);

extern "C" void usbWriteLoop(void);




/* For task2 ... task 3...  configure their settings in cubemx GUI 
 * and generate new code, 
 * then call the loop# func in the loop of startTask0#  
 */

//====================================================================//






#endif
