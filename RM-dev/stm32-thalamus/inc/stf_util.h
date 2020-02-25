#ifndef __STF_UTIL_H
#define __STF_UTIL_H


#include "stf_dependancy.h"



inline uint8_t set_byte_msb_zero(uint8_t byte) {
    return byte & 0x7F; //0x7F = 1000,0000 in binary
}

inline uint8_t set_byte_msb_one(uint8_t byte) {
    return byte | 0x80; // 0x80 = 1000,0000 in binary
}



#endif