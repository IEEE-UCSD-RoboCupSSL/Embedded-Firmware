#ifndef __STF_UTIL_H
#define __STF_UTIL_H


#include "stf_dependancy.h"

#define Pi 3.14159265358979323846

#define range(x,y) x,y
#define from_range(x,y) x,y
#define to_range(x,y) x,y



/*=============================FORMAT STRINGS==============================*/
#define FORMAT_PARAMS const char* format, ...	
#define FORMAT_STRINGS(buffer_pointer) do{ \
	va_list args; \
    va_start(args, format); \
    vsprintf(buffer_pointer, format, args);\
	va_end(args); \
}while(0)

/*=========================================================================*/






namespace stf {
    //config write/read mode for byte
    inline uint8_t set_byte_msb_zero(uint8_t byte) {
        return byte & 0x7F; //0x7F = 1000,0000 in binary
    }
    inline uint8_t set_byte_msb_one(uint8_t byte) {
        return byte | 0x80; // 0x80 = 1000,0000 in binary
    }

    // map
    template <typename T1, typename T2, typename T3>
    T1 map(T1 x, from_range(T2 fmin, T2 fmax), to_range(T3 tmin, T3 tmax)) {
        if(x < fmin || x > fmax) {
            exception("map: invalid param: x is not within from_range");
            return 0;
        }
        return T1(((double(x) - double(fmin)) / (double(fmax) - double(fmin))) 
                * (double(tmax) - double(tmin)) + double(tmin));
    }
    


    // wrap some optimized basic arm_math func
    inline float fast_cos(float x) {return arm_cos_f32(x);}
    inline float fast_sin(float x) {return arm_sin_f32(x);}
      


    // fast inverse square: 1/(x)^(1/2)
    float fast_inv_sqrt(float x); 

    // radian to degree
    template <typename T>
    T radian_to_degree_template(T radian) {
        return radian * 180.00f / Pi;
    }
    inline float radian_to_degree(float radian) {return radian_to_degree_template(radian);}
    inline double radian_to_degree(double radian) {return radian_to_degree_template(radian);}
    inline float rtd(float r) {return radian_to_degree(r);}
    inline double rtd(double r) {return radian_to_degree(r);}

    // degree to radian
    template <typename T>
    T degree_to_radian_template(T degree) {
            return degree * Pi / 180.00f;
    }
    inline float degree_to_radian(float degree) {return degree_to_radian_template(degree);}
    inline double degree_to_radian(double degree) {return degree_to_radian_template(degree);}
    inline float dtr(float d) {return degree_to_radian(d);}
    inline double dtr(double d) {return degree_to_radian(d);}

}


#endif
