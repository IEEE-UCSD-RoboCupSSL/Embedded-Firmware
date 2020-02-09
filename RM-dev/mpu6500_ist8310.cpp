#include "mpu6500_ist8310.h"


/*********************Register Map**********************/
#define MPU6500_WHO_AM_I 0x75


/*
                       _oo0oo_
                      o8888888o
                      88" . "88
                      (| -_- |)
                      0\  =  /0
                    ___/`---'\___
                  .' \\|     |// '.
                 / \\|||  :  |||// \
                / _||||| -:- |||||- \
               |   | \\\  -  /// |   |
               | \_|  ''\---/''  |_/ |
               \  .-\__  '-'  ___/-. /
             ___'. .'  /--.--\  `. .'___
          ."" '<  `.___\_<|>_/___.' >' "".
         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
         \  \ `_.   \_ __\ /__ _/   .-` /  /
     =====`-.____`.___ \_____/___.-`___.-'=====
                       `=---='
*/


using namespace stf;


void MPU6500::write_reg(byte_t address, byte_t byte) {
    enable_chip_select();
    // MSB = 0 for write
    address = set_byte_msb_zero(address);
    spi_bus_ptr->tranceive(address);
    spi_bus_ptr->tranceive(byte);
    disable_chip_select();
}
byte_t MPU6500::read_reg(byte_t address) {
    byte_t rtn;
    enable_chip_select();
    // MSB = 1 for read
    address = set_byte_msb_one(address);
    spi_bus_ptr->tranceive(address);
    rtn = spi_bus_ptr->tranceive(address); //this address byte serve as a dummy byte
    disable_chip_select();
    return rtn;  
}

byte_t MPU6500::init(void) {
    delay(500, RTOS); //delay to wait for imu pre-heat
    id = read_reg(MPU6500_WHO_AM_I);

    return id;
}