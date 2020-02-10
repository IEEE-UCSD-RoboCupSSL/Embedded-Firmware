#ifndef __MPU6500_IST8310_H
#define __MPU6500_IST8310_H

#include "stf.h"


/* IMU interface*/  /* this (interface)class should resides in a different file, but due to the 
relatively smaller scale of this project and need for convenience, put it here instead to reduce # of files*/
namespace DjiRM {
    class IMU {
    private:

    public:
        IMU() {

        }
        ~IMU() {

        }
    };
}


/* MPU6500 IMU Source */
class MPU6500 {
private:
    bool is_hardware_chip_select; //if hardware css is enabled, no software is needed to control the css pin
    stf::SPI *spi_bus_ptr;
    stf::GPIO *chip_select_ptr;
    void enable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::Low);}
    void disable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::High);}
    void write_reg(byte_t address, byte_t byte);
    byte_t read_reg(byte_t address);

    byte_t id;

public:
    
    MPU6500(stf::SPI& spi_bus) {
        this->spi_bus_ptr = &spi_bus;
        is_hardware_chip_select = true;
        spi_bus.set_txrx_timeout(66); //magic timeout value, not too important
    }
    MPU6500(stf::SPI& spi_bus, stf::GPIO& chip_select) {
        this->spi_bus_ptr = &spi_bus;
        this->chip_select_ptr = &chip_select;
        chip_select_ptr->write(stf::High);
        is_hardware_chip_select = false;
        spi_bus.set_txrx_timeout(66); //magic timeout value, not too important
    }
    ~MPU6500(void) {}

    byte_t init(void);

    byte_t read_who_am_i_reg(void);
    

};



#endif
