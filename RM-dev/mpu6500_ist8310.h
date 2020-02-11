#ifndef __MPU6500_IST8310_H
#define __MPU6500_IST8310_H

#include "stf.h"


/* IMU interface*/  /* this (interface)class should resides in a different file, but due to the 
relatively smaller scale of this project and need for convenience, put it here instead to reduce # of files*/
/*
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
*/



/* MPU6500 + IST8310 IMU Source */
// although the class is named as MPU6500, but it really should be named as MPU6500_IST8310 which extends the MPU6500 class
// but for simplication, they get mixed up here

class MPU6500 {
// Citation: Content in this class is most referenced from the open source example imu code provided by Dji Robomaster
private:
    bool is_hardware_chip_select; //if hardware css is enabled, no software is needed to control the css pin
    stf::SPI *spi_bus_ptr;
    stf::GPIO *chip_select_ptr;
    void enable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::Low);}
    void disable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::High);}
    void write_reg(byte_t address, byte_t byte);
    byte_t read_reg(byte_t address);
    //void read_bytes(byte_t address, uint8_t num_bytes, char* buffer);

    byte_t id;
    enum GyroScale {_250dps, _500dps, _1000dps, _2000dps};
    enum AccelScale {_2g, _4g, _8g, _16g};
public:

    int16_t gyro_x, gyro_y, gyro_z;
    int16_t gyro_x_offset, gyro_y_offset, gyro_z_offset;
    
    int16_t accel_x, accel_y, accel_z;
    int16_t accel_x_offset, accel_y_offset, accel_z_offset;
    int16_t temperature;

    
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
    void measure_offset(int iter = 300);
    void read_data(void);

    void set_gyro_full_scale_range(GyroScale scale);
    void set_accel_full_scale_range(AccelScale scale);

    byte_t read_who_am_i_reg(void);
    


};



#endif
