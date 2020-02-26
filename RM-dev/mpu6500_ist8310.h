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
    void mpu_i2c_write_reg(byte_t address, byte_t byte);
    byte_t mpu_i2c_read_reg(byte_t address);
    void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_address, uint8_t num_bytes);

    byte_t id;
    enum GyroScale {_250dps, _500dps, _1000dps, _2000dps};
    enum AccelScale {_2g, _4g, _8g, _16g};

    
public:

    int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
    int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
    
    int16_t accel_x = 0, accel_y = 0, accel_z = 0;
    int16_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;

    int16_t compass_x = 0, compass_y = 0, compass_z = 0;
    int16_t compass_x_offset = 0, compass_y_offset = 0, compass_z_offset = 0;

    double temperature = 0; // degree celsius


    
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
    void read_accel_data(void);
    void read_gyro_data(void);
    void read_temp_data(void);
    void read_compass_data(void);
    void read_data(void) {
        read_accel_data();
        read_gyro_data();
        read_temp_data();
        read_compass_data();
    }

    void set_gyro_full_scale_range(GyroScale scale);
    void set_accel_full_scale_range(AccelScale scale);

    byte_t read_who_am_i_reg(void);
    std::string data_string(void);

};



#endif
