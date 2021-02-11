#ifndef __MPU6500_IST8310_H
#define __MPU6500_IST8310_H

#include "stf.h"




class MPU6500_IST8310 {
// Citation: implementations of this class partially references the open source example imu code provided by Dji Robomaster
private:
    bool is_hardware_chip_select; //if hardware css is enabled, no software is needed to control the css pin
    stf::SPI *spi_bus_ptr;
    stf::GPIO *chip_select_ptr;
    byte_t id;
    enum GyroScale {_250dps, _500dps, _1000dps, _2000dps};
    enum AccelScale {_2g, _4g, _8g, _16g};


    void enable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::Low);}
    void disable_chip_select(void) {if(is_hardware_chip_select == false) chip_select_ptr->write(stf::High);}
    void write_reg(byte_t address, byte_t byte);
    byte_t read_reg(byte_t address);
    void mpu_i2c_write_reg(byte_t address, byte_t byte);
    byte_t mpu_i2c_read_reg(byte_t address);
    void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_address, uint8_t num_bytes);
    void measure_offset(int iter = 300);

    void collect_accel_data(void);
    void collect_gyro_data(void);
    void collect_temp_data(void);
    void collect_compass_data(void);
//    void collect_all_data(void) {
//        collect_accel_data();
//        collect_gyro_data();
//        collect_temp_data();
//        collect_compass_data();
//    }




    int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
    int16_t gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
    
    int16_t accel_x = 0, accel_y = 0, accel_z = 0;
    int16_t accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;

    int16_t mag_x = 0, mag_y = 0, mag_z = 0;
    int16_t mag_x_offset = 0, mag_y_offset = 0, mag_z_offset = 0;

    double temperature = 0; // degree celsius

public:
    
    struct data {
    	int16_t x, y, z;
    	std::string to_string(void) {
			char str[25];
			sprintf(str, "[%6d, %6d, %6d]", x, y, z);
			return std::string(str);
    	}
    };


    MPU6500_IST8310(stf::SPI& spi_bus) {
        this->spi_bus_ptr = &spi_bus;
        is_hardware_chip_select = true;
        spi_bus.set_txrx_timeout(66); //magic timeout value, not too important
    }
    MPU6500_IST8310(stf::SPI& spi_bus, stf::GPIO& chip_select) {
        this->spi_bus_ptr = &spi_bus;
        this->chip_select_ptr = &chip_select;
        chip_select_ptr->write(stf::High);
        is_hardware_chip_select = false;
        spi_bus.set_txrx_timeout(66); //magic timeout value, not too important
    }
    ~MPU6500_IST8310(void) {}

    // default:  Gyro scale = +-2000dps, Accel scale = +-8g 
    byte_t init(void);
    byte_t init(stf::GPIO& ist8310_reset) {
        ist8310_reset.write(stf::Low); // low resets
        stf::delay(100);
        ist8310_reset.write(stf::High); // High sets
        return init();
    }
    
    void calibrate(int iter = 300) {measure_offset(iter);}
    data read_accel_data(void);
    data read_gyro_data(void);
    data read_compass_data(void);
    double read_temp_data(void);


    void set_gyro_full_scale_range(GyroScale scale);
    void set_accel_full_scale_range(AccelScale scale);

    byte_t read_who_am_i_reg(void);
    // std::string data_string(void);

};



#endif
