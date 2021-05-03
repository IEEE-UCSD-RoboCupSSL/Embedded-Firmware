#include "IMU/mpu6500_ist8310.hpp"
#include <string.h>

// Code referenced from bsp_imu.c from the example code provided by Dji

/*********************Register Map**********************/
// Reference: Register Map macros are copy-pasted from the 
//            open source example code provided by Dji RoboMaster
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)
#define MPU6500_SMPLRT_DIV          (0x19)
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)
#define MPU6500_LP_ACCEL_ODR        (0x1E)
#define MPU6500_MOT_THR             (0x1F)
#define MPU6500_FIFO_EN             (0x23)
#define MPU6500_I2C_MST_CTRL        (0x24)
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)
#define MPU6500_I2C_MST_STATUS      (0x36)
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)
#define MPU6500_FIFO_R_W            (0x74)
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x70
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)


// IST8310 internal reg address
#define IST8310_ADDRESS 0x0E
#define IST8310_DEVICE_ID_A 0x10

// IST8310 register map. For details see IST8310 datasheet
#define IST8310_WHO_AM_I 0x00
#define IST8310_R_CONFA 0x0A
#define IST8310_R_CONFB 0x0B
#define IST8310_R_MODE 0x02

#define IST8310_R_XL 0x03
#define IST8310_R_XM 0x04
#define IST8310_R_YL 0x05
#define IST8310_R_YM 0x06
#define IST8310_R_ZL 0x07
#define IST8310_R_ZM 0x08

#define IST8310_AVGCNTL 0x41
#define IST8310_PDCNTL 0x42

#define IST8310_ODR_MODE 0x01 //sigle measure mode
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


void MPU6500_IST8310::write_reg(byte_t address, byte_t byte) {
    enable_chip_select();
    // MSB = 0 for write
    address = set_byte_msb_zero(address);
    spi_bus_ptr->tranceive(address);
    spi_bus_ptr->tranceive(byte);
    disable_chip_select();
}
byte_t MPU6500_IST8310::read_reg(byte_t address) {
    byte_t rtn;
    enable_chip_select();
    // MSB = 1 for read
    address = set_byte_msb_one(address);
    spi_bus_ptr->tranceive(address);
    rtn = spi_bus_ptr->tranceive(address); //this address byte serve as a dummy byte
    disable_chip_select();
    return rtn;  
}

void MPU6500_IST8310::mpu_i2c_write_reg(byte_t address, byte_t byte) {
    write_reg(MPU6500_I2C_SLV1_CTRL, 0x00); // turn off first
    delay(10);
    write_reg(MPU6500_I2C_SLV1_REG, address);
    delay(10);
    write_reg(MPU6500_I2C_SLV1_DO, byte);
    delay(10);
    /* turn on slave 1 with one byte transmitting */
    write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    delay(10);
}

byte_t MPU6500_IST8310::mpu_i2c_read_reg(byte_t address) {
    byte_t ret;
    //write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    //delay(10);
    write_reg(MPU6500_I2C_SLV4_REG, address);
    delay(10);
    write_reg(MPU6500_I2C_SLV4_CTRL, 0x80); // enable reading
    delay(10);
    ret = read_reg(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay(10);
    return ret;
}

/* (function copy pasted from dji sample code)
 *initialize the MPU6500 I2C Slave 0 for I2C reading.
 *device_address: slave device address, Address[6:0]
 */
void MPU6500_IST8310::mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_address, uint8_t num_bytes)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    write_reg(MPU6500_I2C_SLV1_ADDR, device_address);
    delay(2);
    write_reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    delay(2);
    write_reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    delay(2);

    /* use slave0,auto read data */
    write_reg(MPU6500_I2C_SLV0_ADDR, device_address | 0x80);
    delay(2);
    write_reg(MPU6500_I2C_SLV0_REG, reg_address);
    delay(2);

    /* every eight mpu6500 internal samples one i2c master read */
    write_reg(MPU6500_I2C_SLV4_CTRL, 0x03);
    delay(2);
    /* enable slave 0 and 1 access delay */
    //write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x00);
    write_reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    delay(2);
    /* enable slave 1 auto transmit */
    write_reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    delay(6); 
    /* enable slave 0 with data_num bytes reading */
    write_reg(MPU6500_I2C_SLV0_CTRL, 0x80 | num_bytes);
    delay(2);
}



byte_t MPU6500_IST8310::init(void) {

    /***** init MPU6500 *****/

    delay(500); //delay to wait for imu pre-heat
    id = read_reg(MPU6500_WHO_AM_I);
    delay(1);

    //Reset Sequence
    write_reg(MPU6500_PWR_MGMT_1, 0x80); //0x80 == [1000,0000]b | reset
    delay(100);
    write_reg(MPU6500_SIGNAL_PATH_RESET, 0x00); //0x00 == [0000,0000]b | reset all signal pat
    delay(100);

    //Config device
    write_reg(MPU6500_PWR_MGMT_1, 0x03); //0x03 == [0000,0011]b | Auto select best available clock source
    delay(1);
    write_reg(MPU6500_PWR_MGMT_2, 0x00); //0x00 == [0000,0000]b | Enable both accelerometer and gyro
    delay(1);
    write_reg(MPU6500_CONFIG, 0x04); /*0x04 == [0000,0100]b | FreeSync & FIFO modes disabled, 
                                                              DLPF(digital low pass filter) config bit is 4
                                       Gyro[bandwidth=20Hz, Delay=9.9ms, Fs=1KHz], 
                                       temperature sensor[bandwidth=20Hz, Delay=8.3ms] */
    delay(1);
    write_reg(MPU6500_GYRO_CONFIG, 0x18); //0x18 == [0001,1000]b | Gyro scale = 2000dps
    delay(1);
    write_reg(MPU6500_ACCEL_CONFIG, 0x10); //0x10 == [0001,0000]b | Accel scale = +-8g
    delay(1);
    write_reg(MPU6500_ACCEL_CONFIG_2, 0x02); /*0x02 == [0000,0010]b | 
                                Acc DLPF [bandwidth=92Hz, Delay=7.8ms, Noise Density=220ug/rtHz, Rate=1KHz] */
    delay(1);
    // Enable I2C supplementary bus for IST8310
    write_reg(MPU6500_USER_CTRL, 0x20); 
      /*0x20 == [0010,0000]b | I2C_MST_EN set to 1, Enable the I2C Master I/F 
                                          module; pins ES_DA and ES_SCL are isolated
                                          from pins SDA/SDI and SCL/ SCLK. */
    delay(10);

    /***** init IST8310 *****/
    byte_t byte_read;
    std::string debug;
	// enable iic smaster mode 
    write_reg(MPU6500_USER_CTRL, 0x30); //0x30 == [0011,0000]b
    delay(10);
	
    // enable iic 400khz 
    write_reg(MPU6500_I2C_MST_CTRL, 0x0D);  // 0x0D == [0000,1101]b 
    delay(10);

    // turn on slave 1 for ist write and slave 4 to ist read 
    write_reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    delay(10);

    write_reg(MPU6500_I2C_SLV4_ADDR, IST8310_ADDRESS | 0x80);
    delay(10);

    mpu_i2c_write_reg(IST8310_R_CONFB, 0x01); 
    delay(10);
    byte_read = mpu_i2c_read_reg(IST8310_WHO_AM_I);
    if(byte_read != IST8310_DEVICE_ID_A) {
        debug = "IST8310 WHO_AM_I reg not matching: " + std::to_string(byte_read);
        exception(debug.c_str());
    }
    delay(10);

	/* soft reset */
    mpu_i2c_write_reg(IST8310_R_CONFB, 0x01); 
    delay(10);

	/* config as ready mode to access register */
    mpu_i2c_write_reg(IST8310_R_CONFA, 0x00); 
    delay(10);
    byte_read = mpu_i2c_read_reg(IST8310_R_CONFA); 
    if(byte_read != 0x00) {
        debug = "Config IST8310_R_CONFA failed: " + std::to_string(byte_read);
        exception(debug.c_str());
    }
    delay(10);

	/* normal state, no int */
    mpu_i2c_write_reg(IST8310_R_CONFB, 0x00);
    delay(100);
    byte_read = mpu_i2c_read_reg(IST8310_R_CONFB); 
    if(byte_read != 0x00) {
        debug = "Config IST8310_R_CONFB failed: " + std::to_string(byte_read);
        exception(debug.c_str());
    }
    delay(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    mpu_i2c_write_reg(IST8310_AVGCNTL, 0x24); //[0010,0100]b
    delay(10);
    byte_read = mpu_i2c_read_reg(IST8310_AVGCNTL); 
    if(byte_read != 0x24) {
        debug = "Config IST8310_AVGCNTL failed: " + std::to_string(byte_read);
        exception(debug.c_str());
    }
    delay(10);

    /* Set/Reset pulse duration setup,normal mode */
    mpu_i2c_write_reg(IST8310_PDCNTL, 0xC0);
    delay(10);
    mpu_i2c_write_reg(IST8310_PDCNTL, 0xC0);
    delay(10);
    byte_read = mpu_i2c_read_reg(IST8310_PDCNTL); 
    if(byte_read != 0xC0) {
        debug = "Config IST8310_PDCNTL failed: " + std::to_string(byte_read);
        exception(debug.c_str());
    }
    delay(10);

    /* turn off slave1 & slave 4 */
    write_reg(MPU6500_I2C_SLV1_CTRL, 0x00);
    delay(10);
    write_reg(MPU6500_I2C_SLV4_CTRL, 0x00);
    delay(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    delay(10);

//    offset = measure_offset(0);
//    serial << "Offset: " << (float)offset << stf::endl;

    return id;
}


void MPU6500_IST8310::collect_accel_data(void) {
    accel_x = read_reg(MPU6500_ACCEL_XOUT_H) << 8 | read_reg(MPU6500_ACCEL_XOUT_L);
    accel_y = read_reg(MPU6500_ACCEL_YOUT_H) << 8 | read_reg(MPU6500_ACCEL_YOUT_L); 
    accel_z = read_reg(MPU6500_ACCEL_ZOUT_H) << 8 | read_reg(MPU6500_ACCEL_ZOUT_L); 
    accel_x -= accel_x_offset; accel_y -= accel_y_offset; accel_z -= accel_z_offset;
}

void MPU6500_IST8310::collect_gyro_data(void) {
    gyro_x = read_reg(MPU6500_GYRO_XOUT_H) << 8 | read_reg(MPU6500_GYRO_XOUT_L);
    gyro_y = read_reg(MPU6500_GYRO_YOUT_H) << 8 | read_reg(MPU6500_GYRO_YOUT_L);
    gyro_z = read_reg(MPU6500_GYRO_ZOUT_H) << 8 | read_reg(MPU6500_GYRO_ZOUT_L);
    gyro_x -= gyro_x_offset; gyro_y -= gyro_y_offset; gyro_z -= gyro_z_offset;
} 

void MPU6500_IST8310::collect_temp_data(void) {
    int16_t temp = read_reg(MPU6500_TEMP_OUT_H) << 8 | read_reg(MPU6500_TEMP_OUT_L);
    temperature = temp / 333.87f + 21;
}

void MPU6500_IST8310::collect_compass_data(void) {
    mag_x = read_reg(MPU6500_EXT_SENS_DATA_00) << 8 | read_reg(MPU6500_EXT_SENS_DATA_01);
    mag_y = read_reg(MPU6500_EXT_SENS_DATA_02) << 8 | read_reg(MPU6500_EXT_SENS_DATA_03);
    mag_z = read_reg(MPU6500_EXT_SENS_DATA_04) << 8 | read_reg(MPU6500_EXT_SENS_DATA_05);
    mag_x -= mag_x_offset; mag_y -= mag_y_offset; mag_z -= mag_z_offset;
}



void MPU6500_IST8310::measure_offset(int iter) {
    gyro_x_offset = 0; gyro_y_offset = 0; gyro_z_offset = 0;
    accel_x_offset = 0; accel_y_offset = 0; accel_z_offset = 0;
    mag_x_offset = 0; mag_y_offset = 0; mag_z_offset = 0;
    for(int i = 0; i < iter; i++) {    
        collect_accel_data();
        collect_gyro_data();
        collect_compass_data();
        gyro_x_offset += gyro_x; gyro_y_offset += gyro_y; gyro_z_offset += gyro_z;
        accel_x_offset += accel_x; accel_y_offset += accel_y;  accel_z_offset += accel_z;
        mag_x_offset += mag_x; mag_y_offset += mag_y; mag_z_offset += mag_z;
        delay(5);
    }
    gyro_x_offset /= iter; gyro_y_offset /= iter; gyro_z_offset /= iter;
    accel_x_offset /= iter; accel_y_offset /= iter; accel_z_offset /= iter;
    mag_x_offset /= iter; mag_y_offset /= iter; mag_z_offset /= iter;
}

void MPU6500_IST8310::set_gyro_full_scale_range(GyroScale scale) {
   delay(1);
   if(scale == _250dps) write_reg(MPU6500_GYRO_CONFIG, 0x00);
   if(scale == _500dps) write_reg(MPU6500_GYRO_CONFIG, 0x08);
   if(scale == _1000dps) write_reg(MPU6500_GYRO_CONFIG, 0x10);
   if(scale == _2000dps) write_reg(MPU6500_GYRO_CONFIG, 0x18);
   delay(1);
}
void MPU6500_IST8310::set_accel_full_scale_range(AccelScale scale) {
   delay(1);
   if(scale == _2g) write_reg(MPU6500_ACCEL_CONFIG, 0x00);
   if(scale == _4g) write_reg(MPU6500_ACCEL_CONFIG, 0x08);
   if(scale == _8g) write_reg(MPU6500_ACCEL_CONFIG, 0x10);
   if(scale == _16g) write_reg(MPU6500_ACCEL_CONFIG, 0x18);
   delay(1);
}


//std::string MPU6500_IST8310::data_string(void) {
//    char str[150];
//    sprintf(str, "Accel[%6d, %6d, %6d] Gyro[%6d, %6d, %6d] Compass[%6d, %6d, %6d] temp[%6lf]",
//            accel_x, accel_y, accel_z,
//            gyro_x, gyro_y, gyro_z,
//            mag_x, mag_y, mag_z,
//            temperature);
//    return std::string(str);
//}


MPU6500_IST8310::data MPU6500_IST8310::read_accel_data(void) {
	collect_accel_data();
	data d;
	d.x = this->accel_x;
	d.y = this->accel_y;
	d.z = this->accel_z;
	return d;
}

MPU6500_IST8310::data MPU6500_IST8310::read_gyro_data(void) {
	collect_gyro_data();
	data d;
	d.x = this->gyro_x;
	d.y = this->gyro_y;
	d.z = this->gyro_z;
	return d;
}

MPU6500_IST8310::data MPU6500_IST8310::read_compass_data(void) {
	collect_compass_data();
	data d;
	d.x = this->mag_x;
	d.y = this->mag_y;
	d.z = this->mag_z;
	return d;
}

double MPU6500_IST8310::read_compass_angle(void) {
	collect_compass_data();
	double angle, degrees;
	angle = atan2(mag_y,  mag_x);
	degrees = angle*180.00 / 3.14159265;
	return degrees ;
}

double MPU6500_IST8310::read_temp_data(void) {
	collect_temp_data();
	return temperature;
}
