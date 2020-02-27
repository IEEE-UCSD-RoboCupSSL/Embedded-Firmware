#include "app_main.h"
#include "dji_m2006_motor.h"
#include "mpu6500_ist8310.h"

#include <iostream>
#include <vector>
#include <sstream>
#include <string>


using namespace stf;
using namespace std;

GPIO green_led(Green_LED_GPIO_Port, Green_LED_Pin);
GPIO red_led(Red_LED_GPIO_Port, Red_LED_Pin);
GPIO button(Button_GPIO_Port, Button_Pin);
GPIO motor_power_switch_02(Motor_Power_Switch_02_GPIO_Port, Motor_Power_Switch_02_Pin);

extern UART_HandleTypeDef huart2;
USART serial(&huart2);

extern CAN_HandleTypeDef hcan1;
DjiRM::M2006_Motor motors(&hcan1);

extern SPI_HandleTypeDef hspi4;
SPI ras_spi(&hspi4);

extern SPI_HandleTypeDef hspi5;
SPI imu_spi(&hspi5);
GPIO imu_chip_select(SPI5_CS_GPIO_Port, SPI5_CS_Pin);
MPU6500 imu(imu_spi, imu_chip_select);


GPIO ras_spi_cs(SPI4_CS_GPIO_Port, SPI4_CS_Pin);
GPIO ist8310_reset(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin);

bool blinkLED_swicth = true;

void setup(void) {
    serial << "=========================================================" << stf::endl;
    

    byte_t id = imu.init(ist8310_reset);
    imu.measure_offset();
    serial << "IMU[MPU6500] ID = " << int(id) << stf::endl;

  
    //motor_power_switch_02.write(High);
}

void loop0(void) {
    //motors.init();
    //while(button.read() == Low);
    blinkLED_swicth = false;
    
    
    while(1) {
        imu.read_data();
        
        
        serial << imu.data_string() << stf::endl;
        
        delay(500);
    }

/*
    ras_spi.set_txrx_timeout(1000000);

    string str;
    byte_t byte;
    string dummy_str = "xxxx";
    while(1) {
        serial << "[receiving ...]" << stf::endl;
        //serial << ras_spi.get_txrx_status() << ", " << ras_spi.get_txrx_status() << stf::endl;
        while(ras_spi_cs.read() == Low);
        str = ras_spi.tranceive(dummy_str); 
        //byte = ras_spi.tranceive(0xFF);
        //serial << (char)byte << stf::endl;
        serial << str << stf::endl;
        //delay(500);
    }
*/


    /*
    uint16_t angle, speed, torque;

    while(1) {
        motors.set_current(1000, 0, 0, 0);
        angle = motors.get_raw_angle(DjiRM::Motor1);
        speed = motors.get_raw_speed(DjiRM::Motor1);
        torque = motors.get_raw_torque(DjiRM::Motor1);
        serial << "[Angle : " << angle  << "]";
        serial << "[Speed : " << speed  << "]";
        serial << "[Torque: " << torque << "]" << stf::endl;
    }
    */
}



void loop1(void) {
    
    while (blinkLED_swicth == true) {
       delay(300);
       green_led.write(High);
       red_led.write(Low);
       delay(300);
       green_led.write(Low);
       red_led.write(High);
   }
}

void exception(const char* str) {
    serial << stf::endl << "*****" << string(str) << stf::endl;
}