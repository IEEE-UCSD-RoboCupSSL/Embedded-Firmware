#include "app_main.h"
#include "dji_m2006_motor.h"

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




bool blinkLED_swicth = true;

void setup(void) {
    serial << "Hello World" << stf::endl;
    motor_power_switch_02.write(High);
}

void loop0(void) {
    motors.init();
    while(button.read() == Low);
    blinkLED_swicth = false;
    
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
}



void loop1(void) {
    
    while (blinkLED_swicth == true) {
       delay(100);
       green_led.write(High);
       red_led.write(Low);
       delay(100);
       green_led.write(Low);
       red_led.write(High);
   }
}
