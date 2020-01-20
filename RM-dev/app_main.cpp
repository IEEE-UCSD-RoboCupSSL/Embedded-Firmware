#include "app_main.h"
#include "dji_m2006_motor.h"

#include <iostream>
#include <vector>
#include <sstream>


using namespace stf;
using namespace std;

GPIO green_led(Green_LED_GPIO_Port, Green_LED_Pin);
GPIO red_led(Red_LED_GPIO_Port, Red_LED_Pin);
GPIO button(Button_GPIO_Port, Button_Pin);
GPIO motor_power_switch_02(Motor_Power_Switch_02_GPIO_Port, Motor_Power_Switch_02_Pin);

extern CAN_HandleTypeDef hcan1;
DjiRM::M2006_Motor motors(&hcan1);


bool blinkLED_swicth = true;

void setup(void) {
   motor_power_switch_02.write(High);
}

void loop0(void) {
    motors.init();
    while(button.read() == Low);
    blinkLED_swicth = false;
    while(1) {
        motors.motor_test();
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
