#include "app_main.h"
#include "USB/usb_device_vcp.h"
#include "Motor/dji_m2006_motor.hpp"
#include "IMU/mpu6500_ist8310.hpp"
#include "IMU/Adafruit_AHRS_Mahony.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <iostream>
#include <vector>
#include <sstream>
#include <string>


using namespace stf;
using namespace std;

GPIO green_led(Green_LED_GPIO_Port, Green_LED_Pin);
GPIO red_led(Red_LED_GPIO_Port, Red_LED_Pin);
GPIO button(Button_GPIO_Port, Button_Pin);
GPIO motor_power_switch_01(Motor_Power_Switch_01_GPIO_Port, Motor_Power_Switch_01_Pin);
GPIO motor_power_switch_02(Motor_Power_Switch_02_GPIO_Port, Motor_Power_Switch_02_Pin);
GPIO motor_power_switch_03(Motor_Power_Switch_03_GPIO_Port, Motor_Power_Switch_03_Pin);
GPIO motor_power_switch_04(Motor_Power_Switch_04_GPIO_Port, Motor_Power_Switch_04_Pin);


extern TIM_HandleTypeDef htim2;
Timer pwm_signal(&htim2, 2, TIM32Bit);

USB_VCP usb;

extern UART_HandleTypeDef huart2;
USART serial(&huart2);

extern CAN_HandleTypeDef hcan1;
// DJI EX: P = 1.5, I = 0.1
DjiRM::M2006_Motor motors(&hcan1, 1.5, 10, 0.1); // Manually Enter PID consts here (P.S.: can also use update_pid_const(...) to dynamically updating them later)

extern SPI_HandleTypeDef hspi4;
SPI ras_spi(&hspi4);

extern SPI_HandleTypeDef hspi5;
SPI imu_spi(&hspi5);
GPIO imu_chip_select(SPI5_CS_GPIO_Port, SPI5_CS_Pin);
MPU6500_IST8310 imu(imu_spi, imu_chip_select);
Adafruit_Mahony ahrs;
float ahrs_update_freq = 50; // Hz



GPIO ras_spi_cs(SPI4_CS_GPIO_Port, SPI4_CS_Pin);
GPIO ist8310_reset(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin);

bool blinkLED_switch = true;

bool is_motor_initialized = false;
bool is_imu_initialized = false;
bool is_usb_initialized = false;

QueueHandle_t io_message_queue;
bool is_message_queue_initialized = false;


void setup(void) {
    blinkLED_switch = false;
    serial << "=========================================================" << stf::endl;
//    serial << "Hello World" << stf::endl;
    


    motor_power_switch_01.write(High);
    motor_power_switch_02.write(High);
    motor_power_switch_03.write(High);
    motor_power_switch_04.write(High);

//    serial << "Before motor init" << stf::endl;
	motors.init();
	is_motor_initialized = true;
//	serial << "After Motor init " << stf::endl;

	pwm_signal.init_pwm_generation(1000, 1000);
	pwm_signal.pwm_generation_begin(Channel2);



	io_message_queue = xQueueCreate(1, 64);
	is_message_queue_initialized = true;

	usb.init();
	//usb.send_packet("Hi\n\r");
	is_usb_initialized = true;

    byte_t id = imu.init(ist8310_reset);
    imu.calibrate();
    serial << "IMU[MPU6500] ID = " << int(id) << stf::endl;
    ahrs.begin(ahrs_update_freq);
    is_imu_initialized = true;


    blinkLED_switch = true;
}

void defaultLoop(void) {

//	if(!is_motor_initialized) return;
//
//    // wait until white button is pressed to proceed, for safety reasons
//	while(button.read() == Low){
//		motors.set_current(0, 0, 0, 0);
//	}
//
//    // motors.motor_test(DjiRM::Motor2);
//	while(true){
//		motors.set_velocity(10, 10, 10, 10);
////		delay(2000);
////		motors.set_velocity(25, 25, 25, 25);
////		delay(2000);
////		motors.set_velocity(50, 50, 50, 50);
//		delay(2000);
////		motors.set_velocity(100, 100, 100, 100);
//	}
//
//	while(true){
//		motors.set_current(1000, 1000, 1000, 1000);
//	}

	// pwm_signal.set_pwm_duty_cycle_cnt(Channel1, 50);
//	while (true) {
//		pwm_signal.set_pwm_duty_cycle<float>(Channel2, 50);
//	}


//	while(true) {
//		serial << "Accel: " << imu.read_accel_data().to_string() << stf::endl;
//		serial << "Gyro: " << imu.read_gyro_data().to_string() << stf::endl;
//		serial << "Yaw: " << (int)ahrs.getYaw() << stf::endl; // not working
//		serial << "Pitch: " << (int)ahrs.getPitch() << stf::endl;
//		serial << "Roll: " << (int)ahrs.getRoll() << stf::endl;
		// serial << "Magnetometer: " << imu.read_compass_data().to_string() << stf::endl;

//	}




//    while(true) { // do nothing
//    	delay(1000);
//    }
	delay(1000);
}



void blinkLEDLoop(void) {
    
    while (blinkLED_switch == true) {
       delay(300);
       green_led.write(High);
       red_led.write(Low);
       delay(300);
       green_led.write(Low);
       red_led.write(High);
   }
}

void updatePIDLoop(void) {
//	if (is_motor_initialized) {
//		motors.pid_update_motor_currents();
//		delay(motors.get_ctrl_period_ms());
//	}
	delay(1000);
}

void updateIMULoop(void) {
//	if(is_imu_initialized) {
//		uint32_t update_period = (uint32_t) (1000.00f / ahrs_update_freq);
//		MPU6500_IST8310::data accel, gyro, mag;
//
//		accel = imu.read_accel_data();
//		gyro = imu.read_gyro_data();
//		mag = imu.read_compass_data();
//		ahrs.update(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag.x, mag.y, mag.z);
//
//		delay(update_period);
//	}
	delay(1000);
}

// Allows for continuous output of motor info
void printInfoLoop(void) {
//    int16_t speed, torque;
//    uint16_t angle;
//	// serial << "Motor on" << stf::endl;
//	angle = motors.get_raw_angle(DjiRM::Motor2);
//	speed = motors.get_raw_speed(DjiRM::Motor2);
//	torque = motors.get_raw_torque(DjiRM::Motor2);
//	serial << "[Angle : " << angle  << "]";
//	serial << "[Speed : " << speed  << "]";
//	serial << "[Torque: " << torque << "]" << stf::endl;

	// float or double CANNOT be printed
//	serial << (int32_t)(motors.get_velocity(DjiRM::Motor3)*100.00 / 100.0) << "."
//			<< (int32_t)(motors.get_velocity(DjiRM::Motor3)*100.00) % 100 << stf::endl;
	delay(1000);

}

void sensorsLoop(void) {
	delay(1000);
}

void actuatorsLoop(void) {
//	if(is_message_queue_initialized && is_usb_initialized && is_motor_initialized){
//		char cmd[64];
//		int length;
//		std::string cmd_str = "0,0,0";
//		Parsed_cmd parsed_cmd;
//		Wheel_speeds ws;
//
//		memset(cmd, 64, sizeof(char));
//
//		parsed_cmd.x = 0;
//		parsed_cmd.y = 0;
//		parsed_cmd.omega = 0;
//
//		ws.RF = 0;
//		ws.RB = 0;
//		ws.LF = 0;
//		ws.LB = 0;
//
//		// wait until white button is pressed to proceed, for safety reasons
//		while(button.read() == Low){
//			motors.set_current(0, 0, 0, 0);
//		}
//
//
//		// motors.motor_test(DjiRM::Motor2);
//		while(1){
//			// Mapping: RF, RB, LB, LF
//			motors.set_velocity(ws.RF, ws.RB, ws.LB, ws.LF);
//	//		delay(2000);
//	//		motors.set_velocity(25, 25, 25, 25);
//	//		delay(2000);
////			motors.set_velocity(50, 50, 50, 50);
//	//		motors.set_velocity(100, 100, 100, 100);
//
//			xQueuePeek(io_message_queue, cmd, 0);
//			length = strlen(cmd);
//
//			if(length < 4) continue;
//
//			cmd_str = std::string((const char*) cmd, length);
////
//			parsed_cmd = DjiRM::M2006_Motor::parse_cmd(cmd_str);
//
//			ws = DjiRM::M2006_Motor::bw_transformation(parsed_cmd.x, parsed_cmd.y, parsed_cmd.omega);
//
//
//			delay(1000);
//		}
//
//		motors.stop();
//		motors.set_current(0, 0, 0, 0);
//	}
	delay(1000);
}

void usbReadLoop(void){
	if(is_usb_initialized && is_message_queue_initialized) {
		std::string line = usb.read_line('\n');
//		usb.send_packet(line.append("\n"));
		const char* line_cstring = line.c_str();
//		serial << line.c_str() << stf::endl;

		xQueueSendToBack(io_message_queue, line_cstring, 0);
//		serial << "Sending Queue back..." << stf::endl;
	}

	delay(100);
}

// Write from RoboMaster to RP4
void usbWriteLoop(void){
	if(is_usb_initialized && is_message_queue_initialized){
		char cmd[64];
		int length;


		// pdTrue if received, otherwise returns pdFalse
		bool check = xQueueReceive(io_message_queue, cmd, 0);

		if (check) {
			serial << "Queue received..." << stf::endl;
			length = strlen(cmd);

			std::string cmd_str = std::string((const char*) cmd, length);

			usb.send_packet(cmd_str.append("\n"));

			serial << cmd << stf::endl;
		}
	}

	delay(1000);

}



void stf::exception(const char* str) {
    serial << stf::endl << "*****" << string(str) << stf::endl;
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
