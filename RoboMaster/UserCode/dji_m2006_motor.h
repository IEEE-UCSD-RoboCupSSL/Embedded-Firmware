#ifndef __DJI_M2006_MOTOR_H
#define __DJI_M2006_MOTOR_H

#include "stf.h"
#include "pid.hpp"


namespace DjiRM {
    enum motor_id : int {
        Motor1 = 0,
        Motor2 = 1,
        Motor3 = 2,
        Motor4 = 3
    };
}

namespace DjiRM {
    /* Only utilizing the OOP conventions, in fact for simplicity,
       only one instantiation is allowed due to complications 
       in sharing object fields with the CAN interrupt callback functions,
       (only new this object once, or bug would occur for receiving
        encoder data. To make it support multiple instance, i.e. 
        multiple CAN buses and more than 4 motors, please modify the
        source code of this class. Our current application only
        needs 4 motors in total, thus being too lazy to generalize it)*/
    class M2006_Motor {
    private:
        CAN_HandleTypeDef *hcanx;
        CAN_TxHeaderTypeDef tx_header;
        CAN_FilterTypeDef  filter_config;
        uint8_t tx_data[8];
        uint32_t tx_mailbox;
        /* All variables for rx are defined as static variable in the .cpp file,        
         * This is due to Rx interrupt callback is defined as a C function, which
         * can not reside or share variable within a class */

        float pid_ctrl_freq_Hz = 5000.00; // default 5000Hz
        PID_Controller<float> m1_ctrl;
		PID_Controller<float> m2_ctrl;
		PID_Controller<float> m3_ctrl;
		PID_Controller<float> m4_ctrl;

		int16_t max_raw_speed = 19400;

		float m1_vel, m2_vel, m3_vel, m4_vel;

    public:

        M2006_Motor(CAN_HandleTypeDef *hcanx, float Kp, float Ki, float Kd) :
    										    m1_ctrl(Kp, Ki, Kd),
												m2_ctrl(Kp, Ki, Kd),
												m3_ctrl(Kp, Ki, Kd),
												m4_ctrl(Kp, Ki, Kd) {
            this->hcanx = hcanx;
        }

        M2006_Motor(CAN_HandleTypeDef *hcanx, float Kp, float Ki, float Kd, float ctrl_freq_Hz) :
    										    m1_ctrl(Kp, Ki, Kd),
												m2_ctrl(Kp, Ki, Kd),
												m3_ctrl(Kp, Ki, Kd),
												m4_ctrl(Kp, Ki, Kd) {
            this->hcanx = hcanx;
            this->pid_ctrl_freq_Hz = ctrl_freq_Hz;
        }


        ~M2006_Motor(void) {}

        void init(void);

        void set_current(int16_t ESC1_Curr, int16_t ESC2_Curr, int16_t ESC3_Curr, int16_t ESC4_Curr);

        void stop(void);

        uint16_t get_raw_angle(motor_id m_id);
        int16_t get_raw_speed(motor_id m_id);
        int16_t get_raw_torque(motor_id m_id);
        
        float get_velocity(motor_id m_id);

        void update_pid_consts(float Kp, float Ki, float Kd);
		uint32_t get_ctrl_period_ms(void);
		void pid_update_motor_currents(void);

		// vel range: -100.00 ~ 100.00, where 100.00 means 100% of max possible velocity
		void set_velocity(float m1_vel, float m2_vel, float m3_vel, float m4_vel);


        void motor_test(void);
        void motor_test(motor_id m_id);
    };
}

#endif
