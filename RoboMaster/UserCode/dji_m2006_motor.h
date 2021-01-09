#ifndef __DJI_M2006_MOTOR_H
#define __DJI_M2006_MOTOR_H

#include "stf.h"


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
         * can not reside or share variable within a class
         */
    public:
        M2006_Motor(CAN_HandleTypeDef *hcanx) {
            this->hcanx = hcanx;
        }
        ~M2006_Motor(void) {

        }

        void init(void);

        void set_current(int16_t ESC1_Curr, int16_t ESC2_Curr, int16_t ESC3_Curr, int16_t ESC4_Curr);

        void stop(void);

        uint16_t get_raw_angle(motor_id m_id);
        int16_t get_raw_speed(motor_id m_id);
        int16_t get_raw_torque(motor_id m_id);
        

        void motor_test(void);

    };
}

#endif
