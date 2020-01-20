#ifndef __DJI_M2006_MOTOR_H
#define __DJI_M2006_MOTOR_H

#include "stf.h"


namespace DjiRM {
    class M2006_Motor {
    private:
        CAN_HandleTypeDef *hcanx;
        CAN_TxHeaderTypeDef tx_header;
        CAN_RxHeaderTypeDef rx_header;
        CAN_FilterTypeDef  filter_config;
        uint8_t tx_data[8];
        uint8_t rx_data[8];
        uint32_t tx_mailbox;
    public:
        M2006_Motor(CAN_HandleTypeDef *hcanx) {
            this->hcanx = hcanx;
        }
        ~M2006_Motor(void) {

        }

        void init(void);

        void set_current(int16_t ESC1_Curr, int16_t ESC2_Curr, int16_t ESC3_Curr, int16_t ESC4_Curr);

        void motor_test(void);
    };
}

#endif