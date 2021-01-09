#include "dji_m2006_motor.h"

#include <iostream>
#include <vector>
#include <sstream>

using namespace DjiRM;

static const int16_t max_current = 9900; // 9.9A


/* These should be declared as memeber variables in OOP design,
 * but since interrupt is involved, there is a small complication to it,
 * i'm just being lazy and do this in C language's convention. 
 * There should be only one motor class instance anyways!
 */ 
static CAN_HandleTypeDef* __hcanx;
static const uint32_t motor1_can_id = 0x201;
static const uint32_t motor2_can_id = 0x202;
static const uint32_t motor3_can_id = 0x203;
static const uint32_t motor4_can_id = 0x204;
static volatile uint8_t motor_idx;
static volatile uint16_t angle_data[4];
static volatile int16_t speed_data[4];
static volatile int16_t torque_data[4];

void M2006_Motor::init(void) {
    __hcanx = hcanx;

    //Overwrite automatic settings
    hcanx->Init.Prescaler = 3;
    hcanx->Init.TimeSeg2 = CAN_BS2_4TQ; //Assign special, longer 4
    //Apply settings
    if (HAL_CAN_Init(hcanx) != HAL_OK)
	{
    	Error_Handler();
	}
    // Configure CAN filter
    filter_config.FilterBank = 0;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = 0x0000;
    filter_config.FilterIdLow = 0x0000;
    filter_config.FilterMaskIdHigh = 0x0000;
    filter_config.FilterMaskIdLow = 0x0000;
    filter_config.FilterFIFOAssignment = CAN_FilterFIFO0;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;   
    HAL_CAN_ConfigFilter(hcanx, &filter_config);

    // Start
    HAL_CAN_Start(hcanx);

    // Activate CAN receive interrupt for encoder data
    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void M2006_Motor::set_current(int16_t ESC1_Curr, int16_t ESC2_Curr, int16_t ESC3_Curr, int16_t ESC4_Curr) {
    if(ESC1_Curr > max_current) ESC1_Curr = max_current;
    if(ESC1_Curr < -max_current) ESC1_Curr = -max_current;

    if(ESC2_Curr > max_current) ESC2_Curr = max_current;
    if(ESC2_Curr < -max_current) ESC2_Curr = -max_current;

    if(ESC3_Curr > max_current) ESC3_Curr = max_current;
    if(ESC3_Curr < -max_current) ESC3_Curr = -max_current;

    if(ESC4_Curr > max_current) ESC4_Curr = max_current;
    if(ESC4_Curr < -max_current) ESC4_Curr = -max_current;
        
    tx_header.StdId = 0x200;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 0x08;
    tx_header.TransmitGlobalTime = DISABLE;
    tx_data[0] = (ESC1_Curr >> 8);
    tx_data[1] = ESC1_Curr;

    tx_data[2] = (ESC2_Curr >> 8);
    tx_data[3] = ESC2_Curr;

    tx_data[4] = (ESC3_Curr >> 8);
    tx_data[5] = ESC3_Curr;

    tx_data[6] = (ESC4_Curr >> 8);
    tx_data[7] = ESC4_Curr;
    HAL_CAN_AddTxMessage(hcanx, &tx_header, tx_data, &tx_mailbox);
}

void M2006_Motor::stop(void){
	this->set_current(0,0,0,0);
}

void M2006_Motor::motor_test(void) {
    int increment_T = 3; // 3 milliseconds delay for every current increment
    int16_t current_increment = 10; // 0.1A increment

    for(int rnd = 0; rnd < 4; rnd++) {
        stf::notify("led: all on");
        stf::delay(300);
        stf::notify("led: all off");
        for(int16_t curr = 0; curr < max_current; curr += current_increment) {
            if(rnd == 0) set_current(curr, 0, 0, 0);
            if(rnd == 1) set_current(0, curr, 0, 0);
            if(rnd == 2) set_current(0, 0, curr, 0);
            if(rnd == 3) set_current(0, 0, 0, curr);
            stf::delay(increment_T);
        }
        stf::notify("led: green on");
        stf::delay(300);
        stf::notify("led: green off");
        for(int16_t curr = max_current; curr > -max_current; curr -= current_increment) {
            if(rnd == 0) set_current(curr, 0, 0, 0);
            if(rnd == 1) set_current(0, curr, 0, 0);
            if(rnd == 2) set_current(0, 0, curr, 0);
            if(rnd == 3) set_current(0, 0, 0, curr);
            stf::delay(increment_T);
        }
        stf::notify("led: red on");
        stf::delay(300);
        stf::notify("led: red off");
        for(int16_t curr = -max_current; curr < 0; curr += current_increment) {
            if(rnd == 0) set_current(curr, 0, 0, 0);
            if(rnd == 1) set_current(0, curr, 0, 0);
            if(rnd == 2) set_current(0, 0, curr, 0);
            if(rnd == 3) set_current(0, 0, 0, curr);
            stf::delay(increment_T);
        }
    }
}


/* Read encoder in FMP0 interrupt that triggers 
 * everytime a new message arrived through CAN
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    if(hcan == __hcanx) {
        uint8_t rx_data[8];
        CAN_RxHeaderTypeDef rx_header;
        HAL_CAN_GetRxMessage(__hcanx, CAN_RX_FIFO0, &rx_header, rx_data);

        if(rx_header.StdId == motor1_can_id) motor_idx = 0;
        if(rx_header.StdId == motor2_can_id) motor_idx = 1;
        if(rx_header.StdId == motor3_can_id) motor_idx = 2;
        if(rx_header.StdId == motor4_can_id) motor_idx = 3;

        angle_data[motor_idx] = (uint16_t)(rx_data[0]<<8 | rx_data[1]);
        speed_data[motor_idx] = (int16_t)(rx_data[2]<<8 | rx_data[3]);
        torque_data[motor_idx] = (int16_t)(rx_data[4]<<8 | rx_data[5]);
    }
}


uint16_t M2006_Motor::get_raw_angle(motor_id m_id) {
    return angle_data[m_id];
}
int16_t M2006_Motor::get_raw_speed(motor_id m_id) {
    return speed_data[m_id];
}
int16_t M2006_Motor::get_raw_torque(motor_id m_id) {
    return torque_data[m_id];
}
