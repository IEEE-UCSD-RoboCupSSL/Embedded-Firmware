#include "dji_m2006_motor.h"

#include <iostream>
#include <vector>
#include <sstream>

using namespace DjiRM;

void M2006_Motor::init(void) {
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

}

void M2006_Motor::set_current(int16_t ESC1_Curr, int16_t ESC2_Curr, int16_t ESC3_Curr, int16_t ESC4_Curr) {
    int16_t max_current = 9500; // 9.5A
    if(ESC1_Curr > max_current) ESC1_Curr = max_current;
    if(ESC1_Curr < -max_current) ESC1_Curr = -max_current;
    if(ESC2_Curr > max_current) ESC1_Curr = max_current;
    if(ESC2_Curr < -max_current) ESC1_Curr = -max_current;
    if(ESC3_Curr > max_current) ESC1_Curr = max_current;
    if(ESC3_Curr < -max_current) ESC1_Curr = -max_current;
    if(ESC4_Curr > max_current) ESC1_Curr = max_current;
    if(ESC4_Curr < -max_current) ESC1_Curr = -max_current;
        
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

void M2006_Motor::motor_test(void) {
    int test_T = 50;

    // M1
    for(int curr = 0; curr < 9500; curr += 100) {
        set_current(curr, 0, 0, 0);
        stf::delay(test_T);
    }
    for(int curr = 9500; curr > -9500; curr -= 100) {
        set_current(curr, 0, 0, 0);
        stf::delay(test_T);
    }
    for(int curr = -9500; curr < 0; curr += 100) {
        set_current(curr, 0, 0, 0);
        stf::delay(test_T);
    }

    // M2
    for(int curr = 0; curr < 9500; curr += 100) {
        set_current(0, curr, 0, 0);
        stf::delay(test_T);
    }
    for(int curr = 9500; curr > -9500; curr -= 100) {
        set_current(0, curr, 0, 0);
        stf::delay(test_T);
    }
    for(int curr = -9500; curr < 0; curr += 100) {
        set_current(0, curr, 0, 0);
        stf::delay(test_T);
    }

    // M3
    for(int curr = 0; curr < 9500; curr += 100) {
        set_current(0, 0, curr, 0);
        stf::delay(test_T);
    }
    for(int curr = 9500; curr > -9500; curr -= 100) {
        set_current(0, 0, curr, 0);
        stf::delay(test_T);
    }
    for(int curr = -9500; curr < 0; curr += 100) {
        set_current(0, 0, curr, 0);
        stf::delay(test_T);
    }
    
    // M4
    for(int curr = 0; curr < 9500; curr += 100) {
        set_current(0, 0, 0, curr);
        stf::delay(test_T);
    }
    for(int curr = 9500; curr > -9500; curr -= 100) {
        set_current(0, 0, 0, curr);
        stf::delay(test_T);
    }
    for(int curr = -9500; curr < 0; curr += 100) {
        set_current(0, 0, 0, curr);
        stf::delay(test_T);
    }
}