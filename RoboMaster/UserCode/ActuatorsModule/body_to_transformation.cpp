//#include "dji_m2006_motor.hpp"
//
//#include <iostream>
//#include <vector>
//#include <sstream>
//
//using namespace DjiRM;
//
//static const int16_t max_current = 9999; // 9.999A
//
//
///* These should be declared as memeber variables in OOP design,
// * but since interrupt is involved, there is a small complication to it,
// * i'm just being lazy and do this in C language's convention.
// * There should be only one motor class instance anyways!
// */
//static CAN_HandleTypeDef* __hcanx;
//static const uint32_t motor1_can_id = 0x201;
//static const uint32_t motor2_can_id = 0x202;
//static const uint32_t motor3_can_id = 0x203;
//static const uint32_t motor4_can_id = 0x204;
//static volatile uint8_t motor_idx;
//static volatile uint16_t angle_data[4];
//static volatile int16_t speed_data[4];
//static volatile int16_t torque_data[4];
//
//
//
//
//
//void M2006_Motor::init(void) {
//    __hcanx = hcanx;
//
//    //Overwrite automatic settings
//    hcanx->Init.Prescaler = 3;
//    hcanx->Init.TimeSeg2 = CAN_BS2_4TQ; //Assign special, longer 4
//    //Apply settings
//    if (HAL_CAN_Init(hcanx) != HAL_OK)
//	{
//    	Error_Handler();
//	}
//    // Configure CAN filter
//    filter_config.FilterBank = 0;
//    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
//    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
//    filter_config.FilterIdHigh = 0x0000;
//    filter_config.FilterIdLow = 0x0000;
//    filter_config.FilterMaskIdHigh = 0x0000;
//    filter_config.FilterMaskIdLow = 0x0000;
//    filter_config.FilterFIFOAssignment = CAN_FilterFIFO0;
//    filter_config.FilterActivation = ENABLE;
//    filter_config.SlaveStartFilterBank = 14;
//    HAL_CAN_ConfigFilter(hcanx, &filter_config);
//
//    // Start
//    HAL_CAN_Start(hcanx);
//
//    // Activate CAN receive interrupt for encoder data
//    HAL_CAN_ActivateNotification(hcanx, CAN_IT_RX_FIFO0_MSG_PENDING);
//
//	m1_ctrl.init(pid_ctrl_freq_Hz);
//	m2_ctrl.init(pid_ctrl_freq_Hz);
//	m3_ctrl.init(pid_ctrl_freq_Hz);
//	m4_ctrl.init(pid_ctrl_freq_Hz);
//}
//
//
//void M2006_Motor::bw_transformation(int16_t vx, int16_t vy, int16_t ang_vel) {
//	int16_t phi = 0;
//	int16_t theta = 0;
//	RF = vy*std::cos(phi) - vx*sin(phi);
//	RB = vy*cos(theta) + vx*sin(theta);
//	LB = -vy*cos(theta) - vx*sin(theta);
//	LF = -vy*cos(phi) - vx*sin(phi);
//}
//
//
