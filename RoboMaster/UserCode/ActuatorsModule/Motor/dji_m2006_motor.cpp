#include "dji_m2006_motor.hpp"

#include <iostream>
#include <vector>
#include <sstream>

using namespace DjiRM;

static const int16_t max_current = 9999; // 9.999A


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

	m1_ctrl.init(pid_ctrl_freq_Hz);
	m2_ctrl.init(pid_ctrl_freq_Hz);
	m3_ctrl.init(pid_ctrl_freq_Hz);
	m4_ctrl.init(pid_ctrl_freq_Hz);
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

// make velocity unit-less (%)
float M2006_Motor::get_velocity(motor_id m_id) {
	float raw_speed = get_raw_speed(m_id);

	if (raw_speed > max_raw_speed) {
		raw_speed = max_raw_speed;
	}
	if (raw_speed < -max_raw_speed) {
		raw_speed = -max_raw_speed;
	}
	return stf::map(raw_speed,
					   from_range((float)-max_raw_speed, (float)max_raw_speed),
					   to_range((float)-100.00, (float)100.00));

}



void M2006_Motor::update_pid_consts(float Kp, float Ki, float Kd) {
	m1_ctrl.update_pid_consts(Kp, Ki, Kd);
	m2_ctrl.update_pid_consts(Kp, Ki, Kd);
	m3_ctrl.update_pid_consts(Kp, Ki, Kd);
	m4_ctrl.update_pid_consts(Kp, Ki, Kd);
}

// Calculate period from frequency
uint32_t M2006_Motor::get_ctrl_period_ms(void) {
	return (uint32_t)((1.00 / (float)pid_ctrl_freq_Hz) * 1000.00);
}

void M2006_Motor::pid_update_motor_currents(void) {
	float new_curr1, new_curr2, new_curr3, new_curr4;
	// Argument == error
	// set_velocity() sets m1-m4_vel
	new_curr1 = m1_ctrl.calculate(m1_vel - get_velocity(Motor1));
	new_curr2 = m2_ctrl.calculate(m2_vel - get_velocity(Motor2));
	new_curr3 = m3_ctrl.calculate(m3_vel - get_velocity(Motor3));
	new_curr4 = m4_ctrl.calculate(m4_vel - get_velocity(Motor4));

	if (new_curr1 > 100.00 ) new_curr1 = 100.00;
	if (new_curr1 < -100.00 ) new_curr1 = -100.00;
	if (new_curr2 > 100.00 ) new_curr2 = 100.00;
	if (new_curr2 < -100.00 ) new_curr2 = -100.00;
	if (new_curr3 > 100.00 ) new_curr3 = 100.00;
	if (new_curr3 < -100.00 ) new_curr3 = -100.00;
	if (new_curr4 > 100.00 ) new_curr4= 100.00;
	if (new_curr4 < -100.00 ) new_curr4 = -100.00;



	// map from percentage to amperes
	new_curr1 = stf::map(new_curr1, from_range((float)-100.00, (float)100.00),
									to_range((float)-max_current, (float)max_current));
	new_curr2 = stf::map(new_curr2, from_range((float)-100.00, (float)100.00),
									to_range((float)-max_current, (float)max_current));
	new_curr3 = stf::map(new_curr3, from_range((float)-100.00, (float)100.00),
									to_range((float)-max_current, (float)max_current));
	new_curr4 = stf::map(new_curr4, from_range((float)-100.00, (float)100.00),
									to_range((float)-max_current, (float)max_current));

	set_current((int16_t)new_curr1, (int16_t)new_curr2, (int16_t)new_curr3, (int16_t)new_curr4);
}

// vel range: -100.00 ~ 100.00, where 100.00 means 100% of max possible velocity
void M2006_Motor::set_velocity(float m1_vel, float m2_vel, float m3_vel, float m4_vel) {
	this->m1_vel = m1_vel;
	this->m2_vel = m2_vel;
	this->m3_vel = m3_vel;
	this->m4_vel = m4_vel;
}

void M2006_Motor::stop(void){
	set_velocity(0.00, 0.00, 0.00, 0.00);
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


void M2006_Motor::motor_test(motor_id m_id) {
    int increment_T = 3; // 3 milliseconds delay for every current increment
    int16_t current_increment = 10; // 0.1A increment

	stf::notify("led: all on");
	stf::delay(300);
	stf::notify("led: all off");
	// Slowly accelerate motor
	for(int16_t curr = 0; curr < max_current; curr += current_increment) {
		if(m_id == Motor1) set_current(curr, 0, 0, 0);
		if(m_id == Motor2) set_current(0, curr, 0, 0);
		if(m_id == Motor3) set_current(0, 0, curr, 0);
		if(m_id == Motor4) set_current(0, 0, 0, curr);
		stf::delay(increment_T);
	}
	stf::notify("led: green on");
	stf::delay(300);
	stf::notify("led: green off");
	// Slowly decelerate motor to negative max
	for(int16_t curr = max_current; curr > -max_current; curr -= current_increment) {
		if(m_id == Motor1) set_current(curr, 0, 0, 0);
		if(m_id == Motor2) set_current(0, curr, 0, 0);
		if(m_id == Motor3) set_current(0, 0, curr, 0);
		if(m_id == Motor4) set_current(0, 0, 0, curr);
		stf::delay(increment_T);
	}
	stf::notify("led: red on");
	stf::delay(300);
	stf::notify("led: red off");
	// Accelerate back to zero
	for(int16_t curr = -max_current; curr < 0; curr += current_increment) {
		if(m_id == Motor1) set_current(curr, 0, 0, 0);
		if(m_id == Motor2) set_current(0, curr, 0, 0);
		if(m_id == Motor3) set_current(0, 0, curr, 0);
		if(m_id == Motor4) set_current(0, 0, 0, curr);
		stf::delay(increment_T);
	}
}

