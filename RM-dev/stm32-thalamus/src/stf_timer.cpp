#include "stf_timer.h"

using namespace stf;

/* Macro definition */
#define Failed false
#define Succeeded true


/* Private function declaration*/
static void timPWM_IN_IT_CallBack(TIM *, HAL_TIM_ActiveChannel);
static void timEnc_OverFlow_IT_CallBack(TIM *);


/* Static member initialization */
uint16_t TIM::numActiveTIMs = 0;
TIM * TIM::ActiveTIMs[] = {nullptr};


/**
 *  @brief  Create a new timer object.
 *  @param  htim 
 *  @param  APBx_DivFactor 
 *  @param  xBitTIM 
 */
TIM::TIM(TIM_HandleTypeDef * htim, uint32_t APBx_DivFactor, TIM_Resolution xBitTIM) : 
    htim(htim), APBx_Div_Factor(APBx_DivFactor), xBitTIM(xBitTIM), isEncMode(false) {
    for(int i = 0; i < numActiveTIMs; i++) {
		if(ActiveTIMs[i]->htim == htim) {
			ActiveTIMs[i] = this;
			return;
		}
    }
	ActiveTIMs[numActiveTIMs++] = this;
}


/**
 * @brief default destructor
 */ 
TIM::~TIM() = default;


/*=======================Universal Function================================*/
/** This section is aimed for timer that multitasks, if a timer is solely 
 * focus on e.g. pwm generation, go to PWM Generation in another section
 * below.
 */


/**
 * @brief get Auto Reload Register value
 * @return current Auto Reload Register value
 */
uint32_t TIM::getARR() {
    this->ARR = __HAL_TIM_GET_AUTORELOAD(this->htim);
	return this->ARR;
}


/**
 * @brief get Capture/Compare
 * @param channel
 * @return current Capture/Compare
 */
uint32_t TIM::getCCR(uint32_t channel) {
    this->CCR = __HAL_TIM_GET_COMPARE(this->htim, channel);
	return this->CCR;
}


/**
 * @brief get counter
 * @return current counter
 */
uint32_t TIM::getCNT() {
    this->CNT = __HAL_TIM_GET_COUNTER(this->htim);
	return this->CNT;
}


/**
 * @brief get pre-scaler
 * @return current pre-scaler
 */
uint32_t TIM::getPrescaler() {
    return this->TimerPrescaler;
}


/**
 * @brief set Auto Reload Register value
 * @param ARR_val new Auto Reload Register value
 */
void TIM::setARR(uint32_t ARR_val) {
    if(this->xBitTIM == TIM_16bit) {
		if(ARR_val > 0xFFFF) {
			exception("stf_timer.cpp: setARR() | AutoReload must < 0xFFFF for a 16 bit timer");
        }
	}
	else if(this->xBitTIM == TIM_32bit) {
		if(ARR_val > 0xFFFFFFFF) {
			exception("stf_timer.cpp: setARR() | AutoReload must < 0xFFFFFFFF for a 32 bit timer");
        }
	}

	this->ARR = ARR_val;
	__HAL_TIM_SET_AUTORELOAD(this->htim, this->ARR);
}


/**
 * @brief set Capture/Compare
 * @param channel
 * @param CCR_val new Capture/Compare value
 */
void TIM::setCCR(uint32_t channel, uint32_t CCR_val) {
    this->CCR = CCR_val;
	__HAL_TIM_SET_COMPARE(this->htim, channel, CCR_val);
}


/**
 * @brief set counter
 * @param CNT_val new counter value
 */
void TIM::setCNT(uint32_t CNT_val) {
    this->CNT = CNT_val;
	__HAL_TIM_SET_COUNTER(this->htim, CNT_val);
}


/**
 * @brief set pre-scalar
 * @param prescaler_val new pre-scalar value
 */
void TIM::setPrescaler(uint32_t prescaler_val) {
    this->TimerPrescaler = prescaler_val;
	__HAL_TIM_SET_PRESCALER(this->htim, this->TimerPrescaler);
}


/**
 * @brief
 * @param channel
 * @return Index associate with channel
 */
uint8_t TIM::tim_channel_index(uint32_t channel) {
	switch(channel) {
		case TIM_CH1:
			return 1;

		case TIM_CH2:
			return 2;

		case TIM_CH3:
			return 3;

		case TIM_CH4:
			return 4;

		default:
			return 0;
	}
}


/*=======================Basic Counting & Interrupt==========================*/


/**
 * @brief TODO
 * @param AutoReload_count
 * @param timer_frequency
 * @return TODO
 * Warning!: For 16bit timers, AutoRelaod_count must be less than 2^16
 * & timer frequency must be greater than a certain
 * threshold such that make prescaler value less than 2^16 (refer to source code)
 */
bool TIM::initBasicCounting(uint32_t AutoReload_count, uint32_t timer_frequency) {
    this->setARR(AutoReload_count);
    return this->setFrequency(timer_frequency);
}


/**
 * @brief TODO
 * @param timer_frequency
 * @return TODO
 */
bool TIM::setFrequency(uint32_t timer_frequency) {
	uint32_t TimerMaxFrequency = HAL_RCC_GetHCLKFreq() / this->APBx_Div_Factor;

	if(timer_frequency > TimerMaxFrequency) {
		exception("THL_Timer.c: timSetFrequency() | timer_frequency must be less or equal than TimerMaxFrequency");
		return Failed;
	}

	this->setPrescaler(TimerMaxFrequency / timer_frequency - 1);

	//Prescaled frequency is subject to rounding error
	this->ActualFreq = TimerMaxFrequency / (this->getPrescaler() + 1);
	return Succeeded;
}


/**
 * @brief TODO
 */
void TIM::countBegin() {
	HAL_TIM_Base_Start(this->htim);
}


/**
 * @brief TODO
 */
void TIM::countEnd() {
	HAL_TIM_Base_Stop(this->htim);
}


/**
 * @brief TODO
 */
void TIM::countBeginIT() {
	HAL_TIM_Base_Start_IT(this->htim);
}


/** 
 * @brief TODO
 */
void TIM::countEndIT() {
	HAL_TIM_Base_Stop_IT(this->htim);
}


/**
 * @brief TODO
 * @return current counter
 * This method is equivalent to getCNT
 */
uint32_t TIM::getCount() {
	return this->getCNT();
}


/**
 * @brief TODO
 * @param instance
 */
__weak void timPE_IT_CallBack(TIM * instance) {
	UNUSED(instance);
}


/**
 * @brief TODO
 * @param instance
 */
__weak void timSysT_IT_CallBack(TIM * instance) {
	UNUSED(instance);
}


/*============================PWM Input/Output===============================*/

//Output: PWM generation

/** PWM Frequency Explaination
 *  - Timer frequency and PWM frequency are two distinct concepts!
 *    [SystemCoreClock] => [TimerClock/Freq] => [pwm_frequency]  (signals are produced from left to right).
 *    Timer clock is system clock divided by a division factor.
 *    Timer clock determines the frequency of each timer tick.
 *
 *    PWM signals are generated by setting a counter value <= ([Max_Count], a.k.a [timer period]), this
 *    counter value is what gets written to TIM->CCRx register. The timer output compare (a hardware function)
 *    pulls the signal line HIGH until the counter counts up to this TIM->CCRx value, and then pull it LOW till
 *    Max_Count. Technically, the [PwmDutyCycle] = [CCR] / [Max_Count] (in percentage).
 *
 * 	- The [TimerPrescaler] corresponds to a register that can be configured to set the desired timer frequency
 * 	  by dividing the "APB bus clock" scaled from [SystemCoreClock], a.k.a [HCLK_Frequency], with the TimerPrescaler 
 *    value. Details of the clock relations can be looked up in STM32CubeMx software's clock configuration section,
 * 	  where a clock tree is presented.
 *
 * 	- [TimerMaxFrequency] = [HCLK] / [APBx_Bus_DivisionFactor] = [APBx_Bus_Frequency].
 *
 * 	- [TimerPrescaler] = [TimerMaxFrequency] / [TimerFrequency:(the desired frequency by user)].
 *
 * 	- [TimerFrequency:(desired)] = [Max_Count:(refer to 2nd paragraph)] * [pwm_frequency:(desired)].
 * 	  Basically the timer frequency is determined both by max count and pwm frequency,
 * 	  where the max count determines the resolution of this pwm signal.
 *
 * 	- [Max_Count] * [pwm_frequency] must be <= [TimerMaxFrequency], increasing PWM_Freq makes the driver
 * 	  respond quicker at the cost of resolution, while increasing the maximum counter value gives a higher
 * 	  resolution as the expense of responsiveness.
 *
 * 	- Turning the frequency down saves power consumption, but it usually dosen't matter.
 *
 * 	- For regular DC motor, 1kHZ-10kHZ is recommended in general.
 *
 * 	- For drone ESCs, frequency is much lower, refer to the ESC datasheet.
 *
 */


/** 
 * @brief TODO
 * @param max_count determines the resolution, e.g. for .1  precision, max_count is typically 1,000, for .01 
 * 					precision, max_count is typically 10,000
 * @param pwm_frequency refers to the frequency of each PWM Cycle's period, which is the time period between two 
 * 						rising edge of a typical pwm pulse
 * @return TODO
 */
bool TIM::initPwmOut(uint32_t max_count, uint32_t pwm_frequency) {
	return this->setPwmFrequency(max_count, pwm_frequency);
}


/**
 * @brief Set PWM frequency at runtime
 * @param max_count determines the resolution, e.g. for .1  precision, max_count is typically 1,000, for .01 
 * 					precision, max_count is typically 10,000
 * @param pwm_frequency refers to the frequency of each PWM Cycle's period, which is the time period between two 
 * 						rising edge of a typical pwm pulse
 * @return TODO
 */
bool TIM::setPwmFrequency(uint32_t max_count, uint32_t pwm_frequency) {
	double TimerMaxFrequency = HAL_RCC_GetHCLKFreq() / this->APBx_Div_Factor;
	double TimerFrequency = max_count * pwm_frequency;

	if(TimerFrequency > TimerMaxFrequency) {
		exception("THL_Timer.c: setPwmFrequency() | max_count * pwm_frequency must be less or equal than TimerMaxFrequency");
		return Failed;
	}

	this->ActualFreq = TimerFrequency;
	this->setPrescaler(TimerMaxFrequency / TimerFrequency - 1);

	//Minimize rounding error
	this->setARR(((uint32_t)TimerMaxFrequency / (this->TimerPrescaler + 1) ) / pwm_frequency);
	return Succeeded;
}


/** 
 * @brief TODO
 * @param channel TIM Channels to be enabled
 */
void TIM::pwmGenBegin(uint32_t channel) {
	HAL_TIM_PWM_Start(this->htim, channel);
	this->setPwmDutyCycle(channel, 0);
}


/**
 * @brief TODO
 * @param channel TIM Channels ro be disabled
 */
void TIM::pwmGenEnd(uint32_t channel) {
	HAL_TIM_PWM_Stop(this->htim, channel);
}


/**
 * @brief TODO
 * @param channel
 * @param dutyCycleCnt
 */
void TIM::setPwmDutyCycle(uint32_t channel, uint32_t dutyCycleCnt) {
	this->setCCR(channel, dutyCycleCnt);
}


/**
 * @brief TODO
 * @param channel
 * @param dutyCyclePercent
 */
void TIM::pwmWrite(uint32_t channel, double dutyCyclePercent) {
	if(dutyCyclePercent > 100.00f) {
		dutyCyclePercent = 100.00f;
	}

	dutyCyclePercent /= 100.00f;
	this->setPwmDutyCycle(channel, (uint32_t)(dutyCyclePercent * (double)this->ARR));
}


// Input: Periodic input capture interpreted as PWM pulse

/** 
 * @brief PWM input: Typical use cases: PPM radio remote input, SR04 Ultrasonic sensor Timer's counter counts at 
 * [timer_freq = max_cnt * pwm_freq]. Same as pwm_gen, max_count here is used to determine resolution.
 * Generally it is recommended to set the resolution so that the timer_freq is less than 1,000,000 Hz = 1Mhz
 *
 * Unlike other methods, max_count here is not used to set Autoreload,
 * To reduce the chances of counter overflow, ARR is set to be the largest 16-bit value 0xFFFF,
 * even though this edge case has been taken care.
 * It is required to have [# of Autoreload(ARR) count > # of counts within a pulse],
 * ARR count is 0xFFFF, # of counts within a pulse = pulse_width / (1/timer_freq).
 * @param IC_fields
 * @param max_count
 * @param pwm_frequency
 * @return TODO
 * Note: Capture interrupt MUST be enabled, use CubeMx to enable it
 */
bool TIM::initPwmIn(TIM::TIM_IC * IC_fields, uint32_t max_count, uint32_t pwm_frequency) {
	uint32_t timer_frequency = max_count * pwm_frequency;
	uint32_t TimerMaxFrequency = HAL_RCC_GetHCLKFreq() / this->APBx_Div_Factor;
	if(timer_frequency > TimerMaxFrequency) {
		exception("THL_Timer.c: timSetFrequency() | timer_frequency must be less or equal than TimerMaxFrequency");
		return Failed;
	}

	this->initIC(IC_fields, timer_frequency);

	for(auto i = 1U; i < TIM_NUM_CHANNELS + 1; i++) {
		IC_fields->IC_FirstEdge[i] = 0;
		IC_fields->PulseWidth[i] = 0;
	}

	this->IC_fields->isUsedForPwmInput = true;
	this->IC_fields->pwm_input_max_count = max_count;

	return Succeeded;
}


/**
 * @brief TODO
 * @param channel
 * @param pulse_polarity
 */
void TIM::pwmIcBegin(uint32_t channel, PulseLevel pulse_polarity) {
	if(pulse_polarity == TIM_PulseOnHigh) {
		this->setICPolarity(channel, TIM_IC_RisingEdge);
	}
	else if(pulse_polarity == TIM_PulseOnLow)
		this->setICPolarity(channel, TIM_IC_FallingEdge);

	this->IC_fields->pulse_polarity = pulse_polarity;

	this->icBeginIT(channel);
}


/**
 * @brief TODO
 * @param channel
 */
void TIM::pwmIcEnd(uint32_t channel) {
	this->icEndIT(channel);
}


/**
 * @brief TODO
 * @param channel
 * @return TODO
 */
int32_t TIM::getPulseWidth(uint32_t channel) {
	return this->IC_fields->PulseWidth[tim_channel_index(channel)];
}


/**
 * @brief TODO
 * @param channel
 * @return TODO
 */
double TIM::pwmRead(uint32_t channel) {
	return ((double)this->getPulseWidth(channel) / (double)this->IC_fields->pwm_input_max_count) * 100.00f;
}


/**
 * @brief TODO
 * @param instance
 * @param active_channel
 */
void timPWM_IN_IT_CallBack(TIM * instance, HAL_TIM_ActiveChannel active_channel) {
	uint32_t channel;
	if(instance->IC_fields->isUsedForPwmInput) {
		switch(active_channel) {
			case TIM_Active_CH1:
				channel = TIM_CH1;
				break;

			case TIM_Active_CH2:
				channel = TIM_CH2;
				break;

			case TIM_Active_CH3:
				channel = TIM_CH3;
				break;

			case TIM_Active_CH4:
				channel = TIM_CH4;
				break;
			
			default:
				exception("This code should not be reached.");
		}

		if(instance->IC_fields->pulse_polarity == TIM_PulseOnHigh) {
			if(instance->IC_fields->ICpolarity[TIM::tim_channel_index(channel)] == TIM_IC_RisingEdge) {
				instance->IC_fields->IC_FirstEdge[TIM::tim_channel_index(channel)] = 
					instance->getCapVal(channel);

				instance->setICPolarity(channel, TIM_IC_FallingEdge);
			}
			else if(instance->IC_fields->ICpolarity[TIM::tim_channel_index(channel)] == TIM_IC_FallingEdge) {
				instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] = 
					instance->getCapVal(channel) - 
					instance->IC_fields->IC_FirstEdge[TIM::tim_channel_index(channel)];

				if(instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] < 0) {
					instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] += instance->getARR() + 1;
													
				}

				instance->setICPolarity(channel, TIM_IC_RisingEdge);
			}
		}
		else {
			if(instance->IC_fields->ICpolarity[TIM::tim_channel_index(channel)] == TIM_IC_FallingEdge) {
				instance->IC_fields->IC_FirstEdge[TIM::tim_channel_index(channel)] = 
					instance->getCapVal(channel);

				instance->setICPolarity(channel, TIM_IC_RisingEdge);
			}
			else if(instance->IC_fields->ICpolarity[TIM::tim_channel_index(channel)] == TIM_IC_RisingEdge) {
				instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] = 
					instance->getCapVal(channel) - 
					instance->IC_fields->IC_FirstEdge[TIM::tim_channel_index(channel)];
				if(instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] < 0) {
					instance->IC_fields->PulseWidth[TIM::tim_channel_index(channel)] += instance->getARR() + 1;
													
				}

				instance->setICPolarity(channel, TIM_IC_FallingEdge);
			}
		}
	}
}


/**
 * @brief TODO
 * @param IC_fields
 * @param timer_frequency
 * @return TODO
 */
uint32_t TIM::initIC(TIM::TIM_IC * IC_fields, uint32_t timer_frequency) {
	if(this->xBitTIM == TIM_32bit) {
		this->setARR(0xFFFFFFFF);
	}
	else if(this->xBitTIM == TIM_16bit) {
		this->setARR(0xFFFF);
	}
	this->setFrequency(timer_frequency);

	this->IC_fields = IC_fields;
	this->IC_fields->isUsedForPwmInput = false;
	return this->ActualFreq;
}


/**
 * @brief TODO
 * @param channel
 * @param icPolarity
 */
void TIM::setICPolarity(uint32_t channel, uint32_t icPolarity) {
	this->IC_fields->ICpolarity[tim_channel_index(channel)] = icPolarity;
	__HAL_TIM_SET_CAPTUREPOLARITY(this->htim, channel, icPolarity);
}


/**
 * @brief TODO
 * @param channel
 */
void TIM::icBeginIT(uint32_t channel) {
	HAL_TIM_IC_Start_IT(this->htim, channel);
}


/**
 * @brief TODO
 * @param channel
 */
void TIM::icEndIT(uint32_t channel) {
	HAL_TIM_IC_Stop_IT(this->htim, channel);
}


/**
 * @brief TODO
 * @param channel
 * @return TODO
 */
uint32_t TIM::getCapVal(uint32_t channel) {
	return HAL_TIM_ReadCapturedValue(this->htim, channel);
}


/**
 * @brief TODO
 * @param htim
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim) {
	for(int i = 0; i < TIM::numActiveTIMs; i++) {
		if(TIM::ActiveTIMs[i]->htim == htim) {
			HAL_TIM_ActiveChannel active_channel = htim->Channel;
			timPWM_IN_IT_CallBack(TIM::ActiveTIMs[i], active_channel);
			timEnc_OverFlow_IT_CallBack(TIM::ActiveTIMs[i]);
			timIC_IT_CallBack(TIM::ActiveTIMs[i], active_channel);
		}
	}
}


/**
 * @brief TODO
 * @param instance
 * @param active_channel
 */
__weak void timIC_IT_CallBack(TIM * instance, HAL_TIM_ActiveChannel active_channel) {
	UNUSED(instance);
	UNUSED(active_channel);
}


/*======================(Quadrature) Encoder Mode==========================*/
//Reminder: Enable Encoder Mode in CubeMx


/**
 * @brief TODO
 */
void TIM::initEnc() {
	if(this->xBitTIM == TIM_16bit) {
		this->setARR(0xFFFF);
	}
	else if(this->xBitTIM == TIM_32bit) {
		this->setARR(0xFFFFFFFF);
	}
	this->isEncMode = true;
	this->ENC_CNT   = 0;
	this->ENC_OverFlow = 0;
}


/**
 * @brief TODO
 */
void TIM::encBegin() {
	HAL_TIM_Encoder_Start(this->htim, TIM_CHANNEL_ALL);
	this->resetEnc();
}


/**
 * @brief TODO
 */
void TIM::encBegin_IT() {
	HAL_TIM_Encoder_Start_IT(this->htim, TIM_CHANNEL_ALL);
	this->resetEnc();
}


/**
 * @brief TODO
 */
void TIM::encEnd() {
	HAL_TIM_Encoder_Stop(this->htim, TIM_CHANNEL_ALL);
}


/**
 * @brief TODO
 */
void TIM::encEnd_IT() {
	HAL_TIM_Encoder_Stop_IT(this->htim, TIM_CHANNEL_ALL);
}


/**
 * @brief TODO
 */
void TIM::resetEnc() {
	this->setCNT(0);
	this->ENC_CNT = 0;
	this->ENC_OverFlow = 0;
}


/**
 * @brief TODO
 * @return TODO
 */
int32_t TIM::getEncCNT() {
	if(this->xBitTIM == TIM_16bit) {
		this->ENC_CNT = (int16_t)__HAL_TIM_GET_COUNTER(this->htim) + this->ENC_OverFlow;
	}
	else {
		this->ENC_CNT = (int32_t)__HAL_TIM_GET_COUNTER(this->htim);
	}
	return this->ENC_CNT;
}


/**
 * @brief TODO
 * @return TODO
 */
int32_t TIM::getOverFlowPart_32bit() {
	return this->ENC_OverFlow;
}


/**
 * @brief TODO
 * @param instance
 */
void timEnc_OverFlow_IT_CallBack(TIM * instance) {
	if(instance->isEncMode == true) {
		if(instance->xBitTIM == TIM_32bit) {

			if((int16_t)__HAL_TIM_GET_COUNTER(instance->htim) <= -1000000000) {  //1,000,000,000 using decimal base here for simplicity
				instance->ENC_OverFlow--;
				instance->setCNT(0);
			}
			else if((int16_t)__HAL_TIM_GET_COUNTER(instance->htim) >= 1000000000) {
				instance->ENC_OverFlow++;
				instance->setCNT(0);
			}
		}
		else if(instance->xBitTIM == TIM_16bit) {

			// 32767 == 0x7FFF == 0xFFFF/2, setting it to be a bit less than 0x7FFF in case of failure of interrupt capture (very rare)
			if((int16_t)__HAL_TIM_GET_COUNTER(instance->htim) <= -32760) {
				instance->ENC_OverFlow -= 32760;
				instance->setCNT(0);
			}
			else if((int16_t)__HAL_TIM_GET_COUNTER(instance->htim) >= 32760) {
				instance->ENC_OverFlow += 32760;
				instance->setCNT(0);
			}
		}
	}
}