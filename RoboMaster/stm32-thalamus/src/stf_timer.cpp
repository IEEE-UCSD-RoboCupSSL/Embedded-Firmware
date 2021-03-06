#include "stf_timer.h"

#ifdef HAL_TIM_MODULE_ENABLED

using namespace stf;

uint32_t num_tims = 0;
Timer* active_tims[Max_Num_TIMs];


Timer::Timer(TIM_HandleTypeDef *htimx, uint32_t APBx_division_factor, 
              tim_resolution resolution) {
	this->htimx = htimx;
	this->APBx_division_factor = APBx_division_factor;
	this->resolution = resolution;
	this->mode = Uninitialized;
}

Timer::~Timer() {}

void Timer::set_prescaler(uint32_t prescaler_val) {
	this->prescaler = prescaler_val;
	__HAL_TIM_SET_PRESCALER(htimx, prescaler);
}
uint32_t Timer::get_prescaler(void) {
	return this->prescaler; 
}

void Timer::set_ARR(uint32_t ARR_val) {
	if(this->resolution == TIM16Bit) {
		if(ARR_val > 0xFFFF) {
			exception("ARR exceeds max limit");
			return;
		}
	}
	if(this->resolution == TIM32Bit) {
		if(ARR_val > 0xFFFFFFFF) {
			exception("ARR exceeds max limit");
			return;
		}
	}
	this->ARR = ARR_val;
	__HAL_TIM_SET_AUTORELOAD(htimx, this->ARR);
}
uint32_t Timer::get_ARR(void) {
	this->ARR = __HAL_TIM_GET_AUTORELOAD(this->htimx);
	return this->ARR;
}

void Timer::set_CCR(tim_channel channel, uint32_t CCR_val) {
	if(CCR_val > this->ARR) {
		exception("Invalid: CRR is greater than ARR");
		return;
	}
	this->CCR = CCR_val;
	__HAL_TIM_SET_COMPARE(htimx, channel, this->CCR);
}
uint32_t Timer::get_CCR(tim_channel channel) {
	this->CCR = __HAL_TIM_GET_COMPARE(this->htimx, channel);
	return this->CCR;
}

void Timer::set_CNT(uint32_t CNT_val) {
	this->CNT = CNT_val;
	__HAL_TIM_SET_COUNTER(htimx, CNT_val);
}
uint32_t Timer::get_CNT(void) {
	this->CNT = __HAL_TIM_GET_COUNTER(htimx);
	return this->CNT;
}

uint32_t Timer::channel_to_idx(tim_channel channel) {
	if(channel == Channel1) return 0;
	if(channel == Channel2) return 1;
	if(channel == Channel3) return 2;
	if(channel == Channel4) return 3;
	exception("channel_to_idx: invalid channel");
	return 0;
}
tim_channel Timer::idx_to_channel(uint32_t idx) {
	if(idx == 0) return Channel1;
	if(idx == 1) return Channel2;
	if(idx == 2) return Channel3;
	if(idx == 3) return Channel4;
	exception("idx_to_channel: invalid idx");
	return Channel1;
}


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
 * 	  by dividing the "APB bus clock" scaled from [SystemCoreClock], a.k.a [HCLK_Frequency], with the TimerPrescaler value.
 * 	  Details of the clock relations can be looked up in STM32CubeMx software's clock configuration section,
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

/* max_count determines the resolution,
 * e.g. for .1  precision, max_count is typically 1,000,
 *      for .01 precision, max_count is typically 10,000
 * pwm_frequency refers to the frequency of each PWM Cycle's period,
 * which is the time period between two rising edge of a typical pwm pulse
 */


void Timer::set_pwm_frequency(uint32_t pwm_period_cnt, uint32_t pwm_freq_Hz) {
	uint32_t max_freq_allowed = HAL_RCC_GetHCLKFreq() / APBx_division_factor;
	uint32_t timer_freq = pwm_period_cnt * pwm_freq_Hz;  
	if(timer_freq > max_freq_allowed) {
		exception("timer frequency exceeds max limit for pwm generation");
		return;
	}
	this->timer_frequency = timer_freq;
	this->prescaler = max_freq_allowed / timer_frequency - 1;
	this->ARR = max_freq_allowed / (prescaler + 1) / pwm_freq_Hz;
	set_prescaler(prescaler);
}


void Timer::set_timer_frequency(uint32_t timer_freq_Hz) {
	uint32_t max_freq_allowed = HAL_RCC_GetHCLKFreq() / APBx_division_factor;
	if(timer_freq_Hz > max_freq_allowed) {
		exception("timer frequency exceeds max limit");
		return;
	}
	this->prescaler = max_freq_allowed / timer_freq_Hz - 1;
	this->timer_frequency = max_freq_allowed / (this->prescaler + 1);
	set_prescaler(prescaler);
} 



void Timer::init_input_capture(uint32_t timer_freq_Hz) {
	if(resolution == TIM16Bit) set_ARR(0xFFFFFFFF);
	if(resolution == TIM32Bit) set_ARR(0xFFFF);
	set_timer_frequency(timer_freq_Hz);
	this->mode = InputCapture;
}






// Input: Periodic input capture interpreted as PWM pulse

/* PWM input: Typical use cases: PPM radio remote input, SR04 Ultrasonic sensor
 * Timer's counter counts at [timer_freq = max_cnt * pwm_freq]
 * Same as pwm_gen, max_count here is used to determine resolution.
 * Generally it is recommended to set the resolution so that the timer_freq is less than 1,000,000 Hz = 1Mhz
 *
 * Unlike other methods, max_count here is not used to set Autoreload,
 * To reduce the chances of counter overflow, ARR is set to be the largest 16-bit value 0xFFFF,
 * even though this edge case has been taken care.
 * It is required to have [# of Autoreload(ARR) count > # of counts within a pulse],
 * ARR count is 0xFFFF, # of counts within a pulse = pulse_width / (1/timer_freq).
 *
 * Note: Capture interrupt MUST be enabled, use CubeMx to enable it
 */

void Timer::init_pwm_input(uint32_t pwm_period_cnt, uint32_t pwm_freq_Hz) {
	uint32_t timer_freq = pwm_period_cnt * pwm_freq_Hz;
	uint32_t max_freq_allowed = HAL_RCC_GetHCLKFreq() / APBx_division_factor;
	if(timer_freq > max_freq_allowed) {
		exception("timer frequency exceeds max limit for pwm input");
		return;
	}
	init_input_capture(timer_freq);

	for(int i = 0; i < Num_Channels; i++) {
		input_capture[i].first_edge = 0;
		input_capture[i].pulse_width = 0;
		input_capture[i].pwm_input_ARR = pwm_period_cnt;
	}

	this->mode = PWM_Input;
}

void Timer::pwm_input_begin(tim_channel channel, logic_level pulse_polarity) {
	// interpreting high (logic 1) signals as pulses
	if(pulse_polarity == High) {
		set_input_capture_polarity(channel, InputCaptureOnRisingEdge);
	}

	//interpreting low (logic 0) signals as pulses
	if(pulse_polarity == Low) {
		set_input_capture_polarity(channel, InputCaptureOnFallingEdge);
	}
	
	input_capture[channel_to_idx(channel)].latched_level = pulse_polarity;

	input_capture_begin(channel, Interrupt);
}

void Timer::pwm_input_end(tim_channel channel) {
	input_capture_end(channel, Interrupt);	
}


static void timer_pwm_input_interrupt_task(stf::Timer *instance, 
                                HAL_TIM_ActiveChannel active_channel) {
	tim_channel channel;
	if(instance->mode == PWM_Input) {
		if(active_channel == HAL_TIM_ACTIVE_CHANNEL_1) channel = Channel1;
		if(active_channel == HAL_TIM_ACTIVE_CHANNEL_2) channel = Channel2;
		if(active_channel == HAL_TIM_ACTIVE_CHANNEL_3) channel = Channel3;
		if(active_channel == HAL_TIM_ACTIVE_CHANNEL_4) channel = Channel4;

		//pulses on high signals
		tim_ic* ic_struct = &(instance->input_capture[instance->channel_to_idx(channel)]);
		if(ic_struct->latched_level == High) {
			if(ic_struct->polarity == InputCaptureOnRisingEdge) {
				ic_struct->first_edge = HAL_TIM_ReadCapturedValue(instance->get_htimx(), channel);
				ic_struct->polarity = InputCaptureOnFallingEdge;
				__HAL_TIM_SET_CAPTUREPOLARITY(instance->get_htimx(), channel, InputCaptureOnFallingEdge);
			}
			else if(ic_struct->polarity == InputCaptureOnFallingEdge) {
				ic_struct->pulse_width = HAL_TIM_ReadCapturedValue(instance->get_htimx(), channel) 
							- ic_struct->first_edge;
				if(ic_struct->pulse_width < 0) {
					ic_struct->pulse_width += __HAL_TIM_GET_AUTORELOAD(instance->get_htimx()) + 1;
				}
				ic_struct->polarity = InputCaptureOnRisingEdge;
				__HAL_TIM_SET_CAPTUREPOLARITY(instance->get_htimx(), channel, InputCaptureOnRisingEdge);
			}
		}
		
		//pulses on low signals
		if(ic_struct->latched_level == Low) {
			if(ic_struct->polarity == InputCaptureOnFallingEdge) {
				ic_struct->first_edge = HAL_TIM_ReadCapturedValue(instance->get_htimx(), channel);
				ic_struct->polarity = InputCaptureOnRisingEdge;
				__HAL_TIM_SET_CAPTUREPOLARITY(instance->get_htimx(), channel, InputCaptureOnRisingEdge);
			}
			else if(ic_struct->polarity == InputCaptureOnRisingEdge) {
				ic_struct->pulse_width = HAL_TIM_ReadCapturedValue(instance->get_htimx(), channel) 
							- ic_struct->first_edge;
				if(ic_struct->pulse_width < 0) {
					ic_struct->pulse_width += __HAL_TIM_GET_AUTORELOAD(instance->get_htimx()) + 1;
				}
				ic_struct->polarity = InputCaptureOnFallingEdge;
				__HAL_TIM_SET_CAPTUREPOLARITY(instance->get_htimx(), channel, InputCaptureOnFallingEdge);
			}
		}
	}
}





void Timer::init_encoder(void) {
	if(resolution == TIM16Bit) set_ARR(0xFFFF);
	if(resolution == TIM32Bit) set_ARR(0xFFFFFFFF);
	this->mode = Encoder;
	this->encoder_count = 0;
	this->encoder_overflow = 0;
}

void Timer::encoder_begin(periph_mode mode) {
	// Polling mode can't automatically take care overflow
	if(mode == Polling) HAL_TIM_Encoder_Start(htimx, TIM_CHANNEL_ALL);
	
	// Overflow is automatically taken care in interrupt mode
	if(mode == Interrupt) HAL_TIM_Encoder_Start_IT(htimx, TIM_CHANNEL_ALL);
	reset_encoder();
}

static void timer_encoder_overflow_interrupt_task(stf::Timer *instance) {
	if(instance->mode != Encoder) return;
	if(instance->get_timer_resolution() == TIM16Bit) {
		int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(instance->get_htimx()); 
		// 32767 == 0x7FFF == 0xFFFF/2 (positive + negative), setting it to 32760 
		// to give it a bit redundency in case any wierd issue occur
		int16_t overflow_threshold = 32760;
		if(cnt <= -overflow_threshold) {
			instance->encoder_overflow -= overflow_threshold;
			instance->interrupt_safe_set_CNT(0);
			__HAL_TIM_SET_COUNTER(instance->get_htimx(), 0);
		}
		else if(cnt >= overflow_threshold) {
			instance->encoder_overflow += overflow_threshold;
			instance->interrupt_safe_set_CNT(0);
			__HAL_TIM_SET_COUNTER(instance->get_htimx(), 0);
		}
	}
	if(instance->get_timer_resolution() == TIM32Bit) {
		int32_t cnt = (int32_t)__HAL_TIM_GET_COUNTER(instance->get_htimx());
		// 2,147,483,647‬ == 0x7FFFFFFF setting it to 2000,000,000 to make it whole and 
		// to give it a bit redundency in case any wierd issue occur
		int32_t overflow_threshold = 2000000000;
		if(cnt <= -overflow_threshold) {
			instance->encoder_overflow -= overflow_threshold;
			instance->interrupt_safe_set_CNT(0);
			__HAL_TIM_SET_COUNTER(instance->get_htimx(), 0);
		}
		else if(cnt >= overflow_threshold) {
			instance->encoder_overflow += overflow_threshold;
			instance->interrupt_safe_set_CNT(0);
			__HAL_TIM_SET_COUNTER(instance->get_htimx(), 0);
		}
	}
	
}

/*** interrupt handler callbacks ***/

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htimx) {
	for(uint32_t i = 0; i < num_tims; i++) {
		if(active_tims[i]->get_htimx() == htimx) {
			timer_pwm_input_interrupt_task(active_tims[i], htimx->Channel);
			timer_encoder_overflow_interrupt_task(active_tims[i]);
			timer_input_captured_interrupt_task(active_tims[i], htimx->Channel);
		}
	}
}

// weak def of period elasped callback is already overwritten in main.c by freeRTOS
void call_this_inside_HAL_TIM_PeriodElaspedCallback(TIM_HandleTypeDef *htimx) {
	for(uint32_t i = 0; i < num_tims; i++) {
		if(active_tims[i]->get_htimx() == htimx) {
			timer_period_elasped_interrupt_task(active_tims[i]);
		}
	}
}







#endif
