#ifndef __STF_TIMER_H
#define __STF_TIMER_H 

#include "stf_dependancy.h"

#ifdef HAL_TIM_MODULE_ENABLED

#define Max_Num_TIMs 20
#define Num_Channels 4

enum tim_channel {
	Channel1 = TIM_CHANNEL_1,
	Channel2 = TIM_CHANNEL_2,
	Channel3 = TIM_CHANNEL_3,
	Channel4 = TIM_CHANNEL_4
};


enum tim_resolution {
	TIM16Bit,
	TIM32Bit
};


enum tim_capture_edge {
	InputCaptureOnRisingEdge = TIM_INPUTCHANNELPOLARITY_RISING,
	InputCaptureOnFallingEdge = TIM_INPUTCHANNELPOLARITY_FALLING,
	InputCaptureOnBothEdge = TIM_INPUTCHANNELPOLARITY_BOTHEDGE 
};


enum tim_mode {
    Uninitialized,
    Basic,
    PWM_Generation,
    PWM_Input,
    InputCapture,
    Encoder
};

/* 
 * Since this framework utilizes FreeRTOS, 
 * basic timer interrupt function is omitted 
 * from the Timer class implementation.
 * To use a timer interrupt, please use 
 * software timer via FreeRTOS
 */
namespace stf {


    typedef struct {
        volatile tim_capture_edge polarity;
        volatile int32_t first_edge;
        volatile int32_t pulse_width;
        volatile logic_level latched_level;
        volatile uint32_t pwm_input_ARR;
    }tim_ic;
	class Timer {
	private:
		TIM_HandleTypeDef *htimx;
		/* prescaler value that determines the clock freq of the timer*/
		volatile uint32_t prescaler;
		
		/* ARR == Auto Reload Register, determines timer period
		 * ARR value is the maxcount value a timer's counter can reach 
		 * before auto reload back to count 0
		 */
		volatile uint32_t ARR; 

		/* CCR == Capture or Compare Register 
		 * depending on the context, this register can
		 * be for storing captured value 
		 * or setting compare threshold 
		 */
		volatile uint32_t CCR; 

		volatile uint32_t CNT; // current count value

		/* check clock tree in CubeMx software and datasheet, it can be 
		 * APB1 bus or APB2 bus etc, depending on which timer
		 * to instantiate. 
         * Usually the Advanced Timers are hooked on APB1, 
         * where the Normal or Basic Timers are hooked on APB2
		 */
		uint32_t APBx_division_factor; 
		
		/* Specify this is a 16bit timer or 32bit timer*/
		tim_resolution resolution;
		
        /* timer frequency determines the frequency of each timer's counter count*/
		uint32_t timer_frequency;

        

        
    
	public:
        tim_ic input_capture[Num_Channels];
        volatile tim_mode mode;
        volatile int64_t encoder_count = 0;
        volatile int64_t encoder_overflow = 0;

		Timer(TIM_HandleTypeDef *htimx, uint32_t APBx_division_factor, 
              tim_resolution resoltion);
		~Timer();


        void set_prescaler(uint32_t prescaler_val);
        uint32_t get_prescaler(void);
    
        // CCR : compare or capture register
        void set_CCR(tim_channel channel, uint32_t CCR_val);
        uint32_t get_CCR(tim_channel channel);

        // ARR : auto reload register
        void set_ARR(uint32_t ARR_val);
        uint32_t get_ARR(void);

        // CNT : timer counts
        void set_CNT(uint32_t CNT_val);
        uint32_t get_CNT(void);

        static uint32_t channel_to_idx(tim_channel channel);
        static tim_channel idx_to_channel(uint32_t idx);
        
        void set_timer_frequency(uint32_t timer_freq_Hz);

        /*** basic counting and basic timer interrupt***/
        void init(uint32_t max_count, uint32_t timer_freq_Hz) {
            set_ARR(max_count);
            set_timer_frequency(timer_freq_Hz);
        }

        void counting_begin(periph_mode mode = Interrupt) {
            if(mode == Polling) HAL_TIM_Base_Start(htimx);
            if(mode == Interrupt) HAL_TIM_Base_Start_IT(htimx);
        }

        void counting_end(periph_mode mode = Interrupt) {
            if(mode == Polling) HAL_TIM_Base_Stop(htimx);
            if(mode == Interrupt) HAL_TIM_Base_Stop_IT(htimx);
        }

        inline uint32_t get_count(void) {return get_CNT();}


        /*** pwm generation ***/
        /* pwm period's unit is in num of timer's counter counts */ 

        void init_pwm_generation(uint32_t pwm_period_cnt, uint32_t pwm_freq_Hz) {
            set_pwm_frequency(pwm_period_cnt, pwm_freq_Hz);
            this->mode = PWM_Generation;
        }

        void set_pwm_frequency(uint32_t pwm_period_cnt, uint32_t pwm_freq_Hz);
        void set_pwm_duty_cycle_cnt(tim_channel channel, uint32_t duty_cycle) {
            set_CCR(channel, duty_cycle);
        }

        template <typename Type>
        void set_pwm_duty_cycle(tim_channel channel, Type duty_cycle_in_percent) {
            if((double)duty_cycle_in_percent > 100.00f) duty_cycle_in_percent = (Type)100.00f;
            uint32_t cnt = duty_cycle_in_percent * (Type)this->ARR / (Type)100.00f;
            set_pwm_duty_cycle_cnt(channel, cnt);
        }


        void pwm_generation_begin(tim_channel channel) {
            HAL_TIM_PWM_Start(htimx, channel);
            set_pwm_duty_cycle(channel, 0);
        }

        void pwm_generation_end(tim_channel channel) {
            HAL_TIM_PWM_Stop(htimx, channel);
        }

        /*** input capture ***/
        void init_input_capture(uint32_t timer_freq_Hz);

        //capture at rising vs falling edge
        void set_input_capture_polarity(tim_channel channel, 
                                        tim_capture_edge polarity) {
            input_capture[channel_to_idx(channel)].polarity = polarity;
            __HAL_TIM_SET_CAPTUREPOLARITY(htimx, channel, polarity);
        }

        // omitted DMA mode bc it's a little different
        void input_capture_begin(tim_channel channel, periph_mode mode = Polling) {
            if(mode == Polling) HAL_TIM_IC_Start(htimx, channel);
            if(mode == Interrupt) HAL_TIM_IC_Start_IT(htimx, channel);
        }
        void input_capture_end(tim_channel channel, periph_mode mode = Polling) {
            if(mode == Polling) HAL_TIM_IC_Stop(htimx, channel);
            if(mode == Interrupt) HAL_TIM_IC_Stop_IT(htimx, channel);
        }

        uint32_t read_input_captured_value(tim_channel channel) {
            return HAL_TIM_ReadCapturedValue(htimx, channel);
        }

        
        /*** pwm input (interrupt must be enabled!)***/ 
        void init_pwm_input(uint32_t pwm_period_cnt, uint32_t pwm_freq_Hz);
        // utilizes input capture interrupt to implement
        void pwm_input_begin(tim_channel channel, logic_level pulse_polarity);
        void pwm_input_end(tim_channel channel);
        int32_t get_pulse_width_cnt(tim_channel channel) {
            return input_capture[channel_to_idx(channel)].pulse_width;
        }

        // return pulse width as percentage of one pwm_period, which is captured_cnt/total_cnt_in_one_period
        template <typename Type>
        Type get_pulse_width(tim_channel channel) {
            return (Type)get_pulse_width_cnt(channel) * (Type)100.00f 
                    / (Type)input_capture[channel_to_idx(channel)].pwm_input_ARR;
        }

        /*** encoder mode ***/
        void init_encoder(void);
        // When using interrupt mode, overflow is automatically resolved within Interrupt task function
        void encoder_begin(periph_mode mode =  Polling);
        void encoder_end(periph_mode mode = Polling) {
            if(mode == Polling) HAL_TIM_Encoder_Stop(htimx, TIM_CHANNEL_ALL);
            if(mode == Interrupt) HAL_TIM_Encoder_Stop_IT(htimx, TIM_CHANNEL_ALL);
        }
        void reset_encoder(void) {
            set_CNT(0);
            this->encoder_count = 0;
            this->encoder_overflow = 0;
        }
        int64_t get_encoder_count(void) {
            if(resolution == TIM16Bit) encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(htimx) + encoder_overflow;
            if(resolution == TIM32Bit) encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(htimx) + encoder_overflow;
            return encoder_count;
        }
        
        


        /********************/
        inline TIM_HandleTypeDef *get_htimx(void) {return htimx;}
        inline uint32_t get_timer_freq_Hz(void) {return timer_frequency;}
        inline tim_resolution get_timer_resolution(void) {return resolution;}
        inline uint32_t get_APBx_division_factor(void) {return APBx_division_factor;}
        inline int64_t get_encoder_overflow(void) {return encoder_overflow;}
        // stm32 interrupt is not friendly with C++ OOP feature, inline function is safer to use
        inline void interrupt_safe_set_CNT(uint32_t CNT_val) {CNT = CNT_val;}
	};
}

__weak void timer_input_captured_interrupt_task(stf::Timer *instance, 
                                HAL_TIM_ActiveChannel active_channel);
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htimx);


__weak void timer_period_elasped_interrupt_task(stf::Timer *instance);
// weak def of period elasped callback is already overwritten in main.c by freeRTOS
extern "C" void call_this_inside_HAL_TIM_PeriodElaspedCallback(TIM_HandleTypeDef *htimx);


#endif
#endif