#ifndef __STF_TIMER_H
#define __STF_TIMER_H


#include "stf_dependancy.h"


#ifdef HAL_TIM_MODULE_ENABLED


#define TIM_CH1 TIM_CHANNEL_1
#define TIM_CH2 TIM_CHANNEL_2
#define TIM_CH3 TIM_CHANNEL_3
#define TIM_CH4 TIM_CHANNEL_4

#define TIM_Active_CH1 HAL_TIM_ACTIVE_CHANNEL_1
#define TIM_Active_CH2 HAL_TIM_ACTIVE_CHANNEL_2
#define TIM_Active_CH3 HAL_TIM_ACTIVE_CHANNEL_3
#define TIM_Active_CH4 HAL_TIM_ACTIVE_CHANNEL_4

#define TIM_IC_RisingEdge  TIM_INPUTCHANNELPOLARITY_RISING
#define TIM_IC_FallingEdge TIM_INPUTCHANNELPOLARITY_FALLING
#define TIM_IC_BothEdge    TIM_INPUTCHANNELPOLARITY_BOTHEDGE


static const size_t TIM_NUM_CHANNELS = 4;
static const size_t MAX_NUM_TIMS = 25;


enum PulseLevel {
    TIM_PulseOnHigh = 1,
    TIM_PulseOnLow  = 0
};


//Refer to datasheet
enum TIM_Resolution {
    TIM_16bit = 0,
    TIM_32bit = 1
};


namespace stf {
    struct TIM {
        //Aux-struct for saving RAM space in case input capture is not used
        struct TIM_IC {
            volatile uint32_t ICpolarity[TIM_NUM_CHANNELS + 1];
            volatile int32_t IC_FirstEdge[TIM_NUM_CHANNELS + 1];
            volatile int32_t PulseWidth[TIM_NUM_CHANNELS + 1];
            volatile bool isUsedForPwmInput;
            volatile PulseLevel pulse_polarity;
            uint32_t pwm_input_max_count;
        };

        TIM_HandleTypeDef * htim;

        volatile uint32_t TimerPrescaler;

        //ARR: (Counter) Auto Reload Register value, a.k.a timer period
        //ARR = max_count = *htim->init->Period
        volatile uint32_t ARR; 

        volatile uint32_t CCR; //Capture/Compare (context dependent)
        volatile uint32_t CNT;

        uint32_t APBx_Div_Factor;
        uint32_t ActualFreq;
        TIM_Resolution xBitTIM;

        volatile bool isEncMode;
        volatile int32_t ENC_CNT;
        volatile int32_t ENC_OverFlow;

        volatile TIM_IC * IC_fields;

        static uint16_t numActiveTIMs;
        static TIM* ActiveTIMs[MAX_NUM_TIMS];

        TIM(TIM_HandleTypeDef *, uint32_t, TIM_Resolution);
        ~TIM();

        /*=======================Universal Functions=================================*/
        uint32_t getARR();
        uint32_t getCCR(uint32_t);
        uint32_t getCNT();
        uint32_t getPrescaler();

        void setARR(uint32_t);
        void setCCR(uint32_t, uint32_t);
        void setCNT(uint32_t);
        void setPrescaler(uint32_t);
        
        static uint8_t tim_channel_index(uint32_t);
        /*===========================================================================*/

        /*=======================Basic Counting & Interrupt==========================*/
        bool initBasicCounting(uint32_t, uint32_t);

        bool setFrequency(uint32_t);
        void countBegin();
        void countEnd();

        void countBeginIT();
        void countEndIT();
        uint32_t getCount();
        /*===========================================================================*/

        /*=============================PWM Input/Output==============================*/

        //PWM Generation
        bool initPwmOut(uint32_t, uint32_t);
        bool setPwmFrequency(uint32_t, uint32_t);
        void pwmGenBegin(uint32_t);
        void pwmGenEnd(uint32_t);
        void setPwmDutyCycle(uint32_t, uint32_t);
        void pwmWrite(uint32_t, double);

        //PWM Capture (Essentially Input Capture interpreted as PWM signal)
        bool initPwmIn(TIM_IC *, uint32_t, uint32_t);
        void pwmIcBegin(uint32_t, PulseLevel);
        void pwmIcEnd(uint32_t);
        int32_t getPulseWidth(uint32_t);
        double pwmRead(uint32_t);
        /*===========================================================================*/

        /*================Input Capture(Interrupt Mode Only)========================*/
        uint32_t initIC(TIM_IC *, uint32_t);
        void setICPolarity(uint32_t, uint32_t);
        void icBeginIT(uint32_t);
        void icEndIT(uint32_t);
        uint32_t getCapVal(uint32_t);
        /*===========================================================================*/

        /*======================(Quadrature) Encoder Mode============================*/
        void initEnc();
        void encBegin();
        void encBegin_IT();
        void encEnd();
        void encEnd_IT();
        void resetEnc();
        int32_t getEncCNT();
        int32_t getOverFlowPart_32bit();
        /*===========================================================================*/
    };
}


//Interrupt handler
__weak void timPE_IT_CallBack(stf::TIM * instance);
__weak void timSysT_IT_CallBack(stf::TIM * instance);

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef * htim);
__weak void timIC_IT_CallBack(stf::TIM * instance, HAL_TIM_ActiveChannel active_channel);

#endif 


#endif /* !__STF_TIMER_H */