/* Author: Hector Montenegro */

#ifndef __PID_H_
#define __PID_H_

#include <iostream>

template <class T> // This PID code can handle (math)vector value
class INC_PID_Controller {

    /*
     * This is the generic (time domain) PID controller class, which can be configured to be
     * either P, PD, PI, or PID controller.
     *
     * This PID controller can be used on various types, such as integers, floating point,
     * mathematical vectors, or even matrices, as long as the operators +, -, *, / are
     * defined for the object. This class provides handy methods for the case when the derivative term and/or integral term
     * can be directly measured instead of derived based on the error measurement itself,
     * which can be useful when data from alternative measurement sources are less noisy.
     * For example, when a speed sensor is more reliable than deriving speed from a distance
     * sensor, one can use the calculate(curr_error, error_rate) function and feed both sensors
     * data into the pid calculation.
     *
     *
     * The general PID formula:
     *  u(t) = Kp * e(t) + Ki * intergral(e(t)) + Kd * d(e(t))/dt
     *       = [Kp, Ki, Kd] * [error, error's integral, erorr's derivative]^T
     * Values of constants Kp, Ki, Kd are normally determined by experiments(trial on error),
     * which adjust the weights on the proportional, integral and derivative terms respectively.
     *
     * Conceptual Explaination:
     *  control: control algorithm often involves a physical variable to be controlled,
     *           such as location, velocity, temperature, etc. The variable is sometimes
     *           a vector.
     *           In order to control a physical variable, one needs to be able to measure the
     *           actual value of this variable, and knowing a desired value to which the
     *           control system will drive the actual value.
     *
     *  e(t) : the error value with respect to time. Error is usually defined as
     *          error(t) = (Desired value at time t) - (Actual value at time t)
     *          the goal of the control system is simply trying to reduce this error to be zero
     *
     *  Proportional Term Kp*e(t) : the error amount times a ratio to be mapped to the amount of
     *                              system's response, the intuitive idea is simply the larger
     *                              the error is, the bigger the response is, the faster the error
     *                              is minimized
     *  Integral Term Ki * integral(e(t)): the integral term records the cumulative error sum,
     *                                if a system keeps having non-zero steady error for a prolonged
     *                                time, the integral term will in time cumulates a large error sum
     *                                to contribute to a bigger system response. Steady error can come
     *                                from the overdamping of the derivative term, or external interference
     *                                to the system.
     *  Derivative Term Kd * derivative(e(t)) : the derivative term corresponds to the
     *                             rate of error over time, which is used to dampen the overshoot
     *                             when actual value is oscillating around the desired value with
     *                             a large momentum due to system reponse being too fast/drastic.
     *                             This term is usually on the mathematical opposite direction of the other 2 terms.
     *
     * The specific PID formula used here with some unit scaling is discretized as:
     *  u(t) =   [Kp * e(t)]
     *         - [Kd * (e(t) - e(t-1))/period_ms]
     *         + [Ki * sum{ e(0)*period_s + e(1)*period_s ..... + e(t)*period_s }]
     *  period_ms = millisecond time difference between the current error measurement and the previous error measuremnt
     *  period_s = unit-in-second version of the above
     *
     *  the derivative and integral term uses different units to keep the Kp, Kd, Ki constants
     *  relatively at the similar scale, or else the integral term for example
     *  might get extremely small in scare if the unit is also milliseconds, imagine a typical
     *  control system having a 10ms period (100Hz), and the reponse level is mapped to -100%~100%,
     *  the integral constant will normally be below 10^-5 to have this term's scale makes sense,
     *  practically speaking.
     *  The scaling is generally arbitrary because it's linear formular and the constant is determined
     *  by hand based on experimenting results.
     *
     *  When we say "tuning the pid", it means to determine the pid constants Kp, Ki, Kd by experiments.
     *  A general rule-of-thumb procedure for tuning pid constants:
     *   1. first, try different Kp values of variouse scale, and nailed it at a specific
     *      range that provides enough system response to correct the error and even produces
     *      some overshoot oscillations deliberately
     *   2. then, play around the Kd value to try to dampen the overshoot oscillation to make the
     *      system stable and smooth, but at the same time not undershoot
     *   3. last, add a small Ki to counter any overdamping or external disturbance to the system
     *
     *
     *  PID algorithm is a simple but yet effective tool for LTI(linear time invariant) systems
     *    and often is also a effective approximation for linear with small time variant systems.
     *  For systems running in more complicated environment, please use some more advanced controls,
     *  or manipulate the PID "constants" to be turned to variable over other factors. All the
     *  pid constants are public variable that can be easily accessed at real-time.
     */


public:
    // proportional, integral and derivative constants
    float Kp, Ki, Kd;

    INC_PID_Controller(float Kp, float Ki, float Kd) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }

    void update_pid_consts(float Kp, float Ki, float Kd) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }

    // Fixed time interval mode, often coupled with a timer callback
    void init(float frequency_Hz) {
        this->period_ms = (1.00 / frequency_Hz) * 1000.00;
        this->is_first_time = true;
        this->is_fixed_time_interval = true;
    }

    // Dynamic time interval mode, need to pass in a function handle to measure the curr time in millisec
    void init(float (*millis)(void)) {
        this->millis_func = millis;
        this->is_first_time = true;
        this->is_fixed_time_interval = false;
    }


    /** calculate the PID output **/
    //  error(t) = (Desired value at time t) - (Actual value at time t)
    /* used when there is only one data source for measuring the error  */
    T calculate(T curr_error) {
        if(is_first_time) return first_time_handle(curr_error);
        if (is_second_time) return second_time_handle(curr_error);
        if (is_third_time) return third_time_handle(curr_error);
        float period = get_period();
        T derivative = (curr_error - prev_error) / period;
        this->integral += curr_error * period / 1000.000; // scale Ki to be in similar range constants
        T output = (Kp * (curr_error - prev_error)) + (Kd * derivative) + (Ki * integral);
        prev_error = curr_error;
        return output;
    }

    /* methods below are normally used when there exists
       direct measurement of error_rate or error_sum that is
       more accurate or stable than the calculated rate/sum
       on noisy error measurement itself  */

    /* used when [error measurement] and [error derivative measurement] come
       from different data sources  */
    T calculate(T curr_error, T error_rate) {
        if(is_first_time) return first_time_handle(curr_error);
        float period = get_period();
        T derivative = (curr_error - prev_error) / period;
        this->integral += curr_error * period / 1000.000;
        T output = (Kp * curr_error) + (Kd * error_rate) + (Ki * integral);
        return output;
    }

    /* used when [error] and [error integral] both
       have their respective way of measuring directly */
    T calculate_s(T curr_error, T error_sum) {
        if(is_first_time) return first_time_handle(curr_error);
        if (is_second_time) return second_time_handle(curr_error);
        float period = get_period();
        T derivative = (curr_error - prev_error) / period;
        T output = (Kp * curr_error) + (Kd * derivative) + (Ki * error_sum);
        return output;
    }

    /* used when [error], [error derivative] and [error integral]
       all have their respective way of measuring directly */
    T calculate(T curr_error, T error_rate, T error_sum) {
        return (Kp * curr_error) + (Kd * error_rate) + (Ki * error_sum);
    }



private:

    bool is_first_time, is_second_time, is_third_time;
    bool is_fixed_time_interval;
    T integral;
    T prev_error;
    float period_ms; //unit: millisec
    float prev_time_ms; // unit: millisec
    float (*millis_func)(void);

    T first_time_handle(T curr_error) {
        this->integral = curr_error - curr_error; // a workaround to get zero/zero_vector of a generic type
        this->prev_error = curr_error;
        this->is_first_time = false;
        return Kp * curr_error;
    }

    T second_time_handle(T curr_error) {
		this->integral = curr_error - curr_error; // a workaround to get zero/zero_vector of a generic type
		this->prev_error = curr_error;
		this->is_second_time = false;
		return Kp * curr_error;
	}

    T third_time_handle(T curr_error) {
		this->integral = curr_error - curr_error; // a workaround to get zero/zero_vector of a generic type
		this->prev_error = curr_error;
		this->is_third_time = false;
		return Kp * curr_error;
	}

    float get_period() {
        if(this->is_fixed_time_interval) {
            return this->period_ms;
        }
        else {
            float curr_time_ms = this->millis_func();
            float dt = curr_time_ms - this->prev_time_ms;
            this->prev_time_ms = curr_time_ms;
            // std::cout << dt << std::endl; // debug
            return dt;
        }
    }

};

#endif
