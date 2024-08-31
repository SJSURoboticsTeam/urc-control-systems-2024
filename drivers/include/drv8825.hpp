#pragma once
#include <array>
#include <libhal/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/servo.hpp>

namespace sjsu::drivers {

class drv8825 : public hal::servo {
    public:
        enum class step_factor{
            one=0,//full step
            one_over_2,// 1/2 step
            one_over_4,// 1/4 step
            one_over_8,// 1/8 step
            one_over_16,// 1/16 step
            one_over_32// 1/32 step
        };
    private:
        hal::output_pin& m_direction_pin;
        hal::output_pin& m_step_pin;
        hal::steady_clock& m_clock;
        hal::time_duration m_step_period;
        std::array<hal::output_pin*,3> m_mode_pins;

        long m_partial_steps=0;// total 1/32 steps taken from pos 0 (so reverse directions steps subtract)
        double m_partial_steps_to_deg=1;
        step_factor m_step_factor = step_factor::one;

    public:
        drv8825(
            hal::output_pin& p_direction_pin, 
            hal::output_pin& p_step_pin, 
            hal::steady_clock& p_steady_clock, 
            step_factor p_step_factor, 
            int p_steps_per_rotation, 
            hal::time_duration p_step_period,
            std::array<hal::output_pin*,3> p_mode_pins
        );
        //goes counter-clockwise for negative numbers
        void step(long p_steps);
        void set_step_factor(step_factor p_step_factor);
        // how many 1/32 steps from position 0
        long get_partial_steps();
        void set_position(hal::degrees p_position);
        hal::degrees get_position();
        //can lose resolution
        void set_position(double p_position);
        void driver_position(hal::degrees p_position);
};
}