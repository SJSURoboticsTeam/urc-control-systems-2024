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

        struct ctor_params {
            hal::output_pin& direction_pin;
            hal::output_pin& step_pin;
            hal::steady_clock& steady_clock;
            step_factor motor_step_factor;
            int full_steps_per_rotation;
            hal::time_duration step_half_period;
            std::array<hal::output_pin*,3> mode_pins;
        };

        explicit drv8825(ctor_params const& p_params);

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

    private:
        hal::output_pin& m_direction_pin;
        hal::output_pin& m_step_pin;
        hal::steady_clock& m_clock;
        hal::time_duration m_step_half_period;
        std::array<hal::output_pin*,3> m_mode_pins;

        // total 1/32 steps taken from pos 0 (so reverse directions steps subtract)
        long m_partial_steps=0;
        int m_full_steps_per_rotation=1;
        step_factor m_step_factor = step_factor::one;
};
}