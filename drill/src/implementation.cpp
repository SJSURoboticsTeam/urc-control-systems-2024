#include "../include/implementation.hpp"
#include <cmath>
#include <cstdlib>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/units.hpp>
#include <../resource_list.hpp>
#include <cassert>

using namespace std::chrono_literals;
using namespace hal::literals;

namespace sjsu::drill {

    drill_class::drill_class(  
        hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> motor, 
        hal::v5::strong_ptr<hal::steady_clock> clock,
        hal::v5::strong_ptr<sjsu::drivers::drv8825> step_motor_driver,
        hal::v5::strong_ptr<sjsu::drivers::sht21> soil_sensor
    ):
    m_drill(motor), 
    m_clock(clock), 
    m_stepper_driver(step_motor_driver),
    m_soil_sensor(soil_sensor)
    {

    }

    void drill_class::stop()
    {
        m_drill->velocity_control(0);
    }

    void drill_class::set_velocity(int rpm)
    {


        m_drill->velocity_control(rpm);

    }




    // void drill_class::step(long p_steps)
    // {
    //     m_partial_steps += p_steps * (32 >> static_cast<int>(m_step_factor));
    //     if (p_steps < 0) {
    //         p_steps *= -1;
    //         m_direction_pin->level(false);
    //     } 
        
    //     else {
    //         m_direction_pin->level(true);
    //         }
    //         // 650ns delay for direction change on datasheet (extra 50ns for tolerance)
    //         hal::delay(*m_clock, 700ns);

    //     for (; p_steps > 0; p_steps--) {
    //             m_step_pin->level(true);
    //             hal::delay(*m_clock, m_step_half_period);
    //             m_step_pin->level(false);
    //             hal::delay(*m_clock, m_step_half_period);
    //         }

    // }
}