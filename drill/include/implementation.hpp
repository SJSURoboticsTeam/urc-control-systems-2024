#pragma once


#include "../../drivers/include/drv8825.hpp"
#include "../../drivers/include/soil_sensor_sht21.hpp"
#include <cmath>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>



namespace sjsu::drill
{
    class drill_class
    {
        public:

            drill_class(

                hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> motor, 
                hal::v5::strong_ptr<hal::steady_clock> clock,
                hal::v5::strong_ptr<sjsu::drivers::drv8825> step_motor_driver,
                hal::v5::strong_ptr<sjsu::drivers::sht21> soil_sensor

                 );
                 
            void stop();
            
            void set_velocity(int rpm);

            // rf



        private:     

            hal::v5::strong_ptr<hal::actuator::rmd_mc_x_v2> m_drill;
            hal::v5::strong_ptr<hal::steady_clock> m_clock;
            hal::v5::strong_ptr<sjsu::drivers::drv8825> m_stepper_driver;
            hal::v5::strong_ptr<sjsu::drivers::sht21> m_soil_sensor;
            bool direction = true;


    };
}