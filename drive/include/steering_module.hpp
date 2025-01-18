#pragma once

#include <libhal/servo.hpp>
#include <libhal/motor.hpp>
#include "../../drivers/include/tmag5273.hpp"
namespace sjsu::drive {
/**
 * @brief A struct that holds the steering and drive motors.
 * 
 */
struct steering_module
{
    hal::servo* steer; //might change thsi to rmd_mcx_v2
    hal::motor* propulsion;
    drivers::tmag5273* tmag;
};


/**
 * @brief Setting of a single wheel.
 * 
 */
struct wheel_setting {
    /**
     * @brief Angle of the steering motor. Expects 0 to be straight forward
     */
    hal::degrees angle;
    /**
     * @brief Speed of the propulsion motor. This is a percentage, out of 100, not an rpm.
     * 
     * @note THIS DOES CORRESPOND WITH RPM RIGHT NOW DUE TO HOW `hal::motor` IS SET UP.
     * 
     */
    hal::rpm wheel_speed;
};


}  // namespace sjsu::drive
