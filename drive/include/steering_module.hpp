#pragma once

#include <libhal/input_pin.hpp>
#include <libhal/servo.hpp>
#include <libhal/motor.hpp>
#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>

#include "../../drivers/include/tmag5273.hpp"
namespace sjsu::drive {
/**
 * @brief A struct that holds the steering and drive motors.
 * 
 */
struct steering_module
{
    hal::actuator::rmd_mc_x_v2* steer; //might change thsi to rmd_mcx_v2
    std::optional<hal::motor*> propulsion;
    std::optional<hal::input_pin*> limit_switch;
    // drivers::tmag5273* tmag;
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

struct start_wheel_setting{
    std::uint16_t steer_id;
    std::uint16_t prop_id;
    float geer_ratio;
    bool reversed;
    int homing_offset;
    int homing_angle;
    hal::rpm max_speed;

};


}  // namespace sjsu::drive
