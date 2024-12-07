#pragma once 

#include "./vector2.hpp"
#include <span>
#include <libhal/units.hpp>
#include "steering_module.hpp"

namespace sjsu::drive {
/**
 * @brief A utility that calculates wheel rotations and speeds for ackermann steering
 * 
 * @warning Maximum angular velocity can only be used if the scale factors for these members are set correctly. (They are currently aren't)
 * 
 */
class ackermann_steering {
    public:
        /**
         * @brief Construct an ackermann steering manager
         * 
         * @param p_wheel_locations The locations of each wheel, as a vector in 2d Cartesian space
         * @param p_wheel_settings_span A location to store the calculated settings for each wheel
         * @param p_maximum_wheel_speed Unused. The maximum linear speed of the steering manager.
         * @param p_maximum_angular_velocity Unused. The maximum angular speed (rate of turn) of the steering manager.
         * 
         * @note `p_maximum_angular_velocity` and `p_maximum_wheel_speed` are unused at the moment, since this requires some scaling factors to be correctly set.
         */
        ackermann_steering(std::span<vector2> p_wheel_locations, std::span<wheel_setting> p_wheel_settings_span, hal::rpm p_maximum_wheel_speed, hal::rpm p_maximum_angular_velocity);

        /**
         * @brief Get the current wheel settings, last computed by a call to `calculate_wheel_settings()`
         * 
         * @return A span of wheel settings, with each element corresponding to the setting of that wheel.
         */
        std::span<wheel_setting> get_wheel_settings();

        /**
         * @brief Calculate the angles and wheel speeds of all the wheels configured in this object.
         * 
         * @param p_signed_turning_radius Location of the turning circle center.
         * @param p_heading Direction of rover travel.
         * @param p_wheel_speed Speed of an imaginary wheel
         * @return A span of wheel settings, with each element corresponding to the setting of that wheel.
         */
        std::span<wheel_setting> calculate_wheel_settings(float p_signed_turning_radius, hal::degrees p_heading, hal::rpm p_wheel_speed);
    private:
        std::span<vector2> m_wheel_locations;
        std::span<wheel_setting> m_wheel_settings;
        hal::rpm m_maximum_wheel_speed, m_maximum_angular_velocity;
};

}