#pragma once
#include <libhal/units.hpp>

namespace sjsu::drive {
    /**
     * @brief This structure defines a valid steering configuration for the drive system.
     * 
     */
    struct drive_configuration {
        /**
         * @brief Steering Angle corresponds to the turning radius. 
         * 
         *  - Turning 90 degrees puts the rover in spin mode, with spinning right as forward.
         *  - Turning 0 degrees moves the rover straight.
         *  - Turning -90 degrees puts the rover is spin mode, with spinning left as forward.
         * 
         *  - Negative values are left turns.
         *  - Positive values are right turns.
         * 
         * Specifically, this is the angle of an imaginary wheel one unit in front of the rover.
         * The equation is `steering_radius = 1/tan(steering_angle)`
         * 
         * @note Values can be beyond 90 and -90, but these correspond to rover moving "backwards". So a 180 degree steering angle corresponds to the rover moving straight backwards.  
         */
        hal::degrees steering_angle = 0.0;
        /**
         * @brief The rover direction of travel, relative to its front (Red button).
         * 
         * @note The rover always has a direction of travel, even when turning. 
         * Turning the rover will rotate the direction the front is facing, but not
         * its direction of travel, relative to its front.
         */
        hal::degrees wheel_heading = 0.0;
        /**
         * @brief Wheel speed as a percentage.
         * 
         * @note THIS IS NOT THE MAX SPEED IN RPMS.  If we want to change this to rpms, we should move away from
         * using `hal::motor.`
         * @note To change the actual maximum speed of the motors, you have to change it in the `platforms/lpc4078.cpp`
         * 
         */
        float wheel_speed = 0.0;
    };


    /**
     * @brief This structure contains the rate of change of each member of `drive_configuration`, reltative to seconds.
     * 
     */
    struct drive_configuration_rate {
        using degrees_per_second = float;
        using percents_per_second = float;
        /**
         * @brief Rate at which the steering angle is changing.
         * In degrees per second to keep units consistent with `drive_configuration`
         */
        degrees_per_second steering_angle_rate = 0.0;
        /**
         * @brief Rate at which the wheel heading is changing.
         * In degrees per second to keep units consistent with `drive_configuration`
         */
        degrees_per_second wheel_heading_rate = 0.0;
        /**
         * @brief Rate at which the wheel speed is changing.
         * 
         * In Percents pers second, since the wheel speed is a percent.
         * 
         */
        percents_per_second wheel_speed_rate = 0.0;
    };

    /**
     * @brief This structure contains the sensitivity of each member of `drive_configuration`.
     * 
     */
    struct drive_configuration_sensitivity {
        /**
         * @brief How quickly the system responds to differences between the target and the current steering angle.
         * In (degrees/sec)/(degrees), since it is a scaling factor.
         * 
         * @note This unit should be hertz, but frequency doesn't really mean anything here.
         */
        float steering_angle = 0.0;
        /**
         * @brief How quickly the system responds to differences between the target and the current wheel heading.
         * In (degrees/sec)/(degrees), since it is a scaling factor.
         * 
         * @note This unit should be hertz, but frequency doesn't really mean anything here.
         */
        float wheel_heading = 0.0;
        /**
         * @brief How quickly the system responds to differences between the target and the current wheel speeds.
         * In (percent/sec)/(percent), since it is a scaling factor.
         * 
         * @note This unit should be hertz, but frequency doesn't really mean anything here.
         */
        float wheel_speed = 0.0;
    };
}