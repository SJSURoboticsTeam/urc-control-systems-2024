#pragma once

#include "drive_configuration.hpp"
namespace sjsu::drive {

    /**
     * @brief This class helps smoothly change driving configurations without moving
     * any of the wheels too fast or accelerating to quickly. The updater will constantly attempt to move 
     * its `current` configuration towards a `target` configuration, based on the difference between the two configurations.
     * 
     * Basically does this formula: `current += clamp((target - current) * kP, -max_change, max_change)`
     * for `steering_angle`, `wheel_heading`, and `wheel_speed`.
     * 
     * @note This is essentially a P controller: https://en.wikipedia.org/wiki/Proportional_control.
     */
    class drive_configuration_updater {
        using seconds = float;
        public: 
            /**
             * @brief Set the target configuration. The Updater will attempt to move towards this configuration.
             * 
             * @param p_target The new target configuration.
             */
            void set_target(drive_configuration p_target);
            /**
             * @brief Update the current configuration. Performs all the necessary math. Get the current configuration by calling `get_current()`.
             * 
             * @param change_in_time Change in time since last call in seconds.
             */
            void update(seconds change_in_time);
            /**
             * @brief Get the current configuration the updater is currently at.
             * 
             * @return The current configuration the update is at.
             */
            drive_configuration get_current();
            /**
             * @brief Get the configuration the updater is trying to move towards.
             * 
             * @return The current target configuration the updater is trying to achieve
             */
            drive_configuration get_target();
            /**
             * @brief Get the rates of change calculated in the last call to update.
             * 
             * @return The rate of change of `wheel_heading`, `steering_angle`, and `wheel_speed` 
             */
            drive_configuration_rate get_rates();
            /**
             * @brief Set the sensitivities of `wheel_heading`, `steering_angle`, and `wheel_speed`. This affects how sensitive changes to updater is to the difference between `target` and `current`
             * 
             * @param p_sensitivity The new sensitivites.
             */
            void set_sensitivity(drive_configuration_sensitivity p_sensitivity);
            /**
             * @brief Set the max rates of change of `wheel_heading`, `steering_angle`, and `wheel_speed`. The limits the maximum change the updater will make to the current configuration.
             * 
             * @param p_max_rate The maximum rates of change
             */
            void set_max_rate(drive_configuration_rate p_max_rate);
        private:
            /**
             * @brief The current configuration of the updater, last computed by `update()`. This should be used as the current configuration of the robot.
             * 
             */
            drive_configuration m_current;
            /**
             * @brief The target configuration of the updater. The updater will attempt to move towards this configuration.
             * 
             */
            drive_configuration m_target;
            /**
             * @brief The sensitivity of the updater to the error between target and current configurations.
             * 
             */
            drive_configuration_sensitivity m_sensitivity;
            /**
             * @brief The rates at which the configuration changes, last computed by `update`.
             * 
             */
            drive_configuration_rate m_delta;
            /**
             * @brief The maximum rate that configuration is allowed to change.
             * 
             */
            drive_configuration_rate m_max_rate;
    };
}