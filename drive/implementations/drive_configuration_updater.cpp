#include "../include/drive_configuration_updater.hpp"
#include <algorithm>

namespace sjsu::drive {
    void drive_configuration_updater::set_target(drive_configuration p_target) {
        m_target = p_target;
    }
    void drive_configuration_updater::update(seconds dt) {
        m_delta.steering_angle_rate = (m_target.steering_angle - m_current.steering_angle) * m_sensitivity.steering_angle;
        m_delta.wheel_heading_rate = (m_target.wheel_heading - m_current.wheel_heading) * m_sensitivity.wheel_heading;
        m_delta.wheel_speed_rate = (m_target.wheel_speed - m_current.wheel_speed) * m_sensitivity.wheel_speed;

        m_delta.steering_angle_rate = std::clamp(m_delta.steering_angle_rate, -m_max_rate.steering_angle_rate, m_max_rate.steering_angle_rate);
        m_delta.wheel_heading_rate = std::clamp(m_delta.wheel_heading_rate, -m_max_rate.wheel_heading_rate, m_max_rate.wheel_heading_rate);
        m_delta.wheel_speed_rate = std::clamp(m_delta.wheel_speed_rate, -m_max_rate.wheel_speed_rate, m_max_rate.wheel_speed_rate);

        m_current.steering_angle += m_delta.steering_angle_rate * dt;
        m_current.wheel_heading += m_delta.wheel_heading_rate * dt;
        m_current.wheel_speed += m_delta.wheel_speed_rate * dt;
    }
    drive_configuration drive_configuration_updater::get_current() {
        return m_current;
    }
    drive_configuration drive_configuration_updater::get_target() {
        return m_target;
    }
    drive_configuration_rate drive_configuration_updater::get_rates() {
        return m_delta;
    }
    void drive_configuration_updater::set_sensitivity(drive_configuration_sensitivity p_sensitivity) {
        m_sensitivity = p_sensitivity;
    }
    void drive_configuration_updater::set_max_rate(drive_configuration_rate p_max_rate) {
        m_max_rate = p_max_rate;
    }
}