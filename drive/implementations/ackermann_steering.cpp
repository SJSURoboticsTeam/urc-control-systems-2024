#include "../include/ackermann_steering.hpp"
#include <cmath>
#include <algorithm>

using namespace sjsu::drive;

ackermann_steering::ackermann_steering(std::span<vector2> p_wheel_locations, std::span<wheel_setting> p_wheel_settings_span, hal::rpm p_maximum_wheel_speed, hal::rpm p_maximum_angular_velocity) {
    m_wheel_locations = p_wheel_locations;
    m_maximum_wheel_speed = p_maximum_wheel_speed;
    m_maximum_angular_velocity = p_maximum_angular_velocity;
    m_wheel_settings = p_wheel_settings_span;
}

std::span<wheel_setting> ackermann_steering::get_wheel_settings() {
    return m_wheel_settings;
}
// p_signed_radius is purple coming from red (steering angle)
// p_heading is the yellow part ()
std::span<wheel_setting> ackermann_steering::calculate_wheel_settings(float p_signed_turning_radius, hal::degrees p_heading, hal::rpm p_speed) {
    float turning_radius = std::abs(p_signed_turning_radius);
    float imaginary_wheel_distance = sqrt(1 + turning_radius * turning_radius);

    // Bounds checking:
    // float maximum_speed_due_to_angular_speed_constraint = m_maximum_angular_velocity * imaginary_wheel_distance;
    // p_speed = std::clamp(p_speed, -maximum_speed_due_to_angular_speed_constraint, maximum_speed_due_to_angular_speed_constraint);
    p_speed = std::clamp(p_speed, -m_maximum_wheel_speed, m_maximum_wheel_speed);



    vector2 turning_circle_center = vector2::from_polar(p_signed_turning_radius, -p_heading * std::numbers::pi / 180);

    for(size_t i = 0; i < m_wheel_locations.size(); i ++) {
        
        if(std::isinf(p_signed_turning_radius)) {
            // Translate mode;
            m_wheel_settings[i].angle = p_heading;
            m_wheel_settings[i].wheel_speed = p_speed;
        }else {
            vector2 direction = m_wheel_locations[i] - turning_circle_center;
            if(p_signed_turning_radius < 0) {
                m_wheel_settings[i].angle = vector2::bearing_angle(vector2::rotate_90_ccw(direction)) * 180 / std::numbers::pi;
            }else {
                m_wheel_settings[i].angle = vector2::bearing_angle(vector2::rotate_90_cw(direction)) * 180 / std::numbers::pi;
            }
            
            float wheel_distance = vector2::length(direction);
            m_wheel_settings[i].wheel_speed = p_speed * wheel_distance / imaginary_wheel_distance;
        }
    }

    // if(m_maximum_wheel_speed < max_speed_of_wheels) {
    //     float wheel_speed_scale_factor = m_maximum_wheel_speed / max_speed_of_wheels;
    //     for(size_t i = 0; i < m_wheel_settings.size(); i ++) {
    //         m_wheel_settings[i].wheel_speed *= wheel_speed_scale_factor;
    //     }
    // }

    return m_wheel_settings;
}