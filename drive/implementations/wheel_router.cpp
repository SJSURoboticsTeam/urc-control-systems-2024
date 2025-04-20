#include "wheel_router.hpp"
#include "settings.hpp"

#include <libhal/error.hpp>
#include <optional>


namespace sjsu::drive {
  wheel_router::wheel_router(std::span<steering_module> p_steering_modules) : m_steering_modules(p_steering_modules) {}

  void wheel_router::move(std::span<wheel_setting> p_settings) {
    if(m_steering_modules.size() != p_settings.size()) {
        // Wheel Settings and leg mismatch.
        throw hal::argument_out_of_domain(this);
    }

    for(size_t i = 0; i < m_steering_modules.size(); i ++) {
        //changed but probably should fix later
        m_steering_modules[i].steer->acquire_servo(5).position(p_settings[i].angle * angle_correction_factor);
        m_steering_modules[i].propulsion.value()->power(p_settings[i].wheel_speed);
    }
    
  }

//   hal::result<motor_feedback> wheel_router::get_motor_feedback() {
//     return motor_feedback{};
//   }

}  // namespace sjsu::drive
