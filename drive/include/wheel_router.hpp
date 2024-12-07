#pragma once

#include "steering_module.hpp"
#include <span>

namespace sjsu::drive {
/**
 * @brief A class that can represent any configuration of wheels.
 * 
 */
class wheel_router
{
public:
  /**
   * @brief Create a wheel router
   * 
   * @param p_steering_modules Span containing the information of each steerable wheel.
   */
  wheel_router(std::span<steering_module> p_steering_modules);
  
  /**
   * @brief Move each wheel to a given setting.
   * @warning If `p_settings.size() != m_legs.size()` the function immediately returns without returning an error. This should be fixed
   * 
   * @param p_settings A span of wheel speeds and wheel angles, each corresponding to a single leg
   */
  void move(std::span<wheel_setting> p_settings);

//   hal::result<motor_feedback> get_motor_feedback();

private:
  // member variables

  std::span<steering_module> m_steering_modules;
};
}  // namespace sjsu::drive
