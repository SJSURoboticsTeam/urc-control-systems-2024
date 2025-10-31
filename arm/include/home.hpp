// TODO
#pragma once


#include "../hardware_map.hpp"
#include <cstddef>
#include <array>

class Home
{
private:
  sjsu::arm::resources::arm_joints m_arm_joints;
  std::array<hal::v5::strong_ptr<hal::input_pin>, 6> m_home_pins;

  public:
  Home(sjsu::arm::resources::arm_joints p_arm_joints, const std::array<hal::v5::strong_ptr<hal::input_pin>, 6>& p_home_pins )
    : m_arm_joints(p_arm_joints), m_home_pins(p_home_pins) {}


  void home_all_joints()
  {
    for (size_t i = 0; i < m_home_pins.size(); i++)
    {
      while (!m_home_pins[i]->level())
      {
        m_arm_joints[i].set_velocity(30);
      }
      m_arm_joints[i].set_velocity(0);
    }
  }
};