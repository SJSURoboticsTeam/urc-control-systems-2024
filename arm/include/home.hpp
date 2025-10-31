// TODO
#pragma once


#include "../hardware_map.hpp"
#include <cstddef>
#include <array>

class Home
{
public:
  Home(hal::strong_ptr<sjsu::arm::resources::arm_joints> p_arm_joints, hal::strong_ptr<std::array<hal::v5::strong_ptr<hal::input_pin>, 6>> p_home_pins )
    : m_arm_joints(p_arm_joints), m_home_pins(p_home_pins) {}


  void home_all_joints();

private:
  hal::strong_ptr<sjsu::arm::resources::arm_joints> m_arm_joints;
  hal::strong_ptr<std::array<hal::v5::strong_ptr<hal::input_pin>, 6>> m_home_pins;

};