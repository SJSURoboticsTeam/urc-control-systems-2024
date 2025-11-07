// TODO
#pragma once


#include "../hardware_map.hpp"
#include <cstddef>
#include <array>

namespace sjsu::arm {
class Home
{
  public:
    Home(resources::arm_joints p_arm_joints, resources::limit_pins p_home_pins )
      : m_arm_joints(p_arm_joints), m_home_pins(p_home_pins) {}

    void home_all_joints();

  private:
    resources::arm_joints m_arm_joints;
    resources::limit_pins m_home_pins;
    
  };}
