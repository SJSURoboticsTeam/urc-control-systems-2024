// TODO
#include "../include/home.hpp"

namespace sjsu::arm {
void Home::home_all_joints()
{
  for (size_t i = 0; i < m_home_pins.size(); i++) {
    if ((m_home_pins)[i] != nullptr) { 
      while (!(m_home_pins)[i]->level())
      {
        (m_arm_joints)[i]->set_velocity(3,3); // 3 degrees per second
      }
      (m_arm_joints)[i]->set_velocity(0, 3);
      }
    }
}
}  // namespace sjsu::arm