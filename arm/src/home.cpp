// TODO
#include "../include/home.hpp"

void Home::home_all_joints()
{
for (size_t i = 0; i < m_home_pins->size(); i++)
    {
      while (!(*m_home_pins)[i]->level())
      {
        (*m_arm_joints)[i]->set_velocity(30);
      }
      (*m_arm_joints)[i]->set_velocity(0);
    }
}