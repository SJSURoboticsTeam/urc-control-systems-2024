#include "../include/science_state_machine.hpp"

namespace sjsu::science {
science_state_machine::science_state_machine(
  hal::actuator::rc_servo p_arm_servo,
  hal::actuator::rc_servo p_trap_door,
  hal::actuator::rc_servo p_mixer,
  hal::actuator::rc_servo p_door,
  hal::v5::strong_ptr<carousel> p_carousel,
  hal::v5::strong_ptr<pump_manager> p_pump_manager,
  hal::v5::strong_ptr<hal::steady_clock> p_clock,
  hal::v5::strong_ptr<hal::serial> p_terminal)
  : m_arm_servo(p_arm_servo)
  , m_trap_door(p_trap_door)
  , m_mixer(p_mixer)
  , m_door(p_door)
  , m_carousel(p_carousel)
  , m_pump_manager(p_pump_manager)
  , m_clock(p_clock)
  , m_terminal(p_terminal)
{
}

int science_state_machine::get_num_vials()
{
    return 0;
}

void science_state_machine::turn_on_pump([[maybe_unused]] auto pump, [[maybe_unused]] hal::time_duration duration)
{
}

void science_state_machine::run_state_machine([[maybe_unused]] science_states state)
{
}

}  // namespace sjsu::science
