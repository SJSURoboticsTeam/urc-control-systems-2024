#include "../include/science_state_machine.hpp"
using namespace std::chrono_literals;
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
  switch(state){
    case science_state_machine::science_states::HOME_CAROUSEL:
      break;
    case science_state_machine::science_states::CUP_OUTSIDE:
      m_door.position(180);
      hal::delay(*m_clock, 1000ms);
      m_arm_servo.position(180);
      hal::delay(*m_clock, 1000ms);
      break;
    case science_state_machine::science_states::CUP_INSIDE:
      m_arm_servo.position(0);
      hal::delay(*m_clock, 1000ms);
      m_door.position(0);
      hal::delay(*m_clock, 1000ms);
      break;
    case science_state_machine::science_states::DUMP_SAMPLE: 
      // trapdoor servo deg: 0 close, 120 open)
      m_trap_door.position(0);
      hal::delay(*m_clock, 1000ms);
      m_trap_door.position(120); // keep open for 3s
      hal::delay(*m_clock, 3000ms);
      m_trap_door.position(0);
      hal::delay(*m_clock, 1000ms);
      break;
    case science_state_machine::science_states::ADD_DI_WATER:
      break;
    case science_state_machine::science_states::MIX_DI_WATER:
      break;
    case science_state_machine::science_states::ADD_MIX_BENEDICT_REAGENT:
      break;
    case science_state_machine::science_states::ADD_BIURET_REAGENT:
      break;
    case science_state_machine::science_states::ADD_KALLING_REAGENT:
      break;
    case science_state_machine::science_states::READ_COLOR_SENSOR:
      break;
  }
}

}  // namespace sjsu::science
