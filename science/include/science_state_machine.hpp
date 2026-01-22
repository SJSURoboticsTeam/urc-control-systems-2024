#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>

#include "carousel.hpp"
#include "pump_manager.hpp"

namespace sjsu::science {
class science_state_machine
{
private:
  enum science_states
  {
    HOME_CAROUSEL,
    CUP_OUTSIDE,
    CUP_INSIDE,
    DUMP_SAMPLE,
    ADD_MIX_DI_WATER,
    ADD_MIX_BENEDICT_REAGENT,
    ADD_MIX_BIURET_REAGENT,
    ADD_MIX_KALLING_REAGENT,
    READ_COLOR_SENSOR
  };

  hal::v5::strong_ptr<hal::actuator::rc_servo> m_arm_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_trap_door;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_mixer;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_door;
  hal::v5::strong_ptr<carousel> m_carousel;
  hal::v5::strong_ptr<pump_manager> m_pump_manager;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::serial> m_terminal;
  //TODO: add color sensor

  int m_vials_finished = 0;

public:
  science_state_machine(hal::v5::strong_ptr<hal::actuator::rc_servo> p_arm_servo,
                        hal::v5::strong_ptr<hal::actuator::rc_servo> p_trap_door,
                        hal::v5::strong_ptr<hal::actuator::rc_servo> p_mixer,
                        hal::v5::strong_ptr<hal::actuator::rc_servo> p_door,
                        hal::v5::strong_ptr<carousel> p_carousel,
                        hal::v5::strong_ptr<pump_manager> p_pump_manager,
                        hal::v5::strong_ptr<hal::steady_clock> p_clock,
                        hal::v5::strong_ptr<hal::serial> p_terminal);
    int get_num_vials();
    void mix_solution(int rotations);
    void run_state_machine(science_states state);
};
}  // namespace sjsu::science
