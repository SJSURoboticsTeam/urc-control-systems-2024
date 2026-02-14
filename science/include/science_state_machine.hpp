#pragma once 
#include <libhal-actuator/rc_servo.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/input_pin.hpp> 

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
    ADD_DI_WATER,
    MIX_DI_WATER,
    ADD_MIX_BENEDICT_REAGENT,
    ADD_BIURET_REAGENT,
    ADD_KALLING_REAGENT,
    READ_COLOR_SENSOR
  };

  hal::v5::strong_ptr<hal::actuator::rc_servo> m_arm_servo;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_trap_door;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_mixer;
  hal::v5::strong_ptr<hal::actuator::rc_servo> m_door;
    hal::v5::strong_ptr<hal::actuator::rc_servo> m_cache;
  hal::v5::strong_ptr<hal::input_pin> m_top_door_limit_switch;
  hal::v5::strong_ptr<hal::input_pin> m_bottom_door_limit_switch;
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
                        hal::v5::strong_ptr<hal::actuator::rc_servo> p_cache,
                        hal::v5::strong_ptr<hal::input_pin> p_top_door_limit_switch,
                        hal::v5::strong_ptr<hal::input_pin> p_bottom_door_limit_switch,
                        hal::v5::strong_ptr<carousel> p_carousel,
                        hal::v5::strong_ptr<pump_manager> p_pump_manager,
                        hal::v5::strong_ptr<hal::steady_clock> p_clock,
                        hal::v5::strong_ptr<hal::serial> p_terminal);
    int get_num_vials();
    void turn_on_pump(auto pump, hal::time_duration duration);
    void run_state_machine(science_states state);
    void create_can_messages();
};
}  // namespace sjsu::science
