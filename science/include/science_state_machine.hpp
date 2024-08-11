#pragma once
#include "../applications/application.hpp"
// #include "mission_control.hpp"
#include <libhal/servo.hpp>
#include <libhal/units.hpp>



namespace sjsu::science {
class science_state_machine
{
private:
  hardware_map_t& hardware;
  int m_count;
  int vials_used;
  // mission_control::status sm_m_status;

public:
  // struct status
  // {
  //   int heartbeat_count = 0;
  //   int is_operational = 1;
  //   int sample_recieved = 0;
  //   int pause_play = 0;
  //   int contianment_reset = 0;
  //   int num_vials_used = 0;
  //   int is_sample_finished = 0;
  // };
  // status m_status;

  //Pass in status and update with can
  science_state_machine(sjsu::science::hardware_map_t& application);

  enum class science_states
  {
    GET_SAMPLES,
    MOLISCH_TEST,
    BIURET_TEST,
    RESET
  };
  // auto sm = science_state_machine()
  // science_state_machine::create()

  // vial2_position current_position= SAMPLE;
  // science_states current_state= GET_SAMPLES;

  void run_state_machine(science_states current_state);

  void mix_solution();
  void turn_on_pump(auto pump, hal::time_duration duration);
  void move_sample(int position);
  void containment_reset();

  int get_num_vials_left();

  // mission_control::status get_status();
};

}  // namespace sjsu::science