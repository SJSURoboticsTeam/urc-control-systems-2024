#include "./application.hpp"
#include "../platform-implementations/science_state_machine.hpp"

namespace sjsu::science {

void application(hardware_map_t& hardware_map)
{
  using namespace std::literals;


  // auto& in_deionized_water_pump_pin =
  // *hardware_map.in_deionized_water_pump_pin; auto& in_sample_pump_pin =
  // *hardware_map.in_sample_pump_pin; auto& in_molisch_pump_pin =
  // *hardware_map.in_molisch_pump_pin; auto& in_sulfuric_acid_pin =
  // *hardware_map.in_sulfuric_acid_pin; auto& in_biuret_pump_pin =
  // *hardware_map.in_biuret_pump_pin;
  auto& pump_controller = *(hardware_map.pump_controller);

  // auto& pwm_1_6 = *hardware_map.pwm_1_6;
  // auto& pwm_1_5 = *hardware_map.pwm_1_5;
  // auto& adc_4 = *hardware_map.adc_4;
  // auto& adc_5 = *hardware_map.adc_5;
  auto& mixing_servo = *hardware_map.mixing_servo;
  auto& revolver_controller = *hardware_map.revolver_controller;

  auto& clock = *hardware_map.steady_clock;
  auto& terminal = *hardware_map.terminal;
  auto& mission_control = *(hardware_map.mc);
  auto& steady_clock = *hardware_map.steady_clock;
  // auto& revolver = *hardware_map.revolver;

  // auto& can = *hardware_map.can;
  // auto& i2c = *hardware_map.i2c;

  // auto loop_count = 0;

  // auto& myScienceRobot = *hardware_map.myScienceRobot;
  mission_control::mc_commands commands;
  // auto state_machine = HAL_CHECK(science_state_machine::create( hardware_map,
  // mission_control::m_status));
  int state = 0;
  hal::print(terminal, "Hello, World\n");

  while (true) {
    // Print message

    auto timeout = hal::create_timeout(clock, 2s);

    commands = mission_control.get_command(timeout).value();
    commands.print(&terminal);
    auto state = science_state_machine::science_states::GET_SAMPLES;
    int state_num = static_cast<int>(state);

    while (commands.sample_recieved == 1 && state_num < 4) {
      if (commands.pause_play == 1) {
        while (commands.pause_play == 1) {
          hal::print(terminal, "Waiting for Mission Control\n");
          commands = mission_control.get_command(timeout).value();
        }
      } else if (commands.contianment_reset == 1) {
        // state_machine.run_state_machine(science_state_machine::science_states::RESET);
        hal::print(terminal, "Serious Error Occured Or \n");
        break;
      } else {
        // state_machine.run_state_machine(state);
        state_num++;
        state = static_cast<science_state_machine::science_states>(state_num);
      }
      commands = mission_control.get_command(timeout).value();
    }
    // hal::print<64>(terminal, "%d samples left\n",
    // state_machine.get_num_vials_left());
    state = science_state_machine::science_states::GET_SAMPLES;
    
  }
}

}  // namespace sjsu::science
