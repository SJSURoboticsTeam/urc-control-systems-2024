#include "./application.hpp"
#include "../implementations/science_state_machine.hpp"

namespace sjsu::science {

void application(hardware_map_t& hardware_map)
{
  using namespace std::literals;

  // auto& pump_controller = *(hardware_map.pump_controller);

  // auto& mixing_servo = *hardware_map.mixing_servo;
  // auto& revolver_controller = *hardware_map.revolver_controller;

  auto& clock = *hardware_map.steady_clock;
  auto& terminal = *hardware_map.terminal;
  // auto& mission_control = *(hardware_map.mc);
  // auto& revolver = *hardware_map.revolver;

  // auto& can = *hardware_map.can;
  // auto& i2c = *hardware_map.i2c;


  // auto& myScienceRobot = *hardware_map.myScienceRobot;
  // mission_control::mc_commands commands;
  science_state_machine state_machine(hardware_map);

  // int state = 0;
  hal::print(terminal, "Hello, World\n");

  while (true){
    state_machine.run_state_machine(science_state_machine::science_states::GET_SAMPLES);
    hal::delay(clock, 500ms);
    state_machine.run_state_machine(science_state_machine::science_states::MOLISCH_TEST);
    hal::delay(clock, 500ms);
    state_machine.run_state_machine(science_state_machine::science_states::BIURET_TEST);
    hal::delay(clock, 500ms);
    state_machine.run_state_machine(science_state_machine::science_states::RESET);
    hal::delay(clock, 500ms);
  }

  // while (true) {
  //   // Print message

  //   auto timeout = hal::create_timeout(clock, 2s);

  //   commands = mission_control.get_command(timeout).value();
  //   commands.print(&terminal);
  //   auto state = science_state_machine::science_states::GET_SAMPLES;
  //   int state_num = static_cast<int>(state);

  //   while (commands.sample_recieved == 1 && state_num < 4) {
  //     if (commands.pause_play == 1) {
  //       while (commands.pause_play == 1) {
  //         hal::print(terminal, "Waiting for Mission Control\n");
  //         commands = mission_control.get_command(timeout).value();
  //       }
  //     } else if (commands.contianment_reset == 1) {
  //       // state_machine.run_state_machine(science_state_machine::science_states::RESET);
  //       hal::print(terminal, "Serious Error Occured Or \n");
  //       break;
  //     } else {
  //       // state_machine.run_state_machine(state);
  //       state_num++;
  //       state = static_cast<science_state_machine::science_states>(state_num);
  //     }
  //     commands = mission_control.get_command(timeout).value();
  //   }
  //   // hal::print<64>(terminal, "%d samples left\n",
  //   // state_machine.get_num_vials_left());
  //   state = science_state_machine::science_states::GET_SAMPLES;
    
  // }

}

}  // namespace sjsu::science
