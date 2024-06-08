#include "../applications/application.hpp"
#include "../platform-implementations/science_state_machine.hpp"

namespace sjsu::science {

hal::status application(application_framework& p_framework)
{
  using namespace std::literals;
  
  // auto& in_deionized_water_pump_pin = *p_framework.in_deionized_water_pump_pin;
  // auto& in_sample_pump_pin = *p_framework.in_sample_pump_pin;
  // auto& in_molisch_pump_pin = *p_framework.in_molisch_pump_pin;
  // auto& in_sulfuric_acid_pin = *p_framework.in_sulfuric_acid_pin;
  // auto& in_biuret_pump_pin = *p_framework.in_biuret_pump_pin;
  auto& pump_controller = *(p_framework.pump_controller);
  

  // auto& pwm_1_6 = *p_framework.pwm_1_6;
  // auto& pwm_1_5 = *p_framework.pwm_1_5;
  // auto& adc_4 = *p_framework.adc_4;
  // auto& adc_5 = *p_framework.adc_5;
  auto& mixing_servo = *p_framework.mixing_servo;
  auto& revolver_controller = *p_framework.revolver_controller;

  auto& clock = *p_framework.steady_clock;
  auto& terminal = *p_framework.terminal;
  auto& mission_control = *(p_framework.mc);
  auto& steady_clock = *p_framework.steady_clock;
  // auto& revolver = *p_framework.revolver;

  // auto& can = *p_framework.can;
  // auto& i2c = *p_framework.i2c;

  // auto loop_count = 0;
 
  // auto& myScienceRobot = *p_framework.myScienceRobot;
  mission_control::mc_commands commands;
  // auto state_machine = HAL_CHECK(science_state_machine::create( p_framework, mission_control::m_status));
  int state = 0;
  hal::print(terminal, "Hello, World\n");


  
  while (true) {
    // Print message
    
    auto timeout = hal::create_timeout(clock, 2s);

    commands = mission_control.get_command(timeout).value();
    commands.print(&terminal); 
   auto state = science_state_machine::science_states::GET_SAMPLES;
   int state_num = static_cast<int>(state);

    while(commands.sample_recieved == 1 && state_num <4 ){
      if(commands.pause_play == 1 ){
        while(commands.pause_play == 1 ){
          hal::print(terminal, "Waiting for Mission Control\n");
          commands = mission_control.get_command( timeout).value();
        }
      }else if(commands.contianment_reset == 1){
        // state_machine.run_state_machine(science_state_machine::science_states::RESET);
        hal::print(terminal, "Serious Error Occured Or \n");
        break;
      }else{
          // state_machine.run_state_machine(state);
          state_num++;
          state = static_cast<science_state_machine::science_states>(state_num);
      }
      commands = mission_control.get_command( timeout).value();
    }
    // hal::print<64>(terminal, "%d samples left\n", state_machine.get_num_vials_left());
    state =science_state_machine::science_states::GET_SAMPLES;;
  }
}
}  // namespace sjsu::science
