#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/motor.hpp>
#include <span>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>
#include <libhal-arm-mcu/stm32f1/input_pin.hpp>

#include "ackermann_steering.hpp"
#include <array>
#include <libhal-micromod/micromod.hpp>
#include <libhal-util/units.hpp>

#include "../applications/application.hpp"

#include "settings.hpp"
#include "steering_module.hpp"
#include "vector2.hpp"

namespace sjsu::drive {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;


  static auto& counter = hal::micromod::v1::uptime_clock();

  static auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

  hal::print<1028>(terminal, "Console is intialized\n");


  // static hal::stm32f1::input_pin fl_pin_1('A', 0);  // 60 spi1_sck
  static hal::stm32f1::input_pin fr_pin_2('B', 6);  // 62 spi1_copi
  // static hal::stm32f1::input_pin bl_pin_3('A', 1);  // 64 spi1_cipo
  // static hal::stm32f1::input_pin br_pin_4('B', 0);  // 34 A0
  hal::print<1028>(terminal, "failing input pin\n");


  // static std::array<vector2, 3> wheel_locations = {
  //   vector2::from_bearing(1, -60 * std::numbers::pi / 180),
  //   vector2::from_bearing(1, 60 * std::numbers::pi / 180),
  //   vector2::from_bearing(1, std::numbers::pi),
  // };

  // static std::array<wheel_setting, wheel_locations.size()> wheel_settings;

  // Note: Maximum speed is not used
  // static ackermann_steering steering(wheel_locations, wheel_settings, 2, 2);

  // WARINING: THESE MODULES HAVE NOT BEEN INITIALIZED
  // static std::array<steering_module, 1> modules;

  static hal::can_transceiver* can_transceiver;
  static hal::can_bus_manager* bus_man;
  static hal::can_identifier_filter* idf0;
  static hal::can_identifier_filter* idf1;
  static hal::can_identifier_filter* idf2;
  static hal::can_identifier_filter* idf3;

  // static hal::can_identifier_filter* idf4;
  // static hal::can_identifier_filter* idf5;
  // static hal::can_identifier_filter* idf6;
  // static hal::can_identifier_filter* idf7;

  // static hal::can_mask_filter* controller_mask;

  static std::array<hal::can_message, 8> receive_buffer{};
  can_transceiver = &hal::micromod::v1::can_transceiver(receive_buffer);
  
  bus_man = &hal::micromod::v1::can_bus_manager();
  idf0 = &hal::micromod::v1::can_identifier_filter0();
  idf1 = &hal::micromod::v1::can_identifier_filter1();
  idf2 = &hal::micromod::v1::can_identifier_filter2();
  idf3 = &hal::micromod::v1::can_identifier_filter3();
  // idf4 = &hal::micromod::v1::can_identifier_filter4();
  // idf5 = &hal::micromod::v1::can_identifier_filter5();
  // idf6 = &hal::micromod::v1::can_identifier_filter6();
  // idf7 = &hal::micromod::v1::can_identifier_filter7();

  // controller_mask = &hal::micromod::v1::can_mask_filter0();
  idf0->allow(0x105);

  hal::print<1028>(terminal, "Identifier filter\n");
  hal::can_message_finder homing_reader(*can_transceiver, 0x105);

  while(true){
    std::optional<hal::can_message> msg = homing_reader.find();
    if(msg){
      hal::print(terminal, "Done homing\n");
    }
  }


  // controller_mask->allow(hal::can_mask_filter::pair{.id = 0x100, .mask = 0x7F0});
  hal::print<1028>(terminal, "can initialized\n");
  bus_man->baud_rate(1.0_MHz);

  static std::array<start_wheel_setting, 3> start_wheel_setting_arr = {
    front_left_wheel_setting,
    front_right_wheel_setting,
    back_left_wheel_setting,
  };

  static std::span<start_wheel_setting,3> start_wheel_setting_span =
  start_wheel_setting_arr;

  // static std::array<start_wheel_setting, 1> start_wheel_setting_arr = {
  //   back_left_wheel_setting
  // };

  // static std::span<start_wheel_setting, 1> start_wheel_setting_span =
  //   start_wheel_setting_arr;

  hal::print<1028>(terminal, "Stetting struct intialized\n");

  // ------- STEERING MOUDULE 1 ---------
  // static hal::actuator::rmd_mc_x_v2 mc_x_front_left_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[0].geer_ratio,
  //   start_wheel_setting_arr[0].prop_id);
  // static auto front_left_prop =
  //   mc_x_front_left_prop.acquire_motor(start_wheel_setting_arr[0].max_speed);
  static hal::actuator::rmd_mc_x_v2* mc_x_front_left_steer;
  try{
    static hal::actuator::rmd_mc_x_v2 temp1(
      *can_transceiver,
      *idf1,
      counter,
      start_wheel_setting_arr[0].geer_ratio,
      start_wheel_setting_arr[0].steer_id);
    mc_x_front_left_steer = &temp1;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static steering_module front_left_leg = {
    .steer = mc_x_front_left_steer,
    // .propulsion = &front_left_prop,
    .propulsion = nullptr,
    .limit_switch = &fr_pin_2,
  };

  // ------- STEERING MOUDULE 2 ---------
  // static hal::actuator::rmd_mc_x_v2 mc_x_front_right_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[1].geer_ratio,
  //   start_wheel_setting_arr[1].prop_id);
  // static auto front_right_prop =
  //   mc_x_front_right_prop.acquire_motor(start_wheel_setting_arr[1].max_speed);

  static hal::actuator::rmd_mc_x_v2* mc_x_front_right_steer;
  try{
    static hal::actuator::rmd_mc_x_v2 temp2(
      *can_transceiver,
      *idf2,
      counter,
      start_wheel_setting_arr[1].geer_ratio,
      start_wheel_setting_arr[1].steer_id);
    mc_x_front_right_steer = &temp2;
  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static steering_module front_right_leg = {
    .steer = mc_x_front_right_steer,
    .propulsion = nullptr,
    // .propulsion = &front_right_prop,
    .limit_switch= &fr_pin_2,
  };

  // ------- STEERING MOUDULE 3 ---------
  // static hal::actuator::rmd_mc_x_v2 mc_x_back_left_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[2].geer_ratio,
  //   start_wheel_setting_arr[2].prop_id);

  // static auto back_left_prop =
  //   mc_x_back_left_prop.acquire_motor(start_wheel_setting_arr[2].max_speed);

  static hal::actuator::rmd_mc_x_v2* mc_x_back_left_steer;
  try {
    static hal::actuator::rmd_mc_x_v2 temp3(
      *can_transceiver,
      *idf3,
      counter,
      start_wheel_setting_arr[2].geer_ratio,
      start_wheel_setting_arr[2].steer_id);
      mc_x_back_left_steer = &temp3;
    hal::print<1028>(terminal, "RMD created\n");

  } catch (const hal::exception& e) {
    hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  }

  static steering_module back_left_leg = {
    .steer = mc_x_back_left_steer,
    // .propulsion = &back_left_prop,
    .propulsion = nullptr,
    .limit_switch = &fr_pin_2,
  };

  // ------- STEERING MOUDULE 4 ---------
  // static hal::actuator::rmd_mc_x_v2 mc_x_back_right_prop(
  //   *can_transceiver,
  //   *idf,
  //   counter,
  //   start_wheel_setting_arr[3].geer_ratio,
  //   start_wheel_setting_arr[3].prop_id);
  // static auto back_right_prop =
  //   mc_x_back_left_prop.acquire_motor(start_wheel_setting_arr[3].max_speed);

  // static hal::actuator::rmd_mc_x_v2* mc_x_back_right_steer;
  // try {
  // static hal::actuator::rmd_mc_x_v2 temp(
  //   *can_transceiver,
  //   *idf4,
  //   counter,
  //   start_wheel_setting_arr[3].geer_ratio,
  //   start_wheel_setting_arr[3].steer_id);
  // mc_x_back_right_steer = &temp;
  // }catch (const hal::exception& e) {
  //   hal::print<1028>(terminal, "Exception code %d\n", e.error_code());
  // }

  // static steering_module back_right_leg = {
  //   .steer = mc_x_back_right_steer,
  //   // .propulsion = &back_right_prop,
  //   .propulsion = nullptr,
  //   .limit_switch = &br_pin_4,
  // };

  static std::array<steering_module, 3> steering_modules_arr = {
    front_left_leg, front_right_leg, back_left_leg
  };
  static std::span<steering_module, 3> steering_modules_span =
    steering_modules_arr;

  // static std::array<steering_module, 1> steering_modules_arr = {
  //   back_left_leg
  // };
  // static std::span<steering_module, 1> steering_modules_span =
  //   steering_modules_arr;

  return hardware_map_t{
    .clock = &counter, .terminal = &terminal,
    .can_transceiver = can_transceiver, .can_bus_manager = bus_man,
    // .can_mask_filter = controller_mask,
    .steering_modules = steering_modules_span,
    .start_wheel_setting_span = start_wheel_setting_span,
    .reset = []() { hal::cortex_m::reset(); }
  };
}
}  // namespace sjsu::drive
