// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-soft/rc_servo.hpp>
// #include <libhal-pca/pca9685.hpp>

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-pca/pca9685.hpp>
#include <libhal-lpc40/pwm.hpp>
//#include <libhal-lpc40/system_controller.hpp> //not sure why we need this?
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include <libhal-util/units.hpp>

// #include <implementations/esp8266_mission_control.cpp>
#include <implementations/pump_manager.hpp>
#include <implementations/revolver.hpp>
// #include <implementations/helper.hpp>

#include <libhal-micromod/micromod.hpp>
#include <applications/application.hpp>

namespace sjsu::science {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;


  // static std::array<hal::byte, 8192> recieve_buffer1{};

  // static auto uart1 = HAL_CHECK((hal::lpc40::uart::get(1,
  //                                                      recieve_buffer1,
  //                                                      hal::serial::settings{
  //                                                        .baud_rate = 115200,
  //                                                      })));

  // static constexpr std::string_view ssid =
  //   "TP-Link_FC30";  // change to wifi name that you are using
  // static constexpr std::string_view password =
  //   "R0Bot1cs3250";  // change to wifi password you are using

  // // still need to decide what we want the static IP to be
  // static constexpr std::string_view ip = "192.168.0.216";
  // static constexpr auto socket_config = hal::esp8266::at::socket_config{
  //   .type = hal::esp8266::at::socket_type::tcp,
  //   .domain = "192.168.0.211",
  //   .port = 5000,
  // };

  // static constexpr char* get_request = "GET / %s HTTP/1.1\r\n"
  //                                                 "Host: 192.168.0.211:5000\r\n"
  //                                                 "Keep-Alive: timeout=1000\r\n"
  //                                                 "Connection: keep-alive\r\n"
  //                                                 "\r\n";
  
  // static constexpr std::string_view get_request_meta_data = "";


  // char get_request[get_prefix.size() + get_request_meta_data.size() + 200];
  // static std::array<hal::byte, 2048> buffer{};
  // static auto helper = serial_mirror(uart1, uart0);

  // auto timeout = hal::create_timeout(counter, 10s);
  // //break right here
  // static auto esp8266 = HAL_CHECK(hal::esp8266::at::create(uart1, timeout));
  // HAL_CHECK(hal::write(uart0, "created AT\n"));

  // auto mc_timeout = hal::create_timeout(counter, 10s);
  // static auto esp_mission_control =
  //   sjsu::science::esp8266_mission_control::create(esp8266,
  //                                              uart0,
  //                                              ssid,
  //                                              password,
  //                                              socket_config,
  //                                              ip,
  //                                              mc_timeout,
  //                                              buffer);
  // while (esp_mission_control.has_error()) {
  //   mc_timeout = hal::create_timeout(counter, 10s);
  //   esp_mission_control =
  //     sjsu::science::esp8266_mission_control::create(esp8266,
  //                                                uart0,
  //                                                ssid,
  //                                                password,
  //                                                socket_config,
  //                                                ip,
  //                                                mc_timeout,
  //                                                buffer);
  // }
  // static auto science_mission_control = esp_mission_control.value();
  // mission_control* test_mc = nullptr; // TODO(Kirthika): This will crash the code, pls change 
 
  auto& counter = hal::micromod::v1::uptime_clock();

  auto& output_deionized_water_pump_pin = hal::micromod::v1::output_g0();
  auto& output_sample_pump_pin = hal::micromod::v1::output_g1();
  auto& output_molisch_pump_pin = hal::micromod::v1::output_g2();
  auto& output_sulfuric_acid_pin = hal::micromod::v1::output_g3();
  auto& output_biuret_pump_pin = hal::micromod::v1::output_g4();

  static pump_manager pump_controller(
    counter, 
    output_deionized_water_pump_pin,
    output_sample_pump_pin, 
    output_molisch_pump_pin,
    output_sulfuric_acid_pin,
    output_biuret_pump_pin);
    
//Mixing servo
  auto& i2c = hal::micromod::v1::i2c();
  static constexpr hal::byte address = 0b100'0000;
  hal::pca::pca9685 pca9685(i2c, address);

  auto pwm0 = pca9685.get_pwm_channel<0>(); 
  auto servo_settings = hal::soft::rc_servo::settings{
    .min_angle = 0.0_deg,
    .max_angle = 360.0_deg,
    .min_microseconds = 500,
    .max_microseconds = 2500,
  };

  static hal::soft::rc_servo mixing_servo(pwm0, servo_settings);

//Creating Revolver
  static auto pwm1 = pca9685.get_pwm_channel<1>(); 
  auto revolver_servo_settings = hal::soft::rc_servo::settings{
      .min_angle = hal::degrees(0.0),
      .max_angle = hal::degrees(180.0),
      .min_microseconds = 1000,
      .max_microseconds = 2000,
    };
  static hal::soft::rc_servo revolving_servo(pwm1, revolver_servo_settings);

  auto& revolver_pin =  hal::micromod::v1::input_g0();

  auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

  static revolver revolver_controller(revolving_servo, revolver_pin, counter, terminal);

  
  return hardware_map_t{
    
    .pump_controller = &pump_controller,
    .mixing_servo = &mixing_servo,
    .revolver_controller = &revolver_controller,

    .steady_clock = &counter,
    .terminal = &terminal,
    // .mc = test_mc,
    .reset = []() { hal::cortex_m::reset(); },
  }; 
};
}  // namespace sjsu::science
