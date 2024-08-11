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

#include <libhal-lpc40/adc.hpp>
#include <libhal-lpc40/can.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/input_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-lpc40/pwm.hpp>
// #include <libhal-lpc40/system_controller.hpp> //not sure why we need this?
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/uart.hpp>

#include "../../platform-implementations/esp8266_mission_control.cpp"
#include <libhal-util/units.hpp>

#include "../hardware_map.hpp"
namespace sjsu::science {
hal::status initialize_processor()
{
  hal::cortex_m::initialize_data_section();
  hal::cortex_m::initialize_floating_point_unit();

  return hal::success();
}

hal::result<application_framework> initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  // Set the MCU to the maximum clock speed
  HAL_CHECK(hal::lpc40::clock::maximum(12.0_MHz));

  auto& clock = hal::lpc40::clock::get();
  auto cpu_frequency = clock.get_frequency(hal::lpc40::peripheral::cpu);
  static hal::cortex_m::dwt_counter counter(cpu_frequency);

  // Serial
  static std::array<hal::byte, 1024> recieve_buffer0{};
  static auto uart0 = HAL_CHECK((hal::lpc40::uart::get(0,
                                                       recieve_buffer0,
                                                       hal::serial::settings{
                                                         .baud_rate = 38400,
                                                       })));
  
  //ESP
  static std::array<hal::byte, 8192> recieve_buffer1{};
  static auto uart1 = HAL_CHECK((hal::lpc40::uart::get(1,
                                                       recieve_buffer1,
                                                       hal::serial::settings{
                                                         .baud_rate = 115200,
                                                       })));

  static constexpr std::string_view ssid =
    "TP-Link_FC30";  // change to wifi name that you are using
  static constexpr std::string_view password =
    "R0Bot1cs3250";  // change to wifi password you are using

  // still need to decide what we want the static IP to be
  static constexpr std::string_view ip = "192.168.0.216";
  static constexpr auto socket_config = hal::esp8266::at::socket_config{
    .type = hal::esp8266::at::socket_type::tcp,
    .domain = "192.168.0.211",
    .port = 5000,
  };
  HAL_CHECK(hal::write(uart0, "created Socket\n"));
  static constexpr std::string_view get_request = "GET /science HTTP/1.1\r\n"
                                                  "Host: 192.168.0.211:5000\r\n"
                                                  "Keep-Alive: timeout=1000\r\n"
                                                  "Connection: keep-alive\r\n"
                                                  "\r\n";

  static std::array<hal::byte, 2048> buffer{};
  // static auto helper = serial_mirror(uart1, uart0);

  auto timeout = hal::create_timeout(counter, 10s);
  //break right here
  static auto esp8266 = HAL_CHECK(hal::esp8266::at::create(uart1, timeout));
  HAL_CHECK(hal::write(uart0, "created AT\n"));

  auto mc_timeout = hal::create_timeout(counter, 10s);
  static auto esp_mission_control =
    sjsu::science::esp8266_mission_control::create(esp8266,
                                               uart0,
                                               ssid,
                                               password,
                                               socket_config,
                                               ip,
                                               mc_timeout,
                                               buffer,
                                               get_request);
  while (esp_mission_control.has_error()) {
    mc_timeout = hal::create_timeout(counter, 10s);
    esp_mission_control =
      sjsu::science::esp8266_mission_control::create(esp8266,
                                                 uart0,
                                                 ssid,
                                                 password,
                                                 socket_config,
                                                 ip,
                                                 mc_timeout,
                                                 buffer,
                                                 get_request);
  }
  static auto science_mission_control = esp_mission_control.value();

  // Don't think we need can for the science applications thus far
  static hal::can::settings can_settings{ .baud_rate = 1.0_MHz };
  static auto can = HAL_CHECK((hal::lpc40::can::get(2, can_settings)));

  static auto in_pin0 =
    HAL_CHECK(hal::lpc40::input_pin::get(1, 15, hal::input_pin::settings{}));
  static auto in_pin1 =
    HAL_CHECK(hal::lpc40::input_pin::get(1, 23, hal::input_pin::settings{}));
  static auto in_pin2 =
    HAL_CHECK(hal::lpc40::input_pin::get(1, 22, hal::input_pin::settings{}));

  static auto pwm_1_6 = HAL_CHECK((hal::lpc40::pwm::get(1, 6)));
  static auto pwm_1_5 = HAL_CHECK((hal::lpc40::pwm::get(1, 5)));

  static auto adc_4 = HAL_CHECK(hal::lpc40::adc::get(4));
  static auto adc_5 = HAL_CHECK(hal::lpc40::adc::get(5));

  static auto i2c = HAL_CHECK(hal::lpc40::i2c::get(2));

  auto deionized_water_pump = HAL_CHECK(hal::lpc40::output_pin::get(1, 19, hal::output_pin::settings{}));
  auto sample_pump = HAL_CHECK(hal::lpc40::output_pin::get(1, 28, hal::output_pin::settings{}));
  auto molisch_reagent_pump = HAL_CHECK(hal::lpc40::output_pin::get(1, 20, hal::output_pin::settings{}));
  auto sulfuric_acid_pump = HAL_CHECK(hal::lpc40::output_pin::get(1, 19, hal::output_pin::settings{}));
  auto biuret_reagent = HAL_CHECK(hal::lpc40::output_pin::get(1, 9, hal::output_pin::settings{}));

  return application_framework{
    .terminal = &uart0,
    .can = &can,
    .in_pin0 = &in_pin0,
    .in_pin1 = &in_pin1,
    .in_pin2 = &in_pin2,
    .pwm_1_6 = &pwm_1_6,
    .pwm_1_5 = &pwm_1_5,
    .adc_4 = &adc_4,
    .adc_5 = &adc_5,
    // .esp = &uart1,
    .i2c = &i2c,
    .steady_clock = &counter,
    .deionized_water_pump = &deionized_water_pump,
    .sample_pump = &sample_pump,
    .molisch_reagent_pump = &molisch_reagent_pump,
    .sulfuric_acid_pump = &sulfuric_acid_pump,
    .biuret_reagent = &biuret_reagent,
    .mc = &science_mission_control,
    .reset = []() { hal::cortex_m::reset(); },

  };
};
}  // namespace sjsu::science
