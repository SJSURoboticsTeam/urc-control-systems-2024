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
#include <libhal/can.hpp>
#include <string_view>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include "ackermann_steering.hpp"
#include <array>
#include <libhal-micromod/micromod.hpp>
#include <libhal-util/units.hpp>

#include "../applications/application.hpp"

#include "vector2.hpp"

namespace sjsu::drive {

hardware_map_t initialize_platform()
{
  using namespace hal::literals;
  using namespace std::chrono_literals;

  auto& counter = hal::micromod::v1::uptime_clock();

  auto& terminal = hal::micromod::v1::console(hal::buffer<1024>);

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

  hal::can* can = nullptr;
  hal::can_transceiver* ct;
  hal::can_bus_manager* bm;
  hal::can_identifier_filter* idf;
  if constexpr (use_can_v1) {
    can = &hal::micromod::v1::can();
  } else {
    static std::array<hal::can_message, 4> receive_buffer{};
    ct = &hal::micromod::v1::can_transceiver(receive_buffer);
    bm = &hal::micromod::v1::can_bus_manager();
    idf = &hal::micromod::v1::can_identifier_filter0();
  }

  return hardware_map_t{
    .clock = &counter,
    .terminal = &terminal,
    .can = can,
    .can_transceiver = ct,
    .can_bus_manager = bm,
    .can_identifier_filter = idf,

    .reset = []() { hal::cortex_m::reset(); }
    //   // .steering = &steering,
    //   .reset = []() { hal::cortex_m::reset(); },
    // };
  };
}
}  // namespace sjsu::drive
