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
#include "hardware_map.hpp"
#include <libhal/error.hpp>

#include <libhal-util/serial.hpp>
int main()
{
  auto hardware_map = sjsu::drivers::initialize_platform();
  try {

    hal::print<1024>(*hardware_map.terminal, "huh?");
    application(hardware_map);
  } catch (...) {
    hal::print<1024>(*hardware_map.terminal,
                     "A resource was not found");
  }

  return 0;
}

// namespace boost {
// void throw_exception(std::exception const&)
// {
//   hal::halt();
// }
// }  // namespace boost