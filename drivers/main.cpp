// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include "hardware_map.hpp"

int main()
{
  sjsu::drivers::initialize_platform();
  // auto console = sjsu::drivers::resources::console();
  // try {
  //   sjsu::drivers::application();
  // } catch (...) {
  //   hal::print(*console, "ERROR! \n");
  // }
  sjsu::drivers::application();
  std::terminate();
}
// libhal-arm-mcu specific APIs defined to reduce code size
extern "C"
{
  // This gets rid of an issue with libhal-exceptions in Debug mode.
  void __assert_func()  // NOLINT
  {
  }
}

// Override global new operator
void* operator new(std::size_t)
{
  throw std::bad_alloc();
}

// Override global new[] operator
void* operator new[](std::size_t)
{
  throw std::bad_alloc();
}

void* operator new(unsigned int, std::align_val_t)
{
  throw std::bad_alloc();
}

// Override global delete operator
void operator delete(void*) noexcept
{
}

// Override global delete[] operator
void operator delete[](void*) noexcept
{
}

// Optional: Override sized delete operators (C++14 and later)
void operator delete(void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::size_t) noexcept
{
}

void operator delete[](void*, std::align_val_t) noexcept
{
}

void operator delete(void*, std::align_val_t) noexcept
{
}