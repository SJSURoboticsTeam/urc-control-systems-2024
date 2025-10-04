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
#pragma once

#include <libhal-arm-mcu/system_control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/adc.hpp>
#include <libhal/can.hpp>
#include <libhal/dac.hpp>
#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/interrupt_pin.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/spi.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/stream_dac.hpp>
#include <libhal/timer.hpp>
#include <libhal/usb.hpp>
#include <libhal/zero_copy_serial.hpp>

namespace custom {
/**
 * @brief A stand in interface until libhal supports an official watchdog
 * interface.
 *
 */
class watchdog
{
public:
  watchdog() = default;
  virtual void start() = 0;
  virtual void reset() = 0;
  virtual void set_countdown_time(hal::time_duration p_wait_time) = 0;
  virtual bool check_flag() = 0;
  virtual void clear_flag() = 0;
  virtual ~watchdog() = default;
};
}  // namespace custom

namespace resources {
// =======================================================
// Defined by each platform file
// =======================================================
/**
 * @brief Allocator for driver memory
 *
 * The expectation is that the implementation of this allocator is a
 * std::pmr::monotonic_buffer_resource with static memory storage, meaning the
 * memory is fixed in size and memory cannot be deallocated. This is fine for
 * the demos.
 *
 * @return std::pmr::polymorphic_allocator<>
 */
std::pmr::polymorphic_allocator<> driver_allocator();
/**
 * @brief Steady clock that provides the current uptime
 *
 * @return hal::v5::strong_ptr<hal::steady_clock>
 */
hal::v5::strong_ptr<hal::steady_clock> clock();
hal::v5::strong_ptr<hal::serial> console();
hal::v5::strong_ptr<hal::zero_copy_serial> zero_copy_serial();
hal::v5::strong_ptr<hal::output_pin> status_led();
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager();
hal::v5::strong_ptr<hal::can_interrupt> can_interrupt();
hal::v5::strong_ptr<hal::adc> adc();
hal::v5::strong_ptr<hal::input_pin> input_pin();
hal::v5::strong_ptr<hal::i2c> i2c();
hal::v5::strong_ptr<hal::interrupt_pin> interrupt_pin();
hal::v5::strong_ptr<hal::pwm> pwm();
hal::v5::strong_ptr<hal::timer> timed_interrupt();
hal::v5::strong_ptr<hal::pwm16_channel> pwm_channel();
hal::v5::strong_ptr<hal::pwm_group_manager> pwm_frequency();
hal::v5::strong_ptr<hal::spi> spi();
hal::v5::strong_ptr<hal::output_pin> spi_chip_select();
hal::v5::strong_ptr<hal::stream_dac_u8> stream_dac();
hal::v5::strong_ptr<hal::dac> dac();
hal::v5::strong_ptr<custom::watchdog> watchdog();

inline void reset()
{
  hal::cortex_m::reset();
}
inline void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}
}  // namespace resources

// Application function is implemented by one of the .cpp files.
void initialize_platform();
void application();