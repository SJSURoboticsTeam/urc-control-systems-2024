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

#pragma once

#include <string_view>

#include <libhal-util/serial.hpp>

/**
 * @brief Debug serial data on one serial port via another
 *
 * This class takes two serial ports, a "primary" and a "mirror". When the write
 * and read APIs are used, data is passed directly through to the primary. On
 * write operations, mirror will also write the same data. On read, the mirror
 * will report the read data by also writing it to its port. This way, a user
 * using a serial to USB debugger or some other debugging device can inspect
 * what is being written to and read from the primary serial port.
 *
 */
class serial_mirror : public hal::serial
{
public:
  serial_mirror(hal::serial& p_primary, hal::serial& p_mirror)
    : m_primary(&p_primary)
    , m_mirror(&p_mirror)
  {
  }

  hal::status driver_configure(const settings& p_settings) override
  {
    return m_primary->configure(p_settings);
  }

  hal::result<write_t> driver_write(std::span<const hal::byte> p_data) override
  {
    HAL_CHECK(hal::write(*m_mirror, "WRITE:["));
    HAL_CHECK(m_mirror->write(p_data));
    HAL_CHECK(hal::write(*m_mirror, "]\n"));
    return m_primary->write(p_data);
  }

  hal::result<read_t> driver_read(std::span<hal::byte> p_data) override
  {
    auto result = m_primary->read(p_data);

    if (result.has_value() && result.value().data.size() != 0) {
      HAL_CHECK(m_mirror->write(p_data));
    }

    return result;
  }

  hal::result<hal::serial::flush_t> driver_flush() override
  {
    return m_primary->flush();
  }

private:
  hal::serial* m_primary;
  hal::serial* m_mirror;
};

inline std::string_view to_string_view(std::span<const hal::byte> p_span)
{
  return std::string_view(reinterpret_cast<const char*>(p_span.data()),
                          p_span.size());
}