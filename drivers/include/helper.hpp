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
  serial_mirror(hal::v5::strong_ptr<hal::serial> p_primary,
                hal::v5::strong_ptr<hal::serial> p_mirror)
    : m_primary(&p_primary)
    , m_mirror(&p_mirror)
  {
  }

  void driver_configure(settings const& p_settings) override
  {
    m_primary->configure(p_settings);
  }

  void driver_write(std::span<hal::byte const> p_data) override
  {
    hal::write(*m_mirror, "WRITE:[");
    m_mirror->write(p_data);
    hal::write(*m_mirror, "]\n");
    return m_primary->write(p_data);
  }

  read_t driver_read(std::span<hal::byte> p_data) override
  {
    auto result = m_primary->read(p_data);

    if (result.has_value() && result.value().data.size() != 0) {
      m_mirror->write(p_data);
    }

    return result;
  }

  void driver_flush() override
  {
    m_primary->flush();
  }

private:
  hal::serial* m_primary;
  hal::serial* m_mirror;
};

inline std::string_view to_string_view(std::span<hal::byte const> p_span)
{
  return std::string_view(reinterpret_cast<char const*>(p_span.data()),
                          p_span.size());
}