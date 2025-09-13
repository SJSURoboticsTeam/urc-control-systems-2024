#pragma once
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>

#include <libhal/units.hpp>

namespace sjsu::drivers {
class apds9960
{
private:
  void write_register(hal::byte register_address, hal::byte value);
  void read_register(hal::byte register_address, std::span<hal::byte> out);

  hal::v5::strong_ptr<hal::i2c> m_i2c;
  hal::v5::strong_ptr<hal::steady_clock> m_clock;
  hal::v5::strong_ptr<hal::serial> m_terminal;
  hal::byte m_address = 0x39;

public:
  apds9960(hal::v5::strong_ptr<hal::i2c> p_i2c,
           hal::v5::strong_ptr<hal::steady_clock> p_clock,
           hal::v5::strong_ptr<hal::serial> p_terminal);

  /// @brief list of named registers for apds9960
  enum register_set
  {
    enable = 0x80,

    // store rgbc
    cdatal = 0x94,
    cdatah = 0x95,
    rdatal = 0x96,
    rdatah = 0x97,
    gdatal = 0x98,
    gdatah = 0x99,
    bdatal = 0x9A,
    bdatah = 0x9B,
  };

  struct color
  {
    hal::u16 clear_data;
    hal::u16 red_data;
    hal::u16 green_data;
    hal::u16 blue_data;
  };

  void default_enable();

  color readColor();
};
}  // namespace sjsu::drivers