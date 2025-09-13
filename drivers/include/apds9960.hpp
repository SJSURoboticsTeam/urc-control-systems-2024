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
  enum register_set{
    enable = 0x80,

    atime = 0x81,  //  internal integration time of ALS/Color analog to digital
                   //  converters
    wtime = 0x83,  //  amount of time in a low power mode between Proximity
                   //  and/or ALS cycles
    // pers = 0x8C, // (non-gesture)
    // config1 = 0x8D,
    // ppulse = 0x8E,
    control = 0x8F,
    // config2 = 0x90,
    id = 0x92,
    status = 0x93,

    // store rgbc
    cdatal = 0x94,
    cdatah = 0x95,
    rdatal = 0x96,
    rdatah = 0x97,
    gdatal = 0x98,
    gdatah = 0x99,
    bdatal = 0x9A,
    bdatah = 0x9B,

    // proximity photodiodes
    // poffset_ur = 0x9D,
    // poffset_dl = 0x9E,

    // config3 = 0x9F,
    gpenth = 0xA0,
    gexth = 0xA1,
    gconf1 = 0xA2,
    gconf2 = 0xA3,
    goffset_u = 0xA4,
    goffset_d = 0xA5,
    goffset_l = 0xA7,
    goffset_r = 0xA9,
    gpulse = 0xA6,
    gconf3 = 0xAA,
    gconf4 = 0xAB,
    gflvl = 0xAE,
    gstatus = 0xAF,
    // gfifo_u = 0xFC,
    // gfifo_d = 0xFD,
    // gfifo_l = 0xFE,
    // gfifo_r = 0xFF
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