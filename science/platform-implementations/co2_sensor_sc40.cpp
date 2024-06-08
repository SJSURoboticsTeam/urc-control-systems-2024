#pragma once

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>
#include <span>

// i2cc address is 0x62
// need a buffer
// 3 things to get from the i2c bus
// use std::span<hal::byte> for buffer to get data
// use i2c bus 2 for the dev 2 board while testing

using namespace hal::literals;
using namespace std::chrono_literals;
namespace sjsu::science {
class co2_sensor
{
private:
  co2_sensor(hal::i2c& p_i2c, hal::steady_clock& c)
    : m_i2c(p_i2c)
    , steady_clock{ c }
  {

    // start periodic measurement //write to hex 0x21b1
    //  hal::write(m_i2c, addresses::address,
    //  std::span{a},hal::never_timeout()); hal::write(m_i2c,
    //  addresses::address, std::span{a}); hal::transaction(sensor_address,
    //  start_address, nullptr, )
  }

public:
  enum addresses : hal::byte
  {
    address = 0x62,
    start_first_half = 0x21,
    start_second_half = 0xb1,
    read_first_half = 0xec,
    read_second_half = 0x05,
    stop_first_half = 0x3f,
    stop_second_half = 0x86
  };

  static hal::result<co2_sensor> create(hal::i2c& p_i2c,
                                        hal::steady_clock& clock)
  {
    co2_sensor co2_sensor(p_i2c, clock);
    HAL_CHECK(co2_sensor.start());
    return co2_sensor;
  }

  hal::status start()
  {
    std::array<hal::byte, 2> start_address{ { start_first_half, start_second_half } };
    HAL_CHECK(hal::write(m_i2c, addresses::address, start_address, hal::never_timeout()));
    return hal::success();
  }

  hal::result<int16_t> read_co2()
  {
    // std::array<hal::byte,9> buffer; //3 bytes for co2, temperature and
    // humidity
    std::array<const hal::byte, 2> read_address{ read_first_half,
                                                 read_second_half };

    std::array<hal::byte, 9> buffer;

    // hal::write_then_read(m_i2c, addresses::address, burrito, buffer,
    // hal::never_timeout());
    HAL_CHECK(hal::write(m_i2c, addresses::address, read_address));
    hal::delay(steady_clock, 1ms);

    HAL_CHECK(
      hal::read(m_i2c, addresses::address, buffer, hal::never_timeout()));
    int16_t result = buffer[0] << 8 | buffer[1] << 0;

    return result;
  }

  // function that converts data into measurements that we need
  hal::status stop()
  {
    std::array<hal::byte, 2> stop_address{ { stop_first_half, stop_second_half } };

    HAL_CHECK(hal::write(
      m_i2c, addresses::address, std::span{ stop_address }, hal::never_timeout()));
  }

  hal::i2c& m_i2c;
  hal::steady_clock& steady_clock;
};
}  // namespace science