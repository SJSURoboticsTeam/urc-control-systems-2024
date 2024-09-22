#pragma once
#include <cstdint>
#include <libhal-util/i2c.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>
#include <sys/types.h>

// debugging purposes

namespace sjsu::drivers {
class tla2528
{
private:
  enum OpCodes : hal::byte
  {
    // Requests to read data from the mux. (See Figure 29 on datasheet)
    SingleRegisterRead = 0b0001'0000,

    // Writes data to a register given an address. (See Figure 31 on data
    // sheet)
    SingleRegisterWrite = 0b0000'1000,

    // Sets bits in a given register, requires addr
    SetBit = 0b0001'1000,

    // Clears bits in a given register
    ClearBit = 0b0010'0000,

    // Continuously reads data from a group of registers.  Provide the addr of
    // the first to read from, reads happen on clk, if it runs out of valid
    // addresses to read, it returns zeros.
    // Master ACKs to confirm data has
    // gotten through. (See Figure 30 on datasheet)
    ContinuousRegisterRead = 0b0011'0000,

    // Continuously writes data to a group of registers. Provide the first
    // address to write to. Then, send data as bytes slave will automatically
    // write the data to the next register in accending order, data is
    // seperated by ACKs (slave ACKs). (See Figure 32 on datasheet)
    ContinuousRegisterWrite = 0b0010'1000
  };

  /**
   * @brief Most of these are just for configurations, if you need to change
   them, see bit layout (Section 7.6 on datasheet) And use SetBit and ClearBit
   appropriately to set and clear the bits dictated by each setting. (See
   Table 8 on data sheet)
   *
   */
  enum RegisterAddresses : hal::byte
  {
    SYSTEM_STATUS = 0x0,
    GENERAL_CFG = 0x1,
    DATA_CFG = 0x2,
    OSR_CFG = 0x3,
    OPMODE_CFG = 0x4,
    PIN_CFG = 0x5,
    GPIO_CFG = 0x7,
    GPO_DRIVE_CFG = 0x9,
    GPO_VALUE = 0xB,
    GPI_VALUE = 0xD,
    SEQUENCE_CFG = 0x10,

    // Channel selection register, write the channel number 0-7 that you are
    // trying to read from during the write frame.
    CHANNEL_SEL = 0x11,
    AUTO_SEQ_CH_SEL = 0x12
  };

  enum class PinMode : hal::byte
  {
    AnalogInput = 0b000,
    DigitalInput = 0b100,
    DigitalOutputOpenDrain = 0b110,
    DigitalOutputPushPull = 0b111
  };
  hal::byte m_i2c_address;
  hal::i2c& m_bus;
  hal::steady_clock& m_clk;
  // first 3 bits store: PIN_CFG, GPIO_CFG, GPO_DRIVE_CFG in that order
  // last 3 bits store selected channel
  hal::byte m_mode;

public:
  tla2528(hal::i2c& p_i2c, hal::steady_clock& p_clk, hal::byte p_i2c_address);

  void set_channel(hal::byte channel);
  void set_pin_mode(PinMode p_mode);

  void set_digital_out(bool level);
  void set_digital_out(bool level, hal::byte channel);
  bool get_digital_in();
  bool get_digital_in(hal::byte channel);
  uint16_t get_analog_in();
  uint16_t get_analog_in(hal::byte channel);

};
}  // namespace sjsu::drivers
