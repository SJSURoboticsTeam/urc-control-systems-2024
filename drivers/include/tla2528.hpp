#pragma once
#include <array>
#include <libhal-util/i2c.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/i2c.hpp>
#include <libhal/steady_clock.hpp>

// debugging purposes

namespace tla {
class tla2528
{
public:
  tla2528(hal::i2c& p_i2c, hal::steady_clock& p_clk)
    : m_bus(p_i2c)
    , m_clk{ p_clk }
  {
  }

  hal::result<uint16_t> read_one(hal::byte channel)
  {
    using namespace std::literals;

    if (channel > 7) {
      return hal::new_error(
        "Invalid channel\n");  // TODO: switch to an error number,
                               // strings take up too much space
    }
    std::array<hal::byte, 2> data_buffer;
    std::array<hal::byte, 3> selection_cmd_buffer = {
      OpCodes::SingleRegisterWrite,    // Command to write data to a register
      RegisterAddresses::CHANNEL_SEL,  // Register to select channel
      channel
    };
    std::array<hal::byte, 1> read_cmd_buffer = { OpCodes::SingleRegisterRead };

    HAL_CHECK(hal::write(m_bus, m_mux_i2c_id, selection_cmd_buffer));
    HAL_CHECK(hal::write(m_bus, m_mux_i2c_id, read_cmd_buffer));
    HAL_CHECK(hal::read(m_bus, m_mux_i2c_id, data_buffer));
    hal::delay(m_clk, 1ms);

    uint16_t data = (data_buffer[0] << 4) | (data_buffer[1] >> 4);

    // return voltage_to_degree(data, 4096, 360);

    return data;
    // std::pair<float, float> p_input_range;
    // std::pair<float, float> p_output_range;

    // p_input_range = std::make_pair<float, float>(0.0, 4000.0);
    // p_output_range = std::make_pair<float, float>(0,0, 360.0);
    
    // auto mapped_data = constexpr float hal::map (
    //   data, // modifies data directly?
    //   p_input_range /*something something gotta check*/ ,
    //   p_output_range /*potentiometer range*/  
    // ) 
    // return mapped_data;
  };

  // TODO: Improve code, more testing
  hal::result<std::array<int, 8>> read_all()
  {
    std::array<int, 8> result = {};
    for (int i = 0; i < 8; i++) {
      result[i] = HAL_CHECK(read_one(i));
    }

    return result;
  }

private:
  enum OpCodes
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
  enum RegisterAddresses
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
  const hal::byte m_mux_i2c_id = 0x17;
  hal::i2c& m_bus;
  hal::steady_clock& m_clk;
};
}  // namespace tla
