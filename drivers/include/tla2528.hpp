#pragma once
#include <libhal/i2c.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

//adapters
class tla2528_adc;
class tla2528_input_pin;
class tla2528_output_pin;

class tla2528
{
private:
  friend tla2528_adc;
  friend tla2528_input_pin;
  friend tla2528_output_pin;

  static constexpr hal::byte default_address = 0x10;//address for no resistors
  //address default reference voltage
  //TODO: look at volts literal
  static constexpr float default_reference_voltage = 3.3;

  enum op_codes : hal::byte {
    // Requests to read data from the mux. (See Figure 29 on datasheet)
    single_register_read = 0b0001'0000,

    // Writes data to a register given an address. (See Figure 31 on datasheet)
    single_register_write = 0b0000'1000,

    // Sets bits in a given register, requires addr
    set_bit = 0b0001'1000,

    // Clears bits in a given register
    clear_bit = 0b0010'0000,

    // Continuously reads data from a group of registers.  Provide the addr of
    // the first to read from, reads happen on clk, if it runs out of valid
    // addresses to read, it returns zeros.
    // Master ACKs to confirm data has
    // gotten through. (See Figure 30 on datasheet)
    continuous_register_read = 0b0011'0000,

    // Continuously writes data to a group of registers. Provide the first
    // address to write to. Then, send data as bytes slave will automatically
    // write the data to the next register in accending order, data is
    // seperated by ACKs (slave ACKs). (See Figure 32 on datasheet)
    continuous_register_write = 0b0010'1000
  };

  /**
   * @brief Most of these are just for configurations, if you need to change
   them, see bit layout (Section 7.6 on datasheet) And use SetBit and ClearBit
   appropriately to set and clear the bits dictated by each setting. (See
   Table 8 on data sheet)
   *
   */
  enum register_addresses : hal::byte {
    system_status = 0x0,
    general_cfg = 0x1,
    data_cfg = 0x2,
    osr_cfg = 0x3,
    opmode_cfg = 0x4,
    pin_cfg = 0x5,
    gpio_cfg = 0x7,
    gpo_drive_cfg = 0x9,
    gpo_value = 0xB,
    gpi_value = 0xD,
    sequence_cfg = 0x10,

    // Channel selection register, write the channel number 0-7 that you are
    // trying to read from during the write frame.
    channel_sel = 0x11,
    auto_seq_ch_sel = 0x12
  };
  hal::i2c& m_bus;
  hal::byte m_i2c_address;
  float m_analog_supply_voltage;
  hal::byte m_channel = 0x08;// stores selected channel to reduce i2c requests
  hal::byte m_object_created = 0x00;// tracks adapter channel reservations
  hal::byte m_pin_cfg = 0x00;
  hal::byte m_gpio_cfg = 0x00;
  hal::byte m_gpo_drive_cfg = 0x00;
  hal::byte m_gpo_value = 0x00;

  void set_analog_channel(hal::byte p_channel);
  void reset();

public:
  enum class pin_mode : hal::byte
  {
    analog_input,
    digital_input,
    digital_output_open_drain,
    digital_output_push_pull
  };

  tla2528(hal::i2c& p_i2c, 
    hal::byte p_i2c_address=default_address, 
    float p_analog_supply_voltage=default_reference_voltage
  );

  void set_pin_mode(pin_mode p_mode,hal::byte p_channel);

  //If channel is not set to correct config then undefined behavior will occur
  void set_digital_out(hal::byte p_channel, bool level);
  void set_digital_out(hal::byte p_values);
  bool get_digital_out(hal::byte p_channel);
  bool get_digital_in(hal::byte p_channel);
  hal::byte get_digital_in();

  float get_analog_in(hal::byte p_channel);

};
}  // namespace sjsu::drivers
