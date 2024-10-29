#pragma once
#include <libhal/i2c.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

// adapters
class tla2528_adc;
class tla2528_input_pin;
class tla2528_output_pin;

/**
 * @brief tla2528 is a gpio expander & adc mux driver
 *
 * tla2528 has 8 pins which can be independently operated as an adc,
 * digital input, and digital out over i2c. The i2c address is configured by
 * resistors connected to the chip There are no options for internal pull up or
 * pull down resistors. The output pins have the option of push-pull or
 * open-drain. When in adc mode there is an option (UNIMPLEMENTED) to increase
 * reading granularity though sampling averaging.
 */
class tla2528
{
public:
  enum class pin_mode : hal::byte
  {
    analog_input,
    digital_input,
    digital_output_open_drain,
    digital_output_push_pull
  };

  /**
   * @param p_i2c_address i2c address configured on the tla, by default is set
   * to the i2c address of no resistors attached to address config pins.
   */
  tla2528(hal::i2c& p_i2c, hal::byte p_i2c_address = default_address);

  /**
   * @brief set what service a pin will provide
   *
   * @param p_channel which pin to configure, if out of range (>7) an exception
   * will be thrown
   *
   * if adapters are made for a pin an exception may be thrown to prevent
   * invalid behavior
   */
  void set_pin_mode(pin_mode p_mode, hal::byte p_channel);

  /**
   * @brief set digital output level pin
   *
   * @param p_channel if out of range (>7) an exception will be thrown
   *
   * The device will write to a register that caches the desired output state.
   * When a pin is in a digital output mode it will reference the desired state
   * cache. If the pin is not set to digital output it will just be stored.
   */
  void set_digital_out(hal::byte p_channel, bool level);
  /**
   * @brief set digital output levels on all pins
   *
   * @param p_value The byte is used as a bit field of bool values. i.e the 0th
   * bit in the byte will set the 0 pin.
   *
   * The device will write to a register that caches the desired output state.
   * When a pin is in a digital output mode it will reference the desired state
   * cache. If the pin is not set to digital output it will just be stored.
   */
  void set_digital_bus_out(hal::byte p_values);
  /**
   * @brief read digital output state of an output pin
   *
   * @param p_channel if out of range (>7) an exception will be thrown
   *
   * If a pin is not set to output pin the returned state will be used once it
   * changes to an output pin.
   */
  bool get_digital_out(hal::byte p_channel);
  /**
   * @brief read digital output state of an output all pins
   *
   * If the pin is not set to output pin the returned state will be used once it
   * changes to an output pin.
   */
  hal::byte get_digital_bus_out();

  /**
   * @brief read the digital levels of all pins
   *
   * If a pin is not set to digital input or output the returned value may not
   * correlate with the true value.
   */
  bool get_digital_in(hal::byte p_channel);
  /**
   * @brief read the digital level of all pins
   *
   * @return The byte is used as a bit field of bool values. i.e the 0th bit in
   * the byte will be the 0 pin's level.
   *
   * If the pin is not set to digital input or output the returned value may not
   * correlate with the true value.
   */
  hal::byte get_digital_bus_in();

  /**
   * @brief read the analog input of a pin.
   *
   * @param p_channel if out of range (>7) an exception will be thrown
   *
   * If the pin is not set to analog input the returned value may not correlate
   * with the true value.
   */
  float get_analog_in(hal::byte p_channel);

  friend tla2528_adc;
  friend tla2528_input_pin;
  friend tla2528_output_pin;

private:
  // i2c address for no resistors
  static constexpr hal::byte default_address = 0x10;

  enum op_codes : hal::byte
  {
    single_register_read = 0b0001'0000,
    single_register_write = 0b0000'1000,
    set_bit = 0b0001'1000,
    clear_bit = 0b0010'0000,
    // Continuously reads data from a group of registers. Provide the first
    // address to read from, if it runs out of valid addresses to read, it
    // returns zeros. (See Figure 30 on datasheet)
    continuous_register_read = 0b0011'0000,
    // Continuously writes data to a group of registers. Provide the first
    // address to write to.The data sent will automatically write the data to
    // the next register in ascending order. (See Figure 32 on datasheet)
    continuous_register_write = 0b0010'1000
  };

  enum register_addresses : hal::byte
  {
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
    channel_sel = 0x11,
    auto_seq_ch_sel = 0x12
  };

  void set_analog_channel(hal::byte p_channel);
  void throw_if_invalid_channel(hal::byte p_channel);
  void reset();

  hal::i2c& m_i2c_bus;
  hal::byte m_i2c_address;
  hal::byte m_channel = 0x08;  // stores selected channel to reduce i2c requests
  hal::byte m_object_created = 0x00;  // tracks adapter channel reservations
  hal::byte m_gpo_value = 0x00;
};
}  // namespace sjsu::drivers
