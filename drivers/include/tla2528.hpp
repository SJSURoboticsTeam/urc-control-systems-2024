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
   * @param p_i2c i2c bus of the device
   *
   * @param p_i2c_address i2c address configured on the tla, by default is set
   * to the i2c address of no resistors attached to address config pins.
   */
  tla2528(hal::i2c& p_i2c, hal::byte p_i2c_address = default_address);

  /**
   * @brief set what service a pin will provide
   *
   * @param p_mode tla2528::pin_mode enum of desired pin mode
   *
   * @param p_channel which pin to configure
   *
   * @throws hal::argument_out_of_domain - if p_channel out of range (>7)
   *
   * @throws hal::resource_unavailable_try_again - if adapters are made for a
   * pin an exception may be thrown to prevent invalid behavior
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   */
  void set_pin_mode(pin_mode p_mode, hal::byte p_channel);

  /**
   * @brief set digital output level pin
   *
   * @param p_channel pin to set output
   *
   * @param p_high the output level of the pin, true is high, false is low.
   *
   * @throws hal::argument_out_of_domain - if p_channel out of range (>7)
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   *
   * The device will write to a register that caches the desired output state.
   * When a pin is in a digital output mode it will reference the desired state
   * cache. If the pin is not set to digital output it will just be stored.
   */
  void set_digital_out(hal::byte p_channel, bool p_high);
  /**
   * @brief set digital output levels on all pins
   *
   * @param p_values The byte is used as a bit field of bool values to set the
   * pin outputs. i.e the 0th bit in the byte will set the 0 pin. If a bit is
   * 1 it is high. If the bit is 0 it is low.
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   *
   * The device will write to a register that caches the desired output state.
   * When a pin is in a digital output mode it will reference the desired state
   * cache. If the pin is not set to digital output it will just be stored.
   */
  void set_digital_bus_out(hal::byte p_values);
  /**
   * @brief read digital output state register of an output pin
   *
   * @param p_channel pin you would like to get the ouput value
   *
   * @return if the pin's output value register is high
   *
   * @throws hal::argument_out_of_domain - if p_channel out of range (>7)
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   *
   * If a pin is not set to output pin the returned state will be used once it
   * changes to an output pin.
   */
  bool get_digital_out(hal::byte p_channel);
  /**
   * @brief read digital output state register of all output pins
   *
   * @return The byte is used as a bit field of bool values to give the pins'
   * register. i.e the 0th bit in the byte will be the 0 pin's stored value. If
   * a bit is 1 it is high. If the bit is 0 it is low.
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   *
   * If the pin is not set to output pin the returned state will be used once it
   * changes to an output pin.
   */
  hal::byte get_digital_bus_out();

  /**
   * @brief read the digital level of a pins
   *
   * @return if the pin's digital read value is high.
   *
   * @throws hal::argument_out_of_domain - if p_channel out of range. (>7)
   *
   * @throws hal::no_such_device - no device responded on i2c bus
   *
   * If a pin is not set to digital input or output the returned value may not
   * correlate with the true value.
   */
  bool get_digital_in(hal::byte p_channel);
  /**
   * @brief read the digital levels of all pins
   *
   * @return The byte is used as a bit field of bool values to give the pins'
   * digital read values. i.e the 0th bit in the byte will be the 0 pin's stored
   * value. If a bit is 1 it is high. If the bit is 0 it is low.
   *
   * @throws hal::no_such_device - no device responded on i2c bus
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
   * @return adc reading as a float between 0 and 1 inclusive
   *
   * @throws hal::argument_out_of_domain - if p_channel out of range. (>7)
   *
   * @throws hal::no_such_device - no device responded on i2c bus
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
