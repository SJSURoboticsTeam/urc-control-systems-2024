#pragma once
#include "../../drivers/include/sk9822.hpp"
#include <libhal/units.hpp>

namespace sjsu::hub {

class beacon
{
public:
  /**
   * @param p_beacon an sk9822 (LED STRIP) object that is used
   *
   * @brief beacon constructor that takes in the sk9822 driver and sets the
   default RGB brightness to white and some default brightness
   */

  beacon(sjsu::drivers::sk9822 p_beacon);

  /**
   * @param period time duration we want to static flash the led strip
   *
   * @brief set the led strip shine a static color for p_period long and
   * brightness based off beacon_setting
   */
  void static_flash(hal::time_duration p_period);

  /**
   * @param period time duration we want to static flash the led strip
   *
   * @brief set the led strip shine a blinking color for p_period long and
   * brightness based off beacon_setting
   */
  void blink_flash(hal::time_duration p_period);

  /**
   * @param p_beacon_settings takes in rgb_brightness struct and sets the beacon
   to that setting
   *
   * @brief sets the beacon_settings equal to sets p_beacon_settings
   */
  void set_beacon(sjsu::drivers::rgb_brightness p_beacon_settings);

  /**
   * @brief turns off the LEDs on the LED strip
   */
  void off_beacon();

  /**
   * @brief turns on the beacon with beacon_settings
   */
  void on_beacon();

private:
  sjsu::drivers::sk9822 m_beacon;
  sjsu::drivers::rgb_brightness beacon_setting;
};

}  // namespace sjsu::hub