
#include <libhal-util/serial.hpp>
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {

constexpr hal::byte display_lines = 4;
constexpr hal::byte display_columns = 20;
constexpr hal::byte default_i2c_address = 0x28;
class nhd0420d3z
{
public:
  /**
   * @brief the nhd0420d3z is a lcd text display with 4 lines and 20 columns of
   * characters
   * @param p_i2c i2c bus
   * @param p_i2c_address i2c address of the disply
   */
  nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address = default_i2c_address);
  /**
   * @brief clears screen and displays string on screen
   */
  void display_message(std::string_view p_str);
  /**
   * @brief sets if the display is on or off
   *
   * @param p_on if display is on
   *
   * wait 100 microseconds before issuing next command to display
   */
  void power(bool p_on);
  /**
   * @brief write character to screen at current cursor position
   *
   * @param p_character character to be written to screen
   *
   * Automatically moves cursor. Wait 100 microseconds before issuing next command to display
   */
  void write_char(char p_character);
  /**
   * @brief move cursor to specified position
   *
   * @param p_line line of cursor (default 0)
   * @param p_column column of cursor (default 0)
   *
   * wait 100 microseconds before issuing next command to display
   */
  void set_cursor_position(hal::byte p_line = 0, hal::byte p_column = 0);
  /**
   * @brief moves cursor to the right
   *
   * wait 100 micro seconds before issuing next command to display
   */
  void move_cursor_right();
  /**
   * @brief moves cursor to 0,0 with home command
   * wait 1.5 milliseconds before issuing next command to display.
   */
  void home_cursor();
  void clear_screen();

private:
  hal::i2c& m_i2c_bus;
  hal::byte m_i2c_address;
  hal::byte m_cursor_line = 0;
  hal::byte m_cursor_column = 0;

  hal::byte coordinates_to_position(hal::byte line, hal::byte column);
  void send_data(hal::byte data);
  void send_prefix();
};

}  // namespace sjsu::drivers