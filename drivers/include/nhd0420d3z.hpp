
#include <libhal-util/serial.hpp>
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace sjsu::drivers {
class nhd0420d3z
{
public:
  /**
   * @brief Constructs a new nhd0420d3z (LCD text display with 4 lines and 20
   * columns of characters) driver instance.
   *
   * @param p_i2c refers to the I2C bus the display is connected to.
   * @param p_i2c_address refers to the 7-bit I2C address. The datasheet
   * default of 0x50 includes the R/W bit; 0x28 is the 7-bit equivalent.
   * The address can be changed on the display via command 0xFE 0x62, but
   * the LSB must always be 0.
   */
  nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address = default_i2c_address);

  /**
   * @brief First clears the screen, then displays a string on the LCD, handling
   * newlines and line wrapping.
   *
   * @param p_str refers to the text to display. Characters beyond the
   * display's 80-character capacity (4 lines x 20 columns) may not be
   * visible.
   */
  void display_message(std::string_view p_str);

  /**
   * @brief Turns the display on or off.
   *
   * The display contents are preserved in DDRAM when off and reappear when
   * turned back on. Default state after power-up is on. 
   * 
   * Execution time: 100 us.
   *
   * @param p_on set to true to turn the display on, false to turn it off.
   */
  void power(bool p_on);

  /**
   * @brief Writes a single ASCII character at the current cursor position.
   *
   * The cursor advances one position automatically on the display. Valid
   * displayable ranges from the datasheet:
   *   - 0x00-0x07: user-defined custom characters
   *   - 0x20-0x7F: standard ASCII character set
   *   - 0xA0-0xFD: factory-masked characters on the ST7066U
   *   - 0xFE: reserved (command prefix, do not use as a character)
   *
   * Note: this method does NOT update the cursor tracker
   * TODO: fix to update cursor tracker
   *
   * Execution time: 100 us.
   *
   * @param p_character refers to the character to display.
   */
  void write_char(char p_character);

  /**
   * @brief Moves the cursor to a specific row and column on the display.
   *
   * Execution time: 100 us.
   *
   * @param p_line refers to the row index by 0 index (0-3)
   * @param p_column refers to the column index by 0 index (0-19).
   */
  void set_cursor_position(hal::byte p_line = 0, hal::byte p_column = 0);

  /**
   * @brief Moves the cursor one position to the right.
   *
   * If cursor tracker is at the end of row the command is suppressed
   *
   * Execution time: 100 us.
   */
  void move_cursor_right();

  /**
   * @brief Returns the cursor to top left
   *
   * The display contents are not altered
   *
   * Execution time: 1.5 ms.
   */
  void home_cursor();

  /**
   * @brief Clears all displayed text and resets the cursor to line 1,
   * column 1.
   *
   * Note: does NOT update cursor tracker
   * TODO: fix cursor tracker
   *
   * Execution time: 1.5 ms.
   */
  void clear_screen();

private:
  hal::i2c& m_i2c_bus;
  hal::byte m_i2c_address;
  hal::byte m_cursor_line = 0;    // Current row (0-3)
  hal::byte m_cursor_column = 0;  // Current column (0-19)

  constexpr static hal::byte display_lines = 4;
  constexpr static hal::byte display_columns = 20;
  constexpr static hal::byte default_i2c_address = 0x28;

  /**
   * @brief Converts a (line, column) coordinate to a DDRAM address byte.
   *
   * The datasheet defines the following DDRAM address ranges per row:
   *
   *   Line 1: 0x00 - 0x13
   *   Line 2: 0x40 - 0x53
   *   Line 3: 0x14 - 0x27
   *   Line 4: 0x54 - 0x67
   *
   * This driver treats the display as a grid where line selects the row
   * (0-3) and column selects the column (0-19).
   *
   * @param line refers to the row index (0-3).
   * @param column refers to the column index (0-19).
   * @return DDRAM address byte suitable for the Set Cursor command.
   */
  hal::byte coordinates_to_position(hal::byte line, hal::byte column);

  /**
   * @brief Sends a single raw byte to the display over I2C.
   *
   * This is the low-level transport used by all other methods. It wraps
   * the byte in a single-element array and performs an I2C write
   * transaction to the display's slave address using hal::write().
   *
   * @param data refers to the byte to transmit.
   */
  void send_data(hal::byte data);

  /**
   * @brief Sends the command prefix byte (0xFE) to the display.
   *
   * All display commands must be preceded by 0xFE. This is called
   * internally before every command byte. Sending a data byte WITHOUT
   * this prefix causes the display to interpret it as an ASCII character
   * to write at the current cursor position.
   */
  void send_prefix();
};

}  // namespace sjsu::drivers